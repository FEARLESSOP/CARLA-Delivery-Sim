#!/usr/bin/env python3
"""Offline tests: delivery drones on the LastMile charging stations' drone
decks (delivery/station.DeckStation + DroneFleet's deck plugs). No server.

  ../../carla-venv/bin/python tests/test_station_deck.py

LastMile 充電ステーションのドローンデッキ（DeckStation とプラグ動作）の
オフラインテスト。
"""

import os
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
HIL = os.path.dirname(HERE)
sys.path.insert(0, HIL)

import carla   # noqa: F401,E402  (station -> backends imports it)
from delivery.battery import Battery, BatteryModel  # noqa: E402
from delivery.backends import (CHARGING, TO_STATION, TO_VAN, DOCKED,  # noqa: E402
                               AT_STATION)
from delivery.drone import LAND  # noqa: E402
from delivery import station as S  # noqa: E402

FAILS = []
NOW = [0.0]                 # the test's clock, for the plug recorder / 時計


def check(name, cond, detail=""):
    print("  %s  %s%s" % ("PASS" if cond else "FAIL", name, (" | %s" % (detail,)) if detail != "" else ""))
    if not cond:
        FAILS.append(name)


class FakeStations(object):
    """charge_sites.Stations stand-in: 2 sites, deck 2.4 m over the floor.
    charge_sites.Stations の代役（2 基、デッキは床から 2.4 m）。"""

    FLOOR = {1: 10.0, 2: 3.0}

    def __init__(self):
        self.sites = [{"id": 1, "frame": None}, {"id": 2, "frame": None}]
        self.plugs = []     # (t, site, pad, u)

    def deck_pads(self, sid):
        ox = 0.0 if sid == 1 else 200.0
        z = self.FLOOR[sid] + 2.4
        return [(ox + 1.0 * k, 5.0, z) for k in range(3)], 90.0 * sid

    def deck_plug(self, sid, i, u):
        self.plugs.append((round(NOW[0], 3), sid, i, u))


class Unit(object):
    """DroneUnit stand-in: parks, repairs, flies on command. / DroneUnit の代役。"""

    def __init__(self, i, level=40.0):
        self.id, self.status, self.van, self.pos, self.yaw = i, DOCKED, None, (0.0, 0.0, 0.0), 0.0
        self.pad = self.pad_target = self.flight = None
        self.battery = Battery(level)
        self.h_speed, self.v_speed = 12.0, 4.0
        self.note, self.crash_detail, self.crashes, self.trips, self.km = "", "", 0, 0, 0.0
        self.transfers, self.arrive = [], False

    def world_pos(self):
        return self.pos

    def park(self, pad, pos, status=CHARGING):
        self.van, self.pad, self.pos = None, pad, tuple(pos)
        self.status, self.note, self.flight = status, "", None
        self.parked_yaw = self.yaw

    def repair(self, pad, pos, level):
        self.battery.level = level
        self.park(pad, pos, CHARGING)

    def start_transfer(self, target_fn, z, to_van=None):
        self.flight = types.SimpleNamespace(state="takeoff")
        self.status = TO_STATION if to_van is None else TO_VAN
        self.van, self.pad = to_van, None
        self.transfers.append((round(NOW[0], 3), "station" if to_van is None else to_van.vid))

    def update(self, t, dt, wind):
        if self.arrive:
            self.arrive = False
            return [("arrived", None)]
        return []

    def collect_parcels(self, t):
        pass


class Van(object):
    def __init__(self, vid):
        self.vid, self.drone, self.incoming, self.lmp = vid, None, None, False

    def dock_pos(self):
        return (20.0, 5.0, 0.5)

    def release_drone(self, u):
        pass

    def accept_drone(self, u):
        self.drone = u

    def on_drone_event(self, t, u, was, ev):
        pass


def run(fleet, t0, t1, dt=0.1):
    t = t0
    while t < t1 - 1e-9:
        t = round(t + dt, 6)
        NOW[0] = t
        fleet.step(t, dt, (0.0, 0.0))
    return t


def fleet_for(station, units, log):
    return S.DroneFleet(units, station, BatteryModel(), lambda a, b: 30.0,
                        lambda *a, **k: log.append(a[1]), wind_hold=0.0, rain_ground=0.0)


print("1. DeckStation: 3 pads per station / デッキのパッド")
fs = FakeStations()
ds = S.DeckStation(fs)
check("6 pads (3 per station, station order)", len(ds.pads) == 6 and ds.n_deck == 6, ds.pads)
check("pad z = deck surface (floor + 2.4 m)",
      all(abs(ds.pads[i][2] - 12.4) < 1e-9 for i in range(3)) and
      all(abs(ds.pads[i][2] - 5.4) < 1e-9 for i in range(3, 6)))
check("pad_owner: (site id, deck pad)", [ds.pad_owner(i) for i in range(6)] ==
      [(1, 0), (1, 1), (1, 2), (2, 0), (2, 1), (2, 2)])
check("pad_yaw: the station's drone yaw", [ds.pad_yaw(i) for i in range(6)] == [90.0] * 3 + [180.0] * 3)
check("free_pad: deck pad 1 of station 1 first", ds.free_pad() == 0)
check("free_pad(near): the nearest station's deck", ds.free_pad((199.0, 0.0, 0.0)) == 3)
ds.taken[3] = 7
check("then its pad 2", ds.free_pad((199.0, 0.0, 0.0)) == 4)
ds.release(types.SimpleNamespace(id=7))
check("release frees the pad", 3 not in ds.taken and ds.free_pad((199.0, 0.0, 0.0)) == 3)
loc = ds.location
check("location: middle of the deck nearest the centre", abs(loc[0] - 1.0) < 1e-9 or abs(loc[0] - 201.0) < 1e-9, loc)
check("location with centre: that station's deck", abs(S.DeckStation(fs, centre=(190.0, 0.0)).location[0] - 201.0) < 1e-9)
dx = S.DeckStation(fs, extra=[(50.0, 50.0, 0.02)])
check("extra pads come after the decks, no owner, no yaw",
      len(dx.pads) == 7 and dx.pad_owner(6) is None and dx.pad_yaw(6) is None)
for i in range(6):
    dx.taken[i] = 100 + i
check("decks full: the extra pad", dx.free_pad((50.0, 50.0)) == 6 and dx.free_pad() == 6)

print("2. standbys start plugged in, squared / 待機機は充電中から")
fs = FakeStations()
ds = S.DeckStation(fs)
units = [Unit(1, 100.0), Unit(2, 100.0)]
log = []
fleet = fleet_for(ds, units, log)
check("deck station: the fleet drives its Stations", fleet.deck is fs)
fleet.start([])
check("standbys on deck pads 1 and 2, at the deck yaw",
      [u.pad for u in units] == [0, 1] and all(u.parked_yaw == 90.0 for u in units))
NOW[0] = 0.1
fleet.step(0.1, 0.1, (0.0, 0.0))
check("their plugs go straight in", sorted(fs.plugs) == [(0.1, 1, 0, 1.0), (0.1, 1, 1, 1.0)], fs.plugs)

print("3. a drone flies in, lands, gets plugged / 着陸してプラグ挿入")
fs = FakeStations()
ds = S.DeckStation(fs)
u = Unit(1, 40.0)
u.pos = (10.0, 5.0, 20.0)
van = Van(1)
log = []
fleet = fleet_for(ds, [u], log)
NOW[0] = 0.0
check("sent to the station", fleet.send_to_station(u, 0.0) and u.pad_target == 0 and ds.taken == {0: 1})
u.flight.state, u.yaw = LAND, 0.0
t = run(fleet, 0.0, 0.5)
check("squaring on the way down (90 deg/s, not snapped)", 40.0 < u.yaw < 50.0, u.yaw)
u.arrive = True
t = run(fleet, t, 0.6)          # touch-down at t = 0.6 / 着地
check("parked on deck pad 1 at the deck yaw", u.pad == 0 and u.status in AT_STATION and u.parked_yaw == 90.0
      and u.pos == ds.pads[0])
t = run(fleet, t, 0.6 + 4.9)
check("no plug in the first 5 s", fs.plugs == [], fs.plugs)
t = run(fleet, t, 0.6 + 8.0)
first = fs.plugs[0]
full = next(p for p in fs.plugs if p[3] == 1.0)
check("plug starts ~5 s after touch-down", abs(first[0] - 5.7) < 0.11 and first[1:3] == (1, 0), first)
check("plug fully in 1.4 s later", abs(full[0] - 7.0) < 0.11, full)
us = [p[3] for p in fs.plugs]
check("plug only moves in", us == sorted(us) and len(us) >= 10, us)

print("4. a van asks: plug out, then lift-off / 抜去してから離陸")
fs.plugs[:] = []
u.battery.level = 100.0
fleet.request(van, 20.0)
t_req = round(t + 0.1, 6)
t = run(fleet, t, t_req)
check("the van gets it, but it doesn't lift off yet", van.incoming is u and u.transfers[-1][1] == "station"
      and u.status in AT_STATION, u.transfers)
check("not picked twice while unplugging", u.id in fleet._launch)
t = run(fleet, t, t_req + 2.0)
outs = [p for p in fs.plugs]
zero = next(p for p in outs if p[3] == 0.0)
lift = u.transfers[-1]
check("plug out over 0.6 s", abs(zero[0] - (t_req + 0.6)) < 0.11 and [p[3] for p in outs] ==
      sorted([p[3] for p in outs], reverse=True), outs)
check("lift-off 1.1 s after the plug is out", lift[1] == 1 and abs(lift[0] - (t_req + 1.7)) < 0.11, lift)
check("off the deck: pad freed, timer gone", 0 not in ds.taken and fleet._deck_pads == {} and u.status == TO_VAN)
check("standby_sent logged", "standby_sent" in log)

print("5. the van changes its mind while unplugging / 抜去中の取り消し")
fs = FakeStations()
ds = S.DeckStation(fs)
u = Unit(1, 100.0)
van = Van(1)
fleet = fleet_for(ds, [u], [])
fleet.start([])
t = run(fleet, 0.0, 1.0)
fleet.request(van, 20.0)
t = run(fleet, t, 1.3)          # 0.2 s into the unplug / 抜去 0.2 s
fleet.recall(u, t)
van.incoming = None
t = run(fleet, t, 5.0)
check("stays on the deck", u.status in AT_STATION and u.pad == 0 and len(u.transfers) == 0)
check("plug goes back in from where it was", fs.plugs[-1][3] == 1.0 and min(p[3] for p in fs.plugs) > 0.5,
      fs.plugs)

print("6. left the pad without the hold: plug out at once / 保持なしの離脱")
fs.plugs[:] = []
u.status, u.pad = TO_VAN, None
t = run(fleet, t, 6.0)
check("plug retracted over 0.6 s, timer gone", fs.plugs[-1][3] == 0.0 and
      abs(fs.plugs[-1][0] - 5.6) < 0.11 and fleet._deck_pads == {}, fs.plugs)

print("7. repair lands on a deck squared / 回収機もデッキへ")
fs = FakeStations()
ds = S.DeckStation(fs)
u = Unit(1, 0.0)
u.status, u.down_t, u.fail_reason, u.pos = "crashed", 0.0, "motor_failure", (205.0, 0.0, 0.0)
fleet = fleet_for(ds, [u], [])
fleet.recover_s = 1.0
t = run(fleet, 0.0, 1.5)
check("on the nearest station's deck pad 1, deck yaw, 50 %", u.pad == 3 and u.parked_yaw == 180.0
      and u.status in AT_STATION, (u.pad, u.parked_yaw))
t = run(fleet, t, 9.0)
check("its plug goes in (station 2, deck pad 0)", fs.plugs and fs.plugs[-1][1:] == (2, 0, 1.0), fs.plugs[-1:])

print("8. plain ChargeStation: as before / 通常パッドは従来どおり")
cs = S.ChargeStation([(0.0, 0.0, 0.02), (1.8, 0.0, 0.02), (3.6, 0.0, 0.02)])
check("location = mean, free_pad = first free", cs.location == (1.8, 0.0, 0.02) and cs.free_pad() == 0)
u, w = Unit(1, 100.0), Unit(2, 40.0)
u.yaw, w.yaw, w.pos = 33.0, 77.0, (5.0, 0.0, 10.0)
van = Van(1)
fleet = fleet_for(cs, [u, w], [])
check("no deck", fleet.deck is None)
fleet.start([])
check("standbys parked without a yaw change", u.pad == 0 and w.pad == 1 and u.parked_yaw == 33.0)
cs.release(w)
w.status, w.pad = DOCKED, None
check("send_to_station picks the first free pad", fleet.send_to_station(w, 0.0) and w.pad_target == 1)
w.flight.state = LAND
t = run(fleet, 0.0, 0.5)
check("no squaring on a plain pad", w.yaw == 77.0)
w.arrive = True
t = run(fleet, t, 0.6)
check("parked on pad 2, own yaw", w.pad == 1 and w.parked_yaw == 77.0 and w.status in AT_STATION)
fleet.request(van, 20.0)
t = run(fleet, t, 0.7)
check("standby lifts off at once (no plug)", van.incoming is u and u.status == TO_VAN and
      u.transfers[-1] == (0.7, 1) and 0 not in cs.taken, u.transfers)
fake = types.SimpleNamespace(pads=[], release=lambda u: None)
fl = S.DroneFleet([Unit(3)], fake, BatteryModel(), None, None)
fl.step(0.0, 0.1, (0.0, 0.0))
check("a bare station object (no deck) still steps", fl.deck is None)

print("\n%d failure(s)" % len(FAILS))
sys.exit(1 if FAILS else 0)
