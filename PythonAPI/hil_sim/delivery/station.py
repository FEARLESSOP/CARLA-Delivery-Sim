"""The charge station and the pool of drones (one per van plus standbys).

Drones are not tied to a van for good. Each van starts with one; the rest
wait on charge-station pads as standbys. When a van's drone can't make its
next delivery and charging in the van would take too long, the drone flies
to the station and the van asks for a standby. The fleet sends the station
drone with the most charge that can reach the van and still make the
delivery; a drone that went to charge becomes a standby itself once it has
charge again, ready for whichever van needs one next ("swap back when
needed"). Crashed drones are recovered after a while and return to the
station with a part-charged battery.

充電ステーションとドローンの共用プール（車ごとに 1 機＋待機機）。車の機体が
次の配達に足りず、車内充電では時間がかかりすぎるとき、機体はステーションへ
飛び、車は待機機を要請する。プールは車まで飛べて配達もこなせる最も充電の
多い待機機を送る。充電に行った機体は充電後に待機機となる。墜落機は一定時間後
に回収され、一部充電の状態でステーションに戻る。

With LastMile charging stations the pads are the stations' drone decks
(:class:`DeckStation`, 3 pads each): a drone that lands is squared to the
deck, plugged in ~5 s after touch-down and unplugged before it lifts off,
as in the web viewer's charging tab.
LastMile 充電ステーションではデッキの 3 パッドを使う。着地した機体は向きを
揃え、約 5 秒後にプラグを差し、離陸前に抜く（Web ビューアと同じ）。
"""

import math

from delivery.backends import (IN_VAN, AT_STATION, FLYING, CHARGING, STANDBY, CRASHED,
                               TO_STATION, TO_VAN, DELIVERING)
from delivery.drone import LAND

# drone-deck plug timing (lmh_parts.json station.timeline.deck) / デッキのプラグ時間 [s]
DECK_PLUG_DELAY_S = 5.0     # touch-down -> plug starts in / 着地からプラグ挿入開始まで
DECK_PLUG_IN_S = 1.4        # plug going in / 挿入
DECK_PLUG_OUT_S = 0.6       # plug coming out / 抜去
DECK_LIFT_GAP_S = 1.1       # plug out -> lift-off / 抜去完了から離陸まで
DECK_SQUARE_DEG_S = 90.0    # turn rate onto the deck yaw while landing / 着陸中の旋回速度


class ChargeStation(object):
    """Landing pads (world x, y, z) where drones fast-charge. / 急速充電パッド。"""

    def __init__(self, pads):
        self.pads = [tuple(p) for p in pads]
        self.taken = {}             # pad index -> drone id (parked or on its way)

    @property
    def location(self):
        n = float(len(self.pads))
        return tuple(sum(p[i] for p in self.pads) / n for i in range(3))

    def free_pad(self):
        return next((i for i in range(len(self.pads)) if i not in self.taken), None)

    def release(self, unit):
        for i, uid in list(self.taken.items()):
            if uid == unit.id:
                del self.taken[i]


class DeckStation(ChargeStation):
    """The LastMile charging stations' drone decks as one ChargeStation.

    ``stations`` is a charge_sites.Stations: every site gives its 3 deck pads
    (``deck_pads``, z = pad surface, where the drone's leg bottoms sit) in
    station order, pad i of the list -> ``pad_owner(i)`` = (site id, deck
    pad 0..2). ``extra`` pads (x, y, z), e.g. choose_station's sidewalk
    pads, come after the decks for drones the decks can't hold (no plug, no
    yaw). ``centre`` (x, y): ``location`` is the station nearest to it
    (default: the middle of all decks).
    全ステーションのドローンデッキ（各 3 パッド）を 1 つの充電所として扱う。
    ``extra`` はデッキに入りきらない機体用の追加パッド（歩道など）。
    """

    def __init__(self, stations, extra=(), centre=None):
        pads, self.owners, self.yaws = [], [], []
        for s in stations.sites:
            pts, yaw = stations.deck_pads(s["id"])
            for k, p in enumerate(pts):
                pads.append(tuple(p))
                self.owners.append((s["id"], k))
                self.yaws.append(yaw)
        for p in extra:
            pads.append(tuple(p))
            self.owners.append(None)
            self.yaws.append(None)
        ChargeStation.__init__(self, pads)
        self.stations, self.centre = stations, centre
        self.n_deck = sum(1 for o in self.owners if o is not None)
        self._mid = {}              # site id -> middle of its deck / デッキ中心
        for sid in {o[0] for o in self.owners if o is not None}:
            ps = [p for p, o in zip(self.pads, self.owners) if o is not None and o[0] == sid]
            self._mid[sid] = tuple(sum(p[i] for p in ps) / len(ps) for i in range(3))

    def pad_owner(self, i):
        """(site id, deck pad 0..2) of pad ``i``, or None for an extra pad."""
        return self.owners[i]

    def pad_yaw(self, i):
        """Yaw (deg) a drone sits at on pad ``i`` (None: keep its own)."""
        return self.yaws[i]

    @property
    def location(self):
        if not self._mid:
            return ChargeStation.location.fget(self)
        c = self.centre
        if c is None:
            ms = list(self._mid.values())
            c = tuple(sum(m[i] for m in ms) / len(ms) for i in range(2))
        return min(self._mid.values(), key=lambda m: math.hypot(m[0] - c[0], m[1] - c[1]))

    def _group_mid(self, i):
        o = self.owners[i]
        return self.pads[i] if o is None else self._mid[o[0]]

    def free_pad(self, near=None):
        """A free pad: decks before extra pads; with ``near`` (x, y, ..) on the
        nearest station's deck; within a deck pad 1 first (the web viewer's
        landing order). / 空きパッド（``near`` があれば最寄りのデッキ）。"""
        free = [i for i in range(len(self.pads)) if i not in self.taken]
        if not free:
            return None
        if near is None:
            return free[0]

        def key(i):
            m = self._group_mid(i)
            return (self.owners[i] is None, math.hypot(m[0] - near[0], m[1] - near[1]), i)
        return min(free, key=key)


class EvChargers(object):
    """Road-side EV chargers for the pods, one pod per charger.

    ``spots`` are road points (x, y, z). ``kinds`` (optional, one per spot):
    which vehicle a charger is for -- a charging-station bay is "lmp" (bay 03,
    the LMP-4 pods) or "lmh" (bay 04, the LastMile Hubs); None = any.
    ポッド用の路側 EV 充電器（1 基 1 台）。``kinds`` はベイの車種。
    """

    def __init__(self, spots, kinds=None):
        self.spots = [tuple(p) for p in spots]
        self.kinds = list(kinds) if kinds else [None] * len(self.spots)
        self.taken = {}             # charger index -> van id

    def claim(self, van, near=None, kind=None):
        """A free charger for ``van`` (the nearest to ``near``) of ``kind``, or None."""
        free = [i for i in range(len(self.spots)) if i not in self.taken
                and (kind is None or self.kinds[i] in (None, kind))]
        if not free:
            return None
        if near is not None:
            free.sort(key=lambda i: math.hypot(self.spots[i][0] - near[0],
                                               self.spots[i][1] - near[1]))
        self.taken[free[0]] = van.vid
        return free[0]

    def release(self, van):
        for i, vid in list(self.taken.items()):
            if vid == van.vid:
                del self.taken[i]


class DroneFleet(object):
    """Charging, swaps, standby dispatch and recovery for every drone.

    ``cruise_z(a, b)`` gives a safe height for a straight flight a -> b; ``log``
    is the run's event log. With a :class:`DeckStation` the deck plugs are
    driven through its Stations object (or ``deck`` / :meth:`attach_deck`).
    全機の充電・交換・待機機派遣・回収を管理する。DeckStation ならデッキの
    プラグも動かす。
    """

    def __init__(self, units, station, battery_model, cruise_z, log, recover_s=90.0,
                 repair_level=50.0, wind_margin=3.0, wind_hold=10.0, pod_draw=0.02,
                 rain_ground=20.0, deck=None):
        self.units, self.station, self.bm = units, station, battery_model
        self.cruise_z, self.log = cruise_z, log
        self.recover_s, self.repair_level = recover_s, repair_level
        self.wind_margin = wind_margin
        self.wind_hold = wind_hold  # no take-offs above this wind (0 = never hold)
        self.pod_draw = pod_draw    # pod-% used per drone-% charged in a pod
        self.requests = {}          # van -> % it needs for its next delivery
        self.swaps = self.charge_trips = self.repairs = 0
        self._wind = 0.0
        self.rain = 0.0             # CARLA precipitation 0-100 (delivery_sim reads it) / 雨量
        self.rain_ground = rain_ground  # at this rain flying drones come down (0 = never)
        self._deck_src = deck       # charge_sites.Stations override / プラグ操作先
        self._deck_pads = {}        # deck pad -> {"uid", "since", "out", "from"} / パッドのタイマー
        self._plug_u = {}           # deck pad -> plug position last sent / 送信済みプラグ位置
        self._launch = {}           # drone id -> (unit, van, z, lift-off t) / 抜去待ちの発進

    # -- drone deck / ドローンデッキ
    def attach_deck(self, stations):
        """Drive the deck plugs of ``stations`` (charge_sites.Stations).
        デッキのプラグを動かす Stations を設定する。"""
        self._deck_src = stations

    @property
    def deck(self):
        """The Stations whose deck plugs move, or None (plain pads).
        プラグを動かす Stations（通常パッドなら None）。"""
        if not hasattr(self.station, "pad_owner"):
            return None
        return self._deck_src or getattr(self.station, "stations", None)

    def _pad_yaw(self, pad):
        f = getattr(self.station, "pad_yaw", None)
        return None if f is None or pad is None else f(pad)

    def _free_pad(self, near):
        if self.deck is not None:
            return self.station.free_pad(near)
        return self.station.free_pad()

    def _land_on(self, u, pad, t, status=CHARGING, repair=None):
        """Put ``u`` on ``pad`` (squared to the deck) and start its plug timer;
        ``t`` None = already plugged in. / パッドに載せ、プラグのタイマーを開始。"""
        yaw = self._pad_yaw(pad)
        if yaw is not None:
            u.yaw = yaw
        if repair is None:
            u.park(pad, self.station.pads[pad], status)
        else:
            u.repair(pad, self.station.pads[pad], repair)
        if self.deck is not None and self.station.pad_owner(pad) is not None:
            self._deck_pads[pad] = {"uid": u.id, "since": t, "out": None, "from": 0.0}

    @staticmethod
    def _plug_at(st, t):
        """Plug position 0 (out) .. 1 (in the socket) of a pad timer at ``t``."""
        if st["out"] is not None:
            return max(0.0, st["from"] - (t - st["out"]) / DECK_PLUG_OUT_S)
        if st["since"] is None:
            return 1.0
        return min(1.0, max(0.0, (t - st["since"] - DECK_PLUG_DELAY_S) / DECK_PLUG_IN_S))

    def _unplug(self, pad, t):
        st = self._deck_pads[pad]
        if st["out"] is None:
            st["from"], st["out"] = self._plug_at(st, t), t

    def _replug(self, u, t):
        """A launch called off: the plug goes back in from where it is.
        発進取り消し：プラグを今の位置から差し直す。"""
        st = self._deck_pads.get(getattr(u, "pad", None))
        if st is not None and st["uid"] == u.id and st["out"] is not None:
            cur = self._plug_at(st, t)
            st["out"], st["since"] = None, t - DECK_PLUG_DELAY_S - cur * DECK_PLUG_IN_S
        u.note = ""

    def _hold_for_plug(self, u, van, z, t):
        """``u`` is plugged into a deck: pull the plug and lift off after it
        (True), or nothing to pull (False). / プラグを抜いてから離陸させる。"""
        pad = getattr(u, "pad", None)
        st = self._deck_pads.get(pad)
        if st is None or st["uid"] != u.id or self._plug_at(st, t) <= 0.0:
            return False
        self._unplug(pad, t)
        ready = t + st["from"] * DECK_PLUG_OUT_S + DECK_LIFT_GAP_S
        self._launch[u.id] = (u, van, z, ready)
        u.note = "unplugging for van %d" % van.vid
        return True

    def _square(self, u, dt):
        """Landing on a deck pad: turn onto the deck yaw on the way down.
        デッキへ降下中に機首をデッキの向きへ回す。"""
        if u.status != TO_STATION or getattr(getattr(u, "flight", None), "state", None) != LAND:
            return
        yaw = self._pad_yaw(getattr(u, "pad_target", None))
        if yaw is None:
            return
        d = (yaw - u.yaw + 180.0) % 360.0 - 180.0
        step = DECK_SQUARE_DEG_S * dt
        u.yaw += max(-step, min(step, d))

    def _deck_step(self, t):
        """Lift-offs whose plug is out, and every deck plug's position.
        抜去済みの機体を離陸させ、各デッキのプラグを動かす。"""
        deck = self.deck
        if deck is None:
            return
        for uid, (u, van, z, ready) in list(self._launch.items()):
            if t + 1e-9 < ready:
                continue
            del self._launch[uid]
            if van.incoming is not u or self.grounded or u.status not in AT_STATION:
                if van.incoming is u:
                    van.incoming = None     # the van asks again / 車が再要請する
                self._replug(u, t)
                continue
            self.station.release(u)
            u.note = ""
            u.start_transfer(van.dock_pos, z, to_van=van)
        units = {u.id: u for u in self.units}
        for pad, st in list(self._deck_pads.items()):
            u = units.get(st["uid"])
            here = u is not None and u.status in AT_STATION and getattr(u, "pad", None) == pad
            if not here:
                self._unplug(pad, t)        # left without the hold: pull it now / 即抜去
            v = round(self._plug_at(st, t), 3)
            if self._plug_u.get(pad, 0.0) != v:
                self._plug_u[pad] = v
                site, k = self.station.pad_owner(pad)
                deck.deck_plug(site, k, v)
            if not here and v <= 0.0:
                del self._deck_pads[pad]

    # -- setup / 初期配置
    def start(self, vans):
        """One drone per van (four per LastMile pod, on its deck); the rest
        wait at the station. / 車ごとに 1 機（LastMile ポッドは 4 機）、残りは待機。"""
        pool = iter(self.units)
        used = 0
        for van in vans:
            for _ in range(4 if getattr(van, "lmp", False) else 1):
                u = next(pool, None)
                if u is None:
                    break
                van.accept_drone(u)
                used += 1
        for u in self.units[used:]:
            pad = self.station.free_pad()
            if pad is None:
                raise RuntimeError("%d pads, not enough for the standby drones / 待機機のパッドが足りない"
                                   % len(self.station.pads))
            self.station.taken[pad] = u.id
            self._land_on(u, pad, None, STANDBY)       # plugged in already / 充電中から開始

    @property
    def grounded(self):
        """Wind too strong to send a drone up (it would be blown into things).

        Drones already in the air carry on; nothing new takes off.
        強風で新たな離陸を禁止（飛行中の機体はそのまま）。
        """
        return self.raining or (self.wind_hold > 0 and self._wind > self.wind_hold)

    @property
    def raining(self):
        """Rain: every drone in the air comes down and none takes off.
        雨: 飛行中の機体はすべて墜落し、離陸もしない。"""
        return self.rain_ground > 0 and self.rain >= self.rain_ground

    def ground_reason(self):
        """Why take-offs are held, for notes and logs. / 離陸禁止の理由。"""
        if self.raining:
            return "rain %.0f%%" % self.rain
        return "wind %.1f m/s" % self._wind

    def plan_wind(self):
        """Wind speed to plan energy for: what blows now, plus a margin."""
        return self._wind + self.wind_margin

    # -- requests from vans / 車からの要請
    def request(self, van, need):
        self.requests[van] = need

    def cancel(self, van):
        self.requests.pop(van, None)

    def send_to_station(self, unit, t):
        """Fly ``unit`` to a free pad if it can make it. / 届くならステーションへ。"""
        src = unit.world_pos()
        pad = self._free_pad(src)
        if pad is None or self.grounded:
            return False
        dst = self.station.pads[pad]
        z = self.cruise_z(src, dst)
        cost = self.bm.transfer_cost(src, dst, z, unit.h_speed, unit.v_speed,
                                     self.plan_wind())
        if unit.battery.level < cost + 5.0:     # land with 5 % to spare
            return False
        van = unit.van
        self.station.taken[pad] = unit.id
        unit.pad_target = pad
        unit.start_transfer(lambda: dst, z)
        if van is not None:
            van.release_drone(unit)
        self.charge_trips += 1
        self.log(t, "drone_to_station", van=van.vid if van else None,
                 pos=src, detail="D%d at %.0f%%, needs %.0f%% to get there"
                 % (unit.id, unit.battery.level, cost))
        return True

    def recall(self, unit, t):
        """A drone flying to a van that no longer wants it goes back to charge."""
        if unit.id in self._launch:     # still on the deck: stays to charge / デッキに残る
            del self._launch[unit.id]
            self._replug(unit, t)
        elif unit.status == TO_VAN and not self.send_to_station(unit, t):
            unit.note = "no free pad"

    # -- per tick / 毎 tick
    def step(self, t, dt, wind):
        self._wind = math.hypot(*wind)
        deck = self.deck
        for u in self.units:
            if u.status in IN_VAN:
                # trickle-charged from the pod's own battery (not from a flat one)
                # ポッドの電池から低速充電（ポッドが空なら充電しない）
                pod = getattr(u.van, "energy", None)
                if u.battery.level < 100.0 and (pod is None or pod.level > 1.0):
                    u.battery.charge(dt, self.bm.slow)
                    if pod is not None:
                        pod.drain(dt, self.bm.slow * self.pod_draw)
            elif u.status in AT_STATION:
                u.battery.charge(dt, self.bm.fast)
                u.status = STANDBY if u.battery.level >= 99.5 else CHARGING
            elif u.status == CRASHED and t - u.down_t > self.recover_s:
                self._repair(u, t)
            elif u.status == DELIVERING and getattr(u.flight, "holding", False):
                self._check_hold(u, t)
            was = u.status
            if self.raining and u.status in FLYING:
                # rain shorts the motors: down it comes / 雨でモーターが止まり墜落
                u._crash(t, "rain")
                self._route(u, was, ("crashed", "rain"), t)
                continue
            if deck is not None:
                self._square(u, dt)
            for ev in u.update(t, dt, wind):
                self._route(u, was, ev, t)
            u.collect_parcels(t)
        self._serve(t)
        self._deck_step(t)

    def _check_hold(self, u, t):
        """A drone circling its meeting point goes to charge before it can't.
        合流地点で待つ機体は、充電所へ飛べなくなる前に向かう。"""
        pad = self._free_pad(u.pos)
        if pad is None:
            return
        dst = self.station.pads[pad]
        cost = self.bm.transfer_cost(u.pos, dst, self.cruise_z(u.pos, dst), u.h_speed,
                                     u.v_speed, self.plan_wind())
        if u.battery.level < cost + 8.0:
            van, waited = u.van, u.flight.hold_s
            if self.send_to_station(u, t):
                self.log(t, "drone_divert", van=van.vid if van else None, pos=u.pos,
                         detail="D%d waited %.0f s at the meeting point, %.0f%% left: to charge"
                         % (u.id, waited, u.battery.level))

    def _route(self, u, was, ev, t):
        kind, data = ev
        if kind == "arrived" and was == TO_STATION:
            pad = u.pad_target
            self._land_on(u, pad, t)
            self.log(t, "drone_charging", pos=u.pos,
                     detail="D%d on pad %d at %.0f%%" % (u.id, pad + 1, u.battery.level))
        elif kind == "arrived" and was == TO_VAN:
            van = u.van
            van.accept_drone(u)
            van.incoming = None
            self.swaps += 1
            self.log(t, "drone_arrived", van=van.vid, pos=van.roof(),
                     detail="D%d at %.0f%%" % (u.id, u.battery.level))
        elif kind == "crashed":
            if was == TO_STATION:
                self.station.release(u)
            self.log(t, "drone_crash", van=u.van.vid if u.van else None, pos=u.pos,
                     detail="D%d: %s (battery %.0f%%, wind %.1f m/s%s)"
                     % (u.id, data, u.battery.level, self._wind,
                        ", " + u.crash_detail if data.startswith("collision") else ""))
            if u.van is not None:
                u.van.on_drone_event(t, u, was, ev)
        elif u.van is not None:
            u.van.on_drone_event(t, u, was, ev)

    def _serve(self, t):
        """Send standbys to vans that are waiting for one. / 待機機を派遣。"""
        if self.grounded:
            return
        for van, need in list(self.requests.items()):
            if van.drone is not None or van.incoming is not None:
                del self.requests[van]
                continue
            dest = van.dock_pos()
            best = None
            for u in self.units:
                if u.status not in AT_STATION or u.id in self._launch:
                    continue
                z = self.cruise_z(u.pos, dest)
                cost = self.bm.transfer_cost(u.pos, dest, z, u.h_speed, u.v_speed,
                                             self.plan_wind())
                if u.battery.level >= need + cost and (
                        best is None or u.battery.level > best[0].battery.level):
                    best = (u, z)
            if best is None:
                continue
            u, z = best
            if not self._hold_for_plug(u, van, z, t):  # deck: lifts off once unplugged / 抜去後に離陸
                self.station.release(u)
                u.start_transfer(van.dock_pos, z, to_van=van)
            van.incoming = u
            del self.requests[van]
            self.log(t, "standby_sent", van=van.vid, pos=u.pos,
                     detail="D%d at %.0f%% (van needs %.0f%%)" % (u.id, u.battery.level, need))

    def _repair(self, u, t):
        pad = self._free_pad(u.pos)
        if pad is None:
            return
        self.station.taken[pad] = u.id
        reason = u.fail_reason
        self._land_on(u, pad, t, repair=self.repair_level)
        self.repairs += 1
        self.log(t, "drone_repaired", pos=u.pos,
                 detail="D%d back at the station after %s, battery %.0f%%"
                 % (u.id, reason, u.battery.level))

    # -- reporting / 集計
    def counts(self):
        c = {}
        for u in self.units:
            c[u.status] = c.get(u.status, 0) + 1
        return c

    def summary(self):
        return {"drones": len(self.units), "swaps": self.swaps,
                "charge_trips": self.charge_trips, "repairs": self.repairs,
                "crashes": sum(u.crashes for u in self.units),
                "trips": sum(u.trips for u in self.units),
                "drone_km": round(sum(u.km for u in self.units), 3),
                "status": self.counts()}

