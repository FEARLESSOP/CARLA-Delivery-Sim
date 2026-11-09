#!/usr/bin/env python3
"""bus_schedule.py - timetabled buses at Oita Station (GTFS-JP timetable).

Real Oita buses drive into the map, call at their stops (Oita Station's
platforms 1-8, the 降車専用 pole, 上野の森口, 要町 and the city stops on
the way) and leave it again, as the published timetable says.

  data:   data/oita_bus/ (stops.json, routes.json, timetable.json; read
          data/oita_bus/sources.md first: sources, schema, gaps)
  on:     maps whose maps.json entry has "bus_schedule": "data/oita_bus"
          (generate_traffic.py --bus-schedule auto, the default)
  report: carla-venv/bin/python bus_schedule.py report --xodr maps/build/OitaStation/OitaStation.xodr
  live:   maps/bus_live.py (after the new OitaStation is imported)

How it works / 仕組み
  * Planning (offline-testable, a carla.Map is enough): every pass of a
    route pattern through the map becomes a lane path. Each stop goes to the
    kerb-side (left-most, keep-left) driving lane next to its pole, with the
    pole on the bus's left; the map-edge entry / exit goes to an inbound /
    outbound lane just inside the ring road; the candidate lanes are chosen
    together so the whole path is shortest (delivery/route.py
    SafeRoutePlanner; CARLA's planner runs away on these maps, guide ch. 3).
    A stop that would put the 10.4 m bus into a junction is moved back.
  * Spawn time: the first in-map stop time minus the path length at a
    typical bus speed (and never before the last stop outside the map).
    Trips that start at the station come in empty (回送) from the edge their
    route leaves by and wait at the pole; trips that end there leave empty.
  * Driving: the Traffic Manager drives each bus along its path
    (set_path, no own lane changes), with the map's country profile (Japan:
    lights, 止まれ, walkers, lane speeds via traffic_profile.RuleKeeper).
    This module slows the bus down to its stop (door at the pole), holds it
    for the dwell (boarding 20-40 s, alighting 10-20 s; at Oita Station
    until the scheduled departure), blinks left when pulling in and right
    when pulling out, never stops in a junction (waits before it when there
    is no room behind the bus ahead), and removes it at the exit edge.
    The stricter of the two speed wishes wins (SpeedArbiter).
  * Safety: at most ``max_buses`` at once; a bus without progress for
    ``stuck_s`` is removed and logged; every bus goes at exit (and so after
    the F5 map switch, which restarts generate_traffic).
  * Clock: by default the timetable runs at REAL time from the time of day
    the sim shows (data/weather_status.json); the day/night cycle usually
    runs a day in 5-15 minutes, which would flood the station. Changing the
    clock on the dashboard (clock_cmd) re-anchors it. --bus-time / --bus-day
    fix it for experiments. The day type follows the date (weekday /
    saturday / sunday_holiday; every national holiday is sunday_holiday,
    also 2026-11-03 and 11-23, which the 大分バス feed misses).
  * Status for the dashboard: data/bus_status.json (buses on the map,
    next departures per platform).

Credit / 出典 (CC BY 4.0, must be shown with the data):
  出典：大分県オープンデータカタログ（BODIK）「大分バスGTFSデータ」「大分交通GTFSデータ」
  「高速バスGTFSデータ」（CC BY 4.0、大分県）を加工して作成; road names © OpenStreetMap contributors.

大分駅の路線バス: 公開時刻表 (GTFS-JP) どおりにバスがマップへ入り、停留所
（大分駅前 1〜8 番のりば・降車専用・上野の森口・要町ほか）に停まり、出て行く。
停留所はポールの左側の歩道寄り車線、マップ端は環状道路の内側の流入・流出車線。
時刻はシムの表示時刻から実時間で進める（昼夜サイクルは 1 日 5〜15 分のため）。
"""

import argparse
import datetime
import json
import math
import os
import random
import re
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(HERE, '..', 'carla'))          # agents (SafeRoutePlanner)
MAPS_PATH = os.path.join(HERE, 'maps', 'maps.json')
STATUS_PATH = os.path.join(HERE, 'data', 'bus_status.json')
WEATHER_STATUS = os.path.join(HERE, 'data', 'weather_status.json')
WEATHER_SETTINGS = os.path.join(HERE, 'weather_cycle.json')

ROLE = 'timetable_bus'          # role_name of our buses / 本モジュールのバスの role_name
CREDIT = ('出典：大分県オープンデータカタログ（BODIK）「大分バスGTFSデータ」「大分交通GTFSデータ」'
          '「高速バスGTFSデータ」（CC BY 4.0、大分県）を加工して作成; road names © OpenStreetMap contributors')

# Blueprints, first found wins: the Oita bus (cars.json "oitabus", npc_only), then
# CARLA's own bus as a stand-in. door_m: boarding door's position along the bus,
# from the actor origin (+ forward); the Erga's middle door is 0.85 m behind the
# mid-axle point. / ブループリント（先に見つかったもの）。door_m: 乗車扉の前後位置
BUS_MODELS = [
    {'id': 'vehicle.oitabus.erga', 'length_m': 10.43, 'door_m': -0.85},
    {'id': 'vehicle.mitsubishi.fusorosa', 'length_m': 10.3, 'door_m': -0.85},
]

DEFAULTS = {
    'max_buses': 25,           # at once / 同時に走るバスの上限
    'stuck_s': 180.0,          # no progress this long -> removed / これだけ進まなければ削除
    'bus_kmh': 40.0,           # never faster than this (and the lane limit) / 上限速度
    'plan_kmh': 18.0,          # typical speed for the spawn times (signals incl.) / 時刻計算の平均速度
    'layover_s': 150.0,        # a departing bus is at its pole this early / 始発バスの到着の早さ
    'station_hold_max_s': 300.0,   # longest hold at Oita Station / 大分駅での最長待機
    'dwell_s': {'both': [20, 40], 'pick_up_only': [20, 35], 'drop_off_only': [10, 20]},
    'approach_m': 60.0,        # stop control starts this far before the stop / 停車制御の開始距離
    'decel': 1.0,              # gentle bus braking to the stop (m/s2) / 停車時の減速度
    'creep_kmh': 6.0,
    'creep_m': 3.0,
    'hold_decel': 3.0,         # the TM's own braking once told to stop / TM の停止時減速度
    'stop_margin_m': 0.3,
    'still_mps': 0.15,
    'stop_tol_m': 3.0,         # door within this of the pole = arrived / 到着とみなす距離
    'blink_s': 3.0,            # right blinker when pulling out / 発車時の右ウインカー
    'spawn_clear_m': 15.0,     # no vehicle this close to the spawn point / 生成位置の空き
    'spawn_late_max_s': 240.0, # give up a spawn this late / これ以上遅れたら生成しない
    'free_drive_s': 45.0,      # a bus with no way out drives on this long, then goes / 出口の無いバス
    'reroute_max': 2,
    'off_route_m': 10.0,
    'junction_room_m': 3.0,    # wait before a junction without room behind the bus ahead
    'junction_wait_max_s': 60.0,
    'stop_gap_ok_m': 6.0,      # pole within this of the kerb lane's edge = clean / 綺麗に対応
    'stop_gap_max_m': 25.0,    # farther = not mapped / これ以上離れたら対応不可
    'plan_ahead_s': 900.0,     # plan visits due within this / この先の便を事前に計画
    'status_period_s': 2.0,
    'seed': None,
}

DAY_TYPES = ('weekday', 'saturday', 'sunday_holiday')


# ---------------------------------------------------------------------------
# time and calendar / 時刻と暦
# ---------------------------------------------------------------------------
def gtfs_seconds(text):
    """'07:05' / '07:05:30' / '32:10' (GTFS, past midnight) -> seconds since
    the service day's midnight (32:10 -> 115800). None for None / ''.
    GTFS 時刻（24 時超え可）を秒へ。"""
    if text is None or text == '':
        return None
    parts = [int(p) for p in str(text).strip().split(':')]
    while len(parts) < 3:
        parts.append(0)
    return parts[0] * 3600 + parts[1] * 60 + parts[2]


def hhmm(sec):
    """Seconds -> 'HH:MM:SS' (hours may pass 24, as GTFS). / 秒を時刻表記へ"""
    sec = int(round(sec))
    return '%02d:%02d:%02d' % (sec // 3600, (sec // 60) % 60, sec % 60)


def _nth_weekday(year, month, weekday, n):
    d = datetime.date(year, month, 1)
    d += datetime.timedelta(days=(weekday - d.weekday()) % 7)
    return d + datetime.timedelta(weeks=n - 1)


def jp_holidays(year):
    """Japanese national holidays of ``year`` (国民の祝日に関する法律, 2020s
    rules): fixed days, Happy Mondays, the equinoxes, substitute holidays
    (振替休日) and the citizens' holiday between two holidays (国民の休日).
    日本の祝日（振替休日・国民の休日を含む）。"""
    d = datetime.date
    days = {d(year, 1, 1), d(year, 2, 11), d(year, 2, 23), d(year, 4, 29), d(year, 5, 3),
            d(year, 5, 4), d(year, 5, 5), d(year, 8, 11), d(year, 11, 3), d(year, 11, 23),
            _nth_weekday(year, 1, 0, 2), _nth_weekday(year, 7, 0, 3),
            _nth_weekday(year, 9, 0, 3), _nth_weekday(year, 10, 0, 2)}
    k = int((year - 1980) / 4)
    days.add(d(year, 3, int(20.8431 + 0.242194 * (year - 1980) - k)))     # 春分の日
    days.add(d(year, 9, int(23.2488 + 0.242194 * (year - 1980) - k)))     # 秋分の日
    # 国民の休日: a weekday between two holidays / 祝日に挟まれた平日
    for h in sorted(days):
        mid = h + datetime.timedelta(days=1)
        if mid not in days and (h + datetime.timedelta(days=2)) in days and mid.weekday() != 6:
            days.add(mid)
    # 振替休日: a holiday on a Sunday moves to the next non-holiday / 日曜の祝日の振替
    for h in sorted(days):
        if h.weekday() == 6:
            sub = h + datetime.timedelta(days=1)
            while sub in days:
                sub += datetime.timedelta(days=1)
            days.add(sub)
    return days


def day_type(date):
    """'weekday' | 'saturday' | 'sunday_holiday' for a date. Every national
    holiday is sunday_holiday (the 大分バス feed misses 2026-11-03 / 11-23).
    日付のダイヤ種別（祝日は日祝ダイヤ）。"""
    if date.weekday() == 6 or date in jp_holidays(date.year):
        return 'sunday_holiday'
    if date.weekday() == 5:
        return 'saturday'
    return 'weekday'


# ---------------------------------------------------------------------------
# the timetable data / 時刻表データ
# ---------------------------------------------------------------------------
def _load(path):
    with open(path, encoding='utf-8') as fh:
        return json.load(fh)


def map_config(map_name, maps=None):
    """(folder, options) when the loaded map's maps.json entry has
    "bus_schedule" (a folder, or {"data": folder, ...options}), else None.
    maps.json に bus_schedule があるマップだけ有効。"""
    import traffic_profile
    if maps is None:
        try:
            maps = _load(MAPS_PATH)
        except (OSError, ValueError):
            return None
    entry = traffic_profile.map_entry(map_name, maps)
    if not entry or not entry.get('bus_schedule'):
        return None
    cfg = entry['bus_schedule']
    opts = {}
    if isinstance(cfg, dict):
        opts = {k: v for k, v in cfg.items() if k in DEFAULTS}
        cfg = cfg.get('data')
    if not cfg:
        return None
    folder = cfg if os.path.isabs(cfg) else os.path.join(HERE, cfg)
    return folder, opts


class Timetable(object):
    """stops.json + routes.json + timetable.json (data/oita_bus, sources.md).
    `visits(day)` turns the trips of a day type into map visits: one per
    pass of the trip's route pattern through the map (a route that leaves the
    map and comes back is two visits). / 時刻表。便をマップ通過単位に分ける。"""

    def __init__(self, folder):
        self.folder = folder
        self.stops = {s['id']: s for s in _load(os.path.join(folder, 'stops.json'))}
        self.patterns = {}
        for p in _load(os.path.join(folder, 'routes.json')):
            self.patterns.setdefault(p['route_id'], []).append(p)
        try:
            # GTFS shape of each pass (tools/shapes.py): the roads the real
            # route uses / 実際の路線の形状
            self.shapes = _load(os.path.join(folder, 'shapes.json'))
        except (OSError, ValueError):
            self.shapes = {}
        tt = _load(os.path.join(folder, 'timetable.json'))
        self.trips = {}
        for t in tt['trips']:
            self.trips.setdefault(t['day_type'], []).append(t)
        self.unmatched = 0

    def pattern_of(self, trip):
        seq = [s[0] for s in trip['in_map']]
        for p in self.patterns.get(trip['route_id'], ()):
            if p['stops_in_map'] == seq:
                return p
        return None

    @staticmethod
    def pass_key(pattern, i):
        return '%s|%s#%d' % (pattern['route_id'], ','.join(pattern['stops_in_map']), i)

    def visits(self, dtype):
        """Visits of one day type, sorted by their first stop time. Each:
        trip, route, pass_key, pass_index, stops [(id, arr_s, dep_s, flag)],
        entry, exit (edge dicts or {'kind': 'starts_in_map' | 'ends_in_map'}),
        prev_s (time at the last stop before the map, first pass only).
        便を通過単位の訪問へ。"""
        out = []
        for trip in self.trips.get(dtype, ()):
            pat = self.pattern_of(trip)
            if pat is None:
                self.unmatched += 1
                continue
            passes = pat.get('map_passes') or [{'stops': pat['stops_in_map'],
                                                 'entry': pat['entry'], 'exit': pat['exit']}]
            calls = list(trip['in_map'])
            k = 0
            for i, ps in enumerate(passes):
                n = len(ps['stops'])
                mine = calls[k:k + n]
                k += n
                if not mine:
                    continue     # crosses a corner without a stop: no time known / 停留所無しの通過
                out.append({
                    'trip': trip['gtfs_trip_id'], 'route': trip['route_id'],
                    'name': pat.get('name') or '', 'dest': pat.get('destination') or '',
                    'operator': pat.get('operator') or '', 'type': trip.get('type'),
                    'pass_key': self.pass_key(pat, i), 'pass_index': i, 'passes': len(passes),
                    'stops': [(c[0], gtfs_seconds(c[1]), gtfs_seconds(c[2]), c[3] or '') for c in mine],
                    'entry': ps['entry'], 'exit': ps['exit'],
                    'other_edges': [e for j, q in enumerate(passes) if j != i
                                    for e in (q['entry'], q['exit']) if _is_edge(e)],
                    'prev_s': gtfs_seconds(trip.get('prev_stop_time')) if i == 0 else None,
                })
        out.sort(key=lambda v: v['stops'][0][1])
        return out


def _is_edge(e):
    return bool(e) and str(e.get('kind', '')).startswith('boundary')


# ---------------------------------------------------------------------------
# planning on the road logic (a carla.Map; no server needed)
# 道路ロジック上の計画（carla.Map だけで可）
# ---------------------------------------------------------------------------
def _v(x, y):
    return (x, y)


def _fwd(w):
    r = math.radians(w.transform.rotation.yaw)
    return math.cos(r), math.sin(r)


def _xy(w):
    l = w.transform.location
    return l.x, l.y


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1]


def _right(f):
    return -f[1], f[0]          # CARLA: x east, y south -> right of (1, 0) is (0, 1)


class LanePlanner(object):
    """Maps stops and map edges onto lanes and plans each visit's path.
    `wmap`: carla.Map (live or carla.Map(name, xodr)); `carla`: the module.
    停留所・マップ端を車線へ対応させ、訪問ごとの経路を計画する。"""

    def __init__(self, wmap, carla, cfg=None, timetable=None, xodr=None, step=2.0):
        self.map = wmap
        self.carla = carla
        self.cfg = dict(DEFAULTS, **(cfg or {}))
        self.tt = timetable
        self.step = step
        from delivery.route import SafeRoutePlanner
        self.planner = SafeRoutePlanner(wmap, step)
        self.driving = carla.LaneType.Driving
        xodr = xodr if xodr is not None else wmap.to_opendrive()
        self.bounds = self._header_bounds(xodr)
        self.grid = {}
        self.cell = 10.0
        for w in wmap.generate_waypoints(step):
            if w.lane_type != self.driving:
                continue
            x, y = _xy(w)
            self.grid.setdefault((int(x // self.cell), int(y // self.cell)), []).append(w)
        self.model = BUS_MODELS[0]
        self._stop_cands = {}
        self._trace = {}
        self.plans = {}
        self.problems = {}

    @staticmethod
    def _header_bounds(xodr):
        m = re.search(r'<header[^>]*?north="([-\d.]+)"[^>]*?south="([-\d.]+)"[^>]*?'
                      r'east="([-\d.]+)"[^>]*?west="([-\d.]+)"', xodr[:4000])
        if not m:
            return None
        n, s, e, w = (float(v) for v in m.groups())
        return {'north': n, 'south': s, 'east': e, 'west': w}

    def covers(self, stops, slack=15.0):
        """Does the loaded road logic reach every stop? (The old OitaStation
        level, +-423 m, does not: no buses there.) / 全停留所が道路ロジックの範囲内か"""
        b = self.bounds
        if not b:
            return True
        for s in stops:
            if not (b['west'] - slack <= s['x'] <= b['east'] + slack
                    and b['south'] - slack <= s['y'] <= b['north'] + slack):
                return False
        return True

    def near(self, x, y, r):
        """Driving-lane waypoints within r m of CARLA (x, y). / 近くの車線点"""
        out = []
        c = self.cell
        for i in range(int((x - r) // c), int((x + r) // c) + 1):
            for j in range(int((y - r) // c), int((y + r) // c) + 1):
                for w in self.grid.get((i, j), ()):
                    wx, wy = _xy(w)
                    if (wx - x) ** 2 + (wy - y) ** 2 <= r * r:
                        out.append(w)
        return out

    # -- lanes -------------------------------------------------------------
    def kerb_lane(self, w):
        """The left-most driving lane of w's direction at w (keep-left: the
        kerb lane), found geometrically (netconvert --lefthand lane ids).
        同じ向きで最も左の走行車線（歩道寄り）。"""
        f = _fwd(w)
        r = _right(f)
        x0, y0 = _xy(w)
        best, best_off = w, 0.0
        for getter in ('get_left_lane', 'get_right_lane'):
            cur = w
            for _ in range(6):
                try:
                    n = getattr(cur, getter)()
                except RuntimeError:
                    n = None
                if n is None or n.lane_type != self.driving or _dot(_fwd(n), f) <= 0.0:
                    break
                nx, ny = _xy(n)
                off = _dot((nx - x0, ny - y0), r)
                if off < best_off - 0.5:
                    best, best_off = n, off
                cur = n
        return best

    def _along(self, w, d):
        """The waypoint d m along w's lane (negative = back), staying on w's
        road at forks. / 車線に沿って d m 先（負は手前）"""
        if abs(d) < 0.05:
            return w
        nxt = w.next(d) if d > 0 else w.previous(-d)
        if not nxt:
            return None
        same = [q for q in nxt if q.road_id == w.road_id]
        return (same or nxt)[0]

    def _project(self, w, x, y):
        """w moved along its lane to the point nearest (x, y). / 最寄り点へ移動"""
        for _ in range(3):
            wx, wy = _xy(w)
            lon = _dot((x - wx, y - wy), _fwd(w))
            if abs(lon) < 0.3:
                break
            n = self._along(w, lon)
            if n is None:
                break
            w = n
        return w

    def footprint_clear(self, door_wp):
        """No junction under the bus standing with its door at door_wp.
        扉位置で停車したバスが交差点に掛からないか。"""
        m = self.model
        centre = -m['door_m']
        half = m['length_m'] / 2.0 + 0.5
        d = centre - half
        while d <= centre + half + 1e-6:
            w = self._along(door_wp, d)
            if w is None or w.is_junction:
                return False
            d += 1.0
        return True

    def stop_candidates(self, stop_id):
        """Kerb-lane stopping places for a pole: [(door waypoint, gap m, shift
        m)], the pole on the bus's left, nearest first. Cached.
        ポールの左に扉が来る停車位置の候補。"""
        if stop_id in self._stop_cands:
            return self._stop_cands[stop_id]
        s = self.tt.stops[stop_id]
        px, py = s['x'], -s['y']
        rmax = self.cfg['stop_gap_max_m'] + 4.0
        nearest = {}
        for w in self.near(px, py, rmax):
            if w.is_junction:
                continue
            key = (w.road_id, w.section_id, w.lane_id)
            wx, wy = _xy(w)
            d = (wx - px) ** 2 + (wy - py) ** 2
            if key not in nearest or d < nearest[key][0]:
                nearest[key] = (d, w)
        out, seen = [], set()
        for _d, w in sorted(nearest.values(), key=lambda t: t[0]):
            k0 = self.kerb_lane(w)
            k = self._project(k0, px, py)
            if k.is_junction or k.road_id != k0.road_id:
                k = k0                         # the pole faces a junction: start here / 交差点前
            key = (k.road_id, k.lane_id)
            if key in seen:
                continue
            seen.add(key)
            f = _fwd(k)
            kx, ky = _xy(k)
            lat = _dot((px - kx, py - ky), _right(f))
            if lat > 0.5:
                continue                       # pole on the right: wrong side / ポールが右側
            gap = max(0.0, -lat - k.lane_width / 2.0)
            if gap > self.cfg['stop_gap_max_m']:
                continue
            door = k if self.footprint_clear(k) else None
            if door is None:
                for sh in [-2.0 * i for i in range(1, 16)] + [2.0 * i for i in range(1, 8)]:
                    q = self._along(k, sh)
                    if q is not None and q.road_id == k.road_id and not q.is_junction \
                            and self.footprint_clear(q):
                        door = q
                        break
                if door is None:
                    continue
            # signed offset of the door from the pole along the lane (+: pole
            # ahead) / 扉からポールまでの前後のずれ（+ はポールが前方）
            dx, dy = _xy(door)
            shift = _dot((px - dx, py - dy), _fwd(door))
            if abs(shift) > 30.0:
                continue
            out.append((door, gap, round(shift, 1)))
        out.sort(key=lambda t: t[1] + 0.2 * abs(t[2]))
        self._stop_cands[stop_id] = out[:4]
        return self._stop_cands[stop_id]

    def edge_candidates(self, edge, inbound):
        """Inbound / outbound kerb lanes near a map-edge crossing (see
        _edge_candidates); a wider search along the edge when the GTFS
        crossing has no lane of that direction close by.
        マップ端の流入・流出車線（近くに無ければ範囲を広げる）。"""
        c = self._edge_candidates(edge, inbound, 45.0)
        return c or self._edge_candidates(edge, inbound, 150.0)

    def _edge_candidates(self, edge, inbound, reach):
        """Lanes just inside the ring road at a map-edge crossing, heading
        into the map (inbound) or out of it: [waypoint], best first.
        マップ端（環状道路の内側）の流入・流出車線。"""
        x, y = float(edge['x']), -float(edge['y'])
        b = self.bounds or {'north': 640.0, 'south': -640.0, 'east': 640.0, 'west': -640.0}
        side = edge.get('side')
        if side not in ('north', 'south', 'east', 'west'):
            ax = abs(x) / max(b['east'], 1.0) > abs(y) / max(b['north'], 1.0)
            side = ('east' if x > 0 else 'west') if ax else ('south' if y > 0 else 'north')
        # inward unit vector and the edge line in CARLA coordinates / 内向き単位ベクトル
        if side == 'north':
            inward, edge_c, along = (0.0, 1.0), -b['north'], x
        elif side == 'south':
            inward, edge_c, along = (0.0, -1.0), -b['south'], x
        elif side == 'east':
            inward, edge_c, along = (-1.0, 0.0), b['east'], y
        else:
            inward, edge_c, along = (1.0, 0.0), b['west'], y
        cx = x if side in ('north', 'south') else edge_c + inward[0] * 45.0
        cy = y if side in ('east', 'west') else edge_c + inward[1] * 45.0
        found = {}
        for w in self.near(cx, cy, reach + 20.0):
            if w.is_junction:
                continue
            wx, wy = _xy(w)
            depth = (wx - edge_c) * inward[0] if side in ('east', 'west') else (wy - edge_c) * inward[1]
            if not 20.0 <= depth <= 90.0:
                continue
            tang = abs((wy if side in ('east', 'west') else wx) - along)
            if tang > reach:
                continue
            fd = _dot(_fwd(w), inward)
            if (inbound and fd < 0.5) or (not inbound and fd > -0.5):
                continue
            k = self.kerb_lane(w)
            key = (k.road_id, k.lane_id)
            score = tang + 0.3 * abs(depth - 35.0)
            if key not in found or score < found[key][0]:
                found[key] = (score, k)
        return [w for _s, w in sorted(found.values(), key=lambda t: t[0])][:3]

    def via_candidates(self, x, y, tx, ty):
        """Kerb lanes at a point of the route's GTFS shape (CARLA x, y),
        heading along the shape's tangent (tx, ty). / 形状上の経由車線"""
        found = {}
        for w in self.near(x, y, 9.0):
            if w.is_junction or _dot(_fwd(w), (tx, ty)) < 0.8:
                continue
            k = self.kerb_lane(w)
            wx, wy = _xy(w)
            d = (wx - x) ** 2 + (wy - y) ** 2
            key = (k.road_id, k.lane_id)
            if key not in found or d < found[key][0]:
                found[key] = (d, k)
        return [(w, 0.0, 0.0) for _d, w in sorted(found.values(), key=lambda t: t[0])][:2]

    def _add_vias(self, visit, layers, kinds, metas, spacing=100.0, clear=35.0):
        """Insert via layers along the pass's GTFS shape between the points,
        so the path keeps to the real route's roads (the shortest path would
        cut through side streets). / 形状に沿った経由点を挿入（裏道を避ける）"""
        shape = (self.tt.shapes or {}).get(visit['pass_key']) if self.tt else None
        if not shape or len(shape) < 2:
            return layers, kinds, metas
        pts = [(x, -y) for x, y in shape]
        arc = [0.0]
        for i in range(1, len(pts)):
            arc.append(arc[-1] + math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]))
        # shape position of each layer (None: off the shape, e.g. 回送)
        pos, lo = [], 0
        for kind, meta, layer in zip(kinds, metas, layers):
            if kind == 'stop':
                st = self.tt.stops[meta[1]]
                sx, sy = st['x'], -st['y']
            elif (kind == 'spawn' and _is_edge(visit['entry'])) or \
                    (kind == 'exit' and _is_edge(visit['exit'])):
                sx, sy = _xy(layer[0][0])         # the lane point inside the ring / 環状道路の内側
            else:
                pos.append(None)                  # 回送: off the shape / 形状外
                continue
            bi = min(range(lo, len(pts)), key=lambda k: (pts[k][0] - sx) ** 2 + (pts[k][1] - sy) ** 2)
            lo = bi
            pos.append(arc[bi])
        out_l, out_k, out_m = [layers[0]], [kinds[0]], [metas[0]]
        for i in range(1, len(layers)):
            a, b = pos[i - 1], pos[i]
            if a is not None and b is not None and b - a > 2 * clear:
                t = a + clear
                while t < b - clear:
                    k = max(1, min(len(arc) - 1, next((j for j in range(len(arc)) if arc[j] >= t), len(arc) - 1)))
                    f = (t - arc[k - 1]) / max(1e-6, arc[k] - arc[k - 1])
                    x = pts[k - 1][0] + f * (pts[k][0] - pts[k - 1][0])
                    y = pts[k - 1][1] + f * (pts[k][1] - pts[k - 1][1])
                    L = max(1e-6, math.hypot(pts[k][0] - pts[k - 1][0], pts[k][1] - pts[k - 1][1]))
                    c = self.via_candidates(x, y, (pts[k][0] - pts[k - 1][0]) / L,
                                            (pts[k][1] - pts[k - 1][1]) / L)
                    if c:
                        out_l.append(c)
                        out_k.append('via')
                        out_m.append(None)
                    t += spacing
            out_l.append(layers[i])
            out_k.append(kinds[i])
            out_m.append(metas[i])
        return out_l, out_k, out_m

    # -- paths -------------------------------------------------------------
    @staticmethod
    def _wkey(w):
        l = w.transform.location
        return (w.road_id, w.lane_id, round(l.x, 1), round(l.y, 1))

    def trace(self, a, b):
        """(length m, route) from waypoint a to b, (inf, []) when there is no
        sane way (none, or a runaway loop). Cached. / 経路と長さ（無ければ inf）"""
        key = (self._wkey(a), self._wkey(b))
        if key in self._trace:
            return self._trace[key]
        la, lb = a.transform.location, b.transform.location
        route = self.planner.trace_route(la, lb)
        res = (float('inf'), [])
        if route:
            pts = [w.transform.location for w, _o in route]
            length = sum(pts[i].distance(pts[i + 1]) for i in range(len(pts) - 1))
            straight = la.distance(lb)
            end_ok = pts[-1].distance(lb) < 6.0 and pts[0].distance(la) < 6.0
            if end_ok and length <= 3.0 * straight + 400.0:
                res = (length, route)
        self._trace[key] = res
        return res

    def plan(self, visit):
        """The lane path of a visit (cached per pass of a route pattern), or
        None with the reason in self.problems[pass_key].
        Points: entry edge (or the 回送 edge, or the first stop), stops, exit
        edge (or the 回送 exit). / 訪問の経路（失敗時は理由を problems へ）"""
        key = visit['pass_key']
        if key in self.plans:
            return self.plans[key]
        plan, why = self._plan(visit)
        self.plans[key] = plan
        if why:
            self.problems[key] = why
        return plan

    def _plan(self, visit):
        why = []
        layers, kinds, metas = [], [], []
        entry, exit_ = visit['entry'], visit['exit']
        # where it comes from / どこから来るか
        start_edge = entry if _is_edge(entry) else None
        deadhead_in = False
        if start_edge is None:
            # starts at a stop (the station): comes in empty from the edge its
            # route leaves by, else appears at the stop / 始発: 出口側の端から回送
            for e in ([exit_] if _is_edge(exit_) else []) + visit.get('other_edges', []):
                if self.edge_candidates(e, True):
                    start_edge, deadhead_in = e, True
                    break
        if start_edge is not None:
            c = self.edge_candidates(start_edge, True)
            if c:
                layers.append([(w, 0.0, 0.0) for w in c])
                kinds.append('spawn')
                metas.append(None)
            else:
                why.append('entry edge %s (%.0f, %.0f) has no inbound lane'
                           % (start_edge.get('street') or start_edge.get('side'),
                              start_edge['x'], start_edge['y']))
                deadhead_in = False
        for ci, call in enumerate(visit['stops']):
            sid = call[0]
            c = self.stop_candidates(sid)
            if not c:
                st = self.tt.stops.get(sid, {})
                why.append('stop %s %s (%.0f, %.0f): no kerb lane with the pole on its left within %.0f m'
                           % (sid, st.get('name'), st.get('x', 0), st.get('y', 0),
                              self.cfg['stop_gap_max_m']))
                return None, why
            layers.append(c)
            kinds.append('stop')
            metas.append((ci,) + tuple(call))
        end_edge = exit_ if _is_edge(exit_) else None
        deadhead_out = False
        if end_edge is None:
            for e in ([entry] if _is_edge(entry) else []) + visit.get('other_edges', []):
                if self.edge_candidates(e, False):
                    end_edge, deadhead_out = e, True
                    break
        if end_edge is not None:
            c = self.edge_candidates(end_edge, False)
            if c:
                layers.append([(w, 0.0, 0.0) for w in c])
                kinds.append('exit')
                metas.append(None)
            else:
                why.append('exit edge %s (%.0f, %.0f) has no outbound lane'
                           % (end_edge.get('street') or end_edge.get('side'),
                              end_edge['x'], end_edge['y']))
        if kinds[0] != 'spawn' and _is_edge(entry):
            why.append('spawns at its first stop instead')
        layers, kinds, metas = self._add_vias(visit, layers, kinds, metas)
        while True:
            path, fail = self._viterbi(layers)
            if fail is None:
                break
            if kinds[fail] == 'via':
                del layers[fail], kinds[fail], metas[fail]          # off the lanes: dropped
            elif kinds[fail - 1] == 'via':
                del layers[fail - 1], kinds[fail - 1], metas[fail - 1]
            elif kinds[fail] == 'exit':
                why.append('no path from the last stop to the exit edge: drives on, then goes')
                layers, kinds, metas = layers[:-1], kinds[:-1], metas[:-1]
                deadhead_out = False
            elif fail == 1 and kinds[0] == 'spawn':
                why.append('no path from the entry edge to the first stop: spawns at the stop')
                layers, kinds, metas = layers[1:], kinds[1:], metas[1:]
                deadhead_in = False
            elif kinds.count('stop') > 1:
                # a stop that cannot be reached from the one before: passed
                # without stopping / 前の停留所から行けない停留所は通過
                st = self.tt.stops.get(metas[fail][1], {})
                why.append('stop %s %s cannot be reached from %s: passed without stopping'
                           % (metas[fail][1], st.get('name'),
                              metas[fail - 1][1] if metas[fail - 1] else 'the entry'))
                del layers[fail], kinds[fail], metas[fail]
            else:
                why.append('no path to point %d (%s)' % (fail, kinds[fail]))
                return None, why
        return self._assemble(visit, layers, kinds, metas, path, why, deadhead_in,
                              deadhead_out and kinds[-1] == 'exit')

    def _viterbi(self, layers):
        """One candidate per layer, shortest whole path: (path, None), or
        (None, index of the first layer that cannot be reached).
        各層から 1 候補、全体最短。到達不能な層の番号を返す。"""
        best = [(c[1] * 3.0 + abs(c[2]), [i]) for i, c in enumerate(layers[0])]
        for li in range(1, len(layers)):
            nxt = []
            for j, cj in enumerate(layers[li]):
                opt = (float('inf'), None)
                for cost, path in best:
                    length, _r = self.trace(layers[li - 1][path[-1]][0], cj[0])
                    tot = cost + length + cj[1] * 3.0 + abs(cj[2])
                    if tot < opt[0]:
                        opt = (tot, path + [j])
                if opt[1] is not None:
                    nxt.append(opt)
            if not nxt:
                return None, li
            best = nxt
        return min(best, key=lambda t: t[0])[1], None

    def _assemble(self, visit, layers, kinds, metas, path, why, deadhead_in, deadhead_out):
        pts = []         # polyline: (x, y, junction, road) / 経路の折れ線
        marks = []       # (kind, index in pts, waypoint, extra)
        chosen = [layers[i][path[i]] for i in range(len(layers))]
        for i, (w, gap, shift) in enumerate(chosen):
            if i == 0:
                x, y = _xy(w)
                pts.append((x, y, w.is_junction, w.road_id, w.transform.location.z))
            else:
                _l, route = self.trace(chosen[i - 1][0], w)
                for rw, _o in route[1:]:
                    x, y = _xy(rw)
                    if pts and (x - pts[-1][0]) ** 2 + (y - pts[-1][1]) ** 2 < 0.25:
                        continue
                    pts.append((x, y, rw.is_junction, rw.road_id, rw.transform.location.z))
            marks.append((kinds[i], len(pts) - 1, w, gap, shift, metas[i]))
        # a point on our road may lie in another road's junction (overlapping
        # roads, e.g. the OitaStation bus pool): TM's nearest waypoint there is
        # the junction's, so count it as a junction point / 他の道路の交差点内の点も交差点扱い
        if self.map is not None:
            import carla
            for i, p in enumerate(pts):
                if not p[2]:
                    loc = carla.Location(p[0], p[1], p[4])
                    q = self.map.get_waypoint(loc)
                    if q is not None and q.is_junction and q.transform.location.distance(loc) < 0.5:
                        pts[i] = (p[0], p[1], True, p[3], p[4])
        s = [0.0]
        for i in range(1, len(pts)):
            s.append(s[-1] + math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]))
        points = []
        for kind, idx, w, gap, shift, call in marks:
            if kind == 'via':
                continue
            x, y = _xy(w)
            p = {'kind': kind, 's': s[idx], 'x': x, 'y': y, 'z': w.transform.location.z,
                 'yaw': w.transform.rotation.yaw, 'road': w.road_id, 'lane': w.lane_id}
            if kind == 'stop':
                # call: index into the visit's stops (the same for every trip of
                # this pass; the times come from each trip) / 訪問内の停留所番号
                ci, sid = call[0], call[1]
                p.update(stop=sid, call=ci, gap=round(gap, 1), shift=round(shift, 1))
            points.append(p)
        for a, b in zip(points, points[1:]):
            if b['s'] - a['s'] > 3.0 * math.hypot(b['x'] - a['x'], b['y'] - a['y']) + 400.0:
                why.append('long way round between %s and %s' % (a['kind'], b['kind']))
        plan = {'points': points, 'xs': [p[0] for p in pts], 'ys': [p[1] for p in pts],
                'junction': [p[2] for p in pts], 'roads': [p[3] for p in pts],
                'zs': [round(p[4], 2) for p in pts], 's': s,
                'deadhead_in': deadhead_in, 'deadhead_out': deadhead_out,
                'length': s[-1]}
        return plan, why


def path_locations(plan, s_from=0.0, spacing=10.0):
    """(x, y, z) points for the Traffic Manager's set_path: every ``spacing`` m,
    and the first and last point of each road outside junctions (TM picks a
    junction's way out by the road id of the next point), never inside a
    junction. / TM の set_path 用の点（交差点内は除く）"""
    xs, ys, jn, roads, s = plan['xs'], plan['ys'], plan['junction'], plan['roads'], plan['s']
    zs = plan.get('zs') or [0.0] * len(xs)
    n = len(xs)
    # a point is usable when no junction point is within ``clear`` m of it:
    # at a junction's border the TM's nearest waypoint can be the junction's
    # own lane / 交差点の境界から clear m 以内の点は使わない
    clear = 4.0
    ok = [False] * n
    j = 0
    for i in range(n):
        while j < n and s[j] < s[i] - clear:
            j += 1
        k = j
        bad = False
        while k < n and s[k] <= s[i] + clear:
            if jn[k]:
                bad = True
                break
            k += 1
        ok[i] = not bad
    out, last_s = [], -1e9
    for i in range(n):
        if s[i] < s_from + 5.0 or not ok[i]:
            continue
        run_end = i + 1 >= n or not ok[i + 1] or roads[i + 1] != roads[i]
        run_start = i == 0 or not ok[i - 1] or roads[i - 1] != roads[i]
        if run_start or run_end or s[i] - last_s >= spacing:
            out.append((xs[i], ys[i], zs[i]))
            last_s = s[i]
    return out


def spawn_time(visit, plan, cfg):
    """When the bus must appear: the first stop's time minus the way to it at
    ``plan_kmh`` (回送 in: the departure minus layover too), never before the
    trip's last stop outside the map. / バスを出す時刻"""
    v = cfg['plan_kmh'] / 3.6
    first = plan['points'][0]
    stops = [p for p in plan['points'] if p['kind'] == 'stop']
    if not stops:
        return None
    s1 = stops[0]
    _sid, arr, dep, _flag = visit['stops'][s1['call']]
    arr = dep if arr is None else arr
    dep = arr if dep is None else dep
    if first['kind'] == 'stop':                      # appears at its first stop / 停留所に出現
        return dep - cfg['layover_s']
    t = arr - (s1['s'] - first['s']) / v
    if plan.get('deadhead_in'):
        t = dep - cfg['layover_s'] - (s1['s'] - first['s']) / v
    elif visit.get('prev_s') is not None:
        t = max(t, visit['prev_s'])
    return t


# ---------------------------------------------------------------------------
# runtime / 実行時
# ---------------------------------------------------------------------------
class SpeedArbiter(object):
    """Stands in for the Traffic Manager in traffic_profile.RuleKeeper: for a
    bus, the RuleKeeper's wish (lane speed, 止まれ, walker hold) and this
    module's wish (slow to the stop, dwell) are combined and the stricter is
    sent; other cars pass straight through. A stop is percentage 100 (target
    0), a speed is set_desired_speed (which also ends a stop). The bus never
    changes lanes on its own (it follows its path).
    RuleKeeper とバス制御の速度要求を合成し、厳しい方を TM へ送る。"""

    def __init__(self, tm):
        self.tm = tm
        self.buses = {}          # id -> {'rk': kmh|0|None, 'bus': kmh|0|None, 'sent': ..., 'actor'}

    def __getattr__(self, name):
        return getattr(self.tm, name)

    def add(self, actor):
        self.buses[actor.id] = {'rk': None, 'bus': None, 'sent': None, 'actor': actor}

    def remove(self, vid):
        self.buses.pop(vid, None)

    def _send(self, vid):
        b = self.buses[vid]
        wishes = [w for w in (b['rk'], b['bus']) if w is not None]
        if not wishes:
            return
        eff = min(wishes)
        if eff == b['sent']:
            return
        b['sent'] = eff
        if eff <= 0.0:
            self.tm.vehicle_percentage_speed_difference(b['actor'], 100.0)
        else:
            self.tm.set_desired_speed(b['actor'], float(eff))

    # what RuleKeeper calls / RuleKeeper が呼ぶもの
    def set_desired_speed(self, actor, kmh):
        b = self.buses.get(actor.id)
        if b is None:
            return self.tm.set_desired_speed(actor, kmh)
        b['rk'] = float(kmh)
        self._send(actor.id)

    def vehicle_percentage_speed_difference(self, actor, pct):
        b = self.buses.get(actor.id)
        if b is None:
            return self.tm.vehicle_percentage_speed_difference(actor, pct)
        if pct >= 100.0:
            b['rk'] = 0.0
            self._send(actor.id)
        else:
            self.tm.vehicle_percentage_speed_difference(actor, pct)

    def auto_lane_change(self, actor, allow):
        if actor.id in self.buses:
            return None                      # buses follow their path / 経路どおり
        return self.tm.auto_lane_change(actor, allow)

    # what the bus controller calls / バス制御が呼ぶもの
    def bus_wish(self, vid, kmh):
        b = self.buses.get(vid)
        if b is None:
            return
        b['bus'] = None if kmh is None else max(0.0, float(kmh))
        self._send(vid)

    def held_by_rules(self, vid):
        b = self.buses.get(vid)
        return b is not None and b['rk'] == 0.0


class BusClock(object):
    """The timetable's clock: service date + seconds since its midnight,
    advancing with simulated time (real time; generate_traffic paces the
    world to the wall clock). Anchored at the time of day the sim shows
    (weather_cycle), at --bus-time, or at the wall clock; re-anchored when the
    dashboard sets the sim clock (weather_cycle.json clock_cmd).
    時刻表の時計（シム時間で実時間に進む）。表示時刻に合わせ、ダッシュボード
    で時刻を変えたら合わせ直す。"""

    def __init__(self, fixed_time=None, fixed_day=None, today=None):
        self.fixed_time = fixed_time
        self.fixed_day = fixed_day if fixed_day in DAY_TYPES else None
        self.today = today or datetime.date.today()
        self.date = self.today
        self.base = 0.0
        self.t0 = None
        self.seq = None
        self.source = ''

    def day_type(self):
        return self.fixed_day or day_type(self.date)

    def anchor(self, elapsed, hours=None, source=''):
        """Set the clock to ``hours`` (0-24) at sim time ``elapsed``. Times of
        day before 03:00 belong to the previous service day (GTFS 24:00+).
        3 時前は前日のダイヤ（GTFS の 24 時超え）。"""
        if hours is None:
            n = datetime.datetime.now()
            hours = n.hour + n.minute / 60.0 + n.second / 3600.0
            source = source or 'wall clock'
        self.date = self.today
        sec = hours * 3600.0
        if hours < 3.0:
            self.date = self.today - datetime.timedelta(days=1)
            sec += 86400.0
        self.base = sec
        self.t0 = elapsed
        self.source = source

    def now(self, elapsed):
        return self.base + (elapsed - (self.t0 if self.t0 is not None else elapsed))

    def roll(self, elapsed):
        """Past 27:00 the next service day starts. Returns True when it did.
        27 時を過ぎたら翌日のダイヤへ。"""
        if self.now(elapsed) >= 27 * 3600.0:
            self.base -= 86400.0
            self.date += datetime.timedelta(days=1)
            self.today = self.date
            return True
        return False


def read_sim_clock(max_age_s=120.0):
    """(hours, clock_cmd seq) from weather_cycle's status / settings; hours
    None when the status file is stale or missing. / シムの表示時刻"""
    hours, seq = None, None
    try:
        with open(WEATHER_STATUS) as fh:
            st = json.load(fh)
        if time.time() - float(st.get('t', 0)) <= max_age_s:
            hours = float(st['hours'])
    except (OSError, ValueError, KeyError, TypeError):
        pass
    try:
        with open(WEATHER_SETTINGS) as fh:
            cmd = json.load(fh).get('clock_cmd') or {}
        seq = cmd.get('seq')
    except (OSError, ValueError, AttributeError):
        pass
    return hours, seq


def is_night():
    try:
        with open(WEATHER_STATUS) as fh:
            st = json.load(fh)
        return bool(st.get('night') or st.get('lamps_on'))
    except (OSError, ValueError):
        return False


def project(plan, x, y, lo=0, hi=None):
    """(arc length, distance off, index) of (x, y) on a plan's polyline,
    searching points lo..hi. / 経路の折れ線への射影"""
    xs, ys, s = plan['xs'], plan['ys'], plan['s']
    n = len(xs)
    hi = n - 1 if hi is None else min(n - 1, hi)
    lo = max(0, min(lo, n - 1))
    best = (float('inf'), lo, 0.0)
    for i in range(lo, max(lo + 1, hi)):
        j = min(i + 1, n - 1)
        dx, dy = xs[j] - xs[i], ys[j] - ys[i]
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 < 1e-9 else max(0.0, min(1.0, ((x - xs[i]) * dx + (y - ys[i]) * dy) / L2))
        d = math.hypot(x - xs[i] - t * dx, y - ys[i] - t * dy)
        if d < best[0]:
            best = (d, i, t)
    d, i, t = best
    j = min(i + 1, n - 1)
    return s[i] + t * (s[j] - s[i]), d, i


class BusRun(object):
    """One bus on one visit: drive -> approach -> stopping -> dwell ->
    pullout -> drive ... -> exit (or, with no way out, free -> gone). A pure
    state machine (no CARLA): `step()` gets the bus's position and speed and
    returns what to do. Times come from the visit (one trip), places from the
    plan (shared by every trip of the pass).
    1 台のバスの状態機械（CARLA 不要）。時刻は便から、位置は計画から。"""

    def __init__(self, visit, plan, actor_id, model, cfg, rng, station_ids=()):
        self.visit = visit
        self.plan = plan
        self.id = actor_id
        self.model = model
        self.cfg = cfg
        self.rng = rng
        self.stops = []
        for p in plan['points']:
            if p['kind'] != 'stop':
                continue
            sid, arr, dep, flag = visit['stops'][p['call']]
            self.stops.append({'stop': sid, 's': p['s'], 'x': p['x'], 'y': p['y'],
                               'arr': arr if arr is not None else dep,
                               'dep': dep if dep is not None else arr,
                               'flag': flag or 'both', 'station': sid in station_ids})
        last = plan['points'][-1]
        self.exit = dict(last) if last['kind'] == 'exit' else None
        self.next_stop = 0
        self.phase = 'drive'
        self.s = plan['points'][0]['s']
        self.idx = 0
        self.best_s = self.s
        self.progress_t = None
        self.dwell_until = None
        self.blink_until = None
        self.free_until = None
        self.wait_since = None
        self.reroutes = 0
        self.actor = None
        self.blink = None

    # -- geometry ----------------------------------------------------------
    def locate(self, x, y):
        """Arc length of (x, y) on the path near the last known point, and
        the distance off it. / 経路上の位置と経路からの距離"""
        s, d, i = project(self.plan, x, y, self.idx - 10, self.idx + 80)
        self.idx = i
        return s, d

    def target_s(self, stop):
        """Where the actor origin stops for the door to be at the pole.
        扉がポールに来る車両原点の位置。"""
        return stop['s'] - self.model['door_m']

    def junction_before(self, s_to):
        """(entry s, exit s) of the first junction on the path between the bus
        and s_to, or None. / 次の交差点区間"""
        jn, s = self.plan['junction'], self.plan['s']
        start = None
        i = self.idx
        for i in range(self.idx, len(s)):
            if s[i] > s_to and start is None:
                break
            if jn[i] and start is None and s[i] > self.s:
                start = i
            elif start is not None and not jn[i]:
                return s[start], s[i]
        if start is not None:
            return s[start], s[i]
        return None

    def dwell_time(self, stop, clock_now):
        lo, hi = self.cfg['dwell_s'].get(stop['flag'], self.cfg['dwell_s']['both'])
        t = self.rng.uniform(lo, hi)
        if stop['station'] and stop['dep'] is not None:
            # Oita Station: wait for the departure time / 大分駅は発車時刻まで
            t = max(t, min(stop['dep'] - clock_now, self.cfg['station_hold_max_s']))
        return t

    def approach_kmh(self, d):
        c = self.cfg
        v = math.sqrt((c['creep_kmh'] / 3.6) ** 2 + 2.0 * c['decel'] * max(0.0, d - c['creep_m']))
        return max(c['creep_kmh'], v * 3.6)

    # -- the state machine -------------------------------------------------
    def step(self, now, clock_now, x, y, speed, blocked_ahead=None, queued=False):
        """One control step. ``blocked_ahead``: path position of the rear of a
        bus standing ahead (or None); ``queued``: waiting behind a standing
        bus (no stuck count). Returns actions: ('wish', kmh|0|None),
        ('blink', 'left'|'right'|None), ('event', name, stop id),
        ('remove', reason), ('reroute',). / 1 ステップ分の指示"""
        acts = []
        s, off = self.locate(x, y)
        self.s = s
        if self.progress_t is None:
            self.progress_t = now
        if s > self.best_s + 1.0:
            self.best_s = s
            self.progress_t = now
        if self.phase == 'free':
            if now >= self.free_until or now - self.progress_t > self.cfg['stuck_s']:
                acts.append(('remove', 'done'))
            return acts
        if self.phase == 'dwell' or queued:
            self.progress_t = now
        elif now - self.progress_t > self.cfg['stuck_s']:
            return [('remove', 'stuck')]
        if off > self.cfg['off_route_m'] and self.phase in ('drive', 'approach'):
            if self.reroutes < self.cfg['reroute_max']:
                self.reroutes += 1
                return [('reroute',)]
            return [('remove', 'off route')]
        done = self.next_stop >= len(self.stops)
        if done and self.exit is not None and s >= self.exit['s'] - 3.0:
            return [('remove', 'exit')]
        if self.phase == 'pullout':
            if now < self.blink_until:
                return acts
            acts.append(('blink', None))
            self.phase = 'drive'
        if done:
            if self.exit is None:
                # no way out planned: drive on a little, then go / 出口無し
                self.phase = 'free'
                self.free_until = now + self.cfg['free_drive_s']
                acts.append(('wish', None))
            return acts
        stop = self.stops[self.next_stop]
        if self.phase == 'dwell':
            if now >= self.dwell_until:
                self.next_stop += 1
                self.phase = 'pullout'
                self.blink_until = now + self.cfg['blink_s']
                acts += [('wish', None), ('blink', 'right'), ('event', 'depart', stop['stop'])]
            return acts
        tgt = self.target_s(stop)
        d = tgt - s
        if d < -(self.cfg['stop_tol_m'] + 2.0):
            # passed it without stopping / 停まれずに通過
            self.next_stop += 1
            self.phase = 'drive'
            return acts + [('event', 'missed', stop['stop']), ('wish', None), ('blink', None)]
        if self.phase == 'drive':
            if d > self.cfg['approach_m']:
                return acts
            self.phase = 'approach'
        # approach / stopping: slow down to the stop; with a bus standing
        # ahead, wait before a junction on the way unless there is room
        # behind it / 停留所へ減速。前にバスが停車中なら交差点の手前で待つ
        hold_s = tgt
        if blocked_ahead is not None and blocked_ahead < tgt + self.model['length_m']:
            jb = self.junction_before(blocked_ahead)
            room = blocked_ahead - jb[1] if jb is not None else None
            if room is not None and room < self.model['length_m'] + self.cfg['junction_room_m']:
                if self.wait_since is None:
                    self.wait_since = now
                if now - self.wait_since < self.cfg['junction_wait_max_s']:
                    hold_s = min(hold_s, jb[0] - self.model['length_m'] / 2.0 - 1.0)
            else:
                self.wait_since = None
        else:
            self.wait_since = None
        dh = hold_s - s
        if d < 40.0:
            acts.append(('blink', 'left'))
        margin = speed * speed / (2.0 * self.cfg['hold_decel']) + speed * 0.15 + self.cfg['stop_margin_m']
        if dh <= margin:
            self.phase = 'stopping'
            acts.append(('wish', 0.0))
        else:
            if self.phase == 'stopping' and dh <= margin + 1.0:
                acts.append(('wish', 0.0))           # hysteresis / ヒステリシス
            else:
                self.phase = 'approach'
                acts.append(('wish', min(self.cfg['bus_kmh'], self.approach_kmh(dh))))
        if self.phase == 'stopping' and abs(d) <= self.cfg['stop_tol_m'] \
                and speed < self.cfg['still_mps']:
            self.phase = 'dwell'
            self.dwell_until = now + self.dwell_time(stop, clock_now)
            acts += [('wish', 0.0), ('blink', None), ('event', 'arrive', stop['stop'])]
        return acts

    def rebase(self, plan):
        """A new path (re-route): the stops and exit still ahead get their
        place on it. / 新しい経路上に残りの停留所と出口を置き直す"""
        self.plan = plan
        self.idx = 0
        self.s = self.best_s = 0.0
        for st in self.stops[self.next_stop:]:
            st['s'] = project(plan, st['x'], st['y'])[0]
        if self.exit is not None:
            self.exit['s'] = project(plan, self.exit['x'], self.exit['y'])[0]


class BusSchedule(object):
    """Runs the timetable on the live world. Call `update(snapshot)` after
    every tick (it works every ``period`` s of sim time) and `destroy_all()`
    at exit. / 時刻表を実行する本体。毎ティック update、終了時 destroy_all。"""

    def __init__(self, world, client, tm, carla, timetable, planner, clock, cfg=None,
                 rule_keeper=None, profile=None, log=print, status_path=STATUS_PATH,
                 blueprint=None, role=ROLE):
        self.world = world
        self.role = role
        self.client = client
        self.carla = carla
        self.tt = timetable
        self.planner = planner
        self.clock = clock
        self.cfg = dict(DEFAULTS, **(cfg or {}))
        self.rng = random.Random(self.cfg['seed'])
        self.rk = rule_keeper
        self.profile = profile
        self.log = log
        self.status_path = status_path
        self.arb = SpeedArbiter(tm)
        if rule_keeper is not None:
            rule_keeper.tm = self.arb       # RuleKeeper's wishes pass through the arbiter
        self.tm = tm
        self.bp, self.model = blueprint if blueprint else self._blueprint()
        self.runs = {}
        self.queue = []
        self.pending = {}          # visit index -> first spawn attempt / 生成待ち
        self.qi = 0
        self.next_t = None
        self.next_status = 0.0
        self.next_clock_check = 0.0
        self.counts = {'spawned': 0, 'finished': 0, 'stuck': 0, 'skipped': 0, 'missed': 0,
                       'unplanned': 0, 'lost': 0}
        self.lane_kmh = None
        self.station_ids = {k for k, st in self.tt.stops.items() if st.get('is_station_platform')}
        self.vehicle_ids = set()
        self.next_vehicles = 0.0
        self.night = False
        self.period = 0.2
        self.own_rk = False        # this module ticks the RuleKeeper itself / RuleKeeper を自分で回す

    # -- set-up ------------------------------------------------------------
    def _blueprint(self):
        lib = self.world.get_blueprint_library()
        for m in BUS_MODELS:
            try:
                bp = lib.find(m['id'])
            except (IndexError, RuntimeError):
                bp = None
            if bp is not None:
                if bp.has_attribute('role_name'):
                    bp.set_attribute('role_name', self.role)
                return bp, m
        raise RuntimeError('no bus blueprint (%s) / バスのブループリントが無い'
                           % ', '.join(m['id'] for m in BUS_MODELS))

    def clear_leftovers(self):
        """Buses left by a killed run (our role_name). / 前回の残り"""
        ids = [a.id for a in self.world.get_actors().filter('vehicle.*')
               if a.attributes.get('role_name') == self.role]
        if ids:
            self.client.apply_batch_sync([self.carla.command.DestroyActor(i) for i in ids], False)
            self.log('[bus] removed %d bus(es) left by an earlier run / 前回の残りのバスを削除' % len(ids))

    def build_queue(self, now_s):
        """Visits of the clock's service day (and the previous day's after-
        midnight trips) from the clock's time on. / 当日（と前日の深夜便）"""
        self.queue = []
        today = self.clock.day_type()
        prev = self.clock.fixed_day or day_type(self.clock.date - datetime.timedelta(days=1))
        for v in self.tt.visits(prev):
            if v['stops'][0][1] >= 86400.0:
                v = dict(v, stops=[(a, b - 86400 if b is not None else None,
                                    c - 86400 if c is not None else None, f)
                                   for a, b, c, f in v['stops']],
                         prev_s=None if v['prev_s'] is None else v['prev_s'] - 86400)
                self.queue.append(v)
        self.queue += self.tt.visits(today)
        self.queue.sort(key=lambda v: v['stops'][0][1])
        # skip what is already over: first stop more than 2 min ago
        # 既に終わった便は飛ばす
        self.qi = 0
        while self.qi < len(self.queue) and self.queue[self.qi]['stops'][0][1] < now_s - 120.0:
            self.qi += 1
        self.pending = {}
        self.log('[bus] %s %s (%s): %d visits today, from %s / 本日の便'
                 % (self.clock.date.isoformat(), today, self.clock.source,
                    len(self.queue) - self.qi, hhmm(now_s)))

    # -- the loop ----------------------------------------------------------
    def update(self, snapshot):
        now = snapshot.timestamp.elapsed_seconds
        if self.next_t is not None and now < self.next_t:
            return
        self.next_t = now + self.period
        if self.clock.t0 is None:
            self._anchor(now)
        if now >= self.next_clock_check:
            self.next_clock_check = now + 5.0
            self._check_clock(now)
            self.night = is_night()
        if self.clock.roll(now):
            self.build_queue(self.clock.now(now))
        if self.own_rk and self.rk is not None:
            try:
                self.rk.update(snapshot)
            except Exception as exc:     # the buses go on without it / 補助なしで続行
                self.log('[bus] rule helper off: %s / 補助を停止' % exc)
                try:
                    self.rk.release_all()
                except Exception:
                    pass
                self.rk = None
                self.own_rk = False
        clock_now = self.clock.now(now)
        self._spawn_due(now, clock_now, snapshot)
        self._drive(now, clock_now, snapshot)
        if now >= self.next_status:
            self.next_status = now + self.cfg['status_period_s']
            self.write_status(clock_now, snapshot)

    def _anchor(self, now):
        if self.clock.fixed_time is not None:
            self.clock.anchor(now, self.clock.fixed_time / 3600.0, '--bus-time')
        else:
            hours, seq = read_sim_clock()
            self.clock.seq = seq
            self.clock.anchor(now, hours, 'sim clock' if hours is not None else 'wall clock')
        self.build_queue(self.clock.now(now))

    def _check_clock(self, now):
        if self.clock.fixed_time is not None:
            return
        hours, seq = read_sim_clock()
        if seq is not None and seq != self.clock.seq and hours is not None:
            self.clock.seq = seq
            self.clock.anchor(now, hours, 'sim clock (set on the dashboard)')
            self.log('[bus] clock set to %s / 時刻を合わせ直した' % hhmm(self.clock.now(now)))
            self.build_queue(self.clock.now(now))

    def _spawn_due(self, now, clock_now, snapshot):
        # plan ahead a little each step (a few ms per pass) / 少しずつ事前計画
        budget = 2
        j = self.qi
        while j < len(self.queue) and budget > 0:
            v = self.queue[j]
            if v['stops'][0][1] > clock_now + self.cfg['plan_ahead_s']:
                break
            if v['pass_key'] not in self.planner.plans:
                self.planner.plan(v)
                budget -= 1
            j += 1
        # spawn what is due / 時刻の来た便を出す
        i = self.qi
        while i < len(self.queue):
            v = self.queue[i]
            if v['stops'][0][1] > clock_now + self.cfg['plan_ahead_s']:
                break
            if i in self.pending and self.pending[i] == 'done':
                i += 1
                continue
            plan = self.planner.plans.get(v['pass_key'], 'unplanned')
            if plan == 'unplanned':
                break                                    # planned on a later step / 次回計画
            if plan is None:
                self.counts['unplanned'] += 1
                self.pending[i] = 'done'
                i += 1
                continue
            t = spawn_time(v, plan, self.cfg)
            if t is None or t > clock_now:
                i += 1
                continue
            if clock_now - t > self.cfg['spawn_late_max_s']:
                if i not in self.pending:
                    self.counts['skipped'] += 1
                    self.log('[bus] %s skip %s %s: %.0f s late / 遅れのため見送り'
                             % (hhmm(clock_now), v['name'], v['dest'], clock_now - t))
                self.pending[i] = 'done'
                i += 1
                continue
            if len(self.runs) >= self.cfg['max_buses']:
                break
            if self._spawn(v, plan, now, clock_now, snapshot):
                self.pending[i] = 'done'
            else:
                self.pending.setdefault(i, now)
            i += 1
        while self.qi < len(self.queue) and self.pending.get(self.qi) == 'done':
            self.qi += 1

    def _vehicles(self, now):
        if now >= self.next_vehicles:
            self.next_vehicles = now + 2.0
            try:
                self.vehicle_ids = {a.id for a in self.world.get_actors().filter('vehicle.*')}
            except RuntimeError:
                pass
        return self.vehicle_ids

    def _spawn(self, v, plan, now, clock_now, snapshot):
        c = self.carla
        p0 = plan['points'][0]
        for vid in self._vehicles(now):
            a = snapshot.find(vid)
            if a is None:
                continue
            l = a.get_transform().location
            if (l.x - p0['x']) ** 2 + (l.y - p0['y']) ** 2 < self.cfg['spawn_clear_m'] ** 2:
                return False                                 # occupied: retry / 塞がっている
        start_s = p0['s']
        if p0['kind'] == 'stop':
            start_s = p0['s'] - self.model['door_m']
        tf = c.Transform(c.Location(p0['x'], p0['y'], p0['z'] + 0.6), c.Rotation(yaw=p0['yaw']))
        actor = self.world.try_spawn_actor(self.bp, tf)
        if actor is None:
            return False
        tm = self.tm
        actor.set_autopilot(True, tm.get_port())
        self.arb.add(actor)
        if self.profile:
            import traffic_profile
            traffic_profile.apply_static(tm, [actor], self.profile)
        tm.update_vehicle_lights(actor, True)
        tm.auto_lane_change(actor, False)
        for meth in ('random_left_lanechange_percentage', 'random_right_lanechange_percentage'):
            fn = getattr(tm, meth, None)
            if fn is not None:
                fn(actor, 0.0)
        tm.set_path(actor, [c.Location(x, y, z) for x, y, z in path_locations(plan, start_s)])
        self.arb.bus_wish(actor.id, self._free_kmh(p0['x'], p0['y'], p0['z']))
        if self.rk is not None:
            self.rk.ids.append(actor.id)
        run = BusRun(v, plan, actor.id, self.model, self.cfg, self.rng, self.station_ids)
        run.actor = actor
        self.runs[actor.id] = run
        self.counts['spawned'] += 1
        how = 'at its first stop' if p0['kind'] == 'stop' else (
            'empty from the edge (回送)' if plan.get('deadhead_in') else 'at the edge')
        self.log('[bus] %s spawn %s %s -> %s (%s, %s) id %d / 出発'
                 % (hhmm(clock_now), v['name'], v['dest'], v['stops'][0][0], how,
                    (v['entry'] or {}).get('street') or (v['entry'] or {}).get('kind'), actor.id))
        return True

    def _free_kmh(self, x, y, z):
        """The bus's own speed wish when it is not stopping: the bus limit
        (the RuleKeeper adds the lane limit, 止まれ and walkers); without a
        RuleKeeper, 0.9 x the lane's xodr limit too. / 停車中でない時の上限"""
        if self.rk is not None:
            return self.cfg['bus_kmh']
        if self.lane_kmh is None:
            import traffic_profile
            try:
                _sec, self.lane_kmh = traffic_profile.lane_speeds_from_xodr(
                    self.planner.map.to_opendrive())
            except Exception:
                self.lane_kmh = {}
        kmh = 30.0
        try:
            w = self.planner.map.get_waypoint(self.carla.Location(x, y, z))
            kmh = self.lane_kmh.get((w.road_id, w.lane_id)) or 30.0
        except (RuntimeError, AttributeError):
            pass
        return min(self.cfg['bus_kmh'], 0.9 * kmh)

    def _drive(self, now, clock_now, snapshot):
        pos = {}
        for vid, run in list(self.runs.items()):
            a = snapshot.find(vid)
            if a is None:
                self._finish(run, 'lost', clock_now)
                continue
            l = a.get_transform().location
            vel = a.get_velocity()
            pos[vid] = (l, math.sqrt(vel.x * vel.x + vel.y * vel.y))
        # buses standing (at a stop, or held) / 停車中のバス
        standing = [(r, pos[r.id][0]) for r in self.runs.values()
                    if r.id in pos and r.phase in ('stopping', 'dwell')]
        for vid, run in list(self.runs.items()):
            if vid not in pos:
                continue
            l, speed = pos[vid]
            blocked, queued = None, False
            if run.phase in ('approach', 'stopping', 'drive') and standing:
                half = self.model['length_m'] / 2.0
                for other, ol in standing:
                    if other is run:
                        continue
                    if (ol.x - l.x) ** 2 + (ol.y - l.y) ** 2 > 90.0 ** 2:
                        continue
                    so, off, _i = project(run.plan, ol.x, ol.y, run.idx, run.idx + 60)
                    if off < 2.0 and so > run.s:
                        rear = so - half
                        blocked = rear if blocked is None else min(blocked, rear)
                if blocked is not None and blocked - (run.s + half) < 6.0 and speed < 0.5:
                    queued = True              # right behind a standing bus / 停車中のバスの後ろ
            for act in run.step(now, clock_now, l.x, l.y, speed, blocked, queued):
                self._do(run, act, now, clock_now, l)

    def _do(self, run, act, now, clock_now, loc):
        kind = act[0]
        if kind == 'wish':
            kmh = act[1]
            if kmh is None:
                kmh = self._free_kmh(loc.x, loc.y, loc.z)
            elif kmh > 0.0:
                kmh = min(kmh, self.cfg['bus_kmh'])
            self.arb.bus_wish(run.id, kmh)
        elif kind == 'blink':
            self._blink(run, act[1])
        elif kind == 'event':
            st = self.tt.stops.get(act[2], {})
            if act[1] == 'missed':
                self.counts['missed'] += 1
            self.log('[bus] %s %s %s %s at %s %s / %s'
                     % (hhmm(clock_now), act[1], run.visit['name'], run.visit['dest'],
                        st.get('name', act[2]), ('(%s)' % st['platform']) if st.get('platform') else '',
                        {'arrive': '到着', 'depart': '発車', 'missed': '通過'}.get(act[1], '')))
        elif kind == 'reroute':
            self._reroute(run, loc)
        elif kind == 'remove':
            self._finish(run, act[1], clock_now)

    def _blink(self, run, side):
        if side == run.blink:
            return
        run.blink = side
        vls = self.carla.VehicleLightState
        try:
            if side is None:
                self.tm.update_vehicle_lights(run.actor, True)
                return
            self.tm.update_vehicle_lights(run.actor, False)
            bits = vls.LeftBlinker if side == 'left' else vls.RightBlinker
            if self.night:
                bits = bits | vls.Position | vls.LowBeam
            run.actor.set_light_state(vls(bits))
        except RuntimeError:
            pass

    def _reroute(self, run, loc):
        """Back onto the path: a new way from where the bus is to the next
        point still ahead, then the rest of the old path. / 現在地から次の点
        へ引き直し、残りは元の経路"""
        c = self.carla
        pl = self.planner
        w = pl.map.get_waypoint(loc)
        old = run.plan
        ahead = [st for st in run.stops[run.next_stop:]] + ([run.exit] if run.exit else [])
        if w is None or not ahead:
            return
        nxt = ahead[0]
        target = pl.map.get_waypoint(c.Location(nxt['x'], nxt['y'], loc.z))
        _length, route = pl.trace(w, target)
        if not route:
            self.log('[bus] %s could not re-route / 経路の再計算に失敗' % run.visit['name'])
            return
        k = next((i for i, sv in enumerate(old['s']) if sv >= nxt['s']), len(old['s']) - 1)
        xs = [_xy(rw)[0] for rw, _o in route] + old['xs'][k + 1:]
        ys = [_xy(rw)[1] for rw, _o in route] + old['ys'][k + 1:]
        jn = [rw.is_junction for rw, _o in route] + old['junction'][k + 1:]
        roads = [rw.road_id for rw, _o in route] + old['roads'][k + 1:]
        zs = [rw.transform.location.z for rw, _o in route] + (old.get('zs') or [loc.z] * len(old['xs']))[k + 1:]
        sv = [0.0]
        for i in range(1, len(xs)):
            sv.append(sv[-1] + math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1]))
        run.rebase(dict(old, xs=xs, ys=ys, junction=jn, roads=roads, zs=zs, s=sv, length=sv[-1]))
        self.tm.set_path(run.actor, [c.Location(x, y, z) for x, y, z in path_locations(run.plan, 0.0)])
        self.log('[bus] %s %s re-routed / 経路を再計算' % (run.visit['name'], run.visit['dest']))

    def _finish(self, run, reason, clock_now):
        self.runs.pop(run.id, None)
        self.arb.remove(run.id)
        if reason in ('stuck', 'off route'):
            self.counts['stuck'] += 1
        elif reason == 'lost':
            self.counts['lost'] += 1
        else:
            self.counts['finished'] += 1
        try:
            self.client.apply_batch_sync([self.carla.command.DestroyActor(run.id)], False)
        except RuntimeError:
            pass
        self.log('[bus] %s %s %s %s (%s) / %s'
                 % (hhmm(clock_now), 'exit' if reason in ('exit', 'done') else 'removed',
                    run.visit['name'], run.visit['dest'], reason,
                    '退出' if reason in ('exit', 'done') else '削除'))

    def destroy_all(self):
        """Every bus goes (exit, map switch). / 全バスを削除"""
        ids = list(self.runs)
        self.runs.clear()
        if ids:
            try:
                self.client.apply_batch_sync([self.carla.command.DestroyActor(i) for i in ids], False)
            except RuntimeError:
                pass
            self.log('[bus] destroyed %d bus(es) / バスを削除' % len(ids))
        try:
            os.remove(self.status_path)
        except OSError:
            pass

    # -- status ------------------------------------------------------------
    def write_status(self, clock_now, snapshot=None):
        """data/bus_status.json: clock, buses on the map, next departures per
        Oita Station platform. / ダッシュボード用の状態"""
        buses = []
        for run in self.runs.values():
            nxt = run.stops[run.next_stop]['stop'] if run.next_stop < len(run.stops) else None
            b = {'id': run.id, 'route': run.visit['name'], 'dest': run.visit['dest'],
                 'phase': run.phase, 'next_stop': self.tt.stops.get(nxt, {}).get('name') if nxt else None,
                 'platform': self.tt.stops.get(nxt, {}).get('platform') if nxt else None}
            if snapshot is not None:
                a = snapshot.find(run.id)
                if a is not None:
                    l = a.get_transform().location
                    b.update(x=round(l.x, 1), y=round(l.y, 1))
            buses.append(b)
        plat = {}
        for v in self.queue[self.qi:]:
            if v['stops'][0][1] > clock_now + 3 * 3600:
                break
            for sid, arr, dep, flag in v['stops']:
                st = self.tt.stops.get(sid, {})
                p = st.get('platform')
                if not st.get('is_station_platform') or p is None or dep is None or dep < clock_now:
                    continue
                lst = plat.setdefault(p, [])
                if len(lst) < 3 and flag != 'drop_off_only':
                    lst.append({'time': hhmm(dep)[:5], 'route': v['name'], 'dest': v['dest']})
        st = {'t': time.time(), 'clock': hhmm(clock_now)[:5], 'date': self.clock.date.isoformat(),
              'day_type': self.clock.day_type(), 'clock_source': self.clock.source,
              'buses': buses, 'next_departures': plat, 'counts': dict(self.counts),
              'model': self.model['id'], 'credit': CREDIT}
        try:
            tmp = self.status_path + '.tmp'
            with open(tmp, 'w', encoding='utf-8') as fh:
                json.dump(st, fh, ensure_ascii=False)
            os.replace(tmp, self.status_path)
        except OSError:
            pass


def setup(world, client, tm, carla, mode='auto', rule_keeper=None, profile=None,
          bus_time=None, bus_day='auto', log=print, max_buses=None, traffic_profile_force='auto',
          own_rules=True, role=ROLE, status_path=STATUS_PATH):
    """generate_traffic.py's hook: a BusSchedule on maps with maps.json
    "bus_schedule" (mode auto), or None. Never raises for a missing feature
    (no data, no bus blueprint, the old road logic): it logs and returns None.
    generate_traffic.py のフック。対象外のマップでは None。"""
    if mode == 'none':
        return None
    wmap = world.get_map()
    found = map_config(wmap.name)
    if mode not in ('auto', 'none'):
        found = (mode if os.path.isabs(mode) else os.path.join(HERE, mode), {})
    if found is None:
        return None
    folder, opts = found
    if max_buses is not None:
        opts['max_buses'] = max_buses
    try:
        tt = Timetable(folder)
    except (OSError, ValueError, KeyError) as exc:
        log('[bus] timetable unreadable (%s): no buses / 時刻表を読めない' % exc)
        return None
    xodr = wmap.to_opendrive()
    planner = LanePlanner(wmap, carla, opts, tt, xodr=xodr)
    if not planner.covers(list(tt.stops.values())):
        log('[bus] the loaded road logic of %s does not reach the timetable\'s stops (an older '
            'level?): no buses / 道路ロジックが停留所に届かない（古いレベル？）' % wmap.name)
        return None
    if profile is None:
        import traffic_profile
        _name, profile = traffic_profile.select_profile(wmap.name, force=traffic_profile_force)
    clock = BusClock(fixed_time=gtfs_seconds(bus_time) if bus_time else None,
                     fixed_day=None if bus_day in (None, 'auto') else bus_day)
    own_rk = False
    if rule_keeper is None and profile is not None and own_rules:
        # no traffic cars (-n 0): the buses get a RuleKeeper of their own
        # (止まれ, walkers, lane speeds) / 一般車が無い時はバス専用の RuleKeeper
        import traffic_profile
        rule_keeper = traffic_profile.RuleKeeper(world, wmap, tm, [], profile,
                                                 driving_lane_type=carla.LaneType.Driving, xodr=xodr)
        own_rk = True
    try:
        sched = BusSchedule(world, client, tm, carla, tt, planner, clock, opts,
                            rule_keeper=rule_keeper, profile=profile, log=log,
                            role=role, status_path=status_path)
    except RuntimeError as exc:
        log('[bus] %s' % exc)
        return None
    sched.own_rk = own_rk
    sched.clear_leftovers()
    log('[bus] timetabled buses on %s: %s, up to %d at once / 時刻表バス'
        % (wmap.name.rsplit('/', 1)[-1], sched.model['id'], sched.cfg['max_buses']))
    log('[bus] %s' % CREDIT)
    return sched


# ---------------------------------------------------------------------------
# offline report / オフライン報告
# ---------------------------------------------------------------------------
def report(xodr_path, folder, out=None, day='weekday', limit=None):
    """Map every pass of every route pattern on an xodr and print what does
    not map cleanly. / 全パターンを対応させ、問題を報告する。"""
    import carla
    name = os.path.splitext(os.path.basename(xodr_path))[0]
    with open(xodr_path) as fh:
        xodr = fh.read()
    wmap = carla.Map(name, xodr)
    tt = Timetable(folder)
    pl = LanePlanner(wmap, carla, None, tt, xodr=xodr)
    res = {'xodr': xodr_path, 'stops': {}, 'passes': {}, 'problems': {}}
    print('road logic covers the stops: %s' % pl.covers(list(tt.stops.values())))
    for sid, st in sorted(tt.stops.items()):
        c = pl.stop_candidates(sid)
        res['stops'][sid] = {'name': st['name'], 'platform': st.get('platform'),
                             'x': st['x'], 'y': st['y'],
                             'candidates': [{'road': w.road_id, 'lane': w.lane_id, 'gap_m': round(g, 1),
                                             'shift_m': sh, 'yaw': round(w.transform.rotation.yaw)}
                                            for w, g, sh in c]}
        flag = 'OK' if c and c[0][1] <= pl.cfg['stop_gap_ok_m'] else ('FAR' if c else 'NONE')
        print('%-4s %-6s %-28s (%6.1f, %6.1f): %s %s' % (
            sid, st.get('platform') or '', st['name'], st['x'], st['y'], flag,
            ', '.join('road %d lane %d gap %.1f%s' % (w.road_id, w.lane_id, g,
                                                       (' shift %+.0f' % sh) if sh else '')
                      for w, g, sh in c[:2])))
    seen = {}
    for v in tt.visits(day):
        seen.setdefault(v['pass_key'], v)
    keys = sorted(seen)[:limit] if limit else sorted(seen)
    ok = 0
    for k in keys:
        v = seen[k]
        plan = pl.plan(v)
        why = pl.problems.get(k, [])
        if plan:
            ok += 1
            res['passes'][k] = {'length_m': round(plan['length']), 'points': [
                {kk: p[kk] for kk in ('kind', 's', 'road', 'lane') if kk in p} for p in plan['points']],
                'first_kind': plan['points'][0]['kind'], 'last_kind': plan['points'][-1]['kind'],
                'deadhead_in': plan['deadhead_in'], 'deadhead_out': plan['deadhead_out']}
        if why or not plan:
            res['problems'][k] = why or ['no plan']
    print('%d / %d passes planned (%s); %d with remarks' % (ok, len(keys), day, len(res['problems'])))
    if out:
        with open(out, 'w', encoding='utf-8') as fh:
            json.dump(res, fh, ensure_ascii=False, indent=1)
    return res


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    sub = ap.add_subparsers(dest='cmd')
    r = sub.add_parser('report', help='map stops/routes on an xodr, offline / オフライン対応表')
    r.add_argument('--xodr', default=os.path.join(HERE, 'maps', 'build', 'OitaStation', 'OitaStation.xodr'))
    r.add_argument('--data', default=os.path.join(HERE, 'data', 'oita_bus'))
    r.add_argument('--day', default='weekday', choices=DAY_TYPES)
    r.add_argument('--out', default=None, help='JSON report / JSON の出力先')
    r.add_argument('--limit', type=int, default=None)
    d = sub.add_parser('day', help='day type of a date / 日付のダイヤ種別')
    d.add_argument('date', help='YYYY-MM-DD')
    args = ap.parse_args()
    if args.cmd == 'report':
        report(args.xodr, args.data, args.out, args.day, args.limit)
    elif args.cmd == 'day':
        print(day_type(datetime.date.fromisoformat(args.date)))
    else:
        ap.print_help()


if __name__ == '__main__':
    main()
