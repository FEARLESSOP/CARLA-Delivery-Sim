"""Country driving profiles for the Traffic Manager (traffic_profiles.json).

generate_traffic.py asks `select_profile()` which profile the loaded map
gets. Only maps that maps.json marks as Japanese (``"road_style": "japan"``,
or ``"keep_left"`` in the ``Japan`` package) get the ``japan`` profile; every
other map gets None and generate_traffic.py runs exactly as before (no extra
Traffic Manager call is made). The map is read when generate_traffic starts,
so the F5 map switch (the pipeline restarts it on the new map) re-selects.

The japan profile has two parts:

* `apply_static()` -- per-vehicle Traffic Manager settings, once after the
  spawn: never run lights / signs / walkers / vehicles, a longer following
  gap, no keep-"slow"-lane rule (CARLA counts these netconvert --lefthand
  roads as right-hand, so that rule would push cars into the overtaking
  lane), blinkers and lights driven by the Traffic Manager.
* `RuleKeeper` -- a small helper called from the tick loop (every
  ``period_s`` of simulated time, client-side map queries only, no RPC):
    - desired speed = the posted limit of the lane the car is on (xodr lane
      <speed>) x factor; the Traffic Manager itself only knows the speed of
      the last 274 sign a car passed (30 km/h until then);
    - no overtaking / lane change within ``no_lane_change_before_junction_m``
      of a junction (道路交通法 30 条);
    - stop for a pedestrian on the carriageway ahead, either half of the
      road, until they have left it (38 条); the Traffic Manager alone brakes
      only for a walker inside its own path. A hold is released after
      ``max_hold_s`` so a stuck walker cannot park a car for good (the Traffic
      Manager's own collision check still protects that walker).
      With the map's crossings (hokousha.json) it also stops for a walker
      WAITING at the kerb of an unsignalised crossing (heading into it or
      standing there, up to ``stuck_s``): NPA "Pedestrians First at Road
      Crossings" -- stop when a pedestrian is crossing or about to cross.
    - 止まれ (43 条): at each 206 stop line of the map (`StopLines`, from
      carla.Map landmarks) slow down, stop with the front bumper at the line
      for ``min_stop_s``, then go first come first served once the junction
      is clear. The TM cannot: its all-way stop runs only when the car's
      light state is not Green, and a car under no light reads Green
      (TrafficLightStage.cpp), so it never stops at junctions without lights.

交通マネージャーの国別プロファイル。maps.json で日本のマップとされたもの
（road_style "japan"、または Japan パッケージの keep_left）だけに japan
プロファイルを使い、それ以外のマップでは従来どおり（追加の TM 呼び出し無し）。
"""

import json
import math
import os
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
PROFILES_PATH = os.path.join(HERE, 'traffic_profiles.json')
MAPS_PATH = os.path.join(HERE, 'maps', 'maps.json')


def _load_json(path):
    try:
        with open(path) as fh:
            return json.load(fh)
    except (OSError, ValueError):
        return None


def map_entry(map_name, maps):
    """maps.json entry of a loaded map ('Japan/Maps/OitaKosen/OitaKosen' or
    'OitaKosen'), or None. / ロード中マップの maps.json 項目。"""
    short = (map_name or '').rsplit('/', 1)[-1]
    for entry in (maps or {}).values():
        if isinstance(entry, dict) and entry.get('name') == short:
            return entry
    return None


def select_profile(map_name, maps=None, config=None, force='auto'):
    """Return (profile_name, profile_dict) for this map, or (None, None) for
    the default traffic. `force`: 'auto' (maps.json decides), 'none', or a
    profile name. / このマップのプロファイル。既定交通なら (None, None)。"""
    if config is None:
        config = _load_json(PROFILES_PATH) or {}
    profiles = config.get('profiles') or {}
    if force and force != 'auto':
        if force == 'none' or force not in profiles:
            return None, None
        return force, profiles[force]
    if maps is None:
        maps = _load_json(MAPS_PATH) or {}
    entry = map_entry(map_name, maps)
    if entry is None:
        return None, None
    named = entry.get('traffic_profile')
    if named:
        return (named, profiles[named]) if named in profiles else (None, None)
    for rule in config.get('select') or []:
        when = rule.get('when') or {}
        if when and all(entry.get(k) == v for k, v in when.items()) \
                and rule.get('profile') in profiles:
            return rule['profile'], profiles[rule['profile']]
    return None, None


def _tm_call(tm, names, *args):
    """Call the first Traffic Manager method that exists (0.9.16 renamed
    keep_right_rule_percentage to keep_slow_lane_rule_percentage)."""
    for name in names:
        fn = getattr(tm, name, None)
        if fn is not None:
            fn(*args)
            return True
    return False


def apply_static(tm, vehicles, profile):
    """Per-vehicle Traffic Manager settings of a profile, once after the
    spawn. / 生成後に一度だけ車両毎の TM 設定を適用する。"""
    for v in vehicles:
        for key, meth in (('ignore_lights_percentage', 'ignore_lights_percentage'),
                          ('ignore_signs_percentage', 'ignore_signs_percentage'),
                          ('ignore_walkers_percentage', 'ignore_walkers_percentage'),
                          ('ignore_vehicles_percentage', 'ignore_vehicles_percentage')):
            if key in profile:
                getattr(tm, meth)(v, float(profile[key]))
        if 'distance_to_leading_vehicle_m' in profile:
            tm.distance_to_leading_vehicle(v, float(profile['distance_to_leading_vehicle_m']))
        if 'keep_slow_lane_rule_percentage' in profile:
            _tm_call(tm, ('keep_slow_lane_rule_percentage', 'keep_right_rule_percentage'),
                     v, float(profile['keep_slow_lane_rule_percentage']))
        if profile.get('update_vehicle_lights'):
            tm.update_vehicle_lights(v, True)


def lane_speeds_from_xodr(xodr):
    """{(road, section, lane): km/h} and {(road, lane): km/h} from the lane
    <speed> records of an OpenDRIVE string (unit m/s unless stated).
    / OpenDRIVE の車線 <speed> から車線毎の最高速度 (km/h)。"""
    factor = {'m/s': 3.6, 'km/h': 1.0, 'mph': 1.609344}
    by_section, by_lane = {}, {}
    try:
        root = ET.fromstring(xodr)
    except ET.ParseError:
        return by_section, by_lane
    for road in root.findall('road'):
        try:
            rid = int(road.get('id'))
        except (TypeError, ValueError):
            continue
        lanes = road.find('lanes')
        if lanes is None:
            continue
        for sec_i, section in enumerate(lanes.findall('laneSection')):
            for side in ('left', 'right'):
                group = section.find(side)
                if group is None:
                    continue
                for lane in group.findall('lane'):
                    sp = lane.find('speed')
                    if sp is None or sp.get('max') is None:
                        continue
                    try:
                        kmh = float(sp.get('max')) * factor.get(sp.get('unit', 'm/s'), 3.6)
                        lid = int(lane.get('id'))
                    except (TypeError, ValueError):
                        continue
                    by_section[(rid, sec_i, lid)] = kmh
                    by_lane.setdefault((rid, lid), kmh)
    return by_section, by_lane


def curve_kmh(wp, kmh, cfg):
    """Speed for the bends ahead: the posted limit stays (100 on the ring), but a
    car slows so it takes the tightest bend within its braking distance at
    ``lat_accel`` and brakes at ``decel`` before it (the ring's 15 m corners:
    ~24 km/h). Waypoints only, no RPC. / 前方のカーブに合わせた速度。"""
    a_lat = float(cfg.get('lat_accel', 3.0))
    dec = float(cfg.get('decel', 2.5))
    step = float(cfg.get('step_m', 5.0))
    v = kmh / 3.6
    reach = min(float(cfg.get('lookahead_m', 80.0)), v * v / (2.0 * dec) + 15.0)
    best = kmh
    cur, d = wp, 0.0
    try:
        yaw0 = cur.transform.rotation.yaw
    except AttributeError:                # stand-in waypoints / 代用の waypoint
        return kmh
    while d < reach:
        nxt = cur.next(step)
        if not nxt:
            break
        cur = nxt[0]
        d += step
        dyaw = abs((cur.transform.rotation.yaw - yaw0 + 180.0) % 360.0 - 180.0)
        yaw0 = cur.transform.rotation.yaw
        k = math.radians(dyaw) / step                  # curvature (1/m) / 曲率
        if k < 1e-4:
            continue
        v_bend = math.sqrt(a_lat / k)
        v_now = math.sqrt(v_bend * v_bend + 2.0 * dec * max(0.0, d - step))
        best = min(best, v_now * 3.6)
    return best


def _forward(wp):
    """True if traffic on this lane moves towards growing s. / 車線の進行が s 増加方向か。"""
    try:
        nx = wp.next(0.5)
        if nx and nx[0].road_id == wp.road_id:
            return nx[0].s > wp.s
        pv = wp.previous(0.5)
        if pv and pv[0].road_id == wp.road_id:
            return pv[0].s < wp.s
    except RuntimeError:
        pass
    return wp.lane_id < 0


class StopLines(object):
    """The map's 止まれ stop lines (OpenDRIVE 206 signals, e.g. our
    "Stencil_STOP"), read from carla.Map.get_all_landmarks_of_type('206'),
    so any map with 206 signals works. Each stop applies to the lanes in its
    validity (or, without one, every lane whose travel direction matches its
    orientation) at the landmark's s. For each stop the lanes leading to it
    are sampled back ``approach_m`` (through upstream junctions), so
    `ahead(wp)` answers "which stop line is ahead, how far" with one dict
    lookup. The junction right after the line (bounding box) is the area
    that must be clear before the car goes.
    / 地図の「止まれ」停止線（206 信号）。車線毎に停止線までの距離を引ける。
    """

    def __init__(self, wmap, cfg, lane_type=None):
        self.stops = []
        self.index = {}          # (road, lane) -> [(s, dist, fwd, stop_i)]
        approach = float(cfg.get('approach_m', 40.0))
        get = getattr(wmap, 'get_all_landmarks_of_type', None)
        if get is None:
            return
        for lm in get(str(cfg.get('landmark_type', '206'))):
            ori = str(getattr(lm, 'orientation', 'Positive'))
            lanes = set()
            for a, b in (lm.get_lane_validities() or []):
                lanes.update(i for i in range(min(a, b), max(a, b) + 1) if i != 0)
            if not lanes:
                lanes = set(range(-6, 0)) | set(range(1, 7))
            for lane in sorted(lanes):
                try:
                    wp = wmap.get_waypoint_xodr(lm.road_id, lane, lm.s)
                except RuntimeError:
                    wp = None
                if wp is None or wp.lane_id != lane or wp.road_id != lm.road_id:
                    continue
                if lane_type is not None and wp.lane_type != lane_type:
                    continue
                fwd = _forward(wp)
                if ('Positive' in ori and not fwd) or ('Negative' in ori and fwd):
                    continue
                self._add(len(self.stops), lm, wp, fwd, approach, 2.0)

    def _add(self, i, lm, wp, fwd, approach, step):
        loc = wp.transform.location
        jkey, cx, cy, ex, ey = ('stop', i), loc.x, loc.y, 4.0, 4.0
        w, d = wp, 0.0
        while d < 15.0:                                    # junction after the line / 停止線の先の交差点
            nxt = w.next(1.0)
            if not nxt:
                break
            w, d = nxt[0], d + 1.0
            if w.is_junction:
                bb = w.get_junction().bounding_box
                jkey, cx, cy = w.get_junction().id, bb.location.x, bb.location.y
                ex, ey = bb.extent.x, bb.extent.y          # axis-aligned box / 軸平行の箱
                break
        self.stops.append({'id': getattr(lm, 'id', str(i)), 'road': wp.road_id,
                           'lane': wp.lane_id, 's': wp.s, 'jkey': jkey,
                           'cx': cx, 'cy': cy, 'ex': ex, 'ey': ey, 'r': math.hypot(ex, ey)})
        seen = set()

        def put(w, dist, f):
            key = (w.road_id, w.lane_id, round(w.s, 1))
            if key in seen:
                return False
            seen.add(key)
            self.index.setdefault((w.road_id, w.lane_id), []).append((w.s, dist, f, i))
            return True

        put(wp, 0.0, fwd)
        frontier = [(wp, 0.0)]
        while frontier and len(seen) < 400:
            w, dist = frontier.pop()
            if dist + step > approach + 1e-6:
                continue
            for p in w.previous(step):
                if put(p, dist + step, _forward(p)):
                    frontier.append((p, dist + step))

    def ahead(self, wp):
        """(stop_i, metres from this waypoint to the stop line along the lane;
        < 0 = past it) or None. The nearest stop still ahead wins.
        / 前方の停止線と距離（負 = 通過済み）。"""
        samples = self.index.get((wp.road_id, wp.lane_id))
        if not samples:
            return None
        best = {}
        for s, dist, fwd, i in samples:
            gap = abs(s - wp.s)
            if i not in best or gap < best[i][0]:
                best[i] = (gap, dist + ((s - wp.s) if fwd else (wp.s - s)))
        cand = sorted((d, i) for i, (_g, d) in best.items())
        for d, i in cand:
            if d > -3.0:
                return i, d
        return cand[-1][1], cand[-1][0]


class _Pt(object):
    __slots__ = ('x', 'y')

    def __init__(self, x, y):
        self.x, self.y = x, y


class RuleKeeper(object):
    """Client-side helper for the rules the Traffic Manager cannot follow by
    itself (see the module doc). Call `update(snapshot)` after every tick;
    it does work only every ``period_s`` of simulated time and never raises
    into the tick loop's way (generate_traffic wraps it).
    / TM だけでは守れない規則を補う小さな補助。"""

    def __init__(self, world, wmap, tm, vehicle_ids, profile, driving_lane_type=None,
                 cyclist_ids=(), xodr=None):
        self.world = world
        self.map = wmap
        self.tm = tm
        self.ids = list(vehicle_ids)
        self.cyclists = set(cyclist_ids)
        self.period = float(profile.get('period_s', 0.2))
        self.speed_cfg = profile.get('speed')
        self.lc_dist = float(profile.get('no_lane_change_before_junction_m') or 0.0)
        y = profile.get('yield_to_walkers') or {}
        self.yield_cfg = y if y.get('enabled') else None
        self.lane_type = driving_lane_type
        self.by_section, self.by_lane = ({}, {})
        if self.speed_cfg:
            self.by_section, self.by_lane = lane_speeds_from_xodr(
                xodr if xodr is not None else wmap.to_opendrive())
        self.next_t = None
        self.next_slow_t = None
        self.next_walkers_t = None
        self.walker_ids = []
        self.target = {}         # id -> desired km/h last sent / 送った目標速度
        self.lane_change = {}    # id -> auto lane change on? / 車線変更の可否
        self.held = {}           # id -> [held_since, last_seen] / 停止中
        self.cooldown = {}       # id -> no new hold before t / 再停止の禁止時刻
        self.still = {}          # walker id -> [x, y, still since] / 静止中の歩行者
        self.holds = 0
        # crossings of the map (hokousha.json, maps/osm_city/hokousha_crossings.py):
        # stop also for walkers WAITING at an unsignalised crossing (38 条)
        # 横断歩道データ: 信号の無い横断歩道で待つ歩行者にも止まる（38 条）
        self.crossings = None
        if self.yield_cfg and self.yield_cfg.get('waiting_at_crossings', True):
            import hokousha
            data = hokousha.load(getattr(wmap, 'name', ''))
            if data is not None:
                self.crossings = [c for c in data.get('crossings', []) if not c.get('signal')]
        self.waiting_stops = 0
        # 止まれ: full stop at the stop line, then go when the junction is clear
        # (43 条). The TM's own all-way stop runs only under a non-green light,
        # i.e. never at junctions without lights (TrafficLightStage.cpp).
        # 止まれ: 停止線で完全に止まり、交差点が空いてから進む（43 条）。
        s = profile.get('stop_signs') or {}
        self.stop_cfg = s if s.get('enabled') else None
        self.stop_lines = None
        self.stop_state = {}     # id -> dict(stop, mode, t0, still, arrival, sent)
        self.last_go = {}        # junction key -> sim time of the last release
        self.all_vehicles = []
        self.next_vehicles_t = None
        self.stop_count = {'stops': 0, 'timeouts': 0, 'missed': 0}
        if self.stop_cfg:
            try:
                self.stop_lines = StopLines(wmap, self.stop_cfg, driving_lane_type)
            except Exception as exc:     # never break the rest / 他の補助は続ける
                print('[traffic] 止まれ helper off: %s / 一時停止の補助を停止' % exc)
            if self.stop_lines is not None and not self.stop_lines.stops:
                self.stop_lines = None
            if self.stop_lines is not None:
                print('[traffic] 止まれ: %d stop line(s) from the map\'s 206 signals '
                      '/ 停止線 %d 本' % (len(self.stop_lines.stops), len(self.stop_lines.stops)))

    # -- helpers ----------------------------------------------------------
    def lane_kmh(self, wp):
        key = (wp.road_id, wp.section_id, wp.lane_id)
        kmh = self.by_section.get(key)
        if kmh is None:
            kmh = self.by_lane.get((wp.road_id, wp.lane_id))
        return kmh

    def desired_kmh(self, vid, wp):
        cfg = self.speed_cfg
        kmh = self.lane_kmh(wp) if wp is not None else None
        if kmh is None or kmh <= 0.0:
            kmh = float(cfg.get('fallback_kmh', 30))
        kmh = max(float(cfg.get('min_kmh', 10)), kmh * float(cfg.get('factor', 1.0)))
        if wp is not None and cfg.get('curve'):
            kmh = min(kmh, max(float(cfg.get('min_kmh', 10)), curve_kmh(wp, kmh, cfg['curve'])))
        if vid in self.cyclists:
            kmh = min(kmh, float(cfg.get('cyclist_max_kmh', 18)))
        return kmh

    def junction_ahead(self, wp):
        if wp.is_junction:
            return True
        d = 5.0
        while d <= self.lc_dist + 1e-6:
            for nxt in wp.next(d):
                if nxt.is_junction:
                    return True
            d += 5.0
        return False

    def _get_wp(self, loc, on_road_only=False):
        kw = {}
        if self.lane_type is not None:
            kw['lane_type'] = self.lane_type
        if on_road_only:
            kw['project_to_road'] = False
        return self.map.get_waypoint(loc, **kw)

    # -- main -------------------------------------------------------------
    def update(self, snapshot):
        now = snapshot.timestamp.elapsed_seconds
        if self.next_t is not None and now < self.next_t:
            return
        self.next_t = now + self.period
        slow = self.next_slow_t is None or now >= self.next_slow_t
        if slow:
            self.next_slow_t = now + max(self.period, 0.5)

        cars = []
        for vid in self.ids:
            act = snapshot.find(vid)
            if act is None:      # destroyed (TM idle clean-up) / 削除済み
                continue
            cars.append((vid, act.get_transform(), act.get_velocity()))
        self.ids = [c[0] for c in cars]

        if slow:
            for vid, tf, _vel in cars:
                wp = self._get_wp(tf.location)
                if wp is None:
                    continue
                if self.speed_cfg:
                    kmh = self.desired_kmh(vid, wp)
                    old = self.target.get(vid)
                    self.target[vid] = kmh
                    if vid not in self.held and not self._stop_owns(vid) \
                            and (old is None or abs(old - kmh) > 0.5):
                        self.tm.set_desired_speed(self._actor(vid), kmh)
                if self.lc_dist > 0.0:
                    allow = not self.junction_ahead(wp)
                    if self.lane_change.get(vid) != allow:
                        self.lane_change[vid] = allow
                        self.tm.auto_lane_change(self._actor(vid), allow)

        if self.stop_lines is not None:
            try:
                self._stop_signs(now, snapshot, cars)
            except Exception as exc:     # keep the walker / speed rules / 他の規則は続ける
                print('[traffic] 止まれ helper off: %s / 一時停止の補助を停止' % exc)
                self.stop_lines = None
                for vid in [v for v in self.stop_state if self._stop_owns(v)]:
                    del self.stop_state[vid]
                    if vid not in self.held:
                        self.tm.set_desired_speed(self._actor(vid), self._lane_target(vid))
                self.stop_state.clear()
        if self.yield_cfg:
            self._yield(now, snapshot, cars)

    def _actor(self, vid):
        # TM setters take an Actor; the world's cache answers without RPC
        # once the actor is known. / TM の設定は Actor を受け取る。
        cache = getattr(self, '_actors', None)
        if cache is None:
            cache = self._actors = {}
        act = cache.get(vid)
        if act is None:
            act = cache[vid] = self.world.get_actor(vid)
        return act

    def _walkers_on_road(self, now, snapshot):
        if self.next_walkers_t is None or now >= self.next_walkers_t:
            self.next_walkers_t = now + 2.0
            self.walker_ids = [a.id for a in self.world.get_actors().filter('walker.pedestrian.*')]
        # A walker that has not moved for stuck_s is a navmesh agent stuck on
        # the road (seen live on OitaKosen: one stood in a junction for
        # minutes), not someone crossing: holding every car behind it for
        # max_hold_s would jam the junction. / stuck_s 動かない歩行者は
        # ナビメッシュで詰まったもの（渡っていない）として無視する。
        stuck_s = float(self.yield_cfg.get('stuck_s', 8.0))
        out, kerb = [], []
        for wid in self.walker_ids:
            act = snapshot.find(wid)
            if act is None:
                continue
            loc = act.get_transform().location
            st = self.still.get(wid)
            if st is None or abs(loc.x - st[0]) + abs(loc.y - st[1]) > 0.2:
                self.still[wid] = [loc.x, loc.y, now]
            elif now - st[2] > stuck_s:
                continue
            if self._get_wp(loc, on_road_only=True) is not None:
                out.append(loc)
            elif self.crossings:
                vel = act.get_velocity()
                kerb.append((loc.x, loc.y, vel.x, vel.y))
        if kerb:
            # a walker at the kerb of an unsignalised crossing, heading into it or
            # standing there: the crossing's centre line counts as occupied
            # 信号の無い横断歩道の縁で渡ろうとする歩行者: 横断歩道上を占有とみなす
            import hokousha
            for rec in hokousha.waiting_crossings({'crossings': self.crossings}, kerb):
                self.waiting_stops += 1
                out += [_Pt(x, y) for x, y in hokousha.crossing_points(rec)]
        return out

    def _yield(self, now, snapshot, cars):
        cfg = self.yield_cfg
        walkers = self._walkers_on_road(now, snapshot)
        lateral = float(cfg.get('lateral_m', 4.5))
        for vid, tf, vel in cars:
            speed = math.hypot(vel.x, vel.y)
            look = max(float(cfg.get('min_lookahead_m', 12)),
                       speed * float(cfg.get('lookahead_s', 2.5)) + 4.0)
            yaw = math.radians(tf.rotation.yaw)
            fx, fy = math.cos(yaw), math.sin(yaw)
            seen = False
            for w in walkers:
                dx, dy = w.x - tf.location.x, w.y - tf.location.y
                ahead = dx * fx + dy * fy
                side = -dx * fy + dy * fx
                if 0.5 < ahead < look and abs(side) < lateral:
                    seen = True
                    break
            h = self.held.get(vid)
            if seen and h is None and now >= self.cooldown.get(vid, 0.0):
                self.held[vid] = [now, now]
                self.holds += 1
                # 100 % under the limit = target 0; undone by set_desired_speed
                # 制限速度の 100 % 減 = 目標 0（set_desired_speed で解除）
                self.tm.vehicle_percentage_speed_difference(self._actor(vid), 100.0)
            elif h is not None:
                if seen:
                    h[1] = now
                timed_out = now - h[0] > float(cfg.get('max_hold_s', 20))
                if timed_out or now - h[1] > float(cfg.get('release_s', 1.0)):
                    del self.held[vid]
                    if timed_out:
                        self.cooldown[vid] = now + 5.0
                    self._release(vid)

    def _stop_owns(self, vid):
        st = self.stop_state.get(vid)
        return st is not None and st['mode'] != 'done'

    def _lane_target(self, vid):
        kmh = self.target.get(vid)
        if kmh is None:
            kmh = float((self.speed_cfg or {}).get('fallback_kmh', 30))
        return kmh

    def _release(self, vid):
        # end of a walker hold; a 止まれ hold / approach owns the speed then
        # 歩行者の停止を解除（止まれの停止・減速中はそちらが速度を持つ）
        st = self.stop_state.get(vid)
        if self._stop_owns(vid):
            if st['mode'] in ('hold', 'wait'):
                self.tm.vehicle_percentage_speed_difference(self._actor(vid), 100.0)
            else:
                st['sent'] = None        # approach speed resent next period / 次回再送
            return
        self.tm.set_desired_speed(self._actor(vid), self._lane_target(vid))

    def release_all(self):
        """Undo the walker and 止まれ holds: every held / slowed car gets its
        lane target back (also used when the helper is switched off after an
        error, so no car stays at target 0). Not needed at exit: the cars go
        too. / 歩行者・止まれの停止と減速を全て解除し車線の目標速度に戻す。"""
        owned = [vid for vid in self.stop_state if self._stop_owns(vid)]
        self.stop_state.clear()
        for vid in list(self.held):
            del self.held[vid]
            self._release(vid)                 # stop_state empty: lane target / 車線の目標速度
        for vid in owned:
            if vid not in self.held:
                try:
                    self.tm.set_desired_speed(self._actor(vid), self._lane_target(vid))
                except Exception:              # destroyed meanwhile / 削除済み
                    pass

    # -- 止まれ (stop sign) ----------------------------------------------
    def _stop_signs(self, now, snapshot, cars):
        """Slow down towards the stop line, stop with the front bumper at it
        (speed < still_mps for min_stop_s), then go first come first served
        once the junction is clear. Lever: percentage_speed_difference 100
        (target 0), undone by set_desired_speed. / 停止線へ減速し、前端を
        停止線に合わせて完全停止、先着順で交差点が空けば発進。"""
        cfg = self.stop_cfg
        lines = self.stop_lines
        a_comf = float(cfg.get('comfort_decel_mps2', 1.5))
        a_hold = float(cfg.get('hold_decel_mps2', 4.0))
        creep = float(cfg.get('creep_kmh', 7.0)) / 3.6
        margin = float(cfg.get('line_margin_m', 0.3))
        approach_m = float(cfg.get('approach_m', 40.0))
        max_hold = float(cfg.get('max_hold_s', 12.0))
        alive = set(c[0] for c in cars)
        for vid in [v for v in self.stop_state if v not in alive]:
            del self.stop_state[vid]               # destroyed: never blocks a queue / 削除済み
        for vid, tf, vel in cars:
            speed = math.hypot(vel.x, vel.y)
            st = self.stop_state.get(vid)
            wp = self._get_wp(tf.location)
            info = lines.ahead(wp) if wp is not None else None
            d = None
            if info is not None:
                d = info[1] - self._half_length(vid) - margin   # front bumper -> line / 前端から停止線
            if st is not None and st['mode'] == 'done':
                if info is None or info[0] != st['stop'] or d < -6.0:
                    del self.stop_state[vid]       # through the junction / 通過
                continue
            if st is None:
                if info is None or d > approach_m:
                    continue
                if d < -1.0:                       # appeared past the line / 停止線の先に出現
                    self.stop_state[vid] = {'stop': info[0], 'mode': 'done'}
                    continue
                st = self.stop_state[vid] = {'stop': info[0], 'mode': 'approach', 't0': now,
                                             'still': None, 'arrival': None, 'sent': None}
            if st['mode'] == 'approach':
                if info is None or info[0] != st['stop'] or d > approach_m + 5.0:
                    del self.stop_state[vid]       # turned off before the line / 手前で離脱
                    if vid not in self.held:
                        self.tm.set_desired_speed(self._actor(vid), self._lane_target(vid))
                    continue
                if d < -1.0:                       # could not stop in time / 止まれず通過
                    self.stop_count['missed'] += 1
                    self._stop_done(vid, st, now)
                    continue
                # brake now if the TM (target 0 at a_hold) would only just stop
                # at the line, counting half a period of reaction (the mean
                # delay until the next check) / 今止めれば停止線で止まる距離
                # （半周期の遅れ込み）なら停止
                if d <= 0.5 * speed * self.period + speed * speed / (2.0 * a_hold) + 0.1:
                    st['mode'], st['t0'] = 'hold', now
                    self.tm.vehicle_percentage_speed_difference(self._actor(vid), 100.0)
                    continue
                # comfortable braking curve that reaches creep speed creep_m
                # before the line / creep_m 手前で徐行速度になる減速曲線
                v_ref = math.sqrt(creep * creep + 2.0 * a_comf *
                                  max(0.0, d - float(cfg.get('creep_m', 3.0))))
                kmh = min(self._lane_target(vid), max(creep, v_ref) * 3.6)
                if vid not in self.held and (st['sent'] is None or abs(st['sent'] - kmh) > 2.0):
                    st['sent'] = kmh
                    self.tm.set_desired_speed(self._actor(vid), kmh)
                continue
            # hold / wait
            if now - st['t0'] > max_hold:          # never stuck for good / 永久停止しない
                self.stop_count['timeouts'] += 1
                self._stop_done(vid, st, now)
                continue
            if st['mode'] == 'hold':
                if speed < float(cfg.get('still_mps', 0.1)):
                    if st['still'] is None:
                        st['still'] = now
                    if now - st['still'] >= float(cfg.get('min_stop_s', 2.0)):
                        st['mode'], st['arrival'] = 'wait', st['still']
                        self.stop_count['stops'] += 1
                else:
                    st['still'] = None
            if st['mode'] == 'wait' and self._may_go(vid, st, now, snapshot):
                self._stop_done(vid, st, now)

    def _stop_done(self, vid, st, now):
        st['mode'] = 'done'
        self.last_go[self.stop_lines.stops[st['stop']]['jkey']] = now
        if vid not in self.held:
            self.tm.set_desired_speed(self._actor(vid), self._lane_target(vid))

    def _half_length(self, vid):
        try:
            return float(self._actor(vid).bounding_box.extent.x)
        except AttributeError:
            return 2.3

    def _may_go(self, vid, st, now, snapshot):
        """First come first served among the cars waiting at this junction,
        a gap after the last one released, and no vehicle in the junction or
        coming towards it within clear_m (the crossing road has priority).
        / 先着順・直前の発進から間隔・交差点内と優先道路の接近車なし。"""
        cfg = self.stop_cfg
        stop = self.stop_lines.stops[st['stop']]
        jkey = stop['jkey']
        first = min((o['arrival'], ovid) for ovid, o in self.stop_state.items()
                    if o['mode'] == 'wait' and self.stop_lines.stops[o['stop']]['jkey'] == jkey)
        if first[1] != vid:
            return False
        if now - self.last_go.get(jkey, -1e9) < float(cfg.get('follow_gap_s', 2.5)):
            return False
        if self.next_vehicles_t is None or now >= self.next_vehicles_t:
            self.next_vehicles_t = now + 2.0
            self.all_vehicles = [a.id for a in self.world.get_actors().filter('vehicle.*')]
        clear_m = float(cfg.get('clear_m', 15.0))
        cx, cy, r = stop['cx'], stop['cy'], stop['r']
        for oid in self.all_vehicles:
            if oid == vid:
                continue
            o = self.stop_state.get(oid)
            if o is not None and o['mode'] != 'done' and \
                    self.stop_lines.stops[o['stop']]['jkey'] == jkey:
                continue                           # also stopping here / ここで止まる車
            act = snapshot.find(oid)
            if act is None:
                continue
            loc = act.get_transform().location
            dx, dy = cx - loc.x, cy - loc.y
            dist = math.hypot(dx, dy)
            if dist > r + clear_m:
                continue
            # Behind (or standing at) a stop line of this junction: it has to
            # stop there itself, moving or not. Checked before the box: the
            # +2.5 m box reaches over some stop lines (7 of 74 on OitaKosen),
            # and a car held there by ANOTHER process (the delivery vans'
            # helper vs generate_traffic's) would otherwise block us until
            # max_hold_s. / この交差点の停止線の手前（停止中も）の車は除外。
            # 箱判定より先に行う（別プロセスが止めている車で詰まらないため）。
            owp = self._get_wp(loc)
            oi = self.stop_lines.ahead(owp) if owp is not None else None
            if oi is not None and self.stop_lines.stops[oi[0]]['jkey'] == jkey \
                    and oi[1] >= float(cfg.get('at_line_m', -0.5)):
                continue                           # at / behind a stop line here / 停止線の手前
            if abs(dx) < stop['ex'] + 2.5 and abs(dy) < stop['ey'] + 2.5:
                return False                       # (partly) in the junction / 交差点内
            v = act.get_velocity()
            if math.hypot(v.x, v.y) < 1.0 or v.x * dx + v.y * dy <= 0.0:
                continue                           # parked / moving away / 停止中・離れる
            return False                           # coming with priority / 優先側から接近
        return True


def describe(name, profile):
    sp = profile.get('speed') or {}
    return ('[traffic] %s traffic profile: lights/signs/walkers ignored 0 %%, gap %.1f m, '
            'lane speed x%.2f, no lane change %.0f m before junctions, yield to walkers %s, '
            '止まれ full stop %s / %s 交通プロファイルを適用'
            % (name, float(profile.get('distance_to_leading_vehicle_m', 0) or 0),
               float(sp.get('factor', 1.0)) if sp else 0.0,
               float(profile.get('no_lane_change_before_junction_m') or 0),
               'on' if (profile.get('yield_to_walkers') or {}).get('enabled') else 'off',
               'on' if (profile.get('stop_signs') or {}).get('enabled') else 'off',
               name))
