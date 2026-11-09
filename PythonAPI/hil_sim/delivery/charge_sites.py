"""LastMile charging stations standing in the world (the web viewer's charge
dock: LastMileHub_ChargeDock.glb -> static.prop.lmhchargestation).

A station is 14 m x 9.8 m. Its frame (CARLA axes, origin = where the hub
parks in bay 04, on the ground): the vehicles drive in nose first towards +x,
so the -x edge is the open side and faces a kerb-side driving lane. Bay 03 is
the LMP-4 pod's (with its plug unit), bay 04 the LastMile Hub's (same plug
unit design); the Mini Pod pads and the drone deck stand behind them.

Placement: on free ground beside a kerb lane, the station's open edge
STATION_GAP m behind the kerb, the bays square to the lane. Free ground =
plain lawn: never over a building, a road, a pavement or footpath, markings,
a court, a car park or a house lot. On our generated maps that comes from
the map build's surface grid (maps/build/<Map>/surface_grid.npz, made by
osm_city/surface_grid.py, because road and pavement raycast alike there);
on other maps from the raycast labels. A spot picked on
the dashboard map (delivery_zones.json "charge") is moved onto the nearest
kerb lane on its side and checked: no building or other object over the
footprint (raycast), not on a road, the ground flat enough, the lane straight
there and away from junctions. "random" places N stations itself the same
way. Spots that fail are reported with the reason.

Driving in (like the off-road yard in yard.py): the vehicle drives on the road
to the bay's entry point in the lane, then follows a scripted path -- turn on
the spot to face the bay, drive straight in -- and later reverses straight out
and turns back onto the lane. One vehicle moves at a station at a time. While
it charges the bay's plug unit pushes its head 0.375 m to the vehicle's front
inlet (charge.js) and pulls it back at the end.

The pure parts (layout, bay frames, paths) have no CARLA dependency
(tests/test_charge_sites.py). / 世界に置く LastMile 充電ステーション。
"""

import json
import math
import os
import random

HERE = os.path.dirname(os.path.abspath(__file__))
PARTS_PATH = os.path.join(HERE, "assets", "lmh_parts.json")
BUILD_DIR = os.path.join(os.path.dirname(HERE), "maps", "build")

STATION_BP = "static.prop.lmhchargestation"
PLUG_CHAIN = ("ZCarriage", "YCarriage", "BoomStage", "Head")
STATION_GAP = 0.5          # open edge this far behind the kerb's top edge / 縁石から開口部まで
MAX_SETBACK = 4.0          # open edge at most this far beyond the lane edge (close to the road) / 道路の近く
APRON_BP = "static.prop.lmhapron"
FLOOR_LIFT = 0.06           # station floor over the median lawn height / 床の持ち上げ
QUEUE_L, QUEUE_D = 6.0, 2.8  # waiting bay beside the station: along the road, depth / 待機ベイ
QUEUE_GAP = 1.0             # between the station and its waiting bay / 待機ベイとの間隔
RING_OFF, RING_W = 1.3, 1.8  # footpath round the station: offset from it, width (kerbs incl.) / 周回歩道
WALK_SEG = 2.0              # one path segment (= the paving texture's repeat) / 歩道 1 枚の長さ
WALK_BP, WALK_CORNER_BP = "static.prop.lmhwalk", "static.prop.lmhwalkcorner"
MIN_SPACING = 60.0         # between stations (random) / ステーション間隔
FLAT_TOL = 0.9             # ground height range allowed under the footprint (hilly maps) / 地面の高低差
CLEAR_H = 6.2              # nothing above the ground up to this height (station 5.9 m) / 上空の空き
SNAP_R = 30.0              # a picked spot looks this far for a kerb lane / 道路を探す範囲
PLUG_REACH = 0.375         # boom + head travel to the inlet / ヘッドの移動量
PLUG_S = 3.0               # plug-in / unplug time / 接続・切断の時間
SPEEDS = {"drive": 1.5, "pivot": 0.8, "reverse": 1.0}

# the station as measured from the model (lmh_parts.json "station" replaces
# these once the asset build has run) / モデルの寸法（資産ビルド後は json から）
LAYOUT = {
    "bbox_min": [-4.02, -7.21, 0.0], "bbox_max": [9.98, 2.6, 5.93], "entrance_x": -4.02,
    "bays": {"lmp": [-0.52, -4.4, 0.0], "lmh": [0.0, 0.0, 0.0]},
    "plug_units": {
        "lmp": {"ZCarriage": [2.83, -5.16, 0.08], "YCarriage": [2.83, -5.16, 1.06],
                "BoomStage": [2.12, -5.16, 1.06], "Head": [2.12, -5.16, 1.06]},
        "lmh": {"ZCarriage": [2.83, -0.46, 0.08], "YCarriage": [2.83, -0.46, 1.06],
                "BoomStage": [2.12, -0.46, 1.06], "Head": [2.12, -0.46, 1.06]}},
    "plug_props": {"ZCarriage": "static.prop.lmhplugz", "YCarriage": "static.prop.lmhplugy",
                   "BoomStage": "static.prop.lmhplugboom", "Head": "static.prop.lmhplughead"},
}


def layout():
    """LAYOUT updated from lmh_parts.json "station" when it is there. / 寸法。"""
    out = json.loads(json.dumps(LAYOUT))
    try:
        with open(PARTS_PATH) as f:
            st = json.load(f).get("station") or {}
    except (OSError, ValueError):
        st = {}
    for k in ("bbox_min", "bbox_max", "entrance_x", "plug_props", "roof_clearance", "pads", "pad_pins",
              "pad_plugs", "pod_poses", "pad_lane", "deck_pads", "deck_plugs", "deck_drone_yaw",
              "pad_pins_prop", "pad_plug_prop", "deck_plug_prop", "pad_pins_motion", "pad_plug_motion",
              "deck_plug_motion", "walk"):
        if st.get(k) is not None:
            out[k] = st[k]
    for k in ("bays", "plug_units"):
        if isinstance(st.get(k), dict):
            out[k].update({kk: v for kk, v in st[k].items() if kk in ("lmp", "lmh")})
    return out


def _wrap(a):
    return (a + 180.0) % 360.0 - 180.0


class Frame(object):
    """A station's pose: origin (x, y, z) and yaw (deg, local +x = into the
    bays, away from the road). / ステーションの姿勢。"""

    def __init__(self, x, y, z, yaw):
        self.x, self.y, self.z, self.yaw = x, y, z, yaw
        r = math.radians(yaw)
        self.c, self.s = math.cos(r), math.sin(r)

    def world(self, lx, ly):
        """Local (x, y) -> world (x, y). CARLA: +y local = right of +x."""
        return (self.x + lx * self.c - ly * self.s, self.y + lx * self.s + ly * self.c)

    def corners(self, lay):
        (x0, y0, _), (x1, y1, _) = lay["bbox_min"], lay["bbox_max"]
        return [self.world(x, y) for x, y in ((x0, y0), (x1, y0), (x1, y1), (x0, y1))]

    def samples(self, lay, step=2.0):
        """Ground sample points over the footprint (every ``step`` m, edges included)."""
        (x0, y0, _), (x1, y1, _) = lay["bbox_min"], lay["bbox_max"]
        nx, ny = max(1, int(math.ceil((x1 - x0) / step))), max(1, int(math.ceil((y1 - y0) / step)))
        return [self.world(x0 + (x1 - x0) * i / nx, y0 + (y1 - y0) * j / ny)
                for i in range(nx + 1) for j in range(ny + 1)]


def place(lane_x, lane_y, lane_yaw, kerb_off, side, lay, along=0.0):
    """The station frame beside a lane point: ``side`` +1 = kerb on the
    lane's left (keep-left), -1 = on its right; ``kerb_off`` = lane centre to
    the kerb's top edge (m); ``along`` slides it along the lane. The open edge
    stands STATION_GAP behind the kerb, the bays point away from the lane.
    車線脇のステーション姿勢（開口部は縁石の STATION_GAP 奥、ベイは車線と直角）。"""
    r = math.radians(lane_yaw)
    fx, fy = math.cos(r), math.sin(r)
    nx, ny = (fy, -fx) if side > 0 else (-fy, fx)          # lane -> kerb / 車線から縁石へ
    d = kerb_off + STATION_GAP - lay["entrance_x"]
    # centre the bays (their mean local y) on the lane point / ベイの中心を車線点へ
    bys = [b[1] for b in lay["bays"].values()]
    yaw = math.degrees(math.atan2(ny, nx))
    f = Frame(0.0, 0.0, 0.0, yaw)
    oy = sum(bys) / len(bys)
    wx, wy = f.world(0.0, oy)
    return Frame(lane_x + nx * d + fx * along - wx, lane_y + ny * d + fy * along - wy, 0.0, yaw)


def bay_frame(frame, lay, kind, lane_yaw, kerb_off):
    """Bay ``kind``'s pose and its entry point in the lane:
    {"bay": (x, y), "bay_yaw", "entry": (x, y), "entry_yaw"}. / ベイと入口。"""
    bx, by, _ = lay["bays"][kind]
    d = kerb_off + STATION_GAP - lay["entrance_x"]          # origin -> lane centre along -x
    return {"bay": frame.world(bx, by), "bay_yaw": frame.yaw,
            "entry": frame.world(-d, by), "entry_yaw": lane_yaw}


def path_in(bay, start):
    """From the vehicle at ``start`` (x, y, yaw) near the entry into the bay:
    to the entry, turn on the spot, straight in. [(x, y, yaw, speed)] every
    10 cm, like yard.path_in. / 入口からベイへ。"""
    from delivery.yard import _line, _pivot
    sp = SPEEDS
    ex, ey = bay["entry"]
    pts = _line(start[:2], (ex, ey), sp["drive"], yaw=start[2]) if math.hypot(start[0] - ex, start[1] - ey) > 0.05 else []
    y0 = start[2]
    y1 = y0 + _wrap(bay["bay_yaw"] - y0)
    pts += _pivot(ex, ey, y0, y1, sp["pivot"])
    pts += _line((ex, ey), bay["bay"], sp["drive"], yaw=y1)
    return pts or [(ex, ey, y1, sp["drive"])]


def path_out(bay):
    """Reverse straight out to the entry, turn on the spot onto the lane."""
    from delivery.yard import _line, _pivot
    sp = SPEEDS
    ex, ey = bay["entry"]
    y0 = bay["bay_yaw"]
    pts = _line(bay["bay"], (ex, ey), sp["reverse"], yaw=y0)
    pts += _pivot(ex, ey, y0, y0 + _wrap(bay["entry_yaw"] - y0), sp["pivot"])
    return pts


def queue_path_in(q, start):
    """Into the waiting bay: along the lane to the point beside it, then
    sideways (the heading stays the lane's) onto it. / 待機ベイへ横移動。"""
    from delivery.yard import _line
    sp = SPEEDS
    ex, ey = q["entry"]
    pts = _line(start[:2], (ex, ey), sp["drive"], yaw=q["bay_yaw"]) if math.hypot(start[0] - ex, start[1] - ey) > 0.05 else []
    pts += _line((ex, ey), q["bay"], 0.8, yaw=q["bay_yaw"])
    return pts or [(ex, ey, q["bay_yaw"], 0.8)]


def queue_path_out(q):
    from delivery.yard import _line
    return _line(q["bay"], q["entry"], 0.8, yaw=q["bay_yaw"])


def ring_local(lay, q_y):
    """The footpath round a station (its waiting bay included), local
    polyline from the pavement at one front corner, round the back, to the
    other: [(x, y)]. / ステーション周回歩道（局所座標の折れ線）。"""
    x0, x1 = lay["entrance_x"], lay["bbox_max"][0]
    y0, y1 = lay["bbox_min"][1], lay["bbox_max"][1]
    if q_y is not None:
        y0, y1 = min(y0, q_y[0]), max(y1, q_y[1])
    g = RING_OFF + RING_W / 2.0
    half, seg = RING_W / 2.0, WALK_SEG
    xa = x0 - 0.5
    # every straight run (between the corner pieces) a whole number of path
    # segments: the path sits up to a segment further out, never overlapping
    # 直線部を歩道 1 枚の整数倍に（重ならないよう少し外側へ）
    side = (x1 + g) - xa - half
    xb = xa + math.ceil(side / seg - 1e-9) * seg + half
    back = (y1 + g) - (y0 - g) - 2 * half
    extra = math.ceil(back / seg - 1e-9) * seg - back
    ya, yb = y0 - g - extra / 2.0, y1 + g + extra / 2.0
    return [(xa, ya), (xb, ya), (xb, yb), (xa, yb)]


def plug_offsets(u):
    """Local x offset of BoomStage and Head for plug-in progress u (0..1):
    the boom does the first part fast, the last 50 mm slowly (charge.js).
    プラグの伸び（最後の 50 mm はゆっくり）。"""
    u = max(0.0, min(1.0, u))
    fast = PLUG_REACH - 0.05
    x = fast * min(u / 0.7, 1.0) + 0.05 * max(0.0, (u - 0.7) / 0.3)
    return -x


# ---------------------------------------------------------------- CARLA side
def _ground_hits(world, carla, x, y, z):
    return world.cast_ray(carla.Location(x, y, z + CLEAR_H + 1.0), carla.Location(x, y, z - 3.0))


GROUND = ("Roads", "Sidewalks", "Ground", "Terrain", "RoadLines", "NONE", "None", "Other")
TAKEN_WHY = {"lot": "on a house lot or car park", "court": "on a court or pitch", "site": "on a fenced site",
             "walk": "on a pavement or footpath", "kerb": "on a kerb", "road": "on a road",
             "marking": "on road markings", "street": "street furniture in the way", "tree": "a tree in the way",
             "building": "a building stands there", "none": "outside the built map"}
LABEL_WHY = {"Roads": "on a road", "RoadLines": "on road markings", "Sidewalks": "on a pavement or footpath",
             "Buildings": "a building stands there", "Water": "on water", "Railtrack": "on a railway"}


class SurfaceGrid(object):
    """The map build's surface grid (osm_city/surface_grid.py): what covers
    each 0.5 m cell; ``taken(x, y)`` (CARLA point) -> None if plain lawn,
    else the KINDS name. / 地表グリッド（芝の空き地か）。"""

    def __init__(self, path):
        import numpy as np
        d = np.load(path)
        self.free, self.kind = d["free"], d["kind"]
        self.x0, self.y0, self.res = float(d["x0"]), float(d["y0"]), float(d["res"])
        self.kinds = [str(k) for k in d["kinds"]]

    @classmethod
    def for_map(cls, map_name):
        path = os.path.join(BUILD_DIR, str(map_name).replace("\\", "/").split("/")[-1], "surface_grid.npz")
        try:
            return cls(path) if os.path.exists(path) else None
        except (OSError, ValueError, KeyError):
            return None

    def taken(self, x, y):
        c = int((x - self.x0) / self.res)
        r = int((-y - self.y0) / self.res)              # CARLA y south -> map y north / 南北反転
        if not (0 <= r < self.free.shape[0] and 0 <= c < self.free.shape[1]):
            return "none"
        if self.free[r, c]:
            return None
        k = self.kinds[int(self.kind[r, c])]
        return "lawn edge" if k == "lawn" else k


class Finder(object):
    """Finds and checks station spots. ``buildings`` = delivery_sim's
    building objects (bounding boxes), ``reachable(loc)`` the road-graph
    test, ``keep_left`` the map's driving side. / 設置場所の探索と検査。"""

    def __init__(self, world, wmap, buildings=(), reachable=None, keep_left=True, log=print, vehicles=()):
        import carla
        self.carla, self.world, self.map = carla, world, wmap
        self.lay = layout()
        self.side = 1 if keep_left else -1
        self.reach = reachable
        self.log = log
        self.grid = SurfaceGrid.for_map(wmap.name)
        self.posts = self._posts()
        # the LastMile vehicles that must be able to charge here: [{"kind":
        # "lmp"|"lmh", "half_w", "half_l", "height"}] (from the spawned cars)
        # ここで充電できなければならない車両の寸法
        self.vehicles = list(vehicles)
        self.half_w = max([1.15] + [v["half_w"] + 0.2 for v in self.vehicles])
        self.turn_r = max([2.6] + [math.hypot(v["half_w"], v["half_l"]) + 0.3 for v in self.vehicles])
        self.veh_top = max([4.3] + [v["height"] + 0.3 for v in self.vehicles])     # clear height for driving / 走行に必要な高さ
        self.unfit = None
        roof = self.lay.get("roof_clearance") or {}
        for v in self.vehicles:
            r = roof.get(v["kind"])
            if r is not None and v["height"] + 0.1 > r:
                self.unfit = "the %s (%.2f m tall) does not fit under bay %s's canopy (%.2f m)" % (
                    "LMP-4" if v["kind"] == "lmp" else "hub", v["height"], "03" if v["kind"] == "lmp" else "04", r)
        self.boxes = []
        for o in buildings or ():
            bb = o.bounding_box
            self.boxes.append((bb.location.x - bb.extent.x, bb.location.y - bb.extent.y,
                               bb.location.x + bb.extent.x, bb.location.y + bb.extent.y))

    def _posts(self):
        """Pole bases the rays and the grid miss: street lamps, signals,
        speed posts, pedestrian signals spawned from blueprints (BP_JpGaito
        ...), CARLA's own poles. [(x, y)] / 光線とグリッドで拾えない柱。"""
        import re
        C = self.carla
        pat = re.compile(r"Gaito|Shingo|Sokudo|Hokousha|Pole|Lamp|Light|Sign|Signal|Hydrant|Post", re.I)
        out = []
        try:
            objs = self.world.get_environment_objects(C.CityObjectLabel.Any)
        except RuntimeError:
            return out
        for o in objs:
            if str(o.type) in ("Poles", "TrafficLight", "TrafficSigns") or pat.search(o.name or ""):
                L = o.transform.location
                out.append((L.x, L.y))
        return out

    def _post_in(self, frame, x0, x1, y0, y1, grow):
        """A pole base inside the local box [x0, x1] x [y0, y1] grown by ``grow``."""
        for px, py in self.posts:
            dx, dy = px - frame.x, py - frame.y
            if abs(dx) > 40.0 or abs(dy) > 40.0:
                continue
            lx = dx * frame.c + dy * frame.s
            ly = -dx * frame.s + dy * frame.c
            if x0 - grow <= lx <= x1 + grow and y0 - grow <= ly <= y1 + grow:
                return True
        return False

    # -- the lane and its kerb / 車線と縁石
    def _kerb_lane(self, wp):
        C = self.carla
        if wp is None or wp.lane_type != C.LaneType.Driving or wp.is_junction:
            return False
        n = wp.get_left_lane() if self.side > 0 else wp.get_right_lane()
        return not (n is not None and n.lane_type == C.LaneType.Driving and n.lane_id * wp.lane_id > 0)

    def _kerb_off(self, wp):
        """Lane centre -> kerb top edge, measured on the model by stepping out
        until the ground steps up (boxes._kerb), else half the lane + 0.3."""
        from delivery import boxes
        t = wp.transform
        f = t.get_forward_vector()
        nx, ny = (f.y, -f.x) if self.side > 0 else (-f.y, f.x)
        k = boxes._kerb(self.world, t.location, nx, ny, wp.lane_width)
        return (k[0], k[1]) if k else (wp.lane_width / 2.0 + 0.3, None)

    def _edge(self, wp, along=0.0, reach=None):
        """Lane centre -> the station's open edge: with a surface grid, where
        free lawn starts behind the kerb, pavement and verge (vehicles cross
        those like a driveway; the station never stands on them), else the
        measured kerb + STATION_GAP. None: no free ground within ``reach``.
        開口部の位置（グリッドがあれば歩道・路肩の奥の芝の始まり）。"""
        if self.grid is None:
            return self._kerb_off(wp)[0] + STATION_GAP
        lay, t = self.lay, wp.transform
        y0, y1 = lay["bbox_min"][1], lay["bbox_max"][1]
        ex = lay["entrance_x"]
        e = wp.lane_width / 2.0
        reach = reach or wp.lane_width / 2.0 + MAX_SETBACK
        while e <= reach:
            fr = place(t.location.x, t.location.y, t.rotation.yaw, e - STATION_GAP, self.side, lay, along)
            if all(self.grid.taken(*fr.world(ex + dx, y0 + (y1 - y0) * k / 20.0)) is None
                   for dx in (0.0, 0.5, 1.0) for k in range(21)):
                return e
            e += 0.25
        return None

    def _straight(self, wp, reach=10.0):
        y0 = wp.transform.rotation.yaw
        for w in wp.next(reach) + wp.previous(reach):
            if w.is_junction or abs(_wrap(w.transform.rotation.yaw - y0)) > 15.0:
                return False
        return True

    def _raised(self, x, y, z, top=None, ref=None):
        """(ground z, label of what stands between 0.35 m and ``top`` m over
        it, or None); higher things (tree crowns over a footpath or a lane)
        do not count. / 地面から 0.35 m〜top m にある物。"""
        hits = [h for h in _ground_hits(self.world, self.carla, x, y, z)
                if str(h.label) not in ("Pedestrians", "Car", "Truck", "Bus", "Bicycle", "Motorcycle",
                                        "Rider", "Vehicles", "Dynamic")]
        if not hits:
            return None, None
        if ref is not None:
            # the surface at the expected height: a road deck can lie over the
            # terrain mesh (both hit): measure from the one at/below ref + 0.5
            # 想定高さ付近の面を基準に（道路は地形の上に載ることがある）
            below = [h.location.z for h in hits if h.location.z <= ref + 0.5]
            g = max(below) if below else min(h.location.z for h in hits)
        else:
            g = min(h.location.z for h in hits)
        lim = g + (top if top is not None else CLEAR_H)
        inside = [h for h in hits if g + 0.35 < h.location.z <= lim]
        return g, (str(max(inside, key=lambda h: h.location.z).label) if inside else None)

    def _corridor(self, frame, wp, kerb_off, half_w=None, step=0.25):
        """Each bay's drive-in strip (lane centre -> bay, vehicle width, a ray
        every ``step`` m both ways): None if clear, else what stands in it
        (poles, guardrails, walls...) or "markings" when the way in crosses
        a painted area (a bus stop, a zebra crossing).
        ベイへの進入路（車幅）に障害物や停留所・横断歩道の標示が無いか。"""
        lay = self.lay
        half_w = half_w or self.half_w
        m = int(round(2 * half_w / step))
        for kind in lay["bays"]:
            b = bay_frame(frame, lay, kind, wp.transform.rotation.yaw, kerb_off)
            (ex, ey), (bx, by) = b["entry"], b["bay"]
            L = math.hypot(bx - ex, by - ey)
            ux, uy = (bx - ex) / L, (by - ey) / L
            painted = total = 0
            if self.grid is not None:                  # from the lane centre / 車線中心から
                for i in range(int(L / step) + 1):
                    for j in range(m + 1):
                        o = -half_w + j * step
                        k = self.grid.taken(ex + ux * i * step - uy * o, ey + uy * i * step + ux * o)
                        total += 1
                        painted += k == "marking"
                if painted > 0.15 * total:
                    return "markings"
            start = wp.lane_width / 2.0 - 0.2          # rays from the lane edge (the lane is road) / 車線端から
            for i in range(int((L - start) / step) + 1):
                d = start + i * step
                for j in range(m + 1):
                    o = -half_w + j * step
                    zr = wp.transform.location.z + (frame.z - wp.transform.location.z) * min(1.0, d / max(L, 0.1))
                    _, lab = self._raised(ex + ux * d - uy * o, ey + uy * d + ux * o, wp.transform.location.z,
                                          self.veh_top, ref=max(zr, wp.transform.location.z))
                    if lab is not None:
                        return lab
        return None

    def _turn_room(self, frame, wp, kerb_off):
        """Room for each vehicle to turn on the spot at its bay's entry in the
        lane (radius = the longest vehicle's half diagonal): None, or what is
        in the way. その場旋回の余地（最長車両の半対角）。"""
        lay, z = self.lay, wp.transform.location.z
        for kind in lay["bays"]:
            ex, ey = bay_frame(frame, lay, kind, wp.transform.rotation.yaw, kerb_off)["entry"]
            for px, py in self.posts:
                if math.hypot(px - ex, py - ey) < self.turn_r:
                    return "Poles"
            for rr in (self.turn_r, self.turn_r * 0.6):
                for k in range(24):
                    a = 2 * math.pi * k / 24
                    _, lab = self._raised(ex + rr * math.cos(a), ey + rr * math.sin(a), z, self.veh_top,
                                          ref=max(z, frame.z) if rr > wp.lane_width / 2.0 else z)
                    if lab is not None:
                        return lab
        return None

    # -- the check / 検査
    def check(self, frame, wp, kerb_off, taken=()):
        """None if the station fits at ``frame`` beside lane ``wp`` and every
        LastMile vehicle can drive in and out, else why."""
        C, lay = self.carla, self.lay
        if self.unfit:
            return self.unfit
        cx, cy = frame.world((lay["bbox_min"][0] + lay["bbox_max"][0]) / 2,
                             (lay["bbox_min"][1] + lay["bbox_max"][1]) / 2)
        for t in taken:
            if math.hypot(cx - t[0], cy - t[1]) < 18.0:
                return "overlaps another station"
        if not self._straight(wp):
            return "the road bends or a junction is too close"
        (bx0, by0, _), (bx1, by1, _) = lay["bbox_min"], lay["bbox_max"]
        if self._post_in(frame, bx0, bx1, by0, by1, 0.5):
            return "a lamp post or signal pole stands there"
        d = kerb_off + STATION_GAP - lay["entrance_x"]
        for kind, (bx, by, _) in lay["bays"].items():
            if self._post_in(frame, -d + wp.lane_width / 2.0, bx, by - 1.15, by + 1.15, 0.3):
                return "a lamp post or signal pole blocks the way in"
        if self.grid is not None:
            for x, y in frame.samples(lay, step=self.grid.res):     # every cell / 全セル
                k = self.grid.taken(x, y)
                if k is not None:
                    return TAKEN_WHY.get(k, "too close to a road, pavement or plot" if k == "lawn edge" else k)
        r = self._extras_grid(frame, wp)
        if r is not None:
            return r
        zs, pts = [], frame.samples(lay, step=1.0)
        for x, y in pts:
            if any(b[0] <= x <= b[2] and b[1] <= y <= b[3] for b in self.boxes):
                return "a building stands there"
            w = self.map.get_waypoint(C.Location(x, y, wp.transform.location.z), project_to_road=True,
                                      lane_type=C.LaneType.Driving)
            if w is not None and math.hypot(w.transform.location.x - x, w.transform.location.y - y) \
                    < w.lane_width / 2.0 + 0.4:
                return "on a road"
            hits = [h for h in _ground_hits(self.world, C, x, y, wp.transform.location.z)
                    if str(h.label) not in ("Pedestrians", "Car", "Truck", "Bus", "Bicycle", "Motorcycle",
                                            "Rider", "Vehicles", "Dynamic")]
            if not hits:
                return "no ground there"
            if self.grid is None:
                low = min(hits, key=lambda h: h.location.z)
                if str(low.label) in LABEL_WHY:
                    return LABEL_WHY[str(low.label)]
            g = min(h.location.z for h in hits)
            if max(h.location.z for h in hits) > g + 0.35:
                lab = str(max(hits, key=lambda h: h.location.z).label)
                return {"Vegetation": "trees in the way", "Buildings": "a building stands there",
                        "Poles": "a pole in the way", "Fences": "a fence in the way",
                        "Walls": "a wall in the way"}.get(lab, "something stands there (%s)" % lab)
            zs.append(g)
        zs.sort()
        if zs[-1] - zs[0] > FLAT_TOL:
            return "the ground is not flat (%.1f m)" % (zs[-1] - zs[0])
        frame.z = zs[len(zs) // 2]
        lab = self._corridor(frame, wp, kerb_off)
        if lab is not None:
            return {"markings": "the way in crosses a bus stop or crossing", "Poles": "a pole blocks the way in", "GuardRail": "a guardrail blocks the way in",
                    "Vegetation": "trees block the way in", "Fences": "a fence blocks the way in",
                    "Walls": "a wall blocks the way in"}.get(lab, "something blocks the way in (%s)" % lab)
        lab = self._turn_room(frame, wp, kerb_off)
        if lab is not None:
            return {"Poles": "a pole leaves no room to turn in the lane", "GuardRail": "a guardrail leaves no room to turn",
                    "Walls": "a wall leaves no room to turn"}.get(lab, "no room to turn in the lane (%s)" % lab)
        if abs(frame.z - wp.transform.location.z) > 0.6:
            return "the ground is %.1f m %s the road" % (abs(frame.z - wp.transform.location.z),
                                                        "above" if frame.z > wp.transform.location.z else "below")
        if self.reach is not None:
            for kind in lay["bays"]:
                e = bay_frame(frame, lay, kind, wp.transform.rotation.yaw, kerb_off)["entry"]
                if not self.reach(C.Location(e[0], e[1], wp.transform.location.z)):
                    return "a vehicle could not drive there and away again"
        # a waiting bay off the lane beside it, and a footpath round both
        # 車線外の待機ベイと、周りの歩道
        q = frame.extra.get("queue_y")
        qx = frame.extra.get("queue_x", 0.0)
        if self._area(frame, wp, lay["entrance_x"] + qx, lay["entrance_x"] + qx + QUEUE_D, q[0], q[1]) is not None:
            return "no room for a waiting bay beside it"
        r = self._ring(frame, wp, ring_local(lay, q))
        if r is not None:
            return "no room for a footpath round it (%s)" % r
        return None

    def _extras_grid(self, frame, wp):
        """Grid-only pass for the waiting bay and the footpath (cheap, before
        any raycast): sets frame.extra, or returns why. / グリッドだけの事前検査。"""
        lay = self.lay
        frame.extra = {}
        for dx, q_y in self._queue_sides(lay, wp, frame):
            if self._area(frame, wp, lay["entrance_x"] + dx, lay["entrance_x"] + dx + QUEUE_D, q_y[0], q_y[1],
                          rays=False) is None and \
                    self._ring(frame, wp, ring_local(lay, q_y), rays=False) is None:
                frame.extra = {"queue_y": q_y, "queue_x": dx}
                return None
        return "no room for a waiting bay and a footpath beside it"

    def _queue_sides(self, lay, wp, frame):
        """Waiting-bay candidates (x offset behind the open edge, local y
        range), upstream of the station first (vehicles wait before it),
        nearest first. / 待機ベイの候補（上流側・近い順）。"""
        f = wp.transform.get_forward_vector()
        ly = (-frame.s * f.x + frame.c * f.y)            # lane heading along local y / 車線方向の局所 y 成分
        out = []
        for gap in (QUEUE_GAP, 2.5, 4.0):
            up = (lay["bbox_min"][1] - gap - QUEUE_L, lay["bbox_min"][1] - gap)
            down = (lay["bbox_max"][1] + gap, lay["bbox_max"][1] + gap + QUEUE_L)
            for dx in (0.0, 1.0, 2.0):
                out += [(dx, down), (dx, up)] if ly < 0 else [(dx, up), (dx, down)]
        return out

    def _area(self, frame, wp, x0, x1, y0, y1, allow=(), rays=True):
        """None if the local box is free (grid: plain lawn or ``allow`` kinds;
        no pole; nothing raised), else what is there. / 区域が空いているか。"""
        if self._post_in(frame, x0, x1, y0, y1, 0.3):
            return "a pole"
        if self.grid is not None:
            nx, ny = max(1, int((x1 - x0) / 0.5)), max(1, int((y1 - y0) / 0.5))
            for i in range(nx + 1):
                for j in range(ny + 1):
                    k = self.grid.taken(*frame.world(x0 + (x1 - x0) * i / nx, y0 + (y1 - y0) * j / ny))
                    if k is not None and k not in allow:
                        return TAKEN_WHY.get(k, k)
        if not rays:
            return None
        z = wp.transform.location.z
        for i in range(int(x1 - x0) + 1):
            for j in range(int(y1 - y0) + 1):
                g, lab = self._raised(*frame.world(min(x0 + i, x1), min(y0 + j, y1)), z, self.veh_top)
                if lab is not None:
                    return lab
                if g is not None and abs(g - frame.z) > FLAT_TOL:
                    return "uneven ground"
        return None

    def _ring(self, frame, wp, pts, rays=True):
        """None if the footpath band along ``pts`` is free (lawn, or pavement
        where it meets the pavement), else what is in the way."""
        h = RING_W / 2.0
        for (ax, ay), (bx, by) in zip(pts, pts[1:]):
            L = math.hypot(bx - ax, by - ay)
            ux, uy = (bx - ax) / L, (by - ay) / L
            r = self._area_line(frame, wp, (ax, ay), (ux, uy), L, h, rays)
            if r is not None:
                return r
        return None

    def _area_line(self, frame, wp, a, u, L, h, rays=True):
        allow = ("walk", "lawn edge")
        z = wp.transform.location.z
        for i in range(int(L / 0.5) + 1):
            for o in (-h, 0.0, h):
                lx, ly = a[0] + u[0] * i * 0.5 - u[1] * o, a[1] + u[1] * i * 0.5 + u[0] * o
                x, y = frame.world(lx, ly)
                if self.grid is not None:
                    k = self.grid.taken(x, y)
                    if k is not None and k not in allow:
                        return TAKEN_WHY.get(k, k)
                if rays and i % 2 == 0:
                    _, lab = self._raised(x, y, z, 2.2)
                    if lab is not None:
                        return lab
        for px, py in self.posts:
            dx, dy = px - frame.x, py - frame.y
            lx, ly = dx * frame.c + dy * frame.s, -dx * frame.s + dy * frame.c
            t = (lx - a[0]) * u[0] + (ly - a[1]) * u[1]
            if -0.3 <= t <= L + 0.3 and abs(-(lx - a[0]) * u[1] + (ly - a[1]) * u[0]) < h + 0.3:
                return "a pole"
        return None

    def at(self, x, y, yaw=None, taken=()):
        """Fit a station to a spot picked on the map: (site, None) or (None, why)."""
        C, lay = self.carla, self.lay
        best, why = None, "no kerb-side lane within %.0f m" % SNAP_R
        cands = []
        for ring in (0.0, 6.0, 12.0, 20.0, SNAP_R):
            for k in range(1 if ring == 0 else 12):
                a = 2 * math.pi * k / 12
                w = self.map.get_waypoint(C.Location(x + ring * math.cos(a), y + ring * math.sin(a), 0.0),
                                          project_to_road=True, lane_type=C.LaneType.Driving)
                if w is not None and self._kerb_lane(w):
                    cands.append(w)
        seen = set()
        scored = []
        for w in cands:
            key = (w.road_id, w.lane_id, round(w.s / 4.0))
            if key in seen:
                continue
            seen.add(key)
            t = w.transform
            f = t.get_forward_vector()
            nx, ny = (f.y, -f.x) if self.side > 0 else (-f.y, f.x)
            lat = (x - t.location.x) * nx + (y - t.location.y) * ny
            if lat < 2.0:
                continue                        # the spot is not on this lane's kerb side / 縁石側でない
            if yaw is not None and abs(_wrap(math.degrees(math.atan2(ny, nx)) - yaw)) > 45.0:
                continue
            scored.append((abs(lat - 8.0) + 0.0, w))
        scored.sort(key=lambda s: s[0])
        for _, w in scored[:6]:
            t = w.transform
            along = (x - t.location.x) * math.cos(math.radians(t.rotation.yaw)) + \
                    (y - t.location.y) * math.sin(math.radians(t.rotation.yaw))
            e = self._edge(w, along)
            if e is None:
                why = "no free ground (plain lawn) within %.0f m of the road here" % MAX_SETBACK
                best = best or why
                continue
            koff = e - STATION_GAP
            fr = place(t.location.x, t.location.y, t.rotation.yaw, koff, self.side, lay, along)
            r = self.check(fr, w, koff, taken)
            if r is None:
                return self._site(fr, w, koff), None
            why = r
            best = best or r
        return None, (best or why)

    def random(self, n, rng=None, taken=()):
        """Up to ``n`` stations placed by the sim, spread out. / ランダム配置。"""
        C = self.carla
        rng = rng or random.Random()
        wps = [w for w in self.map.generate_waypoints(10.0) if self._kerb_lane(w)]
        rng.shuffle(wps)
        out, centres = [], list(taken)
        for w in wps:
            if len(out) >= n:
                break
            t = w.transform
            if any(math.hypot(t.location.x - c[0], t.location.y - c[1]) < MIN_SPACING for c in centres):
                continue
            for along in (0.0, 3.0, -3.0, 6.0, -6.0):        # slide to dodge a lamp post / 柱を避けてずらす
                e = self._edge(w, along)
                if e is None:
                    continue
                koff = e - STATION_GAP
                fr = place(t.location.x, t.location.y, t.rotation.yaw, koff, self.side, self.lay, along)
                r = self.check(fr, w, koff, centres)
                if r is None:
                    s = self._site(fr, w, koff)
                    out.append(s)
                    centres.append(s["centre"])
                    break
                if not r.startswith(("a lamp post", "a pole")):
                    break                                # sliding will not help / ずらしても無駄
        return out

    def _site(self, fr, wp, koff):
        lay = self.lay
        cx, cy = fr.world((lay["bbox_min"][0] + lay["bbox_max"][0]) / 2,
                          (lay["bbox_min"][1] + lay["bbox_max"][1]) / 2)
        bays = {k: bay_frame(fr, lay, k, wp.transform.rotation.yaw, koff) for k in lay["bays"]}
        d = koff + STATION_GAP - lay["entrance_x"]                # origin -> lane centre / 原点から車線中心
        # where the access road starts: the kerb's top, read on each bay line
        # (surface grid) / 進入路の始まり（縁石の上）
        start = wp.lane_width / 2.0 + 0.3
        if self.grid is not None:
            starts = []
            for kind, (bx, by, _) in lay["bays"].items():
                k = wp.lane_width / 2.0 - 0.5
                while k < d - 0.5:
                    if self.grid.taken(*fr.world(-d + k, by)) not in ("road", "marking"):
                        break
                    k += 0.25
                starts.append(k + 0.25)
            start = max(starts)
        extra = getattr(fr, "extra", {}) or {}
        queue = None
        if extra.get("queue_y"):
            qy = extra["queue_y"]
            yc = (qy[0] + qy[1]) / 2.0
            qx = extra.get("queue_x", 0.0)
            queue = {"bay": fr.world(lay["entrance_x"] + qx + QUEUE_D / 2.0, yc), "bay_yaw": wp.transform.rotation.yaw, "x": qx,
                     "entry": fr.world(-d, yc), "entry_yaw": wp.transform.rotation.yaw, "y": qy}
        return {"frame": fr, "centre": (cx, cy), "corners": fr.corners(lay), "bays": bays, "queue": queue,
                "ring": [fr.world(x, y) for x, y in ring_local(lay, extra.get("queue_y"))],
                "ring_local": ring_local(lay, extra.get("queue_y")),
                "lane_z": wp.transform.location.z, "apron_x": (-d + start, lay["entrance_x"]),
                "apron_y": (min(b[1] for b in lay["bays"].values()) - self.half_w,
                            max(b[1] for b in lay["bays"].values()) + self.half_w)}


class Stations(object):
    """The stations spawned in the world: the static dock plus each bay's
    plug unit (moving parts attached to it). / 世界のステーションと可動部。"""

    def __init__(self, world, client, sites, sources=None):
        import carla
        self.carla, self.world, self.client = carla, world, client
        self.lay = layout()
        self.sites, self.actors, self.plugs = [], [], []
        lib = world.get_blueprint_library()
        if not lib.filter(STATION_BP):
            return
        for i, s in enumerate(sites):
            fr = s["frame"]
            # the floors are zero-thickness planes: lift them clear of the lawn / 床は厚み 0：芝より上へ
            st = world.try_spawn_actor(lib.find(STATION_BP),
                                       carla.Transform(carla.Location(fr.x, fr.y, fr.z + FLOOR_LIFT),
                                                       carla.Rotation(yaw=fr.yaw)))
            if st is None:
                continue
            s = dict(s, id=len(self.sites) + 1, source=(sources[i] if sources else "random"), actor=st)
            units = {}
            for kind, piv in self.lay["plug_units"].items():
                parts = {}
                for name in PLUG_CHAIN:
                    bp = self.lay["plug_props"].get(name)
                    if not bp or not lib.filter(bp) or name not in piv:
                        continue
                    x, y, z = piv[name]
                    parts[name] = world.spawn_actor(lib.find(bp), carla.Transform(carla.Location(x, y, z)),
                                                    attach_to=st)
                units[kind] = parts
            s["plugs"], s["plug_u"] = units, {k: 0.0 for k in units}
            # Mini Pod pads (lock pins + plug) and drone-deck plugs, at rest
            # ミニポッドのパッド（ピン・プラグ）とドローンデッキのプラグ
            s["pad_parts"], s["deck_parts"], s["pad_u"], s["deck_u"] = [], [], {}, {}
            for i, (pins, plug) in enumerate(zip(self.lay.get("pad_pins", []), self.lay.get("pad_plugs", []))):
                parts = {}
                for name, bp, piv in (("pins", self.lay.get("pad_pins_prop"), pins),
                                      ("plug", self.lay.get("pad_plug_prop"), plug)):
                    if bp and lib.filter(bp):
                        parts[name] = world.spawn_actor(lib.find(bp), carla.Transform(carla.Location(*piv)),
                                                        attach_to=st)
                s["pad_parts"].append(parts)
            for piv in self.lay.get("deck_plugs", []):
                bp = self.lay.get("deck_plug_prop")
                s["deck_parts"].append(world.spawn_actor(lib.find(bp), carla.Transform(carla.Location(*piv)),
                                                         attach_to=st) if bp and lib.filter(bp) else None)
            s["apron"] = self._apron(lib, s)
            self.sites.append(s)

    def _apron(self, lib, s):
        """The access road: driveway slabs from the kerb to the open edge over
        both bays' drive-in strips, each on the ground under it (a few mm
        apart in height where they overlap). / 縁石から開口部までの進入路。"""
        C = self.carla
        if not lib.filter(APRON_BP) or "apron_x" not in s:
            return []
        fr = s["frame"]
        out = []
        q = s.get("queue")
        if q:                                   # the waiting bay and its crossing / 待機ベイと横断部
            out += self._slabs(lib, fr, s["apron_x"][0], self.lay["entrance_x"] + q.get("x", 0.0) + QUEUE_D,
                               q["y"][0], q["y"][1], 0.001)
        out += self._walk(lib, fr, s.get("ring_local", []))      # the footpath round it / 周回歩道
        (x0, x1), (y0, y1) = s["apron_x"], s["apron_y"]
        if x1 - x0 < 0.2:
            return out
        return out + self._slabs(lib, fr, x0, x1, y0, y1, 0.0)

    def _ground_at(self, wx, wy, fr):
        C = self.carla
        for h in self.world.cast_ray(C.Location(wx, wy, fr.z + 2.0), C.Location(wx, wy, fr.z - 3.0)):
            if str(h.label) in GROUND:
                return h.location.z
        return fr.z

    def _lay(self, lib, bp, fr, lx, ly, ang, length, width, dz=0.0):
        """Spawn a flat piece (path segment, corner, slab) so it follows the
        ground: heights read at its front, back, left and right give its
        pitch and roll; it sits on that plane, lifted where the lawn would
        poke through (one height per piece left steps and grass showing on
        uneven lawn). / 地面に沿って傾けて置く（段差・芝の突き抜け防止）。"""
        C = self.carla
        a = math.radians(fr.yaw + ang)
        fx, fy, rx, ry = math.cos(a), math.sin(a), -math.sin(a), math.cos(a)
        cx, cy = fr.world(lx, ly)
        hl, hw = length / 2.0, width / 2.0
        zf = self._ground_at(cx + fx * hl, cy + fy * hl, fr)
        zb = self._ground_at(cx - fx * hl, cy - fy * hl, fr)
        zr = self._ground_at(cx + rx * hw, cy + ry * hw, fr)
        zl = self._ground_at(cx - rx * hw, cy - ry * hw, fr)
        plane = (zf + zb + zr + zl) / 4.0
        lift = 0.0                                        # nothing under it higher than its plane / 下の盛り上がり
        for tf in (-0.85, 0.0, 0.85):
            for tw in (-0.85, 0.0, 0.85):
                g = self._ground_at(cx + fx * hl * tf + rx * hw * tw, cy + fy * hl * tf + ry * hw * tw, fr)
                zp = plane + (zf - zb) / 2.0 * tf + (zr - zl) / 2.0 * tw
                lift = max(lift, g - zp)
        pitch = math.degrees(math.atan2(zf - zb, length))
        roll = math.degrees(math.atan2(zl - zr, width))   # CARLA roll +: the right side goes down / 右が下がる
        act = self.world.try_spawn_actor(lib.find(bp), C.Transform(
            C.Location(cx, cy, plane + lift + 0.01 + dz), C.Rotation(pitch=pitch, yaw=fr.yaw + ang, roll=roll)))
        return [act] if act is not None else []

    def _walk(self, lib, fr, pts):
        """The footpath round the station in the map's pavement look:
        LMHWalk segments end to end along each side and LMHWalkCorner at each
        turn (kerbs outside). Each side is a whole number of segments
        (ring_local), so nothing overlaps; neighbours share the height at
        every joint (the ground there, never below it) and each segment
        pitches from one joint to the next, so the path is one continuous
        surface. Falls back to plain slabs. / 周回歩道（重なり無し・継ぎ目の高さ共通）。"""
        if len(pts) < 2:
            return []
        if not (lib.filter(WALK_BP) and lib.filter(WALK_CORNER_BP)):
            out = []
            for a, b in zip(pts, pts[1:]):
                out += self._path_slabs(lib, fr, a, b)
            return out
        C = self.carla
        half = RING_W / 2.0
        seg = float((self.lay.get("walk") or {}).get("segment", WALK_SEG))
        out = []

        def joint_z(lx, ly, ux, uy):
            """Ground height across the path at a joint: its highest point
            (centre or either kerb), so the lawn never pokes through."""
            wx, wy = fr.world(lx, ly)
            a = math.radians(fr.yaw + math.degrees(math.atan2(uy, ux)))
            rx, ry = -math.sin(a), math.cos(a)
            return max(self._ground_at(wx, wy, fr), self._ground_at(wx + rx * half, wy + ry * half, fr),
                       self._ground_at(wx - rx * half, wy - ry * half, fr))

        def spawn(bp, lx, ly, ang, z, pitch=0.0):
            wx, wy = fr.world(lx, ly)
            a = self.world.try_spawn_actor(lib.find(bp), C.Transform(
                C.Location(wx, wy, z + 0.01), C.Rotation(pitch=pitch, yaw=fr.yaw + ang)))
            if a is not None:
                out.append(a)

        n = len(pts)
        runs = []
        # corners first: their height is where both sides meet / 角の高さを先に決める
        corner_z = {}
        for i in range(1, n - 1):                          # the highest ground under the square / 角の下の最高点
            cx, cy = pts[i]
            corner_z[i] = max(self._ground_at(*fr.world(cx + dx, cy + dy), fr=fr)
                              for dx in (-half, 0.0, half) for dy in (-half, 0.0, half))

        def need(ax, ay, ux, uy, d0, step, z0, z1):
            """How far the piece from d0 to d0 + step must go up so the lawn
            under it (9 points) never pokes through. / 芝が突き抜けない持ち上げ量。"""
            m = 0.0
            for t in (0.15, 0.5, 0.85):
                cx, cy = ax + ux * (d0 + step * t), ay + uy * (d0 + step * t)
                zp = z0 + (z1 - z0) * t
                for o in (-half * 0.9, 0.0, half * 0.9):
                    m = max(m, self._ground_at(*fr.world(cx - uy * o, cy + ux * o), fr=fr) - zp)
            return m
        for i in range(n - 1):
            (ax, ay), (bx, by) = pts[i], pts[i + 1]
            L = math.hypot(bx - ax, by - ay)
            ux, uy = (bx - ax) / L, (by - ay) / L
            s0 = half if i > 0 else 0.0                    # room for the corners / 角の分
            s1 = L - (half if i + 1 < n - 1 else 0.0)
            run = s1 - s0
            k = max(1, int(round(run / seg)))
            exact = abs(k * seg - run) < 0.02
            step = seg if exact else run / k               # not whole: spread evenly (old maps) / 端数あり
            ang = math.degrees(math.atan2(uy, ux))
            zs = [joint_z(ax + ux * (s0 + step * j), ay + uy * (s0 + step * j), ux, uy) for j in range(k + 1)]
            if i > 0:
                zs[0] = corner_z[i]
            if i + 1 < n - 1:
                zs[-1] = corner_z[i + 1]
            for j in range(k):                             # lift over bumps between joints / 継ぎ目間の盛り上がり
                up = need(ax, ay, ux, uy, s0 + step * j, step, zs[j], zs[j + 1])
                if up > 0.0:
                    zs[j] += up
                    zs[j + 1] += up
            if i > 0:                                      # a corner only ever goes up / 角は上げるだけ
                corner_z[i] = max(corner_z[i], zs[0])
            if i + 1 < n - 1:
                corner_z[i + 1] = max(corner_z[i + 1], zs[-1])
            runs.append((ax, ay, ux, uy, s0, step, k, ang, zs, i))
        for ax, ay, ux, uy, s0, step, k, ang, zs, i in runs:
            if i > 0:
                zs[0] = max(zs[0], corner_z[i])
            if i + 1 < n - 1:
                zs[-1] = max(zs[-1], corner_z[i + 1])
            for j in range(k):
                d = s0 + step * (j + 0.5)
                pitch = math.degrees(math.atan2(zs[j + 1] - zs[j], step))
                spawn(WALK_BP, ax + ux * d, ay + uy * d, ang, (zs[j] + zs[j + 1]) / 2.0, pitch)
        for i in range(1, n - 1):                          # the turns / 角
            (px, py), (cx, cy), (nx, ny) = pts[i - 1], pts[i], pts[i + 1]
            l1, l2 = math.hypot(cx - px, cy - py), math.hypot(nx - cx, ny - cy)
            d1 = ((cx - px) / l1, (cy - py) / l1)
            d2 = ((nx - cx) / l2, (ny - cy) / l2)
            # kerbs on the corner's local +x and +y (= +x turned 90 deg): the
            # two edges no segment joins, +d1 and -d2 / 縁石は外側の 2 辺
            rot = lambda v: (-v[1], v[0])
            e2 = (-d2[0], -d2[1])
            xd = d1 if math.hypot(rot(d1)[0] - e2[0], rot(d1)[1] - e2[1]) < 0.1 else e2
            spawn(WALK_CORNER_BP, cx, cy, math.degrees(math.atan2(xd[1], xd[0])), corner_z[i])
        return out

    def _path_slabs(self, lib, fr, a, b):
        """2 m slabs along a footpath segment, turned with it. / 歩道の舗装板。"""
        C = self.carla
        L = math.hypot(b[0] - a[0], b[1] - a[1])
        n = max(1, int(math.ceil(L / 2.0)))
        yaw = fr.yaw + math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))
        out = []
        for k in range(n):
            u = (k + 0.5) / n
            wx, wy = fr.world(a[0] + (b[0] - a[0]) * u, a[1] + (b[1] - a[1]) * u)
            z = self._ground_at(wx, wy, fr)
            act = self.world.try_spawn_actor(lib.find(APRON_BP), C.Transform(
                C.Location(wx, wy, z + 0.004 + 0.002 * (k % 2)), C.Rotation(yaw=yaw)))
            if act is not None:
                out.append(act)
        return out

    def _slabs(self, lib, fr, x0, x1, y0, y1, dz):
        C = self.carla
        size = 2.0
        nx, ny = max(1, int(math.ceil((x1 - x0) / size))), max(1, int(math.ceil((y1 - y0) / size)))
        out = []
        if x1 - x0 < 0.2:
            return out
        for i in range(nx):
            for j in range(ny):
                # tiles spread evenly so they cover the area exactly (they overlap a little)
                lx = x0 + size / 2 + (x1 - x0 - size) * (i / (nx - 1) if nx > 1 else 0.5)
                ly = y0 + size / 2 + (y1 - y0 - size) * (j / (ny - 1) if ny > 1 else 0.5)
                out.extend(self._lay(lib, APRON_BP, fr, lx, ly, 0.0, size, size, dz + 0.003 * ((i + j) % 2)))
        return out

    def plug(self, site_id, kind, u):
        """Plug unit of bay ``kind`` at progress u (0 parked .. 1 in the inlet)."""
        s = self.sites[site_id - 1]
        u = round(max(0.0, min(1.0, u)), 3)
        if s["plug_u"].get(kind) == u:
            return
        s["plug_u"][kind] = u
        piv = self.lay["plug_units"][kind]
        dx = plug_offsets(u)
        C = self.carla
        cmds = []
        for name in ("BoomStage", "Head"):
            a = s["plugs"].get(kind, {}).get(name)
            if a is not None:
                x, y, z = piv[name]
                cmds.append(C.command.ApplyTransform(a.id, C.Transform(C.Location(x + dx, y, z))))
        if cmds:
            self.client.apply_batch(cmds)

    # -- Mini Pod pads and the drone deck / パッドとデッキ
    def pad_poses(self, site_id):
        """World poses (x, y, z, yaw) of the Mini Pods charging on pads M1-M3
        (nose +y local, rear on the pins) / パッド上のミニポッドの姿勢。"""
        s = self.sites[site_id - 1]
        fr = s["frame"]
        out = []
        for x, y, z, yaw in self.lay.get("pod_poses", []):
            wx, wy = fr.world(x, y)
            out.append((wx, wy, fr.z + FLOOR_LIFT + z, _wrap(fr.yaw + yaw)))
        return out

    def pad_lane(self, site_id):
        """The web viewer's approach lane to the pads in world terms:
        {"y_local", "far": (x, y), "reverse_m"} / パッドへの進入路。"""
        s = self.sites[site_id - 1]
        pl = self.lay.get("pad_lane") or {}
        return {"y_local": pl.get("y", 1.0), "far": s["frame"].world(pl.get("far_x", 14.48), pl.get("y", 1.0)),
                "reverse_m": pl.get("reverse_m", 1.1)}

    def deck_pads(self, site_id):
        """World points (x, y, z) of the drone-deck pads (z = pad surface) and
        the yaw drones sit at. / ドローンデッキのパッド位置と機首方位。"""
        s = self.sites[site_id - 1]
        fr = s["frame"]
        pts = [fr.world(x, y) + (fr.z + FLOOR_LIFT + z,) for x, y, z in self.lay.get("deck_pads", [])]
        return pts, _wrap(fr.yaw + self.lay.get("deck_drone_yaw", 180.0))

    def pad(self, site_id, i, pins_u, plug_u):
        """Pad M(i+1): lock pins in (pins_u 0..1, +y into the pod's latches),
        plug in (plug_u 0..1, +y into its rear socket). / ピンとプラグ。"""
        s = self.sites[site_id - 1]
        key = (round(pins_u, 3), round(plug_u, 3))
        if i >= len(s["pad_parts"]) or s["pad_u"].get(i) == key:
            return
        s["pad_u"][i] = key
        C = self.carla
        cmds = []
        for name, u, pivs, mo in (("pins", pins_u, self.lay.get("pad_pins", []), self.lay.get("pad_pins_motion") or {}),
                                  ("plug", plug_u, self.lay.get("pad_plugs", []), self.lay.get("pad_plug_motion") or {})):
            a = s["pad_parts"][i].get(name)
            if a is not None and i < len(pivs):
                x, y, z = pivs[i]
                amt = mo.get("amount", 0.07 if name == "pins" else 0.06) * max(0.0, min(1.0, u))
                cmds.append(C.command.ApplyTransform(a.id, C.Transform(C.Location(x, y + amt, z))))
        if cmds:
            self.client.apply_batch(cmds)

    def deck_plug(self, site_id, i, u):
        """Deck pad (i+1)'s plug into a drone's left-flank socket (u 0..1)."""
        s = self.sites[site_id - 1]
        u = round(max(0.0, min(1.0, u)), 3)
        if i >= len(s["deck_parts"]) or s["deck_parts"][i] is None or s["deck_u"].get(i) == u:
            return
        s["deck_u"][i] = u
        x, y, z = self.lay["deck_plugs"][i]
        amt = (self.lay.get("deck_plug_motion") or {}).get("amount", -0.1546) * u
        C = self.carla
        self.client.apply_batch([C.command.ApplyTransform(s["deck_parts"][i].id, C.Transform(C.Location(x, y + amt, z)))])

    def destroy(self):
        for s in self.sites:
            for a in [p for parts in s.get("pad_parts", []) for p in parts.values()] + \
                    [a for a in s.get("deck_parts", []) if a is not None]:
                try:
                    a.destroy()
                except RuntimeError:
                    pass
            for parts in s.get("plugs", {}).values():
                for a in parts.values():
                    try:
                        a.destroy()
                    except RuntimeError:
                        pass
            for a in [s["actor"]] + s.get("apron", []):
                try:
                    a.destroy()
                except RuntimeError:
                    pass
        self.sites = []


class WalkerGuard(object):
    """Walkers go round a charging station on its footpath instead of through
    it (the props carry no collision and the navmesh was baked without
    them). Twice a second: a walker that is inside a station (with its
    waiting bay) is put on the nearest footpath point; one that will walk in
    within 3 s is sent along the footpath, corner by corner, to the far side
    and on in the direction it was going.
    歩行者はステーションを横切らず周回歩道を通る（侵入しそうなら迂回させる）。"""

    def __init__(self, world, stations):
        import carla
        self.carla, self.world = carla, world
        self.lay = stations.lay
        self.zones = []
        for s in stations.sites:
            fr = s["frame"]
            y0, y1 = self.lay["bbox_min"][1], self.lay["bbox_max"][1]
            q = s.get("queue")
            if q:
                y0, y1 = min(y0, q["y"][0]), max(y1, q["y"][1])
            self.zones.append({"frame": fr, "rect": (self.lay["entrance_x"], self.lay["bbox_max"][0] + 0.3,
                                                     y0 - 0.3, y1 + 0.3),
                               "ring": s.get("ring_local") or [], "centre": s["centre"]})
        self._t = self._list_t = -1e9
        self.ctrl = {}                 # walker id -> its AI controller / 歩行者 -> コントローラ
        self.detour = {}               # walker id -> [world (x, y, z) still to go] / 残りの迂回点
        self.count = 0

    @staticmethod
    def _local(fr, x, y):
        dx, dy = x - fr.x, y - fr.y
        return dx * fr.c + dy * fr.s, -dx * fr.s + dy * fr.c

    @staticmethod
    def _inside(r, lx, ly):
        return r[0] <= lx <= r[1] and r[2] <= ly <= r[3]

    def step(self, t):
        if not self.zones or t - self._t < 0.5:
            return
        self._t = t
        C = self.carla
        if t - self._list_t > 3.0:
            self._list_t = t
            self.ctrl = {}
            for c in self.world.get_actors().filter("controller.ai.walker"):
                p = c.parent
                if p is not None:
                    self.ctrl[p.id] = c
        snap = self.world.get_snapshot()
        for wid, c in list(self.ctrl.items()):
            a = snap.find(wid)
            if a is None:
                self.detour.pop(wid, None)
                continue
            L, v = a.get_transform().location, a.get_velocity()
            if wid in self.detour:                       # on its way round / 迂回中
                pts = self.detour[wid]
                if math.hypot(L.x - pts[0][0], L.y - pts[0][1]) < 1.2:
                    pts.pop(0)
                    if pts:
                        self._go(c, pts[0])
                    else:
                        del self.detour[wid]
                continue
            for z in self.zones:
                if math.hypot(L.x - z["centre"][0], L.y - z["centre"][1]) > 25.0 or not z["ring"]:
                    continue
                fr, r = z["frame"], z["rect"]
                lx, ly = self._local(fr, L.x, L.y)
                vx, vy = v.x * fr.c + v.y * fr.s, -v.x * fr.s + v.y * fr.c
                inside = self._inside(r, lx, ly)
                ahead = self._inside(r, lx + vx * 3.0, ly + vy * 3.0)
                if not inside and not ahead:
                    continue
                ring = z["ring"]
                if inside:                               # put it on the footpath / 歩道へ戻す
                    k = min(range(len(ring)), key=lambda i: math.hypot(ring[i][0] - lx, ring[i][1] - ly))
                    wx, wy = fr.world(*ring[k])
                    try:
                        a_actor = self.world.get_actor(wid)
                        a_actor.set_location(C.Location(wx, wy, fr.z + 1.0))
                    except RuntimeError:
                        pass
                    lx, ly = ring[k]
                sp = math.hypot(vx, vy) or 1.0
                px, py = lx + vx / sp * 20.0, ly + vy / sp * 20.0      # where it was heading / 向かっていた先
                i = min(range(len(ring)), key=lambda k: math.hypot(ring[k][0] - lx, ring[k][1] - ly))
                j = min(range(len(ring)), key=lambda k: math.hypot(ring[k][0] - px, ring[k][1] - py))
                step = 1 if j >= i else -1
                route = [ring[k] for k in range(i, j + step, step)]
                if not self._inside(r, px, py):
                    route.append((px, py))
                self.detour[wid] = [fr.world(x, y) + (fr.z + 0.5,) for x, y in route]
                self._go(c, self.detour[wid][0])
                self.count += 1
                break

    def _go(self, c, p):
        try:
            c.go_to_location(self.carla.Location(p[0], p[1], p[2]))
        except RuntimeError:
            pass
