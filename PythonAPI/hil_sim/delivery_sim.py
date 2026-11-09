#!/usr/bin/env python
"""Car + drone last-mile delivery mission, on any map, with one or more vans.

At random times a batch of 3-5 orders arrives. Each order is either
``face_to_face`` (the delivery car drives to the building) or ``drone``. For
the drone orders the car first finds the central point of those orders,
parks on the road there and launches the drone, which visits every drop
point and returns to land on the car. With ``--vans N`` every batch goes to
exactly one van (the nearest free one), so no order is worked twice.
Inside run_pipeline.py it runs ``--passive`` (the traffic script owns the
tick). See docs/DELIVERY.md.

Drone backends / ドローンのバックエンド:
    --drone kinematic   plain CARLA 0.9.16 (default) / 素の CARLA（既定）
    --drone airsim      CarlaAir (CARLA + AirSim in one UE4 process)

Usage / 使い方:
    python delivery_sim.py                          # Town03, kinematic drone
    python delivery_sim.py --first-now --seed 1     # first batch immediately
    python delivery_sim.py --map Town05 --vans 3    # three vans on Town05
    python run_pipeline.py --map Town05 --delivery 3   # inside the full rig
    python delivery_sim.py --drone airsim --airsim-calibrate   # on CarlaAir

任意のマップで 1 台以上の配送車が動く、車 + ドローンのラストワンマイル配送
ミッション（``--vans N``、各バッチは最寄りの空車 1 台だけが担当）。ランダムな
時刻に 3〜5 件の注文が届き、各注文は ``face_to_face``（配送車が建物まで行く）
か ``drone``。ドローン注文は、車がまずそれらの中心地点を求めて道路上に停車し、
ドローンを発進させる。ドローンは全投下地点を回って車に戻り着陸する。
"""

import argparse
import csv
import datetime
import glob
import json
import math
import os
import random
import signal
import collections
import time
import types
import copy
import sys

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
sys.path.append('../carla')

import carla  # noqa: E402
from lane_spawns import lane_spawn_points  # noqa: E402
from agents.navigation.basic_agent import BasicAgent  # noqa: E402
from delivery.route import SafeRoutePlanner  # noqa: E402  (bounded on imported maps)

from delivery import planning  # noqa: E402
from delivery.planning import DRONE, FACE_TO_FACE, CHARGE, Building, Stop  # noqa: E402
from delivery.traffic import Traffic  # noqa: E402
from delivery.backends import (DroneUnit, Hazards, OBSTACLE_HIT, DRONE_RADIUS,  # noqa: E402
                               DOCKED, RELOADING, DELIVERING, FLYING, FALLING, CRASHED,
                               IN_VAN, solid_labels)
from delivery.hotkeys import Hotkeys  # noqa: E402
from delivery.battery import Battery, BatteryModel, PodEnergyModel  # noqa: E402
from delivery.wind import Wind  # noqa: E402
from delivery.station import ChargeStation, DeckStation, DroneFleet, EvChargers  # noqa: E402
from delivery.battery import charge_rate  # noqa: E402
from delivery import live  # noqa: E402
from delivery import pod as podlib  # noqa: E402
from delivery import lmp_pod, lmp_rig  # noqa: E402
from delivery import backends  # noqa: E402  (OVERLAYS: the tooltip switch)
from delivery import boxes  # noqa: E402
from delivery import charge_sites  # noqa: E402  (LastMile charging stations)
from delivery.lmp_van import LmpVanMixin, LMP_DRONE, LMP_DIRECT  # noqa: E402
from delivery import yard as yardlib  # noqa: E402
from delivery import rules as van_rules  # noqa: E402  (Japan profile: 止まれ etc.)
from delivery import zones as zonelib  # noqa: E402  (pod zones, no-fly zones)
from delivery import drone_sensors  # noqa: E402  (the Drones page's sensor suite)
from delivery import lmh_hub  # noqa: E402  (LastMile Hub + Mini Pods)
from delivery.lmh_hub import LmhVanMixin, LMH_STOP, LMH_WORK  # noqa: E402
import recovery  # noqa: E402  (reverse out of a wedge)

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
VAN_BLUEPRINTS = ("vehicle.mercedes.sprinter", "vehicle.volkswagen.t2_2021",
                  "vehicle.volkswagen.t2", "vehicle.carlamotors.carlacola")

# building meshes that are not somewhere you deliver to / 配送先ではない建物メッシュ
NOT_DELIVERABLE = ("awning", "shelter", "fence", "wall", "kiosk", "bridge", "tunnel",
                   "billboard", "canopy", "gas", "tower")

COLOR = {FACE_TO_FACE: carla.Color(255, 140, 0), DRONE: carla.Color(0, 160, 255),
         "hub": carla.Color(220, 0, 220), "done": carla.Color(0, 200, 0),
         "station": carla.Color(120, 255, 60), "ev": carla.Color(255, 210, 40)}

# car states / 車の状態
IDLE, DRIVING, HANDOVER, DRONE_OUT = "idle", "driving", "handover", "drone_out"
# what a van does once it reaches a stop, in order (see Van.arrive)
# 停車地点に着いてからの手順（Van.arrive 参照）
STOPPING, JACKS_DOWN, HATCH_OPEN = "stopping", "jacks_down", "hatch_open"
DOCKING = "docking"          # creeping the side port level with a box station / 箱へ横付け
HATCH_CLOSE, JACKS_UP = "hatch_close", "jacks_up"
# the pod's EV battery / ポッドの EV バッテリー
EV_CHARGING, WAIT_CHARGER, STRANDED = "ev_charging", "waiting_charger", "stranded"
YARD_IN, YARD_OUT = "yard_in", "yard_out"     # scripted moves in an off-road yard
QUEUED = "queued"        # in a charging station's waiting bay, off the lane / 待機ベイで待機
# face-to-face handover through the PodV2 side port / サイドポートでの対面受け渡し
SIDE_OPEN, ARM_OUT, PARCEL_DROP = "side_open", "arm_out", "parcel_drop"
ARM_IN, SIDE_CLOSE = "arm_in", "side_close"
PARCEL_LINGER_S = 10.0       # a dropped parcel is picked up after this / 置いた荷物が消えるまで
GROUND_LABELS = ("Roads", "Sidewalks", "Ground", "Terrain", "RoadLines")

# off-road guard / 路外逸脱ガード
OFFROAD_MARGIN = 1.0    # beyond the lane edge, outside junctions [m] / 車線端からの許容 [m]
OFFROAD_S = 1.0         # ...for this long triggers recovery / この時間続けば回復
STUCK_S = 8.0           # throttle on but not moving / アクセル中なのに停止
RETRY_WINDOW_S = 20.0   # a 2nd trigger this soon moves the van / 再発時は車を移動
NUDGE_M = 6.0           # a stuck van is moved this far ahead along its route [m]
MAX_RECOVERIES = 5      # per stop, then the van is placed at the stop / 停車地ごとの上限


def loc(p):
    return carla.Location(x=p[0], y=p[1], z=p[2])


def tup(location):
    return (location.x, location.y, location.z)


# --- world analysis / ワールド解析 -----------------------------------------

MAPS_JSON = os.path.join(os.path.dirname(os.path.abspath(__file__)), "maps", "maps.json")
_BUILDINGS = {}


def map_buildings(world):
    """The map's buildings: CARLA's Buildings-tagged objects, or for your own
    maps whose buildings came in under another tag (Oita Station: merged
    tiles tagged Terrain) the building table named in maps/maps.json
    ("buildings_csv": id, name, height_m, x_east_m, y_north_m, footprint_m2;
    CARLA x = east, y = -north), as square footprints of the same area.
    建物一覧：CARLA の Buildings タグ、無ければ maps.json の建物表から作る。"""
    name = world.get_map().name.split("/")[-1]
    if name in _BUILDINGS:
        return _BUILDINGS[name]
    objs = list(world.get_environment_objects(carla.CityObjectLabel.Buildings))
    if not objs:
        try:
            entry = next((m for m in json.load(open(MAPS_JSON)).values()
                          if isinstance(m, dict) and m.get("name") == name), {})
        except (OSError, ValueError):
            entry = {}
        path = os.path.expanduser(entry.get("buildings_csv", ""))
        if path and os.path.exists(path):
            for r in csv.DictReader(open(path, encoding="utf-8")):
                h = float(r["height_m"])
                half = max(math.sqrt(float(r["footprint_m2"] or 16.0)), 4.0) / 2.0
                bb = carla.BoundingBox(carla.Location(float(r["x_east_m"]), -float(r["y_north_m"]), h / 2.0),
                                       carla.Vector3D(half, half, h / 2.0))
                objs.append(types.SimpleNamespace(name=r["name"] or r["id"], bounding_box=bb))
            print("  %d buildings from %s (none tagged Buildings in the map) / 建物表から読込"
                  % (len(objs), os.path.basename(path)), flush=True)
    _BUILDINGS[name] = objs
    return objs


NOT_PAVEMENT = ("Roads", "RoadLines", "Buildings", "Vehicles", "Car", "Truck", "Bus", "Motorcycle",
                "Bicycle", "Pedestrians", "Rider", "Dynamic", "Vegetation", "Poles")


def pavement_drop(world, drive, center, inset=0.4, reach=3.0, step=0.25):
    """The pavement point in front of a building on a map whose pavements
    are only in the model (no sidewalk lanes in the road logic): step out
    from the lane edge towards the building until the ground rises onto the
    kerb (a ray every ``step`` m), then ``inset`` m further in. None if no
    pavement within ``reach`` m of the lane edge.
    道路情報に歩道が無いマップ用：車線端から建物側へ進み、縁石を越えた歩道上の点。"""
    t = drive.transform
    L, r = t.location, t.get_right_vector()
    lat = (center.x - L.x) * r.x + (center.y - L.y) * r.y
    s = 1.0 if lat >= 0 else -1.0
    nx, ny = r.x * s, r.y * s

    def ground(x, y, top):
        for h in world.cast_ray(carla.Location(x, y, top + 2.0), carla.Location(x, y, top - 1.5)):
            return h
        return None

    road = ground(L.x, L.y, L.z)
    if road is None:
        return None
    rz = road.location.z
    d = drive.lane_width / 2.0 - 0.3
    while d < drive.lane_width / 2.0 + reach and d + inset < abs(lat):
        h = ground(L.x + nx * d, L.y + ny * d, rz)
        if h is not None and str(h.label) not in NOT_PAVEMENT and rz + 0.06 < h.location.z < rz + 0.45:
            px, py = L.x + nx * (d + inset), L.y + ny * (d + inset)
            p = ground(px, py, h.location.z)
            if p is None or str(p.label) in NOT_PAVEMENT or abs(p.location.z - h.location.z) > 0.1:
                return None                              # not clear pavement / 歩道でない
            return (px, py, p.location.z)
        d += step
    return None


def find_buildings(world, wmap, max_sidewalk_gap=15.0, max_curb_gap=12.0, grid=12.0):
    """Buildings that have a sidewalk in front and a drivable lane beside it.

    Returns (buildings, tallest_roof_z). Sub-meshes of one block that share
    the same sidewalk spot are merged.
    前に歩道、横に走行車線がある建物を返す。同じ歩道地点の部分メッシュは統合。
    """
    objs = map_buildings(world)
    buildings, seen, roof = [], set(), -1e9
    model_pavements = bool(objs) and not boxes._any_sidewalk(wmap)
    for o in objs:
        bb = o.bounding_box
        roof = max(roof, bb.location.z + bb.extent.z)
        if max(bb.extent.x, bb.extent.y) < 2.0 or \
                any(k in o.name.lower() for k in NOT_DELIVERABLE):
            continue
        center = bb.location
        if model_pavements:
            # pavements only in the model (OitaKosen): the kerb lane nearest
            # the building, the pavement point in front of it / 模型だけの歩道
            drive = wmap.get_waypoint(center, project_to_road=True, lane_type=carla.LaneType.Driving)
            if drive is None or drive.is_junction:
                continue
            dl = drive.transform.location
            if math.hypot(dl.x - center.x, dl.y - center.y) > max(bb.extent.x, bb.extent.y) + max_sidewalk_gap:
                continue
            drop = pavement_drop(world, drive, center)
            if drop is None:
                continue
            key = (round(drop[0] / grid), round(drop[1] / grid))
            if key in seen:
                continue
            seen.add(key)
            buildings.append(Building(len(buildings) + 1, o.name, drop, tup(dl)))
            continue
        side = wmap.get_waypoint(center, project_to_road=True,
                                 lane_type=carla.LaneType.Sidewalk)
        if side is None:
            continue
        reach = max(bb.extent.x, bb.extent.y) + max_sidewalk_gap
        if side.transform.location.distance(center) > reach:
            continue
        drive = wmap.get_waypoint(side.transform.location, project_to_road=True,
                                  lane_type=carla.LaneType.Driving)
        if drive is None or drive.is_junction or \
                drive.transform.location.distance(side.transform.location) > max_curb_gap:
            continue
        drop = tup(side.transform.location)
        key = (round(drop[0] / grid), round(drop[1] / grid))
        if key in seen:
            continue
        seen.add(key)
        buildings.append(Building(len(buildings) + 1, o.name, drop,
                                  tup(drive.transform.location)))
    return buildings, roof


# everything the drone has to fly over; flat ground classes are left out
# ドローンが越える対象。地面系の平坦なクラスは除外
OBSTACLE_LABELS = ("Buildings", "Vegetation", "Poles", "TrafficLight", "TrafficSigns",
                   "Walls", "Fences", "Bridge", "GuardRail", "Static")


def find_obstacles(world):
    """Footprint + top of every building, tree, pole... in the map.

    マップ内の建物・樹木・ポールなどの平面形状と上端高さ。
    """
    out = []
    for name in OBSTACLE_LABELS:
        label = getattr(carla.CityObjectLabel, name, None)
        if label is None:
            continue
        for o in (map_buildings(world) if name == "Buildings" else world.get_environment_objects(label)):
            bb = o.bounding_box
            out.append(planning.Obstacle(bb.location.x, bb.location.y, bb.extent.x,
                                         bb.extent.y, bb.rotation.yaw,
                                         bb.location.z + bb.extent.z))
    return out


def make_surface_fn(world, step=20.0):
    """Highest surface (ground, roof, tree) straight below a leg, by raycast.

    Catches hills, which the obstacle list does not contain. One downward ray
    every ``step`` metres; the first hit is the top surface there.
    区間の真下にある最も高い面（地面・屋根・樹木）をレイキャストで求める。
    障害物リストに無い丘を検出する。``step`` m ごとに下向きのレイを 1 本飛ばし、
    最初に当たった点をその地点の最上面とする。
    """
    def surface(a, b):
        n = max(1, int(planning.dist2d(a, b) // step))
        top = None
        for i in range(n + 1):
            u = i / float(n)
            x, y = a[0] + u * (b[0] - a[0]), a[1] + u * (b[1] - a[1])
            hits = world.cast_ray(carla.Location(x, y, 600.0), carla.Location(x, y, -100.0))
            if hits:
                z = hits[0].location.z
                top = z if top is None else max(top, z)
        return top
    return surface


def keep_left_map(wmap):
    """True when traffic drives on the left (maps/maps.json "keep_left"). / 左側通行の自作マップ。"""
    name = wmap.name.split("/")[-1]
    try:
        return any(isinstance(m, dict) and m.get("name") == name and m.get("keep_left")
                   for m in json.load(open(MAPS_JSON)).values())
    except (OSError, ValueError):
        return False


def hub_candidates(wmap, spacing=4.0):
    """Kerb-side, non-junction driving-lane points where the car may park.

    The kerb lane has no driving lane beside it on the kerb side: the right
    on keep-right maps, the left on keep-left ones (Oita Station). On a
    two-way road the lane by the centre line has oncoming traffic beside it,
    so it is not taken.
    車が停車できる、交差点外の路肩側の走行車線上の点（右側通行は右、左側通行は左）。
    """
    left = keep_left_map(wmap)
    out = []
    for wp in wmap.generate_waypoints(spacing):
        if wp.lane_type != carla.LaneType.Driving or wp.is_junction:
            continue
        kerb_side = wp.get_left_lane() if left else wp.get_right_lane()
        if kerb_side is not None and kerb_side.lane_type == carla.LaneType.Driving:
            continue
        # where there is pavement close by, it must be on the kerb side (not
        # across the median) / 近くに歩道があるなら路肩側にあること
        L = wp.transform.location
        sw = wmap.get_waypoint(L, project_to_road=True, lane_type=carla.LaneType.Sidewalk)
        if sw is not None and sw.transform.location.distance(L) < 8.0:
            f, q = wp.transform.get_forward_vector(), sw.transform.location
            on_right = f.x * (q.y - L.y) - f.y * (q.x - L.x) > 0
            if on_right == left:
                continue
        out.append(tup(L))
    return out


def make_road_dist(grp):
    def road_dist(a, b):
        route = grp.trace_route(loc(a), loc(b))
        if len(route) < 2:
            return planning.dist2d(a, b)
        pts = [w.transform.location for w, _ in route]
        return sum(pts[i].distance(pts[i + 1]) for i in range(len(pts) - 1))
    return road_dist


# --- logging / 記録 ---------------------------------------------------------

# air kept clear around a hovering drone: its 0.4 m half-span plus room to
# drift in a light gust / 投下時に機体周囲に確保する空間（半幅＋流され余裕）
HOVER_CLEARANCE = 1.0


class SkyCheck(object):
    """Pre-flight checks on the real geometry, by raycast.

    The obstacle list is bounding boxes, which miss balconies, awnings and
    tree crowns over a sidewalk, so drones planned on it alone clipped
    buildings at the drop. Before every flight the straight line at cruise
    height and the air column over the drop are ray-cast: a blocked line is
    flown higher, and a blocked drop is served from the nearest clear spot
    towards the road. What still hits something after that (a gust pushing
    the drone into a wall) is a real failure.
    実際の形状に対する飛行前チェック（レイキャスト）。障害物リストはバウンディ
    ングボックスのためバルコニー・日除け・歩道上の樹冠を見落とし、投下地点で
    建物に接触していた。飛行前に巡航高度の直線と投下地点上空の空間を調べ、
    塞がっていれば高度を上げる／道路側の空いた地点から投下する。
    """

    def __init__(self, world, obstacles, surface, args):
        self.world, self.obstacles, self.surface, self.args = world, obstacles, surface, args
        self.solid = solid_labels(OBSTACLE_HIT)

    def _blocked(self, a, b):
        for h in self.world.cast_ray(loc(a), loc(b)):
            if h.label in self.solid:
                return h
        return None

    def column_clear(self, x, y, top, bottom, r=0.0):
        """No solid thing in a vertical column of radius ``r``.

        Rays down the middle and on rings at r and r/2 (17 rays): one ring
        alone let a lamp arm or pole between the rays through.
        中心と半径 r・r/2 の円上の計 17 本（1 周だけでは街灯の腕などがすり抜けた）。
        """
        pts = [(x, y)]
        for rr in ((r, r / 2.0) if r else ()):
            pts += [(x + rr * math.cos(math.radians(a)), y + rr * math.sin(math.radians(a)))
                    for a in range(0, 360, 45)]
        return all(self._blocked((px, py, top), (px, py, bottom)) is None for px, py in pts)

    def open_sky(self, p, up=60.0):
        """Nothing overhead (a tree, a bridge) of a parked pod at ``p``."""
        return self.column_clear(p[0], p[1], p[2] + up, p[2] + 4.5)

    def hover_point(self, drop, toward, top):
        """Where to hover for ``drop``: (point, metres off the drop), or (None, None).

        The drop itself if the air above it is clear for the drone's width
        plus a drift margin; else the nearest clear spot on the way to
        ``toward`` (the curb, beside the open road); else on rings up to 6 m
        around the drop. Nowhere clear means the drop can't be flown safely.
        投下地点の上空が機体幅＋流され余裕で空いていればそこ。だめなら縁石へ
        向かう途中、さらに周囲 6 m までの円上で最も近い空いた地点。無ければ None。
        """
        dx, dy = toward[0] - drop[0], toward[1] - drop[1]
        n = math.hypot(dx, dy)
        cands = [(drop[0], drop[1])]
        if n > 0.5:
            steps = int(min(n, 12.0) / 0.75)
            cands += [(drop[0] + dx / n * 0.75 * k, drop[1] + dy / n * 0.75 * k)
                      for k in range(1, steps + 1)]
        for r in (1.5, 3.0, 4.5, 6.0):
            cands += [(drop[0] + r * math.cos(math.radians(a)),
                       drop[1] + r * math.sin(math.radians(a))) for a in range(0, 360, 45)]
        for x, y in cands:
            if self.column_clear(x, y, top, drop[2] + 0.8, r=HOVER_CLEARANCE):
                return (x, y, drop[2]), math.hypot(x - drop[0], y - drop[1])
        return None, None

    def surface_z(self, x, y, z):
        """Height of whatever is underfoot near ``z`` (a sidewalk waypoint sits
        at road level, below the raised sidewalk). / 実際の足元の高さ。"""
        hits = self.world.cast_ray(carla.Location(x, y, z + 3.0), carla.Location(x, y, z - 3.0))
        return hits[0].location.z if hits else z

    def _top_at(self, x, y, z):
        for h in self.world.cast_ray(carla.Location(x, y, z + 400.0), carla.Location(x, y, z - 1.0)):
            if h.label in self.solid:
                return h.location.z
        return z

    def flight_z(self, a, b):
        """A safe cruise height for the flight a -> b: the highest of its legs
        when it has to go round no-fly zones. / 迂回する場合は各区間の最高値。"""
        pts = zonelib.air_path(a, b)
        if len(pts) <= 1:
            return self._line_z(a, b)
        here, z = a, -1e9
        for p in pts:
            nxt = (p[0], p[1], b[2])
            z = max(z, self._line_z(here, nxt))
            here = nxt
        return z

    def _line_z(self, a, b):
        """A safe cruise height for the straight flight a -> b.

        The footprint rule (planning.leg_altitude) first, then raised until a
        ray along the line at that height hits nothing.
        """
        if self.args.cruise_alt:
            return max(a[2], b[2]) + self.args.cruise_alt
        z = planning.leg_altitude(a, b, self.obstacles, corridor=self.args.corridor,
                                  margin=self.args.clearance, min_alt=self.args.min_alt,
                                  surface=self.surface(a, b))[0]
        # three parallel lines a drone-width-plus apart: the drone is 0.77 m
        # wide and drifts a little even in light wind
        # 機体幅＋余裕の間隔で平行な 3 本（機体幅 0.77 m、微風でも少し流される）
        dx, dy = b[0] - a[0], b[1] - a[1]
        n = math.hypot(dx, dy) or 1.0
        ox, oy = -dy / n * 1.0, dx / n * 1.0
        for _ in range(6):
            hit = None
            for k in (0.0, 1.0, -1.0):
                hit = self._blocked((a[0] + k * ox, a[1] + k * oy, z + 0.2),
                                    (b[0] + k * ox, b[1] + k * oy, z + 0.2))
                if hit is not None:
                    break
            if hit is None:
                return z
            z = max(z + 2.0, self._top_at(hit.location.x, hit.location.y, z) + 3.0)
        return z


def choose_station(wmap, buildings, n_pads, sky):
    """A row of ``n_pads`` charge pads on a sidewalk near the middle of the city.

    Pads are 1.8 m apart, each with open sky and 0.6 m of clearance around it.
    街の中心付近の歩道に 1.8 m 間隔で充電パッドを並べる（上空と周囲 0.6 m が空いていること）。
    """
    drops = [b.drop for b in buildings]
    c = planning.centroid(drops)
    for drop in sorted(drops, key=lambda d: planning.dist2d(d, c))[:60]:
        wp = wmap.get_waypoint(loc(drop), project_to_road=True,
                               lane_type=carla.LaneType.Sidewalk)
        if wp is not None:
            pads = []
            while len(pads) < n_pads and wp is not None:
                p = wp.transform.location
                pads.append((p.x, p.y, sky.surface_z(p.x, p.y, p.z) + 0.02))
                nxt = wp.next(1.8)
                wp = nxt[0] if nxt else None
            if len(pads) == n_pads and all(
                    sky.column_clear(x, y, z + 60.0, z + 0.5, r=0.6) for x, y, z in pads):
                return ChargeStation(pads)
        else:
            # Fallback for maps without OpenDrive sidewalks: line pads along roadside
            d_loc = loc(drop)
            pads = []
            for i in range(n_pads):
                px = d_loc.x + i * 1.8
                py = d_loc.y
                pads.append((px, py, sky.surface_z(px, py, d_loc.z) + 0.02))
            if len(pads) == n_pads and all(
                    sky.column_clear(x, y, z + 60.0, z + 0.5, r=0.6) for x, y, z in pads):
                return ChargeStation(pads)
    return None


def choose_chargers(wmap, hubs, near, n, spacing=12.0):
    """``n`` road-side spots for EV chargers, the closest to ``near`` that are
    ``spacing`` apart: [((x, y, z), lane yaw)]. / 充電器の路側位置。"""
    out = []
    for h in sorted(hubs, key=lambda h: planning.dist2d(h, near)):
        if all(planning.dist2d(h, p) >= spacing for p, _ in out):
            yaw = wmap.get_waypoint(loc(h)).transform.rotation.yaw
            out.append((h, yaw))
            if len(out) == n:
                break
    return out


MAP_LANE_KINDS = {"driving": "d", "sidewalk": "s", "shoulder": "h", "parking": "p",
                  "restricted": "h", "biking": "h", "bidirectional": "d", "entry": "d",
                  "exit": "d", "onRamp": "d", "offRamp": "d"}


def map_lanes(wmap, step=2.0):
    """Every lane of the road logic as CARLA's own top-down map draws it
    (PythonAPI/examples/no_rendering_mode.py): its surface as a polygon to
    the lane's width, and the lane markings along its edges, solid or broken,
    white or yellow. The lanes come from the OpenDRIVE file itself, section by
    section, so pavements are included and each lane runs to its section's
    ends. Returns (lanes, marks, centre lines of the driving lanes).
    lanes: [kind, [x, y, x, y, ...]]  kind d driving, s sidewalk, h shoulder, p parking
    marks: [colour w/y, broken 0/1, [x, y, ...]]
    道路論理の各車線を CARLA の 2D 地図と同じ形（車線幅の面と区画線）で返す。"""
    import xml.etree.ElementTree as ET
    lanes, marks, centres = [], [], []

    def shift(wp, d):
        t = wp.transform
        r = t.get_right_vector()
        return (round(t.location.x + r.x * d, 2), round(t.location.y + r.y * d, 2))

    def flat(pts):
        return [v for pt in pts for v in pt]

    try:
        roads = list(ET.fromstring(wmap.to_opendrive()).iter("road"))
    except ET.ParseError:
        return lanes, marks, centres
    for road in roads:
        rid = int(road.get("id"))
        secs = list(road.iter("laneSection"))
        starts = [float(x.get("s")) for x in secs]
        ends = starts[1:] + [float(road.get("length"))]
        for sec, s0, s1 in zip(secs, starts, ends):
            if s1 - s0 < 0.05:
                continue
            n = max(1, int(math.ceil((s1 - s0) / step)))
            ss = [s0 + 0.01 + (s1 - s0 - 0.02) * i / n for i in range(n + 1)]
            for lane in sec.iter("lane"):
                kind = MAP_LANE_KINDS.get(lane.get("type"))
                lid = int(lane.get("id"))
                if not kind or lid == 0:
                    continue
                wps = []
                for sv in ss:
                    w = wmap.get_waypoint_xodr(rid, lid, sv)
                    if w is not None and w.lane_id == lid:
                        wps.append(w)
                if len(wps) < 2 or max(w.lane_width for w in wps) < 0.2:
                    continue
                left = [shift(w, -w.lane_width / 2) for w in wps]
                right = [shift(w, w.lane_width / 2) for w in wps]
                lanes.append([kind, flat(left + right[::-1])])
                if kind != "d":
                    continue
                centres.append([[round(w.transform.location.x, 1), round(w.transform.location.y, 1)]
                                for w in wps])
                for side, sign in (("left_lane_marking", -1), ("right_lane_marking", 1)):
                    run, key = [], None
                    for w in wps + [None]:
                        lm = getattr(w, side) if w is not None else None
                        k = (str(lm.type), lm.color == carla.LaneMarkingColor.Yellow) if lm is not None else None
                        if k != key or w is None:
                            if key is not None and len(run) > 1:
                                name, yellow = key
                                parts = ([name] if name in ("Solid", "Broken") else
                                         [name[:5], name[5:]] if name in (
                                             "SolidSolid", "SolidBroken", "BrokenSolid", "BrokenBroken") else [])
                                for i, part in enumerate(parts):
                                    pts = [shift(q, sign * (q.lane_width / 2 + 0.25 * i)) for q in run]
                                    marks.append(["y" if yellow else "w", 1 if part == "Broken" else 0, flat(pts)])
                            run = run[-1:] if w is not None else []
                            key = k
                        if w is not None:
                            run.append(w)
    return lanes, marks, centres


def export_map(world, wmap, station, chargers=None, boxmgr=None, hubs=None):
    """Lanes and lane markings (as CARLA draws its map), building footprints,
    the station, chargers and box stations, for the dashboard's 2D map;
    ``stops`` = the kerb points a pod may stop at (meeting points snap to them).
    ダッシュボードの 2D 地図用の車線・区画線・建物・充電所。"""
    lanes, marks, roads = map_lanes(wmap)
    blds = []
    for o in map_buildings(world):
        bb = o.bounding_box
        blds.append([round(bb.location.x, 1), round(bb.location.y, 1), round(bb.extent.x, 1),
                     round(bb.extent.y, 1), round(bb.rotation.yaw, 1)])
    return {"map": wmap.name, "roads": roads, "lanes": lanes, "marks": marks, "buildings": blds,
            "station": [[round(x, 2), round(y, 2)] for x, y, _ in station.pads],
            "chargers": [[round(x, 2), round(y, 2)] for x, y, _ in
                         (chargers.spots if chargers else [])],
            "boxes": [[round(b.site.loc[0], 2), round(b.site.loc[1], 2)]
                      for b in (boxmgr.boxes.values() if boxmgr else [])],
            "stops": [[round(h[0], 1), round(h[1], 1)] for h in (hubs or [])]}


class EventLog(object):
    FIELDS = ("sim_t", "van", "event", "batch", "order", "kind", "x", "y", "z", "detail")

    def __init__(self, enabled):
        self.rows = []
        self.f = self.w = None
        self.dir = None
        if enabled:
            stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            self.dir = os.path.join(BASE_DIR, "data", "delivery", stamp)
            os.makedirs(self.dir, exist_ok=True)
            self.f = open(os.path.join(self.dir, "events.csv"), "w", newline="")
            self.w = csv.DictWriter(self.f, self.FIELDS)
            self.w.writeheader()

    def __call__(self, t, event, order=None, batch=None, pos=None, detail="", van=None):
        row = {"sim_t": round(t, 2), "van": van if van is not None else "", "event": event,
               "batch": batch if batch is not None else (order.batch if order else ""),
               "order": order.id if order else "", "kind": order.kind if order else "",
               "x": "", "y": "", "z": "", "detail": detail}
        if pos is not None:
            row.update(x=round(pos[0], 2), y=round(pos[1], 2), z=round(pos[2], 2))
        self.rows.append(row)
        if self.w:
            self.w.writerow(row)
            self.f.flush()
        tag = "[%7.1fs]%s %-16s" % (t, (" van%d" % van) if van is not None else "", event)
        what = (" #%d %s" % (order.id, order.kind)) if order else ""
        print(tag + what + ((" " + detail) if detail else ""), flush=True)

    def close(self, summary):
        if self.dir:
            with open(os.path.join(self.dir, "summary.json"), "w") as f:
                json.dump(summary, f, indent=2, ensure_ascii=False)
        if self.f:
            self.f.close()


# --- one van / 配送車 1 台 ------------------------------------------------------

# label colour per van; debug text glows, so these stay light
# 車ごとのラベル色（デバッグ文字は発光するため明るめ）
VAN_COLORS = [carla.Color(255, 255, 255), carla.Color(255, 220, 0), carla.Color(0, 255, 180),
              carla.Color(255, 90, 200), carla.Color(120, 190, 255), carla.Color(255, 150, 60)]


class Van(LmpVanMixin):
    """One delivery van, the drone it currently carries and its off-road guard.

    It works one batch at a time, handed to it by the Fleet's dispatcher. Its
    drone comes from the shared pool (``sky.drones``) and may be swapped for
    a standby when the battery runs low.
    配送車 1 台・現在積んでいるドローン・路外逸脱ガード。ドローンは共用プール
    から来て、電池が減ると待機機と交換されることがある。
    """

    def __init__(self, vid, args, world, car, hubs, grp, log, obstacles, gen, sky,
                 pod=None):
        self.vid, self.args, self.world, self.car = vid, args, world, car
        self.hubs, self._log, self.obstacles, self.gen = hubs, log, obstacles, gen
        self.sky = sky           # wind, battery model, drone pool, raycast checks
        self.drone = None        # DroneUnit in this van (set by the pool) / 車内の機体
        self.incoming = None     # a drone flying here to replace it / 向かってくる機体
        self.surface = make_surface_fn(world)
        self.color = VAN_COLORS[(vid - 1) % len(VAN_COLORS)]
        self.wmap = world.get_map()
        self.road_dist = make_road_dist(grp)
        # in the car frame: the roof spot (the pod's hatch) and where the drone
        # parks (inside the pod; on the roof of a stock van)
        # 車座標：屋根の位置（ポッドのハッチ）と機体の格納位置（ポッド内、標準の車は屋根）
        self.pod = pod
        self.lmp = False
        if isinstance(pod, lmp_pod.LastMilePod):
            self.lmp_init()                 # 4 drones on deck, magazine / 4 機と電池
        if pod is not None:
            self.pad, self.dock = pod.pad, pod.dock
        else:
            bb = car.bounding_box
            self.pad = self.dock = carla.Location(bb.location.x, bb.location.y,
                                                  bb.location.z + bb.extent.z + 0.15)
        self.stages = []         # what is left to do at this stop / この停車地点の残り手順
        # the steering PID must know the real step: run_pipeline's traffic
        # master runs at 30 Hz, standalone sync at 20 Hz
        # 操舵 PID に実際の刻みを渡す（パイプラインの tick マスタは 30 Hz、単体は 20 Hz）
        step = world.get_settings().fixed_delta_seconds or 0.05
        self.agent = BasicAgent(car, target_speed=args.speed,
                                # BasicAgent caps throttle at 0.75, which cannot pull
                                # the 3 t van away on Town03's ~17 deg hills
                                # 既定のスロットル上限 0.75 では 3 t の車が坂道発進できない
                                opt_dict={"ignore_traffic_lights": args.ignore_lights,
                                          "max_throttle": 1.0, "dt": step},
                                map_inst=self.wmap, grp_inst=grp)
        self.stops = []          # current batch plan / 現在のバッチの停車計画
        self.pending = []        # undelivered orders of the current batch
        self.state = IDLE
        self.timer = 0.0
        self.delivered = []
        self.failed = []         # orders that could not be delivered / 配達失敗
        self.batches = 0
        # the pod's EV battery: pods start part-charged so they don't all need
        # a charger at once / 全車が同時に充電しないよう初期残量はばらつかせる
        self.pem = sky.pem
        self.energy = Battery(random.Random(args.seed + 300 + vid).uniform(
            min(args.pod_start_min, 100.0), 100.0))
        self.pod_km = 0.0
        self.ev_charges = 0
        self.fleet = None        # set by Fleet (to hand orders back) / Fleet が設定
        self._last_pos = None
        self._strand_t = self._wait_t = 0.0
        self.yard_bay = None     # bay index in the charging yard / ヤードのベイ番号
        self._unplug_t = None    # charging-station plug: when it started to pull out / プラグ抜き開始
        self.charge_asap = False # sent to charge from the dashboard: next time it is free / 充電指示
        self._ref_z = None       # ground-ray reference height for scripted moves / 地面探索の基準高さ
        self._next_bay = None    # the bay claimed while waiting in a waiting bay / 待機中に確保したベイ
        self._queue_t = None     # when it started waiting / 待機開始
        self._yield_t = 0.0      # waiting for a gap in the traffic / 交通の切れ目待ち
        self.charge_kw, self.charge_kwh = 0.0, 0.0   # charger power now, energy this visit / 充電電力と量
        self.pack_kwh = (args.hub_kwh if self.CHARGE_KIND == "lmh" else (args.lmp_kwh or args.pod_kwh))
        self.handover_queue = []  # orders still to hand out at this stop / 未受け渡し
        self.dropped = []        # (parcel actor, time it goes) / 置いた荷物と消える時刻
        self._fall = None        # parcel falling from the tray / 落下中の荷物
        self._mover = None
        # physics off for scripted moves (yard, tow); the dashboard's vehicle
        # tuning leaves the pod alone then / スクリプト移動中は物理オフ
        self.kinematic = False
        self._ground_z = {}
        # the drone stop being worked / 作業中のドローン地点
        self.drone_queue = []    # orders still to fly / 未飛行の注文
        self.current = None      # order on the drone right now / 運搬中の注文
        self.reload_t = self.drone_wait = 0.0
        self._plans = {}         # order id -> (hover point, cruise z, moved m)
        self._said = set()       # one-off log lines already printed at this stop
        self._offroad_t = self._stuck_t = 0.0
        self._reverse = None          # recovery.ReverseOut while backing out / 後退中
        self._last_recover_t = -1e9
        self._stop_recoveries = 0
        self.max_offset = 0.0
        self.reroutes = self.lane_resets = 0
        # Japan profile maps: delivery/rules.VanRules (set by Fleet), else None
        # 日本のマップでは交通ルール（Fleet が設定）。他は None
        self.rules = None

    def log(self, *a, **k):
        self._log(*a, van=self.vid, **k)

    def _world(self, spot):
        # Transform.transform() rewrites its argument in place: pass a copy,
        # or the spot creeps away on every call (it reached z=30 km)
        # Transform.transform() は引数を書き換えるためコピーを渡す
        p = self.car.get_transform().transform(carla.Location(spot.x, spot.y, spot.z))
        return (p.x, p.y, p.z)

    def roof(self):
        return self._world(self.pad)

    def dock_pos(self):
        return self._world(self.dock)

    def car_pos(self):
        return tup(self.car.get_location())

    # -- transitions
    def batch_need(self, stops):
        """Pod battery % to drive ``stops`` and then on to a charger, reserve
        included. / 停車地を回って充電器まで走るのに必要な %（予備込み）。"""
        here, metres = self.car_pos(), 0.0
        for s in stops:
            metres += self.road_dist(here, s.location)
            here = s.location
        chargers = self.sky.chargers.spots
        if chargers:
            metres += min(planning.dist2d(here, c) for c in chargers) * 1.4
        km = metres / 1000.0
        # driving at ~60 % of the speed limit (lights, traffic), plus each stop's work
        # 走行は制限速度の約 6 割（信号・交通）、各停車地の作業時間を加算
        secs = km * 1000.0 / (self.args.speed / 3.6 * 0.6)
        for s in stops:
            secs += (self.args.handover_s if s.kind == FACE_TO_FACE else
                     70.0 * len(s.orders)) + 2 * (self.args.jack_s + self.args.hatch_s)
        return self.pem.cost(km, secs) + self.pem.reserve

    def assign(self, batch, t):
        """Take ``batch`` if the battery can do it; False means charge first."""
        hubs = self.hubs
        zs = self.fleet.zones if self.fleet is not None else None
        air = zs.air if zs is not None else None
        mine = zs.meet_of(self.vid) if zs is not None else []
        if mine:
            # the user's meeting points: drones launch and land only there
            # (each snapped to the nearest kerb point the pod can stop at)
            # ユーザーが選んだ合流地点だけで発進・回収（最寄りの停車可能点へ）
            hubs = [min(self.hubs, key=lambda h, p=p: planning.dist2d(h, p)) for p in mine]
        elif air:
            # nothing flies over a no-fly zone: drone orders inside one go face
            # to face, and the drone stop is never inside one
            # 飛行禁止区域：中のドローン注文は対面に、ハブは区域外に
            hubs = [h for h in self.hubs if not air.blocked(h)]
        if air:
            for o in batch:
                if o.kind == DRONE and (air.blocked(o.drop) or not hubs):
                    o.kind = FACE_TO_FACE
                    self.log(t, "nofly_f2f", o, batch=o.batch, pos=o.drop,
                             detail="#%d is in a no-fly zone: delivered face to face" % o.id)
            hubs = hubs or self.hubs
        stops = planning.plan_batch(batch, self.car_pos(), hubs,
                                    self.road_dist, self.args.hub_radius)
        need = self.batch_need(stops)
        if self.energy.level < need:
            self.log(t, "batch_declined", batch=batch[0].batch,
                     detail="pod at %.0f%%, the batch needs ~%.0f%%: charging first"
                     % (self.energy.level, need))
            return False
        self.pending = list(batch)
        self.stops = stops
        plan = " -> ".join("%s(%s)" % (s.kind, ",".join(str(o.id) for o in s.orders))
                           for s in self.stops)
        self.log(t, "plan", batch=batch[0].batch, detail=plan)
        for s in self.stops:
            if s.kind == DRONE:
                self.log(t, "hub_chosen", batch=batch[0].batch, pos=s.location,
                         detail="drops=%s centroid=(%.1f, %.1f)" % (
                             [o.id for o in s.drone_order],
                             *planning.centroid([o.drop for o in s.orders])[:2]))
        self.drive_to_next(t)
        return True

    CHARGE_KIND = "lmp"          # charging-station bay 03 (LastMile Hubs: bay 04) / ベイ 03

    def _yard_of(self, idx):
        """The scripted-move area of charger ``idx``: the map's yard, a
        charging-station bay, or None (a road-side charger). / 経路の場所。"""
        if self.sky.yard:
            return self.sky.yard
        if idx is None:
            return None
        return (getattr(self.sky, "station_bays", None) or {}).get(idx)

    def go_charge(self, t, why):
        """Drive to a free EV charger (or wait for one). / EV 充電器へ向かう。

        With a charging yard on this map, or at a charging station, the pod
        drives to the entry and the bay is reached by a scripted move
        (yard_move).
        """
        yard = self.sky.yard
        idx = self.sky.chargers.claim(self, near=None if yard else self.car_pos(), kind=self.CHARGE_KIND)
        if idx is None and self._go_queue(t, why):
            return
        if idx is None:
            if self.state != WAIT_CHARGER:
                self.log(t, "waiting_for_charger", pos=self.car_pos(),
                         detail="pod at %.0f%%: every charger is taken" % self.energy.level)
            self.state, self._wait_t = WAIT_CHARGER, 0.0
            return
        yard = self._yard_of(idx)
        self.yard_bay = idx
        if yard:
            ex, ey = yard["entry"]
            spot = (ex, ey, 0.0)
            where = yard["name"] if yard.get("kind") == "station" else "bay %d in the %s" % (idx + 1, yard["name"])
        else:
            spot = self.sky.chargers.spots[idx]
            where = "charger %d" % (idx + 1)
        self.stops = [Stop(CHARGE, spot, [])]
        self.log(t, "to_charger", pos=spot,
                 detail="pod at %.0f%% (%s), %s" % (self.energy.level, why, where))
        self.drive_to_next(t)

    def _go_queue(self, t, why):
        """Every bay of this vehicle's kind is taken: wait in the nearest free
        waiting bay of a charging station (off the lane) instead of where it
        is. False when there is none. / 空きベイが無い：最寄りの待機ベイへ。"""
        slots = getattr(self.sky, "queue_slot", None)
        if not slots:
            return False
        here = self.car_pos()
        free = [sid for sid, v in slots.items() if v is None]
        if not free:
            return False
        sid = min(free, key=lambda k: planning.dist2d(here, tuple(self.sky.station_bays[("q", k)]["entry"]) + (0.0,)))
        slots[sid] = self.vid
        q = self.sky.station_bays[("q", sid)]
        self.yard_bay, self._next_bay, self._queue_t = ("q", sid), None, t
        ex, ey = q["entry"]
        self.stops = [Stop(CHARGE, (ex, ey, 0.0), [])]
        self.log(t, "to_waiting_bay", pos=(ex, ey, 0.0),
                 detail="pod at %.0f%% (%s): every %s bay taken, waiting at %s"
                 % (self.energy.level, why, "hub" if self.CHARGE_KIND == "lmh" else "LMP-4", q["name"]))
        self.drive_to_next(t)
        return True

    def charger_kw(self):
        """Power of the charger it is plugged into: a charging-station bay
        (DC) or a road-side / yard charger (AC). / 充電器の電力。"""
        b = self._yard_of(self.yard_bay)
        return self.args.station_kw if b is not None and b.get("kind") == "station" else self.args.roadside_kw

    def _plugged(self):
        """At a charging-station bay: only once the plug unit is in. / プラグ接続済みか。"""
        b = self._yard_of(self.yard_bay)
        st = getattr(self.sky, "stations", None)
        if b is None or b.get("kind") != "station" or st is None:
            return True
        return st.sites[b["station"] - 1]["plug_u"].get(b["bay_kind"], 0.0) >= 0.999

    def _lane_clear(self, t, yard):
        """No vehicle or walker in the way of a scripted move at a station:
        nothing within the turning circle of the entry or along the path to
        the bay, and no vehicle closing in along the lane within 30 m.
        ステーションでの移動前に、周囲と車線の接近車両を確認。"""
        sky = self.sky
        cache = sky.__dict__.setdefault("_traffic", {"t": -1e9, "ids": []})
        if t - cache["t"] > 2.0:
            acts = self.world.get_actors()
            cache["ids"] = [a.id for a in acts.filter("vehicle.*")] + [a.id for a in acts.filter("walker.pedestrian.*")]
            cache["t"] = t
        snap = self.world.get_snapshot()
        (ex, ey), (bx, by) = yard["entry"], yard["bay"]
        r = math.radians(yard.get("entry_yaw", 0.0))
        fx, fy = math.cos(r), math.sin(r)
        turn_r = getattr(sky.stations, "turn_r", 2.8) if getattr(sky, "stations", None) else 2.8
        for aid in cache["ids"]:
            if aid == self.car.id:
                continue
            a = snap.find(aid)
            if a is None:
                continue
            L = a.get_transform().location
            dx, dy = L.x - ex, L.y - ey
            if math.hypot(dx, dy) < turn_r + 1.5:
                return False
            # along the path to the bay (walkers on the pavement) / ベイへの経路上
            px, py = bx - ex, by - ey
            n = px * px + py * py or 1.0
            u = max(0.0, min(1.0, (dx * px + dy * py) / n))
            if math.hypot(L.x - (ex + px * u), L.y - (ey + py * u)) < 1.8:
                return False
            v = a.get_velocity()
            along, lat = dx * fx + dy * fy, abs(-dx * fy + dy * fx)
            closing = (along < 0 and v.x * fx + v.y * fy > 0.5) or (along > 0 and v.x * fx + v.y * fy < -0.5)
            if lat < 6.0 and abs(along) < 30.0 and closing:
                return False
        return True

    def give_back(self, t, why):
        """Hand the orders not yet delivered back to the dispatcher. / 注文を返却。"""
        rest = [o for o in self.pending if o not in self.failed]
        if rest and self.fleet is not None:
            self.fleet.dispatcher.push_front(rest)
            self.log(t, "orders_returned", batch=rest[0].batch,
                     detail="%s: #%s back in the queue" % (why, ",".join(str(o.id) for o in rest)))
        self.pending, self.stops = [], []

    def drive_to_next(self, t):
        s = self.stops[0]
        self._stop_recoveries = 0
        # route from where the van really is, not the planner's last target
        # 前回の目標点ではなく、車の実際の位置から経路を引く
        self.agent.set_destination(loc(s.location), start_location=self.car.get_location())
        self.state = DRIVING
        self.log(t, "drive", batch=s.batch, pos=s.location,
                 detail="to %s stop" % s.kind)

    def _detour_to(self, target):
        """Route to ``target`` by way of a point further along the road. The
        planner finds no route to a spot just behind the van in its own lane
        (it would have to drive backwards), so go on and round the block.
        True when such a route was set.
        同じ車線のすぐ後ろの地点へは経路が引けないため、先の地点を経由する。"""
        here = self.wmap.get_waypoint(self.car.get_location())
        goal = self.wmap.get_waypoint(carla.Location(*target))
        if here is None or goal is None:
            return False
        for ahead in (30.0, 60.0, 120.0):
            for via in here.next(ahead):
                try:
                    plan = self.agent.trace_route(here, via)
                    back = self.agent.trace_route(via, goal)
                except (RuntimeError, ValueError, KeyError):
                    continue
                # a route back along the same lane comes out empty or stops
                # short of the goal / 同一車線を戻る経路は空か手前で終わる
                if plan and back and back[-1][0].transform.location.distance(goal.transform.location) < 5.0:
                    self.agent.set_global_plan(plan + back[1:])
                    return True
        return False

    def dock_step(self, t, dt):
        """Creep along the kerb lane until the side port is level with the box
        station (the stop's location is where the pod's centre must be).
        箱型ステーションにサイドポートが並ぶまで低速で前後に寄せる。"""
        self.timer += dt
        tf = self.car.get_transform()
        f, p = tf.get_forward_vector(), tf.location
        goal = self.stops[0].location
        e = (goal[0] - p.x) * f.x + (goal[1] - p.y) * f.y            # along the heading / 進行方向の残り
        v = self.car.get_velocity()
        speed = v.x * f.x + v.y * f.y
        if (abs(e) < 0.15 and abs(speed) < 0.15) or self.timer > 25.0:
            self.arrive(t)
            return
        want = max(-0.8, min(1.4, 0.6 * e))                         # m/s / 目標速度
        # keep on the parking lane's centre line: heading error plus the
        # sideways offset (Stanley), so the box ends where the arm expects it
        # 駐車車線の中心線に沿う（向きの誤差＋横ずれ）
        lane = self.wmap.get_waypoint(carla.Location(goal[0], goal[1], goal[2]))
        lyaw = math.radians(lane.transform.rotation.yaw)
        head = math.atan2(math.sin(lyaw - math.radians(tf.rotation.yaw)), math.cos(lyaw - math.radians(tf.rotation.yaw)))
        lp = lane.transform.location
        side = (p.x - lp.x) * math.sin(lyaw) - (p.y - lp.y) * math.cos(lyaw)   # >0: left of the line / 線の左
        steer = head + math.atan2(1.5 * side, abs(speed) + 0.5)
        if want < 0:
            steer = -head                                           # backing up: only straighten / 後退時
        c = carla.VehicleControl(steer=max(-1.0, min(1.0, steer / 0.7)), reverse=want < 0)
        if abs(speed) < abs(want) - 0.1:
            c.throttle = 0.35
        elif abs(speed) > abs(want) + 0.1 or abs(e) < 0.15:
            c.brake = 0.4
        self.car.apply_control(c)

    def arrive(self, t):
        """Plan what happens at this stop; Van.step works through it.

        Pod: come to a full stop, lower the jacks, then hand over (face to face)
        or open the roof hatch, fly every drop and close it again (drone),
        raise the jacks and drive on. A stock van skips the jacks and hatch.
        ポッド：完全停止 → ジャッキを下ろす → 対面受け渡し、またはハッチを開けて
        全投下を飛行し閉じる → ジャッキを上げて出発。標準の車はジャッキ・ハッチ無し。
        """
        s = self.stops[0]
        self.hold()
        self.log(t, "arrive", batch=s.batch, pos=self.car_pos(),
                 detail="%s stop" % s.kind)
        if s.kind == CHARGE:
            work = [EV_CHARGING]
        elif self.lmp:
            work = [LMP_DRONE if s.kind == DRONE else LMP_DIRECT]
        elif s.kind == FACE_TO_FACE and self.pod is not None and self.pod.side_port:
            work = [SIDE_OPEN, ARM_OUT, PARCEL_DROP, ARM_IN, SIDE_CLOSE]
        else:
            work = [HANDOVER] if s.kind == FACE_TO_FACE else [DRONE_OUT]
        if self.pod is not None and s.kind == DRONE and not self.lmp:
            work = [HATCH_OPEN] + work + [HATCH_CLOSE]
        if self.pod is not None:
            work = [JACKS_DOWN] + work + [JACKS_UP]
        q = self._yard_of(self.yard_bay) if s.kind == CHARGE else None
        if q is not None and q.get("kind") == "queue":
            work = [YARD_IN, QUEUED, YARD_OUT]     # wait off the lane / 車線外で待つ
        elif s.kind == CHARGE and q:
            work = [YARD_IN] + work + [YARD_OUT]
        self.stages = [STOPPING] + work
        self._begin(t, STOPPING)

    def _begin(self, t, stage):
        self.state, self.timer = stage, 0.0
        if stage == LMP_DRONE:
            self.lmp_begin_drone(t)
        elif stage == LMP_DIRECT:
            self.lmp_begin_direct(t)
        elif stage == HANDOVER:
            self.timer = self.args.handover_s
        elif stage == DRONE_OUT:
            s = self.stops[0]
            self.drone_queue = list(s.drone_order)
            self.current, self.reload_t, self.drone_wait = None, 0.0, 0.0
            self._said = set()
            self.log(t, "drone_stop", batch=s.batch, pos=self.roof(),
                     detail="%d drops, one round trip each; drone %s" % (
                         len(self.drone_queue),
                         "D%d at %.0f%%" % (self.drone.id, self.drone.battery.level)
                         if self.drone else "none (requesting one)"))
        elif stage in (JACKS_DOWN, JACKS_UP, HATCH_OPEN, HATCH_CLOSE):
            self.log(t, stage, batch=self.stops[0].batch)
        elif stage in (YARD_IN, YARD_OUT):
            self._mover = None
        elif stage == SIDE_OPEN:
            self.handover_queue = list(self.stops[0].orders)
            self.log(t, stage, batch=self.stops[0].batch)
        elif stage == ARM_OUT:
            self.pod.load_parcel()
        elif stage == PARCEL_DROP:
            self.drop_parcel(t)
        elif stage == EV_CHARGING:
            self.charge_kwh = 0.0
            b = self._yard_of(self.yard_bay)
            if b is not None and b.get("kind") == "station" and b.get("bay_kind") == "lmh" \
                    and hasattr(self, "station_pods_begin"):
                self.station_pods_begin(t, self.sky.stations, b["station"])    # Mini Pods to the pads / パッドへ
            self.log(t, "ev_charging", pos=self.car_pos(),
                     detail="pod at %.0f%%, charging to %.0f%%"
                     % (self.energy.level, self.pem.target))

    def _next_stage(self, t):
        self.stages.pop(0)
        if self.stages:
            self._begin(t, self.stages[0])
        else:
            self.leave_stop(t)

    def _say(self, t, key, event, **kw):
        """Log ``event`` once per drone stop. / 1 地点につき 1 回だけ記録。"""
        if key not in self._said:
            self._said.add(key)
            self.log(t, event, **kw)

    def trip_plan(self, order):
        """(hover point, cruise z, metres moved off the drop) for ``order``.

        The hover point is None when there is no clear air near the drop.
        """
        if order.id not in self._plans:
            home, sky = self.dock_pos(), self.sky.check
            z0 = sky.flight_z(home, order.drop)
            hover, moved = sky.hover_point(order.drop, order.building.curb, z0)
            z = sky.flight_z(home, hover) if hover is not None else z0
            self._plans[order.id] = (hover, z, moved)
        return self._plans[order.id]

    def trip_need(self, order, unit=None):
        """Battery % the next trip needs, reserve included. / 次の往復に必要な %。"""
        hover, z, _ = self.trip_plan(order)
        bm = self.sky.bm
        u = unit or self.drone
        h, v = (u.h_speed, u.v_speed) if u else (self.args.drone_speed, self.args.climb_speed)
        return bm.trip_cost(self.dock_pos(), hover, z, h, v, self.args.hover_s, 3.0,
                            self.sky.drones.plan_wind()) + bm.reserve

    def drone_work(self, t, dt):
        """The drone stop: one round trip per order, the battery checked first.

        Enough charge: fly. A little short: trickle-charge in the van for up
        to --max-charge-wait. Too far short: the drone flies to the charge
        station and a standby is sent here. No drone at all for
        --drone-wait-max: the remaining orders fail.
        十分なら飛行。少し不足なら車内で最大 --max-charge-wait 充電。大きく
        不足ならステーションへ飛ばし待機機を呼ぶ。機体が来ないまま
        --drone-wait-max 経過で残りの注文は失敗。
        """
        self.hold()
        u = self.drone
        if u is not None and u.status == DELIVERING:
            return                      # events arrive through on_drone_event
        if u is not None and u.status == RELOADING:
            self.reload_t -= dt
            if self.reload_t > 0.0:
                return
            u.status, u.note = DOCKED, ""
        if not self.drone_queue:
            self._end_drone_work(t)
            self._next_stage(t)
            return
        order = self.drone_queue[0]
        if self.trip_plan(order)[0] is None:
            # no clear air near the drop (arcade, deep overhang): refuse it
            # rather than fly into the building / 安全な投下空間が無い：飛ばずに失敗
            self.fail_order(t, order, "no_safe_drop")
            return
        need = self.trip_need(order)
        batch = order.batch
        if u is None:
            self.drone_wait += dt
            if self.incoming is None:
                self.sky.drones.request(self, need)
                self._say(t, "req", "drone_requested", batch=batch, pos=self.roof(),
                          detail="needs a drone with %.0f%% + the flight here" % need)
            if self.drone_wait > self.args.drone_wait_max:
                for o in list(self.drone_queue):
                    self.fail_order(t, o, "no_drone")
                self.drone_queue = []
            return
        if self.sky.drones.grounded:
            u.note = "grounded: " + self.sky.drones.ground_reason()
            self._say(t, "hold", "rain_hold" if self.sky.drones.raining else "wind_hold",
                      batch=batch, pos=self.roof(),
                      detail="%s: D%d waits in the pod" % (self.sky.drones.ground_reason(), u.id))
            return
        self._said.discard("hold")
        if u.battery.level >= need:
            hover, z, moved = self.trip_plan(order)
            u.start_trip(self, order, hover, z)
            self.current = order
            self.drone_queue.pop(0)
            self.drone_wait = 0.0
            self._said.discard("charge")
            where = (", hovering %.1f m off the drop (overhang)" % moved) if moved > 0.1 else ""
            self.log(t, "drone_launch", order, pos=self.roof(),
                     detail="D%d at %.0f%% (trip needs ~%.0f%%), cruise %.0f m above the van%s"
                     % (u.id, u.battery.level, need - self.sky.bm.reserve,
                        z - self.roof()[2], where))
            return
        wait = self.sky.bm.wait_to_reach(u.battery.level, need, self.sky.bm.slow)
        if wait <= self.args.max_charge_wait:
            u.note = "charging in van to %.0f%%" % need
            self._say(t, "charge", "drone_charging_in_van", batch=batch, pos=self.roof(),
                      detail="D%d at %.0f%%, next trip needs %.0f%% (~%.0f s)"
                      % (u.id, u.battery.level, need, wait))
            return
        if self.sky.drones.send_to_station(u, t):
            self.sky.drones.request(self, need)
        else:
            u.note = "charging in van (can't reach the station)"
            self._say(t, "stuck", "drone_charging_in_van", batch=batch, pos=self.roof(),
                      detail="D%d at %.0f%% can't reach the station; charging here"
                      % (u.id, u.battery.level))

    def _end_drone_work(self, t):
        self.sky.drones.cancel(self)
        if self.incoming is not None:
            self.sky.drones.recall(self.incoming, t)
            self.incoming = None

    def on_drone_event(self, t, unit, was, ev):
        """Something happened to a drone working for this van. / 機体のイベント。"""
        kind, data = ev
        if self.lmp:
            # several drones out at once: each order travels with its drone
            # 複数機が同時に飛ぶため、注文は機体ごとに追う
            if kind == "delivered":
                self.finish_order(t, data, data.drop)
            elif kind == "parcel_lost":
                self.fail_order(t, data, "parcel_lost")
            elif kind == "crashed":
                order = self.in_air.pop(unit, None)
                if order is not None and order in self.pending:
                    self.fail_order(t, order, data)
                self.lmp_on_drone_event(t, unit, kind)
                self.release_drone(unit)
            else:
                if kind == "trip_done":
                    self.in_air.pop(unit, None)
                self.lmp_on_drone_event(t, unit, kind)
            return
        if kind == "delivered":
            self.finish_order(t, data, data.drop)
        elif kind == "parcel_lost":
            self.fail_order(t, data, "parcel_lost")
        elif kind == "trip_done":
            self.current = None
            if self.lmp:
                return
            self.log(t, "drone_back", pos=self.roof(),
                     detail="D%d at %.0f%%" % (unit.id, unit.battery.level))
            if self.drone_queue:
                self.reload_t = self.args.reload_s
                unit.status, unit.note = RELOADING, "loading the next parcel"
        elif kind == "crashed":
            if was == DELIVERING and self.current is not None and self.current in self.pending:
                self.fail_order(t, self.current, data)
            self.current = None
            self.release_drone(unit)
            if self.incoming is unit:
                self.incoming = None

    def fail_order(self, t, order, reason):
        order.failed_t, order.fail_reason = t, reason
        if order in self.pending:
            self.pending.remove(order)
        if order in self.drone_queue:
            self.drone_queue.remove(order)
        self.failed.append(order)
        self.gen.release(order)
        self.log(t, "failed", order, pos=order.drop, detail=reason)

    def finish_order(self, t, order, pos):
        order.delivered_t = t
        self.pending.remove(order)
        self.delivered.append(order)
        self.gen.release(order)
        self.log(t, "delivered", order, pos=pos,
                 detail="wait=%.1fs" % (t - order.created_t))

    def leave_stop(self, t):
        done = self.stops.pop(0)
        if done.kind == CHARGE and isinstance(self.yard_bay, tuple):
            # out of the waiting bay: on to the bay it claimed / 待機ベイから確保したベイへ
            slots = getattr(self.sky, "queue_slot", {})
            if slots.get(self.yard_bay[1]) == self.vid:
                slots[self.yard_bay[1]] = None
            self.yard_bay, self._queue_t = self._next_bay, None
            b = self._yard_of(self.yard_bay)
            ex, ey = b["entry"] if b else self.sky.chargers.spots[self.yard_bay][:2]
            self.stops = [Stop(CHARGE, (ex, ey, 0.0), [])]
            self.log(t, "to_charger", pos=(ex, ey, 0.0), detail="from the waiting bay to %s"
                     % (b["name"] if b else "charger %d" % (self.yard_bay + 1)))
            self.drive_to_next(t)
            return
        if done.kind == CHARGE:
            self.sky.chargers.release(self)
            self.ev_charges += 1
            self.log(t, "ev_charged", pos=self.car_pos(),
                     detail="pod at %.0f%%" % self.energy.level)
            self.stops, self.state = [], IDLE
            return
        if self.stops:
            self.drive_to_next(t)
        else:
            self.batches += 1
            self.log(t, "batch_done", batch=done.orders[0].batch)
            self.state = IDLE

    # -- per-tick
    def energy_step(self, t, dt):
        """Drain for distance driven and onboard systems; strand at 0 %."""
        p = self.car_pos()
        if self._last_pos is not None:
            d = planning.dist2d(p, self._last_pos)
            if d < 20.0:            # not a teleport (guard reset, tow) / 瞬間移動は除く
                self.pod_km += d / 1000.0
                self.energy.level = max(self.energy.level - self.pem.per_km * d / 1000.0, 0.0)
        self._last_pos = p
        if self.state == EV_CHARGING:
            if self.args.pod_charge_rate > 0:            # the old fixed rate / 従来の固定速度
                rate = self.pem.charge
            else:                                         # from charger power and pack size / 電力と容量から
                rate = charge_rate(self.energy.level, self.charger_kw(), self.pack_kwh) if self._plugged() else 0.0
            before = self.energy.level
            self.energy.charge(dt, rate)
            got = (self.energy.level - before) / 100.0 * self.pack_kwh
            self.charge_kw = got / dt * 3600.0 if dt > 0 else 0.0
            self.charge_kwh += got
            b = self._yard_of(self.yard_bay)
            if b is not None and b.get("kind") == "station":
                tot = self.sky.__dict__.setdefault("station_kwh", {})
                tot[b["station"]] = tot.get(b["station"], 0.0) + got
        else:
            self.charge_kw = 0.0
            self.energy.drain(dt, self.pem.idle)
        if self.lmp:
            self.pack_step(dt)
        if self.energy.dead and self.state not in (STRANDED, EV_CHARGING):
            self.strand(t)

    def tow(self, t):
        """A flat pod is towed to a charger (into a yard bay, if there is a
        yard) and charged there. / 電池切れのポッドを充電器まで牽引。"""
        yard = self.sky.yard
        idx = self.sky.chargers.claim(self, near=None if yard else self.car_pos(), kind=self.CHARGE_KIND)
        if idx is None:
            return                              # every charger busy: try again later
        yard = self._yard_of(idx)
        if self.lmp:
            self.lmp_reset()                    # roof shut, lift down / 全て格納
        elif self.pod is not None:
            self.pod.pose(jacks=0.0, hatch=0.0, side=0.0, arm=0.0)
            self.pod.release_parcel()
        spot = self.sky.chargers.spots[idx]
        self._last_pos = None
        self.yard_bay = idx
        if yard:
            bx, by = yard["bay"] if yard.get("kind") == "station" else spot[:2]
            self._ref_z = yard.get("z")
            self.car.set_simulate_physics(False)
            self.kinematic = True
            self.car.set_transform(carla.Transform(
                carla.Location(bx, by, self._ground(bx, by) + 0.04),
                carla.Rotation(yaw=yard.get("bay_yaw", 0.0))))
        else:
            self._teleport(self.wmap.get_waypoint(carla.Location(*spot)).transform)
        self.log(t, "pod_towed", pos=spot, detail="to charger %d" % (idx + 1))
        ex, ey = yard["entry"] if yard else spot[:2]
        self.stops = [Stop(CHARGE, (ex, ey, 0.0), [])]
        work = [EV_CHARGING]
        if self.pod is not None:
            work = [JACKS_DOWN] + work + [JACKS_UP]
        self.stages = work + ([YARD_OUT] if yard else [])
        self._begin(t, self.stages[0])

    def drop_parcel(self, t):
        """Let go of the parcel on the arm's tray; it falls to the ground.

        CARLA can't detach the tray's parcel, so a free one is spawned where it
        was and dropped (parcel_fall). The order is delivered as it lets go.
        トレイの荷物を離して地面へ落とす（取り外せないため同じ位置に新たに生成）。
        """
        x, y, z = self.pod.release_parcel()
        yaw = self.car.get_transform().rotation.yaw
        box = self.world.try_spawn_actor(
            self.world.get_blueprint_library().find(podlib.PARCEL_PROP),
            carla.Transform(carla.Location(x, y, z), carla.Rotation(yaw=yaw)))
        ground = self._ground(x, y)
        self._fall = [box, x, y, z, 0.0, ground, yaw]
        if self.handover_queue:
            o = self.handover_queue.pop(0)
            self.finish_order(t, o, (x, y, ground))
        if box is not None:
            self.dropped.append((box, t + PARCEL_LINGER_S))

    def parcel_fall(self, dt):
        """Advance the falling parcel; True once it is on the ground."""
        if self._fall is None or self._fall[0] is None:
            return True
        box, x, y, z, v, ground, yaw = self._fall
        v += 9.81 * dt
        z = max(z - v * dt, ground)
        box.set_transform(carla.Transform(carla.Location(x, y, z), carla.Rotation(yaw=yaw)))
        self._fall[3:5] = [z, v]
        if z <= ground:
            self._fall = None
            return True
        return False

    def collect_parcels(self, t):
        """Parcels left on the ground go after PARCEL_LINGER_S. / 一定時間後に消す。"""
        for box, due in [d for d in self.dropped if d[1] <= t]:
            box.destroy()
        self.dropped = [d for d in self.dropped if d[1] > t]

    def cleanup(self):
        for box, _ in self.dropped:
            box.destroy()
        self.dropped = []
        if self.lmp:                    # the box station and the customer / 箱と客
            m = self.pod.mech
            m.station, m.person = False, None
            self.pod.apply(m)

    def _ground(self, x, y):
        """Road / sidewalk height at (x, y), cached on a 0.5 m grid. The ray
        runs from 2 m over the car's own height to 3 m under it (hilly maps:
        the ground is not near z 0; 2 m keeps a charging-station canopy out).
        路面の高さ（車の高さ基準。丘陵地図では地面が z 0 付近に無い）。"""
        key = (round(x * 2.0), round(y * 2.0))
        if key not in self._ground_z:
            ref = self._ref_z if self._ref_z is not None else self.car_pos()[2]
            z = ref
            for h in self.world.cast_ray(carla.Location(x, y, ref + 2.0), carla.Location(x, y, ref - 3.0)):
                if str(h.label) in GROUND_LABELS:
                    z = h.location.z
                    break
            self._ground_z[key] = z
        return self._ground_z[key]

    def yard_move(self, t, dt):
        """Follow the scripted path into or out of the charging yard.

        One pod moves in the yard at a time (``sky.yard_lane``); the others
        wait. Physics is off while the car follows the path and comes back on
        at the road. / ヤードの決まった経路を移動（同時に 1 台、移動中は物理なし）。
        """
        yard = self._yard_of(self.yard_bay)
        lanes = self.sky.__dict__.setdefault("yard_lanes", {})
        key = yard.get("station", "yard")              # one vehicle moving per yard / station
        if self._mover is None:
            if lanes.get(key) not in (None, self.vid):
                self.hold()
                return                          # someone else is moving in the yard
            if yard.get("kind") in ("station", "queue") and not self._lane_clear(t, yard):
                self.hold()
                self._yield_t += dt
                if self._yield_t < 40.0:
                    return                      # wait for a gap in the traffic / 交通の切れ目を待つ
                self.log(t, "yield_timeout", pos=self.car_pos(), detail="no gap for 40 s: going anyway")
            self._yield_t = 0.0
            lanes[key] = self.vid
            self._ref_z = yard.get("z")
            self._ground_z = {}
            if yard.get("kind") == "station":
                tf = self.car.get_transform()
                pts = (charge_sites.path_in(yard, (tf.location.x, tf.location.y, tf.rotation.yaw))
                       if self.state == YARD_IN else charge_sites.path_out(yard))
            elif yard.get("kind") == "queue":
                tf = self.car.get_transform()
                pts = (charge_sites.queue_path_in(yard, (tf.location.x, tf.location.y, tf.rotation.yaw))
                       if self.state == YARD_IN else charge_sites.queue_path_out(yard))
            elif self.state == YARD_IN:
                pts = yardlib.path_in(yard, self.yard_bay, self.car_pos()[:2])
            else:
                pts = yardlib.path_out(yard, self.yard_bay)
            self._mover = yardlib.Mover(pts)
            self.car.set_simulate_physics(False)
            self.kinematic = True
            self.log(t, self.state, pos=self.car_pos(),
                     detail="%s, %.0f m" % (yard["name"] if yard.get("kind") in ("station", "queue")
                                            else "bay %d" % (self.yard_bay + 1), len(pts) / 10.0))
        (x, y, yaw), done = self._mover.step(dt)
        self.car.set_transform(carla.Transform(carla.Location(x, y, self._ground(x, y) + 0.04),
                                               carla.Rotation(yaw=yaw)))
        if done:
            self._mover = None
            lanes[key] = None
            if self.state == YARD_OUT:
                self.car.set_simulate_physics(True)
                self.kinematic = False
                self.car.set_target_velocity(carla.Vector3D())
            self._next_stage(t)

    def strand(self, t):
        """Flat battery: the pod stops where it is; its orders fail; it is towed
        to a charger after --pod-recover-s. / 電池切れ：停止し注文は失敗、牽引待ち。"""
        self.log(t, "pod_stranded", pos=self.car_pos(), detail="battery empty")
        # a parcel already on a flying drone can still be delivered
        # 飛行中のドローンが運んでいる荷物はまだ配達できる
        flying = self.current if (self.drone is not None and
                                  self.drone.status == DELIVERING) else None
        for o in list(self.pending):
            if o is not flying:
                self.fail_order(t, o, "pod_battery_dead")
        self._end_drone_work(t)
        self.drone_queue, self.stops, self.stages = [], [], []
        self.sky.chargers.release(self)
        self.state, self._strand_t = STRANDED, 0.0

    def step(self, t, dt):
        self.energy_step(t, dt)
        if self.lmp:
            self.lmp_tick(t, dt)            # tracks, drone prep, pose / 機構・準備・姿勢
        if self.dropped:
            self.collect_parcels(t)
        if self.state == IDLE:
            self.hold()
            if self.energy.level < self.pem.low or self.charge_asap:
                why = "sent to charge" if self.charge_asap else "low"
                self.charge_asap = False
                self.go_charge(t, why)
        elif self.state == WAIT_CHARGER:
            self.hold()
            self._wait_t += dt
            if self._wait_t > 1.0:
                self.go_charge(t, "low")
        elif self.state == QUEUED:
            self.hold()
            self.timer += dt
            if self.timer > 1.0:                          # a bay free (here or elsewhere)? / 空きベイは？
                self.timer = 0.0
                idx = self.sky.chargers.claim(self, near=self.car_pos(), kind=self.CHARGE_KIND)
                if idx is not None:
                    self._next_bay = idx
                    self.log(t, "bay_free", pos=self.car_pos(), detail="leaving the waiting bay for %s"
                             % (self._yard_of(idx) or {}).get("name", "charger %d" % (idx + 1)))
                    self._next_stage(t)
        elif self.state == STRANDED:
            self.hold()
            self._strand_t += dt
            if self._strand_t > self.args.pod_recover_s:
                self.tow(t)
        elif self.state == EV_CHARGING:
            self.hold()
            bay = self._yard_of(self.yard_bay)
            if bay is not None and bay.get("kind") == "station" and getattr(self.sky, "stations", None):
                # the bay's plug unit pushes into the front inlet, pulls back
                # out once charged (charge.js) / プラグの接続と切断
                self.timer += dt
                pods = hasattr(self, "station_pods_step") and bay["bay_kind"] == "lmh"
                if pods:
                    self.station_pods_step(t, dt)
                if self._unplug_t is None:
                    self.sky.stations.plug(bay["station"], bay["bay_kind"], self.timer / charge_sites.PLUG_S)
                    if self.energy.level >= self.pem.target and self.timer >= charge_sites.PLUG_S:
                        if pods:
                            self.station_pods_recall(t)
                        if not pods or self.station_pods_done(t):
                            self._unplug_t = self.timer
                else:
                    u = 1.0 - (self.timer - self._unplug_t) / charge_sites.PLUG_S
                    self.sky.stations.plug(bay["station"], bay["bay_kind"], u)
                    if u <= 0.0:
                        self._unplug_t = None
                        self._next_stage(t)
            elif self.energy.level >= self.pem.target:
                self._next_stage(t)
        elif self.state in (YARD_IN, YARD_OUT):
            self.yard_move(t, dt)
        elif self.state in (SIDE_OPEN, SIDE_CLOSE, ARM_OUT, ARM_IN):
            self.hold()
            door = self.state in (SIDE_OPEN, SIDE_CLOSE)
            self.timer += dt
            u = self.timer / max(self.args.side_door_s if door else self.args.arm_s, 1e-3)
            u = min(u, 1.0) if self.state in (SIDE_OPEN, ARM_OUT) else max(1.0 - u, 0.0)
            self.pod.pose(**{"side" if door else "arm": u})
            if u in (0.0, 1.0):
                if self.state == ARM_IN and self.handover_queue:
                    # more parcels for this stop: out again / この地点の次の荷物
                    self.stages[1:1] = [ARM_OUT, PARCEL_DROP, ARM_IN]
                self._next_stage(t)
        elif self.state == PARCEL_DROP:
            self.hold()
            if self.parcel_fall(dt):
                self.timer += dt
                if self.timer > 0.6:            # a beat before pulling the arm in
                    self._next_stage(t)
        elif self.state == DRIVING:
            s = self.stops[0]
            if s.kind != CHARGE and self.energy.level < self.pem.critical:
                self.give_back(t, "pod battery at %.0f%%" % self.energy.level)
                self.go_charge(t, "critical")
                return
            near = planning.dist2d(self.car_pos(), s.location) < self.args.arrive_radius
            box_stop = (self.lmp and s.kind == FACE_TO_FACE and s.orders
                        and getattr(getattr(s.orders[0], "building", None), "box", None) is not None)
            if box_stop and planning.dist2d(self.car_pos(), s.location) < 12.0:
                self.state, self.timer = DOCKING, 0.0
            elif box_stop and self.agent.done():
                # the route ran out short of the box: docking from here would
                # stretch the arm across the street. Route again; after a few
                # tries put the pod beside the box, logged, like a stuck van.
                # 箱の手前で経路が尽きた。再経路、数回失敗したら箱の横へ移す
                self._stop_recoveries += 1
                if self._stop_recoveries > MAX_RECOVERIES:
                    tf = self.wmap.get_waypoint(carla.Location(*s.location)).transform
                    self._teleport(tf)
                    self.log(t, "gave_up_driving", batch=s.batch, pos=tup(tf.location),
                             detail="no route to the box station")
                    self.state, self.timer = DOCKING, 0.0
                else:
                    self.reroutes += 1
                    self.log(t, "reroute", batch=s.batch, pos=self.car_pos(),
                             detail="route ended %.0f m short of the box station"
                             % planning.dist2d(self.car_pos(), s.location))
                    if not self._detour_to(s.location):
                        self.agent.set_destination(carla.Location(*s.location),
                                                   start_location=self.car.get_location())
            elif near or self.agent.done():
                self.arrive(t)
            elif getattr(self, "_reverse", None) is not None:
                # backing out of a wedge, then the route is re-planned / 後退して再計画
                tf = self.car.get_transform()
                yaw, lat, _ = recovery._lane_frame(self.wmap, tf.location, carla)
                out = self._reverse.step(dt, tf.location.x, tf.location.y, tf.rotation.yaw, yaw, lat)
                if out is None:
                    self._reverse = None
                    self.agent.set_destination(carla.Location(*self.stops[0].location),
                                               start_location=self.car.get_location())
                else:
                    thr, brk, st, rev = out
                    self.car.apply_control(carla.VehicleControl(throttle=thr, brake=brk, steer=st, reverse=rev))
            else:
                control = self.agent.run_step()
                if self.rules is not None:
                    # 止まれ / walkers / lane speed (Japan profile) / 日本の交通ルール
                    control = self.rules.apply(self.car.id, self.agent, self.args.speed, control)
                self.car.apply_control(control)
                self.guard(t, dt, control)
        elif self.state == DOCKING:
            self.dock_step(t, dt)
        elif self.state == STOPPING:
            # the jacks only come down once the van is at a full stop
            # ジャッキは完全に停止してから下ろす
            self.hold()
            self.timer += dt
            v = self.car.get_velocity()
            if math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z) < 0.05 or self.timer > 5.0:
                self._next_stage(t)
        elif self.state in (JACKS_DOWN, JACKS_UP, HATCH_OPEN, HATCH_CLOSE):
            self.hold()
            jacks = self.state in (JACKS_DOWN, JACKS_UP)
            self.timer += dt
            u = self.timer / max(self.args.jack_s if jacks else self.args.hatch_s, 1e-3)
            u = min(u, 1.0) if self.state in (JACKS_DOWN, HATCH_OPEN) else max(1.0 - u, 0.0)
            self.pod.pose(**{"jacks" if jacks else "hatch": u})
            if u in (0.0, 1.0):
                self._next_stage(t)
        elif self.state == HANDOVER:
            self.hold()
            self.timer -= dt
            if self.timer <= 0.0:
                for o in list(self.stops[0].orders):
                    self.finish_order(t, o, o.building.curb)
                self._next_stage(t)
        elif self.state == DRONE_OUT:
            self.drone_work(t, dt)
        elif self.state == LMP_DRONE:
            self.lmp_drone_work(t, dt)
        elif self.state == LMP_DIRECT:
            self.lmp_direct_work(t, dt)

    def guard(self, t, dt, control):
        """Catch the van leaving the road or wedging itself, and recover.

        First a re-plan from where the van is; if it trips again within
        RETRY_WINDOW_S the van is moved (onto the lane, or ahead along its
        route). After MAX_RECOVERIES at one stop it is placed at the stop.
        車の路外逸脱や立ち往生を検知して回復する。まず現在地から再計画し、
        RETRY_WINDOW_S 内に再発したら車を移動（車線へ、または経路の先へ）。
        1 停車地で MAX_RECOVERIES 回を超えたら停車地に置く。
        """
        here = self.car.get_location()
        wp = self.wmap.get_waypoint(here)
        if not wp.is_junction:
            off = here.distance(wp.transform.location)
            self.max_offset = max(self.max_offset, off)
            limit = 0.5 * wp.lane_width + OFFROAD_MARGIN
            self._offroad_t = self._offroad_t + dt if off > limit else 0.0
        else:
            self._offroad_t = 0.0
        speed = self.car.get_velocity().length()
        pushing = control.throttle > 0.3 and control.brake < 0.1
        self._stuck_t = self._stuck_t + dt if pushing and speed < 0.3 else 0.0
        why = ("off road" if self._offroad_t > OFFROAD_S else
               "stuck" if self._stuck_t > STUCK_S else None)
        if why is None:
            return
        self._offroad_t = self._stuck_t = 0.0
        stop = self.stops[0]
        batch = stop.batch
        self._stop_recoveries += 1
        if self._stop_recoveries > MAX_RECOVERIES:
            # never loop forever: put the van at the stop and carry on, logged
            # 無限ループを避け、車を停車地に置いて続行（記録する）
            tf = self.wmap.get_waypoint(carla.Location(*stop.location)).transform
            self._teleport(tf)
            self.log(t, "gave_up_driving", batch=batch, pos=tup(tf.location),
                     detail="%s, %d recoveries failed" % (why, MAX_RECOVERIES))
            self.arrive(t)
            return
        if why == "stuck" and t - self._last_recover_t >= RETRY_WINDOW_S:
            # first strike when wedged: reverse out, like a driver would
            # 最初は後退して抜け出す
            self._reverse = recovery.ReverseOut(here.x, here.y)
            self.reroutes += 1
            self.log(t, "reverse_out", batch=batch, pos=tup(here), detail=why)
            self._last_recover_t = t
            return
        if t - self._last_recover_t < RETRY_WINDOW_S:
            # second strike. Off road: back onto the nearest lane. Stuck: a few
            # metres ahead along the route -- putting a stalled van back on the
            # same spot (say, a steep hill) just stalls it again.
            # 2 回目。路外なら最寄り車線へ、立ち往生なら経路に沿って数 m 先へ
            # （同じ場所、例えば急坂に戻しても再び止まるだけ）。
            tf = wp.transform if why == "off road" else self._route_ahead(here, wp)
            self._teleport(tf)
            self.lane_resets += 1
            self.log(t, "lane_reset", batch=batch, pos=tup(tf.location), detail=why)
            start = tf.location
        else:
            self.reroutes += 1
            self.log(t, "reroute", batch=batch, pos=tup(here), detail=why)
            start = here
        self._last_recover_t = t
        self.agent.set_destination(carla.Location(*stop.location), start_location=start)

    def _route_ahead(self, here, wp):
        """Transform of the first route waypoint NUDGE_M ahead of the van."""
        for w, _ in self.agent.get_local_planner()._waypoints_queue:
            if w.transform.location.distance(here) >= NUDGE_M:
                return w.transform
        nxt = wp.next(NUDGE_M)
        return nxt[0].transform if nxt else wp.transform

    def _teleport(self, tf):
        tf = carla.Transform(tf.location + carla.Location(z=0.3), tf.rotation)
        self.car.set_target_velocity(carla.Vector3D())
        self.car.set_transform(tf)

    def hold(self):
        self.car.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0,
                                                    hand_brake=True))

    def draw(self, dbg, life):
        for o in self.pending:
            c = COLOR[o.kind]
            dbg.draw_point(loc(o.drop) + carla.Location(z=1.0), size=0.3, color=c,
                           life_time=life)
            dbg.draw_string(loc(o.drop) + carla.Location(z=3.0),
                            "#%d %s  van%d" % (o.id, "DRONE" if o.kind == DRONE else "F2F",
                                               self.vid),
                            color=c, life_time=life)
        for s in self.stops:
            if s.kind == DRONE:
                hub = loc(s.location)
                dbg.draw_point(hub + carla.Location(z=1.0), size=0.4,
                               color=COLOR["hub"], life_time=life)
                dbg.draw_string(hub + carla.Location(z=4.0), "DRONE HUB van%d" % self.vid,
                                color=COLOR["hub"], life_time=life)
                for o in s.drone_order:
                    if o in self.pending:
                        dbg.draw_line(hub + carla.Location(z=1.0),
                                      loc(o.drop) + carla.Location(z=1.0),
                                      thickness=0.05, color=COLOR["hub"], life_time=life)
        u = self.drone
        dbg.draw_string(loc(self.roof()) + carla.Location(z=2.0),
                        "VAN %d  POD %.0f%%  %s  %s  pending:%d  done:%d  failed:%d" % (
                            self.vid, self.energy.level, self.state,
                            "D%d %.0f%%" % (u.id, u.battery.level) if u else "no drone",
                            len(self.pending), len(self.delivered), len(self.failed)),
                        color=self.color, life_time=life)

    def summary(self):
        return {"van": self.vid, "delivered": len(self.delivered),
                "failed": len(self.failed), "pod_battery": round(self.energy.level, 1),
                "pod_km": round(self.pod_km, 2), "ev_charges": self.ev_charges,
                "charge_asap": bool(self.charge_asap), "charge_kw": round(self.charge_kw, 1),
                "batches_completed": self.batches,
                "max_lane_offset_m": round(self.max_offset, 2),
                "reroutes": self.reroutes, "lane_resets": self.lane_resets}


class LmhVan(LmhVanMixin, Van):
    CHARGE_KIND = "lmh"          # charging-station bay 04 / ベイ 04
    """A LastMile Hub (LMH-3): drives like a van, delivers with three Mini
    Pods from distribution points (delivery/lmh_hub.py, lmh_rig.py).
    Its own speed cap: the hub runs on the Prius drivetrain, so it is held
    to --lmh-speed (25 km/h) or less.
    LastMile ハブ。配布地点からミニポッド 3 台で配達する。"""

    def __init__(self, vid, args, world, car, hubs, grp, log, obstacles, gen, sky,
                 client, pod_base, shared, real_hub):
        a = copy.copy(args)
        a.speed = min(args.speed, args.lmh_speed)
        Van.__init__(self, vid, a, world, car, hubs, grp, log, obstacles, gen, sky, pod=None)
        self.lmh_init(client, pod_base, shared, real_hub)

    def assign(self, batch, t):
        # every order is a ground delivery by a pod / 全てポッドの地上配達
        for o in batch:
            if o.kind == DRONE and self.fleet is not None and self.fleet.nofly(o.drop):
                self.log(t, "nofly_minipod", o, batch=o.batch, pos=o.drop,
                         detail="#%d is in a no-fly zone: sent with hub %d's mini pods" % (o.id, self.vid))
            o.kind = FACE_TO_FACE
        return self.lmh_assign(batch, t)

    def arrive(self, t):
        if self.stops and self.stops[0].kind == LMH_STOP:
            return self.lmh_arrive(t)
        return Van.arrive(self, t)

    def _begin(self, t, stage):
        if stage == LMH_WORK:
            return self.lmh_begin(t)
        return Van._begin(self, t, stage)

    def give_back(self, t, why):
        self.lmh_give_back(t, why)

    def step(self, t, dt):
        self.lmh_tick(t, dt)
        Van.step(self, t, dt)
        if self.state == LMH_WORK:
            self.lmh_work(t, dt)

    def draw(self, dbg, life):
        Van.draw(self, dbg, life)
        self.lmh_draw(dbg, life)

    def cleanup(self):
        Van.cleanup(self)
        self.lmh_cleanup()

    def summary(self):
        out = Van.summary(self)
        out.update(self.lmh_summary())
        return out


# --- the fleet / 車隊 ---------------------------------------------------------

class Fleet(object):
    """Order generator + dispatcher + every van, the wind and the drone pool.

    New batches go into the dispatcher; each free van is handed the queued
    batch nearest to it, so no order is ever worked by two vans. Every tick it
    also blows the wind, steps every drone, applies dashboard commands and
    publishes the live state.
    注文生成・配車係・全車両・風・ドローンプール。新しいバッチは配車係に入り、
    空いた車には最も近い待機バッチが渡される。毎 tick 風と全機を進め、
    ダッシュボードのコマンドを適用し、状態を公開する。
    """

    EVENTS_SHOWN = ("failed", "drone_crash", "drone_to_station", "standby_sent", "wind_hold",
                    "rain_hold", "rain_over",
                    "battery_swap", "drone_switch", "box_station", "pod_showcase",
                    "drone_ready", "loading", "drone_home", "landing", "drone_waiting",
                    "drone_divert", "zones", "nofly_f2f", "orders_cancelled", "reverse_out",
                    "to_charger", "ev_charging", "ev_charged", "pod_stranded", "pod_towed",
                    "yard_in", "yard_out", "side_open",
                    "orders_returned", "batch_declined", "waiting_for_charger",
                    "drone_arrived", "drone_charging", "drone_repaired", "wind",
                    "delivered", "drone_charging_in_van", "batch_arrived",
                    # LastMile hubs and their Mini Pods / LastMile ハブとミニポッド
                    "pod_out", "pod_door", "pod_handover", "pod_back", "pod_docked", "pod_fault",
                    "hub_dist", "hub_meet", "pod_stuck", "pod_recovered", "pod_rescue", "pod_range",
                    "nofly_minipod", "hub_load")

    def __init__(self, args, world, vans, gen, log, sky, livew=None):
        self.args, self.world, self.vans, self.gen, self.log = args, world, vans, gen, log
        self.sky, self.live = sky, livew
        self.dispatcher = planning.Dispatcher()
        self.rng = random.Random(args.seed + 11)
        # at most this many batches wait for a van: enough that a van that
        # finishes always finds work, not so many that orders go stale
        # 待機バッチの上限：空いた車が必ず仕事を見つけ、注文が古くならない程度
        self.backlog = args.order_backlog or len(vans)
        for v in vans:
            v.fleet = self
        # the dashboard's showcase: one LastMile pod scrubbed through the HTML cycle
        # ダッシュボードのショーケース（1 台を HTML の手順でスクラブ）
        self.showcase = None
        self.shell_mode = args.shell
        self.pod_msg = ""
        backends.OVERLAYS["on"] = bool(args.tooltips)
        for v in vans:
            if v.lmp:
                v.pod.set_shell(self.shell_mode)
        self._draw_t = self._pub_t = self._weather_t = -1e9
        self._weather_wind = None
        self.rules = None        # delivery/rules.VanRules on Japan profile maps
        # pod delivery zones and no-fly zones (the dashboard's City Map)
        # ポッドの配送ゾーンと飛行禁止区域（ダッシュボードの地図で設定）
        self.zones = zonelib.Zones()
        self.cancelled = 0
        self._zones_mtime, self._zones_t, self._hungry = None, -1e9, False
        self.zones_poll(0.0, force=True)
        # the Drones page: each drone's sensor readings, from the map's geo
        # reference and budgeted raycasts / ドローンページ用のセンサー値
        self.geo = self._georef()
        self.suites = {}
        self.probe = backends.SensorProbe(world)
        self.probe.start(getattr(args, "host", "127.0.0.1"), getattr(args, "port", 2000),
                         lambda: self.sky.drones.units)
        self.sensor_ms = 0.0
        self.minipod_ms = 0.0
        self.lmh_shared = None   # LastMile hubs: router, obstacles, probe / ハブ共用

    def _georef(self):
        """The map's geo reference (CARLA's own transform_to_geolocation)."""
        try:
            m = self.world.get_map()
            g = [m.transform_to_geolocation(carla.Location(x, y, 0.0))
                 for x, y in ((0.0, 0.0), (1000.0, 0.0), (0.0, 1000.0))]
            return drone_sensors.GeoRef(*[(a.latitude, a.longitude, a.altitude) for a in g])
        except RuntimeError:
            return drone_sensors.GeoRef()

    def sensor_readings(self, t):
        """Sensor readings of every drone (the live state's drones[].sensors),
        budgeted: rays <= 1 ms, the maths ~0.1 ms per drone.
        全機のセンサー値（レイは 1 ms 以内）。"""
        t0 = time.perf_counter()
        units = self.sky.drones.units
        if not self.probe.threaded:
            self.probe.run(units)
        wind = self.sky.wind.vector(t)
        st = self.sky.drones.station.location
        rain = self.sky.drones.rain
        out = {}
        for u in units:
            suite = self.suites.get(u.id)
            if suite is None:
                suite = self.suites[u.id] = drone_sensors.SensorSuite(u.id, self.geo)
            snap = backends.sensor_snapshot(u, wind, self.probe, st)
            snap["rain"] = rain
            out[u.id] = suite.update(t, snap)
        self.sensor_ms = (time.perf_counter() - t0) * 1000.0
        return out

    def set_rules(self, rules):
        """Japanese rules for the vans (None = off, every other map).
        / 配送車の交通ルール（None なら無効）。"""
        was, self.rules = self.rules, rules
        for v in self.vans:
            v.rules = rules
            if rules is None and was is not None:
                v.agent.set_target_speed(v.args.speed)       # undo a speed cap / 上限を解除

    def rules_step(self):
        if self.rules is None:
            return
        sc = self.showcase
        ids = [v.car.id for v in self.vans
               if v.state == DRIVING and not (sc is not None and v is sc["van"])]
        try:
            self.rules.update(self.world.get_snapshot(), ids)
        except Exception as exc:     # never stop the deliveries / 配送は止めない
            print("  delivery traffic rules off: %s / 配送車の交通ルールを停止" % exc,
                  flush=True)
            self.set_rules(None)

    @property
    def batches(self):
        return sum(v.batches for v in self.vans)

    # -- delivery zones / 配送ゾーン
    def zones_poll(self, t, force=False):
        """Pick up zones saved on the dashboard (data/delivery_zones.json),
        checked once a second. / ダッシュボードで保存したゾーンを毎秒確認。"""
        now = time.time()
        if not force and now - self._zones_t < 1.0:
            return
        self._zones_t = now
        try:
            mtime = os.path.getmtime(zonelib.ZONES_PATH)
        except OSError:
            mtime = None
        if not force and mtime == self._zones_mtime:
            return
        self._zones_mtime = mtime
        new = zonelib.Zones(zonelib.load(self.world.get_map().name))
        if not force and new.summary() == self.zones.summary():
            return
        self.zones = new
        zonelib.set_active(new.air)
        for v in self.vans:
            v._plans = {}                  # re-plan altitudes round the zones / 高度を再計画
        if force and not new.pods and not new.nofly and not new.meet and not new.dist:
            return
        self.log(t, "zones", detail="pod zones: %s; %d no-fly zone(s); meeting points: %s; "
                 "distribution points: %s" % (
                     ", ".join("pod %d" % k for k in sorted(new.pods)) or "none", len(new.nofly),
                     ", ".join("pod %d x%d" % (k, len(p)) for k, p in sorted(new.meet.items())) or "auto",
                     ", ".join("hub %d x%d" % (k, len(p)) for k, p in sorted(new.dist.items())) or "auto"))
        self._cancel_orphans(t)

    def _vids(self):
        return [v.vid for v in self.vans]

    # -- who may take what (pod zones, no-fly zones, LastMile hubs) / 担当の判定
    # An LMH hub without a pod zone of its own has the no-fly zones as its
    # area by default (its Mini Pods drive; nothing flies), while any are
    # drawn; a hub with a user zone uses that zone like an LMP pod. No-fly
    # orders no hub covers stay face to face by the drone pod (old way).
    # ゾーンの無い LastMile ハブは既定で飛行禁止区域が担当（ポッドは地上走行）。
    def _lmh_vids(self):
        return {v.vid for v in self.vans if getattr(v, "lmh", False)}

    def nofly(self, p):
        """Is p inside a no-fly zone (no margin)? / 飛行禁止区域の中か。"""
        air = self.zones.air
        return bool(air) and air.blocked(p, 0.0)

    def _default_hubs(self):
        """LMH hubs whose area is the no-fly zones (no user zone, zones drawn)."""
        if not self.zones.nofly:
            return set()
        return {v for v in self._lmh_vids() if self.zones.zone_of(v) is None}

    def area_of(self, vid):
        """'zone' (user's pod zone), 'nofly' (an LMH hub's default) or 'any'."""
        if self.zones.zone_of(vid) is not None:
            return "zone"
        return "nofly" if vid in self._default_hubs() else "any"

    def _nofly_batch(self, batch):
        return bool(batch) and all(self.nofly(o.drop) for o in batch)

    def _home(self, batch):
        """The pod whose zone holds most of ``batch`` (None: outside them all)."""
        vids = self._vids()
        owners = collections.Counter(self.zones.owner(o.drop, vids) for o in batch)
        return owners.most_common(1)[0][0] if owners else None

    def _region(self, batch):
        """The area a batch belongs to: a vid (user zone), "nofly" (the default
        hubs) or None (outside every zone). / バッチの区域。"""
        home = self._home(batch)
        if self._nofly_batch(batch) and self._default_hubs():
            return home if home in self._lmh_vids() else "nofly"
        return home

    def _servable(self, batch):
        return any(self._can(batch, v) for v in self._vids())

    def _can(self, batch, vid):
        """May pod ``vid`` take ``batch``? Only its own zone's orders, and the
        pods without a zone take the rest; no-fly orders go to an LMH hub
        (one whose zone holds them first, else a default hub).
        自分のゾーンの注文だけ担当。飛行禁止区域の注文は LastMile ハブへ。"""
        vids = self._vids()
        hubs, default = self._lmh_vids(), self._default_hubs()
        if self._nofly_batch(batch) and hubs:
            home = self._home(batch)
            if home in hubs:
                return vid == home
            if default:
                return vid in default
        if vid in default:
            return False                 # default hubs only serve the no-fly zones
        if not any(self.zones.zone_of(v) for v in vids) and not default:
            return True
        home = self._home(batch)
        if home is not None:
            return vid == home
        return self.zones.zone_of(vid) is None

    def _split_nofly(self, batch):
        """[batch] or [no-fly part, rest] when an LMH hub can take the no-fly
        orders. / 飛行禁止区域の注文を分けてハブへ。"""
        if not self._lmh_vids() or not self.zones.nofly:
            return [batch]
        nf = [o for o in batch if self.nofly(o.drop)]
        if not nf or len(nf) == len(batch):
            return [batch]
        if not any(self._can(nf, v) for v in self._lmh_vids()):
            return [batch]
        return [nf, [o for o in batch if not self.nofly(o.drop)]]

    def _cancel_orphans(self, t):
        """Queued orders no pod may take any more (every pod has a zone and
        they are outside all of them): cancelled. / 担当できない注文を取消。"""
        keep = []
        for b in self.dispatcher.queue:
            if self._servable(b):
                keep.append(b)
                continue
            for o in b:
                self.gen.release(o)
            self.cancelled += len(b)
            self.log(t, "orders_cancelled", batch=b[0].batch,
                     detail="#%s: outside every pod zone" % ",".join(str(o.id) for o in b))
        self.dispatcher.queue = keep

    def _set_area(self, t, ready):
        """Where the next batch is made: the zone of a pod that is free and has
        nothing queued first, else any zone (pods without a zone count as one
        area, outside every zone; default LMH hubs as the no-fly area).
        次のバッチを作る区域（空いたポッドを優先）。"""
        g, z = self.gen, self.zones
        nofly = (lambda p: z.air.blocked(p, 0.0)) if z.air else None
        g.no_drone = (lambda p: z.air.blocked(p)) if z.air else None
        vids = self._vids()
        default = self._default_hubs()
        zoned = [v for v in vids if z.zone_of(v)]
        self._hungry = False
        if not zoned and not default:
            g.area = g.area_key = None
            return
        unzoned = [v for v in vids if v not in zoned and v not in default]
        regions = zoned + (["nofly"] if default else []) + ([None] if unzoned else [])
        queued = collections.Counter(self._region(b) for b in self.dispatcher.queue)
        idle = {v.vid for v in ready}
        members = lambda r: (default if r == "nofly" else unzoned if r is None else [r])
        hungry = [r for r in regions if not queued[r] and any(u in idle for u in members(r))]
        self._hungry = bool(hungry)
        pool = hungry or regions
        r = self.rng.choices(pool, [len(unzoned) if r is None else 1 for r in pool])[0]
        g.area_key = r
        if r == "nofly":
            # ground orders for the hubs: drone kinds allowed (pods deliver them)
            # ハブ向けの地上注文（ドローン種別でもポッドが配達）
            g.no_drone = None
            g.area = nofly
        elif r is None and default:
            g.area = (lambda p: z.owner(p, vids) is None and not nofly(p))
        else:
            g.area = (lambda p, r=r: z.owner(p, vids) == r)

    def step(self, t, dt):
        self.commands(t)
        self.zones_poll(t)
        if getattr(self, "boxes", None) is not None:
            self.boxes.step(t, dt)                 # customers collecting parcels / 受取客
        if getattr(self.sky, "walker_guard", None) is not None:
            self.sky.walker_guard.step(t)           # walkers round the charging stations / 歩行者の迂回
        self.sky.wind.step(t, dt)
        wind = self.sky.wind.vector(t)
        lmh = getattr(self, "lmh_shared", None)
        if lmh is not None:                       # walkers / cars near the pods / ポッド周辺
            lmh["obstacles"].refresh(t, [p.pos for v in self.vans if getattr(v, "lmh", False)
                                         for p in v.crew.pods if p.out()])
        ready = [v for v in self.vans
                 if v.state == IDLE and v.energy.level >= v.pem.low and not v.charge_asap]
        self._set_area(t, ready)
        if ready and not any(self._can(b, v.vid) for b in self.dispatcher.queue for v in ready):
            # a van with nothing to do: the next batch comes within seconds
            # 仕事の無い車がいれば、次のバッチを数秒以内に
            self.gen.prompt(t, self.rng.uniform(2.0, 6.0))
        if len(self.dispatcher.queue) < self.backlog or self._hungry:
            batch = self.gen.poll(t)
            if batch:
                self.log(t, "batch_arrived", batch=batch[0].batch,
                         detail="%d orders" % len(batch))
                for o in batch:
                    self.log(t, "order", o, pos=o.drop, detail=o.building.name)
                for part in self._split_nofly(batch):
                    self.dispatcher.add(part)
        free = [(v.vid, v.car_pos()) for v in ready]
        by_id = {v.vid: v for v in self.vans}
        for vid, b in self.dispatcher.assign(free, can=self._can):
            self.log(t, "dispatch", batch=b[0].batch, van=vid,
                     detail="orders %s" % [o.id for o in b])
            if not by_id[vid].assign(b, t):
                self.dispatcher.push_front(b)
                by_id[vid].go_charge(t, "not enough for batch %d" % b[0].batch)
        self.rules_step()
        sc = self.showcase
        for v in self.vans:
            if sc is not None and v is sc["van"]:
                v.hold()
                if sc["playing"]:
                    sc["t"] = (sc["t"] + dt * sc["speed"]) % lmp_rig.COMBINED_TOTAL
                v.showcase_pose(sc["t"])
            else:
                v.step(t, dt)
        self.sky.drones.step(t, dt, wind)
        self.draw(t)
        self.weather(t)
        self.publish(t)

    # -- wind / 風
    def commands(self, t):
        """Wind commands from the dashboard. / ダッシュボードからの風コマンド。"""
        if self.live is None:
            return
        w = self.sky.wind
        for c in self.live.commands():
            cmd = c.get("cmd")
            if cmd == "gust":
                g = w.gust(t, float(c.get("speed", 10.0)), c.get("dir"),
                           float(c.get("duration", 6.0)))
                self.log(t, "wind", detail="gust from the dashboard: %.0f m/s towards "
                         "%.0f deg for %.0f s" % (g["peak"], g["dir"], g["dur"]))
            elif cmd == "wind":
                w.set_base(c.get("speed"), c.get("dir"))
                self.log(t, "wind", detail="base wind set to %.1f m/s towards %.0f deg"
                         % (w.base_speed, w.base_dir))
            elif cmd == "calm":
                w.calm()
                self.log(t, "wind", detail="calm")
            elif cmd == "gust_rate":
                w.gust_rate = max(float(c.get("value", 0.0)), 0.0)
                self.log(t, "wind", detail="random gusts: %.1f per minute" % w.gust_rate)
            elif cmd in ("shell", "pod_takeover", "pod_release", "pod_time", "pod_play"):
                self.pod_command(t, c)
            elif cmd == "charge":
                # send a vehicle to charge (when it is free) / 充電へ向かわせる
                v = next((v for v in self.vans if v.vid == int(c.get("van", -1))), None)
                if v is not None:
                    v.charge_asap = True
                    self.log(t, "charge_request", pos=v.car_pos(),
                             detail="van %d sent to charge from the dashboard (at %.0f%%)" % (v.vid, v.energy.level))
            elif cmd == "tooltips":
                backends.OVERLAYS["on"] = bool(c.get("on"))
                self.log(t, "view", detail="tooltips %s" % ("on" if backends.OVERLAYS["on"] else "off"))

    def set_shell(self, mode):
        if mode in lmp_pod.SHELL_MODES:
            self.shell_mode = mode
            for v in self.vans:
                if v.lmp:
                    v.pod.set_shell(mode)

    def pod_command(self, t, c):
        """Shell mode and the showcase, from the dashboard (Delivery tab, Settings).
        外殻モードとショーケース（ダッシュボードの Pod タブ）。"""
        cmd, sc = c.get("cmd"), self.showcase
        if cmd == "shell":
            self.set_shell(c.get("mode"))
            self.log(t, "pod_view", detail="shell: %s" % self.shell_mode)
        elif cmd == "pod_takeover":
            van = next((v for v in self.vans if v.vid == c.get("van")), None)
            why = "no pod %s" % c.get("van") if van is None else van.showcase_ok()
            if why:
                self.pod_msg = "can't take over: " + why
                return
            if sc is not None:
                sc["van"].showcase_release()
            self.showcase = {"van": van, "t": float(c.get("t", 0.0)), "playing": True,
                             "speed": float(c.get("speed", 1.0))}
            self.pod_msg = ""
            if getattr(self, "camera", None) is not None:
                self.camera.lock(van)
            self.log(t, "pod_showcase", van=van.vid, detail="dashboard took the pod over")
        elif cmd == "pod_release" and sc is not None:
            sc["van"].showcase_release()
            self.log(t, "pod_showcase", van=sc["van"].vid, detail="pod released")
            self.showcase = None
        elif cmd == "pod_time" and sc is not None:
            sc["t"] = lmp_rig.clamp(float(c.get("t", 0.0)), 0.0, lmp_rig.COMBINED_TOTAL - 1e-3)
        elif cmd == "pod_play" and sc is not None:
            sc["playing"] = bool(c.get("playing", True))
            sc["speed"] = lmp_rig.clamp(float(c.get("speed", sc["speed"])), 0.1, 4.0)

    def weather(self, t):
        """Let the trees sway with the wind (CARLA weather wind_intensity), and
        read the rain that grounds the drones (weather_cycle.py).
        風で木を揺らし、ドローンを止める雨量を読む。"""
        if t - self._weather_t < 2.0:
            return
        self._weather_t = t
        was = self.sky.drones.raining
        self.sky.drones.rain = float(self.world.get_weather().precipitation)
        if self.sky.drones.raining != was:
            self.log(t, "rain_hold" if not was else "rain_over",
                     detail="rain %.0f%%: %s" % (self.sky.drones.rain,
                     "drones in the air come down, no take-offs" if not was else "drones may fly again"))
        speed = math.hypot(*self.sky.wind.vector(t))
        intensity = min(100.0, speed / 15.0 * 100.0)
        if self._weather_wind is None or abs(intensity - self._weather_wind) > 5.0:
            w = self.world.get_weather()
            w.wind_intensity = intensity
            self.world.set_weather(w)
            self._weather_wind = intensity

    # -- drawing / 描画
    def draw(self, t):
        if t - self._draw_t < 0.5:
            return
        self._draw_t = t
        dbg, life = self.world.debug, 0.6
        for u in self.sky.drones.units:              # drawn drones (no 3-D model) / 描画機体
            u.draw_idle(life)
        if not backends.OVERLAYS["on"]:              # tooltips off / ツールチップ非表示
            return
        for o in self.dispatcher.orders:            # waiting for a free van
            dbg.draw_point(loc(o.drop) + carla.Location(z=1.0), size=0.3,
                           color=COLOR[o.kind], life_time=life)
            dbg.draw_string(loc(o.drop) + carla.Location(z=3.0),
                            "#%d %s  queued" % (o.id, "DRONE" if o.kind == DRONE else "F2F"),
                            color=COLOR[o.kind], life_time=life)
        for v in self.vans:
            v.draw(dbg, life)
        st = self.sky.drones.station
        for i, p in enumerate(st.pads):
            # debug lines glow: a thin, dim outline, or the bloom hides the drones
            # デバッグ線は発光するため細く暗く（明るいと機体が光に埋もれる）
            dbg.draw_box(carla.BoundingBox(loc(p), carla.Vector3D(0.6, 0.6, 0.03)),
                         carla.Rotation(), thickness=0.02, color=carla.Color(10, 45, 10),
                         life_time=life)
        c = st.location
        dbg.draw_string(carla.Location(c[0], c[1], c[2] + 3.0), "CHARGE STATION",
                        color=COLOR["station"], life_time=life)
        for i, p in enumerate(self.sky.chargers.spots):
            if i in getattr(self.sky, "station_bays", {}):
                continue                        # a charging station: its model is there / モデルあり
            who = self.sky.chargers.taken.get(i)
            dbg.draw_box(carla.BoundingBox(loc(p), carla.Vector3D(2.6, 1.3, 0.03)),
                         carla.Rotation(yaw=self.sky.charger_yaw[i]), thickness=0.03,
                         color=carla.Color(45, 40, 5), life_time=life)
            dbg.draw_string(carla.Location(p[0], p[1], p[2] + 2.5),
                            "EV CHARGER %d%s" % (i + 1, "  V%d" % who if who else ""),
                            color=COLOR["ev"], life_time=life)
        for u in self.sky.drones.units:
            if u.status in FLYING or u.status in ("charging", "standby", "crashed"):
                lvl = u.battery.level
                col = (carla.Color(0, 255, 0) if lvl > 50 else
                       carla.Color(255, 200, 0) if lvl > 25 else carla.Color(255, 40, 40))
                p = u.world_pos()
                dbg.draw_string(carla.Location(p[0], p[1], p[2] + 1.2),
                                "D%d %.0f%%%s" % (u.id, lvl,
                                                   " X" if u.status == "crashed" else ""),
                                color=col, life_time=life)

    # -- live state for the dashboard / ダッシュボード用の状態
    def publish(self, t):
        if self.live is None or t - self._pub_t < 0.5:
            return
        self._pub_t = t
        r1 = lambda v: round(v, 1)
        drones = []
        try:
            sensors = self.sensor_readings(t)
        except Exception as e:      # never stop the sim for the dashboard / 表示用で止めない
            print("[delivery] drone sensors: %s" % e, file=sys.stderr)
            sensors = {}
        for u in self.sky.drones.units:
            p = u.world_pos()
            drones.append({"id": u.id, "status": u.status, "note": u.note,
                           "battery": round(u.battery.level, 1),
                           "van": u.van.vid if u.van else None,
                           "x": r1(p[0]), "y": r1(p[1]), "z": r1(p[2]), "yaw": r1(u.yaw),
                           "drift": round(u.drift.offset, 2), "trips": u.trips,
                           "crashes": u.crashes, "fail": u.fail_reason,
                           "order": u.order.id if u.order else None,
                           # for drone_viewer.py: the body to hang a camera on
                           # drone_viewer.py 用：カメラを付ける機体
                           "actor": u.body.id if u.body is not None else None,
                           "phase": getattr(u.flight, "state", None) if u.flight else None,
                           "km": round(u.km, 2), "air_s": round(getattr(u, "air_s", 0.0)),
                           "sensors": sensors.get(u.id)})
        vans, hubs, orders = [], [], []
        for v in self.vans:
            tf = v.car.get_transform()
            vans.append({"id": v.vid, "actor": v.car.id, "kinematic": v.kinematic,
                         "x": r1(tf.location.x), "y": r1(tf.location.y),
                         "yaw": r1(tf.rotation.yaw), "state": v.state,
                         "battery": r1(v.energy.level), "pod_km": round(v.pod_km, 2),
                         "ev_charges": v.ev_charges,
                         "drone": v.drone.id if v.drone else None,
                         "incoming": v.incoming.id if v.incoming else None,
                         "pending": len(v.pending), "delivered": len(v.delivered),
                         "failed": len(v.failed), "zone": self.zones.zone_of(v.vid) is not None,
                         "kind": "lmh" if getattr(v, "lmh", False) else "lmp"})
            if getattr(v, "lmh", False):
                vans[-1].update(v.lmh_van_state(), area=self.area_of(v.vid))
            for s in v.stops:
                if s.kind == DRONE:
                    hubs.append({"van": v.vid, "x": r1(s.location[0]), "y": r1(s.location[1])})
            for o in v.pending:
                status = ("in_flight" if o is v.current or o in getattr(v, "in_air", {}).values()
                          else "assigned")
                orders.append({"id": o.id, "kind": o.kind, "x": r1(o.drop[0]),
                               "y": r1(o.drop[1]), "status": status, "van": v.vid, "zone": o.zone})
        for o in self.dispatcher.orders:
            orders.append({"id": o.id, "kind": o.kind, "x": r1(o.drop[0]), "y": r1(o.drop[1]),
                           "status": "queued", "van": None, "zone": o.zone})
        done = sorted([o for v in self.vans for o in v.delivered + v.failed],
                      key=lambda o: o.failed_t or o.delivered_t or 0.0)[-40:]
        for o in done:
            orders.append({"id": o.id, "kind": o.kind, "x": r1(o.drop[0]), "y": r1(o.drop[1]),
                           "status": "failed" if o.fail_reason else "delivered",
                           "van": o.van, "reason": o.fail_reason})
        t1 = time.perf_counter()
        minipods = []
        for v in self.vans:
            if getattr(v, "lmh", False):
                try:
                    minipods += v.lmh_pod_states(t)
                except Exception as e:      # never stop the sim for the dashboard / 表示用で止めない
                    print("[delivery] minipod state: %s" % e, file=sys.stderr)
        self.minipod_ms = (time.perf_counter() - t1) * 1000.0
        mine = {v.car.id for v in self.vans}
        cars = [[r1(a.get_transform().location.x), r1(a.get_transform().location.y),
                 r1(a.get_transform().rotation.yaw)]
                for a in self.world.get_actors().filter("vehicle.*") if a.id not in mine]
        events = [{"t": e["sim_t"], "van": e["van"], "event": e["event"],
                   "order": e["order"], "detail": e["detail"]}
                  for e in self.log.rows[-300:] if e["event"] in self.EVENTS_SHOWN][-30:]
        st = self.sky.drones.station
        self.live.write_state({
            "running": True, "t": r1(t), "map": self.world.get_map().name,
            "wind": dict(self.sky.wind.state(t), grounded=self.sky.drones.grounded,
                         hold=self.sky.drones.wind_hold, drone_max=self.args.drone_max_wind,
                         rain=r1(self.sky.drones.rain), raining=self.sky.drones.raining,
                         rain_ground=self.sky.drones.rain_ground),
            "drones": drones, "vans": vans, "hubs": hubs, "minipods": minipods,
            "minipod_ms": round(self.minipod_ms, 2),
            "orders": orders, "cars": cars, "events": events,
            "station": {"pads": [[r1(p[0]), r1(p[1])] for p in st.pads],
                        "x": r1(st.location[0]), "y": r1(st.location[1])},
            "chargers": [{"x": r1(p[0]), "y": r1(p[1]), "van": self.sky.chargers.taken.get(i)}
                         for i, p in enumerate(self.sky.chargers.spots)],
            "stations": self.station_state(t), "station_errors": list(getattr(self.sky, "station_errors", [])),
            "stats": self.stats(t), "pod": self.pod_state(), "zones": self.zones.summary(),
            "tooltips": backends.OVERLAYS["on"],
            "geo": self.geo.to_dict(), "sensor_ms": round(self.sensor_ms, 2)})

    def pod_state(self):
        """The dashboard's pod panels: shell mode, LastMile pods, the showcase and its steps."""
        sc = self.showcase
        show = None
        if sc is not None:
            scen, step = lmp_rig.step_at(sc["t"])
            show = {"van": sc["van"].vid, "t": round(sc["t"], 2), "playing": sc["playing"],
                    "speed": sc["speed"], "scenario": scen, "step": step["id"],
                    "title": step["title"]}
        lmp_vans = [{"id": v.vid, "state": v.state, "free": v.showcase_ok() is None,
                     "job": (v._job or {}).get("phase"), "out": len(v.flying_units()),
                     "drones": [None if u is None else {"id": u.id, "slot": s,
                                                        "battery": round(u.battery.level)}
                                for u, s in zip(v.lmp_units, v.slots)],
                     "packs": [None if p is None else round(p) for p in v.packs]}
                    for v in self.vans if v.lmp]
        steps = [dict(s, scenario="drone") for s in lmp_rig.DRONE_STEPS] + [
            dict(s, scenario="direct", t0=s["t0"] + lmp_rig.DRONE_TOTAL,
                 t1=s["t1"] + lmp_rig.DRONE_TOTAL) for s in lmp_rig.DIRECT_STEPS]
        return {"shell": self.shell_mode, "vans": lmp_vans, "showcase": show,
                "total": lmp_rig.COMBINED_TOTAL, "steps": steps, "msg": self.pod_msg}

    def station_state(self, t):
        """The charging stations for the dashboard: footprint, bays, who is in
        each bay and doing what. / ダッシュボード用の充電ステーション状態。"""
        st = getattr(self.sky, "stations", None)
        if st is None or not st.sites:
            return []
        r1 = lambda v: round(v, 1)
        by_vid = {v.vid: v for v in self.vans}
        phase_of = {YARD_IN: "in", YARD_OUT: "out", JACKS_DOWN: "in", JACKS_UP: "unplugging"}
        out, idx = [], {}
        for i, b in self.sky.station_bays.items():
            if b.get("kind") == "station":                # not the waiting bays / 待機ベイは除く
                idx[(b["station"], b["bay_kind"])] = i
        for s in st.sites:
            bays = []
            for kind, b in sorted(s["bays"].items()):
                i = idx.get((s["id"], kind))
                vid = self.sky.chargers.taken.get(i) if i is not None else None
                v = by_vid.get(vid)
                phase = "free"
                if v is not None:
                    if v.state == EV_CHARGING:
                        u = s["plug_u"].get(kind, 0.0)
                        phase = "unplugging" if v._unplug_t is not None else ("charging" if u >= 1.0 else "plugging")
                    else:
                        phase = phase_of.get(v.state, "in")
                inbay = v is not None and phase != "free"
                bays.append({"kind": kind, "x": r1(b["bay"][0]), "y": r1(b["bay"][1]), "yaw": r1(b["bay_yaw"]),
                             "van": vid, "phase": phase,
                             "level": r1(v.energy.level) if inbay else None,
                             "kw": r1(v.charge_kw) if inbay and v.state == EV_CHARGING else None,
                             "energy_kwh": round(v.charge_kwh, 2) if inbay and v.state == EV_CHARGING else None})
            # who waits for this station, and its waiting bay / 待機中の車両と待機ベイ
            queue, qv = [], (getattr(self.sky, "queue_slot", {}) or {}).get(s["id"])
            for v in self.vans:
                if isinstance(v.yard_bay, tuple) and v.yard_bay[1] == s["id"] and v._queue_t is not None:
                    queue.append({"van": v.vid, "kind": v.CHARGE_KIND, "wait_s": r1(t - v._queue_t)})
            q = s.get("queue")
            pads = []
            for v in self.vans:
                vs = v.lmh_van_state() if hasattr(v, "lmh_van_state") else {}
                if vs.get("station") == s["id"] and vs.get("pads"):
                    pads = vs["pads"]
            out.append({"id": s["id"], "x": r1(s["centre"][0]), "y": r1(s["centre"][1]), "yaw": r1(s["frame"].yaw),
                        "source": s.get("source", "random"),
                        "corners": [[r1(x), r1(y)] for x, y in s["corners"]], "bays": bays,
                        "energy_kwh": round((getattr(self.sky, "station_kwh", {}) or {}).get(s["id"], 0.0), 2),
                        "queue": queue,
                        "queue_spot": ({"x": r1(q["bay"][0]), "y": r1(q["bay"][1]), "yaw": r1(q["bay_yaw"]), "van": qv}
                                       if q else None),
                        "pads": pads or [{"pod": None, "level": None, "phase": "free"} for _ in range(3)],
                        "deck": self._deck_state(s["id"]),
                        "ring": [[r1(x), r1(y)] for x, y in s.get("ring", [])]})
        return out

    def _deck_state(self, sid):
        """The station's 3 drone-deck pads: which drone, its battery, plug phase."""
        st, ds = self.sky.drones.station, []
        if not hasattr(st, "pad_owner"):
            return [{"drone": None, "level": None, "phase": "free"} for _ in range(3)]
        units = {u.id: u for u in self.sky.drones.units}
        for i in range(len(st.pads)):
            own = st.pad_owner(i)
            if own is None or own[0] != sid:
                continue
            uid = st.taken.get(i)
            u = units.get(uid)
            plug = self.sky.stations.sites[sid - 1].get("deck_u", {}).get(own[1], 0.0)
            phase = "free" if u is None else ("charging" if plug >= 0.999 else ("plugging" if plug > 0 else "in"))
            ds.append({"drone": uid, "level": round(u.battery.level, 1) if u is not None else None, "phase": phase})
        return ds

    def stats(self, t):
        delivered = [o for v in self.vans for o in v.delivered]
        failed = [o for v in self.vans for o in v.failed]
        reasons = {}
        for o in failed:
            key = o.fail_reason.split(":")[0]
            reasons[key] = reasons.get(key, 0) + 1
        return {"delivered": len(delivered), "failed": len(failed),
                "failed_by_reason": reasons,
                "delivered_drone": sum(o.kind == DRONE for o in delivered),
                "delivered_face_to_face": sum(o.kind == FACE_TO_FACE for o in delivered),
                "queued": len(self.dispatcher.orders),
                "cancelled_outside_zones": self.cancelled,
                "busy_vans": sum(v.state != IDLE for v in self.vans),
                "ev_charges": sum(v.ev_charges for v in self.vans),
                "pod_km": round(sum(v.pod_km for v in self.vans), 2),
                "pods_stranded": sum(v.state == STRANDED for v in self.vans),
                "gusts": len(self.sky.wind.history), **self.sky.drones.summary()}

    def summary(self, t):
        delivered = [o for v in self.vans for o in v.delivered]
        waits = [o.delivered_t - o.created_t for o in delivered]
        per = [v.summary() for v in self.vans]
        out = {"map": self.world.get_map().name, "drone_backend": self.args.drone,
               "seed": self.args.seed, "vans": len(self.vans),
               "sim_seconds": round(t, 1),
               "batches_completed": self.batches}
        out.update(self.stats(t))
        out.update({
            "still_queued": len(self.dispatcher.orders),
            "batches_skipped_all_buildings_busy": self.gen.skipped,
            "mean_wait_s": round(sum(waits) / len(waits), 1) if waits else None,
            "max_lane_offset_m": max(p["max_lane_offset_m"] for p in per),
            "reroutes": sum(p["reroutes"] for p in per),
            "lane_resets": sum(p["lane_resets"] for p in per),
            "traffic_rules": self.rules.summary() if self.rules is not None else None,
            "per_van": per,
            "per_drone": [{"drone": u.id, "status": u.status,
                           "battery": round(u.battery.level, 1), "trips": u.trips,
                           "km": round(u.km, 3), "crashes": u.crashes}
                          for u in self.sky.drones.units]})
        return out


class CameraControl(object):
    """The CARLA window's camera: yours to fly, until you lock it with Q / E.

    Free (the default): this script never moves the spectator, so you fly it
    yourself (hold the right mouse button, WASD). In the CARLA window, Q / E
    lock the camera onto the previous / next target, cycling through every
    pod, then every drone in the air, then every crashed drone. Moving the
    camera yourself unlocks it. ``--follow auto`` follows whichever van is
    busy, as before, until you press Q / E or move the camera.
    CARLA ウィンドウのカメラ：通常は自由操作。Q / E で前 / 次の対象（全ポッド、
    飛行中の機体、墜落機）に固定し、自分でカメラを動かすと解除。
    """

    MOVED_M, MOVED_DEG = 1.0, 4.0   # further than this from where we put it: you moved it
    GRACE_S = 0.6       # CARLA's own camera also moves on Q / E: ignore that bit

    def __init__(self, world, args, vans, drones):
        self.world, self.vans, self.drones = world, vans, drones
        self.spectator = world.get_spectator()
        self.mode = args.follow             # free | auto | none
        self.target = ("van", vans[args.follow_van - 1]) if args.follow_van else None
        self.followed = None                # the van --follow auto is on
        self.hotkeys = Hotkeys(keys=("q", "e", "x")) if self.mode != "none" else None
        self.on_x = None                    # X: next shell view (set by main) / X: 外殻表示
        self._set = collections.deque(maxlen=3)    # transforms we put the camera at
        self._grace = 0.0
        if self.hotkeys is not None and not self.hotkeys.ok:
            print("  camera: Q / E need an X11 session; the camera is free / "
                  "Q・E には X11 が必要", flush=True)
        elif self.hotkeys is not None:
            print("  camera: free. In the CARLA window press Q / E to lock onto the "
                  "previous / next pod or drone; move the camera to unlock / "
                  "CARLA ウィンドウで Q・E: ポッドとドローンを順に追従、動かすと解除",
                  flush=True)

    def targets(self):
        """Pods, then drones in the air, then crashed drones. / 追従対象の順番。"""
        return ([("van", v) for v in self.vans] +
                [("drone", u) for u in self.drones if u.status in FLYING + (FALLING,)] +
                [("drone", u) for u in self.drones if u.status == CRASHED])

    @staticmethod
    def label(tg):
        kind, o = tg
        return "pod V%d" % o.vid if kind == "van" else "D%d (%s)" % (o.id, o.status)

    def step(self):
        if self.mode == "none":
            return
        now = time.monotonic()
        keys = self.hotkeys.poll() if self.hotkeys is not None else []
        if "x" in keys and self.on_x is not None:
            self.on_x()
        keys = [k for k in keys if k in ("q", "e")]
        if keys:
            self._cycle(1 if keys[-1] == "e" else -1, now)
        self._follow_through()
        if self.target is None and self.mode != "auto":
            return
        if self._set and now > self._grace and not self._ours(self.spectator.get_transform()):
            if self.target is not None:
                print("camera: free (you moved it) / カメラ自由", flush=True)
            self.target, self.mode = None, "free"
            self._set.clear()
            return
        tf = self._view()
        if tf is not None:
            self.spectator.set_transform(tf)
            self._set.append(tf)
            if self.target is not None and backends.OVERLAYS["on"]:
                p = tf.location + tf.get_forward_vector() * 6.0
                self.world.debug.draw_string(p + carla.Location(z=1.5),
                                             "CAM: " + self.label(self.target),
                                             color=carla.Color(255, 255, 255), life_time=0.1)

    def lock(self, van):
        """Watch ``van`` (the dashboard took it over). / この車を追従。"""
        if self.mode == "none":
            return
        self.target = ("van", van)
        self._grace = time.monotonic() + self.GRACE_S
        self._set.clear()

    def _cycle(self, step, now):
        tgs = self.targets()
        if not tgs:
            return
        cur = next((i for i, tg in enumerate(tgs)
                    if self.target is not None and tg[1] is self.target[1]), None)
        i = (0 if step > 0 else len(tgs) - 1) if cur is None else (cur + step) % len(tgs)
        self.target = tgs[i]
        self._grace = now + self.GRACE_S
        self._set.clear()
        print("camera: locked on %s [%d/%d] / カメラ固定" % (self.label(self.target), i + 1,
                                                           len(tgs)), flush=True)

    def _follow_through(self):
        """A drone that lands goes on being watched in its pod; a wreck that is
        recovered frees the camera. / 着陸した機体はポッドを追従、回収で解除。"""
        if self.target is None or self.target[0] != "drone":
            return
        u = self.target[1]
        if u.status in FLYING + (FALLING, CRASHED):
            return
        if u.status in IN_VAN and u.van is not None:
            self.target = ("van", u.van)
        else:
            self.target = None
            print("camera: free (D%d is %s) / カメラ自由" % (u.id, u.status), flush=True)

    def _ours(self, cur):
        """``cur`` is (nearly) where we last put the camera."""
        for tf in self._set:
            dyaw = abs((cur.rotation.yaw - tf.rotation.yaw + 180.0) % 360.0 - 180.0)
            if (cur.location.distance(tf.location) <= self.MOVED_M and dyaw <= self.MOVED_DEG
                    and abs(cur.rotation.pitch - tf.rotation.pitch) <= self.MOVED_DEG):
                return True
        return False

    def _view(self):
        if self.target is None:            # --follow auto / 自動追従
            self.followed = pick_followed(self.vans, self.followed)
            u = self.followed.drone or self.followed.incoming
            if u is not None and u.flying:
                return self._drone_view(u)
            return self._van_view(self.followed)
        kind, o = self.target
        if kind == "van":
            return self._van_view(o)
        return self._wreck_view(o) if o.status == CRASHED else self._drone_view(o)

    @staticmethod
    def _van_view(van):
        tf = van.car.get_transform()
        back = tf.get_forward_vector() * -14.0
        return carla.Transform(tf.location + back + carla.Location(z=9.0),
                               carla.Rotation(pitch=-25.0, yaw=tf.rotation.yaw))

    @staticmethod
    def _drone_view(u):
        x, y, z = u.world_pos()
        r = math.radians(u.yaw)
        return carla.Transform(carla.Location(x - 6.0 * math.cos(r), y - 6.0 * math.sin(r), z + 3.0),
                               carla.Rotation(pitch=-22.0, yaw=u.yaw))

    @staticmethod
    def _wreck_view(u):
        x, y, z = u.world_pos()
        return carla.Transform(carla.Location(x - 4.0, y - 3.0, z + 3.2),
                               carla.Rotation(pitch=-32.0, yaw=37.0))


def find_addresses(wmap, spacing=40.0, grid=25.0):
    """Fallback delivery points for maps with few buildings (highways, rural).

    A sidewalk spot beside an outer driving lane stands in for a building.
    建物の少ないマップ（高速道路・郊外）用の代替配送地点。最外側車線の
    横の歩道を建物の代わりにする。
    """
    out, seen = [], set()
    for wp in wmap.generate_waypoints(spacing):
        if wp.lane_type != carla.LaneType.Driving or wp.is_junction:
            continue
        side = None
        for get_neighbor in (lambda w: w.get_left_lane(), lambda w: w.get_right_lane()):
            s = get_neighbor(wp)
            for _ in range(4):
                if s is None or s.lane_type == carla.LaneType.Sidewalk:
                    break
                if s.lane_type == carla.LaneType.Driving:
                    s = None
                    break
                s = get_neighbor(s)
            if s and s.lane_type == carla.LaneType.Sidewalk:
                side = s
                break
        drop = None
        if side is not None:
            drop = tup(side.transform.location)
        else:
            sw = wmap.get_waypoint(wp.transform.location, project_to_road=True,
                                   lane_type=carla.LaneType.Sidewalk)
            if sw and wp.transform.location.distance(sw.transform.location) <= 20.0:
                drop = tup(sw.transform.location)
            else:
                yaw_rad = math.radians(wp.transform.rotation.yaw)
                drop = (wp.transform.location.x - 2.5 * math.sin(yaw_rad),
                        wp.transform.location.y + 2.5 * math.cos(yaw_rad),
                        wp.transform.location.z)
        key = (round(drop[0] / grid), round(drop[1] / grid))
        if key in seen:
            continue
        seen.add(key)
        out.append((drop, tup(wp.transform.location)))
    return out


def setup_stations(world, client, wmap, buildings, reachable, args, rng, cars=()):
    """The LastMile charging stations: the spots picked on the dashboard map
    (manual) or ``--charge-stations`` placed by the sim (random), each checked
    for free ground (charge_sites.Finder), spawned. Returns (Stations or None,
    [{"x", "y", "why"}] for the picked spots that could not be used).
    充電ステーションの配置（手動の地点、またはランダム）と生成。"""
    if not world.get_blueprint_library().filter(charge_sites.STATION_BP):
        print("  charging stations: %s is not installed (delivery/assets/build_lastmilehub.sh "
              "--install); road-side chargers instead / 充電ステーション未導入" % charge_sites.STATION_BP,
              flush=True)
        return None, []
    # every LastMile vehicle in the sim must be able to drive in, turn and fit
    # 全ての LastMile 車両が入れること（車幅・旋回・屋根の高さ）
    vehicles = []
    for kind, car in cars:
        bb = car.bounding_box
        vehicles.append({"kind": kind, "half_w": bb.extent.y, "half_l": bb.extent.x,
                         "height": bb.location.z + bb.extent.z})
    finder = charge_sites.Finder(world, wmap, buildings, reachable=reachable, keep_left=keep_left_map(wmap),
                                 vehicles=vehicles)
    if finder.unfit:
        print("  charging stations: %s / 充電ステーションに入れない車両" % finder.unfit, flush=True)
    sites, sources, errors = [], [], []
    if args.charge_mode == "manual":
        for p in zonelib.load(zonelib.map_key(wmap.name)).get("charge", []):
            site, why = finder.at(p["x"], p["y"], p.get("yaw"), taken=[q["centre"] for q in sites])
            if site is not None:
                sites.append(site)
                sources.append("manual")
            else:
                errors.append({"x": p["x"], "y": p["y"], "why": why})
                print("  charging station at (%.0f, %.0f) left out: %s / 設置不可" % (p["x"], p["y"], why),
                      flush=True)
    else:
        sites = finder.random(max(args.charge_stations, 0), rng)
        sources = ["random"] * len(sites)
        if len(sites) < args.charge_stations:
            print("  charging stations: only %d of %d fit on free ground beside the roads / 空き地不足"
                  % (len(sites), args.charge_stations), flush=True)
    return charge_sites.Stations(world, client, sites, sources), errors


def spawn_vans(world, wmap, rng, n, min_sep=40.0, pod=False, lmp=False):
    """n delivery vans at spawn points at least ``min_sep`` apart (if possible).

    ``lmp``: the LastMile pod vehicle; ``pod``: the PodV2 vehicle, or its
    hidden carrier car (delivery/pod.py).
    """
    lib = world.get_blueprint_library()
    if lmp:
        bp = lib.find(lmp_pod.VEHICLE)
    elif pod and lib.filter(podlib.POD_VEHICLE):
        bp = lib.find(podlib.POD_VEHICLE)       # the pod is its own vehicle
    elif pod:
        bp = lib.find(podlib.CARRIER)           # pod prop on a hidden car
        bp.set_attribute("color", podlib.CARRIER_COLOR)
    else:
        bp = next((lib.filter(b)[0] for b in VAN_BLUEPRINTS if lib.filter(b)), None)
        bp = bp or rng.choice(list(lib.filter("vehicle.*")))
        if bp.has_attribute("color"):
            bp.set_attribute("color", "255,255,255")
    bp.set_attribute("role_name", "delivery")
    points = lane_spawn_points(wmap)   # on today's lanes / 現在の車線上
    rng.shuffle(points)
    cars = []
    for sep in (min_sep, 10.0):         # relax spacing on small maps / 狭いマップでは緩める
        for sp in points:
            if len(cars) >= n:
                break
            if any(sp.location.distance(c.get_transform().location) < sep for c in cars):
                continue
            car = world.try_spawn_actor(bp, sp)
            if car is not None:
                cars.append(car)
        if len(cars) >= n:
            break
    if len(cars) < n:
        for c in cars:
            c.destroy()
        raise RuntimeError("only %d free spawn points for %d vans / 配送車を配置できません"
                           % (len(cars), n))
    return cars


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=2000)
    p.add_argument("--map", default="Town03",
                   help='map to load if not already loaded; "" keeps the current map / '
                        '未ロードならロードするマップ（"" で現在のマップ）')
    p.add_argument("--vans", type=int, default=1,
                   help="number of delivery vans (each with its own drone) / 配送車の台数")
    p.add_argument("--seed", type=int, default=None)
    p.add_argument("--passive", action="store_true",
                   help="another client (run_pipeline's traffic) owns the tick: never "
                        "change world settings or tick, just follow / "
                        "他クライアントが tick を持つ（設定変更・tick をしない）")
    p.add_argument("--async", dest="async_", action="store_true",
                   help="don't drive the world tick (default for airsim). The van's "
                        "steering is unstable in async mode / 非同期モード（車の操舵が不安定）")
    p.add_argument("--sync", action="store_true",
                   help="tick the world at 20 Hz; the default for the kinematic drone / "
                        "20 Hz 同期モード（kinematic の既定）")
    p.add_argument("--speedup", type=float, default=1.0,
                   help="sim speed vs real time in sync mode, 0 = as fast as possible "
                        "(on the RTX 5080 that is ~8x) / 実時間に対する速度（0 = 最大速）")
    p.add_argument("--duration", type=float, default=0.0,
                   help="stop after N sim seconds (0 = until Ctrl-C) / N 秒後に終了")
    p.add_argument("--max-batches", type=int, default=0,
                   help="stop after N completed batches (all vans) / 全車で N バッチ完了で終了")
    # orders / 注文
    p.add_argument("--min-orders", type=int, default=3)
    p.add_argument("--max-orders", type=int, default=5)
    p.add_argument("--min-gap", type=float, default=20.0,
                   help="min sim seconds between batches for ONE van; divided by the "
                        "number of vans / 1 台あたりのバッチ間隔の最小 [s]（台数で割る）")
    p.add_argument("--max-gap", type=float, default=90.0,
                   help="max sim seconds between batches for ONE van; divided by the "
                        "number of vans / 1 台あたりのバッチ間隔の最大 [s]")
    p.add_argument("--order-backlog", type=int, default=0,
                   help="at most this many batches wait for a van (0 = one per van); "
                        "an idle van with nothing waiting gets a batch within seconds / "
                        "待機バッチの上限（0 = 車の台数）")
    p.add_argument("--first-now", action="store_true",
                   help="release the first batch immediately / 最初のバッチを即時発生")
    p.add_argument("--drone-prob", type=float, default=0.5,
                   help="probability an order is a drone order / ドローン注文の確率")
    p.add_argument("--drone-radius", type=float, default=150.0,
                   help="drone orders of one batch lie within this many metres / "
                        "同一バッチのドローン注文の範囲 [m]")
    # car / 車
    p.add_argument("--speed", type=float, default=30.0, help="km/h")
    p.add_argument("--ignore-lights", action="store_true")
    p.add_argument("--traffic-profile", default="auto",
                   help="country driving rules for the vans from traffic_profiles.json: "
                        "auto (maps.json decides: Japanese maps get japan -- 止まれ full stop, "
                        "yield to walkers, lane speed cap), none, or a profile name "
                        "/ 配送車の交通ルール（auto / none / 名前）")
    p.add_argument("--handover-s", type=float, default=8.0,
                   help="face-to-face handover time / 対面受け渡し時間 [s]")
    p.add_argument("--jack-s", type=float, default=2.0,
                   help="pod jacks: time to lower or raise / ジャッキの昇降時間 [s]")
    p.add_argument("--hatch-s", type=float, default=2.5,
                   help="pod roof hatch: time to open or close / ハッチの開閉時間 [s]")
    p.add_argument("--side-door-s", type=float, default=1.2,
                   help="pod side door: time to open or close / サイドドアの開閉時間 [s]")
    p.add_argument("--arm-s", type=float, default=2.0,
                   help="pod side arm: time to slide out or in / サイドアームの伸縮時間 [s]")
    p.add_argument("--arrive-radius", type=float, default=4.0)
    p.add_argument("--hub-radius", type=float, default=120.0,
                   help="search radius around the drone-order centroid / 重心周辺の探索半径 [m]")
    # drone / ドローン
    p.add_argument("--drone", choices=("kinematic", "airsim"), default="kinematic")
    p.add_argument("--pod-model", choices=("auto", "lastmile", "pod"), default="auto",
                   help="which pod: lastmile = the LastMile pod (LMP-4, 4 drones, the HTML's "
                        "animations; delivery/assets/build_lastmile.sh), pod = the earlier "
                        "PodV2; auto = LastMile when it is installed / ポッドの種類")
    p.add_argument("--lmp-speed", type=float, default=1.0,
                   help="LastMile pod: speed of its animations (the HTML's timing x this) "
                        "/ アニメーション速度")
    p.add_argument("--meet-hold-s", type=float, default=90.0,
                   help="LastMile pod: battery a drone keeps for waiting at the meeting point "
                        "(seconds of hovering) / 合流地点での待機分の電池 [s]")
    p.add_argument("--pod-kwh", type=float, default=10.0,
                   help="pod battery size: charging drones and packs drains it in proportion "
                        "/ ポッド電池容量 [kWh]")
    p.add_argument("--drone-wh", type=float, default=500.0,
                   help="one drone battery pack / ドローン電池 1 個 [Wh]")
    p.add_argument("--shell", choices=lmp_pod.SHELL_MODES, default="solid",
                   help="LastMile pod shell at start: solid, cutaway, xray, frame (X in the "
                        "CARLA window cycles it, or the dashboard's Settings > Delivery sim) / 外殻の表示")
    p.add_argument("--no-box-stations", action="store_true",
                   help="LastMile pods set their own box down beside them at face-to-face stops "
                        "instead of using box stations placed in the world / 世界の箱を使わない")
    p.add_argument("--box-spacing", type=float, default=110.0,
                   help="minimum distance between box stations in the world, m / 箱の最小間隔 [m]")
    p.add_argument("--tooltips", action="store_true",
                   help="start with the markers, labels and drone trails shown in the CARLA "
                        "window (the dashboard toggles them) / マーカー・ラベル・軌跡を表示して開始")
    p.add_argument("--stock-models", action="store_true",
                   help="ignore the pod/drone models: stock van + drawn drone / "
                        "ポッド・ドローンモデルを使わない")
    p.add_argument("--cruise-alt", type=float, default=0.0,
                   help="fixed cruise height above the van for every leg; 0 = per leg, "
                        "from what is under it / 全区間固定の巡航高度（0 = 区間ごとに算出）")
    p.add_argument("--clearance", type=float, default=8.0,
                   help="fly this far above the tallest obstacle on a leg / "
                        "区間上の最も高い障害物からの余裕 [m]")
    p.add_argument("--min-alt", type=float, default=15.0,
                   help="never cruise lower than this above the ground / 最低巡航高度 [m]")
    p.add_argument("--corridor", type=float, default=5.0,
                   help="obstacles this close to a leg's path count / "
                        "経路からこの距離以内の障害物を考慮 [m]")
    p.add_argument("--drone-speed", type=float, default=12.0, help="m/s horizontal")
    p.add_argument("--climb-speed", type=float, default=4.0, help="m/s vertical")
    p.add_argument("--hover-s", type=float, default=4.0,
                   help="hover time at each drop / 投下地点でのホバリング時間 [s]")
    p.add_argument("--reload-s", type=float, default=5.0,
                   help="time to load the next parcel back in the van / "
                        "車内で次の荷物を積む時間 [s]")
    # pod EV battery / ポッドの EV バッテリー
    p.add_argument("--pod-drain-km", type=float, default=5.0,
                   help="pod battery used per km driven, %% / 走行 1 km あたりの消費 [%%]")
    p.add_argument("--pod-idle-drain", type=float, default=0.02,
                   help="pod systems (computer, sensors, climate), %%/s / 車載システムの消費 [%%/s]")
    p.add_argument("--pod-charge-rate", type=float, default=0.0,
                   help="fixed charging rate at an EV charger, %%/s (0 = from the charger power and the "
                        "pack size: --station-kw / --roadside-kw, --lmp-kwh / --hub-kwh) / 固定充電速度 [%%/s]")
    p.add_argument("--pod-low", type=float, default=30.0,
                   help="an idle pod below this goes to charge, %% / これ未満で充電へ [%%]")
    p.add_argument("--pod-critical", type=float, default=8.0,
                   help="a busy pod below this hands its orders back and charges, %% / "
                        "これ未満で注文を返却して充電 [%%]")
    p.add_argument("--pod-charge-to", type=float, default=95.0,
                   help="charge up to this, %% / 充電目標 [%%]")
    p.add_argument("--pod-start-min", type=float, default=45.0,
                   help="pods start between this and 100 %% / 初期残量の下限 [%%]")
    p.add_argument("--ev-chargers", type=int, default=0,
                   help="road-side EV chargers for the pods (0 = one per two pods) / "
                        "ポッド用 EV 充電器の数")
    p.add_argument("--no-ev-yard", action="store_true",
                   help="charge pods at road-side chargers even where the map has an "
                        "off-road charging yard (Town05's courtyard) / 路側充電器を使う")
    p.add_argument("--charge-stations", type=int, default=0,
                   help="LastMile charging stations (the web viewer's charge dock) on free ground "
                        "beside the roads; they replace the road-side chargers (random mode) / "
                        "充電ステーションの数")
    p.add_argument("--station-kw", type=float, default=50.0,
                   help="charging-station bay power (DC, the web viewer's 50 kW cabinet) / ステーションの電力")
    p.add_argument("--roadside-kw", type=float, default=22.0,
                   help="road-side / yard charger power (AC) / 路側充電器の電力")
    p.add_argument("--lmp-kwh", type=float, default=0.0,
                   help="LMP-4 traction battery (0 = --pod-kwh) / LMP-4 の電池容量")
    p.add_argument("--hub-kwh", type=float, default=10.0, help="LastMile Hub traction battery / ハブの電池容量")
    p.add_argument("--pad-kw", type=float, default=3.0, help="Mini Pod pad charger at a station / パッドの電力")
    p.add_argument("--charge-mode", choices=("random", "manual"), default="random",
                   help="random: the sim places --charge-stations itself; manual: the spots "
                        "picked on the dashboard map (delivery_zones.json charge) / 配置方法")
    p.add_argument("--pod-recover-s", type=float, default=120.0,
                   help="a pod that ran flat is towed to a charger after this / "
                        "電池切れのポッドが牽引されるまで [s]")
    # drone pool and batteries / 機体プールとバッテリー
    p.add_argument("--standby-drones", type=int, default=2,
                   help="spare drones waiting at the charge station / "
                        "充電ステーションの待機機の数")
    p.add_argument("--battery-drain", type=float, default=0.25,
                   help="battery use in level flight, %%/s (more with a parcel, "
                        "climbing or wind) / 水平飛行時の消費 [%%/s]")
    p.add_argument("--charge-slow", type=float, default=0.08,
                   help="charging in the van, %%/s / 車内充電 [%%/s]")
    p.add_argument("--charge-fast", type=float, default=1.5,
                   help="charging at the station, %%/s / ステーション充電 [%%/s]")
    p.add_argument("--battery-reserve", type=float, default=15.0,
                   help="never plan a trip that lands below this, %% / 計画上の残量下限 [%%]")
    p.add_argument("--max-charge-wait", type=float, default=20.0,
                   help="charge in the van up to this long before swapping the drone / "
                        "交換せず車内で充電して待つ最大時間 [s]")
    p.add_argument("--drone-wait-max", type=float, default=240.0,
                   help="a van with no drone gives up its drone orders after this / "
                        "機体が来ない場合に注文を諦めるまでの時間 [s]")
    p.add_argument("--recover-s", type=float, default=90.0,
                   help="a crashed drone is back at the station after this / "
                        "墜落機がステーションに戻るまでの時間 [s]")
    # wind / 風
    p.add_argument("--wind", type=float, default=3.0, help="base wind, m/s / 基本風速")
    p.add_argument("--wind-dir", type=float, default=None,
                   help="direction the wind blows towards, deg (CARLA yaw); "
                        "default random / 風が吹いていく方向 [度]")
    p.add_argument("--gust-rate", type=float, default=0.5,
                   help="random gusts per minute (0 = only dashboard gusts) / 毎分の突風回数")
    p.add_argument("--gust-max", type=float, default=14.0,
                   help="strongest random gust, m/s / ランダム突風の最大 [m/s]")
    p.add_argument("--drone-max-wind", type=float, default=11.0,
                   help="strongest wind a drone can hold its position against, m/s / "
                        "ドローンが抗える最大風速")
    p.add_argument("--wind-hold", type=float, default=10.0,
                   help="no drone takes off while the wind is above this, m/s "
                        "(0 = never hold) / この風速を超える間は離陸しない")
    p.add_argument("--rain-ground", type=float, default=20.0,
                   help="CARLA precipitation (0-100) from which every flying drone "
                        "comes down and none takes off (weather_cycle.py makes the "
                        "rain; 0 = rain never grounds) / この雨量でドローンは墜落・離陸禁止")
    # failures / 故障
    p.add_argument("--motor-fail-rate", type=float, default=1.0,
                   help="motor failures per flight hour / 飛行 1 時間あたりのモーター故障")
    p.add_argument("--parcel-wind", type=float, default=12.0,
                   help="above this wind a hovering drone can lose its parcel, m/s / "
                        "荷物が外れうる風速")
    p.add_argument("--lost-drift", type=float, default=0.0,
                   help="0 (default): a gust only pushes a drone off its path and it flies "
                        "back; it crashes only if it hits something. >0: pushed further than "
                        "this (m), it is lost / 0: 衝突しない限り経路に戻る。>0: この距離で喪失 [m]"),
    p.add_argument("--no-random-failures", action="store_true",
                   help="no motor failures or torn-off parcels (collisions, flat "
                        "batteries and being blown away still count) / 確率的な故障を無効化")
    p.add_argument("--live-dir", default=live.DEFAULT_DIR,
                   help="where the live state for the dashboard goes / ダッシュボード用の状態")
    p.add_argument("--no-live", action="store_true",
                   help="don't write the live state / ライブ状態を書かない")
    p.add_argument("--airsim-port", type=int, default=41451)
    p.add_argument("--airsim-vehicle", default="")
    p.add_argument("--airsim-offset", type=float, nargs=3, metavar=("OX", "OY", "OZ"),
                   default=None, help="CARLA->NED offsets, map-specific / マップ固有のオフセット")
    p.add_argument("--airsim-calibrate", action="store_true",
                   help="measure the offsets at startup / 起動時にオフセットを計測")
    # background traffic / 背景交通
    p.add_argument("--vehicles", type=int, default=30,
                   help="autopilot cars driving around / 周囲を走る自動運転車の台数")
    p.add_argument("--walkers", type=int, default=40,
                   help="pedestrians walking around / 歩行者の人数")
    p.add_argument("--walker-cross", type=float, default=0.0,
                   help="share of walkers crossing roads; the van does not brake for "
                        "them / 道路を横断する歩行者の割合（配送車は歩行者に減速しない）")
    p.add_argument("--tm-port", type=int, default=8000)
    p.add_argument("--follow", choices=("free", "auto", "none"), default="free",
                   help="CARLA window camera: free (fly it yourself; Q / E in the CARLA "
                        "window lock onto pods and drones), auto (follow the busy van), "
                        "none (never touch it) / 観戦カメラ")
    p.add_argument("--follow-van", type=int, default=0,
                   help="start with the camera locked on this van; 0 = don't / "
                        "起動時にカメラを固定する車（0 = 固定しない）")
    p.add_argument("--no-log", action="store_true")
    # LastMile hubs with Mini Pods / LastMile ハブとミニポッド
    p.add_argument("--lmh", type=int, default=0,
                   help="LastMile hubs (LMH-3) that deliver with 3 Mini Pods each, ids after the "
                        "drone pods' / ミニポッドで配達する LastMile ハブの台数")
    p.add_argument("--lmh-speed", type=float, default=25.0,
                   help="hub speed cap, km/h (it runs on the Prius drivetrain) / ハブの速度上限")
    p.add_argument("--minipod-speed", type=float, default=1.6, help="m/s (Japan: <= 6 km/h)")
    p.add_argument("--minipod-range", type=float, default=1500.0,
                   help="metres per pod trip, out and back / 1 往復の距離上限")
    p.add_argument("--minipod-kwh", type=float, default=1.0, help="pod pack / ポッドの電池")
    p.add_argument("--minipod-drain-km", type=float, default=5.0, help="%% of the pack per km")
    p.add_argument("--minipod-inset", type=float, default=0.4,
                   help="pod path: metres inside the kerb-side lane edge / 車線端からの距離")
    p.add_argument("--minipod-fail-rate", type=float, default=0.3,
                   help="random pod faults (motor, lidar, collision) per hour of driving")
    p.add_argument("--minipod-recover-s", type=float, default=90.0,
                   help="a crashed pod is brought back to its bay after this")
    p.add_argument("--minipod-fail-timeout", type=float, default=120.0,
                   help="a crashed pod's order fails if no pod takes it over within this")
    a = p.parse_args(argv)
    if not 1 <= a.min_orders <= a.max_orders:
        p.error("need 1 <= --min-orders <= --max-orders")
    if a.vans < 0 or a.lmh < 0 or a.vans + a.lmh < 1:
        p.error("need --vans >= 1 or --lmh >= 1")
    if a.drone == "airsim":
        p.error("the airsim backend has no battery / charge-station model yet; "
                "use the default kinematic drone / airsim は未対応")
    if a.drone == "airsim" and a.sync:
        p.error("--sync would freeze AirSim's clock; run airsim asynchronously")
    if a.sync and (a.async_ or a.passive):
        p.error("--sync cannot be combined with --async or --passive")
    # BasicAgent's PID assumes a steady 20 Hz; in async mode missed or uneven
    # frames make it over-steer and leave the road, so sync is the default.
    # Passive: someone else (run_pipeline's traffic) already runs it at 20 Hz.
    # BasicAgent の PID は 20 Hz 前提。非同期ではフレーム落ちで操舵が過大になり
    # 道路を外れるため同期を既定とする。passive では他者が 20 Hz で進める。
    a.sync = a.drone == "kinematic" and not a.async_ and not a.passive
    a.follow_van = min(max(a.follow_van, 0), a.vans + a.lmh)
    return a


def pick_followed(vans, current):
    """Van for the spectator: stay on the current one while it is busy, else
    jump to one whose drone is flying, else to any busy van.

    観戦カメラの対象：今の車が作業中なら維持、そうでなければドローン飛行中の
    車、次に作業中の車へ切り替える。
    """
    if current is not None and current.state != IDLE:
        return current
    flying = [v for v in vans if v.drone is not None and v.drone.flying]
    busy = [v for v in vans if v.state != IDLE]
    return (flying or busy or [current or vans[0]])[0]


def wait_tick(world, sync, timeout=10.0, patience=120.0):
    """Advance (sync) or follow (passive/async) one world tick.

    A passive follower tolerates the tick master pausing -- run_pipeline
    restarts it when it freezes -- for up to ``patience`` seconds.
    1 tick 進める（同期）か追従する（受動・非同期）。受動側は tick マスタの
    停止（run_pipeline がフリーズ時に再起動する）を ``patience`` 秒まで待つ。
    """
    if sync:
        return world.tick()
    deadline = time.monotonic() + patience
    while True:
        try:
            return world.wait_for_tick(timeout).frame
        except RuntimeError:
            if time.monotonic() > deadline:
                raise
            print("  waiting for the world to tick... / tick 待ち", flush=True)


def main(argv=None):
    args = parse_args(argv)
    if args.seed is None:
        # random run, but printed so it can be replayed with --seed
        # 毎回ランダム。--seed で再現できるよう表示する
        args.seed = random.randrange(1000000)
    print("seed %d (replay with --seed %d) / シード" % (args.seed, args.seed), flush=True)
    rng = random.Random(args.seed)
    client = carla.Client(args.host, args.port)
    client.set_timeout(60.0)
    world = client.get_world()
    if args.map and not world.get_map().name.endswith(args.map):
        if args.passive:
            raise SystemExit("--passive never loads maps; the current map is %s"
                             % world.get_map().name)
        print("loading %s ... / %s をロード中" % (args.map, args.map), flush=True)
        world = client.load_world(args.map)
    wmap = world.get_map()

    original = world.get_settings()
    if args.sync:
        s = world.get_settings()
        s.synchronous_mode, s.fixed_delta_seconds = True, 0.05
        world.apply_settings(s)
    elif args.passive and not original.synchronous_mode:
        print("  WARNING: the world is asynchronous and nobody steps it at a fixed "
              "rate; the vans may steer poorly / 非同期ワールドのため操舵が不安定な"
              "可能性あり", flush=True)
    if args.passive:
        # a crashed earlier run of ours may have left vans behind
        # 以前の異常終了で残った配送車を片付ける
        stale = [a for a in world.get_actors().filter("vehicle.*")
                 if a.attributes.get("role_name") == "delivery"]
        props = [a for a in world.get_actors().filter("static.prop.*")
                 if a.type_id in podlib.ALL_PROPS + lmp_pod.ALL_PROPS
                 or a.type_id.startswith("static.prop.lmh")
                 or a.attributes.get("role_name") in (lmh_hub.ROLE_POD, lmh_hub.ROLE_PARCEL)]
        for a in props + stale:
            a.destroy()
        if stale:
            print("  removed %d leftover delivery vans / 残っていた配送車を削除" % len(stale))
        if args.vehicles or args.walkers:
            print("  --passive: background traffic is left to run_pipeline "
                  "/ 背景交通は run_pipeline に任せる", flush=True)
            args.vehicles = args.walkers = 0

    print("analysing %s ... / マップ解析中" % wmap.name, flush=True)
    buildings, roof_z = find_buildings(world, wmap)
    grp = SafeRoutePlanner(wmap, 2.0)
    if grp.runaway_segments:
        print("  route planner: %d road segments could not be sampled (imported road logic); "
              "linked end to end / サンプリング不能な区間 %d" % (grp.runaway_segments, grp.runaway_segments),
              flush=True)
    hubs = hub_candidates(wmap)
    # only stops a van can drive to and away from (one-way streets leading
    # nowhere: imported maps) / 行けて出られる停車地のみ（一方通行の行き止まり）
    can_reach = boxes.reachable(grp)
    n_hubs = len(hubs)
    hubs = [h for h in hubs if can_reach(loc(h))]
    if len(hubs) < n_hubs:
        print("  %d stopping places left out: a van could not drive both to and away from them "
              "/ 出入りできない停車候補 %d" % (n_hubs - len(hubs), n_hubs - len(hubs)), flush=True)
    if len(buildings) < 20:
        # few real buildings (highway / rural maps): add roadside addresses
        # 建物が少ないマップ（高速・郊外）では道路沿いの地点を追加
        extra = find_addresses(wmap)
        for drop, curb in extra:
            buildings.append(Building(len(buildings) + 1, "roadside address", drop, curb))
        print("  only a few buildings here: added %d roadside addresses "
              "/ 建物が少ないため道路沿いの地点を %d 件追加" % (len(extra), len(extra)))
    if len(buildings) < args.max_orders:
        raise SystemExit("only %d delivery points on %s / 配送地点が不足"
                         % (len(buildings), wmap.name))
    if not hubs:
        raise SystemExit("no road points to park on / 停車できる道路地点が無い")
    if roof_z < 0 and buildings:
        roof_z = max(b.drop[2] for b in buildings) + 15.0
    print("  %d delivery points, %d hub candidates, tallest roof z=%.1f m"
          % (len(buildings), len(hubs), roof_z), flush=True)

    use_lmp = (not args.stock_models and args.pod_model in ("auto", "lastmile")
               and lmp_pod.available(world))
    if args.pod_model == "lastmile" and not use_lmp:
        print("  LastMile pod not installed (delivery/assets/build_lastmile.sh) "
              "/ LastMile ポッド未導入", flush=True)
    use_pod = not args.stock_models and (use_lmp or podlib.available(world))
    if not args.stock_models and not use_pod:
        print("  pod model not installed (delivery/assets/build_models.sh): stock van "
              "/ ポッド未導入のため標準の車", flush=True)
    cars = spawn_vans(world, wmap, rng, args.vans, pod=use_pod, lmp=use_lmp)
    actors = list(cars)
    # a just-spawned actor reports (0, 0, 0) until the next tick; planning from
    # that sent the van's route from the map origin, straight through buildings
    # スポーン直後のアクターは次の tick まで (0, 0, 0) を返す。そのまま計画すると
    # 経路がマップ原点から引かれ、車が建物に突っ込む
    wait_tick(world, args.sync)
    pods = [None] * len(cars)
    # LastMile hubs: spawned once the vans have a real position (one tick)
    # LastMile ハブ（配送車の位置が確定してから）
    lmh_cars, lmh_bp, lmh_real = [], None, False
    if args.lmh:
        lmh_cars, lmh_bp, lmh_real = lmh_hub.spawn_hub_cars(
            world, wmap, rng, args.lmh, avoid=[c.get_location() for c in cars])
        actors += lmh_cars
        wait_tick(world, args.sync)
        print("  LastMile hubs: %d x %s%s / LastMile ハブ" % (
            len(lmh_cars), lmh_bp, "" if lmh_real else
            " (LMH-3 not imported yet: the LMP-4 vehicle stands in, Mini Pods are %s boxes)"
            % lmh_hub.FALLBACK_POD_BP), flush=True)
    if (use_pod and cars) or lmh_cars:
        # let the cars land first: they spawn in the air, and a carrier car is
        # measured from its wheels / 着地を待つ（車は空中にスポーンする）
        for _ in range(30):
            wait_tick(world, args.sync)
        if use_lmp and cars:
            pods = [lmp_pod.LastMilePod(world, client, car) for car in cars]
            print("  vans: the LastMile pod %s: %d moving parts each, 4 drones on deck, "
                  "box-station side port; animations from LastMile_Pod_3D.html at x%.1f "
                  "/ LastMile ポッド" % (lmp_pod.VEHICLE, len(pods[0].parts), args.lmp_speed),
                  flush=True)
        elif use_pod and cars:
            pods = [podlib.Pod(world, client, car) for car in cars]
        for p in pods:
            if p is not None:
                actors[:0] = p.actors   # destroyed before its car / 車より先に削除
        if use_pod and cars and not use_lmp:
            print("  vans: %s (jacks, roof hatch%s) / 配送ポッド" % (
                "the delivery pod vehicle, %s" % podlib.POD_VEHICLE if pods[0].standalone else
                "delivery pod on a hidden %s" % podlib.CARRIER,
                ", side port" if pods[0].side_port else ""), flush=True)
    log = EventLog(not args.no_log)
    drones = []
    livew = None
    fleet = None
    traffic = None
    boxmgr = None
    t = 0.0
    missed = 0
    try:
        if args.vehicles or args.walkers:
            traffic = Traffic(client, world, args.tm_port, args.seed, args.sync)
            nv = traffic.spawn_vehicles(args.vehicles,
                                        avoid=[c.get_location() for c in cars])
            nw = traffic.spawn_walkers(args.walkers, args.walker_cross)
            print("  traffic: %d cars, %d walkers / 交通: 車 %d 台, 歩行者 %d 人"
                  % (nv, nw, nv, nw), flush=True)
        obstacles = find_obstacles(world)
        print("  %d obstacles for drone altitude planning / 高度計画用の障害物"
              % len(obstacles), flush=True)
        sky = types.SimpleNamespace()
        sky.check = SkyCheck(world, obstacles, make_surface_fn(world), args)
        lmh_hubs = list(hubs)           # the hubs drive in: no open sky needed / ハブは空不要
        open_hubs = [h for h in hubs if sky.check.open_sky(h)] if cars else list(hubs)
        print("  %d of %d hub candidates have open sky for the drone / "
              "上空が開けた停車候補" % (len(open_hubs), len(hubs)), flush=True)
        hubs = open_hubs or hubs
        sky.bm = BatteryModel(hover=args.battery_drain, slow=args.charge_slow,
                              fast=args.charge_fast, reserve=args.battery_reserve)
        sky.pem = PodEnergyModel(per_km=args.pod_drain_km, idle=args.pod_idle_drain,
                                 charge=args.pod_charge_rate, low=args.pod_low,
                                 critical=args.pod_critical, target=args.pod_charge_to,
                                 pod_kwh=args.pod_kwh, drone_wh=args.drone_wh)
        sky.wind = Wind(random.Random(args.seed + 7), base_speed=args.wind,
                        base_dir=args.wind_dir, gust_rate=args.gust_rate,
                        gust_max=max(args.gust_max, 5.5))
        per_van = 4 if use_lmp else 1          # LastMile: 4 drones on deck / 甲板に 4 機
        n_units = per_van * len(cars) + max(args.standby_drones, 0)
        # pads for the standbys and one drone per pod sent to charge (LastMile
        # drones charge on their pod's deck) / 待機機と各車 1 機分のパッド
        station = choose_station(wmap, buildings, len(cars) + max(args.standby_drones, 0) + 1,
                                 sky.check)
        if station is None:
            raise SystemExit("no clear sidewalk for the charge station / 充電所の場所が無い")
        sky.yard = None if args.no_ev_yard else yardlib.yard_for(wmap.name)
        sky.yard_lane = None            # the pod moving in the yard right now
        sky.yard_lanes = {}             # yard / station -> the vehicle moving there
        sky.stations, sky.station_bays, sky.station_errors = None, {}, []
        if args.charge_stations > 0 or args.charge_mode == "manual":
            sky.stations, sky.station_errors = setup_stations(world, client, wmap, map_buildings(world), can_reach,
                                                              args, random.Random(args.seed + 31),
                                                              [("lmp", c) for c in cars] + [("lmh", c) for c in lmh_cars])
        if sky.stations is not None and sky.stations.sites:
            # each station: bay 03 for an LMP-4 pod, bay 04 for a hub / ベイ 03・04
            spots, kinds = [], []
            for st in sky.stations.sites:
                for kind, b in sorted(st["bays"].items()):
                    sky.station_bays[len(spots)] = dict(
                        b, kind="station", bay_kind=kind, station=st["id"], z=st["frame"].z,
                        name="charging station %d bay %s" % (st["id"], "03" if kind == "lmp" else "04"))
                    spots.append((b["bay"][0], b["bay"][1], st["frame"].z))
                    kinds.append(kind)
            sky.chargers = EvChargers(spots, kinds)
            sky.charger_yaw = [sky.station_bays[i]["bay_yaw"] for i in range(len(spots))]
            sky.yard = None
            sky.queue_slot = {}
            for st in sky.stations.sites:          # waiting bays off the lane / 車線外の待機ベイ
                if st.get("queue"):
                    sky.station_bays[("q", st["id"])] = dict(st["queue"], kind="queue", station=st["id"],
                                                             z=st["frame"].z,
                                                             name="charging station %d waiting bay" % st["id"])
                    sky.queue_slot[st["id"]] = None
            sky.stations.turn_r = 2.8
            # drones charge on the stations' decks (sidewalk pads as overflow) / ドローンはデッキで充電
            station = DeckStation(sky.stations, extra=station.pads)
            sky.walker_guard = charge_sites.WalkerGuard(world, sky.stations)   # round, not through / 迂回
            print("  %d LastMile charging stations (%s): bay 03 for the LMP-4 pods, bay 04 for the hubs "
                  "/ 充電ステーション %d 基" % (len(sky.stations.sites), args.charge_mode,
                                            len(sky.stations.sites)), flush=True)
        elif sky.yard:
            bays = [(x, y, 0.15) for x, y in sky.yard["bays"]]
            sky.chargers = EvChargers(bays)
            sky.charger_yaw = [0.0] * len(bays)
            print("  %d EV charging bays for the pods in the %s (off the road) / "
                  "路外の EV 充電ヤード" % (len(bays), sky.yard["name"]), flush=True)
        else:
            n_ev = args.ev_chargers or max(1, (len(cars) + 1) // 2)
            spots = choose_chargers(wmap, hubs, station.location, n_ev)
            sky.chargers = EvChargers([s for s, _ in spots])
            sky.charger_yaw = [y for _, y in spots]
            print("  %d road-side EV chargers for the pods near the drone station / "
                  "ポッド用 EV 充電器" % len(spots), flush=True)
        hz = Hazards(random=not args.no_random_failures, motor_per_hour=args.motor_fail_rate,
                     parcel_wind=args.parcel_wind, lost_drift=args.lost_drift)
        units = [DroneUnit(i + 1, world, client, random.Random(args.seed + 100 + i), sky.bm, hz,
                           max_wind=args.drone_max_wind, h_speed=args.drone_speed,
                           v_speed=args.climb_speed, hover_s=args.hover_s,
                           model=not args.stock_models, kind="lmp" if use_lmp else "pod")
                 for i in range(n_units)]
        drones = units
        sky.drones = DroneFleet(units, station, sky.bm, sky.check.flight_z, log,
                                recover_s=args.recover_s, wind_hold=args.wind_hold,
                                pod_draw=sky.pem.drone_ratio, rain_ground=args.rain_ground)
        print("  %d drones (%d in vans + %d standby): %s"
              % (n_units, per_van * len(cars), n_units - per_van * len(cars),
                 units[0].blueprint_id if units else "-"))
        print("  charge station: %d pads at (%.0f, %.0f) / 充電ステーション"
              % ((len(station.pads),) + station.location[:2]), flush=True)
        print("  wind: %.1f m/s towards %.0f deg, %.1f random gusts/min up to %.0f m/s; "
              "drones hold against %.0f m/s / 風" % (
                  args.wind, sky.wind.base_dir, args.gust_rate, args.gust_max,
                  args.drone_max_wind), flush=True)
        if args.cruise_alt:
            print("  cruise altitude: fixed %.0f m above the van / 固定巡航高度"
                  % args.cruise_alt, flush=True)
        else:
            print("  cruise altitude: per trip, %.0f m over obstacles, min %.0f m, checked "
                  "by raycast / 往復ごとの巡航高度" % (args.clearance, args.min_alt), flush=True)

        # LastMile box stations in the world (keep-left maps: the side port is
        # on the pod's left, facing the kerb) / 世界の箱型ステーション（左側通行）
        box_targets = []
        if (use_lmp or lmh_cars) and keep_left_map(wmap) and not args.no_box_stations:
            # parcel lockers (LastMile Hub props installed): Mini Pods use the low
            # robot port, LMP-4 pods the upper one / 宅配ロッカー（下段ミニポッド・上段 LMP-4）
            use_lockers = bool(world.get_blueprint_library().filter(boxes.LOCKER_BP))
            sites = boxes.find_box_sites(world, wmap, random.Random(args.seed), spacing=args.box_spacing,
                                         planner=grp, lockers=use_lockers)
            if sky.stations is not None and sky.stations.sites:
                # the props carry no collision, so the box search cannot see the stations
                # ステーションには衝突が無いため距離で除外
                sites = [b for b in sites if all(math.hypot(b.loc[0] - st["centre"][0], b.loc[1] - st["centre"][1])
                                                 > 14.0 for st in sky.stations.sites)]
            boxmgr = boxes.BoxStations(world, client, sites, random.Random(args.seed), lockers=use_lockers)
            box_targets = [Building(100000 + s.id, "box station %d" % s.id, s.loc, s.park, box=s.id)
                           for s in sites if s.id in boxmgr.boxes]
            print("  %d LastMile box stations on the pavements; face-to-face orders go to them "
                  "/ 歩道の箱型ステーション %d 台" % (len(box_targets), len(box_targets)), flush=True)
        gen = planning.OrderGenerator(
            buildings, random.Random(args.seed), min_gap=args.min_gap,
            max_gap=args.max_gap, min_batch=args.min_orders, max_batch=args.max_orders,
            drone_prob=args.drone_prob, drone_radius=args.drone_radius,
            fleet_size=len(cars) + len(lmh_cars), boxes=box_targets)
        vans = [Van(i + 1, args, world, car, hubs, grp, log, obstacles, gen, sky,
                    pod=pod) for i, (car, pod) in enumerate(zip(cars, pods))]
        sky.drones.start(vans)          # drones only for the drone pods / ドローンはポッドのみ
        lmh_shared = None
        if lmh_cars:
            left = keep_left_map(wmap)
            lmh_shared = {"keep_left": left,
                          "router": lmh_hub.KerbRouter(wmap, left, inset=args.minipod_inset),
                          "obstacles": lmh_hub.ObstacleCache(world, exclude=[c.id for c in cars + lmh_cars]),
                          "probe": lmh_hub.MiniPodProbe()}
            lmh_vans = [LmhVan(len(cars) + k + 1, args, world, car, lmh_hubs, grp, log, obstacles, gen, sky,
                               client, 100 + 3 * k, lmh_shared, lmh_real)
                        for k, car in enumerate(lmh_cars)]
            vans += lmh_vans
            lmh_shared["probe"].start(args.host, args.port,
                                      lambda: [p for v in lmh_vans for p in v.crew.pods])
            print("  LastMile hubs %s: 3 Mini Pods each (P%d..P%d), kerb-side paths %s, %.1f m/s, "
                  "%.0f m per trip / ミニポッド" % (
                      ", ".join("V%d" % v.vid for v in lmh_vans), 101, 100 + 3 * len(lmh_vans),
                      "on the left" if left else "on the right", args.minipod_speed, args.minipod_range),
                  flush=True)
        for v in vans:
            v.boxes = boxmgr
        if not args.no_live:
            livew = live.LiveWriter(args.live_dir)
            livew.write_map(export_map(world, wmap, station, sky.chargers, boxmgr, hubs))
            print("  live state for the dashboard: %s / ダッシュボード用" % livew.dir, flush=True)
        fleet = Fleet(args, world, vans, gen, log, sky, livew)
        fleet.boxes = boxmgr
        fleet.lmh_shared = lmh_shared
        if lmh_shared is not None:
            geo = fleet.geo
            for v in vans:
                if getattr(v, "lmh", False):
                    for su in v.suites.values():
                        su.geo = geo
        # Japan profile maps only; any other map: None, no extra call
        # 日本のプロファイルのマップのみ。他のマップでは何もしない
        fleet.set_rules(van_rules.make(world, wmap, args.traffic_profile,
                                       lane_type=carla.LaneType.Driving))
        camera = CameraControl(world, args, vans, sky.drones.units)
        fleet.camera = camera
        if use_lmp:
            def next_shell():
                modes = lmp_pod.SHELL_MODES
                fleet.set_shell(modes[(modes.index(fleet.shell_mode) + 1) % len(modes)])
                print("pod shell: %s / 外殻表示" % fleet.shell_mode, flush=True)
            camera.on_x = next_shell

        stop = {"now": False}
        signal.signal(signal.SIGINT, lambda *_: stop.update(now=True))
        signal.signal(signal.SIGTERM, lambda *_: stop.update(now=True))

        frame = wait_tick(world, args.sync)
        t0 = world.get_snapshot().timestamp.elapsed_seconds
        gen.schedule_from(0.0, immediate=args.first_now)
        log(0.0, "start", detail="%s, %d van(s), %s drone, seed=%s, %s" % (
            wmap.name, len(vans), args.drone, args.seed,
            "passive" if args.passive else ("sync" if args.sync else "async")))
        last = 0.0
        wall0 = time.monotonic()
        while not stop["now"]:
            new = wait_tick(world, args.sync)
            if not args.sync and new > frame + 1:
                missed += new - frame - 1     # we fell behind the tick master
            frame = new
            t = world.get_snapshot().timestamp.elapsed_seconds - t0
            dt, last = max(t - last, 0.0), t
            if args.sync and args.speedup > 0:
                # world.tick() returns as soon as the server has stepped, so
                # without this the run races ahead of the wall clock
                # world.tick() はサーバが進めた直後に戻るため、これが無いと
                # 実時間より速く進んでしまう
                ahead = t / args.speedup - (time.monotonic() - wall0)
                if ahead > 0:
                    time.sleep(ahead)
            fleet.step(t, dt)
            camera.step()
            if args.duration and t >= args.duration:
                break
            if args.max_batches and fleet.batches >= args.max_batches:
                break
    finally:
        if fleet is not None:
            log(t, "stop")
            summary = fleet.summary(t)
            if not args.sync:
                summary["missed_ticks"] = missed
            log.close(summary)
            print(json.dumps(summary, indent=2, ensure_ascii=False))
            if log.dir:
                print("log: %s" % log.dir)
        else:
            log.close({})
        if traffic is not None:
            traffic.destroy()
        for d in drones:
            d.destroy()
        for v in (fleet.vans if fleet is not None else []):
            v.cleanup()
        if boxmgr is not None:
            boxmgr.destroy()
        if getattr(sky, "stations", None) is not None:
            sky.stations.destroy()
        if livew is not None:
            livew.close()
        for a in actors:
            a.destroy()
        if args.sync:
            world.apply_settings(original)


if __name__ == "__main__":
    main()
