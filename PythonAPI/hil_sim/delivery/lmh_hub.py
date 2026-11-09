"""The LastMile Hub (LMH-3) and its Mini Pods in CARLA (mixed into delivery_sim's Van).

The hub is a vehicle driven like every other van (BasicAgent, the Japan rules,
the off-road guard, reverse-out). At a batch it drives to a DISTRIBUTION POINT
at the kerb (the user's ``dist`` points from data/delivery_zones.json, else
the kerb point nearest the orders), parks, runs the HTML's sequence (scan,
divert, lift, load, release) and sends up to three Mini Pods out with one
parcel each. Pods drive the kerb side of the road (the 路側帯 strip inside
the kerb-side lane edge; delivery/lmh_rig.kerb_path), cross roads only at
junctions, and deliver in front of the place (parcel pushed out onto the
ground) or into a box station (pushed into its intake). The hub waits for
them, or drives to a MEETING POINT (the user's ``meet`` points) where they
come back, reverse up into their bays and charge from the hub. At a charging
station (bay 04) the low pods charge on the Mini Pod pads behind it instead
(station_pods_*; delivery_sim calls them).

All the pod logic is server-free in lmh_rig (HubCrew, MiniPod); this module
gives it the world (routes on the lane graph, obstacles, ground heights by a
rate-limited ray thread) and shows it (props: the hub's moving parts and the
pods, posed by ApplyTransform like lmp_pod).

Until the LMH assets are imported (needs CARLA stopped), the hub is the LMP-4
vehicle with its shell and each Mini Pod a cardboard box prop (logged once).

LastMile ハブとミニポッドの CARLA 側。経路・障害物・地面高さを状態機械に渡し、
プロップを動かす。充電ステーションではポッドがパッドで充電する。アセット未導入時は LMP-4 の車体と段ボール箱で代用する。
"""

import copy
import math
import random
import threading
import time

import carla

from delivery import boxes as boxes_mod
from delivery import lmh_rig as L
from delivery import lmp_pod
from delivery import lmp_rig
from delivery import minipod_sensors as MS
from delivery import planning
from delivery.planning import Stop

LMH_STOP, LMH_WORK = "lmh", "lmh_work"
FALLBACK_POD_BP = "static.prop.box01"          # 0.66 x 0.66 x 0.70 m: close to the pod / 代用
FALLBACK_PARCEL_BP = "static.prop.parcel"
FALLBACK_HUB_SHELL = ("static.prop.lmpshell", "static.prop.lmpshellstreet")
ROLE_POD, ROLE_PARCEL, ROLE_HUB = "lmh_minipod", "lmh_parcel", "lmh_hub"
HIDDEN = carla.Transform(carla.Location(z=-60.0))
GROUND = ("Roads", "Sidewalks", "Ground", "Terrain", "RoadLines", "NONE", "None")
PARCEL_LINGER_S = 10.0
MEET_SNAP = 15.0


def _tf(p):
    return carla.Transform(carla.Location(p[0], p[1], p[2]),
                           carla.Rotation(pitch=p[3], yaw=p[4], roll=p[5]))


def _has(seq, o):
    return any(x is o for x in seq)


# -- blueprints / ブループリント -----------------------------------------------------
def hub_blueprint(world, parts=None):
    """(blueprint, real) for the hub: the LMH vehicle if imported, else the
    LMP-4 vehicle, else a stock van. / ハブの車体（未導入なら代用）。"""
    lib = world.get_blueprint_library()
    parts = parts or L.load_parts()
    want = parts["hub"].get("blueprint", L.HUB_BP)
    if lib.filter(want):
        return lib.find(want), True
    for b in (lmp_pod.VEHICLE, "vehicle.mercedes.sprinter", "vehicle.volkswagen.t2_2021"):
        if lib.filter(b):
            return lib.find(b), False
    return lib.filter("vehicle.*")[0], False


def spawn_hub_cars(world, wmap, rng, n, avoid=(), min_sep=40.0):
    """n hub vehicles on lane spawn points away from ``avoid`` (other vans)."""
    from lane_spawns import lane_spawn_points
    bp, real = hub_blueprint(world)
    if bp.has_attribute("role_name"):
        bp.set_attribute("role_name", "delivery")
    if bp.has_attribute("color") and not real:
        bp.set_attribute("color", "235,235,235")
    pts = lane_spawn_points(wmap)
    rng.shuffle(pts)
    cars = []
    others = list(avoid)
    for sep in (min_sep, 10.0):
        for sp in pts:
            if len(cars) >= n:
                break
            if any(sp.location.distance(c) < sep for c in others):
                continue
            car = world.try_spawn_actor(bp, sp)
            if car is not None:
                cars.append(car)
                others.append(sp.location)
        if len(cars) >= n:
            break
    return cars, bp.id, real


# -- routes on the lane graph / 車線グラフ上の経路 ------------------------------------
class KerbRouter(object):
    """Kerb-side pod paths on the road network: A* over waypoints, both
    directions (a sidewalk robot may go against the traffic), kerb-side lanes
    only outside junctions (no mid-block crossing), any lane inside one
    (crossing at the junction). / 路肩側の経路（交差点でのみ横断）。"""

    def __init__(self, wmap, keep_left=True, inset=0.4, step=3.0):
        self.wmap, self.left, self.inset, self.step = wmap, keep_left, inset, step
        self.calls, self.ms = 0, 0.0

    def _kerb_lane(self, wp):
        side = wp.get_left_lane() if self.left else wp.get_right_lane()
        return side is None or side.lane_type != carla.LaneType.Driving

    def _snap(self, p):
        wp = self.wmap.get_waypoint(carla.Location(p[0], p[1], p[2]), project_to_road=True,
                                    lane_type=carla.LaneType.Driving)
        for _ in range(4):
            if wp is None or wp.is_junction or self._kerb_lane(wp):
                break
            nxt = wp.get_left_lane() if self.left else wp.get_right_lane()
            if nxt is None or nxt.lane_type != carla.LaneType.Driving:
                break
            wp = nxt
        return wp

    @staticmethod
    def _key(wp, step):
        return (wp.road_id, wp.section_id, wp.lane_id, int(round(wp.s / step)))

    def route(self, a, b, max_m=1600.0):
        """[(x, y, z)] from a to b (b included), or None. / a から b への経路。"""
        t0 = time.perf_counter()
        self.calls += 1
        try:
            wa, wb = self._snap(a), self._snap(b)
            if wa is None or wb is None:
                return None
            gl = wb.transform.location
            step = self.step
            gkey = (wb.road_id, wb.section_id, wb.lane_id)

            def nb(wp):
                out = []
                for w in wp.next(step) + wp.previous(step):
                    if w.lane_type != carla.LaneType.Driving:
                        continue
                    if not w.is_junction and not self._kerb_lane(w):
                        continue
                    out.append((self._key(w, step), w,
                                w.transform.location.distance(wp.transform.location) or step))
                return out

            def goal(wp):
                if (wp.road_id, wp.section_id, wp.lane_id) == gkey and abs(wp.s - wb.s) <= step * 0.75:
                    return True
                return wp.transform.location.distance(gl) < step * 0.8

            h = lambda wp: wp.transform.location.distance(gl)
            if a is not None and math.hypot(a[0] - b[0], a[1] - b[1]) < 6.0:
                return [tuple(b)]
            wps = L.astar((self._key(wa, step), wa), nb, goal, h, max_cost=max_m, max_nodes=6000)
            if wps is None:
                return None
            rows = [(w.transform.location.x, w.transform.location.y, w.transform.location.z,
                     w.transform.rotation.yaw, w.lane_width) for w in wps]
            pts = L.kerb_path(rows, self.left, self.inset)
            # leave the first kerb point that is behind ``a`` out / 後ろの点を省く
            while len(pts) > 1 and math.hypot(pts[1][0] - a[0], pts[1][1] - a[1]) < \
                    math.hypot(pts[0][0] - pts[1][0], pts[0][1] - pts[1][1]):
                pts.pop(0)
            return pts + [tuple(b)]
        finally:
            self.ms = (time.perf_counter() - t0) * 1000.0


# -- obstacles / 障害物 ---------------------------------------------------------------
class ObstacleCache(object):
    """Walkers and vehicles near the pods, refreshed at 10 Hz (the actor list
    every 2 s; positions from the client's cached state, no RPC per actor).
    ポッド周辺の歩行者・車両（10 Hz で更新）。"""

    def __init__(self, world, exclude=()):
        self.world, self.exclude = world, set(exclude)
        self._list_t, self._t = -1e9, -1e9
        self.walkers, self.vehicles, self.ext = [], [], {}
        self.near = []

    def refresh(self, t, centres):
        if t - self._t < 0.1:
            return
        self._t = t
        if t - self._list_t > 2.0:
            self._list_t = t
            acts = self.world.get_actors()
            self.walkers = list(acts.filter("walker.pedestrian.*"))
            self.vehicles = [a for a in acts.filter("vehicle.*") if a.id not in self.exclude]
        out = []
        if not centres:
            self.near = out
            return
        for kind, actors in (("walker", self.walkers), ("vehicle", self.vehicles)):
            for a in actors:
                try:
                    tf = a.get_transform()
                except RuntimeError:
                    continue
                x, y = tf.location.x, tf.location.y
                if not any(abs(x - c[0]) < 30.0 and abs(y - c[1]) < 30.0 for c in centres):
                    continue
                if kind == "vehicle":
                    e = self.ext.get(a.id)
                    if e is None:
                        bb = a.bounding_box
                        e = self.ext[a.id] = (bb.extent.x, bb.extent.y)
                    out.append(("vehicle", x, y, tf.rotation.yaw, e[0], e[1]))
                else:
                    out.append(("walker", x, y, 0.0, 0.3, 0.3))
        self.near = out

    def around(self, p, r=6.0):
        return [o for o in self.near if abs(o[1] - p[0]) < r and abs(o[2] - p[1]) < r]


class MiniPodProbe(object):
    """Rate-limited raycasts for the pods (ground height under each moving
    pod, and lidar-like ranges front / rear / left / right), on a thread with
    its own client, like the drones' SensorProbe: in synchronous mode a ray
    can wait ~50 ms for the frame, and the sim loop must never wait.
    ポッド用のレイ（地面高さと前後左右の距離）。別スレッドで実行。"""

    def __init__(self, max_rays=16, reach=10.0):
        self.cache = {}
        self.max_rays, self.reach = max_rays, reach
        self._i = {"g": 0, "d": 0}
        self.rays = 0
        self.world = None

    def start(self, host, port, pods_fn, period=0.25):
        def loop():
            try:
                c = carla.Client(host, port)
                c.set_timeout(10.0)
                self.world = c.get_world()
            except RuntimeError as e:
                print("[delivery] minipod probe: no client (%s)" % e, flush=True)
                return
            while True:
                t0 = time.time()
                try:
                    self.run(list(pods_fn()))
                except RuntimeError:
                    time.sleep(1.0)
                except Exception as e:      # never take the sim down / 止めない
                    print("[delivery] minipod probe: %s" % e, flush=True)
                    time.sleep(1.0)
                time.sleep(max(period - (time.time() - t0), 0.02))
        threading.Thread(target=loop, name="minipod-probe", daemon=True).start()

    def _ground(self, p):
        hits = self.world.cast_ray(carla.Location(p[0], p[1], p[2] + 1.2),
                                   carla.Location(p[0], p[1], p[2] - 1.0))
        best = None
        for h in hits:
            z = h.location.z
            # skip the pod's own body (up to 0.7 m) and anything far below
            if z > p[2] + 0.25 or z < p[2] - 0.8 or str(h.label) not in GROUND:
                continue
            best = z if best is None else max(best, z)
        return best

    def _dir(self, p, yaw):
        c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
        a = carla.Location(p[0] + c * 0.45, p[1] + s * 0.45, p[2] + 0.45)
        b = carla.Location(p[0] + c * self.reach, p[1] + s * self.reach, p[2] + 0.45)
        best = None
        for h in self.world.cast_ray(a, b):
            d = math.hypot(h.location.x - p[0], h.location.y - p[1])
            best = d if best is None or d < best else best
        return best

    def run(self, pods):
        live = [p for p in pods if p.out()]
        if not live or self.world is None:
            return
        n, cast = len(live), 0
        for _ in range(n):
            if cast >= self.max_rays // 2:
                break
            p = live[self._i["g"] % n]
            self._i["g"] += 1
            c = self.cache.setdefault(p.id, {})
            c["ground"] = self._ground(p.pos)
            c["ground_xy"] = (p.x, p.y)
            cast += 1
        jobs = [(p, d) for p in live for d in (("front", 0.0), ("rear", 180.0), ("left", -90.0), ("right", 90.0))]
        for _ in range(len(jobs)):
            if cast >= self.max_rays:
                break
            p, (k, off) = jobs[self._i["d"] % len(jobs)]
            self._i["d"] += 1
            self.cache.setdefault(p.id, {})[k] = self._dir(p.pos, p.yaw + off)
            cast += 1
        self.rays += cast


# -- props / プロップ ------------------------------------------------------------------
class PodBody(object):
    """The props of one Mini Pod: body (+ wheels, lid, shutter, flag and the
    parcel with the real model). Attached to the hub while docked (it rides
    along), free in the world while out; CARLA can't detach, so a switch
    respawns them. With the stand-in box the docked pod is hidden.
    ミニポッドのプロップ（格納中はハブに取付、外では独立）。"""

    def __init__(self, world, client, parts, real, bay=1):
        self.world, self.client, self.parts, self.real = world, client, parts, real
        self.lib = world.get_blueprint_library()
        mp = parts["minipod"]
        bodies = mp.get("body_variants") or [mp.get("body")]
        lids = mp.get("lid_variants") or []
        self.body_bp = bodies[(bay - 1) % len(bodies)] if real else FALLBACK_POD_BP
        if real and not self.lib.filter(self.body_bp):
            self.body_bp = mp.get("body")
        self.part_defs = []
        for p in mp.get("parts", []):
            p = dict(p)
            if p.get("name") == "Lid" and lids and self.lib.filter(lids[(bay - 1) % len(lids)]):
                p["prop"] = lids[(bay - 1) % len(lids)]           # P1..P3 decals / 号機の表示
            if real and self.lib.filter(p.get("prop", "-")):
                self.part_defs.append(p)
        # a shutter rolled up is a second prop (CARLA can't scale) / 巻き上げ状態は別プロップ
        self.swaps = {}
        for p in self.part_defs:
            m = p.get("motion") if isinstance(p.get("motion"), dict) else {}
            sw = (m.get("runtime") or {}).get("swap_prop")
            if sw and self.lib.filter(sw):
                self.swaps[p["name"]] = sw
        drop = (mp.get("deliver") or {}).get("ground_drop", {}).get("loc")
        self.drop_x = abs(drop[0]) if drop else L.POD_HL + 0.5
        pc = parts.get("parcels", {}).get("small", "")
        self.parcel_bp = pc if real and self.lib.filter(pc) else FALLBACK_PARCEL_BP
        self.body, self.part_actors, self.parcel = None, {}, None
        self.mode = None                 # "attached" | "free"
        self._last = {}

    def _bp(self, name, role):
        bp = self.lib.find(name)
        if bp.has_attribute("role_name"):
            bp.set_attribute("role_name", role)
        return bp

    def destroy(self):
        for a in list(self.part_actors.values()) + [self.parcel, self.body]:
            if a is not None:
                try:
                    a.destroy()
                except RuntimeError:
                    pass
        self.body, self.part_actors, self.parcel, self._last = None, {}, None, {}

    def place(self, mode, car=None, rel=None, world_tf=None):
        """(Re)spawn attached to ``car`` at ``rel`` (hub frame pose) or free at
        ``world_tf``. / 取付・独立の切替。"""
        self.destroy()
        self.mode = mode
        bp = self._bp(self.body_bp, ROLE_POD)
        if mode == "attached":
            tf = _tf(rel) if (self.real and rel is not None) else HIDDEN
            self.body = self.world.try_spawn_actor(bp, tf, attach_to=car)
        else:
            self.body = self.world.try_spawn_actor(bp, world_tf)
        if self.body is None:
            return
        for p in self.part_defs:
            pose = L.motion_pose(p, 0.0)
            a = self.world.try_spawn_actor(self._bp(p["prop"], ROLE_POD), _tf(pose) if pose else HIDDEN,
                                           attach_to=self.body)
            if a is not None:
                self.part_actors[p["name"]] = a
            sw = self.swaps.get(p["name"])
            if sw:
                a = self.world.try_spawn_actor(self._bp(sw, ROLE_POD), HIDDEN, attach_to=self.body)
                if a is not None:
                    self.part_actors[p["name"] + "#up"] = a

    def cmds(self, pod, t, world_tf=None, out=None):
        """ApplyTransform commands for this tick (only what changed)."""
        out = [] if out is None else out
        if self.body is None:
            return out
        if world_tf is not None:
            out.append(carla.command.ApplyTransform(self.body.id, world_tf))
        if not self.part_actors:
            return out
        vals = {"Lid": pod.lid.at(t), "Shutter": pod.shutter.at(t), "FlagMast": pod.flag.at(t),
                "Flag": pod.flag.at(t)}
        for p in self.part_defs:
            a = self.part_actors.get(p["name"])
            if a is None:
                continue
            n = p["name"]
            m = p.get("motion") if isinstance(p.get("motion"), dict) else {}
            if n.startswith("Wheel"):
                # left / right by the wheel's side (skid steer) / 左右で回転が違う
                side = pod.odo_l if float(p.get("loc", (0, 0, 0))[1]) < 0 else pod.odo_r
                if m.get("per") == "metre":
                    spin = float(m.get("amount", -674.0)) * side
                else:
                    spin = -math.degrees(side / L.WHEEL_R)
                pose = L.motion_pose(p, 0.0, spin_deg=spin % 360.0)
            else:
                pose = L.motion_pose(p, vals.get(n, 0.0))
            self._put(a, pose, out)
            up = self.part_actors.get(n + "#up")
            if up is not None:            # rolled-up shutter: shown instead / 巻上げ時に表示
                rest = L.motion_pose(dict(p, motion=[]), 0.0)
                self._put(up, rest if pose is None else None, out)
        return out

    def _put(self, a, pose, out):
        key = tuple(round(v, 3) for v in pose) if pose else "hidden"
        if self._last.get(a.id) != key:
            self._last[a.id] = key
            out.append(carla.command.ApplyTransform(a.id, _tf(pose) if pose else HIDDEN))


class HubProps(object):
    """The hub's moving parts (gates, lifts, ramp doors, plugs, pins) as props
    attached to the hub vehicle; with the LMP-4 stand-in, its shell instead.
    ハブの可動部品（代用時は LMP-4 の外殻）。"""

    def __init__(self, world, client, car, parts, real):
        self.world, self.client, self.car, self.real = world, client, car, real
        lib = world.get_blueprint_library()
        self.actors, self._last = {}, {}
        self.defs = []
        if real:
            for p in parts["hub"].get("parts", []):
                if not lib.filter(p.get("prop", "-")):
                    continue
                pose = L.motion_pose(p, 0.0)
                a = world.try_spawn_actor(lib.find(p["prop"]), _tf(pose) if pose else HIDDEN, attach_to=car)
                if a is not None:
                    self.actors[p["name"]] = a
                    self.defs.append(p)
        elif car.type_id == lmp_pod.VEHICLE:
            for name in FALLBACK_HUB_SHELL:
                if lib.filter(name):
                    a = world.try_spawn_actor(lib.find(name), carla.Transform(), attach_to=car)
                    if a is not None:
                        self.actors[name] = a

    def cmds(self, mech, t, out):
        for p in self.defs:
            a = self.actors.get(p["name"])
            pose = L.motion_pose(p, mech.value(p["name"], t))
            key = tuple(round(v, 3) for v in pose) if pose else "hidden"
            if a is not None and self._last.get(a.id) != key:
                self._last[a.id] = key
                out.append(carla.command.ApplyTransform(a.id, _tf(pose) if pose else HIDDEN))
        return out

    def destroy(self):
        for a in self.actors.values():
            try:
                a.destroy()
            except RuntimeError:
                pass
        self.actors = {}


# -- the van side / 配送車側 ---------------------------------------------------------
class LmhVanMixin(object):
    """Methods for a delivery_sim.Van that is a LastMile Hub. The van is also
    the crew's ``env`` (see lmh_rig.HubCrew). / LastMile ハブとしての Van。"""

    def lmh_init(self, client, pod_base, shared, real_hub):
        a = self.args
        self.lmh, self.kind = True, "lmh"
        self.client = client
        self.parts = L.load_parts()
        self.real_hub = real_hub
        self.shared = shared                  # router, obstacles, probe (one per fleet)
        self.pem = copy.copy(self.pem)        # own copy: critical is lifted while pods are out
        self._crit = self.pem.critical
        self.power = L.PodPower(kwh=a.minipod_kwh, per_km=a.minipod_drain_km, range_m=a.minipod_range)
        self.hz = L.PodHazards(random_on=not a.no_random_failures, per_hour=a.minipod_fail_rate,
                               recover_s=a.minipod_recover_s, fail_timeout=a.minipod_fail_timeout)
        rng = random.Random(a.seed + 700 + self.vid)
        pods = [L.MiniPod(pod_base + b, self.vid, b, random.Random(rng.random()), self.power, self.hz,
                          speed=a.minipod_speed) for b in (1, 2, 3)]
        for p in pods:
            p.battery = rng.uniform(80.0, 100.0)
        self.crew = L.HubCrew(self.vid, pods, self.power, self.hz, keep_left=shared["keep_left"])
        real_pod = bool(self.world.get_blueprint_library().filter(self.parts["minipod"].get("body", "-")))
        self.real_pod = real_pod
        self.bodies = {p.id: PodBody(self.world, client, self.parts, real_pod, p.bay) for p in pods}
        self.hubprops = HubProps(self.world, client, self.car, self.parts, real_hub)
        self.suites = {p.id: MS.MiniPodSuite(p.id, shared.get("geo")) for p in pods}
        self.parcels = []                     # (actor, due) parcels left in front of places
        self._t = 0.0
        self._tf = self.car.get_transform()
        e = self.car.bounding_box.extent
        self._half = (max(e.x, L.HUB_HALF_L), max(e.y, L.HUB_HALF_W))
        self.lmh_delivered = 0
        for p in pods:
            self._place_body(p)

    # -- env for the crew / 状態機械への情報
    def hub_pose(self):
        tf = self._tf
        return (tf.location.x, tf.location.y, tf.location.z, tf.rotation.yaw)

    def parked(self):
        if self.state in ("driving", "yard_in", "yard_out", "docking", "stopping"):
            return False
        v = self.car.get_velocity()
        return math.hypot(v.x, v.y) < 0.3

    def bay(self, n):
        return L.bay_geometry(self.parts, n)

    def hub_half(self):
        """Half length / width of the real hub vehicle (stand-in: the LMP-4,
        4.3 x 2.0 m). / 実車の半長・半幅。"""
        return self._half

    def ground(self, x, y, z):
        return z

    def route(self, pod, a, b):
        return self.shared["router"].route(a, b, max_m=self.power.range_m)

    def mission(self, order):
        """Front drop or delivery box for ``order``. / 前置きか宅配ボックスか。"""
        bld = order.building
        box = getattr(bld, "box", None)
        boxes = getattr(self, "boxes", None)
        if box is not None and boxes is not None and boxes.has_locker(box):
            # the parcel locker's robot port (mini.js m_aptB): turn, reverse
            # up to the port with the rear, flap + shutter, belt pushes it in
            # ロッカーのロボットポートへ後退し、ベルトで押し込む
            site = boxes.boxes[box].site
            r = math.radians(site.yaw)
            ux, uy = -math.sin(r), math.cos(r)                 # locker +y: its front / 正面
            px, py, _ = boxes_mod.LOCKER["port"]
            c = boxes.locker_point(box, (px, py + L.POD_HL + 0.01, 0.0))
            lock = (c.x, c.y, c.z)
            stop = (lock[0] + ux * L.BOX_BACK, lock[1] + uy * L.BOX_BACK, lock[2])
            return {"kind": L.ORDER_BOX, "stop": stop, "face_yaw": math.degrees(math.atan2(uy, ux)),
                    "back": L.BOX_BACK, "drop": lock, "box": box, "locker": True}
        if box is not None and boxes is not None and box in boxes.boxes:
            site = boxes.boxes[box].site
            r = math.radians(site.yaw)
            ux, uy = -math.sin(r), math.cos(r)                 # box +y: its intake side / 投入口側
            try:
                f = lmp_rig.parts()["LMPStationFlap"]["instances"]["BoxStation_IntakeFlap"]
            except (KeyError, OSError, ValueError):
                f = (0.0, 0.3, 0.9)
            fx, fy = f[0], abs(f[1])
            c = boxes.world_point(box, (fx, fy + L.POD_HL + 0.02, 0.0))
            lock = (c.x, c.y, site.loc[2])
            stop = (lock[0] + ux * L.BOX_BACK, lock[1] + uy * L.BOX_BACK, lock[2])
            return {"kind": L.ORDER_BOX, "stop": stop, "face_yaw": math.degrees(math.atan2(uy, ux)),
                    "back": L.BOX_BACK, "drop": tuple(site.loc), "box": box}
        d, c = bld.drop, bld.curb
        dx, dy = c[0] - d[0], c[1] - d[1]
        n = math.hypot(dx, dy) or 1.0
        ux, uy = dx / n, dy / n                                # road-ward / 道路側
        stop = (d[0] + ux * L.FRONT_GAP, d[1] + uy * L.FRONT_GAP, d[2])
        return {"kind": L.ORDER_FRONT, "stop": stop, "face_yaw": math.degrees(math.atan2(uy, ux)),
                "back": 0.0, "drop": tuple(d), "box": None}

    def blocked(self, pod, pts):
        return L.blocked(pts, self.shared["obstacles"].around(pod.pos))

    def slow(self, pod):
        return any(o[0] == "walker" and math.hypot(o[1] - pod.x, o[2] - pod.y) < L.SLOW_R
                   for o in self.shared["obstacles"].around(pod.pos))

    def door(self, hub, bay):
        return self.crew.mech.door[bay - 1].at(self._t)

    # -- planning / 計画
    def lmh_assign(self, batch, t):
        zs = self.fleet.zones if self.fleet is not None else None
        dists = zs.dist_of(self.vid) if zs is not None else []
        meets = zs.meet_all(self.vid) if zs is not None else []
        reach = self.power.range_m / 2.0
        cands = L.snap(dists, self.hubs) if dists else self.hubs
        groups = L.plan_groups(batch, cands, reach, fixed=bool(dists))
        stops = [Stop(LMH_STOP, tuple(p), os) for p, os in groups]
        stops = planning.order_stops(self.car_pos(), stops, self.road_dist)
        mpts = L.snap(meets, self.hubs) if meets else []
        for s in stops:
            s.meet_pt = L.choose_meet(s.orders, mpts, reach, here=s.location) if mpts else None
            s.resume = False
        need = self.batch_need(stops)
        if self.energy.level < need:
            self.log(t, "batch_declined", batch=batch[0].batch,
                     detail="hub at %.0f%%, the batch needs ~%.0f%%: charging first" % (self.energy.level, need))
            return False
        self.pending, self.stops = list(batch), stops
        self.log(t, "plan", batch=batch[0].batch, detail=" -> ".join(
            "dist(%s)" % ",".join(str(o.id) for o in s.orders) for s in stops))
        for s in stops:
            self._log_dist(t, s, dists)
        self.drive_to_next(t)
        return True

    def _log_dist(self, t, s, dists):
        kinds = collections_counter(self.mission(o)["kind"] for o in s.orders)
        self.log(t, "hub_dist", batch=s.batch, pos=s.location,
                 detail="distribution point (%s) for #%s (%s)%s" % (
                     "yours" if dists else "auto", ",".join(str(o.id) for o in s.orders),
                     ", ".join("%d %s" % (n, k) for k, n in kinds.items()),
                     "; pods meet the hub at (%.0f, %.0f)" % s.meet_pt[:2] if s.meet_pt else ""))

    def lmh_arrive(self, t):
        s = self.stops[0]
        self.hold()
        self.log(t, "arrive", batch=s.batch, pos=self.car_pos(),
                 detail="meeting point" if s.resume else "distribution point")
        self.stages = ["stopping", LMH_WORK]
        self._begin(t, "stopping")

    def lmh_begin(self, t):
        self.state, self.timer = LMH_WORK, 0.0
        s = self.stops[0]
        if s.resume:
            self.log(t, "hub_meet", batch=s.batch, pos=self.car_pos(),
                     detail="parked at the meeting point; %d pod(s) coming back" % len(self.crew.pods_out()))
            return
        out = {id(p.order) for p in self.crew.pods if p.order is not None}
        orders = [o for o in s.orders if _has(self.pending, o) and id(o) not in out]
        self.crew.start(t, orders, s.meet_pt)

    def lmh_work(self, t, dt):
        self.hold()
        s = self.stops[0] if self.stops else None
        crew = self.crew
        if s is None:
            self.state = "idle"
            return
        if crew.busy():
            m = s.meet_pt
            if (m is not None and not s.resume and not crew.queue and not crew.rescues and
                    crew.pods_out() and crew.can_leave(t) and
                    math.hypot(m[0] - self._tf.location.x, m[1] - self._tf.location.y) > MEET_SNAP):
                s.location, s.resume = tuple(m), True
                crew.hub_leaving(t, self, m)
                self.log(t, "hub_meet", batch=s.batch, pos=m,
                         detail="pods out: the hub drives to the meeting point (%.0f, %.0f)" % m[:2])
                self.drive_to_next(t)
            return
        rest = crew.deferred
        crew.deferred = []
        rest = [o for o in rest if _has(self.pending, o)]
        if rest:
            groups = L.plan_groups(rest, self.hubs, self.power.range_m / 2.0)
            new = [Stop(LMH_STOP, tuple(p), os) for p, os in groups]
            for n in new:
                n.meet_pt, n.resume = None, False
                self._log_dist(t, n, [])
            self.stops[1:1] = new
        self.leave_stop(t)

    # -- charging station: pods on the pads behind bay 04 / 充電ステーションのパッド
    def station_pods_begin(self, t, stations, site_id):
        """The hub starts charging in bay 04 of station ``site_id`` (parked on
        the bay pose, physics off): every docked pod under L.PAD_START % drives
        to a Mini Pod pad (M1..M3, one each), the bay doors opening as at a stop.
        ``stations`` = charge_sites.Stations. / ベイ 04 で充電開始：低いポッドをパッドへ。"""
        self._stations = stations
        car = getattr(self, "car", None)
        if car is not None:
            try:
                self._tf = car.get_transform()            # the bay pose, not last tick's / ベイの姿勢
            except RuntimeError:
                pass
        fr = stations.sites[site_id - 1]["frame"]
        poses = list(stations.pad_poses(site_id))[:3]
        evs = self.crew.station_begin(t, self, site_id, (fr.x, fr.y, fr.z, fr.yaw), poses,
                                      stations.pad_lane(site_id))
        for e in evs:
            self._crew_event(t, e)

    def station_pods_step(self, t, dt):
        """Every tick while the hub is at the station: pads (pins, plug,
        charging at the pad power) and the pods leaving them. The pods' own
        motion runs in lmh_tick. / 毎 tick：パッドの充電と発進（走行は lmh_tick）。"""
        st = self.crew.station
        if st is None:
            return
        a = self.args
        # pad_kw at 90 % into the pod pack, %/s / パッド電力（効率 90 %）
        rate = getattr(a, "pad_kw", 3.0) * 0.9 / max(getattr(a, "minipod_kwh", 1.0), 0.05) * 100.0 / 3600.0
        for e in self.crew.station_step(t, dt, self, rate):
            self._crew_event(t, e)
        stations = getattr(self, "_stations", None)
        if stations is None:
            return
        for i, pad in enumerate(st["pads"]):
            try:
                stations.pad(st["site"], i, pad["pins"].at(t), pad["plug"].at(t))
            except (RuntimeError, IndexError):
                break

    def station_pods_recall(self, t):
        """The hub has charged: pods still on pads unplug and come back now."""
        self.crew.station_recall(t)

    def station_pods_done(self, t):
        """True when no pod is out on station business (all docked, doors
        shut); after a recall that also ends the pad session. / 全ポッド格納済みか。"""
        done = self.crew.station_done(t)
        if done and self.crew.station is not None and self.crew.station["recall"]:
            self.crew.station = None
        return done

    # -- every tick / 毎 tick
    def lmh_tick(self, t, dt):
        self._t = t
        self._tf = self.car.get_transform()
        out = bool(self.crew.pods_out())
        self.pem.critical = -1.0 if out else self._crit       # never strand pods / ポッドを置き去りにしない
        # a low hub keeps its pack for driving: its pods top up on the
        # station pads instead / 残量が少ないハブはポッドへ給電しない
        # nor on its way to charge or while charging: the pods top up on the
        # station's pads (web viewer) / 充電に向かう間・充電中も給電しない
        charging = bool(self.stops) and getattr(self.stops[0], "kind", None) == "charge"
        self.crew.feed = self.energy.level > L.HUB_FEED_MIN and not charging
        for e in self.crew.step(t, dt, self):
            self._crew_event(t, e)
        if self.crew.station is not None and self.crew.station_done(t) and not self.parked():
            self.crew.station = None          # the hub left the station without a recall / 呼び戻し無しで出発
        if self.crew.hub_draw:
            self.energy.level = max(self.energy.level - self.crew.hub_draw / max(self.args.pod_kwh, 0.1), 0.0)
            self.crew.hub_draw = 0.0
        self._render(t)
        if self.parcels:
            for a, due in [p for p in self.parcels if p[1] <= t]:
                try:
                    a.destroy()
                except RuntimeError:
                    pass
            self.parcels = [p for p in self.parcels if p[1] > t]

    def _crew_event(self, t, e):
        ev, p, o = e["ev"], e.get("pod"), e.get("order")
        tag = ("P%d" % p.id) if p is not None else ""
        if ev == "pod_out":
            m = p.mission or {}
            self.log(t, "pod_out", o, pos=p.pos, detail="%s from bay %d: #%d, %s%s, %.0f m" % (
                tag, p.bay, o.id, "delivery box %s" % m.get("box") if m.get("kind") == L.ORDER_BOX
                else "front drop", "" if not p._routes or len(p._routes) < 2 else " (taking a crashed pod's parcel)",
                sum(L.path_length(r) for r in p._routes)))
        elif ev == "pod_door":
            self.log(t, "pod_door", o, pos=p.pos, detail="%s at %s" % (
                tag, "the delivery box" if e.get("box") else "the front of the place"))
        elif ev == "pod_handover":
            self._handover(t, p, o, e.get("box"))
        elif ev == "pod_back":
            self.log(t, "pod_back", pos=p.pos, detail="%s returning (%.0f%%)" % (tag, p.battery))
        elif ev == "pod_docked":
            self.log(t, "pod_docked", pos=p.pos, detail="%s in bay %d, charging from the hub (%.0f%%)"
                     % (tag, p.bay, p.battery))
        elif ev == "pod_fault":
            self.log(t, "pod_fault", o or p.order, pos=p.pos, detail="%s: %s (%s)" % (
                tag, e.get("why"), (p.damage or {}).get("why", "")))
        elif ev == "pod_rescue":
            self.log(t, "pod_rescue", o, pos=p.pos, detail="%s took the parcel off the crashed pod" % tag)
        elif ev == "pod_recovered":
            self.log(t, "pod_recovered", pos=self.car_pos(), detail="%s back in bay %d (service crew)" % (tag, p.bay))
        elif ev in ("pod_stuck", "pod_stuck_try"):
            self.log(t, ev, pos=p.pos, detail="%s %s" % (tag, e.get("detail", "")))
        elif ev == "order_failed":
            if _has(self.pending, o):
                self.fail_order(t, o, e.get("reason", "minipod"))
        elif ev == "deferred":
            self.log(t, "pod_range", o, pos=o.drop, detail="#%d is out of pod range from here: another stop" % o.id)
        elif ev == "cycle":
            self.log(t, "hub_load", pos=self.car_pos(), detail="scan, divert, lift, load: %d pod(s)" % e.get("n", 0))
        elif ev == "pod_to_pad":
            st = self.crew.station or {}
            self.log(t, ev, pos=p.pos, detail="%s from bay %d to pad M%d of station %s (%.0f%%)" % (
                tag, p.bay, e.get("pad", 0), st.get("site"), p.battery))
        elif ev == "pod_on_pad":
            self.log(t, ev, pos=p.pos, detail="%s on pad M%d: pins in, plug in (%.0f%%)" % (
                tag, e.get("pad", 0), p.battery))
        elif ev == "pod_off_pad":
            self.log(t, ev, pos=p.pos, detail="%s leaves pad M%d (%s, %.0f%%)" % (
                tag, e.get("pad", 0), e.get("why", ""), p.battery))

    def _handover(self, t, p, o, box):
        m = p.mission or {}
        r = math.radians(p.yaw)
        back = self.bodies[p.id].drop_x                      # deliver.ground_drop: x -0.85
        bx, by = p.x - math.cos(r) * back, p.y - math.sin(r) * back
        if box and getattr(self, "boxes", None) is not None and m.get("box") in self.boxes.boxes:
            self.boxes.store(m["box"], t, locker=bool(m.get("locker")))   # the customer comes for it / 客が受け取りに来る
            pos = m.get("drop") or (bx, by, p.z)
            self.boxes.pose(m["box"], 0.0, 0.0)
        else:
            # the ground under the parcel (the pavement past the kerb), not under the pod
            # 荷物の下の地面（縁石の先の歩道）
            gz = None
            for h in self.world.cast_ray(carla.Location(bx, by, p.z + 1.0), carla.Location(bx, by, p.z - 1.0)):
                if str(h.label) in GROUND and h.location.z < p.z + 0.6:
                    gz = h.location.z
                    break
            if gz is None:
                gz = self.shared["probe"].cache.get(p.id, {}).get("ground")
            z = gz if gz is not None else p.z
            pos = (bx, by, z)
            bp = self.world.get_blueprint_library().find(self.bodies[p.id].parcel_bp)
            if bp.has_attribute("role_name"):
                bp.set_attribute("role_name", ROLE_PARCEL)
            a = self.world.try_spawn_actor(bp, carla.Transform(carla.Location(bx, by, z + 0.02),
                                                               carla.Rotation(yaw=p.yaw)))
            if a is not None:
                self.parcels.append((a, t + PARCEL_LINGER_S))
        self.log(t, "pod_handover", o, pos=pos, detail="P%d: %s" % (
            p.id, "pushed into delivery box %s" % m.get("box") if box else "parcel put down in front of the place"))
        if o is not None and _has(self.pending, o):
            self.lmh_delivered += 1
            self.finish_order(t, o, pos)

    # -- showing it / 表示
    def _place_body(self, p):
        b = self.bodies[p.id]
        if p.status in (L.DOCKED, L.LOADING):
            dock, _ = L.bay_geometry(self.parts, p.bay)
            b.place("attached", car=self.car, rel=(dock[0], dock[1], dock[2], 0.0, dock[3], 0.0))
        else:
            b.place("free", world_tf=self._pod_tf(p))

    def _inside_hub(self, p):
        tf = self._tf
        a, l = L._frame((tf.location.x, tf.location.y), tf.rotation.yaw, (p.x, p.y))
        return abs(a) < self._half[0] + 0.4 and abs(l) < self._half[1] + 0.4

    def _pod_tf(self, p):
        if not self.real_pod and self._inside_hub(p):
            # the stand-in box is solid: inside the hub's body it would shove the
            # hub (it did, 6 m, live) -- hide it below the road until it is out
            # 代用の箱は衝突判定があるため、車体内では路面下に隠す
            return carla.Transform(carla.Location(p.x, p.y, -60.0), carla.Rotation(yaw=p.yaw))
        z = p.z
        c = self.shared["probe"].cache.get(p.id)
        # at a station the floor is a plane over the lawn the ray finds: keep the floor height
        # ステーションの床は芝の上の面：床の高さのまま
        if c and c.get("ground") is not None and c.get("ground_xy") and \
                math.hypot(c["ground_xy"][0] - p.x, c["ground_xy"][1] - p.y) < 1.5 and abs(c["ground"] - z) < 0.6 \
                and p.status not in (L.ROLLING_OUT, L.DOCKING) + L.STATION:
            z = c["ground"]
        return carla.Transform(carla.Location(p.x, p.y, z + 0.01), carla.Rotation(yaw=p.yaw))

    def _render(self, t):
        cmds = []
        for p in self.crew.pods:
            b = self.bodies[p.id]
            want = "attached" if p.status in (L.DOCKED, L.LOADING) else "free"
            if b.mode != want or b.body is None:
                self._place_body(p)
            b.cmds(p, t, world_tf=self._pod_tf(p) if want == "free" else None, out=cmds)
            if p.status == L.HANDOVER and (p.mission or {}).get("kind") == L.ORDER_BOX \
                    and getattr(self, "boxes", None) is not None:
                if p.mission.get("locker"):
                    self.boxes.port(p.mission["box"], p.flap(t))
                else:
                    self.boxes.pose(p.mission["box"], p.flap(t), 0.0)
        self.hubprops.cmds(self.crew.mech, t, cmds)
        if cmds:
            self.client.apply_batch(cmds)

    def lmh_draw(self, dbg, life):
        for p in self.crew.pods:
            if not p.out() and p.status != L.CRASHED:
                continue
            c = carla.Color(255, 40, 40) if p.status in (L.CRASHED, L.STUCK) else carla.Color(0, 200, 255)
            dbg.draw_string(carla.Location(p.x, p.y, p.z + 1.2), "P%d %s %.0f%%" % (p.id, p.status, p.battery),
                            color=c, life_time=life)
            m = p.mission or {}
            if m.get("locker") and getattr(self, "boxes", None) is not None \
                    and p.status in (L.TO_DOOR, L.HANDOVER):
                # the locker's screen (mini.js kScreen) / ロッカーの画面表示
                if p.status == L.TO_DOOR:
                    if math.hypot(p.x - m["stop"][0], p.y - m["stop"][1]) > 3.0:
                        continue
                    txt, col = "ROBOT PORT POD DOCKING", carla.Color(47, 224, 122)
                elif not p.parcel_out:
                    txt, col = "ROBOT PORT RECEIVING", carla.Color(47, 224, 122)
                else:
                    txt, col = "PARCEL IN - CODE SENT", carla.Color(255, 194, 26)
                q = self.boxes.locker_point(m["box"], (0.0, boxes_mod.LOCKER["port"][1], 1.45))
                dbg.draw_string(q, txt, color=col, life_time=life)
        for s in self.stops:
            if s.kind == LMH_STOP:
                dbg.draw_string(carla.Location(s.location[0], s.location[1], s.location[2] + 4.0),
                                "%s HUB %d" % ("MEET" if s.resume else "DIST", self.vid),
                                color=carla.Color(0, 200, 255), life_time=life)

    # -- published state / 公開する状態
    def lmh_van_state(self):
        s = self.stops[0] if self.stops and self.stops[0].kind == LMH_STOP else None
        r1 = lambda v: round(v, 1)
        dist = meet = None
        if s is not None:
            if not s.resume:
                dist = [r1(s.location[0]), r1(s.location[1])]
            m = s.meet_pt if s.meet_pt is not None else (None if s.resume else s.location)
            if s.resume:
                m = s.location
            meet = [r1(m[0]), r1(m[1])] if m is not None else None
        st = self.crew.station
        return {"kind": "lmh", "pods_docked": sum(p.status in (L.DOCKED, L.LOADING) for p in self.crew.pods),
                "dist": dist, "meet": meet, "phase": self.crew.phase if s is not None or self.crew.busy() else "idle",
                "real_model": bool(self.real_hub),
                "station": st["site"] if st is not None else None, "pads": self.crew.pad_states()}

    def lmh_pod_states(self, t):
        out = []
        probe = self.shared["probe"].cache
        hx, hy = self._tf.location.x, self._tf.location.y
        for p in self.crew.pods:
            d = p.state(t)
            b = self.bodies[p.id]
            d["actor"] = b.body.id if b.body is not None else None
            c = probe.get(p.id, {})
            near = None
            for o in self.shared["obstacles"].around(p.pos, 8.0):
                dd = math.hypot(o[1] - p.x, o[2] - p.y)
                near = dd if near is None or dd < near else near
            snap = {"status": p.status, "x": p.x, "y": p.y, "z": p.z, "yaw": p.yaw, "speed": p.speed,
                    "battery": p.battery, "odo_l": p.odo_l, "odo_r": p.odo_r, "lid": d["lid"],
                    "shutter": d["shutter"], "flag": d["flag"], "km": p.km,
                    "payload_kg": 2.4 if (p.order is not None and not p.parcel_out) else 0.0,
                    "fail": p.fail, "ranges": {k: c.get(k) for k in ("front", "rear", "left", "right")} if p.out() else {},
                    "near": near, "hub_d": math.hypot(p.x - hx, p.y - hy), "estop": False,
                    "per_km": self.power.per_km, "reserve": self.power.reserve}
            d["sensors"] = self.suites[p.id].update(t, snap)
            d["damage"] = MS.damage(p.damage)
            out.append(d)
        return out

    def showcase_ok(self):
        return "a LastMile hub has no showcase"

    def lmh_give_back(self, t, why):
        """Orders not on a pod go back to the queue; those on pods stay. / 返却。"""
        carried = [p.order for p in self.crew.pods if p.order is not None]
        self.crew.queue, self.crew.deferred = [], []
        rest = [o for o in self.pending if o not in self.failed and not _has(carried, o)]
        if rest and self.fleet is not None:
            self.fleet.dispatcher.push_front(rest)
            self.log(t, "orders_returned", batch=rest[0].batch,
                     detail="%s: #%s back in the queue" % (why, ",".join(str(o.id) for o in rest)))
        self.pending, self.stops = [o for o in carried if _has(self.pending, o)], []

    def lmh_cleanup(self):
        for b in self.bodies.values():
            b.destroy()
        for a, _ in self.parcels:
            try:
                a.destroy()
            except RuntimeError:
                pass
        self.parcels = []
        self.hubprops.destroy()

    def lmh_summary(self):
        pods = self.crew.pods
        return {"kind": "lmh", "pod_deliveries": self.lmh_delivered,
                "pod_trips": sum(p.trips for p in pods), "pod_km": round(sum(p.km for p in pods), 3),
                "pod_faults": sum(1 for p in pods if p.fail), "real_model": bool(self.real_hub)}


def collections_counter(it):
    import collections
    return collections.Counter(it)
