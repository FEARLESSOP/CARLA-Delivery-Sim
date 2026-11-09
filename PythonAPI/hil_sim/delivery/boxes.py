"""LastMile box stations standing in the world, like post boxes.

Face-to-face orders go to one of these instead of the box the pod used to
set down beside itself. Each box stands on the pavement just behind the kerb
of a kerb-side driving lane, away from junctions, the boxes spread out
(``spacing``). Its intake faces the road, so a pod parked in the kerb lane
reaches it with its side port; its compartment door faces the pavement,
where the customer collects.

The pod's side port is on its left, so boxes are placed only where the kerb
is on the left of the traffic: keep-left maps (Oita Station). On keep-right
maps the pod keeps setting its own box down.

A site is checked against the scene before a box goes there: pavement under
its footprint and under the customer's spot, nothing in the way up to 2 m.
After a delivery the parcel stays in the box; a pedestrian comes along the
pavement a while later, opens the compartment, takes it and walks off.

Parcel lockers (``lockers=True``, the default once the LastMile Hub props are
installed) replace the stations: the web viewer's Apartment B locker
(static.prop.lmhlocker) with two robot ports. The low one is at Mini Pod bay
height, so the locker stands LOCKER_SET m behind the kerb and a Mini Pod
reverses up onto the pavement to it (mini.js m_aptB); the upper one, added
straight above, is at the LMP-4 side-port tray height, and the LMP-4 parked in
the kerb lane slides its arm out to it. The customer collects from the locker.

世界に置かれた LastMile 箱型ステーション（ポストのように街に点在）。対面注文は
いずれかの箱に配達する。歩道の縁石のすぐ内側に置き、投入口は車道側、受取口は
歩道側。ポッドのサイドポートは左側なので、左側通行のマップでのみ使う。配達後、
歩行者が来て荷物を受け取る。
"""

import json
import math
import os
import random
from dataclasses import dataclass

import carla

from delivery import lmp_rig as R

STATION_BP, FLAP_BP, DOOR_BP = "static.prop.lmpstation", "static.prop.lmpstationflap", "static.prop.lmpstationdoor"
PARCEL_BP = "static.prop.lmpparcelxl"
HALF_W, HALF_D = 0.65, R.STN_D / 2.0      # footprint half width (along the road) and depth / 半寸法
KERB_W, KERB_GAP = 0.2, 0.25               # kerb, then pavement between kerb and box / 縁石と隙間
MIN_PAVEMENT = 1.8                        # box + room for the customer / 箱＋客の幅
MAX_REACH = 3.0                           # longest side-port slide allowed / スライドの最大伸び
CLEAR_H = 2.0                             # nothing in the way up to this height / この高さまで空き
PARK_AHEAD = -R.STN_X                     # pod centre is this far ahead of the box / 箱より前方の車体中心

# the Mini Pods' parcel locker (lmh_parts.json "locker", from the model) / ミニポッド用ロッカー
LOCKER_BP, LOCKER_FLAP_BP = "static.prop.lmhlocker", "static.prop.lmhlockerflap"
LOCKER_SET = 0.8                          # port face behind the kerb's top edge (pod fits in front) / 縁石から投入口
LOCKER_GAP = 0.6                          # between station and locker along the kerb / 箱とロッカーの間隔
LOCKER = {"size": [2.4, 0.8, 2.18], "bbox_min": [-1.2, -0.025, 0.0], "bbox_max": [1.2, 0.775, 2.18],
          "port": [0.0, 0.575, 0.47], "flap_pivot": [0.0, 0.555, 0.65], "flap_open_deg": 77.0}
try:
    with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "assets", "lmh_parts.json")) as _f:
        LOCKER.update(json.load(_f).get("locker") or {})
except (OSError, ValueError):
    pass


@dataclass
class BoxSite(object):
    id: int
    loc: tuple          # ground point under the box's origin / 箱の原点（地面）
    yaw: float          # same as the pod's heading when parked beside it / 駐車時の車の向き
    park: tuple         # where the pod's centre stops / 車体中心の停車位置
    walk_from: tuple    # pavement point the customer comes from / 客の出発点
    locker: tuple = None  # ground point under the Mini Pods' locker, same yaw / ロッカーの原点


def _tup(l):
    return (l.x, l.y, l.z)


def _clear_column(world, p, ground_z, tol=0.06):
    """Standing ground (not road) at ground_z under p, nothing between it and CLEAR_H above.
    p の下が歩道で、その上 CLEAR_H まで何も無い。"""
    hits = world.cast_ray(carla.Location(p.x, p.y, ground_z + 25.0), carla.Location(p.x, p.y, ground_z - 3.0))
    ground = None
    for h in hits:
        z = h.location.z
        if ground_z + 0.15 < z < ground_z + CLEAR_H:
            return False                                   # something in the way / 障害物
        if abs(z - ground_z) <= tol:                      # on the same raised pavement / 同じ歩道面
            ground = h
            break
    # pavement, not road; imported maps may leave the pavement untagged (Oita: NONE)
    # 車道でないこと（取り込みマップでは歩道が無タグのことがある）
    return ground is not None and str(ground.label) not in ("Roads", "RoadLines")


def _pavement(wmap, wp):
    """The pavement beside a kerb lane: a sidewalk lane next to it on either
    side, or (roads converted from OSM keep their sidewalks apart) the nearest
    sidewalk within 9 m. The caller checks it is on the kerb side.
    走行車線脇の歩道：隣接する歩道車線、無ければ 9 m 以内の最寄りの歩道。"""
    for step in (lambda w: w.get_left_lane(), lambda w: w.get_right_lane()):
        n = step(wp)
        for _ in range(4):
            if n is None or n.lane_type == carla.LaneType.Driving:
                break
            if n.lane_type == carla.LaneType.Sidewalk:
                return n
            n = step(n)
    sw = wmap.get_waypoint(wp.transform.location, project_to_road=True, lane_type=carla.LaneType.Sidewalk)
    if sw is not None and sw.transform.location.distance(wp.transform.location) <= 9.0:
        return sw
    return None


def _any_sidewalk(wmap):
    try:
        import xml.etree.ElementTree as ET
        return any(l.get("type") == "sidewalk" for l in ET.fromstring(wmap.to_opendrive()).iter("lane"))
    except Exception:
        return False


class _ModelPavement(object):
    """Stand-in for a pavement lane on the kerb side (keep-left: the left),
    2.5 m out from the lane's edge; previous() steps back along the lane.
    道路情報に無い歩道の代わり（縁石側、車線端から 2.5 m）。"""
    lane_width = 2.0

    def __init__(self, wp):
        self.wp = wp
        t = wp.transform
        f = t.get_forward_vector()
        d = wp.lane_width / 2 + 2.5
        self.transform = carla.Transform(carla.Location(t.location.x + f.y * d, t.location.y - f.x * d,
                                                        t.location.z + 0.15), t.rotation)

    def previous(self, dist):
        back = self.wp.previous(dist)
        return [_ModelPavement(back[0])] if back else []

    def next(self, dist):
        ahead = self.wp.next(dist)
        return [_ModelPavement(ahead[0])] if ahead else []


def _ground_z(world, x, y, z):
    hits = world.cast_ray(carla.Location(x, y, z + 3.0), carla.Location(x, y, z - 3.0))
    return hits[0].location.z if hits else None


def _kerb(world, L, nx, ny, lane_width):
    """(distance from the lane centre to the kerb's top edge, pavement height),
    by stepping out from the lane centre until the ground rises onto the
    pavement; None if no kerb within reach. / 縁石までの距離と歩道の高さ。"""
    road = _ground_z(world, L.x, L.y, L.z)
    if road is None:
        return None
    d = max(lane_width / 2.0 - 0.6, 0.5)
    while d < lane_width / 2.0 + 3.5:
        z = _ground_z(world, L.x + nx * d, L.y + ny * d, road)
        if z is not None and z > road + 0.08:
            return d + KERB_W, z                          # past the kerb stone / 縁石の先
        d += 0.1
    return None


def reachable(planner):
    """Test for 'a pod can drive to this spot and away again': the spot's
    road segment lies in the largest part of the road graph where every
    place can be reached from every other. Off it (a one-way street that
    only leads in, the map's edge) a pod could get there and not out, or not
    get there. None (no test) without a planner.
    ポッドが行けて、かつ出られる場所か（道路グラフの最大強連結成分）。"""
    if planner is None:
        return None
    import networkx as nx
    core = max(nx.strongly_connected_components(planner._graph), key=len)

    def ok(loc):
        try:
            a, b = planner._localize(loc)
        except (KeyError, TypeError):
            return False
        return a in core and b in core
    return ok


def find_box_sites(world, wmap, rng, spacing=110.0, max_boxes=30, step=12.0, planner=None, lockers=False):
    """Spread-out kerb-side pavement spots for box stations (keep-left maps).
    With a route planner, only spots a pod can drive to and away from.
    ``lockers``: only spots with room for a Mini Pod locker beside the
    station (behind it along the lane) and for the pod docked at its port.
    歩道沿いの箱の設置場所（左側通行マップ用）。"""
    can_reach = reachable(planner)
    has_sidewalks = _any_sidewalk(wmap)
    cands = []
    for wp in wmap.generate_waypoints(step):
        if wp.lane_type != carla.LaneType.Driving or wp.is_junction:
            continue
        kerb = wp.get_left_lane()
        if kerb is not None and kerb.lane_type == carla.LaneType.Driving:
            continue                                      # not the kerb lane / 路肩側でない
        if any(n.is_junction for n in wp.next(12.0) + wp.previous(12.0)):
            continue                                      # too close to a junction / 交差点に近い
        sw = _pavement(wmap, wp)
        if sw is None and not has_sidewalks:
            # road logic without pavement lanes (maps built by osm_city): the
            # pavement is only in the 3-D model; the kerb test below finds it
            # 歩道車線の無い道路情報: 歩道は 3D モデルのみ（下の縁石判定で確認）
            sw = _ModelPavement(wp)
        if sw is None or sw.lane_width < MIN_PAVEMENT:
            continue
        cands.append((wp, sw))
    rng.shuffle(cands)
    sites = []
    for wp, sw in cands:
        L, f = wp.transform.location, wp.transform.get_forward_vector()
        S = sw.transform.location
        v = carla.Vector3D(S.x - L.x, S.y - L.y, 0.0)
        along = v.x * f.x + v.y * f.y
        lat = carla.Vector3D(v.x - along * f.x, v.y - along * f.y, 0.0)
        dist = math.hypot(lat.x, lat.y)
        if dist < 0.5:
            continue
        if -f.y * lat.x + f.x * lat.y > 0:                # pavement on the right: wrong side / 右側
            continue
        # The kerb, measured on the 3-D model: the road logic's lane edge need
        # not match the modelled road (Oita: generated separately and wider),
        # and imported maps may not tag road vs pavement. Walk out from the
        # lane centre until the ground steps up onto the raised pavement.
        # 縁石は 3D モデルで測る（道路情報の車線端とモデルの縁石は一致しない）。
        # 車線中心から外へ進み、歩道の段差で地面が上がる所を探す。
        nx, ny = f.y, -f.x                                # left of the lane / 車線の左
        kerb = _kerb(world, L, nx, ny, wp.lane_width)
        if kerb is None:
            continue
        k_off, gz = kerb
        if lockers:
            # the parcel locker: front (+y local) to the road, its port face
            # LOCKER_SET behind the kerb (a Mini Pod docks on the pavement in
            # front of it); the LMP-4's arm must reach its upper port
            # ロッカー：正面を車道へ、縁石から LOCKER_SET 奥（ミニポッドは歩道上で接続）
            lo, hi, port = LOCKER["bbox_min"], LOCKER["bbox_max"], LOCKER["port"]
            off = k_off + LOCKER_SET + port[1]
            if R.SLIDE_MAX - (-(k_off + LOCKER_SET + R.STN_D / 2.0) - R.STN_ZC) > MAX_REACH:
                continue                                  # too far for the arm / アームが届かない
            cx, cy = L.x + nx * off, L.y + ny * off
            if any(math.hypot(cx - s.loc[0], cy - s.loc[1]) < spacing for s in sites):
                continue
            pts = [carla.Location(cx + f.x * a - nx * b, cy + f.y * a - ny * b, gz)
                   for a in (lo[0] + 0.05, 0.0, hi[0] - 0.05) for b in (lo[1] + 0.05, port[1])]
            pts += [carla.Location(cx + f.x * a - nx * (port[1] + d), cy + f.y * a - ny * (port[1] + d), gz)
                    for a in (-0.3, 0.3) for d in (0.1, 0.7)]          # docked pod, customer / ポッド・客
        else:
            off = k_off + KERB_GAP + HALF_D
            if R.SLIDE_MAX - (-off - R.STN_ZC) > MAX_REACH:
                continue                                  # too far for the arm / アームが届かない
            cx, cy = L.x + nx * off, L.y + ny * off
            if any(math.hypot(cx - s.loc[0], cy - s.loc[1]) < spacing for s in sites):
                continue
            pts = [carla.Location(cx + f.x * a + nx * b, cy + f.y * a + ny * b, gz)
                   for a in (-HALF_W, 0.0, HALF_W) for b in (-HALF_D, HALF_D)]
            pts.append(carla.Location(cx + nx * (HALF_D + 0.55), cy + ny * (HALF_D + 0.55), gz))   # customer / 客
        if not all(_clear_column(world, p, gz) for p in pts):
            continue                                      # not all on the pavement, or blocked / 歩道外・障害物
        locker = (cx, cy, gz) if lockers else None
        park = wmap.get_waypoint(carla.Location(cx - nx * off + f.x * PARK_AHEAD,
                                                cy - ny * off + f.y * PARK_AHEAD, L.z))
        if park is None or park.is_junction or park.road_id != wp.road_id or park.lane_id != wp.lane_id:
            continue
        if can_reach is not None and not can_reach(park.transform.location):
            continue                                      # could get stuck there / 出られない
        back = sw.previous(14.0) or sw.next(14.0)
        if not back:
            continue
        sites.append(BoxSite(len(sites) + 1, (cx, cy, gz), wp.transform.rotation.yaw,
                             _tup(park.transform.location), _tup(back[0].transform.location), locker))
        if len(sites) >= max_boxes:
            break
    return sites


class _Box(object):
    def __init__(self, site, station, flap, door):
        self.site, self.station, self.flap, self.door = site, station, flap, door
        self.tf = carla.Transform(carla.Location(*site.loc), carla.Rotation(yaw=site.yaw))
        self.pose_key = (0.0, 0.0)
        self.parcels = []                 # parcel actors in the compartment / 区画内の荷物
        self.where = []                   # per parcel: "station" or "locker" / 保管場所
        self.locker = self.lflap = self.uflap = None   # parcel locker, low + upper port flaps / ロッカー
        self.ltf = None
        self.flap_key = {False: 0.0, True: 0.0}
        self.pickup_t = None
        self.customer = None              # [walker, phase, timer, carried parcel] / 客の状態


class BoxStations(object):
    """The box stations in the world and their customers. / 箱と受取客。"""

    def __init__(self, world, client, sites, rng=None, lockers=False):
        self.world, self.client, self.rng = world, client, rng or random.Random()
        lib = world.get_blueprint_library()
        self.lib = lib
        self.boxes = {}
        if lockers and lib.filter(LOCKER_BP) and lib.filter(LOCKER_FLAP_BP):
            # parcel lockers: one per site, two robot ports (Mini Pod low,
            # LMP-4 high) / 宅配ロッカー（下段ミニポッド・上段 LMP-4）
            for s in sites:
                tf = carla.Transform(carla.Location(*s.loc), carla.Rotation(yaw=s.yaw))
                lk = world.try_spawn_actor(lib.find(LOCKER_BP), tf)
                if lk is None:
                    continue
                b = self.boxes[s.id] = _Box(s, None, None, None)
                b.locker, b.ltf = lk, tf
                b.lflap = world.spawn_actor(lib.find(LOCKER_FLAP_BP), self._flap_tf(0.0), attach_to=lk)
                if LOCKER.get("flap_pivot_upper"):
                    b.uflap = world.spawn_actor(lib.find(LOCKER_FLAP_BP), self._flap_tf(0.0, True), attach_to=lk)
            return
        if not (lib.filter(STATION_BP) and lib.filter(FLAP_BP) and lib.filter(DOOR_BP)):
            return
        local = R.station_local(0.0, 0.0)
        for s in sites:
            st = world.try_spawn_actor(lib.find(STATION_BP),
                                       carla.Transform(carla.Location(*s.loc), carla.Rotation(yaw=s.yaw)))
            if st is None:
                continue
            fl = world.spawn_actor(lib.find(FLAP_BP), self._tf(local["LMPStationFlap"]), attach_to=st)
            dr = world.spawn_actor(lib.find(DOOR_BP), self._tf(local["LMPStationDoor"]), attach_to=st)
            self.boxes[s.id] = _Box(s, st, fl, dr)

    @staticmethod
    def _tf(pose):
        x, y, z, pitch, yaw, roll = pose
        return carla.Transform(carla.Location(x, y, z), carla.Rotation(pitch=pitch, yaw=yaw, roll=roll))

    def __len__(self):
        return len(self.boxes)

    def world_point(self, box_id, local):
        """A point in the box's frame -> world. / 箱座標の点をワールドへ。"""
        b = self.boxes[box_id]
        return b.tf.transform(carla.Location(*local))     # transform() changes its argument / 引数を書き換える

    def has_locker(self, box_id):
        b = self.boxes.get(box_id)
        return b is not None and b.locker is not None

    def locker_point(self, box_id, local):
        """A point in the locker's frame -> world. / ロッカー座標の点をワールドへ。"""
        return self.boxes[box_id].ltf.transform(carla.Location(*local))

    @staticmethod
    def _flap_tf(f, upper=False):
        x, y, z = LOCKER["flap_pivot_upper" if upper else "flap_pivot"]
        return carla.Transform(carla.Location(x, y, z), carla.Rotation(roll=LOCKER["flap_open_deg"] * f))

    def port(self, box_id, flap, upper=False):
        """Open / close a robot-port flap of the locker (hinged at the top,
        swings inward): the low one (Mini Pods) or the upper one (LMP-4).
        ロボットポートの扉（下段：ミニポッド、上段：LMP-4）。"""
        b = self.boxes.get(box_id)
        a = None if b is None else (b.uflap if upper else b.lflap)
        f = round(flap, 3)
        if a is None or f == b.flap_key[upper]:
            return
        b.flap_key[upper] = f
        self.client.apply_batch([carla.command.ApplyTransform(a.id, self._flap_tf(f, upper))])

    def pose(self, box_id, flap, cell_door):
        """Open / close the intake flap and the compartment door. / 扉の開閉。"""
        b = self.boxes.get(box_id)
        key = (round(flap, 3), round(cell_door, 3))
        if b is None or b.flap is None or key == b.pose_key:
            return
        b.pose_key = key
        local = R.station_local(flap, cell_door)
        self.client.apply_batch([carla.command.ApplyTransform(b.flap.id, self._tf(local["LMPStationFlap"])),
                                 carla.command.ApplyTransform(b.door.id, self._tf(local["LMPStationDoor"]))])

    def store(self, box_id, t, locker=False):
        """A parcel is in the compartment (or the locker: filed inside, out of
        sight); a customer will come for it. / 荷物を保管。"""
        b = self.boxes.get(box_id)
        if b is None:
            return
        if (locker or b.station is None) and b.locker is not None:
            b.parcels.append(None)
            b.where.append("locker")
            if b.pickup_t is None:
                b.pickup_t = t + self.rng.uniform(20.0, 60.0)
            return
        b.where.append("station")
        cx, cy, cz = R.station_point(R.CELL_CENTER)
        p = self.world.try_spawn_actor(self.lib.find(PARCEL_BP),
                                       carla.Transform(carla.Location(cx, cy, cz + R.PARCEL_XL_H / 2.0)),
                                       attach_to=b.station)
        b.parcels.append(p)
        if b.pickup_t is None:
            b.pickup_t = t + self.rng.uniform(20.0, 60.0)

    def step(self, t, dt):
        """Customers come for parcels waiting in the boxes. / 客が受け取りに来る。"""
        for b in self.boxes.values():
            if b.customer is None and b.parcels and b.pickup_t is not None and t >= b.pickup_t:
                self._spawn_customer(b, t)
            if b.customer is not None:
                self._walk(b, t, dt)

    def _spawn_customer(self, b, t):
        bps = self.lib.filter("walker.pedestrian.*")
        if not bps:
            return
        bp = self.rng.choice(bps)
        if bp.has_attribute("is_invincible"):
            bp.set_attribute("is_invincible", "true")
        start = carla.Location(*b.site.walk_from) + carla.Location(z=1.0)
        w = self.world.try_spawn_actor(bp, carla.Transform(start))
        if w is None:
            b.pickup_t = t + 5.0                         # try again shortly / 少し後に再試行
            return
        b.customer = [w, "to_box", 0.0, None]

    def _walk(self, b, t, dt):
        w, phase, timer, carried = b.customer
        b.customer[2] = timer + dt
        at_locker = bool(b.where) and b.where[0] == "locker"
        if at_locker:
            px, py, _ = LOCKER["port"]
            front = self.locker_point(b.site.id, (px, py + 0.55, 0.0))
        else:
            front = self.world_point(b.site.id, R.station_point(R.CUSTOMER_FRONT))
        home = carla.Location(*b.site.walk_from)
        target = front if phase == "to_box" else home
        try:
            here = w.get_location()
        except RuntimeError:
            b.customer = None
            return
        d = math.hypot(target.x - here.x, target.y - here.y)
        ctl = carla.WalkerControl()
        if phase in ("to_box", "leave") and d > 0.35:
            ctl.direction = carla.Vector3D((target.x - here.x) / d, (target.y - here.y) / d, 0.0)
            ctl.speed = min(1.4, d * 1.5)
        w.apply_control(ctl)
        if phase == "to_box" and (d <= 0.35 or b.customer[2] > 40.0):
            b.customer[1:3] = ["open", 0.0]
        elif phase == "open":                             # door, parcel to hands, door / 扉→受取→扉
            u = b.customer[2]
            if not at_locker:
                self.pose(b.site.id, 0.0, min(u / 0.6, 1.0) if u < 1.4 else max(1.0 - (u - 1.4) / 0.6, 0.0))
            if u >= 0.9 and carried is None and b.parcels:
                p = b.parcels.pop(0)
                if b.where:
                    b.where.pop(0)
                if p is not None:
                    p.destroy()
                b.customer[3] = self.world.try_spawn_actor(
                    self.lib.find(PARCEL_BP), carla.Transform(carla.Location(x=0.36, z=0.05)), attach_to=w)
            if u >= 2.0:
                b.customer[1:3] = ["leave", 0.0]
        elif phase == "leave" and (d <= 0.6 or b.customer[2] > 40.0):
            for a in (b.customer[3], w):
                if a is not None:
                    a.destroy()
            b.customer = None
            b.pickup_t = t + self.rng.uniform(15.0, 40.0) if b.parcels else None

    def destroy(self):
        for b in self.boxes.values():
            if b.customer is not None:
                for a in (b.customer[3], b.customer[0]):
                    if a is not None:
                        try:
                            a.destroy()
                        except RuntimeError:
                            pass
            for a in b.parcels + [b.uflap, b.lflap, b.locker, b.flap, b.door, b.station]:
                if a is not None:
                    try:
                        a.destroy()
                    except RuntimeError:
                        pass
        self.boxes = {}
