"""LastMile Hub LMH-3 with Mini Delivery Pods: timeline, geometry and the pod
state machine (server-free; tests/test_lmh.py).

The animation is ported from the user's web viewer, the "Mini pods" tab of
LastMile_Pod_3D.html (``lastmile _site 2/lastmile _site/js_3d_lastmile/
mini.js`` + ``mini_hub.js``, 89 s loop), the way lmp_rig.py ports the LMP-4:

  scan (storage loop runs past the scan tunnel) -> divert (gates 3, 2, 1 fire
  0.9 s apart) -> lift (carriages down 0.72 m) -> load (shutter up, rollers
  push the parcel 0.64 m into the pod, shutter down, carriage up) -> release
  (plug out, pins out, ramp doors fold down) -> roll out (1.2 m/s down the
  ramp, flag up once clear, 1.6 m/s on the street) ... -> return to the ramp
  foot, turn to face the kerb, reverse 1.69 m up into the bay at 0.6 m/s ->
  pins in, plug in, door up.

Deliveries never enter a building (the user's rule). A pod either
  - FRONT DROP: stops at the drop point in front of the place with its rear
    to it, rolls the rear shutter up and the floor belt pushes the parcel out
    onto the ground (the HTML's locker push, ~0.77 m), shutter down; or
  - DELIVERY BOX: drives to a box station (delivery/boxes.py), turns, reverses
    0.39 m up to its intake, the flap opens, the shutter rolls up and the belt
    pushes the parcel in (the HTML's Apartment B robot port), flap and shutter
    close; the box then keeps it for its customer.

At a charging station (delivery/charge_sites.py, the web viewer's charge tab)
the hub charges in bay 04 and its low pods charge on pads M1..M3 behind it:
down the ramps, along the hub-side lane, a U-turn behind the pads, along the
pad lane, turn, reverse 1.1 m onto the pad; pins in, plug in; back the same
way when full or recalled (HubCrew.station_*).

Frames: the hub's vehicle frame is CARLA's (x forward, y right, z up). The
model's bay doors are on its LEFT (file -Z = curb side), so on keep-left maps
they face the kerb. HTML (x, y-up, z) -> CARLA (x, z, y), like lmp_rig.H.

Parts come from delivery/assets/lmh_parts.json (written by the asset build);
until it exists a stub with the README's numbers is used (``STUB``).

LastMile ハブ（LMH-3）とミニ配送ポッドのタイムライン・幾何・状態機械（サーバー不要）。
アニメーションはユーザーの Web ビューアの Mini pods タブを移植。建物には入らず、
建物前に荷物を置く（前置き）か、箱型ステーションへ押し込む（宅配ボックス）。
充電ステーションではハブがベイ 04 で充電し、ポッドは後ろのパッド M1〜M3 で充電する。
"""

import heapq
import json
import math
import os
import random

from delivery.lmp_rig import clamp, lerp, smooth, rotator

HERE = os.path.dirname(os.path.abspath(__file__))
PARTS_PATH = os.path.join(HERE, "assets", "lmh_parts.json")

# -- geometry (README / mini_hub.js, CARLA vehicle frame) / 幾何 ---------------
HUB_BP = "vehicle.delivery.lastmilehub"
HUB_HALF_L, HUB_HALF_W = 1.625, 0.74
BAYS_X = (-0.66, 0.0, 0.66)           # bay centre lines (P1, P2, P3) / ベイ中心
DOCK_Y, FLOOR_Z = -0.31, 0.20         # docked pod centre, bay floor / 格納位置と床
RAMP_FOOT_Y = -2.0                    # ramp foot (Z -200 cm) / スロープの下
DOCK_YAW = -90.0                      # docked: nose toward the bay door (kerb) / 扉向き
DOCK_REV = DOCK_Y - RAMP_FOOT_Y       # 1.69 m reversed up the ramp / 後退距離
# Mini pod (pod frame: x front, y right, z up) / ミニポッド
POD_HL, POD_HW, POD_TOP = 0.35, 0.27, 0.665
WHEEL_R, WHEEL_X, WHEEL_Y = 0.085, (0.235, 0.0, -0.235), 0.235
CARGO = (-0.08, 0.0, 0.306)           # parcel bottom centre on the floor belt / 荷物位置
PUSH_OUT = 0.77                       # belt push into a locker (CARGO_X + 0.85) / 押出し量
BOX_BACK = 0.39                       # reverse up to a locker intake / 箱への後退
FRONT_GAP = POD_HL + 0.5              # pod centre from the drop point: the parcel (0.85 m
                                      # behind the centre) lands on it / 荷物が置き点に乗る位置

# speeds (mini.js: VP 1.6, VIN 1.2, VR 0.6) / 速度
V_POD, V_RAMP, V_REV = 1.6, 1.2, 0.6
V_SLOW = 0.8                          # walkers within SLOW_R / 歩行者の近く
SLOW_R = 4.0
SPIN_DPS = 160.0                      # skid-steer turn on the spot / その場旋回
ACCEL = 1.0
BLOCK_REACH = 2.5                     # obstacle check ahead on the path / 前方の障害物
STUCK_S = 6.0                         # blocked this long -> recovery / 回復まで
MAX_TRIES = 3                         # back up + replan, then skip ahead / 後退再計画の上限
SKIP_M = 4.0

# -- hub sequence, seconds from the start of the work at a stop --------------
# mini.js times minus the 7 s drive-in (m_scan starts at 0). / 停車後の秒数
STEPS = (("scan", 0.0, 6.0), ("divert", 6.0, 12.0), ("lift", 12.0, 16.0),
         ("load", 16.0, 22.0), ("release", 22.0, 27.0), ("deploy", 27.0, 33.0))
GATE_T = (8.8, 7.9, 7.0)              # each lane's gate fires (P3's first) / ゲート作動
OUT_T = (27.4, 27.7, 30.0)            # roll-out per bay / 発進時刻
WORK_S = OUT_T[2]                     # the last pod leaves at 30 s / 最後の発進

ORDER_FRONT, ORDER_BOX = "front", "box"

# pod statuses (the dashboard codes against these) / ポッドの状態
DOCKED, LOADING, ROLLING_OUT, TO_DOOR, HANDOVER = "docked", "loading", "rolling_out", "to_door", "handover"
RETURNING, DOCKING, WAITING, CRASHED, STUCK = "returning", "docking", "waiting", "crashed", "stuck"
# charging-station trips: to a pad behind bay 04, charging on it, back to the hub
# 充電ステーション：パッドへ・パッドで充電中・ハブへ戻る
TO_PAD, ON_PAD, FROM_PAD = "to_pad", "on_pad", "from_pad"
STATUSES = (DOCKED, LOADING, ROLLING_OUT, TO_DOOR, HANDOVER, RETURNING, DOCKING, WAITING, CRASHED, STUCK,
            TO_PAD, ON_PAD, FROM_PAD)
STATION = (TO_PAD, ON_PAD, FROM_PAD)
OUT = (ROLLING_OUT, TO_DOOR, HANDOVER, RETURNING, DOCKING, WAITING, STUCK) + STATION

# -- charging-station pads (delivery/charge_sites.py, station-local frame: +x
# into the bays, +y right; the hub in bay 04 at (0, 0) nose +x, its bay doors
# on local -y) / 充電ステーションのパッド（ステーション座標）
PAD_START, PAD_FULL = 97.0, 98.0      # at a station every pod not full goes to a pad (web viewer) / パッドへ・満充電
HUB_FEED_MIN = 50.0                   # the hub feeds its docked pods only above this (its own range first) / ハブ給電の下限
STN_LANE_Y = -2.55                    # hub-side lane: open floor between bay 04 and bay 03 (LMP-4 from
                                      # y -3.4), clear of bay 04's DC cabinet (x 3.45..4.09, y -1.91..-0.89)
                                      # / ハブ側の通路（ベイ 04 の充電盤を避ける）
STN_TURN_X = 7.75                     # U-turn behind the pads, still on the floor (x <= 9.98) / パッド奥で転回
V_STN = 1.2                           # in the station yard / 構内の速度
STN_STAGGER = 3.0                     # between pods leaving the hub / ハブからの発進間隔
STN_GAP_S = 4.0                       # between pods leaving the pads / パッドからの発進間隔
PAD_PLUG_S, PAD_UNPLUG_S = 1.3, 0.9   # pins + plug in, plug + pins out (charge.js) / 接続・切断の時間


def work_phase(el):
    """The hub's step at ``el`` s into the work at a stop. / 作業の段階。"""
    for name, a, b in STEPS:
        if el < b:
            return name
    return "pods_out"


# -- parts / 部品 ---------------------------------------------------------------
def _stub():
    """lmh_parts.json as the README describes the model (until the asset build
    writes the real one). / README の値による仮の部品表。"""
    gate = rotator(0.0, math.radians(-45.0), 0.0)[1]          # HTML -45 deg about Y
    door = rotator(math.radians(-90.7), 0.0, 0.0)[2]          # HTML -90.7 deg about X
    lid = rotator(0.0, 0.0, math.radians(-66.0))[0]           # HTML -66 deg about Z
    hub_parts = []
    bays = []
    for i, bx in enumerate(BAYS_X):
        n = i + 1
        hub_parts += [
            {"name": "Diverter_Gate_%d" % n, "prop": "static.prop.lmhdivertergate",
             "loc": [bx - 0.195 + 1.0 - 1.0, -0.255, 1.22], "rot": [0, 0, 0],
             "motion": {"type": "rotate", "axis": "z", "amount": gate, "space": "local"}},
            {"name": "Lift_%d_Carriage" % n, "prop": "static.prop.lmhliftcarriage",
             "loc": [bx, 0.41, 1.22], "rot": [0, 0, 0],
             "motion": {"type": "slide", "axis": "z", "amount": -0.72, "space": "local"}},
            {"name": "Bay_%d_RampDoor" % n, "prop": "static.prop.lmhrampdoor",
             "loc": [bx, -0.74, 0.156], "rot": [0, 0, 0],
             "motion": {"type": "rotate", "axis": "x", "amount": door, "space": "local"}},
            {"name": "Bay_%d_ChargePlug" % n, "prop": "static.prop.lmhchargeplug",
             "loc": [bx, 0.06, 0.44], "rot": [0, 0, 0],
             "motion": {"type": "slide", "axis": "y", "amount": 0.038, "space": "local"}},
            {"name": "Bay_%d_LockPins" % n, "prop": "static.prop.lmhlockpins",
             "loc": [bx, 0.0, 0.30], "rot": [0, 0, 0],
             "motion": {"type": "slide", "axis": "y", "amount": 0.058, "space": "local"}}]
        bays.append({"id": n, "dock": [bx, DOCK_Y, FLOOR_Z, DOCK_YAW], "ramp_foot": [bx, RAMP_FOOT_Y, 0.0]})
    wheels = []
    for k, (side, x) in enumerate([(-1, x) for x in WHEEL_X] + [(1, x) for x in WHEEL_X]):
        wheels.append({"name": "Wheel%d" % (k + 1), "prop": "static.prop.lmhminipodwheel",
                       "loc": [x, side * WHEEL_Y, WHEEL_R], "rot": [0, 0, 0],
                       "motion": {"type": "rotate", "axis": "y", "amount": 360.0, "space": "local"}})
    pod_parts = wheels + [
        {"name": "Lid", "prop": "static.prop.lmhminipodlid", "loc": [0.18, 0.0, POD_TOP], "rot": [0, 0, 0],
         "motion": {"type": "rotate", "axis": "y", "amount": lid, "space": "local"}},
        {"name": "Shutter", "prop": "static.prop.lmhminipodshutter", "loc": [-0.352, 0.0, 0.6], "rot": [0, 0, 0],
         "motion": {"type": "scale", "axis": "z", "amount": 0.04, "space": "local"}},
        {"name": "FlagMast", "prop": "static.prop.lmhminipodflagmast", "loc": [-0.22, 0.17, 0.62], "rot": [0, 0, 0],
         "motion": [{"type": "slide", "axis": "z", "amount": 0.259},
                    {"type": "scale", "axis": "z", "from": 0.06, "amount": 1.0}]},
        {"name": "Flag", "prop": "static.prop.lmhminipodflag", "loc": [-0.22, 0.17, 0.64], "rot": [0, 0, 0],
         "motion": [{"type": "slide", "axis": "z", "amount": 0.517},
                    {"type": "scale", "axis": "z", "from": 0.001, "amount": 1.0}]}]
    return {
        "_stub": True,
        "hub": {"blueprint": HUB_BP, "size": [3.25, 1.48, 2.09], "parts": hub_parts, "bays": bays},
        "minipod": {"body": "static.prop.lmhminipod", "size": [0.70, 0.54, 0.67], "speed_mps": V_POD,
                    "parts": pod_parts,
                    "sensors_mounts": {"lidar": [0.0, 0.0, 0.70], "cam_front": [0.353, 0.0, 0.44],
                                       "cam_rear": [-0.353, 0.0, 0.50], "cam_left": [0.0, -0.272, 0.50],
                                       "cam_right": [0.0, 0.272, 0.50], "cam_down_front": [0.34, 0.0, 0.25],
                                       "cam_lid": [0.10, 0.0, 0.67],
                                       "ultrasonic": [[0.352, y, 0.245] for y in (-0.19, -0.06, 0.06, 0.19)]
                                       + [[-0.352, y, 0.27] for y in (-0.15, 0.15)],
                                       "gnss": [-0.20, 0.0, 0.67]}},
        "parcels": {"small": "static.prop.lmhparcels", "large": "static.prop.lmhparcell"},
        "timeline": {"steps": [list(s) for s in STEPS], "gate_t": list(GATE_T), "out_t": list(OUT_T)}}


STUB = _stub()
_PARTS = {"mtime": None, "data": None}


def load_parts(path=None):
    """lmh_parts.json (re-read when it changes), else the stub. Missing keys
    come from the stub. / 部品表（無ければ仮のもの）。"""
    path = path or PARTS_PATH
    try:
        mt = os.path.getmtime(path)
    except OSError:
        return STUB
    if _PARTS["mtime"] != (path, mt):
        try:
            with open(path) as fh:
                d = json.load(fh)
        except (OSError, ValueError):
            return STUB
        out = dict(STUB)
        out.pop("_stub", None)
        for k in ("hub", "minipod", "parcels", "timeline"):
            if isinstance(d.get(k), dict):
                out[k] = dict(STUB[k], **d[k])
        if len(out["hub"].get("bays") or []) < 3:
            out["hub"]["bays"] = STUB["hub"]["bays"]
        _PARTS.update(mtime=(path, mt), data=out)
    return _PARTS["data"]


def bay_geometry(parts, bay):
    """(dock (x, y, z, yaw), ramp foot (x, y, z)) of bay 1..3 in the hub frame."""
    b = next((b for b in parts["hub"]["bays"] if int(b.get("id", 0)) == bay), parts["hub"]["bays"][bay - 1])
    return tuple(b["dock"]), tuple(b["ramp_foot"])


_AXES = {"x": (1.0, 0.0, 0.0), "y": (0.0, 1.0, 0.0), "z": (0.0, 0.0, 1.0)}
_ROT_INDEX = {"y": 0, "z": 1, "x": 2}          # pitch, yaw, roll / 回転軸


def motion_pose(part, u, spin_deg=None):
    """(x, y, z, pitch, yaw, roll) of ``part`` at motion value ``u`` (0..1), or
    None when it is hidden (a scale motion CARLA cannot do at runtime: rolled up
    / folded away). ``spin_deg`` overrides a wheel's rotation.
    部品の姿勢（実行中に拡縮できないため、縮んだ部品は隠す）。"""
    x, y, z = (float(v) for v in part.get("loc", (0, 0, 0)))
    rot = [float(v) for v in (part.get("rot") or (0, 0, 0))]
    motions = part.get("motion") or []
    if isinstance(motions, dict):
        motions = [motions]
    u = clamp(float(u), 0.0, 1.0)
    if u < 0.05 and (part.get("hide_when_stowed") or any(m.get("hide_when_stowed") for m in motions)):
        return None                       # the flag only exists raised / 旗は上げたときだけ
    for m in motions:
        kind, axis, amt = m.get("type"), str(m.get("axis", "z")).lower(), float(m.get("amount", 0.0))
        if kind == "slide":
            a = _AXES.get(axis, _AXES["z"])
            x, y, z = x + a[0] * amt * u, y + a[1] * amt * u, z + a[2] * amt * u
        elif kind == "rotate":
            k = _ROT_INDEX.get(axis, 1)
            rot[k] += (spin_deg if spin_deg is not None else amt * u)
        elif kind == "scale":
            s = lerp(float(m.get("from", 1.0)), amt, u)
            if s < 0.35:
                return None
    return (x, y, z, rot[0], rot[1], rot[2])


# -- tracks / トラック ------------------------------------------------------------
class Track(object):
    """A value moved by timed ramps (smooth in between), like the HTML's keyed().
    時刻指定のランプで動く値（間は smooth 補間）。"""

    def __init__(self, v=0.0):
        self.keys = [(-1e9, float(v))]

    def at(self, t):
        ks = self.keys
        if t <= ks[0][0]:
            return ks[0][1]
        for (t0, v0), (t1, v1) in zip(ks, ks[1:]):
            if t < t1:
                return lerp(v0, v1, smooth((t - t0) / max(t1 - t0, 1e-6)))
        return ks[-1][1]

    def ramp(self, t0, dur, to):
        """From its value at t0 to ``to`` over ``dur`` s (later keys dropped)."""
        v0 = self.at(t0)
        self.keys = [k for k in self.keys if k[0] < t0] + [(t0, v0), (t0 + max(dur, 1e-3), float(to))]

    def end(self):
        return self.keys[-1][0]

    def target(self):
        return self.keys[-1][1]


class HubMech(object):
    """The hub's moving parts per bay: gate, lift (1 = down), door (1 = open
    ramp), plug and pins (1 = in). / ハブの可動部（ベイごと）。"""

    def __init__(self):
        self.gate = [Track() for _ in range(3)]
        self.lift = [Track() for _ in range(3)]
        self.door = [Track() for _ in range(3)]
        self.plug = [Track(1.0) for _ in range(3)]
        self.pins = [Track(1.0) for _ in range(3)]
        self.loop = Track()            # storage loop running (scan tunnel) / 搬送ループ

    def schedule_load(self, t0, bays):
        """The scan..release sequence for ``bays`` (indices 0..2) from t0;
        returns {bay: roll-out time}. / 積込〜解放の手順。"""
        self.loop.ramp(t0 + 0.3, 0.6, 1.0)
        self.loop.ramp(t0 + 11.0, 0.6, 0.0)
        out = {}
        for i in bays:
            g = t0 + GATE_T[i]
            self.gate[i].ramp(g - 0.5, 0.3, 1.0)
            self.gate[i].ramp(g + 0.5, 0.3, 0.0)
            self.lift[i].ramp(t0 + 12.3 + 0.35 * i, 2.5, 1.0)
            self.lift[i].ramp(t0 + 20.3 + 0.3 * i, 1.2, 0.0)
            self.plug[i].ramp(t0 + 22.3, 0.5, 0.0)
            self.pins[i].ramp(t0 + 22.8, 0.4, 0.0)
            self.door[i].ramp(t0 + 23.4, 1.8, 1.0)
            out[i] = t0 + OUT_T[i]
        return out

    def open_door(self, t, i):
        if self.door[i].target() < 0.5:
            self.door[i].ramp(t, 1.6, 1.0)

    def close_door(self, t, i):
        if self.door[i].target() > 0.5:
            self.door[i].ramp(t, 1.5, 0.0)

    def docked(self, t, i):
        """Pod in: pins, plug, door up (mini.js). / 格納後：ピン・プラグ・扉。"""
        self.pins[i].ramp(t + 0.1, 0.4, 1.0)
        self.plug[i].ramp(t + 0.5, 0.4, 1.0)
        self.door[i].ramp(t + 1.0, 1.4, 0.0)

    def doors_shut(self, t):
        return all(d.at(t) < 0.01 and d.target() < 0.5 for d in self.door)

    def value(self, name, t):
        """Motion value 0..1 of hub part ``name`` (README node names)."""
        for i in range(3):
            n = str(i + 1)
            if name == "Diverter_Gate_" + n:
                return self.gate[i].at(t)
            if name == "Lift_%s_Carriage" % n:
                return self.lift[i].at(t)
            if name == "Bay_%s_RampDoor" % n:
                return self.door[i].at(t)
            if name == "Bay_%s_ChargePlug" % n:
                return 1.0 - self.plug[i].at(t)        # motion = retract / 引込み量
            if name == "Bay_%s_LockPins" % n:
                return 1.0 - self.pins[i].at(t)
        return 0.0


# -- kerb-side paths / 路肩側の経路 -----------------------------------------------
def kerb_point(x, y, z, yaw, lane_width, keep_left=True, inset=0.4):
    """The point ``inset`` m inside the kerb-side edge of a lane (left edge on
    keep-left maps). / 車線の路肩側の端から inset 内側の点。"""
    r = math.radians(yaw)
    off = max(lane_width / 2.0 - inset, 0.0)
    nx, ny = (math.sin(r), -math.cos(r)) if keep_left else (-math.sin(r), math.cos(r))
    return (x + nx * off, y + ny * off, z)


def kerb_path(wps, keep_left=True, inset=0.4, min_gap=0.3):
    """``wps``: [(x, y, z, yaw, lane_width)] along the lanes -> kerb-side points."""
    out = []
    for w in wps:
        p = kerb_point(w[0], w[1], w[2], w[3], w[4], keep_left, inset)
        if not out or math.hypot(p[0] - out[-1][0], p[1] - out[-1][1]) >= min_gap:
            out.append(p)
    return out


def path_length(pts):
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))


def astar(start, neighbors, is_goal, h, max_cost=1e9, max_nodes=20000):
    """Generic A*: ``start`` = (key, node); ``neighbors(node)`` -> [(key, node,
    cost)]; returns the node list or None. / 汎用 A*。"""
    k0, n0 = start
    best, prev, nodes = {k0: 0.0}, {}, {k0: n0}
    heap, i = [(h(n0), 0, k0)], 1
    seen = 0
    while heap:
        f, _, k = heapq.heappop(heap)
        node = nodes[k]
        g = best[k]
        if f - h(node) > g + 1e-9:
            continue
        if is_goal(node):
            out = [node]
            while k in prev:
                k = prev[k]
                out.append(nodes[k])
            return out[::-1]
        seen += 1
        if seen > max_nodes:
            return None
        for k2, n2, c in neighbors(node):
            g2 = g + c
            if g2 > max_cost or g2 >= best.get(k2, 1e18):
                continue
            best[k2], prev[k2], nodes[k2] = g2, k, n2
            heapq.heappush(heap, (g2 + h(n2), i, k2))
            i += 1
    return None


def _frame(hub_xy, hub_yaw, p):
    r = math.radians(hub_yaw)
    dx, dy = p[0] - hub_xy[0], p[1] - hub_xy[1]
    return dx * math.cos(r) + dy * math.sin(r), -dx * math.sin(r) + dy * math.cos(r)


def _world(hub_xy, hub_yaw, a, l):
    r = math.radians(hub_yaw)
    return hub_xy[0] + a * math.cos(r) - l * math.sin(r), hub_xy[1] + a * math.sin(r) + l * math.cos(r)


def join_hub(path, hub_xy, hub_yaw, ramp_foot, margin=0.5, half=(HUB_HALF_L, HUB_HALF_W)):
    """A kerb path that starts beside the hub, made to start at the ramp foot
    and run clear of the hub's body (along its side at the ramp foot's offset)
    until past its end. / 経路の始点をスロープ下にし、車体を避けて合流させる。"""
    # ``half``: the real vehicle's half length / width (the LMP-4 stand-in is
    # bigger than the LMH-3) / 実車の半長・半幅
    half_l, half_w = half[0] + POD_HL + margin, half[1] + POD_HW + 0.15
    keep = list(path)
    while keep:
        a, l = _frame(hub_xy, hub_yaw, keep[0])
        if abs(a) < half_l and abs(l) < half_w + 1.0:
            keep.pop(0)
        else:
            break
    fa, fl = _frame(hub_xy, hub_yaw, ramp_foot)
    if not keep:
        return [tuple(ramp_foot)]
    na, _ = _frame(hub_xy, hub_yaw, keep[0])
    side_a = half_l if na >= fa else -half_l
    sx, sy = _world(hub_xy, hub_yaw, side_a, fl)
    return [tuple(ramp_foot), (sx, sy, ramp_foot[2])] + keep


def pad_route(frame, foot, lane_pt, lane_y=1.0, z=0.0):
    """The station route from a pod's ramp foot to a pad's lane point (the web
    viewer's approach: along the pad lane heading -x, the pad on its left).
    ``frame`` = the station's (x, y, z, yaw). Ahead off the ramp onto the
    hub-side lane (local y STN_LANE_Y), along it past the plug unit and the DC
    cabinet, a U-turn behind the pads (centre x STN_TURN_X) onto the pad lane
    (local y ``lane_y``), back along it to ``lane_pt``. World [(x, y, z)];
    reversed it is the way back. / ランプ下からパッド前までの構内経路
    （ハブ側の通路 → パッド奥で U ターン → パッドの通路）。逆順で帰り道。"""
    fxy, fyaw = (frame[0], frame[1]), frame[3]
    bx, _ = _frame(fxy, fyaw, foot)
    lx, ly = _frame(fxy, fyaw, lane_pt)
    tx = max(STN_TURN_X, lx + 0.7)
    r, cy = (lane_y - STN_LANE_Y) / 2.0, (lane_y + STN_LANE_Y) / 2.0
    loc = [(bx, STN_LANE_Y), (tx, STN_LANE_Y)]
    loc += [(tx + r * math.cos(math.radians(a)), cy + r * math.sin(math.radians(a))) for a in range(-75, 90, 15)]
    loc += [(tx, lane_y), (lx, ly)]
    return [_world(fxy, fyaw, a, l) + (z,) for a, l in loc]


# -- obstacles / 障害物 ------------------------------------------------------------
def ahead_points(pos, path, i, reach=BLOCK_REACH, step=0.5):
    """Points on the path from ``pos`` up to ``reach`` m ahead. / 前方の経路点。"""
    out, here, left = [], (pos[0], pos[1]), reach
    for p in path[i:]:
        d = math.hypot(p[0] - here[0], p[1] - here[1])
        while d > 1e-6 and left > 0:
            s = min(step, d, left)
            here = (here[0] + (p[0] - here[0]) * s / d, here[1] + (p[1] - here[1]) * s / d)
            out.append(here)
            left -= s
            d = math.hypot(p[0] - here[0], p[1] - here[1])
        if left <= 0:
            break
    return out


def blocked(points, obstacles, pod_hw=POD_HW):
    """First obstacle touching the pod's sweep over ``points``: obstacles are
    (kind, x, y, yaw, ex, ey); walkers count as 0.35 m discs, vehicles as their
    box. / 掃引範囲に入る最初の障害物。"""
    for ob in obstacles:
        kind, ox, oy, oyaw, ex, ey = ob
        r = math.radians(oyaw)
        c, s = math.cos(r), math.sin(r)
        for p in points:
            dx, dy = p[0] - ox, p[1] - oy
            if kind == "walker":
                if dx * dx + dy * dy < (pod_hw + 0.35) ** 2:
                    return ob
            else:
                lx, ly = dx * c + dy * s, -dx * s + dy * c
                if abs(lx) < ex + pod_hw + 0.1 and abs(ly) < ey + pod_hw + 0.1:
                    return ob
    return None


# -- planning: distribution and meeting points / 配布地点・合流地点 -----------------
def _d2(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def snap(points, cands):
    """Each point -> the nearest kerb candidate (like meeting points)."""
    if not cands:
        return [tuple(p) + ((0.0,) if len(p) == 2 else ()) for p in points]
    return [min(cands, key=lambda c, p=p: _d2(c, p)) for p in points]


def plan_groups(orders, cands, reach, fixed=False, detour=1.4):
    """Split ``orders`` into [(stop point, [orders])]: every order within
    ``reach`` m of pod path (straight line x ``detour``) of its stop.
    ``fixed``: ``cands`` are the hub's distribution points (pick the best
    one); else they are every kerb point and the stop is the one nearest the
    group's centroid. An order no stop can reach gets a stop next to it.
    注文を配布地点ごとに分ける（ポッドの到達距離内）。"""
    rest = list(orders)
    out = []
    if not cands:
        return [(o.drop, [o]) for o in rest]
    ok = lambda p, o: _d2(p, o.drop) * detour <= reach
    while rest:
        if fixed:
            p = max(cands, key=lambda c: (sum(ok(c, o) for o in rest),
                                          -sum(_d2(c, o.drop) for o in rest if ok(c, o))))
        else:
            cx = sum(o.drop[0] for o in rest) / len(rest)
            cy = sum(o.drop[1] for o in rest) / len(rest)
            p = min(cands, key=lambda c: _d2(c, (cx, cy)))
        got = [o for o in rest if ok(p, o)]
        if not got:
            o = rest[0]
            near = min(cands, key=lambda c: _d2(c, o.drop))
            if fixed or not ok(near, o):
                near = min(cands, key=lambda c: _d2(c, o.drop))
            p, got = near, [o] + [q for q in rest[1:] if ok(near, q)]
        out.append((p, got))
        rest = [o for o in rest if o not in got]
    return out


def choose_meet(orders, meets, reach, here=None, detour=1.4):
    """The meeting point the pods of ``orders`` return to: the one with the
    least pod travel that every pod can reach; None if none can.
    合流地点（全ポッドが届く中で移動量が最小のもの）。"""
    best, best_cost = None, None
    for m in meets:
        if any(_d2(m, o.drop) * detour > reach for o in orders):
            continue
        cost = sum(_d2(m, o.drop) for o in orders) + (0.2 * _d2(m, here) if here else 0.0)
        if best_cost is None or cost < best_cost:
            best, best_cost = m, cost
    return best


# -- the pod's battery and failures / 電池と故障 ---------------------------------
class PodPower(object):
    """Mini pod pack: % per km, per handover, idle; charges docked from the
    hub's battery. / ミニポッドの電池（ハブから充電）。"""

    def __init__(self, kwh=1.0, per_km=5.0, per_drop=0.8, idle=0.004, charge=0.35,
                 reserve=15.0, range_m=1500.0):
        self.kwh, self.per_km, self.per_drop, self.idle = kwh, per_km, per_drop, idle
        self.charge, self.reserve, self.range_m = charge, reserve, range_m

    def trip_need(self, out_m, back_m=None):
        km = (out_m + (out_m if back_m is None else back_m)) / 1000.0
        return self.per_km * km + self.per_drop + self.reserve

    def range_left(self, level):
        return max(level - self.reserve, 0.0) / max(self.per_km, 1e-6) * 1000.0


class PodHazards(object):
    """Rare random faults while moving (per hour of driving), like the drones'
    Hazards. / 走行中のまれな故障。"""

    KINDS = (("motor", 0.4), ("lidar", 0.3), ("collision", 0.3))

    def __init__(self, random_on=True, per_hour=0.3, recover_s=90.0, fail_timeout=120.0):
        self.random, self.per_hour = random_on, per_hour
        self.recover_s, self.fail_timeout = recover_s, fail_timeout

    def roll(self, rng, dt):
        if not self.random or self.per_hour <= 0 or rng.random() >= self.per_hour / 3600.0 * dt:
            return None
        x, acc = rng.random(), 0.0
        for k, w in self.KINDS:
            acc += w
            if x < acc:
                return k
        return self.KINDS[-1][0]


def damage_for(why, rng_or_seed=0):
    """Broken / damaged parts for a fault (ids of the GLB / mount keys)."""
    kind = (why or "").split(":")[0]
    if kind == "motor":
        n = int(why.split(":")[1]) if ":" in why else 1
        return {"parts": {"Wheel%d" % n: "broken"}, "why": "motor %d failed" % n}
    if kind == "lidar":
        return {"parts": {"lidar": "broken"}, "why": "lidar stopped returning points"}
    if kind == "collision":
        return {"parts": {"cam_front": "damaged", "ultrasonic": "damaged", "Wheel1": "damaged",
                          "Wheel4": "damaged"}, "why": "front bumper hit an obstacle"}
    if kind == "battery":
        return {"parts": {"battery": "broken"}, "why": "battery ran flat"}
    return {"parts": {}, "why": why or "fault"}


# -- one Mini Pod / ミニポッド 1 台 --------------------------------------------------
def _wrap(a):
    return (a + 180.0) % 360.0 - 180.0


class MiniPod(object):
    """One Mini Delivery Pod: kinematic, driven by a queue of moves (path,
    spin, reverse, back, wait), with the HTML's part tracks (lid, shutter,
    flag, belt). The hub crew (HubCrew) gives it its missions.
    ミニポッド 1 台（動作キュー、部品トラック）。"""

    def __init__(self, pid, hub, bay, rng=None, power=None, hazards=None, speed=V_POD):
        self.id, self.hub, self.bay = pid, hub, bay      # bay 1..3
        self.rng = rng or random.Random(pid)
        self.power = power or PodPower()
        self.hz = hazards or PodHazards(random_on=False)
        self.v_max = speed
        self.status = DOCKED
        self.x = self.y = self.z = 0.0
        self.yaw = 0.0
        self.speed = 0.0
        self.battery = 100.0
        self.order = None            # the parcel it carries / 運搬中の注文
        self.mission = None          # {"kind", "stop", "face_yaw", "back", "drop", "box"}
        self.lid, self.shutter, self.flag, self.belt = Track(), Track(), Track(), Track()
        self.odo_l = self.odo_r = 0.0
        self.trips, self.km = 0, 0.0
        self.fail, self.damage, self.down_t = None, None, None
        self.moves = []
        self.path, self.i = [], 0
        self.events = []
        self._block_t, self._tries = 0.0, 0
        self._resume = None          # status to go back to after "stuck" / 回復後の状態
        self.block_by = None
        self.parcel_out = False      # parcel already handed over / 受け渡し済み
        self._routes = []
        self.t = 0.0
        self.handover_t = None
        self.pad = None              # station pad M1..M3 while on a pad trip / 充電パッド番号
        self._in_step, self._outside = False, []

    # -- helpers
    @property
    def pos(self):
        return (self.x, self.y, self.z)

    def out(self):
        return self.status in OUT

    def _emit(self, ev, **kw):
        # outside step() (a forced fault): kept for the next step's events
        # step() の外で起きた出来事は次の step で返す
        (self.events if self._in_step else self._outside).append(dict(kw, ev=ev, pod=self))

    def set_path(self, pts, v=None, check=True):
        self.moves = [{"k": "path", "pts": [tuple(p) for p in pts], "v": v or self.v_max, "check": check}]
        self.path, self.i = self.moves[0]["pts"], 0

    def force_fault(self, t, kind):
        """Fail now (tests, dashboard). / 強制故障。"""
        if kind == "motor":
            kind = "motor:%d" % self.rng.randint(1, 6)
        self.status, self.speed, self.fail = CRASHED, 0.0, kind
        self.damage, self.down_t, self.moves = damage_for(kind), t, []
        self._emit("fault", why=kind)

    # -- missions (from the crew) / 任務
    def launch(self, t, ramp, routes, order, mission):
        """Leave the bay: down the ramp (``ramp`` = [dock, ramp foot]) then the
        ``routes`` (one path, or two with a stop at a crashed pod in between
        to take its parcel) to the mission's stop point. / 発進。"""
        self.status, self.order, self.mission = ROLLING_OUT, order, mission
        self.parcel_out = False
        self.moves = [{"k": "path", "pts": [tuple(p) for p in ramp], "v": V_RAMP, "check": False}]
        self.path, self.i = self.moves[0]["pts"], 0
        self._routes = [[tuple(p) for p in r] for r in routes]
        self.flag.ramp(t + 0.9, 0.8, 1.0)
        self._emit("out")

    def go_home(self, t, route, parked=True):
        self.status = RETURNING
        self.set_path(route)
        self._home_parked = parked

    def dock_now(self, t, dock_xyz, dock_yaw):
        """At the ramp foot: turn to face the kerb, wait for the door, reverse
        up into the bay. / 扉を向き、扉が開くのを待って後退で格納。"""
        self.status = DOCKING
        self.flag.ramp(t, 0.7, 0.0)
        self.moves = [{"k": "spin", "yaw": dock_yaw}, {"k": "wait_door"},
                      {"k": "rev", "to": tuple(dock_xyz), "v": V_REV}]

    def pad_trip(self, t, t_out, ramp, route, pad):
        """To a charging pad of a station (HubCrew.station_begin): wait in the
        bay until ``t_out`` (door coming down), down the ramp (``ramp`` =
        [dock, ramp foot]), the station ``route`` to the pad's lane point, turn
        to the pad's yaw (nose +y local) and reverse onto it (``pad`` = (x, y,
        z, yaw)). / 充電パッドへ（扉待ち → ランプ → 構内経路 → 旋回 → 後退）。"""
        x, y, z, yaw = pad
        self.status, self.order, self.mission, self.parcel_out = TO_PAD, None, None, False
        self.moves = [{"k": "wait", "until": t_out},
                      {"k": "path", "pts": [tuple(p) for p in ramp], "v": V_RAMP, "check": False, "emit": "clear"},
                      {"k": "path", "pts": [tuple(p) for p in route], "v": V_STN, "check": False},
                      {"k": "spin", "yaw": yaw}, {"k": "rev", "to": (x, y, z), "v": V_REV}]
        self.path, self.i = [], 0
        self.flag.ramp(t_out + 0.9, 0.8, 1.0)

    def pad_leave(self, t, route, foot, dock_yaw):
        """Off the pad: forward onto the pad lane, the station route back to the
        hub-side lane opposite its bay, turn to face the door and reverse to the
        ramp foot; the crew then docks it. / パッドを出てランプ下へ戻る。"""
        self.status = FROM_PAD
        self.moves = [{"k": "path", "pts": [tuple(p) for p in route], "v": V_STN, "check": False},
                      {"k": "spin", "yaw": dock_yaw}, {"k": "rev", "to": tuple(foot), "v": V_REV}]
        self.path, self.i = self.moves[0]["pts"], 0

    def _start_handover(self, t):
        m = self.mission or {}
        self.status = HANDOVER
        self.moves = [{"k": "spin", "yaw": m.get("face_yaw", self.yaw)}]
        if m.get("back", 0.0) > 0.01:
            r = math.radians(m.get("face_yaw", self.yaw))
            b = m["back"]
            self.moves.append({"k": "rev", "to": (self.x - math.cos(r) * b, self.y - math.sin(r) * b,
                                                  self.z), "v": V_REV})
        self.moves.append({"k": "handover"})

    # -- stepping / 1 ステップ
    def step(self, t, dt, env):
        """Advance by dt; returns the events of this step. / dt 進めて出来事を返す。"""
        self.t = t
        self.events, self._outside = self._outside, []
        self._in_step = True
        try:
            self._step(t, dt, env)
        finally:
            self._in_step = False
        return self.events

    def _step(self, t, dt, env):
        if self.status == CRASHED:
            self.speed = 0.0
            return
        if self.status in (DOCKED, LOADING, ON_PAD):
            self.speed = 0.0
            return
        if self.status == WAITING:
            self.speed = 0.0
            self.battery = max(self.battery - self.power.idle * dt, 0.0)
            return
        moving = self.speed > 0.05 or (self.moves and self.moves[0]["k"] in ("spin", "rev", "back"))
        if moving:
            kind = self.hz.roll(self.rng, dt)
            if kind:
                self.force_fault(t, kind)
                return
        self.battery = max(self.battery - self.power.idle * dt, 0.0)
        if self.battery <= 0.0:
            self.force_fault(t, "battery")
            return
        if not self.moves:
            self._moves_done(t, env)
            return
        m = self.moves[0]
        k = m["k"]
        if k == "path":
            if self.path is not m["pts"]:
                self.path, self.i = m["pts"], 0
            if self._follow(t, dt, env, m):
                self.moves.pop(0)
                if m.get("emit"):            # e.g. "clear" off the ramp / ランプを降りた等
                    self._emit(m["emit"])
        elif k == "spin":
            if self._spin(dt, m["yaw"]):
                self.moves.pop(0)
        elif k in ("rev", "back"):
            if self._reverse(dt, m):
                self.moves.pop(0)
                if k == "back":
                    self._after_back(t, env)
        elif k == "wait_door":
            self.speed = 0.0
            if env.door(self.hub, self.bay) >= 0.98:
                self.moves.pop(0)
        elif k == "wait":
            self.speed = 0.0
            if t >= m["until"]:
                self.moves.pop(0)
        elif k == "pickup":                 # take the parcel off a crashed pod / 故障機から回収
            self.speed = 0.0
            if "until" not in m:
                m["until"] = t + 4.0
                self._emit("rescue")
            elif t >= m["until"]:
                self.moves.pop(0)
                if self.moves and self.moves[0]["k"] == "path":
                    self.path, self.i = self.moves[0]["pts"], 0
        elif k == "handover":
            self._handover(t, env, m)
        if not self.moves:
            self._moves_done(t, env)
        return

    def _move_by(self, d, heading_deg, reverse=False):
        r = math.radians(heading_deg)
        self.x += math.cos(r) * d * (-1 if reverse else 1)
        self.y += math.sin(r) * d * (-1 if reverse else 1)
        s = -d if reverse else d
        self.odo_l += s
        self.odo_r += s
        self.km += d / 1000.0
        self.battery = max(self.battery - self.power.per_km * d / 1000.0, 0.0)

    def _spin(self, dt, yaw):
        self.speed = 0.0
        e = _wrap(yaw - self.yaw)
        step = SPIN_DPS * dt
        if abs(e) <= max(step, 1.0):
            turn = e
            done = True
        else:
            turn = math.copysign(step, e)
            done = False
        self.yaw = _wrap(self.yaw + turn)
        arc = math.radians(turn) * WHEEL_Y
        self.odo_l -= arc
        self.odo_r += arc
        return done

    def _follow(self, t, dt, env, m):
        pts = m["pts"]
        if self.i >= len(pts):
            self.speed = 0.0
            return True
        if m.get("check", True):
            ob = env.blocked(self, ahead_points(self.pos, pts, self.i))
        else:
            ob = None
        if ob is not None:
            self.speed = 0.0
            self.block_by = ob[0]
            self._block_t += dt
            if self._block_t > STUCK_S:
                self._recover(t, env)
            return False
        self.block_by = None
        self._block_t = 0.0
        if self.status == STUCK:
            self.status, self._resume = self._resume or TO_DOOR, None
        v_max = m.get("v", self.v_max)
        if m.get("check", True) and env.slow(self):
            v_max = min(v_max, V_SLOW)
        p = pts[self.i]
        want = math.degrees(math.atan2(p[1] - self.y, p[0] - self.x))
        d_to = math.hypot(p[0] - self.x, p[1] - self.y)
        if d_to > 0.05 and abs(_wrap(want - self.yaw)) > 60.0:
            self._spin(dt, want)            # skid-steer: turn on the spot / その場旋回
            return False
        self.speed = min(v_max, self.speed + ACCEL * dt)
        left = self.speed * dt
        while left > 1e-6 and self.i < len(pts):
            p = pts[self.i]
            dx, dy = p[0] - self.x, p[1] - self.y
            d = math.hypot(dx, dy)
            if d < 1e-6:
                self.i += 1
                continue
            s = min(left, d)
            head = math.degrees(math.atan2(dy, dx))
            turn = _wrap(head - self.yaw)
            self.yaw = _wrap(self.yaw + clamp(turn, -SPIN_DPS * dt, SPIN_DPS * dt))
            arc = math.radians(clamp(turn, -SPIN_DPS * dt, SPIN_DPS * dt)) * WHEEL_Y
            self.odo_l -= arc
            self.odo_r += arc
            f = s / d
            self.z += (p[2] - self.z) * f
            self._move_by(s, head)
            left -= s
            if s >= d - 1e-6:
                self.x, self.y = p[0], p[1]
                self.i += 1
        return self.i >= len(pts)

    def _reverse(self, dt, m):
        if m["k"] == "back":
            r = math.radians(self.yaw)
            m.setdefault("to", (self.x - math.cos(r) * m.get("dist", 1.0),
                                self.y - math.sin(r) * m.get("dist", 1.0), self.z))
        to = m["to"]
        d = math.hypot(to[0] - self.x, to[1] - self.y)
        self.speed = m.get("v", V_REV)
        s = min(self.speed * dt, d)
        if d > 1e-6:
            f = s / d
            self.x += (to[0] - self.x) * f
            self.y += (to[1] - self.y) * f
            self.z += (to[2] - self.z) * f
            self.odo_l -= s
            self.odo_r -= s
            self.km += s / 1000.0
            self.battery = max(self.battery - self.power.per_km * s / 1000.0, 0.0)
        if d - s < 1e-4:
            self.x, self.y, self.z = to
            self.speed = 0.0
            return True
        return False

    def _recover(self, t, env):
        """Blocked too long: back up 1 m and re-plan; after MAX_TRIES jump to
        the next route point past the obstacle (logged). / 立ち往生からの回復。"""
        self._block_t = 0.0
        if self.status != STUCK:
            self._resume, self.status = self.status, STUCK
        self._tries += 1
        if self._tries > MAX_TRIES:
            self._tries = 0
            pts, i = self.path, self.i
            here, gone = (self.x, self.y), 0.0
            while i < len(pts) - 1 and gone < SKIP_M:
                gone += math.hypot(pts[i][0] - here[0], pts[i][1] - here[1])
                here = pts[i]
                i += 1
            p = pts[min(i, len(pts) - 1)]
            self.x, self.y, self.z = p
            self.i = min(i + 1, len(pts))
            self.status, self._resume = self._resume or TO_DOOR, None
            self._emit("stuck", detail="blocked by a %s %d times: moved %.1f m ahead"
                       % (self.block_by or "?", MAX_TRIES, gone))
            return
        self.moves.insert(0, {"k": "back", "dist": 1.0, "v": 0.5})
        self._emit("stuck_try", detail="blocked by a %s: backing up 1 m (try %d)"
                   % (self.block_by or "?", self._tries))

    def _after_back(self, t, env):
        if not self.moves or self.moves[0]["k"] != "path":
            return
        target = self.moves[0]["pts"][-1]
        new = env.route(self, self.pos, target)
        if new:
            self.moves[0]["pts"] = [tuple(p) for p in new]
            self.path, self.i = self.moves[0]["pts"], 0
        else:
            self.i = max(self.i - 1, 0)

    def _handover(self, t, env, m):
        """Shutter up, belt pushes the parcel out (onto the ground in front of
        the place, or into the box's intake), shutter down. mini.js aptB times.
        シャッターを上げ、ベルトで荷物を押し出し、シャッターを下ろす。"""
        self.speed = 0.0
        box = (self.mission or {}).get("kind") == ORDER_BOX
        if "t0" not in m:
            m["t0"] = t
            self.handover_t = t
            self.shutter.ramp(t + 0.1, 0.6, 1.0)
            self.belt.ramp(t + 0.7, 1.4, 1.0)
            self.shutter.ramp(t + 2.2, 0.6, 0.0)
            self.belt.ramp(t + 2.8, 0.01, 0.0)
            self._emit("at_door", box=box)
        el = t - m["t0"]
        if not self.parcel_out and el >= 2.1:
            self.parcel_out = True
            self.battery = max(self.battery - self.power.per_drop, 0.0)
            self._emit("handover", box=box)
        if el >= 2.9:
            self.moves.pop(0)

    def flap(self, t):
        """Box intake flap value while handing over into a box (0..1)."""
        if self.handover_t is None or (self.mission or {}).get("kind") != ORDER_BOX:
            return 0.0
        el = t - self.handover_t
        if el < 0.0:
            return 0.0
        return smooth(clamp(el / 0.5, 0.0, 1.0)) * (1.0 - smooth(clamp((el - 2.3) / 0.5, 0.0, 1.0)))

    def parcel_offset(self, t):
        """How far the belt has pushed the parcel out behind the pod (m)."""
        return PUSH_OUT * self.belt.at(t)

    def _moves_done(self, t, env):
        st = self.status
        if st == STUCK:
            st = self.status = self._resume or TO_DOOR
            self._resume = None
        if st == ROLLING_OUT:
            self.status = TO_DOOR
            self._emit("clear")
            self.moves = []
            for k, r in enumerate(self._routes):
                if k:
                    self.moves.append({"k": "pickup"})
                self.moves.append({"k": "path", "pts": r, "v": self.v_max, "check": True})
            self.path, self.i = (self.moves[0]["pts"], 0) if self.moves else ([], 0)
        elif st == TO_DOOR:
            self._start_handover(t)
        elif st == HANDOVER:
            self.handover_t = None
            self.order = None
            self.trips += 1
            self.status = WAITING             # until the crew sends it home / 帰還指示待ち
            self._emit("done")
        elif st == RETURNING:
            self._emit("home")
        elif st == DOCKING:
            self.status = DOCKED
            self.speed = 0.0
            self.mission = None
            self._emit("docked")
        elif st == TO_PAD:
            self.status, self.speed = ON_PAD, 0.0
            self._emit("on_pad")
        elif st == FROM_PAD:
            self._emit("pad_home")            # at the ramp foot: the crew docks it / ランプ下に到着

    def state(self, t, path_n=30):
        """The published dict (sensors / damage are added by the caller)."""
        r1 = lambda v: round(v, 1)
        pts = self.path[self.i:self.i + path_n] if self.out() else []
        m = self.mission or {}
        dest = m.get("drop")
        return {"id": self.id, "hub": self.hub, "bay": self.bay, "status": self.status,
                "x": r1(self.x), "y": r1(self.y), "z": round(self.z, 2), "yaw": r1(self.yaw),
                "speed": round(self.speed, 2), "battery": r1(self.battery),
                "order": self.order.id if self.order is not None else None,
                "dest": [r1(dest[0]), r1(dest[1])] if dest and self.order is not None else None,
                "kind": m.get("kind") if self.order is not None else None,
                "lid": round(self.lid.at(t), 2), "shutter": round(self.shutter.at(t), 2),
                "flag": round(self.flag.at(t), 2), "trips": self.trips, "km": round(self.km, 3),
                "fail": self.fail, "path": [[r1(p[0]), r1(p[1])] for p in pts],
                "damage": self.damage, "pad": self.pad}


# -- the hub's crew: which pod goes where / ハブの指令 --------------------------------
class HubCrew(object):
    """Runs a hub's three pods at its stops (server-free).

    ``env`` (CARLA side, or a stub in the tests) answers:
      hub_pose() -> (x, y, z, yaw); parked() -> bool
      route(pod, a, b) -> [(x, y, z)] kerb-side path a -> b (b included) or None
      mission(order) -> {"kind", "stop", "face_yaw", "back", "drop", "box"}
      blocked(pod, points) -> obstacle tuple or None; slow(pod) -> bool
      door(hub, bay) -> door value; bay(n) -> (dock (x, y, z, yaw), ramp foot
      (x, y, z)) in the hub frame; ground(x, y, z_hint) -> ground z
    ハブの 3 台のポッドへの指令（どの注文をどのポッドで、いつ発進・帰還・格納）。"""

    def __init__(self, vid, pods, power, hazards=None, keep_left=True):
        self.vid, self.pods, self.power = vid, pods, power
        self.hz = hazards or PodHazards(random_on=False)
        self.mech = HubMech()
        self.keep_left = keep_left
        self.queue = []              # orders still to send from this stop / 未発送
        self.deferred = []           # out of pod range from here / 範囲外
        self.cycle = None            # {"t0", "bays": {i: (pod, order, mission, route)}, "out": {i: t}}
        self.rescues = []            # [{"order", "pos", "t"}] parcels on crashed pods / 故障機の荷物
        self.phase = "idle"
        self.home = None             # (x, y, z): where pods return while the hub drives / 合流点
        self.meet = None
        self.hub_draw = 0.0          # % of the pod packs charged since last read / 充電量
        self.feed = True             # the hub feeds its docked pods (off while its own pack is low) / 給電中
        self.station = None          # charging-station pad session (station_begin) / パッド充電の状態
        self.events = []

    # -- geometry
    def bay_world(self, pod, env):
        """(dock (x, y, z), dock yaw, ramp foot (x, y, z)) of the pod's bay, world."""
        x, y, z, yaw = env.hub_pose()
        (dx, dy, dz, dyaw), (rx, ry, rz) = env.bay(pod.bay)
        wx, wy = _world((x, y), yaw, dx, dy)
        fx, fy = _world((x, y), yaw, rx, ry)
        return (wx, wy, z + dz), _wrap(yaw + dyaw), (fx, fy, env.ground(fx, fy, z + rz))

    def _emit(self, ev, **kw):
        self.events.append(dict(kw, ev=ev))

    @staticmethod
    def _half(env):
        f = getattr(env, "hub_half", None)
        return f() if f is not None else (HUB_HALF_L, HUB_HALF_W)

    def pods_out(self):
        return [p for p in self.pods if p.status not in (DOCKED, LOADING)]

    def can_leave(self, t):
        """May the hub drive off now (meeting point)? Every pod still out is on
        its own (none on the ramp), the cycle is over and the doors are shut.
        ハブが移動してよいか（スロープ上にポッドが無く、扉が閉じている）。"""
        return (self.cycle is None and self.mech.doors_shut(t) and
                not any(p.status in (ROLLING_OUT, DOCKING, LOADING) for p in self.pods))

    def hub_leaving(self, t, env, home):
        """The hub drives to ``home`` (meeting point): pods coming back head
        there and wait. / ハブが合流地点へ移動する。"""
        self.home = tuple(home)
        for p in self.pods:
            if p.status == RETURNING:
                r = env.route(p, p.pos, self.home) or [self.home]
                p.go_home(t, r, parked=False)

    def busy(self):
        return bool(self.queue or self.cycle or self.rescues or
                    any(p.status != DOCKED for p in self.pods) or
                    not self.mech.doors_shut(getattr(self, "t", 0.0)))

    def start(self, t, orders, meet=None):
        """Orders to send from the stop the hub is at (or driving to)."""
        self.queue = [o for o in orders if o not in self.queue] + self.queue
        self.meet = meet

    # -- every tick / 毎 tick
    def step(self, t, dt, env):
        self.events = []
        self.t = t
        parked = env.parked()
        for p in self.pods:
            if p.status in (DOCKED, LOADING) and not p.out():
                d, yaw, _ = self.bay_world(p, env)
                p.x, p.y, p.z, p.yaw = d[0], d[1], d[2], yaw
                if p.status == DOCKED and p.battery < 100.0 and self.feed:
                    q = min(self.power.charge * dt, 100.0 - p.battery)
                    p.battery += q
                    self.hub_draw += q * self.power.kwh
            for e in p.step(t, dt, env):
                self._pod_event(t, e, env)
        for p in self.pods:                        # door for pods coming home / 帰還時の扉
            if p.status == DOCKING and p.moves and p.moves[0]["k"] in ("spin", "wait_door"):
                self.mech.open_door(t, p.bay - 1)
            if p.status == CRASHED and t - (p.down_t or t) > self.hz.recover_s:
                self._recover(t, p, env)
        if parked:
            for p in self.pods:                    # hub parked again: come to its ramp / 再合流
                if p.status == WAITING or (p.status == RETURNING and not getattr(p, "_home_parked", True)):
                    self._send_home(t, p, env)
        for r in list(self.rescues):
            if t - r["t"] > self.hz.fail_timeout:
                self.rescues.remove(r)
                self._emit("order_failed", order=r["order"], reason="minipod_fault: no pod to recover it")
        if self.cycle is not None:
            self._run_cycle(t, env)
        elif parked and (self.queue or self.rescues) and self.station is None:
            self._start_cycle(t, env)            # never from a charging bay / 充電ベイからは出さない
        self._set_phase(t, parked)
        return self.events

    def _set_phase(self, t, parked):
        if self.cycle is not None:
            ph = work_phase(t - self.cycle["t0"])
        elif self.station is not None:
            ph = "station"
        elif not parked and any(p.out() for p in self.pods):
            ph = "to_meet"
        elif any(p.status != DOCKED for p in self.pods):
            ph = "wait_pods"
        elif self.queue:
            ph = "ready"
        else:
            ph = "idle"
        self.phase = ph

    def _start_cycle(self, t, env):
        free = [p for p in self.pods if p.status == DOCKED and
                self.mech.door[p.bay - 1].target() < 0.5]
        if not free:
            return
        jobs = {}
        work = [(r, r["order"], r["pos"]) for r in self.rescues] + [(None, o, None) for o in self.queue]
        for r, o, via in work:
            left = [q for q in free if q.bay - 1 not in jobs]
            if not left:
                break
            why, plan = "range", None
            for q in sorted(left, key=lambda q: -q.battery):
                why, plan = self._plan(q, o, env, via)
                if plan is not None or why == "range":
                    break
            if plan is not None:
                jobs[plan[0].bay - 1] = plan
                (self.rescues if r is not None else self.queue).remove(r if r is not None else o)
            elif why == "range" and r is None:
                self.queue.remove(o)
                self.deferred.append(o)
                self._emit("deferred", order=o)
            # "battery": wait for the pods to charge / 充電を待つ
        if not jobs:
            return
        outs = self.mech.schedule_load(t, list(jobs))
        for i, (p, o, mission, routes) in jobs.items():
            p.status, p.order, p.mission = LOADING, o, mission
            p.shutter.ramp(t + 16.2 + 0.5 * i, 0.7, 1.0)
            p.shutter.ramp(t + 19.7 + 0.5 * i, 0.7, 0.0)
        self.cycle = {"t0": t, "jobs": jobs, "out": outs, "gone": set()}
        self._emit("cycle", n=len(jobs))

    def _plan(self, pod, order, env, via=None):
        """("ok", (pod, order, mission, routes)) or (why, None): "range" (too
        far for a pod from here) or "battery" (this pod is too low).
        ポッドが届くか・電池が足りるか。"""
        mission = env.mission(order)
        _, _, foot = self.bay_world(pod, env)
        legs = [foot] + ([via] if via is not None else []) + [mission["stop"]]
        routes = []
        for a, b in zip(legs, legs[1:]):
            r = env.route(pod, a, b)
            if not r:
                return "range", None
            routes.append(list(r))
        x, y, _, yaw = env.hub_pose()
        routes[0] = join_hub(routes[0], (x, y), yaw, foot, half=self._half(env))
        out_m = sum(path_length(r) for r in routes)
        if out_m * 2.0 > self.power.range_m:
            return "range", None
        if pod.battery < self.power.trip_need(out_m):
            return "battery", None
        return "ok", (pod, order, mission, routes)

    def _run_cycle(self, t, env):
        c = self.cycle
        for i, (p, o, mission, routes) in c["jobs"].items():
            if i in c["gone"] or t < c["out"][i]:
                continue
            dock, yaw, foot = self.bay_world(p, env)
            p.x, p.y, p.z, p.yaw = dock[0], dock[1], dock[2], yaw
            p.launch(t, [dock, foot], routes, o, mission)
            c["gone"].add(i)
            self._emit("pod_out", pod=p, order=o)
        if len(c["gone"]) == len(c["jobs"]):
            self.cycle = None

    def _send_home(self, t, p, env):
        dock, yaw, foot = self.bay_world(p, env)
        if math.hypot(p.x - foot[0], p.y - foot[1]) < 0.3:
            p.go_home(t, [foot], parked=True)
            return
        r = env.route(p, p.pos, foot)
        if not r:
            r = [foot]
        x, y, _, hyaw = env.hub_pose()
        r = list(reversed(join_hub(list(reversed(r)), (x, y), hyaw, foot, half=self._half(env))))
        p.go_home(t, r, parked=True)

    def _pod_event(self, t, e, env):
        p, ev = e["pod"], e["ev"]
        if ev == "clear":
            self.mech.close_door(t + 0.2, p.bay - 1)
        elif ev == "at_door":
            self._emit("pod_door", pod=p, order=p.order, box=e.get("box"))
        elif ev == "handover":
            self._emit("pod_handover", pod=p, order=p.order, box=e.get("box"))
        elif ev == "done":
            if env.parked():
                self._send_home(t, p, env)
            else:
                target = self.home or env.hub_pose()[:3]
                r = env.route(p, p.pos, target) or [target]
                p.go_home(t, r, parked=False)
            self._emit("pod_back", pod=p)
        elif ev == "home":
            if env.parked():
                dock, yaw, foot = self.bay_world(p, env)
                if math.hypot(p.x - foot[0], p.y - foot[1]) < 0.5:
                    p.dock_now(t, dock, yaw)
                else:
                    self._send_home(t, p, env)
            else:
                p.status = WAITING
        elif ev == "docked":
            self.mech.docked(t, p.bay - 1)
            self._end_trip(p)
            self._emit("pod_docked", pod=p)
        elif ev == "on_pad":
            st = self.station
            k = st["trips"].get(p.id) if st is not None else None
            if k is not None:
                pad = st["pads"][k]
                pad["phase"], pad["t_on"] = "plugging", t
                pad["pins"].ramp(t + 0.2, 0.5, 1.0)        # pins in, then the plug (charge.js) / ピン→プラグ
                pad["plug"].ramp(t + 0.7, 0.6, 1.0)
                self._emit("pod_on_pad", pod=p, pad=k + 1)
        elif ev == "pad_home":
            dock, yaw, _ = self.bay_world(p, env)
            p.dock_now(t, dock, yaw)
        elif ev == "fault":
            if p.order is not None and not p.parcel_out:
                self.rescues.append({"order": p.order, "pos": p.pos, "t": t})
            if self.station is not None:           # its pad is free again; the trip ends at recovery
                for pad in self.station["pads"]:   # パッドを空ける（回収で終了）
                    if pad["pod"] is p:
                        pad["pod"], pad["phase"] = None, "free"
            self._emit("pod_fault", pod=p, why=e.get("why"))
        elif ev == "rescue":
            for q in self.pods:                    # the parcel changes pods / 荷物を移す
                if q is not p and q.status == CRASHED and q.order is p.order:
                    q.order = None
            self._emit("pod_rescue", pod=p, order=p.order)
        elif ev in ("stuck", "stuck_try"):
            self._emit("pod_stuck" if ev == "stuck" else "pod_stuck_try", pod=p, detail=e.get("detail"))

    def _recover(self, t, p, env):
        """A crashed pod is brought back to its bay (service crew). / 回収。"""
        carried = any(q is not p and q.order is p.order for q in self.pods)
        if p.order is not None and not p.parcel_out and not carried:
            self.rescues = [r for r in self.rescues if r["order"] is not p.order]
            self._emit("order_failed", order=p.order, reason="minipod_fault: %s" % p.fail)
        p.order, p.mission, p.moves = None, None, []
        p.status, p.fail, p.damage, p.down_t, p.speed = DOCKED, None, None, None, 0.0
        p.flag.ramp(t, 0.01, 0.0)
        self.mech.docked(t, p.bay - 1)
        self._end_trip(p)
        self._emit("pod_recovered", pod=p)

    # -- charging-station pads (charge_sites, bay 04) / 充電ステーションのパッド
    def station_begin(self, t, env, site, frame, poses, lane, start=PAD_START):
        """The hub has parked in bay 04 of station ``site`` (``frame`` = its
        (x, y, z, yaw)): every docked pod under ``start`` % goes to a pad
        (``poses``: world (x, y, z, yaw) of a pod charging on M1..M3; ``lane``:
        charge_sites pad_lane). The doors open as at a stop. Bay 3's pod leaves
        first and takes M1 (the farthest along the pad lane), so the pods drive
        in one line and none passes a stopped one. Returns the events.
        ベイ 04 で充電を始めたら、電池が start 未満のポッドをパッドへ送る（ベイ 3 → M1 の順）。"""
        self.events = []
        if self.station is not None and self.station["trips"]:
            return self.events                 # a session still has pods out: keep it / 前回の続き
        self.station = st = {"site": site, "frame": tuple(frame), "poses": [tuple(p) for p in poses],
                             "lane": dict(lane or {}), "trips": {}, "left": {}, "recall": False,
                             "pads": [{"pod": None, "phase": "free", "pins": Track(), "plug": Track(),
                                       "t_on": None, "t_off": None} for _ in poses]}
        go = sorted([p for p in self.pods if p.status == DOCKED and p.battery < start and
                     self.mech.door[p.bay - 1].target() < 0.5], key=lambda p: -p.bay)[:len(poses)]
        for k, p in enumerate(go):
            i, s = p.bay - 1, t + 0.5 + STN_STAGGER * k
            self.mech.plug[i].ramp(s, 0.5, 0.0)              # released as at a stop / 停車時と同じ解放
            self.mech.pins[i].ramp(s + 0.5, 0.4, 0.0)
            self.mech.door[i].ramp(s + 1.1, 1.8, 1.0)
            dock, yaw, foot = self.bay_world(p, env)
            foot = (foot[0], foot[1], st["poses"][k][2])     # ramp ends on the station floor / 床の高さ
            p.x, p.y, p.z, p.yaw = dock[0], dock[1], dock[2], yaw
            p.pad_trip(t, s + 3.1, [dock, foot], self._pad_route(foot, k), st["poses"][k])
            p.pad = k + 1
            st["trips"][p.id] = k
            st["pads"][k]["pod"], st["pads"][k]["phase"] = p, "in"
            self._emit("pod_to_pad", pod=p, pad=k + 1)
        return self.events

    def _pad_route(self, foot, k):
        st = self.station
        x, y, z, yaw = st["poses"][k]
        back, r = st["lane"].get("reverse_m", 1.1), math.radians(yaw)
        lane_pt = (x + math.cos(r) * back, y + math.sin(r) * back)    # ahead of the pod on its pad / パッド前
        return pad_route(st["frame"], foot, lane_pt, st["lane"].get("y_local", 1.0), z)

    def station_step(self, t, dt, env, rate):
        """Every tick at the station: pad pins and plug, charging at ``rate``
        %/s, leaving when full or recalled (in an order where no pod passes
        another), bay doors ahead of pods coming home. Returns the events.
        パッドの毎 tick 処理（ピン・プラグ、充電、満充電か呼び戻しで発進、帰還時の扉）。"""
        self.events = []
        st = self.station
        if st is None:
            return self.events
        want = []
        for k, pad in enumerate(st["pads"]):
            p = pad["pod"]
            if p is None:
                continue
            if pad["phase"] == "plugging" and t >= pad["t_on"] + PAD_PLUG_S:
                pad["phase"] = "charging"
            if pad["phase"] == "charging":
                p.battery = min(p.battery + rate * dt, 100.0)
                if p.battery >= PAD_FULL or st["recall"]:
                    want.append((p.bay, k, p))
            elif pad["phase"] == "unplugging" and t >= pad["t_off"] + PAD_UNPLUG_S:
                _, yaw, foot = self.bay_world(p, env)
                foot = (foot[0], foot[1], st["poses"][k][2])
                p.pad_leave(t, list(reversed(self._pad_route(foot, k))), foot, yaw)
                pad["phase"] = "out"
                self._emit("pod_off_pad", pod=p, pad=k + 1,
                           why="full" if p.battery >= PAD_FULL else "recalled")
            elif pad["phase"] == "out":
                x, y = st["poses"][k][:2]
                if p.status != FROM_PAD or math.hypot(p.x - x, p.y - y) > 1.5:
                    pad["pod"], pad["phase"] = None, "free"     # cleared the pad / パッドを離れた
        for _, k, p in sorted(want, key=lambda w: w[0]):
            if self._may_leave(t, p):
                pad = st["pads"][k]
                pad["phase"], pad["t_off"] = "unplugging", t
                pad["plug"].ramp(t, 0.5, 0.0)                  # plug out, then the pins / プラグ→ピン
                pad["pins"].ramp(t + 0.5, 0.4, 0.0)
                st["left"][p.id] = t
        for p in self.pods:                                    # open the bay ahead of it / 扉を先に開く
            if p.status == FROM_PAD and p.id in st["trips"]:
                _, _, foot = self.bay_world(p, env)
                if math.hypot(p.x - foot[0], p.y - foot[1]) < 2.5:
                    self.mech.open_door(t, p.bay - 1)
        return self.events

    def _may_leave(self, t, pod):
        """May ``pod`` leave its pad now? Not while a pod still drives in;
        behind another leaving pod only if that one docks in a lower bay
        (farther along the hub-side lane, so it is never passed at its ramp)
        and left STN_GAP_S before. / パッドを出てよいか（追い越しの無い順と間隔）。"""
        st = self.station
        for q in self.pods:
            if q is pod or q.id not in st["trips"]:
                continue
            if q.status == TO_PAD:
                return False
            if q.id in st["left"] and (q.bay > pod.bay or t - st["left"][q.id] < STN_GAP_S):
                return False
        return True

    def station_recall(self, t):
        """The hub has charged: every pod still on a pad comes back now (a pod
        that has not rolled out yet stays in its bay). / 呼び戻し。"""
        st = self.station
        if st is None:
            return
        st["recall"] = True
        for p in self.pods:
            if p.id in st["trips"] and p.status == TO_PAD and p.moves and p.moves[0]["k"] == "wait":
                p.status, p.moves, p.speed = DOCKED, [], 0.0
                p.flag.ramp(t, 0.01, 0.0)
                self.mech.docked(t, p.bay - 1)
                self._end_trip(p)

    def station_done(self, t):
        """No pod out on station business: all back in their bays, doors shut.
        全ポッドが格納され扉が閉じたか。"""
        st = self.station
        return (st is None or not st["trips"]) and self.mech.doors_shut(t)

    def _end_trip(self, p):
        st = self.station
        p.pad = None
        if st is None:
            return
        st["trips"].pop(p.id, None)
        st["left"].pop(p.id, None)
        for pad in st["pads"]:
            if pad["pod"] is p:
                pad["pod"], pad["phase"] = None, "free"

    def pad_states(self, n=3):
        """[{"pod", "level", "phase"}] per pad (free ones too) for the dashboard."""
        pads = self.station["pads"] if self.station is not None else []
        out = [{"pod": d["pod"].id if d["pod"] is not None else None,
                "level": round(d["pod"].battery, 1) if d["pod"] is not None else None,
                "phase": d["phase"]} for d in pads]
        return out + [{"pod": None, "level": None, "phase": "free"} for _ in range(n - len(out))]
