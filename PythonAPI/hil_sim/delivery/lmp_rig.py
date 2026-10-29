"""The LastMile pod's animation, ported from LastMile_Pod_3D.html (no carla import).

The HTML poses every moving part from a cycle time t (timeline.js: drone
delivery, direct.js: direct delivery). This module has the same keyframe
tables and maths, so a part is where the HTML has it at the same t.

  drone_cycle(t)    drone delivery, 12 steps, DRONE_TOTAL s
  direct_cycle(t)   direct delivery to a box station, 8 steps, DIRECT_TOTAL s
  combined(t)       both, one after the other (the dashboard's slider)

Each returns a Mech: the state of every mechanism (jack stroke, lift
height, shuttle axes...). ``part_poses(mech)`` turns it into a pose for every
prop instance in the pod frame -- CARLA axes, metres, origin on the ground
between the axles -- as rest pivot (lmp_parts.json, from the model) plus the
HTML motion's offset from the rest pose the model was exported in.

The HTML is Y-up with +z to the street side; CARLA's (x, y, z) is the
HTML's (x, z, y), so its numbers are used as they are (``H`` below turns an
HTML point into a CARLA one).

LastMile_Pod_3D.html のアニメーションを移植（carla 非依存）。同じキーフレーム表
と計算で、同じ時刻なら HTML と同じ位置に部品が来る。
"""

import json
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
PARTS_PATH = os.path.join(HERE, "lmp_parts.json")

# -- geometry (mech.js P, drone.js L.DRONE, drone.js L.PARCEL) ------------------
FLOOR, ROOF_TOP, DECK_Y = 0.8, 3.2, 1.87
SLOTS = (-1.6, -0.8, 0.0, 0.8, 1.6)
LAUNCH, BUFFER = 1, 2
DRONE_Z, PAD_TOP, LIFT_BASE, PAD_T = -0.3, 1.91, 1.89, 0.04
LIFT_MIN, LIFT_MAX = 0.08, 1.3
AISLE_Z, FAR_Z, TRAY_T = 0.4, -0.3, 0.025
PORT_X, PORT_Y = -1.8, 1.19
LX = SLOTS[LAUNCH]
D_S = 0.88
HOOK_Y, WINCH_Y, LUG_Y = 0.185 * D_S, 0.198 * D_S, 0.406 * D_S
BATT = (0.0, 0.26 * D_S, 0.1 * D_S)
LAUNCH_TOP = LIFT_BASE + LIFT_MIN + PAD_T
BATT_Y = LAUNCH_TOP + BATT[1]
BAIL = 0.034
HOIST_UP = 2.8
PARCEL_M_H, PARCEL_XL_H = 0.10, 0.20       # box heights (bail excluded) / 箱の高さ
MAG_SLOTS = (-0.285, -0.095, 0.095, 0.285)
JACK_STROKE = 0.26
JACK_GROUND = 0.032     # the model's ground is 3.2 cm above the tyre bottom / 接地補正
SLIDE_MAX = 1.45
# box station (direct_env.js STATION), in the pod frame / 箱型ステーション
STN_X, STN_Z0, STN_D, STN_BASE = PORT_X, -1.6, 0.6, 0.12
STN_Z1 = STN_Z0 - STN_D
STN_ZC = (STN_Z0 + STN_Z1) / 2.0
# the compartment used: column 1, row 2 (direct_env.js STN.CELL) -- door C08 in the
# model (its README and the station's screen say C-06) / 使用する区画（モデルでは C08）
STN_COLS, STN_ROWS, STN_CELL = (-0.54, 0.0, 0.54), (0.25, 0.67, 1.09, 1.51), (1, 2)
STATION_DOOR = "BoxStation_Door_C08"
STATION_ORIGIN = (STN_X, 0.0, STN_ZC)     # HTML point of the station model's origin
CELL_CENTER = (STN_X + STN_COLS[STN_CELL[0]], STN_ROWS[STN_CELL[1]] + 0.02, STN_ZC - 0.05)
BUILDING_DOOR = (0.6, STN_BASE, -5.0)
CUSTOMER_FRONT = (PORT_X, STN_BASE, STN_Z1 - 0.55)
HANDS = (0.0, 1.02, 0.36)                 # in the customer's frame / 客の手の位置


def clamp(v, a, b):
    return a if v < a else b if v > b else v


def smooth(t):
    t = clamp(t, 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def lerp(a, b, t):
    return a + (b - a) * t


def keyed(keys, t):
    """kit.js L.keyed: smoothstep between keyframes (t, [values])."""
    if t <= keys[0][0]:
        return list(keys[0][1])
    for (ta, va), (tb, vb) in zip(keys, keys[1:]):
        if t <= tb:
            u = smooth((t - ta) / (tb - ta)) if tb > ta else 1.0
            return [lerp(a, b, u) for a, b in zip(va, vb)]
    return list(keys[-1][1])


def steps_from(table):
    acc, out = 0.0, []
    for sid, title, dur in table:
        out.append({"id": sid, "title": title, "dur": dur, "t0": acc, "t1": acc + dur})
        acc += dur
    return out, acc


# -- step tables (data.js L.STEPS, direct.js L.DIRECT_STEPS) -------------------
DRONE_STEPS, DRONE_TOTAL = steps_from([
    ("drive", "Drive in and stabilize", 6), ("assign", "Order assigned", 3),
    ("retrieve", "Shuttle retrieves the tray", 6), ("present", "Present and weigh", 6),
    ("hook", "Tether pick-up and scan", 6), ("battery", "Battery swap", 7),
    ("lift", "Open roof, raise lift", 6), ("fly", "Drone exits the pod", 7),
    ("return", "Return and precision landing", 8), ("stow", "Lower and close", 7),
    ("switch", "Switch drones", 11), ("depart", "Raise jacks, drive on", 3)])
DIRECT_STEPS, DIRECT_TOTAL = steps_from([
    ("d_drive", "Drive to the building", 7), ("d_assign", "XL parcel assigned", 3),
    ("d_retrieve", "Shuttle retrieves the tray", 6), ("d_load", "Load the side port", 5),
    ("d_transfer", "Slide into the box station", 7), ("d_notify", "Station notifies the customer", 3),
    ("d_collect", "Customer collects", 9), ("d_depart", "Return tray, drive on", 5)])
COMBINED_TOTAL = DRONE_TOTAL + DIRECT_TOTAL
S = {s["id"]: s["t0"] for s in DRONE_STEPS + DIRECT_STEPS}
DUR = {s["id"]: s["dur"] for s in DRONE_STEPS + DIRECT_STEPS}


def at(sid, dt=0.0):
    return S[sid] + dt


def step_at(t):
    """The (scenario, step) the combined timeline is in at ``t``."""
    if t < DRONE_TOTAL:
        return "drone", next((s for s in DRONE_STEPS if t < s["t1"]), DRONE_STEPS[-1])
    t -= DRONE_TOTAL
    return "direct", next((s for s in DIRECT_STEPS if t < s["t1"]), DIRECT_STEPS[-1])


# -- the drone switch (timeline.js buildSwitch, any bay) ------------------------
def slot_top(i, H):
    return pad_top(H) if i == LAUNCH else PAD_TOP


def pad_top(H):
    return LIFT_BASE + H + PAD_T


def build_switch(t0, launch_drone, bay_drone, bay_slot, buffer=BUFFER):
    """Hoist keys and drone moves for a switch starting at ``t0``: the used
    drone (on the lift) to the buffer bay, the charged one from ``bay_slot``
    onto the lift, then the used one into ``bay_slot``.
    Returns (hoist keys [t, [x, y, latch]], moves [(drone, from, to, tA, tB)], end t).
    使用済み機をバッファへ、充電済み機をリフトへ、使用済み機を空いた区画へ。
    """
    park = SLOTS[4]
    tt, cx, latch = [t0], [park], [0]
    keys = [[0.0, [park, HOIST_UP, 0]], [t0, [park, HOIST_UP, 0]]]
    moves = []

    def key(dt, x, y, l):
        tt[0] += dt
        keys.append([tt[0], [x, y, l]])

    def travel(x):
        key(max(0.5, abs(x - cx[0]) * 0.55), x, HOIST_UP, latch[0])
        cx[0] = x

    def move(di, a, b):
        travel(SLOTS[a])
        lug_a, lug_b = slot_top(a, LIFT_MIN) + LUG_Y, slot_top(b, LIFT_MIN) + LUG_Y
        key(0.4, cx[0], lug_a, 0)
        ta = tt[0]
        key(0.15, cx[0], lug_a, 1)
        latch[0] = 1
        key(0.45, cx[0], HOIST_UP, 1)
        travel(SLOTS[b])
        key(0.45, cx[0], lug_b, 1)
        tb = tt[0]
        key(0.15, cx[0], lug_b, 0)
        latch[0] = 0
        key(0.35, cx[0], HOIST_UP, 0)
        moves.append((di, a, b, ta, tb))

    if launch_drone is not None:
        move(launch_drone, LAUNCH, buffer)
    move(bay_drone, bay_slot, LAUNCH)
    if launch_drone is not None:
        move(launch_drone, buffer, bay_slot)
    travel(park)
    return keys, moves, tt[0]


# -- flight curves (stage.js), centripetal Catmull-Rom by arc length -----------
class Curve(object):
    def __init__(self, pts, samples=400):
        self.p = [tuple(map(float, q)) for q in pts]
        segs, self.table = [], [(0.0, self.p[0])]
        n = len(self.p) - 1
        last, total = self.p[0], 0.0
        for k in range(1, samples + 1):
            q = self._raw(k / float(samples) * n)
            total += math.dist(q, last)
            self.table.append((total, q))
            last = q
        self.length = total

    def _raw(self, s):
        """Centripetal Catmull-Rom through the points (three.js CatmullRomCurve3)."""
        p, n = self.p, len(self.p) - 1
        i = min(int(s), n - 1)
        u = s - i
        p1, p2 = p[i], p[i + 1]
        p0 = p[i - 1] if i > 0 else tuple(2 * a - b for a, b in zip(p1, p2))
        p3 = p[i + 2] if i + 2 <= n else tuple(2 * b - a for a, b in zip(p1, p2))

        def dt(a, b):
            return max(math.dist(a, b) ** 0.5, 1e-4)
        d0, d1, d2 = dt(p0, p1), dt(p1, p2), dt(p2, p3)
        out = []
        for a, b, c, d in zip(p0, p1, p2, p3):
            t1 = (b - a) / d0 - (c - a) / (d0 + d1) + (c - b) / d1
            t2 = (c - b) / d1 - (d - b) / (d1 + d2) + (d - c) / d2
            t1, t2 = t1 * d1, t2 * d1
            c0, c1 = b, t1
            c2 = -3 * b + 3 * c - 2 * t1 - t2
            c3 = 2 * b - 2 * c + t1 + t2
            out.append(c0 + c1 * u + c2 * u * u + c3 * u * u * u)
        return tuple(out)

    def at(self, u):
        target = clamp(u, 0.0, 1.0) * self.length
        lo, hi = 0, len(self.table) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if self.table[mid][0] < target:
                lo = mid
            else:
                hi = mid
        (ta, a), (tb, b) = self.table[lo], self.table[hi]
        f = (target - ta) / (tb - ta) if tb > ta else 0.0
        return tuple(lerp(x, y, f) for x, y in zip(a, b))

    def tangent(self, u):
        a, b = self.at(max(0.0, u - 0.002)), self.at(min(1.0, u + 0.002))
        d = [y - x for x, y in zip(a, b)]
        n = math.sqrt(sum(c * c for c in d)) or 1.0
        return tuple(c / n for c in d)


_S0 = (LX, pad_top(LIFT_MAX), DRONE_Z)
FLIGHT_OUT = Curve([_S0, (_S0[0], _S0[1] + 1.2, _S0[2]), (_S0[0], _S0[1] + 3.5, _S0[2]),
                    (_S0[0] - 1.6, _S0[1] + 6, _S0[2] - 1.0),
                    (_S0[0] - 4.5, _S0[1] + 8, _S0[2] - 3), (-11, _S0[1] + 10, -7.5)])
FLIGHT_BACK = Curve([(-11, _S0[1] + 10, -7.5), (_S0[0] - 4.5, _S0[1] + 7.5, _S0[2] - 3),
                     (_S0[0] - 1.2, _S0[1] + 4.5, _S0[2] - 0.6), (_S0[0], _S0[1] + 2.2, _S0[2]),
                     (_S0[0], _S0[1] + 0.6, _S0[2]), _S0])


# -- the mechanism state / 機構の状態 ---------------------------------------------
class Mech(object):
    """Everything a pose needs; HTML coordinates, as the HTML keeps them."""

    def __init__(self):
        self.jacks, self.doors, self.lift = 1.0, 0.0, LIFT_MIN
        self.shuttle = [1.2, 1.0, 0.0]               # sx, fork top y, fork reach z
        self.presenter = 1.19
        self.swap = [0.52, 0.0, 0.095]               # jaw z, drop, magazine dx
        self.hoist = [SLOTS[4], HOIST_UP, 0.0]       # x, gripper y, latch
        self.port = [0.0, 0.0]                       # slide ext, door open
        self.flap, self.cell_door = 0.0, 0.0
        self.tray_m = (0.2, 1.19, FAR_Z)             # drone-parcel tray (bottom centre)
        self.tray_xl = (0.7, 0.86, FAR_Z)            # direct-parcel tray
        self.parcel_m = (0.2, 1.19 + TRAY_T + PARCEL_M_H / 2, FAR_Z)   # centre, or None
        self.parcel_m_on = "pod"                     # pod | drone | gone
        self.parcel_xl = (0.7, 0.86 + TRAY_T + PARCEL_XL_H / 2, FAR_Z)
        self.parcel_xl_on = "pod"                    # pod | person | gone
        self.batt_a = ("drone", None)                # ("drone"|"pod", point)
        self.batt_b = ("pod", (LX + 0.095 + MAG_SLOTS[2], BATT_Y, 0.3))
        # drones D1..D4 (index 0..3): slot index, or a free point (hoist / flight)
        self.drone_slot = [0, LAUNCH, 3, 4]
        self.drone_free = [None, None, None, None]   # (point, yaw, roll) when not on a pad
        self.launch = 1                              # index of the drone on the lift
        self.hook, self.spin = 0.0, 0.0              # launch drone: hook drop, prop spin 0..1
        self.station, self.person = False, None      # box station shown; customer (point, yaw, walk, carry)

    def drone_pose(self, i):
        """(point, yaw, roll) of drone ``i`` (HTML frame), or None (no drone)."""
        if self.drone_free[i] is not None:
            return self.drone_free[i]
        s = self.drone_slot[i]
        if s is None:
            return None
        return (SLOTS[s], slot_top(s, self.lift), DRONE_Z), 0.0, 0.0


REST = Mech()     # the pose the model was exported in (drone cycle at t = 5.5) / 書き出し姿勢


# -- drone delivery (timeline.js) ----------------------------------------------
SX, SY, REST_Y, PRES_UP = 0.2, 1.19, 1.19, 1.6
FORK_IN = FAR_Z - AISLE_Z
HOME, PARK = [1.2, 1.0, 0.0], 0.75
JAW_DRONE, JAW_MAG = DRONE_Z + BATT[2] + 0.09, 0.39


def _drone_keys():
    k = {
        "shuttle": [
            [0, HOME], [at("retrieve", 0), HOME],
            [at("retrieve", 1.6), [SX, SY - 0.012, 0]], [at("retrieve", 3.0), [SX, SY - 0.012, FORK_IN]],
            [at("retrieve", 3.6), [SX, SY + 0.02, FORK_IN]], [at("retrieve", 5.2), [SX, SY + 0.02, 0]],
            [at("present", 0.2), [SX, SY + 0.02, 0]], [at("present", 1.2), [LX, SY + 0.02, 0]],
            [at("present", 2.4), [LX, SY + 0.02, FORK_IN]], [at("present", 3.0), [LX, SY - 0.02, FORK_IN]],
            [at("present", 4.0), [LX, SY - 0.02, 0]], [at("present", 5.4), [PARK, SY - 0.02, 0]],
            [at("stow", 0.0), [PARK, SY - 0.02, 0]], [at("stow", 0.8), [LX, SY - 0.02, 0]],
            [at("stow", 1.4), [LX, SY - 0.02, FORK_IN]], [at("stow", 1.8), [LX, SY + 0.02, FORK_IN]],
            [at("stow", 2.6), [LX, SY + 0.02, 0]], [at("stow", 3.6), [SX, SY + 0.02, 0]],
            [at("stow", 4.5), [SX, SY + 0.02, FORK_IN]], [at("stow", 5.0), [SX, SY - 0.012, FORK_IN]],
            [at("stow", 5.8), [SX, SY - 0.012, 0]], [at("stow", 6.8), HOME]],
        "presenter": [[0, [REST_Y]], [at("present", 4.0), [REST_Y]], [at("present", 5.6), [PRES_UP]],
                      [at("battery", 0.2), [PRES_UP]], [at("battery", 1.6), [REST_Y]]],
        "hook": [[0, [0]], [at("hook", 0.3), [0]], [at("hook", 2.0), [1]], [at("hook", 2.6), [1]],
                 [at("hook", 5.0), [0]]],
        "swap": [
            [0, [0.52, 0, 0.095]], [at("battery", 0.2), [0.52, 0, 0.095]],
            [at("battery", 1.0), [JAW_DRONE, 0, 0.095]], [at("battery", 1.5), [JAW_DRONE, 1, 0.095]],
            [at("battery", 3.0), [JAW_MAG, 1, 0.095]], [at("battery", 3.4), [JAW_MAG, 0, 0.095]],
            [at("battery", 4.4), [JAW_MAG, 0, -0.095]], [at("battery", 4.8), [JAW_MAG, 1, -0.095]],
            [at("battery", 6.3), [JAW_DRONE, 1, -0.095]], [at("battery", 6.6), [JAW_DRONE, 0, -0.095]],
            [at("lift", 0.6), [0.52, 0, -0.095]]],
        "doors": [[0, [0]], [at("lift", 0.3), [0]], [at("lift", 2.3), [1]], [at("stow", 3.6), [1]],
                  [at("stow", 5.2), [0]]],
        "lift": [[0, [LIFT_MIN]], [at("lift", 2.6), [LIFT_MIN]], [at("lift", 5.4), [LIFT_MAX]],
                 [at("stow", 0.5), [LIFT_MAX]], [at("stow", 3.5), [LIFT_MIN]]],
        "spin": [[0, [0]], [at("lift", 4.4), [0]], [at("lift", 5.6), [0.45]], [at("fly", 0.6), [1]],
                 [at("return", 6.8), [1]], [at("return", 7.6), [0]]],
        "jacks": [[0, [0]], [at("drive", 2.8), [0]], [at("drive", 4.0), [1]], [at("depart", 0.0), [1]],
                  [at("depart", 0.9), [0]]],
    }
    return k


DK = _drone_keys()
SWITCH_KEYS, SWITCH_MOVES, _ = build_switch(at("switch"), 1, 2, 3)
HOME_SLOTS = [0, LAUNCH, 3, 4]


def _place(m, moves, hoist, t, base):
    """Drones per the switch moves (timeline.js placeDrone). / 移動に沿って機体を置く。"""
    gx, gy, _ = hoist
    for i in range(4):
        slot, free = base[i], None
        for di, a, b, ta, tb in moves:
            if di != i:
                continue
            if t >= tb:
                slot = b
                continue
            if t >= ta:
                free = ((gx, gy - LUG_Y, DRONE_Z), 0.0, 0.0)
            break
        m.drone_slot[i], m.drone_free[i] = slot, free


def drone_cycle(t, m=None, flight=True, switch=None, slots=None):
    """The HTML drone cycle at ``t`` (0..DRONE_TOTAL).

    ``flight`` False leaves the launch drone on the raised pad while the HTML
    has it flying (the sim flies it for real). ``switch`` = (keys, moves) from
    build_switch overrides the HTML's B3 switch. ``slots`` = deck slot of
    drones D1..D4 (None: no such drone) instead of the HTML's [0, 1, 3, 4].
    ``flight`` が False のとき飛行中の機体はパッド上に残す（シミュレータが飛ばす）。
    ``slots`` は D1..D4 の区画（None は機体なし）。
    """
    m = m or Mech()
    base = list(slots) if slots is not None else list(HOME_SLOTS)
    m.launch = base.index(LAUNCH) if LAUNCH in base else None
    K = lambda name: keyed(DK[name], t)
    m.jacks = K("jacks")[0]
    sx, ty, fz = K("shuttle")
    m.shuttle = [sx, ty, fz]
    m.presenter = K("presenter")[0]
    m.lift = H = K("lift")[0]
    m.doors = K("doors")[0]
    m.swap = K("swap")
    keys, moves = switch or (SWITCH_KEYS, SWITCH_MOVES)
    m.hoist = keyed(keys, t)
    m.spin = K("spin")[0]
    m.port, m.flap, m.cell_door, m.station = [0.0, 0.0], 0.0, 0.0, False
    if t < at("switch"):
        m.drone_slot = list(base)
        m.drone_free = [None] * 4
    else:
        _place(m, moves, m.hoist, t, base)

    # tray: rack -> forks -> presenter -> forks -> rack / トレイの移動
    on_forks = at("retrieve", 3.0) <= t < at("present", 3.0) or at("stow", 1.4) <= t < at("stow", 5.8)
    on_pres = at("present", 3.0) <= t < at("stow", 1.4)
    if on_forks:
        tray = (sx, max(ty, REST_Y), AISLE_Z + fz)
    elif on_pres:
        tray = (LX, m.presenter, FAR_Z)
    else:
        tray = (SX, SY, FAR_Z)
    m.tray_m = tray

    # launch drone: pad, or the HTML's flight out / back / 発進機：パッド上または飛行
    li = m.launch
    pt = pad_top(H)
    mode, u = "pad", 0.0
    if li is None:
        flight = False
    if flight and at("fly", 0.8) <= t < S["return"]:
        mode, u = "out", smooth((t - at("fly", 0.8)) / 5.9)
    elif flight and S["return"] <= t < at("return", 6.8):
        s = clamp((t - S["return"]) / 6.8, 0.0, 1.0)
        mode, u = "back", 1.0 - (1.0 - s) ** 2.2
    if t < at("switch") and li is not None:
        if mode == "pad":
            m.drone_free[li] = None
            m.drone_slot[li] = LAUNCH
        else:
            p = (FLIGHT_OUT if mode == "out" else FLIGHT_BACK).at(u)
            tg = (FLIGHT_OUT if mode == "out" else FLIGHT_BACK).tangent(u)
            hz = math.hypot(tg[0], tg[2])
            yaw = roll = 0.0
            if hz > 0.15:
                yaw = (math.atan2(-tg[2], tg[0]) + (math.pi if mode == "back" else 0.0)) * \
                    clamp((hz - 0.15) * 2, 0.0, 1.0)
                roll = -0.22 * clamp(hz, 0.0, 1.0) * math.sin(math.pi * clamp(u, 0.0, 1.0))
            m.drone_free[li] = (p, yaw, roll)

    # tether hook: only while the drone sits on the lowered pad / 吊り下げフック
    hook_amt = K("hook")[0]
    rest = pt + HOOK_Y
    bail_top = m.presenter + TRAY_T + PARCEL_M_H + BAIL
    hook_y = lerp(rest, bail_top, hook_amt)
    m.hook = rest - hook_y

    # parcel: tray -> hook -> drone cradle -> leaves with the drone
    hang = BAIL + PARCEL_M_H / 2
    if t >= S["return"]:
        m.parcel_m_on, m.parcel_m = "gone", None
    elif t < at("hook", 2.6):
        m.parcel_m_on, m.parcel_m = "pod", (tray[0], tray[1] + TRAY_T + PARCEL_M_H / 2, tray[2])
    elif t < at("hook", 5.0):
        m.parcel_m_on, m.parcel_m = "pod", (LX, hook_y - hang, DRONE_Z)
    else:
        m.parcel_m_on, m.parcel_m = "drone", (0.0, HOOK_Y - hang, 0.0)   # in the drone's frame

    # batteries: depleted A drone -> magazine slot 1, charged B slot 2 -> drone
    fzj, drop, mag_dx = m.swap
    mag = lambda k: (LX + mag_dx + MAG_SLOTS[k], BATT_Y, 0.3)
    if t < at("battery", 1.5):
        m.batt_a = ("drone", None)
    elif t < at("battery", 3.0):
        m.batt_a = ("pod", (LX, BATT_Y, fzj - 0.09))
    else:
        m.batt_a = ("pod", mag(1))
    if t < at("battery", 4.8):
        m.batt_b = ("pod", mag(2))
    elif t < at("battery", 6.3):
        m.batt_b = ("pod", (LX, BATT_Y, fzj - 0.09))
    else:
        m.batt_b = ("drone", None)
    m.tray_xl = (0.7, 0.86, FAR_Z)
    m.parcel_xl_on, m.parcel_xl = "pod", (0.7, 0.86 + TRAY_T + PARCEL_XL_H / 2, FAR_Z)
    m.person = None
    return m


# -- direct delivery (direct.js) -----------------------------------------------
DSX, DSY = 0.7, 0.86
DPARK = -0.6


def _direct_keys():
    a = at
    return {
        "shuttle": [
            [0, HOME], [a("d_retrieve", 0), HOME],
            [a("d_retrieve", 1.6), [DSX, DSY - 0.012, 0]], [a("d_retrieve", 3.0), [DSX, DSY - 0.012, FORK_IN]],
            [a("d_retrieve", 3.6), [DSX, DSY + 0.02, FORK_IN]], [a("d_retrieve", 5.2), [DSX, DSY + 0.02, 0]],
            [a("d_load", 1.4), [PORT_X, PORT_Y + 0.02, 0]], [a("d_load", 2.4), [PORT_X, PORT_Y + 0.02, FORK_IN]],
            [a("d_load", 3.0), [PORT_X, PORT_Y - 0.02, FORK_IN]], [a("d_load", 3.9), [PORT_X, PORT_Y - 0.02, 0]],
            [a("d_load", 4.9), [DPARK, PORT_Y - 0.02, 0]],
            [a("d_depart", 0.0), [DPARK, PORT_Y - 0.02, 0]], [a("d_depart", 0.7), [PORT_X, PORT_Y - 0.02, 0]],
            [a("d_depart", 1.2), [PORT_X, PORT_Y - 0.02, FORK_IN]], [a("d_depart", 1.5), [PORT_X, PORT_Y + 0.02, FORK_IN]],
            [a("d_depart", 2.1), [PORT_X, PORT_Y + 0.02, 0]], [a("d_depart", 3.1), [DSX, DSY + 0.02, 0]],
            [a("d_depart", 3.6), [DSX, DSY + 0.02, FORK_IN]], [a("d_depart", 3.9), [DSX, DSY - 0.012, FORK_IN]],
            [a("d_depart", 4.4), [DSX, DSY - 0.012, 0]], [a("d_depart", 5.0), HOME]],
        "slide": [[0, [0]], [a("d_transfer", 1.0), [0]], [a("d_transfer", 3.6), [1]],
                  [a("d_transfer", 4.8), [1]], [a("d_transfer", 6.6), [0]]],
        "door": [[0, [0]], [a("d_transfer", 0.0), [0]], [a("d_transfer", 0.8), [1]],
                 [a("d_transfer", 6.6), [1]], [a("d_transfer", 7.0), [0]]],
        "flap": [[0, [0]], [a("d_transfer", 0.4), [0]], [a("d_transfer", 1.1), [1]],
                 [a("d_transfer", 4.8), [1]], [a("d_transfer", 5.6), [0]]],
        "cell_door": [[0, [0]], [a("d_collect", 4.2), [0]], [a("d_collect", 4.8), [1]],
                      [a("d_collect", 5.8), [1]], [a("d_collect", 6.4), [0]]],
        "carry": [[0, [0]], [a("d_collect", 4.8), [0]], [a("d_collect", 5.6), [1]]],
        "jacks": [[0, [0]], [a("d_drive", 4.9), [0]], [a("d_drive", 6.1), [1]], [a("d_depart", 3.0), [1]],
                  [a("d_depart", 3.8), [0]]],
    }


XK = _direct_keys()


def port_carrier_z(ext):
    return FAR_Z - smooth(ext) * SLIDE_MAX


def direct_cycle(t, m=None, customer=True, slots=None):
    """The HTML direct delivery at ``t`` (0..DIRECT_TOTAL). / 直接配達。"""
    m = drone_cycle(at("assign", 1.0), m, flight=False, slots=slots)   # the rest at rest
    K = lambda name: keyed(XK[name], t + S["d_drive"])
    T = t + S["d_drive"]
    a = at
    m.jacks = K("jacks")[0]
    sx, ty, fz = K("shuttle")
    m.shuttle = [sx, ty, fz]
    ext = K("slide")[0]
    m.port = [ext, K("door")[0]]
    m.flap, m.cell_door = K("flap")[0], K("cell_door")[0]
    m.station = True
    on_forks = a("d_retrieve", 3.0) <= T < a("d_load", 3.0) or a("d_depart", 1.2) <= T < a("d_depart", 3.9)
    on_slide = a("d_load", 3.0) <= T < a("d_depart", 1.2)
    rest = PORT_Y if a("d_load") <= T < a("d_depart", 3.0) else DSY
    if on_forks:
        tray = (sx, max(ty, rest), AISLE_Z + fz)
    elif on_slide:
        tray = (PORT_X, PORT_Y, port_carrier_z(ext))
    else:
        tray = (DSX, DSY, FAR_Z)
    m.tray_xl = tray
    on_tray = (tray[0], tray[1] + TRAY_T + PARCEL_XL_H / 2, tray[2])
    cell = CELL_CENTER
    in_cell = (cell[0], cell[1] + PARCEL_XL_H / 2, cell[2])

    # customer: walks out of the building, collects, walks back in / 客の動き
    c0 = a("d_collect")
    person = None
    if customer and c0 <= T < a("d_collect", 8.9):
        door, front = BUILDING_DOOR, CUSTOMER_FRONT
        if T < a("d_collect", 3.6):
            u = smooth((T - c0) / 3.6)
            p = tuple(lerp(x, y, u) for x, y in zip(door, front))
            yaw, walk = math.atan2(front[0] - door[0], front[2] - door[2]), 0.02 < u < 0.98
        elif T < a("d_collect", 6.0):
            p, yaw, walk = front, 0.0, False
        else:
            u = smooth((T - a("d_collect", 6.0)) / 2.8)
            p = tuple(lerp(x, y, u) for x, y in zip(front, door))
            yaw, walk = math.atan2(door[0] - front[0], door[2] - front[2]), 0.02 < u < 0.98
        person = (p, yaw, walk, K("carry")[0])
    m.person = person

    # parcel: tray -> belt -> compartment -> customer's hands / 荷物の移動
    if T < a("d_transfer", 3.7):
        m.parcel_xl_on, m.parcel_xl = "pod", on_tray
    elif T < a("d_transfer", 4.6):
        u = smooth((T - a("d_transfer", 3.7)) / 0.9)
        m.parcel_xl_on = "pod"
        m.parcel_xl = (PORT_X, lerp(PORT_Y + TRAY_T + PARCEL_XL_H / 2, in_cell[1], u),
                       lerp(port_carrier_z(1.0), cell[2], u))
    elif T < a("d_collect", 4.8):
        m.parcel_xl_on, m.parcel_xl = "pod", in_cell
    elif T < a("d_collect", 5.6) and person is not None:
        u = smooth((T - a("d_collect", 4.8)) / 0.8)
        hw = person_point(person, HANDS)
        m.parcel_xl_on = "pod"
        m.parcel_xl = tuple(lerp(x, y, u) for x, y in zip(in_cell, hw))
    elif T < a("d_collect", 8.9) and person is not None:
        m.parcel_xl_on, m.parcel_xl = "person", HANDS
    else:
        m.parcel_xl_on, m.parcel_xl = "gone", None
    return m


def person_point(person, local):
    """A point in the customer's frame -> HTML frame (three.js rotation about y)."""
    (px, py, pz), yaw = person[0], person[1]
    x, y, z = local
    return (px + x * math.cos(yaw) + z * math.sin(yaw), py + y,
            pz - x * math.sin(yaw) + z * math.cos(yaw))


def combined(t):
    """Drone delivery, then direct delivery: the dashboard slider's timeline."""
    t = clamp(t, 0.0, COMBINED_TOTAL - 1e-3)
    if t < DRONE_TOTAL:
        return drone_cycle(t)
    return direct_cycle(t - DRONE_TOTAL)


# -- poses / 姿勢 -------------------------------------------------------------------
def H(p):
    """HTML point -> CARLA pod frame. / HTML の点を CARLA の座標へ。"""
    return (p[0], p[2], p[1])


def _rot3(rx=0.0, ry=0.0, rz=0.0):
    """three.js Euler XYZ -> rotation matrix (HTML axes)."""
    cx, sx, cy, sy, cz, sz = (math.cos(rx), math.sin(rx), math.cos(ry), math.sin(ry),
                              math.cos(rz), math.sin(rz))
    return [[cy * cz, -cy * sz, sy],
            [sx * sy * cz + cx * sz, -sx * sy * sz + cx * cz, -sx * cy],
            [-cx * sy * cz + sx * sz, cx * sy * sz + sx * cz, cx * cy]]


def rotator(rx=0.0, ry=0.0, rz=0.0):
    """An HTML (three.js Euler XYZ) rotation as CARLA (pitch, yaw, roll) degrees.

    The HTML -> CARLA axis swap is a reflection, so the matrix is conjugated
    by it, then read back the way Unreal's FMatrix::Rotator does.
    軸の入れ替え（鏡映）で行列を変換し、Unreal と同じ手順で回転角に戻す。
    """
    m = _rot3(rx, ry, rz)
    p = [0, 2, 1]
    c = [[m[p[i]][p[j]] for j in range(3)] for i in range(3)]
    fwd = (c[0][0], c[1][0], c[2][0])
    right = (c[0][1], c[1][1], c[2][1])
    up = (c[0][2], c[1][2], c[2][2])
    pitch = math.atan2(fwd[2], math.hypot(fwd[0], fwd[1]))
    yaw = math.atan2(fwd[1], fwd[0])
    sy = (-math.sin(yaw), math.cos(yaw), 0.0)
    roll = math.atan2(sum(a * b for a, b in zip(up, sy)), sum(a * b for a, b in zip(right, sy)))
    return (math.degrees(pitch), math.degrees(yaw), math.degrees(roll))


def _rotate_about(point, pivot, rx=0.0, ry=0.0, rz=0.0):
    m = _rot3(rx, ry, rz)
    d = [point[i] - pivot[i] for i in range(3)]
    return tuple(pivot[i] + sum(m[i][j] * d[j] for j in range(3)) for i in range(3))


_PARTS = None


def parts():
    """lmp_parts.json: prop -> {"instances": {node: pivot (CARLA frame)}, ...}."""
    global _PARTS
    if _PARTS is None:
        with open(PARTS_PATH) as f:
            _PARTS = json.load(f)
    return _PARTS


def pivot(prop, node=None):
    inst = parts()[prop]["instances"]
    p = inst[node] if node else next(iter(inst.values()))
    return (p[0], p[2], p[1])               # -> HTML frame / HTML 座標へ


def _scissor(H_lift):
    hs = H_lift / 3.0
    return hs, math.asin(clamp(hs / 0.76, 0.0, 1.0))


def part_poses(m):
    """{(prop, node): (x, y, z, pitch, yaw, roll)} in the CARLA pod frame for
    every pod part (drones, station and customer are separate: see
    drone_poses / station_poses). / ポッド部品ごとの CARLA 姿勢。"""
    r, out = REST, {}

    def put(prop, node, delta=(0.0, 0.0, 0.0), rot=(0.0, 0.0, 0.0), at_point=None):
        p = at_point if at_point is not None else tuple(a + b for a, b in zip(pivot(prop, node), delta))
        out[(prop, node)] = H(p) + rotator(*rot)

    s_now, s_rest = smooth(m.jacks), smooth(r.jacks)
    jack_y = lambda s: lerp(JACK_STROKE, 0.0, s) - JACK_GROUND * s
    for node in ("Jack_FrontRight", "Jack_FrontLeft", "Jack_RearRight", "Jack_RearLeft"):
        put("LMPJack", node, (0.0, jack_y(s_now) - jack_y(s_rest), 0.0))
    d = 0.64 * (smooth(m.doors) - smooth(r.doors))
    put("LMPHatchFront", None, (d, 0.0, 0.0))
    put("LMPHatchRear", None, (-d, 0.0, 0.0))

    put("LMPLiftPad", None, (0.0, m.lift - r.lift, 0.0))
    hs, th = _scissor(m.lift)
    hs0, th0 = _scissor(r.lift)
    for k in range(6):
        i = k // 2
        dy = (hs - hs0) * (i + 0.5)
        put("LMPScissorA", "Lift_ScissorA_%d" % k, (0.0, dy, 0.0), (0.0, 0.0, th - th0))
        put("LMPScissorB", "Lift_ScissorB_%d" % k, (0.0, dy, 0.0), (0.0, 0.0, -(th - th0)))
        put("LMPLiftPin", "Lift_Pin_%d" % k, (0.0, dy, 0.0))

    (sx, ty, fz), (sx0, ty0, fz0) = m.shuttle, r.shuttle
    put("LMPShuttleMast", None, (sx - sx0, 0.0, 0.0))
    put("LMPShuttleCarriage", None, (sx - sx0, ty - ty0, 0.0))
    put("LMPShuttleFork1", None, (sx - sx0, ty - ty0, (fz - fz0) / 2.0))
    put("LMPShuttleFork2", None, (sx - sx0, ty - ty0, fz - fz0))
    put("LMPPresenterTable", None, (0.0, m.presenter - r.presenter, 0.0))

    (fj, drop, mdx), (fj0, drop0, mdx0) = m.swap, r.swap
    cz, cz0 = clamp(fj, 0.2, 0.52), clamp(fj0, 0.2, 0.52)
    put("LMPSwapperCarrier", None, (0.0, 0.0, cz - cz0))
    put("LMPSwapperBoom", None, (0.0, 0.0, (cz + fj) / 2.0 - (cz0 + fj0) / 2.0))
    put("LMPSwapperJaw", None, (0.0, lerp(0.1, -0.03, drop) - lerp(0.1, -0.03, drop0), fj - fj0))
    put("LMPBatteryMagazine", None, (mdx - mdx0, 0.0, 0.0))

    (gx, gy, _), (gx0, gy0, _) = m.hoist, r.hoist
    put("LMPHoistBridge", None, (gx - gx0, 0.0, 0.0))
    put("LMPHoistGripper", None, (gx - gx0, gy - gy0, 0.0))

    put("LMPTray", "Tray_F-0.2-1.19", tuple(a - b for a, b in zip(m.tray_m, r.tray_m)))
    put("LMPTray", "Tray_F-0.7-0.86", tuple(a - b for a, b in zip(m.tray_xl, r.tray_xl)))
    if m.parcel_m_on == "pod":
        put("LMPParcelM", None, tuple(a - b for a, b in zip(m.parcel_m, r.parcel_m)))
    if m.parcel_xl_on == "pod":
        put("LMPParcelXL", None, tuple(a - b for a, b in zip(m.parcel_xl, r.parcel_xl)))

    e, e0 = smooth(m.port[0]) * SLIDE_MAX, smooth(r.port[0]) * SLIDE_MAX
    put("LMPPortDoor", None, (0.0, 0.5 * (smooth(m.port[1]) - smooth(r.port[1])), 0.0))
    for k, prop in enumerate(("LMPPortStage1", "LMPPortStage2", "LMPPortStage3", "LMPPortCarrier")):
        put(prop, None, (0.0, 0.0, -(e - e0) * k / 3.0))
    # batteries: offsets from where they rest (A in the launch drone, B in slot 2)
    # 電池：静止位置（A は発進機内、B はスロット 2）からの移動量
    rest_pt = {"Battery_A": (LX, LAUNCH_TOP + BATT[1], DRONE_Z + BATT[2]), "Battery_B": REST.batt_b[1]}
    for node, (where, p) in (("Battery_A", m.batt_a), ("Battery_B", m.batt_b)):
        if where == "pod":
            put("LMPBattery", node, tuple(a - b for a, b in zip(p, rest_pt[node])))
    return out


def drone_poses(m):
    """[(x, y, z, pitch, yaw, roll) of drone i's origin, CARLA pod frame] for D1..D4."""
    out = []
    for i in range(4):
        dp = m.drone_pose(i)
        out.append(None if dp is None else H(dp[0]) + rotator(0.0, dp[1], dp[2]))
    return out


PROP_HUBS = None


def prop_poses(spin_deg):
    """The 4 rotors in the drone frame, turned to ``spin_deg`` (diagonal pairs opposite)."""
    hubs = parts()["LMPDroneProp"]["instances"]
    dirs = (1, -1, -1, 1)
    return [tuple(hubs["Drone_D1_Prop%d" % (k + 1)]) + (0.0, d * spin_deg % 360.0, 0.0)
            for k, d in enumerate(dirs)]


def hook_pose(drop):
    hx, hy, hz = parts()["LMPDroneHook"]["instances"]["Drone_D1_Hook"]
    return (hx, hy, hz - drop, 0.0, 0.0, 0.0)


def station_poses(m):
    """Box station, flap and compartment door, in the CARLA pod frame."""
    so = H(STATION_ORIGIN)
    flap = parts()["LMPStationFlap"]["instances"]["BoxStation_IntakeFlap"]
    door = parts()["LMPStationDoor"]["instances"][STATION_DOOR]
    return {"LMPStation": so + (0.0, 0.0, 0.0),
            "LMPStationFlap": (so[0] + flap[0], so[1] + flap[1], so[2] + flap[2]) +
            rotator(-1.3 * smooth(m.flap), 0.0, 0.0),
            "LMPStationDoor": (so[0] + door[0], so[1] + door[1], so[2] + door[2]) +
            rotator(0.0, 1.6 * smooth(m.cell_door), 0.0)}


def person_pose(m):
    """(x, y, z, yaw deg, walking, carrying) of the customer, CARLA pod frame, or None."""
    if m.person is None:
        return None
    p, yaw, walk, carry = m.person
    return H(p) + (rotator(0.0, yaw, 0.0)[1], walk, carry)


# -- tracks: each subsystem on its own timeline / 機構ごとの独立タイムライン ----------
# The sim runs the pod's subsystems independently (loading while driving, the
# crane while the roof is open...). Each track is a window of the HTML's own
# timeline for that subsystem; compose() merges the tracks into one Mech.
# シミュレーションでは機構ごとに独立して動かす（走行中の積み込みなど）。各トラックは
# HTML のタイムラインの区間で、compose() が 1 つの Mech にまとめる。
WINDOWS = {
    # cargo (shuttle, tray, presenter, tether): parcel into the drone on the lift
    "load": ("m", at("retrieve"), at("battery", 1.6)),
    "tray_back": ("m", at("stow", 0.0), at("stow", 6.8)),
    # battery swapper / 電池交換
    "swap": ("batt", at("battery", 0.2), at("lift", 0.6)),
    # roof doors + lift (+ spin of the drone on it) / 屋根扉とリフト
    "open": ("roof", at("lift", 0.0), at("fly", 0.8)),
    "lower": ("roof", at("return", 6.8), at("stow", 3.5)),
    "raise": ("roof", at("lift", 2.6), at("fly", 0.8)),
    "close": ("roof", at("stow", 3.5), at("stow", 5.2)),
    # side port + box station + customer (direct delivery timeline) / 直接配達
    "direct_order": ("x", at("d_retrieve"), at("d_transfer", 7.0)),
    "direct_collect": ("x", at("d_notify"), at("d_depart", 5.0)),
}
HOOK_DONE = at("hook", 5.0)          # the parcel is in the drone's cradle / 荷物が機体へ
SWAP_DONE = at("battery", 3.0)       # the old pack is out / 古い電池を抜いた瞬間
HANDED = at("d_transfer", 4.6)       # the belt has the parcel / ベルトが受け取った


def build_moves(t0, moves):
    """Crane keys for ``moves`` [(drone, from slot, to slot)] from ``t0``
    (the lift is down for every move). Returns (keys, moves with times, end).
    クレーンの移動列（リフトは下げた状態）。"""
    park = SLOTS[4]
    tt, cx, latch = [t0], [park], [0]
    keys = [[0.0, [park, HOIST_UP, 0]], [t0, [park, HOIST_UP, 0]]]
    timed = []

    def key(dt, x, y, l):
        tt[0] += dt
        keys.append([tt[0], [x, y, l]])

    def travel(x):
        key(max(0.5, abs(x - cx[0]) * 0.55), x, HOIST_UP, latch[0])
        cx[0] = x

    for di, a, b in moves:
        travel(SLOTS[a])
        lug_a, lug_b = slot_top(a, LIFT_MIN) + LUG_Y, slot_top(b, LIFT_MIN) + LUG_Y
        key(0.4, cx[0], lug_a, 0)
        ta = tt[0]
        key(0.15, cx[0], lug_a, 1)
        latch[0] = 1
        key(0.45, cx[0], HOIST_UP, 1)
        travel(SLOTS[b])
        key(0.45, cx[0], lug_b, 1)
        tb = tt[0]
        key(0.15, cx[0], lug_b, 0)
        latch[0] = 0
        key(0.35, cx[0], HOIST_UP, 0)
        timed.append((di, a, b, ta, tb))
    travel(park)
    return keys, timed, tt[0]


class PodAnim(object):
    """The pod's tracks: each runs a WINDOWS entry; step() advances them all.

    Track times are HTML times (drone cycle; the "x" track direct cycle), so a
    resting track sits at a time where its subsystem rests.
    ポッドの各トラック。step() で同時に進める。
    """

    REST = {"m": 0.0, "batt": 0.0, "roof": 0.0, "x": 0.0}

    def __init__(self):
        self.t = dict(self.REST)
        self.end = {}                       # track -> end time while it plays
        self.name = {}                      # track -> window playing / 再生中の区間
        self.crane = None                   # [keys, moves, t, end]
        self.shuttle = "m"                  # which cargo track drives the shuttle
        self.x_on = False                   # station/side port in use / 直接配達中

    def play(self, name):
        track, t0, t1 = WINDOWS[name]
        self.t[track], self.end[track], self.name[track] = t0, t1, name
        if track in ("m", "x"):
            self.shuttle = track
        if track == "x":
            self.x_on = True

    def play_crane(self, moves):
        keys, timed, end = build_moves(0.0, moves)
        self.crane = [keys, timed, 0.0, end]
        return timed

    def busy(self, track):
        return track in self.end or (track == "crane" and self.crane is not None)

    def playing(self, track):
        return self.name.get(track) if track in self.end else None

    def step(self, dt):
        """Advance; returns [(track, window, t_before, t_after, finished)]."""
        out = []
        for track in list(self.end):
            a = self.t[track]
            b = min(a + dt, self.end[track])
            self.t[track] = b
            done = b >= self.end[track]
            out.append((track, self.name[track], a, b, done))
            if done:
                del self.end[track]
                self.name.pop(track, None)
        if self.crane is not None:
            c = self.crane
            c[2] = min(c[2] + dt, c[3])
            if c[2] >= c[3]:
                out.append(("crane", "move", 0.0, c[2], True))
                self.crane = None
        return out

    def lift_down(self):
        return not self.busy("roof") and self.name.get("roof") is None and \
            keyed(DK["lift"], self.t["roof"])[0] <= LIFT_MIN + 1e-6

    def roof_closed(self):
        return not self.busy("roof") and keyed(DK["doors"], self.t["roof"])[0] <= 1e-6

    def lift_up(self):
        return not self.busy("roof") and keyed(DK["lift"], self.t["roof"])[0] >= LIFT_MAX - 1e-6

    def mech(self, slots, jacks):
        return compose(self, slots, jacks)


def compose(anim, slots, jacks):
    """One Mech from the pod's tracks. / トラックを 1 つの Mech にまとめる。"""
    base = list(slots)
    m = Mech()
    m.jacks = jacks
    m.launch = base.index(LAUNCH) if LAUNCH in base else None
    roof = drone_cycle(anim.t["roof"], flight=False, slots=base)
    m.doors, m.lift, m.spin = roof.doors, roof.lift, roof.spin
    batt = drone_cycle(anim.t["batt"], flight=False, slots=base)
    m.swap, m.batt_a, m.batt_b = batt.swap, batt.batt_a, batt.batt_b
    cargo = drone_cycle(anim.t["m"], flight=False, slots=base)
    m.presenter, m.tray_m, m.hook = cargo.presenter, cargo.tray_m, cargo.hook
    m.parcel_m, m.parcel_m_on = cargo.parcel_m, cargo.parcel_m_on
    m.shuttle = cargo.shuttle
    if anim.x_on:
        x = direct_cycle(anim.t["x"] - S["d_drive"], slots=base)
        m.tray_xl, m.parcel_xl, m.parcel_xl_on = x.tray_xl, x.parcel_xl, x.parcel_xl_on
        m.port, m.flap, m.cell_door, m.station, m.person = x.port, x.flap, x.cell_door, x.station, x.person
        if anim.shuttle == "x":
            m.shuttle = x.shuttle
    # drones on their slots at the lift's height, or on the crane / 区画またはクレーン上
    m.drone_slot, m.drone_free = base, [None] * 4
    if anim.crane is not None:
        keys, timed, t, _ = anim.crane
        m.hoist = keyed(keys, t)
        gx, gy, _ = m.hoist
        for di, a, b, ta, tb in timed:
            if t >= tb:
                m.drone_slot[di] = b
            elif t >= ta:
                m.drone_slot[di] = a
                m.drone_free[di] = ((gx, gy - LUG_Y, DRONE_Z), 0.0, 0.0)
                break
            else:
                break
    return m
