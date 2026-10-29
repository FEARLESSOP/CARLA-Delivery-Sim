"""Moving parts of the pod and the drone: pivots and poses (no carla import).

The numbers are the part pivots measured by delivery/assets/prepare_models.py,
in each parent's frame (CARLA axes: x forward, y right, z up, metres):

  pod frame    origin on the ground midway between the axles
  drone frame  origin at the bottom of the drone's legs

Poses are (x, y, z, pitch, yaw, roll) relative to the parent, which is how an
attached CARLA actor is moved (ApplyTransform on an attached actor sets its
offset from the parent).

ポッドとドローンの可動部品（支点と姿勢、carla 非依存）。値は
prepare_models.py が測った各部品の支点で、親の座標系（x 前、y 右、z 上 [m]）。
姿勢は親からの相対 (x, y, z, pitch, yaw, roll)。
"""

import math

# -- pod / ポッド ---------------------------------------------------------------
# jack foot pads (x, y); fully down the pad is on the ground
# ジャッキの接地パッド位置。下げ切ると地面に着く
JACKS = {"front_right": (1.970, 0.726), "front_left": (1.970, -0.714),
         "rear_right": (-1.970, 0.726), "rear_left": (-1.970, -0.714)}
# retracted, the pad is this far up: inside the jack's housing in the chassis,
# which starts 0.332 m up / 格納時のパッド高さ（車台のジャッキ収納は 0.332 m から）
JACK_STROKE = 0.35
# roof hatch: two doors hinged on their outer edges, opening like a clamshell
# 屋根ハッチ：外側の辺に蝶番がある 2 枚扉（観音開き）
HATCH_FRONT = (-0.200, -0.294, 3.260)   # front door hinge (its front edge)
HATCH_REAR = (-1.400, -0.294, 3.260)    # rear door hinge (its rear edge)
HATCH_OPEN_DEG = 105.0                  # past vertical, leaning away from the hole
HATCH_XY = (-0.800, -0.294)             # middle of the opening / 開口部の中心
ROOF_Z = 3.30                           # top of the opening / 開口部の上端
# where the drone parks inside the pod: under the hatch, clear of the hidden
# car's roof (~1.5 m)
# 機体の格納位置：ハッチの真下、隠れた車の屋根（約 1.5 m）より上
DOCK_Z = 1.70

# side port (PodV2): a sliding door on the pod's left side and a telescoping
# arm behind it -- three stages and a carrier tray, each 0.5 m, that slide out
# through the door (-y) to hand a parcel over. Pivots: bottom centre of each.
# サイドポート（PodV2）：左側面のスライドドアと、その奥の伸縮アーム（3 段＋
# トレイ、各 0.5 m）。ドアから -y 方向へ伸びて荷物を渡す。
SIDE_DOOR = (-1.800, -1.039, 1.082)
SIDE_DOOR_OUT = 0.025           # steps out of the skin first / まず外側へ
SIDE_DOOR_SLIDE = 0.40          # then slides forward (+x), clear of the opening
SLIDE_PARTS = {"stage1": (-1.800, -0.294, 1.185), "stage2": (-1.800, -0.294, 1.191),
               "stage3": (-1.800, -0.294, 1.197), "carrier": (-1.800, -0.294, 1.203)}
SLIDE_STEP = 0.45               # each part reaches this much further than the last
CARRIER_TOP = 0.038             # tray thickness: the parcel sits on top / トレイ厚

# -- drone / ドローン -----------------------------------------------------------
ROTORS = ((0.238, 0.238, 0.298), (0.238, -0.238, 0.298),
          (-0.238, 0.238, 0.298), (-0.238, -0.238, 0.298))
ROTOR_DIR = (1, -1, -1, 1)      # diagonal pairs turn opposite ways / 対角で逆回転
ROTOR_STEP_DEG = 53.0           # per frame: reads as spinning, never as standing still
PARCEL_BELOW = 0.75             # parcel bottom under the drone origin (on a tether)
PARCEL_HEIGHT = 0.24


def smooth(u):
    """Ease in/out on 0..1 (clamped). / 0..1 のイーズイン・アウト。"""
    u = min(max(u, 0.0), 1.0)
    return u * u * (3.0 - 2.0 * u)


def jack_poses(u):
    """Jacks at ``u`` (0 = retracted, 1 = down on the ground), in the pod frame."""
    z = JACK_STROKE * (1.0 - smooth(u))
    return {k: (x, y, z, 0.0, 0.0, 0.0) for k, (x, y) in JACKS.items()}


def hatch_poses(u):
    """Hatch doors at ``u`` (0 = closed, 1 = open), in the pod frame.

    The front door's panel reaches back (-x) from its hinge, so it opens with
    negative pitch; the rear door's reaches forward and opens with positive.
    前扉は蝶番から後方へ伸びるので負の pitch、後扉は正の pitch で開く。
    """
    a = HATCH_OPEN_DEG * smooth(u)
    return {"front": HATCH_FRONT + (-a, 0.0, 0.0), "rear": HATCH_REAR + (a, 0.0, 0.0)}


def side_door_pose(u):
    """Side door at ``u`` (0 = shut, 1 = open): out of the skin, then forward."""
    out = SIDE_DOOR_OUT * smooth(u / 0.2)
    slide = SIDE_DOOR_SLIDE * smooth((u - 0.2) / 0.8)
    x, y, z = SIDE_DOOR
    return (x + slide, y - out, z, 0.0, 0.0, 0.0)


def slide_poses(u):
    """The arm at ``u`` (0 = stowed, 1 = out): the k-th part reaches k steps.

    Fully out the tray is 1.8 m left of where it is stowed, about 0.8 m
    beyond the pod's side. / 伸び切るとトレイは格納位置から 1.8 m 左。
    """
    out = {}
    for k, name in enumerate(("stage1", "stage2", "stage3", "carrier"), start=1):
        x, y, z = SLIDE_PARTS[name]
        out[name] = (x, y - SLIDE_STEP * k * smooth(u), z, 0.0, 0.0, 0.0)
    return out


def rotor_poses(spin):
    """The four rotors turned to ``spin`` degrees, in the drone frame."""
    return [(x, y, z, 0.0, d * spin % 360.0, 0.0) for (x, y, z), d in zip(ROTORS, ROTOR_DIR)]


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def to_parent(origin, fwd, right, up, pos, yaw):
    """A level, ``yaw``-headed body at world ``pos`` as a pose in a parent frame.

    ``origin``/``fwd``/``right``/``up`` are the parent's world location and axes
    (unit vectors). The rotation is rebuilt the way Unreal's FMatrix::Rotator
    does, so the body stays level even when the parent (a van parked on a
    hill) is tilted.
    ワールドの水平な機体を親座標系の姿勢に変換する。親（坂に停めた車）が
    傾いていても機体が水平を保つよう、Unreal の FMatrix::Rotator と同じ手順で
    回転を求める。
    """
    v = (pos[0] - origin[0], pos[1] - origin[1], pos[2] - origin[2])
    loc = (_dot(v, fwd), _dot(v, right), _dot(v, up))
    r = math.radians(yaw)
    axes = [(math.cos(r), math.sin(r), 0.0), (-math.sin(r), math.cos(r), 0.0), (0.0, 0.0, 1.0)]
    x, y, z = [(_dot(a, fwd), _dot(a, right), _dot(a, up)) for a in axes]
    pitch = math.atan2(x[2], math.hypot(x[0], x[1]))
    yaw_l = math.atan2(x[1], x[0])
    sy = (-math.sin(yaw_l), math.cos(yaw_l), 0.0)   # right axis of (pitch, yaw, 0)
    roll = math.atan2(_dot(z, sy), _dot(y, sy))
    return loc + (math.degrees(pitch), math.degrees(yaw_l), math.degrees(roll))
