"""Line drawing of the LastMile drone (LMP drone) for the dashboard's Drones page.

Reads the real model (LastMilePod_Drone.glb, the same drone the sim flies as
static.prop.lmpdrone: quad-X, origin at the bottom of the skids, +x nose, y
up, +z starboard, metres) and projects every mesh part into a top view and a
starboard side view. Each part (connected piece of a mesh) is drawn as the
convex hull of its projected vertices: every primitive of the model (box,
cylinder, sphere, rod; see the HTML's drone.js) is convex, so the hull is its
exact silhouette. Parts are grouped by what they are (body, arm 1..4, motor
1..4, ESC 1..4, rotor 1..4, battery, winch, hook, gear, lights, lug) or by the
sensor they carry (FPV gimbal camera, downward lidar, GNSS puck, parachute).
The few sensors the model has no geometry for (flight controller IMU + baro,
optical flow, delivery camera, front / rear stereo pairs, datalink antennas,
Remote ID) are added as small mounts at plausible places on the airframe.

Motor numbers follow PX4's quad-X: 1 front-right, 2 rear-left, 3 front-left,
4 rear-right.

Output: web/src/views/drones/droneShape.ts (paths in millimetres). Run with
plain python3 (numpy only) whenever the model changes:

  python3 -I delivery/assets/make_drone_svg.py ~/Downloads/unreal/unreal/LastMilePod_Drone.glb \
      web/src/views/drones/droneShape.ts

LastMile ドローンの実モデル（glb）から上面図と右側面図を作る。各メッシュ部品を
投影して凸包で描き、部品（機体・アーム・モーター・ローター・電池・ウインチ等）
またはセンサーごとに分ける。モデルに形状の無いセンサーは妥当な位置に追加する。
"""

import json
import os
import struct
import sys

import numpy as np

S = 0.88                    # drone.js body scale / 機体の縮尺
PROP_R = 0.17 * S           # rotor radius [m]
MOTORS = {1: (1, 1), 2: (-1, -1), 3: (1, -1), 4: (-1, 1)}   # PX4 quad-X: (sign x, sign z)

# sensors without geometry in the model: part -> list of (kind, centre xyz, size xyz)
# kind "box" or "cyl" (vertical cylinder: size = (diameter, height, diameter))
# モデルに無いセンサーの取付位置（中心・寸法 [m]）
EXTRA = {
    "imu": [("box", (0.03, 0.229, 0.0), (0.05, 0.02, 0.04))],          # flight controller / FC 内
    "baro": [("box", (-0.035, 0.229, -0.02), (0.022, 0.016, 0.022))],
    "cam_down": [("cyl", (0.128, 0.183, 0.042), (0.024, 0.014, 0.024))],
    "flow": [("cyl", (0.128, 0.183, -0.042), (0.022, 0.012, 0.022))],
    "cam_front": [("box", (0.172, 0.282, 0.0), (0.018, 0.016, 0.09)),
                  ("cyl", (0.182, 0.282, 0.032), (0.012, 0.012, 0.012)),
                  ("cyl", (0.182, 0.282, -0.032), (0.012, 0.012, 0.012))],
    "cam_rear": [("box", (-0.172, 0.282, 0.0), (0.018, 0.016, 0.09)),
                 ("cyl", (-0.182, 0.282, 0.032), (0.012, 0.012, 0.012)),
                 ("cyl", (-0.182, 0.282, -0.032), (0.012, 0.012, 0.012))],
    "link": [("box", (-0.205, 0.215, 0.205), (0.006, 0.07, 0.006)),
             ("box", (-0.205, 0.215, -0.205), (0.006, 0.07, 0.006))],
    "remote_id": [("box", (0.0, 0.31, 0.062), (0.034, 0.01, 0.022))],
}
INTERNAL = {"imu", "baro"}  # inside the fuselage / 機体内部
# per view: parts hidden behind / under the airframe, drawn last as dashed
# outlines; and mounts on the shell surface drawn over the body
# ビューごとに隠れる部品（最後に破線で描く）と、外殻上の取付部品（機体の上に描く）
HIDDEN = {"top": {"rangefinder", "cam_down", "flow", "winch", "hook", "imu", "baro"},
          "side": {"imu", "baro"}}
OVER = {"top": ["cam_front", "cam_rear", "remote_id"], "side": ["cam_front", "cam_rear"]}

CT = {5126: np.float32, 5125: np.uint32, 5123: np.uint16, 5121: np.uint8}
NC = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4}


def load_glb(path):
    """[(node name, material, vertices (n, 3) in model metres)], one per connected piece."""
    f = open(path, "rb").read()
    n0 = struct.unpack("<I", f[12:16])[0]
    j = json.loads(f[20:20 + n0])
    o = 20 + n0
    n1 = struct.unpack("<I", f[o:o + 4])[0]
    blob = f[o + 8:o + 8 + n1]

    def acc(i):
        a = j["accessors"][i]
        bv = j["bufferViews"][a["bufferView"]]
        off = bv.get("byteOffset", 0) + a.get("byteOffset", 0)
        k = NC[a["type"]]
        return np.frombuffer(blob, CT[a["componentType"]], a["count"] * k, off).reshape(-1, k)

    parent, mats = {}, {}
    for i, n in enumerate(j["nodes"]):
        for c in n.get("children", []):
            parent[c] = i

    def world(i):
        m = np.eye(4)
        while i is not None:
            n = j["nodes"][i]
            if "matrix" in n:
                m = np.array(n["matrix"], float).reshape(4, 4).T @ m
            elif any(k in n for k in ("translation", "rotation", "scale")):
                raise SystemExit("TRS nodes not handled (%s)" % n.get("name"))
            i = parent.get(i)
        return m

    out = []
    for i, n in enumerate(j["nodes"]):
        if "mesh" not in n:
            continue
        m = world(i)
        for p in j["meshes"][n["mesh"]]["primitives"]:
            P = acc(p["attributes"]["POSITION"]).astype(float)
            P = (np.c_[P, np.ones(len(P))] @ m.T)[:, :3]
            I = acc(p["indices"]).ravel() if "indices" in p else np.arange(len(P))
            mat = j["materials"][p.get("material", 0)]["name"]
            for V in split(P, I):
                out.append((n["name"], mat, V))
    return out


def split(P, I):
    """Connected pieces (vertices welded by position, so flat-shaded faces join)."""
    key = np.round(P / 1e-5).astype(np.int64)
    _, weld = np.unique(key, axis=0, return_inverse=True)
    weld = weld.ravel()
    par = list(range(int(weld.max()) + 1))

    def find(a):
        while par[a] != a:
            par[a] = par[par[a]]
            a = par[a]
        return a

    for a, b, c in weld[I].reshape(-1, 3):
        ra = find(a)
        par[find(b)] = ra
        par[find(c)] = ra
    roots = np.array([find(w) for w in weld])
    return [P[roots == r] for r in np.unique(roots)]


def quadrant(c):
    sx, sz = (1 if c[0] > 0 else -1), (1 if c[2] > 0 else -1)
    return next(k for k, v in MOTORS.items() if v == (sx, sz))


def classify(node, mat, V):
    """(part id, tone) of one piece. Tone picks the fill on the page."""
    c = (V.min(0) + V.max(0)) / 2.0
    size = V.max(0) - V.min(0)
    if node.startswith("Drone_D1_Prop"):
        return "rotor%d" % quadrant(c), "prop"
    if node.startswith("Drone_D1_Hook"):
        return "hook", "safety" if mat == "M_safety" else "metal"
    if mat in ("M_droneBody", "M_droneShell"):
        return "body", "shell" if mat == "M_droneShell" else "core"
    if mat == "M_hv" or (mat == "M_alu" and abs(c[2] - 0.166) < 0.01) or \
            (mat == "M_ledG" and size.max() < 0.015):
        return "battery", "hv" if mat == "M_hv" else ("led" if mat == "M_ledG" else "metal")
    if mat in ("M_ledR", "M_ledG"):
        return "lights", "led"
    if mat == "M_ledB":
        return ("cam_fpv", "lens") if c[0] > 0.15 else ("lights", "led")
    if mat == "M_white":
        return "gnss", "white"
    if mat == "M_safety":
        return "parachute", "safety"
    if mat == "M_hazard" or (mat == "M_steel" and abs(c[0]) < 0.01 and c[1] > 0.3):
        return "lug", "hazard" if mat == "M_hazard" else "metal"
    if mat == "M_steel":
        return "motor%d" % quadrant(c), "metal"
    if mat == "M_aluDark":
        if abs(c[0]) < 0.01 and abs(c[2]) < 0.01:
            return "winch", "metal"
        return "motor%d" % quadrant(c), "metal"
    if mat == "M_alu":
        return "esc%d" % quadrant(c), "metal"
    if mat == "M_black":
        if c[1] < 0.03 or (size[1] > 0.1 and c[1] < 0.15):
            return "gear", "dark"
        if abs(c[1] - 0.251) < 0.01 and size[0] > 0.2:
            return "arm%d" % quadrant(c), "dark"
        if abs(c[2] - 0.088) < 0.01 and size[0] > 0.1:
            return "battery", "dark"
        if c[0] > 0.15:
            return "cam_fpv", "dark"
        if abs(c[0] - 0.097) < 0.01:
            return "rangefinder", "dark"
        if c[0] < -0.05 and c[1] > 0.3:
            return "parachute", "dark"
    raise SystemExit("unclassified piece %s %s at %s" % (node, mat, np.round(c, 3)))


def prim(kind, c, s, n=20):
    """Vertices of an added box / vertical cylinder. / 追加形状の頂点。"""
    c, s = np.array(c), np.array(s) / 2.0
    if kind == "box":
        return np.array([[c[0] + a * s[0], c[1] + b * s[1], c[2] + d * s[2]]
                         for a in (-1, 1) for b in (-1, 1) for d in (-1, 1)])
    a = np.linspace(0, 2 * np.pi, n, endpoint=False)
    ring = np.c_[np.cos(a) * s[0], np.zeros(n), np.sin(a) * s[2]]
    return np.r_[ring + c + [0, -s[1], 0], ring + c + [0, s[1], 0]]


def hull(pts):
    """Convex hull, counter-clockwise (Andrew's monotone chain)."""
    pts = sorted(set(map(tuple, np.round(pts, 5))))
    if len(pts) < 3:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lo, up = [], []
    for p in pts:
        while len(lo) >= 2 and cross(lo[-2], lo[-1], p) <= 0:
            lo.pop()
        lo.append(p)
    for p in reversed(pts):
        while len(up) >= 2 and cross(up[-2], up[-1], p) <= 0:
            up.pop()
        up.append(p)
    return lo[:-1] + up[:-1]


def path(poly):
    return "M" + "L".join("%g %g" % (round(x * 1000, 1), round(y * 1000, 1)) for x, y in poly) + "Z"


# views: model point -> screen (x right, y down), and paint order (drawn first = lowest)
VIEWS = {
    # from above, nose up: screen x = starboard, y = -nose / 上から、機首が上
    "top": (lambda P: np.c_[P[:, 2], -P[:, 0]], lambda P: P[:, 1].max()),
    # from starboard, nose right: screen x = nose, y = -up / 右舷側から、機首が右
    "side": (lambda P: np.c_[P[:, 0], -P[:, 1]], lambda P: P[:, 2].mean()),
}


def main(glb, out):
    pieces = [classify(n, m, V) + (V,) for n, m, V in load_glb(glb)]
    for part, specs in EXTRA.items():
        for kind, c, s in specs:
            pieces.append((part, "internal" if part in INTERNAL else "sensor", prim(kind, c, s)))
    # rotor discs (the blur of a spinning rotor) / 回転円盤
    for k, (sx, sz) in MOTORS.items():
        a = np.linspace(0, 2 * np.pi, 48, endpoint=False)
        c = np.array([sx * 0.2376, 0.3036, sz * 0.2376])
        disc = np.c_[np.cos(a) * PROP_R, np.zeros_like(a), np.sin(a) * PROP_R] + c
        pieces.append(("rotor%d" % k, "disc", disc))
    allv = np.vstack([p[2] for p in pieces])
    views = {}
    for name, (proj, depth) in VIEWS.items():
        q = proj(allv)
        pad = 0.03
        x0, y0 = q.min(0) - pad
        w, h = q.max(0) - q.min(0) + 2 * pad
        parts = {}
        for part, tone, V in pieces:
            parts.setdefault(part, []).append((depth(V), tone, path(hull(proj(V)))))
        order = sorted(parts, key=lambda p: (min(d for d, _, _ in parts[p]) if name == "side"
                                             else max(d for d, _, _ in parts[p])))
        if name == "side":       # rotor discs on top of everything / 回転円盤は最前面
            order.sort(key=lambda p: p.startswith("rotor"))
        order = ([p for p in order if p not in HIDDEN[name] and p not in OVER[name]] +
                 OVER[name] + [p for p in order if p in HIDDEN[name]])
        views[name] = {
            "viewBox": "%g %g %g %g" % (round(x0 * 1000, 1), round(y0 * 1000, 1),
                                        round(w * 1000, 1), round(h * 1000, 1)),
            "parts": [dict({"id": p, "shapes": [{"tone": t, "d": d}
                                                for _, t, d in sorted(parts[p], key=lambda s: s[0])]},
                           **({"hidden": True} if p in HIDDEN[name] else {}))
                      for p in order]}
    span = 2 * (0.2376 * np.sqrt(2) + PROP_R)
    with open(out, "w") as f:
        f.write("// GENERATED by delivery/assets/make_drone_svg.py from LastMilePod_Drone.glb\n"
                "// (the LMP drone the sim flies). Do not edit; re-run the script.\n"
                "// 生成ファイル（実モデルの投影）。編集せずスクリプトを再実行する。\n"
                "/* eslint-disable */\n"
                "export type ShapeTone = 'shell' | 'core' | 'dark' | 'metal' | 'prop' | 'disc' | 'led'"
                " | 'lens' | 'white' | 'safety' | 'hazard' | 'hv' | 'sensor' | 'internal'\n"
                "export interface DroneShape { tone: ShapeTone; d: string }\n"
                "/** hidden: behind / under the airframe in this view (draw dashed) */\n"
                "export interface DronePart { id: string; shapes: DroneShape[]; hidden?: boolean }\n"
                "export interface DroneView { viewBox: string; parts: DronePart[] }\n"
                "/** rotor-tip span of the model [m] / ローター端の全幅 */\n"
                "export const DRONE_SPAN_M = %.3f\n" % span)
        for name, v in views.items():
            f.write("export const DRONE_%s: DroneView = %s\n" % (name.upper(), json.dumps(v, separators=(",", ":"))))
    n = sum(len(p["shapes"]) for v in views.values() for p in v["parts"])
    print("wrote %s: %d parts, %d shapes, span %.3f m, %d bytes" % (
        out, len(views["top"]["parts"]), n, span, os.path.getsize(out)))


if __name__ == "__main__":
    if len(sys.argv) != 3:
        raise SystemExit(__doc__)
    main(sys.argv[1], sys.argv[2])
