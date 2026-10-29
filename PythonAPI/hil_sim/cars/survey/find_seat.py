"""Find a CARLA vehicle's steering wheel in its exported mesh (Blender).

  blender -b -P find_seat.py -- <vehicle.fbx> <out.json>

Splits the mesh into connected pieces and picks the steering wheel: a round
piece 28-60 cm across, thin, off the centre line, in the front half, at dash
height. Parts inside its bounds (hub, logo) join it. Writes its hub, size,
column tilt, the side it is on, its dominant bone, and the vehicle bounds,
in CARLA axes (x forward, y right, z up, metres, mesh origin).

CARLA 車両のメッシュからハンドルを探す（運転席の側・位置・コラム角）。
"""

import json
import math
import sys

import bpy
import numpy as np

src, out = sys.argv[sys.argv.index("--") + 1:][:2]
bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.import_scene.fbx(filepath=src)
meshes = [o for o in bpy.context.scene.objects if o.type == "MESH"]
arm = next((o for o in bpy.context.scene.objects if o.type == "ARMATURE"), None)
bones = [b.name for b in arm.data.bones] if arm else []
res = {"bones": bones}

V, groups_of = [], []
offset = 0
edges_all = []
for o in meshes:
    m = o.matrix_world
    co = np.array([tuple(m @ v.co) for v in o.data.vertices], dtype=np.float64)
    V.append(co)
    e = np.array([tuple(ed.vertices) for ed in o.data.edges], dtype=np.int64) + offset
    edges_all.append(e)
    names = {g.index: g.name for g in o.vertex_groups}
    for v in o.data.vertices:
        if v.groups:
            g = max(v.groups, key=lambda g: g.weight)
            groups_of.append(names.get(g.group, ""))
        else:
            groups_of.append("")
    offset += len(co)
V = np.concatenate(V)
E = np.concatenate(edges_all) if edges_all else np.zeros((0, 2), np.int64)
size = V.max(0) - V.min(0)
if size.max() > 50:                       # centimetres / cm 単位
    V = V / 100.0
V[:, 1] *= -1.0                           # Blender y -> CARLA y / CARLA 座標へ
lo, hi = V.min(0), V.max(0)
res["bounds_lo"], res["bounds_hi"] = lo.round(3).tolist(), hi.round(3).tolist()

# connected components (union-find) / 連結成分
parent = np.arange(len(V))
def find(i):
    r = i
    while parent[r] != r:
        r = parent[r]
    while parent[i] != r:
        parent[i], i = r, parent[i]
    return r
for a, b in E:
    ra, rb = find(a), find(b)
    if ra != rb:
        parent[ra] = rb
roots = np.array([find(i) for i in range(len(V))])
order = np.argsort(roots)
splits = np.flatnonzero(np.diff(roots[order])) + 1
comps = np.split(order, splits)

length = hi[0] - lo[0]
cands = []
for c in comps:
    if len(c) < 24:
        continue
    p = V[c]
    clo, chi = p.min(0), p.max(0)
    d = chi - clo
    ds = np.sort(d)
    centre = (clo + chi) / 2.0
    if not (0.28 <= ds[2] <= 0.62 and ds[1] >= 0.75 * ds[2] and ds[0] <= 0.2):
        continue
    if not (0.15 <= abs(centre[1]) <= 1.1):          # off the centre line / 中心線から外れる
        continue
    if not (lo[0] + 0.25 * length <= centre[0] <= hi[0] - 0.05 * length):   # front part / 前方
        continue
    if not (0.55 <= centre[2] - lo[2] <= 2.6):        # dash height / ダッシュの高さ
        continue
    # ring-ness: points near the rim, few at the centre / 輪らしさ
    q = p - centre
    r = np.sort(np.linalg.norm(q, axis=1))
    ring = float(np.mean(r > 0.3 * ds[2] / 2.0))
    cands.append((ring * len(c) ** 0.25, c, centre, d))
if not cands:
    res["found"] = False
else:
    cands.sort(key=lambda t: -t[0])
    score, c, centre, d = cands[0]
    clo, chi = V[c].min(0) - 0.02, V[c].max(0) + 0.02
    members = [c]
    for o in comps:                                   # hub / logo inside it / 内側の部品
        if o is c or len(o) == 0:
            continue
        p = V[o]
        if np.all(p.min(0) >= clo) and np.all(p.max(0) <= chi):
            members.append(o)
    idx = np.concatenate(members)
    p = V[idx]
    centre = (p.min(0) + p.max(0)) / 2.0
    q = p - p.mean(0)
    w, vec = np.linalg.eigh(q.T @ q)
    axis = vec[:, 0]                                   # smallest spread = column / コラム軸
    if axis[0] < 0:
        axis = -axis
    pitch = math.degrees(math.atan2(axis[2], math.hypot(axis[0], axis[1])))
    yaw = math.degrees(math.atan2(axis[1], axis[0]))
    from collections import Counter
    bone = Counter(groups_of[i] for i in idx).most_common(1)[0][0]
    res.update({"found": True, "hub": centre.round(4).tolist(), "diameter": round(float(max(d)), 3),
                "pitch": round(pitch, 2), "yaw": round(yaw, 2),
                "side": "right" if centre[1] > 0 else "left", "bone": bone,
                "vertex_ids": len(idx), "candidates": len(cands), "score": round(score, 3)})
json.dump(res, open(out, "w"), indent=1)
print("SEAT " + json.dumps({k: v for k, v in res.items() if k != "bones"}))
