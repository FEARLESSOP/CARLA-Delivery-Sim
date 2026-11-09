"""The LastMile drone split into named parts, for the Drones page's 3D view.

LastMilePod_Drone.glb (the model the sim flies as static.prop.lmpdrone) has one
mesh per material, so an arm, its motor and its ESC are pieces of three big
meshes. The page needs them apart: to light up, break or tilt one part (a
broken arm droops with its motor and rotor; a rotor loses a blade). This script
splits every mesh into connected pieces, names each piece with the part it
belongs to (the same classification as make_drone_svg.py: body (core) and
shell (the white cover), arm 1..4,
motor 1..4, ESC 1..4, rotor 1..4, battery, winch, hook, gear, lights, lug, FPV
gimbal, lidar, GNSS puck, parachute; PX4 quad-X numbering) and writes a small
GLB with one node per part and material, named ``<part>__<material>``.
Positions stay in the model frame (metres, origin at the bottom of the skids,
+x nose, +y up, +z starboard); materials keep their names and factors (the
page replaces them with its own physical materials).

Run with plain python3 (numpy only) whenever the model changes:

  python3 -I delivery/assets/make_drone_web.py ~/Downloads/unreal/unreal/LastMilePod_Drone.glb \
      web/src/views/drones/three/lmp_drone_parts.glb

LastMile ドローンの glb を部品ごと（アーム・モーター・ESC・ローター等）に
分け、部品名付きのノードで書き出す（ページの 3D 表示で部品ごとに強調・破損表示する）。
"""

import json
import os
import struct
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from make_drone_svg import CT, NC, classify  # noqa: E402


def read(path):
    f = open(path, "rb").read()
    n0 = struct.unpack("<I", f[12:16])[0]
    j = json.loads(f[20:20 + n0])
    o = 20 + n0
    n1 = struct.unpack("<I", f[o:o + 4])[0]
    return j, f[o + 8:o + 8 + n1]


def pieces(j, blob):
    """[(node name, material index, positions, normals, uvs, triangles (k, 3))] per connected piece."""
    def acc(i):
        a = j["accessors"][i]
        bv = j["bufferViews"][a["bufferView"]]
        off = bv.get("byteOffset", 0) + a.get("byteOffset", 0)
        k = NC[a["type"]]
        return np.frombuffer(blob, CT[a["componentType"]], a["count"] * k, off).reshape(-1, k)

    parent = {}
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
        r = m[:3, :3]
        for p in j["meshes"][n["mesh"]]["primitives"]:
            P = acc(p["attributes"]["POSITION"]).astype(float)
            P = (np.c_[P, np.ones(len(P))] @ m.T)[:, :3]
            N = acc(p["attributes"]["NORMAL"]).astype(float) @ r.T
            N /= np.maximum(np.linalg.norm(N, axis=1, keepdims=True), 1e-9)
            uv_i = p["attributes"].get("TEXCOORD_0")
            UV = acc(uv_i).astype(float) if uv_i is not None else np.zeros((len(P), 2))
            I = acc(p["indices"]).ravel() if "indices" in p else np.arange(len(P))
            T = I.reshape(-1, 3)
            # connected pieces: weld by position, union the triangles / 位置で溶接して連結成分
            key = np.round(P / 1e-5).astype(np.int64)
            _, weld = np.unique(key, axis=0, return_inverse=True)
            weld = weld.ravel()
            par = list(range(int(weld.max()) + 1))

            def find(a):
                while par[a] != a:
                    par[a] = par[par[a]]
                    a = par[a]
                return a

            for a, b, c in weld[T]:
                ra = find(a)
                par[find(b)] = ra
                par[find(c)] = ra
            tri_root = np.array([find(weld[t[0]]) for t in T])
            for root in np.unique(tri_root):
                tris = T[tri_root == root]
                used = np.unique(tris)
                remap = -np.ones(len(P), np.int64)
                remap[used] = np.arange(len(used))
                out.append((n["name"], p.get("material", 0), P[used], N[used], UV[used], remap[tris]))
    return out


def write(path, groups, materials):
    """GLB with one node per (part, material). / 部品・材質ごとのノードで GLB を書く。"""
    blob = bytearray()
    views, accs, meshes, nodes = [], [], [], []

    def add(arr, target, comp, typ, minmax=False):
        while len(blob) % 4:
            blob.append(0)
        data = arr.tobytes()
        views.append({"buffer": 0, "byteOffset": len(blob), "byteLength": len(data), "target": target})
        blob.extend(data)
        a = {"bufferView": len(views) - 1, "componentType": comp, "count": len(arr), "type": typ}
        if minmax:
            a["min"] = arr.min(0).tolist()
            a["max"] = arr.max(0).tolist()
        accs.append(a)
        return len(accs) - 1

    for (part, mat), items in sorted(groups.items()):
        P = np.vstack([x[0] for x in items]).astype(np.float32)
        N = np.vstack([x[1] for x in items]).astype(np.float32)
        UV = np.vstack([x[2] for x in items]).astype(np.float32)
        tris, base = [], 0
        for p, _, _, t in items:
            tris.append(t + base)
            base += len(p)
        I = np.vstack(tris).astype(np.uint32).ravel()
        pa = add(P, 34962, 5126, "VEC3", True)
        na = add(N, 34962, 5126, "VEC3")
        ua = add(UV, 34962, 5126, "VEC2")
        ia = add(I, 34963, 5125, "SCALAR")
        meshes.append({"name": "%s__%s" % (part, materials[mat]["name"]),
                       "primitives": [{"attributes": {"POSITION": pa, "NORMAL": na, "TEXCOORD_0": ua},
                                       "indices": ia, "material": mat}]})
        nodes.append({"name": "%s__%s" % (part, materials[mat]["name"]), "mesh": len(meshes) - 1})
    nodes.append({"name": "LMPDrone", "children": list(range(len(nodes)))})
    while len(blob) % 4:
        blob.append(0)
    j = {"asset": {"version": "2.0", "generator": "make_drone_web.py"},
         "scene": 0, "scenes": [{"name": "LMPDrone", "nodes": [len(nodes) - 1]}],
         "nodes": nodes, "meshes": meshes, "materials": materials,
         "accessors": accs, "bufferViews": views, "buffers": [{"byteLength": len(blob)}]}
    js = json.dumps(j, separators=(",", ":")).encode()
    js += b" " * (-len(js) % 4)
    total = 12 + 8 + len(js) + 8 + len(blob)
    with open(path, "wb") as f:
        f.write(struct.pack("<III", 0x46546C67, 2, total))
        f.write(struct.pack("<II", len(js), 0x4E4F534A) + js)
        f.write(struct.pack("<II", len(blob), 0x004E4942) + bytes(blob))


def main(src, dst):
    j, blob = read(src)
    groups = {}
    for node, mat, P, N, UV, T in pieces(j, blob):
        name = j["materials"][mat]["name"]
        part, _tone = classify(node, name, P)
        if part == "body" and name == "M_droneShell":
            part = "shell"          # the white cover: its own part (cracks, X-ray) / 外殻
        groups.setdefault((part, mat), []).append((P, N, UV, T))
    write(dst, groups, j["materials"])
    names = sorted({p for p, _ in groups})
    print("wrote %s: %d nodes, parts: %s (%d bytes)" % (dst, len(groups), " ".join(names),
                                                     os.path.getsize(dst)))


if __name__ == "__main__":
    if len(sys.argv) != 3:
        raise SystemExit(__doc__)
    main(sys.argv[1], sys.argv[2])
