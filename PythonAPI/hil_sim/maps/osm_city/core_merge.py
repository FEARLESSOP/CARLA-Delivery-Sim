"""Embedded model (maps.json "core"): cut the generated city away over the
model's rectangle and add the model's own ground as walkable meshes (maps/.venv).

  python core_merge.py <build dir>

meta.json "core" = {"fbx": the model, "bounds": [x0, y0, x1, y1] (or "half":
H for the square +-H), "x_scale": the model's x scale into this frame}: the
model keeps everything inside the rectangle, the generated city everything
outside. The bounds are chosen so the edges cross roads square on and away
from junctions (never along a street).
  - ground pieces (roads, kerbs, markings, pavements, lawns, lots, sites,
    courts) are clipped exactly at the edge (uv and normals interpolated), so
    the two meet on one line;
  - buildings and trees inside are not generated at all (layout.py, props.py
    read meta "core", whole footprints); what else is left (street furniture)
    goes by connected piece: a piece whose middle lies inside is left out;
  - pedestrian heads (jp_hokousha) and Japanese plates (jp_plates) stay
    everywhere: they belong to the road logic, which covers the whole map;
  - nav_areas.json: the generated crossings / aprons / lawns inside are left
    out; the model's zebras (bars in its markings) become the crossings there.
The model's ground (Roads, Ground by material, Road_Markings), cut to its
rectangle, x scaled and lifted to the terrain level there (terrain_field.npz core_z), is
written as <kind>_Core_00.npz beside the generated meshes but not in the
manifest: the navmesh (nav_obj.py) and the surface grid see it, the FBX gets
the model's own meshes (core_fbx.py). The model's layers are read once with
Blender into core_layers.npz.

埋め込みモデルの正方形内の生成物を切り取り、モデル自身の地面を歩行用メッシュとして加える。
"""
import json
import os
import subprocess
import sys
import tempfile

import numpy as np

# manifest "kind" values (build.py / jp_hokousha.py) / マニフェストの kind
SURFACE = ("Road_Road", "Road_Curb", "Road_Marking", "Road_Sidewalk", "Road_Grass", "Terrain_Lots", "sites", "courts")
KEEP = ("jp_hokousha", "jp_plates", "buildings", "jp_shopfront")   # generated kinds the core keeps whole (layout keeps buildings off the model's) / コア内でも残す生成物


def core_rect(meta):
    """(x0, y0, x1, y1) of meta "core", or None. / 埋め込みモデルの範囲"""
    c = meta.get("core")
    if not c:
        return None
    if c.get("bounds"):
        return tuple(float(v) for v in c["bounds"])
    h = float(c["half"])
    return (-h, -h, h, h)


def in_rect(x, y, r):
    return (x > r[0]) & (x < r[2]) & (y > r[1]) & (y < r[3])

DUMP = r'''
import bpy, bmesh, sys, numpy as np
a = sys.argv[sys.argv.index("--") + 1:]
bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.import_scene.fbx(filepath=a[0])
out = {}
for name in ("Roads", "Road_Markings", "Ground"):
    o = bpy.data.objects.get(name)
    if not o:
        continue
    bm = bmesh.new(); bm.from_mesh(o.data); bm.transform(o.matrix_world)
    bmesh.ops.triangulate(bm, faces=bm.faces[:])
    out[name] = np.array([[tuple(v.co) for v in f.verts] for f in bm.faces], np.float32)
    out[name + "_mat"] = np.array([f.material_index for f in bm.faces], np.int16)
    out[name + "_names"] = np.array([s.material.name if s.material else "" for s in o.material_slots])
np.savez_compressed(a[1], **out)
print("[core_merge] dumped", {k: len(v) for k, v in out.items() if not k.endswith(("_mat", "_names"))})
'''


def object_kind(name):
    base = name.rsplit("_", 2)[0] if name.count("_") >= 2 else name
    return "_".join(base.split("_")[:2]) if base.startswith(("Road_", "Terrain_", "SM_")) else base


# -- exact clipping / 正確な切り取り
def _clip_poly(P, a, b):
    """Sutherland-Hodgman: keep a . p[:2] <= b. P: (n, k) rows, x y first."""
    if len(P) == 0:
        return P
    out = []
    d = P[:, :2] @ a - b
    for i in range(len(P)):
        j = (i + 1) % len(P)
        if d[i] <= 0:
            out.append(P[i])
        if (d[i] <= 0) != (d[j] <= 0):
            t = d[i] / (d[i] - d[j])
            out.append(P[i] + t * (P[j] - P[i]))
    return np.array(out) if out else P[:0]


def _regions(r, inside):
    """Convex pieces (half-planes a . p <= b) of the rectangle or of the rest."""
    x0, y0, x1, y1 = r
    X, Y = np.array([1.0, 0.0]), np.array([0.0, 1.0])
    if inside:
        return [[(X, x1), (-X, -x0), (Y, y1), (-Y, -y0)]]
    return [[(X, x0)], [(-X, -x1)], [(-X, -x0), (X, x1), (-Y, -y1)], [(-X, -x0), (X, x1), (Y, y0)]]


def clip_rect(V, I, attrs, r, inside):
    """Triangles (V, I) with per-vertex attrs [(n, k) arrays] cut to the
    rectangle r = (x0, y0, x1, y1) (inside) or to the rest of the plane.
    Returns V, I, attrs."""
    if isinstance(r, (int, float)):
        r = (-r, -r, r, r)
    T = V[I]                                                           # (m, 3, 3)
    lo, hi = T[:, :, :2].min(1), T[:, :, :2].max(1)
    in_sq = (lo[:, 0] >= r[0]) & (lo[:, 1] >= r[1]) & (hi[:, 0] <= r[2]) & (hi[:, 1] <= r[3])
    off_sq = (hi[:, 0] <= r[0]) | (lo[:, 0] >= r[2]) | (hi[:, 1] <= r[1]) | (lo[:, 1] >= r[3])
    whole = in_sq if inside else off_sq
    cut = ~(in_sq | off_sq)
    keepI = I[whole]
    newV, newA, newI = [], [[] for _ in attrs], []
    base = len(V)
    widths = [a.shape[1] for a in attrs]
    for tri in I[cut]:
        P0 = np.concatenate([V[tri]] + [a[tri] for a in attrs], axis=1)
        for reg in _regions(r, inside):
            P = P0
            for a_, b_ in reg:
                P = _clip_poly(P, a_, b_)
                if len(P) < 3:
                    break
            if len(P) < 3:
                continue
            k0 = base + sum(len(v) for v in newV)
            newV.append(P[:, :3])
            o = 3
            for n, w in enumerate(widths):
                newA[n].append(P[:, o:o + w])
                o += w
            for k in range(1, len(P) - 1):
                e1, e2 = P[k, :2] - P[0, :2], P[k + 1, :2] - P[0, :2]
                if abs(e1[0] * e2[1] - e1[1] * e2[0]) > 1e-8 or abs(P[k, 2] - P[0, 2]) + abs(P[k + 1, 2] - P[0, 2]) > 1e-4:
                    newI.append((k0, k0 + k, k0 + k + 1))      # no zero-area slivers / 面積ゼロの三角形は捨てる
    if newV:
        V = np.concatenate([V] + newV).astype(V.dtype)
        attrs = [np.concatenate([a] + na).astype(a.dtype) for a, na in zip(attrs, newA)]
        keepI = np.concatenate([keepI, np.array(newI, I.dtype)])
    return compact(V, keepI, attrs)


def compact(V, I, attrs):
    used = np.unique(I)
    remap = np.full(len(V), -1, np.int64)
    remap[used] = np.arange(len(used))
    return V[used], remap[I].astype(np.int32), [a[used] for a in attrs]


def drop_pieces(V, I, attrs, r):
    """Leave out the connected pieces whose middle lies inside the rectangle."""
    from scipy.sparse import coo_matrix                   # (core_rect is imported by carla-venv scripts too)
    from scipy.sparse.csgraph import connected_components
    if not len(I):
        return V, I, attrs
    n = len(V)
    rows = np.concatenate([I[:, 0], I[:, 1], I[:, 2]])
    cols = np.concatenate([I[:, 1], I[:, 2], I[:, 0]])
    # vertices at the same spot belong together (split normals/uv) / 同位置の頂点は同じ部品
    q = np.round(V / 0.002).astype(np.int64)
    _, inv = np.unique(q, axis=0, return_inverse=True)
    inv = inv.ravel()
    first = np.full(inv.max() + 1, -1, np.int64)
    first[inv[::-1]] = np.arange(n)[::-1]
    rows = np.concatenate([rows, np.arange(n)])
    cols = np.concatenate([cols, first[inv]])
    _, lab = connected_components(coo_matrix((np.ones(len(rows)), (rows, cols)), shape=(n, n)), directed=False)
    k = lab.max() + 1
    cnt = np.bincount(lab, minlength=k)
    cx = np.bincount(lab, V[:, 0], k) / np.maximum(cnt, 1)
    cy = np.bincount(lab, V[:, 1], k) / np.maximum(cnt, 1)
    inside = in_rect(cx, cy, r)
    keep = ~inside[lab[I[:, 0]]]
    return compact(V, I[keep], attrs)


def cut_meshes(bd, r):
    md = os.path.join(bd, "meshes")
    man_p = os.path.join(md, "manifest.json")
    man = json.load(open(man_p))
    objs, stats = [], {}
    for o in man["objects"]:
        kind = o.get("kind") or object_kind(o["name"])
        path = os.path.join(md, o["name"] + ".npz")
        d = np.load(path)
        out, tris = {}, 0
        for mat in sorted({k.split("|")[0] for k in d.files}):
            V, I, UV, N = d[mat + "|V"], d[mat + "|I"], d[mat + "|UV"], d[mat + "|N"]
            if kind not in KEEP:
                if kind in SURFACE:
                    V, I, (UV, N) = clip_rect(V, I, [UV, N], r, inside=False)
                else:
                    V, I, (UV, N) = drop_pieces(V, I, [UV, N], r)
            if len(I):
                out.update({mat + "|V": V, mat + "|I": I, mat + "|UV": UV, mat + "|N": N})
                tris += len(I)
        s = stats.setdefault(kind, [0, 0])
        s[0] += o.get("tris", 0)
        s[1] += tris
        if tris:
            np.savez_compressed(path, **out)
            o["tris"] = tris
            o["materials"] = sorted({k.split("|")[0] for k in out})
            objs.append(o)
        else:
            os.remove(path)
    man["objects"] = objs
    json.dump(man, open(man_p, "w"), indent=0)
    for k, (a, b) in sorted(stats.items()):
        print("[core_merge] %-22s %8d -> %8d triangles" % (k, a, b))


# -- the model's ground as walkable meshes / モデルの地面を歩行用メッシュに
def core_kind(layer, mat):
    m = mat.lower()
    if layer == "Roads":
        return "Road_Road", "asphalt"
    if layer == "Road_Markings":
        return "Road_Marking", "line_yellow" if "yellow" in m else "line_white"
    if "lot" in m:
        return "Terrain_Lots", "lot"
    if "kerb" in m or "curb" in m:
        return "Road_Curb", "kerb"
    if "grass" in m:
        return "Road_Grass", "grass"
    return "Road_Sidewalk", "pave"                  # pavements, tactile strips, plazas / 歩道・広場


def core_ground(bd, core, z):
    cache = os.path.join(bd, "core_layers.npz")
    if not os.path.exists(cache) or os.path.getmtime(cache) < os.path.getmtime(core["fbx"]):
        with tempfile.TemporaryDirectory() as td:
            sp = os.path.join(td, "dump.py")
            open(sp, "w").write(DUMP)
            subprocess.check_call([os.environ.get("BLENDER", "blender"), "-b", "--disable-autoexec", "-P", sp, "--",
                                   core["fbx"], cache], stdout=subprocess.DEVNULL)
    d = np.load(cache)
    r = core_rect({"core": core})
    sx = float(core.get("x_scale", 1.0))
    groups = {}
    for layer in ("Roads", "Road_Markings", "Ground"):
        if layer not in d.files:
            continue
        T, M, names = d[layer], d[layer + "_mat"], d[layer + "_names"]
        for mi in np.unique(M):
            kind, mat = core_kind(layer, str(names[mi]) if mi < len(names) else "")
            t = T[M == mi].reshape(-1, 3).astype(np.float64)
            t[:, 0] *= sx                     # the model's own projection -> this frame / モデルの投影を補正
            I = np.arange(len(t)).reshape(-1, 3)
            V, I, _ = clip_rect(t, I, [], r, inside=True)
            if len(I):
                groups.setdefault(kind, {}).setdefault(mat, []).append((V, I))
    md = os.path.join(bd, "meshes")
    # the drive patch (core_patch.py): the model's ground cut where the lanes leave
    # its asphalt, paved there at its road height, kerb faces along the new edge
    # 新しい車線がアスファルトを外れる所: 地面を切り、路面の高さで舗装、縁石の面
    import core_patch
    meta = {"core": core}
    patch = core_patch.drive_patch(bd, meta)
    merged = {}
    for kind, mats in groups.items():
        for mat, parts in mats.items():
            Vs, Is, off = [], [], 0
            for V, I in parts:
                Vs.append(V); Is.append(I + off); off += len(V)
            V = np.concatenate(Vs).astype(np.float64)
            V[:, 2] += z
            merged[(kind, mat)] = V[np.concatenate(Is)]           # (n, 3, 3) world / 三角形
    if patch is not None and not patch.is_empty:
        ground = np.concatenate([P for (k, m), P in merged.items() if k not in ("Road_Road", "Road_Marking")])
        fill, faces = core_patch.build_fill(bd, meta, patch, ground, z)
        region = core_patch.cut_region(bd, meta, patch)
        region_m = core_patch.cut_region(bd, meta, patch, lanes=False)     # paint: only where the ground goes / 塗装
        # the pavement the cut takes stays walkable as a navmesh apron (walkers cross the
        # new asphalt there, as at an estate junction mouth): no pavement piece split
        # 切った歩道は歩行者エリア（エプロン）として残す（歩道網が分かれない）
        import shapely
        pv = [P for (k, m), P in merged.items() if k == "Road_Sidewalk"]
        aprons = []
        if pv:
            pv = np.concatenate(pv)
            xy = pv[:, :, :2]
            ar = (xy[:, 1, 0] - xy[:, 0, 0]) * (xy[:, 2, 1] - xy[:, 0, 1]) - (xy[:, 2, 0] - xy[:, 0, 0]) * (xy[:, 1, 1] - xy[:, 0, 1])
            tri = shapely.polygons(np.concatenate([xy, xy[:, :1]], 1)[np.abs(ar) > 1e-6])
            parts = np.array(list(shapely.get_parts(region)), dtype=object)
            ti, pi = shapely.STRtree(parts).query(tri, predicate="intersects")
            if len(ti):
                cut_pv = shapely.union_all(shapely.intersection(tri[ti], parts[pi]))
                aprons = [g for g in shapely.get_parts(cut_pv.buffer(0.05).buffer(-0.05)) if g.geom_type == "Polygon" and g.area > 0.05]
        open(os.path.join(bd, "core", "drive_aprons.wkb"), "wb").write(
            shapely.to_wkb(shapely.MultiPolygon(aprons) if aprons else shapely.MultiPolygon()))
        cut = 0
        for key in list(merged):
            if key[0] == "Road_Road":
                continue
            P2, src, _ = core_patch.clip_tris(merged[key], region_m if key[0] == "Road_Marking" else region)
            cut += len(merged[key]) - len(np.unique(src))
            merged[key] = P2
        if len(fill):
            merged[("Road_Road", "asphalt")] = np.concatenate([merged.get(("Road_Road", "asphalt"), np.zeros((0, 3, 3))), fill])
        if len(faces):
            merged[("Road_Curb", "kerb")] = np.concatenate([merged.get(("Road_Curb", "kerb"), np.zeros((0, 3, 3))), faces])
        print("[core_merge] drive patch: %d model ground triangles cut, %d fill + %d kerb-face triangles added"
              % (cut, len(fill), len(faces)))
    for kind in groups:
        out = {}
        for (k, mat), P in merged.items():
            if k != kind or not len(P):
                continue
            V = P.reshape(-1, 3).astype(np.float32)
            out[mat + "|V"] = V
            out[mat + "|I"] = np.arange(len(V), dtype=np.int32).reshape(-1, 3)
            out[mat + "|UV"] = np.zeros((len(V), 2), np.float32)
            out[mat + "|N"] = np.tile(np.float32([0, 0, 1]), (len(V), 1))
        np.savez_compressed(os.path.join(md, "%s_Core_00.npz" % kind), **out)
        print("[core_merge] model ground %-14s %7d triangles (%s)"
              % (kind, sum(len(v) for k, v in out.items() if k.endswith("|I")),
                 ", ".join(sorted(k.split("|")[0] for k in out if k.endswith("|I")))))
    return groups


def zebras(groups):
    """The model's zebra crossings: white bars 0.3-0.6 x 2.5-4.5 m, 3 or more
    side by side (as nav_crosswalks.py does without nav_areas.json).
    / モデルの横断歩道（白い帯の並び）"""
    from shapely.geometry import MultiPoint, Polygon
    from shapely.geometry.polygon import orient
    from shapely.ops import unary_union
    tris = []
    for V, I in groups.get("Road_Marking", {}).get("line_white", []):
        for t in V[I]:
            p = Polygon(t[:, :2])
            if p.area > 1e-4:
                tris.append(p.buffer(0.01))
    if not tris:
        return []
    u = unary_union(tris)
    bars = []
    for p in getattr(u, "geoms", [u]):
        rr = p.minimum_rotated_rectangle
        c = np.array(rr.exterior.coords)[:4]
        a, b = np.linalg.norm(c[1] - c[0]), np.linalg.norm(c[2] - c[1])
        if 0.3 < min(a, b) < 0.6 and 2.5 < max(a, b) < 4.5 and p.area > 0.7 * rr.area:
            bars.append(p)
    g = unary_union([b.buffer(0.35) for b in bars])
    out = []
    for grp in getattr(g, "geoms", [g]):
        inside = [b for b in bars if b.intersects(grp)]
        if len(inside) >= 3:
            out.append(orient(MultiPoint([q for b in inside for q in b.exterior.coords]).convex_hull.buffer(0.2, join_style=2), 1.0))
    return out


def clip_nav(bd, r, groups):
    """nav_areas.json: generated crossings, aprons and lawns inside the
    rectangle out, the model's zebras in. / 歩行者エリアを差し替え"""
    p = os.path.join(bd, "nav_areas.json")
    if not os.path.exists(p):
        return
    from shapely import wkt
    A = json.load(open(p))
    if "_generated" not in A:                 # keep the generated set once, so reruns start from it / 再実行用に保存
        A["_generated"] = {k: A.get(k, []) for k in ("crosswalk", "sidewalk", "lawns")}
    gen = A["_generated"]
    keep = lambda hs: [h for h in hs if not in_rect(*wkt.loads(h).centroid.coords[0], r)]
    z = zebras(groups)
    A["crosswalk"] = keep(gen["crosswalk"]) + [h.wkt for h in z]
    A["sidewalk"] = keep(gen["sidewalk"])
    # the model's pavement the drive patch took (core_ground): walkable aprons / パッチで切った歩道
    ap = os.path.join(bd, "core", "drive_aprons.wkb")
    nap = 0
    if os.path.exists(ap):
        import shapely
        g = shapely.from_wkb(open(ap, "rb").read())
        A["sidewalk"] += [q.wkt for q in shapely.get_parts(g) if not q.is_empty]
        nap = len(shapely.get_parts(g))
    A["lawns"] = keep(gen["lawns"])
    json.dump(A, open(p, "w"))
    if nap:
        print("[core_merge] nav_areas.json: %d aprons where the drive patch took the model's pavement" % nap)
    print("[core_merge] nav_areas.json: %d generated crossings kept outside, %d model zebras inside"
          % (len(A["crosswalk"]) - len(z), len(z)))


def core_level(bd):
    p = os.path.join(bd, "terrain_field.npz")
    if not os.path.exists(p):
        return 0.0
    z = float(np.load(p).get("core_z", np.nan)) if "core_z" in np.load(p).files else float("nan")
    return 0.0 if np.isnan(z) else z


KEEP_DUMP = r'''
import bpy, sys, numpy as np
a = sys.argv[sys.argv.index("--") + 1:]
bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.import_scene.fbx(filepath=a[0])
bpy.context.view_layer.update()
out = {}
for name in a[2:]:
    o = bpy.data.objects.get(name)
    if o is None or o.type != "MESH":
        continue
    mw = o.matrix_world
    V = np.array([(mw @ v.co)[:2] for v in o.data.vertices], np.float32)
    o.data.calc_loop_triangles()
    out[name + "_V"] = V
    out[name + "_T"] = np.array([list(t.vertices) for t in o.data.loop_triangles], np.int32)
np.savez_compressed(a[1], **out)
print("[core_merge] kept model objects dumped:", sorted(out))
'''
HARD_SKIP = ("M_plot", "M_gravel")   # model ground buildings may stand on (plots, building sites) / 建物を置ける面


def core_hard(bd, meta):
    """The model's own streets inside the core (map frame, x scaled): asphalt,
    car parks, kerbs, pavements, plazas, planting; everything but its plots.
    layout.py and the lot terraces keep generated buildings off it, as off the
    road logic (an OSM footprint over the model's pavement stood on it).
    The union of the model's own triangles (a raster outline stair-stepped
    the lifted lot edges along streets at an angle to the grid: agent-checker,
    2026-11-09), gaps < 0.3 m closed. Cached in core/hard.wkb.
    モデルの道路・歩道・広場（敷地以外）: 生成建物を置かない（三角形の和、ラスタでは段々になる）"""
    import shapely
    from shapely.geometry import box
    core = meta.get("core")
    if not core:
        return None
    cache = os.path.join(bd, "core_layers.npz")
    if not os.path.exists(cache):
        return None
    cd = os.path.join(bd, "core")
    os.makedirs(cd, exist_ok=True)
    wkb = os.path.join(cd, "hard.wkb")
    if os.path.exists(wkb) and os.path.getmtime(wkb) > max(os.path.getmtime(cache), os.path.getmtime(__file__)):
        return shapely.from_wkb(open(wkb, "rb").read())
    d = np.load(cache)
    sx = float(core.get("x_scale", 1.0))
    tris = []
    for layer in ("Roads", "Ground"):
        if layer not in d.files:
            continue
        T, M, names = d[layer], d[layer + "_mat"], d[layer + "_names"]
        skip = {i for i, n in enumerate(names) if str(n) in HARD_SKIP}
        keep = ~np.isin(M, list(skip)) if skip else np.ones(len(M), bool)
        P = T[keep][:, :, :2].astype(np.float64)
        P[:, :, 0] *= sx
        tris.append(P)
    if not tris:
        return shapely.Polygon()
    P = np.concatenate(tris)
    a = 0.5 * np.abs((P[:, 1, 0] - P[:, 0, 0]) * (P[:, 2, 1] - P[:, 0, 1])
                     - (P[:, 2, 0] - P[:, 0, 0]) * (P[:, 1, 1] - P[:, 0, 1]))
    P = P[a > 1e-6]
    polys = shapely.polygons(np.concatenate([P, P[:, :1]], axis=1))
    g = shapely.union_all(shapely.make_valid(polys)).buffer(0.15, join_style="mitre").buffer(-0.15, join_style="mitre")
    g = g.simplify(0.05).intersection(box(*core_rect(meta)))
    open(wkb, "wb").write(shapely.to_wkb(g))
    print("[core_merge] model streets (no-build): %.0f m2" % g.area)
    return g


def core_asphalt(bd, meta, patch=True):
    """The model's carriageway asphalt inside the core (map frame, x scaled):
    its Roads layer (M_asphalt) and the markings on it (Road_Markings but
    M_tactile, which lies on pavements), gaps < 0.3 m closed. Wider than the
    road logic's lanes (shoulders, junction corner aprons), so kerb-side poles
    (jp_signs.py plates, jp_hokousha.py heads) keep off it as off the layout
    carriageway. None without a core or core_layers.npz. Cached in
    core/asphalt.wkb. With patch (default) the drive patch (core_patch.py: where
    the lanes leave the model's asphalt, paved in the model) is part of it.
    / モデルの車道アスファルト（路肩・交差点の隅を含む、既定で新しい舗装も）"""
    import shapely
    from shapely.geometry import Polygon, box
    core = meta.get("core")
    cache = os.path.join(bd, "core_layers.npz")
    if not core or not os.path.exists(cache):
        return None
    cd = os.path.join(bd, "core")
    os.makedirs(cd, exist_ok=True)
    wkb = os.path.join(cd, "asphalt.wkb")
    if os.path.exists(wkb) and os.path.getmtime(wkb) > os.path.getmtime(cache):
        g = shapely.from_wkb(open(wkb, "rb").read())
        return _with_patch(bd, meta, g) if patch else g
    d = np.load(cache)
    sx = float(core.get("x_scale", 1.0))
    polys = []
    for layer in ("Roads", "Road_Markings"):
        if layer not in d.files:
            continue
        T, M, names = d[layer], d[layer + "_mat"], d[layer + "_names"]
        for t, mi in zip(T, M):
            if mi < len(names) and str(names[mi]) == "M_tactile":
                continue
            p = Polygon([(float(x) * sx, float(y)) for x, y, _ in t])
            if p.is_valid and p.area > 1e-6:
                polys.append(p)
    g = shapely.union_all(polys).buffer(0.15).buffer(-0.15).intersection(box(*core_rect(meta))) if polys \
        else shapely.Polygon()
    open(wkb, "wb").write(shapely.to_wkb(g))
    print("[core_merge] model asphalt (poles keep off): %.0f m2" % g.area)
    return _with_patch(bd, meta, g) if patch else g


def _with_patch(bd, meta, g):
    import core_patch
    p = core_patch.drive_patch(bd, meta)
    return g if p is None or p.is_empty else g.union(p)


MODEL_KEEP = ("JR_Oita_City", "Railway")    # model objects the core keeps besides its ground / 残すモデルの物体


def core_keep(bd, meta, res=0.5):
    """Footprint (map frame, x scaled) of the model objects the core keeps
    standing (maps.json core "keep", default the JR Oita City station building
    and the railway): every other model building is replaced by generated ones
    (the user, 2026-11-08: "revise all texture and building in old map except
    oita station building"); layout.py keeps generated buildings off this.
    Triangles seen from above on a 0.5 m raster, cached in core/keep.wkb.
    残すモデルの物体（駅ビルと鉄道）の外形。他の建物は生成建物に置き換える。"""
    import shapely
    from PIL import Image, ImageDraw
    from shapely.geometry import box
    core = meta.get("core")
    if not core:
        return None
    names = list(core.get("keep", MODEL_KEEP))
    src = os.path.expanduser(core["fbx"])
    cd = os.path.join(bd, "core")
    os.makedirs(cd, exist_ok=True)
    wkb = os.path.join(cd, "keep_%s.wkb" % "_".join(names))
    if os.path.exists(wkb) and os.path.getmtime(wkb) > os.path.getmtime(src):
        return shapely.from_wkb(open(wkb, "rb").read())
    dump = os.path.join(cd, "keep_dump.npz")
    with tempfile.TemporaryDirectory() as td:
        sp = os.path.join(td, "dump.py")
        open(sp, "w").write(KEEP_DUMP)
        subprocess.check_call([os.environ.get("BLENDER", "blender"), "-b", "--disable-autoexec", "-P", sp, "--",
                               src, dump] + names, stdout=subprocess.DEVNULL)
    d = np.load(dump)
    sx = float(core.get("x_scale", 1.0))
    r = core_rect(meta)
    x0, y0 = r[0] - 30, r[1] - 30
    w, h = int((r[2] - r[0] + 60) / res), int((r[3] - r[1] + 60) / res)
    im = Image.new("L", (w, h), 0)
    dr = ImageDraw.Draw(im)
    for n in names:
        if n + "_V" not in d.files:
            print("[core_merge] keep: no object %s in the model" % n)
            continue
        V = d[n + "_V"].astype(np.float64)
        px, py = (V[:, 0] * sx - x0) / res, (V[:, 1] - y0) / res
        for t in d[n + "_T"]:
            dr.polygon([(px[i], py[i]) for i in t], fill=1)
    M = np.array(im) > 0
    # cells -> polygon: union of row runs / セルの連なりを多角形に
    rows = []
    for i in range(h):
        row = M[i]
        if not row.any():
            continue
        e = np.flatnonzero(np.diff(np.concatenate([[0], row.astype(np.int8), [0]])))
        for j0, j1 in zip(e[::2], e[1::2]):
            rows.append(box(x0 + j0 * res, y0 + i * res, x0 + j1 * res, y0 + (i + 1) * res))
    g = shapely.union_all(rows).buffer(0.25).buffer(-0.25) if rows else shapely.Polygon()
    open(wkb, "wb").write(shapely.to_wkb(g))
    print("[core_merge] keep: %s, %.0f m2" % (", ".join(names), g.area))
    return g


def main():
    bd = sys.argv[1]
    meta = json.load(open(os.path.join(bd, "meta.json")))
    core = meta.get("core")
    if not core:
        print("[core_merge] no core in meta.json: nothing to do")
        return
    core["fbx"] = os.path.expanduser(core["fbx"])
    z = core_level(bd)
    r = core_rect(meta)
    print("[core_merge] core x %.0f..%.0f, y %.0f..%.0f m, model lifted %.2f m, x scale %.6f"
          % (r[0], r[2], r[1], r[3], z, float(core.get("x_scale", 1.0))))
    cut_meshes(bd, r)
    groups = core_ground(bd, core, z)
    clip_nav(bd, r, groups)


if __name__ == "__main__":
    main()
