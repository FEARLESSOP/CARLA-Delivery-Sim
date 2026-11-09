"""Drive patch of an embedded model (meta "core", guide ch. 35): where the road
logic's lanes (+ PATCH_MARGIN) leave the model's asphalt (its corner kerbs,
pavements, islands, lots cut by the new junction lanes), that ground is cut
away and paved at the model's road height, with a kerb face along the new
edge up to the ground beside it. So the drivable surface = the road logic
everywhere in the core (check_drive: no kerb in a wheel track).

One source of truth, used by:
  - core_merge.core_ground: the model's ground meshes <kind>_Core_00.npz
    (check_drive, navmesh, surface grid) are cut and filled here;
  - core_materials.apply (core_fbx JOIN, Blender): the same cut and fill on
    the model's FBX ground objects;
  - core_merge.core_asphalt: the patch counts as asphalt, so poles, plates,
    pedestrian heads and speed posts keep off it.
drive_patch() -> core/drive_patch.wkb; build_fill() -> core/drive_patch.npz
(fill and kerb-face triangles, map frame, world z). The model's station
and other kept objects are never cut (only ground objects are).

埋め込みモデル: 新しい車線（+余裕）がモデルのアスファルトを外れる所の地面を切り、
モデルの路面の高さで舗装し、新しい縁に縁石の面を付ける。
"""
import hashlib
import json
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PATCH_MARGIN = 0.25     # beyond the lane edge (check_drive tests 0.2 m inside it) / 車線端の外側の余裕
NEED_IN = 0.15          # lane interior that must lie on asphalt (check_drive: 0.2) / 路面であるべき車線内部
TRACK_W = 0.3           # each wheel track's half width in the need / 轍の半幅
GROW = 1.0              # the cut grows this far round each need (to the lane edge + margin) / 切る範囲の広がり
MIN_PIECE = 0.05        # smaller patch pieces are dropped (m2) / これ未満の片は捨てる
CUT_EDGE = 0.06         # the model's ground within this of the patch along its asphalt goes too (m) / 縁の切り幅
HOLE_FILL = 40.0        # islands inside the patch smaller than this are paved over (m2) / 小島は舗装
FACE_MIN = 0.02         # kerb faces only where the ground beside differs this much (m) / 段差の最小


def _code():
    return hashlib.sha1(open(os.path.abspath(__file__), "rb").read()).hexdigest()[:12]


def lane_polys(lanes, rect):
    """Driving lane polygons (left edge + right edge back) touching the core. / コアに掛かる車線"""
    import shapely
    from shapely.geometry import Polygon, box
    sq = box(*rect)
    out = []
    for l in lanes:
        if l.get("kind") != "driving" or len(l["pts"]) < 2:
            continue
        ring = [tuple(q["l"]) for q in l["pts"]] + [tuple(q["r"]) for q in l["pts"]][::-1]
        p = Polygon(ring)
        if not p.is_valid:
            p = p.buffer(0)                  # as check_drive.py repairs a lane ring / check_drive と同じ修復
        if p.intersects(sq):
            out.append((l, p))
    return out


def drive_patch(bd, meta, log=print):
    """The patch polygon (map frame) or None (no core, no lanes.json, no model layers).
    Cached in core/drive_patch.wkb on lanes.json, the model layers and this code.
    / パッチ多角形（キャッシュ付き）"""
    import shapely
    from shapely.geometry import box
    import core_merge
    core = meta.get("core")
    lj = os.path.join(bd, "lanes.json")
    if not core or not os.path.exists(lj) or not os.path.exists(os.path.join(bd, "core_layers.npz")):
        return None
    cd = os.path.join(bd, "core")
    os.makedirs(cd, exist_ok=True)
    wkb, stamp = os.path.join(cd, "drive_patch.wkb"), os.path.join(cd, "drive_patch.json")
    want = {"lanes": os.path.getmtime(lj), "layers": os.path.getmtime(os.path.join(bd, "core_layers.npz")),
            "code": _code(), "margin": PATCH_MARGIN}
    have = json.load(open(stamp)) if os.path.exists(stamp) else None
    if have and {k: have.get(k) for k in want} == want and os.path.exists(wkb):
        return shapely.from_wkb(open(wkb, "rb").read())
    rect = core_merge.core_rect(meta)
    sq = box(*rect)
    asp = core_merge.core_asphalt(bd, meta, patch=False)
    if asp is not None:
        # markings count only on the model's roads (parking-bay lines on its lots
        # left uncut lot strips in a turning head) / 敷地上の駐車枠線は路面に数えない
        asp = asp.intersection(_roads(bd, meta).buffer(0.3))
    L = json.load(open(lj))
    L = L["lanes"] if isinstance(L, dict) else L
    lp = lane_polys(L, rect)
    if not lp or asp is None:
        return None
    U = shapely.union_all([p for _, p in lp])
    # only where the lane interior check_drive tests (0.2 m in from the edge; 0.15 m
    # here) leaves the asphalt; there the cut reaches PATCH_MARGIN past the lane edge
    # (lanes fitted 0.1 m inside the model's streets keep its kerbs whole)
    # 車線内部（端から 0.15 m 内）がアスファルトを外れる所だけ、そこから車線端 + 余裕まで切る
    # and the wheel tracks check_drive samples (0.35 m in from each edge: at a sharp
    # bend the inner track's cusp leaves the repaired lane ring) / 轍（鋭い曲がりで車線外に出る）
    tracks = []
    for l, _ in lp:
        C = np.array([q["c"] for q in l["pts"]]); Lp = np.array([q["l"] for q in l["pts"]])
        R = np.array([q["r"] for q in l["pts"]]); w = np.array([q["w"] for q in l["pts"]]).clip(0.8)
        f = (0.35 / (w / 2.0))[:, None]
        for E in (Lp + (C - Lp) * f, R + (C - R) * f):
            if len(E) >= 2:
                tracks.append(shapely.LineString(E).buffer(TRACK_W, cap_style="flat"))
    need = shapely.union_all([U.buffer(-NEED_IN, join_style="mitre")] + tracks).difference(asp).intersection(sq)
    need = shapely.union_all([p for p in shapely.get_parts(need) if p.area >= 0.01]) if not need.is_empty else need
    lanes = shapely.union_all([U] + tracks).buffer(PATCH_MARGIN, join_style="mitre", mitre_limit=2.0)
    g = need.buffer(GROW, join_style="round").intersection(lanes).difference(asp).intersection(sq)
    # a turning loop's island (< HOLE_FILL m2) is paved too: a round head, no 2 m kerbed
    # island / 転回ループの中の小島も舗装（丸い転回部）
    # (holes of patch + asphalt, so an island the loop shares with the road counts too)
    if not g.is_empty:
        near = asp.intersection(g.buffer(12.0))
        isl = []
        for p in shapely.get_parts(g.union(near)):
            if p.geom_type != "Polygon":
                continue
            for r in p.interiors:
                h = shapely.Polygon(r)
                if h.area < HOLE_FILL and h.buffer(0.05).intersects(g):
                    isl.append(h)
        if isl:
            g = g.union(shapely.union_all(isl)).difference(asp).intersection(sq)
    # hairlines along the model's asphalt edge closed / アスファルト縁の細片を除く
    g = g.buffer(-0.04, join_style="mitre").buffer(0.04, join_style="mitre").intersection(sq)
    parts = [p for p in shapely.get_parts(g) if p.area >= MIN_PIECE]
    g = shapely.union_all(parts) if parts else shapely.Polygon()
    g = shapely.set_precision(g, 0.001)
    open(wkb, "wb").write(shapely.to_wkb(g))
    # the lanes' tested interior: the model's raised ground goes there too (asphalt under it)
    # 車線内部（ここでは盛り上がった地面も除く。下はアスファルト）
    inner = [g for _, p in lp for g in shapely.get_parts(p.buffer(-NEED_IN).intersection(sq))
             if g.geom_type == "Polygon" and g.area > 1e-4]
    inner += [g for t in tracks for g in shapely.get_parts(t.intersection(sq)) if g.geom_type == "Polygon" and g.area > 1e-4]
    open(os.path.join(cd, "drive_lanes.wkb"), "wb").write(shapely.to_wkb(shapely.GeometryCollection(inner)))
    big = sorted(((p.area, [round(c, 1) for c in p.centroid.coords[0]]) for p in parts), reverse=True)[:12]
    json.dump(dict(want, area_m2=round(g.area, 1), pieces=len(parts), largest=big), open(stamp, "w"), indent=1)
    log("[core_patch] drive patch: %.0f m2 in %d pieces where the lanes (+%.2f m) leave the model's asphalt; "
        "largest %s" % (g.area, len(parts), PATCH_MARGIN, ", ".join("%.0f m2 at (%.0f, %.0f)" % (a, *c) for a, c in big[:5])))
    return g


def _roads(bd, meta):
    """The model's Roads layer (x scaled, gaps < 0.3 m closed), cached in core/roads.wkb.
    / モデルの車道（Roads）のみ"""
    import shapely
    import core_merge
    cache = os.path.join(bd, "core_layers.npz")
    wkb = os.path.join(bd, "core", "roads.wkb")
    if os.path.exists(wkb) and os.path.getmtime(wkb) > os.path.getmtime(cache):
        return shapely.from_wkb(open(wkb, "rb").read())
    d = np.load(cache)
    P = d["Roads"][:, :, :2].astype(np.float64)
    P[:, :, 0] *= float(meta["core"].get("x_scale", 1.0))
    a = 0.5 * np.abs((P[:, 1, 0] - P[:, 0, 0]) * (P[:, 2, 1] - P[:, 0, 1]) - (P[:, 2, 0] - P[:, 0, 0]) * (P[:, 1, 1] - P[:, 0, 1]))
    P = P[a > 1e-6]
    g = shapely.union_all(shapely.make_valid(shapely.polygons(np.concatenate([P, P[:, :1]], 1)))).buffer(0.15).buffer(-0.15)
    g = g.intersection(shapely.box(*core_merge.core_rect(meta)))
    open(wkb, "wb").write(shapely.to_wkb(g))
    return g


def cut_region(bd, meta, patch, lanes=True):
    """What the model's ground loses: the patch, plus CUT_EDGE m along the model's
    asphalt beside it (its own kerb faces stand on that edge, now inside the lanes).
    / 切る範囲: パッチと、隣のアスファルト縁の CUT_EDGE m（元の縁石の面）"""
    import shapely
    import core_merge
    if patch is None or patch.is_empty:
        return patch
    asp = core_merge.core_asphalt(bd, meta, patch=False)
    edge = patch.buffer(CUT_EDGE).intersection(asp.buffer(CUT_EDGE)) if asp is not None else shapely.Polygon()
    parts = list(shapely.get_parts(patch)) + list(shapely.get_parts(edge))
    if lanes:
        parts += list(shapely.get_parts(lanes_inside(bd)))
    return shapely.GeometryCollection([g for g in parts if g.geom_type == "Polygon" and not g.is_empty])


def lanes_inside(bd):
    """The lanes' tested interior over the core (drive_patch writes it). / 車線内部"""
    import shapely
    p = os.path.join(bd, "core", "drive_lanes.wkb")
    return shapely.from_wkb(open(p, "rb").read()) if os.path.exists(p) else shapely.Polygon()


# ---------------------------------------------------------------- triangles
def clip_tris(P, patch):
    """Triangles (n, 3, 3) minus the patch. Returns (P2 (m, 3, 3), src (m,), W (m, 3, 3)):
    each new vertex = sum_k W[.., k] * P[src][k] (barycentric, so z, UV and any
    per-corner attribute follow the old triangle's plane). Faces seen edge-on from
    above (kerb faces, walls) go when their middle lies in the patch.
    三角形からパッチを除く（新しい頂点は元の三角形の重心座標）"""
    import shapely
    n = len(P)
    I3 = np.tile(np.eye(3), (n, 1, 1))
    if patch is None or patch.is_empty or n == 0:
        return P, np.arange(n), I3
    xy = P[:, :, :2]
    cross = (xy[:, 1, 0] - xy[:, 0, 0]) * (xy[:, 2, 1] - xy[:, 0, 1]) - \
            (xy[:, 2, 0] - xy[:, 0, 0]) * (xy[:, 1, 1] - xy[:, 0, 1])
    flat = np.abs(cross) > 1e-6
    keep = np.ones(n, bool)
    # the region as separate pieces (patch parts, lane interiors): each triangle meets
    # only its own few / 部品ごと（三角形ごとに近くの数個だけ）
    tiles = np.array([g for g in shapely.get_parts(patch) if g.geom_type == "Polygon" and g.area > 1e-9], dtype=object)
    ttree = shapely.STRtree(tiles)

    def local(g):
        idx = ttree.query(g, predicate="intersects")
        return shapely.union_all(tiles[idx]) if len(idx) else None
    new_P, new_src, new_W = [], [], []
    # edge-on faces (kerb faces, walls): clipped along their length in (t, z) / 真横の面は長さ方向に切る
    eo = np.nonzero(~flat)[0]
    if len(eo):
        lines = shapely.linestrings(xy[eo])
        for k in np.unique(ttree.query(lines, predicate="intersects")[0]):
            i = eo[k]
            loc = local(lines[k])
            if loc is None:
                continue
            keep[i] = False
            q = xy[i]
            span = np.ptp(q, axis=0)
            if span.max() < 1e-6:
                continue
            j0 = int(np.argmax(span))
            a_, b_ = q[np.argmin(q[:, j0])], q[np.argmax(q[:, j0])]
            u = (b_ - a_) / np.linalg.norm(b_ - a_)
            tz = np.column_stack([(q - a_) @ u, P[i, :, 2]])                    # (3, 2)
            M = np.array([[tz[0, 0] - tz[2, 0], tz[1, 0] - tz[2, 0]], [tz[0, 1] - tz[2, 1], tz[1, 1] - tz[2, 1]]])
            if abs(np.linalg.det(M)) < 1e-9:
                continue
            Mi = np.linalg.inv(M)
            rest = shapely.LineString([a_, b_]).difference(loc)
            for seg in shapely.get_parts(rest):
                if seg.is_empty or seg.length < 1e-4:
                    continue
                ts = sorted(float((np.asarray(c) - a_) @ u) for c in np.asarray(seg.coords)[[0, -1]])
                strip = shapely.box(ts[0], tz[:, 1].min() - 1, ts[1], tz[:, 1].max() + 1)
                piece = shapely.Polygon(tz).intersection(strip)
                for tri in shapely.get_parts(shapely.constrained_delaunay_triangles(piece)) if not piece.is_empty else []:
                    if tri.area < 1e-8:
                        continue
                    C = np.asarray(tri.exterior.coords)[:3]
                    l12 = (Mi @ (C - tz[2]).T).T
                    W = np.column_stack([l12, 1.0 - l12.sum(1)])                   # (3, 3)
                    Q = W @ P[i]
                    n0 = np.cross(P[i, 1] - P[i, 0], P[i, 2] - P[i, 0])
                    if np.cross(Q[1] - Q[0], Q[2] - Q[0]) @ n0 < 0:
                        Q, W = Q[::-1], W[::-1]
                    new_P.append(Q[None]); new_W.append(W[None]); new_src.append(np.array([i]))
    fi = np.nonzero(flat)[0]
    tri = shapely.polygons(np.concatenate([xy[fi], xy[fi][:, :1]], axis=1))
    for k in np.unique(ttree.query(tri, predicate="intersects")[0]):
        i = fi[k]
        loc = local(tri[k])
        if loc is None:
            continue
        keep[i] = False
        rest = tri[k].difference(loc)
        if rest.is_empty or rest.area < 1e-6:
            continue
        parts = [q for q in shapely.get_parts(shapely.constrained_delaunay_triangles(rest)) if q.area > 1e-7]
        if not parts:
            continue
        C = np.array([np.asarray(q.exterior.coords)[:3] for q in parts])         # (k, 3, 2)
        a, b, c = xy[i]
        T = np.array([[a[0] - c[0], b[0] - c[0]], [a[1] - c[1], b[1] - c[1]]])
        Ti = np.linalg.inv(T)
        l12 = np.einsum("ij,kvj->kvi", Ti, C - c)                                # (k, 3, 2)
        W = np.concatenate([l12, 1.0 - l12.sum(-1, keepdims=True)], -1)          # (k, 3, 3)
        Q = np.einsum("kvj,jd->kvd", W, P[i])
        # same winding as the old face / 元の面と同じ向き
        qc = (Q[:, 1, 0] - Q[:, 0, 0]) * (Q[:, 2, 1] - Q[:, 0, 1]) - (Q[:, 2, 0] - Q[:, 0, 0]) * (Q[:, 1, 1] - Q[:, 0, 1])
        flip = np.sign(qc) != np.sign(cross[i])
        Q[flip] = Q[flip][:, ::-1]
        W[flip] = W[flip][:, ::-1]
        new_P.append(Q); new_W.append(W); new_src.append(np.full(len(Q), i))
    idx = np.nonzero(keep)[0]
    P2 = [P[idx]] + new_P
    src = [idx] + new_src
    W2 = [I3[idx]] + new_W
    return np.concatenate(P2), np.concatenate(src), np.concatenate(W2)


def surface_top(tris, pts):
    """Highest up-facing triangle z under each point (nan where none). / 各点の上面の高さ"""
    import shapely
    out = np.full(len(pts), np.nan)
    if not len(tris):
        return out
    xy = tris[:, :, :2]
    cross = (xy[:, 1, 0] - xy[:, 0, 0]) * (xy[:, 2, 1] - xy[:, 0, 1]) - (xy[:, 2, 0] - xy[:, 0, 0]) * (xy[:, 1, 1] - xy[:, 0, 1])
    up = np.abs(cross) > 1e-6
    T = tris[up]
    tree = shapely.STRtree(shapely.polygons(np.concatenate([T[:, :, :2], T[:, :1, :2]], axis=1)))
    pi, ti = tree.query(shapely.points(pts), predicate="intersects")
    for i, t in zip(pi, ti):
        (x1, y1, z1), (x2, y2, z2), (x3, y3, z3) = T[t]
        det = (y2 - y3) * (x1 - x3) + (x3 - x2) * (y1 - y3)
        if abs(det) < 1e-12:
            continue
        a = ((y2 - y3) * (pts[i, 0] - x3) + (x3 - x2) * (pts[i, 1] - y3)) / det
        b = ((y3 - y1) * (pts[i, 0] - x3) + (x1 - x3) * (pts[i, 1] - y3)) / det
        z = a * z1 + b * z2 + (1 - a - b) * z3
        if np.isnan(out[i]) or z > out[i]:
            out[i] = z
    return out


def build_fill(bd, meta, patch, ground, z_road, log=print):
    """Fill and kerb-face triangles (world z) -> core/drive_patch.npz.
    ground: the model's uncut non-asphalt ground triangles (world z), for the
    height beside each new edge. / 舗装と縁石面の三角形"""
    import shapely
    from shapely.geometry import box
    import core_merge
    out = os.path.join(bd, "core", "drive_patch.npz")
    if patch is None or patch.is_empty:
        np.savez_compressed(out, fill=np.zeros((0, 3, 3)), faces=np.zeros((0, 3, 3)))
        return np.zeros((0, 3, 3)), np.zeros((0, 3, 3))
    tris = [q for q in shapely.get_parts(shapely.constrained_delaunay_triangles(patch)) if q.area > 1e-7]
    C = np.array([np.asarray(q.exterior.coords)[:3] for q in tris])
    cr = (C[:, 1, 0] - C[:, 0, 0]) * (C[:, 2, 1] - C[:, 0, 1]) - (C[:, 2, 0] - C[:, 0, 0]) * (C[:, 1, 1] - C[:, 0, 1])
    C[cr < 0] = C[cr < 0][:, ::-1]                       # facing up / 上向き
    fill = np.concatenate([C, np.full(C.shape[:2] + (1,), z_road)], -1)
    # kerb faces along the new edge where the ground beside stands higher or lower
    # 新しい縁: 隣の地面との段差に縁石の面 /
    asp = core_merge.core_asphalt(bd, meta, patch=False)
    sq_edge = box(*core_merge.core_rect(meta)).exterior
    near_asp = asp.buffer(0.03) if asp is not None else shapely.Polygon()
    shapely.prepare(near_asp)
    segs = []
    for poly in shapely.get_parts(patch):
        for is_ext, ring in [(True, poly.exterior)] + [(False, r) for r in poly.interiors]:
            ccw = ring.is_ccw
            X = np.asarray(ring.coords)
            for a, b in zip(X[:-1], X[1:]):
                d = b - a
                Ls = float(np.hypot(*d))
                if Ls < 1e-3:
                    continue
                # outward normal: exterior CCW -> right of travel; holes the other way / 外向き法線
                nrm = np.array([d[1], -d[0]]) / Ls
                if is_ext != ccw:
                    nrm = -nrm
                k = max(1, int(np.ceil(Ls / 1.0)))
                for j in range(k):
                    p0, p1 = a + d * (j / k), a + d * ((j + 1) / k)
                    segs.append((p0, p1, nrm))
    if not segs:
        faces = np.zeros((0, 3, 3))
    else:
        mid = np.array([(p0 + p1) / 2 for p0, p1, _ in segs])
        nrm = np.array([n for _, _, n in segs])
        probe = mid + nrm * 0.08
        ok = ~shapely.contains_xy(near_asp, probe[:, 0], probe[:, 1])
        lin = np.array([g.buffer(0.05) for g in shapely.get_parts(lanes_inside(bd))], dtype=object)
        if len(lin):
            pi, _ = shapely.STRtree(lin).query(shapely.points(mid), predicate="within")
            ok[np.unique(pi)] = False                                      # never inside a lane / 車線内には置かない
        ok &= shapely.distance(shapely.points(mid), sq_edge) > 0.05
        zt = surface_top(ground, probe)
        F = []
        for (p0, p1, n), good, z in zip(segs, ok, zt):
            if not good or np.isnan(z) or abs(z - z_road) < FACE_MIN:
                continue
            lo, hi = min(z, z_road), max(z, z_road)
            A, B, Bt, At = (np.array([*p0, lo]), np.array([*p1, lo]), np.array([*p1, hi]), np.array([*p0, hi]))
            # facing the road (into the patch, -n) / 路面側を向く
            t1, t2 = np.array([A, Bt, B]), np.array([A, At, Bt])
            nn = np.cross(t1[1] - t1[0], t1[2] - t1[0])
            if nn[:2] @ (-n) < 0:
                t1, t2 = t1[[0, 2, 1]], t2[[0, 2, 1]]
            F += [t1, t2]
        faces = np.array(F) if F else np.zeros((0, 3, 3))
    np.savez_compressed(out, fill=fill, faces=faces)
    log("[core_patch] paved %.0f m2 at the model's road height (z %.2f), %d kerb-face triangles along the new edges"
        % (patch.area, z_road, len(faces)))
    return fill, faces
