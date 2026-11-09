"""Loose props of an embedded model (maps.json "core") the extended map leaves
out (Blender, in the core_fbx.py JOIN).

  - drop_bicycles: the parked bicycles of the v8 model's street fronts
    (source/streetfront.py: rows of mamachari in front of shops and flat
    entrances, in Street_Furniture). The user asked for them to go
    (2026-11-08: "delete all bycycle in the font of store"); their shops are
    gone too (every model building but the station is generated now).
    A bicycle = two M_tyre rings 1.0 m apart in one vertical plane; every face
    within 0.42 m (sideways) of the axis between them and below 1.1 m goes
    (frame, seat, basket).
  - drop_shopfront: the rest of the v8 street-front props (streetfront.decorate,
    vending.py) standing in front of the replaced shops (user 2026-11-08:
    "improve it into japan style and put it in the right place"; shopfront_jp.py
    puts Japanese ones by the generated doors): A-frame menu boards
    (M_board_atlas) and the hero vending pair at the station plazas
    (M_vm_shell/misc/disp/lid/glass/inside) go whole (only streetfront draws
    them); vending boxes (M_vm_box) with a face within 0.5 m of a v8 building
    wall (their backs stood 0.04 m off it; zone.py's car-park row of 4 stays);
    planters (soil top <= 1.3 x 0.5 m, wood or granite box 0.45 m tall) and pots
    (M_planter_red, r <= 0.24, 0.2-0.36 m tall) within 1.5 m of a v8 wall, with
    their leaf blobs (and any leaf group left within 0.9 m above a removed
    item's foot: nothing floats). Plaza planters (1.9 m boxes, the 2 m ring),
    benches, lamps, bollards, fences, parasols, bins and shelters stay.
    The v8 wall feet come from the model's Buildings_* (core/v8_walls.npz,
    ensure_walls, made once with Blender).
  - drop_cut_texts: the model's road words (止まれ etc., Road_Marking_Markings)
    that the core cut sliced through at the rectangle: the left-over half is a
    broken glyph (agent-checker, 2026-11-09), so the whole word within 7 m of
    a cut goes; the generated markings own those approaches.

埋め込みモデルの小物のうち、拡張マップで除くもの（店先の自転車・看板・鉢・自販機）。
"""
import os

import numpy as np

TYRE = "M_tyre"
WHEELBASE = (0.85, 1.15)     # rear to front wheel centre (streetfront.bicycle: 1.0 m) / 前後輪の間隔
REACH = 0.42                 # frame, seat, basket within this of the axis / 軸からの範囲
TOP = 1.1                    # basket top 1.0 m / かご上端
# tyres, basket, saddle and the frame colours (streetfront.BIKE_MATS); nothing else goes
# タイヤ・かご・サドル・フレームの色だけ
BIKE = {TYRE, "M_metal_silver", "M_car_trim", "M_steel_dark", "M_planter_red", "M_vending_body", "M_paint_white"}


def _seg_dist(P, A, B):
    """Distance of points P (n, 2) to segments A-B (n, 2) each."""
    d = B - A
    t = np.clip(((P - A) * d).sum(1) / np.maximum((d * d).sum(1), 1e-9), 0.0, 1.0)
    return np.linalg.norm(P - (A + d * t[:, None]), axis=1)


def drop_bicycles(objs, log=print):
    import bmesh
    from mathutils import kdtree
    n_bikes = n_faces = 0
    # (the FBX splits the rings into loose faces: a "wheel" here is one tyre
    # face, paired with one 1 m along; the bicycle count is len(tyre faces) / 96)
    for o in objs:
        if o.type != "MESH":
            continue
        mats = [m.name if m else "" for m in o.data.materials]
        if TYRE not in mats:
            continue
        ti = mats.index(TYRE)
        bm = bmesh.new()
        bm.from_mesh(o.data)
        bm.faces.ensure_lookup_table()
        # tyre rings: connected M_tyre faces / タイヤの輪
        seen, wheels = set(), []
        for f in bm.faces:
            if f.material_index != ti or f.index in seen:
                continue
            st, ring = [f], []
            seen.add(f.index)
            while st:
                g = st.pop()
                ring.append(g)
                for e in g.edges:
                    for h in e.link_faces:
                        if h.material_index == ti and h.index not in seen:
                            seen.add(h.index)
                            st.append(h)
            V = np.array([v.co[:] for g in ring for v in g.verts])
            c = V.mean(0)
            q = V[:, :2] - c[:2]
            u = np.linalg.svd(q, full_matrices=False)[2][0]       # the ring's plane, horizontally / 輪の面の向き
            wheels.append((c, u))
        if not wheels:
            bm.free()
            continue
        C = np.array([w[0] for w in wheels])
        U = np.array([w[1] for w in wheels])
        kd = kdtree.KDTree(len(C))
        for i, c in enumerate(C):
            kd.insert((c[0], c[1], 0.0), i)
        kd.balance()
        # pair each wheel with the one 1 m along its own plane (rows stand side by side)
        # 同じ面内で 1 m 先の輪と組にする（自転車は横に並ぶ）
        used, segs = set(), []
        for i in range(len(C)):
            if i in used:
                continue
            best = None
            for _, j, d in kd.find_range((C[i][0], C[i][1], 0.0), WHEELBASE[1]):
                if j == i or j in used or d < WHEELBASE[0]:
                    continue
                v = (C[j] - C[i])[:2] / d
                if abs(v @ U[i]) > 0.97 and abs(v @ U[j]) > 0.97 and abs(C[j][2] - C[i][2]) < 0.1:
                    if best is None or abs(d - 1.0) < abs(best[1] - 1.0):
                        best = (j, d)
            if best is not None:
                used |= {i, best[0]}
                segs.append((C[i], C[best[0]]))
        if not segs:
            bm.free()
            continue
        A = np.array([s[0][:2] for s in segs])
        B = np.array([s[1][:2] for s in segs])
        Z0 = np.array([min(s[0][2], s[1][2]) - 0.34 for s in segs])     # ground under the wheels / 車輪の下の地面
        M = (A + B) / 2
        kd2 = kdtree.KDTree(len(M))
        for i, m in enumerate(M):
            kd2.insert((m[0], m[1], 0.0), i)
        kd2.balance()
        bike_mi = {i for i, m in enumerate(mats) if m in BIKE}
        dead = []
        for f in bm.faces:
            if f.material_index not in bike_mi:
                continue
            P = np.array([v.co[:] for v in f.verts])
            c = P[:, :2].mean(0)
            n = len(P)
            for _, k, _ in kd2.find_range((c[0], c[1], 0.0), 0.5 + REACH):   # every bicycle in reach (rows) / 届く範囲の全自転車
                if (_seg_dist(P[:, :2], np.repeat(A[k:k + 1], n, 0), np.repeat(B[k:k + 1], n, 0)).max() <= REACH
                        and (P[:, 2] - Z0[k]).max() <= TOP and (P[:, 2] - Z0[k]).min() >= -0.1):
                    dead.append(f)
                    break
        n_tyre = sum(1 for f in dead if f.material_index == ti)
        bmesh.ops.delete(bm, geom=dead, context="FACES")
        loose = [v for v in bm.verts if not v.link_faces]
        bmesh.ops.delete(bm, geom=loose, context="VERTS")
        bm.to_mesh(o.data)
        bm.free()
        o.data.update()
        n_bikes += n_tyre // 96
        n_faces += len(dead)
        left = sum(1 for p in o.data.polygons if p.material_index == ti)
        log("[core_props] %s: about %d bicycles (%d faces) removed, %d tyre faces left (unpaired wheels)"
            % (o.name, n_tyre // 96, len(dead), left))
    return n_bikes, n_faces


# ------------------------------------------------------------ v8 shop-front props / 店先の小物
# v8 streetfront.py / vending.py materials (read-only study of the user's generator)
# hero vending pair (vending.hero) and A-frame menu boards (streetfront.board): only
# streetfront.py draws these, every face goes / 店先専用の材質: すべて除く
SF_ONLY = ("M_board_atlas", "M_vm_shell", "M_vm_misc", "M_vm_disp", "M_vm_lid", "M_vm_glass", "M_vm_inside")
VM_BOX = "M_vm_box"          # vending.box: streetfront + zone.py's car-park row (kept) / 店先と駐車場の自販機
SOIL = "M_soil"              # planter top (streetfront 0.75/1.2 x 0.42; north.py plaza 1.9 x 1.9 kept)
PLANTER = ("M_planter_wood", "M_granite")
POT = "M_planter_red"        # streetfront.pot r 0.15-0.19 x 0.32 (north.py ring r 1.95 kept)
LEAVES = "M_leaves_b"
WALL_NEAR = 1.5              # street-front items stand within this of a v8 building wall / 壁からの距離


def _clusters(C, link):
    """Single-linkage groups of points C (n, 3) within ``link`` (xy). / 近い点のまとまり"""
    from mathutils import kdtree
    n = len(C)
    if not n:
        return []
    kd = kdtree.KDTree(n)
    for i, c in enumerate(C):
        kd.insert((c[0], c[1], 0.0), i)
    kd.balance()
    lab = -np.ones(n, int)
    k = 0
    for i in range(n):
        if lab[i] >= 0:
            continue
        lab[i] = k
        st = [i]
        while st:
            j = st.pop()
            for _, m, _ in kd.find_range((C[j][0], C[j][1], 0.0), link):
                if lab[m] < 0:
                    lab[m] = k
                    st.append(m)
        k += 1
    return [np.nonzero(lab == g)[0] for g in range(k)]


def _min_rect(q):
    """Axes (2, 2) and side lengths (long, short) of the smallest rectangle round
    points q (n, 2) centred near 0, tried along every pair direction. / 最小外接矩形"""
    best = None
    for i in range(len(q)):
        for j in range(i + 1, len(q)):
            d = q[j] - q[i]
            ln = np.hypot(*d)
            if ln < 1e-3:
                continue
            a = np.array([d / ln, [-d[1] / ln, d[0] / ln]])
            e = np.abs(q @ a.T).max(0) * 2
            if best is None or e[0] * e[1] < best[1][0] * best[1][1]:
                best = (a, e)
    if best is None:
        return np.eye(2), np.zeros(2)
    a, e = best
    return (a, e) if e[0] >= e[1] else (a[::-1], e[::-1])


def _wall_dist(walls, P):
    """Distance (xy) of points P (n, 2) to the nearest v8 wall-foot segment. / 壁足までの距離"""
    if walls is None or not len(P):
        return np.full(len(P), np.inf)
    A, B = walls[:, 0:2], walls[:, 2:4]
    out = np.full(len(P), np.inf)
    lo = P.min(0) - 3.0
    hi = P.max(0) + 3.0
    m = ((np.minimum(A, B) <= hi).all(1) & (np.maximum(A, B) >= lo).all(1))
    A, B = A[m], B[m]
    if not len(A):
        return out
    d = B - A
    for i, p in enumerate(P):
        t = np.clip(((p - A) * d).sum(1) / np.maximum((d * d).sum(1), 1e-9), 0, 1)
        out[i] = np.min(np.hypot(*(A + d * t[:, None] - p).T))
    return out


def drop_shopfront(objs, bd, log=print):
    """The v8 street-front shop props: menu boards, door planters / pots and the
    vending machines (incl. the hero pair) in front of the replaced buildings.
    Returns {kind: count}, faces removed. / v8 の店先小物を除く"""
    import bmesh
    wp = os.path.join(bd, "core", "v8_walls.npz")
    walls = np.load(wp)["segs"] if os.path.exists(wp) else None
    if walls is None:
        log("[core_props] WARNING: no %s (core_props.ensure_walls): only boards and the hero machines go" % wp)
    cnt = {"board": 0, "vending_hero": 0, "vending": 0, "planter": 0, "pot": 0, "kept_vending": 0,
           "kept_planter": 0, "kept_pot": 0, "leaf_bits": 0}
    n_faces = 0
    for o in objs:
        if o.type != "MESH" or "Street_Furniture" not in o.name:
            continue
        mats = [m.name if m else "" for m in o.data.materials]
        mi = {m: i for i, m in enumerate(mats)}
        mw = o.matrix_world
        bm = bmesh.new()
        bm.from_mesh(o.data)
        bm.faces.ensure_lookup_table()
        F = list(bm.faces)
        FM = np.array([f.material_index for f in F])
        FV = [np.array([(mw @ v.co)[:] for v in f.verts]) for f in F]
        FC = np.array([v.mean(0) for v in FV])
        Fz0 = np.array([v[:, 2].min() for v in FV])
        Fz1 = np.array([v[:, 2].max() for v in FV])
        dead = np.zeros(len(F), bool)
        gone_at = []                 # (x, y, foot z) of each removed planter / pot / 除いた物の位置

        def sel(name):
            return np.nonzero(FM == mi[name])[0] if name in mi else np.zeros(0, int)

        # 1. boards and hero machines: every face / 看板・ヒーロー自販機は全部
        for name in SF_ONLY:
            ids = sel(name)
            dead[ids] = True
            if name == "M_board_atlas":
                cnt["board"] += len(_clusters(FC[ids], 0.45))
            if name == "M_vm_inside":
                cnt["vending_hero"] += len(ids) // 2
        # 2. vending boxes against a v8 building wall (back 0.04 m off it) / 壁際の自販機
        ids = sel(VM_BOX)
        for g in _clusters(FC[ids], 0.9):
            gi = ids[g]
            P = np.concatenate([FV[i][:, :2] for i in gi])
            n_m = max(1, len(gi) // 10)
            if _wall_dist(walls, P).min() <= 0.5:
                dead[gi] = True
                cnt["vending"] += n_m
            else:
                cnt["kept_vending"] += n_m
        # 3. planter boxes: a small soil top (<= 1.3 x 0.5 m) by a wall / 小さなプランター
        leaf_ids = sel(LEAVES)
        box_ids = np.concatenate([sel(m) for m in PLANTER]) if any(m in mi for m in PLANTER) else np.zeros(0, int)
        ids = sel(SOIL)
        for g in _clusters(FC[ids], 0.5):
            gi = ids[g]
            P = np.concatenate([FV[i] for i in gi])
            c = P[:, :2].mean(0)
            ax, ext = _min_rect(P[:, :2] - c)
            top = P[:, 2].max()
            small = ext[0] <= 1.3 and ext[1] <= 0.5
            near = _wall_dist(walls, c[None])[0] <= WALL_NEAR
            if not (small and near):
                cnt["kept_planter"] += 1
                continue
            dead[gi] = True
            cnt["planter"] += 1
            gone_at.append((c[0], c[1], top - 0.45))

            def inside(Pf, grow, z0, z1):
                l = np.abs((Pf[:, :2] - c) @ ax.T)
                return bool((l[:, 0] <= ext[0] / 2 + grow).all() and (l[:, 1] <= ext[1] / 2 + grow).all()
                            and Pf[:, 2].min() >= z0 and Pf[:, 2].max() <= z1)
            for i in box_ids:
                if not dead[i] and inside(FV[i], 0.03, top - 0.48, top + 0.01):
                    dead[i] = True
            for i in leaf_ids:
                if not dead[i] and inside(FV[i], 0.12, top - 0.05, top + 0.75):
                    dead[i] = True
        # 3b. planter boxes whose soil top is gone (cut pieces): the bare small box
        # 土の無いプランター（切断で上面が欠けたもの）
        rest = box_ids[~dead[box_ids]]
        for g in _clusters(FC[rest], 0.5):
            gi = rest[g]
            P = np.concatenate([FV[i] for i in gi])
            c = P[:, :2].mean(0)
            ax, ext = _min_rect(P[:, :2] - c)
            h = P[:, 2].max() - P[:, 2].min()
            if ext[0] <= 1.3 and ext[1] <= 0.5 and 0.3 <= h <= 0.5 and _wall_dist(walls, c[None])[0] <= WALL_NEAR:
                dead[gi] = True
                cnt["planter"] += 1
                gone_at.append((c[0], c[1], P[:, 2].min()))
        # 4. pots: small M_planter_red cylinders by a wall, and their leaf blob / 鉢と葉
        ids = sel(POT)
        for g in _clusters(FC[ids], 0.25):
            gi = ids[g]
            P = np.concatenate([FV[i] for i in gi])
            c = P[:, :2].mean(0)
            r = np.hypot(*(P[:, :2] - c).T).max()
            h = P[:, 2].max() - P[:, 2].min()
            if not (r <= 0.24 and 0.2 <= h <= 0.36 and _wall_dist(walls, c[None])[0] <= WALL_NEAR):
                cnt["kept_pot"] += 1
                continue
            dead[gi] = True
            cnt["pot"] += 1
            z0 = P[:, 2].min()
            gone_at.append((c[0], c[1], z0))
            # its blob (streetfront.pot: centre 0.55 m up, radii 0.22-0.26, jittered) / 鉢の葉
            for i in leaf_ids:
                if not dead[i] and np.hypot(*(FC[i][:2] - c)) <= 0.42 and z0 + 0.15 <= FC[i][2] <= z0 + 1.0:
                    dead[i] = True
        # 5. what is left of a removed item's leaf blob (jittered faces past the box)
        # floats: every leaf group within 0.9 m of a removed item, above its foot
        # 取り除いた物の葉の残り（浮く）も除く
        if gone_at:
            G = np.array(gone_at)
            rest = leaf_ids[~dead[leaf_ids]]
            for g in _clusters(FC[rest], 0.35):
                gi = rest[g]
                c = FC[gi].mean(0)
                d = np.hypot(*(G[:, :2] - c[:2]).T)
                k = int(np.argmin(d))
                if d[k] <= 0.9 and Fz0[gi].min() >= G[k, 2] + 0.1:
                    dead[gi] = True
                    cnt["leaf_bits"] += 1
        kill = [F[i] for i in np.nonzero(dead)[0]]
        bmesh.ops.delete(bm, geom=kill, context="FACES")
        loose = [v for v in bm.verts if not v.link_faces]
        bmesh.ops.delete(bm, geom=loose, context="VERTS")
        bm.to_mesh(o.data)
        bm.free()
        o.data.update()
        n_faces += len(kill)
        log("[core_props] %s: v8 shop-front props removed: %d menu boards, %d hero + %d vending machines, "
            "%d planters, %d pots (%d faces); kept off the walls: %d vending, %d planters, %d pots"
            % (o.name, cnt["board"], cnt["vending_hero"], cnt["vending"], cnt["planter"], cnt["pot"], len(kill),
               cnt["kept_vending"], cnt["kept_planter"], cnt["kept_pot"]))
    return cnt, n_faces


TEXT_MATS = ("M_road_text", "road_text_jp")    # before / after core_materials / 路面文字
TEXT_REACH = 7.0                               # a word's extent from its cut tile / 語の長さ


def drop_cut_texts(objs, bd, log=print, tol=0.05):
    """Remove the model's road words cut by the core rectangle (Blender, JOIN).
    / コアの境界で切れた路面文字を語ごと除く"""
    import json
    import bmesh
    from core_merge import core_rect
    r = core_rect(json.load(open(os.path.join(bd, "meta.json"))))
    n_words = n_faces = 0
    for o in objs:
        if o.type != "MESH":
            continue
        mats = [m.name if m else "" for m in o.data.materials]
        tm = {i for i, m in enumerate(mats) if m in TEXT_MATS or m.split(".")[0] in TEXT_MATS}
        if not tm:
            continue
        mw = o.matrix_world
        bm = bmesh.new()
        bm.from_mesh(o.data)
        faces = [f for f in bm.faces if f.material_index in tm]
        if not faces:
            bm.free()
            continue
        P = [np.array([(mw @ v.co)[:2] for v in f.verts]) for f in faces]
        C = np.array([p.mean(0) for p in P])
        on_edge = np.array([bool(((np.abs(p[:, 0] - r[0]) < tol) | (np.abs(p[:, 0] - r[2]) < tol)
                                  | (np.abs(p[:, 1] - r[1]) < tol) | (np.abs(p[:, 1] - r[3]) < tol)).any())
                            for p in P])
        if not on_edge.any():
            bm.free()
            continue
        cuts = C[on_edge]
        near = np.min(np.linalg.norm(C[:, None, :] - cuts[None, :, :], axis=2), axis=1) <= TEXT_REACH
        dead = [f for f, k in zip(faces, near) if k]
        # words: cut tiles more than a word apart / 語の数（切れたタイルを語ごとにまとめる）
        words = []
        for c in cuts:
            if not any(np.linalg.norm(c - w) < TEXT_REACH for w in words):
                words.append(c)
        bmesh.ops.delete(bm, geom=dead, context="FACES")
        bmesh.ops.delete(bm, geom=[v for v in bm.verts if not v.link_faces], context="VERTS")
        bm.to_mesh(o.data)
        bm.free()
        o.data.update()
        n_words += len(words)
        n_faces += len(dead)
        log("[core_props] %s: %d road words cut by the core edge removed (%d faces) at %s"
            % (o.name, len(words), len(dead), ", ".join("(%.0f, %.0f)" % tuple(w) for w in words)))
    return n_words, n_faces


WALLS_DUMP = r'''
import bpy, sys, numpy as np
from mathutils import Matrix
a = sys.argv[sys.argv.index("--") + 1:]
src, out, SX = a[0], a[1], float(a[2])
bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.import_scene.fbx(filepath=src)
S = Matrix.Diagonal((SX, 1.0, 1.0, 1.0))
segs, owner, names = [], [], []
for o in bpy.data.objects:
    if o.type != "MESH" or not (o.parent and o.parent.name.startswith("Buildings_")):
        continue
    M = S @ o.matrix_world
    V = np.array([(M @ v.co)[:] for v in o.data.vertices])
    if not len(V):
        continue
    z0 = V[:, 2].min()
    k = len(names); names.append(o.name)
    for p in o.data.polygons:
        P = V[list(p.vertices)]
        if P[:, 2].min() > z0 + 0.6:
            continue
        n = np.cross(P[1] - P[0], P[2] - P[0]); ln = np.linalg.norm(n)
        if ln < 1e-9 or abs(n[2]) / ln > 0.2:
            continue
        low = P[P[:, 2] < P[:, 2].min() + 0.05][:, :2]
        if len(low) < 2:
            continue
        i, j = np.unravel_index(np.argmax(((low[:, None] - low[None]) ** 2).sum(-1)), (len(low), len(low)))
        if np.hypot(*(low[i] - low[j])) < 0.05:
            continue
        segs.append([low[i][0], low[i][1], low[j][0], low[j][1], P[:, 2].min()]); owner.append(k)
np.savez_compressed(out, segs=np.array(segs, np.float32), owner=np.array(owner, np.int32), names=np.array(names))
print("[core_props] v8 wall feet: %d buildings, %d segments" % (len(names), len(segs)))
'''


def ensure_walls(bd, src, x_scale):
    """core/v8_walls.npz: the wall feet (bottom edges of the vertical faces near
    each building's foot) of the model's Buildings_*, x-scaled like the cut;
    made once with Blender, redone when the model is newer. / v8 建物の壁足。"""
    import subprocess
    import tempfile
    out = os.path.join(bd, "core", "v8_walls.npz")
    if os.path.exists(out) and os.path.getmtime(out) >= os.path.getmtime(src):
        return out
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with tempfile.TemporaryDirectory() as td:
        sp = os.path.join(td, "walls.py")
        open(sp, "w").write(WALLS_DUMP)
        subprocess.check_call([os.environ.get("BLENDER", "blender"), "-b", "--disable-autoexec", "-P", sp, "--",
                               src, out, str(x_scale)], stdout=subprocess.DEVNULL)
    return out
