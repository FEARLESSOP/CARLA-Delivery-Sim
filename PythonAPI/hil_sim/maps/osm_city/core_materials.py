"""The embedded model's ground in the generated city's materials (maps.json
"core"; core_fbx.py JOIN step). The user (2026-11-08): every texture of the
old model but the station building revised to the generator's look.

  python core_materials.py <build dir>     (prepare only; core_fbx.py calls it)

prepare(bd, prep_fbx), maps/.venv, before the join:
  - dumps the model's flat asphalt (Road_Road_Roads / M_asphalt) and its
    止まれ texture from core/core_prepared.fbx (Blender);
  - cuts the asphalt like the generated carriageway (surfaces_jp.partition on
    the model's own asphalt outline with the road logic's lanes and road
    classes from markings.pkl): wheel paths on TERT+ lanes, repairs, estate
    streets oxidised with a cracked edge -> core/asphalt_cut.npz (each
    triangle in its old plane: the model's asphalt is flat, z 0);
  - writes the two textures the generator lacks, drawn its way
    (textures_jp): pave_plaza (granite setts, grey with darker bands, the
    north plaza in Street View) and road_text_jp (the model's 止まれ glyphs in
    line_white paint on the asphalt under them, at its own scale: the model's
    black text box would show in Unreal, which takes no alpha).
apply(objs, bd), inside the JOIN Blender run after the generated FBX import:
  every ground triangle of the model gets the generator material of the same
  role (ROLE) with UVs in the generator's convention (world metres / tile,
  TILE = ground.py's uv_scale); the generated FBX's own material objects are
  reused, so core and city share one material per role in Unreal. Objects and
  their Road_* names stay (CARLA's semantic tags); the station building,
  railway, furniture, trees, buses and trains keep theirs.

埋め込みモデルの地面を生成都市と同じ材質にする（駅ビルを除く）。
"""
import hashlib
import json
import os
import subprocess
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))

# model object -> {model material: generator material or a rule} / 役割ごとの対応
#   "asphalt": the cut pieces (asphalt_cut.npz); "kerb": kerb_jp on top,
#   kerb_face_jp on the faces; "tactile": pave_guide on strips, pave_warn on
#   areas; "warn": pave_warn; None (or absent): the model's own material stays
ROLE = {
    "Road_Road_Roads": {"M_asphalt": "asphalt", "M_lot": "asphalt_estate"},
    "Terrain_Lots_Plots": {"M_plot": "jp_lot"},          # prepare_oita: plots are Terrain / 敷地は Terrain
    "Road_Sidewalk_Sidewalk": {"M_sidewalk": "pave_block", "M_tactile": "tactile"},
    "Road_Curb_Kerb": {"M_kerb": "kerb"},
    "Road_Grass_Lawn": {"M_grass": "grass"},
    "Road_Marking_Markings": {"M_line_white": "line_white", "M_line_yellow": "line_yellow",
                              "M_tactile": "warn", "M_road_text": "road_text_jp"},
    # plaza ground; its steps, walls and the subway entrance stay the model's
    # 広場の床面（階段・壁・地下入口はモデルのまま）
    "SM_Ground_Plaza": {"M_plaza": "pave_plaza", "M_shukusai_paving": "pave_plaza", "M_gravel": "soil"},
}
# metres per texture repeat (ground.py uv_scale; textures_jp.TILE) / テクスチャ 1 枚の大きさ
TILE = {"asphalt_jp": 5.0, "asphalt_wheel": 5.0, "asphalt_estate": 5.0, "asphalt_crack": 5.0,
        "asphalt_patch": 5.0, "jp_lot": 6.0, "pave_block": 2.0, "pave_plaza": 4.8, "grass": 4.0,
        "kerb_jp": 1.0, "kerb_face_jp": 1.2, "line_white": 1.0, "line_yellow": 1.0,
        "pave_warn": 0.3, "pave_guide": 0.3, "soil": 2.0}
# materials made here (texture, colour, roughness, metallic, smooth) / ここで作る材質
OWN = {"pave_plaza": ("pave_plaza", None, 0.8, 0.0, False),
       "road_text_jp": ("road_text_jp", None, 0.6, 0.0, False)}
GUIDE_MAX_W = 0.36       # a tactile triangle this narrow is a guide strip / これより細い点字ブロックは線状

DUMP = r'''
import bpy, sys, numpy as np
a = sys.argv[sys.argv.index("--") + 1:]
bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.import_scene.fbx(filepath=a[0])
o = bpy.data.objects["Road_Road_Roads"]
me = o.data; me.calc_loop_triangles()
names = [m.name for m in me.materials]
co = np.zeros(len(me.vertices) * 3); me.vertices.foreach_get("co", co); co = co.reshape(-1, 3)
n = len(me.loop_triangles)
tv = np.zeros(n * 3, np.int32); me.loop_triangles.foreach_get("vertices", tv)
tm = np.zeros(n, np.int32); me.loop_triangles.foreach_get("material_index", tm)
T = co[tv.reshape(-1, 3)][tm == names.index("M_asphalt")]
out = {"asphalt": T}
t = bpy.data.objects["Road_Marking_Markings"]
me = t.data; me.calc_loop_triangles()
names = [m.name for m in me.materials]
if "M_road_text" in names:
    k = names.index("M_road_text")
    co = np.zeros(len(me.vertices) * 3); me.vertices.foreach_get("co", co); co = co.reshape(-1, 3)
    uv = np.zeros(len(me.loops) * 2); me.uv_layers[0].data.foreach_get("uv", uv); uv = uv.reshape(-1, 2)
    n = len(me.loop_triangles)
    tv = np.zeros(n * 3, np.int32); me.loop_triangles.foreach_get("vertices", tv)
    tl = np.zeros(n * 3, np.int32); me.loop_triangles.foreach_get("loops", tl)
    tm = np.zeros(n, np.int32); me.loop_triangles.foreach_get("material_index", tm)
    s = tm == k
    out["text_P"] = co[tv.reshape(-1, 3)][s]
    out["text_UV"] = uv[tl.reshape(-1, 3)][s]
    m = me.materials[k]
    for nd in m.node_tree.nodes:
        if nd.type == "TEX_IMAGE" and nd.image and nd.outputs[0].links:
            im = nd.image
            px = np.array(im.pixels[:], np.float32).reshape(im.size[1], im.size[0], 4)
            out["text_img"] = px[::-1]          # Blender rows run up / 行を上から下へ
            break
np.savez_compressed(a[1], **out)
'''


# ----------------------------------------------------------------- prepare (maps/.venv)
def _code():
    return hashlib.sha1(open(os.path.abspath(__file__), "rb").read()
                        + open(os.path.join(HERE, "surfaces_jp.py"), "rb").read()
                        + open(os.path.join(HERE, "textures_jp.py"), "rb").read()).hexdigest()


def prepare(bd, prep):
    """Asphalt cut + own textures for apply(); cached in <bd>/core. / 準備（キャッシュ付き）"""
    cd = os.path.join(bd, "core")
    tex = os.path.join(bd, "textures")
    stamp = os.path.join(cd, "materials_stamp.json")
    want = {"prep": os.path.getmtime(prep), "code": _code(),
            "lanes": os.path.getmtime(os.path.join(bd, "lanes.json")),
            "cls": os.path.getmtime(os.path.join(bd, "markings.pkl"))}
    have = json.load(open(stamp)) if os.path.exists(stamp) else None
    outs = [os.path.join(cd, "asphalt_cut.npz")] + [os.path.join(tex, n + ".png") for n in OWN]
    if have == want and all(os.path.exists(p) for p in outs):
        print("[core_materials] cached")
        return
    dump = os.path.join(cd, "materials_dump.npz")
    with tempfile.TemporaryDirectory() as td:
        sp = os.path.join(td, "dump.py")
        open(sp, "w").write(DUMP)
        subprocess.run([os.environ.get("BLENDER", "blender"), "-b", "--disable-autoexec", "-P", sp, "--", prep, dump],
                       check=True, stdout=subprocess.DEVNULL)
    d = np.load(dump)
    bg = cut_asphalt(bd, d["asphalt"], os.path.join(cd, "asphalt_cut.npz"),
                     d["text_P"] if "text_P" in d.files else None)
    write_textures(tex, d, bg)
    json.dump(want, open(stamp, "w"))


def cut_asphalt(bd, T, out, text=None):
    """The model's asphalt cut into the generator's road surfaces. Under the
    止まれ quads one asphalt only, the one most of them lie on (estate: they
    are on the minor roads), which is also their texture's ground round the
    glyphs: any other tone under them showed the box. Returns its name.
    / 車道の材質分割（止まれの下は一種類: 文字の地と同じ）。その名前を返す"""
    import collections
    import pickle
    import shapely
    sys.path.insert(0, HERE)
    import surfaces_jp
    z = T[..., 2]
    if np.ptp(z) > 0.01:
        # not flat: the asphalt keeps its triangles (one material) / 平らでなければ分割しない
        print("[core_materials] model asphalt not flat (z %.3f..%.3f): not cut" % (z.min(), z.max()))
        np.savez_compressed(out, flat=False)
        return "asphalt_jp"
    zf = float(np.median(z))
    tri = shapely.polygons(T[:, :, :2].astype(np.float64))
    tri = tri[shapely.area(tri) > 1e-8]
    cw = shapely.set_precision(shapely.unary_union(tri), surfaces_jp.G)
    lanes = json.load(open(os.path.join(bd, "lanes.json")))
    x0, y0, x1, y1 = cw.bounds
    near = lambda l: any(x0 - 30 < q["c"][0] < x1 + 30 and y0 - 30 < q["c"][1] < y1 + 30 for q in l["pts"])

    class Stub:            # what partition reads of the Layout / partition が使う属性
        pass
    L = Stub()
    L.carriageway = cw
    L.lanes = [l for l in lanes["lanes"] if l["kind"] == "driving" and near(l)]
    L.approaches = lanes["approaches"]
    L.bus_bays = []
    M = Stub()
    M.cls = pickle.load(open(os.path.join(bd, "markings.pkl"), "rb"))["cls"]
    M.road_cls = collections.defaultdict(int)
    for (r, s), c in M.cls.items():
        M.road_cls[r] = max(M.road_cls[r], c)
    S = surfaces_jp.partition(L, M)
    if text is not None and len(text):
        box = shapely.set_precision(shapely.unary_union(shapely.polygons(text[:, :, :2].astype(np.float64)))
                                    .buffer(0.05, join_style="mitre"), surfaces_jp.G)
        under = {m: float(np.sum(shapely.area(shapely.intersection(g, box)))) for m, g in S.items() if len(g)}
        bg = max(under, key=under.get)
        moved = []
        for mat in list(S):
            if mat == bg or not len(S[mat]):
                continue
            moved += list(shapely.get_parts(shapely.intersection(S[mat], box, grid_size=surfaces_jp.G)))
            S[mat] = shapely.get_parts(shapely.difference(S[mat], box, grid_size=surfaces_jp.G))
        S[bg] = np.concatenate([S[bg], np.array(moved, dtype=object)])
        print("[core_materials] under the road words: %s" % ", ".join("%s %.0f m2" % kv for kv in sorted(under.items())))
    else:
        bg = "asphalt_jp"
        for mat in S:
            g = S[mat]
            S[mat] = g[(shapely.get_type_id(g) == 3) & (shapely.area(g) > 1e-6)] if len(g) else g
    V, I, K, names = [], [], [], []
    n = 0
    for mat, faces in S.items():
        if not len(faces):
            continue
        tris = shapely.get_parts(shapely.constrained_delaunay_triangles(np.asarray(faces, dtype=object)))
        if not len(tris):
            continue
        C = shapely.get_coordinates(shapely.get_exterior_ring(tris)).reshape(-1, 4, 2)[:, :3, :]
        a, b, c = C[:, 0], C[:, 1], C[:, 2]
        cross = (b[:, 0] - a[:, 0]) * (c[:, 1] - a[:, 1]) - (b[:, 1] - a[:, 1]) * (c[:, 0] - a[:, 0])
        keep = np.abs(cross) > 1e-10
        C = C[keep]
        flip = cross[keep] < 0
        C[flip] = C[flip][:, ::-1]                  # every triangle facing up / 上向き
        V.append(C.reshape(-1, 2))
        I.append(np.arange(n, n + 3 * len(C)).reshape(-1, 3))
        K.append(np.full(len(C), len(names), np.int32))
        names.append(mat)
        n += 3 * len(C)
        print("[core_materials] asphalt %-15s %7.0f m2, %6d triangles" % (mat, float(np.sum(shapely.area(faces))), len(C)))
    V = np.concatenate(V)
    np.savez_compressed(out, flat=True, V=np.column_stack([V, np.full(len(V), zf)]).astype(np.float64),
                        I=np.concatenate(I), K=np.concatenate(K), names=np.array(names))
    return bg


def write_textures(tex, d, bg="asphalt_jp"):
    """pave_plaza and road_text_jp, drawn like textures_jp (own random stream).
    / 広場の石畳と止まれのテクスチャ"""
    from PIL import Image
    sys.path.insert(0, HERE)
    import textures_jp as tj
    rng = np.random.default_rng(353)
    # granite setts 0.3 x 0.15 m in stretcher bond, mixed greys, one darker row
    # every 4.8 m (the north plaza: grey setts with darker bands, Street View)
    # 御影石の小舗石（灰色の濃淡、2.4 m ごとに濃い帯）
    n, tile, bw, bh = 1024, 4.8, 0.3, 0.15
    col, h, r = tj.blocks(n, rng, tile=tile, bw=bw, bh=bh, joint=0.005, chamfer=0.005, base=(150, 149, 146))
    yy, xx = np.mgrid[0:n, 0:n].astype(np.float64) * (tile / n)
    row = np.floor(yy / bh).astype(int)
    cl = np.floor((xx + (row % 2) * (bw / 2)) / bw).astype(int)
    nr, nc = int(round(tile / bh)), int(round(tile / bw))
    tone = rng.choice([0.86, 0.95, 1.0, 1.06, 1.14], size=(nr, nc), p=[0.15, 0.25, 0.25, 0.2, 0.15])
    tone[0, :] = 0.74                                  # the darker band / 濃い帯
    col *= tone[row % nr, cl % nc][..., None]
    speck = rng.random((n, n))
    col *= np.where(speck < 0.05, 0.72, np.where(speck > 0.97, 1.18, 1.0))[..., None]   # granite grain / 粒
    tj.save(tex, "pave_plaza", col, h, r, 2.0)
    # 止まれ: the model's glyphs as paint on asphalt at the asphalt's density
    # 止まれ: モデルの文字を塗料で、アスファルトと同じ密度で
    if "text_img" not in d.files:
        return
    P, UV = d["text_P"].astype(np.float64), d["text_UV"].astype(np.float64)
    mpu, mpv = [], []
    for p, t in zip(P, UV):
        A = np.column_stack([t[1] - t[0], t[2] - t[0]])        # uv edges / UV の辺
        if abs(np.linalg.det(A)) < 1e-9:
            continue
        J = np.column_stack([p[1, :2] - p[0, :2], p[2, :2] - p[0, :2]]) @ np.linalg.inv(A)
        mpu.append(np.linalg.norm(J[:, 0])); mpv.append(np.linalg.norm(J[:, 1]))
    mu, mv = float(np.median(mpu)), float(np.median(mpv))      # metres per 0..1 of u, v
    img = d["text_img"]
    mask = np.clip(img[..., 3] * img[..., :3].mean(-1), 0, 1)
    ppm = 1024 / TILE["asphalt_jp"]
    W, H = max(64, int(round(mu * ppm))), max(64, int(round(mv * ppm)))
    W, H = min(W, 2048), min(H, 2048)

    def cover(name, density, w, h, mode="RGB"):
        im = Image.open(os.path.join(tex, name)).convert(mode)
        a = np.asarray(im, np.float64)
        need_w, need_h = int(np.ceil(mu * density)), int(np.ceil(mv * density))
        a = np.tile(a, (need_h // a.shape[0] + 1, need_w // a.shape[1] + 1) + ((1,) if a.ndim == 3 else ()))
        a = a[:need_h, :need_w]
        return np.asarray(Image.fromarray(a.astype(np.uint8)).resize((w, h), Image.BILINEAR), np.float64)
    ground = cover(bg + ".png", ppm, W, H)
    paint = cover("line_white.png", 512.0, W, H)
    m = np.asarray(Image.fromarray((mask * 255).astype(np.uint8)).resize((W, H), Image.BILINEAR), np.float64) / 255
    col = ground * (1 - m[..., None]) + paint * m[..., None]
    rb = cover(bg + "_r.png", ppm, W, H, "L") / 255
    rp = cover("line_white_r.png", 512.0, W, H, "L") / 255
    rough = (rb * (1 - m) + rp * m) ** 2.2                    # saved sRGB-encoded / sRGB で保存済み
    hgt = 0.5 * m + 0.15 * (col.mean(-1) / 255)
    tj.save(tex, "road_text_jp", col, hgt, rough, 1.4)
    print("[core_materials] road_text_jp %dx%d px for %.2f x %.2f m text on %s" % (W, H, mu, mv, bg))


# ----------------------------------------------------------------- apply (Blender)
def _material(name, spec, tex_dir):
    """As blender_build.material(ue=True): the maps where Unreal's FBX import
    looks (roughness map -> Specular, metallic map -> Roughness, ch. 2).
    / UE 取り込み用の材質（blender_build.material と同じ配線）"""
    import bpy
    tex, col, rough, metal, smooth = spec
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    nt = m.node_tree
    bsdf = nt.nodes["Principled BSDF"]
    bsdf.inputs["Roughness"].default_value = rough
    bsdf.inputs["Metallic"].default_value = metal

    def img(fname, colour=True):
        p = os.path.join(tex_dir, fname)
        if not os.path.exists(p):
            return None
        im = bpy.data.images.load(p, check_existing=True)
        if not colour:
            im.colorspace_settings.name = "Non-Color"
        return im
    if tex:
        node = nt.nodes.new("ShaderNodeTexImage")
        node.image = img(tex + ".png")
        nt.links.new(node.outputs["Color"], bsdf.inputs["Base Color"])
    elif col:
        bsdf.inputs["Base Color"].default_value = (col[0] / 255.0, col[1] / 255.0, col[2] / 255.0, 1.0)
    for fn, sock, colour in ((name + "_r.png", "Specular IOR Level", False), (name + "_m.png", "Roughness", False)):
        im = img(fn, colour)
        if im is not None:
            n = nt.nodes.new("ShaderNodeTexImage")
            n.image = im
            nt.links.new(n.outputs["Color"], bsdf.inputs[sock])
    nim = img(tex + "_n.png", False) if tex else None
    if nim is not None:
        tn = nt.nodes.new("ShaderNodeTexImage")
        tn.image = nim
        nm = nt.nodes.new("ShaderNodeNormalMap")
        nt.links.new(tn.outputs["Color"], nm.inputs["Color"])
        nt.links.new(nm.outputs["Normal"], bsdf.inputs["Normal"])
    return m


def _tactile_uv(P, warn_only):
    """Per triangle: (is_guide, uv corners). A guide strip's triangle is thin
    (its minimum rectangle <= GUIDE_MAX_W across): u along it, v across from
    its edge (pave_guide: 4 bars along u); otherwise planar (pave_warn).
    / 点字ブロック: 細い三角形は線状（帯に沿う UV）、他は点状"""
    n = len(P)
    guide = np.zeros(n, bool)
    UV = P[:, :, :2] / TILE["pave_warn"]
    if warn_only:
        return guide, UV
    for k in range(n):
        q = P[k, :, :2]
        best = None
        for e in range(3):
            d = q[(e + 1) % 3] - q[e]
            ln = np.hypot(*d)
            if ln < 1e-6:
                continue
            ax = d / ln
            if ax[0] < -1e-9 or (abs(ax[0]) <= 1e-9 and ax[1] < 0):
                ax = -ax                       # one sense per direction / 向きを統一
            pr = np.array([-ax[1], ax[0]])
            a, b = q @ ax, q @ pr
            w = np.ptp(b)
            if best is None or w < best[0]:
                best = (w, ax, pr, a, b)
        if best is None:
            continue
        w, ax, pr, a, b = best
        if w <= GUIDE_MAX_W and np.ptp(a) > 1.5 * w:
            guide[k] = True
            UV[k] = np.column_stack([a, b - b.min()]) / TILE["pave_guide"]
    return guide, UV


def _drive_patch(bd):
    """core_merge's drive patch for the FBX: (cut region, fill, kerb faces, clip_tris),
    or None. Inside Blender: shapely from maps/.venv (same Python). / パッチ（Blender 内）"""
    pw, pn = os.path.join(bd, "core", "drive_patch.wkb"), os.path.join(bd, "core", "drive_patch.npz")
    if not (os.path.exists(pw) and os.path.exists(pn)):
        return None
    site = os.path.join(os.path.dirname(HERE), ".venv", "lib", "python%d.%d" % sys.version_info[:2], "site-packages")
    if site not in sys.path:
        sys.path.append(site)
    import shapely
    sys.path.insert(0, HERE)
    import core_patch
    meta = json.load(open(os.path.join(bd, "meta.json")))
    patch = shapely.from_wkb(open(pw, "rb").read())
    d = np.load(pn)
    return (core_patch.cut_region(bd, meta, patch), core_patch.cut_region(bd, meta, patch, lanes=False),
            d["fill"], d["faces"], core_patch.clip_tris)


def apply(objs, bd, log=print):
    """Re-material the model's ground objects (Blender, after the generated FBX
    import). / モデルの地面を生成都市の材質に"""
    import bpy
    tex_dir = os.path.join(bd, "textures")
    man = json.load(open(os.path.join(bd, "meshes", "manifest.json")))["materials"]
    cut_p = os.path.join(bd, "core", "asphalt_cut.npz")
    cut = np.load(cut_p) if os.path.exists(cut_p) else None
    made = {}

    def mat(name):
        if name in made:
            return made[name]
        m = bpy.data.materials.get(name)       # the generated FBX's own / 生成 FBX のもの
        if m is None or not m.users:
            spec = OWN.get(name) or man.get(name)
            assert spec, "no material spec for %s" % name
            m = _material(name, tuple(spec), tex_dir)
        made[name] = m
        return m
    stats = {}
    patch = _drive_patch(bd)
    for o in objs:
        rules = ROLE.get(o.name)
        if not rules or o.type != "MESH":
            continue
        me = o.data
        me.calc_loop_triangles()
        old = [m.name if m else "" for m in me.materials]
        co = np.zeros(len(me.vertices) * 3); me.vertices.foreach_get("co", co); co = co.reshape(-1, 3)
        nt = len(me.loop_triangles)
        tv = np.zeros(nt * 3, np.int32); me.loop_triangles.foreach_get("vertices", tv)
        tl = np.zeros(nt * 3, np.int32); me.loop_triangles.foreach_get("loops", tl)
        tm = np.zeros(nt, np.int32); me.loop_triangles.foreach_get("material_index", tm)
        tn = np.zeros(nt * 3); me.loop_triangles.foreach_get("normal", tn); tn = tn.reshape(-1, 3)
        uv = np.zeros(len(me.loops) * 2)
        if me.uv_layers:
            me.uv_layers[0].data.foreach_get("uv", uv)
        uv = uv.reshape(-1, 2)
        P = co[tv.reshape(-1, 3)]                     # (n, 3, 3)
        UV = uv[tl.reshape(-1, 3)]                    # (n, 3, 2)
        newmat = np.array([old[k] if k < len(old) else "" for k in tm], dtype=object)
        keep = np.ones(nt, bool)
        for src, rule in rules.items():
            s = (newmat == src) & keep
            if not s.any() or rule is None:
                continue
            if rule == "asphalt":
                if cut is not None and bool(cut["flat"]):
                    keep &= ~s                       # replaced by the cut pieces / 分割片に置換
                else:
                    newmat[s] = "asphalt_jp"
                    UV[s] = P[s][:, :, :2] / TILE["asphalt_jp"]
            elif rule == "kerb":
                top = s & (tn[:, 2] > 0.7)
                side = s & ~top
                newmat[top] = "kerb_jp"
                UV[top] = P[top][:, :, :2] / TILE["kerb_jp"]
                newmat[side] = "kerb_face_jp"
                t = np.column_stack([-tn[side, 1], tn[side, 0]])
                t /= np.maximum(np.linalg.norm(t, axis=1), 1e-9)[:, None]
                u = np.einsum("nkj,nj->nk", P[side][:, :, :2], t)
                UV[side] = np.stack([u, P[side][:, :, 2]], -1) / TILE["kerb_face_jp"]   # v 0 = road / 路面が v 0
            elif rule in ("tactile", "warn"):
                g, tuv = _tactile_uv(P[s], rule == "warn")
                idx = np.nonzero(s)[0]
                newmat[idx] = np.where(g, "pave_guide", "pave_warn")
                UV[idx] = tuv
            elif rule == "road_text_jp":
                newmat[s] = rule                     # the model's UVs map the glyphs / UV はそのまま
            else:
                newmat[s] = rule
                UV[s] = P[s][:, :, :2] / TILE[rule]
        P, UV, newmat = P[keep], UV[keep], newmat[keep]
        if "asphalt" in rules.values() and cut is not None and bool(cut["flat"]):
            CV, CI, CK, cn = cut["V"], cut["I"], cut["K"], [str(x) for x in cut["names"]]
            cp = CV[CI]
            P = np.concatenate([P, cp])
            UV = np.concatenate([UV, cp[:, :, :2] / TILE["asphalt_jp"]])
            newmat = np.concatenate([newmat, np.array([cn[k] for k in CK], dtype=object)])
        if patch is not None:
            # the drive patch (core_patch.py, as core_merge did to the npz): ground cut
            # where the lanes leave the asphalt, paved there, kerb faces on the new edge
            # 新しい車線の下の地面を切り、舗装し、縁石の面を付ける（npz と同じ）
            region, region_m, fill, faces, clip_tris = patch
            lift = float(o.matrix_world.translation.z)
            if o.name != "Road_Road_Roads":
                P, src, W = clip_tris(P, region_m if o.name == "Road_Marking_Markings" else region)
                UV = np.einsum("nvk,nkd->nvd", W, UV[src])
                newmat = newmat[src]
            else:
                # its raised car parks (M_lot, 0.15 m up: the npz files them as lots) are
                # cut like the lots; the flat asphalt stays / 一段高い駐車場（M_lot）は敷地と同じく切る
                hi = P[:, :, 2].max(axis=1) > 0.05
                if hi.any():
                    Ph, src, W = clip_tris(P[hi], region)
                    UVh = np.einsum("nvk,nkd->nvd", W, UV[hi][src])
                    P = np.concatenate([P[~hi], Ph]); UV = np.concatenate([UV[~hi], UVh])
                    newmat = np.concatenate([newmat[~hi], newmat[hi][src]])
            if o.name == "Road_Road_Roads" and len(fill):
                F = fill.copy(); F[:, :, 2] -= lift
                P = np.concatenate([P, F])
                UV = np.concatenate([UV, F[:, :, :2] / TILE["asphalt_jp"]])
                newmat = np.concatenate([newmat, np.array(["asphalt_jp"] * len(F), dtype=object)])
            if o.name == "Road_Curb_Kerb" and len(faces):
                F = faces.copy(); F[:, :, 2] -= lift
                nrm = np.cross(F[:, 1] - F[:, 0], F[:, 2] - F[:, 0])
                t = np.column_stack([-nrm[:, 1], nrm[:, 0]])
                t /= np.maximum(np.linalg.norm(t, axis=1), 1e-9)[:, None]
                u = np.einsum("nkj,nj->nk", F[:, :, :2], t)
                P = np.concatenate([P, F])
                UV = np.concatenate([UV, np.stack([u, F[:, :, 2]], -1) / TILE["kerb_face_jp"]])
                newmat = np.concatenate([newmat, np.array(["kerb_face_jp"] * len(F), dtype=object)])
        # rebuild the mesh: triangles, one slot per material / 三角形でメッシュを作り直す
        names = sorted(set(newmat))
        slot = {n: k for k, n in enumerate(names)}
        n = len(P)
        nm = bpy.data.meshes.new(o.data.name)
        nm.vertices.add(n * 3)
        nm.vertices.foreach_set("co", P.reshape(-1).astype(np.float32))
        nm.loops.add(n * 3)
        nm.loops.foreach_set("vertex_index", np.arange(n * 3, dtype=np.int32))
        nm.polygons.add(n)
        nm.polygons.foreach_set("loop_start", np.arange(0, n * 3, 3, dtype=np.int32))
        nm.polygons.foreach_set("loop_total", np.full(n, 3, np.int32))
        nm.polygons.foreach_set("material_index", np.array([slot[x] for x in newmat], np.int32))
        nm.polygons.foreach_set("use_smooth", np.zeros(n, bool))
        nm.uv_layers.new(name="UVMap").data.foreach_set("uv", UV.reshape(-1).astype(np.float32))
        for x in names:
            nm.materials.append(mat(x) if x in TILE or x in OWN else bpy.data.materials[x])
        nm.update()
        nm.validate(clean_customdata=False)
        oldme = o.data
        o.data = nm
        if oldme.users == 0:
            bpy.data.meshes.remove(oldme)
        nm.name = o.name
        cnt = {}
        for x in newmat:
            cnt[x] = cnt.get(x, 0) + 1
        stats[o.name] = cnt
    for k, v in stats.items():
        log("[core_fbx] core ground %s: %s" % (k, ", ".join("%s %d" % kv for kv in sorted(v.items()))))
    return stats


if __name__ == "__main__":
    bd = os.path.abspath(sys.argv[1])
    prepare(bd, os.path.join(bd, "core", "core_prepared.fbx"))
