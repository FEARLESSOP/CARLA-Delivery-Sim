"""Unreal Editor step (PythonScript commandlet): the lamp materials of the
Japanese pedestrian heads (osm_city/jp_hokousha.py) after the map import.

  cd <CARLA_SRC>/Unreal/CarlaUE4 && HOKOUSHA_PKG=Japan HOKOUSHA_MAP=OitaKosen \\
    UE4Editor $PWD/CarlaUE4.uproject -run=pythonscript -script=<this> -unattended -nosplash -nullrhi

  HOKOUSHA_TEST=<fbx>  offline test instead: builds everything under
                       /Game/ScratchHokousha in memory, imports the sample head
                       FBX there, checks it, saves NOTHING.

The heads are plain map meshes (Terrain_JpHokousha_<Map>_<NNN>, one per
signalised crossing). CARLA's TrafficLight has no pedestrian state and 4.26
Python cannot author blueprint graphs, so the rig drives the heads from the
client (hil_sim/hokousha.py) with World.apply_color_texture_to_objects(...,
MaterialParameter.Emissive, 1x1 texture). The texel is the head state:

  R = red standing figure lit       G = blue-green walking figure lit
  B = flash (walk figure blinks at 1 Hz in the material, no RPC per blink)
  A = countdown fraction (segment i of 8 lit while (i + 0.5) / 8 < A, on the
      lit lamp's column: lower cyan while walking, upper red while waiting)

Materials, /Game/<PKG>/Hokousha: M_JpHokoushaStop (upper lamp) / Walk (lower
lamp; an unlit figure shows faintly), M_JpHokoushaCount (the lit lamp's column),
M_JpHokoushaBody (grey), M_JpHokoushaFace (black), T_JpHokoushaIdle (default
texel: red figure, full red column = what the head shows before the rig
drives it). Slots by the FBX material names jph_*.

歩行者用灯器のランプ材質。テクスチャパラメータ "Emissive"（1x1）の各チャンネルで
状態を表す。実行時はクライアントが apply_color_texture_to_objects で切り替える。
"""

import os

import unreal

TEST_FBX = os.environ.get("HOKOUSHA_TEST")
PKG = os.environ.get("HOKOUSHA_PKG", "Japan")
NAME = os.environ.get("HOKOUSHA_MAP", "")
DEST = "/Game/ScratchHokousha" if TEST_FBX else "/Game/%s/Hokousha" % PKG
RED = (1.0, 0.03, 0.015)
CYAN = (0.0, 0.85, 0.72)
INTENSITY = 30.0
SEGS = 8

lib = unreal.EditorAssetLibrary
mel = unreal.MaterialEditingLibrary
tools = unreal.AssetToolsHelpers.get_asset_tools()
FAIL = []


def log(s):
    unreal.log_warning("[ue_jp_hokousha] " + str(s))


def check(name, ok, detail=""):
    log(("PASS " if ok else "FAIL ") + name + (" | " + str(detail) if detail else ""))
    if not ok:
        FAIL.append(name)


def save(asset):
    if not TEST_FBX:
        lib.save_loaded_asset(asset, only_if_is_dirty=False)


def idle_texture():
    """T_JpHokoushaIdle: a 4x4 texel block R=255 G=0 B=0 A=255 (stop, full red column)."""
    path = "%s/T_JpHokoushaIdle" % DEST
    if lib.does_asset_exist(path) and not TEST_FBX:
        return lib.load_asset(path)
    png = os.path.join(os.environ.get("TMPDIR", "/tmp"), "T_JpHokoushaIdle.png")
    # minimal PNG writer (no PIL in the editor's python) / エディタの python 用の簡易 PNG
    import struct
    import zlib
    raw = b"".join(b"\x00" + b"\xff\x00\x00\xff" * 4 for _ in range(4))
    def chunk(t, d):
        return struct.pack(">I", len(d)) + t + d + struct.pack(">I", zlib.crc32(t + d) & 0xffffffff)
    with open(png, "wb") as fh:
        fh.write(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", 4, 4, 8, 6, 0, 0, 0))
                 + chunk(b"IDAT", zlib.compress(raw)) + chunk(b"IEND", b""))
    task = unreal.AssetImportTask()
    task.set_editor_property("filename", png)
    task.set_editor_property("destination_path", DEST)
    task.set_editor_property("destination_name", "T_JpHokoushaIdle")
    task.set_editor_property("automated", True)
    task.set_editor_property("replace_existing", True)
    task.set_editor_property("save", False)
    tools.import_asset_tasks([task])
    tex = lib.load_asset(path)
    # uncompressed, sRGB like CARLA's transient textures, no filtering
    # 非圧縮・sRGB（CARLA の一時テクスチャと同じ）・フィルタなし
    tex.set_editor_property("compression_settings", unreal.TextureCompressionSettings.TC_EDITOR_ICON)
    tex.set_editor_property("srgb", True)
    tex.set_editor_property("filter", unreal.TextureFilter.TF_NEAREST)
    tex.set_editor_property("mip_gen_settings", unreal.TextureMipGenSettings.TMGS_NO_MIPMAPS)
    save(tex)
    return tex


def new_material(name):
    path = "%s/%s" % (DEST, name)
    m = lib.load_asset(path) if lib.does_asset_exist(path) else \
        tools.create_asset(name, DEST, unreal.Material, unreal.MaterialFactoryNew())
    mel.delete_all_material_expressions(m)
    return m


def expr(m, cls, x, y, **props):
    e = mel.create_material_expression(m, cls, x, y)
    for k, v in props.items():
        e.set_editor_property(k, v)
    return e


def link(a, out, b, inp):
    if not mel.connect_material_expressions(a, out, b, inp):
        raise RuntimeError("cannot connect %s.%s -> %s.%s" % (a.get_name(), out, b.get_name(), inp))


def mul(m, a, a_out, b, b_out, x, y):
    e = expr(m, unreal.MaterialExpressionMultiply, x, y)
    link(a, a_out, e, "A")
    link(b, b_out, e, "B")
    return e


def colour(m, rgb, x, y):
    return expr(m, unreal.MaterialExpressionConstant3Vector, x, y,
                constant=unreal.LinearColor(rgb[0], rgb[1], rgb[2], 1.0))


def state_sample(m, tex):
    uv = expr(m, unreal.MaterialExpressionConstant2Vector, -1400, 0, r=0.5, g=0.5)
    t = expr(m, unreal.MaterialExpressionTextureSampleParameter2D, -1200, 0,
             parameter_name="Emissive", texture=tex, sampler_type=unreal.MaterialSamplerType.SAMPLERTYPE_COLOR)
    link(uv, "", t, "UVs")
    return t


def figure_material(name, tex, rgb, dim, flashing):
    """Figure: lit = channel (and the 1 Hz blink when B is set); unlit it shows
    faintly in `dim`. / 人形: 点灯はチャンネル値、消灯時は暗く見える"""
    m = new_material(name)
    t = state_sample(m, tex)
    ch = "G" if flashing else "R"
    on, on_out = t, ch
    if flashing:
        # off half of each second while B = 1: on = G * (1 - B * floor(frac(time) * 2))
        # 点滅: B=1 の間は毎秒後半を消灯
        tm = expr(m, unreal.MaterialExpressionTime, -1200, 400)
        fr = expr(m, unreal.MaterialExpressionFrac, -1000, 400)
        link(tm, "", fr, "")
        two = expr(m, unreal.MaterialExpressionConstant, -1000, 520, r=2.0)
        x2 = mul(m, fr, "", two, "", -850, 400)
        fl = expr(m, unreal.MaterialExpressionFloor, -700, 400)
        link(x2, "", fl, "")
        bf = mul(m, t, "B", fl, "", -550, 300)
        inv = expr(m, unreal.MaterialExpressionOneMinus, -400, 300)
        link(bf, "", inv, "")
        on = mul(m, t, "G", inv, "", -250, 200)
        on_out = ""
    k = expr(m, unreal.MaterialExpressionConstant, -400, -200, r=INTENSITY)
    c = colour(m, rgb, -400, -320)
    ck = mul(m, c, "", k, "", -250, -250)
    em = mul(m, ck, "", on, on_out, -100, -150)
    mel.connect_material_property(em, "", unreal.MaterialProperty.MP_EMISSIVE_COLOR)
    base = colour(m, dim, -100, -400)
    mel.connect_material_property(base, "", unreal.MaterialProperty.MP_BASE_COLOR)
    mel.recompile_material(m)
    save(m)
    return m


def count_material(tex):
    """Segments of the lit lamp: lit = saturate(ceil(A - v)) * active, active =
    G on the lower (walk) column (u < 0.5), R on the upper (stop) one; cyan /
    red by column. / 点灯中の灯の区画のみ"""
    m = new_material("M_JpHokoushaCount")
    t = state_sample(m, tex)
    tc = expr(m, unreal.MaterialExpressionTextureCoordinate, -1200, 400)
    v = expr(m, unreal.MaterialExpressionComponentMask, -1000, 400, r=False, g=True, b=False, a=False)
    link(tc, "", v, "")
    d = expr(m, unreal.MaterialExpressionSubtract, -850, 300)
    link(t, "A", d, "A")
    link(v, "", d, "B")
    ce = expr(m, unreal.MaterialExpressionCeil, -700, 300)
    link(d, "", ce, "")
    s1 = expr(m, unreal.MaterialExpressionSaturate, -550, 300)
    link(ce, "", s1, "")
    # upper = round(u): 0 lower column, 1 upper / 上の列 = round(u)
    u = expr(m, unreal.MaterialExpressionComponentMask, -1000, 150, r=True, g=False, b=False, a=False)
    link(tc, "", u, "")
    up = expr(m, unreal.MaterialExpressionRound, -850, 150)
    link(u, "", up, "")
    act = expr(m, unreal.MaterialExpressionLinearInterpolate, -700, 150)
    link(t, "G", act, "A")
    link(t, "R", act, "B")
    link(up, "", act, "Alpha")
    lit = mul(m, s1, "", act, "", -400, 200)
    lp = expr(m, unreal.MaterialExpressionLinearInterpolate, -400, -100)
    link(colour(m, CYAN, -600, -200), "", lp, "A")
    link(colour(m, RED, -600, -80), "", lp, "B")
    link(up, "", lp, "Alpha")
    k = expr(m, unreal.MaterialExpressionConstant, -400, -300, r=INTENSITY)
    ck = mul(m, lp, "", k, "", -250, -200)
    em = mul(m, ck, "", lit, "", -100, -100)
    mel.connect_material_property(em, "", unreal.MaterialProperty.MP_EMISSIVE_COLOR)
    mel.connect_material_property(colour(m, (0.02, 0.02, 0.02), -100, -400), "", unreal.MaterialProperty.MP_BASE_COLOR)
    mel.recompile_material(m)
    save(m)
    return m


def plain(name, rgb, rough, metal):
    m = new_material(name)
    mel.connect_material_property(colour(m, rgb, -400, 0), "", unreal.MaterialProperty.MP_BASE_COLOR)
    for val, prop, y in ((rough, unreal.MaterialProperty.MP_ROUGHNESS, 200), (metal, unreal.MaterialProperty.MP_METALLIC, 300)):
        mel.connect_material_property(expr(m, unreal.MaterialExpressionConstant, -400, y, r=val), "", prop)
    mel.recompile_material(m)
    save(m)
    return m


def build_materials():
    tex = idle_texture()
    mats = {"jph_stop": figure_material("M_JpHokoushaStop", tex, RED, (0.05, 0.008, 0.006), False),
            "jph_walk": figure_material("M_JpHokoushaWalk", tex, CYAN, (0.006, 0.035, 0.032), True),
            "jph_count": count_material(tex),
            "jph_body": plain("M_JpHokoushaBody", (0.30, 0.31, 0.32), 0.5, 0.3),
            "jph_face": plain("M_JpHokoushaFace", (0.008, 0.008, 0.008), 0.4, 0.0)}
    mats["jph_pole"] = mats["jph_body"]
    return tex, mats


def assign(mesh, mats):
    n = 0
    for i, s in enumerate(mesh.static_materials):
        slot = str(s.material_slot_name)
        key = next((k for k in mats if k in slot), None)
        if key is not None:
            mesh.set_material(i, mats[key])
            n += 1
    save(mesh)
    return n


def import_fbx(path):
    ui = unreal.FbxImportUI()
    ui.set_editor_property("import_mesh", True)
    ui.set_editor_property("import_as_skeletal", False)
    ui.set_editor_property("mesh_type_to_import", unreal.FBXImportType.FBXIT_STATIC_MESH)
    ui.set_editor_property("import_materials", False)
    ui.set_editor_property("import_textures", False)
    task = unreal.AssetImportTask()
    sm = ui.get_editor_property("static_mesh_import_data")
    sm.set_editor_property("combine_meshes", True)
    base = os.path.splitext(os.path.basename(path))[0]
    task.set_editor_property("filename", path)
    task.set_editor_property("destination_path", DEST)
    task.set_editor_property("destination_name", base)
    task.set_editor_property("automated", True)
    task.set_editor_property("replace_existing", True)
    task.set_editor_property("save", False)
    task.set_editor_property("options", ui)
    tools.import_asset_tasks([task])
    got = list(task.get_editor_property("imported_object_paths") or [])
    log("imported %s" % got)
    for p in got:
        a = unreal.load_object(None, p)
        if isinstance(a, unreal.StaticMesh):
            return a
    return None


def main():
    tex, mats = build_materials()
    if TEST_FBX:
        for k, m in mats.items():
            names = []
            try:
                names = [str(p) for p in mel.get_texture_parameter_names(m)]
            except Exception as exc:      # older API / 古い API
                names = ["?" + str(exc)]
            if k in ("jph_stop", "jph_walk", "jph_count"):
                check("%s has the Emissive texture parameter" % m.get_name(), "Emissive" in names, names)
                check("%s blend" % m.get_name(), True, m.get_editor_property("blend_mode"))
        check("lamp materials opaque (unlit figure faintly visible)",
              all(mats[k].get_editor_property("blend_mode") == unreal.BlendMode.BLEND_OPAQUE
                  for k in ("jph_stop", "jph_walk", "jph_count")))
        check("idle texel uncompressed sRGB", tex.get_editor_property("srgb") and
              tex.get_editor_property("compression_settings") == unreal.TextureCompressionSettings.TC_EDITOR_ICON)
        mesh = import_fbx(TEST_FBX)
        check("sample head imported", isinstance(mesh, unreal.StaticMesh), mesh)
        if isinstance(mesh, unreal.StaticMesh):
            slots = [str(s.material_slot_name) for s in mesh.static_materials]
            n = assign(mesh, mats)
            check("all 6 slots got our materials", n == 6 and len(slots) == 6, slots)
            got = {str(s.material_slot_name): mesh.get_material(i).get_name() for i, s in enumerate(mesh.static_materials)}
            want = {"jph_stop": "Stop", "jph_walk": "Walk", "jph_count": "Count", "jph_face": "Face",
                    "jph_body": "Body", "jph_pole": "Body"}
            check("slot -> material", all(any(k in sl and v.endswith(want[k]) for sl, v in got.items()) for k in want), got)
            check("name has no sign/light", not any(w in mesh.get_name().lower() for w in ("sign", "light")))
        log("test %s: %d failures %s (nothing saved)" % ("PASSED" if not FAIL else "FAILED", len(FAIL), FAIL))
        return
    root = "/Game/%s/Static" % PKG
    assets = [p for p in lib.list_assets(root, recursive=True, include_folder=False)
              if "JpHokousha" in p.split("/")[-1] and NAME in p.split("/")[-1]]
    n = 0
    for p in assets:
        sm = lib.load_asset(p)
        if isinstance(sm, unreal.StaticMesh) and assign(sm, mats) > 0:
            n += 1
    for p in lib.list_assets(DEST, recursive=False):
        lib.save_asset(p, only_if_is_dirty=False)
    log("%d pedestrian head meshes on the lamp materials (%s)" % (n, DEST))
    if n == 0:
        raise RuntimeError("no *JpHokousha* meshes for %s under %s" % (NAME, root))


main()
