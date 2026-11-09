"""Rebuild the LastMile Hub's materials from the glTF PBR spec (UE4 editor Python).

UE's FBX import keeps only the base colour (texture or constant) and scrambles
the rest (map guide ch. 2). prepare_lastmilehub.py wrote lmh_materials.json
with each material's base colour/texture, metallic, roughness, emissive and
blend mode; this script rewrites every material of the package from it:

  opaque       base (texture x tint), metallic, roughness
  masked       + opacity mask = texture alpha (labels with cut-outs)
  translucent  glass: tinted, opacity 0.26, two-sided
  emissive     LEDs, light strips, lamps: emissive = colour x LMH_GLOW
               (CARLA's sun needs ~1e4..1e5; its own blinkers use ~2.5e5)

All get "used with skeletal mesh" (without it the cooked game draws the grey
checker on the vehicle). Run after build_vehicle.py, before the cook:
  LMH_PKG=LastMileHubModels LMH_SPEC=.../lmh_materials.json \
  UE4Editor CarlaUE4.uproject -run=pythonscript -script=lmh_materials.py

LastMile Hub のマテリアルを glTF の PBR 仕様から作り直す（UE4 エディタ Python）。
"""

import json
import os
import traceback

import unreal

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.environ.get("LMH_PKG", "LastMileHubModels")
SPEC = json.load(open(os.environ["LMH_SPEC"]))
GLOW = float(os.environ.get("LMH_GLOW", "20000"))
TWO_SIDED = ("M_pod_flag",)            # a single triangle / 片面 1 枚の旗
EAL, MEL = unreal.EditorAssetLibrary, unreal.MaterialEditingLibrary
LOG = open(os.path.join(HERE, os.environ.get("LMH_LOG", "lmh_materials.log")), "w")
BLEND = {"opaque": unreal.BlendMode.BLEND_OPAQUE, "masked": unreal.BlendMode.BLEND_MASKED,
         "translucent": unreal.BlendMode.BLEND_TRANSLUCENT}


def log(*a):
    msg = " ".join(str(x) for x in a)
    LOG.write(msg + "\n")
    LOG.flush()
    unreal.log("lmh_materials: " + msg)


def const3(mat, rgb, x, y):
    e = MEL.create_material_expression(mat, unreal.MaterialExpressionConstant3Vector, x, y)
    e.set_editor_property("constant", unreal.LinearColor(rgb[0], rgb[1], rgb[2], 1.0))
    return e


def const1(mat, v, x, y):
    e = MEL.create_material_expression(mat, unreal.MaterialExpressionConstant, x, y)
    e.set_editor_property("r", v)
    return e


def multiply(mat, a, a_out, b, x, y):
    m = MEL.create_material_expression(mat, unreal.MaterialExpressionMultiply, x, y)
    MEL.connect_material_expressions(a, a_out, m, "A")
    MEL.connect_material_expressions(b, "", m, "B")
    return m


def rebuild(mat, spec, tex):
    MEL.delete_all_material_expressions(mat)
    mode = spec["mode"]
    mat.set_editor_property("blend_mode", BLEND[mode])
    mat.set_editor_property("shading_model", unreal.MaterialShadingModel.MSM_DEFAULT_LIT)
    mat.set_editor_property("two_sided", mode != "opaque" or mat.get_name() in TWO_SIDED)
    mat.set_editor_property("used_with_skeletal_mesh", True)
    base = spec["base"]
    white = all(abs(c - 1.0) < 1e-3 for c in base)
    ts = None
    if tex is not None:
        ts = MEL.create_material_expression(mat, unreal.MaterialExpressionTextureSample, -700, -200)
        ts.set_editor_property("texture", tex)
        if white:
            MEL.connect_material_property(ts, "RGB", unreal.MaterialProperty.MP_BASE_COLOR)
        else:
            m = multiply(mat, ts, "RGB", const3(mat, base, -700, 0), -400, -200)
            MEL.connect_material_property(m, "", unreal.MaterialProperty.MP_BASE_COLOR)
        if mode == "masked":
            MEL.connect_material_property(ts, "A", unreal.MaterialProperty.MP_OPACITY_MASK)
    else:
        MEL.connect_material_property(const3(mat, base, -400, -200), "", unreal.MaterialProperty.MP_BASE_COLOR)
    MEL.connect_material_property(const1(mat, spec["metallic"], -400, 0), "", unreal.MaterialProperty.MP_METALLIC)
    MEL.connect_material_property(const1(mat, spec["roughness"], -400, 100), "", unreal.MaterialProperty.MP_ROUGHNESS)
    if mode == "translucent":
        MEL.connect_material_property(const1(mat, spec["opacity"], -400, 200), "", unreal.MaterialProperty.MP_OPACITY)
    em = spec.get("emissive")
    if em:
        MEL.connect_material_property(const3(mat, [c * GLOW for c in em], -400, 300), "",
                                      unreal.MaterialProperty.MP_EMISSIVE_COLOR)
    elif ts is not None and spec.get("printed") and not white:
        # tinted screens (pod eyes, hub display) glow a little / 画面は少し発光
        m = multiply(mat, ts, "RGB", const3(mat, [c * GLOW * 0.05 for c in base], -700, 300), -400, 300)
        MEL.connect_material_property(m, "", unreal.MaterialProperty.MP_EMISSIVE_COLOR)
    MEL.recompile_material(mat)
    EAL.save_loaded_asset(mat)


def main():
    root = "/Game/%s" % PKG
    assets = EAL.list_assets(root, recursive=True, include_folder=False)
    textures, materials = {}, []
    for a in assets:
        cls = str(EAL.find_asset_data(a).asset_class)
        name = a.split("/")[-1].split(".")[0]
        if cls == "Texture2D":
            textures.setdefault(name, []).append(a)
        elif cls == "Material":
            materials.append(a)
    log("materials", len(materials), "textures", sum(len(v) for v in textures.values()))
    done, missing_spec, missing_tex = 0, [], []
    for a in materials:
        name = a.split("/")[-1].split(".")[0]
        if name.startswith("M_LMP_"):
            continue
        spec = SPEC.get(name)
        if spec is None:
            missing_spec.append(a)
            continue
        tex = None
        if spec.get("texture"):
            folder = a.rsplit("/", 1)[0]
            cands = textures.get(spec["texture"], [])
            pick = [c for c in cands if c.rsplit("/", 1)[0] == folder] or cands
            if pick:
                tex = EAL.load_asset(pick[0])
            else:
                missing_tex.append(a)
        rebuild(EAL.load_asset(a), spec, tex)
        done += 1
    log("rebuilt", done)
    if missing_spec:
        log("no spec for", missing_spec)
    if missing_tex:
        log("texture not found for", missing_tex)
    if missing_tex or done == 0:
        raise RuntimeError("materials incomplete")


try:
    main()
    log("OK")
except Exception:
    log("FAILED\n" + traceback.format_exc())
LOG.close()
