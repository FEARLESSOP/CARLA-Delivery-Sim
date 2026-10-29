"""X-ray copies of the LastMile pod's shell (UE4 editor Python).

The HTML's X-ray mode fades the shell materials to 14 % opacity. CARLA can't
change a material at runtime, so the shell props get see-through twins:
LMPShellX / LMPShellStreetX are copies of LMPShell / LMPShellStreet whose
shell slots use M_LMP_Xray (translucent); the other slots stay as they are,
like the HTML. They are added to the package's props list, so CARLA offers
them as static.prop.lmpshellx / static.prop.lmpshellstreetx, and the delivery
code swaps the solid and X-ray shells.

Run after Import.py and before PrepareAssetsForCooking (build_lastmile.sh):
  UE4Editor CarlaUE4.uproject -run=pythonscript -script=make_xray.py

LastMile ポッド外殻の X 線用コピー（UE4 エディタ Python）。外殻スロットを半透明
マテリアルにした複製を作り、プロップ一覧に追加する。
"""

import json
import os

import unreal

PKG = os.environ.get("LMP_PKG", "LastMileModels")
HERE = os.path.dirname(os.path.abspath(__file__))
STATIC = "/Game/%s/Static/Car" % PKG
MAT_DIR = "/Game/%s/Materials" % PKG
SHELL_MATS = ("M_Standard_e4e8ec", "M_Standard_c9cfd5", "M_Printed_1")
OPACITY, TINT = 0.14, (0.86, 0.9, 0.94)
EAL, MEL = unreal.EditorAssetLibrary, unreal.MaterialEditingLibrary
LOG = open(os.path.join(HERE, "make_xray.log"), "w")


def log(*a):
    msg = " ".join(str(x) for x in a)
    LOG.write(msg + "\n")
    LOG.flush()
    unreal.log("make_xray: " + msg)


def xray_material():
    path = MAT_DIR + "/M_LMP_Xray"
    if EAL.does_asset_exist(path):
        EAL.delete_asset(path)
    mat = unreal.AssetToolsHelpers.get_asset_tools().create_asset(
        "M_LMP_Xray", MAT_DIR, unreal.Material, unreal.MaterialFactoryNew())
    mat.set_editor_property("blend_mode", unreal.BlendMode.BLEND_TRANSLUCENT)
    mat.set_editor_property("two_sided", True)
    col = MEL.create_material_expression(mat, unreal.MaterialExpressionConstant3Vector, -300, 0)
    col.set_editor_property("constant", unreal.LinearColor(TINT[0], TINT[1], TINT[2], 1.0))
    op = MEL.create_material_expression(mat, unreal.MaterialExpressionConstant, -300, 200)
    op.set_editor_property("r", OPACITY)
    MEL.connect_material_property(col, "", unreal.MaterialProperty.MP_BASE_COLOR)
    MEL.connect_material_property(op, "", unreal.MaterialProperty.MP_OPACITY)
    MEL.recompile_material(mat)
    EAL.save_loaded_asset(mat)
    log("material", path)
    return mat


def xray_copy(name, mat):
    src = "%s/%s/%s_%s" % (STATIC, name, name, name)
    dst_dir = "%s/%sX" % (STATIC, name)
    dst = "%s/%sX" % (dst_dir, name)
    if not EAL.does_asset_exist(src):
        raise RuntimeError("missing " + src)
    if EAL.does_directory_exist(dst_dir):
        EAL.delete_directory(dst_dir)
    if not EAL.duplicate_asset(src, dst):
        raise RuntimeError("could not duplicate " + src)
    mesh = EAL.load_asset(dst)
    # struct array elements come back as copies: rebuild the list and set it
    # 構造体配列の要素はコピーなので、新しいリストを作って設定する
    slots, swapped = [], 0
    for s in mesh.get_editor_property("static_materials"):
        m = s.get_editor_property("material_interface")
        n = s.get_editor_property("material_slot_name")
        name_of = "%s %s" % (n, m.get_name() if m else "")
        new = unreal.StaticMaterial()
        new.set_editor_property("material_slot_name", n)
        if any(k in name_of for k in SHELL_MATS):
            new.set_editor_property("material_interface", mat)
            swapped += 1
        else:
            new.set_editor_property("material_interface", m)
        slots.append(new)
    mesh.set_editor_property("static_materials", slots)
    EAL.save_loaded_asset(mesh)
    log("copy", dst, "slots", len(slots), "made see-through", swapped)
    if not swapped:
        raise RuntimeError("no shell slots found in " + src)
    return "%s.%sX" % (dst, name)


def add_props(paths):
    cfg = unreal.Paths.project_content_dir() + "%s/Config/%s.Package.json" % (PKG, PKG)
    with open(cfg) as f:
        data = json.load(f)
    props = [p for p in data["props"] if p["name"] not in paths]
    for name, path in paths.items():
        props.append({"name": name, "path": path, "size": "big"})
    data["props"] = props
    with open(cfg, "w") as f:
        json.dump(data, f, indent=4)
    log("props list now", len(props))


try:
    mat = xray_material()
    add_props({n + "X": xray_copy(n, mat) for n in ("LMPShell", "LMPShellStreet")})
    log("OK")
except Exception as e:
    import traceback
    log("FAILED\n" + traceback.format_exc())
LOG.close()
