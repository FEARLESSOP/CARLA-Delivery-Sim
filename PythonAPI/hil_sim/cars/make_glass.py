"""Invisible window glass for one of your cars (UE4 editor Python).

Downloaded models often bring their glass as an opaque material (black from
inside, white from outside). This gives the car's window material slots --
the material names starting with an entry under "glass" in cars.json -- a material that is
not drawn at all (masked, opacity 0), so the windows are fully clear from
inside and out. Run after build_vehicle.py by build_cars.sh, with
CAR_DIR (the car's folder, holding its skeletal mesh) and CAR_GLASS
(comma-separated names).

自作車両の窓ガラスを完全に透明にする（UE4 エディタ Python）。
"""

import os
import traceback

import unreal

PKG = os.environ.get("LMP_PKG", "UserCars")
HERE = os.path.dirname(os.path.abspath(__file__))
MAT_DIR = "/Game/%s/Materials" % PKG
CAR_DIR = os.environ["CAR_DIR"]
GLASS = [g.strip().lower() for g in os.environ.get("CAR_GLASS", "").split(",") if g.strip()]
EAL, MEL = unreal.EditorAssetLibrary, unreal.MaterialEditingLibrary
LOG = open(os.path.join(HERE, os.environ.get("CAR_GLASS_LOG", "make_glass.log")), "w")


def log(*a):
    msg = " ".join(str(x) for x in a)
    LOG.write(msg + "\n")
    LOG.flush()
    unreal.log("make_glass: " + msg)


def glass_material():
    path = MAT_DIR + "/M_CarGlassClear"
    if EAL.does_asset_exist(path):
        return EAL.load_asset(path)
    mat = unreal.AssetToolsHelpers.get_asset_tools().create_asset(
        "M_CarGlassClear", MAT_DIR, unreal.Material, unreal.MaterialFactoryNew())
    # masked with mask 0: every pixel is clipped, nothing is drawn
    # マスク 0 のマスクドマテリアル：何も描画されない
    mat.set_editor_property("blend_mode", unreal.BlendMode.BLEND_MASKED)
    mat.set_editor_property("two_sided", True)
    # it goes on a vehicle (skeletal) mesh: without this flag the cooked game
    # draws the grey checker default material instead
    # 車両（スケルタル）メッシュ用。フラグが無いと市松模様の既定マテリアルになる
    mat.set_editor_property("used_with_skeletal_mesh", True)
    mask = MEL.create_material_expression(mat, unreal.MaterialExpressionConstant, -300, 200)
    mask.set_editor_property("r", 0.0)
    MEL.connect_material_property(mask, "", unreal.MaterialProperty.MP_OPACITY_MASK)
    MEL.recompile_material(mat)
    EAL.save_loaded_asset(mat)
    log("material", path)
    return mat


try:
    # the import names the mesh SK_<Name>_<FBX object>: find it / メッシュを探す
    meshes = [EAL.load_asset(p) for p in EAL.list_assets(CAR_DIR, recursive=False)
              if EAL.find_asset_data(p).asset_class == "SkeletalMesh"]
    if len(meshes) != 1:
        raise RuntimeError("expected one skeletal mesh in %s, got %d" % (CAR_DIR, len(meshes)))
    mesh = meshes[0]
    log("mesh", mesh.get_path_name())
    glass, done = glass_material(), []
    slots = []
    for s in mesh.get_editor_property("materials"):
        name = str(s.get_editor_property("material_slot_name"))
        # names must START with an entry: "glass1" is the windows, not
        # "taillightglass1" / 先頭一致（ランプのガラスは対象外）
        if any(name.lower().startswith(g) for g in GLASS):
            # the slot comes back as a copy: swap its material, keep its names
            # スロットはコピーなので、マテリアルだけ差し替える
            s.set_editor_property("material_interface", glass)
            done.append(name)
        slots.append(s)
    mesh.set_editor_property("materials", slots)
    EAL.save_loaded_asset(mesh)
    log("glass slots", done)
    if GLASS and not done:
        raise RuntimeError("no slot matched %s in %s" % (GLASS, [
            str(s.get_editor_property("material_slot_name")) for s in mesh.get_editor_property("materials")]))
    # Ensure vehicle materials are two-sided to prevent see-through faces on open-wheel / custom geometry
    # オープンホイールや特殊形状で裏面が透過しないよう、車両マテリアルを両面描画（two-sided）に設定
    for p in EAL.list_assets(CAR_DIR, recursive=True):
        if EAL.find_asset_data(p).asset_class == "Material":
            m = EAL.load_asset(p)
            if m and not m.get_editor_property("two_sided"):
                m.set_editor_property("two_sided", True)
                MEL.recompile_material(m)
                EAL.save_loaded_asset(m)
                log("two-sided material", p)
    log("OK")
except Exception:
    log("FAILED\n" + traceback.format_exc())
LOG.close()
