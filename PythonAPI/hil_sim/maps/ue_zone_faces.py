"""See-through faces for the road overlay's zone boxes (UE4 editor Python).

CARLA's debug drawing has lines only, so the overlay fills a zone box's sides
and roof with these panels: /Game/ZoneTint/ZoneTint_<colour>, a copy of the
flat panel with a two-sided, unlit, translucent
material in the zone's own colour and no collision. The panels are 1 m x k m (k = 1, 2, 4 .. 32,
ZoneTint_<colour>_<k>, long side on X, centred, normal +Z), because
static.prop.mesh only takes a uniform scale; road_overlay.py tiles each face
with them.

  UE4Editor CarlaUE4.uproject -run=pythonscript -script=ue_zone_faces.py
  (build_zone_faces.sh runs it, then cooks and installs /Game/ZoneTint)

ゾーン箱の半透明の面（UE4 エディタ Python）。デバッグ描画は線だけなので、
ゾーン色・両面・アンリット・半透明・衝突なしの 1 m 板を作る。
"""

import os

import unreal

DIR = "/Game/ZoneTint"
# the colour is divided by the camera's exposure (EyeAdaptation), so it looks the
# same at noon and at night; a fixed emissive vanished at noon and read as black
# 色を露出で割り、昼も夜も同じ見え方にする（固定値だと昼に消えて黒く見えた）
OPACITY = 0.3
ASPECTS = (1, 2, 4, 8, 16, 32)
TMP = os.path.join(os.path.dirname(os.path.abspath(__file__)), "build", "zone_faces")
# same hues as road_overlay.py (no-fly red + POD_HUES) / road_overlay.py と同じ色
COLOURS = {
    "Red": (1.0, 0.0, 0.0),
    "Cyan": (0.0, 1.0, 1.0),
    "Yellow": (1.0, 1.0, 0.0),
    "Magenta": (1.0, 0.0, 1.0),
    "Lime": (2.0 / 3, 1.0, 0.0),
    "Orange": (1.0, 2.0 / 3, 0.0),
    "Sky": (1.0 / 3, 2.0 / 3, 1.0),
    "Mint": (0.0, 1.0, 2.0 / 3),
    "Violet": (2.0 / 3, 1.0 / 3, 1.0),
}
EAL, MEL = unreal.EditorAssetLibrary, unreal.MaterialEditingLibrary
LOG = open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "ue_zone_faces.log"), "w")


def log(*a):
    LOG.write(" ".join(str(x) for x in a) + "\n")
    LOG.flush()


def material(name, rgb):
    path = "%s/M_ZoneTint_%s" % (DIR, name)
    if EAL.does_asset_exist(path):
        EAL.delete_asset(path)
    mat = unreal.AssetToolsHelpers.get_asset_tools().create_asset(
        "M_ZoneTint_" + name, DIR, unreal.Material, unreal.MaterialFactoryNew())
    mat.set_editor_property("blend_mode", unreal.BlendMode.BLEND_TRANSLUCENT)
    mat.set_editor_property("shading_model", unreal.MaterialShadingModel.MSM_UNLIT)
    mat.set_editor_property("two_sided", True)
    col = MEL.create_material_expression(mat, unreal.MaterialExpressionConstant3Vector, -300, 0)
    col.set_editor_property("constant", unreal.LinearColor(rgb[0], rgb[1], rgb[2], 1.0))
    eye = MEL.create_material_expression(mat, unreal.MaterialExpressionEyeAdaptation, -300, 100)
    div = MEL.create_material_expression(mat, unreal.MaterialExpressionDivide, -100, 50)
    MEL.connect_material_expressions(col, "", div, "A")
    MEL.connect_material_expressions(eye, "", div, "B")
    op = MEL.create_material_expression(mat, unreal.MaterialExpressionConstant, -300, 200)
    op.set_editor_property("r", OPACITY)
    MEL.connect_material_property(div, "", unreal.MaterialProperty.MP_EMISSIVE_COLOR)
    MEL.connect_material_property(op, "", unreal.MaterialProperty.MP_OPACITY)
    MEL.recompile_material(mat)
    EAL.save_loaded_asset(mat)
    return mat


def face(name, k, mat):
    path = "%s/ZoneTint_%s_%d" % (DIR, name, k)
    if EAL.does_asset_exist(path):
        EAL.delete_asset(path)
    task = unreal.AssetImportTask()
    task.set_editor_property("filename", os.path.join(TMP, "ZoneFacePanel_%d.fbx" % k))
    task.set_editor_property("destination_path", DIR)
    task.set_editor_property("destination_name", "ZoneTint_%s_%d" % (name, k))
    task.set_editor_property("automated", True)
    task.set_editor_property("replace_existing", True)
    task.set_editor_property("save", False)
    opts = unreal.FbxImportUI()
    opts.set_editor_property("import_materials", False)
    opts.set_editor_property("import_textures", False)
    opts.set_editor_property("import_mesh", True)
    opts.set_editor_property("import_as_skeletal", False)
    opts.set_editor_property("mesh_type_to_import", unreal.FBXImportType.FBXIT_STATIC_MESH)
    opts.static_mesh_import_data.set_editor_property("combine_meshes", True)
    opts.static_mesh_import_data.set_editor_property("auto_generate_collision", False)
    task.set_editor_property("options", opts)
    unreal.AssetToolsHelpers.get_asset_tools().import_asset_tasks([task])
    mesh = EAL.load_asset(path)
    if mesh is None:
        raise RuntimeError("import failed: " + path)
    slot = unreal.StaticMaterial()
    slot.set_editor_property("material_slot_name", "ZoneFace")
    slot.set_editor_property("material_interface", mat)
    mesh.set_editor_property("static_materials", [slot])
    unreal.EditorStaticMeshLibrary.remove_collisions(mesh)
    # no distance field: the sun's far (distance-field) shadows ignore the material
    # and drew the walls as black strips / 距離場なし（遠景の影で黒い帯になった）
    bs = unreal.EditorStaticMeshLibrary.get_lod_build_settings(mesh, 0)
    bs.set_editor_property("distance_field_resolution_scale", 0.0)
    unreal.EditorStaticMeshLibrary.set_lod_build_settings(mesh, 0, bs)
    mesh.get_editor_property("body_setup").set_editor_property(
        "collision_trace_flag", unreal.CollisionTraceFlag.CTF_USE_SIMPLE_AS_COMPLEX)
    EAL.save_loaded_asset(mesh)
    log("face", path, mesh.get_bounding_box())


try:
    for n, rgb in COLOURS.items():
        mat = material(n, rgb)
        for k in ASPECTS:
            face(n, k, mat)
    log("OK")
except Exception:
    import traceback
    log("FAILED\n" + traceback.format_exc())
LOG.close()
