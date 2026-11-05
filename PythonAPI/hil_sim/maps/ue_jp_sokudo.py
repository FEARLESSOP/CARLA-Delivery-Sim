"""Unreal Editor step (PythonScript commandlet): build BP_JpSokudo<V>, the
Japanese maximum-speed posts (最高速度 323, blue numerals) of the Japan maps,
from osm_city/jp_sokudo.py's FBX and textures.

  cd <CARLA_SRC>/Unreal/CarlaUE4 && SOKUDO_SRC=<build/JpSokudo> SOKUDO_PKG=Japan \\
    UE4Editor $PWD/CarlaUE4.uproject -run=pythonscript -script=<this> -unattended -nosplash -nullrhi

1. imports T_JpSokudo<V> (face textures) and SM_JpSokudo<V> into
   /Game/<PKG>/Sokudo; materials: M_JpSokudoPole (white), M_JpSokudoBack
   (grey), M_JpSokudoFace (texture parameter "Face") and one instance
   MI_JpSokudoFace<V> per value on the mesh's slot 2;
2. duplicates CARLA's BP_SpeedLimit40 (stock asset untouched; an
   ATrafficSignBase with a post and a plate component) into BP_JpSokudo<V>,
   puts SM_JpSokudo<V> (our pole + plate) on the post component and empties
   the plate component;
3. sets each copy's TrafficSignState to the stock BP_SpeedLimit<V>'s (so the
   actor keeps CARLA's type id, e.g. traffic.speed_limit.40; values without a
   stock BP stay UNKNOWN; stock 70 says SPEED_LIMIT_60 and 80 SPEED_LIMIT_90,
   copied as they are).
The speed itself is not in the blueprint: TrafficLightManager::SpawnSignals
adds a USpeedLimitComponent with the OpenDRIVE signal's value and its lane
trigger boxes to whatever ATrafficSignBase class it spawns. Rerunnable;
nothing outside /Game/<PKG>/Sokudo is saved. ue_jp_sokudo_level.py then
points a map's TrafficLightManager.SpeedLimitModels at these classes.

日本の最高速度標識 BP_JpSokudo<V> を作る（CARLA の速度標識 BP の複製、メッシュだけ自作）。
"""

import json
import os

import unreal

SRC = os.environ["SOKUDO_SRC"]
PKG = os.environ.get("SOKUDO_PKG", "Japan")
DEST = "/Game/%s/Sokudo" % PKG
STOCK_DIR = "/Game/Carla/Static/TrafficSign"
STOCK_BP = STOCK_DIR + "/BP_SpeedLimit40"
# the stock BP (dumped in the editor): SceneRootComponent, Box (legacy trigger at
# -210,0,90, kept), RoadSigns_SignPost (SM_RoadSigns01 pole at the origin) and
# SM_SpeedSign40 (the plate, at y +3.25 cm z 218 cm, facing +Y)
POST_COMP = "RoadSigns_SignPost"
PLATE_COMP = "SM_SpeedSign40"

lib = unreal.EditorAssetLibrary
mel = unreal.MaterialEditingLibrary
tools = unreal.AssetToolsHelpers.get_asset_tools()


def log(s):
    unreal.log_warning("[ue_jp_sokudo] " + str(s))


def run_import(filename, name, options=None):
    task = unreal.AssetImportTask()
    task.set_editor_property("filename", filename)
    task.set_editor_property("destination_path", DEST)
    task.set_editor_property("destination_name", name)
    task.set_editor_property("automated", True)
    task.set_editor_property("replace_existing", True)
    task.set_editor_property("save", False)
    if options:
        task.set_editor_property("options", options)
    tools.import_asset_tasks([task])
    return lib.load_asset("%s/%s" % (DEST, name))


def import_mesh(name):
    ui = unreal.FbxImportUI()
    ui.set_editor_property("import_mesh", True)
    ui.set_editor_property("import_as_skeletal", False)
    ui.set_editor_property("mesh_type_to_import", unreal.FBXImportType.FBXIT_STATIC_MESH)
    ui.set_editor_property("import_materials", False)
    ui.set_editor_property("import_textures", False)
    ui.set_editor_property("import_animations", False)
    sm = ui.get_editor_property("static_mesh_import_data")
    sm.set_editor_property("combine_meshes", True)
    sm.set_editor_property("auto_generate_collision", True)
    sm.set_editor_property("generate_lightmap_u_vs", True)
    mesh = run_import(os.path.join(SRC, name + ".fbx"), name, ui)
    if not isinstance(mesh, unreal.StaticMesh):
        raise RuntimeError("import failed: " + name)
    return mesh


def import_texture(name):
    tex = run_import(os.path.join(SRC, name + ".png"), name)
    if not isinstance(tex, unreal.Texture2D):
        raise RuntimeError("texture import failed: " + name)
    tex.set_editor_property("srgb", True)
    lib.save_loaded_asset(tex)
    return tex


def new_material(name):
    path = "%s/%s" % (DEST, name)
    m = lib.load_asset(path) if lib.does_asset_exist(path) else \
        tools.create_asset(name, DEST, unreal.Material, unreal.MaterialFactoryNew())
    mel.delete_all_material_expressions(m)
    return m


def constants(m, rough, metal):
    for val, prop, y in ((rough, unreal.MaterialProperty.MP_ROUGHNESS, 200), (metal, unreal.MaterialProperty.MP_METALLIC, 300)):
        k = mel.create_material_expression(m, unreal.MaterialExpressionConstant, -400, y)
        k.set_editor_property("r", val)
        mel.connect_material_property(k, "", prop)


def plain_material(name, rgb, rough, metal):
    m = new_material(name)
    c = mel.create_material_expression(m, unreal.MaterialExpressionConstant3Vector, -400, 0)
    c.set_editor_property("constant", unreal.LinearColor(rgb[0], rgb[1], rgb[2], 1.0))
    mel.connect_material_property(c, "", unreal.MaterialProperty.MP_BASE_COLOR)
    constants(m, rough, metal)
    mel.recompile_material(m)
    lib.save_loaded_asset(m)
    return m


def face_master(default_tex):
    m = new_material("M_JpSokudoFace")
    t = mel.create_material_expression(m, unreal.MaterialExpressionTextureSampleParameter2D, -500, 0)
    t.set_editor_property("parameter_name", "Face")
    t.set_editor_property("texture", default_tex)
    mel.connect_material_property(t, "RGB", unreal.MaterialProperty.MP_BASE_COLOR)
    constants(m, 0.35, 0.0)
    mel.recompile_material(m)
    lib.save_loaded_asset(m)
    return m


def face_instance(v, master, tex):
    name = "MI_JpSokudoFace%d" % v
    path = "%s/%s" % (DEST, name)
    mi = lib.load_asset(path) if lib.does_asset_exist(path) else \
        tools.create_asset(name, DEST, unreal.MaterialInstanceConstant, unreal.MaterialInstanceConstantFactoryNew())
    mel.set_material_instance_parent(mi, master)
    mel.set_material_instance_texture_parameter_value(mi, "Face", tex)
    mel.update_material_instance(mi)
    lib.save_loaded_asset(mi)
    return mi


def stock_state(v):
    p = "%s/BP_SpeedLimit%d" % (STOCK_DIR, v)
    if not lib.does_asset_exist(p):
        return unreal.TrafficSignState.UNKNOWN
    return unreal.get_default_object(lib.load_blueprint_class(p)).get_editor_property("TrafficSignState")


def main():
    layout = json.load(open(os.path.join(SRC, "layout.json")))
    pole = plain_material("M_JpSokudoPole", (0.80, 0.80, 0.79), 0.45, 0.0)
    back = plain_material("M_JpSokudoBack", (0.30, 0.31, 0.32), 0.40, 0.6)
    texs = {v: import_texture("T_JpSokudo%d" % v) for v in layout["values"]}
    master = face_master(texs[layout["values"][0]])
    built = {}
    for v in layout["values"]:
        mi = face_instance(v, master, texs[v])
        mesh = import_mesh("SM_JpSokudo%d" % v)
        if len(mesh.static_materials) != 3:
            raise RuntimeError("SM_JpSokudo%d has %d slots, expected 3" % (v, len(mesh.static_materials)))
        for i, m in enumerate((pole, back, mi)):
            mesh.set_material(i, m)
        lib.save_loaded_asset(mesh)

        name = "BP_JpSokudo%d" % v
        bp_path = "%s/%s" % (DEST, name)
        if not lib.does_asset_exist(bp_path):
            if not lib.duplicate_asset(STOCK_BP, bp_path):
                raise RuntimeError("duplicate of %s failed" % STOCK_BP)
            log("duplicated %s -> %s" % (STOCK_BP, bp_path))
        bp = lib.load_asset(bp_path)
        cls = lib.load_blueprint_class(bp_path)
        # our post on the stock post component, the stock plate component emptied
        # 支柱コンポーネントに自作メッシュ、元の板コンポーネントは空に
        for comp, m in ((POST_COMP, mesh), (PLATE_COMP, None)):
            t = unreal.load_object(None, "%s.%s_C:%s_GEN_VARIABLE" % (bp_path, name, comp))
            if t is None:
                raise RuntimeError("no component %s in %s" % (comp, bp_path))
            t.set_editor_property("static_mesh", m)
            t.set_editor_property("relative_location", unreal.Vector(0, 0, 0))
            t.set_editor_property("relative_rotation", unreal.Rotator(0, 0, 0))
            t.set_editor_property("relative_scale3d", unreal.Vector(1, 1, 1))
            t.set_editor_property("visible", m is not None)
            if m is None:
                t.set_collision_profile_name("NoCollision")
        cdo = unreal.get_default_object(cls)
        cdo.set_editor_property("TrafficSignState", stock_state(v))
        lib.save_loaded_asset(bp, only_if_is_dirty=False)
        built[v] = (cdo.get_editor_property("TrafficSignState"), mesh.get_name())
    for p in lib.list_assets(DEST, recursive=False):
        lib.save_asset(p, only_if_is_dirty=False)
    log("built %s" % {v: (str(s).split(".")[-1].split(":")[0], m) for v, (s, m) in built.items()})
    log("done")


main()
