"""Unreal Editor step (PythonScript commandlet): street lamps that light up (街灯).

  cd <CARLA_SRC>/Unreal/CarlaUE4 && GAITO_STEP=bp GAITO_SRC=<build/JpGaito> GAITO_PKG=Japan \\
    UE4Editor $PWD/CarlaUE4.uproject -run=pythonscript -script=<this> -unattended -nosplash -nullrhi
  ... GAITO_STEP=level GAITO_PKG=Japan GAITO_MAP=OitaKosen GAITO_HEADS=<build/<Map>/gaito.json> ...

bp:    imports SM_JpGaitoGlow (osm_city/jp_gaito.py) into /Game/<PKG>/Gaito and
       duplicates CARLA's BP_StreetLight (stock asset untouched) into
       BP_JpGaito: the lamp mesh slot gets our plate with CARLA's lamp material
       (the BP_Lights component makes it glow when on), the spot light points
       straight down from the plate, no shadows, and both lights stop drawing
       beyond 80 m, so ~1600 of them stay cheap. The BP_Lights component (a
       CarlaLight, group Street) is CARLA's: the weather and the client's
       LightManager switch it like the stock towns' lamps.
level: loads the map's level, removes the BP_JpGaito actors it has, spawns
       one at every head in gaito.json (osm_city/gaito.py) and saves it.
Log markers decide (the editor may segfault on exit after saving).

街灯: CARLA の BP_StreetLight の複製（自作の発光板、影なし、80 m で非表示）を
各灯具の下に置く。点灯・消灯は CARLA の LightManager（Street）。
"""

import json
import os

import unreal

STEP = os.environ.get("GAITO_STEP", "bp")
PKG = os.environ.get("GAITO_PKG", "Japan")
DEST = "/Game/%s/Gaito" % PKG
STOCK_BP = "/Game/Carla/Static/StreetLight/BP_StreetLight"
BP_NAME = "BP_JpGaito"
BP_PATH = "%s/%s" % (DEST, BP_NAME)
ID_BASE = 100000                           # light Ids from here (unique per lamp) / ライト Id
DRAW_CM, FADE_CM = 8000.0, 2000.0          # lights beyond 80 m are not drawn / 80 m 先は描画しない

lib = unreal.EditorAssetLibrary
tools = unreal.AssetToolsHelpers.get_asset_tools()


def log(s):
    unreal.log_warning("[ue_jp_gaito] " + str(s))


def import_plate(src):
    ui = unreal.FbxImportUI()
    ui.set_editor_property("import_mesh", True)
    ui.set_editor_property("import_as_skeletal", False)
    ui.set_editor_property("mesh_type_to_import", unreal.FBXImportType.FBXIT_STATIC_MESH)
    ui.set_editor_property("import_materials", False)
    ui.set_editor_property("import_textures", False)
    sm = ui.get_editor_property("static_mesh_import_data")
    sm.set_editor_property("combine_meshes", True)
    sm.set_editor_property("auto_generate_collision", False)
    task = unreal.AssetImportTask()
    task.set_editor_property("filename", os.path.join(src, "SM_JpGaitoGlow.fbx"))
    task.set_editor_property("destination_path", DEST)
    task.set_editor_property("destination_name", "SM_JpGaitoGlow")
    task.set_editor_property("automated", True)
    task.set_editor_property("replace_existing", True)
    task.set_editor_property("options", ui)
    tools.import_asset_tasks([task])
    mesh = lib.load_asset(DEST + "/SM_JpGaitoGlow")
    if not isinstance(mesh, unreal.StaticMesh):
        raise RuntimeError("import failed: SM_JpGaitoGlow")
    return mesh


def template(bp_path, name, comp):
    obj = unreal.load_object(None, "%s.%s_C:%s_GEN_VARIABLE" % (bp_path, name, comp))
    if obj is None:
        raise RuntimeError("no component template %s in %s" % (comp, bp_path))
    return obj


def build_bp():
    plate = import_plate(os.environ["GAITO_SRC"])
    lamp_mat = template(STOCK_BP, "BP_StreetLight", "StreetLight_Mesh").get_material(0)
    plate.set_material(0, lamp_mat)
    lib.save_loaded_asset(plate)
    if not lib.does_asset_exist(BP_PATH):
        if not lib.duplicate_asset(STOCK_BP, BP_PATH):
            raise RuntimeError("duplicate of %s failed" % STOCK_BP)
        log("duplicated %s -> %s" % (STOCK_BP, BP_PATH))
    bp = lib.load_asset(BP_PATH)
    m = template(BP_PATH, BP_NAME, "StreetLight_Mesh")
    m.set_editor_property("static_mesh", plate)
    m.set_material(0, lamp_mat)
    m.set_collision_profile_name("NoCollision")
    m.set_editor_property("cast_shadow", False)
    spot = template(BP_PATH, BP_NAME, "SpotLight")
    spot.set_editor_property("relative_location", unreal.Vector(0, 0, -3))
    # unreal.Rotator(roll, pitch, yaw): pitch -90 = straight down / 真下
    spot.set_editor_property("relative_rotation", unreal.Rotator(0, -90, 0))
    spot.set_editor_property("inner_cone_angle", 25.0)
    spot.set_editor_property("outer_cone_angle", 70.0)
    spot.set_editor_property("attenuation_radius", 1800.0)
    point = template(BP_PATH, BP_NAME, "PointLight")
    point.set_editor_property("relative_location", unreal.Vector(0, 0, -15))
    point.set_editor_property("attenuation_radius", 500.0)
    for lc, draw in ((spot, DRAW_CM), (point, DRAW_CM / 2)):
        # the stock lights are Stationary (the towns bake their lighting); a
        # generated level has no baked lighting, so unshadowed Movable lights
        # 純正は Stationary（ライト焼き込み前提）。生成マップでは Movable
        lc.set_editor_property("mobility", unreal.ComponentMobility.MOVABLE)
        lc.set_editor_property("cast_shadows", False)
        lc.set_editor_property("max_draw_distance", draw)
        lc.set_editor_property("max_distance_fade_range", FADE_CM)
    lib.save_loaded_asset(bp, only_if_is_dirty=False)
    for p in lib.list_assets(DEST, recursive=False):
        lib.save_asset(p, only_if_is_dirty=False)
    log("BP_JpGaito saved: plate %s, material %s, spot down 25/70 deg, no shadows, drawn to %.0f m"
        % (plate.get_name(), lamp_mat.get_name(), DRAW_CM / 100))


def place():
    name = os.environ["GAITO_MAP"]
    level = "/Game/%s/Maps/%s/%s" % (PKG, name, name)
    heads = json.load(open(os.environ["GAITO_HEADS"]))["heads"]
    cls = lib.load_blueprint_class(BP_PATH)
    if cls is None:
        raise RuntimeError("no %s: run GAITO_STEP=bp first" % BP_PATH)
    if not unreal.EditorLoadingAndSavingUtils.load_map(level):
        raise RuntimeError("cannot load " + level)
    old = [a for a in unreal.EditorLevelLibrary.get_all_level_actors()
           if a.get_class().get_name() == BP_NAME + "_C"]
    for a in old:
        unreal.EditorLevelLibrary.destroy_actor(a)
    # CARLA's light subsystem keys lights by Id: equal Ids collapse to one
    # ("Light Id overlapping"); the towns set theirs with LightIdSetter
    # ライトは Id で管理されるため、街灯ごとに別の Id を付ける
    taken = [c.get_id() for a in unreal.EditorLevelLibrary.get_all_level_actors()
             for c in a.get_components_by_class(unreal.CarlaLight)]
    next_id = max([ID_BASE - 1] + taken) + 1
    made = 0
    for x, y, z, yaw in heads:
        tf = unreal.Transform(unreal.Vector(x * 100.0, y * 100.0, z * 100.0), unreal.Rotator(0, 0, yaw),
                              unreal.Vector(1, 1, 1))
        # EditorLevelLibrary.spawn_actor_* segfaults without a viewport (guide ch. 24)
        a = unreal.OpenDriveToMap.spawn_actor_with_check_no_collisions(cls, tf)
        if a is not None:
            a.set_folder_path("Gaito")
            for c in a.get_components_by_class(unreal.CarlaLight):
                c.set_editor_property("Id", next_id)
                next_id += 1
            made += 1
    if not unreal.EditorLevelLibrary.save_current_level():
        raise RuntimeError("level save failed: " + level)
    ids = [c.get_id() for a in unreal.EditorLevelLibrary.get_all_level_actors()
           for c in a.get_components_by_class(unreal.CarlaLight)]
    log("%s: %d old removed, %d of %d street lamps placed, light Ids unique: %s (%d)"
        % (level, len(old), made, len(heads), len(ids) == len(set(ids)), len(ids)))


if STEP == "bp":
    build_bp()
else:
    place()
log("done")
