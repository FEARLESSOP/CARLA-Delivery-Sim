"""Unreal Editor step (PythonScript commandlet): build BP_JpShingo, the
Japanese traffic signal of the Japan maps, from osm_city/jp_shingo.py's FBX.

  cd <CARLA_SRC>/Unreal/CarlaUE4 && SHINGO_SRC=<build/JpShingo> SHINGO_PKG=Japan \\
    UE4Editor $PWD/CarlaUE4.uproject -run=pythonscript -script=<this> -unattended -nosplash -nullrhi

1. imports SM_JpShingoPole / Arm / Head / Lamp into /Game/<PKG>/Shingo and
   gives them our own plain materials (M_JpShingoBody grey steel,
   M_JpShingoHood black); the lamp's slot 1 (lens) gets CARLA's
   M_StreetlightsMaster_T10, which the blueprint replaces at run time anyway;
2. duplicates CARLA's BP_TLOpenDrive_RHT (stock asset untouched) into
   /Game/<PKG>/Shingo/BP_JpShingo, so the actor stays a CARLA TrafficLight
   with the same graph: TrafficLightComponent, LightChanged -> On/Off on the
   Green/Yellow/RedLights material lists, the vehicle trigger box;
3. swaps the meshes: pole and arm on the two pole components, no ground base,
   and Heads[0] = our head body at the arm's end with three lamps left to
   right green, yellow, red (layout.json).
Rerunnable: reimports over the same assets. Nothing outside /Game/<PKG>/Shingo
is saved. ue_jp_shingo_level.py then points a map's TrafficLightManager at it.

日本の信号機ブループリント BP_JpShingo を作る（CARLA の信号 BP の複製、メッシュだけ自作）。
"""

import json
import os

import unreal

SRC = os.environ["SHINGO_SRC"]
PKG = os.environ.get("SHINGO_PKG", "Japan")
DEST = "/Game/%s/Shingo" % PKG
STOCK_BP = "/Game/Carla/Blueprints/TrafficLight/BP_TLOpenDrive_RHT"
LAMP_MASTER = "/Game/Carla/Static/TrafficLight/Materials/M_StreetlightsMaster_T10"
BP_NAME = "BP_JpShingo"
# member names of CARLA's user-defined structs (TrafficLightPieceStructure / ST_TrafficLightHead)
F_LIGHTS = "Lights_7_44253FD24302BDE4A2294EB56C46094E"
F_POS = "Position_4_AE0F9C114CCB8E4E1ADBB9A9452DBB3B"
F_SUPPORT = "Support_13_A99B1FFC08D715D0AB1B90D8B412A710"
F_DESC = "Description_10_CCEF901C4FAFAF21FA95949E1860BF6F"
F_MESH = "Mesh_14_E403884A4621A9F4FE04D19222208CCC"
F_REL = "RelativeTransform_17_9B3221F1461CC8B39ECEB8BCA6B54EAD"
F_COLOR = "LightColor_18_20EE2088400337D5F9D46B90E3099CEB"
F_MASKED = "Masked_5_F2A25C244C0B7654B1C241850D47224B"

lib = unreal.EditorAssetLibrary
mel = unreal.MaterialEditingLibrary
tools = unreal.AssetToolsHelpers.get_asset_tools()


def log(s):
    unreal.log_warning("[ue_jp_shingo] " + str(s))


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
    task = unreal.AssetImportTask()
    task.set_editor_property("filename", os.path.join(SRC, name + ".fbx"))
    task.set_editor_property("destination_path", DEST)
    task.set_editor_property("destination_name", name)
    task.set_editor_property("automated", True)
    task.set_editor_property("replace_existing", True)
    task.set_editor_property("save", False)
    task.set_editor_property("options", ui)
    tools.import_asset_tasks([task])
    mesh = lib.load_asset("%s/%s" % (DEST, name))
    if not isinstance(mesh, unreal.StaticMesh):
        raise RuntimeError("import failed: " + name)
    return mesh


def plain_material(name, rgb, rough, metal):
    path = "%s/%s" % (DEST, name)
    m = lib.load_asset(path) if lib.does_asset_exist(path) else \
        tools.create_asset(name, DEST, unreal.Material, unreal.MaterialFactoryNew())
    mel.delete_all_material_expressions(m)
    c = mel.create_material_expression(m, unreal.MaterialExpressionConstant3Vector, -400, 0)
    c.set_editor_property("constant", unreal.LinearColor(rgb[0], rgb[1], rgb[2], 1.0))
    mel.connect_material_property(c, "", unreal.MaterialProperty.MP_BASE_COLOR)
    for val, prop, y in ((rough, unreal.MaterialProperty.MP_ROUGHNESS, 200), (metal, unreal.MaterialProperty.MP_METALLIC, 300)):
        k = mel.create_material_expression(m, unreal.MaterialExpressionConstant, -400, y)
        k.set_editor_property("r", val)
        mel.connect_material_property(k, "", prop)
    mel.recompile_material(m)
    lib.save_loaded_asset(m)
    return m


def set_slots(mesh, mats):
    n = len(mesh.static_materials)
    if n != len(mats):
        raise RuntimeError("%s has %d material slots, expected %d" % (mesh.get_name(), n, len(mats)))
    for i, m in enumerate(mats):
        mesh.set_material(i, m)
    lib.save_loaded_asset(mesh)
    log("%s slots %s" % (mesh.get_name(), [(str(s.material_slot_name), m.get_name()) for s, m in zip(mesh.static_materials, mats)]))


def template(bp_path, comp):
    obj = unreal.load_object(None, "%s.%s_C:%s_GEN_VARIABLE" % (bp_path, BP_NAME, comp))
    if obj is None:
        raise RuntimeError("no component template %s in %s" % (comp, bp_path))
    return obj


def main():
    layout = json.load(open(os.path.join(SRC, "layout.json")))
    body = plain_material("M_JpShingoBody", (0.50, 0.51, 0.50), 0.5, 0.4)
    hood = plain_material("M_JpShingoHood", (0.02, 0.02, 0.02), 0.6, 0.0)
    master = lib.load_asset(LAMP_MASTER)
    meshes = {n: import_mesh(n) for n in ("SM_JpShingoPole", "SM_JpShingoArm", "SM_JpShingoHead", "SM_JpShingoLamp")}
    set_slots(meshes["SM_JpShingoPole"], [body])
    set_slots(meshes["SM_JpShingoArm"], [body])
    set_slots(meshes["SM_JpShingoHead"], [body, hood])
    set_slots(meshes["SM_JpShingoLamp"], [hood, master])       # slot 1 = lens, driven by the BP

    bp_path = "%s/%s" % (DEST, BP_NAME)
    if not lib.does_asset_exist(bp_path):
        if not lib.duplicate_asset(STOCK_BP, bp_path):
            raise RuntimeError("duplicate of %s failed" % STOCK_BP)
        log("duplicated %s -> %s" % (STOCK_BP, bp_path))
    bp = lib.load_asset(bp_path)
    cls = lib.load_blueprint_class(bp_path)

    # pole and arm on the blueprint's own pole components / 柱と腕
    zero = unreal.Vector(0, 0, 0)
    for comp, mesh in (("SM_TrafficLight__VPole_Main", meshes["SM_JpShingoPole"]),
                       ("SM_TrafficLight__HPole01_Long1", meshes["SM_JpShingoArm"]),
                       ("SM_TrafficLight__Ground_base", None)):
        t = template(bp_path, comp)
        t.set_editor_property("static_mesh", mesh)
        t.set_editor_property("relative_location", zero)
        t.set_editor_property("relative_rotation", unreal.Rotator(0, 0, 0))
        # the stock pole is scaled 1.2 (the arm hangs off it, so inherits it): our
        # meshes are real size, and the head (not scaled) must meet the arm
        # 純正の柱は 1.2 倍（腕も継承）。実寸のメッシュなので 1 に戻す（灯器と腕が離れる）
        t.set_editor_property("relative_scale3d", unreal.Vector(1, 1, 1))
        t.set_editor_property("visible", mesh is not None)
        if mesh is None:
            t.set_collision_profile_name("NoCollision")

    # the head and its three lamps / 灯器と 3 灯
    cdo = unreal.get_default_object(cls)
    heads = cdo.get_editor_property("Heads")
    piece = heads[0]
    lights = piece.get_editor_property(F_LIGHTS)
    if len(lights) != 3:
        raise RuntimeError("expected 3 lamps in the stock head, found %d" % len(lights))
    by_colour = {}
    for i in range(3):
        by_colour[str(lights[i].get_editor_property(F_COLOR)).split(".")[-1].split(":")[0]] = i
    new = []
    for spec in layout["lamps_cm"]:
        e = lights[by_colour[spec["colour"]]]
        e.set_editor_property(F_MESH, meshes["SM_JpShingoLamp"])
        e.set_editor_property(F_MASKED, False)
        e.set_editor_property(F_REL, unreal.Transform(unreal.Vector(spec["x"], spec["y"], spec["z"]),
                                                      unreal.Rotator(0, 0, 0), unreal.Vector(1, 1, 1)))
        new.append(e)
    piece.set_editor_property(F_LIGHTS, new)
    hx, hy, hz = layout["head_position_cm"]
    piece.set_editor_property(F_POS, unreal.Transform(unreal.Vector(hx, hy, hz), unreal.Rotator(0, 0, 0), unreal.Vector(1, 1, 1)))
    piece.set_editor_property(F_SUPPORT, meshes["SM_JpShingoHead"])
    piece.set_editor_property(F_DESC, "Japanese horizontal LED head: green, yellow, red from the left")
    cdo.set_editor_property("Heads", [piece])
    cdo.set_editor_property("Props", [])

    lib.save_loaded_asset(bp, only_if_is_dirty=False)
    for p in lib.list_assets(DEST, recursive=False):
        lib.save_asset(p, only_if_is_dirty=False)
    chk = unreal.get_default_object(lib.load_blueprint_class(bp_path)).get_editor_property("Heads")[0]
    log("BP_JpShingo saved: support %s, lamps %s" % (
        chk.get_editor_property(F_SUPPORT).get_name(),
        [(str(l.get_editor_property(F_COLOR)).split(".")[-1].split(":")[0],
          l.get_editor_property(F_MESH).get_name(), l.get_editor_property(F_REL).translation.x)
         for l in chk.get_editor_property(F_LIGHTS)]))
    log("done")


main()
