"""Make the delivery pod (or one of your own cars) a drivable CARLA vehicle (UE4 editor Python).

Run by build_models.sh inside the CARLA source project:
  UE4Editor CarlaUE4.uproject -run=pythonscript -script=build_vehicle.py

PodVehicle.fbx (prepare_models.py) has the Toyota Prius's skeleton layout --
root Vehicle_Base and four wheel bones -- so it is imported onto the Prius's
skeleton and shares its wheel animation blueprint (spinning, steering) and
physics asset. It gets its own wheel blueprints (the pod's 33 cm wheels),
its own vehicle blueprint, and an entry in CARLA's VehicleFactory, which
makes it the blueprint ``vehicle.delivery.pod``. The result goes to the log
and to build_vehicle.log next to this script.

配送ポッドを走行可能な CARLA 車両にする（UE4 エディタ Python）。PodVehicle.fbx
はプリウスと同じスケルトン構成なので、プリウスのスケルトン・車輪アニメーション・
物理アセットを共有し、専用の車輪・車両ブループリントを作って VehicleFactory に
``vehicle.delivery.pod`` として登録する。
"""

import os
import traceback

import unreal

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.environ.get("POD_VEHICLE_FBX", os.path.join(HERE, "PodVehicle.fbx"))
# the LastMile pod (build_lastmile.sh) and cars/build_cars.sh set these; the
# defaults build the PodV2 vehicle
# LastMile ポッドは環境変数で指定（既定値は PodV2）
NAME = os.environ.get("VEH_NAME", "DeliveryPod")
DEST = os.environ.get("VEH_DEST", "/Game/DeliveryModels/Vehicles/DeliveryPod")
PRIUS_BP = "/Game/Carla/Blueprints/Vehicles/ToyotaPrius/BP_ToyotaPrius"
PRIUS_MESH = "/Game/Carla/Static/Car/4Wheeled/Toyota_Prius/SK_Toyota_Prius"
FACTORY = "/Game/Carla/Blueprints/Vehicles/VehicleFactory"
WHEEL_RADIUS_CM = float(os.environ.get("VEH_WHEEL_R_CM", 33.2))
WHEEL_WIDTH_CM = float(os.environ.get("VEH_WHEEL_W_CM", 22.5))
MASS_KG = float(os.environ.get("VEH_MASS", 1450.0))
# half-size x, y, z and centre height of the bounding box, cm / 外形の半寸法と中心高さ [cm]
BOUNDS = [float(v) for v in os.environ.get("VEH_BOUNDS", "226,107,198,198").split(",")]
MAKE, MODEL = os.environ.get("VEH_MAKE", "delivery"), os.environ.get("VEH_MODEL", "pod")
COLOR = [int(v) for v in os.environ.get("VEH_COLOR", "245,245,245").split(",")]
IMPORT_TEXTURES = os.environ.get("VEH_TEXTURES", "0") == "1"

EAL = unreal.EditorAssetLibrary
LOG = open(os.path.join(HERE, os.environ.get("VEH_LOG", "build_vehicle.log")), "w")


def log(*a):
    msg = " ".join(str(x) for x in a)
    LOG.write(msg + "\n")
    LOG.flush()
    unreal.log("build_vehicle: " + msg)


def import_mesh():
    prius = EAL.load_asset(PRIUS_MESH)
    ui = unreal.FbxImportUI()
    ui.set_editor_property("import_mesh", True)
    ui.set_editor_property("import_as_skeletal", True)
    ui.set_editor_property("mesh_type_to_import", unreal.FBXImportType.FBXIT_SKELETAL_MESH)
    ui.set_editor_property("skeleton", prius.get_editor_property("skeleton"))
    ui.set_editor_property("physics_asset", prius.get_editor_property("physics_asset"))
    ui.set_editor_property("create_physics_asset", False)
    ui.set_editor_property("import_materials", True)
    ui.set_editor_property("import_textures", IMPORT_TEXTURES)
    ui.set_editor_property("import_animations", False)
    sk = ui.get_editor_property("skeletal_mesh_import_data")
    sk.set_editor_property("import_morph_targets", False)
    sk.set_editor_property("convert_scene", True)
    task = unreal.AssetImportTask()
    task.set_editor_property("filename", SRC)
    task.set_editor_property("destination_path", DEST)
    task.set_editor_property("destination_name", "SK_" + NAME)
    task.set_editor_property("replace_existing", True)
    task.set_editor_property("automated", True)
    task.set_editor_property("save", True)
    task.set_editor_property("options", ui)
    unreal.AssetToolsHelpers.get_asset_tools().import_asset_tasks([task])
    paths = list(task.get_editor_property("imported_object_paths"))
    meshes = [unreal.load_asset(p) for p in paths]
    meshes = [m for m in meshes if isinstance(m, unreal.SkeletalMesh)]
    if not meshes:
        raise RuntimeError("skeletal mesh import failed: %s" % paths)
    mesh = meshes[0]
    mesh.set_editor_property("physics_asset", prius.get_editor_property("physics_asset"))
    EAL.save_loaded_asset(mesh)
    # the import also made one material per FBX material; save them too, or the
    # packaged game can't build the pod (the editor tends to crash on exit and
    # nothing unsaved survives) / インポートで作られたマテリアルも保存する
    EAL.save_directory(DEST, only_if_is_dirty=False, recursive=True)
    mats = [a for a in EAL.list_assets(DEST, recursive=True)
            if EAL.find_asset_data(a).asset_class in ("Material", "MaterialInstanceConstant")]
    log("materials saved", len(mats))
    if not mats:
        raise RuntimeError("no materials saved with the pod mesh")
    log("mesh", mesh.get_path_name(), "skeleton",
        mesh.get_editor_property("skeleton").get_path_name())
    return mesh


def duplicate(src, name):
    dst = DEST + "/" + name
    if EAL.does_asset_exist(dst):
        EAL.delete_asset(dst)
    if not EAL.duplicate_asset(src, dst):
        raise RuntimeError("could not duplicate %s" % src)
    return dst


def make_wheels():
    out = {}
    for tag in ("FLW", "FRW", "RLW", "RRW"):
        path = duplicate("/Game/Carla/Blueprints/Vehicles/ToyotaPrius/BP_ToyotaPrius_" + tag,
                         "BP_%s_%s" % (NAME, tag))
        cls = EAL.load_blueprint_class(path)
        cdo = unreal.get_default_object(cls)
        cdo.set_editor_property("shape_radius", WHEEL_RADIUS_CM)
        cdo.set_editor_property("shape_width", WHEEL_WIDTH_CM)
        EAL.save_asset(path, only_if_is_dirty=False)
        out[tag] = cls
        log("wheel", path, "r", cdo.get_editor_property("shape_radius"))
    return out


def make_vehicle(mesh, wheels):
    path = duplicate(PRIUS_BP, "BP_" + NAME)
    cls = EAL.load_blueprint_class(path)
    cdo = unreal.get_default_object(cls)
    comp = cdo.get_editor_property("mesh")
    comp.set_editor_property("skeletal_mesh", mesh)
    vm = cdo.get_editor_property("vehicle_movement")
    setups = vm.get_editor_property("wheel_setups")
    order = {"Wheel_Front_Left": "FLW", "Wheel_Front_Right": "FRW",
             "Wheel_Rear_Left": "RLW", "Wheel_Rear_Right": "RRW"}
    # struct array elements come back as copies: build a new list and set it
    # 構造体配列の要素はコピーなので、新しいリストを作って設定する
    new = []
    for w in setups:
        s = unreal.WheelSetup()
        s.set_editor_property("bone_name", w.get_editor_property("bone_name"))
        s.set_editor_property("disable_steering", w.get_editor_property("disable_steering"))
        s.set_editor_property("additional_offset", w.get_editor_property("additional_offset"))
        s.set_editor_property("wheel_class", wheels[order[str(w.get_editor_property("bone_name"))]])
        new.append(s)
    vm.set_editor_property("wheel_setups", new)
    setups = vm.get_editor_property("wheel_setups")
    vm.set_editor_property("mass", MASS_KG)
    # the bounding box CARLA reports (and the traffic manager uses), in cm
    # CARLA が返す（交通マネージャも使う）バウンディングボックス [cm]
    for name in ("vehicle_bounds", "VehicleBounds"):
        try:
            box = cdo.get_editor_property(name)
            box.set_editor_property("box_extent", unreal.Vector(*BOUNDS[:3]))
            box.set_relative_location(unreal.Vector(0.0, 0.0, BOUNDS[3]), False, False)
            log("bounds set via", name)
            break
        except Exception as e:
            log("no bounds property", name, e)
    EAL.save_asset(path, only_if_is_dirty=False)
    log("vehicle", path, "mesh", comp.get_editor_property("skeletal_mesh").get_path_name(),
        "wheels", [str(w.get_editor_property("wheel_class").get_name()) for w in setups])
    return cls


def register(cls):
    fcls = EAL.load_blueprint_class(FACTORY)
    fcdo = unreal.get_default_object(fcls)
    vehicles = [v for v in fcdo.get_editor_property("vehicles")
                if not (str(v.get_editor_property("make")) == MAKE and
                        str(v.get_editor_property("model")) == MODEL)]
    entry = unreal.VehicleParameters()
    entry.set_editor_property("make", MAKE)
    entry.set_editor_property("model", MODEL)
    entry.set_editor_property("class_", cls)
    entry.set_editor_property("number_of_wheels", 4)
    entry.set_editor_property("generation", 2)
    entry.set_editor_property("base_type", "car")
    entry.set_editor_property("recommended_colors", [unreal.Color(r=COLOR[0], g=COLOR[1], b=COLOR[2], a=255)])
    vehicles.append(entry)
    fcdo.set_editor_property("vehicles", vehicles)
    EAL.save_asset(FACTORY, only_if_is_dirty=False)
    log("factory now has", len(fcdo.get_editor_property("vehicles")), "vehicles")


try:
    mesh = import_mesh()
    wheels = make_wheels()
    cls = make_vehicle(mesh, wheels)
    register(cls)
    log("OK")
except Exception:
    log("FAILED\n" + traceback.format_exc())
LOG.close()
