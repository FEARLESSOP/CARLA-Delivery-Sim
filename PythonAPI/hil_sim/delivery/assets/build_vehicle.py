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

import json
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
# optional real drivetrain (cars/drivetrain.py --emit); unset = the Prius's
# 実車のドライブトレイン（任意。未指定ならプリウスのまま）
DRIVETRAIN = os.environ.get("VEH_DRIVETRAIN", "")
# heavy / NPC vehicles (cars/build_cars.sh, car_guide ch. 16); unset = as before:
# VehicleFactory base_type, per-wheel brake / handbrake torque (Nm; the Prius
# blueprints have 800 / 1600), an own physics asset whose chassis body is a box
# "cx,cy,cz,hx,hy,hz" (cm, vehicle frame: centre and half sizes) instead of
# the Prius's body, and the centre-of-mass height (cm above the ground)
# 大型・NPC 車両用（未指定ならこれまで通り）：base_type、車輪ごとのブレーキ
# トルク、車体と同じ大きさの箱を持つ専用物理アセット、重心の高さ
BASE_TYPE = os.environ.get("VEH_BASE_TYPE") or "car"
BRAKE_NM = os.environ.get("VEH_BRAKE_NM", "")
HANDBRAKE_NM = os.environ.get("VEH_HANDBRAKE_NM", "")
BODY = [float(v) for v in os.environ.get("VEH_BODY", "").split(",") if v.strip()]
COM_Z_CM = os.environ.get("VEH_COM_Z_CM", "")

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
    out, brakes = {}, []
    for tag in ("FLW", "FRW", "RLW", "RRW"):
        path = duplicate("/Game/Carla/Blueprints/Vehicles/ToyotaPrius/BP_ToyotaPrius_" + tag,
                         "BP_%s_%s" % (NAME, tag))
        cls = EAL.load_blueprint_class(path)
        cdo = unreal.get_default_object(cls)
        cdo.set_editor_property("shape_radius", WHEEL_RADIUS_CM)
        cdo.set_editor_property("shape_width", WHEEL_WIDTH_CM)
        if BRAKE_NM:
            cdo.set_editor_property("max_brake_torque", float(BRAKE_NM))
        # handbrake only where the Prius has one (rear) / サイドブレーキは後輪のみ
        if HANDBRAKE_NM and cdo.get_editor_property("affected_by_handbrake"):
            cdo.set_editor_property("max_hand_brake_torque", float(HANDBRAKE_NM))
        EAL.save_asset(path, only_if_is_dirty=False)
        out[tag] = cls
        log("wheel", path, "r", cdo.get_editor_property("shape_radius"))
        brakes.append("%s %.0f/%.0f" % (tag, cdo.get_editor_property("max_brake_torque"),
                                        cdo.get_editor_property("max_hand_brake_torque")))
    if BRAKE_NM or HANDBRAKE_NM:
        log("brakes (brake/handbrake Nm)", ", ".join(brakes))
    return out


def apply_drivetrain(vm, d):
    """Gears, final drive, engine and drag from cars/drivetrain.py's JSON
    into the WheeledVehicleMovementComponent4W (UE 4.26 property names,
    dumped from BP_ToyotaPrius). The structs come back as copies: change
    them and set them again. The torque curve stays the Prius's here (its
    keys are not exposed to Python); vehicle_tuning.py applies the real one.
    ギア・終減速比・エンジン・空気抵抗を設定（構造体はコピーなので設定し直す）。
    トルクカーブは Python から書けないため vehicle_tuning.py が実行時に適用。
    """
    eng = vm.get_editor_property("engine_setup")
    eng.set_editor_property("max_rpm", d["max_rpm"])
    eng.set_editor_property("moi", d["moi"])
    for name, value in zip(("damping_rate_full_throttle", "damping_rate_zero_throttle_clutch_engaged",
                            "damping_rate_zero_throttle_clutch_disengaged"), d["damping"]):
        eng.set_editor_property(name, value)
    vm.set_editor_property("engine_setup", eng)
    tr = vm.get_editor_property("transmission_setup")
    gears = []
    for g in d["gears"]:
        gd = unreal.VehicleGearData()
        gd.set_editor_property("ratio", g["ratio"])
        gd.set_editor_property("down_ratio", g["down"])
        gd.set_editor_property("up_ratio", g["up"])
        gears.append(gd)
    tr.set_editor_property("forward_gears", gears)
    for name, key in (("final_ratio", "final"), ("reverse_gear_ratio", "reverse"),
                      ("use_gear_auto_box", "autobox"), ("gear_switch_time", "switch_time"),
                      ("gear_auto_box_latency", "latency"), ("clutch_strength", "clutch"),
                      ("neutral_gear_up_ratio", "neutral_up")):
        tr.set_editor_property(name, d[key])
    vm.set_editor_property("transmission_setup", tr)
    diff = vm.get_editor_property("differential_setup")
    diff.set_editor_property("differential_type", getattr(unreal.VehicleDifferential4W, d["differential"]))
    diff.set_editor_property("front_rear_split", d["front_rear_split"])
    vm.set_editor_property("differential_setup", diff)
    vm.set_editor_property("drag_coefficient", d["drag"])
    vm.set_editor_property("chassis_width", d["chassis_cm"][0])
    vm.set_editor_property("chassis_height", d["chassis_cm"][1])
    tr = vm.get_editor_property("transmission_setup")
    eng = vm.get_editor_property("engine_setup")
    log("drivetrain gears", [round(g.get_editor_property("ratio"), 3) for g in tr.get_editor_property("forward_gears")],
        "final", round(tr.get_editor_property("final_ratio"), 3),
        "reverse", round(tr.get_editor_property("reverse_gear_ratio"), 3),
        "max_rpm", eng.get_editor_property("max_rpm"), "moi", eng.get_editor_property("moi"),
        "diff", vm.get_editor_property("differential_setup").get_editor_property("differential_type"),
        "drag", round(vm.get_editor_property("drag_coefficient"), 3))


def body_setups(pa):
    """The physics asset's body setups. SkeletalBodySetups is not an editor
    property in UE 4.26 for every build, so fall back to the sub-objects by
    name (SkeletalBodySetup_<n> inside the asset's package).
    物理アセットのボディ一覧（プロパティで読めなければ名前で探す）。"""
    try:
        found = list(pa.get_editor_property("skeletal_body_setups"))
        if found:
            return found
    except Exception as e:
        log("skeletal_body_setups not readable:", e)
    path = pa.get_path_name()
    found = []
    for cls in ("SkeletalBodySetup", "BodySetup"):
        for i in range(64):
            o = unreal.find_object(None, "%s:%s_%d" % (path, cls, i))
            if o is not None:
                found.append(o)
    if not found:
        # the _<n> counter runs across the editor session: every body whose outer is the asset
        # 番号はセッション全体で増える: 外側がこのアセットのボディを全部探す
        found = [o for o in unreal.ObjectIterator(unreal.SkeletalBodySetup) if o.get_outer() == pa]
    return found


def make_physics_body(mesh):
    """An own physics asset (a copy of the Prius's) whose chassis body
    (bone Vehicle_Base) is one box the size of the vehicle's body, so a long
    vehicle (the 10.4 m bus) collides at its own ends and PhysX takes its
    inertia from that box at the blueprint mass. Optional centre-of-mass
    height via the body's COM nudge. The wheels stay raycasts as before.
    専用の物理アセット：車体ボディを車体寸法の箱にする（慣性もこの箱から）。"""
    prius_pa = EAL.load_asset(PRIUS_MESH).get_editor_property("physics_asset")
    dst = duplicate(prius_pa.get_path_name().split(".")[0], "PA_" + NAME)
    pa = EAL.load_asset(dst)
    setups = body_setups(pa)
    log("physics asset", dst, "bodies", [str(b.get_editor_property("bone_name")) for b in setups])
    base = [b for b in setups if str(b.get_editor_property("bone_name")) == "Vehicle_Base"]
    if len(base) != 1:
        raise RuntimeError("physics asset: expected one Vehicle_Base body, got %d" % len(base))
    bs = base[0]
    agg = bs.get_editor_property("agg_geom")
    old = {k: len(agg.get_editor_property(k)) for k in
           ("box_elems", "convex_elems", "sphyl_elems", "sphere_elems", "tapered_capsule_elems")}
    for e in agg.get_editor_property("box_elems"):          # what the Prius had / 元の形
        log("prius box", e.get_editor_property("center"), e.get_editor_property("x"),
            e.get_editor_property("y"), e.get_editor_property("z"))
    box = unreal.KBoxElem()
    box.set_editor_property("center", unreal.Vector(BODY[0], BODY[1], BODY[2]))
    box.set_editor_property("x", 2.0 * BODY[3])             # full sizes / 全長
    box.set_editor_property("y", 2.0 * BODY[4])
    box.set_editor_property("z", 2.0 * BODY[5])
    agg.set_editor_property("box_elems", [box])
    for k in ("convex_elems", "sphyl_elems", "sphere_elems", "tapered_capsule_elems"):
        agg.set_editor_property(k, [])
    bs.set_editor_property("agg_geom", agg)
    agg = bs.get_editor_property("agg_geom")
    got = agg.get_editor_property("box_elems")
    if len(got) != 1 or agg.get_editor_property("convex_elems"):
        raise RuntimeError("physics body box not kept: %d boxes, %d convex" % (
            len(got), len(agg.get_editor_property("convex_elems"))))
    if COM_Z_CM:
        inst = bs.get_editor_property("default_instance")
        nudge = unreal.Vector(0.0, 0.0, float(COM_Z_CM) - BODY[2])
        inst.set_editor_property("com_nudge", nudge)
        bs.set_editor_property("default_instance", inst)
        log("centre of mass nudge", bs.get_editor_property("default_instance").get_editor_property("com_nudge"))
    EAL.save_asset(dst, only_if_is_dirty=False)
    mesh.set_editor_property("physics_asset", pa)
    EAL.save_loaded_asset(mesh)
    c = got[0].get_editor_property("center")
    log("physics body box centre (%.1f, %.1f, %.1f) size (%.1f, %.1f, %.1f) cm, replaced %s"
        % (c.x, c.y, c.z, got[0].get_editor_property("x"), got[0].get_editor_property("y"),
           got[0].get_editor_property("z"), old))


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
    if DRIVETRAIN:
        apply_drivetrain(vm, json.load(open(DRIVETRAIN)))
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
    entry.set_editor_property("base_type", BASE_TYPE)
    entry.set_editor_property("recommended_colors", [unreal.Color(r=COLOR[0], g=COLOR[1], b=COLOR[2], a=255)])
    vehicles.append(entry)
    fcdo.set_editor_property("vehicles", vehicles)
    EAL.save_asset(FACTORY, only_if_is_dirty=False)
    log("factory now has", len(fcdo.get_editor_property("vehicles")), "vehicles; base_type", BASE_TYPE)


try:
    mesh = import_mesh()
    if BODY:
        if len(BODY) != 6:
            raise RuntimeError("VEH_BODY needs cx,cy,cz,hx,hy,hz: %s" % BODY)
        make_physics_body(mesh)
    wheels = make_wheels()
    cls = make_vehicle(mesh, wheels)
    register(cls)
    log("OK")
except Exception:
    log("FAILED\n" + traceback.format_exc())
LOG.close()
