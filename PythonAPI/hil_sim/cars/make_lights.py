"""Blinking indicators and mirror glass for one of your cars (UE4 editor Python).

CARLA turns a vehicle's indicators on by setting the scalar material parameter
"Left Blinker" / "Right Blinker" (300 for our Prius-based blueprints, 0 when
off) on every material of the components tagged "emissive"
(BaseVehiclePawn > SetLightListState). The material has to blink by itself:
CARLA's own light master does it with a sine of time.

This script, run after build_vehicle.py / make_glass.py by build_cars.sh:
  1. makes /Game/<PKG>/Materials/M_CarBlinker (opaque amber lens, emissive
     = colour x Glow (250000, CARLA's level) x (parameter > 0) x a 1.5 Hz
     on/off square wave, used
     with skeletal meshes) and its instances MI_CarBlinkerL / MI_CarBlinkerR
     (Side 0 = reads "Left Blinker", Side 1 = "Right Blinker");
  2. makes M_CarMirror (dark polished glass: base 0.10, metallic 1,
     roughness 0.15);
  3. swaps the car's slots named in CAR_BLINK_L / CAR_BLINK_R / CAR_MIRROR
     (comma-separated, exact names from prepare_car.py: <Name>_BlinkerL,
     <Name>_BlinkerR, <Name>_MirrorGlass);
  4. tags the blueprint's vehicle mesh "emissive" so CARLA sets the
     parameters on it.
Env: CAR_DIR (the car's folder with SK_* and BP_*), CAR_LIGHTS_LOG.

自作車両のウインカー（点滅）とミラーのマテリアル（UE4 エディタ Python）。
CARLA は "emissive" タグ付きコンポーネントの全マテリアルに "Left Blinker" /
"Right Blinker" を設定するだけなので、点滅はマテリアル側で行う。
"""

import os
import traceback

import unreal

PKG = os.environ.get("LMP_PKG", "UserCars")
HERE = os.path.dirname(os.path.abspath(__file__))
MAT_DIR = "/Game/%s/Materials" % PKG
CAR_DIR = os.environ["CAR_DIR"]


def names(var):
    return [g.strip().lower() for g in os.environ.get(var, "").split(",") if g.strip()]


LEFT, RIGHT, MIRROR = names("CAR_BLINK_L"), names("CAR_BLINK_R"), names("CAR_MIRROR")
EAL, MEL = unreal.EditorAssetLibrary, unreal.MaterialEditingLibrary
TOOLS = unreal.AssetToolsHelpers.get_asset_tools()
LOG = open(os.path.join(HERE, os.environ.get("CAR_LIGHTS_LOG", "make_lights.log")), "w")
AMBER = unreal.LinearColor(1.0, 0.33, 0.0, 1.0)


def log(*a):
    msg = " ".join(str(x) for x in a)
    LOG.write(msg + "\n")
    LOG.flush()
    unreal.log("make_lights: " + msg)


def new_material(name):
    mat = TOOLS.create_asset(name, MAT_DIR, unreal.Material, unreal.MaterialFactoryNew())
    mat.set_editor_property("two_sided", True)
    # on a vehicle (skeletal) mesh: without this flag the cooked game draws
    # the grey checker default material / スケルタル用フラグ（無いと市松模様）
    mat.set_editor_property("used_with_skeletal_mesh", True)
    return mat


def expr(mat, cls, x, y, **props):
    e = MEL.create_material_expression(mat, cls, x, y)
    for k, v in props.items():
        e.set_editor_property(k, v)
    return e


def link(a, b, pin=""):
    # single-input nodes (Frac, Round, Saturate) take "" or "Input" / 単入力ノード
    for p in ([pin] if pin else ["", "Input"]):
        if MEL.connect_material_expressions(a, "", b, p):
            return
    raise RuntimeError("could not connect %s -> %s.%s" % (a.get_name(), b.get_name(), pin))


def blinker_master():
    path = MAT_DIR + "/M_CarBlinker"
    if EAL.does_asset_exist(path):
        return EAL.load_asset(path)
    mat = new_material("M_CarBlinker")
    P = unreal.MaterialExpressionScalarParameter
    left = expr(mat, P, -1400, 0, parameter_name="Left Blinker", default_value=0.0)
    right = expr(mat, P, -1400, 120, parameter_name="Right Blinker", default_value=0.0)
    side = expr(mat, P, -1400, 240, parameter_name="Side", default_value=0.0)
    pick = expr(mat, unreal.MaterialExpressionLinearInterpolate, -1150, 100)
    link(left, pick, "A")
    link(right, pick, "B")
    link(side, pick, "Alpha")
    # CARLA sends the light's intensity (300, or 500000 on its own cars):
    # anything above 0 means on / 0 より大きければ点灯
    big = expr(mat, unreal.MaterialExpressionMultiply, -950, 100, const_b=1000.0)
    link(pick, big, "A")
    on = expr(mat, unreal.MaterialExpressionSaturate, -800, 100)
    link(big, on)
    # 1.5 Hz on/off square wave from the engine clock / 1.5 Hz の点滅
    t = expr(mat, unreal.MaterialExpressionTime, -1400, 400)
    rate = expr(mat, P, -1400, 500, parameter_name="Blink Rate", default_value=1.5)
    tr = expr(mat, unreal.MaterialExpressionMultiply, -1150, 420)
    link(t, tr, "A")
    link(rate, tr, "B")
    frac = expr(mat, unreal.MaterialExpressionFrac, -1000, 420)
    link(tr, frac)
    wave = expr(mat, unreal.MaterialExpressionRound, -850, 420)
    link(frac, wave)
    flash = expr(mat, unreal.MaterialExpressionMultiply, -650, 200)
    link(on, flash, "A")
    link(wave, flash, "B")
    colour = expr(mat, unreal.MaterialExpressionVectorParameter, -1400, -250,
                  parameter_name="Blinker Color", default_value=AMBER)
    # CARLA's own light master emits Blinker Color x 0.5 x intensity (500000 on
    # the 2020 cars) x sine: a glow of 40 was invisible in daylight (live test
    # 2026-11-04). Match CARLA's level / CARLA 純正と同じ明るさ（40 では見えない）
    glow = expr(mat, P, -1400, -120, parameter_name="Glow", default_value=250000.0)
    cg = expr(mat, unreal.MaterialExpressionMultiply, -1000, -200)
    link(colour, cg, "A")
    link(glow, cg, "B")
    emis = expr(mat, unreal.MaterialExpressionMultiply, -450, -100)
    link(cg, emis, "A")
    link(flash, emis, "B")
    MEL.connect_material_property(emis, "", unreal.MaterialProperty.MP_EMISSIVE_COLOR)
    # the lens when off: dim amber / 消灯時：暗いアンバーのレンズ
    lens = expr(mat, P, -1400, -380, parameter_name="Lens", default_value=0.35)
    base = expr(mat, unreal.MaterialExpressionMultiply, -1000, -330)
    link(colour, base, "A")
    link(lens, base, "B")
    MEL.connect_material_property(base, "", unreal.MaterialProperty.MP_BASE_COLOR)
    rough = expr(mat, unreal.MaterialExpressionConstant, -450, 120, r=0.2)
    MEL.connect_material_property(rough, "", unreal.MaterialProperty.MP_ROUGHNESS)
    MEL.recompile_material(mat)
    EAL.save_loaded_asset(mat)
    log("material", path)
    return mat


def blinker_instance(master, name, side):
    path = MAT_DIR + "/" + name
    if EAL.does_asset_exist(path):
        return EAL.load_asset(path)
    mi = TOOLS.create_asset(name, MAT_DIR, unreal.MaterialInstanceConstant,
                            unreal.MaterialInstanceConstantFactoryNew())
    MEL.set_material_instance_parent(mi, master)
    MEL.set_material_instance_scalar_parameter_value(mi, "Side", side)
    EAL.save_loaded_asset(mi)
    log("instance", path, "Side", side)
    return mi


def mirror_material():
    path = MAT_DIR + "/M_CarMirror"
    if EAL.does_asset_exist(path):
        return EAL.load_asset(path)
    mat = new_material("M_CarMirror")
    # dark polished glass: a near-white chrome reflects the sky capture and
    # reads as see-through / 明るすぎると空が映り透けて見えるので暗めに
    col = expr(mat, unreal.MaterialExpressionConstant3Vector, -500, 0,
               constant=unreal.LinearColor(0.10, 0.10, 0.11, 1.0))
    MEL.connect_material_property(col, "", unreal.MaterialProperty.MP_BASE_COLOR)
    metal = expr(mat, unreal.MaterialExpressionConstant, -500, 150, r=1.0)
    MEL.connect_material_property(metal, "", unreal.MaterialProperty.MP_METALLIC)
    rough = expr(mat, unreal.MaterialExpressionConstant, -500, 250, r=0.15)
    MEL.connect_material_property(rough, "", unreal.MaterialProperty.MP_ROUGHNESS)
    MEL.recompile_material(mat)
    EAL.save_loaded_asset(mat)
    log("material", path)
    return mat


def tag_emissive():
    """CARLA sets the light parameters only on components tagged "emissive".
    BP の車体メッシュに "emissive" タグを付ける。"""
    bps = [p for p in EAL.list_assets(CAR_DIR, recursive=False)
           if EAL.find_asset_data(p).asset_class == "Blueprint"
           and str(EAL.find_asset_data(p).asset_name).startswith("BP_")
           and not str(EAL.find_asset_data(p).asset_name)[-4:] in ("_FLW", "_FRW", "_RLW", "_RRW")]
    if len(bps) != 1:
        raise RuntimeError("expected one vehicle blueprint in %s, got %s" % (CAR_DIR, bps))
    cls = EAL.load_blueprint_class(bps[0])
    comp = unreal.get_default_object(cls).get_editor_property("mesh")
    tags = list(comp.get_editor_property("component_tags"))
    if "emissive" not in [str(t) for t in tags]:
        tags.append("emissive")
        comp.set_editor_property("component_tags", tags)
        EAL.save_asset(bps[0], only_if_is_dirty=False)
    log("blueprint", bps[0], "mesh tags", [str(t) for t in comp.get_editor_property("component_tags")])


try:
    meshes = [EAL.load_asset(p) for p in EAL.list_assets(CAR_DIR, recursive=False)
              if EAL.find_asset_data(p).asset_class == "SkeletalMesh"]
    if len(meshes) != 1:
        raise RuntimeError("expected one skeletal mesh in %s, got %d" % (CAR_DIR, len(meshes)))
    mesh = meshes[0]
    log("mesh", mesh.get_path_name())
    swap = {}
    if LEFT or RIGHT:
        master = blinker_master()
        swap.update({n: blinker_instance(master, "MI_CarBlinkerL", 0.0) for n in LEFT})
        swap.update({n: blinker_instance(master, "MI_CarBlinkerR", 1.0) for n in RIGHT})
    if MIRROR:
        swap.update({n: mirror_material() for n in MIRROR})
    done, slots = [], []
    for s in mesh.get_editor_property("materials"):
        name = str(s.get_editor_property("material_slot_name"))
        if name.lower() in swap:
            # the slot comes back as a copy: swap its material, keep its names
            # スロットはコピーなので、マテリアルだけ差し替える
            s.set_editor_property("material_interface", swap[name.lower()])
            done.append(name)
        slots.append(s)
    mesh.set_editor_property("materials", slots)
    EAL.save_loaded_asset(mesh)
    log("light slots", done)
    missing = sorted(set(swap) - {d.lower() for d in done})
    if missing:
        raise RuntimeError("slots not found %s in %s" % (missing, [
            str(s.get_editor_property("material_slot_name")) for s in mesh.get_editor_property("materials")]))
    if LEFT or RIGHT:
        tag_emissive()
    log("OK")
except Exception:
    log("FAILED\n" + traceback.format_exc())
LOG.close()
