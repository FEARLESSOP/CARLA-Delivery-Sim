"""Offline test of BP_JpSokudo<V> (Unreal PythonScript commandlet, saves nothing).

  cd <CARLA_SRC>/Unreal/CarlaUE4 && SOKUDO_PKG=Japan UE4Editor $PWD/CarlaUE4.uproject \\
    -run=pythonscript -script=<this> -unattended -nosplash -nullrhi
  -> "[jp_sokudo_test] PASS n/n" or "FAIL ..." lines

Loads CARLA's small OpenDriveMap (not saved) and checks, per value V:
  - BP_JpSokudo<V> is a CARLA ATrafficSignBase (SpawnSignals spawns
    SpawnActor<ATrafficSignBase>, so anything else would be null) with
    CARLA's TrafficSignState for V (type id traffic.speed_limit.V);
  - its only mesh is SM_JpSokudo<V>: slots pole / back / MI_JpSokudoFace<V>
    whose "Face" texture is T_JpSokudo<V>; plate faces +Y like CARLA's
    SM_SpeedSign40, plate bottom 2.0 m over the origin (1.85 m over a 0.15 m
    pavement), collision present;
  - a TrafficLightManager given the level step's mapping picks
    BP_JpSokudo<V> for subtype "V" and keeps CARLA's 90-120; the speed value
    itself is added by SpawnSignals (USpeedLimitComponent = signal value), not
    by the blueprint, so the class choice is the only thing that changes;
  - every Sokudo asset name is free of "sign"/"light".

BP_JpSokudo のオフライン試験（保存なし）。
"""

import os

import unreal

PKG = os.environ.get("SOKUDO_PKG", "Japan")
DEST = "/Game/%s/Sokudo" % PKG
STOCK = "/Game/Carla/Static/TrafficSign"
lib = unreal.EditorAssetLibrary
results = []


def check(name, ok, detail=""):
    results.append(bool(ok))
    unreal.log_warning("[jp_sokudo_test] %s %s %s" % ("ok  " if ok else "FAIL", name, detail))


def face_dir(mesh):
    """+1 if the plate (its widest part) sits on +Y of the pole, -1 on -Y, 0 for X."""
    bb = mesh.get_bounding_box()
    ext = {"x": bb.max.x - bb.min.x, "y": bb.max.y - bb.min.y}
    if ext["y"] > ext["x"]:
        return 0
    return 1 if abs(bb.max.y) > abs(bb.min.y) else -1


def main():
    unreal.EditorLoadingAndSavingUtils.load_map("/Game/Carla/Maps/OpenDriveMap")
    vals = sorted(int(p.split("/")[-1].split(".")[0][11:]) for p in lib.list_assets(DEST, recursive=False)
                  if p.split("/")[-1].startswith("BP_JpSokudo"))
    check("blueprints 20..80 present", set(vals) >= {20, 30, 40, 50, 60, 70, 80}, vals)
    # CARLA's plate hangs on the +Y side of its pole (SM_SpeedSign40 at y +3.25 cm): faces +Y
    # CARLA の板は支柱の +Y 側（+Y 向き）
    st = unreal.OpenDriveToMap.spawn_actor_with_check_no_collisions(
        lib.load_blueprint_class(STOCK + "/BP_SpeedLimit40"), unreal.Transform())
    plate = [c for c in st.get_components_by_class(unreal.StaticMeshComponent) if c.get_name() == "SM_SpeedSign40"]
    sf = (1 if plate[0].get_editor_property("relative_location").y > 0 else -1) if plate else None
    check("CARLA's plate faces +Y", sf == 1, sf)
    classes = {}
    for v in vals:
        cls = lib.load_blueprint_class("%s/BP_JpSokudo%d" % (DEST, v))
        classes[str(v)] = cls
        a = unreal.OpenDriveToMap.spawn_actor_with_check_no_collisions(cls, unreal.Transform())
        check("%d: is a CARLA TrafficSignBase" % v, isinstance(a, unreal.TrafficSignBase), a.get_class().get_name())
        sp = "%s/BP_SpeedLimit%d" % (STOCK, v)
        want = unreal.get_default_object(lib.load_blueprint_class(sp)).get_editor_property("TrafficSignState") \
            if lib.does_asset_exist(sp) else unreal.TrafficSignState.UNKNOWN
        check("%d: TrafficSignState as CARLA's" % v, a.get_editor_property("TrafficSignState") == want,
              str(a.get_editor_property("TrafficSignState")))
        comps = [c for c in a.get_components_by_class(unreal.StaticMeshComponent) if c.static_mesh]
        ok = len(comps) == 1 and comps[0].static_mesh.get_name() == "SM_JpSokudo%d" % v
        check("%d: only mesh SM_JpSokudo%d" % (v, v), ok, [c.static_mesh.get_name() for c in comps])
        if not ok:
            continue
        c = comps[0]
        mats = [c.get_material(i).get_name() if c.get_material(i) else None for i in range(c.get_num_materials())]
        check("%d: materials" % v, mats == ["M_JpSokudoPole", "M_JpSokudoBack", "MI_JpSokudoFace%d" % v], mats)
        mi = c.get_material(2)
        tex = unreal.MaterialEditingLibrary.get_material_instance_texture_parameter_value(mi, "Face") \
            if isinstance(mi, unreal.MaterialInstanceConstant) else None
        check("%d: face texture T_JpSokudo%d" % (v, v), tex is not None and tex.get_name() == "T_JpSokudo%d" % v,
              tex.get_name() if tex else None)
        bb = c.static_mesh.get_bounding_box()
        check("%d: plate faces +Y like CARLA's model" % v, face_dir(c.static_mesh) == sf == 1,
              "ours %d stock %s" % (face_dir(c.static_mesh), sf))
        check("%d: plate 60 cm, bottom 200 cm, top 260 cm" % v, abs(bb.max.x - bb.min.x - 60) < 1.5 and 255 < bb.max.z < 270,
              "x %.1f..%.1f z %.1f..%.1f" % (bb.min.x, bb.max.x, bb.min.z, bb.max.z))
        body = c.static_mesh.get_editor_property("body_setup")
        check("%d: has collision, blocks like CARLA's post" % v,
              body is not None and str(c.get_collision_profile_name()) == "BlockAll", str(c.get_collision_profile_name()))
        empty = [x for x in a.get_components_by_class(unreal.StaticMeshComponent) if x.get_name() == "SM_SpeedSign40"]
        check("%d: CARLA's plate component empty, hidden, no collision" % v,
              len(empty) == 1 and empty[0].static_mesh is None and not empty[0].is_visible()
              and str(empty[0].get_collision_profile_name()) == "NoCollision", "")
        loc = c.get_editor_property("relative_location")
        check("%d: mesh at the actor origin" % v, loc.is_near_equal(unreal.Vector(0, 0, 0), 0.1), str(loc))

    # the class SpawnSignals picks for subtype "V" / SpawnSignals が選ぶクラス
    mgr = unreal.OpenDriveToMap.spawn_actor_with_check_no_collisions(
        unreal.TrafficLightManager.static_class(), unreal.Transform())
    m = mgr.get_editor_property("SpeedLimitModels")
    before = {str(k): v.get_name() for k, v in m.items()}
    for k, c in classes.items():
        m[k] = c
    mgr.set_editor_property("SpeedLimitModels", m)
    after = {str(k): v.get_name() for k, v in mgr.get_editor_property("SpeedLimitModels").items()}
    check("manager: subtype V -> BP_JpSokudoV_C", all(after.get(k) == "BP_JpSokudo%s_C" % k for k in classes), after)
    kept = {k: after.get(k) for k in before if k not in classes}
    check("manager: CARLA's 90-120 kept", all(after.get(k) == before[k] for k in kept) and "90" in kept, kept)

    bad = [p for p in lib.list_assets(DEST, recursive=True)
           if any(w in p.split("/")[-1].split(".")[0].lower() for w in ("sign", "light"))]
    check("no 'sign'/'light' in asset names", not bad, bad)
    n = sum(results)
    unreal.log_warning("[jp_sokudo_test] %s %d/%d" % ("PASS" if n == len(results) else "FAIL", n, len(results)))


main()
