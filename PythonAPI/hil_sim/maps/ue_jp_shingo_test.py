"""Offline test of BP_JpShingo (Unreal PythonScript commandlet, saves nothing).

  cd <CARLA_SRC>/Unreal/CarlaUE4 && SHINGO_PKG=Japan UE4Editor $PWD/CarlaUE4.uproject \\
    -run=pythonscript -script=<this> -unattended -nosplash -nullrhi
  -> "[jp_shingo_test] PASS n/n" or "FAIL ..." lines

Loads CARLA's small OpenDriveMap (not saved), spawns BP_JpShingo the way the
TrafficLightManager does (World->SpawnActor), and checks:
  - it is a CARLA TrafficLightBase with a TrafficLightComponent;
  - pole/arm components carry our meshes, the ground base is gone;
  - the construction script made 3 lamp components (SM_JpShingoLamp) with a
    dynamic M_StreetlightsMaster_T10 on slot 1, left to right green, yellow,
    red (x = -35, 0, +35 cm on the head), facing +Y like CARLA's model;
  - set_traffic_light_state(Red/Yellow/Green) (what the server, the
    controllers and signal_timing.py end up calling) turns On/Off = 1 on
    exactly the matching lamp, 0 on the others;
  - every Shingo asset name is free of "sign"/"light".

BP_JpShingo のオフライン試験（保存なし）。
"""

import os

import unreal

PKG = os.environ.get("SHINGO_PKG", "Japan")
DEST = "/Game/%s/Shingo" % PKG
lib = unreal.EditorAssetLibrary
results = []


def check(name, ok, detail=""):
    results.append(bool(ok))
    unreal.log_warning("[jp_shingo_test] %s %s %s" % ("ok  " if ok else "FAIL", name, detail))


def onoff(mid):
    return round(mid.get_scalar_parameter_value("On/Off"), 2)


def main():
    unreal.EditorLoadingAndSavingUtils.load_map("/Game/Carla/Maps/OpenDriveMap")
    cls = lib.load_blueprint_class(DEST + "/BP_JpShingo")
    check("blueprint class loads", cls is not None)
    a = unreal.OpenDriveToMap.spawn_actor_with_check_no_collisions(cls, unreal.Transform())
    check("is a CARLA TrafficLightBase", isinstance(a, unreal.TrafficLightBase), a.get_class().get_name())
    tlc = a.get_editor_property("TrafficLightComponent")
    check("has a TrafficLightComponent", isinstance(tlc, unreal.TrafficLightComponent))

    comps = {c.get_name(): c for c in a.get_components_by_class(unreal.StaticMeshComponent)}
    mesh = lambda n: comps[n].static_mesh.get_name() if n in comps and comps[n].static_mesh else None
    check("pole mesh", mesh("SM_TrafficLight__VPole_Main") == "SM_JpShingoPole", mesh("SM_TrafficLight__VPole_Main"))
    check("arm mesh", mesh("SM_TrafficLight__HPole01_Long1") == "SM_JpShingoArm", mesh("SM_TrafficLight__HPole01_Long1"))
    check("no ground base", mesh("SM_TrafficLight__Ground_base") is None)
    for n in ("SM_TrafficLight__VPole_Main", "SM_TrafficLight__HPole01_Long1"):
        if n in comps:     # real-size meshes: the stock 1.2 pole scale lifts the arm off the head
            sc = comps[n].get_world_scale()
            check("%s at scale 1 (arm meets the head clamps)" % n,
                  max(abs(sc.x - 1), abs(sc.y - 1), abs(sc.z - 1)) < 1e-3, str(sc))
    heads = [c for c in comps.values() if c.static_mesh and c.static_mesh.get_name() == "SM_JpShingoHead"]
    check("one head body", len(heads) == 1)
    if heads:
        loc = heads[0].get_editor_property("relative_location")
        check("head 6 m out along -X, above 5 m", loc.x < -500 and loc.z > 500, str(loc))

    lists = {n: list(a.get_editor_property(n)) for n in ("GreenLights", "YellowLights", "RedLights")}
    check("one lamp per colour", all(len(v) == 1 for v in lists.values()), {k: len(v) for k, v in lists.items()})
    xs = {}
    for n, v in lists.items():
        if not v:
            continue
        mid = v[0]
        comp = mid.get_outer()
        ok = (isinstance(mid, unreal.MaterialInstanceDynamic) and isinstance(comp, unreal.StaticMeshComponent)
              and comp.static_mesh.get_name() == "SM_JpShingoLamp" and comp.get_material(1) == mid
              and mid.get_editor_property("parent").get_name() == "M_StreetlightsMaster_T10")
        check("%s: lamp mesh, dynamic lamp material on slot 1" % n, ok,
              "%s / %s" % (comp.get_name(), mid.get_editor_property("parent").get_name()))
        check("%s: housing material on slot 0" % n, comp.get_material(0).get_name() == "M_JpShingoHood",
              comp.get_material(0).get_name())
        xs[n] = comp.get_editor_property("relative_location").x
    lamp = lib.load_asset(DEST + "/SM_JpShingoLamp")
    bb = lamp.get_bounding_box()
    check("lamp hood opens towards +Y (the approaching driver)", bb.max.y > 20 and bb.min.y > -1, "y %.1f..%.1f cm" % (bb.min.y, bb.max.y))
    check("left to right green, yellow, red", xs.get("GreenLights", 0) < xs.get("YellowLights", 0) < xs.get("RedLights", 0), xs)

    states = [(unreal.TrafficLightState.RED, "RedLights"), (unreal.TrafficLightState.YELLOW, "YellowLights"),
              (unreal.TrafficLightState.GREEN, "GreenLights"), (unreal.TrafficLightState.RED, "RedLights")]
    for st, on in states:
        a.set_traffic_light_state(st)
        vals = {n: [onoff(m) for m in v] for n, v in lists.items()}
        good = all(vals[n] == ([1.0] if n == on else [0.0]) for n in vals) and a.get_traffic_light_state() == st
        check("state %s lights only %s" % (st.name, on), good, vals)

    bad = [p for p in lib.list_assets(DEST, recursive=True)
           if any(w in p.split("/")[-1].split(".")[0].lower() for w in ("sign", "light"))]
    check("no 'sign'/'light' in asset names", not bad, bad)
    n = sum(results)
    unreal.log_warning("[jp_shingo_test] %s %d/%d" % ("PASS" if n == len(results) else "FAIL", n, len(results)))


main()
