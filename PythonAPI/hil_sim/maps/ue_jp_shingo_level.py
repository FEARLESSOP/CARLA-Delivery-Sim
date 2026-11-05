"""Unreal Editor step after a Japan-map import (build_map.sh, PythonScript
commandlet): make the map spawn BP_JpShingo instead of CARLA's US light.

  SHINGO_PKG=Japan SHINGO_MAP=OitaKosen UE4Editor ... -run=pythonscript -script=<this>
  (SHINGO_LEVEL=/Game/.../Level overrides the level path; SHINGO_VERIFY=1 only
  reads the level and reports, saves nothing)

How CARLA picks the model: at BeginPlay the game mode takes the level's
ATrafficLightManager (or spawns one with the C++ defaults
BP_TLOpenDrive_RHT/_LHT), and TrafficLightManager::SpawnTrafficLights spawns
TrafficLightModel_RHT (or _LHT on rule="LHT" roads) at every OpenDRIVE
traffic-light signal, then registers it with its controller and group. Both
properties are EditAnywhere, so a manager actor saved in the level with both
set to BP_JpShingo_C makes every light of this map Japanese, while other maps
(no manager in the level) keep CARLA's. TrafficLightsGenerated stays false, so
the lights are still generated from the xodr at run time: controllers,
timing, freeze and signal_timing.py work unchanged.

取り込み後の UE 処理: マップに TrafficLightManager を置き、信号モデルを BP_JpShingo にする。
"""

import os

import unreal

PKG = os.environ.get("SHINGO_PKG", "Japan")
NAME = os.environ.get("SHINGO_MAP", "")
LEVEL = os.environ.get("SHINGO_LEVEL") or "/Game/%s/Maps/%s/%s" % (PKG, NAME, NAME)
VERIFY = os.environ.get("SHINGO_VERIFY") == "1"
BP = "/Game/%s/Shingo/BP_JpShingo" % PKG
lib = unreal.EditorAssetLibrary


def log(s):
    unreal.log_warning("[ue_jp_shingo_level] " + str(s))


def main():
    cls = lib.load_blueprint_class(BP)
    if cls is None:
        raise RuntimeError("no %s: run ue_jp_shingo.py first" % BP)
    if not unreal.EditorLoadingAndSavingUtils.load_map(LEVEL):
        raise RuntimeError("cannot load " + LEVEL)
    mgrs = [a for a in unreal.EditorLevelLibrary.get_all_level_actors() if isinstance(a, unreal.TrafficLightManager)]
    if VERIFY:
        for m in mgrs:
            log("manager %s: RHT=%s LHT=%s generated=%s" % (
                m.get_name(), m.get_editor_property("TrafficLightModel_RHT").get_name(),
                m.get_editor_property("TrafficLightModel_LHT").get_name(),
                m.get_editor_property("TrafficLightsGenerated")))
        ok = len(mgrs) == 1 and all(m.get_editor_property(p) == cls for m in mgrs
                                    for p in ("TrafficLightModel_RHT", "TrafficLightModel_LHT"))
        log("%s: %d manager(s), Japanese model %s" % (LEVEL, len(mgrs), "SET" if ok else "MISSING"))
        return
    for extra in mgrs[1:]:                      # one manager per level / 1 レベルに 1 つ
        unreal.EditorLevelLibrary.destroy_actor(extra)
    if mgrs:
        mgr = mgrs[0]
    else:
        # EditorLevelLibrary.spawn_actor_* needs a viewport (segfaults in a
        # commandlet); CARLA's own helper spawns straight into the editor world
        mgr = unreal.OpenDriveToMap.spawn_actor_with_check_no_collisions(
            unreal.TrafficLightManager.static_class(), unreal.Transform())
    mgr.set_editor_property("TrafficLightModel_RHT", cls)
    mgr.set_editor_property("TrafficLightModel_LHT", cls)
    mgr.set_editor_property("TrafficLightsGenerated", False)
    mgr.set_actor_label("JpShingoTrafficManager")
    if not unreal.EditorLevelLibrary.save_current_level():
        raise RuntimeError("level save failed: " + LEVEL)
    log("%s: TrafficLightManager %s spawns %s (RHT and LHT)" % (LEVEL, mgr.get_name(), cls.get_name()))


main()
