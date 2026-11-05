"""Unreal Editor step after a Japan-map import (build_map.sh step 2e,
PythonScript commandlet): make the map spawn BP_JpSokudo<V> (blue-numeral
最高速度 posts) for its OpenDRIVE speed-limit signals instead of CARLA's.

  SOKUDO_PKG=Japan SOKUDO_MAP=OitaKosen UE4Editor ... -run=pythonscript -script=<this>
  (SOKUDO_LEVEL=/Game/.../Level overrides the level path; SOKUDO_VERIFY=1 only
  reads the level and reports, saves nothing)

How CARLA picks the model: TrafficLightManager::SpawnSignals spawns, for a
type-274 signal, SpeedLimitModels[<signal subtype>] (C++ defaults "30".."120"
-> /Game/Carla/Static/TrafficSign/BP_SpeedLimit<V>; a subtype with no entry
spawns nothing), then adds a USpeedLimitComponent with the signal's value
and lane trigger boxes. SpeedLimitModels is an EditAnywhere map on the
manager, so the manager saved in this level (ue_jp_shingo_level.py puts it
there; spawned here if that step is off) gets our keys "20".."80" ->
BP_JpSokudo<V>_C. Keys we do not build (90-120) keep CARLA's. Other maps
are untouched. Our signs.py writes subtype == value (km/h).

取り込み後の UE 処理: マップの TrafficLightManager の速度標識モデルを BP_JpSokudo にする。
"""

import os

import unreal

PKG = os.environ.get("SOKUDO_PKG", "Japan")
NAME = os.environ.get("SOKUDO_MAP", "")
LEVEL = os.environ.get("SOKUDO_LEVEL") or "/Game/%s/Maps/%s/%s" % (PKG, NAME, NAME)
VERIFY = os.environ.get("SOKUDO_VERIFY") == "1"
DEST = "/Game/%s/Sokudo" % PKG
lib = unreal.EditorAssetLibrary


def log(s):
    unreal.log_warning("[ue_jp_sokudo_level] " + str(s))


def ours():
    out = {}
    for p in lib.list_assets(DEST, recursive=False):
        n = p.split("/")[-1].split(".")[0]
        if n.startswith("BP_JpSokudo") and n[11:].isdigit():
            out[n[11:]] = lib.load_blueprint_class("%s/%s" % (DEST, n))
    return out


def models(mgr):
    return {str(k): (v.get_name() if v else None) for k, v in mgr.get_editor_property("SpeedLimitModels").items()}


def main():
    want = ours()
    if not want or None in want.values():
        raise RuntimeError("no BP_JpSokudo<V> in %s: run ue_jp_sokudo.py first" % DEST)
    # load_map of a missing level returns a fresh untitled world: check the file first
    # 存在しないレベルを load_map すると無題のワールドになるので先にファイルを確認
    umap = os.path.join(unreal.Paths.project_content_dir(), LEVEL[len("/Game/"):] + ".umap")
    if not os.path.isfile(umap) or not unreal.EditorLoadingAndSavingUtils.load_map(LEVEL):
        raise RuntimeError("cannot load " + LEVEL)
    mgrs = [a for a in unreal.EditorLevelLibrary.get_all_level_actors() if isinstance(a, unreal.TrafficLightManager)]
    if VERIFY:
        for m in mgrs:
            log("manager %s: SpeedLimitModels %s" % (m.get_name(), models(m)))
        ok = len(mgrs) == 1 and all(models(mgrs[0]).get(k) == c.get_name() for k, c in want.items())
        log("%s: %d manager(s), Japanese speed posts %s" % (LEVEL, len(mgrs), "SET" if ok else "MISSING"))
        return
    for extra in mgrs[1:]:                      # one manager per level / 1 レベルに 1 つ
        unreal.EditorLevelLibrary.destroy_actor(extra)
    if mgrs:
        mgr = mgrs[0]
    else:
        # no signal step: a manager with CARLA's default lights / 信号ステップ無し: 既定の信号のまま
        mgr = unreal.OpenDriveToMap.spawn_actor_with_check_no_collisions(
            unreal.TrafficLightManager.static_class(), unreal.Transform())
        mgr.set_editor_property("TrafficLightsGenerated", False)
        mgr.set_actor_label("JpSokudoTrafficManager")
    m = mgr.get_editor_property("SpeedLimitModels")
    for k, c in sorted(want.items()):
        m[k] = c
    mgr.set_editor_property("SpeedLimitModels", m)
    got = models(mgr)
    if any(got.get(k) != c.get_name() for k, c in want.items()):
        raise RuntimeError("SpeedLimitModels not set: %s" % got)
    if not unreal.EditorLevelLibrary.save_current_level():
        raise RuntimeError("level save failed: " + LEVEL)
    log("%s: TrafficLightManager %s speed models %s" % (LEVEL, mgr.get_name(), got))
    log("speed posts set: %s" % ",".join(sorted(want, key=int)))


main()
