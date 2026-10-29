"""Remove all collision from every delivery prop (UE4 editor Python).

CARLA's PrepareAssetsForCooking sets every prop to "complex as simple", which
makes the whole visible mesh solid. The pod rides attached to a hidden car and
the drone is teleported every tick, so any collision shoves the car around
(it tipped the car on its side). No simple shapes + "simple as complex" means
the mesh has no collision at all.

Run between PrepareAssetsForCooking and the cook (build_models.sh does this):
  UE4Editor CarlaUE4.uproject -run=pythonscript -script=strip_collision.py

配送プロップの衝突を全て削除する（UE4 エディタ Python）。CARLA の
PrepareAssetsForCooking は全メッシュを "complex as simple" にするため、
ポッドが隠れた車を押して横転させた。単純形状なし＋"simple as complex" で衝突なし。
"""

import os

import unreal

ROOT = "/Game/%s/Static" % os.environ.get("LMP_PKG", "DeliveryModels")

for path in unreal.EditorAssetLibrary.list_assets(ROOT, recursive=True):
    mesh = unreal.EditorAssetLibrary.load_asset(path)
    if not isinstance(mesh, unreal.StaticMesh):
        continue
    unreal.EditorStaticMeshLibrary.remove_collisions(mesh)
    body = mesh.get_editor_property("body_setup")
    body.set_editor_property("collision_trace_flag",
                             unreal.CollisionTraceFlag.CTF_USE_SIMPLE_AS_COMPLEX)
    mesh.modify()
    unreal.EditorAssetLibrary.save_asset(path, only_if_is_dirty=False)
    unreal.log("strip_collision: %s -> %d simple shapes" % (
        path, unreal.EditorStaticMeshLibrary.get_simple_collision_count(mesh)))
