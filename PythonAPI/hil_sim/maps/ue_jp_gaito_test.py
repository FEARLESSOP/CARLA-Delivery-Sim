"""Offline UE test of BP_JpGaito (ue_jp_gaito.py): spawn it in a level that is
not saved, check the parts, switch it on and off like the LightManager does.

  cd <CARLA_SRC>/Unreal/CarlaUE4 && GAITO_PKG=Japan GAITO_MAP=OitaKosen UE4Editor $PWD/CarlaUE4.uproject \\
    -run=pythonscript -script=<this> -unattended -nosplash -nullrhi

街灯 BP のオフライン確認（保存しないレベルに出して点灯・消灯）。
"""

import os

import unreal

PKG = os.environ.get("GAITO_PKG", "Japan")
NAME = os.environ.get("GAITO_MAP", "OitaKosen")
BP = "/Game/%s/Gaito/BP_JpGaito" % PKG
FAILS, N = [], [0]


def check(name, ok, detail=""):
    N[0] += 1
    unreal.log_warning("[jp_gaito_test] %s %s %s" % ("PASS" if ok else "FAIL", name, detail))
    if not ok:
        FAILS.append(name)


unreal.EditorLoadingAndSavingUtils.load_map("/Game/%s/Maps/%s/%s" % (PKG, NAME, NAME))
cls = unreal.EditorAssetLibrary.load_blueprint_class(BP)
check("blueprint exists", cls is not None)
a = unreal.OpenDriveToMap.spawn_actor_with_check_no_collisions(
    cls, unreal.Transform(unreal.Vector(0, 0, 60000), unreal.Rotator(0, 0, 30), unreal.Vector(1, 1, 1)))
check("spawns", a is not None)
mesh = [c for c in a.get_components_by_class(unreal.StaticMeshComponent)][0]
spot = [c for c in a.get_components_by_class(unreal.SpotLightComponent)][0]
point = [c for c in a.get_components_by_class(unreal.PointLightComponent) if not isinstance(c, unreal.SpotLightComponent)][0]
cl = [c for c in a.get_components_by_class(unreal.CarlaLight)]
check("our glowing plate as the lamp mesh", mesh.static_mesh.get_name() == "SM_JpGaitoGlow", mesh.static_mesh.get_name())
check("plate has no collision", mesh.get_collision_profile_name() == "NoCollision", mesh.get_collision_profile_name())
fwd = spot.get_forward_vector()
check("spot points straight down", fwd.z < -0.99, str(fwd))
check("lights are Movable (no baked lighting on generated maps)",
      spot.get_editor_property("mobility") == unreal.ComponentMobility.MOVABLE
      and point.get_editor_property("mobility") == unreal.ComponentMobility.MOVABLE)
check("no shadows", not spot.get_editor_property("cast_shadows") and not point.get_editor_property("cast_shadows"))
check("not drawn beyond 80 m", 0 < spot.get_editor_property("max_draw_distance") <= 8000.0)
check("one CarlaLight in the Street group", len(cl) == 1 and cl[0].get_light_type() == unreal.LightType.STREET,
      [c.get_light_type() for c in cl])
if cl:
    L = cl[0]
    check("off at spawn", not spot.is_visible() and spot.get_editor_property("intensity") == 0.0)
    L.register_light()
    L.set_light_on(True)
    check("on: spot lit", spot.is_visible() and spot.get_editor_property("intensity") > 1000.0,
          spot.get_editor_property("intensity"))
    check("on: plate glows (dynamic lamp material)", isinstance(mesh.get_material(0), unreal.MaterialInstanceDynamic),
          mesh.get_material(0).get_name())
    L.set_light_on(False)
    check("off again", not spot.is_visible())
bad = [p for p in unreal.EditorAssetLibrary.list_assets("/Game/%s/Gaito" % PKG, recursive=True)
       if "light" in p.lower().split("/")[-1] or "sign" in p.lower().split("/")[-1]]
check("no 'light'/'sign' in our asset names", not bad, bad)
unreal.log_warning("[jp_gaito_test] %s %d/%d" % ("PASS" if not FAILS else "FAIL", N[0] - len(FAILS), N[0]))
