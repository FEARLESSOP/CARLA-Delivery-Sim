"""Export every VehicleFactory vehicle's skeletal mesh to FBX (UE4 editor Python).

  UE4Editor CarlaUE4.uproject -run=pythonscript -script=export_carla_vehicles.py
  (SURVEY_OUT = output folder)

Writes <make>.<model>.fbx and vehicles.json (blueprint id -> mesh path,
material slots, base type, wheels). carla_eyes.py measures the driver seat
from these. / VehicleFactory の全車両メッシュを FBX に書き出す。
"""

import json
import os
import traceback

import unreal

OUT = os.environ["SURVEY_OUT"]
FACTORY = "/Game/Carla/Blueprints/Vehicles/VehicleFactory"
EAL = unreal.EditorAssetLibrary
LOG_PATH = os.path.join(OUT, "export.log")
# a "try" line with no result after it = the editor died on that vehicle: skip it
# 結果のない "try" 行 = その車両で落ちた。次回は飛ばす
CRASHED = set()
if os.path.exists(LOG_PATH):
    last = None
    for line in open(LOG_PATH):
        w = line.split()
        if len(w) >= 2 and w[0] == "try":
            last = w[1]
        elif w:
            last = None
        if w and w[0] == "crashed":
            CRASHED.add(w[1])
    if last:
        CRASHED.add(last)
LOG = open(LOG_PATH, "a")
for c in CRASHED:
    LOG.write("crashed %s\n" % c)


def log(*a):
    LOG.write(" ".join(str(x) for x in a) + "\n")
    LOG.flush()


# the editor can crash part way (seen after ~28 vehicles): keep what was
# done and skip it on the next run / 途中で落ちても続きから再開できる
JSON = os.path.join(OUT, "vehicles.json")
info = json.load(open(JSON)) if os.path.exists(JSON) else {}
try:
    cdo = unreal.get_default_object(EAL.load_blueprint_class(FACTORY))
    for v in cdo.get_editor_property("vehicles"):
        bp_id = "vehicle.%s.%s" % (v.get_editor_property("make"), v.get_editor_property("model"))
        try:
            cls = v.get_editor_property("class_")
            vcdo = unreal.get_default_object(cls)
            mesh = vcdo.get_editor_property("mesh").get_editor_property("skeletal_mesh")
            slots = [str(m.get_editor_property("material_slot_name"))
                     for m in mesh.get_editor_property("materials")]
            fbx = os.path.join(OUT, bp_id + ".fbx")
            if bp_id in info and info[bp_id].get("exported") and os.path.exists(fbx):
                continue
            if bp_id in CRASHED:
                info[bp_id] = {"exported": False, "crashed": True}
                continue
            log("try", bp_id)
            task = unreal.AssetExportTask()
            task.set_editor_property("object", mesh)
            task.set_editor_property("filename", fbx)
            task.set_editor_property("automated", True)
            task.set_editor_property("replace_identical", True)
            task.set_editor_property("prompt", False)
            opt = unreal.FbxExportOption()
            opt.set_editor_property("collision", False)
            task.set_editor_property("options", opt)
            ok = unreal.Exporter.run_asset_export_task(task)
            info[bp_id] = {"mesh": mesh.get_path_name(), "slots": slots, "exported": bool(ok),
                           "base_type": str(v.get_editor_property("base_type")),
                           "wheels": int(v.get_editor_property("number_of_wheels"))}
            log(bp_id, "ok" if ok else "EXPORT FAILED", mesh.get_path_name())
            json.dump(info, open(JSON, "w"), indent=2)
        except Exception as e:
            log(bp_id, "ERROR", e)
    json.dump(info, open(os.path.join(OUT, "vehicles.json"), "w"), indent=2)
    log("OK", len(info))
except Exception:
    log("FAILED\n" + traceback.format_exc())
LOG.close()
