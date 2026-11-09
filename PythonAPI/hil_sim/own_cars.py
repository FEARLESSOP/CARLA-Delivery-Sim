"""The user's own vehicles in cars/cars.json, as blueprint ids.

Plain Python (no CARLA). Vehicles marked ``"npc_only": true`` (the Oita bus,
car_guide ch. 16) are driven only by the scripts made for them: they are
never parked (parked_cars.py), never picked for random traffic
(generate_traffic.py), never offered as "Your car" (launch_options.catalog)
and never a random or cycled hero model (manual_control_steeringwheel).

cars/cars.json の自作車両（ブループリント ID）。npc_only の車両（大分バス）は
専用スクリプトだけが動かし、駐車・一般交通・自車の選択肢には出さない。
"""
import json
import os

HERE = os.path.dirname(os.path.abspath(__file__))
CARS_JSON = os.path.join(HERE, "cars", "cars.json")


def load(path=CARS_JSON):
    """cars.json entries {key: entry} (notes and broken files skipped)."""
    try:
        with open(path) as fh:
            cfg = json.load(fh)
    except (OSError, ValueError):
        return {}
    return {k: v for k, v in cfg.items()
            if not k.startswith("_") and isinstance(v, dict) and v.get("make") and v.get("model")}


def blueprint_id(entry):
    return "vehicle.%s.%s" % (entry["make"], entry["model"])


def user_car_ids(path=CARS_JSON, npc=False):
    """Blueprint ids of the user's cars; ``npc`` True includes npc_only ones."""
    return {blueprint_id(e) for e in load(path).values() if npc or not e.get("npc_only")}


def npc_only_ids(path=CARS_JSON):
    """Blueprint ids of the npc_only vehicles. / npc_only 車両の ID"""
    return {blueprint_id(e) for e in load(path).values() if e.get("npc_only")}
