"""Write lmh_parts.json, the contract for the LastMile Hub LMH-3 and its Mini Pods.

Reads lmh_measured.json (prepare_lastmilehub.py: pivots and sizes measured on
the model) and adds what the model can't tell: the motions of the web
viewer's "Mini pods" tab (newer site copy, js_3d_lastmile/mini_hub.js +
mini.js), bay / ramp positions, sensor mounts and the timeline.

Frames (metres, degrees): the hub frame is the CARLA vehicle frame of
vehicle.delivery.lastmilehub (x forward, y right, z up, origin on the ground
midway between the axles); the pod frame likewise for a Mini Pod (origin on
the ground at the footprint centre, nose +x). Rotations [pitch, yaw, roll]
as carla.Rotation. Every prop is spawned with rot [0, 0, 0] relative to its
parent frame at rest: its mesh axes are the parent's axes.

Motions: "rotate" adds ``amount`` degrees to the rotator component of
``axis`` (x = roll, y = pitch, z = yaw) of the part's own (local) pose;
"slide" adds ``amount`` metres along the parent-frame ``axis``; "scale" is the
web viewer's motion, which CARLA can't do at run time -- such parts carry
``runtime`` with what to do instead. Sign conventions follow carla.Rotation
(positive roll tips +z toward +y, positive pitch lifts +x toward +z,
positive yaw turns +x toward +y); checked by rendering the props back onto
the vehicle (previews/lmh_contract_check.png).

"station" (when lmh_measured.json has it) is the whole charging station as one
static prop (static.prop.lmhchargestation) plus its moving parts, with the
plug / pad / deck-plug motions and the timing of the web viewer's "Charging
station" tab (js_3d_lastmile/charge.js + charge_env.js).

  python3 -I compose_lmh_parts.py <lmh_measured.json> <out lmh_parts.json> [mini.js]

LastMile Hub と Mini Pod の部品配置・動作の契約ファイルを書く。
"""

import json
import math
import re
import sys

# web viewer constants (mini_hub.js, newer site copy) / Web ビューアの定数
DOOR_Z, DOOR_Y, DOOR_H, RAMP_TOP, WALK_Y, CURB_Z = -0.74, 0.156, 0.86, 0.044, 0.15, -0.92
DOOR_OPEN = math.degrees(math.atan2(RAMP_TOP, DOOR_H) +
                         math.acos((WALK_Y - DOOR_Y) / math.hypot(DOOR_H, RAMP_TOP)))
RAMP_END = DOOR_Z - DOOR_H * math.sin(math.radians(DOOR_OPEN)) + RAMP_TOP * math.cos(math.radians(DOOR_OPEN))
BAYS_X, POD_Z, LANE = [-0.66, 0.0, 0.66], -0.31, -2.0
POD_FLOOR = 0.2
POD = {"HL": 0.35, "HW": 0.27, "WR": 0.085, "FLOOR": 0.3, "TOP": 0.665, "LID_X": 0.18,
       "DOOR_TOP": 0.6, "SOCKET_Y": 0.24, "CARGO_X": -0.08}
CARGO_Y = POD["FLOOR"] + 0.006
VP, VIN, VR = 1.6, 1.2, 0.6           # cruise, indoor, reverse into the bay (m/s)
MAST_RAISE = 0.55 * (1 - 0.06)        # mast scale 0.06 -> 1 about its foot

# MINI_STEPS of mini.js (used when the file isn't given) / mini.js が無い時の既定
STEPS = [("m_drive", "Drive to the Mini Pod zone", 7), ("m_scan", "Scan the stored parcels", 6),
         ("m_divert", "Diverter gates sort into 3 lanes", 6), ("m_lift", "Vertical lifts lower the parcels", 4),
         ("m_load", "Roller transfer into the pods", 6), ("m_release", "Unlock and lower the ramps", 5),
         ("m_deploy", "Mini Pods roll out", 6), ("m_route", "Independent routes", 4),
         ("m_aptA", "Apartment A: hand-over at the door", 7), ("m_aptB", "Apartment B: parcel locker", 7),
         ("m_officeC", "Office C: lobby delivery", 8), ("m_return", "Return to the hub", 12),
         ("m_dock", "Dock, lock and charge", 7), ("m_depart", "Drive on to the next zone", 4)]

# web viewer "Charging station" tab (charge.js CHARGE_STEPS, 111 s loop) / 充電タブ
CHARGE_STEPS = [("c_arrive", "Drive into the charge dock", 7), ("c_align", "Port opens, plug unit aligns", 5),
                ("c_insert", "Automatic plug-in", 6), ("h_arrive", "Mini-pod hub drives into bay 04", 8),
                ("h_insert", "Plug-in at the hub's front inlet", 5),
                ("c_land", "Drones fly in to the charging deck", 10),
                ("c_dplug", "Pad plugs dock into the drones", 5),
                ("h_pods", "Mini Pods reverse onto the charging row", 14),
                ("c_charge", "Charging everything at once", 12), ("c_takeoff", "Charged drones take off", 9),
                ("h_podsout", "Charged Mini Pods drive off", 12), ("h_unplug", "Hub unplugs and backs out", 8),
                ("c_unplug", "Unplug and close the port", 6), ("c_depart", "Raise jacks, back out", 4)]
# plug unit (charge_env.js L.DOCK): boom axis parked / at the inlet, boom stroke,
# lateral range; the web viewer's parking offsets (CARLA y) / プラグユニット
PLUG_PARK_Z, PLUG_INLET_Z, PLUG_STROKE, PLUG_Z_RANGE = 1.06, 1.15, 0.375, 0.15
PLUG_WEB_OFFSET = {"lmp": -0.035, "lmh": -0.02}
# Mini Pod pads (HUBBAY): strokes, lane, far end, speeds; deck pad plug stroke
# (pod_inside.js PLUG_OUT -0.28 -> PLUG_IN = drone PORT z -0.155 * 0.88 + 0.011)
PAD_PIN_STROKE, PAD_PLUG_STROKE = 0.07, 0.06
PAD_LANE_Y, PAD_FAR_X, PAD_BACK = 1.0, 15.0 - 0.52, 1.1
DECK_PLUG_STROKE = 0.28 + (-0.155 * 0.88 + 0.011)


def r(v, n=4):
    return [round(x, n) for x in v] if isinstance(v, (list, tuple)) else round(v, n)


def steps_from(path):
    if not path:
        return STEPS
    txt = open(path, encoding="utf-8").read()
    found = re.findall(r'\{\s*id:\s*"(m_\w+)",\s*title:\s*"([^"]+)",\s*dur:\s*([\d.]+)', txt)
    return [(i, t, float(d)) for i, t, d in found] or STEPS


def part(name, prop, loc, motion, **extra):
    d = {"name": name, "prop": "static.prop." + prop.lower(), "loc": r(loc), "rot": [0.0, 0.0, 0.0],
         "motion": motion}
    d.update(extra)
    return d


def rot(axis, amount, **k):
    return dict({"type": "rotate", "axis": axis, "amount": round(amount, 3), "space": "local"}, **k)


def slide(axis, amount, **k):
    return dict({"type": "slide", "axis": axis, "amount": round(amount, 4), "space": "local"}, **k)


def trap_leg(d, v, a):
    """Duration of a trapezoid speed profile over ``d`` m (charge.js trapLeg)."""
    ta = min(v / a, math.sqrt(d / a))
    return 2 * ta + (d - a * ta * ta) / (a * ta)


def station_section(st):
    """lmh_parts.json "station" from lmh_measured.json "station": poses,
    pivots, moving-part props and motions, charge.js timing.
    充電ステーション（静的プロップ＋可動部・動作・タイミング）。"""
    acc, t0 = 0.0, {}
    for sid, _, dur in CHARGE_STEPS:
        t0[sid] = acc
        acc += dur

    def at(sid, dt):
        return round(t0[sid] + dt, 3)

    def keys(rows):
        # web keys [t, lateral y, boom axis z, extension 0..1, latch 0..1] ->
        # prop offsets from rest / Web のキー -> 静止位置からの移動量
        return [{"t": t, "ZCarriage": r(float(dy)), "YCarriage": r(z - PLUG_PARK_Z),
                 "BoomStage": r(-e * PLUG_STROKE), "Head": r(-e * PLUG_STROKE), "latch": l}
                for t, dy, z, e, l in rows]

    pk, iz, ol, oh = PLUG_PARK_Z, PLUG_INLET_Z, PLUG_WEB_OFFSET["lmp"], PLUG_WEB_OFFSET["lmh"]
    plug_keys = {
        "lmp": keys([(0.0, 0, pk, 0, 0), (at("c_align", 1.2), 0, pk, 0, 0),
                     (at("c_align", 2.4), ol - 0.006, iz + 0.004, 0, 0), (at("c_align", 3.4), ol, iz, 0, 0),
                     (at("c_insert", 0.4), ol, iz, 0, 0), (at("c_insert", 2.4), ol, iz, 0.87, 0),
                     (at("c_insert", 3.8), ol, iz, 1, 0), (at("c_insert", 4.3), ol, iz, 1, 1),
                     (at("c_unplug", 1.4), ol, iz, 1, 1), (at("c_unplug", 1.8), ol, iz, 1, 0),
                     (at("c_unplug", 3.8), ol, iz, 0, 0), (at("c_unplug", 5.2), 0, pk, 0, 0)]),
        "lmh": keys([(0.0, 0, pk, 0, 0), (at("h_arrive", 5.9), 0, pk, 0, 0),
                     (at("h_arrive", 7.1), oh + 0.005, iz + 0.004, 0, 0), (at("h_arrive", 7.9), oh, iz, 0, 0),
                     (at("h_insert", 0.2), oh, iz, 0, 0), (at("h_insert", 2.0), oh, iz, 0.87, 0),
                     (at("h_insert", 3.2), oh, iz, 1, 0), (at("h_insert", 3.6), oh, iz, 1, 1),
                     (at("h_unplug", 1.2), oh, iz, 1, 1), (at("h_unplug", 1.6), oh, iz, 1, 0),
                     (at("h_unplug", 3.2), oh, iz, 0, 0), (at("h_unplug", 4.4), 0, pk, 0, 0)])}
    pods = []
    for i, p in enumerate(st["pads"]):
        ta = at("h_pods", 0.3 + 1.4 * i)
        on = round(ta + trap_leg(PAD_FAR_X - p[0], 2.5, 1.25) + 0.9 + trap_leg(PAD_BACK, 0.6, 0.6), 3)
        tu = at("h_podsout", 0.2 + 1.4 * (2 - i))
        pods.append({"pad": "M%d" % (i + 1), "arrive": ta, "on_pad": on,
                     "pins_in": [round(on + 0.2, 3), round(on + 0.7, 3)],
                     "plug_in": [round(on + 0.7, 3), round(on + 1.3, 3)],
                     "plug_out": [tu, round(tu + 0.5, 3)], "pins_out": [round(tu + 0.5, 3), round(tu + 0.9, 3)],
                     "drive_off": round(tu + 1.0, 3)})
    deck = []
    for i in range(len(st["deck_pads"])):
        ta, tp, tu = at("c_land", 0.3 + 1.7 * i), at("c_dplug", 0.4 + 1.2 * i), at("c_takeoff", 0.2 + 1.4 * (2 - i))
        deck.append({"pad": "P%d" % (i + 1), "fly_in": ta, "touch_down": round(ta + 5.2, 3),
                     "plug_in": [tp, round(tp + 1.4, 3)], "plug_out": [tu, round(tu + 0.6, 3)],
                     "lift_off": round(tu + 1.7, 3)})
    out = {
        "blueprint": "static.prop.lmhchargestation",
        "walk": dict(st.get("walk") or {}, note="footpath round the station: 1 m segments along +x and "
                     "corner squares (kerbs on +x and +y), origin bottom centre"),
        "apron": dict(st.get("apron") or {}, note="driveway slab, origin bottom centre: tiled by the sim "
                      "from the kerb to the open edge over both bays' drive-in strips"),
        "note": "origin = the bay 04 hub pose on the ground (where vehicle.delivery.lastmilehub parks); "
                "entrance = the -x edge (open side, faces the road); vehicles drive in nose first toward +x "
                "and reverse out. Moving parts are separate static props spawned at their pivots with "
                "rot [0, 0, 0] in this frame (CARLA axes). The floors are zero-thickness planes at z 0.",
        "bbox_min": st["bbox_min"], "bbox_max": st["bbox_max"], "size": st["size"],
        "entrance_x": st["entrance_x"],
        "bays": st["bays"],
        "bay_vehicles": {"lmp": "vehicle.delivery.lastmile", "lmh": "vehicle.delivery.lastmilehub"},
        "bays_note": "[x, y, yaw deg] of the parked vehicle (yaw 0 = nose +x); lmp = bay 03 (LMP-4), "
                     "lmh = bay 04 (LastMile Hub)",
        "bay_boxes": st.get("bay_boxes"),
        "roof_clearance": st.get("roof_clearance"),
        "roof_note": "metres above the floor: the lowest structure over each bay's drive-in strip (box "
                     "width) from the entrance edge to the stop bar, i.e. the underside of the canopy's cross "
                     "beams (one at the entrance edge, more over the bay); bay_boxes = painted bay outline",
        "plug_props": {"ZCarriage": "static.prop.lmhplugz", "YCarriage": "static.prop.lmhplugy",
                       "BoomStage": "static.prop.lmhplugboom", "Head": "static.prop.lmhplughead"},
        "plug_units": st["plug_units"],
        "plug_units_note": "rest poses = parked (boom axis z %.2f, retracted); Base is part of the "
                           "station mesh" % PLUG_PARK_Z,
        "plug_motion": {
            "ZCarriage": slide("y", 0.0, range=[-PLUG_Z_RANGE, PLUG_Z_RANGE],
                               note="lateral correction of the vehicle's parking offset; YCarriage, "
                                    "BoomStage and Head ride on it"),
            "YCarriage": slide("z", r(PLUG_INLET_Z - PLUG_PARK_Z),
                               note="boom height %.2f parked -> %.2f at the inlet" % (PLUG_PARK_Z, PLUG_INLET_Z)),
            "BoomStage": slide("x", -PLUG_STROKE, note="toward the vehicle's front inlet; rides on YCarriage"),
            "Head": slide("x", -PLUG_STROKE, note="rides on the boom stage (move with it)")},
        "pad_pins_prop": "static.prop.lmhdockpadpins", "pad_plug_prop": "static.prop.lmhdockpadplug",
        "deck_plug_prop": "static.prop.lmhdeckplug",
        "pads": st["pads"], "pad_pins": st["pad_pins"], "pad_plugs": st["pad_plugs"],
        "pod_poses": [r([p[0], p[1], 0.0, 90.0], 3) for p in st["pads"]],
        "pad_pins_motion": slide("y", PAD_PIN_STROKE, note="+y into the pod's latch receivers"),
        "pad_plug_motion": slide("y", PAD_PLUG_STROKE, note="+y into the pod's rear socket (after the pins)"),
        "pad_lane": {"y": PAD_LANE_Y, "far_x": round(PAD_FAR_X, 3), "reverse_m": PAD_BACK,
                     "note": "web viewer: Mini Pods come along y %.1f from x %.2f (behind the station, "
                             "heading -x), turn on the spot to nose +y and reverse %.2f m onto the pad; they "
                             "leave forward, turn to +x and drive off the same way" % (
                                 PAD_LANE_Y, PAD_FAR_X, PAD_BACK)},
        "deck_pads": st["deck_pads"], "deck_plugs": st["deck_plugs"], "deck_drone_yaw": 180.0,
        "deck_plug_motion": slide("y", r(-DECK_PLUG_STROKE),
                                  note="rest = retracted; -y toward the pad centre into the socket on the "
                                       "drone's left flank (drones sit on the pads nose -x, yaw 180)"),
        "timeline": {
            "source": "charge.js", "loop_s": round(acc, 2),
            "steps": [{"id": sid, "title": title, "t0": round(t0[sid], 2), "t1": round(t0[sid] + dur, 2)}
                      for sid, title, dur in CHARGE_STEPS],
            "interp": "smoothstep between keys (L.keyed / L.smooth)",
            "plug_keys": plug_keys,
            "lmp": {"drive_in": [0.0, 5.0], "drive_in_m": 7.5, "drive_in_mps": 3.0, "jacks_down": [5.3, 6.5],
                    "port_cover_open": [at("c_align", 0.2), at("c_align", 1.2)],
                    "power_on": [at("c_insert", 4.6), at("c_insert", 5.8)],
                    "port_cover_shut": [at("c_unplug", 4.4), at("c_unplug", 5.4)],
                    "jacks_up": [at("c_depart", 0.0), at("c_depart", 0.8)],
                    "reverse_out": [at("c_depart", 0.8), round(acc, 2)]},
            "lmh": {"drive_in": [at("h_arrive", 0.0), at("h_arrive", 4.6)], "drive_in_m": 6.9,
                    "drive_in_mps": 3.0, "inlet_cover_open": [at("h_arrive", 4.9), at("h_arrive", 5.9)],
                    "power_on": [at("h_insert", 3.8), at("h_insert", 4.9)],
                    "inlet_cover_shut": [at("h_unplug", 3.6), at("h_unplug", 4.4)],
                    "reverse_out": [at("h_unplug", 4.6), at("h_unplug", 8.0)], "reverse_mps2": 1.0},
            "pods": pods, "deck": deck,
            "note": "web viewer timing (s); [t0, t1] = a smoothstep move. The plug always retracts "
                    "before the pins release; drones take off and pods leave in reverse order"},
    }
    return out


def main():
    meas = json.load(open(sys.argv[1]))
    out_path = sys.argv[2]
    steps = steps_from(sys.argv[3] if len(sys.argv) > 3 else None)
    hp, pp, props = meas["hub"]["pivots"], meas["pod"]["pivots"], meas["props"]
    veh = meas["hub"]["vehicle"]

    hub_parts = []
    for i in (1, 2, 3):
        door = "LMHRampDoor" + ("" if i == 1 else str(i))
        hub_parts.append(part("Bay_%d_RampDoor" % i, door, hp["Bay_%d_RampDoor" % i],
                              rot("x", -DOOR_OPEN, note="pivot = bottom hinge; folds down onto the "
                                  "sidewalk (curb = left, -y); open lies at sidewalk height %.2f m" % WALK_Y)))
    for i in (1, 2, 3):
        hub_parts.append(part("Lift_%d_Carriage" % i, "LMHLiftCarriage", hp["Lift_%d_Carriage" % i],
                              slide("z", -0.72, note="roller top 1.22 -> 0.50 m")))
    for i in (1, 2, 3):
        hub_parts.append(part("Diverter_Gate_%d" % i, "LMHDiverterGate", hp["Diverter_Gate_%d" % i],
                              rot("z", 45.0, note="paddle swings from along +x to across the belt")))
    for i in (1, 2, 3):
        hub_parts.append(part("Bay_%d_ChargePlug" % i, "LMHChargePlug", hp["Bay_%d_ChargePlug" % i],
                              slide("y", 0.038, note="rest = plugged into the docked pod; +y retracts")))
    for i in (1, 2, 3):
        hub_parts.append(part("Bay_%d_LockPins" % i, "LMHLockPins", hp["Bay_%d_LockPins" % i],
                              slide("y", 0.058, note="rest = locked; +y unlocks")))
    hub_parts.append(part("Hub_FrontChargePort_Cover", "LMHPortCover", hp["Hub_FrontChargePort_Cover"],
                          slide("z", 0.25, note="DC inlet cover, front wall, curb-side corner, 1.15 m high")))

    bays = []
    for i, x in enumerate(BAYS_X, 1):
        pod = meas["hub"]["pods"]["P%d" % i]
        bays.append({"id": i, "dock": r(pod["loc"][:3] + [pod["yaw"]], 3),
                     "ramp_foot": r([x, LANE, WALK_Y], 3),
                     "door": "Bay_%d_RampDoor" % i, "plug": "Bay_%d_ChargePlug" % i,
                     "pins": "Bay_%d_LockPins" % i, "lift": "Lift_%d_Carriage" % i,
                     "gate": "Diverter_Gate_%d" % i, "pod_body": "static.prop.lmhminipod" + ("" if i == 1 else str(i)),
                     "pod_lid": "static.prop.lmhpodlid" + ("" if i == 1 else str(i))})

    slots = []
    for n, s in sorted(meas["hub"]["parcel_slots"].items()):
        slots.append({"name": n, "prop": "static.prop.lmhparcell" if n.startswith("Parcel_P") else
                      "static.prop.lmhparcels", "loc": r(s["loc"], 3), "rot": [0.0, s["yaw"], 0.0],
                      "model_size": s["size"]})

    w = 2 * math.pi * meas["pod"]["wheel_radius"]
    pod_parts = []
    for k in range(1, 7):
        pod_parts.append(part("Wheel%d" % k, "LMHPodWheel", pp["Wheel%d" % k],
                              rot("y", -360.0 / w, per="metre", note="spin per metre driven forward "
                                  "(negative pitch rolls forward); skid steer: left and right sides differ "
                                  "when turning")))
    pod_parts.append(part("Lid", "LMHPodLid", pp["Lid"],
                          rot("y", -66.0, note="pivot = front hinge; the rear edge lifts")))
    sb = meas["pod"]["shutter_box"]
    pod_parts.append(part("Shutter", "LMHPodShutter", pp["Shutter"],
                          {"type": "scale", "axis": "z", "amount": 0.04, "space": "local",
                           "runtime": {"swap_prop": "static.prop.lmhpodshutterup",
                                       "note": "CARLA can't scale a prop: open = destroy the shutter "
                                               "and spawn the rolled-up one at the same loc (both "
                                               "pivot on the top edge)"}},
                          opening={"min": sb["min"], "max": sb["max"],
                                   "note": "rear cargo opening (rear face x %.3f, floor z %.3f)" % (
                                       -POD["HL"], POD["FLOOR"])}))
    foot = pp["FlagMastFoot"]
    pod_parts.append(part("FlagMast", "LMHFlagMast", [foot[0], foot[1], round(foot[2] - MAST_RAISE, 4)],
                          slide("z", MAST_RAISE, web={"type": "scale", "axis": "z", "from": 0.06, "to": 1.0},
                                note="rest = stowed (as docked); pivot = mast foot; raising slides the "
                                     "mast out of the body (web viewer scales it instead); raise only "
                                     "once the pod is out of its bay")))
    fl = pp["Flag"]
    pod_parts.append(part("Flag", "LMHFlag", [fl[0], fl[1], round(fl[2] - MAST_RAISE, 4)],
                          slide("z", MAST_RAISE, hide_when_stowed=True,
                                note="rides on the mast top; the web viewer scales it 0.001 -> 1, so "
                                     "spawn/show it only when raised")))

    hl = POD["HL"]
    minipod = {
        "body": "static.prop.lmhminipod",
        "body_variants": ["static.prop.lmhminipod", "static.prop.lmhminipod2", "static.prop.lmhminipod3"],
        "lid_variants": ["static.prop.lmhpodlid", "static.prop.lmhpodlid2", "static.prop.lmhpodlid3"],
        "size": [0.70, 0.54, 0.67], "model_bbox": [meas["pod"]["bbox_min"], meas["pod"]["bbox_max"]],
        "height_lidar_top": 0.72, "wheel_radius": meas["pod"]["wheel_radius"],
        "speed_mps": VP, "speed_indoor_mps": VIN, "speed_reverse_mps": VR,
        "spin_time_s": "0.3 + |angle| / pi * 1.1 (turns on the spot, skid steer)",
        "parts": pod_parts,
        "cargo": {"loc": r([POD["CARGO_X"], 0.0, CARGO_Y]),
                  "note": "parcel bottom centre on the floor belt, under the lid"},
        "deliver": {
            "rear_push": {"to_loc": r([-0.85, 0.0, CARGO_Y]), "duration_s": 1.4,
                          "note": "floor belt pushes the parcel 0.77 m out through the open rear "
                                  "shutter (into a locker / delivery-box port at floor height)"},
            "ground_drop": {"loc": r([-0.85, 0.0, 0.0]),
                            "note": "the same push with nothing behind the pod: the parcel ends on "
                                    "the ground behind it (spawn the parcel prop with z at the ground)"},
            "lid_handover": {"lid_open_deg": -66.0, "note": "recipient takes the parcel from under the lid"}},
        "sensors_mounts": {
            "lidar": [0.27, 0.0, 0.72],
            "cam_front": [hl + 0.005, 0.0, 0.50],
            "cam_rear": [-hl - 0.008, 0.0, 0.64],
            "cam_left": [0.08, -(POD["HW"] + 0.01), 0.58],
            "cam_right": [0.08, POD["HW"] + 0.01, 0.58],
            "cam_down_front": [hl - 0.03, 0.0, 0.60],
            "ultrasonic": [[hl + 0.004, y, 0.245] for y in (-0.19, -0.06, 0.06, 0.19)] +
                          [[-hl - 0.006, y, 0.215] for y in (-0.20, 0.20)],
            "gnss": [0.21, 0.12, 0.67]},
        "sensors_rot_hint": {
            "cam_front": [0, 0, 0], "cam_rear": [0, 180, 0], "cam_left": [0, -90, 0], "cam_right": [0, 90, 0],
            "cam_down_front": [-50, 0, 0], "ultrasonic_front": [0, 0, 0], "ultrasonic_rear": [0, 180, 0]},
        "sensors_note": "lidar puck (0.27, 0, 0.665..0.715), side cameras and the four front ultrasonic "
                        "sensors are on the model; front/rear/down cameras, the two rear ultrasonics and "
                        "GNSS are plausible places on the shell (not modelled)",
    }

    dp = meas.get("dock", {}).get("pivots", {})
    dock = None
    if dp:
        dparts = [part("HubBay04", "LMHDockBay", dp["HubBay04"], None,
                       note="floor, canopy, 50 kW cabinet and screen of bay 04 + the Mini Pod row; "
                            "hub parked at the origin facing +x")]
        for i in (1, 2, 3):
            dparts.append(part("MiniPodPad_M%d" % i, "LMHDockPad", dp["MiniPodPad_M%d" % i], None,
                               pod_pose=r([dp["MiniPodPad_M%d" % i][0], -0.10, 0.0, 90.0], 3),
                               note="pod parks nose +y (yaw 90), reversing 1.10 m in from the lane"))
            dparts.append(part("MiniPodPad_M%d_LockPins" % i, "LMHDockPadPins",
                               dp["MiniPodPad_M%d_LockPins" % i], slide("y", 0.07, note="+y into the pod")))
            dparts.append(part("MiniPodPad_M%d_Plug" % i, "LMHDockPadPlug", dp["MiniPodPad_M%d_Plug" % i],
                               slide("y", 0.06, note="+y into the pod's rear socket (after the pins)")))
        dparts += [
            part("HubPlugUnit_Base", "LMHPlugBase", dp["HubPlugUnit_Base"], None),
            part("HubPlugUnit_ZCarriage", "LMHPlugZ", dp["HubPlugUnit_ZCarriage"],
                 slide("y", 0.0, range=[-0.15, 0.15], note="lateral correction of the hub's parking offset")),
            part("HubPlugUnit_YCarriage", "LMHPlugY", dp["HubPlugUnit_YCarriage"],
                 slide("z", 0.09, note="boom height 1.06 parked -> 1.15 at the inlet; rides on ZCarriage")),
            part("HubPlugUnit_BoomStage", "LMHPlugBoom", dp["HubPlugUnit_BoomStage"],
                 slide("x", -0.375, note="toward the hub's front inlet; rides on YCarriage")),
            part("HubPlugUnit_Head", "LMHPlugHead", dp["HubPlugUnit_Head"],
                 slide("x", -0.375, note="rides on the boom stage (move with it)"))]
        dock = {"note": "charging station; place it so its origin is where the hub parks (bay 04). "
                        "Parts are static props in the world, frame = that origin, CARLA axes.",
                "parts": dparts}

    acc, tl = 0.0, []
    for sid, title, dur in steps:
        tl.append({"id": sid, "title": title, "t0": round(acc, 2), "t1": round(acc + dur, 2)})
        acc += dur

    doc = {
        "_about": __doc__.split("\n\n")[0] + " Generated by compose_lmh_parts.py from lmh_measured.json; "
                  "do not edit by hand (rebuild with build_lastmilehub.sh).",
        "_frames": "metres; hub/pod frames = CARLA vehicle frame (x forward, y right, z up); rot = "
                   "[pitch, yaw, roll] deg; motion.axis x=roll y=pitch z=yaw for rotate, parent axis for "
                   "slide; the bay doors are on the hub's LEFT (-y) = the curb on keep-left maps",
        "hub": {
            "blueprint": "vehicle.delivery.lastmilehub",
            "size": [3.25, 1.48, 2.09],
            "model_bbox": [meas["hub"]["model_bbox_min"], meas["hub"]["model_bbox_max"]],
            "wheels": veh["wheels"], "wheel_radius": veh["wheel_radius"], "mass_kg": 1500,
            "parts": hub_parts,
            "bays": bays,
            "curb": {"door_hinge_y": DOOR_Z, "ramp_end_y": round(RAMP_END, 3), "curb_y": CURB_Z,
                     "sidewalk_z": WALK_Y, "lane_y": LANE,
                     "note": "web viewer layout: hub at the kerb with its left side 0.18 m from it; "
                             "pods run on the sidewalk lane y = -2.0 after leaving the ramp"},
            "parcel_slots": slots,
            "parcel_lift_low_z": 0.5,
        },
        "minipod": minipod,
        "parcels": {"small": "static.prop.lmhparcels", "large": "static.prop.lmhparcell",
                    "size_small": props["LMHParcelS"]["size"], "size_large": props["LMHParcelL"]["size"],
                    "pivot": "bottom centre (loc z = the surface it stands on)"},
        "timeline": {"source": "mini.js", "loop_s": round(acc, 2), "steps": tl,
                     "events": [
                         {"t": 6.2, "what": "hub stops after 12.4 m (4 m/s in)"},
                         {"t": 14.0, "what": "Diverter_Gate_3 fires"}, {"t": 14.9, "what": "Diverter_Gate_2 fires"},
                         {"t": 15.8, "what": "Diverter_Gate_1 fires"},
                         {"t": 19.3, "what": "lifts 1,2,3 down 0.72 m in 2.5 s, 0.35 s apart"},
                         {"t": 29.3, "what": "plugs retract"}, {"t": 29.8, "what": "pins retract"},
                         {"t": 30.4, "what": "ramp doors fold down (until 32.2)"},
                         {"t": 34.4, "what": "P1 leaves"}, {"t": 34.7, "what": "P2 leaves"},
                         {"t": 37.0, "what": "P3 leaves"},
                         {"t": 85.6, "what": "hub pulls away (1.6 m/s^2)"}],
                     "note": "web viewer timing; in the CARLA scenario the pods don't enter buildings: "
                             "they drop the parcel on the ground or push it into a delivery box"},
    }
    if dock:
        doc["dock"] = dock
    if meas.get("station"):
        doc["station"] = station_section(meas["station"])
    if meas.get("locker"):
        lk = dict(meas["locker"])
        lk.update({"blueprint": "static.prop.lmhlocker", "flap": "static.prop.lmhlockerflap",
                   "flap_motion": rot("x", lk["flap_open_deg"], note="pivot = top hinge; opens inward (-y)"),
                   "note": "mini.js Apartment B parcel locker: origin on the pavement, front (robot port) "
                           "+y; the pod reverses up to the port and its floor belt pushes the parcel in"})
        doc["locker"] = lk
    with open(out_path, "w") as f:
        json.dump(doc, f, indent=1)
    print("wrote %s: %d hub parts, %d pod parts, %d dock parts, door open %.2f deg, ramp end y %.3f" % (
        out_path, len(hub_parts), len(pod_parts), len(dock["parts"]) if dock else 0, DOOR_OPEN, RAMP_END))


if __name__ == "__main__":
    main()
