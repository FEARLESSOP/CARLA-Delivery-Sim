#!/usr/bin/env python3
"""road_overlay.py - the road logic drawn into the world, like CARLA's OpenDRIVE
view in the UE4 editor.

  ../../carla-venv/bin/python road_overlay.py          (run_pipeline.py starts it)
  ../../carla-venv/bin/python road_overlay.py --bench maps/build/OitaStation/OitaStation.xodr

What the Traffic Manager, the autopilot and the NPCs drive on is the OpenDRIVE
map (carla.Map), on the generated maps made from the OSM streets (osm_city:
OSM -> netconvert -> xodr). This helper draws it around your car with CARLA's
debug lines, which the camera sensors (your driving window) see:

  road         red lines along both edges of every Driving lane
  direction    lane centre and an arrow every 10 m (keep-left shows here)
  junctions    one orange path per junction lane, junction boxes yellow
  lanes        other lane types (sidewalk, shoulder, parking, biking ...)
  rules        止まれ / give-way and speed landmarks, traffic lights with their
               stop bars in the light's colour, lane speed limits on the road
  lane_change  lane boundaries: colour = may the AI change lanes there
               (waypoint.lane_change: cyan both ways, yellow one way, white
               none), dashed / continuous = the painted marking
  ids          road / lane / junction ids
  osm          the source OSM highway ways (generated maps only), green

Text near the hero is drawn with line strokes: CARLA's draw_string goes to the
server window's HUD only and never shows in a camera image. A second area is
drawn round what the CarlaUE4 window looks at (control "spectator"), larger
when its camera is high, with draw_string labels there. Debug lines are
self-lit and bloom, so colours are sent very dim (hue x brightness ~3).

The geometry is precomputed once per map load (numpy arrays). Every refresh
(2 s) only what lies within ``radius`` is drawn, nearest first, at most
``max_shapes`` (800) shapes, arrows and text only within ``detail_radius``
(45 m), each living a little longer than the refresh,
so nothing stays behind when the overlay is turned off or the map changes.
Never persistent lines. Idle (draws nothing) while off.

Control file ``data/road_overlay.json`` (O key in the driving window, the
dashboard's "Road logic overlay" card), read every refresh; the state goes to
``data/road_overlay_status.json``.

道路ロジック（AI が使う OpenDRIVE の道路・車線・規制）を自車の周囲に線で描く。
UE4 エディタの OpenDRIVE 表示と同じ考え方。O キーまたはダッシュボードで切替。
"""

import argparse
import json
import math
import os
import signal
import sys
import threading
import time
import xml.etree.ElementTree as ET

import numpy as np

try:
    import carla
except ImportError:          # control-file helpers work without it / 設定ファイル操作だけなら不要
    carla = None

HERE = os.path.dirname(os.path.abspath(__file__))
CONTROL_PATH = os.path.join(HERE, "data", "road_overlay.json")
STATUS_PATH = os.path.join(HERE, "data", "road_overlay_status.json")
MAPS_JSON = os.path.join(HERE, "maps", "maps.json")
BUILD_DIR = os.path.join(HERE, "maps", "build")
ZONES_PATH = os.path.join(HERE, "data", "delivery_zones.json")

# --------------------------------------------------------------------------
# control file / 設定ファイル
# --------------------------------------------------------------------------
LAYERS = ("road", "direction", "junctions", "lanes", "rules", "lane_change", "ids", "osm", "zones")
LAYER_DEFAULTS = {"road": True, "direction": True, "junctions": True, "lanes": True,
                  "rules": True, "lane_change": False, "ids": False, "osm": False, "zones": True}
LAYER_LABELS = {
    "road": ("Road edges", "道路の端"),
    "direction": ("Driving direction", "走行方向"),
    "junctions": ("Junctions", "交差点"),
    "lanes": ("Other lane types", "その他の車線種別"),
    "rules": ("Rules: signs, lights, speed", "規制：標識・信号・速度"),
    "lane_change": ("Lane change", "車線変更"),
    "ids": ("Road / lane ids", "道路・車線 ID"),
    "osm": ("Raw OSM ways", "元の OSM 道路"),
    "zones": ("Delivery & no-fly zones", "配送ゾーン・飛行禁止区域"),
}
# Server cost (measured live 2026-11-09, OitaKosen): 2000 shapes a second took
# the server from 20 to 13 frames/s. So: a 2 s refresh, at most 800 shapes,
# arrows / text / sign details only within ``detail_radius``.
# サーバー負荷の実測から既定値を決めた（2 秒ごと・最大 800 本・詳細は 45 m 以内）。
DEFAULTS = {"on": False, "radius": 120.0, "max_shapes": 800, "refresh": 2.0, "detail_radius": 45.0,
            "brightness": 3.0, "spectator": True, "layers": dict(LAYER_DEFAULTS)}
LIMITS = {"radius": (20.0, 400.0), "max_shapes": (100, 10000), "refresh": (0.5, 5.0),
          "detail_radius": (10.0, 200.0), "brightness": (1.0, 12.0)}


def _read_json(path):
    try:
        with open(path) as fh:
            return json.load(fh)
    except (OSError, ValueError):
        return None


def _write_json(path, obj):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w") as fh:
        json.dump(obj, fh, indent=1)
    os.replace(tmp, path)


def clean_control(raw):
    """Known keys only, clamped; missing ones from DEFAULTS. / 既定値で補い範囲を整える。"""
    s = json.loads(json.dumps(DEFAULTS))
    raw = raw if isinstance(raw, dict) else {}
    for k in ("on", "spectator"):
        if k in raw:
            s[k] = bool(raw[k])
    for k, (lo, hi) in LIMITS.items():
        if raw.get(k) is None:
            continue
        try:
            v = float(raw[k])
        except (TypeError, ValueError):
            continue
        if math.isfinite(v):
            s[k] = max(lo, min(hi, v))
    s["max_shapes"] = int(s["max_shapes"])
    layers = raw.get("layers")
    if isinstance(layers, dict):
        for k in LAYERS:
            if k in layers:
                s["layers"][k] = bool(layers[k])
    return s


def load_control(path=None):
    return clean_control(_read_json(path or CONTROL_PATH))


def save_control(s, path=None):
    s = clean_control(s)
    _write_json(path or CONTROL_PATH, s)
    return s


def update_control(changes, path=None):
    """Merge ``changes`` ({on, radius, max_shapes, refresh, layers: {..}}) into
    the file. / 変更をファイルに反映する。"""
    s = load_control(path)
    changes = changes if isinstance(changes, dict) else {}
    for k in ("on", "spectator") + tuple(LIMITS):
        if k in changes:
            s[k] = changes[k]
    if isinstance(changes.get("layers"), dict):
        s["layers"].update({k: v for k, v in changes["layers"].items() if k in LAYERS})
    return save_control(s, path)


def toggle(path=None):
    """Flip "on"; returns the new state (the O key). / オン・オフを切り替える。"""
    s = load_control(path)
    return update_control({"on": not s["on"]}, path)["on"]


def read_status(path=None):
    st = _read_json(path or STATUS_PATH) or {}
    st["live"] = time.time() - float(st.get("t", 0) or 0) < 6.0
    return st


def helper_alive(path=None):
    """Is road_overlay.py running (fresh status file)? / ヘルパーが動作中か。"""
    return read_status(path)["live"]


# --------------------------------------------------------------------------
# styles: hue (0..1 per channel) and thickness (m)
# スタイル（色相と太さ）
#
# Debug lines are unlit and emissive: a channel value of 255 (even 22, see
# traction_fx.py) blooms the whole camera image white. Measured live on
# OitaKosen in daylight (maps/road_overlay_live.py --calibrate 1,3,6,12,24,48,
# 2026-11-09): 1-3 draw clean saturated lines, 6 and up glow. So the colour
# sent is round(hue x brightness) with brightness ~3 (control "brightness",
# dimmed at night), and the hues are picked to stay apart at those small
# integers (thirds: 0, 1/3, 2/3, 1). The dashboard legend shows hue x 255.
# 線は自発光：255 では画面全体が白飛びする。実測で 1〜3 が見やすい。
# --------------------------------------------------------------------------
T3 = 1.0 / 3.0
STYLE_LIST = [
    ("edge", (1, 0, 0), 0.05),              # road border of a Driving lane / 道路の端
    ("edge_inner", (2 * T3, 0, 0), 0.03),   # between two drivable lanes / 車線の境
    ("edge_j", (1, T3, 0), 0.045),          # junction lane / 交差点内の車線
    ("edge_j_inner", (2 * T3, T3, 0), 0.03),
    ("edge_hero", (1, 2 * T3, 2 * T3), 0.08),   # your lane / 自車の車線
    ("centre", (0, T3, 1), 0.03),
    ("centre_hero", (T3, 2 * T3, 1), 0.05),
    ("arrow", (0, T3, 1), 0.04),
    ("arrow_hero", (T3, 2 * T3, 1), 0.05),
    ("jbox", (1, 1, 0), 0.04),
    ("lc_both", (0, 1, 1), 0.04),
    ("lc_one", (1, 2 * T3, 0), 0.04),
    ("lc_none", (1, 1, 1), 0.04),
    ("Sidewalk", (2 * T3, 2 * T3, 2 * T3), 0.04),
    ("Shoulder", (2 * T3, T3, 0), 0.04),
    ("Parking", (2 * T3, 0, 1), 0.04),
    ("Bus", (0, 2 * T3, 2 * T3), 0.04),
    ("Biking", (1, T3, 2 * T3), 0.04),
    ("Bidirectional", (1, 0, 2 * T3), 0.04),
    ("Median", (2 * T3, 2 * T3, 0), 0.04),
    ("other", (T3, T3, T3), 0.04),          # Border, Restricted, Tram, Stop ...
    ("speed", (1, 1, 1), 0.05),
    ("ids", (2 * T3, 2 * T3, 1), 0.03),
    ("stop_bar", (1, 0, 0), 0.12),
    ("yield_bar", (1, 1, 1), 0.12),
    ("osm", (0, 1, 0), 0.04),
    ("nofly_zone", (1, 0, 0), 0.10),
    ("pod_zone", (0, 1, 1), 0.08),
]
SID = {n: i for i, (n, _, _) in enumerate(STYLE_LIST)}
OTHER_TYPES = ("Sidewalk", "Shoulder", "Parking", "Bus", "Biking", "Bidirectional", "Median")
# remaps: junction layer off -> road colour; your lane -> highlight
REMAP_NOJ = np.arange(len(STYLE_LIST), dtype=np.int16)
REMAP_NOJ[SID["edge_j"]], REMAP_NOJ[SID["edge_j_inner"]] = SID["edge"], SID["edge_inner"]
REMAP_HERO = np.arange(len(STYLE_LIST), dtype=np.int16)
for _n in ("edge", "edge_inner", "edge_j", "edge_j_inner"):
    REMAP_HERO[SID[_n]] = SID["edge_hero"]
REMAP_HERO[SID["centre"]], REMAP_HERO[SID["arrow"]] = SID["centre_hero"], SID["arrow_hero"]
HERO_GAIN = 4.0 / 3.0         # your lane a step brighter / 自車の車線は一段明るく

LIGHT_HUE = {"Red": (1, 0, 0), "Yellow": (1, 1, 0), "Green": (0, 1, 0)}
LIGHT_OFF = (T3, T3, T3)
STOP_HUE, YIELD_HUE, SPEED_SIGN_HUE = (1, 0, 0), (1, 1, 1), (1, T3, T3)
NIGHT_DIM = 0.4               # sun below the horizon: brighter exposure, dimmer lines / 夜は暗く

# pod palette for in-world boxes (matching dashboard delivery palette)
# ポッドごとの色（ダッシュボードのパレットと統一）
POD_HUES = [
    (0.0, 1.0, 1.0),          # Pod 1: Cyan
    (1.0, 1.0, 0.0),          # Pod 2: Yellow
    (1.0, 0.0, 1.0),          # Pod 3: Magenta
    (2.0 / 3, 1.0, 0.0),      # Pod 4: Lime (LMH Hub)
    (1.0, 2.0 / 3, 0.0),      # Pod 5: Orange
    (1.0 / 3, 2.0 / 3, 1.0),  # Pod 6: Sky blue
    (0.0, 1.0, 2.0 / 3),      # Pod 7: Mint
    (2.0 / 3, 1.0 / 3, 1.0),  # Pod 8: Violet
]


# see-through faces of the zone boxes: panels from maps/build_zone_faces.sh
# (1 m x k m, k = 1..32), spawned as static.prop.mesh, tiled per face
# ゾーン箱の半透明の面（maps/build_zone_faces.sh のパネルを並べる）
FACE_COLOURS = dict(zip([(1.0, 0.0, 0.0)] + POD_HUES,
                        ["Red", "Cyan", "Yellow", "Magenta", "Lime", "Orange", "Sky", "Mint", "Violet"]))
FACE_ASPECTS = (32, 16, 8, 4, 2, 1)
FACE_ROLE = "zone_face"


def face_tiles(w, h, min_side=0.3, limit=40):
    """Cover a w x h rectangle with k x 1 panels (k a power of two), no overlap.
    Returns (cu, cv, k, side, long_on_u) per panel. / 矩形をパネルで埋める"""
    out, u0, v0 = [], 0.0, 0.0
    while w >= min_side and h >= min_side and len(out) < limit:
        if w >= h:
            n = int(w / h + 1e-6)
            for k in FACE_ASPECTS:
                while n >= k and len(out) < limit:
                    out.append((u0 + k * h / 2.0, v0 + h / 2.0, k, h, True))
                    u0, w, n = u0 + k * h, w - k * h, n - k
        else:
            n = int(h / w + 1e-6)
            for k in FACE_ASPECTS:
                while n >= k and len(out) < limit:
                    out.append((u0 + w / 2.0, v0 + k * w / 2.0, k, w, False))
                    v0, h, n = v0 + k * w, h - k * w, n - k
    return out


class ZoneFaces(object):
    """Keeps one set of translucent panels on every zone box. / ゾーン箱の面"""

    def __init__(self, world, log=print):
        self.world, self.log = world, log
        self.key, self.actors, self.bp = None, [], None
        try:
            self.bp = world.get_blueprint_library().find("static.prop.mesh")
            for a in world.get_actors().filter("static.prop.mesh"):
                if a.attributes.get("role_name") == FACE_ROLE:   # left by a crashed run / 前回の残り
                    a.destroy()
        except (AttributeError, IndexError, RuntimeError):
            self.bp = None

    def clear(self):
        for a in self.actors:
            try:
                a.destroy()
            except RuntimeError:
                pass
        self.actors, self.key = [], None

    def _panel(self, colour, k, side, loc, rot):
        bp = self.bp
        path = "/Game/ZoneTint/ZoneTint_%s_%d.ZoneTint_%s_%d" % (colour, k, colour, k)
        bp.set_attribute("mesh_path", path)
        bp.set_attribute("scale", "%.3f" % side)
        bp.set_attribute("role_name", FACE_ROLE)
        a = self.world.try_spawn_actor(bp, carla.Transform(loc, rot))
        if a is not None:
            self.actors.append(a)

    def sync(self, boxes):
        if self.bp is None or carla is None:
            return
        key = tuple((zb["kind"], zb["hue"]) + tuple(round(zb[k], 1) for k in ("x0", "y0", "x1", "y1", "cz", "hz"))
                    for zb in boxes)
        if key == self.key and (not self.actors or self.actors[0].is_alive):
            return
        self.clear()
        self.key = key
        R = carla.Rotation
        for zb in boxes:
            colour = FACE_COLOURS.get(tuple(zb["hue"]), "Red")
            x0, x1 = sorted((zb["x0"], zb["x1"]))
            y0, y1 = sorted((zb["y0"], zb["y1"]))
            zb_, zt = zb["cz"] - zb["hz"], zb["cz"] + zb["hz"]
            for cu, cv, k, s, lu in face_tiles(x1 - x0, y1 - y0):            # roof / 屋根
                self._panel(colour, k, s, carla.Location(x0 + cu, y0 + cv, zt), R(yaw=0 if lu else 90))
            for y in (y0, y1):                                               # walls along x / x 方向の壁
                for cu, cv, k, s, lu in face_tiles(x1 - x0, zt - zb_):
                    self._panel(colour, k, s, carla.Location(x0 + cu, y, zb_ + cv),
                                R(roll=90) if lu else R(pitch=90, yaw=90))
            for x in (x0, x1):                                               # walls along y / y 方向の壁
                for cu, cv, k, s, lu in face_tiles(y1 - y0, zt - zb_):
                    self._panel(colour, k, s, carla.Location(x, y0 + cu, zb_ + cv),
                                R(yaw=90, roll=90) if lu else R(pitch=90))
        self.log("[road_overlay] zone faces: %d panels on %d boxes" % (len(self.actors), len(boxes)))


def channel(hue, level):
    """(r, g, b) ints sent to CARLA for a hue at a brightness level (>= 1 where
    the hue has the channel, so a dim level never turns a colour black).
    色相と明るさから CARLA に送る値。"""
    return tuple(0 if h <= 0 else max(1, int(round(h * level))) for h in hue)


def display_rgb(hue):
    """The hue at full strength, for the dashboard legend. / 凡例用"""
    return [int(round(255 * h)) for h in hue]


# the dashboard card's legend / ダッシュボードの凡例
LEGEND = [
    ("road", "edge", "Road edge (Driving lane)", "道路の端（走行車線）"),
    ("road", "edge_hero", "Your lane", "自車の車線"),
    ("direction", "arrow", "Lane centre + direction", "車線中心と走行方向"),
    ("junctions", "edge_j", "Junction path (lane centre)", "交差点内の進路（車線中心）"),
    ("junctions", "jbox", "Junction box", "交差点の範囲"),
    ("lanes", "Sidewalk", "Sidewalk", "歩道"),
    ("lanes", "Shoulder", "Shoulder", "路肩"),
    ("lanes", "Parking", "Parking", "駐車"),
    ("lanes", "Biking", "Biking", "自転車"),
    ("lanes", "other", "Other lane type", "その他"),
    ("rules", "stop_bar", "Stop (止まれ) / red light", "止まれ・赤信号"),
    ("rules", "yield_bar", "Give way / speed (white text)", "譲れ・制限速度（白文字）"),
    ("lane_change", "lc_both", "AI may change lanes both ways (dashed = broken paint)",
     "AI は両方向に車線変更可（破線 = 路面も破線）"),
    ("lane_change", "lc_one", "AI may change lanes one way", "AI は片方向のみ車線変更可"),
    ("lane_change", "lc_none", "AI may not change lanes", "AI は車線変更不可"),
    ("ids", "ids", "Road / lane / junction id", "道路・車線・交差点 ID"),
    ("osm", "osm", "Raw OSM way", "元の OSM 道路"),
    ("zones", "nofly_zone", "No-fly zone (airspace box)", "飛行禁止区域（赤色の直方体）"),
    ("zones", "pod_zone", "Pod delivery zone (ground box)", "配送ゾーン（ポッド別の色付き直方体）"),
]


def legend():
    return [{"layer": layer, "key": key, "rgb": display_rgb(STYLE_LIST[SID[key]][1]), "en": en, "ja": ja}
            for layer, key, en, ja in LEGEND]


def layer_info():
    return [{"key": k, "en": LAYER_LABELS[k][0], "ja": LAYER_LABELS[k][1],
             "default": LAYER_DEFAULTS[k]} for k in LAYERS]


# --------------------------------------------------------------------------
# stroke font: polylines on a 1 x 1 box, y up / 線で書く文字
# --------------------------------------------------------------------------
_BOX = [(0, 0), (1, 0), (1, 1), (0, 1), (0, 0)]
FONT = {
    "0": [_BOX, [(0, 0), (1, 1)]],
    "1": [[(0.25, 0.8), (0.5, 1), (0.5, 0)]],
    "2": [[(0, 1), (1, 1), (1, 0.5), (0, 0.5), (0, 0), (1, 0)]],
    "3": [[(0, 1), (1, 1), (1, 0), (0, 0)], [(0.2, 0.5), (1, 0.5)]],
    "4": [[(0, 1), (0, 0.5), (1, 0.5)], [(1, 1), (1, 0)]],
    "5": [[(1, 1), (0, 1), (0, 0.5), (1, 0.5), (1, 0), (0, 0)]],
    "6": [[(1, 1), (0, 1), (0, 0), (1, 0), (1, 0.5), (0, 0.5)]],
    "7": [[(0, 1), (1, 1), (0.4, 0)]],
    "8": [_BOX, [(0, 0.5), (1, 0.5)]],
    "9": [[(1, 0.5), (0, 0.5), (0, 1), (1, 1), (1, 0), (0, 0)]],
    "-": [[(0.15, 0.5), (0.85, 0.5)]],
    "/": [[(0, 0), (1, 1)]],
    "A": [[(0, 0), (0, 1), (1, 1), (1, 0)], [(0, 0.5), (1, 0.5)]],
    "B": [[(0, 0), (0, 1), (0.75, 1), (0.75, 0.5), (0, 0.5)], [(0.75, 0.5), (1, 0.5), (1, 0), (0, 0)]],
    "C": [[(1, 1), (0, 1), (0, 0), (1, 0)]],
    "D": [[(0, 0), (0, 1), (0.6, 1), (1, 0.6), (1, 0.4), (0.6, 0), (0, 0)]],
    "E": [[(1, 1), (0, 1), (0, 0), (1, 0)], [(0, 0.5), (0.7, 0.5)]],
    "F": [[(1, 1), (0, 1), (0, 0)], [(0, 0.5), (0.7, 0.5)]],
    "G": [[(1, 1), (0, 1), (0, 0), (1, 0), (1, 0.5), (0.5, 0.5)]],
    "H": [[(0, 0), (0, 1)], [(1, 0), (1, 1)], [(0, 0.5), (1, 0.5)]],
    "I": [[(0.5, 0), (0.5, 1)], [(0.2, 1), (0.8, 1)], [(0.2, 0), (0.8, 0)]],
    "J": [[(1, 1), (1, 0), (0, 0), (0, 0.3)]],
    "K": [[(0, 0), (0, 1)], [(1, 1), (0, 0.5), (1, 0)]],
    "L": [[(0, 1), (0, 0), (1, 0)]],
    "M": [[(0, 0), (0, 1), (0.5, 0.5), (1, 1), (1, 0)]],
    "N": [[(0, 0), (0, 1), (1, 0), (1, 1)]],
    "O": [_BOX],
    "P": [[(0, 0), (0, 1), (1, 1), (1, 0.5), (0, 0.5)]],
    "R": [[(0, 0), (0, 1), (1, 1), (1, 0.5), (0, 0.5), (1, 0)]],
    "S": [[(1, 1), (0, 1), (0, 0.5), (1, 0.5), (1, 0), (0, 0)]],
    "T": [[(0, 1), (1, 1)], [(0.5, 1), (0.5, 0)]],
    "U": [[(0, 1), (0, 0), (1, 0), (1, 1)]],
    "V": [[(0, 1), (0.5, 0), (1, 1)]],
    "W": [[(0, 1), (0.25, 0), (0.5, 0.5), (0.75, 0), (1, 1)]],
    "X": [[(0, 0), (1, 1)], [(0, 1), (1, 0)]],
    "Y": [[(0, 1), (0.5, 0.5), (1, 1)], [(0.5, 0.5), (0.5, 0)]],
    "Z": [[(0, 1), (1, 1), (0, 0), (1, 0)]],
}


def text_segments(text, centre, right, up, h, w=None, gap=0.4):
    """Line segments ((x,y,z), (x,y,z)) writing ``text`` centred on ``centre``
    in the plane spanned by unit vectors ``right`` (reading direction) and
    ``up``; ``h`` character height, ``w`` width (m). / 文字を線分で書く。"""
    w = w or 0.55 * h
    adv = w * (1.0 + gap)
    chars = str(text).upper()
    x0 = -(adv * len(chars) - w * gap) / 2.0
    cx, cy, cz = centre
    out = []
    for k, ch in enumerate(chars):
        for poly in FONT.get(ch, ()):
            pts = []
            for px, py in poly:
                a = x0 + k * adv + px * w
                b = (py - 0.5) * h
                pts.append((cx + right[0] * a + up[0] * b, cy + right[1] * a + up[1] * b,
                            cz + right[2] * a + up[2] * b))
            out.extend(zip(pts, pts[1:]))
    return out


# --------------------------------------------------------------------------
# geometry helpers / 幾何
# --------------------------------------------------------------------------
def _pl_dist(p, a, b):
    """Distance of point p from line a-b (3-D). / 点と直線の距離"""
    ab = (b[0] - a[0], b[1] - a[1], b[2] - a[2])
    ap = (p[0] - a[0], p[1] - a[1], p[2] - a[2])
    n2 = ab[0] * ab[0] + ab[1] * ab[1] + ab[2] * ab[2]
    if n2 < 1e-12:
        return math.sqrt(ap[0] ** 2 + ap[1] ** 2 + ap[2] ** 2)
    cx = ap[1] * ab[2] - ap[2] * ab[1]
    cy = ap[2] * ab[0] - ap[0] * ab[2]
    cz = ap[0] * ab[1] - ap[1] * ab[0]
    return math.sqrt((cx * cx + cy * cy + cz * cz) / n2)


def simplify(pts, tol=0.06, max_len=25.0):
    """Indices of the points kept so that the polyline stays within ``tol`` m
    of the original and no piece is longer than ``max_len`` (greedy).
    直線部分をまとめる（許容誤差 tol、最大長 max_len）。"""
    n = len(pts)
    if n <= 2:
        return list(range(n))
    keep, i = [0], 0
    while i < n - 1:
        j = i + 1
        while j + 1 < n:
            e = pts[j + 1]
            if math.dist(pts[i], e) > max_len:
                break
            if any(_pl_dist(pts[k], pts[i], e) > tol for k in range(i + 1, j + 1)):
                break
            j += 1
        keep.append(j)
        i = j
    return keep


def seg_dist(px, py, ax, ay, bx, by):
    """2-D distance of (px, py) from segments a-b (numpy arrays). / 点と線分の距離"""
    dx, dy = bx - ax, by - ay
    l2 = dx * dx + dy * dy
    t = ((px - ax) * dx + (py - ay) * dy) / np.maximum(l2, 1e-9)
    t = np.clip(t, 0.0, 1.0)
    return np.hypot(ax + t * dx - px, ay + t * dy - py)


def road_speeds_from_xodr(xodr):
    """{road id: km/h} from the road-level <type><speed> records (CARLA's
    towns). / 道路単位の最高速度（CARLA の町）。"""
    factor = {"m/s": 3.6, "km/h": 1.0, "mph": 1.609344}
    out = {}
    try:
        root = ET.fromstring(xodr)
    except ET.ParseError:
        return out
    for road in root.findall("road"):
        try:
            rid = int(road.get("id"))
        except (TypeError, ValueError):
            continue
        for t in road.findall("type"):
            sp = t.find("speed")
            if sp is not None and sp.get("max") not in (None, "no limit", "undefined"):
                try:
                    out[rid] = float(sp.get("max")) * factor.get(sp.get("unit", "m/s"), 3.6)
                    break
                except ValueError:
                    pass
    return out


class SpeedLookup(object):
    """Lane limit: hil_sim/lane_limits.py (the lane <speed> on your own maps;
    CARLA's sign triggers miss keep-left lanes, map guide ch. 29), else the
    road-level <speed> (CARLA's towns). / 車線の制限速度"""

    def __init__(self, wmap):
        self.ll = None
        try:
            from lane_limits import LaneLimits
            self.ll = LaneLimits()
            self.ll._load(wmap)
            if not self.ll.by_section:
                self.ll = None
        except Exception as exc:                       # noqa: BLE001
            print("[road_overlay] lane limits: %s" % exc)
            self.ll = None
        self.road = {} if self.ll is not None else road_speeds_from_xodr(wmap.to_opendrive())

    def kmh(self, wmap, wp):
        if self.ll is not None:
            v = self.ll.limit_wp(wmap, wp)
            if v:
                return int(v)
        v = self.road.get(wp.road_id)
        return int(round(v)) if v else None


class LineBuf(object):
    """Collects line rows, frozen into numpy arrays. / 線分を集めて numpy 配列にする。"""

    def __init__(self):
        self.rows = []

    def add(self, a, b, layer, style, chain=-1, inter=False, ref=None, detail=None):
        """``ref``: a label's centre (its distance); ``detail``: drawn only
        within the detail radius (labels and lane centres by default).
        ref はラベルの中心、detail は詳細半径の内側だけ描く。"""
        ra, rb = (a, b) if ref is None else (ref, ref)
        detail = (ref is not None) if detail is None else detail
        self.rows.append((a[0], a[1], a[2], b[0], b[1], b[2], ra[0], ra[1], rb[0], rb[1],
                          LAYERS.index(layer), SID[style], chain, inter, detail))

    def freeze(self):
        return LineTable(self.rows)


class LineTable(object):
    def __init__(self, rows):
        arr = np.array(rows, dtype=np.float64).reshape(-1, 15)
        self.a = arr[:, 0:3].astype(np.float32)
        self.b = arr[:, 3:6].astype(np.float32)
        self.ref = arr[:, 6:10].astype(np.float32)
        self.layer = arr[:, 10].astype(np.int16)
        self.style = arr[:, 11].astype(np.int16)
        self.chain = arr[:, 12].astype(np.int32)
        self.inter = arr[:, 13].astype(bool)
        self.detail = arr[:, 14].astype(bool)
        self.minx = np.minimum(self.ref[:, 0], self.ref[:, 2])
        self.maxx = np.maximum(self.ref[:, 0], self.ref[:, 2])
        self.miny = np.minimum(self.ref[:, 1], self.ref[:, 3])
        self.maxy = np.maximum(self.ref[:, 1], self.ref[:, 3])

    def __len__(self):
        return len(self.a)

    def near(self, hx, hy, r, on, no_inter=False, detail_r=None):
        """(indices, distances) of rows within ``r`` whose layer is on; detail
        rows (labels, lane centres) only within ``detail_r``."""
        m = on[self.layer]
        if no_inter:
            m &= ~self.inter
        m &= (self.minx <= hx + r) & (self.maxx >= hx - r) & (self.miny <= hy + r) & (self.maxy >= hy - r)
        idx = np.nonzero(m)[0]
        if not len(idx):
            return idx, np.zeros(0)
        f = self.ref[idx]
        d = seg_dist(hx, hy, f[:, 0], f[:, 1], f[:, 2], f[:, 3])
        k = d <= r
        if detail_r is not None:
            k &= ~self.detail[idx] | (d <= detail_r)
        return idx[k], d[k]


def _lane_type(wp):
    return str(wp.lane_type)


def _dashed(marking):
    """Is a lane marking painted broken (dashed)? / 破線の標示か"""
    try:
        t = str(marking.type)
    except (AttributeError, RuntimeError):
        return False
    return t.startswith("Broken") or t == "BottsDots"


def _lc_bits(wp):
    """(right allowed, left allowed) of a waypoint's lane change. / 車線変更の可否"""
    v = int(wp.lane_change)
    return bool(v & 1), bool(v & 2)


# --------------------------------------------------------------------------
# precompute / 事前計算
# --------------------------------------------------------------------------
# heights above the lane (m): a little above 0.1 so small mesh / xodr offsets
# do not hide the lines (OitaKosen: 99 % of lanes within 0.5 m of the ground)
# 車線からの高さ（メッシュとのずれで隠れないよう少し上げる）
Z_EDGE, Z_CENTRE, Z_ARROW, Z_TEXT, Z_BAR, Z_OSM = 0.15, 0.17, 0.20, 0.18, 0.19, 0.35
ARROW_EVERY, SPEED_EVERY, SPEED_FIRST = 10.0, 60.0, 12.0
DRIVABLE = ("Driving", "Bidirectional")


class RoadGeometry(object):
    """Everything static of the overlay, built once from a carla.Map.
    マップから一度だけ作る静的な形状。"""

    def __init__(self, wmap, step=2.0, log=print):
        t0 = time.time()
        self.wmap, self.step = wmap, step
        self.name = wmap.name.split("/")[-1]
        self.speed = SpeedLookup(wmap)
        lines, arrows = LineBuf(), []
        self.chain_of = {}              # (road, lane) -> chain id / 道路・車線ごとの番号
        groups, others = {}, {}
        wps = wmap.generate_waypoints(step)
        for wp in wps:
            groups.setdefault((wp.road_id, wp.section_id, wp.lane_id), []).append(wp)
            for side in (0, 1):
                x = wp.get_left_lane() if side == 0 else wp.get_right_lane()
                hops = 0
                while x is not None and hops < 6:
                    lt = _lane_type(x)
                    if lt == "Driving":            # has its own waypoints / 自分の点がある
                        break
                    if lt != "NONE":
                        others.setdefault((x.road_id, x.section_id, x.lane_id), {})[round(x.s, 2)] = x
                    x = x.get_left_lane() if side == 0 else x.get_right_lane()
                    hops += 1
        self.n_waypoints = len(wps)
        self.boxes, seen_j = [], set()
        self.labels = []              # (x, y, z, text, layer, style) for draw_string / 文字ラベル
        speed_chains = {}
        for key, seq in groups.items():
            seq.sort(key=lambda w: w.s, reverse=key[2] > 0)       # travel order / 走行順
            chain = self.chain_of.setdefault((key[0], key[2]), len(self.chain_of))
            junction = seq[0].is_junction
            self._lane(lines, arrows, key, seq, chain, junction, drive=True)
            if junction and seq[0].junction_id not in seen_j:
                seen_j.add(seq[0].junction_id)
                self._junction(lines, seq[0])
            elif not junction:
                speed_chains.setdefault((key[0], key[2]), []).append((key[1], seq))
            mid = seq[len(seq) // 2]
            self._ids(lines, mid, "R%d L%d" % (key[0], key[2]), chain)
        for key, by_s in others.items():
            seq = sorted(by_s.values(), key=lambda w: w.s, reverse=key[2] > 0)
            self._lane(lines, arrows, key, seq, -1, False, drive=False)
        n_speed = 0
        for (road, lane), secs in speed_chains.items():
            secs.sort(key=lambda t: t[0], reverse=lane > 0)
            n_speed += self._speed_labels(lines, [w for _, seq in secs for w in seq])
        self.n_lanes, self.n_other = len(groups), len(others)
        self.n_junctions, self.n_speed = len(self.boxes), n_speed
        self.signs = self._landmarks(lines)
        self.lines = lines.freeze()
        for sg in self.signs:                 # sign names for the server window / 標識名
            txt = {"206": "STOP", "205": "YIELD"}.get(sg["type"], str(sg["value"] or ""))
            if txt:
                self.labels.append((sg["x"], sg["y"], sg["z"] + 3.2, txt, "rules",
                                    "stop_bar" if sg["type"] == "206" else "speed"))
        self.label_xy = np.array([[t[0], t[1]] for t in self.labels], np.float32).reshape(-1, 2)
        self.label_layer = np.array([LAYERS.index(t[4]) for t in self.labels], np.int16)
        if arrows:
            a = np.array(arrows, dtype=np.float64)
            self.arrow_a, self.arrow_b = a[:, 0:3].astype(np.float32), a[:, 3:6].astype(np.float32)
            self.arrow_style, self.arrow_chain = a[:, 6].astype(np.int16), a[:, 7].astype(np.int32)
        else:
            self.arrow_a = self.arrow_b = np.zeros((0, 3), np.float32)
            self.arrow_style = np.zeros(0, np.int16)
            self.arrow_chain = np.zeros(0, np.int32)
        self.arrow_mid = (self.arrow_a[:, :2] + self.arrow_b[:, :2]) / 2.0
        if self.boxes:
            self.box_xy = np.array([[b[1], b[2]] for b in self.boxes], np.float32)
            self.box_rad = np.array([b[3] for b in self.boxes], np.float32)
        else:
            self.box_xy, self.box_rad = np.zeros((0, 2), np.float32), np.zeros(0, np.float32)
        # lane centre points for the OSM layer's heights / OSM 用の高さ参照
        self.centre_pts = np.array([[w.transform.location.x, w.transform.location.y, w.transform.location.z]
                                    for w in wps], np.float32).reshape(-1, 3)
        self.seconds = time.time() - t0
        log("[road_overlay] %s: %d lanes (+%d other), %d junctions, %d lines, %d arrows, %d signs "
            "in %.1f s / 事前計算" % (self.name, self.n_lanes, self.n_other, self.n_junctions,
                                     len(self.lines), len(self.arrow_a), len(self.signs), self.seconds))

    # -- one lane / 1 車線
    def _lane(self, lines, arrows, key, seq, chain, junction, drive):
        road, sec, lane = key
        own_inner = abs(lane) == 1
        if own_inner and lane > 0:
            nb = seq[len(seq) // 2].get_left_lane()
            own_inner = nb is None or _lane_type(nb) == "NONE" or nb.lane_id > 0
        pts = {"outer": [], "inner": [], "centre": []}
        attr = {"outer": [], "inner": []}
        fwd = []
        for wp in seq:
            self._sample(wp, pts, attr, fwd, own_inner, drive)
        if not drive:
            st = _lane_type(seq[0])
            st = st if st in OTHER_TYPES else "other"
        tails = []
        last = seq[-1]
        try:
            for nx in last.next(self.step):
                if (nx.road_id, nx.section_id, nx.lane_id) != key and \
                        nx.transform.location.distance(last.transform.location) < 2.5 * self.step:
                    tails.append(nx)
        except RuntimeError:
            pass
        # inside a junction a lane is drawn as one orange path (its centre),
        # not two edges: the connectors overlap and their edges were a web of
        # short pieces (OitaKosen: 20 000 of 31 000 road lines)
        # 交差点内の車線は中心線 1 本（端の線は重なり合って多すぎた）
        for name in (() if drive and junction else ("outer", "inner")):
            P = pts[name]
            if not P:
                continue
            A = attr[name]
            polys = [(P, A)]
            for nx in tails:                                     # into the next lane / 次の車線へ
                tp, ta, tf = {"outer": [], "inner": [], "centre": []}, {"outer": [], "inner": []}, []
                self._sample(nx, tp, ta, tf, own_inner, drive)
                if not tp[name]:
                    continue
                if len(tails) == 1:                              # one way on: same polyline / 1 本に
                    polys[0] = (P + [tp[name][0]], A + [A[-1]])
                else:
                    polys.append(([P[-1], tp[name][0]], [A[-1]]))
            for P_, A_ in polys:
                self._runs(lines, P_, A_, drive, junction, chain, None if drive else st)
        if drive:
            C = list(pts["centre"])
            ends = [(nx.transform.location.x, nx.transform.location.y, nx.transform.location.z + Z_CENTRE)
                    for nx in tails]
            if len(ends) == 1:
                C.append(ends[0])
                ends = []
            if junction:
                layer, style, tol, det = "road", "edge_j", 0.3, False
            else:
                layer, style, tol, det = "direction", "centre", 0.25, True
            keep = simplify(C, tol, 50.0)
            for i, j in zip(keep, keep[1:]):
                lines.add(C[i], C[j], layer, style, chain, detail=det)
            for e in ends:
                lines.add(C[-1], e, layer, style, chain, detail=det)
            C = pts["centre"]
            run, nxt = 0.0, ARROW_EVERY / 2.0
            for i in range(len(C)):
                if i:
                    run += math.dist(C[i - 1][:2], C[i][:2])
                if run >= nxt:
                    f = fwd[i]
                    c = C[i]
                    arrows.append((c[0] - f[0], c[1] - f[1], c[2] + Z_ARROW - Z_CENTRE,
                                   c[0] + f[0], c[1] + f[1], c[2] + Z_ARROW - Z_CENTRE,
                                   SID["arrow"], chain))
                    nxt += ARROW_EVERY

    def _sample(self, wp, pts, attr, fwd, own_inner, drive):
        tf = wp.transform
        l, r, f = tf.location, tf.get_right_vector(), tf.get_forward_vector()
        h = wp.lane_width / 2.0
        pts["outer"].append((l.x + r.x * h, l.y + r.y * h, l.z + Z_EDGE))
        pts["centre"].append((l.x, l.y, l.z + Z_CENTRE))
        fn = math.hypot(f.x, f.y) or 1.0
        fwd.append((f.x / fn, f.y / fn))
        right_ok, left_ok = _lc_bits(wp) if drive else (False, False)
        nb = wp.get_right_lane()
        inter = drive and nb is not None and _lane_type(nb) in DRIVABLE
        code, dashed = 0, False
        if inter:
            code = int(right_ok) + 2 * int(_lc_bits(nb)[1])
            dashed = _dashed(wp.right_lane_marking)
        attr["outer"].append((inter, code, dashed))
        if own_inner:
            pts["inner"].append((l.x - r.x * h, l.y - r.y * h, l.z + Z_EDGE))
            nb = wp.get_left_lane()
            inter = drive and nb is not None and _lane_type(nb) in DRIVABLE
            code, dashed = 0, False
            if inter:
                code = int(left_ok) + 2 * int(_lc_bits(nb)[1])
                dashed = _dashed(wp.left_lane_marking)
            attr["inner"].append((inter, code, dashed))

    def _runs(self, lines, P, A, drive, junction, chain, other_style):
        """Split a boundary where its kind changes, simplify each run, add the
        road lines and the lane-change lines. / 境界線を種類ごとに分けて追加"""
        n = len(P)
        if n < 2:
            return
        i = 0
        while i < n - 1:
            j = i + 1
            while j < n - 1 and A[j] == A[i]:
                j += 1
            run = P[i:j + 1]
            inter, code, dashed = A[i]
            if drive:
                st = ("edge_j" if junction else "edge") + ("_inner" if inter else "")
            else:
                st = other_style
            keep = simplify(run, 0.15, 40.0)       # straight runs -> long chords / 直線は長く
            layer = "road" if drive else "lanes"
            for a, b in zip(keep, keep[1:]):
                lines.add(run[a], run[b], layer, st, chain, inter)
            if drive and inter:
                # colour: may the AI change lanes here (waypoint.lane_change);
                # look: the painted marking, dashed = broken paint
                # 色は AI の車線変更可否、破線か実線かは路面の標示
                lc = ("lc_none", "lc_one", "lc_one", "lc_both")[code]
                if not dashed:
                    for a, b in zip(keep, keep[1:]):
                        lines.add(run[a], run[b], "lane_change", lc, chain)
                else:                                   # dashes 4 m on, 4 m off / 破線
                    for k in range(0, len(run) - 1, 4):
                        e = min(k + 2, len(run) - 1)
                        lines.add(run[k], run[e], "lane_change", lc, chain)
            i = j

    def _junction(self, lines, wp):
        try:
            j = wp.get_junction()
        except RuntimeError:
            return
        if j is None:
            return
        bb = j.bounding_box
        c = bb.location
        rad = math.hypot(bb.extent.x, bb.extent.y)
        self.boxes.append((bb, c.x, c.y, rad, j.id))
        z = c.z + Z_TEXT
        for a, b in text_segments("J%d" % j.id, (c.x, c.y, z), (1, 0, 0), (0, -1, 0), 2.0):
            lines.add(a, b, "ids", "ids", ref=(c.x, c.y))
        self.labels.append((c.x, c.y, z + 1.0, "J%d" % j.id, "ids", "ids"))

    def _ids(self, lines, wp, text, chain):
        tf = wp.transform
        l, f, r = tf.location, tf.get_forward_vector(), tf.get_right_vector()
        if wp.is_junction:
            text += " J%d" % wp.junction_id
        for a, b in text_segments(text, (l.x, l.y, l.z + Z_TEXT + 0.01), (f.x, f.y, 0.0), (-r.x, -r.y, 0.0), 0.7):
            lines.add(a, b, "ids", "ids", ref=(l.x, l.y))
        self.labels.append((l.x, l.y, l.z + 0.5, text, "ids", "ids"))

    def _speed_labels(self, lines, seq):
        """The lane limit written on the road every 60 m. / 60 m ごとに制限速度"""
        if len(seq) < 2:
            return 0
        cum = [0.0]
        for a, b in zip(seq, seq[1:]):
            la, lb = a.transform.location, b.transform.location
            cum.append(cum[-1] + math.hypot(lb.x - la.x, lb.y - la.y))
        total = cum[-1]
        if total < 8.0:
            return 0
        targets = [total / 2.0] if total < SPEED_FIRST + 4.0 else \
            [SPEED_FIRST + k * SPEED_EVERY for k in range(int((total - SPEED_FIRST) // SPEED_EVERY) + 1)]
        n, i = 0, 0
        for t in targets:
            while i < len(seq) - 1 and cum[i] < t:
                i += 1
            wp = seq[i]
            kmh = self.speed.kmh(self.wmap, wp)
            if not kmh:
                continue
            tf = wp.transform
            l, f, r = tf.location, tf.get_forward_vector(), tf.get_right_vector()
            for a, b in text_segments(str(kmh), (l.x, l.y, l.z + Z_TEXT), (r.x, r.y, 0.0), (f.x, f.y, 0.0),
                                      2.4, 1.0, gap=0.35):
                lines.add(a, b, "rules", "speed", ref=(l.x, l.y))
            self.labels.append((l.x, l.y, l.z + 0.5, "%d" % kmh, "rules", "speed"))
            n += 1
        return n

    def _landmarks(self, lines):
        """Stop (206), give way (205), speed (274): a bar across the lane they
        apply to (stop / give way) and a sign drawn facing you (refresh).
        止まれ・譲れ・速度標識"""
        signs, seen = [], set()
        try:
            lms = self.wmap.get_all_landmarks()
        except RuntimeError:
            lms = []
        for lm in lms:
            if lm.type not in ("206", "205", "274"):
                continue
            loc = lm.transform.location
            k = (lm.type, round(loc.x), round(loc.y))
            if k in seen:
                continue
            seen.add(k)
            value = None
            if lm.type == "274":
                try:
                    value = int(round(float(lm.value)))
                    if lm.unit == "mph":
                        value = int(round(value * 1.609344))
                except (TypeError, ValueError):
                    value = None
            wp = lm.waypoint
            if wp is None:
                lane = -1 if str(lm.orientation) == "Positive" else 1
                try:
                    wp = self.wmap.get_waypoint_xodr(lm.road_id, lane, lm.s)
                except RuntimeError:
                    wp = None
                if wp is None:
                    wp = self.wmap.get_waypoint(loc)
            if wp is not None and lm.type in ("206", "205"):
                tf = wp.transform
                l, r = tf.location, tf.get_right_vector()
                h = wp.lane_width / 2.0
                lines.add((l.x - r.x * h, l.y - r.y * h, l.z + Z_BAR), (l.x + r.x * h, l.y + r.y * h, l.z + Z_BAR),
                          "rules", "stop_bar" if lm.type == "206" else "yield_bar")
            base_z = wp.transform.location.z if wp is not None else loc.z
            signs.append({"type": lm.type, "x": loc.x, "y": loc.y, "z": base_z, "value": value})
        return signs


# --------------------------------------------------------------------------
# raw OSM ways (generated maps) / 元の OSM 道路
# --------------------------------------------------------------------------
DRIVE_HIGHWAYS = {"motorway", "trunk", "primary", "secondary", "tertiary", "unclassified", "residential",
                  "service", "living_street", "road", "busway", "motorway_link", "trunk_link",
                  "primary_link", "secondary_link", "tertiary_link"}


def osm_source(map_name, maps_json=MAPS_JSON, build_dir=BUILD_DIR):
    """(osm path, meta dict) of a generated map, or (None, reason).
    生成マップの OSM と meta.json（無ければ理由）。"""
    meta = _read_json(os.path.join(build_dir, map_name, "meta.json"))
    if not isinstance(meta, dict) or "lat0" not in meta:
        return None, "no build dir"
    entries = _read_json(maps_json) or {}
    for e in entries.values():
        if isinstance(e, dict) and e.get("name") == map_name and e.get("osm"):
            p = os.path.expanduser(e["osm"])
            if os.path.isfile(p):
                return p, meta
    return None, "no osm"


def osm_ways(path, meta, highways=DRIVE_HIGHWAYS):
    """[[(x, y), ...]] of the drivable highway ways in CARLA coordinates
    (x east, y = -north), projected like maps/osm_city/geo.py.
    車道の OSM ウェイを CARLA 座標で（geo.py と同じ投影）。"""
    sys.path.insert(0, os.path.join(HERE, "maps", "osm_city"))
    try:
        from geo import Frame
    finally:
        sys.path.pop(0)
    fr = Frame(meta)
    nodes, ways = {}, []
    for _ev, el in ET.iterparse(path, events=("end",)):
        if el.tag == "node":
            try:
                nodes[el.get("id")] = (float(el.get("lon")), float(el.get("lat")))
            except (TypeError, ValueError):
                pass
            el.clear()
        elif el.tag == "way":
            hw = None
            refs = []
            for ch in el:
                if ch.tag == "nd":
                    refs.append(ch.get("ref"))
                elif ch.tag == "tag" and ch.get("k") == "highway":
                    hw = ch.get("v")
            if hw in highways:
                pts = []
                for r in refs:
                    if r in nodes:
                        x, yn = fr.xy(*nodes[r])
                        pts.append((x, -yn))
                if len(pts) >= 2:
                    ways.append(pts)
            el.clear()
        elif el.tag == "relation":
            el.clear()
    return ways


def osm_lines(ways, centre_pts, piece=8.0, cell=10.0):
    """LineTable of the ways, each point lifted to the nearest lane's height.
    OSM ウェイの線（高さは最寄りの車線）。"""
    grid = {}
    zs = centre_pts[:, 2] if len(centre_pts) else np.zeros(1)
    z_default = float(np.median(zs)) if len(zs) else 0.0
    for i, (x, y) in enumerate(centre_pts[:, :2]):
        grid.setdefault((int(math.floor(x / cell)), int(math.floor(y / cell))), []).append(i)

    def height(x, y):
        cx, cy = int(math.floor(x / cell)), int(math.floor(y / cell))
        for ring in (1, 2, 3):
            best, bd = None, 1e18
            for gx in range(cx - ring, cx + ring + 1):
                for gy in range(cy - ring, cy + ring + 1):
                    for i in grid.get((gx, gy), ()):
                        d = (centre_pts[i, 0] - x) ** 2 + (centre_pts[i, 1] - y) ** 2
                        if d < bd:
                            best, bd = i, d
            if best is not None:
                return float(centre_pts[best, 2])
        return z_default

    buf = LineBuf()
    for w in ways:
        dense = [w[0]]
        for (ax, ay), (bx, by) in zip(w, w[1:]):
            n = max(1, int(math.ceil(math.hypot(bx - ax, by - ay) / piece)))
            for k in range(1, n + 1):
                dense.append((ax + (bx - ax) * k / n, ay + (by - ay) * k / n))
        p3 = [(x, y, height(x, y) + Z_OSM) for x, y in dense]
        for a, b in zip(p3, p3[1:]):
            buf.add(a, b, "osm", "osm")
    return buf.freeze()


# --------------------------------------------------------------------------
# the overlay: select and draw / 選んで描く
# --------------------------------------------------------------------------
def _light_state_name(light):
    try:
        return str(light.get_state())
    except RuntimeError:
        return "Unknown"


class Overlay(object):
    """Draws a RoadGeometry around a point with a DebugHelper-like object.
    world: carla.World (or a fake with get_actors()); debug: world.debug.
    道路ロジックを指定位置の周りに描く。"""

    def __init__(self, world, wmap, debug=None, geometry=None, log=print):
        self.world, self.wmap, self.log = world, wmap, log
        self.debug = debug if debug is not None else world.debug
        self.geo = geometry or RoadGeometry(wmap, log=log)
        self.lights = self._lights()
        self.osm_table, self.osm_state = None, "off"
        self._osm_thread = None
        self._colors = None
        self._zones_mtime = None
        self._zones_data = {"pods": {}, "nofly": []}
        self.faces = ZoneFaces(world, log=log)

    def _ground_z(self, x, y):
        pts = self.geo.centre_pts
        if len(pts) == 0:
            return 0.0
        d2 = (pts[:, 0] - x) ** 2 + (pts[:, 1] - y) ** 2
        near = np.nonzero(d2 < 120.0 * 120.0)[0]
        if not len(near):
            return float(pts[np.argmin(d2), 2])
        w = 1.0 / np.maximum(1.0, np.sqrt(d2[near]))
        return float(np.sum(pts[near, 2] * w) / np.sum(w))

    def _load_zones(self):
        try:
            mtime = os.path.getmtime(ZONES_PATH)
        except OSError:
            mtime = 0
        if self._zones_mtime != mtime:
            self._zones_mtime = mtime
            try:
                from delivery import zones as delivery_zones
                self._zones_data = delivery_zones.load(self.geo.name, ZONES_PATH)
            except Exception as exc:                   # noqa: BLE001
                self.log("[road_overlay] zones load: %s" % exc)
                self._zones_data = {"pods": {}, "nofly": []}
        return self._zones_data

    # -- static lights / 信号機
    def _lights(self):
        out = []
        try:
            actors = self.world.get_actors().filter("traffic.traffic_light")
        except RuntimeError:
            return out
        for tl in actors:
            try:
                base = tl.get_location()
                bars = []
                for wp in tl.get_stop_waypoints():
                    tf = wp.transform
                    l, r = tf.location, tf.get_right_vector()
                    h = wp.lane_width / 2.0
                    bars.append(((l.x - r.x * h, l.y - r.y * h, l.z + Z_BAR),
                                 (l.x + r.x * h, l.y + r.y * h, l.z + Z_BAR), (l.x, l.y, l.z + Z_BAR)))
            except Exception as exc:                   # noqa: BLE001
                self.log("[road_overlay] light %s: %s" % (getattr(tl, "id", "?"), exc))
                continue
            out.append({"actor": tl, "x": base.x, "y": base.y, "z": base.z, "bars": bars})
        return out

    # -- OSM layer, built in the background the first time / 初回に裏で作る
    def ensure_osm(self):
        if self.osm_state != "off":
            return
        path, meta = osm_source(self.geo.name)
        if path is None:
            self.osm_state = "none (%s)" % meta
            return
        self.osm_state = "loading"

        def work():
            try:
                t0 = time.time()
                ways = osm_ways(path, meta)
                self.osm_table = osm_lines(ways, self.geo.centre_pts)
                self.osm_state = "ok"
                self.log("[road_overlay] OSM: %d ways, %d lines in %.1f s" % (len(ways), len(self.osm_table),
                                                                              time.time() - t0))
            except Exception as exc:                   # noqa: BLE001
                self.osm_state = "error: %s" % exc
                self.log("[road_overlay] OSM layer failed: %s" % exc)

        self._osm_thread = threading.Thread(target=work, daemon=True)
        self._osm_thread.start()

    def colors(self, level):
        """carla.Color per style at this brightness (hero styles a step up).
        明るさごとのスタイル色。"""
        key = round(level, 2)
        if self._colors is None or self._colors[0] != key:
            hero = {SID["edge_hero"], SID["centre_hero"], SID["arrow_hero"]}
            self._colors = (key, [self.color(h, level * (HERO_GAIN if i in hero else 1.0))
                                  for i, (_, h, _) in enumerate(STYLE_LIST)])
        return self._colors[1]

    def color(self, hue, level):
        rgb = channel(hue, level)
        return carla.Color(*rgb) if carla is not None else rgb

    # -- per refresh / 毎回
    def _sign_shapes(self, s, hx, hy):
        """Lines of a sign drawn upright, facing (hx, hy). / こちらを向いた標識"""
        vx, vy = s["x"] - hx, s["y"] - hy
        n = math.hypot(vx, vy) or 1.0
        right, up = (-vy / n, vx / n, 0.0), (0.0, 0.0, 1.0)
        head = (s["x"], s["y"], s["z"] + 2.6)
        out = [((s["x"], s["y"], s["z"]), (head[0], head[1], head[2] - 0.55), None, 0.04)]

        def ring(k, rad, rot):
            pts = [(head[0] + right[0] * rad * math.cos(rot + 2 * math.pi * i / k),
                    head[1] + right[1] * rad * math.cos(rot + 2 * math.pi * i / k),
                    head[2] + rad * math.sin(rot + 2 * math.pi * i / k)) for i in range(k + 1)]
            return list(zip(pts, pts[1:]))

        if s["type"] == "206":
            rgb, segs, label = STOP_HUE, ring(8, 0.5, math.pi / 8), "STOP"
        elif s["type"] == "205":
            rgb, segs, label = YIELD_HUE, ring(3, 0.55, -math.pi / 2), "YIELD"
        else:
            rgb, segs, label = SPEED_SIGN_HUE, ring(12, 0.5, 0.0), None
            if s["value"]:
                segs += [(a, b) for a, b in text_segments(str(s["value"]), head, right, up, 0.42, 0.24)]
        out[0] = out[0][:2] + (rgb, 0.03)
        out += [(a, b, rgb, 0.04) for a, b in segs]
        if label:
            out += [(a, b, rgb, 0.03) for a, b in
                    text_segments(label, (head[0], head[1], head[2] - 0.9), right, up, 0.32)]
        return out

    def _light_shapes(self, L, detail=True):
        """A light's stop bars in its colour; near you also its pole and a link
        to each bar. / 信号の停止線（近くでは柱と停止線への線も）"""
        rgb = LIGHT_HUE.get(_light_state_name(L["actor"]), LIGHT_OFF)
        top = (L["x"], L["y"], L["z"] + 4.0)
        out = [((L["x"], L["y"], L["z"]), top, rgb, 0.04)] if detail else []
        for a, b, c in L["bars"]:
            out.append((a, b, rgb, 0.12))
            if detail:
                out.append((top, c, rgb, 0.02))
        return out, top, rgb

    def refresh(self, ctl, centre, hero_wp=None, life=1.3, night=False, spectator=None):
        """Draw one refresh. Returns stats. / 1 回分を描き、統計を返す。

        ``centre``: (x, y[, z]) of your car, or None (no hero); ``spectator``:
        the area the CarlaUE4 window looks at, {x, y, r, thick} (see
        spectator_area), or None. Both areas share ``max_shapes``: items are
        ranked by distance / area radius. Near your car: stroked text, arrows,
        sign details (what the driving camera sees); in the spectator area:
        draw_string labels (the server window shows them, cameras do not) and
        thicker lines that read from above.
        自車の周りと CarlaUE4 ウィンドウのカメラの周りに描く（線数は共有）。"""
        t0 = time.time()
        L = ctl["layers"]
        cap = int(ctl["max_shapes"])
        r0 = float(ctl["radius"])
        rd = min(r0, float(ctl.get("detail_radius", DEFAULTS["detail_radius"])))
        level = max(1.0, float(ctl.get("brightness", DEFAULTS["brightness"])) * (NIGHT_DIM if night else 1.0))
        on = np.array([bool(L.get(k)) for k in LAYERS], bool)
        if L.get("osm"):
            self.ensure_osm()
        geo = self.geo
        hero_chain = -1
        if hero_wp is not None:
            hero_chain = geo.chain_of.get((hero_wp.road_id, hero_wp.lane_id), -1)
        areas = []                    # (x, y, radius, detail radius, thickness x, strings) / 描く範囲
        if centre is not None:
            areas.append((float(centre[0]), float(centre[1]), r0, rd, 1.0, False))
        if spectator is not None:
            sx, sy, sr = float(spectator["x"]), float(spectator["y"]), float(spectator["r"])
            if not areas or math.hypot(sx - areas[0][0], sy - areas[0][1]) > r0 / 2.0:
                areas.append((sx, sy, sr, 0.0, float(spectator.get("thick", 1.0)), True))

        def best(parts):
            """Merge per-area (idx, d/r, area) lists, keeping each index once at
            its nearest area. / 範囲ごとの候補をまとめる。"""
            if not parts:
                return np.zeros(0, np.int64), np.zeros(0), np.zeros(0, np.int16)
            idx = np.concatenate([p[0] for p in parts])
            nd = np.concatenate([p[1] for p in parts])
            ar = np.concatenate([np.full(len(p[0]), p[2], np.int16) for p in parts])
            o = np.argsort(nd, kind="stable")
            idx, nd, ar = idx[o], nd[o], ar[o]
            _, first = np.unique(idx, return_index=True)
            return idx[first], nd[first], ar[first]

        # candidates: (normalised distance, kind, index, area) / 候補
        cand = []
        tables = [geo.lines] + ([self.osm_table] if L.get("osm") and self.osm_table is not None else [])
        for ti, T in enumerate(tables):
            parts = []
            for ai, (ax, ay, ar_, ad, _th, _st) in enumerate(areas):
                i, d = T.near(ax, ay, ar_, on, no_inter=bool(L.get("lane_change")), detail_r=ad)
                parts.append((i, d / ar_, ai))
            cand.append((ti,) + best(parts))
        if L.get("direction") and len(geo.arrow_mid) and areas and not areas[0][5]:
            ax, ay, ar_, ad = areas[0][:4]                 # arrows only near your car / 矢印は自車の近く
            d = np.hypot(geo.arrow_mid[:, 0] - ax, geo.arrow_mid[:, 1] - ay)
            i = np.nonzero(d <= ad)[0]
            cand.append((100, i, d[i] / ar_, np.zeros(len(i), np.int16)))
        if L.get("junctions") and len(geo.box_xy):
            parts = []
            for ai, (ax, ay, ar_, _ad, _th, _st) in enumerate(areas):
                d = np.maximum(0.0, np.hypot(geo.box_xy[:, 0] - ax, geo.box_xy[:, 1] - ay) - geo.box_rad)
                i = np.nonzero(d <= ar_)[0]
                parts.append((i, d[i] / ar_, ai))
            cand.append((101,) + best(parts))
        if len(geo.label_xy) and any(a[5] for a in areas):
            parts = []
            lab_on = on[geo.label_layer]
            for ai, (ax, ay, ar_, _ad, _th, st_) in enumerate(areas):
                if not st_:
                    continue
                d = np.hypot(geo.label_xy[:, 0] - ax, geo.label_xy[:, 1] - ay)
                i = np.nonzero(lab_on & (d <= ar_))[0]
                parts.append((i, d[i] / ar_, ai))
            cand.append((104,) + best(parts))
        extra = []                    # (a, b, hue, thickness, area) / 標識・信号
        boxes_extra = []
        if L.get("rules"):
            ed, eb = [], []
            for s in geo.signs:
                for ai, (ax, ay, ar_, ad, _th, _st) in enumerate(areas):
                    d = math.hypot(s["x"] - ax, s["y"] - ay)
                    if d <= ad:                        # bars are static lines / 停止線は静的
                        for sh in self._sign_shapes(s, ax, ay):
                            extra.append(sh + (ai,))
                            ed.append(d / ar_)
                        break
            for Lt in self.lights:
                for ai, (ax, ay, ar_, ad, _th, _st) in enumerate(areas):
                    d = math.hypot(Lt["x"] - ax, Lt["y"] - ay)
                    if d <= ar_:
                        shapes, top, rgb = self._light_shapes(Lt, d <= ad)
                        for sh in shapes:
                            extra.append(sh + (ai,))
                            ed.append(d / ar_)
                        if d <= ad:
                            boxes_extra.append((top, rgb))
                            eb.append(d / ar_)
                        break
            cand.append((102, np.arange(len(ed)), np.array(ed), np.zeros(len(ed), np.int16)))
            cand.append((103, np.arange(len(eb)), np.array(eb), np.zeros(len(eb), np.int16)))
        zone_boxes = []
        if L.get("zones", True):
            zd = self._load_zones()
            for nf in zd.get("nofly") or []:
                x0, y0, x1, y1 = nf["x0"], nf["y0"], nf["x1"], nf["y1"]
                cx, cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
                hx, hy = abs(x1 - x0) / 2.0, abs(y1 - y0) / 2.0
                zg = self._ground_z(cx, cy)
                cz, hz = zg + 30.5, 30.0
                name = nf.get("name") or "NO-FLY"
                zone_boxes.append({"kind": "nofly", "cx": cx, "cy": cy, "cz": cz,
                                   "hx": hx, "hy": hy, "hz": hz,
                                   "x0": x0, "y0": y0, "x1": x1, "y1": y1,
                                   "hue": (1.0, 0.0, 0.0), "name": name, "top_z": zg + 60.5})
            for pid_str, pz in (zd.get("pods") or {}).items():
                x0, y0, x1, y1 = pz["x0"], pz["y0"], pz["x1"], pz["y1"]
                cx, cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
                hx, hy = abs(x1 - x0) / 2.0, abs(y1 - y0) / 2.0
                zg = self._ground_z(cx, cy)
                cz, hz = zg + 13.0, 12.5
                try:
                    pid = int(pid_str)
                except (ValueError, TypeError):
                    pid = 1
                hue = POD_HUES[(pid - 1) % len(POD_HUES)]
                name = "HUB %s" % pid if pid == 4 else "POD %s" % pid
                zone_boxes.append({"kind": "pod", "pid": pid, "cx": cx, "cy": cy, "cz": cz,
                                   "hx": hx, "hy": hy, "hz": hz,
                                   "x0": x0, "y0": y0, "x1": x1, "y1": y1,
                                   "hue": hue, "name": name, "top_z": zg + 25.5})
            if zone_boxes:
                parts = []
                for ai, (ax, ay, ar_, _ad, _th, _st) in enumerate(areas):
                    d_list = []
                    for zb in zone_boxes:
                        dx = max(0.0, max(zb["x0"] - ax, ax - zb["x1"]))
                        dy = max(0.0, max(zb["y0"] - ay, ay - zb["y1"]))
                        d_list.append(math.hypot(dx, dy))
                    d_arr = np.array(d_list)
                    idx = np.nonzero(d_arr <= ar_ + 50.0)[0]
                    if len(idx):
                        parts.append((idx, d_arr[idx] / ar_, ai))
                cand.append((105,) + best(parts))

        D = np.concatenate([c[2] for c in cand]) if cand else np.zeros(0)
        K = np.concatenate([np.full(len(c[1]), c[0], np.int16) for c in cand]) if cand else np.zeros(0, np.int16)
        I = np.concatenate([c[1] for c in cand]).astype(np.int64) if cand else np.zeros(0, np.int64)
        A_ = np.concatenate([c[3] for c in cand]).astype(np.int16) if cand else np.zeros(0, np.int16)
        candidates = len(D)
        order = np.argsort(D, kind="stable")[:cap]
        K, I, A_ = K[order], I[order], A_[order]
        thick = np.array([a[4] for a in areas] or [1.0])
        t_sel = time.time() - t0

        dbg = self.debug
        colors = self.colors(level)
        Loc = carla.Location if carla is not None else (lambda x, y, z: (x, y, z))
        n = 0
        by = {}
        for ti, T in enumerate(tables):
            m = K == ti
            sel, sa = I[m], A_[m]
            if not len(sel):
                continue
            st = T.style[sel]
            if not L.get("junctions"):
                st = REMAP_NOJ[st]
            if hero_chain >= 0:
                hit = (T.chain[sel] == hero_chain) & (sa == 0)
                st = np.where(hit, REMAP_HERO[st], st)
            A, B, lay = T.a[sel].tolist(), T.b[sel].tolist(), T.layer[sel].tolist()
            for a, b, s, ly, th in zip(A, B, st.tolist(), lay, thick[sa].tolist()):
                dbg.draw_line(Loc(*a), Loc(*b), thickness=STYLE_LIST[s][2] * th, color=colors[s],
                              life_time=life, persistent_lines=False)
                by[LAYERS[ly]] = by.get(LAYERS[ly], 0) + 1
            n += len(sel)
        sel = I[K == 100]
        if len(sel):
            st = geo.arrow_style[sel]
            if hero_chain >= 0:
                st = np.where(geo.arrow_chain[sel] == hero_chain, REMAP_HERO[st], st)
            for a, b, s in zip(geo.arrow_a[sel].tolist(), geo.arrow_b[sel].tolist(), st.tolist()):
                dbg.draw_arrow(Loc(*a), Loc(*b), thickness=STYLE_LIST[s][2], arrow_size=0.5,
                               color=colors[s], life_time=life, persistent_lines=False)
            by["direction"] = by.get("direction", 0) + len(sel)
            n += len(sel)
        m = K == 101
        for k, ai in zip(I[m].tolist(), A_[m].tolist()):
            bb = geo.boxes[k][0]
            dbg.draw_box(bb, bb.rotation, thickness=STYLE_LIST[SID["jbox"]][2] * thick[ai],
                         color=colors[SID["jbox"]], life_time=life, persistent_lines=False)
        if m.any():
            by["junctions"] = by.get("junctions", 0) + int(m.sum())
            n += int(m.sum())
        sel = I[K == 102]
        for k in sel.tolist():
            a, b, rgb, th, ai = extra[k]
            dbg.draw_line(Loc(*a), Loc(*b), thickness=th * thick[ai], color=self.color(rgb, level),
                          life_time=life, persistent_lines=False)
        sel3 = I[K == 103]
        for k in sel3.tolist():
            top, rgb = boxes_extra[k]
            if carla is not None:
                bb = carla.BoundingBox(Loc(*top), carla.Vector3D(0.3, 0.3, 0.3))
                rot = carla.Rotation()
            else:
                bb, rot = top, None
            dbg.draw_box(bb, rot, thickness=0.04, color=self.color(rgb, level),
                         life_time=life, persistent_lines=False)
        if len(sel) or len(sel3):
            by["rules"] = by.get("rules", 0) + len(sel) + len(sel3)
            n += len(sel) + len(sel3)
        sel4 = I[K == 104]
        for k in sel4.tolist():
            # HUD text in the server window: not bloomed, full colour / サーバー画面の文字
            x, y, z, text, layer, style = geo.labels[k]
            rgb = display_rgb(STYLE_LIST[SID[style]][1])
            col = carla.Color(*rgb) if carla is not None else tuple(rgb)
            dbg.draw_string(Loc(x, y, z), text, draw_shadow=True, color=col, life_time=life,
                            persistent_lines=False)
        if len(sel4):
            by["labels"] = len(sel4)
            n += len(sel4)
        m5 = K == 105
        for k, ai in zip(I[m5].tolist(), A_[m5].tolist()):
            zb = zone_boxes[k]
            st_key = "nofly_zone" if zb["kind"] == "nofly" else "pod_zone"
            th_val = STYLE_LIST[SID[st_key]][2] * thick[ai]
            if carla is not None:
                bb = carla.BoundingBox(Loc(zb["cx"], zb["cy"], zb["cz"]),
                                       carla.Vector3D(zb["hx"], zb["hy"], zb["hz"]))
                rot = carla.Rotation()
            else:
                bb = type("_B", (), {
                    "location": type("_L", (), {"x": zb["cx"], "y": zb["cy"], "z": zb["cz"]})(),
                    "extent": type("_L", (), {"x": zb["hx"], "y": zb["hy"], "z": zb["hz"]})(),
                    "rotation": None,
                })()
                rot = None
            dbg.draw_box(bb, rot, thickness=th_val, color=self.color(zb["hue"], level),
                         life_time=life, persistent_lines=False)
        self.faces.sync(zone_boxes)
        if m5.any():
            by["zones"] = int(m5.sum())
            n += int(m5.sum())
        t_all = time.time() - t0
        return {"shapes": n, "candidates": candidates, "capped": candidates > cap, "by_layer": by,
                "select_ms": round(t_sel * 1000, 2), "draw_ms": round((t_all - t_sel) * 1000, 2),
                "ms": round(t_all * 1000, 2), "hero_chain": hero_chain, "osm": self.osm_state,
                "level": round(level, 2), "night": bool(night), "areas": len(areas),
                "spectator_area": bool(areas and areas[-1][5]),
                "spectator_radius": round(areas[-1][2], 1) if areas and areas[-1][5] else None}


def spectator_area(wmap, tf, base_radius, max_radius=400.0):
    """Where the CarlaUE4 window looks: {x, y, r, thick, height}. The ground
    point along the view (the spectator flies high and looks down), else a
    point ahead; radius and line thickness grow with the height above the road.
    CarlaUE4 ウィンドウのカメラが見ている地面の点と、高さに応じた半径・太さ。"""
    loc, f = tf.location, tf.get_forward_vector()
    ground = wmap.get_waypoint(loc)
    gz = ground.transform.location.z if ground is not None else 0.0
    h = max(0.0, loc.z - gz)
    r = min(max_radius, max(base_radius, 1.5 * h))
    if f.z < -0.17 and h > 1.0:
        t = min(h / -f.z, 3.0 * r)
        x, y = loc.x + f.x * t, loc.y + f.y * t
    else:
        n = math.hypot(f.x, f.y) or 1.0
        x, y = loc.x + f.x / n * r * 0.5, loc.y + f.y / n * r * 0.5
    return {"x": x, "y": y, "r": r, "thick": max(1.0, min(6.0, h / 25.0)), "height": round(h, 1)}


# --------------------------------------------------------------------------
# the helper process / ヘルパープロセス
# --------------------------------------------------------------------------
def find_hero(world, role="hero"):
    for a in world.get_actors().filter("vehicle.*"):
        if a.attributes.get("role_name") == role:
            return a
    return None


def write_status(st, path=None):
    st = dict(st, t=time.time())
    try:
        _write_json(path or STATUS_PATH, st)
    except OSError:
        pass


def geometry_status(geo):
    return {"map": geo.name, "ready": True, "precompute_s": round(geo.seconds, 2),
            "lanes": geo.n_lanes, "other_lanes": geo.n_other, "junctions": geo.n_junctions,
            "lines": len(geo.lines), "arrows": len(geo.arrow_a), "signs": len(geo.signs),
            "speed_labels": geo.n_speed}


def run(args):
    stop = {"flag": False}

    def on_term(_sig, _frm):
        stop["flag"] = True

    signal.signal(signal.SIGTERM, on_term)
    try:
        os.nice(5)                  # a background helper / 背景のヘルパー
    except OSError:
        pass
    print("[road_overlay] running; control %s / 起動" % args.control)
    client = ov = None
    world_id, base = None, {}
    while not stop["flag"]:
        try:
            if client is None:
                client = carla.Client(args.host, args.port)
                client.set_timeout(20.0)
            world = client.get_world()
            if ov is None or world.id != world_id:
                ov = None
                write_status({"ready": False, "building": True, "on": load_control(args.control)["on"]},
                             args.status)
                wmap = world.get_map()
                ov = Overlay(world, wmap)
                world_id, base = world.id, geometry_status(ov.geo)
                base["lights"] = len(ov.lights)
            hero, last_check, last_status = None, time.time(), 0.0
            last_real = last_sim = None
            night, night_t = False, 0.0
            lights_retry = time.time() + 10.0
            while not stop["flag"]:
                now = time.time()
                if now - last_check > 3.0:
                    last_check = now
                    if client.get_world().id != world_id:
                        print("[road_overlay] the world changed -> rebuilding / マップ変更")
                        break
                    if not ov.lights and now > lights_retry:
                        ov.lights = ov._lights()
                        base["lights"] = len(ov.lights)
                        lights_retry = now + 30.0
                ctl = load_control(args.control)
                if not ctl["on"]:
                    ov.faces.clear()
                    if now - last_status > 2.0:
                        write_status(dict(base, on=False, shapes=0), args.status)
                        last_status = now
                    last_real = last_sim = None
                    time.sleep(0.25)
                    continue
                if hero is None or not hero.is_alive:
                    hero = find_hero(world, args.role)
                loc = hero.get_location() if hero is not None else None
                hero_wp = ov.wmap.get_waypoint(loc) if loc is not None else None
                spec = None
                if ctl["spectator"] or loc is None:    # the CarlaUE4 window's camera / サーバー画面
                    spec = spectator_area(ov.wmap, world.get_spectator().get_transform(), ctl["radius"])
                if last_real is None:                  # first refresh: measure the sim rate / 初回に計測
                    last_real, last_sim = time.time(), world.get_snapshot().timestamp.elapsed_seconds
                    time.sleep(0.5)
                    now = time.time()
                sim = world.get_snapshot().timestamp.elapsed_seconds
                rate = max(0.0, (sim - last_sim) / max(1e-3, now - last_real))
                last_real, last_sim = now, sim
                # life_time counts sim seconds: one refresh of real time at the
                # measured sim rate, 20 % overlap / 寿命はシミュ時間：実時間の更新間隔に合わせる
                life = ctl["refresh"] * 1.2 * min(4.0, max(0.2, rate))
                if now - night_t > 10.0:
                    night_t = now
                    try:
                        night = world.get_weather().sun_altitude_angle < 0.0
                    except RuntimeError:
                        pass
                st = ov.refresh(ctl, (loc.x, loc.y, loc.z) if loc is not None else None, hero_wp, life, night,
                                spec)
                if spec is not None:
                    st["spectator_height"] = spec["height"]
                write_status(dict(base, on=True, hero=hero is not None, life=round(life, 2),
                                  sim_rate=round(rate, 2), radius=ctl["radius"], **st), args.status)
                last_status = now
                wake = now + ctl["refresh"]
                while not stop["flag"] and time.time() < wake:   # stays responsive to "off"
                    time.sleep(0.1)
                    if not load_control(args.control)["on"]:
                        break
        except RuntimeError as exc:
            print("[road_overlay] %s -> reconnecting / 再接続" % exc)
            client = ov = None
            write_status({"ready": False, "error": str(exc)[:200]}, args.status)
            for _ in range(20):
                if stop["flag"]:
                    break
                time.sleep(0.1)
    if ov is not None:
        ov.faces.clear()
    write_status({"ready": False, "stopped": True}, args.status)
    print("[road_overlay] stopped / 終了")


class _NullDebug(object):
    def __init__(self):
        self.calls = 0

    def _count(self, *a, **k):
        self.calls += 1

    draw_line = draw_arrow = draw_box = draw_point = draw_string = _count


def bench(xodr_path, radius=120.0, cap=None):
    """Precompute timing and shapes per refresh on an xodr, no server.
    サーバー無しで事前計算の時間と 1 回の形状数を測る。"""
    name = os.path.splitext(os.path.basename(xodr_path))[0]
    t0 = time.time()
    wmap = carla.Map(name, open(xodr_path).read())
    print("map load %.2f s" % (time.time() - t0))

    class W(object):
        def get_actors(self):
            class A(list):
                def filter(self, _p):
                    return []
            return A()

    dbg = _NullDebug()
    ov = Overlay(W(), wmap, debug=dbg)
    ctl = clean_control({"on": True, "radius": radius, "max_shapes": cap or DEFAULTS["max_shapes"]})
    cap = ctl["max_shapes"]
    pts = ov.geo.centre_pts[:: max(1, len(ov.geo.centre_pts) // 50)]
    stats = [ov.refresh(ctl, p) for p in pts]
    sh = [s["shapes"] for s in stats]
    print("precompute %.2f s; shapes/refresh mean %.0f max %d (cap %d); select %.1f ms, draw (no-op) %.1f ms"
          % (ov.geo.seconds, np.mean(sh), max(sh), cap, np.mean([s["select_ms"] for s in stats]),
             np.mean([s["draw_ms"] for s in stats])))
    return ov, stats


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--role", default="hero", help="role_name of the car to centre on")
    ap.add_argument("--control", default=CONTROL_PATH)
    ap.add_argument("--status", default=STATUS_PATH)
    ap.add_argument("--bench", metavar="XODR", help="time the precompute on an xodr (no server)")
    args = ap.parse_args()
    if args.bench:
        bench(args.bench)
        return
    try:
        run(args)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
