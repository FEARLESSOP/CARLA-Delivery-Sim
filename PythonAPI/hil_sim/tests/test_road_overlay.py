#!/usr/bin/env python3
"""Offline tests: the road logic overlay (road_overlay.py). No server: the maps
are carla.Map objects built from the xodr files, the world and its debug
helper are fakes that record every draw call.

  ../../carla-venv/bin/python tests/test_road_overlay.py

1. control file: defaults, clamping, atomic writes, toggle
2. keybind: O, no wheel button, no clash with any other default
3. geometry from OitaStation's xodr: edges at +-width/2, arrows along the lane,
   speed labels from the lane <speed>, stop bars, junction boxes
4. radius filter, shape cap (nearest first), layer switches, hero highlight
5. lights: stop bars in the light's current colour
6. raw OSM ways: projected onto the lanes (generated maps), skipped elsewhere
7. the helper loop: idle while off, draws when on, stops drawing when off,
   rebuilds after a map change, exits on SIGTERM
8. precompute timing on OitaStation and OitaKosen

道路ロジック表示のオフラインテスト（サーバー不要）。
"""

import math
import os
import signal
import sys
import tempfile
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
HIL = os.path.dirname(HERE)
sys.path.insert(0, HIL)

import numpy as np  # noqa: E402
import carla  # noqa: E402
import road_overlay as ro  # noqa: E402
import keybinds as kb  # noqa: E402

FAILS = []
CARLA_ROOT = os.path.dirname(os.path.dirname(HIL))
XODR = {n: os.path.join(HIL, "maps", "build", n, n + ".xodr") for n in ("OitaStation", "OitaKosen")}
XODR["Town03"] = os.path.join(CARLA_ROOT, "CarlaUE4", "Content", "Carla", "Maps", "OpenDrive", "Town03.xodr")


def check(name, cond, detail=""):
    print("  %s  %s%s" % ("PASS" if cond else "FAIL", name, (" | %s" % (detail,)) if detail != "" else ""))
    if not cond:
        FAILS.append(name)


class FakeDebug(object):
    """Records draw calls as (kind, a, b, thickness, rgb, life, persistent)."""

    def __init__(self):
        self.calls = []

    def draw_line(self, begin, end, thickness=0.1, color=None, life_time=-1.0, persistent_lines=True):
        self.calls.append(("line", (begin.x, begin.y, begin.z), (end.x, end.y, end.z), thickness,
                           (color.r, color.g, color.b), life_time, persistent_lines))

    def draw_arrow(self, begin, end, thickness=0.1, arrow_size=0.1, color=None, life_time=-1.0,
                   persistent_lines=True):
        self.calls.append(("arrow", (begin.x, begin.y, begin.z), (end.x, end.y, end.z), thickness,
                           (color.r, color.g, color.b), life_time, persistent_lines))

    def draw_box(self, box, rotation, thickness=0.1, color=None, life_time=-1.0, persistent_lines=True):
        c = box.location
        self.calls.append(("box", (c.x, c.y, c.z), (c.x, c.y, c.z), thickness,
                           (color.r, color.g, color.b), life_time, persistent_lines))

    def draw_string(self, location, text, draw_shadow=False, color=None, life_time=-1.0, persistent_lines=True):
        p = (location.x, location.y, location.z)
        self.calls.append(("string", p, p, text, (color.r, color.g, color.b), life_time, persistent_lines))

    def draw_point(self, *a, **k):
        self.calls.append(("point",) + (None,) * 6)


class Actors(list):
    def filter(self, pattern):
        p = pattern.rstrip("*")
        return Actors(a for a in self if a.type_id.startswith(p))


class FakeLight(object):
    type_id = "traffic.traffic_light"

    def __init__(self, loc, stops, state="Red"):
        self.id, self.loc, self.stops, self.state = 900, loc, stops, state
        self.is_alive = True
        self.attributes = {}

    def get_location(self):
        return self.loc

    def get_stop_waypoints(self):
        return self.stops

    def get_state(self):
        return self.state


class FakeWorld(object):
    def __init__(self, actors=()):
        self.actors = Actors(actors)
        self.debug = FakeDebug()

    def get_actors(self):
        return self.actors


def rgb(name, level=3.0):
    """The colour CARLA gets for a style at the default brightness (your lane
    a step brighter). / 既定の明るさで送られる色"""
    if name.endswith("_hero"):
        level *= ro.HERO_GAIN
    return ro.channel(ro.STYLE_LIST[ro.SID[name]][1], level)


def segdist(c, a, b):
    return float(ro.seg_dist(c[0], c[1], np.array([a[0]]), np.array([a[1]]), np.array([b[0]]), np.array([b[1]]))[0])


# ---------------------------------------------------------------------------
print("1. control file / 設定ファイル")
d = tempfile.mkdtemp(prefix="ro_test_")
P = os.path.join(d, "data", "road_overlay.json")
s = ro.load_control(P)
check("defaults when missing: off, 120 m, 800 shapes / 2 s, detail 45 m, brightness 3, spectator on, "
      "road/direction/junctions/rules on, ids/osm off",
      s["on"] is False and s["radius"] == 120.0 and s["max_shapes"] == 800 and s["refresh"] == 2.0
      and s["detail_radius"] == 45.0 and s["brightness"] == 3.0 and s["spectator"] is True
      and s["layers"]["road"] and s["layers"]["direction"]
      and s["layers"]["junctions"] and s["layers"]["rules"] and not s["layers"]["ids"]
      and not s["layers"]["osm"], s)
ro.save_control({"on": "yes", "radius": 5000, "max_shapes": -3, "refresh": "x", "junk": 1,
                 "layers": {"road": 0, "bogus": True}}, P)
s = ro.load_control(P)
check("clamped and cleaned", s["on"] is True and s["radius"] == 400.0 and s["max_shapes"] == 100
      and s["refresh"] == 2.0 and "junk" not in s and s["layers"]["road"] is False
      and "bogus" not in s["layers"], s)
check("atomic write leaves no .tmp", not os.path.exists(P + ".tmp"))
with open(P, "w") as fh:
    fh.write("{broken")
check("a broken file reads as the defaults", ro.load_control(P) == ro.clean_control({}))
ro.save_control({}, P)
check("toggle flips on", ro.toggle(P) is True and ro.load_control(P)["on"] is True)
check("toggle again flips off", ro.toggle(P) is False and ro.load_control(P)["on"] is False)
s = ro.update_control({"layers": {"ids": True}, "radius": 60}, P)
check("update merges one layer, keeps the others", s["layers"]["ids"] and s["layers"]["road"]
      and s["radius"] == 60.0)
check("no helper status -> not alive", ro.helper_alive(os.path.join(d, "none.json")) is False)
check("legend and layer info for the dashboard",
      len(ro.layer_info()) == len(ro.LAYERS) and all(len(x["rgb"]) == 3 for x in ro.legend()))

# ---------------------------------------------------------------------------
print("2. keybind / キー割り当て")
dflt = kb.defaults()
check("road_overlay action, key O, no wheel button",
      "road_overlay" in dflt and dflt["road_overlay"]["keys"] == ["o"] and dflt["road_overlay"]["wheel"] == [])
clash = kb.conflicts(dflt)
check("no default clashes with O", "o" not in clash["keys"], clash["keys"].get("o"))
others = {k for a, c in dflt.items() if a != "road_overlay" for k in c["keys"]}
check("O is no other action's default key", "o" not in others)
check("Ctrl+O (eye up) is a different binding", kb.parse_key("ctrl+o") != kb.parse_key("o"))

# ---------------------------------------------------------------------------
print("3. geometry: OitaStation / 形状")
t0 = time.time()
wmap = carla.Map("OitaStation", open(XODR["OitaStation"]).read())
world = FakeWorld()
ov = ro.Overlay(world, wmap)
geo = ov.geo
print("     precompute %.2f s (load + build %.2f s)" % (geo.seconds, time.time() - t0))
check("3700+ lanes, junction boxes, signs", geo.n_lanes >= 3700 and geo.n_junctions > 100
      and len(geo.signs) > 300, (geo.n_lanes, geo.n_junctions, len(geo.signs)))
T = geo.lines
lay = {k: int((T.layer == i).sum()) for i, k in enumerate(ro.LAYERS)}
check("lines in road / direction / rules / lane_change / ids", all(lay[k] > 0 for k in
      ("road", "direction", "rules", "lane_change", "ids")), lay)
check("speed labels from the lane <speed> (lane_limits)", geo.n_speed > 100, geo.n_speed)
# an outer edge lies width/2 right of the lane centre / 外側の端は中心から幅/2
wps = wmap.generate_waypoints(2.0)
wp = next(w for w in wps if not w.is_junction and w.lane_width > 2.5)
l, r = wp.transform.location, wp.transform.get_right_vector()
want = (l.x + r.x * wp.lane_width / 2, l.y + r.y * wp.lane_width / 2)
road = np.nonzero(T.layer == ro.LAYERS.index("road"))[0]
dd = ro.seg_dist(want[0], want[1], T.a[road, 0], T.a[road, 1], T.b[road, 0], T.b[road, 1])
check("a road edge passes width/2 right of the lane centre", dd.min() < 0.1, round(float(dd.min()), 3))
check("edges lifted ~0.1 m above the lane", abs(float(T.a[road[np.argmin(dd)], 2]) - (l.z + ro.Z_EDGE)) < 0.3)
# arrows point the way the lane drives (keep-left shows here) / 矢印は走行方向
C = np.array([[w.transform.location.x, w.transform.location.y] for w in wps])
F = np.array([[w.transform.get_forward_vector().x, w.transform.get_forward_vector().y] for w in wps])
ok = 0
rng = np.random.default_rng(1)
picks = rng.choice(len(geo.arrow_a), 200, replace=False)
for k in picks:
    m = (geo.arrow_a[k, :2] + geo.arrow_b[k, :2]) / 2
    v = geo.arrow_b[k, :2] - geo.arrow_a[k, :2]
    v = v / (np.linalg.norm(v) or 1)
    near = np.argsort(np.hypot(*(C - m).T))[:6]
    ok += max(float(F[j] @ v) for j in near) > 0.95
check("arrows follow the waypoints' driving direction", ok >= 195, "%d/200" % ok)
stop_bars = int((T.style == ro.SID["stop_bar"]).sum())
check("a stop bar for every 206 sign", stop_bars >= 0.9 * sum(1 for s_ in geo.signs if s_["type"] == "206"),
      stop_bars)
check("text: '40' is 8 strokes, unknown characters skipped",
      len(ro.text_segments("40", (0, 0, 0), (1, 0, 0), (0, 1, 0), 2.0)) == 8
      and len(ro.text_segments("4~", (0, 0, 0), (1, 0, 0), (0, 1, 0), 2.0)) == 3)
check("simplify keeps the ends and merges a straight run",
      ro.simplify([(i * 2.0, 0.0, 0.0) for i in range(10)]) == [0, 9])

# ---------------------------------------------------------------------------
print("4. radius, cap, layers, hero / 半径・上限・レイヤー・自車")
centre = tuple(float(v) for v in geo.centre_pts[len(geo.centre_pts) // 2])


def draw(ctl_over=None, centre_=centre, hero_wp=None):
    ctl = ro.clean_control(dict({"on": True}, **(ctl_over or {})))
    world.debug.calls = []
    st = ov.refresh(ctl, centre_, hero_wp, life=1.3)
    return st, list(world.debug.calls)


st, calls = draw()
check("defaults draw something, within the cap of 800", 0 < st["shapes"] <= 800 and len(calls) == st["shapes"],
      (st["shapes"], len(calls)))
vals = [v for c in calls for v in c[4]]
check("dim colours: no channel above 4 at brightness 3 (debug lines bloom)", max(vals) <= 4, max(vals))
check("thin lines (<= 0.12 m; stop bars the thickest)", max(c[3] for c in calls) <= 0.12 + 1e-6,
      max(c[3] for c in calls))
arr = [c for c in calls if c[0] == "arrow"]
check("arrows only within the detail radius (45 m)", arr and max(segdist(centre, c[1], c[2]) for c in arr) <= 46.0)
txt = [c for c in calls if c[4] == rgb("speed") and c[3] == ro.STYLE_LIST[ro.SID["speed"]][2]]
check("stroked speed text only within ~45 m", all(segdist(centre, c[1], c[2]) <= 45.0 + 4.0 for c in txt))
_, cb = draw({"brightness": 8})
check("brightness 8 -> brighter channels", max(v for c in cb for v in c[4]) > max(vals))
world.debug.calls = []
ov.refresh(ro.clean_control({"on": True}), centre, None, life=1.3, night=True)
check("night: dimmer (the camera exposes brighter)", max(v for c in world.debug.calls for v in c[4]) < max(vals))
check("legend = the Python hues x 255", all(x["rgb"] == [round(255 * h) for h in ro.STYLE_LIST[ro.SID[x["key"]]][1]]
                                           for x in ro.legend()))
check("never persistent lines, life 1.3 s", all(c[6] is False and c[5] == 1.3 for c in calls))
check("no draw_string / draw_point (not seen by cameras)", not any(c[0] in ("string", "point") for c in calls))
far = max(segdist(centre, c[1], c[2]) for c in calls if c[0] in ("line", "arrow"))
check("everything within the radius (+ label size)", far <= 120.0 + 4.0, round(far, 1))
st60, calls60 = draw({"radius": 60})
far60 = max(segdist(centre, c[1], c[2]) for c in calls60 if c[0] in ("line", "arrow"))
check("radius 60 -> nothing beyond ~64 m, fewer shapes", far60 <= 64.0 and len(calls60) < len(calls),
      (round(far60, 1), len(calls60)))
stc, callsc = draw({"max_shapes": 300})
farc = max(segdist(centre, c[1], c[2]) for c in callsc if c[0] in ("line", "arrow"))
check("cap 300: at most 300 shapes, the nearest ones", len(callsc) <= 300 and stc["capped"] and farc < far,
      (len(callsc), round(farc, 1)))
none = {k: False for k in ro.LAYERS}
st0, calls0 = draw({"layers": none})
check("every layer off -> nothing drawn", st0["shapes"] == 0 and not calls0)
_, cr = draw({"layers": dict(none, road=True, junctions=True)})
cols = {c[4] for c in cr}
check("road + junctions: red edges, orange junction paths, junction boxes",
      {rgb("edge"), rgb("edge_j"), rgb("jbox")} <= cols <= {rgb("edge"), rgb("edge_inner"), rgb("edge_j"),
                                                             rgb("jbox")}
      and all(c[0] in ("line", "box") for c in cr), cols)
_, crn = draw({"layers": dict(none, road=True)})
check("road only (junctions off): junction paths drawn red like roads, all lines",
      crn and {c[4] for c in crn} <= {rgb("edge"), rgb("edge_inner")} and all(c[0] == "line" for c in crn))
_, cd = draw({"layers": dict(none, direction=True)})
check("direction only: arrows + blue centre lines", any(c[0] == "arrow" for c in cd)
      and {c[4] for c in cd} == {rgb("arrow")})
_, cj = draw({"layers": dict(none, junctions=True)})
check("junctions only: junction boxes", cj and all(c[0] == "box" and c[4] == rgb("jbox") for c in cj), len(cj))
_, cl = draw({"layers": dict(none, road=True, lane_change=True), "radius": 400, "max_shapes": 10000})
cols = {c[4] for c in cl}
inner = sum(1 for c in cl if c[4] == rgb("edge_inner"))
check("lane change on: lane boundaries in cyan / yellow / white, no plain inner edges",
      cols & {rgb("lc_both"), rgb("lc_one"), rgb("lc_none")} and inner == 0, cols)
lcl = [c for c in cl if c[4] == rgb("lc_both")]
check("... broken paint drawn dashed (short pieces)", any(math.dist(c[1], c[2]) <= 4.5 for c in lcl))
_, cru = draw({"layers": dict(none, rules=True)})
check("rules: stop bars and the white speed text", {rgb("stop_bar"), rgb("speed")} <= {c[4] for c in cru})
_, ci = draw({"layers": dict(none, ids=True)})
check("ids only: id strokes", ci and {c[4] for c in ci} == {rgb("ids")})
hero = min(wps, key=lambda w: w.transform.location.distance(carla.Location(*centre)))
_, ch = draw({"layers": dict(none, road=True, direction=True)}, hero_wp=hero)
check("your lane highlighted (thicker, brighter)", any(c[4] == rgb("edge_hero") and c[3] > 0.06 for c in ch)
      and any(c[4] == rgb("arrow_hero") for c in ch))
# a stop sign faces you / 標識はこちらを向く
s206 = next(s_ for s_ in geo.signs if s_["type"] == "206")
sh = ov._sign_shapes(s206, s206["x"] - 20.0, s206["y"])
oct_ = [x for x in sh if x[3] == 0.04]
check("stop sign: octagon in a plane facing the viewer", len(oct_) >= 8 and
      all(abs(a[0] - s206["x"]) < 1e-6 and abs(b[0] - s206["x"]) < 1e-6 for a, b, _, _ in oct_))
check("selection is fast (< 20 ms)", st["select_ms"] < 20, st["select_ms"])

# ---------------------------------------------------------------------------
print("5. traffic lights / 信号")
stop_wp = hero
light = FakeLight(carla.Location(stop_wp.transform.location.x + 3, stop_wp.transform.location.y, 0.0), [stop_wp])
world.actors = Actors([light])
ov.lights = ov._lights()
_, cl1 = draw({"layers": dict(none, rules=True)})
LRED, LGREEN = ro.channel(ro.LIGHT_HUE["Red"], 3), ro.channel(ro.LIGHT_HUE["Green"], 3)
SW = stop_wp.transform.location


def bar_cols(calls_):
    """Colours of the thick bars across the stop waypoint. / 停止線の色"""
    return {c[4] for c in calls_ if c[0] == "line" and abs(c[3] - 0.12) < 1e-6
            and math.dist(((c[1][0] + c[2][0]) / 2, (c[1][1] + c[2][1]) / 2), (SW.x, SW.y)) < 0.3}


bars = [c for c in cl1 if c[0] == "line" and abs(c[3] - 0.12) < 1e-6
        and math.dist(((c[1][0] + c[2][0]) / 2, (c[1][1] + c[2][1]) / 2), (SW.x, SW.y)) < 0.3]
check("a red light: its stop bar is red", bar_cols(cl1) == {LRED}, bar_cols(cl1))
check("the bar spans the lane width", abs(math.dist(bars[0][1][:2], bars[0][2][:2]) - stop_wp.lane_width) < 0.05)
light.state = "Green"
_, cl2 = draw({"layers": dict(none, rules=True)})
check("turns green -> the bar is green on the next refresh", bar_cols(cl2) == {LGREEN}, bar_cols(cl2))
check("a box at the light head", any(c[0] == "box" and c[4] == LGREEN for c in cl2))
world.actors = Actors()
ov.lights = []

# ---------------------------------------------------------------------------
print("5b. the CarlaUE4 window's area (spectator) / サーバー画面の範囲")
far_c = None
for p_ in geo.centre_pts[::50]:
    if math.hypot(p_[0] - centre[0], p_[1] - centre[1]) > 400:
        far_c = (float(p_[0]), float(p_[1]))
        break
spec = {"x": far_c[0], "y": far_c[1], "r": 150.0, "thick": 4.0}
world.debug.calls = []
st2 = ov.refresh(ro.clean_control({"on": True, "layers": dict(ro.LAYER_DEFAULTS, ids=True)}), centre, hero,
                 life=1.3, spectator=spec)
c2 = world.debug.calls
near_s = [c for c in c2 if c[0] == "line" and segdist(far_c, c[1], c[2]) <= 150.0]
near_h = [c for c in c2 if c[0] == "line" and segdist(centre, c[1], c[2]) <= 120.0]
check("far apart: two areas, both drawn, one shared cap", st2["areas"] == 2 and near_s and near_h
      and st2["shapes"] <= 800, (st2["areas"], len(near_s), len(near_h), st2["shapes"]))
strs = [c for c in c2 if c[0] == "string"]
check("labels in the spectator area are draw_string (full colour), none near your car",
      strs and all(segdist(far_c, c[1], c[2]) <= 150.0 for c in strs) and max(max(c[4]) for c in strs) == 255,
      len(strs))
check("thicker lines in the spectator area (4x)", any(abs(c[3] - 4 * 0.05) < 1e-6 for c in near_s)
      and not any(c[3] > 0.13 for c in near_h))
check("no arrows in the spectator area", not any(c[0] == "arrow" and segdist(far_c, c[1], c[2]) < 150 for c in c2))
world.debug.calls = []
st3 = ov.refresh(ro.clean_control({"on": True}), centre, None, life=1.3,
                 spectator={"x": centre[0] + 20, "y": centre[1], "r": 150.0, "thick": 4.0})
check("spectator near your car: one area only", st3["areas"] == 1 and not st3["spectator_area"])
world.debug.calls = []
st4 = ov.refresh(ro.clean_control({"on": True}), None, None, life=1.3, spectator=spec)
check("no hero: drawn round the spectator only", st4["areas"] == 1 and st4["spectator_area"] and st4["shapes"] > 0
      and all(segdist(far_c, c[1], c[2]) <= 150.0 + 4 for c in world.debug.calls if c[0] == "line"))
world.debug.calls = []
ov.refresh(ro.clean_control({"on": True, "spectator": False}), centre, None, life=1.3)
check("spectator option off: no strings", not any(c[0] == "string" for c in world.debug.calls))
sw = min(wps, key=lambda w: math.hypot(w.transform.location.x - far_c[0], w.transform.location.y - far_c[1]))
gl = sw.transform.location
tf = carla.Transform(carla.Location(gl.x - 100.0, gl.y, gl.z + 100.0), carla.Rotation(pitch=-45.0, yaw=0.0))
sa = ro.spectator_area(wmap, tf, 120.0)
check("spectator 100 m up looking down 45 deg: area on the ground ahead, r 150, lines 4x",
      abs(sa["x"] - gl.x) < 15 and abs(sa["y"] - gl.y) < 15 and abs(sa["r"] - 150) < 15 and sa["thick"] >= 3.5, sa)
sa2 = ro.spectator_area(wmap, carla.Transform(carla.Location(gl.x, gl.y, gl.z + 2.0)), 120.0)
check("spectator at street level: base radius, normal lines", sa2["r"] == 120.0 and sa2["thick"] == 1.0, sa2)

# ---------------------------------------------------------------------------
print("6. raw OSM ways / 元の OSM")
path, meta = ro.osm_source("OitaKosen")
if path is None:
    print("     (OitaKosen .osm not on this PC: skipped)")
else:
    km = carla.Map("OitaKosen", open(XODR["OitaKosen"]).read())
    kov = ro.Overlay(FakeWorld(), km)
    ways = ro.osm_ways(path, meta)
    pts = np.array([q for w in ways for q in w])
    inb = (np.abs(pts[:, 0]) < meta["half_x"] - 20) & (np.abs(pts[:, 1]) < meta["half_y"] - 20)
    pts = pts[inb][::5]
    Ck = kov.geo.centre_pts[:, :2]
    dist = np.array([np.hypot(*(Ck - p).T).min() for p in pts])
    check("OSM ways lie on the lanes (CARLA y = -north)", np.median(dist) < 3.0, round(float(np.median(dist)), 2))
    flip = np.array([np.hypot(*(Ck - p * [1, -1]).T).min() for p in pts])
    check("... and not with y unflipped", np.median(flip) > 3 * np.median(dist), round(float(np.median(flip)), 1))
    kov.debug = FakeDebug()
    c0 = tuple(float(v) for v in kov.geo.centre_pts[len(kov.geo.centre_pts) // 3])
    ctl = ro.clean_control({"on": True, "layers": dict(none, osm=True)})
    kov.refresh(ctl, c0)
    kov._osm_thread.join(30)
    kov.debug.calls = []
    st = kov.refresh(ctl, c0)
    zs = [c[1][2] for c in kov.debug.calls]
    check("OSM layer draws green lines once loaded", st["osm"] == "ok" and kov.debug.calls
          and {c[4] for c in kov.debug.calls} == {rgb("osm")}, (st["osm"], len(kov.debug.calls)))
    check("OSM lines follow the hills (heights from the lanes)", np.ptp(zs) > 1.0 if zs else False,
          round(float(np.ptp(zs)), 1) if zs else "")
tm = carla.Map("Town03", open(XODR["Town03"]).read())
tov = ro.Overlay(FakeWorld(), tm)
check("Town03: no build dir -> OSM skipped cleanly", ro.osm_source("Town03")[0] is None)
tov.refresh(ro.clean_control({"on": True, "layers": dict(none, osm=True)}), (0, 0, 0))
check("... the layer reports why", tov.osm_state.startswith("none"), tov.osm_state)
check("Town03: other lane types (sidewalk, shoulder, parking)", tov.geo.n_other > 100
      and int((tov.geo.lines.layer == ro.LAYERS.index("lanes")).sum()) > 0)
tov.debug = FakeDebug()
tov.refresh(ro.clean_control({"on": True, "radius": 400, "max_shapes": 10000,
                              "layers": dict(none, lanes=True)}), (0, 0, 0))
cols = {c[4] for c in tov.debug.calls}
check("... each in its own colour", {rgb("Sidewalk"), rgb("Shoulder")} <= cols, cols)
tl = tov.geo.lines
lcs = {ro.STYLE_LIST[s_][0] for s_, l_ in zip(tl.style.tolist(), tl.layer.tolist()) if ro.LAYERS[l_] == "lane_change"}
check("Town03 (laneChange in its xodr): lane change allowed and forbidden boundaries",
      {"lc_both", "lc_none"} <= lcs, lcs)
lk = (tl.layer == ro.LAYERS.index("lane_change")) & (tl.style == ro.SID["lc_none"])
check("... solid paint drawn continuous (long chords)",
      (np.hypot(*(tl.b[lk, :2] - tl.a[lk, :2]).T) > 6).any())
check("Town03: speed labels from the road <speed> (no lane speeds)", tov.geo.n_speed > 0, tov.geo.n_speed)

# ---------------------------------------------------------------------------
print("7. the helper loop / ヘルパーのループ")


class LoopWorld(FakeWorld):
    def __init__(self, wid, m):
        FakeWorld.__init__(self)
        self.id, self.m = wid, m

    def get_map(self):
        return self.m

    def get_snapshot(self):
        class S(object):
            pass
        sn = S()
        sn.timestamp = S()
        sn.timestamp.elapsed_seconds = time.time()
        return sn

    def get_weather(self):
        return carla.WeatherParameters(sun_altitude_angle=45.0)

    def get_spectator(self):
        class Sp(object):
            def get_transform(self_):
                return carla.Transform(carla.Location(0, 0, 0))
        return Sp()


class FakeClient(object):
    world = None

    def __init__(self, *a):
        pass

    def set_timeout(self, t):
        pass

    def get_world(self):
        return FakeClient.world


ctlp = os.path.join(d, "loop.json")
stp = os.path.join(d, "loop_status.json")
ro.save_control({"on": False, "refresh": 0.5}, ctlp)
w1 = LoopWorld(1, tm)
FakeClient.world = w1
real_client = ro.carla.Client
ro.carla.Client = FakeClient
args = type("A", (), {"host": "x", "port": 1, "role": "hero", "control": ctlp, "status": stp})()
result = {}


def script():
    time.sleep(2.0)
    result["idle_calls"] = len(w1.debug.calls)
    result["idle_status"] = ro.read_status(stp)
    ro.update_control({"on": True}, ctlp)
    time.sleep(1.5)
    result["on_calls"] = len(w1.debug.calls)
    result["on_status"] = ro.read_status(stp)
    ro.update_control({"on": False}, ctlp)
    time.sleep(0.6)
    n = len(w1.debug.calls)
    time.sleep(1.5)
    result["off_growth"] = len(w1.debug.calls) - n
    w2 = LoopWorld(2, tm)                       # a new map / マップ変更
    FakeClient.world = w2
    ro.update_control({"on": True}, ctlp)
    time.sleep(5.0)
    result["new_world_calls"] = len(w2.debug.calls)
    os.kill(os.getpid(), signal.SIGTERM)


th = threading.Thread(target=script, daemon=True)
th.start()
t0 = time.time()
ro.run(args)
ro.carla.Client = real_client
signal.signal(signal.SIGTERM, signal.SIG_DFL)
check("idle while off: nothing drawn, status says off", result.get("idle_calls") == 0
      and result["idle_status"].get("on") is False and result["idle_status"].get("ready"), result.get("idle_calls"))
check("on: draws, status reports shapes", result.get("on_calls", 0) > 0
      and result["on_status"].get("shapes", 0) > 0, (result.get("on_calls"), result.get("on_status", {}).get("shapes")))
check("off again: no more drawing", result.get("off_growth") == 0, result.get("off_growth"))
check("a new world: rebuilt and drawing there", result.get("new_world_calls", 0) > 0, result.get("new_world_calls"))
check("SIGTERM: run() returns, status says stopped", ro.read_status(stp).get("stopped") is True
      and time.time() - t0 < 20)

# ---------------------------------------------------------------------------
print("8. precompute timing / 事前計算の時間")
for n in ("OitaStation", "OitaKosen"):
    t0 = time.time()
    m = carla.Map(n, open(XODR[n]).read())
    g = ro.RoadGeometry(m, log=lambda *a: None)
    tot = time.time() - t0
    print("     %s: %d lanes, %d lines, %d arrows; precompute %.2f s, with map load %.2f s"
          % (n, g.n_lanes, len(g.lines), len(g.arrow_a), g.seconds, tot))
    check("%s precompute under 15 s" % n, g.seconds < 15.0, round(g.seconds, 2))

print("\n%s" % ("ALL PASS / 全て合格" if not FAILS else "FAILED: %s" % ", ".join(FAILS)))
sys.exit(1 if FAILS else 0)
