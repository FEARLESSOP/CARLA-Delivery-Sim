"""Marking audit for Japanese-style maps (maps/.venv, offline; guide ch. 32).

  python check_markings.py <osm> <build dir> [--json out.json] [--no-mesh]

Reads lanes.json, crossings.pkl, <Name>.xodr, markings.pkl (build.py: every
painted element) and the Road_Marking meshes, and reports, with CARLA x/y
(y south), every marking that breaks the Japanese rules of guide ch. 4:

  crossings    band skewed > 5 deg to the lanes it crosses (and to the kerb at
               its ends), band not kerb to kerb, band over junction lanes, bands
               of two crossings overlapping, band far longer than the road
  stop lines   not square to the lane (> 5 deg), thickness outside 0.3-0.45 m,
               not 1-3 m before the crossing (or the junction edge), a second
               line on one lane, a line on an approach with neither a 206 nor a
               light, a 206 on a signalised road, 止まれ text on a signalised
               approach or not behind its line / off its lane, a 206 or a light
               with no line, a line that is not in the painted mesh
  arrows       on single-lane approaches
  lines        edge line missing on a MAIN/TERT/MINOR section, centre line
               missing on a two-way TERT+ section >= 5.5 m
  paint        Road_Marking triangles over the kerb, an island or off the road
  notes        (not counted) a light whose stop waypoint lies past the painted
               line (cars would stop on the crossing): road logic, not paint

Singapore maps (maps.json "sg_lta", markings_sg.py; markings.pkl "style":
"sg_lta") get the LTA rule set instead (check_sg, guide ch. 51): crossings as
above; J stop lines 0.30 m, 1-3 m before the signalised crossing (or 0.3-1.5 m
before the junction mouth), one per lane, none without a light, the light's s
(CARLA's stop, set by signs.py) not past the line and its pole (pole_s) not
before it; give-way lines off signalised roads; every zebra approach lane
with room has its zig-zag, and no zig-zag over yellow paint (RMS9 note 5); no
white over yellow paint; arrows only on 2+ lane approaches and never on
expressways; kerb and centre lines present by class; paint off the road.
--area x0,y0,x1,y1 (map frame; default BUILD_AREA) audits only what lies inside
(crop builds: the crop's edge is cut). Deck paint (Road_MarkingDeck) counts.

Without markings.pkl (a build before 2026-11-06) the stop lines, 止まれ and
arrows are reconstructed the way the old generator placed them (from the
crossings.pkl stop list and the lights), so old builds can be audited too.
Exit status 1 when anything is wrong.

日本式路面標示の検査（横断歩道の直角・縁石間、停止線の位置と向き、止まれ・矢印・外側線、縁石上の塗装）。
"""

import argparse
import collections
import json
import math
import os
import pickle
import sys
import xml.etree.ElementTree as ET

import numpy as np
import shapely
from shapely.geometry import LineString, Point, Polygon
from shapely.ops import substring, unary_union
from shapely.strtree import STRtree

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import layout  # noqa: E402
import markings_jp as MJ  # noqa: E402

SKEW_DEG = 5.0
KERB_TOL = 0.35          # band end this close to the carriageway edge = kerb to kerb / 縁石までの許容
STOP_GAP = (1.0, 3.0)    # stop line front edge before the crossing / junction edge / 停止線と横断歩道・交差点端の間隔
STOP_THICK = (0.28, 0.47)
STOP_206_MAX = 4.0       # a 止まれ 206 at most this far before its line / 206 と停止線の距離


def carla(p):
    return "(%.1f, %.1f)" % (p[0], -p[1])


def heading(v):
    return math.degrees(math.atan2(v[1], v[0]))


def ang180(a, b):
    """Angle between two directions ignoring sense, degrees 0..90. / 向きを無視した角度差。"""
    d = abs((heading(a) - heading(b) + 90.0) % 180.0 - 90.0)
    return d


class Paint(object):
    """The painted triangles of the Road_Marking meshes, per colour. / 塗装メッシュ。"""

    def __init__(self, out, kinds=("Road_Marking",)):
        self.tri = {"w": [], "y": []}
        man = json.load(open(os.path.join(out, "meshes", "manifest.json")))
        for o in man["objects"]:
            if o["kind"] not in kinds:
                continue
            d = np.load(os.path.join(out, "meshes", o["name"] + ".npz"))
            for k in d.files:
                if not k.endswith("|V"):
                    continue
                col = "y" if "yellow" in k else "w"
                V, I = d[k], d[k[:-2] + "|I"]
                T = V[I][:, :, :2]
                self.tri[col].append(shapely.polygons(np.concatenate([T, T[:, :1]], axis=1)))
        self.tri = {c: (np.concatenate(v) if v else np.zeros(0, object)) for c, v in self.tri.items()}
        self.tree = {c: STRtree(v) for c, v in self.tri.items() if len(v)}
        self.n = {c: len(v) for c, v in self.tri.items()}

    def hits(self, xy, col="w"):
        """For each point: is it on paint of that colour? / 各点が塗装上か。"""
        pts = shapely.points(np.asarray(xy, float))
        on = np.zeros(len(pts), bool)
        if col in self.tree:
            i, j = self.tree[col].query(pts, predicate="intersects")
            on[np.unique(i)] = True
        return on

    def centroids(self, col):
        return shapely.centroid(self.tri[col]) if len(self.tri[col]) else []


def rect_axes(poly):
    """(centre, long-axis unit vector, length, thickness) of a line-like polygon.
    線状の多角形の中心・長軸・長さ・太さ。"""
    mr = poly.minimum_rotated_rectangle
    c = np.asarray(mr.exterior.coords)[:4]
    e0, e1 = c[1] - c[0], c[2] - c[1]
    l0, l1 = np.linalg.norm(e0), np.linalg.norm(e1)
    u, length, thick = (e0 / l0, l0, l1) if l0 >= l1 else (e1 / l1, l1, l0)
    return np.asarray(mr.centroid.coords[0]), u, length, thick


def legacy_elements(extra, data, lanes_by, sig_roads):
    """Stop lines, 止まれ and arrows as the generator placed them before
    2026-11-06 (straight extrapolation of the lane-end heading; signal setback
    from any crossing within 15 m to the side). / 旧生成規則の再構成。"""
    appr = collections.defaultdict(list)
    for a in data["approaches"]:
        appr[(a["road"], a["junction"])].append(a)
    stops = {(r, j): b for r, j, b in extra.get("stop", [])}
    out = []
    for (road, j), al in appr.items():
        byl = collections.OrderedDict()
        for a in al:
            byl.setdefault(a["lane"], set()).update(a["turns"])
        lanes = [(lanes_by.get((road, ln)), t) for ln, t in byl.items()]
        lanes = [(l, t) for l, t in lanes if l is not None and len(l["pts"]) >= 3]
        if not lanes:
            continue
        signal, stop = road in sig_roads, (road, j) in stops
        if not (signal or stop):
            continue
        end = np.mean([l["pts"][-1]["c"] for l, t in lanes], axis=0)
        P = np.array([q["c"] for q in lanes[0][0]["pts"]])
        f = P[-1] - P[-3]
        f = f / (np.linalg.norm(f) or 1.0)
        if stop:
            back = stops[(road, j)] - MJ.STOP_W
        else:
            back = 0.5
            for cr in extra["crossings"]:
                v = np.array(cr["c"]) - end
                along, side = -float(v @ f), abs(float(v @ np.array([-f[1], f[0]])))
                if -3.0 < along < 15.0 and side < 15.0:
                    back = max(back, along + cr["width"] / 2 + 2.0)
        where = {"road": road, "junction": j, "signal": signal, "back": back, "legacy": True}
        for l, turns in lanes:
            P = np.array([q["c"] for q in l["pts"]])
            f = P[-1] - P[-3]
            f = f / (np.linalg.norm(f) or 1.0)
            nrm = np.array([-f[1], f[0]])
            w = l["pts"][-1]["w"]
            ln = LineString(P)
            c = P[-1] - f * (back + MJ.STOP_W / 2)
            g = LineString([c - nrm * (w / 2 + 0.05), c + nrm * (w / 2 + 0.05)]).buffer(MJ.STOP_W / 2, cap_style="flat")
            out.append(dict(kind="stop_signal" if signal else "stop", geom=g, lane=l["lane"], **where))
            if stop and ln.length > back + 10.0:
                a0, a1 = back + MJ.STOP_W + 1.5, back + MJ.STOP_W + 1.5 + 10.0
                g = Polygon([P[-1] - f * a0 - nrm * 0.75, P[-1] - f * a0 + nrm * 0.75,
                             P[-1] - f * a1 + nrm * 0.75, P[-1] - f * a1 - nrm * 0.75])
                out.append(dict(kind="tomare", geom=g, lane=l["lane"], **where))
            if len(lanes) >= 2 and turns:
                for dist in (7.0, 27.0):
                    if dist + back + 8.0 < ln.length:
                        p = ln.interpolate(ln.length - back - dist)
                        out.append(dict(kind="arrow", geom=Point(p.x, p.y).buffer(0.5), lane=l["lane"],
                                        turns=sorted(turns), **where))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("osm")
    ap.add_argument("out")
    ap.add_argument("--json")
    ap.add_argument("--no-mesh", action="store_true")
    ap.add_argument("--area", default=os.environ.get("BUILD_AREA"),
                    help="x0,y0,x1,y1 (map frame) or 'tiles' (the built tiles): Singapore rule set only, "
                         "audit only inside (crop builds) / 範囲内のみ検査（シンガポール規則）")
    a = ap.parse_args()
    out = os.path.abspath(a.out)
    meta = json.load(open(os.path.join(out, "meta.json")))
    name = meta["name"]
    xodr = os.path.join(out, name + ".xodr")
    data = json.load(open(os.path.join(out, "lanes.json")))
    extra = pickle.load(open(os.path.join(out, "crossings.pkl"), "rb"))
    crossings = extra["crossings"]
    L = layout.Layout(meta, os.path.join(out, "lanes.json"), a.osm)
    cw, jl = L.carriageway, L.junction_lanes
    off_road = unary_union([L.kerb, L.islands])
    lanes = [l for l in L.lanes if l["junction"] < 0 and len(l["pts"]) >= 2]
    lanes_by = {(l["road"], l["lane"]): l for l in lanes}
    lane_lines = [(LineString([q["c"] for q in l["pts"]]), l) for l in lanes]
    tree = STRtree([g for g, _ in lane_lines])
    jlanes = [(LineString([q["c"] for q in l["pts"]]), l) for l in L.lanes if l["junction"] >= 0 and len(l["pts"]) >= 2]
    jtree = STRtree([g for g, _ in jlanes]) if jlanes else None
    x = MJ.xodr_roads(xodr)
    root = ET.parse(xodr).getroot()
    s206 = collections.defaultdict(list)
    s_light = collections.defaultdict(list)
    for r in root.findall("road"):
        rid = int(r.get("id"))
        for s in r.findall("signals/signal"):
            if s.get("type") == "206":
                s206[rid].append(float(s.get("s")))
            elif s.get("type") == "1000011":
                s_light[rid].append(float(s.get("s")))
    # the lights as the xodr has them now (signs.py drops those on stub roads after
    # lanes.json was written) / 現在の xodr の信号機
    sig_roads = set(s_light)
    stale = {lt["road"] for lt in data["lights"]} - sig_roads
    sections = {}
    for l in lanes:
        sections.setdefault((l["road"], l["section"]), []).append(l)
    from core_merge import core_rect
    cls = MJ.road_classes(sections, x["way"], L.osm.ways, core_rect(L.meta))
    paint = None if a.no_mesh else Paint(out)
    mp = os.path.join(out, "markings.pkl")
    if os.path.exists(mp):
        elements = pickle.load(open(mp, "rb"))["elements"]
        source = "markings.pkl (%d elements)" % len(elements)
    else:
        elements = legacy_elements(extra, data, lanes_by, sig_roads)
        source = "legacy reconstruction (%d elements; no markings.pkl)" % len(elements)
    if os.path.exists(mp) and pickle.load(open(mp, "rb")).get("style") == "sg_lta":
        # Singapore (LTA) rule set / シンガポール（LTA）の規則で検査
        sys.exit(check_sg(a, out, meta, data, extra, L, elements, lanes, lanes_by, lane_lines, tree, x, s_light))
    problems, notes = [], []

    def bad(cat, p, msg, **kw):
        rec = {"cat": cat, "carla": [round(float(p[0]), 1), round(-float(p[1]), 1)],
               "map": [round(float(p[0]), 1), round(float(p[1]), 1)], "msg": msg}
        rec.update(kw)
        problems.append(rec)

    # -- crossings / 横断歩道
    # Singapore-style crossings (markings.py) carry no "band": from their pieces and width
    # band の無い横断歩道（markings.py）は区間と幅から作る
    from shapely.ops import unary_union as _uu
    for cr in crossings:
        if "band" not in cr:
            cr["band"] = _uu([g.buffer(cr.get("width", 4.0) / 2, cap_style="flat") for g in cr.get("segs", [])])
    bands = []
    for k, cr in enumerate(crossings):
        c, f, n = np.array(cr["c"]), np.array(cr["f"]), np.array(cr["n"])
        band = cr["band"]
        bands.append(band)
        tag = {"crossing": k, "src": cr.get("src", "")}
        # heading of the lanes the band crosses (both directions, mod 180) / 帯が跨ぐ車線の方位
        hs = []
        for i in tree.query(band):
            ln, l = lane_lines[int(i)]
            hit = ln.intersection(band)
            if hit.is_empty:
                continue
            q = hit.centroid if hit.geom_type != "Point" else hit
            hs.append(MJ.tangent(ln, ln.project(q)))
        if hs:
            fm = MJ.heading_mean(hs, f)
            skew = ang180(fm, f)
            if skew > SKEW_DEG:
                bad("crossing skew", c, "band %.1f deg off the lanes it crosses (%d lanes)" % (skew, len(hs)),
                    skew=round(skew, 1), **tag)
            # the lanes it should cross all have about the same heading / 跨ぐ車線が揃っているか
            spread = max(ang180(h, fm) for h in hs)
            if spread > 30.0:
                bad("crossing lanes", c, "band crosses lanes %.0f deg apart (a side street or the junction)" % spread,
                    **tag)
        else:
            bad("crossing lanes", c, "band crosses no road lane (only junction lanes?)", **tag)
        # kerb direction at the band ends / 両端の縁石の向き
        for t in (cr["t0"], cr["t1"]):
            e = c + n * t
            d = cw.boundary.distance(Point(e))
            if d > KERB_TOL:
                where = "inside the carriageway" if cw.contains(Point(e)) else "over the kerb"
                bad("crossing span", e, "band end %.2f m from the kerb line (%s)" % (d, where), **tag)
            else:
                # direction of the kerb segment under the end / 端の下の縁石線分の向き
                bl = cw.boundary
                q = bl.interpolate(bl.project(Point(e)))
                seg = None
                for part in getattr(bl, "geoms", [bl]):
                    if part.distance(q) < 1e-6:
                        s = part.project(q)
                        seg = MJ.tangent(part, s, 0.6)
                        break
                if seg is not None:
                    ks = ang180(seg, f)
                    if ks > 25.0:
                        # a kerb at a corner radius is not the road's direction: note only
                        # 隅切り部の縁石は道路の向きと違う: 参考情報
                        notes.append({"cat": "crossing kerb", "carla": [round(float(e[0]), 1), round(-float(e[1]), 1)],
                                      "msg": "kerb under the band end runs %.0f deg to the band's road axis "
                                             "(corner radius?)" % ks, **tag})
        inj = band.intersection(jl).area / band.area if band.area else 0.0
        if inj > 0.1:
            bad("crossing in junction", c, "%.0f%% of the band lies over junction lanes" % (100 * inj), **tag)
        # width of the road under the band vs the band's span / 帯の長さと道路幅
        ws = sum(l["pts"][len(l["pts"]) // 2]["w"] for i in tree.query(band)
                 for ln, l in [lane_lines[int(i)]] if ln.intersects(band))
        span = sum(sg.length for sg in cr["segs"])          # paved length (medians left out) / 舗装部の長さ
        if ws and span > ws + 2 * layout.GUTTER + 4.0:
            bad("crossing span", c, "band %.1f m of paving over %.1f m of lanes (across a side street or the flare?)"
                % (span, ws), **tag)
    for i in range(len(bands)):
        for k in range(i + 1, len(bands)):
            ov = bands[i].intersection(bands[k]).area
            if ov > 0.2:
                bad("crossing overlap", bands[i].intersection(bands[k]).centroid.coords[0],
                    "crossings %d and %d overlap by %.1f m2" % (i, k, ov), crossing=i, other=k)

    # -- stop lines, 止まれ, arrows / 停止線・止まれ・矢印
    per_lane = collections.Counter()
    lines_on = collections.defaultdict(list)       # road -> [(s_front, kind)] / 道路ごとの停止線
    for e in elements:
        if e["kind"] not in ("stop", "stop_signal"):
            continue
        l = lanes_by.get((e["road"], e.get("lane")))
        g = e["geom"]
        if l is None or g is None or g.is_empty:
            continue
        ln = LineString([q["c"] for q in l["pts"]])
        ctr, u, length, thick = rect_axes(g)
        s_c = ln.project(Point(ctr))
        tg = MJ.tangent(ln, s_c)
        skew = abs(90.0 - ang180(u, tg))
        tag = {"road": e["road"], "junction": e.get("junction"), "lane": e.get("lane"), "kind": e["kind"]}
        per_lane[(e["road"], e.get("lane"), e.get("junction"))] += 1
        if skew > SKEW_DEG:
            bad("stop skew", ctr, "stop line %.1f deg off square to its lane" % skew, skew=round(skew, 1), **tag)
        if not (STOP_THICK[0] <= thick <= STOP_THICK[1]):
            bad("stop thickness", ctr, "stop line %.2f m thick" % thick, **tag)
        s_front = s_c + thick / 2.0
        lines_on[e["road"]].append((s_front, e["kind"], ctr))
        gap_end = ln.length - s_front
        # the crossing ahead on this lane / この車線の先の横断歩道
        P = np.asarray(ln.coords)
        fe = P[-1] - P[-2]
        fe = fe / (np.linalg.norm(fe) or 1.0)
        ext = LineString(list(map(tuple, P)) + [tuple(P[-1] + fe * 3.0)])
        gap_cross = None
        for k, cr in enumerate(crossings):
            hit = ext.intersection(cr["band"])
            if hit.is_empty:
                continue
            s_min = min(ext.project(Point(q)) for gg in getattr(hit, "geoms", [hit]) for q in gg.coords)
            if ln.length - s_min <= MJ.STOP_NEAR_CROSS:
                gc = s_min - s_front
                gap_cross = gc if gap_cross is None else min(gap_cross, gc)
        if gap_cross is not None:
            if not (STOP_GAP[0] <= gap_cross <= STOP_GAP[1]):
                bad("stop position", ctr, "stop line %.1f m before the crossing (want %.0f-%.0f)"
                    % (gap_cross, STOP_GAP[0], STOP_GAP[1]), **tag)
        else:
            # a signal line also stays before its pole (markings_jp STOP_POLE_CLEAR)
            # 信号の停止線は信号柱の手前でもよい
            hi = STOP_GAP[1]
            pb = MJ.pole_back(x["lights"].get(e["road"], []), [l]) if e["kind"] == "stop_signal" else None
            if pb is not None:
                hi = max(hi, pb + MJ.STOP_POLE_CLEAR + 0.3)
            if not (0.3 <= gap_end <= hi):
                bad("stop position", ctr, "stop line %.1f m before the junction edge, no crossing (want 0.3-%.1f)"
                    % (gap_end, hi), **tag)
        if e["kind"] == "stop_signal":
            pb = MJ.pole_back(x["lights"].get(e["road"], []), [l])
            if pb is not None and gap_end < pb + 0.2:
                bad("stop after pole", ctr, "signal stop line %.1f m past its light pole" % (pb - gap_end), **tag)
        # xodr s of the line's front edge (lane centre length != reference line length
        # on curves): from the lane points' own s / 停止線前端の xodr s（車線点の s から）
        cum = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(P, axis=0).T))])
        sx = np.array([q["s"] for q in l["pts"]])
        xs_front = float(np.interp(s_front, cum, sx))
        fwd = sx[-1] >= sx[0]
        before = [(xs_front - s) if fwd else (s - xs_front) for s in s206.get(e["road"], [])]
        has206 = any(-0.5 <= b <= STOP_206_MAX for b in before)
        if e["kind"] == "stop":
            if e["road"] in sig_roads:
                bad("stop at signal", ctr, "止まれ-style stop line on a signalised road", **tag)
            elif not has206:
                near = [round(float(b), 1) for b in before]
                bad("stop without 206", ctr, "止まれ stop line with no 206 within %.0f m before it (206 offsets %s)"
                    % (STOP_206_MAX, near), **tag)
        else:
            if e["road"] not in sig_roads:
                bad("signal line without light", ctr, "signal stop line on a road with no traffic light", **tag)
            if s206.get(e["road"]):
                bad("206 at signal", ctr, "a 206 stop sign on a signalised road", **tag)
            for s in s_light.get(e["road"], []):
                if ((s - xs_front) if fwd else (xs_front - s)) > 0.5:
                    notes.append({"cat": "stop waypoint", "carla": [round(float(ctr[0]), 1), round(-float(ctr[1]), 1)],
                                  "msg": "light stop waypoint %.1f m past the painted line (cars stop there)"
                                         % abs(s - xs_front), **tag})
        if paint is not None and not paint.hits([ctr])[0]:
            bad("stop unpainted", ctr, "stop line centre is not on white paint in the Road_Marking mesh", **tag)
    for key, nline in per_lane.items():
        if nline > 1:
            l = lanes_by.get(key[:2])
            bad("stop double", l["pts"][-1]["c"], "%d stop lines on one approach lane" % nline, road=key[0],
                lane=key[1], junction=key[2])
    for e in elements:
        if e["kind"] == "tomare" and e.get("geom") is not None and not e["geom"].is_empty:
            l = lanes_by.get((e["road"], e.get("lane")))
            if l is None:
                continue
            ln = LineString([q["c"] for q in l["pts"]])
            ctr = np.asarray(e["geom"].centroid.coords[0])
            if e["road"] in sig_roads:
                bad("tomare at signal", ctr, "止まれ text on a signalised approach", road=e["road"])
            lines = [s for s, kind, c in lines_on.get(e["road"], [])]
            s_text = max(ln.project(Point(q)) for q in np.asarray(e["geom"].envelope.exterior.coords))
            if lines and s_text > min(lines) + 0.05:
                bad("tomare position", ctr, "止まれ text reaches %.1f m past its stop line" % (s_text - min(lines)),
                    road=e["road"])
            lp = Polygon([q["l"] for q in l["pts"]] + [q["r"] for q in l["pts"]][::-1]).buffer(0.15)
            if not lp.contains(e["geom"]) and e["geom"].difference(lp).area > 0.05:
                bad("tomare off lane", ctr, "止まれ text %.2f m2 outside its lane" % e["geom"].difference(lp).area,
                    road=e["road"])
        if e["kind"] == "arrow":
            l = lanes_by.get((e["road"], e.get("lane")))
            if l is not None and len(sections.get((l["road"], l["section"]), [])) < 2:
                bad("arrow single lane", e["geom"].centroid.coords[0], "lane arrow on a single-lane approach",
                    road=e["road"])
    # 206 / light without a line / 線の無い 206・信号
    for rid, ss in s206.items():
        if rid in sig_roads:
            l = next((l for l in lanes if l["road"] == rid), None)
            if l is not None:
                bad("206 at signal", l["pts"][-1]["c"], "road %d carries a 206 and a traffic light" % rid, road=rid)
        if not lines_on.get(rid):
            l = next((l for l in lanes if l["road"] == rid), None)
            if l is not None:
                bad("206 without line", l["pts"][-1]["c"], "206 on road %d but no stop line painted" % rid, road=rid)
    for rid in sig_roads:
        if not any(kind == "stop_signal" for s, kind, c in lines_on.get(rid, [])):
            l = next((l for l in lanes if l["road"] == rid), None)
            if l is not None:
                bad("signal without line", l["pts"][-1]["c"], "traffic light on road %d but no stop line" % rid,
                    road=rid)

    # -- edge and centre lines where the class requires them / 格に応じた外側線・中央線
    lh = meta.get("lefthand", False)
    if paint is not None:
        cedges = []
        for key, ls in sections.items():
            ls = sorted(ls, key=lambda l: abs(l["lane"]))
            l = ls[-1] if lh else ls[0]
            cedges.append((LineString([p["r" if lh else "l"] for p in l["pts"]]), l))
        ctree = STRtree([g for g, _ in cedges])
        for key, ls in sections.items():
            c = cls[key]
            ls = sorted(ls, key=lambda l: abs(l["lane"]))
            kerb = ls[0] if lh else ls[-1]
            kedge = LineString([p["l" if lh else "r"] for p in kerb["pts"]])
            if c >= MJ.MINOR and kedge.length > 8.0:
                o = MJ.offset(kedge, (1.0 if lh else -1.0) * 0.1)
                if o is not None and o.length > 2.0:
                    pts = [o.interpolate(t, normalized=True) for t in np.linspace(0.1, 0.9, 7)]
                    hits = paint.hits([(p.x, p.y) for p in pts])
                    if hits.sum() < 3:
                        bad("edge missing", pts[3].coords[0], "edge line missing on a class-%d section (%d/7 samples)"
                            % (c, hits.sum()), road=key[0], section=key[1])
            if c >= MJ.TERT and len(ls) == 1:
                ce = LineString([p["r" if lh else "l"] for p in ls[-1 if lh else 0]["pts"]])
                if ce.length < 12.0:
                    continue
                pts = [ce.interpolate(t, normalized=True) for t in np.linspace(0.05, 0.95, 7)]
                opp = None
                for i in ctree.query(ce.buffer(1.5)):
                    e2, l2 = cedges[int(i)]
                    if l2["road"] == key[0]:
                        continue
                    if sum(1 for p in pts if e2.distance(p) <= 1.5) >= 4:
                        opp = l2
                        break
                if opp is None:
                    continue
                width = float(np.median([q["w"] for q in ls[0]["pts"]])) + float(np.median([q["w"] for q in opp["pts"]]))
                if width < MJ.CENTRE_MIN or cls.get((opp["road"], opp["section"]), c) < MJ.TERT:
                    continue
                probes = [ce.interpolate(t, normalized=True) for t in np.linspace(0.1, 0.9, 12)]
                xy = [(p.x, p.y) for p in probes]
                hits = paint.hits(xy, "w") | paint.hits(xy, "y")
                if hits.sum() < 3:
                    bad("centre missing", probes[6].coords[0], "centre line missing on a two-way class-%d section "
                        "%.1f m wide (%d/12 samples)" % (c, width, hits.sum()), road=key[0], section=key[1])

    # -- paint over kerbs, islands, off the road / 縁石・島・車道外の塗装
    if paint is not None:
        over = off_road.buffer(-0.02)
        for col in ("w", "y"):
            cen = paint.centroids(col)
            if not len(cen):
                continue
            xy = shapely.get_coordinates(cen)
            on_off = shapely.contains_xy(over, xy[:, 0], xy[:, 1])
            lots = unary_union([L.parking, L.driveways]).buffer(0.05)      # car-park bay lines / 駐車枡の線
            outside = ~shapely.contains_xy(cw.buffer(0.03), xy[:, 0], xy[:, 1]) & \
                ~shapely.contains_xy(lots, xy[:, 0], xy[:, 1])
            for mask, what in ((on_off, "over the kerb or an island"), (outside & ~on_off, "off the carriageway")):
                if mask.any():
                    p = xy[mask][0]
                    bad("paint off road", p, "%d %s paint triangles %s (e.g. here)" % (mask.sum(), col, what),
                        count=int(mask.sum()))

    # -- report / 報告
    cats = collections.Counter(p["cat"] for p in problems)
    print("check_markings: %s; elements from %s; %d crossings, %d stop-line elements, %d 206, %d lights%s"
          % (name, source, len(crossings), sum(1 for e in elements if e["kind"] in ("stop", "stop_signal")),
             sum(len(v) for v in s206.values()), len(sig_roads),
             ("; paint %d white / %d yellow triangles" % (paint.n["w"], paint.n["y"])) if paint else "")
          + ("; lanes.json lights no longer in the xodr: roads %s" % sorted(stale) if stale else ""))
    for p in sorted(problems, key=lambda p: (p["cat"], p["carla"])):
        print("  FAIL %-24s CARLA %s: %s" % (p["cat"], carla(p["map"]), p["msg"]))
    for q in notes:
        print("  note %-24s CARLA (%.1f, %.1f): %s" % (q["cat"], q["carla"][0], q["carla"][1], q["msg"]))
    print("summary: " + (", ".join("%s %d" % kv for kv in sorted(cats.items())) if cats else "clean")
          + "; notes %d" % len(notes))
    print("markings check: %s (%d problems)" % ("PASS" if not problems else "FAIL", len(problems)))
    if a.json:
        json.dump({"pass": not problems, "problems": problems, "notes": notes, "summary": dict(cats)},
                  open(a.json, "w"), indent=1, ensure_ascii=False)
    sys.exit(1 if problems else 0)


def check_sg(a, out, meta, data, extra, L, elements, lanes, lanes_by, lane_lines, tree, x, s_light):
    """The LTA rule set (markings_sg.py, guide ch. 51). Returns the exit status.
    LTA 規則での検査（終了コードを返す）。"""
    import markings_sg as MS
    name = meta["name"]
    crossings = extra["crossings"]
    cw, jl = L.carriageway, L.junction_lanes
    area = None
    if a.area == "tiles":
        # the ground tiles this build made (BUILD_AREA builds whole 250 m tiles), 30 m in
        # この ビルドが作ったタイル（30 m 内側）
        import ground
        man = json.load(open(os.path.join(out, "meshes", "manifest.json")))
        made = {o["name"].rsplit("_", 1)[1] for o in man["objects"] if o["kind"] == "Road_Road"}
        area = unary_union([tb for tn, tb in ground.tiles(meta) if tn in made]).buffer(-30.0)
    elif a.area:
        from shapely.geometry import box as _box
        x0, y0, x1, y1 = [float(v) for v in a.area.split(",")]
        area = _box(x0 + 30.0, y0 + 30.0, x1 - 30.0, y1 - 30.0)      # 30 m in from the crop edge / 端から 30 m 内側
    inside = (lambda p: True) if area is None else (lambda p: area.contains(Point(float(p[0]), float(p[1]))))
    paint = None if a.no_mesh else Paint(out, ("Road_Marking", "Road_MarkingDeck"))
    ground_paint = None if a.no_mesh else Paint(out)
    sig_roads = set(s_light)
    problems, notes = [], []
    # each light's s and pole s (signs.py userData pole_s) / 信号の s と柱の s
    light_s = {}
    for r_ in ET.parse(os.path.join(out, name + ".xodr")).getroot().findall("road"):
        for sg_ in r_.findall("signals/signal"):
            if sg_.get("type") == "1000011":
                ps_ = [u.get("value") for u in sg_.findall("userData") if u.get("code") == "pole_s"]
                light_s[int(r_.get("id"))] = (float(sg_.get("s")), float(ps_[0]) if ps_ else None)
    import build as _build
    cfg = _build.road_style(name, "sg_lta") or {}
    cfg = cfg if isinstance(cfg, dict) else {}
    # the ground parts of the lanes (flyover parts are checked on their own level)
    # 車線の地上部分（高架部分は別扱い）
    glines = [(LineString([q["c"] for q in l["pts"]]), l) for l in MS.ground_runs(lanes) if len(l["pts"]) >= 2]
    gtree = STRtree([g for g, _ in glines])

    def bad(cat, p, msg, **kw):
        if not inside(p):
            return
        rec = {"cat": cat, "carla": [round(float(p[0]), 1), round(-float(p[1]), 1)],
               "map": [round(float(p[0]), 1), round(float(p[1]), 1)], "msg": msg}
        rec.update(kw)
        problems.append(rec)

    # classes as the generator saw them / 生成時の道路の格
    cls = pickle.load(open(os.path.join(out, "markings.pkl"), "rb")).get("cls", {})
    sections = collections.defaultdict(list)
    for l in lanes:
        sections[(l["road"], l["section"])].append(l)
    # -- crossings: square, kerb to kerb, off junction lanes, no overlap / 横断歩道
    bands = []
    for k, cr in enumerate(crossings):
        c, f, n = np.array(cr["c"]), np.array(cr["f"]), np.array(cr["n"])
        band = cr["band"]
        bands.append(band)
        tag = {"crossing": k, "src": cr.get("src", ""), "kind": cr.get("kind")}
        hs = []
        for i in gtree.query(band):
            ln, l = glines[int(i)]
            hit = ln.intersection(band)
            if hit.is_empty:
                continue
            q = hit.centroid if hit.geom_type != "Point" else hit
            hs.append(MJ.tangent(ln, ln.project(q)))
        if hs:
            fm = MJ.heading_mean(hs, f)
            if ang180(fm, f) > SKEW_DEG:
                bad("crossing skew", c, "band %.1f deg off the lanes it crosses" % ang180(fm, f), **tag)
            if max(ang180(h, fm) for h in hs) > 30.0:
                bad("crossing lanes", c, "band crosses lanes of a side street or the junction", **tag)
        else:
            bad("crossing lanes", c, "band crosses no road lane", **tag)
        for t in (cr["t0"], cr["t1"]):
            e = c + n * t
            d = cw.boundary.distance(Point(e))
            if d > KERB_TOL:
                bad("crossing span", e, "band end %.2f m from the kerb line" % d, **tag)
        # junction share: in all, and over junction lanes that turn across the band (not a
        # dead-end loop's neck); caps as markings_sg (50 % / 20 %) / 交差点車線上の割合（全体・転回）
        inj = band.intersection(jl).area / band.area if band.area else 0.0
        if inj > 0.0:
            dead = set(getattr(L, "dead_end_junctions", []) or [])
            turning = []
            for m in L.lanes:
                if m["junction"] < 0 or len(m["pts"]) < 2 or m["junction"] in dead:
                    continue
                g = LineString([q["c"] for q in m["pts"]])
                if g.intersects(band) and abs(float(MJ.tangent(g, g.project(band.centroid)) @ f)) < \
                        math.cos(math.radians(15.0)):
                    turning.append(MS.lane_poly(m))
            tshare = unary_union(turning).intersection(band).area / band.area if turning else 0.0
            if inj > MS.JUNCTION_TOTAL_MAX or tshare > MS.JUNCTION_TURN_MAX:
                bad("crossing in junction", c, "%.0f%% of the band over junction lanes, %.0f%% over turning ones "
                    "(max %.0f / %.0f)" % (100 * inj, 100 * tshare, 100 * MS.JUNCTION_TOTAL_MAX,
                                           100 * MS.JUNCTION_TURN_MAX), **tag)
            elif inj > 0.1 and inside(c):
                notes.append({"cat": "crossing at a junction", "carla": [round(float(c[0]), 1), round(-float(c[1]), 1)],
                              "msg": "%.0f%% over junction lanes, %.0f%% over turning ones" % (100 * inj, 100 * tshare)})
    for i in range(len(bands)):
        for k in range(i + 1, len(bands)):
            ov = bands[i].intersection(bands[k]).area
            if ov > 0.2:
                bad("crossing overlap", bands[i].intersection(bands[k]).centroid.coords[0],
                    "crossings %d and %d overlap by %.1f m2" % (i, k, ov))
    # lights on short roads (< 8 m: a stub, or the crossing sits on it) are served by the
    # line on the roads entering the small junction before them (markings_sg)
    # 短い道路（8 m 未満）の信号は手前の小交差点に入る道路の停止線で
    short_pred, frontier = set(), [r for r in sig_roads if x["len"].get(r, 99.0) < 12.0]
    appr_at = collections.defaultdict(set)
    for ap_ in data["approaches"]:
        appr_at[ap_["junction"]].add(ap_["road"])
    for _ in range(4):                       # up to 4 hops, as markings_sg / 最大 4 段
        nxt = []
        for r in frontier:
            j0 = x["pred"].get(r)
            if j0 is None or j0 in short_pred:
                continue
            short_pred.add(j0)
            nxt += [r2 for r2 in appr_at[j0] if x["len"].get(r2, 99.0) < 12.0]
        frontier = nxt
    # -- stop and give-way lines / 停止線・譲れ線
    per_lane = collections.Counter()
    lined = set()
    for e in elements:
        if e["kind"] not in ("stop_signal", "giveway"):
            continue
        l = lanes_by.get((e["road"], e.get("lane")))
        g = e["geom"]
        if g is None or g.is_empty or l is None:
            continue
        zz = np.array([q.get("z", 0.0) for q in l["pts"]])
        on = (zz > MS.Z0) if e.get("level") == "deck" else (zz <= MS.Z0)
        if on.sum() >= 2:
            l = dict(l, pts=[q for q, ok in zip(l["pts"], on) if ok])      # this level's part / この階の部分
        P = np.array([q["c"] for q in l["pts"]])
        ln = LineString(P)
        ctr, u, length, thick = rect_axes(g.convex_hull if e["kind"] == "giveway" else g)
        s_c = ln.project(Point(ctr))
        skew = abs(90.0 - ang180(u, MJ.tangent(ln, s_c)))
        tag = {"road": e["road"], "lane": e.get("lane"), "junction": e.get("junction"), "kind": e["kind"]}
        if skew > SKEW_DEG:
            bad("stop skew", ctr, "%s %.1f deg off square to its lane" % (e["kind"], skew), **tag)
        if e["kind"] == "giveway":
            if e["road"] in sig_roads:
                bad("giveway at signal", ctr, "give-way line on a signalised approach", **tag)
            continue
        lined.add(e["road"])
        lined.update(e.get("serves") or [])        # lights of short roads beyond (markings_sg) / 先の短い道路の信号
        per_lane[(e["road"], e.get("lane"), e.get("junction"))] += 1
        if not (0.28 <= thick <= 0.32):
            bad("stop thickness", ctr, "stop line %.2f m thick (LTA J: 0.30)" % thick, **tag)
        if e["road"] not in sig_roads and (e.get("junction") in short_pred or set(e.get("serves") or []) & sig_roads):
            pass                    # serves the light of a short road beyond the junction / 先の短い道路の信号
        elif e["road"] not in sig_roads:
            if e["road"] in {lt["road"] for lt in data["lights"]} and inside(ctr):
                notes.append({"cat": "light dropped", "carla": [round(float(ctr[0]), 1), round(-float(ctr[1]), 1)],
                              "msg": "lanes.json light no longer in the xodr (signs.py ran after build.py)"})
            else:
                bad("signal line without light", ctr, "stop line on a road with no light", **tag)
        s_front = s_c + thick / 2.0
        gap_end = ln.length - s_front
        fe = P[-1] - P[-2]
        fe = fe / (np.linalg.norm(fe) or 1.0)
        ext = LineString(list(map(tuple, P)) + [tuple(P[-1] + fe * 3.0)])
        gap_cross = None
        for cr in crossings:
            hit = ext.intersection(cr["band"])
            if hit.is_empty:
                continue
            ss = [ext.project(Point(q)) for gg in getattr(hit, "geoms", [hit]) for q in gg.coords]
            s_min, s_max = min(ss), max(ss)
            if ln.length - s_min <= MS.STOP_NEAR_CROSS and not (s_min < 2.0 and ln.length - s_max >= 3.0):
                gc = s_min - s_front
                gap_cross = gc if gap_cross is None else min(gap_cross, gc)
        # one straight line across the approach: a crossing that only some of its lanes
        # run through sets every lane's line back (generator: max over the lanes)
        # 停止線は進入路で 1 本: 一部の車線だけが通る横断歩道でも全車線を下げる
        cross_back = 0.0
        for m in lanes:
            if m["road"] != e["road"] or len(m["pts"]) < 3:
                continue
            Pm = np.array([q["c"] for q in m["pts"] if (q.get("z", 0.0) > MS.Z0) == (e.get("level") == "deck")])
            if len(Pm) < 3:
                continue
            fm_ = Pm[-1] - Pm[-3]
            fm_ = fm_ / (np.linalg.norm(fm_) or 1.0)
            em = LineString(list(map(tuple, Pm)) + [tuple(Pm[-1] + fm_ * 3.0)])
            for cr in crossings:
                hit = em.intersection(cr["band"])
                if hit.is_empty:
                    continue
                ss = [em.project(Point(q)) for gg in getattr(hit, "geoms", [hit]) for q in gg.coords]
                s_min, s_max = min(ss), max(ss)
                gap = (em.length - 3.0) - s_min
                if gap <= MS.STOP_NEAR_CROSS and not (s_min < 2.0 and (em.length - 3.0) - s_max >= 3.0):
                    cross_back = max(cross_back, gap + MS.STOP_BACK)
        # the generator keeps one line across the approach before the pole of ANY of its lanes
        # 停止線は進入路全車線の信号柱の手前（全車線で最大）
        # the light as signs.py left it: its s (CARLA's stop) and the pole (userData pole_s);
        # markings_sg places no line by the pole (netconvert's light s is a placeholder)
        # signs.py が置いた信号の s（停止位置）と柱（pole_s）で判定
        light_back = pole_back = None
        if e["road"] in light_s:
            cum_ = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(P, axis=0).T))])
            sx_ = np.array([q["s"] for q in l["pts"]])
            back_of = lambda s_: float(cum_[-1] - np.interp(s_, sx_, cum_)) if sx_[-1] >= sx_[0] else \
                float(np.interp(s_, sx_[::-1], cum_[::-1]))
            light_back = back_of(light_s[e["road"]][0])
            if light_s[e["road"]][1] is not None:
                pole_back = back_of(light_s[e["road"]][1])
        hi = max(1.5, cross_back + 1.0)       # RMS9 note 3: further back where a crossing needs it / 必要なら更に手前
        if e["road"] not in sig_roads and (e.get("junction") in short_pred or set(e.get("serves") or []) & sig_roads):
            hi = max(hi, 8.0)       # moved up from a short road beyond (its crossing / setback) / 先の短い道路から移動
        # no crossing between the line and the mouth (cars would stop past it) / 停止線と交差点の間に横断歩道が無いこと
        ahead = substring(ln, min(ln.length, s_front + 0.05), ln.length)
        for cr in crossings:
            if ahead.length > 0.1 and ahead.intersects(cr["band"]) and gap_cross is None and \
                    ln.project(ahead.intersection(cr["band"]).centroid) > 2.0:
                bad("stop after crossing", ctr, "a crossing lies between the stop line and the junction", **tag)
                break
        if gap_cross is not None and gap_cross < 1.0:
            bad("stop position", ctr, "stop line %.1f m before the crossing (LTA: 2 m)" % gap_cross, **tag)
        elif gap_cross is not None and gap_cross > 3.0 and gap_end > hi:
            bad("stop position", ctr, "stop line %.1f m before the crossing (LTA: 2 m)" % gap_cross, **tag)
        elif gap_cross is None and not (0.3 <= gap_end <= hi):
            bad("stop position", ctr, "stop line %.1f m before the junction mouth (want 0.3-%.1f)" % (gap_end, hi),
                **tag)
        if pole_back is not None and pole_back > gap_end - 0.2:
            bad("stop after pole", ctr, "light pole %.1f m before the stop line" % (pole_back - gap_end), **tag)
        if light_back is not None and light_back < gap_end - 0.3:
            bad("light past line", ctr, "CARLA stops %.1f m past the painted line (light s; run signs.py)"
                % (gap_end - light_back), **tag)
        if paint is not None and not paint.hits([ctr])[0]:
            bad("stop unpainted", ctr, "stop line centre not on white paint", **tag)
    for key, nline in per_lane.items():
        if nline > 1:
            l = lanes_by.get(key[:2])
            bad("stop double", l["pts"][-1]["c"], "%d stop lines on one approach lane" % nline, road=key[0])
    # a light on a stub road (< 3 m) is served by the line of the road entering the small
    # junction before it / 短い分割道路の信号は手前の道路の停止線で
    pred_of_line = {}
    for e in elements:
        if e["kind"] == "stop_signal" and e.get("junction") is not None:
            pred_of_line.setdefault(e["junction"], set()).add(e["road"])
    for rid in sig_roads:
        if rid in lined:
            continue
        if x["len"].get(rid, 99.0) < 12.0:
            # walk up the short roads before it (as markings_sg moved the line) / 手前の短い道路をたどる
            seen, front, served = set(), [rid], False
            for _ in range(4):
                nxt = []
                for r in front:
                    j0 = x["pred"].get(r)
                    if j0 is None or j0 in seen:
                        continue
                    seen.add(j0)
                    if pred_of_line.get(j0):
                        served = True
                    nxt += [r2 for r2 in appr_at[j0] if x["len"].get(r2, 99.0) < 12.0]
                front = nxt
            if served:
                continue
        l = next((l for l in lanes if l["road"] == rid), None)
        if l is not None and LineString([q["c"] for q in l["pts"]]).length > 3.0:
            bad("signal without line", l["pts"][-1]["c"], "traffic light on road %d but no stop line" % rid, road=rid)
    # -- zebras: zig-zags on every approach with room, none over yellow / ジグザグ線
    zz = collections.defaultdict(set)
    for e in elements:
        if e["kind"] == "zigzag":
            zz[e["crossing"]].add((e["road"], e["lane"], e["side"]))
    for k, cr in enumerate(crossings):
        if cr.get("kind") != "zebra":
            continue
        for i in gtree.query(cr["band"].buffer(0.3)):
            ln, l = glines[int(i)]
            if ln.distance(cr["band"]) > 0.3:
                continue
            hit = ln.intersection(cr["band"])
            if hit.is_empty:
                continue
            s0 = min(ln.project(Point(q)) for g in getattr(hit, "geoms", [hit]) for q in getattr(g, "coords", []))
            ids = sorted({abs(m["lane"]) for m in sections[(l["road"], l["section"])]})
            if cls.get((l["road"], l["section"])) in (MS.ESTATE, MS.NARROW):
                continue            # estate-road zebras have no zig-zags (Street View) / 団地内はジグザグ無し
            if s0 - 1.0 >= MS.ZZ_LEG + 0.6 and abs(l["lane"]) == ids[0] and (l["road"], l["lane"], "kerb") not in zz[k]:
                bad("zigzag missing", ln.interpolate(s0 - 3.0).coords[0], "zebra %d: kerb lane approach without its "
                    "zig-zag" % k, road=l["road"])
    if paint is not None and len(paint.tri["y"]):
        ytree = STRtree(paint.tri["y"])
        for e in elements:
            if e["kind"] != "zigzag":
                continue
            idx = ytree.query(e["geom"], predicate="intersects")
            ov = sum(paint.tri["y"][int(i)].intersection(e["geom"]).area for i in idx)
            if ov > 0.01:
                bad("zigzag over yellow", e["geom"].centroid.coords[0], "zig-zag over %.2f m2 of yellow paint" % ov)
    # -- white over yellow (overlapping paint z-fights) / 白と黄の重なり
    if paint is not None and len(paint.tri["y"]) and len(paint.tri["w"]):
        wtree = STRtree(paint.tri["w"])
        yc = paint.centroids("y")
        i, j = wtree.query(yc, predicate="intersects")
        if len(i):
            p0 = shapely.get_coordinates(yc[i[0]])[0]
            bad("paint overlap", p0, "%d yellow paint triangles lie on white paint" % len(np.unique(i)))
    # -- every painted element in its own colour (bus lanes, bays, BUS, yellow boxes yellow; the
    # rest white) / 各要素の色（バスレーン・停車帯・BUS・黄色ボックスは黄、他は白）
    if paint is not None:
        yellow_kinds = {"bus_lane", "bus_word", "bus_bay", "yellow_box"}
        for e in elements:
            g = e.get("geom")
            if g is None or g.is_empty or e["kind"] in ("stop_signal",):
                continue
            want = "y" if e["kind"] in yellow_kinds else "w"
            if e.get("col", want) != want:
                bad("element colour", g.centroid.coords[0], "%s generated %s (LTA: %s)" % (e["kind"], e.get("col"), want))
                continue
            ps = [q.representative_point() for q in getattr(g, "geoms", [g]) if q.area > 0.02][:6]
            if not ps:
                continue
            xy = [(p.x, p.y) for p in ps]
            ok, other = paint.hits(xy, want), paint.hits(xy, "w" if want == "y" else "y")
            if ok.sum() == 0 and other.sum() > 0:
                bad("element colour", ps[0].coords[0], "%s painted %s in the mesh (LTA: %s)"
                    % (e["kind"], "yellow" if want == "w" else "white", "yellow" if want == "y" else "white"))
    # -- arrows / 矢印
    for e in elements:
        if e["kind"] != "arrow":
            continue
        l = lanes_by.get((e["road"], e.get("lane")))
        if l is None:
            continue
        n = len({abs(m["lane"]) for m in sections[(l["road"], l["section"])]})
        c0 = e["geom"].centroid.coords[0]
        if n < 2:
            bad("arrow single lane", c0, "lane arrow on a single-lane approach", road=e["road"])
        if cls.get((l["road"], l["section"])) == MS.EXPWY:
            bad("arrow expressway", c0, "lane arrow on an expressway", road=e["road"])
    # -- kerb and centre lines by class / 格ごとの外側線・中央線
    lh = meta.get("lefthand", False)
    if ground_paint is not None:
        cedges = []
        for key, ls in sections.items():
            ls = sorted(ls, key=lambda l: abs(l["lane"]))
            cedges.append((LineString([p["r" if lh else "l"] for p in ls[-1 if lh else 0]["pts"]]), ls[-1]))
        ctree = STRtree([g for g, _ in cedges])
        bayz = unary_union([b["poly"].buffer(1.0) for b in getattr(L, "bus_bays", []) if "poly" in b])
        skip = unary_union([cr["band"].buffer(14.0) for cr in crossings] + [bayz, jl.buffer(2.0)])
        for key, ls in sections.items():
            c = cls.get(key)
            if c is None or c == MS.NARROW:
                continue
            ls = sorted(ls, key=lambda l: abs(l["lane"]))
            kl = ls[0]
            if max(q.get("z", 0.0) for q in kl["pts"]) > MS.Z0:
                continue
            ke = LineString([p["l" if lh else "r"] for p in kl["pts"]])
            if ke.length < 12.0:
                continue
            pts = [ke.interpolate(t, normalized=True) for t in np.linspace(0.1, 0.9, 9)]
            pts = [p for p in pts if not skip.contains(p)]
            if len(pts) >= 4:
                xy = []
                for p in pts:
                    s_ = ke.project(p)
                    v = MJ.tangent(ke, s_)
                    nrm = np.array([-v[1], v[0]]) * (1.0 if lh else -1.0)
                    xy += [(p.x + nrm[0] * d, p.y + nrm[1] * d) for d in (0.15, 0.2, 0.3)]
                hw = ground_paint.hits(xy, "w").reshape(-1, 3).any(axis=1)
                hy = ground_paint.hits(xy, "y").reshape(-1, 3).any(axis=1)
                hit = hw | hy
                if hit.sum() < len(pts) // 2:
                    bad("edge missing", pts[len(pts) // 2].coords[0], "kerb line missing on a %s section (%d/%d)"
                        % (MS.CLASS_NAME.get(c, c), hit.sum(), len(pts)), road=key[0])
                else:
                    # the colour the class wants: main roads double yellow (white on flyover
                    # ramps), estate white (maps.json estate_kerb), slip roads / expressways white
                    # 格の色: 幹線は黄、団地内・導流路・高速は白
                    want = "y" if (c == MS.MAIN and cfg.get("main_kerb", "double_yellow") == "double_yellow") or \
                        (c == MS.ESTATE and cfg.get("estate_kerb", "white") != "white") else "w"
                    good = hy if want == "y" else hw
                    if good.sum() < (hit.sum() + 1) // 2:
                        bad("kerb colour", pts[len(pts) // 2].coords[0], "kerb line on a %s section is %s, LTA wants %s "
                            "(%d/%d samples)" % (MS.CLASS_NAME.get(c, c), "white" if want == "y" else "yellow",
                                                 "yellow" if want == "y" else "white", good.sum(), hit.sum()),
                            road=key[0])
            ce_l = ls[-1]
            ce = LineString([p["r" if lh else "l"] for p in ce_l["pts"]])
            if ce.length < 12.0:
                continue
            probes = [ce.interpolate(t, normalized=True) for t in np.linspace(0.1, 0.9, 12)]
            opp = None
            for i in ctree.query(ce.buffer(0.6)):
                e2, l2 = cedges[int(i)]
                if l2["road"] != key[0] and sum(1 for p in probes if e2.distance(p) <= 0.5) >= 6:
                    opp = l2
                    break
            if opp is None or cls.get((opp["road"], opp["section"])) == MS.NARROW:
                continue
            probes = [p for p in probes if not skip.contains(p)]
            if len(probes) < 4:
                continue
            xy = [(p.x, p.y) for p in probes]
            # the E line is broken: a probe hits a dash or the side of a pair / 破線: 半分程度
            near = []
            for p in probes:
                v = MJ.tangent(ce, ce.project(p))
                near += [(p.x + v[0] * d, p.y + v[1] * d) for d in (-2.0, -1.0, 0.0, 1.0, 2.0)]
                nrm = np.array([-v[1], v[0]])
                near += [(p.x + nrm[0] * 0.1, p.y + nrm[1] * 0.1), (p.x - nrm[0] * 0.1, p.y - nrm[1] * 0.1)]
            hit = ground_paint.hits(near, "w").reshape(len(probes), -1).any(axis=1)
            if hit.sum() < len(probes) // 2:
                bad("centre missing", probes[len(probes) // 2].coords[0], "centre line missing on a two-way %s section"
                    % MS.CLASS_NAME.get(c, c), road=key[0])
    # -- ground paint over kerbs / islands / off the road / 縁石・島・車道外の塗装
    if ground_paint is not None:
        over = unary_union([L.kerb, L.islands]).buffer(-0.02)
        lots = unary_union([L.parking, L.driveways]).buffer(0.05)
        for col in ("w", "y"):
            cen = ground_paint.centroids(col)
            if not len(cen):
                continue
            xy = shapely.get_coordinates(cen)
            on_off = shapely.contains_xy(over, xy[:, 0], xy[:, 1])
            outside = ~shapely.contains_xy(cw.buffer(0.03), xy[:, 0], xy[:, 1]) & ~shapely.contains_xy(lots, xy[:, 0],
                                                                                                         xy[:, 1])
            for mask, what in ((on_off, "over the kerb or an island"), (outside & ~on_off, "off the carriageway")):
                pts_ = [p for p in xy[mask] if inside(p)]
                if pts_:
                    bad("paint off road", pts_[0], "%d %s paint triangles %s" % (len(pts_), col, what), count=len(pts_))
    cats = collections.Counter(p["cat"] for p in problems)
    kinds = collections.Counter(e["kind"] for e in elements)
    print("check_markings (Singapore, LTA): %s; %d crossings (%d zebra, %d signalised), %d lights%s; elements %s"
          % (name, len(crossings), sum(1 for c in crossings if c.get("kind") == "zebra"),
             sum(1 for c in crossings if c.get("kind") == "signal"), len(sig_roads),
             ("; area %s" % (area.bounds,)) if area is not None else "",
             ", ".join("%s %d" % kv for kv in sorted(kinds.items()))))
    if paint is not None:
        print("  paint: %d white / %d yellow triangles (ground + decks)" % (paint.n["w"], paint.n["y"]))
    for p in sorted(problems, key=lambda p: (p["cat"], p["carla"])):
        print("  FAIL %-24s CARLA %s: %s" % (p["cat"], carla(p["map"]), p["msg"]))
    for q in notes[:20]:
        print("  note %-24s CARLA (%.1f, %.1f): %s" % (q["cat"], q["carla"][0], q["carla"][1], q["msg"]))
    print("summary: " + (", ".join("%s %d" % kv for kv in sorted(cats.items())) if cats else "clean")
          + "; notes %d" % len(notes))
    print("markings check: %s (%d problems)" % ("PASS" if not problems else "FAIL", len(problems)))
    if a.json:
        json.dump({"pass": not problems, "problems": problems, "notes": notes, "summary": dict(cats),
                   "rules": "sg_lta"}, open(a.json, "w"), indent=1, ensure_ascii=False)
    return 1 if problems else 0


if __name__ == "__main__":
    main()
