"""xodr lane marks from the PAINTED lines, Japan rule (carla-venv; guide ch. 4).

  carla-venv/bin/python osm_city/lane_change_jp.py build/<Name> [--dry-run]

netconvert writes ``<roadMark type="solid">`` / ``"broken"`` with no
``laneChange``, and CARLA reads every waypoint as ``lane_change = Both``: the
Traffic Manager and the route planner may change lanes anywhere (29 247 of
29 247 OitaStation waypoints). This step rewrites ONLY the ``<roadMark>``
elements of the final xodr from what the model paints (every
``meshes/Road_Marking_*.npz`` tile, the embedded core's included):

  boundary              painted              type / color       laneChange
  between lanes of one  yellow solid         solid / yellow     none (進路変更禁止)
  direction             white solid          solid / white      both (legal in Japan)
                        broken, nothing      broken|none        both
  kerb, median, centre  solid (any colour)   solid / colour     none
  (no lane of the same  broken               broken / colour    both
  direction beyond)     nothing              none               none

A boundary is sampled every 0.5 m along each lane section (a short line
across it, 0.45 m inside to 0.2 m outside: lane lines sit on it, edge lines
0.1 m outside, the MAIN centre's own solid 0.3 m inside, the other
direction's one beyond reach); paint of other kinds (zebras, stop lines,
arrows, diamonds, words: markings.pkl) is masked. A painted piece >= 8 m
(or filling its slot) is solid; shorter pieces <= 9 m of bare road apart
are a broken line; bare road >= 10 m is none, shorter gaps (crossings, the
end before a junction) join a neighbour; sections < 1 m inherit. A section
whose paint changes gets one roadMark per run (``sOffset`` from the section
start). Junction roads carry no paint in
Japan: type none (both between lanes, none outside; TM never changes lanes
in a junction anyway).

Keep-left netconvert roads (``--lefthand``, never ``rule="LHT"``): each road
is one direction, lane 0's mark is the kerb, the last lane's mark faces the
other direction. Only Japan maps (make_osm_city.sh: maps.json road_style
"japan"); runs after core_merge (the core's paint tile) and rewrites the
xodr in place (input kept as ``<Name>.xodr.prelanechange``); idempotent.
Report: ``build/<Name>/lane_change.json``.

塗った線から xodr の roadMark を書く（日本ルール: 黄色実線 = 進路変更禁止、
白実線・破線 = 変更可）。roadMark 以外は一切変えない。
"""

import argparse
import collections
import glob
import json
import os
import pickle
import re
import sys
import xml.etree.ElementTree as ET

import numpy as np
import shapely
from shapely import STRtree

import carla

STEP = 0.5            # sample spacing along the boundary / 境界の標本間隔
INSIDE = 0.45         # probe reaches this far into the lane / 車線内側へ
OUTSIDE = 0.20        # ... and this far beyond the boundary / 外側へ
DZ = 1.5              # paint within this height of the road (overpasses) / 高さの許容
BRIDGE = 1.5          # masked samples inside one painted piece / 1 本の線の中の隙間
SOLID_LEN = 8.0       # a piece this long is a solid line (dashes are 5 m) / 実線とみなす長さ
DASH_GAP = 9.0        # dashes this close form one broken line (gaps 5 m) / 破線の間隔
END_DASH = 6.0        # a lone dash this close to a section end still counts / 区間端の 1 本
TINY = 1.0            # sections shorter than this inherit / これより短い区間は継承
NONE_MIN = 10.0       # shorter unpainted gaps join a neighbour / 短い無塗装は隣へ
MASK_KINDS = {"zebra", "stop", "stop_signal", "diamond", "arrow", "tomare", "bus_text", "text", "speed"}
WIDTH = 0.15


def paint(bd):
    """(STRtree, z) of the white and yellow paint triangles, xodr frame.
    白・黄の塗装三角形（xodr 座標）。"""
    out = {}
    for col in ("white", "yellow"):
        T = []
        for f in sorted(glob.glob(os.path.join(bd, "meshes", "Road_Marking_*.npz"))):
            d = np.load(f)
            if "line_%s|V" % col in d.files:
                T.append(d["line_%s|V" % col][d["line_%s|I" % col]])
        T = np.concatenate(T) if T else np.zeros((0, 3, 3), np.float32)
        polys = shapely.polygons(T[:, :, :2].astype(float)) if len(T) else np.array([])
        out[col] = (STRtree(polys), T[:, :, 2].mean(1) if len(T) else np.zeros(0))
    return out


def mask(bd):
    """Paint that is not a line along the road (markings.pkl). / 線以外の標示。"""
    p = os.path.join(bd, "markings.pkl")
    if not os.path.exists(p):
        return None
    els = pickle.load(open(p, "rb")).get("elements", [])
    g = [e["geom"] for e in els if e.get("kind") in MASK_KINDS and e.get("geom") is not None]
    return STRtree([x.buffer(0.1) for x in g]) if g else None


def sections(root):
    """road id -> (junction?, length, [(s0, s1, {lane id: type})]). / 道路と車線区間。"""
    out = {}
    for road in root.iter("road"):
        rid = int(road.get("id"))
        L = float(road.get("length"))
        secs = road.find("lanes").findall("laneSection")
        starts = [float(s.get("s")) for s in secs]
        ends = starts[1:] + [L]
        lst = []
        for sec, s0, s1 in zip(secs, starts, ends):
            ids = {int(ln.get("id")): ln.get("type") for ln in sec.iter("lane")}
            lst.append((s0, s1, ids))
        out[rid] = (road.get("junction", "-1") != "-1", L, lst)
    return out


def probes(m, rid, lid, ids, s0, s1):
    """Sample positions s and probe lines across the outer boundary of lane
    ``lid`` (lane 0: the reference line) in xodr x/y, with the road z.
    車線 lid の外側境界を横切る短い線（xodr 座標）。"""
    n = max(1, int((s1 - s0 - 0.1) / STEP))
    ss = s0 + 0.05 + (np.arange(n) + 0.5) * (s1 - s0 - 0.1) / n
    if lid == 0:
        src = -1 if -1 in ids else (1 if 1 in ids else None)
        if src is None:
            return ss, [None] * len(ss)
        sgn = -1.0                      # the inner (left) edge of lane +-1, outward = -right
    else:
        src, sgn = lid, 1.0             # the outer edge = the waypoint's right / 外側 = 右
    out = []
    for s in ss:
        w = m.get_waypoint_xodr(rid, src, float(s))
        if w is None or w.lane_id != src:
            out.append(None)
            continue
        t = w.transform
        r = t.get_right_vector()
        c = t.location
        hw = w.lane_width / 2.0
        ex, ey = c.x + sgn * r.x * hw, c.y + sgn * r.y * hw
        ox, oy = sgn * r.x, sgn * r.y
        a = (ex - ox * INSIDE, -(ey - oy * INSIDE))
        b = (ex + ox * OUTSIDE, -(ey + oy * OUTSIDE))
        out.append((a, b, c.z))
    return ss, out


def hits(P, cols, msk):
    """Per probe: 'Y', 'W', 'N' or None (masked / no lane). / 標本ごとの塗り。"""
    idx = [i for i, p in enumerate(P) if p is not None]
    lab = [None] * len(P)
    if not idx:
        return lab
    lines = shapely.linestrings([[P[i][0], P[i][1]] for i in idx])
    z = np.array([P[i][2] for i in idx])
    got = {}
    for col, key in (("yellow", "Y"), ("white", "W")):
        tree, tz = cols[col]
        if len(tz) == 0:
            got[key] = set()
            continue
        a, b = tree.query(lines, predicate="intersects")
        ok = np.abs(tz[b] - z[a]) <= DZ
        got[key] = set(a[ok].tolist())
    masked = set()
    if msk is not None:
        a, _ = msk.query(lines, predicate="intersects")
        masked = set(a.tolist())
    for k, i in enumerate(idx):
        if k in masked and k not in got["Y"]:
            continue
        lab[i] = "Y" if k in got["Y"] else ("W" if k in got["W"] else "N")
    return lab


def runs(ss, lab, s0, s1):
    """[(s start, style)] with style in solid_y/broken_y/solid_w/broken_w/none.
    Painted pieces (one colour; masked samples inside bridged up to 1.5 m):
    >= SOLID_LEN, or filling the section, a solid line; shorter ones 'dashes',
    grouped into a broken line when <= DASH_GAP of unpainted road apart (a
    lone dash only at a section end or in a short section: the dashed line
    runs on in the next section; else noise, e.g. a zebra bar). Unpainted gaps
    under NONE_MIN join a neighbour; longer gaps are 'none'. Gap lengths count
    unpainted samples only (masked crossings, stop lines do not count).
    区間ごとの線種（塗り片 → 実線 / 破線 / 無し）。"""
    ss = list(ss)
    n = len(lab)
    cumN = np.concatenate([[0.0], np.cumsum([STEP if x == "N" else 0.0 for x in lab])])

    def nlen(i, j):                      # unpainted length of samples i..j-1 / 無塗装の長さ
        return float(cumN[j] - cumN[i]) if j > i else 0.0

    pieces = []                          # [colour, first index, last index] / 塗り片
    for k, x in enumerate(lab):
        if x not in ("Y", "W"):
            continue
        if pieces and pieces[-1][0] == x and ss[k] - ss[pieces[-1][2]] <= BRIDGE + STEP + 1e-6 and \
                all(lab[j] is None or lab[j] == x for j in range(pieces[-1][2] + 1, k)):
            pieces[-1][2] = k
        else:
            pieces.append([x, k, k])
    h = STEP / 2.0

    def span(p):
        return max(s0, ss[p[1]] - h), min(s1, ss[p[2]] + h)

    # a solid line cut by a crossing / stop line (masked) goes on beyond it
    # 横断歩道で切れた実線はその先へ続く
    k = 0
    while k + 1 < len(pieces):
        p, q = pieces[k], pieces[k + 1]
        if p[0] == q[0] and nlen(p[2] + 1, q[1]) <= 1.0 and \
                max(span(p)[1] - span(p)[0], span(q)[1] - span(q)[0]) >= SOLID_LEN:
            p[2] = q[2]
            del pieces[k + 1]
        else:
            k += 1

    short = (s1 - s0) < 12.0
    iv = []                              # [a, b, style, first, last] / 区間
    k = 0
    while k < len(pieces):
        c, i0, i1 = pieces[k]
        col = "y" if c == "Y" else "w"
        a, b = span(pieces[k])
        # fills its slot: no unpainted road (> 1 m) to the next paint / section end
        # 隣の塗り（または区間端）まで無塗装が無い = 実線の一部
        prev_end = pieces[k - 1][2] if k > 0 else -1
        next_start = pieces[k + 1][1] if k + 1 < len(pieces) else n
        fills = nlen(prev_end + 1, i0) <= 1.0 and nlen(i1 + 1, next_start) <= 1.0
        if b - a >= SOLID_LEN or fills:
            iv.append([a, b, "solid_" + col, i0, i1])
            k += 1
            continue
        j = k
        while j + 1 < len(pieces) and pieces[j + 1][0] == c and \
                span(pieces[j + 1])[1] - span(pieces[j + 1])[0] < SOLID_LEN and \
                nlen(pieces[j][2] + 1, pieces[j + 1][1]) <= DASH_GAP and \
                all(lab[x] in (None, "N") for x in range(pieces[j][2] + 1, pieces[j + 1][1])):
            j += 1
        bb = span(pieces[j])[1]
        lone = j == k
        if not lone or short or nlen(0, i0) <= END_DASH or nlen(pieces[j][2] + 1, n) <= END_DASH:
            if b - a >= 1.0 or not lone:
                iv.append([a, bb, "broken_" + col, i0, pieces[j][2]])
        k = j + 1
    if not iv:
        return [(s0, "none")]
    # yellow over white where they overlap / 重なりは黄優先
    iv.sort(key=lambda r: (r[0], 0 if r[2].endswith("y") else 1))
    out = [iv[0]]
    for r in iv[1:]:
        q = out[-1]
        if r[0] < q[1]:
            if r[2] == q[2]:
                q[1], q[4] = max(q[1], r[1]), max(q[4], r[4])
                continue
            if q[2].endswith("y") and not r[2].endswith("y"):
                if r[1] <= q[1]:
                    continue
                r[0] = q[1]
            else:
                q[1] = r[0]
        out.append(r)
    # gaps: short ones join the neighbours (split at the middle), long ones none
    # 隙間: 短いものは隣へ、長いものは無し
    res = []
    if nlen(0, out[0][3]) >= NONE_MIN:
        res.append([s0, out[0][0], "none", 0, 0])
    else:
        out[0][0] = s0
    for r in out:
        if res and res[-1][2] != "none":
            if nlen(res[-1][4] + 1, r[3]) >= NONE_MIN:
                res.append([res[-1][1], r[0], "none", 0, 0])
            elif res[-1][2] == r[2]:
                res[-1][1], res[-1][4] = r[1], r[4]
                continue
            else:
                mid = (res[-1][1] + r[0]) / 2.0
                res[-1][1], r[0] = mid, mid
        elif res:
            res[-1][1] = r[0]
        res.append(r)
    if res[-1][2] != "none" and nlen(res[-1][4] + 1, n) >= NONE_MIN:
        res.append([res[-1][1], s1, "none", 0, 0])
    else:
        res[-1][1] = s1
    merged = [res[0]]
    for r in res[1:]:
        if r[2] == merged[-1][2]:
            merged[-1][1] = r[1]
        elif r[1] - r[0] > 1e-3:
            merged.append(r)
    return [(float(r[0]), r[2]) for r in merged]


TYPE = {"solid_y": ("solid", "yellow"), "broken_y": ("broken", "yellow"), "solid_w": ("solid", "white"),
        "broken_w": ("broken", "white"), "none": ("none", "standard")}


def lane_change(style, same_dir):
    """Japan rule (user, 2026-11-09). / 日本ルール。"""
    if same_dir:
        return "none" if style == "solid_y" else "both"
    return "both" if style.startswith("broken") else "none"


def mark_xml(s_off, style, lc, indent):
    t, c = TYPE[style]
    w = "%.2f" % (WIDTH if t != "none" else 0.0)
    return '%s<roadMark sOffset="%s" type="%s" weight="standard" color="%s" width="%s" laneChange="%s" />' % (
        indent, ("%.4f" % s_off).rstrip("0").rstrip(".") if s_off else "0", t, c, w, lc)


RM = re.compile(r'[ \t]*<roadMark\b[^>]*?(?:/>|>.*?</roadMark>)[ \t]*\n?', re.S)


def patch_text(text, marks):
    """Replace the roadMark elements lane by lane; nothing else changes.
    marks: (road, section index, lane id) -> [xml lines]. / roadMark だけ置換。"""
    out, pos, nroad = [], 0, 0
    for mr in re.finditer(r'<road\b[^>]*\bid="([^"]+)"[^>]*>.*?</road>', text, re.S):
        rid = int(mr.group(1))
        block = mr.group(0)
        sec_i = -1
        pieces, p = [], 0
        for ms in re.finditer(r'<laneSection\b.*?</laneSection>', block, re.S):
            sec_i += 1
            sec = ms.group(0)
            sp, q = [], 0
            for ml in re.finditer(r'<lane\b[^>]*\bid="(-?\d+)"[^>]*>.*?</lane>', sec, re.S):
                key = (rid, sec_i, int(ml.group(1)))
                lane = ml.group(0)
                if key in marks:
                    first = RM.search(lane)
                    if first is not None:
                        ind = re.match(r'[ \t]*', first.group(0)).group(0)
                        new = "".join(x % {"i": ind} + "\n" for x in marks[key])
                        lane = lane[:first.start()] + new + RM.sub("", lane[first.start():])
                sp.append(sec[q:ml.start()])
                sp.append(lane)
                q = ml.end()
            sp.append(sec[q:])
            pieces.append(block[p:ms.start()])
            pieces.append("".join(sp))
            p = ms.end()
        pieces.append(block[p:])
        out.append(text[pos:mr.start()])
        out.append("".join(pieces))
        pos = mr.end()
        nroad += 1
    out.append(text[pos:])
    return "".join(out), nroad


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("build")
    ap.add_argument("--xodr", help="input xodr (default build/<Name>.xodr)")
    ap.add_argument("--out", help="output xodr (default: in place)")
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args()
    bd = os.path.abspath(a.build)
    name = json.load(open(os.path.join(bd, "meta.json")))["name"]
    src = a.xodr or os.path.join(bd, name + ".xodr")
    dst = a.out or src
    text = open(src).read()
    if 'rule="LHT"' in text:
        sys.exit("[lane_change_jp] rule=\"LHT\" in %s: netconvert --lefthand roads must not carry it (ch. 3)" % src)
    m = carla.Map(name, text)
    cols = paint(bd)
    msk = mask(bd)
    secs = sections(ET.fromstring(text))
    marks, stat = {}, collections.Counter()
    yellow_same = []
    for rid, (junc, L, lst) in secs.items():
        styl = {}                                    # (section, lane) -> runs / 区間の線種
        for si, (s0, s1, ids) in enumerate(lst):
            for lid in ids:
                if junc:
                    styl[si, lid] = [(s0, "none")]
                else:
                    ss, P = probes(m, rid, lid, ids, s0, s1)
                    styl[si, lid] = runs(ss, hits(P, cols, msk), s0, s1)
        if not junc:
            # sections too short to sample (netconvert leaves 0.2 m ones) take the
            # touching run of the lane next door / 短すぎる区間は隣の区間の線種を引き継ぐ
            for si, (s0, s1, ids) in enumerate(lst):
                if s1 - s0 >= TINY:
                    continue
                for lid in ids:
                    if si > 0 and (si - 1, lid) in styl and lst[si - 1][1] - lst[si - 1][0] >= TINY:
                        styl[si, lid] = [(s0, styl[si - 1, lid][-1][1])]
                    elif si + 1 < len(lst) and (si + 1, lid) in styl:
                        styl[si, lid] = [(s0, styl[si + 1, lid][0][1])]
        for si, (s0, s1, ids) in enumerate(lst):
            for lid, typ in ids.items():
                beyond = lid - 1 if lid < 0 else (lid + 1 if lid > 0 else None)
                same = beyond is not None and ids.get(beyond) == "driving" and typ == "driving"
                lines = []
                for s, st in styl[si, lid]:
                    lc = lane_change(st, same)
                    lines.append(mark_xml(s - s0, st, lc, "%(i)s"))
                    stat[("junction" if junc else ("between" if same else ("kerb" if lid == 0 else "outer")),
                          st, lc)] += 1
                    if same and st == "solid_y":
                        w = m.get_waypoint_xodr(rid, lid, float(min(s1 - 0.05, s + 0.5)))
                        yellow_same.append({"road": rid, "section": si, "lane": lid, "s": round(s, 1),
                                            "x": round(w.transform.location.x, 1) if w else None,
                                            "y": round(w.transform.location.y, 1) if w else None})
                marks[(rid, si, lid)] = lines
    new, nroad = patch_text(text, marks)
    rep = {"xodr": dst, "roads": nroad,
           "marks": {"/".join(k): v for k, v in sorted(stat.items())},
           "yellow_between_lanes": yellow_same}
    for k, v in sorted(stat.items()):
        print("[lane_change_jp] %-9s %-9s laneChange=%-5s %6d" % (k + (v,)))
    print("[lane_change_jp] %d yellow solid runs between lanes of one direction (no lane change)" % len(yellow_same))
    if a.dry_run:
        return
    if dst == src:
        open(src + ".prelanechange", "w").write(text)
    open(dst, "w").write(new)
    json.dump(rep, open(os.path.join(bd, "lane_change.json"), "w"), indent=1)
    print("[lane_change_jp] wrote %s (%d roads; input kept as .prelanechange)" % (dst, nroad))


if __name__ == "__main__":
    main()
