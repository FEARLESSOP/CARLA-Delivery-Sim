"""Where a speed-limit post (OpenDRIVE 274) may stand: the free kerb.

signs.py used to put every 274 at t = +1.6 (keep-left roads: 1.6 m past the
reference-line kerb). On a one-way carriageway of a dual road whose other
carriageway runs right beside that kerb, +1.6 lands IN the other road's lane
(OitaKosen signal 900049, road 3981 -> inside road 3818 lane -1), and CARLA
then also snaps it onto that lane. Now each post goes to the first side that
is free of EVERY road's driving lanes (junction lanes too, lane ends
extended) and of the carriageway:
  1. the reference-line kerb, t = +KERB_T (keep-left; -KERB_T otherwise),
  2. the other kerb, t = -(W + KERB_T) (W = the road's lane widths at s),
  each searched outward up to SEARCH m (past a slip road or a narrow island
  when the kerb itself is taken); the side with the shorter step wins;
  3. none: the post is left out (signs.py) / left as it was (patch), reported.
"Free" = at least LANE_CLEAR from every driving-lane edge, outside the
carriageway (Layout.carriageway when the map's .osm is known, else the
lanes + 0.4 m gutter), and further than CARLA's CheckSignalsOnRoads snap
(0.7 lane widths + margin) from every lane centre line; with an embedded
model (meta "core") also CORE_CLEAR off its asphalt (core_merge.core_asphalt:
shoulders and corners wider than the lanes), so the post stands on its
pavement / kerb top.

  .venv/bin/python speed_spot.py <out_dir> <in.xodr> <out.xodr>
    re-checks the 274 signals of an existing xodr and rewrites ONLY the t of
    those that are not free (text edit: controllers, phases and every other
    byte stay as they were; signs.py must not be rerun on a finished build,
    guide ch. 18).

速度標識（274）の位置: 全道路の走行車線・車道に掛からない側の歩道に置く。
"""

import json
import os
import re
import sys

import numpy as np
from shapely.geometry import LineString, Point, Polygon
from shapely.ops import unary_union

KERB_T = 1.6           # 0.4 m gutter + 0.2 m kerb + 1 m verge (signs.py) / 路肩＋縁石＋植樹帯
LANE_CLEAR = 1.0       # post at least this far from every driving-lane edge / 走行車線端からの距離
LANE_CAP = 2.0         # lane ends extended (carla clamps s at lane ends) / 車線端の延長
CARLA_SNAP, SNAP_MARGIN = 0.7, 0.25
GUTTER = 0.4
CW_CLEAR = 0.5         # pole behind the 0.2 m kerb, not on it / 縁石の後ろ
SEARCH = 8.0           # how far past the kerb a post may go / 歩道から外への探索範囲
CORE_CLEAR = 0.4       # embedded model: post this far off its asphalt (plate half 0.3 + 0.1) / モデルの車道から


def _map_osm(name):
    p = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "maps.json")
    try:
        for v in json.load(open(p)).values():
            if isinstance(v, dict) and v.get("name") == name and v.get("osm"):
                return os.path.expanduser(v["osm"])
    except (OSError, ValueError):
        pass
    return None


class Spots(object):
    def __init__(self, out, meta=None, osm=None):
        self.meta = meta or json.load(open(os.path.join(out, "meta.json")))
        data = json.load(open(os.path.join(out, "lanes.json")))
        self.lanes = [l for l in data["lanes"] if l["kind"] == "driving" and len(l["pts"]) >= 2]
        polys, self.centres = [], []
        for l in self.lanes:
            P = l["pts"]
            g = Polygon([q["l"] for q in P] + [q["r"] for q in P][::-1]).buffer(0)
            if not g.is_empty:
                polys.append(g)
            for e, o in ((P[-1], P[-2]), (P[0], P[1])):
                d = np.array(e["c"]) - np.array(o["c"])
                d = d / (np.linalg.norm(d) or 1.0) * LANE_CAP
                el, er = np.array(e["l"]), np.array(e["r"])
                cap = Polygon([el, er, er + d, el + d]).buffer(0)
                if not cap.is_empty:
                    polys.append(cap)
            self.centres.append((LineString([q["c"] for q in P]), max(q["w"] for q in P)))
        self.lane_area = unary_union(polys)
        self.cw, self.cw_src = None, "lanes + gutter"
        osm = osm or _map_osm(self.meta["name"])
        if osm and os.path.isfile(osm):
            try:
                import layout
                self.cw = layout.Layout(self.meta, os.path.join(out, "lanes.json"), osm).carriageway
                self.cw_src = "Layout.carriageway"
            except Exception as e:                      # noqa: BLE001  layout needs the full toolchain
                print("speed_spot: no Layout (%s): lanes + gutter used" % e)
        if self.cw is None:
            self.cw = self.lane_area.buffer(GUTTER)
        # embedded model: its asphalt is road too / 埋め込みモデルの車道アスファルト
        self.core = None
        if self.meta.get("core"):
            import core_merge
            g = core_merge.core_asphalt(out, self.meta)
            self.core = g if g is not None and not g.is_empty else None
        self.lefthand = bool(self.meta.get("lefthand", False))

    def free(self, x, y):
        """Why the point is not free ('' when it is). / 空いていない理由。"""
        p = Point(x, y)
        if self.cw.distance(p) < CW_CLEAR:
            return "on or within %.1f m of the carriageway" % CW_CLEAR
        if self.lane_area.distance(p) < LANE_CLEAR:
            return "%.2f m from a driving lane" % self.lane_area.distance(p)
        for ln, w in self.centres:
            if ln.distance(p) < CARLA_SNAP * w + SNAP_MARGIN:
                return "within CARLA's snap of a lane centre"
        if self.core is not None and self.core.distance(p) < CORE_CLEAR:
            return "on or within %.1f m of the embedded model's asphalt" % CORE_CLEAR
        return ""

    def road_at(self, road, s):
        """Reference-line point, left normal, lane width sum at s (non-junction lanes < 0)."""
        ls = [l for l in self.lanes if l["road"] == road and l["lane"] < 0
              and l["pts"][0]["s"] - 0.5 <= s <= l["pts"][-1]["s"] + 0.5]
        if not ls:
            return None
        at = {}
        for l in ls:
            P = l["pts"]
            i = int(np.argmin([abs(q["s"] - s) for q in P]))
            at[l["lane"]] = P[i]
        inner, outer = at[max(at)], at[min(at)]
        ref = np.array(inner["l"], float)
        n = ref - np.array(inner["c"], float)
        n /= np.linalg.norm(n) or 1.0
        return ref, n, sum(q["w"] for q in at.values()), np.array(outer["r"], float)

    def choose(self, road, s, kerb_t=KERB_T):
        """(t, side, why_not_first) or (None, None, reasons). / 置く t と側。"""
        g = self.road_at(road, s)
        if g is None:
            return None, None, "road %s has no lanes at s=%.1f in lanes.json" % (road, s)
        ref, n, W, right = g
        # each side: from the kerb outward (across a slip road or a narrow island if
        # need be) up to SEARCH m; the side needing the shorter step wins, the
        # reference kerb on a tie / 各側で外へ探し、近い方（同じなら基準側）
        sides = [("reference kerb", 1.0), ("other kerb", -1.0)] if self.lefthand else [("kerb", -1.0)]
        found, why = [], []
        for side, sg in sides:
            for k in range(int(SEARCH / 0.1) + 1):
                extra = 0.1 * k
                if sg > 0:
                    t, p = kerb_t + extra, ref + n * (kerb_t + extra)
                else:
                    t, p = -(W + kerb_t + extra), right - n * (kerb_t + extra)
                w = self.free(p[0], p[1])
                if not w:
                    found.append((extra, sg < 0, t, side))
                    break
                if k == 0:
                    why.append("%s t=%.2f: %s" % (side, t, w))
            else:
                why.append("%s: nothing free within %.0f m" % (side, SEARCH))
        if not found:
            return None, None, "; ".join(why)
        extra, _, t, side = min(found)
        return t, side + ("" if extra < 0.05 else " +%.1f m (past a neighbouring lane or the model asphalt)" % extra), "; ".join(why)

def patch(out, src, dst):
    """Rewrite the t of 274 signals that stand on a lane; nothing else changes."""
    text = open(src).read()
    S = Spots(out)
    road_of = {}
    for mo in re.finditer(r'<road\b[^>]*\bid="([^"]+)"', text):
        road_of[mo.start()] = int(mo.group(1))
    starts = sorted(road_of)
    new_t, log, n = {}, [], 0
    for mo in re.finditer(r'<signal\s[^>]*type="274"[^>]*>', text):
        tag = mo.group(0)
        n += 1
        road = road_of[max(k for k in starts if k < mo.start())]
        sid = re.search(r'\bid="([^"]*)"', tag).group(1)
        s, t = float(re.search(r'\bs="([^"]*)"', tag).group(1)), float(re.search(r'\bt="([^"]*)"', tag).group(1))
        g = S.road_at(road, s)
        if g is not None:
            ref, nrm, W, right = g
            p = ref + nrm * t if t >= 0 else right - nrm * (-t - W)
            if not S.free(p[0], p[1]):
                continue
        nt, side, why = S.choose(road, s)
        if nt is None:
            log.append("%s road %d: NO free kerb (%s), left as it was" % (sid, road, why))
            continue
        if abs(nt - t) > 1e-3:
            new_t[sid] = "%.2f" % nt
            log.append("%s road %d s=%.1f: t %.2f -> %.2f (%s; %s)" % (sid, road, s, t, nt, side, why))

    def sub(mo):
        tag = mo.group(0)
        sid = re.search(r'\bid="([^"]*)"', tag).group(1)
        if sid not in new_t:
            return tag
        return re.sub(r'\bt="[^"]*"', 't="%s"' % new_t[sid], tag, count=1)
    open(dst, "w").write(re.sub(r'<signal\s[^>]*type="274"[^>]*>', sub, text))
    print("speed_spot: %d speed signals, %d moved (carriageway: %s)" % (n, len(new_t), S.cw_src))
    for line in log:
        print("  " + line)


if __name__ == "__main__":
    patch(*sys.argv[1:4])
