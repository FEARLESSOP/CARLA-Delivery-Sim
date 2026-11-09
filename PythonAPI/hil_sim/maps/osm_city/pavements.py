"""The pavement network of a Japanese map (maps.json "road_style": "japan";
maps/.venv). Walkers only route inside one connected piece of the navmesh
(sidewalk + crosswalk areas, grass excluded since nav_obj.py blocks it), so the
pavements must form ONE network, not 460 fragments (OitaKosen, 2026-11-06).

  歩道 (raised, kerbed, block paving)   on every MAIN / TERT road (markings_jp
      classes: 2+ lanes each way / tertiary up, the ring) and on roads OSM tags
      with a sidewalk: continuous from the kerb back, PAVE_W wide, round the
      junction corners (the band follows the kerb), round dead-end heads, joined
      to the OSM footways (no cut-outs) and to every crossing.
  side-street mouths   a MINOR side street meeting a paved road gets a zebra
      across its mouth MOUTH_SET m up the street (the Akeno practice: zebra at
      the T south of the football field, road_features.json); the pavement wraps
      the corner to the zebra. NARROW mouths (service drives) get an unpainted
      dropped-kerb apron that is a navmesh sidewalk band only.
  路側帯 (MINOR / NARROW streets, 3 m lanes, no pavement in reality): the
      ROSOKU_W strip inside the white edge line, part of the asphalt carriageway
      visually, but its own Road_Sidewalk mesh at road height (surfaces_jp cuts it
      out of the carriageway as "asphalt_rosoku"; the lanes.json lanes do not
      move). Pedestrians may use either side (歩行者は右側通行の区別は不要:
      道交法 10 条 2 項). Continuous round corners; across every mouth of an
      estate junction a navmesh-only band (MOUTH_BAND_W) joins the strips.
  islands   pavement pieces whose connected piece (pavements + strips + crossing
      bands + aprons + kerb connectors at footway ends) is under ISLAND_MIN m2
      are dropped unless a bus stop stands on them.
  nav_areas.json   the crossing bands (crosswalk) and the aprons / connectors
      (sidewalk) for nav_crosswalks.py + nav_obj.py (guide ch. 7, 31).

歩道網: 幹線の歩道（インターロッキング）、生活道路の路側帯、交差点の横断帯、
孤立片の除去、歩行者ナビメッシュ用の面。
"""

import json
import math
import os
import xml.etree.ElementTree as ET

import numpy as np
import shapely
from shapely.geometry import LineString, MultiPolygon, Point, Polygon
from shapely.ops import unary_union
from shapely.validation import make_valid

from markings_jp import MAIN, TERT, MINOR, NARROW, TERT_UP

PAVE_W = {MAIN: 3.5, TERT: 2.5}   # kerb back to pavement back (Street View: 3-4 m on the main road) / 歩道幅
ROSOKU_W = 0.75                    # 路側帯 strip inside the carriageway edge (0.5-0.75 m in practice) / 路側帯幅
MOUTH_SET = 2.5                    # side-street zebra centre, up the street from the junction boundary / 横断歩道の位置
MOUTH_BAND_W = 2.0                 # navmesh band across an estate junction mouth / 生活道路の交差点口の横断帯
ISLAND_MIN = 50.0                  # pavement pieces in smaller connected pieces are dropped / 孤立片の下限
GUIDE_W, GUIDE_OFF = 0.3, 1.85     # 誘導ブロック strip: width, centre from the carriageway edge / 誘導ブロック
WARN_D, WARN_BACK = 0.6, 0.3       # 点状ブロック at crossing kerbs: depth, set-back from the kerb back / 点状ブロック
PIT, PIT_EVERY, PIT_OFF = 1.0, 9.0, 1.1   # street-tree pits in MAIN pavements: size, spacing, centre from the edge / 植樹桝
LANDING_MAX = 8.0                  # a crossing end on bare ground is paved on to the next walkable surface / 横断歩道端の舗装
NAV_HULL = 0.4                     # crosswalk area reaches this far past the band (over the kerb) / 横断帯の余裕
SIDEWALK_TAGS = ("both", "left", "right", "yes")


def polys(g):
    if g is None or g.is_empty:
        return []
    if g.geom_type == "Polygon":
        return [g]
    return [q for q in getattr(g, "geoms", []) if q.geom_type == "Polygon" and not q.is_empty]


def clean(g, min_area=0.3):
    g = make_valid(g)
    return unary_union([p for p in polys(g) if p.area >= min_area])


def lane_poly(l):
    return Polygon([q["l"] for q in l["pts"]] + [q["r"] for q in l["pts"]][::-1]).buffer(0)


class Classes(object):
    """Road classes as markings_jp.MarkingsJP.classify (that needs the finished
    layout; this runs inside it), plus which roads carry a raised pavement.
    markings_jp と同じ道路の格付け＋歩道のある道路。"""

    def __init__(self, L, xodr):
        rid_way = {}
        if os.path.exists(xodr):
            for r in ET.parse(xodr).getroot().findall("road"):
                u = r.find("userData[@code='sumoId']")
                if u is not None:
                    rid_way[int(r.get("id"))] = u.get("value").lstrip("-").split("#")[0]
        self.rid_way = rid_way
        ways = L.osm.ways
        sections = {}
        for l in L.lanes:
            if l["junction"] < 0:
                sections.setdefault((l["road"], l["section"]), []).append(l)
        self.sections = sections
        self.cls, self.paved = {}, {}
        for key, lanes in sections.items():
            l = lanes[0]
            t = ways.get(rid_way.get(l["road"], ""), (None, {}))[1]
            hw = t.get("highway")
            w = float(np.median([q["w"] for q in l["pts"]]))
            try:
                nl = int(float((t.get("lanes") or "0").split(";")[0]))
            except ValueError:
                nl = 0
            if len(lanes) >= 2 or (hw in TERT_UP and nl >= 4):
                c = MAIN
            elif hw in TERT_UP or l["name"] == "Ring Road":
                c = TERT
            elif hw == "service" or t.get("lane_markings") == "no" or w < 2.95:
                c = NARROW
            else:
                c = MINOR
            self.cls[key] = c
            # OSM sidewalk tags make a street paved whatever its class / sidewalk タグ
            sw = any(str(t.get(k, "")).split(";")[0] in SIDEWALK_TAGS
                     for k in ("sidewalk", "sidewalk:both", "sidewalk:left", "sidewalk:right"))
            self.paved[key] = c >= TERT or sw
        self.road_cls = {}
        for (road, sec), c in self.cls.items():
            self.road_cls[road] = max(self.road_cls.get(road, 0), c)
        self.road_paved = {}
        for (road, sec), p in self.paved.items():
            self.road_paved[road] = self.road_paved.get(road, False) or p

    def of(self, l):
        return self.cls.get((l["road"], l["section"]), MINOR)


class Network(object):
    """Builds L.footpaths (pavements + OSM footways), L.pave_band (the block-paved
    part), L.rosoku (路側帯 strips), L.mouth_bands (navmesh aprons), the
    side-street zebras (added to L.features["crossings"] for markings_jp) and,
    after the markings exist, finish(): islands dropped, tactile zones, tree pits,
    nav areas. / 歩道網の構築（配置段階）と仕上げ（標示の後）。"""

    def __init__(self, L, osm_footpaths, xodr):
        from layout import GUTTER, KERB_W
        self.L = L
        self.C = Classes(L, xodr)
        C = self.C
        cw = L.carriageway
        kerb_out = KERB_W
        # lanes by junction / 交差点ごとの車線
        byj = {}
        for l in L.lanes:
            if l["junction"] >= 0 and len(l["pts"]) >= 2:
                byj.setdefault(l["junction"], []).append(lane_poly(l))
        self.jpoly = {j: unary_union(v) for j, v in byj.items()}
        dead = getattr(L, "dead_end_junctions", set())
        # non-junction lanes by class / 格ごとの単路車線
        paved_lanes = {MAIN: [], TERT: []}
        unpaved_lanes = []
        for l in L.lanes:
            if l["junction"] >= 0 or len(l["pts"]) < 2:
                continue
            key = (l["road"], l["section"])
            if C.paved.get(key):
                paved_lanes[MAIN if C.cls[key] == MAIN else TERT].append(lane_poly(l))
            else:
                unpaved_lanes.append(lane_poly(l))
        self.paved_lane_area = unary_union(paved_lanes[MAIN] + paved_lanes[TERT])
        pl_main = unary_union(paved_lanes[MAIN]) if paved_lanes[MAIN] else Polygon()
        pl_tert = unary_union(paved_lanes[TERT]) if paved_lanes[TERT] else Polygon()
        # junctions a paved road enters / 歩道のある道路が入る交差点
        self.paved_j = set()
        jcls = {}
        for j, jp in self.jpoly.items():
            if j in dead:
                continue
            jb = jp.buffer(0.3)
            if jb.intersects(self.paved_lane_area):
                self.paved_j.add(j)
                jcls[j] = MAIN if (not pl_main.is_empty and jb.intersects(pl_main)) else TERT
        # the approaches of every junction: (junction, road, lane, s index of the
        # junction end) / 各交差点の流入・流出路
        self.app = {}
        for l in L.lanes:
            if l["junction"] >= 0 or len(l["pts"]) < 2:
                continue
            p0, p1 = Point(l["pts"][0]["c"]), Point(l["pts"][-1]["c"])
            for j, jp in self.jpoly.items():
                d0, d1 = jp.distance(p0), jp.distance(p1)
                if min(d0, d1) < 0.6:
                    self.app.setdefault(j, []).append((l, d0 <= d1))
        # -- raised pavements / 歩道
        reach = lambda w: GUTTER + kerb_out + w + 0.8
        zone_main = unary_union([pl_main.buffer(reach(PAVE_W[MAIN]), quad_segs=4)]
                                + [self.jpoly[j].buffer(reach(PAVE_W[MAIN]), quad_segs=4)
                                   for j in self.paved_j if jcls[j] == MAIN]) if not pl_main.is_empty else Polygon()
        zone_tert = unary_union([pl_tert.buffer(reach(PAVE_W[TERT]), quad_segs=4)]
                                + [self.jpoly[j].buffer(reach(PAVE_W[TERT]), quad_segs=4)
                                   for j in self.paved_j if jcls[j] == TERT]) if not pl_tert.is_empty else Polygon()
        band = lambda w: cw.buffer(kerb_out + w, quad_segs=3).difference(cw.buffer(kerb_out, quad_segs=3))
        pave = unary_union([band(PAVE_W[MAIN]).intersection(zone_main), band(PAVE_W[TERT]).intersection(zone_tert)])
        if not pave.is_valid:
            pave = make_valid(pave)
        # side streets: the pavement wraps the corner up to the mouth crossing, the
        # street beyond is cut away; MINOR mouths get a zebra, NARROW an apron
        # 脇道: 歩道は角を回って横断歩道まで、その先は切る
        self.mouth_zebras, self.mouth_bands, self.mouth_log = [], [], []
        import geo
        fr = geo.Frame(L.meta)
        feats = L.features.setdefault("crossings", [])
        existing = [Point(fr.xy(c["lon"], c["lat"])) for c in feats if "lat" in c] + \
            [p for p, t, nid in L.osm.points(lambda t: t.get("highway") == "crossing")]
        # markings_jp draws a crossing across the lane NEAREST its point: the point
        # goes up the street until that lane is the street's own (2.5 m up a short
        # arm at the ring the ring's lane was nearer: a zebra across the ring)
        # 横断歩道の点は、最寄り車線がその道路になるまで奥へ（環状道路を横切らない）
        lls = [(LineString([q["c"] for q in l["pts"]]), l["road"]) for l in L.lanes
               if l["junction"] < 0 and len(l["pts"]) >= 2]
        ltree = shapely.STRtree([g for g, _ in lls])

        def set_back(c_pt, d_in, road, width):
            # ... and where the line across (as markings_jp draws it: every piece of
            # carriageway within 10 m of the point) is no wider than the street's
            # plain width (its narrowest within 2.5-10.5 m): at a skewed mouth it ran
            # on across the ring to its outer kerb. The nearest lane may be either
            # direction of the street (netconvert: one road per direction).
            # 横断線が道路の素の幅を超えない所（斜めの口で環状道路まで延びた）
            nrm = np.array([-d_in[1], d_in[0]])
            rows = []
            for s_ in np.arange(MOUTH_SET, MOUTH_SET + 8.01, 0.5):
                q = c_pt + d_in * s_
                i = ltree.query_nearest(Point(*q), max_distance=12.0)
                ok = False
                nq = nrm
                if len(i):
                    g, rd = lls[int(i[0])]
                    w0 = C.rid_way.get(road)
                    ok = rd == road or (w0 is not None and C.rid_way.get(rd) == w0)
                    # the across direction exactly as markings_jp takes it: normal to that
                    # lane at the point / markings_jp と同じ向き（その車線の法線）
                    pq = g.project(Point(*q))
                    a, b = g.interpolate(max(0.0, pq - 1.0)), g.interpolate(min(g.length, pq + 1.0))
                    yaw = math.atan2(b.y - a.y, b.x - a.x)
                    nq = np.array([-math.sin(yaw), math.cos(yaw)])
                g = LineString([q - nq * 30.0, q + nq * 30.0]).intersection(cw)
                segs = [x for x in getattr(g, "geoms", [g]) if x.geom_type == "LineString"
                        and x.distance(Point(*q)) < 10.0]
                if ok and segs and any(unary_union(segs).intersects(jp_.buffer(-0.3))
                                       for jp_ in self.jpoly.values() if jp_.distance(Point(*q)) < 12.0):
                    ok = False          # (into a junction: the ring's junction lanes) / 交差点内に掛かる
                if ok and segs:
                    # the line across must not cut another road's lane (the ring) / 他の道路の車線を横切らない
                    w0 = C.rid_way.get(road)
                    for k_ in ltree.query(unary_union(segs), predicate="intersects"):
                        rd2 = lls[int(k_)][1]
                        if rd2 != road and (w0 is None or C.rid_way.get(rd2) != w0):
                            ok = False
                            break
                rows.append((s_, q, ok, sum(x.length for x in segs) if segs else 1e9))
            ref = min(r[3] for r in rows)
            if ref > 32.0:
                return None, None
            for s_, q, ok, tot in rows:
                # (width: one direction's lanes + gutters; netconvert gives the other
                # direction its own road) / 幅は片方向分
                if ok and tot <= min(ref + 1.0, 2.0 * width + 1.5):
                    return s_, q
            return None, None
        self.mouth_skipped = 0
        cuts = []
        wraps = []          # the corner wrap reaches every mouth crossing / 角の歩道は横断歩道まで
        for j in self.paved_j:
            seen = set()
            arms = {l["road"]: C.cls.get((l["road"], l["section"]), MINOR) for l, _ in self.app.get(j, [])}
            top = max(arms.values()) if arms else MINOR
            jc = self.jpoly[j].centroid
            j_has = any(jc.distance(q) < 20.0 for q in existing)
            for l, at_start in self.app.get(j, []):
                key = (l["road"], l["section"])
                if l["road"] in seen:
                    continue
                seen.add(l["road"])
                cls = C.cls.get(key, MINOR)
                if C.paved.get(key):
                    # a paved arm: zebra across it when it is the lesser road here, or
                    # at a junction of equal TERT roads without any crossing (never
                    # across the ring): the corner pavements stay joined
                    # 歩道のある枝: 格下の枝、または横断歩道の無い同格の交差点の枝に横断歩道
                    if l["name"] == "Ring Road" or cls == MAIN and cls == top:
                        continue
                    if cls < top or not j_has:
                        mouth = self.mouth_geometry(j, l["road"])
                        if mouth is not None:
                            s_, q = set_back(mouth[0], mouth[1], l["road"], mouth[2])
                            if q is not None:
                                self.mouth_zebras.append((float(q[0]), float(q[1]), j, l["road"]))
                    continue
                mouth = self.mouth_geometry(j, l["road"])
                if mouth is None:
                    continue
                c_pt, d_in, width, lanes_here = mouth
                s_set, q_set = set_back(c_pt, d_in, l["road"], width) if cls >= MINOR else (None, None)
                if cls >= MINOR and q_set is None:
                    self.mouth_skipped += 1          # (no room for a zebra: an apron instead) / 余地なし
                    cls = NARROW
                far = s_set + 2.0 + 0.3 if cls >= MINOR else MOUTH_BAND_W / 2 + 1.0 + 0.3
                for ll in lanes_here:
                    pts = ll["pts"]
                    s_end = pts[0]["s"] if Point(pts[0]["c"]).distance(self.jpoly[j]) < 0.6 else pts[-1]["s"]
                    beyond = [q["c"] for q in pts if abs(q["s"] - s_end) >= far]
                    if len(beyond) >= 2:
                        # (never within `far` of the junction: a skewed cut ate the corner
                        # wrap of the arm beside it) / 交差点から far 以内は切らない
                        cut = LineString(beyond).buffer(2 * pts[0]["w"] + GUTTER + kerb_out + PAVE_W[MAIN] + 1.0,
                                                        cap_style="flat").difference(
                            self.jpoly[j].buffer(far + 0.3, quad_segs=4))
                        # only the pavement that belongs to this street: not the band of
                        # any other road's lanes (the paved road, the arm beside it)
                        # この道路の歩道だけを切る（他の道路の帯は残す）
                        w0 = C.rid_way.get(l["road"])
                        oth = [lane_poly(o) for o in L.lanes if o["junction"] < 0 and len(o["pts"]) >= 2
                               and o["road"] != l["road"] and (w0 is None or C.rid_way.get(o["road"]) != w0)
                               and Point(o["pts"][0]["c"]).distance(cut) < 60.0]
                        oth = [o for o in oth if o.distance(cut) < GUTTER + kerb_out + PAVE_W[MAIN] + 0.5]
                        if oth:
                            cut = cut.difference(unary_union(oth).buffer(GUTTER + kerb_out + PAVE_W[MAIN] + 0.3,
                                                                         quad_segs=3))
                        cuts.append(cut)
                if cls >= MINOR:
                    self.mouth_zebras.append((float(q_set[0]), float(q_set[1]), j, l["road"]))
                    wraps.append(Point(*q_set).buffer(2 * width + PAVE_W[TERT] + 1.0, quad_segs=6))
                else:
                    q = c_pt + d_in * (MOUTH_BAND_W / 2 + 1.0)
                    self.mouth_bands.append(self.band_at(q, d_in, MOUTH_BAND_W, width))
                    wraps.append(Point(*q).buffer(2 * width + PAVE_W[TERT] + 1.0, quad_segs=6))
        if wraps:
            pave = unary_union([pave, band(PAVE_W[TERT]).intersection(unary_union(wraps))])
        if cuts:
            pave = pave.difference(unary_union(cuts))
        inside = getattr(L, "paved_inside", L.bounds)
        # medians and traffic islands stay planted (Street View: shrubs on the main
        # road's median); a crossing over one gets a refuge in finish()
        # 分離帯・交通島は植栽のまま（横断歩道部は finish で安全地帯）
        isl = [q for q in polys(getattr(L, "islands", None)) if q.buffer(-3.0).is_empty]    # narrow ones only / 細いもののみ
        if isl:
            pave = pave.difference(unary_union(isl).buffer(0.05))
        self.pave_band = clean(pave.intersection(inside), 1.0)
        # every OSM footway joins; the kerb ring stays clear / OSM の歩道も合わせる
        L.pave_band = self.pave_band
        L.footpaths = clean(unary_union([osm_footpaths, self.pave_band]).difference(L.road_all), 0.3)
        # -- 路側帯 strips and estate junction aprons / 路側帯と生活道路の交差点口
        unpaved_zone = unary_union(unpaved_lanes + [self.jpoly[j] for j in self.jpoly if j not in self.paved_j]) \
            .buffer(GUTTER + 0.3, quad_segs=2) if unpaved_lanes else Polygon()
        strip = cw.difference(cw.buffer(-ROSOKU_W, quad_segs=2))
        strip = strip.intersection(unpaved_zone).difference(self.pave_band.buffer(kerb_out + 0.3, quad_segs=2))
        self.rosoku = clean(strip.intersection(inside), 0.5)
        L.rosoku = self.rosoku
        for j in self.jpoly:
            if j in self.paved_j or j in dead:
                continue
            seen = set()
            for l, at_start in self.app.get(j, []):
                if l["road"] in seen:
                    continue
                seen.add(l["road"])
                mouth = self.mouth_geometry(j, l["road"])
                if mouth is None:
                    continue
                c_pt, d_in, width, _ = mouth
                self.mouth_bands.append(self.band_at(c_pt + d_in * (MOUTH_BAND_W / 2 + 0.3), d_in, MOUTH_BAND_W, width))
        # (each apron reaches NAV_HULL over the kerb, like the crossing bands) / 縁石を越える
        self.mouth_bands = [b.buffer(NAV_HULL, join_style="mitre").intersection(cw.buffer(KERB_W + 0.05))
                            .intersection(inside) for b in self.mouth_bands if b is not None and not b.is_empty]
        self.mouth_bands = [b for b in self.mouth_bands if not b.is_empty]
        L.mouth_bands = self.mouth_bands
        # the side-street zebras go to markings_jp through road_features "crossings"
        # 脇道の横断歩道は road_features の crossings として標示へ
        n_added = 0
        from shapely import wkt
        ring = wkt.loads(L.meta["ring_wkt"]) if L.meta.get("ring_wkt") else None    # the ring's centre line / 外周の中心線
        for x, y, j, road in self.mouth_zebras:
            p = Point(x, y)
            if any(p.distance(q) < 8.0 for q in existing) or not inside.contains(p) or \
                    (ring is not None and not ring.contains(p)):
                continue            # (stubs leaving the map past the ring: nothing to walk to) / 外周の外
            lon, lat = fr.lonlat(x, y)
            feats.append({"lat": lat, "lon": lon, "signal": False, "where": "pavement network: mouth of road %d at junction %d"
                          % (road, j), "_auto": True})
            n_added += 1
        self.mouth_zebra_count = n_added

    def mouth_geometry(self, j, road):
        """(centre point at the junction boundary, unit direction up the street,
        kerb-to-kerb width, the street's lanes there) of road ``road`` where it
        leaves junction ``j``. / 交差点から見た脇道の口の中心・向き・幅。"""
        lanes = [l for l, at_start in self.app.get(j, []) if l["road"] == road]
        if not lanes:
            return None
        jp = self.jpoly[j]
        ends, dirs, ws = [], [], 0.0
        for l in lanes:
            pts = l["pts"]
            at_start = Point(pts[0]["c"]).distance(jp) <= Point(pts[-1]["c"]).distance(jp)
            a, b = (pts[0], pts[min(len(pts) - 1, 3)]) if at_start else (pts[-1], pts[max(0, len(pts) - 4)])
            ends.append(a["c"])
            d = np.array(b["c"]) - np.array(a["c"])
            n = np.linalg.norm(d)
            if n > 1e-6:
                dirs.append(d / n)
            ws += a["w"]
        if not dirs:
            return None
        from layout import GUTTER
        d_in = np.mean(dirs, axis=0)
        d_in /= max(np.linalg.norm(d_in), 1e-9)
        return np.mean(np.array(ends), axis=0), d_in, ws + 2 * GUTTER, lanes

    def band_at(self, q, d_in, w, width):
        """A band w deep across the carriageway at q (perpendicular to d_in). / 横断帯。"""
        n = np.array([-d_in[1], d_in[0]])
        half = width / 2 + 3.0
        line = LineString([q - n * half, q + n * half]).intersection(self.L.carriageway)
        segs = [g for g in getattr(line, "geoms", [line]) if g.geom_type == "LineString" and g.distance(Point(q)) < 1.0]
        if not segs:
            return None
        return unary_union([s.buffer(w / 2, cap_style="flat") for s in segs]).intersection(self.L.carriageway)

    # -- after the markings / 標示の後
    def finish(self, crossings, bus_stops=()):
        """Drop pavement islands, make the tactile zones, tree pits and the navmesh
        areas. crossings: markings_jp.MarkingsJP.crossings. Returns a report dict.
        孤立片の除去・点字ブロック・植樹桝・ナビメッシュ面。"""
        from layout import GUTTER, KERB_W
        L = self.L
        cw = L.carriageway
        bands = [cr["band"] for cr in crossings]
        self.cross_bands = bands
        # a crossing over a median or island: the island piece under it is a paved
        # refuge (kerbs dropped by the band), not grass / 分離帯上の横断歩道: 安全地帯
        refuges = []
        for cr in crossings:
            c, n, f, w = np.array(cr["c"]), np.array(cr["n"]), np.array(cr["f"]), cr["width"]
            rect = Polygon([tuple(c + n * cr["t0"] - f * w / 2), tuple(c + n * cr["t0"] + f * w / 2),
                            tuple(c + n * cr["t1"] + f * w / 2), tuple(c + n * cr["t1"] - f * w / 2)])
            hit = rect.intersection(L.islands)
            refuges += [p for p in polys(hit) if p.area > 0.5]
        # a crossing end on bare ground (the gap of a divided street, a verge):
        # a paved landing on to the next walkable surface, LANDING_MAX m at most
        # 横断歩道の端が地面に出る所: 次の歩行面までの舗装
        walkable = unary_union([L.footpaths, self.rosoku] + bands)
        shapely.prepare(walkable)
        cw_b = cw.buffer(0.05)
        for cr in crossings:
            c, n, f, w = np.array(cr["c"]), np.array(cr["n"]), np.array(cr["f"]), cr["width"]
            for t, sgn in ((cr["t0"], -1.0), (cr["t1"], 1.0)):
                e = c + n * t
                if walkable.distance(Point(*e)) <= KERB_W + 0.3 and all(
                        walkable.distance(Point(*(e + sgn * n * (KERB_W + 0.8) + f * o))) < 0.05 for o in (-1.2, 0, 1.2)):
                    continue
                reach = None
                for d in np.arange(0.5, LANDING_MAX + 0.01, 0.25):
                    q = Point(*(e + sgn * n * d))
                    if d > 1.0 and cw_b.contains(q):
                        # back on a road across a narrow island: the island piece is a
                        # refuge (two-stage crossing; the kerb faces beside the band drop)
                        # 島を越えて道路に戻る: 島の部分を安全地帯に
                        reach = d + 0.05
                        break
                    if walkable.distance(q) < 0.25:
                        reach = d + 0.3
                        break
                if reach is None:
                    continue
                reach = max(reach, KERB_W + 1.2)          # at least a 1.2 m landing / 最低 1.2 m
                a0, a1 = e + sgn * n * KERB_W, e + sgn * n * reach
                refuges.append(Polygon([tuple(a0 - f * w / 2), tuple(a0 + f * w / 2), tuple(a1 + f * w / 2),
                                        tuple(a1 - f * w / 2)]).difference(L.road_all))
        self.refuges = clean(unary_union(refuges), 0.3) if refuges else Polygon()
        if not self.refuges.is_empty:
            # never into a landmark's own ground or a building (the 7-Eleven lot's
            # fence stood on a landing) / 詳細モデルの敷地・建物には入れない
            sites = L.landmark_sites(lambda lm: True)
            keep_out = unary_union([g for g in (sites, getattr(L, "building_area", None)) if g is not None])
            if not keep_out.is_empty:
                self.refuges = clean(self.refuges.difference(keep_out.buffer(0.3)), 0.3)
        if not self.refuges.is_empty:
            L.footpaths = clean(unary_union([L.footpaths, self.refuges]), 0.3)
            self.pave_band = unary_union([self.pave_band, self.refuges])
            L.islands = clean(L.islands.difference(self.refuges), 0.5)
        # kerb connectors where a footway meets a street without a crossing (the
        # kerb is "road" in the navmesh, one 0.2 m cell: a barrier) / 歩道端の縁石を渡す面
        kerb = L.kerb
        conn = []
        foot_pieces = polys(L.footpaths)
        tree = shapely.STRtree(foot_pieces)
        strips = polys(self.rosoku)
        stree = shapely.STRtree(strips)
        for sp in strips:
            near = sp.buffer(KERB_W + 0.35)
            for i in tree.query(near, predicate="intersects"):
                piece = foot_pieces[i]
                touch = piece.buffer(0.3).intersection(kerb).intersection(near)
                for t in polys(touch):
                    if t.area > 0.02:
                        conn.append(t.buffer(0.2, quad_segs=2))
        self.connectors = clean(unary_union(conn), 0.02) if conn else Polygon()
        # connected pieces of the walkable surface / 歩行面の連結成分
        # (crossing bands reach over the kerb, as their navmesh area does) / 横断帯は縁石を越える
        link = lambda: unary_union([L.footpaths, self.rosoku, self.connectors] + self.mouth_bands
                                   + [b.buffer(NAV_HULL, join_style="mitre") for b in bands]).buffer(0.05)
        walk = link()
        comps = polys(walk)
        ctree = shapely.STRtree(comps)
        areas = np.array([c.area for c in comps])
        stops = [Point(p.x, p.y).buffer(4.0) for p in bus_stops]
        keep, dropped, dropped_area = [], 0, 0.0
        for piece in foot_pieces:
            idx = ctree.query(piece.representative_point(), predicate="within")
            a = float(areas[idx].max()) if len(idx) else piece.area
            if a < ISLAND_MIN and not any(piece.intersects(s) for s in stops):
                dropped += 1
                dropped_area += piece.area
                continue
            keep.append(piece)
        L.footpaths = MultiPolygon(keep) if keep else Polygon()
        self.pave_band = clean(self.pave_band.intersection(L.footpaths.buffer(0.01)), 0.5) if keep else Polygon()
        L.pave_band = self.pave_band
        # 路側帯 fragments cut off by a corner pavement: back to plain road / 孤立した路側帯片
        keep_s = []
        for piece in strips:
            idx = ctree.query(piece.representative_point(), predicate="within")
            a = float(areas[idx].max()) if len(idx) else piece.area
            if a >= ISLAND_MIN:
                keep_s.append(piece)
            else:
                dropped += 1
        if L.meta.get("core") and keep_s:
            # embedded model (guide ch. 35): core_merge cuts the meshes at its square and
            # drops the crossings / aprons centred inside, so a strip piece outside that
            # joined the rest only through the square (the seam fill's wedge edge leaving
            # the lane zone) is a fragment too: back to plain road
            # 埋め込みモデル: 範囲内でしか繋がらない範囲外の路側帯片も車道に戻す
            from core_merge import core_rect
            from shapely.geometry import box
            sq = box(*core_rect(L.meta))
            out = lambda gs: [g for g in gs if not sq.contains(g.centroid)]
            walk_o = unary_union([L.footpaths.difference(sq), unary_union(keep_s).difference(sq),
                                  self.connectors.difference(sq)] + out(self.mouth_bands)
                                 + [b.buffer(NAV_HULL, join_style="mitre") for b in out(bands)]).buffer(0.05)
            outc = polys(walk_o)
            otree = shapely.STRtree(outc)
            oareas = np.array([c.area for c in outc])
            ks = []
            for piece in keep_s:
                gone = []
                if piece.intersects(sq.exterior):
                    for part in polys(piece.difference(sq)):
                        idx = otree.query(part.representative_point(), predicate="within")
                        if (float(oareas[idx].max()) if len(idx) else part.area) < ISLAND_MIN:
                            gone.append(part)
                if gone:
                    dropped += len(gone)
                    ks += [q for q in polys(piece.difference(unary_union(gone).buffer(0.02))) if q.area > 0.01]
                else:
                    ks.append(piece)
            keep_s = ks
        self.rosoku = MultiPolygon(keep_s) if keep_s else Polygon()
        L.rosoku = self.rosoku
        walk = link()
        comps = polys(walk)
        self.components = comps
        sizes = sorted([c.area for c in comps], reverse=True)
        # every crossing touching a pavement / strip at both ends? / 横断歩道の両端
        walk_ends = unary_union([L.footpaths, self.rosoku])
        bad_ends = 0
        for cr in crossings:
            c, n = np.array(cr["c"]), np.array(cr["n"])
            for t in (cr["t0"], cr["t1"]):
                q = Point(*(c + n * t))
                if walk_ends.distance(q) > KERB_W + 0.5:
                    bad_ends += 1
        # tactile zones on MAIN pavements (not the ring) / 点字ブロック（幹線のみ）
        main_lanes = [lane_poly(l) for l in L.lanes if l["junction"] < 0 and len(l["pts"]) >= 2
                      and self.C.cls.get((l["road"], l["section"])) == MAIN]
        main_zone = unary_union(main_lanes).buffer(GUTTER + KERB_W + PAVE_W[MAIN] + 0.8, quad_segs=4) \
            if main_lanes else Polygon()
        main_pave = self.pave_band.intersection(main_zone) if not main_zone.is_empty else Polygon()
        guide = Polygon()
        if not main_pave.is_empty:
            ring = cw.buffer(GUIDE_OFF + GUIDE_W / 2, quad_segs=3).difference(cw.buffer(GUIDE_OFF - GUIDE_W / 2, quad_segs=3))
            guide = clean(ring.intersection(main_pave), 0.2)
        warn = []
        for cr in crossings:
            c, n, f, w = np.array(cr["c"]), np.array(cr["n"]), np.array(cr["f"]), cr["width"]
            for t, sgn in ((cr["t0"], -1.0), (cr["t1"], 1.0)):
                a0 = t + sgn * (KERB_W + WARN_BACK)
                a1 = a0 + sgn * WARN_D
                corners = [c + n * a0 - f * w / 2, c + n * a0 + f * w / 2, c + n * a1 + f * w / 2, c + n * a1 - f * w / 2]
                warn.append(Polygon([tuple(p) for p in corners]))
        warn = clean(unary_union(warn).intersection(L.footpaths), 0.05) if warn else Polygon()
        self.guide, self.warn = guide.difference(warn), warn
        # street-tree pits in MAIN pavements: a kerb-side row every PIT_EVERY m, clear
        # of crossings, aprons, drives and the guide strip / 幹線歩道の植樹桝
        pits = []
        if not main_pave.is_empty:
            avoid = unary_union(bands + self.mouth_bands + [L.driveways, self.warn]).buffer(2.5)
            ring = cw.buffer(PIT_OFF, quad_segs=3).boundary.intersection(main_zone)
            for g in getattr(ring, "geoms", [ring]):
                if g.geom_type != "LineString" or g.length < PIT_EVERY:
                    continue
                s = PIT_EVERY / 2
                while s < g.length - 2.0:
                    p = g.interpolate(s)
                    pit = p.buffer(PIT / 2, cap_style="square")
                    if main_pave.contains(pit) and not avoid.intersects(pit) and not guide.intersects(pit.buffer(0.05)):
                        pits.append(pit)
                    s += PIT_EVERY
        self.pits = unary_union(pits) if pits else Polygon()
        L.tree_pits = self.pits
        L.pave_guide, L.pave_warn = self.guide, self.warn
        # navmesh areas / ナビメッシュ用の面
        self.nav = {"crosswalk": [b.buffer(NAV_HULL, join_style="mitre").wkt for b in bands],
                    "sidewalk": [g.wkt for g in polys(unary_union(self.mouth_bands + [self.connectors]))]}
        n_small = sum(1 for a in sizes if a < ISLAND_MIN)
        return {"components": len(comps), "largest_m2": [int(a) for a in sizes[:8]], "under_50m2": n_small,
                "islands_dropped": dropped, "islands_dropped_m2": int(dropped_area), "crossing_ends_off_pavement": bad_ends,
                "pavement_m2": int(self.pave_band.area), "rosoku_m2": int(self.rosoku.area),
                "mouth_zebras": self.mouth_zebra_count, "mouth_bands": len(self.mouth_bands),
                "guide_m2": int(self.guide.area), "warn_m2": int(self.warn.area), "tree_pits": len(pits)}

    def write_nav(self, out):
        """nav_areas.json: crosswalk bands, sidewalk aprons and the lawns that stay
        grass in the navmesh (OSM park / garden / playground; every other grass is
        block, so walkers keep to the network). / ナビメッシュ用の面。"""
        L = self.L
        lawn = unary_union([p for p, t in L.osm.areas(lambda t: t.get("leisure") in ("park", "garden", "playground"))])
        lawn = clean(lawn.intersection(L.bounds), 4.0) if not lawn.is_empty else Polygon()
        self.nav["lawns"] = [p.wkt for p in polys(lawn)]
        json.dump(self.nav, open(os.path.join(out, "nav_areas.json"), "w"))
