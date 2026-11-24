"""The carriageway cut into surface materials, Singapore roads (maps.json "sg_lta";
maps/.venv; guide ch. 51). Same method as surfaces_jp (pieces that share every
vertex, no coplanar overlays), Singapore's classes (markings_sg):

  asphalt_sg_patch   repairs every ~140 m of lane (~300 m on expressways),
                     trench reinstatements on 12 % of estate lanes
  asphalt_sg_xwheel  expressway wheel paths (0.7 m wide, 0.85 m off the centre)
  asphalt_sg_wheel   main-road wheel paths
  asphalt_sg_crack   the outer 0.6 m of estate roads
  asphalt_sg_estate  estate and service roads and their junctions
  asphalt_sg_expwy   expressways and their junctions
  asphalt_sg         everything else (main roads, slip roads, bus bays)

Deterministic (seeded). The wheel-path strips are simplified (5 cm) to keep the
triangle count down on a 24 km2 map.
車道を材質ごとに分割（シンガポール）。
"""

import collections
import math

import numpy as np
import shapely
from shapely.affinity import rotate, translate
from shapely.geometry import LineString, Polygon, box
from shapely.ops import substring, unary_union

from markings import offset, polys
from markings_sg import ESTATE, EXPWY, LINK, MAIN, NARROW, lane_poly
from surfaces_jp import G, coverage

WHEEL_OFF, WHEEL_W = 0.85, 0.7
EDGE_CRACK = 0.6
PATCH_EVERY = {EXPWY: 300.0, MAIN: 140.0, LINK: 140.0, ESTATE: 120.0, NARROW: 160.0}
ORDER = ("asphalt_sg_patch", "asphalt_sg_xwheel", "asphalt_sg_wheel", "asphalt_sg_crack", "asphalt_sg_estate",
         "asphalt_sg_expwy")


def partition(L, M, seed=9):
    """{material: array of faces} covering L.carriageway exactly once; M is the
    ground MarkingsSG (road classes). / 車道を覆う材質ごとの面。"""
    from layout import GUTTER
    rng = np.random.default_rng(seed)
    cw = L.carriageway
    cls_of = lambda l: M.cls.get((l["road"], l["section"]), M.road_cls.get(l["road"], ESTATE))
    jcls = collections.defaultdict(int)
    for a in L.approaches:
        jcls[a["junction"]] = max(jcls[a["junction"]], M.road_cls.get(a["road"], ESTATE))
    zone = {EXPWY: [], "big": []}
    wheel, xwheel, patches = [], [], []
    for l in L.lanes:
        if len(l["pts"]) < 2:
            continue
        c = jcls.get(l["junction"], ESTATE) if l["junction"] >= 0 else cls_of(l)
        lp = lane_poly(l)
        if c == EXPWY:
            zone[EXPWY].append(lp)
        elif c in (MAIN, LINK):
            zone["big"].append(lp)
        if l["junction"] >= 0:
            continue
        cl = LineString([q["c"] for q in l["pts"]])
        if cl.length < 4.0:
            continue
        if c in (EXPWY, MAIN):
            for d in (-WHEEL_OFF, WHEEL_OFF):
                o = offset(cl, d)
                if o is not None and o.length > 2.0:
                    (xwheel if c == EXPWY else wheel).append(
                        o.simplify(0.05).buffer(WHEEL_W / 2, cap_style="flat").intersection(lp))
        every = PATCH_EVERY.get(c, 140.0)
        s = rng.uniform(5.0, every)
        while s < cl.length - 3.0:
            p = cl.interpolate(s)
            a, b = cl.interpolate(max(0.0, s - 1.0)), cl.interpolate(min(cl.length, s + 1.0))
            yaw = math.degrees(math.atan2(b.y - a.y, b.x - a.x))
            w = l["pts"][0]["w"]
            lng, wid = rng.uniform(1.2, 4.5), rng.uniform(0.9, min(2.8, w))
            lat = rng.uniform(-(w - wid) / 2, (w - wid) / 2)
            patches.append(translate(rotate(box(-lng / 2, lat - wid / 2, lng / 2, lat + wid / 2),
                                            yaw + rng.uniform(-3, 3), origin=(0, 0)), p.x, p.y))
            s += rng.exponential(every) + 6.0
        if c in (ESTATE, NARROW) and cl.length > 30.0 and rng.random() < 0.12:
            s0 = rng.uniform(0.0, cl.length - 25.0)
            s1 = min(cl.length, s0 + rng.uniform(15.0, 45.0))
            piece = offset(substring(cl, s0, s1), rng.choice([-1, 1]) * l["pts"][0]["w"] * 0.25)
            if piece is not None:
                patches.append(piece.buffer(0.35, cap_style="flat"))
    for bay in getattr(L, "bus_bays", []):
        if "poly" in bay:
            zone["big"].append(bay["poly"])
    prec = lambda g: shapely.set_precision(unary_union(polys(g)), G)
    clean = lambda g: shapely.set_precision(unary_union([q for q in polys(g) if q.area > 0.01]), G)
    P = clean(prec(prec(unary_union(patches)).intersection(cw, grid_size=G))) if patches else Polygon()
    XW = clean(prec(unary_union([q for g in xwheel for q in polys(g)])).intersection(cw, grid_size=G)
               .difference(P, grid_size=G)) if xwheel else Polygon()
    Wh = clean(prec(unary_union([q for g in wheel for q in polys(g)])).intersection(cw, grid_size=G)
               .difference(unary_union([P, XW]), grid_size=G)) if wheel else Polygon()
    ex = prec(unary_union(zone[EXPWY]).buffer(GUTTER + 0.6)) if zone[EXPWY] else Polygon()
    big = prec(unary_union(zone["big"]).buffer(GUTTER + 0.6)) if zone["big"] else Polygon()
    estate = prec(cw.difference(unary_union([ex, big]), grid_size=G))
    edge = prec(cw.difference(cw.buffer(-EDGE_CRACK), grid_size=G))
    taken = unary_union([P, XW, Wh])
    C = clean(prec(estate.intersection(edge, grid_size=G)).difference(taken, grid_size=G))
    E = clean(estate.difference(unary_union([taken, C]), grid_size=G))
    X = clean(prec(ex.intersection(cw, grid_size=G)).difference(unary_union([taken, C, E, big]), grid_size=G)) \
        if not ex.is_empty else Polygon()
    return coverage(cw, {"asphalt_sg_patch": P, "asphalt_sg_xwheel": XW, "asphalt_sg_wheel": Wh,
                         "asphalt_sg_crack": C, "asphalt_sg_estate": E, "asphalt_sg_expwy": X},
                    order=ORDER, rest="asphalt_sg")
