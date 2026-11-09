"""The carriageway cut into surface materials, Japanese roads (maps/.venv).

The road surface is one draped plane; giving it several looks without any
coplanar overlay (that would z-fight) means cutting it into pieces that
share their edges exactly (all on a 1 mm grid):

  asphalt_patch   repairs: rectangles every ~110 m of lane, and utility
                  trench reinstatements (a strip along an estate street and
                  a cut across it)
  asphalt_wheel   the two wheel paths of every MAIN / TERT lane (0.7 m wide,
                  0.85 m either side of the lane centre)
  asphalt_crack   the outer 0.7 m of estate streets (edge cracking)
  asphalt_estate  the rest of the estate streets (MINOR, NARROW and their
                  junctions)
  asphalt_jp      everything else (main roads, the ring, their junctions)

Deterministic (seeded), so a rebuild gives the same patches.
車道を材質ごとに分割（重ねず、辺を共有）。補修・轍・ひび割れ・生活道路。
"""

import collections
import math

import numpy as np
import shapely
from shapely.affinity import rotate, translate
from shapely.geometry import LineString, Polygon, box
from shapely.ops import unary_union

from markings import offset, polys
from markings_jp import MAIN, TERT

G = 0.001                # common precision grid / 共通の精度格子
WHEEL_OFF, WHEEL_W = 0.85, 0.7
EDGE_CRACK = 0.7
PATCH_EVERY = 110.0


def lane_poly(l):
    return Polygon([q["l"] for q in l["pts"]] + [q["r"] for q in l["pts"]][::-1]).buffer(0)


def partition(L, M, seed=5):
    """{material: array of polygons} covering L.carriageway exactly once. M is the
    MarkingsJP (road classes). / 車道を覆う材質ごとの形状。"""
    from layout import GUTTER
    rng = np.random.default_rng(seed)
    cw = L.carriageway
    cls_of = lambda l: M.cls.get((l["road"], l["section"]), 2)
    # junction class: the highest road approaching it / 交差点の格は流入路の最高
    jcls = collections.defaultdict(int)
    for a in L.approaches:
        jcls[a["junction"]] = max(jcls[a["junction"]], M.road_cls.get(a["road"], 2))
    main_parts, wheel, patches = [], [], []
    for l in L.lanes:
        if len(l["pts"]) < 2:
            continue
        big = (l["junction"] >= 0 and jcls.get(l["junction"], 0) >= TERT) or \
              (l["junction"] < 0 and cls_of(l) >= TERT)
        if big:
            main_parts.append(lane_poly(l))
        if l["junction"] >= 0:
            continue
        c = LineString([q["c"] for q in l["pts"]])
        if c.length < 4.0:
            continue
        lp = lane_poly(l)
        if cls_of(l) >= TERT:
            for d in (-WHEEL_OFF, WHEEL_OFF):
                o = offset(c, d)
                if o is not None and o.length > 2.0:
                    wheel.append(o.buffer(WHEEL_W / 2, cap_style="flat").intersection(lp))
        # repairs along the lane / 補修跡
        s = rng.uniform(5.0, PATCH_EVERY)
        while s < c.length - 3.0:
            p = c.interpolate(s)
            a, b = c.interpolate(max(0.0, s - 1.0)), c.interpolate(min(c.length, s + 1.0))
            yaw = math.degrees(math.atan2(b.y - a.y, b.x - a.x))
            w = l["pts"][0]["w"]
            lng, wid = rng.uniform(1.2, 4.0), rng.uniform(0.9, min(2.6, w))
            lat = rng.uniform(-(w - wid) / 2, (w - wid) / 2)
            r = box(-lng / 2, lat - wid / 2, lng / 2, lat + wid / 2)
            patches.append(translate(rotate(r, yaw + rng.uniform(-4, 4), origin=(0, 0)), p.x, p.y))
            s += rng.exponential(PATCH_EVERY) + 6.0
        # utility trench on some estate streets: along, then across / 生活道路の埋設管工事跡
        if cls_of(l) < TERT and c.length > 30.0 and rng.random() < 0.15:
            s0 = rng.uniform(0.0, c.length - 25.0)
            s1 = min(c.length, s0 + rng.uniform(15.0, 45.0))
            from shapely.ops import substring
            piece = offset(substring(c, s0, s1), rng.choice([-1, 1]) * l["pts"][0]["w"] * 0.25)
            if piece is not None:
                patches.append(piece.buffer(0.35, cap_style="flat"))
                q = c.interpolate(s1)
                a, b = c.interpolate(max(0.0, s1 - 1.0)), c.interpolate(s1)
                yaw = math.degrees(math.atan2(b.y - a.y, b.x - a.x))
                patches.append(translate(rotate(box(-0.3, -6.0, 0.3, 6.0), yaw, origin=(0, 0)), q.x, q.y))
    for bay in getattr(L, "bus_bays", []):
        if "poly" in bay:
            main_parts.append(bay["poly"])
    prec = lambda g: shapely.set_precision(unary_union(polys(g)), G)
    P = prec(prec(unary_union(patches)).intersection(cw, grid_size=G)) if patches else Polygon()
    Wh = prec(prec(unary_union([q for g in wheel for q in polys(g)])).intersection(cw, grid_size=G)
              .difference(P, grid_size=G)) if wheel else Polygon()
    main_zone = prec(unary_union(main_parts).buffer(GUTTER + 0.6))
    estate = prec(cw.difference(main_zone, grid_size=G))
    edge = prec(cw.difference(cw.buffer(-EDGE_CRACK), grid_size=G))
    # tiny slivers go to the plain asphalt, which takes whatever is left, so
    # the pieces cover the carriageway with no gap / 小片は通常舗装へ（隙間なし）
    clean = lambda g: shapely.set_precision(unary_union([q for q in polys(g) if q.area > 0.01]), G)
    P, Wh = clean(P), clean(Wh)
    C = clean(prec(estate.intersection(edge, grid_size=G)).difference(unary_union([P, Wh]), grid_size=G))
    E = clean(estate.difference(unary_union([P, Wh, C]), grid_size=G))
    A = prec(cw.difference(unary_union([P, Wh, C, E]), grid_size=G))
    # 路側帯 (pavements.py): the estate streets' edge strip, cut out as its own
    # piece so ground.py can put it in the Road_Sidewalk object at road height
    # (same asphalt look; the navmesh makes it walkable) / 路側帯の面を別に切る
    R = clean(prec(getattr(L, "rosoku", Polygon()).intersection(cw, grid_size=G))) \
        if getattr(L, "rosoku", None) is not None and not L.rosoku.is_empty else Polygon()
    return coverage(cw, {"asphalt_jp": A, "asphalt_estate": E, "asphalt_crack": C, "asphalt_wheel": Wh,
                         "asphalt_patch": P, "asphalt_rosoku": R})


def coverage(cw, pieces, order=("asphalt_rosoku", "asphalt_patch", "asphalt_wheel", "asphalt_crack", "asphalt_estate"),
             rest="asphalt_jp"):
    """Re-cut the pieces from the noded linework of all their outlines, so
    neighbours share every vertex. Overlays alone leave T-junctions (a vertex
    of one piece on the middle of another's edge): draped, the edge bends at
    the vertex on one side only and the road cracked open by up to 5 cm.
    全ての輪郭をノード化して面を作り直す（T 字接続による路面の割れを防ぐ）。"""
    lines = [cw.boundary] + [g.boundary for g in pieces.values() if not g.is_empty]
    noded = unary_union(lines)
    faces = shapely.get_parts(shapely.polygonize(shapely.get_parts(noded)))
    faces = faces[shapely.area(faces) > 1e-6]
    rp = shapely.point_on_surface(faces)
    shapely.prepare(cw)
    inside = shapely.contains(cw, rp)
    faces, rp = faces[inside], rp[inside]
    out = {}
    taken = np.zeros(len(faces), bool)
    for mat in order:
        g = pieces.get(mat)
        if g is None or g.is_empty:
            out[mat] = faces[:0]
            continue
        shapely.prepare(g)
        m = ~taken & shapely.contains(g, rp)
        taken |= m
        out[mat] = faces[m]
    out[rest] = faces[~taken]
    return out          # {material: array of faces} (kept apart: they share edges) / 面の配列
