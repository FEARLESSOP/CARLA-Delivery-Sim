"""Tests for the Singapore building options (buildings.py, maps.json "building_detail" /
"buildings_geojson") and the low-poly tree crown (Mesh.blob, "tree_detail").

  maps/.venv/bin/python osm_city/test_buildings_sg.py

UE culls back faces, so every face must point out of the solid it bounds: each test
checks the winding normal of every triangle (guide ch. 5, 46). Also: roof boxes stay
inside the footprint, the merged walls cover the facade exactly once, and the options
switch nothing on by themselves.
シンガポールの建物オプションと低ポリゴン樹冠の試験（面の向き・屋上の箱・壁の重なり）。
"""

import os
import random
import sys

import numpy as np
from shapely.geometry import Point, Polygon, box
from shapely.geometry.polygon import orient

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import buildings  # noqa: E402
from mesh import Mesh  # noqa: E402


def tris(m, mats=None):
    """(material, centroid, winding normal, area) for every non-degenerate triangle."""
    out = []
    for mat, (V, N, UV, I) in m.arrays().items():
        if mats and mat not in mats:
            continue
        T = V[I].astype(np.float64)
        n = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0])
        ln = np.linalg.norm(n, axis=1)
        keep = ln > 1e-9
        out += [(mat, c, nn / l, l / 2) for c, nn, l in zip(T[keep].mean(axis=1), n[keep], ln[keep])]
    return out


def check(name, cond):
    print("%-62s %s" % (name, "ok" if cond else "FAIL"))
    assert cond, name


class _OSM(object):
    def areas(self, test):
        return []


class _Layout(object):
    """Just enough of layout.Layout for Builder: a road south of the block."""
    def __init__(self, road, others=()):
        from shapely.ops import unary_union
        self.osm = _OSM()
        self.meta = {"name": "NoSuchMapForTests"}
        self.carriageway = road
        self.road_all = road.buffer(0.5)
        self.building_area = unary_union(list(others)) if others else Polygon()


def builder(road=box(-200, -60, 200, -50), others=(), lod=True):
    L = _Layout(road, others)
    B = buildings.Builder(L, None)
    check("no maps.json entry: every option off", not B.lod and B.geo is None and B.FLOOR == buildings.FLOOR)
    if lod:
        import shapely
        B.lod, B._cw, B._bld = True, L.carriageway, L.building_area
        shapely.prepare(B._cw)
        shapely.prepare(B._bld)
        B.FLOOR, B.VOID, B.FLOOR_EXEC = 2.8, 3.2, 2.6
        B._floor = B.FLOOR
    return B


def test_blob():
    for seg, rings in ((6, 3), (7, 4)):
        m = Mesh()
        jit = random.Random(3)
        m.blob("leaves_dark", 1.0, 2.0, 5.0, 3.0, 2.5, 2.0, seg=seg, rings=rings,
               jitter=lambda j, k: 0.85 + 0.3 * jit.random())
        T = tris(m)
        n_all = sum(len(a[3]) for a in m.arrays().values())
        check("blob %dx%d: %d triangles, none degenerate" % (seg, rings, seg * (2 * rings - 2)),
              n_all == seg * (2 * rings - 2) == len(T))
        V = m.arrays()["leaves_dark"][0]
        seam = V[1:-1].reshape(rings - 1, seg + 1, 3)
        check("blob %dx%d: closed at the seam (no crack)" % (seg, rings), np.allclose(seam[:, 0], seam[:, -1]))
        ctr = np.array([1.0, 2.0, 5.0])
        check("blob %dx%d: every face points out" % (seg, rings),
              all(float(np.dot(n, c - ctr)) > 0 for _, c, n, _ in T))


def test_grid():
    for seen in (2, 0):
        m = Mesh()
        a, b = np.array([0.0, 0.0]), np.array([20.0, 0.0])
        f = buildings.Facade(m, a, b, np.array([0.0, -1.0]))       # outward = -y (CCW edge along +x)
        rows = [(3.2 + k * 2.6, 3.2 + (k + 1) * 2.6, [(u, u + 1.5, 3.2 + k * 2.6 + 0.9, 3.2 + k * 2.6 + 2.2)
                                                      for u in (2.0, 6.0, 10.0, 14.0)]) for k in range(5)]
        f.grid("paint_white", rows, seen, mullions=1)
        T = tris(m)
        wall = [(c, n, A) for mat, c, n, A in T if mat == "paint_white" and abs(c[1]) < 1e-6]
        glass = [(c, n, A) for mat, c, n, A in T if mat.startswith("glass")]
        check("grid seen=%s: wall faces in the facade plane point out" % seen, all(n[1] < -0.99 for c, n, A in wall))
        check("grid seen=%s: glass points out" % seen, all(n[1] < -0.99 for c, n, A in glass))
        # the wall plane is covered exactly once: wall + (flush) openings = the facade
        open_a = 4 * 5 * 1.5 * 1.3
        want = 20.0 * 13.0 - open_a
        check("grid seen=%s: wall area %.1f m2 = facade - openings" % (seen, want),
              abs(sum(A for c, n, A in wall) - want) < 1e-3)
        if not seen:
            check("grid unseen: flush glass fills the openings exactly", abs(sum(A for c, n, A in glass) - open_a) < 1e-3)
        else:
            rev = [(c, n) for mat, c, n, A in T if mat == "paint_white" and c[1] > 1e-6]
            # reveals: sides face into the opening, the sill bottom up, the head down
            ok = True
            for c, n in rev:
                u = c[0] % 4.0
                if abs(n[0]) > 0.99:
                    ok &= (n[0] > 0) == (u < 2.75)          # left jamb faces +u, right jamb -u
            check("grid seen: window reveals face into the opening", ok and len(rev) > 0)


def test_hdb_lod():
    poly = orient(box(0, 0, 60, 14), 1.0)
    for seen_road in (box(-200, -60, 200, -50), box(-500, -900, -490, -890)):
        B = builder(road=seen_road)
        m = Mesh()
        B.b_hdb(m, poly, {"building": "apartments", "building:levels": "12"}, 12, random.Random(1))
        T = tris(m)
        ctr = np.array([30.0, 7.0])
        on_plane = [(c, n) for mat, c, n, A in T if abs(n[2]) < 0.05 and
                    min(abs(c[0]), abs(c[0] - 60), abs(c[1]), abs(c[1] - 14)) < 1e-4 and c[2] > 3.25]
        check("hdb lod: %d facade-plane faces point away from the block" % len(on_plane),
              len(on_plane) > 0 and all(float(np.dot(n[:2], c[:2] - ctr)) > 0 for c, n in on_plane))
        z_top = 3.2 + 11 * 2.8
        check("hdb lod: roof at 3.2 + 11 x 2.8 m", any(abs(c[2] - z_top) < 1e-6 and n[2] > 0.99 for _, c, n, A in T))
        check("hdb lod: nothing beyond FACADE_DEPTH", all(-1.0 - 1e-6 <= c[0] <= 61.0 + 1e-6 and
                                                          -1.0 - 1e-6 <= c[1] <= 15.0 + 1e-6 for _, c, n, A in T))
    # an executive-only block (register flat types): its own 2.6 m storey / エグゼクティブ棟
    B = builder()
    m = Mesh()
    B.b_hdb(m, poly, {"building": "apartments", "@hdb": 1990, "@hdb_exec": "yes"}, 12, random.Random(1))
    check("hdb lod: executive block roof at 3.2 + 11 x 2.6 m",
          any(abs(c[2] - (3.2 + 11 * 2.6)) < 1e-6 and n[2] > 0.99 for _, c, n, A in tris(m)))
    check("hdb lod: executive height in roof_z", abs(B.roof_z("hdb", 12, {"@hdb_exec": "yes"}) - (3.2 + 11 * 2.6)) < 1e-9
          and abs(B.roof_z("hdb", 12) - (3.2 + 11 * 2.8)) < 1e-9)
    check("register street names: TAMPINES STREET 81 -> TAMPINES ST 81",
          buildings._street("Tampines Street 81") == "TAMPINES ST 81" and buildings._blk("618 B") == "618B")
    # the full-height corridor columns: every side points away from its column
    m = Mesh()
    B = builder()
    f = buildings.Facade(m, np.array([0.0, 0.0]), np.array([30.0, 0.0]), np.array([0.0, -1.0]))
    B.hdb_corridor_lod(f, 30.0, 3.2, 8, "paint_white", "paint_teal", "paint_navy", False)
    cols = [(c, n) for mat, c, n, A in tris(m, ["paint_white"])]
    check("corridor columns: faces point out of their column",
          all(abs(n[1]) > 0.99 and n[1] < 0 or abs(n[0]) > 0.99 for c, n in cols) and len(cols) > 0)
    ok = True
    for c, n in cols:
        if abs(n[0]) > 0.99:
            u0 = min(np.linspace(0.2, 29.8, max(2, int(30 // 6.0) + 1)), key=lambda u: abs(u - c[0]))
            ok &= (n[0] > 0) == (c[0] > u0)
    check("corridor columns: side faces point away from the column centre", ok)
    fronts = [(c, n) for c, n in cols if n[1] < -0.99]
    inner = [(c, n) for c, n in fronts if 0.5 < c[0] < 29.5]
    check("corridor columns: inner fronts behind the parapet's inner plane (w < -0.15)",
          len(inner) > 0 and all(c[1] > 0.15 for c, n in inner))
    check("corridor columns: end fronts 2 cm behind the facade (no slit by the next wall)",
          all(abs(c[1] - 0.02) < 1e-6 for c, n in fronts if not 0.5 < c[0] < 29.5))
    # no column face in the edge's end planes u = 0 / u = L (it lay in the next wall's
    # plane at a convex corner: a black strip in Cycles, z-fighting in UE; gate 2026-11-21)
    check("corridor columns: no end face at u = 0 or u = L",
          not any(abs(n[0]) > 0.99 and min(abs(c[0]), abs(c[0] - 30.0)) < 1e-6 for c, n in cols))
    floor = [c for mat, c, n, A in tris(m, ["concrete"]) if n[2] > 0.99]
    check("corridor: a floor on every storey (no view into the solid from above)",
          len({round(c[2], 3) for c in floor}) == 8 and all(0.15 <= c[1] <= 1.6 for c in floor))
    m = Mesh()
    f = buildings.Facade(m, np.array([0.0, 0.0]), np.array([30.0, 0.0]), np.array([0.0, -1.0]))
    B.hdb_corridor(f, 30.0, 3.2, 6.0, "paint_white", "paint_teal", "paint_navy", random.Random(1))
    T = tris(m)
    check("corridor (no lod): no end face at u = 0 or u = L",
          not any(abs(n[0]) > 0.99 and min(abs(c[0]), abs(c[0] - 30.0)) < 1e-6 for _, c, n, A in T))
    check("corridor (no lod): back wall down to the floor, floor drawn",
          any(mat == "corridor_dark" and abs(c[1] - 1.6) < 1e-6 and c[2] < 4.4 for mat, c, n, A in T) and
          any(mat == "concrete" and n[2] > 0.99 and abs(c[2] - 3.45) < 1e-6 for mat, c, n, A in T))


def test_roof_box():
    # a bent block: its centroid lies near the inner corner, an 8 x 5 m box there stuck out
    poly = orient(Polygon([(0, 0), (60, 0), (60, 12), (12, 12), (12, 40), (0, 40)]), 1.0)
    B = builder(lod=False)
    m = Mesh()
    c = poly.centroid
    at = B.roof_box(m, poly, "paint_white", c.x, c.y, 30.0, 33.6, 8.0, 5.0, 0.0)
    V = np.concatenate([a[0] for a in m.arrays().values()])
    check("roof box: placed", at is not None)
    check("roof box: every vertex inside the footprint",
          all(poly.buffer(1e-6).contains(Point(x, y)) for x, y, z in V))
    m = Mesh()
    check("roof box: left out when nothing fits", B.roof_box(m, orient(box(0, 0, 3, 3), 1.0), "paint_white",
                                                              1.5, 1.5, 0, 3, 8, 5, 0.0) is None and not m.arrays())


def test_party_and_house():
    me = orient(box(0, 0, 6, 15), 1.0)
    nb = orient(box(6.5, 0, 12.5, 15), 1.0)
    B = builder(others=[me, nb])
    m = Mesh()
    B.b_house(m, me, {"building": "terrace"}, 2, random.Random(2))
    T = tris(m)
    east = [(mat, c, n) for mat, c, n, A in T if abs(c[0] - 6.0) < 1e-6 and n[0] > 0.99]
    check("house lod: the party wall is plain (one material, no glass)",
          len(east) > 0 and {mat for mat, c, n in east} == {mat for mat, c, n in east if mat.startswith("paint")})
    side = [(c, n) for mat, c, n, A in T if abs(n[2]) < 0.05 and
            min(abs(c[0]), abs(c[0] - 6), abs(c[1]), abs(c[1] - 15)) < 1e-6 and c[2] < 6.5]
    check("house lod: wall faces point out", all(float(np.dot(n[:2], c[:2] - np.array([3.0, 7.5]))) > 0
                                                 for c, n in side))


def test_style_tags():
    t = {"building": "apartments"}
    check("style: an HDB block (register) is hdb", buildings.style_of(dict(t, **{"@hdb": 1990}), None, 12) == "hdb")
    check("style: a block named '... Residences' in the register stays hdb",
          buildings.style_of(dict(t, name="The Residences", **{"@hdb": 1990}), None, 12) == "hdb")
    check("style: an addressed private block is condo",
          buildings.style_of(dict(t, **{"@private": "yes", "addr:housenumber": "5"}), None, 12) == "condo")
    check("style: an HDB car park is carpark",
          buildings.style_of({"building": "yes", "@hdb_carpark": "yes", "@hdb": 1990}, None, 5) == "carpark")
    check("style: without the markers nothing changes",
          buildings.style_of(t, None, 12) == "hdb" and buildings.style_of({"building": "terrace"}, None, 2) == "house")


def test_corridor_corner():
    """Two corridor edges at a convex corner: their floors and ceilings must not overlap
    (both drew the corner square: coplanar faces, black per storey; gate 2026-11-21).
    凸角で接する 2 つの廊下の床・天井が重ならないこと。"""
    from shapely.geometry import Polygon as Pg
    from shapely.ops import unary_union
    poly = orient(box(0, 0, 30, 14), 1.0)
    for lod in (True, False):
        B = builder(road=box(230, -230, 240, -220), lod=lod)      # road to the SE: diagonal front / 斜めの正面
        if not lod:
            B.FLOOR, B.VOID = 2.8, 3.2
        m = Mesh()
        B.b_hdb(m, poly, {"building": "apartments", "building:levels": "6"}, 6, random.Random(1))
        V, N, UV, I = m.arrays()["concrete"]
        T = V[I].astype(np.float64)
        nz = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0])[:, 2]
        planes = {}
        for t, z in zip(T, nz):
            if abs(t[0, 2] - t[1, 2]) < 1e-6 and abs(t[0, 2] - t[2, 2]) < 1e-6 and 3.3 < t[0, 2] < 17.0:
                planes.setdefault((round(t[0, 2], 3), z > 0), []).append(Pg(t[:, :2]))
        worst = 0.0
        for k, ps in planes.items():
            worst = max(worst, sum(p.area for p in ps) - unary_union(ps).area)
        check("corridor corner (%s): floors / ceilings of the two edges do not overlap (%.3f m2)"
              % ("lod" if lod else "no lod", worst), len(planes) >= 8 and worst < 1e-3)


def test_party_eaves():
    """Terraced houses side by side: the roof overhang stops at the shared wall (both eaves ran
    into the neighbour's: coplanar soffits, flicker), the free sides keep it.
    隣家と接する壁では軒を出さない。"""
    A, Bx = orient(box(0, 0, 6, 12), 1.0), orient(box(6, 0, 12, 12), 1.0)
    for name in ("gable_roof", "hip_roof"):
        free = 0.6 if name == "gable_roof" else 0.6 * 3 / 45 ** 0.5      # hip corners grow diagonally / 寄棟は対角方向
        for others, xmax in (((A, Bx), 6.0), ((A,), 6.0 + free)):
            B = builder(road=box(-200, -60, 200, -50), others=others, lod=False)
            m = Mesh()
            if name == "gable_roof":
                B.gable_roof(m, A, 6.55, 1.8, "roof_terracotta", "paint_white", overhang=0.6)
            else:
                B.hip_roof(m, A, 6.55, 1.8, "roof_terracotta", overhang=0.6)
            V = np.concatenate([a[0] for a in m.arrays().values()]).astype(np.float64)
            check("%s, %s: eaves to x = %.2f (want %.2f), free sides overhang" % (
                name, "neighbour" if len(others) > 1 else "alone", V[:, 0].max(), xmax),
                abs(V[:, 0].max() - xmax) < 0.02 and V[:, 0].min() < -free + 0.02 and V[:, 1].min() < -0.2)


def test_palette():
    """colour_notes.json "palette_sg" (General Painter): opt-in by data. / 観測色パレット。"""
    import tempfile, json, os
    import textures
    B = builder()
    poly = orient(box(0, 0, 60, 14), 1.0)
    check("palette off: base_of is the paint, kind_paint the default",
          B.base_of("paint_white") == "paint_white" and B.kind_paint("house", "walls", poly, "x") == "x")
    with tempfile.TemporaryDirectory() as d:
        check("palette off: write_palette writes nothing", textures.write_palette(d, d) == {} and os.listdir(d) == [])
        json.dump({"palette_sg": {"sg_test": [200, 100, 80]}}, open(os.path.join(d, "colour_notes.json"), "w"))
        m = textures.write_palette(d, d)
        files = sorted(os.listdir(d))
        check("palette: paint + void-deck base, each with normal / roughness / metallic maps",
              sorted(m) == ["sg_test", "sg_test_base"] and all(
                  k + x in files for k in m for x in (".png", "_n.png", "_r.png", "_m.png")))
        from PIL import Image
        a = np.asarray(Image.open(os.path.join(d, "sg_test_base.png")), np.float64)
        check("palette: the base band is darker at the bottom row (v = 0) than at the top",
              a[-8:].mean() < a[:8].mean() - 3)
        check("palette: tiles seamlessly (edge rows / columns match their neighbours)",
              np.abs(a[:, 0] - a[:, -1]).mean() < 6 and np.abs(np.diff(a, axis=1)).mean() < 6)

    class _S(object):
        notes = {"palette_sg": {"sg_white": [238, 237, 232]},
                 "kinds_sg": {"house": {"walls": ["sg_white"]}}}
        obs = []

        def colours(self, poly, style, max_dist=None):
            return {"wall": "sg_white", "acc": "sg_blue", "acc2": "sg_white", "alt": [["sg_white", "sg_maroon", "sg_white"]]}
    L = _Layout(box(-200, -60, 200, -50))
    B = buildings.Builder(L, _S())
    check("palette on: void deck paint has its base, kinds pick from the notes",
          B.base_of("sg_white") == "sg_white_base" and B.kind_paint("house", "walls", poly, "x") == "sg_white")
    accs = {B.theme(orient(box(x, 0, x + 30, 14), 1.0))[1] for x in range(0, 3000, 37)}
    check("palette on: seeded theme variants inside a precinct", accs == {"sg_blue", "sg_maroon"})
    m = Mesh()
    B.b_hdb(m, poly, {"building": "apartments", "building:levels": "12"}, 12, random.Random(1))
    check("palette on: void deck walls / columns use the base paint", "sg_white_base" in m.arrays())


if __name__ == "__main__":
    test_blob()
    test_grid()
    test_hdb_lod()
    test_roof_box()
    test_party_and_house()
    test_style_tags()
    test_corridor_corner()
    test_party_eaves()
    test_palette()
    print("all ok")
