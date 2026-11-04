"""Face-direction tests for the Akeno roadside primitives (UE culls back
faces; Blender renders hide that): the multi-span greenhouse, the roof solar
array (buildings_jp.roof_solar) and the closed add_ways of osmfix. Run:
maps/.venv/bin/python osm_city/landmarks/test_akeno.py

明野の部品（連棟ハウス・屋上太陽光パネル・閉じた追加ウェイ）の面の向きの試験。
"""
import math
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from landmarks import akeno  # noqa: E402
from mesh import Mesh  # noqa: E402
import buildings_jp as J  # noqa: E402


def tris(m):
    out = {}
    for mat, (V, N, UV, I) in m.arrays().items():
        T = V[I].astype(np.float64)
        n = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0])
        n /= np.linalg.norm(n, axis=1)[:, None]
        out[mat] = (T, n, N[I[:, 0]].astype(np.float64))
    return out


def test_greenhouse_faces_out():
    """Every face of a closed greenhouse points away from its inside (the
    centroid of the volume under it), winding and stored normal agree; the
    roof faces up; spans as asked. / 全ての面が外向き。"""
    for ang in (0.0, 0.7, math.pi / 2, 2.4):
        m = Mesh()
        ax = (math.cos(ang), math.sin(ang))
        n = akeno.greenhouse(m, (10.0, -5.0), ax, 40.0, 24.0, eave=3.2, rise=1.1, span=4.0)
        assert n == 6, n
        T = tris(m)
        c = np.array([10.0, -5.0]) + np.array(ax) * 20.0 + np.array([-ax[1], ax[0]]) * 12.0
        for mat, (P, nw, ns) in T.items():
            assert (np.einsum("ij,ij->i", nw, ns) > 0.99).all(), mat
            cen = P.mean(axis=1)
            d = cen - np.array([c[0], c[1], 1.6])
            if mat == "akeno_gh_roof":
                assert (nw[:, 2] > 0.5).all(), "roof faces down"
                # each slope leans away from its own ridge (out of the volume) / 斜面は外へ
                continue
            assert (np.einsum("ij,ij->i", nw, d) > 0).all(), "%s: a face points inwards" % mat
        # roof slopes: the low edge of each slope lies on the side its normal leans to
        # 斜面の法線は低い側へ傾く
        P, nw, _ = T["akeno_gh_roof"]
        for k in range(len(P)):
            lo = P[k][np.argmin(P[k][:, 2])]
            hi = P[k][np.argmax(P[k][:, 2])]
            assert float(nw[k][:2] @ (lo - hi)[:2]) >= -1e-9, "slope leans the wrong way"


def test_greenhouse_closed():
    """No gap at the roof line: the gable triangles and the slopes meet (area
    of roof = n * 2 * slope length * A). / 屋根面積の確認。"""
    m = Mesh()
    akeno.greenhouse(m, (0.0, 0.0), (1.0, 0.0), 30.0, 12.0, eave=3.0, rise=1.0, span=4.0)
    P, nw, _ = tris(m)["akeno_gh_roof"]
    area = 0.5 * np.linalg.norm(np.cross(P[:, 1] - P[:, 0], P[:, 2] - P[:, 0]), axis=1).sum()
    want = 3 * 2 * math.hypot(2.0, 1.0) * 30.0
    assert abs(area - want) < 1e-6, (area, want)


def test_roof_solar_faces():
    """Panels on the south half facing up and south, their frames down/north;
    "west" puts them on the west end. / パネルは上・南向き、西端。"""
    for ax in ((1.0, 0.0), (-1.0, 0.0)):
        o = (0.0, 0.0) if ax[0] > 0 else (50.0, 12.0)
        R = J.Rect(o, ax, 50.0, 12.0)
        m = Mesh()
        n = J.roof_solar(m, R, 25.0, "west")
        assert n > 0
        T = tris(m)
        P, nw, ns = T["solar"]
        assert (np.einsum("ij,ij->i", nw, ns) > 0.99).all()
        assert (nw[:, 2] > 0.9).all() and (nw[:, 1] < 0).all(), "panels must face up and south"
        cen = P.mean(axis=1)
        assert cen[:, 1].max() < 6.0 and cen[:, 0].max() < 25.0, "south half, west end"
        F, fn, _ = T["jp_alu"]
        assert ((fn[:, 2] < -0.9) | (fn[:, 1] > 0.9)).all(), "frames face down or north"


def test_osmfix_closed():
    """add_ways with closed: the way ends on its first node (an area). / 閉じたウェイ。"""
    import osmfix
    root = ET.fromstring('<osm><node id="1" lat="33.2" lon="131.6"/><way id="5"><nd ref="1"/></way></osm>')
    meta = {"name": "__test__", "lat0": 33.2, "lon0": 131.6}
    old = osmfix.fixes_for
    osmfix.fixes_for = lambda m: {"add_ways": [{"id": "77", "closed": True, "pts": [[0, 0], [5, 0], [5, 5]],
                                                "tags": {"building": "yes"}}]}
    try:
        osmfix.apply(root, meta, quiet=True)
    finally:
        osmfix.fixes_for = old
    w = [w for w in root.iter("way") if w.get("id") == "77"][0]
    refs = [nd.get("ref") for nd in w.findall("nd")]
    assert refs[0] == refs[-1] and len(refs) == 4, refs


def _jp_item(style, poly, tags, levels=1):
    import random
    import test_buildings_jp as TB
    B, jp = TB.fake_jp()
    it = TB.item(jp, poly, style, poly.buffer(2.0, join_style="mitre"))
    it.t, it.levels = dict(tags), levels
    it.var = jp.pick(it, random.Random(3))
    for k in ("wall", "acc", "roof", "bal"):          # as JP.paint() applies the tags / タグの配色
        if it.t.get("jp:" + k):
            it.var[k] = it.t["jp:" + k]
    return jp, it


def test_flat_garage():
    """The flat-roofed garage row (roof:shape=flat, height, jp:doors=s): white
    walls look out, the roof up, shutters only on the south face. / 陸屋根の車庫棟。"""
    from shapely.geometry import box
    jp, it = _jp_item("shed", box(0, 0, 55, 4.8), {"roof:shape": "flat", "height": "2.8", "jp:wall": "jp_wall_white",
                                                   "jp:acc": "jp_wall_white", "jp:doors": "s"})
    m = Mesh()
    jp.b_shed(m, it, None)
    T = tris(m)
    c = np.array([27.5, 2.4])
    P, nw, _ = T["jp_wall_white"]
    vert = np.abs(nw[:, 2]) < 0.05
    d = P.mean(axis=1)[:, :2] - c
    assert (np.einsum("ij,ij->i", nw[vert][:, :2], d[vert]) > 0).all(), "a garage wall faces in"
    R, rn, _ = T["roof_concrete"]
    assert (rn[:, 2] > 0.99).all() and abs(R[:, :, 2].max() - (0.15 + 2.9)) < 1e-6
    S, sn, _ = T["jp_shutter"]
    assert (sn[:, 1] < -0.99).all() and len(S) >= 2 * 17, "shutters only on the south, one per 3 m"
    assert it.height > 2.9


def test_blank_gable():
    """jp:gable=blank: the mansion's end walls carry no windows and face out.
    窓の無い妻壁。"""
    from shapely.geometry import box
    jp, it = _jp_item("mansion", box(0, 0, 50, 12.8), {"jp:wall": "jp_concrete_white", "jp:acc": "jp_wall_grey",
                                                       "jp:bal": "solid", "jp:gable": "blank"}, levels=8)
    m = Mesh()
    jp.b_mansion(m, it, None)
    T = tris(m)
    for mat, (P, nw, _) in T.items():
        cen = P.mean(axis=1)
        ends = (cen[:, 0] < 0.02) | (cen[:, 0] > 49.98)
        if mat.startswith("glass") or "win" in mat or mat in ("jp_alu",):
            assert not ends.any(), "%s on an end wall" % mat
    P, nw, _ = T["jp_wall_grey"]
    cen = P.mean(axis=1)
    e = np.abs(nw[:, 0]) > 0.99
    assert e.any() and (np.sign(nw[e, 0]) == np.sign(cen[e, 0] - 25.0)).all(), "gable faces out"


def test_pitch_lines():
    g = akeno.pitch_lines((0.0, 0.0), math.pi / 2, 102.0, 61.0)
    x0, y0, x1, y1 = g.bounds
    assert abs((y1 - y0) - 102.12) < 0.05 and abs((x1 - x0) - 61.12) < 0.05, g.bounds


if __name__ == "__main__":
    for k, f in sorted(globals().items()):
        if k.startswith("test_"):
            f()
            print("ok", k)
