"""Offline tests for jartic.py, jp_signs.py and signal_phases.py (maps/.venv).

  maps/.venv/bin/python osm_city/test_jp_signs.py

JARTIC 規制データ・標識板・信号現示のオフライン試験。
"""

import json
import math
import os
import sys
import tempfile
import xml.etree.ElementTree as ET

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import jartic  # noqa: E402
import jp_signs  # noqa: E402
import signal_phases  # noqa: E402
from mesh import Mesh  # noqa: E402

META = {"name": "T", "lat0": 33.234355, "lon0": 131.651875, "half_x": 500.0, "half_y": 600.0}
HEAD = ["拡張版標準フォーマット種別・バージョン", "共通規制種別コード", "点・線・面コード", "県別規制種別名称",
        "ユニークキー", "規制場所の経度緯度", "進入方向(座標)", "指定する方向(座標)", "規制時間1_開始",
        "規制時間1_終了", "速度", "横断歩道設置本数", "信号の有無コード"]


def test_parse():
    """cp932 CSV, map-frame points, cut to the box, hours kept only when limited. / 読み込みと切り出し。"""
    fr = jartic.geo.Frame(META)
    lo, la = fr.lonlat(10.0, 20.0)
    far = fr.lonlat(5000.0, 0.0)
    rows = [HEAD,
            ["k_2.1", "63", "1", "一時停止", "K1", "%f %f" % (lo, la), "%f %f" % fr.lonlat(10.0, 30.0), "", "", "", "",
             "", ""],
            ["k_2.1", "12", "1", "指定方向外進行禁止", "K2", "%f %f" % (lo, la), "", "", "0730", "0810", "", "", ""],
            ["k_2.1", "63", "1", "一時停止", "K3", "%f %f" % far, "", "", "", "", "", "", ""],
            ["k_2.1", "85", "1", "横断歩道", "K4", "%f %f" % (lo, la), "", "", "0000", "2400", "", "3", "1"]]
    with tempfile.TemporaryDirectory() as d:
        p = os.path.join(d, "d.csv")
        with open(p, "w", encoding="cp932", newline="") as fh:
            import csv
            csv.writer(fh).writerows(rows)
        out = jartic.parse(p, META)
    assert [r["key"] for r in out] == ["K1", "K2", "K4"], out
    assert abs(out[0]["xy"][0][0] - 10.0) < 0.05 and abs(out[0]["xy"][0][1] - 20.0) < 0.05
    assert abs(out[0]["entry"][0][1] - 30.0) < 0.05
    assert out[1]["time"] == "0730-0810" and out[2]["time"] is None
    assert out[2]["n_crossings"] == 3 and out[2]["signal"] is True


def test_match_and_turns():
    """The approach whose lane passes the entry point toward the stop wins over
    the one ending nearer but heading elsewhere. / 進入点を通る進入路を選ぶ。"""
    lane_a = {"road": 1, "lane": -1, "pts": [{"c": [0.0, float(y)]} for y in range(40, 4, -1)]}   # southward to (0,5)
    lane_b = {"road": 2, "lane": -1, "pts": [{"c": [float(x), 1.0]} for x in range(-30, 0)]}     # eastward to (-1,1)
    appr = {(1, 9): [{"lane": -1, "end": [0.0, 5.0]}], (2, 9): [{"lane": -1, "end": [-1.0, 1.0]}]}
    lanes_by = {(1, -1): lane_a, (2, -1): lane_b}
    k, s = jartic.match_approach([0.0, 3.0], [0.0, 25.0], appr, lanes_by)
    assert k == (1, 9), (k, s)
    assert jartic.turn_of((0, 0), (0, -10), (0, 10)) == "straight"
    assert jartic.turn_of((0, 0), (0, -10), (-10, 0)) == "left"        # heading north, west is left / 北進で西は左
    assert jartic.turn_of((0, 0), (0, -10), (10, 0)) == "right"
    assert jartic.lanes_along([[0, 50], [0, -50]], [dict(lane_a, junction=-1)]) == [(1, -1)]


def test_plate_faces():
    """Every plate triangle faces where its normal says (UE culls back faces);
    the front faces nf, the back -nf. / 板の三角形の向きが法線と一致すること。"""
    for shape, w, h in (("disc", 0.6, 0.6), ("tri", 0.8, 0.69), ("rect", 0.8, 0.4), ("pent", 0.6, 0.6)):
        m = Mesh()
        nf = np.array([0.6, -0.8])
        jp_signs.plate_mesh(m, "jp_plate_x", "jp_plate_back", (1.0, 2.0, 2.2), nf, shape, w, h)
        for mat, (V, N, UV, I) in m.arrays().items():
            for t in I:
                a, b, c = V[t].astype(float)
                n = np.cross(b - a, c - a)
                if np.linalg.norm(n) < 1e-9:
                    continue
                assert float(n @ N[t[0]]) > 0, (shape, mat)
        V, N = m.arrays()["jp_plate_x"][:2]
        assert np.allclose(N[:, :2], nf, atol=1e-6)
        z = m.arrays()["jp_plate_x"][0][:, 2]
        assert abs(z.max() - z.min() - h) < 1e-4


def test_textures():
    for k in ("tomare", "noparking", "crossing", "bicycle", "oneway", "oneway_l", "aux_0730_0810", "noturn_sl",
              "noturn_lr", "noturn_r", "noentry", "back", "pole"):
        im = jp_signs.texture(k, 256)
        assert im.size == (128, 128) and im.mode == "RGB", k
        assert "sign" not in k and "light" not in k


def test_npa_designs():
    """Faces follow the NPA chart ("Traffic Lights and Road Signs"): 進入禁止 red
    with a white bar, 一方通行 arrow only (white centre row, blue top/bottom),
    横断歩道 a blue pentagon, standard sizes. / NPA の図柄・寸法。"""
    a = np.asarray(jp_signs.texture("noentry", 256), float)
    assert a[30, 64, 0] > 150 and a[30, 64, 2] < 80            # red field / 赤地
    assert a[64, 30].min() > 200 and a[64, 98].min() > 200     # white bar / 白い横棒
    o = np.asarray(jp_signs.texture("oneway", 256), float)
    assert o[64, 64].min() > 200 and o[25, 64, 2] > 120 and o[25, 64, 0] < 60   # arrow mid, blue above
    assert jp_signs.size_of("oneway")[1:] == (0.8, 0.4) and jp_signs.size_of("crossing")[0] == "pent"
    pts, uv = jp_signs.outline("pent", 0.6, 0.6)
    assert len(pts) == 6 and max(v for _, v in pts) == 0.3 and min(v for _, v in pts) == -0.3


def test_phase_split():
    """One junction controller with four lights (N/S and E/W approaches) becomes
    two phase controllers listed by the junction; rerunning changes nothing.
    4 方向 1 コントローラ -> 2 現示、再実行で不変。"""
    xodr = """<OpenDRIVE>
  <road id="1"><signals><signal id="a" dynamic="yes"/></signals></road>
  <road id="2"><signals><signal id="b" dynamic="yes"/></signals></road>
  <road id="3"><signals><signal id="c" dynamic="yes"/></signals></road>
  <road id="4"><signals><signal id="d" dynamic="yes"/></signals></road>
  <controller id="J"><control signalId="a"/><control signalId="b"/><control signalId="c"/><control signalId="d"/></controller>
  <controller id="J"><control signalId="a"/><control signalId="b"/><control signalId="c"/><control signalId="d"/></controller>
  <junction id="7"><controller id="J" type="0" sequence="0"/></junction>
</OpenDRIVE>"""
    lanes = {"approaches": [{"road": 1, "yaw": 90.0}, {"road": 2, "yaw": -88.0}, {"road": 3, "yaw": 2.0},
                            {"road": 4, "yaw": 178.0}], "lanes": []}
    with tempfile.TemporaryDirectory() as d:
        lp = os.path.join(d, "lanes.json")
        json.dump(lanes, open(lp, "w"))
        root = ET.fromstring(xodr)
        done = signal_phases.split(root, lp)
        assert sorted(sorted(g) for g in done["J"]) == [["a", "b"], ["c", "d"]], done
        ids = [c.get("id") for c in root.findall("controller")]
        assert ids == ["J_p1", "J_p2"], ids
        assert [c.get("id") for c in root.find("junction").findall("controller")] == ["J_p1", "J_p2"]
        again = signal_phases.split(root, lp)
        assert again == done and [c.get("id") for c in root.findall("controller")] == ids


def _line(p, q, n=8):
    return [{"c": [p[0] + (q[0] - p[0]) * k / n, p[1] + (q[1] - p[1]) * k / n]} for k in range(n + 1)]


def test_phase_conflicts():
    """Crossing paths never share a phase. A cross junction (N, S, E, W) plus a
    diagonal leg from the south-west whose straight crosses the N-S straights:
    by axis alone (35 deg) it would join N-S; by paths it must not.
    ``conflicts()`` must report the crossing pairs of a single all-lights
    controller and none after split(). 交差する経路は同じ現示にしない。"""
    xodr = ("<OpenDRIVE>" + "".join('<road id="%d"><signals><signal id="s%d" dynamic="yes"/></signals></road>' % (r, r)
                                    for r in (1, 2, 3, 4, 5)) +
            '<controller id="J">' + "".join('<control signalId="s%d"/>' % r for r in (1, 2, 3, 4, 5)) +
            '</controller><junction id="9"><controller id="J" type="0" sequence="0"/></junction></OpenDRIVE>')
    # approach lanes end at the junction edge; junction lanes are straight paths
    # (keep-left: northbound in x=-1.5, southbound x=+1.5 ...) / 進入路と交差点内の直進路
    appr = {1: ((-1.5, -30), (-1.5, -8), (-1.5, 8)),      # from S, northbound
            2: ((1.5, 30), (1.5, 8), (1.5, -8)),           # from N, southbound
            3: ((30, -1.5), (8, -1.5), (-8, -1.5)),        # from E, westbound
            4: ((-30, 1.5), (-8, 1.5), (8, 1.5)),          # from W, eastbound
            5: ((-25, -35), (-6, -10), (6, 8))}            # from SW, diagonal (~55 deg) / 斜めの脚
    lanes = []
    for r, (a, e, x) in appr.items():
        lanes.append({"road": r, "lane": -1, "junction": -1, "pts": _line(a, e)})
        lanes.append({"road": 100 + r, "lane": -1, "junction": 9, "pts": _line(e, x)})
    yaw = {r: math.degrees(math.atan2(e[1] - a[1], e[0] - a[0])) for r, (a, e, x) in appr.items()}
    data = {"approaches": [{"road": r, "yaw": y} for r, y in yaw.items()], "lanes": lanes}
    with tempfile.TemporaryDirectory() as d:
        lp = os.path.join(d, "lanes.json")
        json.dump(data, open(lp, "w"))
        root = ET.fromstring(xodr)
        before = signal_phases.conflicts(root, lp)
        assert ("J", "s1", "s3") in before and ("J", "s1", "s5") not in before or before, before
        assert any(set(b[1:]) == {"s2", "s5"} for b in before), before      # diagonal x southbound / 斜め×南行
        signal_phases.split(root, lp)
        after = signal_phases.conflicts(root, lp)
        assert after == [], after
        ph = {x.get("signalId"): c.get("id") for c in root.findall("controller") for x in c.findall("control")}
        assert ph["s1"] == ph["s2"] and ph["s3"] == ph["s4"] and ph["s1"] != ph["s3"], ph
        assert ph["s5"] not in (ph["s2"], ph["s3"]), ph


def test_stub_lights_removed():
    """A light on a stub road (< 3 m between two junctions of a joined cluster)
    stops cars inside the junction box: split() removes it. / 連結路の信号は削除。"""
    xodr = """<OpenDRIVE>
  <road id="1" length="40"><link><successor elementType="junction" elementId="7"/></link>
    <signals><signal id="a" dynamic="yes"/></signals></road>
  <road id="2" length="0.2"><link><predecessor elementType="junction" elementId="7"/>
    <successor elementType="junction" elementId="8"/></link><signals><signal id="b" dynamic="yes"/></signals></road>
  <controller id="J"><control signalId="a"/><control signalId="b"/></controller>
  <junction id="7"><controller id="J" type="0" sequence="0"/></junction>
</OpenDRIVE>"""
    with tempfile.TemporaryDirectory() as d:
        lp = os.path.join(d, "lanes.json")
        json.dump({"approaches": [{"road": 1, "yaw": 90.0}], "lanes": []}, open(lp, "w"))
        root = ET.fromstring(xodr)
        signal_phases.split(root, lp)
        assert [s.get("id") for s in root.iter("signal")] == ["a"]
        assert [x.get("signalId") for x in root.find("controller").findall("control")] == ["a"]


def test_verified_plates():
    """verified_plates.json: a 303 goes to the one-way road whose exit end is
    near its xy (if it ends at a junction), a two-way road or a far point is
    dropped, "none" entries are only logged. / 現地確認済み標識の道路照合。"""
    def lane(road, ln, pts):
        return {"road": road, "lane": ln, "junction": -1, "pts": [{"c": list(map(float, c))} for c in pts]}
    P = object.__new__(jp_signs.Placer)
    P.road_lanes = {1: [lane(1, -1, [(0, y) for y in range(0, 50, 5)])],                     # one-way north
                    2: [lane(2, -1, [(x, 100) for x in range(0, 60, 5)]),
                        lane(2, 1, [(x, 103) for x in range(55, -5, -5)])],                     # two-way
                    3: [lane(3, -1, [(200, y) for y in range(0, 50, 5)])]}                      # one-way, no junction
    P.appr = {(1, 9): [{"road": 1, "lane": -1}], (2, 9): [{"road": 2, "lane": -1}]}
    placed = []
    P.noentry_at = lambda road, ln, src, **kw: placed.append(("noentry", road, src)) or {"x": 0.0, "y": 45.0}
    P.oneway_at = lambda road, ln, src: placed.append(("oneway", road, src)) or {"x": 0.0, "y": 4.0}
    data = {"plates": [{"kind": "noentry", "end": "exit", "xy": [1.0, 47.0], "road": 1, "evidence": "a.jpg"},
                       {"kind": "oneway", "end": "entry", "xy": [0.5, -1.0], "evidence": "b.jpg"},
                       {"kind": "noentry", "end": "exit", "xy": [56.0, 100.0], "evidence": "c.jpg"},
                       {"kind": "noentry", "end": "exit", "xy": [200.0, 46.0], "evidence": "d.jpg"},
                       {"kind": "noentry", "end": "exit", "xy": [500.0, 500.0], "evidence": "e.jpg"}],
            "none": [{"kind": "oneway", "end": "entry", "xy": [0, 0], "seen": "nothing"}]}
    with tempfile.TemporaryDirectory() as d:
        P.out = d
        P.verified()
        assert P.verified_log == []                               # no file: nothing / ファイル無し
        json.dump(data, open(os.path.join(d, jp_signs.VERIFIED), "w"))
        P.verified()
    assert [(k, r) for k, r, _ in placed] == [("noentry", 1), ("oneway", 1)], placed
    res = [v["result"] for v in P.verified_log]
    assert res[0].startswith("placed") and res[1].startswith("placed"), res
    assert "two-way" in res[2] and "junction" in res[3] and "no road end" in res[4], res
    assert res[5].startswith("no plate") and len(res) == 6, res


def test_plate_clashes():
    """clashes(): a shed roof at plate height over the post is found, the same
    roof 1 m off and the ground under the post are not. / 構造物との干渉。"""
    P = object.__new__(jp_signs.Placer)
    P.ground_z = lambda x, y: 0.15
    roof = np.array([[0.2, -2.0, 2.2], [3.0, -2.0, 2.2], [3.0, 2.0, 2.2], [0.2, 2.0, 2.2]], np.float32)
    ground = np.array([[-5.0, -5.0, 0.1], [5.0, -5.0, 0.1], [5.0, 5.0, 0.1], [-5.0, 5.0, 0.1]], np.float32)
    with tempfile.TemporaryDirectory() as d:
        os.makedirs(os.path.join(d, "meshes"))
        I = np.array([0, 1, 2, 0, 2, 3], np.int32)
        np.savez(os.path.join(d, "meshes", "SM_X.npz"), **{"shed|V": roof, "shed|I": I, "gnd|V": ground, "gnd|I": I})
        np.savez(os.path.join(d, "meshes", "Terrain_JpPlates_T_0.npz"), **{"jp|V": roof, "jp|I": I})
        P.out = d
        assert P.clashes((0.0, 0.0), (0.0, 1.0), "noentry") == ["shed"]      # plate spans x -0.35..0.35
        assert P.clashes((-1.0, 0.0), (0.0, 1.0), "noentry") == []
        assert P.clashes((0.0, 0.0), (1.0, 0.0), "noentry") == []            # facing east: the envelope ends at x 0.1
        assert P.clashes((0.2, 0.0), (1.0, 0.0), "noentry") == ["shed"]


def test_plate_on_signal_pole():
    """Signal poles where CARLA spawns them (reference line at s, offset t, then
    0.25 m on along the offset line; positionInertial as is); a post within
    SIGNAL_CLEAR goes onto the pole, one farther off keeps its own.
    信号柱の位置（CARLA と同じ）と、近くの板の信号柱への取り付け。"""
    xodr = """<OpenDRIVE>
  <road id="1" length="40" junction="-1"><planView>
    <geometry s="0" x="0" y="0" hdg="0" length="20"><line/></geometry>
    <geometry s="20" x="20" y="0" hdg="0" length="20"><arc curvature="0.05"/></geometry></planView>
    <signals><signal id="tl1" s="10" t="-4" dynamic="yes"/><signal id="tl2" s="30" t="-4" dynamic="yes"/>
    <signal id="tl3" s="5" t="2" dynamic="yes"><positionInertial x="7" y="8" z="0" hdg="0"/></signal>
    </signals></road></OpenDRIVE>"""
    with tempfile.TemporaryDirectory() as d:
        open(os.path.join(d, "T.xodr"), "w").write(xodr)
        P = object.__new__(jp_signs.Placer)
        P.out, P.meta = d, {"name": "T"}
        poles = {i: (x, y) for i, x, y in P.signal_poles()}
    assert np.hypot(poles["tl1"][0] - 10.25, poles["tl1"][1] + 4.0) < 0.01, poles
    assert poles["tl3"] == (7.0, 8.0)
    # on the arc (radius 20, centre (20, 20)): offset radius 24, 0.25 m on = 0.25/24 rad more
    a = -math.pi / 2 + 10 * 0.05 + 0.25 / 24.0
    assert np.hypot(poles["tl2"][0] - (20 + 24 * math.cos(a)), poles["tl2"][1] - (20 + 24 * math.sin(a))) < 0.01, poles
    P.posts, P.merged, P.guard = [], [], None
    px, py = poles["tl1"]
    a = P.add("noentry", np.array([px + 0.3, py + 0.4]), (0.0, -1.0), "a", stack=False)
    assert a.get("signal") == "tl1" and (a["x"], a["y"]) == (px, py)
    b = P.add("noparking", np.array([px + 2.0, py]), (0.0, 1.0), "b")
    assert "signal" not in b and len(P.posts) == 2


def test_stop_landmark_at_line():
    """stop_offsets(): a Stencil_STOP 1-3 m before its stop line passes, one 9 m
    upstream fails (TM and drivers would stop early); lanes running against s
    count from the other end. STOP_STEPS keeps posts within STOP_TOL.
    Stencil_STOP は停止線から STOP_TOL 以内。"""
    xodr = """<OpenDRIVE>
  <road id="1" length="100"><signals><signal s="92.43" t="1.0" name="Stencil_STOP" type="206"/></signals></road>
  <road id="2" length="100"><signals><signal s="84.43" t="-3.8" name="Stencil_STOP" type="206"/></signals></road>
  <road id="3" length="100"><signals><signal s="6.0" t="1.0" name="Stencil_STOP" type="206"/></signals></road>
</OpenDRIVE>"""
    off = {r: d for r, _, _, d in jp_signs.stop_offsets(xodr, [(1, 0.0, 94.0, 0.5), (2, 0.0, 94.0, 0.5),
                                                                (3, 100.0, 2.0, 1.0)])}
    ok = lambda d: -0.5 <= d <= jp_signs.STOP_TOL                                   # noqa: E731
    assert ok(off[1]) and not ok(off[2]) and ok(off[3]), off
    assert 1.0 + (jp_signs.STOP_STEPS - 1) * 1.0 <= jp_signs.STOP_TOL               # 1 m before the line + slides


def _core_placer(lanes, cw, asphalt, buildings=None):
    """A Placer with only what settle_core() reads. / settle_core 用の最小構成"""
    from shapely.geometry import Polygon
    from shapely.ops import unary_union

    class L(object):
        building_area = buildings if buildings is not None else Polygon()
    P = object.__new__(jp_signs.Placer)
    P.lane_area, P.cw, P.core_road = lanes, cw, asphalt
    P.kerb_edge = unary_union([cw, asphalt])
    P.L, P.posts, P.dropped, P.stop_posts = L(), [], [], {}
    P.core_moved, P.core_dropped = [], []
    P.ground_z = lambda x, y: 0.15
    P.clashes = lambda *a, **k: []
    return P


def _post(P, kind, x, y, face=(-1.0, 0.0), **kw):
    p = {"x": x, "y": y, "face": np.array(face), "plates": [(kind, False, "t")], **kw}
    P.posts.append(p)
    return p


def test_core_asphalt():
    """core_merge.core_asphalt: the model's Roads + markings (x scaled), not its
    tactile paving, clipped to the core rectangle, cached. / モデルの車道アスファルト"""
    import core_merge
    with tempfile.TemporaryDirectory() as d:
        meta = {"core": {"bounds": [-10, -10, 10, 10], "x_scale": 0.5}}
        assert core_merge.core_asphalt(d, meta) is None                      # no core_layers.npz / 無し
        tri = lambda *p: [[x, y, 0.0] for x, y in p]                          # noqa: E731
        np.savez(os.path.join(d, "core_layers.npz"),
                 Roads=np.array([tri((0, 0), (8, 0), (8, 4)), tri((0, 0), (8, 4), (0, 4))], np.float32),
                 Roads_mat=np.zeros(2, np.int16), Roads_names=np.array(["M_asphalt"]),
                 Road_Markings=np.array([tri((0, 4), (8, 4), (8, 4.2)), tri((0, -3), (4, -3), (4, -1))], np.float32),
                 Road_Markings_mat=np.array([0, 1], np.int16), Road_Markings_names=np.array(["M_line_white", "M_tactile"]))
        g = core_merge.core_asphalt(d, meta)
        assert abs(g.area - 4 * 4.2 / 2 - 4 * 4 / 2) < 0.05, g.area           # x halved, tactile left out
        assert os.path.exists(os.path.join(d, "core", "asphalt.wkb"))
        assert core_merge.core_asphalt(d, meta).equals(g)
        assert core_merge.core_asphalt(d, {}) is None


def test_core_asphalt_posts():
    """settle_core(): a post on the embedded model's asphalt shoulder moves to
    the nearest free spot off it (plate half width + 5 cm), a post already off it
    and one on a signal pole stay, a 止まれ stays 0-2.95 m before its line (its
    206 follows) or is left out, a move across a lane (the other side of the
    road) is refused, maps without a core are untouched.
    モデルのアスファルト上の支柱を最寄りの縁石側へ移す規則。"""
    from shapely.geometry import box
    from shapely.ops import unary_union
    lanes, cw = box(-50, -3, 50, 3), box(-50, -3.2, 50, 3.2)
    asphalt = unary_union([box(-50, -4.5, 50, 4.5), box(20, 3, 30, 6.4)])      # shoulders + a corner apron
    house = unary_union([box(-24, 4.6, -19.6, 8.0), box(-35, -8.0, -28.7, -4.6)])
    P = _core_placer(lanes, cw, asphalt, house)
    a = _post(P, "noparking", 0.0, 4.0)                                        # on the shoulder / 路肩の上
    b = _post(P, "noparking", 10.0, 5.6)                                       # 1.1 m off it / 既に外
    c = _post(P, "crossing", 25.0, 5.0, face=(0.0, -1.0))                      # in the apron / 隅の舗装
    s = _post(P, "noentry", 40.0, 4.0, signal="tl1")                           # on a signal pole / 信号柱
    t = _post(P, "tomare", -20.0, 4.2, stops=[(7, np.array([-18.0, 1.5]), np.array([1.0, 0.0]))])
    P.stop_posts[7] = np.array([-20.0, 4.2])
    # driving west on the south side, line at x -32: the house covers every spot 0-2.95 m before it
    # 西行き、停止線 x -32: その手前 0-2.95 m は家で塞がる
    u = _post(P, "tomare", -30.0, -4.2, face=(1.0, 0.0), stops=[(8, np.array([-32.0, -1.5]), np.array([-1.0, 0.0]))])
    moved, dropped = P.settle_core()
    assert (moved, dropped) == (3, 1), (P.core_moved, P.core_dropped)
    assert abs(a["x"]) < 0.3 and 4.85 - 1e-6 <= a["y"] < 4.96, a                 # straight out, 0.35 off
    assert (b["x"], b["y"]) == (10.0, 5.6) and (s["x"], s["y"]) == (40.0, 4.0)
    assert asphalt.distance(jp_signs.Point(c["x"], c["y"])) >= 0.35 - 1e-9 and np.hypot(c["x"] - 25, c["y"] - 5) < 2.0, c
    assert -19.3 - 1e-6 <= t["x"] <= -18.0 and t["y"] >= 4.85 - 1e-6, t        # slid past the house, before the line
    assert np.allclose(P.stop_posts[7], (t["x"], t["y"])) and u not in P.posts
    assert P.core_dropped[0]["kinds"] == ["tomare"] and P.dropped[0][0] == "tomare"
    for p in P.posts:
        assert np.allclose(p["face"], [-1.0, 0.0]) or np.allclose(p["face"], [0.0, -1.0])   # facing kept / 向きはそのまま
    # the same spot without the stop line: moved past the house (1.7 m) / 停止線が無ければ移動
    P2 = _core_placer(lanes, cw, asphalt, house)
    v = _post(P2, "noparking", -30.0, -4.2, face=(1.0, 0.0))
    assert P2.settle_core() == (1, 0) and v["x"] > -28.4 and v["y"] <= -4.85 + 1e-6, v
    # a narrow street: the only free spot is across the lane / 狭い道: 空きは車線の向こう側のみ
    P3 = _core_placer(box(-50, -0.6, 50, 0.6), box(-50, -0.65, 50, 0.65), box(-50, -1.0, 50, 1.0), box(-50, 1.6, 50, 9))
    _post(P3, "noparking", 0.0, 0.95)
    assert P3.settle_core() == (0, 1) and not P3.posts
    # no core: nothing happens / コア無し: 何もしない
    P4 = _core_placer(lanes, cw, asphalt)
    P4.core_road = None
    w = _post(P4, "noparking", 0.0, 4.0)
    assert P4.settle_core() == (0, 0) and (w["x"], w["y"]) == (0.0, 4.0)


def test_speed_post_off_core_asphalt():
    """speed_spot.Spots: a 274 post keeps CORE_CLEAR off the embedded model's
    asphalt (its shoulder 2 m past the kerb lane): t 1.6 -> the first free t
    past it on the same (reference) kerb; without a core 1.6 as before.
    速度標識はモデルのアスファルトから CORE_CLEAR 離す。"""
    import speed_spot
    from shapely.geometry import LineString, box
    pts = [{"s": float(x), "c": [float(x), -1.75], "l": [float(x), 0.0], "r": [float(x), -3.5], "w": 3.5}
           for x in range(0, 101, 5)]
    S = object.__new__(speed_spot.Spots)
    S.lanes = [{"road": 1, "lane": -1, "pts": pts}]
    S.lane_area = box(0, -3.5, 100, 0)
    S.cw = S.lane_area.buffer(speed_spot.GUTTER)
    S.centres = [(LineString([q["c"] for q in pts]), 3.5)]
    S.lefthand, S.core = True, None
    t, side, _ = S.choose(1, 50.0)
    assert abs(t - speed_spot.KERB_T) < 1e-6 and side == "reference kerb", (t, side)
    S.core = box(0, -8.0, 100, 2.0)                                   # the model's street: 2 m shoulder north
    t, side, why = S.choose(1, 50.0)
    assert 2.0 + speed_spot.CORE_CLEAR - 1e-6 <= t <= 2.0 + speed_spot.CORE_CLEAR + 0.11, (t, side)
    assert side.startswith("reference kerb") and "asphalt" in why, (side, why)
    assert "asphalt" in S.free(50.0, 1.0) and S.free(50.0, t) == ""


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)
    print("all tests passed / 全試験合格")
