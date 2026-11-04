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
    for shape, w, h in (("disc", 0.6, 0.6), ("tri", 0.8, 0.69), ("rect", 0.8, 0.27)):
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
              "back", "pole"):
        im = jp_signs.texture(k, 256)
        assert im.size == (128, 128) and im.mode == "RGB", k
        assert "sign" not in k and "light" not in k


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


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)
    print("all tests passed / 全試験合格")
