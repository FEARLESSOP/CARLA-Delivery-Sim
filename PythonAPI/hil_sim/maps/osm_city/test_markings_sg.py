"""Tests for the Singapore (LTA) markings helpers (maps/.venv; guide ch. 51).

  .venv/bin/python osm_city/test_markings_sg.py

ground_runs splits a lane at the flyover parts; the LTA arrows have the RMS4
size and point the right way; road words fit the lane; the textures write.
シンガポール式標示の補助関数のテスト。
"""

import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import markings_sg as MS  # noqa: E402


def lane(zs):
    pts = [{"c": [float(i), 0.0], "l": [float(i), 1.7], "r": [float(i), -1.7], "w": 3.4, "z": z, "s": float(i)}
           for i, z in enumerate(zs)]
    return {"road": 1, "section": 0, "lane": -1, "junction": -1, "speed": 13.9, "name": "", "pts": pts}


def test_ground_runs():
    l = lane([0.0] * 5 + [1.0, 3.0, 6.5, 3.0, 1.0] + [0.0] * 6)
    g = MS.ground_runs([l], "ground")
    assert len(g) == 2, len(g)
    assert g[0]["at_end"] and g[0]["pts"][-1] is l["pts"][-1], "the run reaching the end comes first"
    assert not g[1]["at_end"]
    d = MS.ground_runs([l], "deck")
    assert len(d) == 1 and len(d[0]["pts"]) == 5 and not d[0]["at_end"]
    flat = lane([0.0] * 6)
    assert MS.ground_runs([flat], "ground") == [flat] and MS.ground_runs([flat], "deck") == []


def test_arrows():
    M = object.__new__(MS.MarkingsSG)
    for turns, ymin, ymax in ((["straight"], -0.4, 0.4), (["left"], -0.2, 2.5), (["right"], -2.5, 0.2),
                              (["straight", "left"], -0.4, 2.2)):
        a = M.arrow_lta((0.0, 0.0), 0.0, turns)
        assert a.is_valid and not a.is_empty, turns
        x0, y0, x1, y1 = a.bounds
        assert 4.5 <= x1 - x0 <= 5.8, (turns, x1 - x0)          # RMS4: 5.5 m / 4.5 m
        assert ymin - 0.05 <= y0 and y1 <= ymax + 0.05, (turns, a.bounds)
    left = M.arrow_lta((0.0, 0.0), 0.0, ["left"])
    assert left.bounds[3] > 1.0, "a left arrow bends to +y (left of travel)"


def test_word_fit():
    M = object.__new__(MS.MarkingsSG)
    g = M.word_fit("SLOW", (0.0, 0.0), 0.0, 2.4, 2.0)
    x0, y0, x1, y1 = g.bounds
    assert abs((x1 - x0) - 2.4) < 0.15, x1 - x0            # along the travel direction / 進行方向の長さ
    assert abs((y1 - y0) - 2.0) < 0.15, y1 - y0            # across the lane / 横幅


def test_textures():
    import textures_sg
    d = tempfile.mkdtemp()
    textures_sg.write(d)
    for name, (tx, col, rough, metal, smooth) in textures_sg.MATERIALS.items():
        for suf in ("", "_n", "_r", "_m"):
            f = os.path.join(d, (tx if suf in ("", "_n") else name) + suf + ".png")
            assert os.path.exists(f), f
        assert "light" not in name and "sign" not in name


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print("ok", name)
