"""Offline test for lht_poles.py's embedded-model rule (carla-venv, no server).

  carla-venv/bin/python osm_city/test_lht_poles.py

A one-way keep-left road (lane -1, 3.5 m, kerb at t = 0) with one light:
  - no core: the pole goes to the nearest t with the head 6 m left of it over
    the lane (t -6.5), as before;
  - core asphalt 4.5 m past the lane (to t -8): the pole keeps CORE_CLEAR off it
    (t -8.3), the head still over the lane, and carla.Map puts the pole there;
  - core asphalt 7 m past the lane: no spot has both, the plain choice stays.
lht_poles.py の埋め込みモデル規則の試験（柱はモデルの車道外、灯器は車線上）。
"""

import json
import os
import subprocess
import sys
import tempfile
import time

import carla
import shapely
from shapely.geometry import box

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import lht_poles  # noqa: E402

XODR = """<?xml version="1.0" standalone="yes"?>
<OpenDRIVE>
    <header revMajor="1" revMinor="4" name="" version="1.00" north="0" south="0" east="0" west="0"/>
    <road name="" length="100.0" id="1" junction="-1">
        <link/>
        <planView>
            <geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="100.0"><line/></geometry>
        </planView>
        <elevationProfile/><lateralProfile/>
        <lanes>
            <laneSection s="0.0">
                <center><lane id="0" type="none" level="false"/></center>
                <right><lane id="-1" type="driving" level="false"><width sOffset="0.0" a="3.5" b="0" c="0" d="0"/></lane></right>
            </laneSection>
        </lanes>
        <signals>
            <signal id="L1" s="60.0" t="-4.800" orientation="+" dynamic="yes" zOffset="0" country="OpenDRIVE" type="1000011" subtype="10" height="0.78" width="0.26">
                <validity fromLane="-1" toLane="-1"/>
            </signal>
        </signals>
    </road>
</OpenDRIVE>
"""


def run(core_box):
    """lht_poles.py on the test road, with a build folder whose core asphalt is
    core_box (map frame) or none; returns (t, pole xy map frame) from carla.Map."""
    with tempfile.TemporaryDirectory() as d:
        src = os.path.join(d, "T.xodr")
        open(src, "w").write(XODR)
        if core_box is not None:
            json.dump({"name": "T", "core": {"bounds": [-10, -50, 110, 50]}}, open(os.path.join(d, "meta.json"), "w"))
            open(os.path.join(d, "core_layers.npz"), "wb").write(b"")          # only its mtime is read / 時刻のみ
            os.makedirs(os.path.join(d, "core"))
            time.sleep(0.01)
            open(os.path.join(d, "core", "asphalt.wkb"), "wb").write(shapely.to_wkb(box(*core_box)))
        out = os.path.join(d, "out.xodr")
        log = subprocess.run([sys.executable, os.path.join(HERE, "lht_poles.py"), src, out],
                             capture_output=True, text=True, check=True).stdout
        text = open(out).read()
    m = carla.Map("t", text)
    lm = [x for x in m.get_all_landmarks() if x.is_dynamic][0]
    import re
    t = float(re.search(r'<signal id="L1" s="[^"]*" t="([^"]*)"', text).group(1))
    return t, (lm.transform.location.x, -lm.transform.location.y), log


def test_no_core():
    t, p, _ = run(None)
    assert abs(t + 6.5) < 0.01, t


def test_core_moves_pole():
    t, p, log = run((-10, -8.0, 110, 0.0))           # lane y -3.5..0, asphalt on to y -8 / 路肩 4.5 m
    assert abs(t + 8.3) < 0.051, (t, log)
    assert p[1] <= -8.0 - lht_poles.CORE_CLEAR + 1e-3, p                   # where CARLA puts it / 実配置
    head = -t - lht_poles.ARM_REACH                                        # metres right of the kerb
    assert lht_poles.HEAD_IN - 1e-6 <= head <= 3.5 - lht_poles.HEAD_IN + 1e-6, head


def test_core_no_spot_keeps_plain():
    t, p, log = run((-10, -10.5, 110, 0.0))          # asphalt 7 m past the lane: head cannot reach / 届かない
    assert abs(t + 6.5) < 0.01, (t, log)
    assert "plain choice kept (still on it): L1" in log, log


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)
    print("all tests passed / 全試験合格")
