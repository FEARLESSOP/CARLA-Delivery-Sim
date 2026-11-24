"""Crop a build dir to a square round one site for landmark iteration (maps/.venv).

  python crop_build.py <build dir> <scratch dir> <lon> <lat> <half_m>

Writes <scratch dir>/meta.json (frame re-centred on lon/lat, half sizes
half_m, a square ring), lanes.json (the lanes, approaches, lights and
junctions within half_m + 30 m, shifted into the new frame) and copies the
*_trace.json files that carry "frame_lonlat0". build.py on it (with the same OSM file, or a crop of it)
takes seconds instead of the whole map's 40+ min; landmark traces that carry
"frame_lonlat0" (landmarks/coral.py) convert themselves into the new frame.
Scratch use only: never import a cropped build. The full build (or
BUILD_AREA, build.py) stays the reference for the checks.

ランドマーク作業用に、1 か所の周りだけの作業用ビルドを作る（座標系をその場所に移す）。
作業用のみ。チェックの正は全体ビルド。
"""

import glob
import json
import os
import shutil
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import geo  # noqa: E402


def main():
    src, dst = sys.argv[1], sys.argv[2]
    lon, lat, R = float(sys.argv[3]), float(sys.argv[4]), float(sys.argv[5])
    os.makedirs(dst, exist_ok=True)
    meta = json.load(open(os.path.join(src, "meta.json")))
    cx, cy = geo.Frame(meta).xy(lon, lat)
    d = json.load(open(os.path.join(src, "lanes.json")))
    sh = lambda p: [round(p[0] - cx, 3), round(p[1] - cy, 3)]
    near = lambda p: abs(p[0] - cx) < R + 30 and abs(p[1] - cy) < R + 30
    lanes = []
    for ln in d["lanes"]:
        if not any(near(q["c"]) for q in ln["pts"]):
            continue
        for q in ln["pts"]:
            for k in ("c", "l", "r"):
                if k in q:
                    q[k] = sh(q[k])
        lanes.append(ln)
    out = {"lanes": lanes, "approaches": [], "lights": [], "junctions": {}}
    for a in d["approaches"]:
        if near(a["end"]):
            a["end"] = sh(a["end"])
            out["approaches"].append(a)
    for a in d["lights"]:
        if near(a["pos"]):
            a["pos"] = sh(a["pos"])
            out["lights"].append(a)
    for k, j in d["junctions"].items():
        if near(j["c"]):
            j["c"] = sh(j["c"])
            out["junctions"][k] = j
    json.dump(out, open(os.path.join(dst, "lanes.json"), "w"))
    m2 = dict(meta, lat0=lat, lon0=lon, half_x=R, half_y=R)
    r = R - 10.0
    m2["ring_wkt"] = "POLYGON ((%s))" % ", ".join("%f %f" % p for p in [(-r, -r), (r, -r), (r, r), (-r, r), (-r, -r)])
    m2["crop_of"] = {"src": os.path.abspath(src), "shift": [cx, cy]}
    json.dump(m2, open(os.path.join(dst, "meta.json"), "w"), indent=1)
    for f in glob.glob(os.path.join(src, "*_trace.json")):
        # only traces that convert themselves (others would land in the wrong place)
        # 自分で座標変換するトレースだけ（他はずれる）
        if "frame_lonlat0" in json.load(open(f)):
            shutil.copy(f, dst)
    print("lanes %d, approaches %d, lights %d, junctions %d; frame shifted by (%.2f, %.2f)"
          % (len(lanes), len(out["approaches"]), len(out["lights"]), len(out["junctions"]), cx, cy))


if __name__ == "__main__":
    main()
