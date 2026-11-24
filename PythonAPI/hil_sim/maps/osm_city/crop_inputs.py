"""Crop a map's inputs to one area for fast landmark iteration (scratch copies only).

  python -I crop_inputs.py <osm> <build dir> <scratch dir> x0 y0 x1 y1

Writes <scratch dir>/crop.osm (ways with any node in the box, all their nodes,
tagged standalone nodes in the box such as bus stops (layout's bus bays need them);
relations whose way members all survive; the original <bounds>, so the map
frame does not move) and <scratch dir>/<Name>/{lanes.json (lanes with a point in
the box), meta.json, <Name>.xodr, grade_sep.json, colour_notes.json,
road_features.json (one snapshot, refused when written > 10 min apart), *_trace.json,
*_photos link}. Then
  BUILD_AREA=... python build.py <scratch>/crop.osm <scratch>/<Name>
builds that site in about a minute instead of ~45 min of Layout on a 24 km2
map. Use a box ~300 m larger than the site. Never use it for a real build.

地図の入力を一部だけに切り出す（詳細モデルの試作用、作業用コピーのみ）。
"""

import glob
import json
import math
import os
import shutil
import sys
import xml.etree.ElementTree as ET


def main():
    src, build, scratch = sys.argv[1:4]
    x0, y0, x1, y1 = map(float, sys.argv[4:8])
    meta = json.load(open(os.path.join(build, "meta.json")))
    out = os.path.join(scratch, meta["name"])
    os.makedirs(out, exist_ok=True)
    R = 6367449.0
    KY = R * math.pi / 180.0
    kx = KY * math.cos(math.radians(meta["lat0"]))
    root = ET.parse(os.path.expanduser(src)).getroot()
    inside = {}
    for n in root.iter("node"):
        x = (float(n.get("lon")) - meta["lon0"]) * kx
        y = (float(n.get("lat")) - meta["lat0"]) * KY
        inside[n.get("id")] = x0 < x < x1 and y0 < y < y1
    keep_w, need_n = set(), set()
    for w in root.iter("way"):
        refs = [nd.get("ref") for nd in w.findall("nd")]
        if any(inside.get(r) for r in refs):
            keep_w.add(w.get("id"))
            need_n.update(refs)
    # tagged nodes in the box that are on no kept way (bus stops: layout.add_bus_bays,
    # shelters) / 箱内の単独のタグ付きノード（バス停など）も残す
    for n in root.iter("node"):
        if inside.get(n.get("id")) and n.find("tag") is not None:
            need_n.add(n.get("id"))
    osm = ET.Element("osm", root.attrib)
    for el in root:
        if el.tag == "bounds" or (el.tag == "node" and el.get("id") in need_n) or \
                (el.tag == "way" and el.get("id") in keep_w):
            osm.append(el)
        elif el.tag == "relation":
            mem = [m for m in el.findall("member") if m.get("type") == "way"]
            if mem and all(m.get("ref") in keep_w for m in mem):
                osm.append(el)
    ET.ElementTree(osm).write(os.path.join(scratch, "crop.osm"), encoding="utf-8", xml_declaration=True)
    # one self-consistent snapshot of the road logic: xodr, lanes.json, grade_sep.json and
    # meta.json copied together (a symlinked xodr rewritten by a later road-logic run mixed
    # with an older lanes.json); refuse a set that was written more than 10 min apart
    # 道路情報を一組で複製（後で書き換わる xodr へのリンクと古い lanes.json が混ざった）
    logic = [os.path.join(build, f) for f in (meta["name"] + ".xodr", "lanes.json", "meta.json")]
    mt = [os.path.getmtime(f) for f in logic if os.path.exists(f)]
    if mt and max(mt) - min(mt) > 600:
        raise SystemExit("road logic files written %.0f min apart (a road-logic run in progress?): %s"
                         % ((max(mt) - min(mt)) / 60, ", ".join(os.path.basename(f) for f in logic)))
    for f in (meta["name"] + ".xodr", "grade_sep.json", "colour_notes.json", "road_features.json"):
        if os.path.exists(os.path.join(build, f)):
            dst = os.path.join(out, f)
            if os.path.islink(dst):
                os.remove(dst)
            shutil.copy(os.path.join(build, f), dst)
    d = json.load(open(os.path.join(build, "lanes.json")))
    n0 = len(d["lanes"])
    d["lanes"] = [ln for ln in d["lanes"] if any(x0 < q["c"][0] < x1 and y0 < q["c"][1] < y1 for q in ln["pts"])]
    json.dump(d, open(os.path.join(out, "lanes.json"), "w"))
    shutil.copy(os.path.join(build, "meta.json"), out)
    for f in glob.glob(os.path.join(build, "*_trace.json")):
        shutil.copy(f, out)
    for f in glob.glob(os.path.join(build, "*_photos")):
        dst = os.path.join(out, os.path.basename(f))
        if not os.path.exists(dst):
            os.symlink(os.path.abspath(f), dst)
    print("crop: %d ways, %d nodes, lanes %d of %d -> %s" % (len(keep_w), len(need_n), len(d["lanes"]), n0, out))


if __name__ == "__main__":
    main()
