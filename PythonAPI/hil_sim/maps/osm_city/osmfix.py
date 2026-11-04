"""Per-map corrections to the OpenStreetMap data (maps.json "osm_fixes").

Some OSM ways are drawn metres off the real road, or cross a building that
the zoom-20 imagery shows whole. They are corrected where the OSM file is
read, for the road logic (roadnet.py) and the 3-D model (osmdata.py) alike,
so both see the same streets. A map without "osm_fixes" is read unchanged.

  "osm_fixes": {
    "move_nodes": {"<node id>": [dx, dy], ...},     # metres east / north (map frame)
    "drop_ways": ["<way id>", ...],
    "split_ways": {"<way id>": ["<node id>", ...]}, # pieces: the way's own id, then <id>01, <id>02, ...
    "way_tags": {"<way id>": {"key": "value" or null}},   # null removes the tag
    "add_ways": [{"id": "<new way id>", "pts": [[x, y], ...], "tags": {...},
                  "closed": true}]            # closed: an area (ring back to the first point)
  }

Applied in that order. Keys starting with "_" are notes. Measure against the
imagery (guide ch. 3, "Per-map OSM corrections").

地図ごとの OSM 修正（maps.json "osm_fixes"）: ノード移動・ウェイ削除・分割・タグ・追加。
道路網（roadnet.py）と 3D モデル（osmdata.py）の両方で OSM を読む時に適用する。
"""

import json
import os
import xml.etree.ElementTree as ET

import geo

HERE = os.path.dirname(os.path.abspath(__file__))


def fixes_for(meta):
    """The "osm_fixes" of the maps.json entry whose name is meta["name"], or
    None. / 地図の osm_fixes（無ければ None）。"""
    try:
        cfg = json.load(open(os.path.join(HERE, "..", "maps.json")))
    except (OSError, ValueError):
        return None
    for v in cfg.values():
        if isinstance(v, dict) and v.get("name") == meta.get("name"):
            fx = v.get("osm_fixes") or None
            # the area features' own fixes (landmarks/akeno.py; AKENO_WIP=1: the
            # work-in-progress entry, scratch builds) / 地物の修正も加える
            af = v.get("_area_features_wip") if os.environ.get("AKENO_WIP") else v.get("area_features")
            wip = (af or {}).get("osm_fixes")
            if wip:
                fx = dict(fx or {})
                fx["drop_ways"] = list(fx.get("drop_ways", [])) + list(wip.get("drop_ways", []))
                fx["add_ways"] = list(fx.get("add_ways", [])) + list(wip.get("add_ways", []))
                for k in ("move_nodes", "way_tags"):
                    fx[k] = dict(fx.get(k, {}), **wip.get(k, {}))
            return fx
    return None


def _tag(way, k, v):
    for t in way.findall("tag"):
        if t.get("k") == k:
            if v is None:
                way.remove(t)
            else:
                t.set("v", str(v))
            return
    if v is not None:
        ET.SubElement(way, "tag", k=k, v=str(v))


def apply(root, meta, quiet=False):
    """Apply the map's OSM fixes to an OSM XML ``root`` in place. Returns the
    counts, or None when the map has none (the tree is then untouched).
    OSM の XML に修正を適用する（修正が無ければ何もしない）。"""
    fx = fixes_for(meta)
    if not fx:
        return None
    fr = geo.Frame(meta)
    n = {"moved": 0, "dropped": 0, "split": 0, "tagged": 0, "added": 0}
    nodes = {nd.get("id"): nd for nd in root.iter("node")}
    ways = {w.get("id"): w for w in root.iter("way")}
    for wid in fx.get("drop_ways", []):
        w = ways.pop(str(wid), None)
        if w is not None:
            root.remove(w)
            n["dropped"] += 1
    for nid, (dx, dy) in fx.get("move_nodes", {}).items():
        nd = nodes.get(str(nid))
        if nd is None:
            continue
        x, y = fr.xy(float(nd.get("lon")), float(nd.get("lat")))
        lon, lat = fr.lonlat(x + float(dx), y + float(dy))
        nd.set("lon", "%.9f" % lon)
        nd.set("lat", "%.9f" % lat)
        n["moved"] += 1
    for wid, at in fx.get("split_ways", {}).items():
        w = ways.get(str(wid))
        if w is None:
            continue
        at = {str(a) for a in at}
        nds = w.findall("nd")
        pieces, cur = [], []
        for i, nd in enumerate(nds):
            cur.append(nd.get("ref"))
            if nd.get("ref") in at and 0 < i < len(nds) - 1:
                pieces.append(cur)
                cur = [nd.get("ref")]
        pieces.append(cur)
        if len(pieces) < 2:
            continue
        tags = [(t.get("k"), t.get("v")) for t in w.findall("tag")]
        for nd in nds:
            w.remove(nd)
        for t in w.findall("tag"):
            w.remove(t)
        pos = list(root).index(w)
        for k, refs in enumerate(pieces):
            if k:
                pw = ET.Element("way", dict(w.attrib))
                pw.set("id", "%s%02d" % (wid, k))
                root.insert(pos + k, pw)
                ways[pw.get("id")] = pw
            else:
                pw = w
            for r in refs:
                ET.SubElement(pw, "nd", ref=r)
            for tk, tv in tags:
                ET.SubElement(pw, "tag", k=tk, v=tv)
        n["split"] += 1
    for wid, tags in fx.get("way_tags", {}).items():
        w = ways.get(str(wid))
        if w is None:
            continue
        for k, v in tags.items():
            _tag(w, k, v)
        n["tagged"] += 1
    kids = list(root)
    first_way = next((i for i, e in enumerate(kids) if e.tag == "way"), len(kids))
    for aw in fx.get("add_ways", []):
        wid = str(aw["id"])
        w = ET.Element("way", id=wid, version="1", visible="true")
        for k, (x, y) in enumerate(aw["pts"]):
            nid = "%s%02d" % (wid, k)
            lon, lat = fr.lonlat(float(x), float(y))
            root.insert(first_way, ET.Element("node", id=nid, version="1", visible="true",
                                              lat="%.9f" % lat, lon="%.9f" % lon))
            first_way += 1
            ET.SubElement(w, "nd", ref=nid)
        if aw.get("closed"):
            # an area (a building, a car park): the ring ends on its first node / 面: 最初のノードで閉じる
            ET.SubElement(w, "nd", ref="%s00" % wid)
        for k, v in aw.get("tags", {}).items():
            ET.SubElement(w, "tag", k=k, v=str(v))
        root.insert(first_way, w)
        ways[wid] = w
        n["added"] += 1
    if not quiet:
        print("osm fixes (maps.json osm_fixes): %(moved)d nodes moved, %(dropped)d ways dropped, "
              "%(split)d split, %(tagged)d retagged, %(added)d added" % n)
    return n
