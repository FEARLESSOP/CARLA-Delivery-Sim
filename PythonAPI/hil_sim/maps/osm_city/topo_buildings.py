"""Buildings from a topoexport GLB (GSI footprints, PLATEAU LOD2 heights) that
OpenStreetMap lacks, added to the OSM file as building ways (maps/.venv).

  python topo_buildings.py <in.osm> <topo.glb> <out.osm> --shift SX SY
                           [--keep-out HALF | X0 Y0 X1 Y1] [--min-area 12]

--shift: map frame (metres east/north of the OSM bounds centre) -> GLB
coordinates, i.e. topo = map + shift (osm_city/terrain.py finds it as
osm_sw_to_topo; shift = that + (half_x, half_y)). A topo building is added
when no OSM building covers more than 30 % of it; untagged OSM buildings get
the height of the topo building under them (height=..., building:levels from it);
the export's 6.0 m placeholder (no measured height) is never written. --keep-out HALF (or X0 Y0 X1 Y1)
leaves the square |x|, |y| < HALF (or that rectangle) alone (an embedded model covers it, maps.json
"core"). Footprint = the union of the building's triangles seen from above.

topoexport の建物（国土地理院の外形・PLATEAU の高さ）のうち OSM に無いものを
OSM の建物ウェイとして追加する。高さの無い OSM 建物には height を付ける。
"""
import argparse
import json
import math
import os
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

import numpy as np
from shapely.geometry import Polygon
from shapely.ops import unary_union
from shapely.strtree import STRtree

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

BLENDER_DUMP = r'''
import bpy, sys, numpy as np
a = sys.argv[sys.argv.index("--") + 1:]
bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.import_scene.gltf(filepath=a[0])
dg = bpy.context.evaluated_depsgraph_get()
T, K = [], []
for k, o in enumerate(o for o in bpy.data.objects if o.type == "MESH" and o.name.startswith("TPX_Buildings")):
    me = o.evaluated_get(dg).to_mesh(); me.calc_loop_triangles()
    V = np.array([(o.matrix_world @ v.co)[:] for v in me.vertices], np.float32)
    V = V[:, [0, 2, 1]] * (1, 1, -1)       # topoexport writes z-up into the y-up glTF: Blender (x, -up, north)
    for t in me.loop_triangles:
        T.append(V[list(t.vertices)]); K.append(k)
np.savez_compressed(a[1], T=np.array(T, np.float32), K=np.array(K, np.int32))
print("DUMP", len(T), "triangles", K[-1] + 1 if K else 0, "buildings")
'''


PLACEHOLDER = 6.0     # the export's height where it has none (899 of 973 in Oita) / 高さ不明時の既定値


def real_height(h):
    """A measured height, or None for the export's 6.0 m placeholder (towers
    came out 2 storeys: ドコモ大分ビル 72 m -> 6 m). / 既定値 6.0 m は高さ不明"""
    return None if h is None or abs(h - PLACEHOLDER) < 0.05 else h


def levels(h):
    """Storeys from a PLATEAU height (~3.2 m a storey): buildings_jp picks the
    style from building:levels (without it a big footprint became a shed and a
    small tall one a house). / 高さから階数（様式は階数で決まる）"""
    return max(1, int(round(h / 3.2)))


def topo_footprints(glb):
    with tempfile.TemporaryDirectory() as td:
        sp, out = os.path.join(td, "dump.py"), os.path.join(td, "b.npz")
        open(sp, "w").write(BLENDER_DUMP)
        subprocess.check_call([os.environ.get("BLENDER", "blender"), "-b", "--disable-autoexec", "-P", sp, "--", glb, out],
                              stdout=subprocess.DEVNULL)
        d = np.load(out)
        T, K = d["T"], d["K"]
    res = []
    for k in np.unique(K):
        t = T[K == k]
        tris = [Polygon(x[:, :2]) for x in t if abs((x[1, 0] - x[0, 0]) * (x[2, 1] - x[0, 1]) - (x[2, 0] - x[0, 0]) * (x[1, 1] - x[0, 1])) > 1e-4]
        if not tris:
            continue
        g = unary_union(tris).buffer(0.05).buffer(-0.05)
        if g.geom_type == "MultiPolygon":
            g = max(g.geoms, key=lambda p: p.area)
        res.append((g, float(t[..., 2].min()), float(t[..., 2].max())))
    return res


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("osm"); ap.add_argument("glb"); ap.add_argument("out")
    ap.add_argument("--shift", nargs=2, type=float, required=True)
    ap.add_argument("--keep-out", type=float, nargs="+", default=[])
    ap.add_argument("--min-area", type=float, default=12.0)
    a = ap.parse_args()
    tree = ET.parse(a.osm)
    root = tree.getroot()
    b = root.find("bounds")
    lat0 = (float(b.get("minlat")) + float(b.get("maxlat"))) / 2
    lon0 = (float(b.get("minlon")) + float(b.get("maxlon"))) / 2
    import geo
    fr = geo.Frame({"lat0": lat0, "lon0": lon0})
    nodes = {n.get("id"): fr.xy(float(n.get("lon")), float(n.get("lat"))) for n in root.iter("node")}
    osm_b = []
    for w in root.iter("way"):
        t = {g.get("k"): g.get("v") for g in w.iter("tag")}
        if "building" not in t:
            continue
        c = [nodes[n.get("ref")] for n in w.iter("nd") if n.get("ref") in nodes]
        if len(c) >= 4:
            p = Polygon(c).buffer(0)
            if p.area > 1:
                osm_b.append((p, w, t))
    sx, sy = a.shift
    topo = []
    for g, z0, z1 in topo_footprints(a.glb):
        g = Polygon([(x - sx, y - sy) for x, y in g.exterior.coords])
        if g.area >= a.min_area and z1 - z0 >= 2.0:
            topo.append((g, z1 - z0))
    idx = STRtree([p for p, _, _ in osm_b])
    added, heighted, kept_out = 0, 0, 0
    # ids far below anything else (roadnet adds its own negative ids below the
    # file's lowest) / 他と衝突しない番号
    nid = min([0] + [int(e.get("id")) for e in root if e.get("id") and e.get("id").lstrip("-").isdigit()]) - 10000000
    for g, h in topo:
        c = g.centroid
        ko = a.keep_out if len(a.keep_out) != 1 else [-a.keep_out[0], -a.keep_out[0], a.keep_out[0], a.keep_out[0]]
        if ko and ko[0] < c.x < ko[2] and ko[1] < c.y < ko[3]:
            kept_out += 1
            continue
        cover = 0.0
        for i in idx.query(g):
            p, w, t = osm_b[i]
            inter = p.intersection(g).area
            cover += inter
            if real_height(h) and inter > 0.5 * p.area and not any(k in t for k in ("height", "building:levels")):
                ET.SubElement(w, "tag", k="height", v="%.1f" % h)
                ET.SubElement(w, "tag", k="building:levels", v=str(levels(h)))
                t["height"] = "%.1f" % h
                heighted += 1
        if cover > 0.3 * g.area:
            continue
        g = g.simplify(0.3)
        if g.geom_type != "Polygon" or g.area < a.min_area:
            continue
        refs = []
        for x, y in list(g.exterior.coords)[:-1]:
            lon, lat = fr.lonlat(x, y)
            ET.SubElement(root, "node", id=str(nid), lat="%.8f" % lat, lon="%.8f" % lon, version="1", visible="true")
            refs.append(nid); nid -= 1
        w = ET.SubElement(root, "way", id=str(nid), version="1", visible="true"); nid -= 1
        for r in refs + refs[:1]:
            ET.SubElement(w, "nd", ref=str(r))
        ET.SubElement(w, "tag", k="building", v="yes")
        if real_height(h):
            ET.SubElement(w, "tag", k="height", v="%.1f" % h)
            ET.SubElement(w, "tag", k="building:levels", v=str(levels(h)))
        ET.SubElement(w, "tag", k="source", v="topoexport (GSI / PLATEAU)")
        added += 1
    # osmium-style order: nodes before ways (netconvert and our reader need it)
    kids = list(root)
    root[:] = [k for k in kids if k.tag in ("note", "meta", "bounds")] + [k for k in kids if k.tag == "node"] + \
              [k for k in kids if k.tag == "way"] + [k for k in kids if k.tag == "relation"]
    tree.write(a.out, encoding="utf-8", xml_declaration=True)
    print("topo buildings: %d in the export, %d added, %d OSM buildings given a height, %d left to the core"
          % (len(topo), added, heighted, kept_out))


if __name__ == "__main__":
    main()
