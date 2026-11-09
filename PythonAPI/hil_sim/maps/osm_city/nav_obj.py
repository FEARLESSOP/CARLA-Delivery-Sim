"""Write a RecastBuilder OBJ for a generated map: CARLA axes (x, z_up, y_carla = -y_map),
areas by material group (road / sidewalk / crosswalk / grass / block), like CARLA's FBX2OBJ.
All 'v' lines first: CARLA's MeshLoaderObj resets the material on every 'v' line,
so 'usemtl' must sit right before its faces.
  python nav_obj.py mesh_dir out.obj cw.npz [cx cy r]   (optional square crop for tests)
cw.npz comes from nav_crosswalks.py. Then RecastBuilder out.obj -> out.bin (guide ch. 7, 31).

Areas: Road_Road / Road_Curb -> road (walkers avoid it, filter 0 excludes it),
crosswalk faces (cw.npz V/I) -> crosswalk, Road_Sidewalk (pavements, footways and the
路側帯 strips inside the carriageway edge) and the cw.npz apron faces (SV/SI) ->
sidewalk, Road_Grass -> block (written as road: see MATERIAL) except the lawns cw.npz lists (parks, playgrounds),
which stay grass (CARLA's walker filter allows grass: walkers wandered off the
pavements onto it), Terrain_Street -> block. The road faces under crosswalk and
apron faces are left out. RecastBuilder marks every face's area over a 5 m prism
above its lowest vertex, faces later in the file winning, so crosswalks go last.
歩行者ナビメッシュ用 OBJ を書き出す。"""
import glob, os, sys
import numpy as np
mesh_dir, out = sys.argv[1], sys.argv[2]
crop = list(map(float, sys.argv[4:7])) if len(sys.argv) > 6 else None
import shapely
from shapely import wkt
CW = np.load(sys.argv[3])
cw_union = shapely.union_all([wkt.loads(h) for h in CW["hulls"]])
SV = CW["SV"] if "SV" in CW.files else np.zeros((0, 3))
SI = CW["SI"] if "SI" in CW.files else np.zeros((0, 3), np.int64)
if len(SI):
    # the aprons' outline (from their own faces) / 帯の輪郭
    ap = shapely.union_all(shapely.polygons(SV[SI][:, :, :2]))
    cw_union = shapely.union_all([cw_union, ap.buffer(0.01)])
shapely.prepare(cw_union)
lawns = shapely.union_all([wkt.loads(h) for h in CW["lawns"]]) if "lawns" in CW.files and len(CW["lawns"]) else None
lawn_mode = "lawns" in CW.files          # Japanese maps (nav_areas.json): grass is block outside lawns / 芝は公園のみ
if lawns is not None:
    shapely.prepare(lawns)
AREA = {"Road_Road": "road", "Road_Curb": "road", "Road_Crosswalk": "crosswalk",
        "Road_Sidewalk": "sidewalk", "Road_Grass": "grass", "Terrain_Street": "block"}
ORDER = ("block", "grass", "road", "sidewalk", "crosswalk")
# CARLA's RecastBuilder knows only road / grass / sidewalk / crosswalk: any other
# material falls back to its default area, which walkers may use (the "block"
# grass and walls became pavement for them: 43 % of spawn points on grass, live
# 2026-11-07). "road" is the one area walker filter 0 excludes
# (Navigation.cpp setExcludeFlags(CARLA_TYPE_ROAD)), so blocked ground goes in as road.
# RecastBuilder は road/grass/sidewalk/crosswalk のみ認識。それ以外は歩行可能な既定扱い
# になるため、歩かせない所は road（歩行者フィルタ 0 が除外する）として書く。
MATERIAL = {"block": "road", "grass": "grass", "road": "road", "sidewalk": "sidewalk", "crosswalk": "crosswalk"}
verts, faces, off = [], {}, 0


def add(area, V, I):
    global off
    used = np.unique(I)
    if not len(used):
        return
    remap = np.zeros(len(V), np.int64)
    remap[used] = np.arange(len(used))
    verts.append(np.stack([V[used, 0], V[used, 2], -V[used, 1]], 1))
    faces.setdefault(area, []).append(remap[I] + off + 1)
    off += len(used)


def cropped(V, I):
    if not crop:
        return I
    c = V[I].mean(1)
    return I[(abs(c[:, 0] - crop[0]) < crop[2]) & (abs(c[:, 1] - crop[1]) < crop[2])]


for f in sorted(glob.glob(os.path.join(mesh_dir, "*.npz"))):
    area = AREA.get(os.path.basename(f).rsplit("_", 2)[0])
    if not area:
        continue
    d = np.load(f)
    for k in d.files:
        if not k.endswith("|V"):
            continue
        V, I = d[k], d[k[:-2] + "|I"]
        I = cropped(V, I)
        if area == "road":
            c = V[I].mean(1)
            I = I[~shapely.contains_xy(cw_union, c[:, 0], c[:, 1])]
        if area == "grass" and lawn_mode:
            c = V[I].mean(1)
            on = shapely.contains_xy(lawns, c[:, 0], c[:, 1]) if lawns is not None else np.zeros(len(I), bool)
            add("block", V, I[~on])
            I = I[on]
        add(area, V, I)
if len(SI):
    add("sidewalk", SV, cropped(SV, SI))
HV, HI = CW["V"], CW["I"]
if len(HI):
    add("crosswalk", HV, cropped(HV, HI))
with open(out, "w") as fo:
    np.savetxt(fo, np.concatenate(verts), fmt="v %.3f %.3f %.3f")
    for area in ORDER:
        parts = faces.get(area)
        if not parts:
            continue
        fo.write("o %s\nusemtl %s\n" % (area, MATERIAL[area]))
        np.savetxt(fo, np.concatenate(parts), fmt="f %d %d %d")
        print(area, sum(len(p) for p in parts), "triangles")
