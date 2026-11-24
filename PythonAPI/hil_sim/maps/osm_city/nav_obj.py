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
maps.json "nav" (opt-in per map, read by meta.json's name; maps without it are
unchanged): "walkable_only" leaves the road and block faces out (walkers never use
them; a 4.7 x 5 km box has ~9,600 tiles of 51.2 m, so tile + poly ids leave 256
polys per tile, and dense estates had up to 348 with them, 250 without);
"grass_materials" [...]: only Road_Grass faces of these materials are grass,
the rest block (verges and medians: no walkers stranded on islands; [] = none:
PasirRisTampines' park lawns pushed one tile over 256 polys, DT_INVALID_PARAM);
"clip_to_box": only faces whose centre lies inside meta.json's half_x / half_y box
(the ground skirt beyond it reached 7.7 x 8.1 km: 23.9 k tiles > 16384 slots, the
north rows failed with DT_OUT_OF_MEMORY).
maps.json "nav"（地図ごと）: 車道・不可領域の面を省く／芝として扱う材質を限定。
歩行者ナビメッシュ用 OBJ を書き出す。"""
import glob, json, os, sys
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


def nav_opts():
    """maps.json "nav" of this build's map (meta.json name; the first entry of that
    name wins, as everywhere), {} when none. / この地図の maps.json "nav"。"""
    bd = os.path.dirname(os.path.abspath(mesh_dir.rstrip("/")))
    try:
        name = json.load(open(os.path.join(bd, "meta.json")))["name"]
        maps = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "maps.json")))
    except (OSError, ValueError, KeyError):
        return {}
    for v in maps.values():
        if isinstance(v, dict) and v.get("name") == name:
            return v.get("nav") or {}
    return {}


OPTS = nav_opts()
SKIP = {"road", "block"} if OPTS.get("walkable_only") else set()
GRASS_MATS = set(OPTS["grass_materials"]) if "grass_materials" in OPTS else None    # [] = no grass / 芝なし
BOX = None
if OPTS.get("clip_to_box"):
    _m = json.load(open(os.path.join(os.path.dirname(os.path.abspath(mesh_dir.rstrip("/"))), "meta.json")))
    BOX = (float(_m["half_x"]), float(_m["half_y"]))
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
    if area in SKIP:
        return
    used = np.unique(I)
    if not len(used):
        return
    remap = np.zeros(len(V), np.int64)
    remap[used] = np.arange(len(used))
    verts.append(np.stack([V[used, 0], V[used, 2], -V[used, 1]], 1))
    faces.setdefault(area, []).append(remap[I] + off + 1)
    off += len(used)


def cropped(V, I):
    if BOX is not None and len(I):
        c = V[I].mean(1)
        I = I[(abs(c[:, 0]) <= BOX[0]) & (abs(c[:, 1]) <= BOX[1])]
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
        if area == "grass" and GRASS_MATS is not None and k[:-2] not in GRASS_MATS:
            add("block", V, I)
            continue
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
