"""Write a RecastBuilder OBJ for a generated map: CARLA axes (x, z_up, y_carla = -y_map),
areas by material group (road / sidewalk / grass / block), like CARLA's FBX2OBJ.
All 'v' lines first: CARLA's MeshLoaderObj resets the material on every 'v' line,
so 'usemtl' must sit right before its faces.
  python nav_obj.py mesh_dir out.obj cw.npz [cx cy r]   (optional square crop for tests)
cw.npz comes from nav_crosswalks.py. Then RecastBuilder out.obj -> out.bin (guide ch. 7).
歩行者ナビメッシュ用 OBJ を書き出す。"""
import glob, os, sys
import numpy as np
mesh_dir, out = sys.argv[1], sys.argv[2]
crop = list(map(float, sys.argv[4:7])) if len(sys.argv) > 6 else None
# crosswalk hulls from nav_crosswalks.py: road faces under them are replaced
import shapely
from shapely import wkt
CW = np.load(sys.argv[3])
cw_union = shapely.union_all([wkt.loads(h) for h in CW["hulls"]])
shapely.prepare(cw_union)
AREA = {"Road_Road": "road", "Road_Curb": "road", "Road_Crosswalk": "crosswalk",
        "Road_Sidewalk": "sidewalk", "Road_Grass": "grass", "Terrain_Street": "block"}
verts, faces, off = [], {}, 0
for f in sorted(glob.glob(os.path.join(mesh_dir, "*.npz"))):
    area = AREA.get(os.path.basename(f).rsplit("_", 2)[0])
    if not area:
        continue
    d = np.load(f)
    for k in d.files:
        if not k.endswith("|V"):
            continue
        V, I = d[k], d[k[:-2] + "|I"]
        if crop:
            c = V[I].mean(1)
            I = I[(abs(c[:, 0] - crop[0]) < crop[2]) & (abs(c[:, 1] - crop[1]) < crop[2])]
        if area == "road":
            c = V[I].mean(1)
            I = I[~shapely.contains_xy(cw_union, c[:, 0], c[:, 1])]
        used = np.unique(I)
        if not len(used):
            continue
        remap = np.zeros(len(V), np.int64)
        remap[used] = np.arange(len(used))
        verts.append(np.stack([V[used, 0], V[used, 2], -V[used, 1]], 1))
        faces.setdefault(area, []).append(remap[I] + off + 1)
        off += len(used)
HV, HI = CW["V"], CW["I"]
if crop:
    c = HV[HI].mean(1)
    HI = HI[(abs(c[:, 0] - crop[0]) < crop[2]) & (abs(c[:, 1] - crop[1]) < crop[2])]
verts.append(np.stack([HV[:, 0], HV[:, 2], -HV[:, 1]], 1))
faces["crosswalk"] = [HI + off + 1]
with open(out, "w") as fo:
    np.savetxt(fo, np.concatenate(verts), fmt="v %.3f %.3f %.3f")
    for area, parts in faces.items():
        fo.write("o %s\nusemtl %s\n" % (area, area))
        np.savetxt(fo, np.concatenate(parts), fmt="f %d %d %d")
        print(area, sum(len(p) for p in parts), "triangles")
