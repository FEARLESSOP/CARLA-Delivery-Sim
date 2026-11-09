"""Embedded model (maps.json "core"): join the model, cut to its square, with
the generated city's FBX (maps/.venv python, runs Blender).

  python core_fbx.py <build dir>

1. cut (Blender): every mesh is first scaled in x by core "x_scale" (the v8
   model projected with 111319.49 m a degree, this frame with 111132.95:
   0.65 m at x = 384 otherwise); the ground layers (Roads, Ground,
   Road_Markings, Railway) are bisected at the rectangle core "bounds" and the
   outside removed; every model building goes (generated buildings replace
   them, layout.py; the user: all except the station building, which is its
   own object JR_Oita_City and stays); the pieces of the mixed layers
   (Street_Furniture, Trees, Cars) whose middle lies outside go;
   Traffic_Signals and Road_Signs go completely (static heads and boards
   placed for the model's own road logic: the new road logic's CARLA lights
   BP_JpShingo, stop/speed signals and the JARTIC plates cover the map);
   against the NEW road logic (layout.py carriageway / carriageway + kerb,
   rasterised at 0.5 m): a parked car, furniture piece or tree trunk with a
   vertex 0.2-4.5 m up over the carriageway is left out, the station building
   is reported if any part is over carriageway or kerb (core/clash.json);
   the parked cars go (core_cars.py turned them into parking spots,
   parked_cars.json: hil_sim/parked_cars.py parks CARLA / user cars there),
   only the Oita Bus buses stay as meshes;
2. prepare (prepare_oita.prepare_mesh_in_blender, as for the model alone):
   names without "sign"/"light", buildings joined per tile, the ground split
   into CARLA's Road_* kinds;
3. join (Blender): the prepared model lifted to the terrain level of its
   square (terrain_field.npz core_z) + the generated <Name>.fbx (kept as
   <Name>_gen.fbx) -> <Name>.fbx; the model's ground (roads, kerbs,
   pavements, lawn, markings, plaza floor) takes the generator's materials
   and UVs on the way (core_materials.py; the station building, railway,
   furniture, trees, buses and trains keep their own); the v8 shop-front
   props (bicycles, menu boards, door planters / pots, vending machines) of
   the replaced shops are left out (core_props.py).
Steps 1-2 are cached in <build>/core/ (redone when the model is newer).

埋め込みモデルを正方形で切り、生成した街の FBX と一つにする。
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

CUT = r'''
import bpy, bmesh, sys
from mathutils import Vector
a = sys.argv[sys.argv.index("--") + 1:]
src, dst = a[0], a[1]
X0, Y0, X1, Y1, SX = (float(v) for v in a[2:7])
import json, numpy as np
M = np.load(a[7])
RES, MX0, MY0 = float(M["res"]), float(M["x0"]), float(M["y0"])
def over(mask, pts):
    c = ((pts[:, 0] - MX0) / RES).astype(int); r = ((pts[:, 1] - MY0) / RES).astype(int)
    ok = (c >= 0) & (r >= 0) & (c < mask.shape[1]) & (r < mask.shape[0])
    return bool(mask[r[ok], c[ok]].any())
clash = {"cars_furniture_trees_dropped": 0, "buildings_over_road": [], "parked_car_pieces_replaced": 0}
BUSES = json.load(open(a[9])).get("buses", [])
def in_bus(x, y):
    import math
    for b in BUSES:
        c, s = math.cos(math.radians(b["yaw"])), math.sin(math.radians(b["yaw"]))
        dx, dy = x - b["x"], y - b["y"]
        if abs(dx * c + dy * s) <= b["length"] / 2 + 0.3 and abs(-dx * s + dy * c) <= b["width"] / 2 + 0.3:
            return True
    return False
bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.import_scene.fbx(filepath=src)
from mathutils import Matrix
SURFACE = ("Roads", "Ground", "Road_Markings", "Railway")
PIECES = ("Street_Furniture", "Trees", "Cars")
KEEP = ("JR_Oita_City",)     # model buildings that stay (the station) / 残す建物（駅ビル）
inside = lambda x, y: X0 < x < X1 and Y0 < y < Y1
gone = {"buildings": 0, "pieces": 0}
S = Matrix.Diagonal((SX, 1.0, 1.0, 1.0))
done = set()
for o in list(bpy.data.objects):
    if o.type != "MESH":
        continue
    if o.name in ("Traffic_Signals", "Road_Signs"):
        bpy.data.objects.remove(o); continue
    # bake the world transform with the x scale into the mesh / x 方向の縮尺をメッシュに焼き込む
    if o.data.users > 1 or o.data.name in done:
        o.data = o.data.copy()
    o.data.transform(S @ o.matrix_world)
    done.add(o.data.name)
    o.matrix_world = Matrix.Identity(4)
# every local matrix to identity too: prepare_oita unparents the buildings, and a
# child left holding the inverse of its parent's (-90 deg x, x100) became 100x
# too big / 親を外しても単位行列のままにする（建物が 100 倍になった）
for o in bpy.data.objects:
    o.matrix_parent_inverse = Matrix.Identity(4)
    o.matrix_basis = Matrix.Identity(4)
bpy.context.view_layer.update()      # bounds and world matrices are stale until an update / 更新するまで古い値
for o in list(bpy.data.objects):
    if o.type != "MESH":
        continue
    if o.parent and o.parent.name.startswith("Buildings_"):
        # every model building is replaced by a generated one (layout.py; the user,
        # 2026-11-08); the station building is its own object and stays
        # モデルの建物はすべて生成建物に置き換える（駅ビルは別の物体で残る）
        bpy.data.objects.remove(o); gone["buildings"] += 1
        continue
    if o.name in KEEP:
        P = np.array([v.co[:] for v in o.data.vertices])
        if len(P) and over(M["road_all"], P[P[:, 2] > 0.2]):
            clash["buildings_over_road"].append(o.name)
        continue
    if o.name in SURFACE:
        bm = bmesh.new(); bm.from_mesh(o.data)
        mw = o.matrix_world; mi = mw.inverted()
        for co, no in (((X1, 0, 0), (1, 0, 0)), ((X0, 0, 0), (-1, 0, 0)), ((0, Y1, 0), (0, 1, 0)), ((0, Y0, 0), (0, -1, 0))):
            pc = mi @ Vector(co)
            pn = (mi.to_3x3().transposed().inverted() @ Vector(no)).normalized()
            geom = bm.verts[:] + bm.edges[:] + bm.faces[:]
            bmesh.ops.bisect_plane(bm, geom=geom, plane_co=pc, plane_no=pn, clear_outer=True)
        bm.to_mesh(o.data); bm.free()
    elif o.name in PIECES:
        bm = bmesh.new(); bm.from_mesh(o.data)
        bm.verts.ensure_lookup_table()
        seen, dead = set(), []
        for v0 in bm.verts:
            if v0.index in seen:
                continue
            stack, part = [v0], []
            seen.add(v0.index)
            while stack:
                v = stack.pop(); part.append(v)
                for e in v.link_edges:
                    w = e.other_vert(v)
                    if w.index not in seen:
                        seen.add(w.index); stack.append(w)
            c = sum((o.matrix_world @ v.co for v in part), Vector()) / len(part)
            if not inside(c.x, c.y):
                dead += part; gone["pieces"] += 1
                continue
            if o.name == "Cars" and not in_bus(c.x, c.y):
                dead += part; clash["parked_car_pieces_replaced"] += 1    # CARLA cars park there / CARLA の車に置換
                continue
            P = np.array([v.co[:] for v in part])
            P = P[(P[:, 2] > 0.2) & (P[:, 2] < 4.5)]
            if len(P) and over(M["carriageway"], P):
                dead += part; clash["cars_furniture_trees_dropped"] += 1
        bmesh.ops.delete(bm, geom=dead, context="VERTS")
        bm.to_mesh(o.data); bm.free()
json.dump(clash, open(a[8], "w"), indent=1)
print("[core_fbx] parked cars replaced by spots (%d pieces), buses kept; against the new road logic: %d furniture/tree/bus pieces on the carriageway left out, %d buildings over carriageway or kerb%s"
      % (clash["parked_car_pieces_replaced"], clash["cars_furniture_trees_dropped"], len(clash["buildings_over_road"]),
         (": " + ", ".join(clash["buildings_over_road"][:8])) if clash["buildings_over_road"] else ""))
bpy.ops.object.select_all(action="SELECT")
bpy.ops.export_scene.fbx(filepath=dst, use_selection=False, apply_unit_scale=True, bake_space_transform=False,
                         mesh_smooth_type="FACE", path_mode="COPY", embed_textures=True)
print("[core_fbx] cut to x %.0f..%.0f, y %.0f..%.0f (x scale %.6f): %d model buildings and %d furniture/tree/car pieces outside "
      "left out (buildings: all, replaced by generated ones), signal heads and road boards removed" % (X0, X1, Y0, Y1, SX, gone["buildings"], gone["pieces"]))
'''

JOIN = r'''
import bpy, sys
a = sys.argv[sys.argv.index("--") + 1:]
core, gen, dst, z = a[0], a[1], a[2], float(a[3])
bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.import_scene.fbx(filepath=core)
core_objs = [o for o in bpy.data.objects]
for o in core_objs:
    if o.parent is None:
        o.location.z += z
bpy.ops.import_scene.fbx(filepath=gen)
# the model's ground in the generated city's materials (core_materials.py)
# モデルの地面を生成都市の材質に
sys.path.insert(0, a[4])
import core_materials
core_materials.apply(core_objs, a[5])
# no parked bicycles, menu boards, door pots / planters or vending machines in front
# of the (replaced) shops (core_props.py; shopfront_jp.py puts Japanese ones by the
# generated doors) / 店先の自転車・看板・鉢・自販機は除く（生成した入口に日本式を置く）
import core_props
core_props.drop_bicycles(core_objs)
core_props.drop_shopfront(core_objs, a[5])
core_props.drop_cut_texts(core_objs, a[5])
names = [o.name for o in bpy.data.objects if o.type == "MESH"]
bad = [n for n in names if "sign" in n.lower() or "light" in n.lower()]
assert not bad, "names with sign/light: %s" % bad[:5]
bpy.ops.object.select_all(action="SELECT")
bpy.ops.export_scene.fbx(filepath=dst, use_selection=True, object_types={"MESH"}, apply_unit_scale=True,
                         bake_space_transform=False, mesh_smooth_type="FACE", path_mode="COPY", embed_textures=True)
print("[core_fbx] joined: %d model objects (lifted %.2f m) + %d generated -> %s"
      % (len(core_objs), z, len(names) - len([o for o in core_objs if o.type == "MESH"]), dst))
'''


def blender(script, *args):
    with tempfile.TemporaryDirectory() as td:
        sp = os.path.join(td, "s.py")
        open(sp, "w").write(script)
        r = subprocess.run([os.environ.get("BLENDER", "blender"), "-b", "--disable-autoexec", "-P", sp, "--"]
                           + [str(a) for a in args], capture_output=True, text=True)
    for line in r.stdout.splitlines():
        if line.startswith(("[core_fbx]", "[core_props]", "[core_materials]")) or "Error" in line:
            print(line)
    if r.returncode:
        sys.exit("[core_fbx] Blender failed:\n" + r.stdout[-3000:] + r.stderr[-3000:])


def road_masks(bd, meta, r, out, res=0.5):
    """The new road logic's carriageway and carriageway + kerb over the core (+25 m),
    as 0.5 m rasters (layout.py, as check_roads.py). / 新しい道路の範囲（ラスタ）"""
    import numpy as np
    import shapely
    import layout
    maps = json.load(open(os.path.join(os.path.dirname(HERE), "maps.json")))
    ent = [v for v in maps.values() if isinstance(v, dict) and v.get("name") == meta["name"] and v.get("core")][0]
    osm = os.path.expanduser(ent["osm"])
    L = layout.Layout(meta, os.path.join(bd, "lanes.json"), osm)
    # 25 m past the edge: kept buildings reach up to ~20 m out / 範囲外 25 m まで
    m = 25.0
    x0, y0 = r[0] - m, r[1] - m
    xs = np.arange(x0, r[2] + m, res) + res / 2
    ys = np.arange(y0, r[3] + m, res) + res / 2
    GX, GY = np.meshgrid(xs, ys)
    cw = L.carriageway.buffer(-0.25)
    ra = L.road_all.buffer(-0.25)
    shapely.prepare(cw); shapely.prepare(ra)
    np.savez_compressed(out, res=res, x0=x0, y0=y0,
                        carriageway=shapely.contains_xy(cw, GX, GY), road_all=shapely.contains_xy(ra, GX, GY))


def main():
    bd = os.path.abspath(sys.argv[1])
    meta = json.load(open(os.path.join(bd, "meta.json")))
    core = meta.get("core")
    if not core:
        print("[core_fbx] no core in meta.json: nothing to do")
        return
    src = os.path.expanduser(core["fbx"])
    sys.path.insert(0, HERE)
    from core_merge import core_level, core_rect
    r = core_rect(meta)
    sx = float(core.get("x_scale", 1.0))
    z = core_level(bd)
    cd = os.path.join(bd, "core")
    os.makedirs(cd, exist_ok=True)
    cut, prep = os.path.join(cd, "core_cut.fbx"), os.path.join(cd, "core_prepared.fbx")
    stamp = os.path.join(cd, "stamp.json")
    import hashlib
    import prepare_oita
    code = hashlib.sha1((CUT + open(prepare_oita.__file__).read()
                         + open(os.path.join(HERE, "core_cars.py")).read()).encode()).hexdigest()
    want = {"src": src, "mtime": os.path.getmtime(src), "bounds": list(r), "x_scale": sx, "code": code}
    have = json.load(open(stamp)) if os.path.exists(stamp) else None
    masks = os.path.join(cd, "road_masks.npz")
    want["lanes_mtime"] = os.path.getmtime(os.path.join(bd, "lanes.json"))
    if have != want or not os.path.exists(prep):
        road_masks(bd, meta, r, masks)
        subprocess.check_call([sys.executable, os.path.join(HERE, "core_cars.py"), bd])
        blender(CUT, src, cut, *r, sx, masks, os.path.join(cd, "clash.json"), os.path.join(bd, "parked_cars.json"))
        if os.path.exists(prep):
            os.remove(prep)
        prepare_oita.prepare_mesh_in_blender(cut, prep)
        json.dump(want, open(stamp, "w"))
    name = meta["name"]
    fbx, gen = os.path.join(bd, name + ".fbx"), os.path.join(bd, name + "_gen.fbx")
    # a <Name>.fbx that is not our joined one is blender_build.py's: keep it
    # as the generated part / 結合済みでなければ生成部分として保存
    mark = os.path.join(cd, "joined_mtime")
    joined = os.path.exists(mark) and os.path.exists(fbx) and open(mark).read() == repr(os.path.getmtime(fbx))
    if os.path.exists(fbx) and not joined:
        shutil.move(fbx, gen)
    man = os.path.join(bd, "meshes", "manifest.json")
    if not os.path.exists(gen) or os.path.getmtime(gen) < os.path.getmtime(man):
        sys.exit("[core_fbx] the generated FBX is older than the meshes (blender_build.py failed?): not joining")
    import core_materials
    core_materials.prepare(bd, prep)      # asphalt cut, plaza/止まれ textures / 車道分割・独自テクスチャ
    import core_props
    core_props.ensure_walls(bd, src, sx)  # v8 wall feet for drop_shopfront / 店先小物の判定用の壁足
    blender(JOIN, prep, gen, fbx, z, HERE, bd)
    open(mark, "w").write(repr(os.path.getmtime(fbx)))


if __name__ == "__main__":
    main()
