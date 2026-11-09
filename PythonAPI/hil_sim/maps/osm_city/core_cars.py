"""Parked cars of an embedded model (maps.json "core") -> parking spots for
real CARLA vehicles (maps/.venv; runs Blender once to read the model).

  python core_cars.py <build dir>

The v8 model's Cars layer holds simple cars it placed where the 2024 GSI
aerial photo shows a car (source/cars.py), and the Oita Bus buses of the
depot. The user wants CARLA's own vehicles and their imported cars there
instead of the model's meshes (2026-11-08), so:
  - the layer is split into vehicles: its triangles filled from above on a
    0.15 m raster, one blob per vehicle; a blob two or more cars long or wide
    (cars touching) is split into that many spots along its axes;
  - each car -> {x, y, z, yaw, length, width} in the map frame (x scaled by
    core "x_scale", z lifted by core_z); a bus (M_oitabus) -> a rectangle the
    core cut keeps (the buses stay as the model's meshes);
  - spots inside the core rectangle only, and none on the new road logic's
    carriageway (core/road_masks.npz, made by core_fbx.py).
Writes <build>/parked_cars.json; hil_sim/parked_cars.py spawns the cars.

埋め込みモデルの駐車車両を、CARLA の車両を置く駐車位置に変える。
"""
import json
import math
import os
import subprocess
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

DUMP = r'''
import bpy, sys, numpy as np
a = sys.argv[sys.argv.index("--") + 1:]
bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.import_scene.fbx(filepath=a[0])
bpy.context.view_layer.update()
o = bpy.data.objects.get("Cars")
if o is None:
    np.savez_compressed(a[1], V=np.zeros((0, 3)), T=np.zeros((0, 3), int), TM=np.zeros(0, int), names=np.array([]))
else:
    mw = o.matrix_world
    V = np.array([(mw @ v.co)[:] for v in o.data.vertices], np.float32)
    o.data.calc_loop_triangles()
    T = np.array([list(t.vertices) for t in o.data.loop_triangles])
    TM = np.array([o.data.polygons[t.polygon_index].material_index for t in o.data.loop_triangles])
    np.savez_compressed(a[1], V=V, T=T, TM=TM, names=np.array([s.material.name if s.material else "" for s in o.material_slots]))
print("[core_cars] dumped")
'''

CAR_L, CAR_W = 4.4, 1.8        # a typical spot / 標準的な駐車枠


def obb(xy):
    """Centre, heading (deg, of the long axis), length, width of points."""
    m = xy.mean(0)
    _, _, vt = np.linalg.svd(xy - m, full_matrices=False)
    q = (xy - m) @ vt.T
    lo, hi = q.min(0), q.max(0)
    cen = m + vt[0] * (lo[0] + hi[0]) / 2 + vt[1] * (lo[1] + hi[1]) / 2
    return cen, math.degrees(math.atan2(vt[0][1], vt[0][0])), float(hi[0] - lo[0]), float(hi[1] - lo[1]), vt


def split_blob(cen, yaw, L, W, vt):
    """Spots for a blob of touching cars: as many along / across its axes as
    fit a car (cars side by side are 1.8 m wide, nose to tail 4.4 m long)."""
    # cars lie along the long axis when the blob is longer than wide by a car width or more
    na, nb = max(1, int(round(L / CAR_L))), max(1, int(round(W / CAR_W)))
    nc, nd = max(1, int(round(L / CAR_W))), max(1, int(round(W / CAR_L)))
    err_ab = abs(L - na * CAR_L) + abs(W - nb * CAR_W)
    err_cd = abs(L - nc * CAR_W) + abs(W - nd * CAR_L)
    spots = []
    if err_ab <= err_cd:
        for i in range(na):
            for j in range(nb):
                p = cen + vt[0] * ((i + 0.5) * L / na - L / 2) + vt[1] * ((j + 0.5) * W / nb - W / 2)
                spots.append((p, yaw, L / na, W / nb))
    else:
        for i in range(nc):
            for j in range(nd):
                p = cen + vt[0] * ((i + 0.5) * L / nc - L / 2) + vt[1] * ((j + 0.5) * W / nd - W / 2)
                spots.append((p, yaw + 90.0, W / nd, L / nc))
    return spots


def vehicles(V, T, TM, names):
    """[(kind, centre, yaw, length, width, zmin)] from the Cars layer."""
    from PIL import Image, ImageDraw
    from scipy import ndimage
    if not len(T):
        return []
    R = 0.15
    x0, y0 = V[:, 0].min() - 2, V[:, 1].min() - 2
    w, h = int((V[:, 0].max() - x0) / R) + 4, int((V[:, 1].max() - y0) / R) + 4
    im = Image.new("L", (w, h), 0)
    dr = ImageDraw.Draw(im)
    px, py = (V[:, 0] - x0) / R, (V[:, 1] - y0) / R
    for t in T:
        dr.polygon([(px[i], py[i]) for i in t], fill=1)
    lab, n = ndimage.label(np.array(im) > 0)
    vl = lab[np.clip(py.astype(int), 0, h - 1), np.clip(px.astype(int), 0, w - 1)]
    bus_mat = list(names).index("M_oitabus") if "M_oitabus" in list(names) else -1
    bus_v = np.zeros(len(V), bool)
    if bus_mat >= 0:
        bus_v[T[TM == bus_mat].ravel()] = True
    out = []
    for k in range(1, n + 1):
        idx = np.where(vl == k)[0]
        if len(idx) < 20:
            continue
        cen, yaw, L, W, vt = obb(V[idx, :2])
        z0 = float(V[idx, 2].min())
        if bus_v[idx].any():
            out.append(("bus", cen, yaw, L, W, z0))
        elif L <= 5.6 and W <= 2.3:
            out.append(("car", cen, yaw, L, W, z0))
        else:
            for p, yw, l, wd in split_blob(cen, yaw, L, W, vt):
                out.append(("car", p, yw, l, wd, z0))
    return out


def main():
    bd = os.path.abspath(sys.argv[1])
    meta = json.load(open(os.path.join(bd, "meta.json")))
    core = meta.get("core")
    if not core:
        print("[core_cars] no core in meta.json: nothing to do")
        return
    from core_merge import core_level, core_rect
    src = os.path.expanduser(core["fbx"])
    cd = os.path.join(bd, "core")
    os.makedirs(cd, exist_ok=True)
    cache = os.path.join(cd, "cars_layer.npz")
    if not os.path.exists(cache) or os.path.getmtime(cache) < os.path.getmtime(src):
        with tempfile.TemporaryDirectory() as td:
            sp = os.path.join(td, "dump.py")
            open(sp, "w").write(DUMP)
            subprocess.check_call([os.environ.get("BLENDER", "blender"), "-b", "--disable-autoexec", "-P", sp, "--",
                                   src, cache], stdout=subprocess.DEVNULL)
    d = np.load(cache)
    V = d["V"].astype(np.float64)
    V[:, 0] *= float(core.get("x_scale", 1.0))
    r = core_rect(meta)
    z = core_level(bd)
    masks = os.path.join(cd, "road_masks.npz")
    M = np.load(masks) if os.path.exists(masks) else None

    def on_road(cx, cy, yaw, L, W):
        if M is None:
            return False
        c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
        res, mx, my, cw = float(M["res"]), float(M["x0"]), float(M["y0"]), M["carriageway"]
        for a in (-L / 2, 0.0, L / 2):
            for b in (-W / 2, 0.0, W / 2):
                px, py = cx + c * a - s * b, cy + s * a + c * b
                i, j = int((py - my) / res), int((px - mx) / res)
                if 0 <= i < cw.shape[0] and 0 <= j < cw.shape[1] and cw[i, j]:
                    return True
        return False

    cars, buses, dropped = [], [], {"outside": 0, "on_road": 0}
    for kind, cen, yaw, L, W, z0 in vehicles(V, d["T"], d["TM"], d["names"]):
        if not (r[0] < cen[0] < r[2] and r[1] < cen[1] < r[3]):
            dropped["outside"] += 1
            continue
        rec = {"x": round(float(cen[0]), 2), "y": round(float(cen[1]), 2), "z": round(z0 + z, 2),
               "yaw": round(yaw, 1), "length": round(L, 2), "width": round(W, 2)}
        if kind == "bus":
            buses.append(rec)
        elif on_road(cen[0], cen[1], yaw, L, W):
            dropped["on_road"] += 1
        else:
            cars.append(rec)
    json.dump({"_doc": "parking spots from the embedded model's parked cars (osm_city/core_cars.py): map frame "
                       "(x east, y north, m; CARLA y = -y, yaw = -yaw), z = ground under the car; yaw along the "
                       "car's long axis (nose either way). hil_sim/parked_cars.py parks CARLA / user cars here. "
                       "buses: the model's own bus meshes, kept by core_fbx.py",
               "map": meta["name"], "frame": "map", "cars": cars, "buses": buses},
              open(os.path.join(bd, "parked_cars.json"), "w"), indent=0)
    print("[core_cars] %d parking spots, %d buses kept as meshes, %d outside the core, %d on the new carriageway left out"
          % (len(cars), len(buses), dropped["outside"], dropped["on_road"]))


if __name__ == "__main__":
    main()
