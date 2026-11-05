"""Live check of the pedestrian signals (after the import, CARLA running on a
Japan map with generate_traffic driving the heads). Read-only except for
one camera sensor it spawns and destroys; never starts a server.

  carla-venv/bin/python hokousha_live.py [--crossing ID] [--seconds 120] [--out DIR]

One frame per second from 8 m in front of a head of the crossing (1.6 m
up, fov 50), named <t>_<phase>_<steps>_<conflict states>.png, plus
walkers.csv (walker ids inside the crossing's waiting zones / on the band
with their speed). Check by eye: the lit lamp = phase (lower cyan walk,
blinking in flash, upper red stop), the column goes down, walk only while
every conflicting vehicle light is red, walkers stand at the kerb on red /
flash and cross on walk.

歩行者用信号のライブ確認（カメラ 1 台を出して消すだけ）。
"""

import argparse
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import carla  # noqa: E402
import hokousha as hk  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--crossing", type=int, default=None)
    ap.add_argument("--seconds", type=float, default=120.0)
    ap.add_argument("--out", default=os.path.join(HERE, "build", "hokousha_live"))
    a = ap.parse_args()
    cl = carla.Client(a.host, a.port)
    cl.set_timeout(20.0)
    world = cl.get_world()
    data = hk.load(world.get_map().name)
    if data is None:
        sys.exit("no hokousha.json for %s" % world.get_map().name)
    ps = hk.PedSignals(world, data, [], send_heads=False)     # same phase rule, no RPC / 同じ規則
    print(ps.describe())
    crs = [c for c in ps.crossings if c.rec.get("heads")]
    cr = next((c for c in crs if c.id == a.crossing), crs[0] if crs else None)
    if cr is None:
        sys.exit("no signalised crossing with heads")
    h = cr.rec["heads"][0]
    fx, fy = h["face"]
    loc = carla.Location(h["x"] + fx * 8.0, h["y"] + fy * 8.0, h["z"] + 1.6)
    yaw = math.degrees(math.atan2(-fy, -fx))
    pitch = math.degrees(math.atan2(h["z"] + 3.1 - loc.z, 8.0))
    bp = world.get_blueprint_library().find("sensor.camera.rgb")
    for k, v in (("image_size_x", "960"), ("image_size_y", "720"), ("fov", "50")):
        bp.set_attribute(k, v)
    cam = world.spawn_actor(bp, carla.Transform(loc, carla.Rotation(pitch=pitch, yaw=yaw)))
    os.makedirs(a.out, exist_ok=True)
    last = {"img": None}
    cam.listen(lambda img: last.__setitem__("img", img))
    csv = open(os.path.join(a.out, "walkers.csv"), "w")
    csv.write("t,phase,walker,where,speed\n")
    conflict = [tl for k in sorted(cr.X) for tl in cr.j.phases[k]]
    t0 = time.time()
    try:
        while time.time() - t0 < a.seconds:
            snap = world.get_snapshot()
            ps.next_t = None
            ps.update(snap)
            st = "".join(hk._state_name(tl.get_state())[0] for tl in conflict)
            steps = int(math.ceil(cr.frac * hk.SEGS - 1e-6))
            t = time.time() - t0
            if last["img"] is not None:
                last["img"].save_to_disk(os.path.join(a.out, "%05.1f_%s_%d_%s.png" % (t, cr.phase, steps, st)))
            for w in world.get_actors().filter("walker.pedestrian.*"):
                p, v = w.get_location(), w.get_velocity()
                where = "band" if hk.in_quad(p.x, p.y, cr.band) else \
                    "kerb" if any(hk.in_quad(p.x, p.y, z) for z, _ in cr.zones) else None
                if where:
                    csv.write("%.1f,%s,%d,%s,%.2f\n" % (t, cr.phase, w.id, where, math.hypot(v.x, v.y)))
            if cr.phase == "walk" and st.strip("R"):
                print("MISMATCH at %.1f s: walk while a conflicting light is %s" % (t, st))
            time.sleep(1.0)
    finally:
        cam.stop()
        cam.destroy()
        csv.close()
    print("frames and walkers.csv in", a.out)


if __name__ == "__main__":
    main()
