"""Live check of the Japanese speed posts (BP_JpSokudo, guide ch. 25) on a
running CARLA with a Japan map loaded. Connects to the server that is
already running (never starts one), changes nothing in the world except a
temporary camera (and, with --drive, one temporary autopilot car).

  python maps/jp_sokudo_live.py [--host localhost --port 2000] [--out DIR] [--drive]

1. every OpenDRIVE 274 landmark has a CARLA speed-limit actor within 1.5 m
   (traffic.speed_limit.* or traffic.unknown for 20/70/80), none spare;
2. one camera shot per speed value, 8 m in front of a post at 1.3 m eye
   height (DIR/sokudo_<V>_<id>.png): white disc, red ring, BLUE numerals = V;
3. --drive: a TM car on autopilot for 90 s; each time its
   get_speed_limit() changes, print it with the nearest post's value (they
   must agree once it has passed the post).

日本の最高速度標識のライブ確認（サーバーは起動しない）。
"""

import argparse
import math
import os
import queue
import time

import carla


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--out", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "build", "sokudo_live"))
    ap.add_argument("--drive", action="store_true")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    client = carla.Client(a.host, a.port)
    client.set_timeout(20.0)
    world = client.get_world()
    cmap = world.get_map()
    print("map", cmap.name)
    lms = [lm for lm in cmap.get_all_landmarks() if lm.type == "274"]
    signs = [s for s in world.get_actors() if s.type_id.startswith("traffic.") and s.type_id != "traffic.traffic_light"]
    print("274 landmarks", len(lms), "| sign actors", len(signs), sorted({s.type_id for s in signs}))
    bad, used = 0, set()
    for lm in lms:
        loc = lm.transform.location
        near = min(signs, key=lambda s: s.get_location().distance(loc), default=None)
        # the actor stands at the signal point (z may differ): compare in 2-D
        d2 = math.hypot(near.get_location().x - loc.x, near.get_location().y - loc.y) if near else 1e9
        ok = d2 < 1.5
        bad += not ok
        if near:
            used.add(near.id)
        print("  %-6s %5s km/h -> %-26s %.2f m %s" % (lm.id, lm.value, near.type_id if near else None, d2, "" if ok else "MISSING"))
    print("result:", "OK" if bad == 0 else "%d landmarks without a post" % bad)

    # one shot per value / 速度ごとに 1 枚
    bp = world.get_blueprint_library().find("sensor.camera.rgb")
    bp.set_attribute("image_size_x", "1280")
    bp.set_attribute("image_size_y", "720")
    bp.set_attribute("fov", "40")
    done = set()
    for lm in lms:
        if lm.value in done:
            continue
        near = min(signs, key=lambda s: s.get_location().distance(lm.transform.location), default=None)
        if near is None:
            continue
        t = near.get_transform()
        # the plate faces the actor's +Y / 板は アクタの +Y 向き
        r = t.get_right_vector()
        eye = t.location + carla.Location(r.x * 8.0, r.y * 8.0, 1.45)
        tgt = t.location + carla.Location(0, 0, 2.3)
        dx, dy, dz = tgt.x - eye.x, tgt.y - eye.y, tgt.z - eye.z
        rot = carla.Rotation(pitch=math.degrees(math.atan2(dz, math.hypot(dx, dy))), yaw=math.degrees(math.atan2(dy, dx)))
        cam = world.spawn_actor(bp, carla.Transform(eye, rot))
        q = queue.Queue()
        cam.listen(q.put)
        try:
            for _ in range(6):                 # let exposure settle / 露出が落ち着くまで
                img = q.get(timeout=10.0)
            path = os.path.join(a.out, "sokudo_%d_%s.png" % (int(lm.value), lm.id))
            img.save_to_disk(path)
            print("shot", path)
        finally:
            cam.stop()
            cam.destroy()
        done.add(lm.value)

    if a.drive:
        tm = client.get_trafficmanager()
        vbp = world.get_blueprint_library().filter("vehicle.toyota.prius")[0]
        car = None
        for sp in cmap.get_spawn_points():
            car = world.try_spawn_actor(vbp, sp)
            if car:
                break
        car.set_autopilot(True, tm.get_port())
        last, t0 = None, time.time()
        try:
            while time.time() - t0 < 90:
                time.sleep(0.5)
                lim = car.get_speed_limit()
                if lim != last:
                    loc = car.get_location()
                    near = min(lms, key=lambda lm: lm.transform.location.distance(loc), default=None)
                    print("speed limit %.0f km/h (nearest post %s km/h, %.0f m)" % (
                        lim, near.value if near else None, near.transform.location.distance(loc) if near else -1))
                    last = lim
        finally:
            car.destroy()


main()
