#!/usr/bin/env python3
"""Live check of the road logic overlay (road_overlay.py) in the running world.

  ../../carla-venv/bin/python maps/road_overlay_live.py [--seconds 20] [--radius 120]

Turns the overlay on for ``--seconds`` around the hero, then off, and saves two
camera shots from a temporary RGB camera behind the hero (overlay off, overlay
on) to maps/build/road_overlay_live/. Measures the server's frame rate before,
during and after, and the overlay's shapes and draw time per refresh.

- If road_overlay.py is running (fresh data/road_overlay_status.json) it only
  flips "on" in data/road_overlay.json and restores the old value afterwards.
- Otherwise (a pipeline started before the overlay existed) it draws in this
  process with the same code, so nothing else has to be restarted.
It never ticks the world, never changes settings, and destroys its camera on
exit (also on Ctrl+C / SIGTERM). The lines expire ~1.4 s after it stops.

道路ロジック表示のライブ確認。約 20 秒オンにしてからオフにし、カメラ画像
（オフ・オン）を保存する。tick はしない。カメラは必ず破棄する。
"""

import argparse
import math
import os
import queue
import signal
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
HIL = os.path.dirname(HERE)
sys.path.insert(0, HIL)

import carla  # noqa: E402
import road_overlay as ro  # noqa: E402

OUT = os.path.join(HERE, "build", "road_overlay_live")


def fps(world, seconds=3.0):
    """Server frames per real second (passive: waits for ticks). / サーバーの FPS"""
    s0 = world.get_snapshot()
    t0 = time.time()
    time.sleep(seconds)
    s1 = world.get_snapshot()
    dt = time.time() - t0
    return (s1.frame - s0.frame) / dt, (s1.timestamp.elapsed_seconds - s0.timestamp.elapsed_seconds) / dt


def shot(cam_q, path, timeout=10.0):
    """Save the next camera image. / 次のカメラ画像を保存"""
    try:
        while True:
            cam_q.get_nowait()                     # drop old frames / 古い画像は捨てる
    except queue.Empty:
        pass
    img = cam_q.get(timeout=timeout)
    img.save_to_disk(path)
    return path


def calibrate(world, hero, cam_q, values, ts):
    """Debug lines are self-lit and bloom: draw the same small scene at each
    channel value and save a shot of each. / 値ごとに線を描いて撮影する。"""
    wmap = world.get_map()
    dbg = world.debug
    for v in values:
        wp = wmap.get_waypoint(hero.get_location())
        pts = [wp]
        for _ in range(12):
            nx = pts[-1].next(2.0)
            if not nx:
                break
            pts.append(nx[0])
        cols = {"red": (v, 0, 0), "blue": (0.15 * v, 0.45 * v, v), "orange": (v, 0.5 * v, 0),
                "white": (v, v, v)}
        C = {k: carla.Color(*[int(round(c)) for c in rgb]) for k, rgb in cols.items()}
        for a, b in zip(pts, pts[1:]):
            for sgn in (1, -1):
                pa = a.transform.location + a.transform.get_right_vector() * (sgn * a.lane_width / 2)
                pb = b.transform.location + b.transform.get_right_vector() * (sgn * b.lane_width / 2)
                pa.z += 0.1
                pb.z += 0.1
                dbg.draw_line(pa, pb, thickness=0.04, color=C["red"], life_time=1.4, persistent_lines=False)
            la, lb = a.transform.location, b.transform.location
            dbg.draw_line(la + carla.Location(z=0.12), lb + carla.Location(z=0.12), thickness=0.03,
                          color=C["blue"], life_time=1.4, persistent_lines=False)
        far = pts[-1].transform
        r = far.get_right_vector()
        dbg.draw_line(far.location - r * 2 + carla.Location(z=0.12), far.location + r * 2 + carla.Location(z=0.12),
                      thickness=0.05, color=C["orange"], life_time=1.4, persistent_lines=False)
        mid = pts[len(pts) // 2].transform
        f, r = mid.get_forward_vector(), mid.get_right_vector()
        c = mid.location
        for a, b in ro.text_segments(str(int(v)), (c.x, c.y, c.z + 0.13), (r.x, r.y, 0), (f.x, f.y, 0), 2.4, 1.0):
            dbg.draw_line(carla.Location(*a), carla.Location(*b), thickness=0.05, color=C["white"],
                          life_time=1.4, persistent_lines=False)
        time.sleep(0.8)
        print("calibrate %g: %s" % (v, shot(cam_q, os.path.join(OUT, "%s_cal_%03d.png" % (ts, int(v))))))
        time.sleep(2.2)                          # let them expire / 消えるまで待つ


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--radius", type=float, default=120.0)
    ap.add_argument("--role", default="hero")
    ap.add_argument("--calibrate", default="",
                    help="brightness ladder instead of the overlay, e.g. 1,2,4,8,16: one shot per channel "
                         "value with red / blue / orange / white lines ahead of the hero / 明るさの確認")
    args = ap.parse_args()

    def on_term(_s, _f):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, on_term)
    os.makedirs(OUT, exist_ok=True)
    client = carla.Client(args.host, args.port)
    client.set_timeout(20.0)
    world = client.get_world()
    # a new client sees no actors until the next tick reaches it (synchronous
    # world ticked by the traffic script): wait up to 15 s / 次の tick まで待つ
    hero = None
    for _ in range(8):
        try:
            world.wait_for_tick(2.0)
        except RuntimeError:
            pass
        hero = ro.find_hero(world, args.role)
        if hero is not None:
            break
    if hero is None:
        print("no '%s' car in the world: start the driving window first / 自車がいません" % args.role)
        return 1
    print("map %s, hero %s (id %d)" % (world.get_map().name, hero.type_id, hero.id))
    helper = ro.helper_alive()
    old = ro.load_control()
    print("road_overlay.py %s" % ("is running: using the control file / ヘルパー動作中" if helper else
                                  "is not running: drawing in this process / このプロセスで描画"))
    cam = scam = None
    cam_q, scam_q = queue.Queue(), queue.Queue()
    ts = time.strftime("%Y%m%d_%H%M%S")
    stats = []
    try:
        bp = world.get_blueprint_library().find("sensor.camera.rgb")
        bp.set_attribute("image_size_x", "1280")
        bp.set_attribute("image_size_y", "720")
        bp.set_attribute("fov", "90")
        bp.set_attribute("role_name", "road_overlay_live")
        cam = world.spawn_actor(bp, carla.Transform(carla.Location(x=-7.0, z=4.5), carla.Rotation(pitch=-18.0)),
                                attach_to=hero)
        cam.listen(cam_q.put)
        if args.calibrate:
            calibrate(world, hero, cam_q, [float(v) for v in args.calibrate.split(",")], ts)
            return 0
        # a second camera where the CarlaUE4 window looks (draw_string labels
        # show only in the window itself) / サーバー画面と同じ位置のカメラ
        stf = world.get_spectator().get_transform()
        scam = world.spawn_actor(bp, stf)
        scam.listen(scam_q.put)
        sa = ro.spectator_area(world.get_map(), stf, args.radius)
        print("spectator at %.0f, %.0f, %.0f m above the road; its area r %.0f m, %.0f m from the hero"
              % (stf.location.x, stf.location.y, sa["height"], sa["r"],
                 math.hypot(sa["x"] - hero.get_location().x, sa["y"] - hero.get_location().y)))
        if helper and old.get("on"):              # measure "before" without it / 計測前はオフ
            ro.update_control({"on": False})
            time.sleep(4.0)
        wm = world.get_map()
        for name, at in (("hero", hero.get_location()), ("spectator area", carla.Location(sa["x"], sa["y"], 0.0))):
            wp_ = wm.get_waypoint(at)
            l_ = wp_.transform.location
            g_ = world.ground_projection(carla.Location(l_.x, l_.y, l_.z + 30.0), 80.0)
            print("%s: nearest lane %.1f m away, lane z - ground z = %s" % (
                name, math.hypot(l_.x - at.x, l_.y - at.y), "%.2f m" % (l_.z - g_.location.z) if g_ else "?"))
        f_before = fps(world)
        print("server before: %.1f frames/s (sim x%.2f) / 表示前" % f_before)
        print("shot (off): %s" % shot(cam_q, os.path.join(OUT, "%s_off.png" % ts)))
        print("shot (spectator, off): %s" % shot(scam_q, os.path.join(OUT, "%s_spec_off.png" % ts)))

        ov = None
        if not helper:
            t0 = time.time()
            ov = ro.Overlay(world, world.get_map())
            print("precompute %.2f s (incl. lights: %d)" % (time.time() - t0, len(ov.lights)))
        ctl = ro.clean_control(dict(old, on=True, radius=args.radius))
        if helper:
            ro.update_control({"on": True, "radius": args.radius})
        t_end = time.time() + args.seconds
        shot_at = time.time() + 4.0
        f_during, shot_path = None, None
        while time.time() < t_end:
            t = time.time()
            if ov is not None:
                h = hero.get_location()
                spec = ro.spectator_area(ov.wmap, world.get_spectator().get_transform(), ctl["radius"]) \
                    if ctl["spectator"] else None
                st = ov.refresh(ctl, (h.x, h.y, h.z), ov.wmap.get_waypoint(h),
                                life=ctl["refresh"] * 1.2 * max(0.2, f_before[1]), spectator=spec)
                stats.append(st)
            if shot_path is None and time.time() >= shot_at:
                shot_path = shot(cam_q, os.path.join(OUT, "%s_on.png" % ts))
                print("shot (on): %s" % shot_path)
                print("shot (spectator, on): %s" % shot(scam_q, os.path.join(OUT, "%s_spec_on.png" % ts)))
            elif f_during is None and shot_path is not None:
                f_during = fps(world, 3.0)
                print("server during: %.1f frames/s (sim x%.2f) / 表示中" % f_during)
            rest = ctl["refresh"] - (time.time() - t)
            if rest > 0:
                time.sleep(rest)
        if helper:
            hs = ro.read_status()
            print("helper status: %s" % {k: hs.get(k) for k in (
                "map", "shapes", "by_layer", "ms", "select_ms", "draw_ms", "capped", "precompute_s", "lights",
                "life", "sim_rate", "level", "night", "areas", "spectator_radius", "spectator_height")})
    except KeyboardInterrupt:
        print("interrupted / 中断")
    finally:
        if helper:
            ro.update_control({"on": False})

        for c in (cam, scam):
            if c is not None:
                try:
                    c.stop()
                    c.destroy()
                except RuntimeError:
                    pass
    time.sleep(2.0)                                 # the lines expire / 線が消えるまで
    time.sleep(2.0)
    print("server after: %.1f frames/s (sim x%.2f) / 表示後" % fps(world))
    if helper and old.get("on"):
        ro.update_control({"on": True})            # as the user had it / 元の状態に戻す
        print("overlay left on, as it was / 元どおりオン")
    if stats:
        n = [s["shapes"] for s in stats]
        print("in-process overlay: %d refreshes, shapes %d-%d (mean %.0f), select %.1f ms, draw %.1f ms "
              "(mean per refresh), capped %d times"
              % (len(stats), min(n), max(n), sum(n) / len(n), sum(s["select_ms"] for s in stats) / len(stats),
                 sum(s["draw_ms"] for s in stats) / len(stats), sum(bool(s["capped"]) for s in stats)))
        per_call = sum(s["draw_ms"] for s in stats) / max(1, sum(n)) * 1000.0
        print("draw call cost: %.1f us per shape (client side, async RPC)" % per_call)
    return 0


if __name__ == "__main__":
    sys.exit(main())
