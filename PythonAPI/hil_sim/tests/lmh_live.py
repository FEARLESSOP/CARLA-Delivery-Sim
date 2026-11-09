#!/usr/bin/env python3
"""Live check of one LastMile Hub + its Mini Pods in a RUNNING world (never
ticks: the traffic script owns the tick; wait_for_tick only).

Spawns one hub (the LMH-3, or the LMP-4 stand-in) parked at a free kerb point,
gives its crew two front-drop orders at roadside addresses nearby, follows one
full cycle (scan .. release, pods out on the kerb side, drop, back, reverse up
into the bays) for up to --duration sim seconds, logs it, optionally saves two
top-down camera frames, and destroys everything it made.

  ../../carla-venv/bin/python tests/lmh_live.py [--duration 240] [--shots DIR]

稼働中のワールドで LastMile ハブ 1 台とミニポッドを 1 サイクル動かす確認（tick しない）。
"""
import argparse
import math
import os
import random
import signal
import sys
import time
import types

HIL = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HIL)
os.chdir(HIL)

import carla  # noqa: E402
import delivery_sim as ds  # noqa: E402
from delivery import lmh_hub, lmh_rig as L  # noqa: E402
from delivery.battery import Battery, PodEnergyModel  # noqa: E402
from delivery.planning import Building, Order, FACE_TO_FACE  # noqa: E402


class LiveHub(lmh_hub.LmhVanMixin):
    """Just enough of a delivery_sim Van for the mixin (parked, no driving)."""

    def __init__(self, world, client, car, args, shared, real):
        self.vid, self.args, self.world, self.car = 9, args, world, car
        self.state, self.pending, self.failed, self.delivered = lmh_hub.LMH_WORK, [], [], []
        self.pem, self.energy = PodEnergyModel(), Battery(90.0)
        self.fleet, self.boxes, self.hubs, self.stops = None, None, [], []
        self.rows = []
        self.lmh_init(client, 100, shared, real)

    def log(self, t, event, order=None, batch=None, pos=None, detail="", van=None):
        self.rows.append((round(t, 1), event, detail))
        print("  %6.1f  %-14s %s" % (t, event, detail), flush=True)

    def finish_order(self, t, o, pos):
        self.pending.remove(o)
        self.delivered.append(o)

    def fail_order(self, t, o, reason):
        self.pending.remove(o)
        self.failed.append(o)
        self.log(t, "failed", detail="#%d %s" % (o.id, reason))

    def hold(self):
        self.car.apply_control(carla.VehicleControl(brake=1.0, hand_brake=True))

    def car_pos(self):
        l = self.car.get_location()
        return (l.x, l.y, l.z)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--duration", type=float, default=300.0)
    ap.add_argument("--shots", default="")
    ap.add_argument("--seed", type=int, default=5)
    a = ap.parse_args()
    args = ds.parse_args(["--lmh", "1", "--vans", "0", "--passive", "--seed", str(a.seed)])
    client = carla.Client(a.host, a.port)
    client.set_timeout(30.0)
    world = client.get_world()
    wmap = world.get_map()
    rng = random.Random(a.seed)
    left = ds.keep_left_map(wmap)
    print("map %s, keep-left %s, sync %s" % (wmap.name, left, world.get_settings().synchronous_mode), flush=True)
    made, hub, cams = [], None, []
    stop = {"now": False}
    signal.signal(signal.SIGTERM, lambda *_: stop.update(now=True))
    signal.signal(signal.SIGINT, lambda *_: stop.update(now=True))
    try:
        # a free kerb spot with two roadside addresses 25-70 m away / 空いた路肩と近くの 2 住所
        cands = ds.hub_candidates(wmap)
        addrs = ds.find_addresses(wmap)
        others = [v.get_location() for v in world.get_actors().filter("vehicle.*")]
        rng.shuffle(cands)
        router = lmh_hub.KerbRouter(wmap, left, inset=args.minipod_inset)
        spot, picks = None, None
        for c in cands[:400]:
            if any(math.hypot(o.x - c[0], o.y - c[1]) < 25.0 for o in others):
                continue
            near = [d for d in addrs if 25.0 < math.hypot(d[0][0] - c[0], d[0][1] - c[1]) < 70.0]
            # short kerb routes only, so one cycle fits the run / 1 サイクルが収まる近さ
            near = [d for d in near if (lambda r: r and L.path_length(r) < 130.0)(router.route(c, d[0]))]
            if len(near) >= 2:
                spot, picks = c, rng.sample(near, 2)
                break
        if spot is None:
            raise SystemExit("no free kerb spot / 空いた路肩がない")
        wp = wmap.get_waypoint(carla.Location(*spot))
        bp, real = lmh_hub.hub_blueprint(world)
        if bp.has_attribute("role_name"):
            bp.set_attribute("role_name", "lmh_test")
        tf = carla.Transform(wp.transform.location + carla.Location(z=0.3), wp.transform.rotation)
        car = world.try_spawn_actor(bp, tf)
        if car is None:
            raise SystemExit("hub spawn failed / ハブを配置できない")
        made.append(car)
        print("hub %s (%s) at (%.1f, %.1f) yaw %.0f" % (bp.id, "LMH-3" if real else "stand-in",
                                                      spot[0], spot[1], wp.transform.rotation.yaw), flush=True)
        for _ in range(40):                       # let it land / 着地を待つ
            world.wait_for_tick(10.0)
        shared = {"keep_left": left, "router": lmh_hub.KerbRouter(wmap, left, inset=args.minipod_inset),
                  "obstacles": lmh_hub.ObstacleCache(world, exclude=[car.id]), "probe": lmh_hub.MiniPodProbe()}
        hub = LiveHub(world, client, car, args, shared, real)
        shared["probe"].start(a.host, a.port, lambda: hub.crew.pods)
        orders = []
        for i, (drop, curb) in enumerate(picks):
            b = Building(900 + i, "address %d" % (i + 1), drop, curb)
            orders.append(Order(900 + i, FACE_TO_FACE, b, 0.0, 1))
            print("order #%d: drop (%.1f, %.1f), %.0f m away" % (900 + i, drop[0], drop[1],
                                                                 math.hypot(drop[0] - spot[0], drop[1] - spot[1])))
        hub.pending = list(orders)
        snap = world.get_snapshot()
        t0 = snap.timestamp.elapsed_seconds
        t_last, t = 0.0, 0.0
        hub.crew.start(0.0, orders)
        shot_at = {}
        if a.shots:
            os.makedirs(a.shots, exist_ok=True)
            cb = world.get_blueprint_library().find("sensor.camera.rgb")
            cb.set_attribute("image_size_x", "1280")
            cb.set_attribute("image_size_y", "720")
            cb.set_attribute("fov", "70")
            # from the street side, above, looking at the bays and the kerb / 道路側の上から
            cam = world.spawn_actor(cb, carla.Transform(carla.Location(x=-6.0, y=6.0, z=8.0),
                                                        carla.Rotation(pitch=-38.0, yaw=-53.0)), attach_to=car)
            cams.append(cam)
            want = {"frame": None}
            cam.listen(lambda img: (img.save_to_disk(want["frame"]), want.update(frame=None))
                       if want["frame"] else None)
        missed, frame, worst = 0, None, 0.0
        wall0 = time.time()
        home = car.get_location()
        moved = 0.0
        while not stop["now"] and t < a.duration:
            f = world.wait_for_tick(10.0).frame
            if frame is not None and f > frame + 1:
                missed += f - frame - 1
            frame = f
            t = world.get_snapshot().timestamp.elapsed_seconds - t0
            dt, t_last = max(t - t_last, 0.0), t
            w0 = time.perf_counter()
            shared["obstacles"].refresh(t, [p.pos for p in hub.crew.pods if p.out()])
            hub.hold()
            hub.lmh_tick(t, dt)
            worst = max(worst, (time.perf_counter() - w0) * 1000.0)
            here = car.get_location()
            moved = max(moved, math.hypot(here.x - home.x, here.y - home.y))
            if a.shots:
                for key, cond in (("release", 24.5 < t), ("out", any(p.status == L.TO_DOOR for p in hub.crew.pods)),
                                  ("docking", any(p.status == L.DOCKING for p in hub.crew.pods))):
                    if cond and key not in shot_at and want["frame"] is None:
                        shot_at[key] = t
                        want["frame"] = os.path.join(a.shots, "lmh_%s.png" % key)
            if int(t * 2) != int((t - dt) * 2) and int(t) % 10 == 0 and int(t * 2) % 2 == 0:
                print("  %6.1f  [wall %4.0f s] phase %-9s %s" % (t, time.time() - wall0, hub.crew.phase, "  ".join(
                    "P%d %s (%.1f,%.1f) %.1fm/s %.0f%%" % (p.id, p.status, p.x, p.y, p.speed, p.battery)
                    for p in hub.crew.pods)), flush=True)
            if t > 40.0 and not hub.crew.busy():
                break
        print("\nresult: delivered %d / failed %d in %.0f s sim; pods %s; missed ticks %d; worst step %.1f ms; "
              "routes %d (last %.0f ms); hub moved at most %.2f m while parked" % (
                  len(hub.delivered), len(hub.failed), t, [p.status for p in hub.crew.pods], missed, worst,
                  shared["router"].calls, shared["router"].ms, moved))
        states = hub.lmh_pod_states(t)
        print("published: %d minipods, keys %s, %d sensors each" % (
            len(states), sorted(states[0])[:8], len(states[0]["sensors"])))
        print("van fields: %s" % hub.lmh_van_state())
    finally:
        for c in cams:
            try:
                c.stop()
                c.destroy()
            except RuntimeError:
                pass
        if hub is not None:
            hub.lmh_cleanup()
        for x in made:
            try:
                x.destroy()
            except RuntimeError:
                pass
        try:
            world.wait_for_tick(10.0)         # the actor list catches up / 一覧の更新を待つ
        except RuntimeError:
            pass
        left_over = [x for x in world.get_actors() if x.attributes.get("role_name") in
                     ("lmh_test", lmh_hub.ROLE_POD, lmh_hub.ROLE_PARCEL)]
        client.apply_batch([carla.command.DestroyActor(x.id) for x in left_over])
        print("cleaned up (%d stragglers) / 片付け完了" % len(left_over), flush=True)


if __name__ == "__main__":
    main()
