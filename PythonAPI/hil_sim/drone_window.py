#!/usr/bin/env python
"""A CARLA window for ONE delivery drone (the dashboard's "Open CARLA view").

The big view shows the drone itself from outside: a chase camera rides behind
and above the drone body (static.prop.lmpdrone), so you watch the real 3D
drone fly through the city, not only what it sees. A strip along the bottom
shows its four onboard cameras (FPV gimbal, delivery camera looking down,
front and rear stereo, as in delivery/drone_sensors.CAMERAS). Keys switch the
big view:

  1 chase (behind the drone)    2 orbit (circles the drone)
  3 FPV   4 delivery cam (down)  5 front stereo   6 rear stereo
  Tab next view   H HUD on/off   F11 full screen   Esc / Q close

A HUD shows the drone id, status and phase, battery, altitude, speed, heading
and damage warnings (broken parts, sensor faults) from the delivery sim's
live state (data/delivery_live/state.json, written twice a second).

It follows the drone through its whole life: when it docks into its pod the
chase camera stays on the pod (rides on the pod car) and the onboard cameras
stop; when the sim respawns the body actor (launch, landing, crash) every
camera re-attaches to the new one. A crashed drone lies on its side, so the
chase camera then frames it from a level viewpoint instead of riding on it.
It only reads the world (never ticks it). Every sensor carries the role_name
``hil_dronewin_<id>``; leftovers of a killed window for the same drone are
destroyed at start, and SIGTERM / closing the window destroys them all.

Run (dashboard.py starts it through drone_windows.py):
    python drone_window.py --drone 3 --host 127.0.0.1 -p 2000
    python drone_window.py --drone 3 --snapshot /tmp/d3.png --duration 12   # test

配送ドローン 1 機の CARLA ウィンドウ。大画面は機体を後方から追うチェイスカメラ
（機体そのものを映す）、下段に機載カメラ 4 台。ポッド格納中はポッドを映し、
機体アクターが変われば付け直す。終了時にセンサーをすべて破棄する。
"""

import argparse
import glob
import math
import os
import signal
import sys
import time
import weakref

try:
    sys.path.append(glob.glob(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'carla', 'dist',
                                           'carla-*%d.%d-%s.egg' % (sys.version_info.major, sys.version_info.minor,
                                                                    'linux-x86_64')))[0])
except IndexError:
    pass

import carla  # noqa: E402
import numpy as np  # noqa: E402
import pygame  # noqa: E402

from delivery import live as delivery_live  # noqa: E402
from delivery.drone_sensors import CAMERAS, CAMERA_ORDER, IN_POD  # noqa: E402

VIEWS = ("chase", "orbit", "fpv", "down", "front", "rear")
VIEW_NAME = {"chase": "CHASE", "orbit": "ORBIT", "fpv": "FPV GIMBAL", "down": "DELIVERY CAM",
             "front": "FRONT STEREO", "rear": "REAR STEREO"}
DOWN = ("falling", "crashed")
STALE_S = 5.0
# chase camera behind / above the drone body (the drone is 0.54 m across)
# 機体の後方上から（機体の幅は 0.54 m）
CHASE_TF = carla.Transform(carla.Location(x=-2.3, z=1.05), carla.Rotation(pitch=-17.0))
CHASE_FOV = 72.0
# on the pod car while the drone is docked / 格納中はポッド車に
POD_TF = carla.Transform(carla.Location(x=-7.5, z=4.2), carla.Rotation(pitch=-22.0))
ORBIT_R, ORBIT_H, ORBIT_DEG_S = 2.8, 1.1, 18.0
GREEN, AMBER, RED, GREY, WHITE = (120, 255, 140), (255, 196, 64), (255, 80, 80), (150, 150, 150), (235, 235, 235)


def role(drone):
    return "hil_dronewin_%d" % int(drone)


def target_of(d, vans):
    """What the outside cameras follow: ("body", actor) in the air / on the
    ground, ("pod", van actor) while docked. / 追う対象（機体かポッド車）。"""
    if d is None:
        return None, None
    if d.get("status") in IN_POD:
        v = vans.get(d.get("van"))
        if v is not None and v.get("actor"):
            return "pod", v["actor"]
    return ("body", d["actor"]) if d.get("actor") else (None, None)


def next_view(v, step=1):
    return VIEWS[(VIEWS.index(v) + step) % len(VIEWS)]


def level_follow(loc, yaw_deg, back=2.6, up=1.1):
    """A level chase transform behind ``loc`` along ``yaw`` (for a drone on its
    side: no roll, no pitch from the body). / 機体の傾きを無視した追従位置。"""
    a = math.radians(yaw_deg)
    x, y, z = loc[0] - math.cos(a) * back, loc[1] - math.sin(a) * back, loc[2] + up
    pitch = -math.degrees(math.atan2(up, back))
    return (x, y, z), (pitch, yaw_deg)


def orbit_pose(loc, t):
    a = math.radians((t * ORBIT_DEG_S) % 360.0)
    x, y, z = loc[0] + math.cos(a) * ORBIT_R, loc[1] + math.sin(a) * ORBIT_R, loc[2] + ORBIT_H
    yaw = math.degrees(math.atan2(loc[1] - y, loc[0] - x))
    pitch = -math.degrees(math.atan2(ORBIT_H, ORBIT_R))
    return (x, y, z), (pitch, yaw)


def warnings_of(d):
    """Damage / fault lines for the HUD (red). / HUD の警告行。"""
    s = (d or {}).get("sensors") or {}
    out = []
    dm = s.get("damage") or {}
    for p, v in sorted((dm.get("parts") or {}).items(), key=lambda kv: kv[1].get("level") != "broken"):
        out.append("%s %s (%s)" % ("BROKEN" if v.get("level") == "broken" else "DAMAGED",
                                   p.upper(), v.get("why", "")))
    for k, v in s.items():
        if isinstance(v, dict) and v.get("health") == "fail" and k != "damage":
            out.append("FAULT %s" % k.upper())
    if d and d.get("fail") and d.get("status") in DOWN:
        out.insert(0, "DOWN: %s" % d["fail"])
    return out


class Cam(object):
    """One camera sensor and its newest frame as a pygame surface. / カメラ 1 台。"""

    def __init__(self, world, size, fov, tf, parent, drone, tick=0.0):
        bp = world.get_blueprint_library().find("sensor.camera.rgb")
        bp.set_attribute("image_size_x", str(size[0]))
        bp.set_attribute("image_size_y", str(size[1]))
        bp.set_attribute("fov", str(fov))
        if tick > 0:
            bp.set_attribute("sensor_tick", str(tick))
        if bp.has_attribute("role_name"):
            bp.set_attribute("role_name", role(drone))
        self.surface, self.size = None, size
        self.parent = parent.id if parent is not None else None
        self.sensor = world.spawn_actor(bp, tf, attach_to=parent) if parent is not None else world.spawn_actor(bp, tf)
        weak = weakref.ref(self)
        self.sensor.listen(lambda im: Cam._on_image(weak, im))

    @staticmethod
    def _on_image(weak, image):
        self = weak()
        if self is None:
            return
        a = np.frombuffer(image.raw_data, dtype=np.uint8).reshape(image.height, image.width, 4)
        self.surface = pygame.surfarray.make_surface(a[:, :, 2::-1].swapaxes(0, 1))

    def move(self, loc, rot):
        self.sensor.set_transform(carla.Transform(carla.Location(*loc), carla.Rotation(pitch=rot[0], yaw=rot[1])))

    def destroy(self, client):
        try:
            self.sensor.stop()
        except RuntimeError:
            pass
        try:
            self.sensor.destroy()
        except RuntimeError:
            try:
                client.apply_batch([carla.command.DestroyActor(self.sensor.id)])
            except RuntimeError:
                pass


def sweep(client, world, drone):
    """Destroy sensors a killed window for this drone left behind. / 残骸の掃除。"""
    stale = [a.id for a in world.get_actors().filter("sensor.*")
             if a.attributes.get("role_name") == role(drone)]
    if stale:
        client.apply_batch([carla.command.DestroyActor(i) for i in stale])
        print("[drone_window] removed %d sensor(s) left by an earlier window" % len(stale))
    return len(stale)


class Window(object):
    def __init__(self, args):
        self.args = args
        self.size = tuple(int(x) for x in args.res.lower().split("x"))
        self.strip_h = max(90, int(self.size[1] * 0.2))
        self.big = (self.size[0], self.size[1] - self.strip_h)
        self.tile = (self.size[0] // 4, self.strip_h)
        pygame.init()
        pygame.font.init()
        self.display = pygame.display.set_mode(self.size, pygame.RESIZABLE | pygame.SCALED | pygame.DOUBLEBUF)
        pygame.display.set_caption("CARLA view - Drone %d" % args.drone)
        mono = pygame.font.match_font("dejavusansmono,liberationmono,monospace")
        self.f = pygame.font.Font(mono, 15)
        self.fb = pygame.font.Font(mono, 22)
        self.fs = pygame.font.Font(mono, 12)
        self.client = carla.Client(args.host, args.port)
        self.client.set_timeout(4.0)
        self.world = None
        self.world_id = None
        self.view = args.view
        self.hud = True
        self.main = None            # the big camera / 大画面カメラ
        self.main_key = None        # (view, target kind, actor) it was made for / 生成条件
        self.strip = {}             # name -> Cam on the body / 機載カメラ
        self.strip_body = None
        self.t0 = time.time()
        self.state, self.drone, self.vans = None, None, {}
        self.status_line = "connecting to CARLA..."

    # -- data / データ
    def read(self):
        st = delivery_live.read_json(os.path.join(self.args.live_dir, delivery_live.STATE))
        if not st or time.time() - st.get("wall_time", 0.0) > STALE_S:
            self.state, self.drone, self.vans = None, None, {}
            return
        self.state = st
        self.drone = next((d for d in st.get("drones", []) if d.get("id") == self.args.drone), None)
        self.vans = {v.get("id"): v for v in st.get("vans", [])}

    def connect(self):
        w = self.client.get_world()
        if w.id != self.world_id:
            # new world (map switch, first look): old sensors are gone / 新しいワールド
            self.main, self.main_key, self.strip, self.strip_body = None, None, {}, None
            sweep(self.client, w, self.args.drone)
            self.world_id = w.id
        self.world = w
        return w

    # -- cameras / カメラ
    def drop_main(self):
        if self.main is not None:
            self.main.destroy(self.client)
        self.main, self.main_key = None, None

    def drop_strip(self):
        for c in self.strip.values():
            c.destroy(self.client)
        self.strip, self.strip_body = {}, None

    def keep_cameras(self):
        d = self.drone
        kind, aid = target_of(d, self.vans)
        target = self.world.get_actor(aid) if aid else None
        if target is None or not target.is_alive:
            self.drop_main()
            self.drop_strip()
            self.status_line = "drone %d has no body in the world" % self.args.drone if d else \
                "drone %d is not in the delivery sim" % self.args.drone
            return None
        down = d.get("status") in DOWN
        view = self.view
        onboard = view in CAMERAS
        if onboard and kind == "pod":
            view = "chase"                  # in the pod: no onboard picture / 格納中
        # how the big camera is mounted / 大画面カメラの取り付け方
        if view == "chase" and kind == "pod":
            mount = ("rigid", POD_TF, CHASE_FOV)
        elif view == "chase" and not down:
            mount = ("rigid", CHASE_TF, CHASE_FOV)
        elif view in CAMERAS:
            c = CAMERAS[view]
            mount = ("rigid", carla.Transform(carla.Location(c["x"], c["y"], c["z"]),
                                              carla.Rotation(pitch=c["pitch"], yaw=c["yaw"])), c["fov"])
        else:
            mount = ("free", None, CHASE_FOV)   # orbit, or a crashed drone / 周回・墜落機
        key = (view, kind, aid, mount[0])
        if key != self.main_key:
            self.drop_main()
            parent = target if mount[0] == "rigid" else None
            tf = mount[1] if mount[1] is not None else target.get_transform()
            self.main = Cam(self.world, self.big, mount[2], tf, parent, self.args.drone)
            self.main_key = key
        if mount[0] == "free":
            loc = target.get_location()
            p = (loc.x, loc.y, loc.z + 0.2)
            if view == "orbit":
                pos, rot = orbit_pose(p, time.time() - self.t0)
            else:
                pos, rot = level_follow(p, float(d.get("yaw") or 0.0))
            self.main.move(pos, rot)
        # onboard strip: on the body while it is out of the pod / 機載カメラ（格納中は停止）
        if kind == "body" and not self.args.no_strip:
            if self.strip_body != aid:
                self.drop_strip()
                for n in CAMERA_ORDER:
                    c = CAMERAS[n]
                    tf = carla.Transform(carla.Location(c["x"], c["y"], c["z"]),
                                         carla.Rotation(pitch=c["pitch"], yaw=c["yaw"]))
                    self.strip[n] = Cam(self.world, self.tile, c["fov"], tf, target, self.args.drone,
                                        tick=1.0 / self.args.strip_fps)
                self.strip_body = aid
        else:
            self.drop_strip()
        self.status_line = ""
        return kind

    # -- drawing / 描画
    def text(self, s, pos, color=WHITE, font=None, anchor="topleft", shade=True):
        f = font or self.f
        img = f.render(s, True, color)
        r = img.get_rect(**{anchor: pos})
        if shade:
            bg = pygame.Surface((r.w + 10, r.h + 4), pygame.SRCALPHA)
            bg.fill((0, 0, 0, 150))
            self.display.blit(bg, (r.x - 5, r.y - 2))
        self.display.blit(img, r)
        return r

    def draw(self, kind):
        self.display.fill((8, 11, 18))
        W, H = self.size
        if self.main is not None and self.main.surface is not None:
            self.display.blit(self.main.surface, (0, 0))
        else:
            self.text(self.status_line or "waiting for the first frame...", (W // 2, self.big[1] // 2),
                      GREY, self.fb, "center")
        # strip / 下段
        for i, n in enumerate(CAMERA_ORDER):
            r = pygame.Rect(i * self.tile[0], self.big[1], self.tile[0], self.tile[1])
            cam = self.strip.get(n)
            if cam is not None and cam.surface is not None:
                self.display.blit(cam.surface, r.topleft)
            else:
                pygame.draw.rect(self.display, (16, 20, 30), r)
                self.text("IN POD" if kind == "pod" else "no picture", r.center, GREY, self.fs, "center", False)
            sel = self.view == n
            pygame.draw.rect(self.display, GREEN if sel else (60, 70, 90), r, 2 if sel else 1)
            self.text("%d %s" % (VIEWS.index(n) + 1, VIEW_NAME[n]), (r.x + 6, r.y + 4), WHITE, self.fs)
        if not self.hud:
            return
        d = self.drone
        if d is None:
            self.text("Drone %d: %s" % (self.args.drone, "delivery sim not running" if self.state is None
                                         else "not in the sim"), (12, 10), AMBER, self.fb)
            return
        s = d.get("sensors") or {}
        bat = float(d.get("battery") or 0.0)
        status = str(d.get("status", "")).upper() + (" / %s" % str(d["phase"]).upper() if d.get("phase") else "")
        self.text("DRONE %02d   %s" % (d["id"], status), (12, 10), WHITE, self.fb)
        agl = (s.get("rangefinder") or {}).get("ground")
        spd = (s.get("gnss") or {}).get("speed")
        hdg = (s.get("mag") or {}).get("heading")
        line = "BAT %3.0f%%   ALT %5.1f m%s   SPD %s   HDG %s" % (
            bat, float(d.get("z") or 0.0), "  (AGL %.1f m)" % agl if agl is not None else "",
            "%.1f m/s" % spd if spd is not None else "-", "%03.0f" % hdg if hdg is not None else "-")
        self.text(line, (12, 42), GREEN if bat > 40 else AMBER if bat > 20 else RED)
        y = 70
        for w in warnings_of(d)[:8]:
            self.text(w, (12, y), RED)
            y += 22
        view = VIEW_NAME[self.view] + (" (on the pod)" if kind == "pod" and self.view in ("chase",) else "")
        if kind == "pod" and self.view in CAMERAS:
            view = "CHASE (in pod: onboard cameras off)"
        self.text(view, (W - 12, 10), GREEN, self.f, "topright")
        self.text("1-6 view  Tab next  H hud  F11 full  Esc close", (W - 12, self.big[1] - 8), GREY, self.fs,
                  "bottomright")

    def close(self):
        self.drop_main()
        self.drop_strip()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--drone", type=int, required=True, help="drone id (as on the dashboard)")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("-p", "--port", type=int, default=2000)
    ap.add_argument("--res", default="1280x800", help="window size WxH (default 1280x800)")
    ap.add_argument("--view", default="chase", choices=VIEWS, help="first big view (default chase)")
    ap.add_argument("--fps", type=int, default=30, help="window redraw cap")
    ap.add_argument("--strip-fps", type=float, default=10.0, help="onboard strip camera rate")
    ap.add_argument("--no-strip", action="store_true", help="no onboard camera strip")
    ap.add_argument("--live-dir", default=delivery_live.DEFAULT_DIR)
    ap.add_argument("--snapshot", default="", help="save the window to this PNG (testing)")
    ap.add_argument("--snapshot-after", type=float, default=6.0, help="seconds before the snapshot")
    ap.add_argument("--duration", type=float, default=0.0, help="close after this many seconds (testing)")
    args = ap.parse_args(argv)

    # SIGTERM (dashboard "close", the drone window replaced) unwinds through
    # finally: every sensor is destroyed / SIGTERM でも finally でセンサーを破棄
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    win = Window(args)
    clock = pygame.time.Clock()
    next_probe, snapped, kind = 0.0, False, None
    rc = 0
    try:
        while True:
            for e in pygame.event.get():
                if e.type == pygame.QUIT:
                    return 0
                if e.type == pygame.KEYDOWN:
                    if e.key in (pygame.K_ESCAPE, pygame.K_q):
                        return 0
                    if e.key == pygame.K_F11:
                        pygame.display.toggle_fullscreen()
                    elif e.key == pygame.K_h:
                        win.hud = not win.hud
                    elif e.key == pygame.K_TAB:
                        win.view = next_view(win.view, -1 if e.mod & pygame.KMOD_SHIFT else 1)
                    elif pygame.K_1 <= e.key <= pygame.K_6:
                        win.view = VIEWS[e.key - pygame.K_1]
            now = time.time()
            if now >= next_probe:            # 4 Hz: state + actors / 4 Hz で確認
                next_probe = now + 0.25
                win.read()
                try:
                    if win.drone is None:
                        # no sim / not in it: no cameras (CARLA not even asked) / 対象なし
                        if win.world is not None:
                            win.drop_main()
                            win.drop_strip()
                        kind = None
                    else:
                        win.connect()
                        kind = win.keep_cameras()
                except RuntimeError as exc:
                    print("[drone_window] carla: %s" % exc)
                    win.world, win.world_id = None, None
                    win.main, win.main_key, win.strip, win.strip_body = None, None, {}, None
                    win.status_line = "CARLA unreachable - retrying..."
                    next_probe = time.time() + 3.0
            elif win.main is not None and win.main_key and win.main_key[3] == "free" and win.drone:
                try:                         # smooth orbit between probes / 周回を滑らかに
                    kind = win.keep_cameras()
                except RuntimeError:
                    pass
            win.draw(kind)
            pygame.display.flip()
            el = now - win.t0
            if args.snapshot and not snapped and el >= args.snapshot_after:
                pygame.image.save(win.display, args.snapshot)
                print("[drone_window] snapshot -> %s" % args.snapshot)
                snapped = True
            if args.duration and el >= args.duration:
                return 0
            clock.tick(args.fps)
    except RuntimeError as exc:
        print("[drone_window] %s" % exc)
        rc = 1
    finally:
        win.close()
        pygame.quit()
    return rc


if __name__ == "__main__":
    sys.exit(main())
