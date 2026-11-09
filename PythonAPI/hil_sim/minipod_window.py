#!/usr/bin/env python
"""A CARLA window for ONE LastMile mini delivery pod (the Mini Pods page's
"Open CARLA view").

The big view shows the pod itself from outside: a chase camera rides behind
and above the pod body (static.prop.lmhminipod*), so you watch the real robot
roll through the city, not only what it sees. A strip along the bottom shows
its six onboard cameras (front, rear, left, right, front-down kerb camera,
the payload camera under the lid), mounted where delivery/assets/
lmh_parts.json puts them (minipod.sensors_mounts; plausible offsets on the
0.70 x 0.54 x 0.67 m body when the file or a mount is missing). Keys switch
the big view:

  1 chase (behind the pod)   2 orbit (circles the pod)
  3 front  4 rear  5 left  6 right  7 front-down  8 lid (payload bay)
  Tab next view   H HUD on/off   F11 full screen   Esc / Q close

A HUD shows the pod id, status, hub / bay, battery, speed, order and damage
warnings (broken parts, sensor faults) from the delivery sim's live state
(data/delivery_live/state.json, written twice a second).

It follows the pod through its whole life: while it is docked in its hub's
bay the chase camera rides on the hub vehicle and the onboard cameras stop;
when the sim respawns the body actor every camera re-attaches to the new one.
A crashed pod may lie on its side, so the chase camera then frames it from a
level viewpoint instead of riding on it. It only reads the world (never ticks
it). Every sensor carries the role_name ``hil_podwin_<id>``; leftovers of a
killed window for the same pod are destroyed at start, and SIGTERM / closing
the window destroys them all.

For tests without mini pods in the sim, ``--actor <carla id>`` follows that
actor instead (any vehicle or prop).

Run (dashboard.py starts it through minipod_windows.py):
    python minipod_window.py --id 3 --host 127.0.0.1 -p 2000
    python minipod_window.py --id 99 --actor 1234 --snapshot /tmp/p.png --duration 12   # test

ミニポッド 1 台の CARLA ウィンドウ。大画面はロボットを後方から追うチェイス
カメラ、下段に機載カメラ 6 台。格納中はハブ車両を映し、本体が変われば付け直す。
終了時にセンサーをすべて破棄する。
"""

import argparse
import glob
import json
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

HERE = os.path.dirname(os.path.abspath(__file__))
PARTS_JSON = os.path.join(HERE, "delivery", "assets", "lmh_parts.json")

VIEWS = ("chase", "orbit", "front", "rear", "left", "right", "down", "lid")
ONBOARD = ("front", "rear", "left", "right", "down", "lid")
VIEW_NAME = {"chase": "CHASE", "orbit": "ORBIT", "front": "FRONT CAM", "rear": "REAR CAM", "left": "LEFT CAM",
             "right": "RIGHT CAM", "down": "KERB CAM (FRONT-DOWN)", "lid": "PAYLOAD CAM (LID)"}
IN_HUB = ("docked", "loading")
DOWN = ("crashed", "stuck")
STALE_S = 5.0
# Onboard cameras in the pod frame (CARLA vehicle frame: x forward, y right,
# z up, metres; pitch / yaw deg) and their lens. lmh_parts.json overrides the
# positions it knows (cam_front, cam_rear, cam_left, cam_right, cam_down_front).
# 機載カメラ（車両座標、lmh_parts.json があれば位置を上書き）
CAMERAS = {
    "front": {"x": 0.355, "y": 0.0, "z": 0.50, "pitch": -5.0, "yaw": 0.0, "fov": 90.0},
    "rear": {"x": -0.358, "y": 0.0, "z": 0.64, "pitch": -8.0, "yaw": 180.0, "fov": 90.0},
    "left": {"x": 0.08, "y": -0.28, "z": 0.58, "pitch": -10.0, "yaw": -90.0, "fov": 100.0},
    "right": {"x": 0.08, "y": 0.28, "z": 0.58, "pitch": -10.0, "yaw": 90.0, "fov": 100.0},
    "down": {"x": 0.32, "y": 0.0, "z": 0.60, "pitch": -50.0, "yaw": 0.0, "fov": 100.0},
    # under the lid, looking down into the payload bay / 蓋の裏から荷室を見下ろす
    "lid": {"x": -0.09, "y": 0.0, "z": 0.63, "pitch": -90.0, "yaw": 0.0, "fov": 110.0},
}
PARTS_KEY = {"front": "cam_front", "rear": "cam_rear", "left": "cam_left", "right": "cam_right", "down": "cam_down_front"}
# chase camera behind / above the pod body (the pod is 0.70 m long, 0.72 m tall)
# ポッド本体の後方上から
CHASE_TF = carla.Transform(carla.Location(x=-2.1, z=1.15), carla.Rotation(pitch=-20.0))
CHASE_FOV = 75.0
# on the hub vehicle while the pod is docked / 格納中はハブ車両に
HUB_TF = carla.Transform(carla.Location(x=-8.0, z=4.4), carla.Rotation(pitch=-22.0))
ORBIT_R, ORBIT_H, ORBIT_DEG_S = 2.4, 1.1, 18.0
GREEN, AMBER, RED, GREY, WHITE = (120, 255, 140), (255, 196, 64), (255, 80, 80), (150, 150, 150), (235, 235, 235)


def role(pod):
    return "hil_podwin_%d" % int(pod)


def load_mounts(path=PARTS_JSON):
    """CAMERAS with the positions / rotations of lmh_parts.json applied. / 取付位置。"""
    cams = {k: dict(v) for k, v in CAMERAS.items()}
    try:
        with open(path) as f:
            mp = json.load(f).get("minipod") or {}
    except (OSError, ValueError):
        return cams
    mounts, rots = mp.get("sensors_mounts") or {}, mp.get("sensors_rot_hint") or {}
    for name, key in PARTS_KEY.items():
        m = mounts.get(key)
        if isinstance(m, (list, tuple)) and len(m) == 3 and all(isinstance(v, (int, float)) for v in m):
            cams[name].update(x=float(m[0]), y=float(m[1]), z=float(m[2]))
        r = rots.get(key)
        if isinstance(r, (list, tuple)) and len(r) >= 2:
            # keep a slight downward look on the level cameras / 水平カメラは少し下向き
            cams[name].update(pitch=float(r[0]) if r[0] else cams[name]["pitch"], yaw=float(r[1]))
    return cams


def target_of(p, vans, actor=None):
    """What the outside cameras follow: ("body", actor) on the street,
    ("hub", hub actor) while docked, ("actor", id) for --actor tests.
    追う対象（本体・ハブ車両・指定アクター）。"""
    if actor:
        return "actor", actor
    if p is None:
        return None, None
    if p.get("status") in IN_HUB or not p.get("actor"):
        v = vans.get(p.get("hub"))
        if v is not None and v.get("actor"):
            return "hub", v["actor"]
    return ("body", p["actor"]) if p.get("actor") else (None, None)


def next_view(v, step=1):
    return VIEWS[(VIEWS.index(v) + step) % len(VIEWS)]


def level_follow(loc, yaw_deg, back=2.3, up=1.1):
    """A level chase transform behind ``loc`` along ``yaw`` (a pod on its side:
    no roll, no pitch from the body). / 本体の傾きを無視した追従位置。"""
    a = math.radians(yaw_deg)
    x, y, z = loc[0] - math.cos(a) * back, loc[1] - math.sin(a) * back, loc[2] + up
    return (x, y, z), (-math.degrees(math.atan2(up, back)), yaw_deg)


def orbit_pose(loc, t):
    a = math.radians((t * ORBIT_DEG_S) % 360.0)
    x, y, z = loc[0] + math.cos(a) * ORBIT_R, loc[1] + math.sin(a) * ORBIT_R, loc[2] + ORBIT_H
    yaw = math.degrees(math.atan2(loc[1] - y, loc[0] - x))
    return (x, y, z), (-math.degrees(math.atan2(ORBIT_H, ORBIT_R)), yaw)


def warnings_of(p):
    """Damage / fault lines for the HUD (red). / HUD の警告行。"""
    out = []
    dm = (p or {}).get("damage") or {}
    parts = dm.get("parts") or {}
    for k, v in sorted(parts.items(), key=lambda kv: kv[1] != "broken"):
        out.append("%s %s%s" % ("BROKEN" if v == "broken" else "DAMAGED", str(k).upper(),
                                " (%s)" % dm["why"] if dm.get("why") else ""))
    for k, s in sorted(((p or {}).get("sensors") or {}).items()):
        if isinstance(s, dict) and s.get("health") == "fail":
            out.append("FAULT %s" % str(s.get("label") or k).upper())
    if p and p.get("fail") and p.get("status") in DOWN:
        out.insert(0, "DOWN: %s" % p["fail"])
    return out


class Cam(object):
    """One camera sensor and its newest frame as a pygame surface. / カメラ 1 台。"""

    def __init__(self, world, size, fov, tf, parent, pod, tick=0.0):
        bp = world.get_blueprint_library().find("sensor.camera.rgb")
        bp.set_attribute("image_size_x", str(size[0]))
        bp.set_attribute("image_size_y", str(size[1]))
        bp.set_attribute("fov", str(fov))
        if tick > 0:
            bp.set_attribute("sensor_tick", str(tick))
        if bp.has_attribute("role_name"):
            bp.set_attribute("role_name", role(pod))
        self.surface, self.size = None, size
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


def sweep(client, world, pod):
    """Destroy sensors a killed window for this pod left behind. / 残骸の掃除。"""
    stale = [a.id for a in world.get_actors().filter("sensor.*") if a.attributes.get("role_name") == role(pod)]
    if stale:
        client.apply_batch([carla.command.DestroyActor(i) for i in stale])
        print("[minipod_window] removed %d sensor(s) left by an earlier window" % len(stale))
    return len(stale)


def cam_tf(c):
    return carla.Transform(carla.Location(c["x"], c["y"], c["z"]), carla.Rotation(pitch=c["pitch"], yaw=c["yaw"]))


class Window(object):
    def __init__(self, args):
        self.args = args
        self.cams = load_mounts(args.parts)
        self.size = tuple(int(x) for x in args.res.lower().split("x"))
        self.strip_h = max(80, int(self.size[1] * 0.18))
        self.big = (self.size[0], self.size[1] - self.strip_h)
        self.tile = (self.size[0] // len(ONBOARD), self.strip_h)
        pygame.init()
        pygame.font.init()
        self.display = pygame.display.set_mode(self.size, pygame.RESIZABLE | pygame.SCALED | pygame.DOUBLEBUF)
        pygame.display.set_caption("CARLA view - Mini pod %d" % args.id)
        mono = pygame.font.match_font("dejavusansmono,liberationmono,monospace")
        self.f = pygame.font.Font(mono, 15)
        self.fb = pygame.font.Font(mono, 22)
        self.fs = pygame.font.Font(mono, 12)
        self.client = carla.Client(args.host, args.port)
        self.client.set_timeout(4.0)
        self.world, self.world_id = None, None
        self.view = args.view
        self.hud = True
        self.main, self.main_key = None, None
        self.strip, self.strip_body = {}, None
        self.t0 = time.time()
        self.state, self.pod, self.vans = None, None, {}
        self.status_line = "connecting to CARLA..."

    # -- data / データ
    def read(self):
        st = delivery_live.read_json(os.path.join(self.args.live_dir, delivery_live.STATE))
        if not st or time.time() - st.get("wall_time", 0.0) > STALE_S:
            self.state, self.pod, self.vans = None, None, {}
            return
        self.state = st
        self.pod = next((p for p in st.get("minipods") or [] if p.get("id") == self.args.id), None)
        self.vans = {v.get("id"): v for v in st.get("vans") or []}

    def connect(self):
        w = self.client.get_world()
        if w.id != self.world_id:
            # new world (map switch, first look): old sensors are gone / 新しいワールド
            self.main, self.main_key, self.strip, self.strip_body = None, None, {}, None
            sweep(self.client, w, self.args.id)
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
        p = self.pod
        kind, aid = target_of(p, self.vans, self.args.actor)
        target = self.world.get_actor(aid) if aid else None
        if target is None or not target.is_alive:
            self.drop_main()
            self.drop_strip()
            self.status_line = ("actor %d is not in the world" % self.args.actor if self.args.actor else
                                "mini pod %d has no body in the world" % self.args.id if p else
                                "mini pod %d is not in the delivery sim" % self.args.id)
            return None
        down = bool(p and p.get("status") in DOWN)
        view = self.view
        if view in ONBOARD and kind == "hub":
            view = "chase"                  # docked: no onboard picture / 格納中
        if view == "chase" and kind == "hub":
            mount = ("rigid", HUB_TF, CHASE_FOV)
        elif view == "chase" and kind == "actor":
            # a test actor of any size: back off by its bounding box / 大きさに合わせる
            ext = target.bounding_box.extent
            back, up = max(2.1, ext.x + 2.2), max(1.15, 2.0 * ext.z + 0.8)
            mount = ("rigid", carla.Transform(carla.Location(x=-back, z=up),
                                              carla.Rotation(pitch=-math.degrees(math.atan2(up * 0.6, back)))), CHASE_FOV)
        elif view == "chase" and not down:
            mount = ("rigid", CHASE_TF, CHASE_FOV)
        elif view in ONBOARD:
            c = self.cams[view]
            mount = ("rigid", cam_tf(c), c["fov"])
        else:
            mount = ("free", None, CHASE_FOV)   # orbit, or a crashed pod / 周回・転倒
        key = (view, kind, aid, mount[0])
        if key != self.main_key:
            self.drop_main()
            parent = target if mount[0] == "rigid" else None
            tf = mount[1] if mount[1] is not None else target.get_transform()
            self.main = Cam(self.world, self.big, mount[2], tf, parent, self.args.id)
            self.main_key = key
        if mount[0] == "free":
            loc = target.get_location()
            q = (loc.x, loc.y, loc.z + 0.3)
            if view == "orbit":
                pos, rot = orbit_pose(q, time.time() - self.t0)
            else:
                yaw = float(p.get("yaw") or 0.0) if p else target.get_transform().rotation.yaw
                pos, rot = level_follow(q, yaw)
            self.main.move(pos, rot)
        # onboard strip: on the body while it is out of the hub / 機載カメラ（格納中は停止）
        if kind in ("body", "actor") and not self.args.no_strip:
            if self.strip_body != aid:
                self.drop_strip()
                for n in ONBOARD:
                    c = self.cams[n]
                    self.strip[n] = Cam(self.world, self.tile, c["fov"], cam_tf(c), target, self.args.id,
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
        for i, n in enumerate(ONBOARD):
            r = pygame.Rect(i * self.tile[0], self.big[1], self.tile[0], self.tile[1])
            cam = self.strip.get(n)
            if cam is not None and cam.surface is not None:
                self.display.blit(cam.surface, r.topleft)
            else:
                pygame.draw.rect(self.display, (16, 20, 30), r)
                self.text("IN HUB" if kind == "hub" else "no picture", r.center, GREY, self.fs, "center", False)
            sel = self.view == n
            pygame.draw.rect(self.display, GREEN if sel else (60, 70, 90), r, 2 if sel else 1)
            self.text("%d %s" % (VIEWS.index(n) + 1, n.upper()), (r.x + 6, r.y + 4), WHITE, self.fs)
        if not self.hud:
            return
        p = self.pod
        if p is None:
            if self.args.actor:
                self.text("TEST: following actor %d (mini pod %d not in the sim)" % (self.args.actor, self.args.id),
                          (12, 10), AMBER, self.fb)
            else:
                self.text("Mini pod %d: %s" % (self.args.id, "delivery sim not running" if self.state is None
                                               else "not in the sim"), (12, 10), AMBER, self.fb)
        else:
            bat = float(p.get("battery") or 0.0)
            hub = " HUB %s BAY %s" % (p.get("hub"), p.get("bay")) if p.get("hub") is not None else ""
            self.text("MINI POD %02d   %s%s" % (p["id"], str(p.get("status", "")).upper().replace("_", " "), hub),
                      (12, 10), WHITE, self.fb)
            spd = p.get("speed")
            line = "BAT %3.0f%%   SPD %s   ORDER %s   LID %s" % (
                bat, "%.1f km/h" % (float(spd) * 3.6) if spd is not None else "-",
                "#%s" % p["order"] if p.get("order") is not None else "-", str(p.get("lid", "-")).upper())
            self.text(line, (12, 42), GREEN if bat > 40 else AMBER if bat > 20 else RED)
            y = 70
            for w in warnings_of(p)[:8]:
                self.text(w, (12, y), RED)
                y += 22
        view = VIEW_NAME[self.view]
        if kind == "hub":
            view = "CHASE (on the hub)" if self.view == "chase" else "CHASE (docked: onboard cameras off)"
        self.text(view, (W - 12, 10), GREEN, self.f, "topright")
        self.text("1-8 view  Tab next  H hud  F11 full  Esc close", (W - 12, self.big[1] - 8), GREY, self.fs,
                  "bottomright")

    def close(self):
        self.drop_main()
        self.drop_strip()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--id", type=int, required=True, help="mini pod id (as on the dashboard)")
    ap.add_argument("--actor", type=int, default=0, help="follow this CARLA actor id instead (testing)")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("-p", "--port", type=int, default=2000)
    ap.add_argument("--res", default="1280x800", help="window size WxH (default 1280x800)")
    ap.add_argument("--view", default="chase", choices=VIEWS, help="first big view (default chase)")
    ap.add_argument("--fps", type=int, default=30, help="window redraw cap")
    ap.add_argument("--strip-fps", type=float, default=10.0, help="onboard strip camera rate")
    ap.add_argument("--no-strip", action="store_true", help="no onboard camera strip")
    ap.add_argument("--parts", default=PARTS_JSON, help="lmh_parts.json (camera mounts)")
    ap.add_argument("--live-dir", default=delivery_live.DEFAULT_DIR)
    ap.add_argument("--snapshot", default="", help="save the window to this PNG (testing)")
    ap.add_argument("--snapshot-after", type=float, default=6.0, help="seconds before the snapshot")
    ap.add_argument("--duration", type=float, default=0.0, help="close after this many seconds (testing)")
    args = ap.parse_args(argv)

    # SIGTERM (dashboard "close", the window replaced) unwinds through finally:
    # every sensor is destroyed / SIGTERM でも finally でセンサーを破棄
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
                    elif pygame.K_1 <= e.key <= pygame.K_8:
                        win.view = VIEWS[e.key - pygame.K_1]
            now = time.time()
            if now >= next_probe:            # 4 Hz: state + actors / 4 Hz で確認
                next_probe = now + 0.25
                win.read()
                try:
                    if win.pod is None and not args.actor:
                        if win.world is not None:
                            win.drop_main()
                            win.drop_strip()
                        kind = None
                    else:
                        win.connect()
                        kind = win.keep_cameras()
                except RuntimeError as exc:
                    print("[minipod_window] carla: %s" % exc)
                    win.world, win.world_id = None, None
                    win.main, win.main_key, win.strip, win.strip_body = None, None, {}, None
                    win.status_line = "CARLA unreachable - retrying..."
                    next_probe = time.time() + 3.0
            elif win.main is not None and win.main_key and win.main_key[3] == "free" and (win.pod or args.actor):
                try:                         # smooth orbit between probes / 周回を滑らかに
                    kind = win.keep_cameras()
                except RuntimeError:
                    pass
            win.draw(kind)
            pygame.display.flip()
            el = now - win.t0
            if args.snapshot and not snapped and el >= args.snapshot_after:
                pygame.image.save(win.display, args.snapshot)
                print("[minipod_window] snapshot -> %s" % args.snapshot)
                snapped = True
            if args.duration and el >= args.duration:
                return 0
            clock.tick(args.fps)
    except RuntimeError as exc:
        print("[minipod_window] %s" % exc)
        rc = 1
    finally:
        win.close()
        pygame.quit()
    return rc


if __name__ == "__main__":
    sys.exit(main())
