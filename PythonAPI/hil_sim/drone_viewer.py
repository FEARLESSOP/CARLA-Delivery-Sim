#!/usr/bin/env python
"""CCTV-style wall of drone cameras for the delivery sim.

One tile per drone. While a drone is in the air a small camera rides on it,
looking down past the side of the body so the parcel hangs in a corner of
the picture instead of blocking it. A drone that is docked, charging or
waiting has no camera at all (nothing is rendered for it) and its tile reads
OFFLINE with its status.

It reads delivery_sim's live state (data/delivery_live/state.json, written
twice a second) to learn which drones fly and which CARLA actor is each
drone's body, and only reads the world: it never ticks it.

Run (run_pipeline.py --drone-cams starts it):
    python drone_viewer.py --host 127.0.0.1 -p 2000 --res 1280x720

配送シミュレーション用の CCTV 風ドローンカメラ一覧。飛行中のドローンにだけ
下向きのカメラ（荷物が画面の隅に映るよう機体の横にずらす）を付け、
飛んでいない機体のタイルは OFFLINE と表示する（描画しない）。
"""

import argparse
import json
import math
import os
import time
import weakref

import carla
import numpy as np
import pygame
from pygame.locals import K_ESCAPE, K_F11, K_q

HERE = os.path.dirname(os.path.abspath(__file__))
STATE = os.path.join(HERE, "data", "delivery_live", "state.json")
AIRBORNE = ("delivering", "to_station", "to_van", "falling")
STATUS_TEXT = {"docked": "DOCKED IN VAN", "reloading": "RELOADING", "charging": "CHARGING",
               "standby": "STANDBY AT STATION", "crashed": "CRASHED", "delivering": "DELIVERING",
               "to_station": "TO STATION", "to_van": "TO VAN", "falling": "FALLING"}
# Camera on the drone body looking straight down, the top of the picture
# towards the drone's nose. It sits 25 cm forward and to the right of the
# centre, so the parcel under the drone (in its cradle, or lowered on the
# tether) shows towards the lower-left instead of in the middle.
# 真下向き・画面上が機首方向。中心から前右へ 25 cm ずらし、荷物は中央でなく左下に映る。
CAM_TF = carla.Transform(carla.Location(x=0.25, y=0.25, z=-0.05),
                         carla.Rotation(pitch=-90.0, yaw=0.0, roll=0.0))
CAM_FOV = 100.0
GREEN, AMBER, RED, GREY = (120, 255, 140), (255, 196, 64), (255, 70, 70), (150, 150, 150)


def grid_for(n):
    """Columns x rows for n tiles, as square as possible. / タイルの並び。"""
    cols = max(1, math.ceil(math.sqrt(n)))
    rows = max(1, math.ceil(n / cols))
    return cols, rows


def read_state():
    try:
        with open(STATE) as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


class DroneCam(object):
    """A camera riding on one drone body while it flies. / 飛行中の機体のカメラ。"""

    def __init__(self, world, body, size, tick):
        bp = world.get_blueprint_library().find("sensor.camera.rgb")
        bp.set_attribute("image_size_x", str(size[0]))
        bp.set_attribute("image_size_y", str(size[1]))
        bp.set_attribute("fov", str(CAM_FOV))
        if tick > 0.0:
            bp.set_attribute("sensor_tick", str(tick))
        self.body_id, self.size, self.surface = body.id, size, None
        self.sensor = world.spawn_actor(bp, CAM_TF, attach_to=body)
        weak = weakref.ref(self)
        self.sensor.listen(lambda im: DroneCam._on_image(weak, im))

    @staticmethod
    def _on_image(weak, image):
        self = weak()
        if self is None:
            return
        a = np.frombuffer(image.raw_data, dtype=np.uint8).reshape(image.height, image.width, 4)
        self.surface = pygame.surfarray.make_surface(a[:, :, 2::-1].swapaxes(0, 1))

    def destroy(self):
        try:
            self.sensor.stop()
            self.sensor.destroy()
        except RuntimeError:
            pass


class Wall(object):
    def __init__(self, size):
        pygame.init()
        pygame.font.init()
        self._flags = pygame.RESIZABLE | pygame.SCALED | pygame.DOUBLEBUF
        self.display = pygame.display.set_mode(size, self._flags)
        pygame.display.set_caption("Drone Cameras")
        self.size = size
        mono = pygame.font.match_font("dejavusansmono,liberationmono,monospace")
        self.f_small = pygame.font.Font(mono, 14)
        self.f_big = pygame.font.Font(mono, 26)
        self.noise = [self._noise_tile() for _ in range(4)]

    def _noise_tile(self):
        n = (np.random.rand(160, 90) * 38 + 12).astype(np.uint8)
        return pygame.surfarray.make_surface(np.stack([n, n, n], axis=-1))

    def toggle_fullscreen(self):
        pygame.display.toggle_fullscreen()

    def tile_rect(self, i, cols, rows):
        w, h = self.size[0] // cols, self.size[1] // rows
        return pygame.Rect((i % cols) * w, (i // cols) * h, w, h)

    def text(self, s, pos, color, big=False, anchor="topleft"):
        f = self.f_big if big else self.f_small
        img = f.render(s, True, color)
        r = img.get_rect(**{anchor: pos})
        shade = pygame.Surface((r.w + 8, r.h + 2), pygame.SRCALPHA)
        shade.fill((0, 0, 0, 140))
        self.display.blit(shade, (r.x - 4, r.y - 1))
        self.display.blit(img, r)

    def draw_tile(self, i, cols, rows, d, cam, sim_t):
        r = self.tile_rect(i, cols, rows)
        live = cam is not None and cam.surface is not None
        if live:
            self.display.blit(pygame.transform.smoothscale(cam.surface, r.size), r.topleft)
        else:
            self.display.blit(pygame.transform.scale(self.noise[(i + int(time.time() * 6)) % 4],
                                                     r.size), r.topleft)
            self.text("OFFLINE", r.center, GREY, big=True, anchor="center")
            if cam is not None:
                self.text("connecting...", (r.centerx, r.centery + 26), GREY, anchor="center")
            elif d is not None:
                self.text(STATUS_TEXT.get(d["status"], d["status"].upper()),
                          (r.centerx, r.centery + 26), GREY, anchor="center")
        pygame.draw.rect(self.display, (70, 70, 70), r, 1)
        if d is None:
            return
        # header: camera number, drone, van / 見出し
        van = ("VAN%d" % d["van"]) if d.get("van") else "----"
        self.text("CAM %02d  D%d  %s" % (i + 1, d["id"], van), (r.x + 6, r.y + 5), (230, 230, 230))
        clock_img = time.strftime("%H:%M:%S")
        self.text(clock_img, (r.right - 6, r.y + 5), (230, 230, 230), anchor="topright")
        if live:
            # blinking REC left of the clock / 時刻の左で点滅する REC
            rec_x = r.right - 6 - self.f_small.size(clock_img)[0] - 16
            self.text("REC", (rec_x, r.y + 5), RED, anchor="topright")
            if int(time.time() * 2) % 2 == 0:
                pygame.draw.circle(self.display, RED,
                                   (rec_x - self.f_small.size("REC")[0] - 12, r.y + 13), 5)
        # footer: status, altitude, battery, order, clock / 状態・高度・電池・注文・時刻
        bat = d.get("battery", 0.0)
        col = GREEN if bat > 40 else AMBER if bat > 20 else RED
        status = STATUS_TEXT.get(d["status"], d["status"].upper())
        if live and d.get("phase"):
            status += " / " + str(d["phase"]).upper()
        foot = "%s  ALT %.0fm  BAT %.0f%%" % (status, d.get("z", 0.0), bat)
        if d.get("order"):
            foot += "  #%d" % d["order"]
        self.text(foot, (r.x + 6, r.bottom - 6), col, anchor="bottomleft")

    def message(self, s):
        self.display.fill((10, 10, 10))
        self.text(s, (self.size[0] // 2, self.size[1] // 2), GREY, big=True, anchor="center")
        pygame.display.flip()


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("-p", "--port", type=int, default=2000)
    ap.add_argument("--res", default="1280x720", help="window size WxH (default 1280x720)")
    ap.add_argument("--fps", type=int, default=20, help="window redraw cap (default 20)")
    ap.add_argument("--cam-fps", type=float, default=10.0,
                    help="frame rate of each drone camera (default 10, CCTV-like). "
                         "Each flying drone's camera is a render pass on the server")
    ap.add_argument("--save", default="", help="also write the wall to this PNG every 2 s (testing)")
    args = ap.parse_args()
    size = tuple(int(x) for x in args.res.lower().split("x"))
    tick = 1.0 / args.cam_fps if args.cam_fps > 0 else 0.0

    wall = Wall(size)
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    cams = {}                    # drone id -> DroneCam / 機体 ID -> カメラ
    clock = pygame.time.Clock()
    world, state, next_probe, next_save = None, None, 0.0, 0.0
    try:
        while True:
            for e in pygame.event.get():
                if e.type == pygame.QUIT:
                    return
                if e.type == pygame.KEYDOWN:
                    if e.key in (K_ESCAPE, K_q):
                        return
                    if e.key == K_F11:
                        wall.toggle_fullscreen()
            now = time.time()
            probed = now >= next_probe
            if probed:                       # 4 Hz: state file + actor checks / 4 Hz で確認
                next_probe = now + 0.25
                state = read_state()
                try:
                    world = client.get_world() if world is None else world
                except RuntimeError:
                    world = None
            if state is None or not state.get("drones"):
                wall.message("Waiting for the delivery sim (no drones yet)...")
                clock.tick(5)
                continue
            if world is None:
                wall.message("CARLA unreachable - retrying...")
                clock.tick(5)
                continue
            drones = sorted(state["drones"], key=lambda d: d["id"])
            cols, rows = grid_for(len(drones))
            tile = wall.tile_rect(0, cols, rows).size
            # cameras only on drones in the air / カメラは飛行中の機体だけ
            for d in drones:
                cam = cams.get(d["id"])
                flying = d["status"] in AIRBORNE and d.get("actor")
                if cam is not None and (not flying or cam.body_id != d["actor"] or cam.size != tile):
                    cam.destroy()
                    cams.pop(d["id"])
                    cam = None
                if flying and cam is None and probed:
                    try:
                        body = world.get_actor(d["actor"])
                    except RuntimeError:
                        body, world = None, None      # CARLA gone: retry / 再接続
                        break
                    if body is not None and body.is_alive:
                        try:
                            cams[d["id"]] = DroneCam(world, body, tile, tick)
                        except RuntimeError as exc:
                            print("[drone_viewer] camera on D%d: %s" % (d["id"], exc))
            gone = set(cams) - {d["id"] for d in drones}
            for k in gone:
                cams.pop(k).destroy()
            wall.display.fill((0, 0, 0))
            for i, d in enumerate(drones):
                wall.draw_tile(i, cols, rows, d, cams.get(d["id"]), state.get("t", 0.0))
            pygame.display.flip()
            if args.save and now >= next_save:
                next_save = now + 2.0
                pygame.image.save(wall.display, args.save)
            clock.tick(args.fps)
    except RuntimeError as exc:
        print("[drone_viewer] %s" % exc)
    finally:
        for c in cams.values():
            c.destroy()
        pygame.quit()


if __name__ == "__main__":
    main()
