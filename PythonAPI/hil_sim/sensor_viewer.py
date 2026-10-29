#!/usr/bin/env python
"""Pygame multi-sensor viewer that attaches to the CARLA hero vehicle.

Renders RGB camera (front/rear), semantic segmentation camera, LiDAR
top-down, and two depth-camera views (raw RGB encoding + grayscale) in
a single pygame window.

Unlike examples/visualize_multiple_sensors.py, this script does NOT
spawn its own vehicle and does NOT change synchronous mode — it waits
for the existing hero vehicle (role_name == 'hero') and attaches to it.
When the hero is replaced (e.g. after F5 map cycle) sensors are
re-attached automatically.

Run:
    python sensor_viewer.py --host 127.0.0.1 -p 2000 --res 1440x720

CARLAのheroビークルにアタッチするPygameマルチセンサービューア。

RGBカメラ（前方/後方）、セマンティックセグメンテーションカメラ、
LiDARトップダウン、および2つの深度カメラビュー（RAW RGBエンコーディング＋
グレースケール）を単一のpygameウィンドウに描画する。

examples/visualize_multiple_sensors.py とは異なり、本スクリプトは
自前の車両をスポーンせず、同期モードも変更しない — 既存のheroビークル
（role_name == 'hero'）を待機してアタッチする。heroが置き換えられた場合
（例: F5によるマップ切り替え後）、センサーは自動的に再アタッチされる。

実行方法:
    python sensor_viewer.py --host 127.0.0.1 -p 2000 --res 1440x720
"""

import argparse
import glob
import os
import sys
import time
import weakref

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import numpy as np
import pygame
from pygame.locals import K_ESCAPE, K_F11, K_q


class DisplayManager:
    def __init__(self, grid_size, window_size, caption="CARLA Sensor Viewer"):
        pygame.init()
        pygame.font.init()
        self._flags = pygame.RESIZABLE | pygame.SCALED | pygame.DOUBLEBUF
        self.display = pygame.display.set_mode(window_size, self._flags)
        pygame.display.set_caption(caption)
        self.grid_size = grid_size
        self.window_size = window_size
        self.panels = []
        self.is_fullscreen = False
        self._windowed_size = tuple(window_size)

    def toggle_fullscreen(self):
        if self.is_fullscreen:
            self.display = pygame.display.set_mode(
                self._windowed_size, self._flags)
            self.is_fullscreen = False
        else:
            self.display = pygame.display.set_mode(
                self._windowed_size,
                self._flags | pygame.FULLSCREEN)
            self.is_fullscreen = True

    def cell_size(self):
        return (int(self.window_size[0] / self.grid_size[1]),
                int(self.window_size[1] / self.grid_size[0]))

    def cell_offset(self, pos):
        cw, ch = self.cell_size()
        return (pos[1] * cw, pos[0] * ch)

    def add(self, panel):
        self.panels.append(panel)

    def render(self):
        self.display.fill((15, 18, 24))
        for p in self.panels:
            p.render(self.display)
        pygame.display.flip()

    def destroy_all(self):
        for p in self.panels:
            try:
                p.destroy()
            except Exception:
                pass
        self.panels = []


# Seconds between frames for every panel sensor; 0 = every simulation
# frame. Overwritten by main() from --sensor-fps.
# 各パネルのセンサーのフレーム間隔（秒）。0 で毎シミュレーションフレーム。
# main() が --sensor-fps から上書きする。
SENSOR_TICK = 1.0 / 20.0

# LiDAR ray-casting runs on the CARLA server's CPU (it is not a GPU pass),
# so beam count x points/s is a direct CPU cost. 64ch/250k was the sample
# default; a 480x360 top-down panel does not resolve that.
# LiDAR のレイキャストは CARLA サーバの CPU で実行される（GPUパスではない）
# ため、ビーム数×毎秒点数がそのままCPU負荷になる。64ch/250k はサンプルの
# 既定値だが、480x360 のトップダウン表示ではその分解能を活かせない。
LIDAR_OPTS = {'range': '80', 'channels': '32',
              'points_per_second': '120000',
              'rotation_frequency': '20'}


def bgra_surface(image):
    """CARLA camera image -> pygame Surface, without a numpy round-trip.

    CARLA delivers BGRA rows straight to SDL; .convert() then copies once
    into the display's pixel format so later blits are a plain memcpy. This
    replaces frombuffer -> slice -> reverse -> swapaxes -> make_surface,
    which cost ~1.2 ms of client CPU per 720p frame against ~0.2 ms here,
    for byte-identical pixels.

    CARLA のカメラ画像を numpy を経由せず pygame Surface に変換する。
    BGRA 行をそのまま SDL に渡し、.convert() でディスプレイのピクセル
    形式へ1回だけコピーするので、以後の blit は単純な memcpy になる。
    旧経路は 720p 1フレームあたり約 1.2 ms、本経路は約 0.2 ms で画素は同一。
    """
    try:
        surface = pygame.image.frombuffer(
            image.raw_data, (image.width, image.height), "BGRA")
        # convert() detaches the Surface from CARLA's buffer, which is only
        # valid for the duration of this callback.
        # convert() で CARLA のバッファから切り離す。バッファはこの
        # コールバックの間しか有効ではない。
        return surface.convert()
    except (ValueError, pygame.error):
        arr = np.frombuffer(image.raw_data, dtype=np.uint8)
        arr = arr.reshape(image.height, image.width, 4)[:, :, :3][:, :, ::-1]
        return pygame.surfarray.make_surface(arr.swapaxes(0, 1))


def _apply_options(bp, options):
    for k, v in options.items():
        if k == 'sensor_tick':      # handled by _throttle / _throttle が担当
            continue
        bp.set_attribute(k, v)


_FONT_CACHE = {}


def cached_font(size, bold=False):
    """SysFont() scans the font directories and loads a FreeType face.

    Called from a render loop it did that 6x per frame; now once per size.
    SysFont() はフォントディレクトリを走査して FreeType フェイスを読み込む。
    描画ループから毎フレーム6回呼ばれていたものを、サイズごとに1回にする。
    """
    key = (size, bold)
    if key not in _FONT_CACHE:
        _FONT_CACHE[key] = pygame.font.SysFont('monospace', size, bold=bold)
    return _FONT_CACHE[key]


class SensorPanel:
    def __init__(self, world, display_mgr, kind, transform, attached,
                 options, pos, label=None):
        self.display_mgr = display_mgr
        self.pos = pos
        self.kind = kind
        self.label = label or kind
        self.surface = None
        self.options = dict(options)
        self.sensor = self._spawn(world, kind, transform, attached)
        display_mgr.add(self)

    def _spawn(self, world, kind, transform, attached):
        bp_lib = world.get_blueprint_library()
        cw, ch = self.display_mgr.cell_size()
        weak = weakref.ref(self)

        def _throttle(bp):
            # A capped sensor_tick makes the server skip the render pass
            # entirely, so it saves GPU draw time *and* the CPU cost of the
            # readback and the colour conversion. These six panels are a
            # monitoring view; they do not need the full simulation rate.
            # sensor_tick を設定するとサーバ側が描画パス自体を省くため、
            # GPUの描画時間とCPUの読み戻し・色変換の両方を節約できる。
            # この6パネルは監視用であり、シミュレーション全速は不要。
            tick = float(self.options.get('sensor_tick', SENSOR_TICK))
            if tick > 0.0 and bp.has_attribute('sensor_tick'):
                bp.set_attribute('sensor_tick', str(tick))
            return bp

        if kind == 'RGBCamera':
            bp = bp_lib.find('sensor.camera.rgb')
            bp.set_attribute('image_size_x', str(cw))
            bp.set_attribute('image_size_y', str(ch))
            _apply_options(bp, self.options)
            s = world.spawn_actor(_throttle(bp), transform, attach_to=attached)
            s.listen(lambda im: SensorPanel._on_rgb(weak, im))
            return s

        if kind == 'SemanticCamera':
            bp = bp_lib.find('sensor.camera.semantic_segmentation')
            bp.set_attribute('image_size_x', str(cw))
            bp.set_attribute('image_size_y', str(ch))
            _apply_options(bp, self.options)
            s = world.spawn_actor(_throttle(bp), transform, attach_to=attached)
            s.listen(lambda im: SensorPanel._on_sem_cam(weak, im))
            return s

        if kind == 'LiDAR':
            bp = bp_lib.find('sensor.lidar.ray_cast')
            bp.set_attribute('range', self.options.get('range', '80'))
            bp.set_attribute('channels', self.options.get('channels', '64'))
            bp.set_attribute('points_per_second',
                             self.options.get('points_per_second', '250000'))
            bp.set_attribute('rotation_frequency',
                             self.options.get('rotation_frequency', '20'))
            bp.set_attribute('dropoff_general_rate',
                             bp.get_attribute('dropoff_general_rate')
                               .recommended_values[0])
            bp.set_attribute('dropoff_intensity_limit',
                             bp.get_attribute('dropoff_intensity_limit')
                               .recommended_values[0])
            bp.set_attribute('dropoff_zero_intensity',
                             bp.get_attribute('dropoff_zero_intensity')
                               .recommended_values[0])
            s = world.spawn_actor(_throttle(bp), transform, attach_to=attached)
            s.listen(lambda im: SensorPanel._on_lidar(weak, im))
            return s

        if kind in ('DepthCameraRaw', 'DepthCameraGray'):
            bp = bp_lib.find('sensor.camera.depth')
            bp.set_attribute('image_size_x', str(cw))
            bp.set_attribute('image_size_y', str(ch))
            _apply_options(bp, self.options)
            s = world.spawn_actor(_throttle(bp), transform, attach_to=attached)
            if kind == 'DepthCameraRaw':
                s.listen(lambda im: SensorPanel._on_depth_raw(weak, im))
            else:
                s.listen(lambda im: SensorPanel._on_depth_gray(weak, im))
            return s

        raise ValueError("unknown sensor kind: %s" % kind)

    @staticmethod
    def _on_rgb(weak, image):
        self = weak()
        if not self:
            return
        image.convert(carla.ColorConverter.Raw)
        self.surface = bgra_surface(image)

    @staticmethod
    def _on_sem_cam(weak, image):
        self = weak()
        if not self:
            return
        image.convert(carla.ColorConverter.CityScapesPalette)
        self.surface = bgra_surface(image)

    @staticmethod
    def _on_lidar(weak, image):
        self = weak()
        if not self:
            return
        cw, ch = self.display_mgr.cell_size()
        lidar_range = 2.0 * float(self.options.get('range', '80'))
        points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
        points = points.reshape((-1, 4))
        xy = points[:, :2] * (min(cw, ch) / lidar_range)
        xy += (0.5 * cw, 0.5 * ch)
        xy = np.fabs(xy).astype(np.int32)
        xy[:, 0] = np.clip(xy[:, 0], 0, cw - 1)
        xy[:, 1] = np.clip(xy[:, 1], 0, ch - 1)
        img = np.zeros((cw, ch, 3), dtype=np.uint8)
        img[xy[:, 0], xy[:, 1]] = (255, 255, 255)
        self.surface = pygame.surfarray.make_surface(img)

    @staticmethod
    def _on_depth_raw(weak, image):
        self = weak()
        if not self:
            return
        image.convert(carla.ColorConverter.Raw)
        self.surface = bgra_surface(image)

    @staticmethod
    def _on_depth_gray(weak, image):
        self = weak()
        if not self:
            return
        image.convert(carla.ColorConverter.Depth)
        self.surface = bgra_surface(image)

    def render(self, display):
        cw, ch = self.display_mgr.cell_size()
        off = self.display_mgr.cell_offset(self.pos)
        if self.surface is not None:
            display.blit(self.surface, off)
        pygame.draw.rect(display, (60, 70, 85),
                         pygame.Rect(off[0], off[1], cw, ch), 1)
        surf = cached_font(13, bold=True).render(
            self.label, True, (255, 230, 120))
        bg = pygame.Surface((surf.get_width() + 10, surf.get_height() + 4))
        bg.set_alpha(160)
        bg.fill((0, 0, 0))
        display.blit(bg, (off[0] + 4, off[1] + 4))
        display.blit(surf, (off[0] + 9, off[1] + 6))

    def destroy(self):
        if self.sensor is not None:
            try:
                self.sensor.stop()
            except Exception:
                pass
            try:
                self.sensor.destroy()
            except Exception:
                pass
            self.sensor = None


def find_hero(world):
    for a in world.get_actors().filter('vehicle.*'):
        if a.attributes.get('role_name') == 'hero':
            return a
    return None


def build_panels(world, display_mgr, hero):
    cam_front = carla.Transform(carla.Location(x=1.6, z=1.7))
    cam_rear = carla.Transform(carla.Location(x=-1.5, z=1.9),
                               carla.Rotation(yaw=180))
    lidar_tf = carla.Transform(carla.Location(x=0.0, z=2.4))

    SensorPanel(world, display_mgr, 'RGBCamera',
                cam_front, hero, {}, pos=[0, 0],
                label='RGB Camera Front')
    SensorPanel(world, display_mgr, 'SemanticCamera',
                cam_front, hero, {}, pos=[0, 1],
                label='Semantic Camera')
    SensorPanel(world, display_mgr, 'RGBCamera',
                cam_rear, hero, {}, pos=[0, 2],
                label='RGB Camera Back')
    SensorPanel(world, display_mgr, 'LiDAR', lidar_tf, hero,
                dict(LIDAR_OPTS), pos=[1, 0],
                label='LiDAR (top-down)')
    SensorPanel(world, display_mgr, 'DepthCameraRaw',
                cam_front, hero, {}, pos=[1, 1],
                label='Depth Camera (Raw)')
    SensorPanel(world, display_mgr, 'DepthCameraGray',
                cam_front, hero, {}, pos=[1, 2],
                label='Depth Camera (Gray)')


def _draw_waiting(display_mgr, text):
    display_mgr.display.fill((15, 18, 24))
    surf = cached_font(22, bold=True).render(text, True, (200, 220, 240))
    w, h = display_mgr.window_size
    display_mgr.display.blit(
        surf, ((w - surf.get_width()) // 2, (h - surf.get_height()) // 2))
    pygame.display.flip()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('-p', '--port', type=int, default=2000)
    ap.add_argument('--res', default='1440x720', help='WxH window size')
    ap.add_argument('--fps', type=int, default=30,
                    help='window redraw cap (default: 30). This is a monitor '
                         'view, not the driving view '
                         '/ ウィンドウ再描画の上限FPS（既定 30）。'
                         'これは監視用ビューであり運転用ビューではない')
    ap.add_argument('--sensor-fps', type=float, default=20.0,
                    help='rate of the six panel sensors (default: 20). Each '
                         'is a render pass on the server, so lowering this '
                         'is the biggest saving. 0 = uncapped '
                         '/ 6つのパネルセンサーのレート（既定 20）。'
                         '各センサーはサーバ側の描画パスなので、'
                         'ここを下げるのが最も効果的。0 で無制限')
    ap.add_argument('--lidar-channels', default='32',
                    help='LiDAR beams (default: 32). Ray-casting is done on '
                         'the server CPU, not the GPU '
                         '/ LiDARのビーム数（既定 32）。'
                         'レイキャストはGPUではなくサーバのCPUで行われる')
    ap.add_argument('--lidar-points', default='120000',
                    help='LiDAR points per second (default: 120000) '
                         '/ LiDARの毎秒点数（既定 120000）')
    args = ap.parse_args()

    global SENSOR_TICK
    SENSOR_TICK = (1.0 / args.sensor_fps) if args.sensor_fps > 0 else 0.0
    LIDAR_OPTS['channels'] = args.lidar_channels
    LIDAR_OPTS['points_per_second'] = args.lidar_points

    width, height = (int(x) for x in args.res.split('x'))

    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)

    display_mgr = DisplayManager(grid_size=[2, 3],
                                 window_size=(width, height))

    current_hero_id = None
    current_map = None
    clock = pygame.time.Clock()
    last_err_msg_ts = 0.0

    # Hero/map discovery is an RPC round trip plus a full actor-list fetch.
    # It used to run once per redrawn frame; the hero only ever changes when
    # the driving script respawns, so 4 Hz is ample and the surfaces keep
    # arriving from the sensor callbacks in between.
    # hero とマップの探索はRPC往復と全アクター一覧の取得を伴う。以前は
    # 再描画のたびに実行していたが、hero が変わるのは運転スクリプトが
    # 再スポーンしたときだけなので 4 Hz で十分。その間もサーフェスは
    # センサーのコールバックから届き続ける。
    probe_interval = 0.25
    next_probe = 0.0
    hero = None
    map_name = None

    try:
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return
                if event.type == pygame.KEYDOWN:
                    if event.key in (K_ESCAPE, K_q):
                        return
                    if event.key == K_F11:
                        display_mgr.toggle_fullscreen()

            now = time.time()
            if now >= next_probe:
                next_probe = now + probe_interval
                try:
                    world = client.get_world()
                    map_name = world.get_map().name.split('/')[-1]
                    hero = find_hero(world)
                except Exception as exc:
                    if now - last_err_msg_ts > 2.0:
                        print("[sensor_viewer] world error: %s" % exc)
                        last_err_msg_ts = now
                    hero = None
                    map_name = None
                    _draw_waiting(display_mgr,
                                  "CARLA unreachable — retrying...")
                    clock.tick(10)
                    continue

            if hero is None:
                if display_mgr.panels:
                    display_mgr.destroy_all()
                    current_hero_id = None
                _draw_waiting(display_mgr,
                              "Waiting for hero vehicle on %s..." % map_name)
                clock.tick(15)
                continue

            if hero.id != current_hero_id or map_name != current_map:
                display_mgr.destroy_all()
                build_panels(world, display_mgr, hero)
                current_hero_id = hero.id
                current_map = map_name
                print("[sensor_viewer] attached to hero id=%d on %s"
                      % (hero.id, map_name))

            display_mgr.render()
            clock.tick(args.fps)

    finally:
        display_mgr.destroy_all()
        pygame.quit()


if __name__ == '__main__':
    main()
