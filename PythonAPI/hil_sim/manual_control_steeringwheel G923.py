#!/usr/bin/env python

# Copyright (c) 2019 Intel Labs
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
# 本コードは MIT ライセンスの条件の下で提供される。
# ライセンス全文は <https://opensource.org/licenses/MIT> を参照。

# Allows controlling a vehicle with a keyboard. For a simpler and more
# documented example, please take a look at tutorial.py.
# キーボードで車両を操作できるスクリプト。より簡潔で解説の充実した例は
# tutorial.py を参照。

"""
Welcome to CARLA manual control with steering wheel Logitech G923/G29.

Drive with the wheel and pedals. To drive start by pressing the brake pedal.
Change wheel_config.ini according to your steering wheel
(use jstest-gtk in Ubuntu to find the axis values).

Without a wheel the keyboard takes over automatically (or pass --keyboard):

    W / Up       : throttle
    S / Down     : brake; once stopped, reverse
    A / D        : steer left / right
    SPACE        : handbrake

    TAB          : next camera (Driver Eye / Hood / Chase)
    N            : toggle mirrors (driver-eye view)
    Z / X        : glance left / right (hold)
    Ctrl+I/K/J/L/U/O : move the driver's eye fwd/back/left/right/down/up (saved per car)
    Q            : toggle reverse
    M            : manual / automatic (wheel: button 9)
    , / .        : gear down / up (wheel: paddles)
    P            : toggle autopilot
    C / Shift+C  : next / previous weather
    [ / ]        : throttle strength down / up (with Shift: brake strength)
    F5           : cycle map
    F11          : fullscreen
    R            : toggle camera recording
    F1           : toggle HUD info
    H / ?        : this help
    ESC / Ctrl+Q : quit

CARLA 手動運転（Logitech G923/G29 ステアリングホイール対応）へようこそ。

ホイールとペダルで運転します。ブレーキペダルを踏むと運転を開始できます。
お使いのホイールに合わせて wheel_config.ini を編集してください
（Ubuntu では jstest-gtk で軸番号を確認できます）。

ホイールが無い場合は自動でキーボード操作に切り替わります（--keyboard でも可）:

    W / ↑        : アクセル
    S / ↓        : ブレーキ。停止後はリバース
    A / D        : 左 / 右へ操舵
    SPACE        : ハンドブレーキ

    TAB          : 次のカメラ（ドライバー視点 / ボンネット / 追跡）
    N            : ミラー表示切替（ドライバー視点）
    Z / X        : 左 / 右をチラ見（押している間）
    Ctrl+I/K/J/L/U/O : 視点を前/後/左/右/下/上へ（車両ごとに保存）
    Q            : リバース切替
    M            : マニュアルトランスミッション切替
    , / .        : シフトダウン / アップ（ホイール：パドル）
    P            : オートパイロット切替
    C / Shift+C  : 次 / 前の天候
    [ / ]        : アクセル強度 下げ / 上げ（Shift 併用でブレーキ強度）
    F5           : マップ切替
    F11          : フルスクリーン
    R            : カメラ録画切替
    F1           : HUD 情報表示切替
    H / ?        : このヘルプ
    ESC / Ctrl+Q : 終了
"""

from __future__ import print_function


# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================


import glob
import os
import sys

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass


# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================


import carla

from carla import ColorConverter as cc

import argparse
import collections
import datetime
import json
import logging
import math
import random
import re
import time
import weakref

# your own cars' driver views and steering wheels (cars/car_views.py)
# 自作車両の視点とハンドル
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'cars'))
import car_views  # noqa: E402

if sys.version_info >= (3, 0):

    from configparser import ConfigParser

else:

    from ConfigParser import RawConfigParser as ConfigParser

try:
    import pygame
    from pygame.locals import KMOD_CTRL
    from pygame.locals import KMOD_SHIFT
    from pygame.locals import K_0
    from pygame.locals import K_9
    from pygame.locals import K_BACKQUOTE
    from pygame.locals import K_BACKSPACE
    from pygame.locals import K_COMMA
    from pygame.locals import K_DOWN
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_F1
    from pygame.locals import K_F5
    from pygame.locals import K_F11
    from pygame.locals import K_LEFT
    from pygame.locals import K_LEFTBRACKET
    from pygame.locals import K_PERIOD
    from pygame.locals import K_RIGHTBRACKET
    from pygame.locals import K_RIGHT
    from pygame.locals import K_SLASH
    from pygame.locals import K_SPACE
    from pygame.locals import K_TAB
    from pygame.locals import K_UP
    from pygame.locals import K_a
    from pygame.locals import K_c
    from pygame.locals import K_d
    from pygame.locals import K_h
    from pygame.locals import K_m
    from pygame.locals import K_n
    from pygame.locals import K_p
    from pygame.locals import K_q
    from pygame.locals import K_r
    from pygame.locals import K_s
    from pygame.locals import K_w
    from pygame.locals import K_x
    from pygame.locals import K_z
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')


# ==============================================================================
# -- Global functions ----------------------------------------------------------
# ==============================================================================


def find_weather_presets():
    rgx = re.compile('.+?(?:(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|$)')
    name = lambda x: ' '.join(m.group(0) for m in rgx.finditer(x))
    presets = [x for x in dir(carla.WeatherParameters) if re.match('[A-Z].+', x)]
    return [(getattr(carla.WeatherParameters, x), name(x)) for x in presets]


def get_actor_display_name(actor, truncate=250):
    name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
    return (name[:truncate - 1] + u'\u2026') if len(name) > truncate else name


# ==============================================================================
# -- World ---------------------------------------------------------------------
# ==============================================================================


# Ctrl + key -> driver-eye nudge (x forward, y right, z up) [m], own cars
# Ctrl + キー -> 視点の移動量 [m]（自作車両）
EYE_STEP = 0.02
EYE_KEYS = {pygame.K_i: (EYE_STEP, 0, 0), pygame.K_k: (-EYE_STEP, 0, 0),
            pygame.K_j: (0, -EYE_STEP, 0), pygame.K_l: (0, EYE_STEP, 0),
            pygame.K_u: (0, 0, -EYE_STEP), pygame.K_o: (0, 0, EYE_STEP)}

MAP_CYCLE = ['Town01', 'Town02', 'Town03', 'Town04', 'Town05',
             'Town06', 'Town07', 'Town10HD', 'MarinaBay', 'SpaFrancorchamps', 'OitaStation']
MAPS_JSON = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'maps', 'maps.json')


def map_start(map_name):
    """The start (grid) transform of one of your own maps, or None.
    自作マップのスタート位置（無ければ None）。"""
    try:
        with open(MAPS_JSON) as f:
            maps = json.load(f)
    except (OSError, ValueError):
        return None
    for key, m in maps.items():
        if not key.startswith('_') and m.get('name') == map_name and 'start' in m:
            st = m['start']
            return carla.Transform(carla.Location(st['x'], st['y'], st['z']),
                                   carla.Rotation(yaw=st['yaw']))
    return None


class World(object):
    def __init__(self, client, carla_world, hud, actor_filter, fov=100.0, start_vehicle=''):
        self.client = client
        self.world = carla_world
        self.hud = hud
        self.player = None
        self.collision_sensor = None
        self.lane_invasion_sensor = None
        self.gnss_sensor = None
        self.camera_manager = None
        self.steering_wheel = None   # own cars: the turning steering wheel / 自作車両のハンドル
        self.wheel_deg = 0.0         # real wheel angle, + = right / 実ハンドル角（右が正）
        self._weather_presets = find_weather_presets()
        self._weather_index = 0
        self._actor_filter = actor_filter
        # current hero model; --start-vehicle picks the first one / 現在の車種（初回は --start-vehicle）
        self._model_id = start_vehicle or None
        self._fov = fov
        self.restart()
        self.world.on_tick(hud.on_world_tick)

    def _current_map_short_name(self):
        return self.world.get_map().name.split('/')[-1].replace('_Opt', '')

    def change_map(self, offset):
        current = self._current_map_short_name()
        try:
            idx = MAP_CYCLE.index(current)
        except ValueError:
            idx = -1
        target = MAP_CYCLE[(idx + offset) % len(MAP_CYCLE)]
        self.hud.notification('Loading map %s...' % target, seconds=5.0)
        try:
            new_world = self.client.load_world(target)
        except Exception as exc:
            self.hud.error('load_world failed: %s' % exc)
            return
        # After load_world, every actor reference from the old world is stale.
        # load_world の後は、旧ワールドの全アクター参照が無効になる。
        self.player = None
        self.collision_sensor = None
        self.lane_invasion_sensor = None
        self.gnss_sensor = None
        self.camera_manager = None
        self.world = new_world
        self.restart(step=0)    # same model on the new map / 新マップでも同じ車種
        self.world.on_tick(self.hud.on_world_tick)
        self.hud.notification('Map: %s' % target)

    def restart(self, step=1):
        """(Re)spawn the hero as the next model, in alphabetical order of
        blueprint id: ``step`` 1 = next, -1 = previous, 0 = same model. The
        first spawn is random.
        車両を（再）生成。ブループリント ID のアルファベット順で次の車種
        （step 1 = 次、-1 = 前、0 = 同じ）。最初の生成のみランダム。
        """
        # Keep same camera config if the camera manager exists. / カメラマネージャがあれば設定を引き継ぐ。
        cam_index = self.camera_manager.index if self.camera_manager is not None else 0
        cam_pos_index = self.camera_manager.transform_index if self.camera_manager is not None else 0
        blueprint = self._next_blueprint(step)
        blueprint.set_attribute('role_name', 'hero')
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)
        # Spawn the player. / プレイヤー車両をスポーン。
        if self.player is not None:
            spawn_point = self.player.get_transform()
            spawn_point.location.z += 2.0
            spawn_point.rotation.roll = 0.0
            spawn_point.rotation.pitch = 0.0
            self.destroy()
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
        if self.player is None:
            # your own maps (maps/maps.json): start on the grid / 自作マップはスタート位置から
            start = map_start(self._current_map_short_name())
            if start is not None:
                self.player = self.world.try_spawn_actor(blueprint, start)
        while self.player is None:
            spawn_points = self.world.get_map().get_spawn_points()
            spawn_point = random.choice(spawn_points) if spawn_points else carla.Transform()
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
        # Set up the sensors. / センサー類をセットアップ。
        self.collision_sensor = CollisionSensor(self.player, self.hud)
        self.lane_invasion_sensor = LaneInvasionSensor(self.player, self.hud)
        self.gnss_sensor = GnssSensor(self.player)
        self.camera_manager = CameraManager(self.player, self.hud, self._fov)
        self.camera_manager.transform_index = cam_pos_index
        self.camera_manager.set_sensor(cam_index, notify=False)
        view = car_views.load(self.player.type_id)
        if view is not None and 'steering_wheel' in view:
            try:
                self.steering_wheel = car_views.SteeringWheel(self.world, self.player, view)
            except RuntimeError as exc:
                print('steering wheel prop: %s' % exc)
        self._model_id = self.player.type_id
        actor_type = get_actor_display_name(self.player)
        self.hud.notification(actor_type)

    def _next_blueprint(self, step):
        """The model ``step`` places after the current one (wraps around).
        現在の車種から ``step`` 番目の車種（末尾の次は先頭）。"""
        models = sorted(self.world.get_blueprint_library().filter(self._actor_filter),
                        key=lambda bp: bp.id)
        ids = [bp.id for bp in models]
        current = self._model_id or (self.player.type_id if self.player is not None else None)
        if current not in ids:
            return random.choice(models)
        if self.player is None:          # first spawn: the requested car itself / 初回はその車種
            return models[ids.index(current)]
        return models[(ids.index(current) + step) % len(models)]

    def next_weather(self, reverse=False):
        self._weather_index += -1 if reverse else 1
        self._weather_index %= len(self._weather_presets)
        preset = self._weather_presets[self._weather_index]
        self.hud.notification('Weather: %s' % preset[1])
        self.player.get_world().set_weather(preset[0])

    def tick(self, clock):
        self.hud.tick(self, clock)
        if self.camera_manager is not None:
            self.camera_manager.update_head(self.player)
        if self.steering_wheel is not None:
            self.steering_wheel.set(self.wheel_deg)

    def render(self, display):
        self.camera_manager.render(display)
        self.hud.render(display)

    def destroy(self):
        sensors = [
            self.camera_manager.sensor,
            self.collision_sensor.sensor,
            self.lane_invasion_sensor.sensor,
            self.gnss_sensor.sensor]
        sensors += list(self.camera_manager.mirror_sensors())
        for sensor in sensors:
            if sensor is not None:
                sensor.stop()
                sensor.destroy()
        if self.steering_wheel is not None:
            self.steering_wheel.destroy()
            self.steering_wheel = None
        if self.player is not None:
            self.player.destroy()

# ==============================================================================
# -- DualControl -----------------------------------------------------------
# ==============================================================================


class DualControl(object):
    def __init__(self, world, start_in_autopilot, keyboard=False):
        self._autopilot_enabled = start_in_autopilot
        if isinstance(world.player, carla.Vehicle):
            self._control = carla.VehicleControl()
            world.player.set_autopilot(self._autopilot_enabled)
        elif isinstance(world.player, carla.Walker):
            self._control = carla.WalkerControl()
            self._autopilot_enabled = False
            self._rotation = world.player.get_transform().rotation
        else:
            raise NotImplementedError("Actor type not supported")
        self._steer_cache = 0.0
        world.hud.notification("Press 'H' or '?' for help.", seconds=4.0)

        # Steering wheel, with a W/A/S/D keyboard fallback so the simulator
        # still runs (and can be tested) when the G923 is not attached.
        # `keyboard=True` forces the fallback even if a wheel is plugged in.
        # ステアリングホイールを初期化。G923 が接続されていなくても
        # W/A/S/D キーボードで運転・テストできるようフォールバックする。
        # keyboard=True ならホイール接続時でもキーボードを強制する。
        self._joystick = None
        pygame.joystick.init()

        if keyboard:
            print("[control] keyboard mode forced (--keyboard)")
        else:
            # Find the G923 wheel by name / 名前で G923 ホイールを検出
            names = []
            g923_index = None
            for i in range(pygame.joystick.get_count()):
                joy = pygame.joystick.Joystick(i)
                joy.init()
                names.append(joy.get_name())
                if "G923" in joy.get_name() or "G29" in joy.get_name():
                    g923_index = i
                    joy.quit()
                    break
                joy.quit()

            if g923_index is None:
                print("[control] G923/G29 wheel not found (joysticks seen: %s)"
                      % (", ".join(names) if names else "none"))
            else:
                self._joystick = pygame.joystick.Joystick(g923_index)
                self._joystick.init()
                print("[control] steering wheel: %s" % names[g923_index])

        self._keyboard = self._joystick is None
        if self._keyboard:
            print("[control] driving with the keyboard: "
                  "W = throttle, S = brake / reverse, A / D = steer, "
                  "SPACE = handbrake")
        # Keyboard pedal state: pressing W/S ramps the pedal instead of
        # slamming it fully down, so the car is drivable without a wheel.
        # キーボードのペダル状態。W/S は一気に全踏みせず徐々に立ち上がる。
        self._kb_throttle = 0.0
        self._kb_brake = 0.0

        # wheel_config.ini lives next to this script; missing entries fall
        # back to the G923 defaults so keyboard mode never needs the file.
        # wheel_config.ini は本スクリプトと同じ場所にある。項目が無い場合は
        # G923 の既定値を使うため、キーボードモードではファイル不要。
        self._parser = ConfigParser()
        self._parser.read(os.path.join(
            os.path.dirname(os.path.abspath(__file__)), 'wheel_config.ini'))

        def _axis(key, default):
            try:
                return int(self._parser.get('G29 Racing Wheel', key))
            except Exception:
                return default

        self._steer_idx = _axis('steering_wheel', 0)
        self._throttle_idx = _axis('throttle', 2)
        self._brake_idx = _axis('brake', 1)
        self._reverse_idx = _axis('reverse', 6)
        self._handbrake_idx = _axis('handbrake', 7)
        # Paddle shifters (G29/G923: right paddle 4, left paddle 5) and the
        # button that switches back to automatic.
        # パドルシフト（G29/G923：右 4、左 5）と AT に戻すボタン。
        self._gear_up_idx = _axis('gear_up', 4)
        self._gear_down_idx = _axis('gear_down', 5)
        self._transmission_idx = _axis('transmission', 9)

        # Pedal direction. PC-mode G923 (046d:c266) rests at +1 and reads -1
        # at full press; PS4-mode (046d:c267) puts the pedals on trigger-style
        # axes that rest at -1 and read +1 pressed. Getting this backwards
        # leaves a pedal stuck near half-press, which reads as permanent
        # braking and the car will not pull away. Run calibrate_wheel.py to
        # set these; unset means "detect from the resting value at startup".
        # ペダルの向き。PC モードの G923 (046d:c266) は静止 +1・全踏 -1、
        # PS4 モード (046d:c267) はトリガー系の軸で静止 -1・全踏 +1。
        # ここを取り違えるとペダルが半踏み付近に固定され、常時ブレーキ状態と
        # なって発進できない。calibrate_wheel.py で設定する。未設定の場合は
        # 起動時の静止値から自動判定する。
        def _invert(key):
            if self._parser.has_option('G29 Racing Wheel', key):
                return self._parser.getboolean('G29 Racing Wheel', key)
            return None

        self._throttle_inv = _invert('throttle_invert')
        self._brake_inv = _invert('brake_invert')

        if self._joystick is not None:
            self._autodetect_pedal_direction()

        def _sens(key, default):
            if self._parser.has_option('Sensitivity', key):
                return float(self._parser.get('Sensitivity', key))
            return default

        self._steer_gain = _sens('steer_gain', 1.83)
        self._steer_curve = _sens('steer_curve', 0.5)
        self._steer_clamp = _sens('steer_clamp', 1.0)
        # lock to lock of the real wheel (G923: 900), for the on-screen wheel
        # 実ハンドルの全回転角（G923 は 900°）。画面内のハンドル用
        self._wheel_half_deg = _sens('wheel_range_deg', 900.0) / 2.0
        self._wheel_deg = 0.0

        # Live pedal tuning, shared with the dashboard's "Pedal Tuning" card
        # through pedal_tuning.json (next to this script). Reloaded on the
        # fly whenever the file changes; [ / ] keys adjust it too.
        #   *_max   : output at full pedal press (0.1 .. 1.0)
        #   *_curve : response exponent; higher = gentler at light press
        # ペダル特性のライブ調整。pedal_tuning.json（本スクリプトと同じ場所）を介して
        # ダッシュボードの「Pedal Tuning」カードと共有される。ファイルが変わると
        # 随時再読み込みされ、[ / ] キーでも調整できる。
        #   *_max   : ペダル全踏時の出力（0.1 .. 1.0）
        #   *_curve : 応答の指数。大きいほど浅い踏み込みで穏やかになる
        self._tuning_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), 'pedal_tuning.json')
        self._tuning = {'throttle_max': 0.8, 'throttle_curve': 1.8,
                        'brake_max': 1.0, 'brake_curve': 2.2}
        self._tuning_mtime = 0.0
        self._tuning_check = 0
        if os.path.isfile(self._tuning_path):
            self._load_tuning()
        else:
            self._save_tuning()

        self._fullscreen = False

    def _load_tuning(self):
        """Reload pedal tuning if pedal_tuning.json changed. Returns True
        when values actually changed.

        pedal_tuning.json が変更されていればペダル設定を再読み込みする。
        値が実際に変化した場合に True を返す。"""
        try:
            mtime = os.path.getmtime(self._tuning_path)
            if mtime == self._tuning_mtime:
                return False
            self._tuning_mtime = mtime
            with open(self._tuning_path) as fh:
                data = json.load(fh)
            old = dict(self._tuning)
            for key, lo, hi in (('throttle_max', 0.1, 1.0),
                                ('throttle_curve', 0.5, 3.0),
                                ('brake_max', 0.1, 1.0),
                                ('brake_curve', 0.5, 3.0)):
                if key in data:
                    self._tuning[key] = max(lo, min(hi, float(data[key])))
            return self._tuning != old
        except Exception:
            return False

    def _save_tuning(self):
        try:
            tmp = self._tuning_path + '.tmp'
            with open(tmp, 'w') as fh:
                json.dump(self._tuning, fh, indent=1)
            os.replace(tmp, self._tuning_path)
            self._tuning_mtime = os.path.getmtime(self._tuning_path)
        except Exception:
            pass

    def _toggle_transmission(self, world):
        """Automatic <-> manual. Manual starts in the gear the car is in, or
        1st when stopped (the gearbox reports N then, and in manual N doesn't
        drive). / AT と MT を切替。MT は現在のギア（停止中は 1 速）から。"""
        c = self._control
        c.manual_gear_shift = not c.manual_gear_shift
        if c.manual_gear_shift:
            gear = world.player.get_control().gear
            c.gear = -1 if c.reverse else max(1, gear)
        else:
            c.gear = -1 if c.reverse else 1
        world.hud.notification('%s Transmission' %
                               ('Manual' if c.manual_gear_shift else 'Automatic'))

    def _shift(self, world, step):
        """Paddle / , . shift: ``step`` +1 up, -1 down, between 1st and the
        car's top gear (reverse stays on its own button; up from R or N goes
        to 1st). In automatic, the first paddle press switches to manual.
        パドル（, .）でシフト。1 速〜最高速ギアの範囲（R は専用ボタン、R・N
        からのシフトアップは 1 速）。AT 中に押すと MT に切り替える。"""
        c = self._control
        if not c.manual_gear_shift:
            self._toggle_transmission(world)
        top = world.hud.gauge.gears
        if c.gear < 1:
            c.gear = 1
        else:
            c.gear = max(1, min(top, c.gear + step))
        world.hud.notification('Gear %d' % c.gear, seconds=0.8)

    def _toggle_signal(self, world, side):
        """Toggle a turn signal (left/right). State is shared with the
        dashboard through turn_signal.json: the dashboard shows the blinking
        indicators and, while a signal is on, logs a broken-line crossing as
        a legal lane change instead of a lane invasion.

        ウインカー（左右）を切り替える。状態は turn_signal.json を介して
        ダッシュボードと共有され、点灯中の破線跨ぎは車線逸脱ではなく
        合法な車線変更として記録される。
        """
        path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), 'turn_signal.json')
        cur = 'off'
        try:
            with open(path) as fh:
                cur = json.load(fh).get('side', 'off')
        except Exception:
            pass
        new = 'off' if cur == side else side
        try:
            tmp = path + '.tmp'
            with open(tmp, 'w') as fh:
                json.dump({'side': new}, fh)
            os.replace(tmp, path)
        except Exception:
            pass
        # Apply the blinker light immediately so it also works when the
        # dashboard is not running.
        # ダッシュボードが起動していなくても点灯するよう、その場で
        # ブリンカーライトを適用する。
        try:
            blink = (int(carla.VehicleLightState.LeftBlinker)
                     | int(carla.VehicleLightState.RightBlinker))
            ls = int(world.player.get_light_state()) & ~blink
            if new == 'left':
                ls |= int(carla.VehicleLightState.LeftBlinker)
            elif new == 'right':
                ls |= int(carla.VehicleLightState.RightBlinker)
            world.player.set_light_state(carla.VehicleLightState(ls))
        except Exception:
            pass
        world.hud.notification(
            'Turn signal: %s' % ('OFF' if new == 'off' else new.upper()))

    def _toggle_fullscreen(self, hud):
        self._fullscreen = not self._fullscreen
        flags = pygame.RESIZABLE | pygame.SCALED | pygame.DOUBLEBUF
        if self._fullscreen:
            flags |= pygame.FULLSCREEN
        pygame.display.set_mode(hud.dim, flags)
        hud.notification(
            'Fullscreen: %s' % ('On' if self._fullscreen else 'Off'))

    def parse_events(self, world, clock):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            elif event.type == pygame.JOYBUTTONDOWN:
                if event.button == 0:
                    world.restart()
                elif event.button == 1:
                    world.hud.toggle_info()
                elif event.button == 2:
                    world.camera_manager.toggle_camera()
                elif event.button == 3:
                    world.next_weather()
                elif event.button == self._reverse_idx:
                    self._control.gear = 1 if self._control.reverse else -1
                elif event.button == self._gear_up_idx:
                    self._shift(world, 1)
                elif event.button == self._gear_down_idx:
                    self._shift(world, -1)
                elif event.button == self._transmission_idx:
                    self._toggle_transmission(world)
                elif event.button in (10, 11):
                    # Turn signals: 10 = right, 11 = left; same button again
                    # switches it off (lane change complete).
                    # ウインカー：10=右・11=左。同じボタンをもう一度押すと
                    # 消灯（車線変更完了）。
                    self._toggle_signal(
                        world, 'right' if event.button == 10 else 'left')
                elif event.button == 23:
                    world.camera_manager.next_sensor()

            elif event.type == pygame.KEYUP:
                if self._is_quit_shortcut(event.key):
                    return True
                elif event.key == K_BACKSPACE:
                    # next model; Shift = previous / 次の車種、Shift で前の車種
                    world.restart(step=-1 if pygame.key.get_mods() & KMOD_SHIFT else 1)
                elif event.key == K_F1:
                    world.hud.toggle_info()
                elif event.key == K_h or (event.key == K_SLASH and pygame.key.get_mods() & KMOD_SHIFT):
                    world.hud.help.toggle()
                elif event.key == K_TAB:
                    world.camera_manager.toggle_camera()
                elif event.key == K_n:
                    world.camera_manager.toggle_mirrors()
                elif event.key == K_c and pygame.key.get_mods() & KMOD_SHIFT:
                    world.next_weather(reverse=True)
                elif event.key == K_c:
                    world.next_weather()
                elif event.key == K_BACKQUOTE:
                    world.camera_manager.next_sensor()
                elif event.key > K_0 and event.key <= K_9:
                    world.camera_manager.set_sensor(event.key - 1 - K_0)
                elif event.key == K_r:
                    world.camera_manager.toggle_recording()
                elif event.key == K_F5:
                    offset = -1 if pygame.key.get_mods() & KMOD_SHIFT else 1
                    world.change_map(offset)
                elif event.key == K_F11:
                    self._toggle_fullscreen(world.hud)
                elif (pygame.key.get_mods() & KMOD_CTRL) and event.key in EYE_KEYS:
                    world.camera_manager.nudge_eye(EYE_KEYS[event.key])
                elif event.key in (K_LEFTBRACKET, K_RIGHTBRACKET):
                    step = 0.05 if event.key == K_RIGHTBRACKET else -0.05
                    key = ('brake_max'
                           if pygame.key.get_mods() & KMOD_SHIFT
                           else 'throttle_max')
                    self._tuning[key] = max(
                        0.1, min(1.0, round(self._tuning[key] + step, 2)))
                    self._save_tuning()
                    world.hud.notification('%s strength: %.0f%%' % (
                        'Brake' if key == 'brake_max' else 'Throttle',
                        self._tuning[key] * 100))
                if isinstance(self._control, carla.VehicleControl):
                    if event.key == K_q:
                        self._control.gear = 1 if self._control.reverse else -1
                    elif event.key == K_m:
                        self._toggle_transmission(world)
                    elif event.key == K_COMMA:
                        self._shift(world, -1)
                    elif event.key == K_PERIOD:
                        self._shift(world, 1)
                    elif event.key == K_p:
                        self._autopilot_enabled = not self._autopilot_enabled
                        world.player.set_autopilot(self._autopilot_enabled)
                        world.hud.notification('Autopilot %s' % ('On' if self._autopilot_enabled else 'Off'))

        # Pick up pedal-tuning changes made on the dashboard (~1x/second).
        # ダッシュボードで行われたペダル設定変更を取り込む（約1回/秒）。
        self._tuning_check += 1
        if self._tuning_check >= 30:
            self._tuning_check = 0
            if self._load_tuning():
                world.hud.notification('Pedal tuning updated from dashboard')

        # Glance left/right with Z / X (works in every camera, strongest in
        # the driver-eye view where it emulates a head check).
        # Z / X で左右をチラ見（全カメラで有効。目視確認を再現する
        # ドライバー視点で最も効果が大きい）。
        keys = pygame.key.get_pressed()
        world.camera_manager.look_input = (
            -1.0 if keys[K_z] else (1.0 if keys[K_x] else 0.0))

        if not self._autopilot_enabled:
            if isinstance(self._control, carla.VehicleControl):
                if self._joystick is not None:
                    self._parse_vehicle_wheel()
                else:
                    self._parse_vehicle_keys(keys, clock.get_time(), world)
                    self._wheel_deg = self._control.steer * self._wheel_half_deg
                world.wheel_deg = self._wheel_deg
                self._control.reverse = self._control.gear < 0
            elif isinstance(self._control, carla.WalkerControl):
                self._parse_walker_keys(keys, clock.get_time())
            world.player.apply_control(self._control)

    def _parse_vehicle_keys(self, keys, milliseconds, world=None):
        """Drive with the keyboard when no wheel is attached.

            W / Up     throttle
            S / Down   brake; once stopped, reverse
            A / D      steer left / right (releasing both re-centres)
            SPACE      handbrake

        Throttle and brake ramp up over a few tenths of a second instead of
        jumping to full, and obey the same strength caps as the pedals
        (pedal_tuning.json, adjustable live with [ / ] and on the dashboard),
        so keyboard driving produces telemetry comparable to wheel driving.

        ホイール未接続時にキーボードで運転する。

            W / ↑      アクセル
            S / ↓      ブレーキ。停止後はリバース
            A / D      左 / 右へ操舵（離すと中央へ戻る）
            SPACE      ハンドブレーキ

        アクセル・ブレーキは一気に全踏みせず数十分の一秒かけて立ち上がり、
        ペダルと同じ強度上限（pedal_tuning.json、[ / ] やダッシュボードから
        走行中に調整可）に従うため、ホイール運転と比較可能なテレメトリになる。
        """
        forward = keys[K_UP] or keys[K_w]
        backward = keys[K_DOWN] or keys[K_s]

        speed = 0.0
        if world is not None:
            try:
                v = world.player.get_velocity()
                speed = math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2)
            except Exception:
                speed = 0.0

        # S brakes while the car rolls forward and only engages reverse once
        # it has (nearly) stopped; W does the mirror image while reversing.
        # Q and the manual gearbox keep working: the gear itself is the state.
        # S は前進中はブレーキで、ほぼ停止してから初めてリバースに入る。
        # 後退中の W はその逆。ギア自体を状態とするため Q とMTモードも有効。
        reversing = self._control.gear < 0
        if not self._control.manual_gear_shift:
            if backward and not forward and speed < 0.5:
                reversing = True
            elif forward and speed < 0.5:
                reversing = False
            self._control.gear = -1 if reversing else 1

        accelerating, braking = (backward, forward) if reversing \
            else (forward, backward)

        # Pedal ramps: ~0.4 s to full throttle, ~0.27 s to full brake, and a
        # quicker release in both cases.
        # ペダルの立ち上がり: アクセル約0.4秒、ブレーキ約0.27秒。戻りは速め。
        rate = max(0.0, milliseconds) / 1000.0 / 0.4
        self._kb_throttle = (min(1.0, self._kb_throttle + rate) if accelerating
                             else max(0.0, self._kb_throttle - 2.0 * rate))
        self._kb_brake = (min(1.0, self._kb_brake + 1.5 * rate) if braking
                          else max(0.0, self._kb_brake - 3.0 * rate))

        self._control.throttle = round(
            self._tuning['throttle_max'] * self._kb_throttle, 3)
        self._control.brake = round(
            self._tuning['brake_max'] * self._kb_brake, 3)

        # Steering: hold A / D to wind the wheel on, release to unwind it.
        # 操舵: A / D を押し続けて切り増し、離すと戻る。
        steer_increment = 8e-4 * milliseconds
        if keys[K_LEFT] or keys[K_a]:
            self._steer_cache -= steer_increment
        elif keys[K_RIGHT] or keys[K_d]:
            self._steer_cache += steer_increment
        elif self._steer_cache > 0.0:
            self._steer_cache = max(0.0, self._steer_cache - steer_increment)
        else:
            self._steer_cache = min(0.0, self._steer_cache + steer_increment)

        limit = min(0.7, self._steer_clamp)
        self._steer_cache = min(limit, max(-limit, self._steer_cache))
        self._control.steer = round(self._steer_cache, 3)
        self._control.hand_brake = keys[K_SPACE]

    def _autodetect_pedal_direction(self):
        """Infer pedal polarity from the resting axis values.

        Only fills in what wheel_config.ini did not state explicitly. An
        axis sitting near -1 with nothing pressed is a trigger-style
        (PS4-mode) pedal; near +1 is the classic G29/PC layout. Anything in
        between means the configured index is not a pedal at all, so warn
        rather than silently drive with a half-pressed brake.

        静止時の軸値からペダルの極性を推定する。

        wheel_config.ini に明示がない項目だけを補う。何も踏んでいない状態で
        -1 付近ならトリガー型（PS4 モード）、+1 付近なら従来の G29/PC 配置。
        その中間の場合は設定された軸がそもそもペダルではないため、
        半踏みブレーキのまま走り出さずに警告する。
        """
        try:
            pygame.event.pump()
            axes = [self._joystick.get_axis(i)
                    for i in range(self._joystick.get_numaxes())]
        except Exception:
            return

        for name, idx, attr in (('throttle', self._throttle_idx, '_throttle_inv'),
                                ('brake', self._brake_idx, '_brake_inv')):
            if idx >= len(axes):
                print("[control] WARNING: %s axis %d does not exist "
                      "(wheel has %d axes) -- run calibrate_wheel.py "
                      "/ %s の軸 %d が存在しません（軸数 %d）。"
                      "calibrate_wheel.py を実行してください"
                      % (name, idx, len(axes), name, idx, len(axes)))
                continue
            rest = axes[idx]
            if getattr(self, attr) is None:
                if rest < -0.5:
                    setattr(self, attr, True)
                elif rest > 0.5:
                    setattr(self, attr, False)
                else:
                    setattr(self, attr, False)
                    print("[control] WARNING: %s axis %d rests at %+.3f, "
                          "not near +1 or -1 -- it is probably the wrong "
                          "axis; run calibrate_wheel.py "
                          "/ %s の軸 %d の静止値が %+.3f で ±1 付近では"
                          "ありません。軸の指定が誤っている可能性があります。"
                          "calibrate_wheel.py を実行してください"
                          % (name, idx, rest, name, idx, rest))

    def _pedal_position(self, value, inverted):
        """Axis value -> 0..1 press position. / 軸値を 0..1 の踏み込み量へ。"""
        pos = (value + 1.0) / 2.0 if inverted else (1.0 - value) / 2.0
        return max(0.0, min(1.0, pos))

    def _parse_vehicle_wheel(self):
        numAxes = self._joystick.get_numaxes()
        jsInputs = [float(self._joystick.get_axis(i)) for i in range(numAxes)]
        # print(jsInputs)  # uncomment to debug axis values / 軸の値のデバッグ時はコメント解除
        jsButtons = [float(self._joystick.get_button(i)) for i in
                     range(self._joystick.get_numbuttons())]

        # Input range: [1, -1] where 1 = released, -1 = fully pressed.
        # Sensitivity values live in wheel_config.ini under [Sensitivity].
        # 入力範囲は [1, -1]。1 = 解放、-1 = 全踏。
        # 感度の設定値は wheel_config.ini の [Sensitivity] にある。
        self._wheel_deg = jsInputs[self._steer_idx] * self._wheel_half_deg
        steerCmd = self._steer_gain * math.tan(
            self._steer_curve * jsInputs[self._steer_idx])
        steerCmd = max(-self._steer_clamp, min(self._steer_clamp, steerCmd))

        # Pedals: normalise the axis (rest = +1, fully pressed = -1) to a
        # 0..1 press position, then apply the response curve and strength
        # cap from pedal_tuning.json (adjustable live on the dashboard).
        # ペダル: 軸値（静止 = +1、全踏 = -1）を 0..1 の踏み込み量に正規化し、
        # pedal_tuning.json の応答カーブと強度上限を適用する
        # （ダッシュボードからライブ調整可能）。
        tpos = self._pedal_position(jsInputs[self._throttle_idx],
                                    self._throttle_inv)
        throttleCmd = self._tuning['throttle_max'] * (
            tpos ** self._tuning['throttle_curve'])
        if tpos < 0.02:            # dead zone / 不感帯
            throttleCmd = 0.0

        bpos = self._pedal_position(jsInputs[self._brake_idx],
                                    self._brake_inv)
        brakeCmd = self._tuning['brake_max'] * (
            bpos ** self._tuning['brake_curve'])
        if bpos < 0.02:
            brakeCmd = 0.0

        self._control.steer = steerCmd * 0.5
        self._control.brake = brakeCmd
        self._control.throttle = throttleCmd

        #toggle = jsButtons[self._reverse_idx]

        self._control.hand_brake = bool(jsButtons[self._handbrake_idx])

    def _parse_walker_keys(self, keys, milliseconds):
        self._control.speed = 0.0
        if keys[K_DOWN] or keys[K_s]:
            self._control.speed = 0.0
        if keys[K_LEFT] or keys[K_a]:
            self._control.speed = .01
            self._rotation.yaw -= 0.08 * milliseconds
        if keys[K_RIGHT] or keys[K_d]:
            self._control.speed = .01
            self._rotation.yaw += 0.08 * milliseconds
        if keys[K_UP] or keys[K_w]:
            self._control.speed = 5.556 if pygame.key.get_mods() & KMOD_SHIFT else 2.778
        self._control.jump = keys[K_SPACE]
        self._rotation.yaw = round(self._rotation.yaw, 1)
        self._control.direction = self._rotation.get_forward_vector()

    @staticmethod
    def _is_quit_shortcut(key):
        return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)


# ==============================================================================
# -- HUD -----------------------------------------------------------------------
# ==============================================================================


# Driver-alert thresholds, shared with the RL rewards via rl/rules.py so
# human feedback and agent training use one definition of "bad driving"
# (see docs/RL_DESIGN.md §3). Falls back to a local copy if the rl package
# is not importable (e.g. the script was copied out of hil_sim/).
# 運転警告のしきい値。rl/rules.py 経由でRL報酬と共有し、「悪い運転」の
# 定義を人間側と学習側で統一する（docs/RL_DESIGN.md §3 参照）。rl
# パッケージが読めない場合（hil_sim/ 外へコピーされた場合など）は
# ローカルコピーにフォールバックする。
try:
    from rl.rules import ALERT_RULES
except ImportError:
    ALERT_RULES = {
        'speeding_kmh_over': 10.0,   # km/h above limit / 制限速度からの超過量
        'speeding_hold_s': 2.0,      # sustained before alerting / 警告までの継続秒数
        'fcw_ttc_s': 1.5,            # forward-collision TTC / 前方衝突警告のTTC
        'fcw_min_kmh': 20.0,         # FCW only above this speed / FCW有効の最低速度
        'thw_warn_s': 1.0,           # tailgating headway / 車間時間警告
    }


class HUD(object):
    def __init__(self, width, height):
        self.dim = (width, height)
        font = pygame.font.Font(pygame.font.get_default_font(), 20)
        font_name = 'courier' if os.name == 'nt' else 'mono'
        fonts = [x for x in pygame.font.get_fonts() if font_name in x]
        default_font = 'ubuntumono'
        mono = default_font if default_font in fonts else fonts[0]
        mono = pygame.font.match_font(mono)
        self._font_mono = pygame.font.Font(mono, 12 if os.name == 'nt' else 14)
        self._notifications = FadingText(font, (width, 40), (0, height - 40))
        self.help = HelpText(pygame.font.Font(mono, 24), width, height)
        self.gauge = Gauge(self.dim)
        self.server_fps = 0
        self.frame = 0
        self.simulation_time = 0
        self._show_info = True
        self._info_text = []
        self._server_clock = pygame.time.Clock()
        # Driver alert banner state / 運転警告バナーの状態
        self._alert_font = pygame.font.Font(mono, 26)
        self._alert = None            # (text, color, until) / (文言, 色, 期限)
        self._alert_cooldown = {}     # category -> earliest next alert time
        self._alert_frame = 0
        self._speeding_since = None
        self._sounds = self._make_sounds()
        # Vehicle roster, rescanned at 1 Hz rather than per frame.
        # world.get_actors() rebuilds a Python object for every actor in the
        # town; this HUD called it twice a frame (FCW + the info panel), so
        # at 60 FPS with 60 vehicles it was ~7200 constructions a second.
        # The roster only changes when traffic spawns or despawns, and the
        # actor handles keep reading live poses from the current snapshot,
        # so caching them costs nothing in accuracy. Same trick as
        # dashboard.py's telemetry loop.
        # 車両名簿は毎フレームではなく1Hzで再取得する。world.get_actors() は
        # 町の全アクター分の Python オブジェクトを作り直すが、この HUD は
        # 毎フレーム2回（前方衝突警告と情報パネル）呼んでいたため、60FPS・
        # 60台で毎秒約7200個を生成していた。名簿が変わるのは交通の出入り時
        # だけで、アクターの参照は最新スナップショットから姿勢を読むため、
        # キャッシュしても精度は落ちない。dashboard.py と同じ手法。
        self._vehicles = []
        self._next_vehicle_scan = 0.0

    @staticmethod
    def _make_sounds():
        """Generate warning beeps in memory (no audio files needed).

        警告音をメモリ上で生成する（音声ファイル不要）。
        """
        try:
            if pygame.mixer.get_init() is None:
                pygame.mixer.init()
            freq, size, channels = pygame.mixer.get_init()

            def tone(hz, dur, vol=0.4):
                n = int(freq * dur)
                t = np.arange(n) / float(freq)
                wave = np.sin(2 * np.pi * hz * t)
                # Short fade in/out against clicks / クリック音防止のフェード
                fade = min(n // 8, 400)
                env = np.ones(n)
                env[:fade] = np.linspace(0, 1, fade)
                env[-fade:] = np.linspace(1, 0, fade)
                samples = (wave * env * vol * 32767).astype(np.int16)
                if channels == 2:
                    samples = np.column_stack((samples, samples))
                return pygame.sndarray.make_sound(np.ascontiguousarray(samples))

            gap = np.zeros(int(freq * 0.06), dtype=np.int16)
            warn = tone(660, 0.15)
            # Double beep for danger / 危険時は2連ビープ
            one = tone(880, 0.12, 0.5)
            arr = pygame.sndarray.array(one)
            gap2 = np.zeros((len(gap),) + arr.shape[1:], dtype=np.int16)
            danger = pygame.sndarray.make_sound(
                np.ascontiguousarray(np.concatenate([arr, gap2, arr])))
            return {'warn': warn, 'danger': danger}
        except Exception:
            return {}   # no audio device: silent alerts / 音声なしでも表示は出す

    def alert(self, category, text, severity='warn',
              seconds=2.5, cooldown=3.0):
        """Show a large driver alert banner (+ beep), rate-limited per
        category so a burst of events does not spam the driver.

        大きな運転警告バナーを表示（ビープ付き）。カテゴリごとに
        クールダウンし、連続イベントで警告が乱発しないようにする。
        """
        now = time.time()
        if now < self._alert_cooldown.get(category, 0.0):
            return
        self._alert_cooldown[category] = now + cooldown
        color = (200, 40, 40) if severity == 'danger' else (200, 150, 20)
        self._alert = (text, color, now + seconds, severity)
        snd = self._sounds.get('danger' if severity == 'danger' else 'warn')
        if snd is not None:
            try:
                snd.play()
            except Exception:
                pass

    def _check_driver_alerts(self, world):
        """Periodic driving-quality checks: speeding and forward TTC.
        (Lane crossing and collision alerts fire from their sensors.)

        定期的な運転品質チェック：速度超過と前方TTC。
        （車線逸脱・衝突はセンサー側から発火する。）
        """
        self._alert_frame = (self._alert_frame + 1) % 10
        if self._alert_frame != 0:      # ~3 Hz at 30 fps / 30fps時に約3Hz
            return
        player = world.player
        try:
            v = player.get_velocity()
            t = player.get_transform()
            speed = 3.6 * math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2)
        except Exception:
            return

        # Speeding, sustained / 速度超過（継続時のみ）
        try:
            limit = float(player.get_speed_limit())
        except Exception:
            limit = 0.0
        now = time.time()
        if limit > 0 and speed > limit + ALERT_RULES['speeding_kmh_over']:
            if self._speeding_since is None:
                self._speeding_since = now
            elif now - self._speeding_since > ALERT_RULES['speeding_hold_s']:
                self.alert('speeding',
                           'SPEEDING %.0f km/h (limit %.0f)' %
                           (speed, limit), 'warn', cooldown=6.0)
        else:
            self._speeding_since = None

        # Forward collision warning via TTC in a 20 deg cone.
        # 前方20度コーン内のTTCによる前方衝突警告。
        if speed < ALERT_RULES['fcw_min_kmh']:
            return
        try:
            yaw = math.radians(t.rotation.yaw)
            fx, fy = math.cos(yaw), math.sin(yaw)
            h_fwd = v.x * fx + v.y * fy
            cos_lim = math.cos(math.radians(20.0))
            best_ttc = None
            for a in self._vehicle_actors(world):
                if a.id == player.id:
                    continue
                al = a.get_location()
                dx, dy = al.x - t.location.x, al.y - t.location.y
                d = math.hypot(dx, dy)
                if d < 0.5 or d > 60.0 or (dx * fx + dy * fy) / d < cos_lim:
                    continue
                av = a.get_velocity()
                closing = h_fwd - (av.x * fx + av.y * fy)
                if closing > 0.5:
                    ttc = d / closing
                    if best_ttc is None or ttc < best_ttc:
                        best_ttc = ttc
            if best_ttc is not None and best_ttc < ALERT_RULES['fcw_ttc_s']:
                self.alert('fcw',
                           'BRAKE! TTC %.1f s' % best_ttc,
                           'danger', cooldown=3.0)
        except Exception:
            pass

    def on_world_tick(self, timestamp):
        self._server_clock.tick()
        self.server_fps = self._server_clock.get_fps()
        self.frame = timestamp.frame
        self.simulation_time = timestamp.elapsed_seconds

    def _vehicle_actors(self, world):
        """Vehicle roster, refreshed at most once a second.

        1秒に1回までしか更新しない車両名簿。
        """
        now = time.time()
        if now >= self._next_vehicle_scan:
            self._next_vehicle_scan = now + 1.0
            try:
                self._vehicles = list(
                    world.world.get_actors().filter('vehicle.*'))
            except Exception:
                self._vehicles = []
        return self._vehicles

    def tick(self, world, clock):
        self._notifications.tick(world, clock)
        self._check_driver_alerts(world)
        if world.player is not None:
            c = world.player.get_control()
            self.gauge.update(world.player, getattr(c, 'manual_gear_shift', False))
        if not self._show_info:
            return
        t = world.player.get_transform()
        v = world.player.get_velocity()
        c = world.player.get_control()
        heading = 'N' if abs(t.rotation.yaw) < 89.5 else ''
        heading += 'S' if abs(t.rotation.yaw) > 90.5 else ''
        heading += 'E' if 179.5 > t.rotation.yaw > 0.5 else ''
        heading += 'W' if -0.5 > t.rotation.yaw > -179.5 else ''
        colhist = world.collision_sensor.get_collision_history()
        collision = [colhist[x + self.frame - 200] for x in range(0, 200)]
        max_col = max(1.0, max(collision))
        collision = [x / max_col for x in collision]
        vehicles = self._vehicle_actors(world)
        self._info_text = [
            'Server:  % 16.0f FPS' % self.server_fps,
            'Client:  % 16.0f FPS' % clock.get_fps(),
            '',
            'Vehicle: % 20s' % get_actor_display_name(world.player, truncate=20),
            'Map:     % 20s' % world.world.get_map().name.split('/')[-1],
            'Simulation time: % 12s' % datetime.timedelta(seconds=int(self.simulation_time)),
            '',
            'Speed:   % 15.0f km/h' % (3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2)),
            u'Heading:% 16.0f\N{DEGREE SIGN} % 2s' % (t.rotation.yaw, heading),
            'Location:% 20s' % ('(% 5.1f, % 5.1f)' % (t.location.x, t.location.y)),
            'GNSS:% 24s' % ('(% 2.6f, % 3.6f)' % (world.gnss_sensor.lat, world.gnss_sensor.lon)),
            'Height:  % 18.0f m' % t.location.z,
            '']
        if isinstance(c, carla.VehicleControl):
            self._info_text += [
                ('Throttle:', c.throttle, 0.0, 1.0),
                ('Steer:', c.steer, -1.0, 1.0),
                ('Brake:', c.brake, 0.0, 1.0),
                ('Reverse:', c.reverse),
                ('Hand brake:', c.hand_brake),
                ('Manual:', c.manual_gear_shift),
                'Gear:        %s' % {-1: 'R', 0: 'N'}.get(c.gear, c.gear)]
        elif isinstance(c, carla.WalkerControl):
            self._info_text += [
                ('Speed:', c.speed, 0.0, 5.556),
                ('Jump:', c.jump)]
        self._info_text += [
            '',
            'Collision:',
            collision,
            '',
            'Number of vehicles: % 8d' % len(vehicles)]
        if len(vehicles) > 1:
            self._info_text += ['Nearby vehicles:']
            distance = lambda l: math.sqrt((l.x - t.location.x)**2 + (l.y - t.location.y)**2 + (l.z - t.location.z)**2)
            vehicles = [(distance(x.get_location()), x) for x in vehicles if x.id != world.player.id]
            # by distance only: two cars at the same distance made sorted() compare
            # the Vehicle objects and crash the HUD / 同距離の車で比較が落ちるため距離のみ
            for d, vehicle in sorted(vehicles, key=lambda dv: dv[0]):
                if d > 200.0:
                    break
                vehicle_type = get_actor_display_name(vehicle, truncate=22)
                self._info_text.append('% 4dm %s' % (d, vehicle_type))

    def toggle_info(self):
        self._show_info = not self._show_info

    def notification(self, text, seconds=2.0):
        self._notifications.set_text(text, seconds=seconds)

    def error(self, text):
        self._notifications.set_text('Error: %s' % text, (255, 0, 0))

    def render(self, display):
        if self._show_info:
            info_surface = pygame.Surface((220, self.dim[1]))
            info_surface.set_alpha(100)
            display.blit(info_surface, (0, 0))
            v_offset = 4
            bar_h_offset = 100
            bar_width = 106
            for item in self._info_text:
                if v_offset + 18 > self.dim[1]:
                    break
                if isinstance(item, list):
                    if len(item) > 1:
                        points = [(x + 8, v_offset + 8 + (1.0 - y) * 30) for x, y in enumerate(item)]
                        pygame.draw.lines(display, (255, 136, 0), False, points, 2)
                    item = None
                    v_offset += 18
                elif isinstance(item, tuple):
                    if isinstance(item[1], bool):
                        rect = pygame.Rect((bar_h_offset, v_offset + 8), (6, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect, 0 if item[1] else 1)
                    else:
                        rect_border = pygame.Rect((bar_h_offset, v_offset + 8), (bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect_border, 1)
                        f = (item[1] - item[2]) / (item[3] - item[2])
                        if item[2] < 0.0:
                            rect = pygame.Rect((bar_h_offset + f * (bar_width - 6), v_offset + 8), (6, 6))
                        else:
                            rect = pygame.Rect((bar_h_offset, v_offset + 8), (f * bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect)
                    item = item[0]
                if item:  # At this point has to be a str. / この時点では必ず文字列。
                    surface = self._font_mono.render(item, True, (255, 255, 255))
                    display.blit(surface, (8, v_offset))
                v_offset += 18
        self.gauge.render(display)
        self._render_alert(display)
        self._notifications.render(display)
        self.help.render(display)

    def _render_alert(self, display):
        """Draw the active alert banner (danger also flashes a red frame).

        表示中の警告バナーを描画する（危険時は赤い枠も点滅）。
        """
        if self._alert is None:
            return
        text, color, until, severity = self._alert
        now = time.time()
        if now > until:
            self._alert = None
            return
        W, H = self.dim
        surf = self._alert_font.render(text, True, (255, 255, 255))
        pad = 16
        bw, bh = surf.get_width() + 2 * pad, surf.get_height() + 14
        bx, by = (W - bw) // 2, int(H * 0.16)
        banner = pygame.Surface((bw, bh))
        banner.set_alpha(215)
        banner.fill(color)
        display.blit(banner, (bx, by))
        display.blit(surf, (bx + pad, by + 7))
        if severity == 'danger':
            # Pulsing red frame / 赤枠の点滅
            pulse = int(120 + 100 * math.sin(now * 12.0))
            frame = pygame.Surface((W, H), pygame.SRCALPHA)
            th = 10
            col = (220, 40, 40, max(0, min(255, pulse)))
            pygame.draw.rect(frame, col, pygame.Rect(0, 0, W, H), th)
            display.blit(frame, (0, 0))


# ==============================================================================
# -- Gauge ---------------------------------------------------------------------
# ==============================================================================


class Gauge(object):
    """Forza-style speedometer: the dial is the rev counter, the speed and
    gear sit in the middle. Bottom-right of the window.

    The red zone starts at the car's shift point: where its engine makes
    peak power (from the torque curve), kept between 75 % and 92 % of max
    RPM so every car has a visible redline. Past it the rev arc turns red
    and, in manual, the gear box flashes with an up arrow: shift up.
    RPM and gear come from Vehicle.get_telemetry_data() (~30 Hz); max RPM,
    torque curve and gear count from the physics control, re-read every 2 s
    (the dashboard's vehicle tuning can change them).

    Forza 風のスピードメーター。ダイヤルが回転計で、中央に速度とギア。
    レッドゾーンは最大出力の回転数（最大回転の 75〜92 %）から。越えると
    回転の弧が赤くなり、MT ではギア表示が点滅して ▲ を出す（シフトアップ）。
    """

    SWEEP = 270.0                   # dial sweep, degrees / ダイヤルの振れ角
    START = 225.0                   # 0 rpm at bottom-left / 0 rpm は左下

    def __init__(self, hud_dim):
        W, H = hud_dim
        self.r = int(max(80, min(140, H * 0.16)))
        self.size = 2 * self.r + 16
        self.pos = (W - self.size - 14, H - self.size - 14)
        bold = pygame.font.match_font('dejavusans,ubuntu,liberationsans,freesans', bold=True)
        self.f_speed = pygame.font.Font(bold, int(self.r * 0.52))
        self.f_gear = pygame.font.Font(bold, int(self.r * 0.30))
        self.f_small = pygame.font.Font(bold, max(10, int(self.r * 0.12)))
        self.f_tick = pygame.font.Font(bold, max(10, int(self.r * 0.14)))
        self.rpm = self.kmh = 0.0
        self.gear, self.manual = 0, False
        self.max_rpm, self.shift_rpm, self.gears = 7000.0, 6000.0, 1
        self._dial = None               # static parts, cached / 静的部分のキャッシュ
        self._dial_key = None
        self._next_tel = self._next_phys = 0.0
        self._player_id = None

    # -- data / データ -------------------------------------------------------
    @staticmethod
    def shift_point(curve, max_rpm):
        """RPM of peak power on the torque curve, within 75-92 % of max.
        トルクカーブ上の最大出力回転数（最大回転の 75〜92 % に制限）。"""
        if not curve or max_rpm <= 0:
            return 0.85 * max_rpm

        def torque(r):
            for (x0, y0), (x1, y1) in zip(curve, curve[1:]):
                if x0 <= r <= x1:
                    return y0 + (y1 - y0) * (r - x0) / max(1e-6, x1 - x0)
            return curve[-1][1] if r > curve[-1][0] else curve[0][1]
        steps = [max_rpm * i / 100.0 for i in range(10, 101)]
        peak = max(steps, key=lambda r: torque(r) * r)
        return min(max(peak, 0.75 * max_rpm), 0.92 * max_rpm)

    def update(self, player, manual):
        """Read the car (rate-limited). / 車両状態を読む（間引きあり）。"""
        now = time.time()
        self.manual = manual
        try:
            v = player.get_velocity()
            self.kmh = 3.6 * math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2)
            if player.id != self._player_id or now >= self._next_phys:
                self._player_id, self._next_phys = player.id, now + 2.0
                pc = player.get_physics_control()
                curve = [(p.x, p.y) for p in pc.torque_curve]
                self.max_rpm = max(1000.0, pc.max_rpm)
                self.shift_rpm = self.shift_point(curve, self.max_rpm)
                self.gears = max(1, len(pc.forward_gears))
            if now >= self._next_tel:
                self._next_tel = now + 1.0 / 30.0
                tel = player.get_telemetry_data()
                self.rpm, self.gear = max(0.0, tel.engine_rpm), tel.gear
        except RuntimeError:
            pass

    # -- drawing / 描画 ------------------------------------------------------
    def _point(self, c, radius, rpm):
        a = math.radians(self.START - self.SWEEP * min(max(rpm / self.max_rpm, 0.0), 1.0))
        return (c + radius * math.cos(a), c - radius * math.sin(a))

    def _band(self, surf, c, r_out, r_in, rpm0, rpm1, color):
        """Filled ring segment from rpm0 to rpm1. / 指定回転域のリング帯。"""
        if rpm1 <= rpm0:
            return
        n = max(2, int(48 * (rpm1 - rpm0) / self.max_rpm))
        outer = [self._point(c, r_out, rpm0 + (rpm1 - rpm0) * i / n) for i in range(n + 1)]
        inner = [self._point(c, r_in, rpm1 - (rpm1 - rpm0) * i / n) for i in range(n + 1)]
        pygame.draw.polygon(surf, color, outer + inner)

    def _build_dial(self):
        s = pygame.Surface((self.size, self.size), pygame.SRCALPHA)
        c, r = self.size // 2, self.r
        pygame.draw.circle(s, (8, 10, 14, 185), (c, c), r + 6)
        pygame.draw.circle(s, (255, 255, 255, 40), (c, c), r + 6, 2)
        self._band(s, c, r - 2, r - 12, 0, self.max_rpm, (255, 255, 255, 38))
        self._band(s, c, r - 2, r - 12, self.shift_rpm, self.max_rpm, (225, 40, 40, 150))
        step = 1000 if self.max_rpm <= 12000 else 2000
        for k in range(0, int(self.max_rpm) + 1, step // 2):
            major = k % step == 0
            red = k >= self.shift_rpm
            col = (235, 60, 60) if red else (230, 230, 230)
            pygame.draw.line(s, col, self._point(c, r - 14, k),
                             self._point(c, r - (26 if major else 20), k), 3 if major else 1)
            if major:
                t = self.f_tick.render(str(k // 1000), True, col)
                x, y = self._point(c, r - 38, k)
                s.blit(t, (x - t.get_width() / 2, y - t.get_height() / 2))
        lbl = self.f_small.render('x1000 rpm', True, (150, 155, 165))
        s.blit(lbl, (c - lbl.get_width() / 2, c + r * 0.44))
        return s

    def render(self, display):
        key = (self.max_rpm, self.shift_rpm, self.r)
        if key != self._dial_key:
            self._dial, self._dial_key = self._build_dial(), key
        surf = self._dial.copy()
        c, r = self.size // 2, self.r
        over = self.rpm >= self.shift_rpm
        # rev arc: white, orange in the last 10 % before the shift point, red past it
        # 回転の弧：白、シフト点手前 10 % で橙、越えたら赤
        col = (235, 50, 50) if over else \
            (255, 160, 40) if self.rpm >= 0.9 * self.shift_rpm else (240, 240, 245)
        self._band(surf, c, r - 2, r - 12, 0, min(self.rpm, self.max_rpm), col)
        tip, base = self._point(c, r - 4, self.rpm), self._point(c, r * 0.66, self.rpm)
        pygame.draw.line(surf, col, base, tip, 3)
        # speed / 速度
        sp = self.f_speed.render('%d' % round(self.kmh), True, (255, 255, 255))
        surf.blit(sp, (c - sp.get_width() / 2, c - sp.get_height() * 0.62))
        kmh = self.f_small.render('km/h', True, (170, 175, 185))
        surf.blit(kmh, (c - kmh.get_width() / 2, c + sp.get_height() * 0.30))
        # gear box: flashes red with an up arrow when it's time to shift (manual)
        # ギア表示：MT でシフト時期になると赤く点滅し ▲ を表示
        gear = {-1: 'R', 0: 'N'}.get(self.gear, str(self.gear))
        shift_now = over and self.manual and 0 < self.gear < self.gears
        flash = shift_now and int(time.time() * 8) % 2 == 0
        g = self.f_gear.render(gear, True, (255, 255, 255))
        bw, bh = max(g.get_width() + 18, int(r * 0.42)), g.get_height() + 4
        box = pygame.Rect(0, 0, bw, bh)
        box.center = (c, int(c + r * 0.80))
        pygame.draw.rect(surf, (225, 40, 40) if flash else (30, 34, 42), box, border_radius=6)
        pygame.draw.rect(surf, (235, 60, 60) if shift_now else (90, 96, 108), box, 2, border_radius=6)
        surf.blit(g, (box.centerx - g.get_width() / 2, box.centery - g.get_height() / 2))
        if shift_now:
            ax, ay = box.right + 10, box.centery
            pygame.draw.polygon(surf, (235, 60, 60), [(ax, ay + 7), (ax + 14, ay + 7), (ax + 7, ay - 7)])
        mode = self.f_small.render('M' if self.manual else 'A', True,
                                   (255, 200, 60) if self.manual else (150, 155, 165))
        surf.blit(mode, (box.left - mode.get_width() - 10, box.centery - mode.get_height() / 2))
        display.blit(surf, self.pos)


# ==============================================================================
# -- FadingText ----------------------------------------------------------------
# ==============================================================================


class FadingText(object):
    def __init__(self, font, dim, pos):
        self.font = font
        self.dim = dim
        self.pos = pos
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)

    def set_text(self, text, color=(255, 255, 255), seconds=2.0):
        text_texture = self.font.render(text, True, color)
        self.surface = pygame.Surface(self.dim)
        self.seconds_left = seconds
        self.surface.fill((0, 0, 0, 0))
        self.surface.blit(text_texture, (10, 11))

    def tick(self, _, clock):
        delta_seconds = 1e-3 * clock.get_time()
        self.seconds_left = max(0.0, self.seconds_left - delta_seconds)
        self.surface.set_alpha(500.0 * self.seconds_left)

    def render(self, display):
        display.blit(self.surface, self.pos)


# ==============================================================================
# -- HelpText ------------------------------------------------------------------
# ==============================================================================


class HelpText(object):
    def __init__(self, font, width, height):
        lines = __doc__.split('\n')
        self.font = font
        self.dim = (680, len(lines) * 22 + 12)
        self.pos = (0.5 * width - 0.5 * self.dim[0], 0.5 * height - 0.5 * self.dim[1])
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)
        self.surface.fill((0, 0, 0, 0))
        for n, line in enumerate(lines):
            text_texture = self.font.render(line, True, (255, 255, 255))
            self.surface.blit(text_texture, (22, n * 22))
            self._render = False
        self.surface.set_alpha(220)

    def toggle(self):
        self._render = not self._render

    def render(self, display):
        if self._render:
            display.blit(self.surface, self.pos)


# ==============================================================================
# -- CollisionSensor -----------------------------------------------------------
# ==============================================================================


class CollisionSensor(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None
        self.history = []
        self._parent = parent_actor
        self.hud = hud
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.collision')
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        # 循環参照を避けるため、ラムダには self への弱参照を渡す。
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: CollisionSensor._on_collision(weak_self, event))

    def get_collision_history(self):
        history = collections.defaultdict(int)
        for frame, intensity in self.history:
            history[frame] += intensity
        return history

    @staticmethod
    def _on_collision(weak_self, event):
        self = weak_self()
        if not self:
            return
        actor_type = get_actor_display_name(event.other_actor)
        self.hud.alert('collision', 'COLLISION with %s' % actor_type,
                       'danger', seconds=3.0, cooldown=2.0)
        impulse = event.normal_impulse
        intensity = math.sqrt(impulse.x**2 + impulse.y**2 + impulse.z**2)
        self.history.append((event.frame, intensity))
        if len(self.history) > 4000:
            self.history.pop(0)


# ==============================================================================
# -- LaneInvasionSensor --------------------------------------------------------
# ==============================================================================


def _signal_side():
    """Current turn-signal side from turn_signal.json ('off' if none).

    turn_signal.json から現在のウインカー状態を返す（無ければ 'off'）。
    """
    try:
        path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), 'turn_signal.json')
        with open(path) as fh:
            side = json.load(fh).get('side', 'off')
        return side if side in ('left', 'right') else 'off'
    except Exception:
        return 'off'


class LaneInvasionSensor(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None
        self._parent = parent_actor
        self.hud = hud
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.lane_invasion')
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        # 循環参照を避けるため、ラムダには self への弱参照を渡す。
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: LaneInvasionSensor._on_invasion(weak_self, event))

    @staticmethod
    def _on_invasion(weak_self, event):
        self = weak_self()
        if not self:
            return
        lane_types = set(x.type for x in event.crossed_lane_markings)
        text = ['%r' % str(x).split()[-1] for x in lane_types]
        # Crossing a solid line is a real driving error -> danger alert;
        # a broken line only gets a mild warning.
        # 実線をまたぐのは明確な運転エラーなので危険警告。破線は軽い注意のみ。
        kinds = ' and '.join(text)
        solid = any('Solid' in str(x) for x in lane_types)
        if solid:
            self.hud.alert('lane', 'LANE DEPARTURE (%s)' % kinds,
                           'danger', seconds=2.5, cooldown=3.0)
        elif _signal_side() != 'off':
            # Broken line crossed with the turn signal on: a legal lane
            # change, so no warning — just a small confirmation.
            # ウインカー点灯中の破線跨ぎは合法な車線変更なので警告しない。
            self.hud.notification(
                'Lane change (%s)' % _signal_side().upper())
        else:
            self.hud.alert('lane', 'Crossed line %s' % kinds,
                           'warn', seconds=2.0, cooldown=3.0)

# ==============================================================================
# -- GnssSensor --------------------------------------------------------
# ==============================================================================


class GnssSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        self.lat = 0.0
        self.lon = 0.0
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.gnss')
        self.sensor = world.spawn_actor(bp, carla.Transform(carla.Location(x=1.0, z=2.8)), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        # 循環参照を避けるため、ラムダには self への弱参照を渡す。
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: GnssSensor._on_gnss_event(weak_self, event))

    @staticmethod
    def _on_gnss_event(weak_self, event):
        self = weak_self()
        if not self:
            return
        self.lat = event.latitude
        self.lon = event.longitude


# ==============================================================================
# -- CameraManager -------------------------------------------------------------
# ==============================================================================


class CameraManager(object):
    """Camera manager with a realistic driver-eye view.

    View 0 is the "Driver Eye": the camera sits where a human driver's head
    would be (left seat, derived from the vehicle bounding box), uses a
    human-like field of view, adds subtle head motion (lean into the corner
    while steering, pitch dive under braking / squat under acceleration) and
    renders rear-view + side mirror insets from three extra low-res cameras,
    horizontally flipped like real mirrors.

    リアルなドライバー視点を備えたカメラマネージャ。

    ビュー 0 は「Driver Eye（ドライバー視点）」: カメラは人間のドライバーの
    頭の位置（左座席、車両バウンディングボックスから算出）に置かれ、人間に
    近い視野角を使用し、頭部の微妙な動き（ステアリング時にコーナー側へ傾く、
    ブレーキ時の前傾／加速時の後傾）を加え、追加の低解像度カメラ 3 台による
    ルームミラー＋サイドミラーの小窓を、実際のミラーと同様に左右反転して
    描画する。
    """

    def __init__(self, parent_actor, hud, fov=100.0):
        self.sensor = None
        self.surface = None
        self._parent = parent_actor
        self.hud = hud
        self.recording = False
        self._fov = float(fov)
        # Driver eye point from the vehicle bounding box: left seat,
        # slightly forward of the cabin centre, ~20 cm below the roof.
        # 車両バウンディングボックスから求めたドライバーの目の位置: 左座席、
        # キャビン中央よりやや前方、ルーフの約 20 cm 下。
        bb = parent_actor.bounding_box
        ext = bb.extent
        # Every vehicle has its seat measured and checked with camera images:
        # your own cars in cars/car_views.json (right-hand drive), CARLA's in
        # cars/carla_views.json (left-hand drive, bikes on the centre line).
        # Anything else: 48 % of the length back from the nose, 30 cm under
        # the roof, left seat -- the rule the measured cars follow (the old
        # guess sat above the roof on most cars, and through it on buses).
        # 全車両の座席位置は計測・画像確認済み（自作は右ハンドル）。未登録の車両は
        # 計測車両から求めた規則で推定する。
        self._view = car_views.load(parent_actor.type_id)
        if self._view is not None:
            self._driver_eye = car_views.eye_location(self._view)
        else:
            self._driver_eye = carla.Location(
                x=bb.location.x + ext.x - 0.96 * ext.x,
                y=bb.location.y - 0.42 * ext.y,
                z=bb.location.z + ext.z - 0.30)
        # Vehicles can provide custom camera positions in cars/car_views.json.
        # Otherwise, derive Hood and Chase transforms from bounding box.
        # 車両独自のカメラ視点が cars/car_views.json に定義されている場合はそれを使用。
        # 未定義の場合はバウンディングボックスから Hood・Chase を算出。
        self._driver_eye_pitch = float(self._view.get('driver_eye_pitch', 0.0)) if self._view else 0.0
        self._driver_eye_yaw = float(self._view.get('driver_eye_yaw', 0.0)) if self._view else 0.0
        if self._view is not None and 'cameras' in self._view:
            self._camera_transforms = []
            self._transform_names = []
            for cam in self._view['cameras']:
                loc = carla.Location(*cam['loc'])
                rot = carla.Rotation(*cam.get('rot', [0.0, 0.0, 0.0]))
                self._camera_transforms.append(carla.Transform(loc, rot))
                self._transform_names.append(cam.get('name', 'Camera'))
            self._driver_eye = self._camera_transforms[0].location
            self._driver_eye_pitch = self._camera_transforms[0].rotation.pitch
            self._driver_eye_yaw = self._camera_transforms[0].rotation.yaw
        else:
            # Hood and chase scale with the car, so a small car and a truck both
            # frame right (they used to be fixed points for every car).
            # ボンネット・追跡視点は車の大きさに合わせる（以前は全車共通の固定位置）。
            top = bb.location.z + ext.z
            self._camera_transforms = [
                carla.Transform(self._driver_eye, carla.Rotation(pitch=self._driver_eye_pitch, yaw=self._driver_eye_yaw)),
                carla.Transform(carla.Location(x=bb.location.x + 0.55 * ext.x, z=top + 0.15)),
                carla.Transform(carla.Location(x=bb.location.x - ext.x - 2.0 - 0.6 * ext.x,
                                               z=top + 1.0 + 0.3 * ext.z),
                                carla.Rotation(pitch=-15))]
            self._transform_names = ['Driver Eye', 'Hood', 'Chase']
        self.transform_index = 0
        # Head dynamics state (driver-eye view only). / 頭部挙動の状態（ドライバー視点のみ）。
        self._head_yaw = 0.0
        self._head_pitch = 0.0
        self.look_input = 0.0
        # Mirrors. / ミラー。
        self.mirrors_enabled = True
        self._mirrors = []          # [(name, sensor), ...]
        self._mirror_surfaces = {}  # name -> pygame surface (already flipped)
        # name -> pygame サーフェス（左右反転済み）
        self._mirror_watchdog = 0   # frames since last aliveness check / 前回の生存確認からのフレーム数
        self.sensors = [
            ['sensor.camera.rgb', cc.Raw, 'Camera RGB'],
            ['sensor.camera.depth', cc.Raw, 'Camera Depth (Raw)'],
            ['sensor.camera.depth', cc.Depth, 'Camera Depth (Gray Scale)'],
            ['sensor.camera.depth', cc.LogarithmicDepth, 'Camera Depth (Logarithmic Gray Scale)'],
            ['sensor.camera.semantic_segmentation', cc.Raw, 'Camera Semantic Segmentation (Raw)'],
            ['sensor.camera.semantic_segmentation', cc.CityScapesPalette,
                'Camera Semantic Segmentation (CityScapes Palette)'],
            ['sensor.lidar.ray_cast', None, 'Lidar (Ray-Cast)']]
        world = self._parent.get_world()
        bp_library = world.get_blueprint_library()
        for item in self.sensors:
            bp = bp_library.find(item[0])
            if item[0].startswith('sensor.camera'):
                bp.set_attribute('image_size_x', str(hud.dim[0]))
                bp.set_attribute('image_size_y', str(hud.dim[1]))
                if bp.has_attribute('fov'):
                    bp.set_attribute('fov', str(self._fov))
            elif item[0].startswith('sensor.lidar'):
                bp.set_attribute('range', '50')
            item.append(bp)
        self.index = None

    def toggle_camera(self):
        self.transform_index = (self.transform_index + 1) % len(self._camera_transforms)
        self._head_yaw = 0.0
        self._head_pitch = 0.0
        self.sensor.set_transform(self._camera_transforms[self.transform_index])
        self._sync_mirrors()
        self.hud.notification(
            'Camera: %s' % self._transform_names[self.transform_index])

    def set_sensor(self, index, notify=True):
        index = index % len(self.sensors)
        needs_respawn = True if self.index is None \
            else self.sensors[index][0] != self.sensors[self.index][0]
        if needs_respawn:
            if self.sensor is not None:
                self.sensor.destroy()
                self.surface = None
            self.sensor = self._parent.get_world().spawn_actor(
                self.sensors[index][-1],
                self._camera_transforms[self.transform_index],
                attach_to=self._parent)
            # We need to pass the lambda a weak reference to self to avoid
            # circular reference.
            # 循環参照を避けるため、ラムダには self への弱参照を渡す。
            weak_self = weakref.ref(self)
            self.sensor.listen(lambda image: CameraManager._parse_image(weak_self, image))
        if notify:
            self.hud.notification(self.sensors[index][2])
        self.index = index
        self._sync_mirrors()

    def next_sensor(self):
        self.set_sensor(self.index + 1)

    def nudge_eye(self, delta):
        """Move the driver's eye (own cars only) and save it. / 視点を動かして保存。"""
        e = self._driver_eye
        self._driver_eye = carla.Location(e.x + delta[0], e.y + delta[1], e.z + delta[2])
        self._camera_transforms[0] = carla.Transform(
            self._driver_eye,
            carla.Rotation(pitch=self._driver_eye_pitch, yaw=self._driver_eye_yaw))
        if self.transform_index == 0 and self.sensor is not None:
            self.sensor.set_transform(self._camera_transforms[0])
        saved = car_views.save_eye(self._parent.type_id, self._driver_eye)
        self.hud.notification('Eye x %.2f  y %.2f  z %.2f%s' % (
            self._driver_eye.x, self._driver_eye.y, self._driver_eye.z,
            ' (saved)' if saved else ''))

    def toggle_recording(self):
        self.recording = not self.recording
        self.hud.notification('Recording %s' % ('On' if self.recording else 'Off'))

    # ------------------------------------------------------------------
    # Driver-eye head dynamics
    # ------------------------------------------------------------------

    def update_head(self, player):
        """Move the driver-eye camera like a human head (call every frame).

        ドライバー視点カメラを人間の頭のように動かす（毎フレーム呼び出す）。"""
        if (self.sensor is None or self.transform_index != 0
                or self.index is None
                or not self.sensors[self.index][0].startswith('sensor.camera')):
            return
        try:
            c = player.get_control()
            a = player.get_acceleration()
            yaw = math.radians(player.get_transform().rotation.yaw)
        except Exception:
            return
        lon_accel = a.x * math.cos(yaw) + a.y * math.sin(yaw)
        steer = getattr(c, 'steer', 0.0)
        if self.look_input:
            target_yaw = 90.0 * self.look_input   # deliberate head check / 意図的な目視確認
            k_yaw = 0.35
        else:
            target_yaw = 15.0 * steer             # look into the corner / コーナーの先を見る
            k_yaw = 0.12
        # Brake dive / acceleration squat, capped at a few degrees.
        # ブレーキ時の前傾／加速時の後傾。数度以内に制限。
        target_pitch = max(-3.5, min(3.5, -0.35 * lon_accel))
        self._head_yaw += k_yaw * (target_yaw - self._head_yaw)
        self._head_pitch += 0.10 * (target_pitch - self._head_pitch)
        self.sensor.set_transform(carla.Transform(
            self._driver_eye,
            carla.Rotation(pitch=self._driver_eye_pitch + self._head_pitch,
                           yaw=self._driver_eye_yaw + self._head_yaw)))

    # ------------------------------------------------------------------
    # Mirrors
    # ------------------------------------------------------------------

    def mirror_sensors(self):
        return [s for _, s in self._mirrors]

    def toggle_mirrors(self):
        self.mirrors_enabled = not self.mirrors_enabled
        self._sync_mirrors()
        self.hud.notification(
            'Mirrors: %s' % ('On' if self.mirrors_enabled else 'Off'))

    def _sync_mirrors(self):
        want = (self.mirrors_enabled and self.transform_index == 0
                and self.index is not None
                and self.sensors[self.index][0] == 'sensor.camera.rgb')
        if want and not self._mirrors:
            self._spawn_mirrors()
        elif not want and self._mirrors:
            self._destroy_mirrors()

    def _mirror_layout(self):
        """Return dict name -> (w, h, blit_x, blit_y) for mirror insets.

        ミラー小窓用の name -> (w, h, blit_x, blit_y) の辞書を返す。"""
        W, H = self.hud.dim
        rw = int(W * 0.26)
        rh = max(1, int(rw / 3.2))
        sw = int(W * 0.17)
        sh = max(1, int(sw / 1.55))
        return {
            'rear':  (rw, rh, (W - rw) // 2, 10),
            'left':  (sw, sh, 10, int(H * 0.42)),
            'right': (sw, sh, W - sw - 10, int(H * 0.42)),
        }

    def _spawn_mirrors(self):
        world = self._parent.get_world()
        bp_lib = world.get_blueprint_library()
        bb = self._parent.bounding_box
        ext = bb.extent
        layout = self._mirror_layout()
        specs = {
            # name: (transform, fov)
            # Rear camera sits ABOVE the roof line (+0.15 m): at roof level
            # or below, the camera ends up inside the roof/hatch mesh on
            # many vehicle models and the mirror renders black.
            # リアカメラはルーフラインより上（+0.15 m）に置く。ルーフと同じ
            # 高さ以下では、多くの車両モデルでカメラがルーフ／ハッチのメッシュ
            # 内部に入り込み、ミラーが真っ黒に描画されるため。
            'rear': (carla.Transform(
                carla.Location(x=bb.location.x - 0.30 * ext.x,
                               z=bb.location.z + ext.z + 0.15),
                carla.Rotation(pitch=-3.0, yaw=180.0)), 60.0),
            'left': (carla.Transform(
                carla.Location(x=bb.location.x + 0.45 * ext.x,
                               y=bb.location.y - (ext.y + 0.15),
                               z=bb.location.z + 0.35 * ext.z),
                carla.Rotation(yaw=192.0)), 55.0),
            'right': (carla.Transform(
                carla.Location(x=bb.location.x + 0.45 * ext.x,
                               y=bb.location.y + (ext.y + 0.15),
                               z=bb.location.z + 0.35 * ext.z),
                carla.Rotation(yaw=168.0)), 55.0),
        }
        weak_self = weakref.ref(self)
        for name, (tf, fov) in specs.items():
            w, h = layout[name][0], layout[name][1]
            bp = bp_lib.find('sensor.camera.rgb')
            bp.set_attribute('image_size_x', str(w))
            bp.set_attribute('image_size_y', str(h))
            bp.set_attribute('fov', str(fov))
            if MIRROR_SENSOR_TICK > 0.0 and bp.has_attribute('sensor_tick'):
                bp.set_attribute('sensor_tick', str(MIRROR_SENSOR_TICK))
            # Retry a bit higher if the spawn collides with the bodywork.
            # スポーンが車体と干渉する場合は少し高い位置で再試行する。
            sensor = None
            for _attempt in range(3):
                try:
                    sensor = world.spawn_actor(
                        bp, tf, attach_to=self._parent)
                    break
                except Exception:
                    tf = carla.Transform(
                        carla.Location(x=tf.location.x, y=tf.location.y,
                                       z=tf.location.z + 0.25),
                        tf.rotation)
            if sensor is None:
                print("Warning: could not spawn '%s' mirror camera" % name)
                continue
            sensor.listen(
                (lambda n: lambda image:
                    CameraManager._parse_mirror(weak_self, n, image))(name))
            self._mirrors.append((name, sensor))

    def _destroy_mirrors(self):
        for _, s in self._mirrors:
            try:
                s.stop()
            except Exception:
                pass
            try:
                s.destroy()
            except Exception:
                pass
        self._mirrors = []
        self._mirror_surfaces = {}

    @staticmethod
    def _parse_mirror(weak_self, name, image):
        self = weak_self()
        if not self:
            return
        surface = bgra_surface(image)
        # Real mirrors are horizontally flipped. / 実際のミラーと同様に左右反転する。
        self._mirror_surfaces[name] = pygame.transform.flip(
            surface, True, False)

    def render(self, display):
        if self.surface is not None:
            display.blit(self.surface, (0, 0))
        if self._mirrors and self.transform_index == 0:
            # Watchdog: if CARLA killed a mirror sensor (map/layer change,
            # respawn glitch), rebuild all mirrors so they come back instead
            # of silently disappearing.
            # ウォッチドッグ: CARLA がミラーセンサーを破棄した場合
            # （マップ／レイヤー変更、リスポーン不具合）、黙って消えたままに
            # ならないよう全ミラーを再構築する。
            self._mirror_watchdog += 1
            if self._mirror_watchdog >= 120:
                self._mirror_watchdog = 0
                try:
                    dead = any(not s.is_alive for _, s in self._mirrors)
                except Exception:
                    dead = True
                if dead:
                    self._destroy_mirrors()
                    self._sync_mirrors()
                    return
            layout = self._mirror_layout()
            for name, (w, h, x, y) in layout.items():
                surf = self._mirror_surfaces.get(name)
                if surf is None:
                    # Placeholder until the first frame arrives, so the
                    # mirror slot is visible instead of missing.
                    # 最初のフレームが届くまでのプレースホルダー。
                    # ミラー枠が欠けずに見えるようにする。
                    pygame.draw.rect(display, (12, 12, 15),
                                     pygame.Rect(x, y, w, h))
                else:
                    display.blit(surf, (x, y))
                pygame.draw.rect(display, (25, 25, 28),
                                 pygame.Rect(x - 2, y - 2, w + 4, h + 4), 3)

    @staticmethod
    def _parse_image(weak_self, image):
        self = weak_self()
        if not self:
            return
        if self.sensors[self.index][0].startswith('sensor.lidar'):
            points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
            points = np.reshape(points, (int(points.shape[0] / 4), 4))
            lidar_data = np.array(points[:, :2])
            lidar_data *= min(self.hud.dim) / 100.0
            lidar_data += (0.5 * self.hud.dim[0], 0.5 * self.hud.dim[1])
            lidar_data = np.fabs(lidar_data) # pylint: disable=E1111
            lidar_data = lidar_data.astype(np.int32)
            lidar_data = np.reshape(lidar_data, (-1, 2))
            lidar_img_size = (self.hud.dim[0], self.hud.dim[1], 3)
            lidar_img = np.zeros(lidar_img_size)
            lidar_img[tuple(lidar_data.T)] = (255, 255, 255)
            self.surface = pygame.surfarray.make_surface(lidar_img)
        else:
            image.convert(self.sensors[self.index][1])
            self.surface = bgra_surface(image)
        if self.recording:
            image.save_to_disk('_out/%08d' % image.frame)


def bgra_surface(image):
    """CARLA camera image -> pygame Surface, without a numpy round-trip.

    CARLA delivers BGRA rows, which SDL can ingest directly; .convert() then
    copies once into the display's own pixel format so later blits are a
    straight memcpy. The old path (frombuffer -> slice -> reverse -> swapaxes
    -> make_surface) cost ~1.2 ms per 720p frame on the client CPU, this one
    ~0.2 ms, for byte-identical pixels.

    CARLA のカメラ画像を numpy を経由せず pygame Surface に変換する。
    CARLA は BGRA 行を返すので SDL がそのまま取り込める。.convert() で
    ディスプレイのピクセル形式へ1回だけコピーするため、以後の blit は
    単純な memcpy になる。旧経路は 720p 1フレームあたり約 1.2 ms を
    クライアントCPUで消費していたが、本経路は約 0.2 ms で画素は同一。
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
        # Older pygame without the "BGRA" format, or no display yet.
        # "BGRA" 未対応の古い pygame、またはディスプレイ未初期化の場合。
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3][:, :, ::-1]
        return pygame.surfarray.make_surface(array.swapaxes(0, 1))


# Seconds between mirror-camera frames; 0 = every simulation frame.
# Overwritten by game_loop() from --mirror-fps.
# ミラーカメラのフレーム間隔（秒）。0 で毎シミュレーションフレーム。
# game_loop() が --mirror-fps から上書きする。
MIRROR_SENSOR_TICK = 1.0 / 30.0


# ==============================================================================
# -- game_loop() ---------------------------------------------------------------
# ==============================================================================


# resolution presets for --res / --res の略称
RES_PRESETS = {'720p': (1280, 720), '1080p': (1920, 1080), '1440p': (2560, 1440),
               '4k': (3840, 2160), '2160p': (3840, 2160)}


def parse_res(text):
    """'1080p' or '1920x1080' -> (1920, 1080). / 解像度の文字列を (幅, 高さ) に。"""
    t = text.strip().lower()
    if t in RES_PRESETS:
        return RES_PRESETS[t]
    try:
        w, h = [int(x) for x in t.split('x')]
    except ValueError:
        raise SystemExit('--res: use WxH (e.g. 1920x1080) or one of %s' % ', '.join(RES_PRESETS))
    return w, h


def game_loop(args):
    global MIRROR_SENSOR_TICK
    # Mirrors are throttled server-side rather than per-frame client-side:
    # a capped sensor_tick skips the render pass entirely, so the saving is
    # on both the GPU (no draw) and the CPU (no readback, no conversion).
    # ミラーはクライアント側ではなくサーバ側で間引く。sensor_tick を設定
    # すると描画パス自体が省かれるため、GPU（描画なし）とCPU（読み戻し・
    # 変換なし）の両方が軽くなる。
    MIRROR_SENSOR_TICK = (1.0 / args.mirror_fps) if args.mirror_fps > 0 else 0.0

    pygame.init()
    pygame.font.init()
    world = None

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(20.0)

        display = pygame.display.set_mode(
            (args.width, args.height),
            pygame.RESIZABLE | pygame.SCALED | pygame.DOUBLEBUF)
        pygame.display.set_caption('CARLA Manual Control (G923)')

        hud = HUD(args.width, args.height)
        world = World(client, client.get_world(), hud, args.filter, args.fov,
                      start_vehicle=args.start_vehicle)
        controller = DualControl(world, args.autopilot, args.keyboard)

        clock = pygame.time.Clock()
        while True:
            # tick() sleeps until the next frame; tick_busy_loop() spins on
            # the CPU and pins a whole core for the same 60 FPS.
            # tick() は次フレームまでスリープする。tick_busy_loop() は
            # CPU をスピンさせ、同じ60FPSのために1コアを占有してしまう。
            clock.tick(args.fps)
            if controller.parse_events(world, clock):
                return
            world.tick(clock)
            world.render(display)
            pygame.display.flip()

    finally:

        if world is not None:
            world.destroy()

        pygame.quit()


# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Manual Control Client')
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-a', '--autopilot',
        action='store_true',
        help='enable autopilot')
    argparser.add_argument(
        '--keyboard',
        action='store_true',
        help='drive with W/A/S/D instead of the wheel (this is also the '
             'automatic fallback when no G923/G29 is detected)')
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default='1920x1080',
        help='window resolution: WxH or 720p / 1080p / 1440p / 4k (default: 1080p)')
    argparser.add_argument(
        '--filter',
        metavar='PATTERN',
        default='vehicle.*',
        help='actor filter (default: "vehicle.*")')
    argparser.add_argument(
        '--start-vehicle',
        metavar='BLUEPRINT',
        default='',
        help='first car, e.g. vehicle.mazda.rx7 (Backspace still cycles through --filter)')
    argparser.add_argument(
        '--fov',
        metavar='DEG',
        default=100.0,
        type=float,
        help='camera horizontal field of view in degrees (default: 100, '
             'close to natural human central vision on a monitor)')
    argparser.add_argument(
        '--fps',
        metavar='N',
        default=60,
        type=int,
        help='client render/render-loop cap in frames per second '
             '(default: 60). Lower it to give CPU back to the simulator '
             '/ クライアント描画ループの上限FPS（既定 60）。'
             '下げるとCPUをシミュレータ側に回せる')
    argparser.add_argument(
        '--mirror-fps',
        metavar='N',
        default=30,
        type=float,
        help='update rate of the three mirror cameras (default: 30). Each '
             'mirror is a full extra render pass on the server, so this is '
             'the cheapest big saving. 0 = uncapped '
             '/ 3つのミラーカメラの更新レート（既定 30）。'
             'ミラー1枚ごとにサーバ側で追加の描画パスが走るため、'
             'ここを下げるのが最も効果的。0 で無制限')
    args = argparser.parse_args()

    args.width, args.height = parse_res(args.res)

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s', args.host, args.port)

    print(__doc__)

    try:

        game_loop(args)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')


if __name__ == '__main__':

    main()
