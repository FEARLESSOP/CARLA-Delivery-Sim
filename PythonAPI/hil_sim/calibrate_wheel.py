#!/usr/bin/env python3
"""Detect the G923/G29 axis layout and write wheel_config.ini.

The G923 enumerates differently depending on its mode switch: in PC mode
(USB id 046d:c266) the pedals sit on ABS_Y/ABS_Z and rest at +1, while in
PS4 mode (046d:c267) they land on the trigger-style axes and rest at -1.
Hard-coded indices that fit one mode silently mis-read the other -- a
pedal stuck at half-press, so the car crawls or refuses to move at all.

This script asks you to press each control, watches which axis actually
moves, and writes the result (including per-pedal inversion) to
wheel_config.ini.

G923/G29 の軸配置を検出し wheel_config.ini を書き出す。

G923 はモードによって列挙が変わる。PC モード (046d:c266) ではペダルは
ABS_Y/ABS_Z にあり静止値は +1、PS4 モード (046d:c267) ではトリガー系の
軸に割り当てられ静止値は -1 になる。片方に合わせた固定インデックスは
もう片方で誤読され、ペダルが半踏み状態に固定されて車が動かなくなる。

本スクリプトは各操作を実際に踏んで（回して）もらい、動いた軸を検出して
反転の有無も含め wheel_config.ini に書き出す。

    <CARLA_ROOT>/carla-venv/bin/python calibrate_wheel.py
"""

import os
import sys
import time

os.environ.setdefault('SDL_VIDEODRIVER', 'dummy')

import pygame

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG = os.path.join(HERE, 'wheel_config.ini')

SETTLE = 0.4        # seconds to let an axis settle / 軸が落ち着くまでの秒数
MOVE_MIN = 0.25     # ignore drift/noise below this / これ未満はノイズとして無視


def sample(joy):
    """Current value of every axis. / 全軸の現在値。"""
    pygame.event.pump()
    return [joy.get_axis(i) for i in range(joy.get_numaxes())]


def rest_values(joy):
    """Average the axes over a short window. / 短時間平均で静止値を取る。"""
    acc = None
    for _ in range(20):
        vals = sample(joy)
        acc = vals if acc is None else [a + b for a, b in zip(acc, vals)]
        time.sleep(0.02)
    return [v / 20.0 for v in acc]


def capture(joy, rest, label_en, label_ja):
    """Wait for one axis to move far from rest, and report it.

    静止値から大きく動いた軸を待って報告する。
    """
    print("\n>>> %s\n>>> %s" % (label_en, label_ja))
    print("    (hold it, then release / 踏み込んだまま数秒 → 離す)")

    best_idx, best_val, best_dev = None, None, 0.0
    deadline = time.time() + 15.0
    while time.time() < deadline:
        vals = sample(joy)
        for i, v in enumerate(vals):
            dev = abs(v - rest[i])
            if dev > best_dev:
                best_idx, best_val, best_dev = i, v, dev
        if best_dev > MOVE_MIN and time.time() > deadline - 12.0:
            # Movement seen; give the user a moment to reach full travel.
            # 動きを検出。全ストロークまでの余裕を与える。
            time.sleep(SETTLE)
            vals = sample(joy)
            for i, v in enumerate(vals):
                dev = abs(v - rest[i])
                if dev > best_dev:
                    best_idx, best_val, best_dev = i, v, dev
            break
        time.sleep(0.02)

    if best_idx is None or best_dev < MOVE_MIN:
        print("    !! no movement detected / 動きを検出できませんでした")
        return None

    # Inverted when the pressed value is greater than rest: the driving
    # script normalises with (1 - v) / 2, which assumes rest=+1 pressed=-1.
    # 押した値が静止値より大きければ反転。運転側は (1 - v) / 2 で
    # 正規化しており、静止=+1・全踏=-1 を前提とするため。
    inverted = best_val > rest[best_idx]
    print("    axis %d  rest=%+.3f  pressed=%+.3f  inverted=%s"
          % (best_idx, rest[best_idx], best_val, inverted))

    print("    release it / 離してください")
    time.sleep(1.2)
    return best_idx, inverted


def main():
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("No joystick found / ジョイスティックが見つかりません")
        return 1

    joy = None
    for i in range(pygame.joystick.get_count()):
        cand = pygame.joystick.Joystick(i)
        cand.init()
        if 'G923' in cand.get_name() or 'G29' in cand.get_name():
            joy = cand
            break
    if joy is None:
        joy = pygame.joystick.Joystick(0)
        joy.init()

    print("Wheel / ホイール: %s" % joy.get_name())
    print("Axes / 軸数: %d" % joy.get_numaxes())
    print("\nKeep hands and feet OFF the wheel and pedals."
          "\nホイールとペダルから手足を離してください。")
    time.sleep(2.0)

    rest = rest_values(joy)
    print("Rest values / 静止値: %s"
          % ['%+.3f' % v for v in rest])

    throttle = capture(joy, rest,
                       "Press the ACCELERATOR pedal fully.",
                       "アクセルペダルを一番奥まで踏んでください。")
    brake = capture(joy, rest,
                    "Press the BRAKE pedal fully.",
                    "ブレーキペダルを一番奥まで踏んでください。")
    steer = capture(joy, rest,
                    "Turn the STEERING WHEEL fully to one side.",
                    "ステアリングを片側いっぱいまで回してください。")

    if not (throttle and brake and steer):
        print("\nCalibration incomplete; wheel_config.ini left untouched."
              "\nキャリブレーション未完了のため wheel_config.ini は変更しません。")
        return 1

    if throttle[0] == brake[0]:
        print("\n!! Throttle and brake resolved to the same axis (%d)."
              "\n!! アクセルとブレーキが同じ軸 (%d) になりました。"
              "\n   Re-run and press only one pedal at a time."
              "\n   一度に片方のペダルだけを踏んで再実行してください。"
              % (throttle[0], throttle[0]))
        return 1

    text = """[G29 Racing Wheel]
# Written by calibrate_wheel.py / calibrate_wheel.py が生成
# Wheel / ホイール: %s

# Axis indices / 軸インデックス
steering_wheel = %d
throttle = %d
brake = %d

# Pedal direction / ペダルの向き
# false = rest +1, pressed -1 (G923 PC mode) / 静止 +1、全踏 -1（PCモード）
# true  = rest -1, pressed +1 (G923 PS4 mode) / 静止 -1、全踏 +1（PS4モード）
throttle_invert = %s
brake_invert = %s

# Button indices / ボタンインデックス
reverse = 6
handbrake = 7

[Sensitivity]
# Steering: output = tan(steer_curve * input) * steer_gain, clamped to +/- steer_clamp.
# Lower steer_curve = less twitchy near centre. Lower steer_gain = less overall response.
# Keep steer_gain = 1 / tan(steer_curve) so full lock lands at the wheel's 900 deg lock.
steer_gain = 1.83
steer_curve = 0.5
steer_clamp = 1.0

# Pedal sensitivity moved to pedal_tuning.json (created on first run).
# Adjust it live from the dashboard's "Pedal Tuning" card, or in the sim
# with [ / ] (throttle strength) and Shift+[ / Shift+] (brake strength).
#   *_max   = output at full pedal press (0.1 .. 1.0)
#   *_curve = response exponent; higher = gentler at light press
""" % (joy.get_name(), steer[0], throttle[0], brake[0],
       str(throttle[1]).lower(), str(brake[1]).lower())

    with open(CONFIG, 'w') as fh:
        fh.write(text)

    print("\nWrote %s" % CONFIG)
    print("書き込みました: %s" % CONFIG)
    print("  steering_wheel = %d" % steer[0])
    print("  throttle       = %d (invert=%s)" % (throttle[0], str(throttle[1]).lower()))
    print("  brake          = %d (invert=%s)" % (brake[0], str(brake[1]).lower()))
    return 0


if __name__ == '__main__':
    sys.exit(main())
