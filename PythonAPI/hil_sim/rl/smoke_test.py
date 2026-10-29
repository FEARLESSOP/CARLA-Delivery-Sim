#!/usr/bin/env python
"""Quick sanity check of CarlaHILEnv without any RL library.

Runs a few episodes with a trivial lane-keeping controller (P-control on
lane offset + gentle throttle) and prints the reward breakdown, so you can
verify the environment, route, sensors and reward wiring before training.

Usage / 使い方 (CARLA must be running / CARLA起動後に):
    python -m rl.smoke_test --episodes 2

RLライブラリなしで環境を検証するスモークテスト。車線オフセットの
P制御＋弱いスロットルで数エピソード走らせ、報酬の内訳を表示する。
学習を始める前に環境・経路・センサ・報酬の配線を確認できる。
"""

import argparse
import os
import sys
from collections import defaultdict

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from rl.carla_env import CarlaHILEnv  # noqa: E402


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--host', default='127.0.0.1')
    p.add_argument('--port', type=int, default=2000)
    p.add_argument('--episodes', type=int, default=2)
    p.add_argument('--max-steps', type=int, default=600)
    args = p.parse_args()

    env = CarlaHILEnv(host=args.host, port=args.port, seed=1)
    try:
        for ep in range(args.episodes):
            obs, info = env.reset()
            total, terms = 0.0, defaultdict(float)
            for step in range(args.max_steps):
                m = info['measurements']
                # Trivial controller: steer back to lane centre, cruise.
                # 単純制御：車線中心へ戻す操舵＋一定スロットル。
                steer = -0.5 * m['lane_offset'] - 0.8 * m['heading_err']
                # Proportional throttle: smooth, avoids self-induced jerk.
                # 比例スロットル：滑らかでジャークを自作しない。
                accel = max(0.0, min(0.5, (25.0 - m['speed_kmh']) * 0.05))
                obs, r, term, trunc, info = env.step([steer, accel])
                total += r
                for k, v in info['reward_breakdown'].items():
                    terms[k] += v
                if term or trunc:
                    break
            print('[smoke] ep%d: steps=%d total=%.1f term=%s trunc=%s'
                  % (ep, step + 1, total, term, trunc))
            for k in sorted(terms):
                print('         %-12s %8.2f' % (k, terms[k]))
    finally:
        env.close()


if __name__ == '__main__':
    main()
