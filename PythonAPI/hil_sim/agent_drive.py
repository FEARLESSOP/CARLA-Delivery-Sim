#!/usr/bin/env python
"""Drive an autonomous `ego` vehicle in the same world as the human hero.

This is the HIL co-simulation client from docs/RL_DESIGN.md §1: the human
drives the `hero` vehicle with the G923 while this script drives an `ego`
vehicle (role_name='ego'). The agent's driving is scored live with the
same rules as the RL reward / driver alerts (rl/rules.py), so its
behaviour is directly comparable to the human sessions.

Policies / ポリシー:
    --policy autopilot        CARLA TrafficManager baseline (default)
    --policy model:PATH       trained stable-baselines3 model (.zip)

Usage / 使い方:
    python agent_drive.py                       # autopilot ego
    python agent_drive.py --policy model:rl/results/models/sac_ego.zip
    # or via the pipeline / パイプライン経由:
    python run_pipeline.py --agent autopilot

人間がG923で `hero` を運転する同一ワールド内で、自動運転の `ego`
（role_name='ego'）を走らせるHIL共走クライアント（docs/RL_DESIGN.md §1）。
エージェントの運転はRL報酬・運転警告と同じルール（rl/rules.py）で
リアルタイム採点され、人間セッションと直接比較できる。
"""

import argparse
import glob
import math
import os
import random
import signal
import sys
import time

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
sys.path.append('../carla')

import carla  # noqa: E402

from rl.rules import step_reward  # noqa: E402


def spawn_ego(world, vehicle_filter, rolename='ego', retries=20):
    """Spawn the ego vehicle at a free spawn point.

    空いているスポーン地点にego車をスポーンする。
    """
    bp_lib = world.get_blueprint_library()
    bps = bp_lib.filter(vehicle_filter) or bp_lib.filter('vehicle.*')
    bp = random.choice(list(bps))
    bp.set_attribute('role_name', rolename)
    spawn_points = world.get_map().get_spawn_points()
    random.shuffle(spawn_points)
    for sp in spawn_points[:retries]:
        ego = world.try_spawn_actor(bp, sp)
        if ego is not None:
            return ego
    raise RuntimeError('no free spawn point for ego / '
                       'ego車のスポーン地点が空いていない')


def run_autopilot(client, world, ego, tm_port, score_cb):
    """Baseline: TrafficManager drives, we only score.

    ベースライン：TrafficManagerが運転し、本スクリプトは採点のみ行う。
    """
    # A TrafficManager instance must exist in this client, otherwise
    # set_autopilot() silently does nothing and the ego never moves.
    # TrafficManagerをこのクライアント内に生成しておく必要がある。
    # ないと set_autopilot() は何もせず ego は動かない。
    client.get_trafficmanager(tm_port)
    ego.set_autopilot(True, tm_port)
    while True:
        world.wait_for_tick()
        score_cb()


def run_model(world, ego, model_path, score_cb):
    """Trained policy drives via the same observation as training.

    学習時と同一の観測を使って学習済みポリシーが運転する。
    """
    try:
        from stable_baselines3 import SAC, PPO
    except ImportError:
        print('[agent] stable-baselines3 not installed: '
              'pip install stable-baselines3 / 未インストールです')
        return 1
    from rl.carla_env import EgoSensing
    from agents.navigation.global_route_planner import GlobalRoutePlanner

    loader = SAC if 'sac' in os.path.basename(model_path).lower() else PPO
    model = loader.load(model_path)
    planner = GlobalRoutePlanner(world.get_map(), sampling_resolution=2.0)
    sensing = None
    prev_t = None
    while True:
        snap = world.wait_for_tick()
        now = snap.timestamp.elapsed_seconds
        dt = (now - prev_t) if prev_t else 0.05
        prev_t = now
        if sensing is None or (sensing.route
                               and sensing.route_idx >= len(sensing.route) - 3):
            # New random route when the old one is finished.
            # 経路を走り切ったら新しいランダム経路を引く。
            sps = world.get_map().get_spawn_points()
            goal = random.choice(sps).location
            route = planner.trace_route(ego.get_location(), goal)
            sensing = EgoSensing(world, ego, route)
            print('[agent] new route: %d points' % len(route))
        _m, obs = sensing.measure(max(1e-3, dt))
        action, _ = model.predict(obs, deterministic=True)
        ego.apply_control(carla.VehicleControl(
            throttle=float(max(0.0, action[1])),
            brake=float(max(0.0, -action[1])),
            steer=float(action[0])))
        score_cb()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--host', default='127.0.0.1')
    p.add_argument('--port', type=int, default=2000)
    p.add_argument('--tm-port', type=int, default=8000)
    p.add_argument('--policy', default='autopilot',
                   help='"autopilot" or "model:PATH_TO_SB3_ZIP"')
    p.add_argument('--filter', default='vehicle.tesla.model3',
                   help='ego blueprint filter / ego車のブループリント')
    p.add_argument('--rolename', default='ego',
                   help='"ego" beside a human hero; "hero" to be logged by '
                        'the dashboard like a human session (benchmark). '
                        '/ 人間と共走なら "ego"、ダッシュボードに人間'
                        'セッションと同様に記録させるなら "hero"。')
    args = p.parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(20.0)
    world = client.get_world()
    ego = spawn_ego(world, args.filter, args.rolename)
    print('[agent] ego spawned: %s (id=%d), policy=%s'
          % (ego.type_id, ego.id, args.policy))

    # Live scoring with the shared rules — lightweight: events that need
    # sensors (collision/lane) are skipped here; per-step terms only.
    # 共有ルールによるリアルタイム採点（軽量版）。センサが必要な
    # イベント項は省略し、ステップ毎の項のみ集計する。
    from rl.carla_env import _front_obstacle
    score = {'r': 0.0, 't0': time.time(), 'last_print': 0.0}

    def score_cb():
        v = ego.get_velocity()
        speed_kmh = 3.6 * math.hypot(v.x, v.y)
        others = [a for a in world.get_actors().filter('vehicle.*')
                  if a.id != ego.id]
        front_d, ttc = _front_obstacle(ego, others)
        speed_ms = speed_kmh / 3.6
        thw = (front_d / speed_ms) if (0.0 < front_d < 60.0
                                       and speed_ms > 0.5) else -1.0
        m = {'speed_kmh': speed_kmh,
             'speed_limit_kmh': float(ego.get_speed_limit() or 0.0),
             'ttc': ttc, 'thw': thw}
        r, _b = step_reward(m, {})
        score['r'] += r
        now = time.time()
        if now - score['last_print'] > 10.0:
            score['last_print'] = now
            print('[agent] %.0fs, %.0f km/h, cumulative rule score %.1f'
                  % (now - score['t0'], speed_kmh, score['r']))

    def cleanup(*_a):
        print('\n[agent] destroying ego (final rule score %.1f)'
              % score['r'])
        try:
            ego.destroy()
        except Exception:
            pass
        sys.exit(0)
    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)

    try:
        if args.policy.startswith('model:'):
            return run_model(world, ego, args.policy[6:], score_cb)
        return run_autopilot(client, world, ego, args.tm_port, score_cb)
    finally:
        try:
            ego.destroy()
        except Exception:
            pass


if __name__ == '__main__':
    sys.exit(main())
