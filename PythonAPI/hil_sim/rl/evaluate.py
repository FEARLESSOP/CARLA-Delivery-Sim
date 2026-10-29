#!/usr/bin/env python
"""Evaluate a trained checkpoint and append the result to eval_history.csv.

Runs N deterministic episodes of CarlaHILEnv with the given model and
records mean reward plus the safety metrics (collisions, lane crossings,
red lights, completion rate, speed) and the CARLA-Leaderboard-style
scores used across the CARLA RL literature -- Route Completion (RC),
Infraction Score (IS) and Driving Score (DS = RC x IS), plus infractions
per km -- so the study's numbers read like everyone else's. Each call
appends ONE row to
`rl/results/eval_history.csv`, keyed by the model's training timesteps —
evaluating several checkpoints builds the improvement curve that
`rl/plot_progress.py` turns into a graph (docs/RL_DESIGN.md §6).

Usage / 使い方 (CARLA must be running / CARLAを起動してから):
    python -m rl.evaluate --model rl/results/models/sac_ego.zip --episodes 5
    # evaluate every checkpoint / 全チェックポイントを順に評価:
    for m in rl/results/models/sac_ego_ckpt_*_steps.zip; do
        python -m rl.evaluate --model "$m" --episodes 3
    done

学習済みチェックポイントを評価し、結果を eval_history.csv に追記する。
決定論的にNエピソード走らせ、平均報酬と安全指標（衝突・車線逸脱・
信号無視・完走率・速度）を記録する。行は学習ステップ数をキーに追記
されるため、複数チェックポイントを評価すると「改善カーブ」ができ、
rl/plot_progress.py でグラフ化できる。
"""

import argparse
import csv
import os
import sys
import time

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from rl import paths  # noqa: E402

from rl.carla_env import CarlaHILEnv  # noqa: E402
from rl.rules import ALERT_RULES  # noqa: E402

CSV_FIELDS = [
    'wall_time', 'model', 'train_timesteps', 'episodes',
    'mean_reward', 'min_reward', 'max_reward', 'mean_steps',
    'collisions_per_ep', 'lane_cross_per_ep', 'red_lights_per_ep',
    'off_road_per_ep', 'completion_rate', 'avg_speed_kmh',
    # Study bookkeeping (docs/RESEARCH_PROTOCOL.md §3-§4): which arm the
    # model was trained in, its seed, and which traffic it was evaluated
    # against. Rows from different arms are only comparable when
    # `eval_traffic` matches. Near-misses are TTC dips below the FCW
    # threshold -- the same event the primary endpoint counts.
    # 研究の帳簿（docs/RESEARCH_PROTOCOL.md §3〜§4）：学習アーム・シード・
    # 評価に使った交通。アーム間の比較は eval_traffic が一致する行同士に
    # 限る。ニアミスは FCW しきい値を下回る TTC の低下で、主要評価項目と
    # 同じイベント。
    'arm', 'seed', 'eval_traffic', 'map', 'near_miss_per_ep',
    # CARLA Leaderboard conventions (leaderboard.carla.org/evaluation_v1_0):
    # RC = fraction of the route completed, IS = product of per-infraction
    # penalties, DS = RC x IS; global infractions are events per km.
    # CARLA Leaderboard の慣例：RC = 経路完了率、IS = 違反ペナルティの積、
    # DS = RC x IS。全体の違反数は km あたりで表す。
    'route_completion', 'infraction_score', 'driving_score',
    'total_km', 'collisions_per_km', 'red_lights_per_km',
    'near_miss_per_km', 'mean_min_ttc',
]

# Leaderboard 1.0/2.0 penalty coefficients. The env's collision sensor
# does not say what was hit, so every collision takes the "vehicle"
# coefficient (0.60) -- the harshest of the three collision classes.
# Leaderboard 1.0/2.0 のペナルティ係数。環境の衝突センサは対象を区別
# しないため、全ての衝突に最も厳しい「車両」係数 0.60 を使う。
PENALTY = {'collision': 0.60, 'red_light': 0.70}


def evaluate(model, env, episodes):
    """Run deterministic episodes, return aggregate metrics dict.

    決定論的エピソードを実行し、集計指標の辞書を返す。
    """
    rewards, steps_l, speeds = [], [], []
    rcs, iss, dss, min_ttcs = [], [], [], []
    km_total = 0.0
    events = {'collision': 0, 'lane': 0, 'red_light': 0,
              'left_road': 0, 'done': 0, 'near_miss': 0}
    for ep in range(episodes):
        obs, info = env.reset()
        total, n, speed_sum = 0.0, 0, 0.0
        in_near_miss = False
        route_len = env.sensing.cumdist[-1] if env.sensing.route else 0.0
        infraction = 1.0
        min_ttc = float('inf')
        while True:
            action, _ = model.predict(obs, deterministic=True)
            obs, r, term, trunc, info = env.step(action)
            total += r
            n += 1
            m = info['measurements']
            speed_sum += m['speed_kmh']
            # One near-miss per *entry* into the FCW zone, not per step
            # spent in it, so a long tailgate counts once.
            # FCW 領域に入った回数で数える（滞在ステップ数ではない）。
            # 長い追従1回は1回として数える。
            risky = (0.0 < m['ttc'] < ALERT_RULES['fcw_ttc_s']
                     and m['speed_kmh'] > ALERT_RULES['fcw_min_kmh'])
            if risky and not in_near_miss:
                events['near_miss'] += 1
            in_near_miss = risky
            if 0.0 < m['ttc'] < min_ttc:
                min_ttc = m['ttc']
            ev = info['events']
            events['collision'] += bool(ev['collision'])
            events['lane'] += bool(ev['crossed_solid'] or ev['crossed_broken'])
            events['red_light'] += bool(ev['red_light'])
            events['left_road'] += bool(ev['left_road'])
            if ev['collision']:
                infraction *= PENALTY['collision']
            if ev['red_light']:
                infraction *= PENALTY['red_light']
            if term or trunc:
                events['done'] += bool(ev['route_done'])
                break
        done_m = max(0.0, route_len - m['route_dist_left'])
        rc = min(1.0, done_m / route_len) if route_len > 0 else 0.0
        if ev['route_done']:
            rc = 1.0
        rcs.append(rc)
        iss.append(infraction)
        dss.append(rc * infraction)
        km_total += done_m / 1000.0
        min_ttcs.append(min_ttc if min_ttc < float('inf') else -1.0)
        rewards.append(total)
        steps_l.append(n)
        speeds.append(speed_sum / max(1, n))
        print('[eval] ep%d: steps=%d reward=%.1f avg=%.1f km/h RC=%.2f '
              'IS=%.2f DS=%.2f'
              % (ep, n, total, speeds[-1], rc, infraction, rc * infraction))
    e = float(episodes)
    km = max(km_total, 1e-6)
    seen_ttc = [t for t in min_ttcs if t > 0]
    return {
        'mean_reward': sum(rewards) / e,
        'min_reward': min(rewards), 'max_reward': max(rewards),
        'mean_steps': sum(steps_l) / e,
        'collisions_per_ep': events['collision'] / e,
        'lane_cross_per_ep': events['lane'] / e,
        'red_lights_per_ep': events['red_light'] / e,
        'off_road_per_ep': events['left_road'] / e,
        'completion_rate': events['done'] / e,
        'avg_speed_kmh': sum(speeds) / e,
        'near_miss_per_ep': events['near_miss'] / e,
        'route_completion': sum(rcs) / e,
        'infraction_score': sum(iss) / e,
        'driving_score': sum(dss) / e,
        'total_km': km_total,
        'collisions_per_km': events['collision'] / km,
        'red_lights_per_km': events['red_light'] / km,
        'near_miss_per_km': events['near_miss'] / km,
        'mean_min_ttc': (sum(seen_ttc) / len(seen_ttc)) if seen_ttc else -1.0,
    }


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--model', required=True,
                   help='SB3 .zip checkpoint / SB3チェックポイント')
    p.add_argument('--episodes', type=int, default=3)
    p.add_argument('--host', default='127.0.0.1')
    p.add_argument('--port', type=int, default=2000)
    p.add_argument('--seed', type=int, default=42,
                   help='same seed across checkpoints -> comparable routes '
                        '/ 同一シードで経路を揃えて比較可能にする')
    p.add_argument('--out', default=paths.EVAL_CSV)
    p.add_argument('--arm', default='',
                   help='study arm the model was trained in (A/B) '
                        '/ モデルを学習した研究アーム')
    p.add_argument('--eval-traffic', default='',
                   help='label of the traffic it is evaluated against, e.g. '
                        '"default" or "human_x1" / 評価に使った交通のラベル')
    p.add_argument('--train-seed', type=int, default=-1,
                   help='training seed of the model / モデルの学習シード')
    args = p.parse_args()

    try:
        from stable_baselines3 import SAC, PPO
    except ImportError:
        print('stable-baselines3 is required / が必要です')
        return 1
    loader = SAC if 'sac' in os.path.basename(args.model).lower() else PPO
    model = loader.load(args.model)
    print('[eval] %s (trained %d steps), %d episode(s)'
          % (args.model, model.num_timesteps, args.episodes))

    env = CarlaHILEnv(host=args.host, port=args.port, seed=args.seed)
    try:
        stats = evaluate(model, env, args.episodes)
        map_name = env.map.name.rsplit('/', 1)[-1]
    finally:
        env.close()

    row = {'wall_time': int(time.time()), 'model': args.model,
           'train_timesteps': model.num_timesteps,
           'episodes': args.episodes,
           'arm': args.arm, 'seed': args.train_seed,
           'eval_traffic': args.eval_traffic,
           'map': map_name}
    row.update({k: round(v, 3) for k, v in stats.items()})
    os.makedirs(os.path.dirname(args.out) or '.', exist_ok=True)
    new_file = not os.path.exists(args.out)
    with open(args.out, 'a', newline='') as fh:
        w = csv.DictWriter(fh, fieldnames=CSV_FIELDS, extrasaction='ignore')
        if new_file:
            w.writeheader()
        w.writerow(row)
    print('[eval] DS %.2f | RC %.2f | IS %.2f | collisions/km %.2f | '
          'near-misses/km %.2f | appended -> %s'
          % (stats['driving_score'], stats['route_completion'],
             stats['infraction_score'], stats['collisions_per_km'],
             stats['near_miss_per_km'], args.out))
    return 0


if __name__ == '__main__':
    sys.exit(main())
