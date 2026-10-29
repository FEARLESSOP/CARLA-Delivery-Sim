#!/usr/bin/env python
"""Render the result graphs in the layout CARLA RL papers use.

Reads two data sources (no CARLA needed / CARLA不要):
  1. rl/results/tb/<arm>_s<seed>*/events*   tensorboard scalars from rl/train_sac.py
                                    (episode reward & length during training)
  2. rl/results/eval_history.csv    checkpoint evaluations from rl/run_arm.py
                                    (Leaderboard-style DS/RC/IS, collisions,
                                    near-misses, per-km infractions; tagged
                                    with arm, seed and evaluation traffic)

and writes three artefacts into rl/results/figures/:

  progress.png       learning curves. Row 1: training episode reward and
                        length per arm, mean +- std over seeds (shaded).
                        Row 2: Driving Score, Route Completion and Collision
                        rate of each checkpoint, evaluated in the same
                        traffic for every arm, vs. training steps.
  results.png           final-checkpoint comparison as grouped bars with
                        std error bars: DS, RC, IS, collision rate, and
                        infractions per km with the human sessions from
                        this rig drawn as a reference line.
  results_table.md      the same numbers as a mean +- std table, one row
                        per arm -- the form results are reported in.

Conventions follow the CARLA Leaderboard (RC, IS, DS = RC x IS; global
infractions as events per km) and the common practice in CARLA RL papers
of reporting mean +- std over 3-5 seeds. Rewards are NOT compared across
arms: arm B trains in harder traffic by construction.

Figure text is English only (the study's figures are shared outside the
lab); code comments and CLI help stay bilingual as elsewhere in the repo.

Usage / 使い方:
    python -m rl.plot_progress
    python -m rl.plot_progress --eval-traffic human_x1 --map Town02

CARLA RL 論文の体裁で結果グラフを描く。progress.png は学習曲線（1段目：
アーム毎の報酬・エピソード長、シード平均±標準偏差。2段目：全アーム
共通の交通で評価した各チェックポイントの Driving Score・Route
Completion・衝突率）。results.png は最終チェックポイントの比較棒グラフ
（標準偏差のエラーバー付き。km あたり違反数には本環境の人間セッションを
参照線として描く）。results_table.md は同じ数値の平均±標準偏差の表。
指標は CARLA Leaderboard の慣例（RC・IS・DS = RC×IS、km あたり違反数）。
報酬はアーム間で比較しない。図中の文字は英語のみ。
"""

import argparse
import csv
import glob
import json
import os
import sys
from collections import defaultdict

import numpy as np

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from rl.plot_style import (ARM_STYLE, C_INK, C_MUTED,  # noqa: E402
                           style_axis)
import matplotlib.pyplot as plt  # noqa: E402

from rl import config, paths  # noqa: E402

HIL_DIR = paths.HIL_DIR
DATA_DIR = os.path.join(HIL_DIR, 'data')

TB_TAGS = ('rollout/ep_rew_mean', 'rollout/ep_len_mean')

# Row-2 learning curves: (csv column, title, ylabel, ylim).
# 2段目の学習曲線。
CURVE_PANELS = [
    ('driving_score', 'Driving Score (RC x IS)', 'DS', (0, 1.05)),
    ('route_completion', 'Route Completion', 'RC', (0, 1.05)),
    ('collision_rate', 'Collision rate', 'episodes with a collision (%)',
     (0, 105)),
]
# results.png panels: (csv column, title, ylabel, human reference key).
# results.png の各パネル。
BAR_PANELS = [
    ('driving_score', 'Driving Score', 'DS (higher is better)', None),
    ('route_completion', 'Route Completion', 'RC (higher is better)', None),
    ('infraction_score', 'Infraction Score', 'IS (higher is better)', None),
    ('collision_rate', 'Collision rate', '% episodes (lower is better)', None),
    ('collisions_per_km', 'Collisions per km', 'events / km (lower is better)',
     'crashes_per_km'),
    ('near_miss_per_km', 'Near-misses per km (TTC < 1.5 s)',
     'events / km (lower is better)', 'near_miss_per_km'),
]
TABLE_COLS = [
    ('driving_score', 'DS ↑', '%.2f'),
    ('route_completion', 'RC ↑', '%.2f'),
    ('infraction_score', 'IS ↑', '%.2f'),
    ('collision_rate', 'Coll. rate (%) ↓', '%.0f'),
    ('collisions_per_km', 'Coll. / km ↓', '%.2f'),
    ('red_lights_per_km', 'Red lights / km ↓', '%.2f'),
    ('near_miss_per_km', 'Near-miss / km ↓', '%.2f'),
    ('mean_min_ttc', 'min TTC (s) ↑', '%.2f'),
    ('avg_speed_kmh', 'Speed (km/h)', '%.1f'),
]


def arm_of(run_name):
    """"B_s1" -> "B"; anything else -> "?" (plotted grey). run名→アーム。"""
    head = run_name.split('_', 1)[0]
    return head if head in ARM_STYLE else '?'


def read_tb_scalars(tb_dir):
    """Return {run_name: {tag: [(step, value), ...]}} from tensorboard logs.

    tensorboardログからスカラーを読み出す。
    """
    try:
        from tensorboard.backend.event_processing.event_accumulator import (
            EventAccumulator)
    except ImportError:
        return {}
    runs = {}
    for run_dir in sorted(glob.glob(os.path.join(tb_dir, '*'))):
        if not os.path.isdir(run_dir):
            continue
        acc = EventAccumulator(run_dir, size_guidance={'scalars': 0})
        try:
            acc.Reload()
        except Exception:
            continue
        data = {}
        for tag in TB_TAGS:
            if tag in acc.Tags().get('scalars', []):
                data[tag] = [(s.step, s.value) for s in acc.Scalars(tag)]
        if data:
            runs[os.path.basename(run_dir)] = data
    return runs


def read_eval_history(csv_path):
    """Rows of eval_history.csv with numeric fields parsed and derived ones
    added (collision_rate in %). / 数値化し派生列（衝突率%）を加えて読む。
    """
    if not os.path.exists(csv_path):
        return []
    rows = []
    with open(csv_path, newline='') as fh:
        for r in csv.DictReader(fh):
            row = {}
            for k, v in r.items():
                if k in ('model', 'arm', 'eval_traffic', 'map'):
                    row[k] = v or ''
                else:
                    try:
                        row[k] = float(v)
                    except (TypeError, ValueError):
                        row[k] = float('nan')
            row['collision_rate'] = 100.0 * min(
                1.0, row.get('collisions_per_ep', float('nan')))
            rows.append(row)
    return rows


def human_reference(map_name):
    """Per-km human rates on this map from data/ (same filter as the event
    map). / data/ からこのマップの人間の km あたり率（イベント図と同条件）。
    """
    try:
        from rl.plot_town_events import load_sessions, group_metrics
    except ImportError:
        return {}
    groups = load_sessions(map_name, {})
    sessions = groups.get('human')
    if not sessions:
        return {}
    gm = group_metrics(sessions)
    near = sum(len(s['events']['near_miss']) for s in sessions)
    gm['near_miss_per_km'] = near / gm['km']
    return gm


def aggregate(series_list, n_bins=60):
    """Mean/std of several (step, value) series on a shared step grid.

    Runs log at different steps, so resample each onto a common grid by
    linear interpolation before averaging. / run 毎にログ位置が違うため
    共通グリッドに線形補間してから平均する。
    """
    series_list = [s for s in series_list if len(s) > 1]
    if not series_list:
        return None
    hi = min(max(x for x, _ in s) for s in series_list)
    lo = max(min(x for x, _ in s) for s in series_list)
    if hi <= lo:
        return None
    grid = np.linspace(lo, hi, n_bins)
    ys = np.array([np.interp(grid, [x for x, _ in s], [y for _, y in s])
                   for s in series_list])
    return grid, ys.mean(0), ys.std(0), len(series_list)


def plot_training(ax, runs, tag, title, ylabel):
    by_arm = defaultdict(list)
    for name, data in runs.items():
        if tag in data:
            by_arm[arm_of(name)].append(data[tag])
    drew = False
    for arm, series in sorted(by_arm.items()):
        color, label = ARM_STYLE.get(arm, (C_MUTED, 'other runs'))
        agg = aggregate(series)
        if agg is None:          # single short run: draw it raw / 生で描く
            for s in series:
                xs, ys = zip(*s)
                ax.plot(xs, ys, color=color, linewidth=1.4, alpha=0.9,
                        label=label)
            drew = True
            continue
        grid, mean, std, n = agg
        ax.plot(grid, mean, color=color, linewidth=1.8,
                label='%s (n=%d)' % (label, n))
        if n > 1:
            ax.fill_between(grid, mean - std, mean + std, color=color,
                            alpha=0.18, linewidth=0)
        drew = True
    style_axis(ax, title, 'environment steps', ylabel)
    if drew:
        ax.legend(fontsize=7, frameon=False, labelcolor=C_INK)
    else:
        ax.text(0.5, 0.5, 'no training data yet', ha='center', va='center',
                transform=ax.transAxes, fontsize=9, color=C_MUTED)


def per_arm_step(rows, key):
    """{arm: {train_timesteps: [values over seeds]}}. アーム×ステップ毎。"""
    out = defaultdict(lambda: defaultdict(list))
    for r in rows:
        v = r.get(key)
        if v is None or np.isnan(v):
            continue
        out[r.get('arm') or '?'][r['train_timesteps']].append(v)
    return out


def plot_curve(ax, rows, key, title, ylabel, ylim):
    """One metric vs. training steps, one line per arm, mean +- std band.

    アーム毎に1本、シード平均±標準偏差の帯。
    """
    by_arm = per_arm_step(rows, key)
    drew = False
    for arm, pts in sorted(by_arm.items()):
        color, label = ARM_STYLE.get(arm, (C_MUTED, 'untagged'))
        xs = np.array(sorted(pts))
        mean = np.array([np.mean(pts[x]) for x in xs])
        std = np.array([np.std(pts[x]) for x in xs])
        n = max(len(v) for v in pts.values())
        ax.plot(xs, mean, color=color, linewidth=1.8, marker='o',
                markersize=4, label='%s (n=%d)' % (label, n))
        if n > 1:
            ax.fill_between(xs, mean - std, mean + std, color=color,
                            alpha=0.18, linewidth=0)
        drew = True
    style_axis(ax, title, 'environment steps', ylabel)
    ax.set_ylim(*ylim)
    if drew:
        ax.legend(fontsize=7, frameon=False, labelcolor=C_INK)
    else:
        ax.text(0.5, 0.5, 'no evaluation rows yet\n'
                '(python -m rl.run_arm --arm A / B)',
                ha='center', va='center', transform=ax.transAxes,
                fontsize=9, color=C_MUTED)


def final_rows(rows):
    """Last checkpoint of every (arm, seed). 各 (アーム, シード) の最終 ckpt。"""
    best = {}
    for r in rows:
        k = (r.get('arm') or '?', r.get('seed'))
        if k not in best or r['train_timesteps'] > best[k]['train_timesteps']:
            best[k] = r
    out = defaultdict(list)
    for (arm, _), r in best.items():
        out[arm].append(r)
    return out


def plot_results(finals, human, out, eval_traffic, map_name):
    arms = sorted(finals)
    fig, axes = plt.subplots(1, len(BAR_PANELS),
                             figsize=(2.3 * len(BAR_PANELS), 3.9), dpi=150)
    fig.subplots_adjust(left=0.05, right=0.99, top=0.78, bottom=0.2,
                        wspace=0.6)
    fig.suptitle('%s: final checkpoints evaluated in "%s" traffic '
                 '(mean +- std over seeds)' % (map_name, eval_traffic),
                 fontsize=11, color=C_INK, x=0.05, ha='left')
    handles = {}
    for ax, (key, title, ylabel, hkey) in zip(axes, BAR_PANELS):
        for i, arm in enumerate(arms):
            vals = [r[key] for r in finals[arm]
                    if not np.isnan(r.get(key, float('nan')))]
            if not vals:
                continue
            color, label = ARM_STYLE.get(arm, (C_MUTED, arm))
            b = ax.bar(i, np.mean(vals), yerr=np.std(vals) if len(vals) > 1 else 0,
                       color=color, width=0.65, capsize=3,
                       error_kw={'elinewidth': 0.8, 'ecolor': C_INK})
            handles[arm] = (b, '%s (n=%d)' % (label, len(vals)))
            ax.annotate('%.2f' % np.mean(vals),
                        (i, np.mean(vals)), ha='center', va='bottom',
                        fontsize=7, color=C_INK, xytext=(0, 3),
                        textcoords='offset points')
        if hkey and human.get(hkey) is not None:
            ax.axhline(human[hkey], color=C_MUTED, linestyle='--',
                       linewidth=1.0)
            ax.annotate('human %.2f' % human[hkey], (len(arms) - 0.5,
                                                     human[hkey]),
                        fontsize=7, color=C_MUTED, ha='right', va='bottom',
                        xytext=(0, 2), textcoords='offset points')
        ax.set_xticks(range(len(arms)))
        ax.set_xticklabels(['arm %s' % a for a in arms], fontsize=8)
        ax.set_xlim(-0.6, len(arms) - 0.4)
        ax.set_ylim(bottom=0)
        if key in ('driving_score', 'route_completion', 'infraction_score'):
            ax.set_ylim(0, 1.1)
        elif key == 'collision_rate':
            ax.set_ylim(0, 110)
        style_axis(ax, title, None, ylabel)
        ax.grid(False, axis='x')
    if handles:
        fig.legend([h for h, _ in handles.values()],
                   [l for _, l in handles.values()], loc='lower left',
                   bbox_to_anchor=(0.05, 0.0), ncol=len(handles), fontsize=8,
                   frameon=False, labelcolor=C_INK)
    if human:
        fig.text(0.99, 0.02, 'dashed: human drivers on this rig, %d sessions, '
                 '%.1f km' % (human['n'], human['km']), fontsize=7,
                 color=C_MUTED, ha='right')
    fig.savefig(out)
    print('[plot] wrote %s' % out)


def write_table(finals, human, out, eval_traffic, map_name):
    lines = ['# %s results: final checkpoints in "%s" traffic'
             % (map_name, eval_traffic), '',
             'Mean ± std over seeds. Arrows mark the better direction. '
             'DS = RC × IS (CARLA Leaderboard convention); per-km rates '
             'over the distance actually driven in evaluation.', '',
             '| Arm | seeds | steps | ' + ' | '.join(c[1] for c in TABLE_COLS)
             + ' |',
             '|---|---|---|' + '---|' * len(TABLE_COLS)]
    for arm in sorted(finals):
        rows = finals[arm]
        _, label = ARM_STYLE.get(arm, (None, arm))
        cells = []
        for key, _, fmt in TABLE_COLS:
            vals = [r[key] for r in rows
                    if not np.isnan(r.get(key, float('nan')))]
            if not vals:
                cells.append('–')
            elif len(vals) > 1:
                cells.append((fmt + ' ± ' + fmt) % (np.mean(vals), np.std(vals)))
            else:
                cells.append(fmt % vals[0])
        steps = int(np.mean([r['train_timesteps'] for r in rows]))
        lines.append('| %s | %d | %dk | %s |'
                     % (label, len(rows), steps // 1000, ' | '.join(cells)))
    if human:
        cells = []
        for key, _, fmt in TABLE_COLS:
            hk = {'collisions_per_km': 'crashes_per_km',
                  'red_lights_per_km': 'red_per_km',
                  'near_miss_per_km': 'near_miss_per_km',
                  'avg_speed_kmh': 'avg_kmh'}.get(key)
            cells.append((fmt % human[hk]) if hk and hk in human else '–')
        lines.append('| human drivers (this rig, %d sessions, %.1f km) | – | – | %s |'
                     % (human['n'], human['km'], ' | '.join(cells)))
    with open(out, 'w') as fh:
        fh.write('\n'.join(lines) + '\n')
    print('[plot] wrote %s' % out)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--tb-dir', default=paths.TB_DIR)
    p.add_argument('--eval-csv', default=paths.EVAL_CSV)
    p.add_argument('--eval-traffic', default=config.EVAL_TRAFFIC,
                   help='which evaluation traffic to plot (rows must match) '
                        '/ 描画する評価交通（一致する行のみ使う）')
    p.add_argument('--map', default=config.STUDY_MAP,
                   help='CARLA town (default from rl/config.py) / 対象の町')
    p.add_argument('--out-dir', default=paths.FIGURES_DIR)
    args = p.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)

    runs = {n: d for n, d in read_tb_scalars(args.tb_dir).items()
            if arm_of(n) != '?'}
    evals = [r for r in read_eval_history(args.eval_csv)
             if (r.get('eval_traffic') or args.eval_traffic) == args.eval_traffic
             and (r.get('map') or args.map) == args.map]
    if not runs and not evals:
        print('[plot] no data yet: run `python -m rl.run_arm --arm A` and '
              '`--arm B` first\n'
              '[plot] データがありません。先に rl.run_arm でアームA/Bを'
              '実行してください')
        return 1

    # --- progress.png -----------------------------------------------------
    fig = plt.figure(figsize=(12.5, 7.4), dpi=150)
    gs = fig.add_gridspec(2, 6, hspace=0.55, wspace=0.9,
                          left=0.06, right=0.97, top=0.88, bottom=0.09)
    fig.suptitle('%s: learning curves per training arm' % args.map,
                 fontsize=12, color=C_INK, x=0.06, ha='left')
    fig.text(0.06, 0.925,
             'Top: training statistics, mean +- std over seeds. '
             'Bottom: each checkpoint evaluated in the same "%s" traffic for '
             'every arm (marker = checkpoint).' % args.eval_traffic,
             fontsize=8, color=C_MUTED)
    plot_training(fig.add_subplot(gs[0, 0:3]), runs, 'rollout/ep_rew_mean',
                  'Training episode reward', 'mean reward')
    plot_training(fig.add_subplot(gs[0, 3:6]), runs, 'rollout/ep_len_mean',
                  'Training episode length', 'mean steps (longer = survived)')
    for i, (key, title, ylabel, ylim) in enumerate(CURVE_PANELS):
        plot_curve(fig.add_subplot(gs[1, 2 * i:2 * i + 2]), evals, key,
                   title, ylabel, ylim)
    out = os.path.join(args.out_dir, 'progress.png')
    fig.savefig(out)
    print('[plot] wrote %s  (tb runs: %d, eval rows on "%s": %d)'
          % (out, len(runs), args.eval_traffic, len(evals)))

    # --- results.png + results_table.md ------------------------------------
    if evals:
        finals = final_rows(evals)
        human = human_reference(args.map)
        plot_results(finals, human, os.path.join(args.out_dir, 'results.png'),
                     args.eval_traffic, args.map)
        write_table(finals, human,
                    os.path.join(args.out_dir, 'results_table.md'),
                    args.eval_traffic, args.map)
    return 0


if __name__ == '__main__':
    sys.exit(main())
