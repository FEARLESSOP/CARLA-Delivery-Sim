#!/usr/bin/env python
"""How does the agent improve with training? Five figures + a delta table.

All of it comes from rl/results/eval_history.csv (rows written by
rl/run_arm.py: every checkpoint of every arm/seed evaluated in the same
traffic) plus the human sessions under data/ as the reference. No CARLA
needed. Output goes to rl/results/figures/:

  improvement_curves.png      the whole metric battery vs. training steps,
                              one line per arm (mean +- std over seeds),
                              human drivers on this rig as a dashed line
  improvement_before_after.png first checkpoint vs. final checkpoint per
                              arm, with the % change written on the bar
  improvement_vs_human.png    agent / human ratio of the per-km rates vs.
                              training steps (1.0 = human parity, below =
                              safer than the humans who drove this rig)
  improvement_tradeoff.png    speed vs. collisions and near-misses per km,
                              one trajectory per arm through its
                              checkpoints -- separates "got safer" from
                              "just drove slower / stood still"
  improvement_outcomes.png    how episodes end at each checkpoint
                              (completed / collision / not finished)
  improvement_table.md        first -> final delta per metric and arm

Usage / 使い方:
    python -m rl.plot_improvement
    python -m rl.plot_improvement --eval-traffic human_x1 --map Town02

Figure text is English only; comments and CLI help are EN + JA.

学習でエージェントがどう改善したかを示す図。全て eval_history.csv
（run_arm.py が全チェックポイントを同一交通で評価した行）と data/ の
人間セッションから描く。curves は指標一式の学習ステップ推移、
before_after は最初と最後のチェックポイントの比較（変化率付き）、
vs_human は人間との比（1.0 で同等、下回れば人間より安全）、
tradeoff は速度と衝突・ニアミス率の軌跡（「安全になった」と「遅く
なっただけ／止まっているだけ」を区別する）、outcomes はエピソードの
終わり方の内訳。table.md は最初→最後の差分表。図中の文字は英語のみ。
"""

import argparse
import os
import sys
from collections import defaultdict

import numpy as np

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from rl.plot_style import (ARM_STYLE, C_DANGER, C_INK, C_MUTED, C_ROAD,  # noqa: E402
                           C_TRAIN, style_axis)
import matplotlib.pyplot as plt  # noqa: E402

from rl import config, paths  # noqa: E402
from rl.plot_progress import (arm_of, final_rows, human_reference,  # noqa: E402
                              read_eval_history)

# (csv key, title, ylabel, human key or None, lower-is-better)
# 指標一式。human key は data/ 側の対応する km あたり率。
BATTERY = [
    ('driving_score', 'Driving Score (RC x IS)', 'DS', None, False),
    ('route_completion', 'Route Completion', 'RC', None, False),
    ('infraction_score', 'Infraction Score', 'IS', None, False),
    ('collisions_per_km', 'Collisions per km', 'events / km',
     'crashes_per_km', True),
    ('near_miss_per_km', 'Near-misses per km (TTC < 1.5 s)', 'events / km',
     'near_miss_per_km', True),
    ('red_lights_per_km', 'Red lights per km', 'events / km', 'red_per_km',
     True),
    ('lane_cross_per_km', 'Lane crossings per km', 'events / km',
     'lane_per_km', True),
    ('avg_speed_kmh', 'Average speed', 'km/h', 'avg_kmh', False),
    ('mean_min_ttc', 'Minimum TTC (episode mean)', 'seconds', None, False),
]
RATIO_KEYS = [k for k in BATTERY if k[3] and k[4]]     # per-km rates only
BEFORE_AFTER = [k for k in BATTERY
                if k[0] in ('driving_score', 'route_completion',
                            'collisions_per_km', 'near_miss_per_km',
                            'red_lights_per_km', 'avg_speed_kmh')]

# Episode outcomes: (label, colour). "not finished" = timeout or off-road.
# エピソードの終わり方。「not finished」はタイムアウトかコース外。
OUTCOMES = [('completed', C_TRAIN), ('collision', C_DANGER),
            ('not finished (timeout / off-road)', C_ROAD)]


def add_derived(rows):
    """Per-km lane crossings and the outcome fractions. / 派生列。"""
    for r in rows:
        km = r.get('total_km', float('nan'))
        eps = r.get('episodes', float('nan'))
        if km and km > 0 and eps and eps > 0:
            r['lane_cross_per_km'] = r.get('lane_cross_per_ep', 0.0) * eps / km
        else:
            r['lane_cross_per_km'] = float('nan')
        done = min(1.0, max(0.0, r.get('completion_rate', 0.0)))
        coll = min(1.0 - done, max(0.0, r.get('collisions_per_ep', 0.0)))
        r['frac_completed'] = done
        r['frac_collision'] = coll
        r['frac_other'] = max(0.0, 1.0 - done - coll)
    return rows


def by_arm_step(rows, key):
    """{arm: (steps[], mean[], std[], n)} for one metric. アーム毎の推移。"""
    acc = defaultdict(lambda: defaultdict(list))
    for r in rows:
        v = r.get(key)
        if v is None or np.isnan(v):
            continue
        acc[r.get('arm') or '?'][r['train_timesteps']].append(v)
    out = {}
    for arm, pts in acc.items():
        xs = np.array(sorted(pts))
        out[arm] = (xs, np.array([np.mean(pts[x]) for x in xs]),
                    np.array([np.std(pts[x]) for x in xs]),
                    max(len(v) for v in pts.values()))
    return out


def first_rows(rows):
    """Earliest checkpoint of every (arm, seed). 各 (アーム, シード) の最初。"""
    first = {}
    for r in rows:
        k = (r.get('arm') or '?', r.get('seed'))
        if k not in first or r['train_timesteps'] < first[k]['train_timesteps']:
            first[k] = r
    out = defaultdict(list)
    for (arm, _), r in first.items():
        out[arm].append(r)
    return out


def human_line(ax, human, hkey, x_right):
    if not hkey or human.get(hkey) is None:
        return
    ax.axhline(human[hkey], color=C_MUTED, linestyle='--', linewidth=1.0)
    ax.annotate('human %.2f' % human[hkey], (x_right, human[hkey]),
                fontsize=7, color=C_MUTED, ha='right', va='bottom',
                xytext=(0, 2), textcoords='offset points')


def empty(ax, msg='no data yet (agent drove 0 km?)'):
    ax.text(0.5, 0.5, msg, ha='center', va='center',
            transform=ax.transAxes, fontsize=9, color=C_MUTED)


# ---------------------------------------------------------------------------
def plot_curves(rows, human, out, eval_traffic, map_name):
    fig, axes = plt.subplots(3, 3, figsize=(12.5, 9.6), dpi=150)
    fig.subplots_adjust(left=0.06, right=0.98, top=0.9, bottom=0.07,
                        hspace=0.55, wspace=0.35)
    fig.suptitle('%s: how the agent changes with training' % map_name,
                 fontsize=12, color=C_INK, x=0.06, ha='left')
    fig.text(0.06, 0.935, 'Every checkpoint evaluated in the same "%s" '
             'traffic; mean +- std over seeds. Dashed: human drivers on '
             'this rig. Arrow in the title = better direction.'
             % eval_traffic, fontsize=8, color=C_MUTED)
    handles = {}
    for ax, (key, title, ylabel, hkey, lower) in zip(axes.flat, BATTERY):
        series = by_arm_step(rows, key)
        x_right = 0
        for arm, (xs, mean, std, n) in sorted(series.items()):
            color, label = ARM_STYLE.get(arm, (C_MUTED, 'untagged'))
            (ln,) = ax.plot(xs, mean, color=color, linewidth=1.8, marker='o',
                            markersize=4, markeredgecolor='white',
                            markeredgewidth=1.0)
            if n > 1:
                ax.fill_between(xs, mean - std, mean + std, color=color,
                                alpha=0.18, linewidth=0)
            handles[arm] = (ln, '%s (n=%d)' % (label, n))
            x_right = max(x_right, xs[-1])
        style_axis(ax, '%s %s' % (title, '↓' if lower else '↑'),
                   'environment steps', ylabel)
        if series:
            human_line(ax, human, hkey, x_right)
            ax.set_ylim(bottom=0)
            if key in ('driving_score', 'route_completion',
                       'infraction_score'):
                ax.set_ylim(0, 1.05)
        else:
            empty(ax)
    if handles:
        fig.legend([h for h, _ in handles.values()],
                   [l for _, l in handles.values()], loc='lower left',
                   bbox_to_anchor=(0.06, 0.0), ncol=len(handles),
                   fontsize=8, frameon=False, labelcolor=C_INK)
    fig.savefig(out)
    print('[improve] wrote %s' % out)


def plot_before_after(firsts, finals, human, out, eval_traffic, map_name):
    arms = sorted(set(firsts) | set(finals))
    fig, axes = plt.subplots(1, len(BEFORE_AFTER),
                             figsize=(2.4 * len(BEFORE_AFTER), 4.0), dpi=150)
    fig.subplots_adjust(left=0.05, right=0.99, top=0.76, bottom=0.2,
                        wspace=0.65)
    fig.suptitle('%s: first checkpoint vs. final checkpoint, "%s" traffic'
                 % (map_name, eval_traffic), fontsize=11, color=C_INK,
                 x=0.05, ha='left')
    fig.text(0.05, 0.86, 'Pale bar = first checkpoint, solid bar = final; '
             'label = change. Mean +- std over seeds. Dashed: human drivers.',
             fontsize=8, color=C_MUTED)
    w = 0.36
    for ax, (key, title, ylabel, hkey, lower) in zip(axes, BEFORE_AFTER):
        for i, arm in enumerate(arms):
            color, _ = ARM_STYLE.get(arm, (C_MUTED, arm))
            for j, (rows, alpha) in enumerate(((firsts.get(arm, []), 0.4),
                                               (finals.get(arm, []), 1.0))):
                vals = [r[key] for r in rows
                        if not np.isnan(r.get(key, float('nan')))]
                if not vals:
                    continue
                ax.bar(i + (j - 0.5) * w, np.mean(vals),
                       yerr=np.std(vals) if len(vals) > 1 else 0,
                       width=w - 0.04, color=color, alpha=alpha, capsize=2,
                       error_kw={'elinewidth': 0.8, 'ecolor': C_INK})
            f = [r[key] for r in firsts.get(arm, [])
                 if not np.isnan(r.get(key, float('nan')))]
            l = [r[key] for r in finals.get(arm, [])
                 if not np.isnan(r.get(key, float('nan')))]
            if f and l:
                d = np.mean(l) - np.mean(f)
                txt = ('%+.0f%%' % (100 * d / abs(np.mean(f)))
                       if abs(np.mean(f)) > 1e-9 else '%+.2f' % d)
                good = (d < 0) if lower else (d > 0)
                ax.annotate(txt, (i + 0.5 * w, np.mean(l)), ha='center',
                            va='bottom', fontsize=7, xytext=(0, 3),
                            textcoords='offset points',
                            color=C_INK if good or d == 0 else C_DANGER)
        human_line(ax, human, hkey, len(arms) - 0.4)
        ax.set_xticks(range(len(arms)))
        ax.set_xticklabels(['arm %s' % a for a in arms], fontsize=8)
        ax.set_xlim(-0.7, len(arms) - 0.3)
        ax.set_ylim(bottom=0)
        if key in ('driving_score', 'route_completion'):
            ax.set_ylim(0, 1.1)
        style_axis(ax, '%s %s' % (title, '↓' if lower else '↑'), None,
                   ylabel)
        ax.grid(False, axis='x')
    fig.savefig(out)
    print('[improve] wrote %s' % out)


def plot_vs_human(rows, human, out, eval_traffic, map_name):
    panels = [k for k in RATIO_KEYS if human.get(k[3])]
    if not panels:
        print('[improve] no human reference for %s; skipping vs_human '
              '/ 人間の参照値が無いため vs_human は省略' % map_name)
        return
    fig, axes = plt.subplots(1, len(panels), figsize=(3.2 * len(panels), 3.9),
                             dpi=150, squeeze=False)
    axes = axes[0]
    fig.subplots_adjust(left=0.07, right=0.99, top=0.78, bottom=0.18,
                        wspace=0.4)
    fig.suptitle('%s: agent relative to the human drivers on this rig'
                 % map_name, fontsize=11, color=C_INK, x=0.07, ha='left')
    fig.text(0.07, 0.87, 'Agent rate / human rate per km, "%s" traffic. '
             '1.0 = human parity; below the line the agent errs less often '
             'than the %d human sessions (%.1f km).'
             % (eval_traffic, human['n'], human['km']), fontsize=8,
             color=C_MUTED)
    handles = {}
    for ax, (key, title, _, hkey, _) in zip(axes, panels):
        series = by_arm_step(rows, key)
        for arm, (xs, mean, std, n) in sorted(series.items()):
            color, label = ARM_STYLE.get(arm, (C_MUTED, 'untagged'))
            ratio = mean / human[hkey]
            (ln,) = ax.plot(xs, ratio, color=color, linewidth=1.8, marker='o',
                            markersize=4, markeredgecolor='white',
                            markeredgewidth=1.0)
            if n > 1:
                ax.fill_between(xs, (mean - std) / human[hkey],
                                (mean + std) / human[hkey], color=color,
                                alpha=0.18, linewidth=0)
            handles[arm] = (ln, '%s (n=%d)' % (label, n))
        ax.axhline(1.0, color=C_MUTED, linestyle='--', linewidth=1.0)
        ax.set_yscale('symlog', linthresh=1.0)
        ax.set_ylim(bottom=0)
        ax.set_yticks([0, 0.5, 1, 2, 5, 10, 20])
        ax.set_yticklabels(['0', '0.5', '1', '2', '5', '10', '20'])
        ax.yaxis.set_minor_locator(plt.NullLocator())
        style_axis(ax, '%s ↓' % title, 'environment steps',
                   'agent / human (symlog)')
        if not series:
            empty(ax)
    if handles:
        fig.legend([h for h, _ in handles.values()],
                   [l for _, l in handles.values()], loc='lower left',
                   bbox_to_anchor=(0.07, 0.0), ncol=len(handles), fontsize=8,
                   frameon=False, labelcolor=C_INK)
    fig.savefig(out)
    print('[improve] wrote %s' % out)


def plot_tradeoff(rows, human, out, eval_traffic, map_name):
    panels = [('collisions_per_km', 'Collisions per km', 'crashes_per_km'),
              ('near_miss_per_km', 'Near-misses per km', 'near_miss_per_km')]
    fig, axes = plt.subplots(1, 2, figsize=(9.5, 4.2), dpi=150)
    fig.subplots_adjust(left=0.08, right=0.98, top=0.78, bottom=0.17,
                        wspace=0.3)
    fig.suptitle('%s: is it safer, or just slower?' % map_name, fontsize=11,
                 color=C_INK, x=0.08, ha='left')
    fig.text(0.08, 0.85, 'Each arm moves through its checkpoints (mean over '
             'seeds, "%s" traffic); the arrow points at the final one.\n'
             'Down and to the right = safer at speed. Diamond = human drivers.'
             % eval_traffic, fontsize=8, color=C_MUTED, linespacing=1.5)
    handles = {}
    for ax, (key, title, hkey) in zip(axes, panels):
        sx = by_arm_step(rows, 'avg_speed_kmh')
        sy = by_arm_step(rows, key)
        for arm in sorted(set(sx) & set(sy)):
            xs_s, mx, _, n = sx[arm]
            xs_y, my, _, _ = sy[arm]
            steps = sorted(set(xs_s) & set(xs_y))
            if not steps:
                continue
            x = np.array([mx[list(xs_s).index(s)] for s in steps])
            y = np.array([my[list(xs_y).index(s)] for s in steps])
            color, label = ARM_STYLE.get(arm, (C_MUTED, 'untagged'))
            (ln,) = ax.plot(x, y, color=color, linewidth=1.4, alpha=0.8,
                            marker='o', markersize=5, markeredgecolor='white',
                            markeredgewidth=1.0)
            if len(x) > 1:
                ax.annotate('', xy=(x[-1], y[-1]), xytext=(x[-2], y[-2]),
                            arrowprops=dict(arrowstyle='-|>', color=color,
                                            lw=1.4, shrinkA=0, shrinkB=4))
            ax.annotate('%dk' % (steps[0] // 1000), (x[0], y[0]), fontsize=7,
                        color=C_MUTED, xytext=(4, 4), textcoords='offset points')
            ax.annotate('%dk' % (steps[-1] // 1000), (x[-1], y[-1]),
                        fontsize=7, color=C_INK, xytext=(4, 4),
                        textcoords='offset points')
            handles[arm] = (ln, '%s (n=%d)' % (label, n))
        if human.get(hkey) is not None and human.get('avg_kmh'):
            ax.plot(human['avg_kmh'], human[hkey], marker='D', markersize=7,
                    color=C_MUTED, markeredgecolor='white', linestyle='none')
            ax.annotate('human', (human['avg_kmh'], human[hkey]), fontsize=7,
                        color=C_MUTED, xytext=(6, -3),
                        textcoords='offset points')
        style_axis(ax, title, 'average speed (km/h) →', 'events / km ↓')
        ax.margins(y=0.12)
        xmax = max([human.get('avg_kmh') or 0.0] +
                   [mx.max() for (_, mx, _, _) in sx.values()] + [1.0])
        ax.set_xlim(0, xmax * 1.25)     # room for the end labels / 端のラベル用
        ax.set_ylim(bottom=0)
        if not handles:
            empty(ax)
    if handles:
        fig.legend([h for h, _ in handles.values()],
                   [l for _, l in handles.values()], loc='lower left',
                   bbox_to_anchor=(0.08, 0.0), ncol=len(handles), fontsize=8,
                   frameon=False, labelcolor=C_INK)
    fig.savefig(out)
    print('[improve] wrote %s' % out)


def plot_outcomes(rows, out, eval_traffic, map_name):
    arms = sorted({r.get('arm') or '?' for r in rows}) or ['?']
    fig, axes = plt.subplots(1, len(arms), figsize=(5.4 * len(arms), 3.9),
                             dpi=150, squeeze=False, sharey=True)
    axes = axes[0]
    left = 0.12 if len(arms) == 1 else 0.07
    fig.subplots_adjust(left=left, right=0.99, top=0.76, bottom=0.2,
                        wspace=0.12)
    fig.suptitle('%s: how evaluation episodes end, per checkpoint'
                 % map_name, fontsize=11, color=C_INK, x=left, ha='left')
    fig.text(left, 0.845, 'Share of episodes in "%s" traffic (mean over '
             'seeds).\nA learning agent turns the red and grey into blue.'
             % eval_traffic, fontsize=8, color=C_MUTED, linespacing=1.5)
    keys = ('frac_completed', 'frac_collision', 'frac_other')
    handles = None
    for ax, arm in zip(axes, arms):
        series = {k: by_arm_step([r for r in rows if (r.get('arm') or '?') == arm], k)
                  .get(arm) for k in keys}
        if any(v is None for v in series.values()):
            empty(ax)
            style_axis(ax, 'arm %s' % arm, 'checkpoint (steps)', 'share of episodes')
            continue
        xs = series['frac_completed'][0]
        bottom = np.zeros(len(xs))
        bars = []
        for k, (label, color) in zip(keys, OUTCOMES):
            vals = series[k][1]
            b = ax.bar(range(len(xs)), vals, bottom=bottom, color=color,
                       width=0.7, edgecolor='white', linewidth=1.5)
            bars.append(b)
            bottom = bottom + vals
        handles = bars
        _, label = ARM_STYLE.get(arm, (None, arm))
        ax.set_xticks(range(len(xs)))
        ax.set_xticklabels(['%dk' % (x // 1000) for x in xs], fontsize=8)
        ax.set_ylim(0, 1.0)
        style_axis(ax, label, 'checkpoint (training steps)',
                   'share of episodes' if ax is axes[0] else None)
        ax.grid(False, axis='x')
    if handles:
        fig.legend(handles, [l for l, _ in OUTCOMES], loc='lower left',
                   bbox_to_anchor=(left, 0.0), ncol=3, fontsize=8,
                   frameon=False, labelcolor=C_INK)
    fig.savefig(out)
    print('[improve] wrote %s' % out)


def write_table(firsts, finals, human, out, eval_traffic, map_name):
    lines = ['# %s: first -> final checkpoint, "%s" traffic'
             % (map_name, eval_traffic), '',
             'Mean over seeds. Δ = final − first; arrows mark the better '
             'direction. Human = drivers on this rig%s.'
             % (' (%d sessions, %.1f km)' % (human['n'], human['km'])
                if human else ''), '',
             '| Arm | seeds | Metric | first | final | Δ | Δ % | human |',
             '|---|---|---|---|---|---|---|---|']
    for arm in sorted(set(firsts) | set(finals)):
        _, label = ARM_STYLE.get(arm, (None, arm))
        n = len(finals.get(arm, []))
        for key, title, _, hkey, lower in BATTERY:
            f = [r[key] for r in firsts.get(arm, [])
                 if not np.isnan(r.get(key, float('nan')))]
            l = [r[key] for r in finals.get(arm, [])
                 if not np.isnan(r.get(key, float('nan')))]
            if not f or not l:
                continue
            mf, ml = np.mean(f), np.mean(l)
            pct = ('%+.0f%%' % (100 * (ml - mf) / abs(mf))
                   if abs(mf) > 1e-9 else '–')
            h = ('%.2f' % human[hkey]) if hkey and human.get(hkey) is not None \
                else '–'
            lines.append('| %s | %d | %s %s | %.2f | %.2f | %+.2f | %s | %s |'
                         % (label, n, title, '↓' if lower else '↑', mf, ml,
                            ml - mf, pct, h))
    with open(out, 'w') as fh:
        fh.write('\n'.join(lines) + '\n')
    print('[improve] wrote %s' % out)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--eval-csv', default=paths.EVAL_CSV)
    p.add_argument('--eval-traffic', default=config.EVAL_TRAFFIC,
                   help='which evaluation traffic to plot / 描画する評価交通')
    p.add_argument('--map', default=config.STUDY_MAP,
                   help='CARLA town (default from rl/config.py) / 対象の町')
    p.add_argument('--out-dir', default=paths.FIGURES_DIR)
    args = p.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)

    rows = [r for r in read_eval_history(args.eval_csv)
            if (r.get('eval_traffic') or args.eval_traffic) == args.eval_traffic
            and (r.get('map') or args.map) == args.map]
    if not rows:
        print('[improve] no evaluation rows for %s / "%s" yet: run '
              '`python -m rl.run_arm` first\n[improve] 評価行がありません。'
              '先に rl.run_arm を実行してください' % (args.map, args.eval_traffic))
        return 1
    add_derived(rows)
    human = human_reference(args.map)
    firsts, finals = first_rows(rows), final_rows(rows)
    o = lambda name: os.path.join(args.out_dir, name)  # noqa: E731
    plot_curves(rows, human, o('improvement_curves.png'), args.eval_traffic,
                args.map)
    plot_before_after(firsts, finals, human, o('improvement_before_after.png'),
                      args.eval_traffic, args.map)
    plot_vs_human(rows, human, o('improvement_vs_human.png'),
                  args.eval_traffic, args.map)
    plot_tradeoff(rows, human, o('improvement_tradeoff.png'),
                  args.eval_traffic, args.map)
    plot_outcomes(rows, o('improvement_outcomes.png'), args.eval_traffic,
                  args.map)
    write_table(firsts, finals, human, o('improvement_table.md'),
                args.eval_traffic, args.map)
    return 0


if __name__ == '__main__':
    sys.exit(main())
