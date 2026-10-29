#!/usr/bin/env python
"""Where on the map do things go wrong? Event map + metric battery per group.

Two figures from the session logs under data/ (no CARLA needed once the
road geometry has been cached):

  rl/results/figures/<map>_events.png   top-down map of the town with every logged
                        collision, lane invasion, red-light run and
                        near-miss (TTC < 1.5 s at > 20 km/h), one panel
                        per driver group -- human sessions vs. agent
                        sessions logged through the same dashboard.
  rl/results/figures/<map>_metrics.png  the dashboard metric battery per group:
                        crashes/km, out-of-lane/km, red lights/km,
                        hard brakes/km, SDLP, speed compliance, avg speed.

"Improvement on this map" is then literal: the same junctions, before and
after. Human sessions are every data/<day>/<time>/ whose summary.json is
on the map; agent sessions are the same folders when they were produced
by agent_drive.py --rolename hero (docs/RL_DESIGN.md §6 step 4-3) and are
listed in data/agent_sessions.json as {"<day>/<time>": "<label>"}.

Road geometry: fetched once from a running CARLA (world.get_map()) and
cached in data/scenarios/<map>_roads.json; with no server and no cache the
driven trace itself is used as the background.

Figure text is English only (the study's figures are shared outside the
lab); code comments and CLI help stay bilingual as elsewhere in the repo.
図中の文字は英語のみ（研究の図は研究室外にも出す）。コメントと CLI の
ヘルプは他と同様に EN+JA。

Usage / 使い方:
    python -m rl.plot_town_events                 # Town02
    python -m rl.plot_town_events --map Town05 --host 127.0.0.1

マップ上のどこで問題が起きるかを描く。data/ のセッションログから、
衝突・車線逸脱・信号無視・ニアミス（20 km/h 超で TTC < 1.5 s）を町の
俯瞰図に重ね、人間セッションとエージェントセッション（同じダッシュ
ボード経由で記録し data/agent_sessions.json に列挙したもの）をグループ
毎に並べる。さらにダッシュボード指標（衝突/km・逸脱/km・信号無視/km・
急ブレーキ/km・SDLP・速度遵守率・平均速度）をグループ毎に棒グラフ化する。
道路形状は稼働中の CARLA から一度取得して data/scenarios/<map>_roads.json
にキャッシュする。サーバもキャッシュも無い場合は走行軌跡を背景にする。
"""

import argparse
import csv
import glob
import json
import os
import sys

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from rl.plot_style import (C_INK, C_MUTED, C_ROAD, C_DANGER, C_PURPLE,  # noqa: E402
                           C_WARN, C_TRAIN, style_axis)
from rl.rules import ALERT_RULES  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402

from rl import config, paths  # noqa: E402

HIL_DIR = paths.HIL_DIR
DATA_DIR = os.path.join(HIL_DIR, 'data')

EVENT_STYLE = {
    # key: (colour, marker, size, label)
    'collision': (C_DANGER, 'X', 70, 'collision'),
    'red_light': (C_PURPLE, 'D', 45, 'red light'),
    'lane': (C_WARN, 's', 22, 'lane invasion'),
    'near_miss': (C_TRAIN, 'o', 14, 'near-miss (TTC < %.1f s)'
                  % ALERT_RULES['fcw_ttc_s']),
}
METRICS = [
    # (key, label, lower-is-better)
    ('crashes_per_km', 'crashes / km', True),
    ('lane_per_km', 'out-of-lane / km', True),
    ('red_per_km', 'red lights / km', True),
    ('hard_brakes_per_km', 'hard brakes / km', True),
    ('sdlp', 'SDLP (m)', True),
    ('speed_compliance', 'speed compliance (%)', False),
    ('avg_kmh', 'avg speed (km/h)', False),
]
MIN_KM = 0.2   # shorter sessions are smoke runs / これ未満は試走


def _f(row, key, default=0.0):
    try:
        return float(row.get(key, default))
    except (TypeError, ValueError):
        return default


def load_sessions(map_name, agent_labels):
    """-> {group_label: [session dict, ...]} for sessions on `map_name`.

    A session dict carries its summary plus the event points found by
    walking telemetry.csv row by row (counter increments -> event at that
    row's x, y). / summary.json と、telemetry.csv を1行ずつ走査して
    カウンタ増分から得たイベント座標を持つ辞書をグループ毎に返す。
    """
    groups = {}
    for sj in sorted(glob.glob(os.path.join(DATA_DIR, '*', '*',
                                            'summary.json'))):
        with open(sj) as fh:
            summary = json.load(fh)
        if summary.get('map') != map_name:
            continue
        if _f(summary, 'trip_km') < MIN_KM:
            continue
        folder = os.path.dirname(sj)
        key = '%s/%s' % (os.path.basename(os.path.dirname(folder)),
                         os.path.basename(folder))
        label = agent_labels.get(key, 'human')
        sess = {'key': key, 'summary': summary, 'trace': [],
                'events': {k: [] for k in EVENT_STYLE}}
        prev = None
        with open(os.path.join(folder, 'telemetry.csv'), newline='') as fh:
            for row in csv.DictReader(fh):
                x, y = _f(row, 'x', None), _f(row, 'y', None)
                if x is None or y is None:
                    continue
                sess['trace'].append((x, y))
                if prev is not None:
                    for ev, col in (('collision', 'collision_count'),
                                    ('lane', 'lane_invasions'),
                                    ('red_light', 'red_light_violations')):
                        if _f(row, col) > _f(prev, col):
                            sess['events'][ev].append((x, y))
                ttc = _f(row, 'ttc', -1.0)
                risky = (0.0 < ttc < ALERT_RULES['fcw_ttc_s']
                         and _f(row, 'speed_kmh') > ALERT_RULES['fcw_min_kmh'])
                was_risky = prev is not None and prev.get('_risky')
                if risky and not was_risky:     # entry, not dwell / 突入回数
                    sess['events']['near_miss'].append((x, y))
                row['_risky'] = risky
                prev = row
        groups.setdefault(label, []).append(sess)
    return groups


def load_roads(map_name, host, port):
    """Road centre-line polylines for the background, cached as JSON.

    背景用の道路中心線。JSON にキャッシュする。
    """
    cache = os.path.join(DATA_DIR, 'scenarios', '%s_roads.json' % map_name)
    if os.path.exists(cache):
        with open(cache) as fh:
            return json.load(fh)
    if not host:
        return None
    try:
        from rl import carla_env  # noqa: F401  (sets up the carla import path)
        import carla
        client = carla.Client(host, port)
        client.set_timeout(10.0)
        world = client.get_world()
        cmap = world.get_map()
        if cmap.name.rsplit('/', 1)[-1] != map_name:
            print('[map] server has %s loaded, not %s; no road background '
                  '/ サーバのマップが違うため道路背景なし' % (cmap.name, map_name))
            return None
        # One polyline per lane: follow each waypoint forward until the
        # lane ends or loops. / 車線毎に1本の折れ線。
        seen, lines = set(), []
        for wp in cmap.generate_waypoints(4.0):
            k = (wp.road_id, wp.section_id, wp.lane_id)
            if k in seen:
                continue
            seen.add(k)
            line, cur, n = [], wp, 0
            while cur is not None and n < 2000:
                line.append((cur.transform.location.x,
                             cur.transform.location.y))
                nxt = cur.next(4.0)
                cur = None
                for c in nxt:
                    ck = (c.road_id, c.section_id, c.lane_id)
                    if ck == k:
                        cur = c
                        break
                n += 1
            if len(line) > 1:
                lines.append(line)
        os.makedirs(os.path.dirname(cache), exist_ok=True)
        with open(cache, 'w') as fh:
            json.dump(lines, fh)
        print('[map] cached %d road polylines -> %s' % (len(lines), cache))
        return lines
    except Exception as exc:   # server absent / サーバ無し
        print('[map] no road geometry (%s); using the driven trace' % exc)
        return None


def group_metrics(sessions):
    """km-weighted metric battery for one group. 距離加重の指標集計。"""
    km = sum(_f(s['summary'], 'trip_km') for s in sessions) or 1e-9

    def per_km(key):
        return sum(_f(s['summary'], key) for s in sessions) / km

    def wmean(key):
        return sum(_f(s['summary'], key) * _f(s['summary'], 'trip_km')
                   for s in sessions) / km
    return {
        'n': len(sessions), 'km': km,
        'crashes_per_km': per_km('crashes'),
        'lane_per_km': per_km('out_of_lane'),
        'red_per_km': per_km('red_light_violations'),
        'hard_brakes_per_km': per_km('hard_brakes'),
        'sdlp': wmean('sdlp'),
        'speed_compliance': wmean('speed_compliance'),
        'avg_kmh': wmean('avg_kmh'),
    }


def plot_event_map(groups, roads, map_name, out):
    names = list(groups)
    fig, axes = plt.subplots(1, len(names), figsize=(6.2 * len(names), 6.8),
                             dpi=150, squeeze=False)
    fig.subplots_adjust(left=0.05, right=0.98, top=0.88, bottom=0.08,
                        wspace=0.12)
    fig.suptitle('%s: where driving errors happen'
                 % map_name, fontsize=12, color=C_INK, x=0.05, ha='left')
    for ax, name in zip(axes[0], names):
        sessions = groups[name]
        if roads:
            for line in roads:
                xs, ys = zip(*line)
                ax.plot(xs, ys, color=C_ROAD, linewidth=2.2, zorder=1,
                        solid_capstyle='round')
        for s in sessions:                       # driven trace / 走行軌跡
            if s['trace']:
                xs, ys = zip(*s['trace'])
                ax.plot(xs, ys, color=C_MUTED, linewidth=0.4, alpha=0.35,
                        zorder=2)
        counts = {}
        for ev, (color, marker, size, label) in EVENT_STYLE.items():
            pts = [p for s in sessions for p in s['events'][ev]]
            counts[ev] = len(pts)
            if pts:
                xs, ys = zip(*pts)
                ax.scatter(xs, ys, s=size, c=color, marker=marker,
                           edgecolors='white', linewidths=0.4, zorder=3,
                           label='%s (%d)' % (label, len(pts)))
        km = sum(_f(s['summary'], 'trip_km') for s in sessions)
        ax.set_title('%s: %d session(s), %.1f km'
                     % (name, len(sessions), km),
                     fontsize=10, color=C_INK, loc='left')
        ax.set_aspect('equal')
        ax.invert_yaxis()            # CARLA top view: +y is down / CARLA俯瞰
        ax.set_xlabel('x (m)', fontsize=8, color=C_MUTED)
        ax.set_ylabel('y (m)', fontsize=8, color=C_MUTED)
        style_axis(ax)
        if any(counts.values()):
            ax.legend(fontsize=7, frameon=True, framealpha=0.85,
                      edgecolor='none', loc='upper center', ncol=2,
                      labelcolor=C_INK)
    fig.savefig(out)
    print('[map] wrote %s' % out)


def plot_metrics(groups, map_name, out):
    names = list(groups)
    stats = {n: group_metrics(groups[n]) for n in names}
    palette = ['#777777', '#3987e5', '#199e70', '#c98500', '#e66767']
    fig, axes = plt.subplots(1, len(METRICS), figsize=(2.1 * len(METRICS), 3.6),
                             dpi=150)
    fig.subplots_adjust(left=0.04, right=0.99, top=0.78, bottom=0.14,
                        wspace=0.55)
    fig.suptitle('%s: dashboard metric battery per group (km-weighted)' % map_name,
                 fontsize=11, color=C_INK, x=0.04, ha='left')
    for ax, (key, label, lower_better) in zip(axes, METRICS):
        vals = [stats[n][key] for n in names]
        bars = ax.bar(range(len(names)), vals,
                      color=palette[:len(names)], width=0.6)
        for b, v in zip(bars, vals):
            ax.annotate('%.2f' % v if v < 10 else '%.0f' % v,
                        (b.get_x() + b.get_width() / 2, b.get_height()),
                        ha='center', va='bottom', fontsize=7, color=C_INK,
                        xytext=(0, 2), textcoords='offset points')
        ax.set_title(label + ('\n(↓ better)' if lower_better else '\n(↑ better)'),
                     fontsize=8, color=C_INK, loc='left', pad=6)
        ax.set_xticks(range(len(names)))
        ax.set_xticklabels(names, fontsize=7,
                           rotation=20, ha='right')
        ax.set_ylim(0, max(vals + [1e-9]) * 1.25)
        style_axis(ax)
    foot = '   '.join('%s: n=%d, %.1f km' % (n, stats[n]['n'], stats[n]['km'])
                      for n in names)
    fig.text(0.04, 0.02, foot, fontsize=7, color=C_MUTED)
    fig.savefig(out)
    print('[map] wrote %s' % out)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--map', default=config.STUDY_MAP,
                   help='CARLA town (default from rl/config.py) / 対象の町')
    p.add_argument('--host', default='127.0.0.1',
                   help='CARLA host for the road background ("" = skip) '
                        '/ 道路背景用の CARLA ホスト（"" で省略）')
    p.add_argument('--port', type=int, default=2000)
    p.add_argument('--agent-sessions',
                   default=os.path.join(DATA_DIR, 'agent_sessions.json'),
                   help='{"<day>/<time>": "<group label>"} for sessions '
                        'driven by an agent / エージェント走行セッションの一覧')
    p.add_argument('--out-dir', default=paths.FIGURES_DIR)
    args = p.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)

    agent_labels = {}
    if os.path.exists(args.agent_sessions):
        with open(args.agent_sessions) as fh:
            agent_labels = json.load(fh)
    groups = load_sessions(args.map, agent_labels)
    if not groups:
        print('[map] no sessions >= %.1f km on %s under %s'
              % (MIN_KM, args.map, DATA_DIR))
        return 1
    for name, sessions in groups.items():
        print('[map] %s: %d session(s)' % (name, len(sessions)))
    roads = load_roads(args.map, args.host, args.port)
    plot_event_map(groups, roads, args.map,
                   os.path.join(args.out_dir, '%s_events.png' % args.map))
    plot_metrics(groups, args.map,
                 os.path.join(args.out_dir, '%s_metrics.png' % args.map))
    return 0


if __name__ == '__main__':
    sys.exit(main())
