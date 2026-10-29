#!/usr/bin/env python
"""Mine human-error scenario clips + driver profiles from session logs.

Scans data/<YYYY-MM-DD>/<HH-MM-SS>/telemetry.csv, finds the moments where
an event counter increments (lane invasion, red light, hard brake,
collision) and slices the surrounding telemetry into JSON "clips"
(docs/RL_DESIGN.md §4). Also measures each session's driver statistics and
converts them into CARLA TrafficManager error parameters, so training
traffic can be made to misbehave like the real humans.

Usage / 使い方:
    python -m rl.extract_scenarios              # scan data/, write data/scenarios/
    python -m rl.extract_scenarios --pre 10 --post 6
    python -m rl.extract_scenarios --data-dir data --out-dir data/scenarios

Outputs / 出力:
    data/scenarios/clips/<day>_<time>_<event>_<n>.json   scenario clips
    data/scenarios/driver_profiles.json                  per-session stats +
    data/scenarios/map_profiles.json                     km-weighted pool per map
                                                         TrafficManager params

セッションログから「人間エラー」シナリオ集とドライバープロファイルを
抽出する。イベントカウンタ（車線逸脱・信号無視・急ブレーキ・衝突）の
増加点を検出し、前後のテレメトリをJSONクリップとして切り出す
（docs/RL_DESIGN.md §4）。同時に各セッションの運転統計を計測し、CARLA
TrafficManagerのエラーパラメータへ変換する。これにより学習用交通を
「実際の人間と同じように」誤らせることができる。
"""

import argparse
import csv
import json
import math
import os
from pathlib import Path

HIL_DIR = Path(__file__).resolve().parent.parent

# counter column -> event type / カウンタ列 -> イベント種別
EVENT_COLUMNS = {
    'lane_invasions': 'lane_invasion',
    'red_light_violations': 'red_light',
    'hard_brake_count': 'hard_brake',
    'collision_count': 'collision',
}


def _f(row, key, default=0.0):
    try:
        return float(row.get(key, default))
    except (TypeError, ValueError):
        return default


def load_session(csv_path):
    """Load one telemetry.csv into a list of dict rows (floats).

    telemetry.csv を辞書行のリストとして読み込む。
    """
    rows = []
    with open(csv_path, newline='') as fh:
        for row in csv.DictReader(fh):
            rows.append(row)
    return rows


def find_events(rows):
    """Yield (index, event_type) where a counter increments.

    カウンタが増加した (行インデックス, イベント種別) を列挙する。
    """
    prev = {c: 0.0 for c in EVENT_COLUMNS}
    for i, row in enumerate(rows):
        for col, etype in EVENT_COLUMNS.items():
            val = _f(row, col)
            if val > prev[col]:
                # A counter can jump by >1 in one row; emit once per row.
                # 1行で2以上増えることもあるが、行につき1回だけ出力する。
                yield i, etype
            prev[col] = val


def slice_clip(rows, idx, pre_s, post_s):
    """Rows within [t_event - pre_s, t_event + post_s] (sim_time based).

    イベント時刻を基準に前後の行を切り出す（sim_time 基準）。
    """
    t_ev = _f(rows[idx], 'sim_time')
    lo = [r for r in rows
          if t_ev - pre_s <= _f(r, 'sim_time') <= t_ev + post_s]
    return t_ev, lo


def driver_profile(rows, summary):
    """Session statistics -> TrafficManager error parameters.

    セッション統計をTrafficManagerのエラーパラメータへ変換する。
    Heuristics (documented so they can be tuned):
    - ignore_lights_percentage: red-light violations per junction pass is
      unknown, so use violations per driven km, capped at 100.
    - distance_to_leading_vehicle: the driver's 25th-percentile THW times
      a typical urban speed (30 km/h -> 8.33 m/s).
    - vehicle_lane_offset noise: reuse the measured SDLP.
    - vehicle_percentage_speed_difference: negative = faster than limit;
      derived from measured speed compliance.
    """
    km = max(0.01, float(summary.get('trip_km', 0.0)) or 0.01)
    thws = sorted(t for t in (_f(r, 'thw', -1.0) for r in rows) if t > 0)
    sdlps = [s for s in (_f(r, 'sdlp') for r in rows) if s > 0]

    def pct(vals, p):
        if not vals:
            return -1.0
        return vals[min(len(vals) - 1, int(p / 100.0 * len(vals)))]

    red = float(summary.get('red_light_violations', 0))
    compliance = float(summary.get('speed_compliance', 100.0))
    prof = {
        'map': summary.get('map', ''),
        'trip_km': km,
        'sdlp_m': (sum(sdlps) / len(sdlps)) if sdlps else 0.0,
        'thw_p25_s': pct(thws, 25), 'thw_p50_s': pct(thws, 50),
        'red_per_km': red / km,
        'hard_brakes_per_km': float(summary.get('hard_brakes', 0)) / km,
        'lane_invasions_per_km': float(summary.get('out_of_lane', 0)) / km,
        'speed_compliance_pct': compliance,
    }
    prof['traffic_manager'] = tm_params(prof)
    return prof


def tm_params(prof):
    """Profile statistics -> TrafficManager parameters (shared heuristics).

    プロファイル統計 → TrafficManager パラメータ（共通の経験則）。
    """
    thw25 = prof['thw_p25_s'] if prof['thw_p25_s'] > 0 else 1.5
    compliance = float(prof.get('speed_compliance_pct', 100.0))
    return {
        'ignore_lights_percentage': min(100.0, round(prof['red_per_km'] * 20.0, 1)),
        'distance_to_leading_vehicle': round(max(1.0, thw25 * 8.33), 1),
        'vehicle_lane_offset_std': round(max(0.0, prof['sdlp_m']), 3),
        'vehicle_percentage_speed_difference':
            round(-(100.0 - compliance) * 0.3, 1),
    }


MIN_POOL_KM = 0.2   # sessions shorter than this are smoke runs / これ未満は試走


def pool_profiles(profiles, min_km=MIN_POOL_KM):
    """Pool per-session profiles into ONE profile per map, weighted by km.

    Arm B of the study (docs/RESEARCH_PROTOCOL.md §2) needs one set of
    TrafficManager parameters per map, not one per session: many sessions
    are 10-second smoke runs whose SDLP is 0 and THW is undefined, and
    letting them vote equally with a 600 s drive would pull every error
    rate toward zero. Rates (per km) are summed over km; distributions
    (SDLP, THW) are km-weighted means; the TrafficManager mapping is then
    recomputed from the pooled numbers with the same heuristics as
    driver_profile(), so a pooled profile is exactly what a single long
    session with those statistics would have produced.

    セッション毎のプロファイルをマップ毎に1つへ、走行距離で重み付けして
    統合する。研究のアームB（docs/RESEARCH_PROTOCOL.md §2）に必要なのは
    セッション毎ではなくマップ毎の TrafficManager パラメータである。
    10秒程度の試走セッション（SDLP=0・THW未定義）を600秒の走行と同じ
    重みで数えると全てのエラー率が0へ引き寄せられる。率（/km）は距離で
    合算、分布（SDLP・THW）は距離加重平均とし、TrafficManager への変換は
    driver_profile() と同じ経験則で統合値から再計算する。
    """
    by_map = {}
    for key, prof in profiles.items():
        km = float(prof.get('trip_km', 0.0))
        if km < min_km:
            continue
        by_map.setdefault(prof.get('map', ''), []).append((key, km, prof))

    pooled = {}
    for map_name, items in by_map.items():
        km_total = sum(km for _, km, _ in items)
        if km_total <= 0:
            continue

        def wmean(field, valid=lambda v: True):
            num = den = 0.0
            for _, km, p in items:
                v = float(p.get(field, -1.0))
                if valid(v):
                    num += v * km
                    den += km
            return (num / den) if den > 0 else -1.0

        def rate(field):
            return sum(float(p.get(field, 0.0)) * km for _, km, p in items) / km_total

        pool = {
            'sessions': [k for k, _, _ in items],
            'n_sessions': len(items),
            'trip_km': round(km_total, 3),
            'sdlp_m': round(wmean('sdlp_m', lambda v: v > 0), 4),
            'thw_p25_s': round(wmean('thw_p25_s', lambda v: v > 0), 2),
            'thw_p50_s': round(wmean('thw_p50_s', lambda v: v > 0), 2),
            'red_per_km': round(rate('red_per_km'), 3),
            'hard_brakes_per_km': round(rate('hard_brakes_per_km'), 3),
            'lane_invasions_per_km': round(rate('lane_invasions_per_km'), 3),
            'speed_compliance_pct': round(wmean('speed_compliance_pct',
                                                lambda v: v >= 0), 2),
        }
        pool['traffic_manager'] = tm_params(pool)
        pooled[map_name] = pool
    return pooled


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--data-dir', default=str(HIL_DIR / 'data'))
    p.add_argument('--out-dir', default=str(HIL_DIR / 'data' / 'scenarios'))
    p.add_argument('--pre', type=float, default=8.0,
                   help='seconds before the event / イベント前の秒数')
    p.add_argument('--post', type=float, default=4.0,
                   help='seconds after the event / イベント後の秒数')
    args = p.parse_args()

    data_dir = Path(args.data_dir)
    clips_dir = Path(args.out_dir) / 'clips'
    clips_dir.mkdir(parents=True, exist_ok=True)

    sessions = sorted(data_dir.glob('*/*/telemetry.csv'))
    profiles, n_clips = {}, 0
    for csv_path in sessions:
        day = csv_path.parent.parent.name
        tm = csv_path.parent.name
        rows = load_session(csv_path)
        if len(rows) < 20:      # skip empty/aborted sessions / 空セッションは除外
            continue
        summary = {}
        sj = csv_path.parent / 'summary.json'
        if sj.exists():
            with open(sj) as fh:
                summary = json.load(fh)

        for idx, etype in find_events(rows):
            t_ev, clip_rows = slice_clip(rows, idx, args.pre, args.post)
            # Row index in the name -> deterministic, re-runs overwrite
            # instead of duplicating. 行番号を名前に含め、再実行時は
            # 複製ではなく上書きになるようにする。
            out = clips_dir / ('%s_%s_%s_row%d.json' % (day, tm, etype, idx))
            with open(out, 'w') as fh:
                json.dump({
                    'session': '%s/%s' % (day, tm),
                    'map': summary.get('map', ''),
                    'vehicle': summary.get('vehicle', ''),
                    'event': etype,
                    'sim_time': t_ev,
                    'pre_s': args.pre, 'post_s': args.post,
                    # Full rows: enough to replay controls or fit BC.
                    # 制御再生やBC学習に十分な全列を保存する。
                    'columns': list(rows[0].keys()),
                    'rows': [[row[c] for c in rows[0].keys()]
                             for row in clip_rows],
                }, fh)
            n_clips += 1

        profiles['%s/%s' % (day, tm)] = driver_profile(rows, summary)
        print('[extract] %s/%s: %d rows, %d event type(s) scanned'
              % (day, tm, len(rows), len(EVENT_COLUMNS)))

    prof_path = Path(args.out_dir) / 'driver_profiles.json'
    with open(prof_path, 'w') as fh:
        json.dump(profiles, fh, indent=1)
    print('[extract] wrote %d clip(s) -> %s' % (n_clips, clips_dir))
    print('[extract] wrote %d profile(s) -> %s' % (len(profiles), prof_path))
    pooled = pool_profiles(profiles)
    pool_path = Path(args.out_dir) / 'map_profiles.json'
    with open(pool_path, 'w') as fh:
        json.dump(pooled, fh, indent=1)
    for map_name, pool in sorted(pooled.items()):
        print('[extract] %s: %d session(s), %.2f km -> %s'
              % (map_name or '?', pool['n_sessions'], pool['trip_km'],
                 json.dumps(pool['traffic_manager'])))
    print('[extract] wrote %d map profile(s) -> %s' % (len(pooled), pool_path))
    if not sessions:
        print('[extract] no telemetry.csv found under %s' % data_dir)


if __name__ == '__main__':
    main()
