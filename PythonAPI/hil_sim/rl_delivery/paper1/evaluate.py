"""Evaluate Paper 1 recovery methods on the scenarios with paired seeds.

  ../../carla-venv/bin/python -m rl_delivery.paper1.evaluate --method B0
  ... --scenarios S1_normal --seeds 3          # a quick check / 動作確認

Every episode is a full operating period (env_fleet.EP_S = 3 h of sim time)
in the Paper 1 world. Everything but the recovery choice follows frozen rules
(fleet_rules.rule_pod for the pod, medium launch height, hover rule: back to
the nearest feasible charger below 20 %). The same seed list is used for
every method (paired comparison).

Outputs (rl_delivery/research/paper1/, docs/PAPER1_BASELINES.md section 5.5):
  results_episodes.csv   one row per (method, scenario, seed)
  decisions.jsonl        one JSON line per recovery decision (with the actual
                         ready time once the drone is ready again)
  summary.csv            mean and 95 % confidence interval per metric
  config.json            the settings of the run

Paper 1 の回収手法を対応のあるシードで全シナリオ評価する。回収先の選択以外は固定ルール。
"""
import argparse
import csv
import json
import math
import os
import time
from multiprocessing import Pool

import numpy as np

from rl_delivery import env_fleet as F
from rl_delivery.fleet_rules import rule_pod
from rl_delivery.paper1 import world as W1
from rl_delivery.paper1.methods import METHODS, LEARNED, ROLLOUT, make_method

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(os.path.dirname(HERE), "research", "paper1")
SEED0 = 1000                     # paired seeds: SEED0 .. SEED0 + n - 1 / 対応のあるシード

EP_COLS = ["method", "scenario", "seed", "sim_hours", "orders", "delivered", "delivered_per_h", "drone_per_h", "pod_per_h",
           "ontime15_per_h", "ontime30_per_h", "ontime60_per_h", "lead_mean_s", "lead_p95_s",
           "ready_n", "ready_time_mean_s", "ready_time_p95_s", "recoveries", "to_pod", "to_station", "to_next",
           "recovery_failures", "recovery_failure_rate", "pad_conflicts", "charger_conflicts",
           "wait_drone_min", "wh_per_parcel", "empty_return_km", "crashes", "flights", "pod_stranded",
           "waiting_orders_end", "fallbacks", "hover_rule_station", "infeasible_picks", "decisions",
           "latency_mean_ms",
           "latency_p95_ms", "pred_ready_mae_s", "wall_s"]
SUMMARY_METRICS = ["delivered_per_h", "ontime15_per_h", "ontime30_per_h", "ontime60_per_h", "lead_mean_s",
                   "lead_p95_s", "ready_time_mean_s", "ready_time_p95_s", "recovery_failure_rate",
                   "pad_conflicts", "charger_conflicts", "wait_drone_min", "wh_per_parcel",
                   "empty_return_km", "to_station", "fallbacks", "infeasible_picks", "latency_mean_ms",
                   "latency_p95_ms",
                   "pred_ready_mae_s"]


def hover_rule(world, d):
    """Frozen rule while hovering for the pod: stay while > 20 %, else go to the
    nearest feasible charger (or the nearest one). / ホバー中の固定ルール"""
    if d["batt"] > 20.0:
        return None
    opts = [o for o in world.options(d) if o[0] == "station"]
    est = [world.estimate(d, o) for o in opts]
    feas = [i for i, e in enumerate(est) if e["feasible"]] or list(range(len(opts)))
    return opts[min(feas, key=lambda i: est[i]["flight_s"])]


def run_episode(method_name, scenario, seed, log_decisions=True, checkpoint=None, method=None, pads=None,
                profile="v1", area=None, rate=None, meet=None, pod_share=0.0, world="v1", record=None):
    """One full episode; returns (episode row, decision records). ``method`` may
    be given directly (a trained model during training). / 1 エピソード"""
    if method is None:
        method = make_method(method_name, checkpoint, world=world)
    world = W1.Paper1World(scenario=scenario, seed=seed, pads=pads, profile=profile, area=area, rate=rate,
                           meet=meet, pod_share=pod_share, world=world)
    world.reset(seed=seed)
    if record is not None:
        # watch mode: frames every tick (5 s) and the event list / ウォッチ用の記録
        world.log, world.events = [], []
        record["world"] = world
    decisions, open_dec, lat = [], {}, []
    n_ready = 0
    t_wall = time.time()
    while True:
        p = world.advance()
        # attach each actual ready time to that drone's open decision / 実際の準備完了時間
        ev = world.p1["ready_events"]
        while n_ready < len(ev):
            did, t_drop, t_ready = ev[n_ready]
            n_ready += 1
            rec = open_dec.pop(did, None)
            if rec is not None and rec["t_drop"] == t_drop:
                rec["actual_ready_s"] = round(t_ready - t_drop, 1)
        if p is None:
            break
        if p[0] == "pod":
            world.pod_act(p[1], world.pod_rule(p[1]))
            continue
        d, kind = p[1], p[2]
        if kind == F.D_AFTER:
            t0 = time.perf_counter()
            opt, info = method.decide(world, d)
            ms = (time.perf_counter() - t0) * 1000.0
            lat.append(ms)
            world.recover(d, opt)
            chosen = ("pod" if opt[1] == 0 else "pod_next") if opt[0] == "pod" else "station%d" % opt[1]
            world._event("recover", drone=d["id"], kind=chosen)
            rec = {"method": method_name, "scenario": scenario, "eval_seed": seed, "t": round(world.t, 1),
                   "drone": d["id"], "batt": round(d["batt"], 1), "chosen": chosen, "latency_ms": round(ms, 3),
                   "t_drop": d.get("p1_drop_t"), "actual_ready_s": None}
            rec.update(info)
            decisions.append(rec)
            open_dec[d["id"]] = rec
        elif kind == F.D_HOVER:
            opt = hover_rule(world, d)
            if opt is None:
                d["ask"] = None
            else:
                world.recover(d, opt, by_method=False)              # frozen rule, not counted / 固定ルール
        elif kind == F.D_LAUNCH:
            world.drone_act(d, kind, 1)                              # medium height / 中高度
        else:
            world.drone_act(d, kind, 1)
    k = world.p1_kpi()
    errs = [abs(r["actual_ready_s"] - r["predicted_ready_s"]) for r in decisions
            if r["actual_ready_s"] is not None and r["predicted_ready_s"] == r["predicted_ready_s"]]
    row = dict(k, method=method_name, scenario=scenario, seed=seed, decisions=len(decisions),
               latency_mean_ms=float(np.mean(lat)) if lat else None,
               latency_p95_ms=float(np.percentile(lat, 95)) if lat else None,
               pred_ready_mae_s=float(np.mean(errs)) if errs else None,
               wall_s=round(time.time() - t_wall, 1))
    return row, (decisions if log_decisions else [])


def _job(args):
    name, scen, seed, ckpt, pads, profile, area, rate, meet, pod_share, world = args
    return run_episode(name, scen, seed, checkpoint=ckpt, pads=pads, profile=profile, area=area, rate=rate,
                       meet=meet, pod_share=pod_share, world=world)


# two-sided 95 % Student-t critical values, df 1..30 (beyond: 1.96 + small-df term)
# t 分布の両側 95 % 臨界値
_T975 = [12.706, 4.303, 3.182, 2.776, 2.571, 2.447, 2.365, 2.306, 2.262, 2.228, 2.201, 2.179, 2.160, 2.145,
         2.131, 2.120, 2.110, 2.101, 2.093, 2.086, 2.080, 2.074, 2.069, 2.064, 2.060, 2.056, 2.052, 2.048,
         2.045, 2.042]


def t975(df):
    return _T975[df - 1] if 1 <= df <= 30 else 1.96 + 2.4 / df


def ci95(x):
    """Mean and 95 % confidence interval (t distribution). / 平均と 95 % 信頼区間"""
    x = np.array([v for v in x if v is not None and not (isinstance(v, float) and math.isnan(v))], float)
    if x.size == 0:
        return None, None, None, 0
    m = float(x.mean())
    if x.size < 2:
        return m, m, m, int(x.size)
    h = float(t975(x.size - 1) * x.std(ddof=1) / math.sqrt(x.size))
    return m, m - h, m + h, int(x.size)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--method", default="B0", choices=sorted(METHODS) + list(LEARNED) + list(ROLLOUT))
    ap.add_argument("--checkpoint", default=None, help="B1-B3: the trained model (.zip) / 学習済みモデル")
    ap.add_argument("--tag", default="", help="file tag (default: the method) / ファイル名の識別子")
    ap.add_argument("--pads", type=int, default=0, help="charging pads per station (0 = all) / 局あたりのパッド数")
    ap.add_argument("--profile", default="v1", help="hardware profile: v1 | typical / 機体プロファイル")
    ap.add_argument("--area", default="", help='zones served by the hub, e.g. "1,3" (default: zone 1) / 担当ゾーン')
    ap.add_argument("--rate", type=float, default=0.0, help="orders per hour (default: the scenario's) / 注文数/時")
    ap.add_argument("--world", default="v1", help="Paper 1 world version: v1 | v2 / 世界のバージョン")
    ap.add_argument("--pod-share", type=float, default=0.0,
                    help="fraction of orders the pod delivers at lockers (default 0) / ポッド配達の割合")
    ap.add_argument("--meet", default="", help="diagnostic meeting point: 0 | 1 | 2 | best (default: the rule) / 合流地点")
    ap.add_argument("--scenarios", default=",".join(W1.SCENARIOS))
    ap.add_argument("--seeds", type=int, default=30)
    ap.add_argument("--workers", type=int, default=12)
    ap.add_argument("--out", default=OUT)
    a = ap.parse_args()
    scen = [s for s in a.scenarios.split(",") if s]
    os.makedirs(a.out, exist_ok=True)
    area = [int(z) for z in a.area.split(",") if z] or None
    meet = (a.meet if a.meet == "best" else int(a.meet)) if a.meet else None
    jobs = [(a.method, s, SEED0 + k, a.checkpoint, a.pads or None, a.profile, area, a.rate or None, meet,
             a.pod_share, a.world)
            for s in scen for k in range(a.seeds)]
    t0 = time.time()
    with Pool(a.workers) as pool:
        res = pool.map(_job, jobs, chunksize=1)
    tag = a.tag or a.method
    ep_path = os.path.join(a.out, "results_episodes_%s.csv" % tag)
    with open(ep_path, "w", newline="") as f:
        w = csv.DictWriter(f, EP_COLS, extrasaction="ignore")
        w.writeheader()
        for row, _d in res:
            w.writerow({k: (round(v, 4) if isinstance(v, float) else v) for k, v in row.items()})
    with open(os.path.join(a.out, "decisions_%s.jsonl" % tag), "w") as f:
        for _row, decs in res:
            for r in decs:
                f.write(json.dumps(r) + "\n")
    with open(os.path.join(a.out, "summary_%s.csv" % tag), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["method", "scenario", "metric", "mean", "ci_low", "ci_high", "n"])
        for s in scen:
            rows = [r for r, _d in res if r["scenario"] == s]
            for mname in SUMMARY_METRICS:
                m, lo, hi, n = ci95([r.get(mname) for r in rows])
                w.writerow([a.method, s, mname] + [None if v is None else round(v, 4) for v in (m, lo, hi)] + [n])
    with open(os.path.join(a.out, "config_%s.json" % tag), "w") as f:
        json.dump({"method": a.method, "checkpoint": a.checkpoint, "pads_per_station": a.pads or None,
                   "profile": a.profile, "profile_values": W1.PROFILES[a.profile], "area": area,
                   "rate_override": a.rate or None, "meet": meet,
                   "pod_share": a.pod_share, "world": a.world, "world_preset": W1.WORLDS[a.world],
                   "scenarios": {s: W1.SCENARIOS[s] for s in scen},
                   "seeds": [SEED0, SEED0 + a.seeds - 1], "episode_s": F.EP_S, "zone": W1.PAPER1_ZONE,
                   "ready_pct": W1.READY_PCT, "reserve_pct": W1.RESERVE_PCT,
                   "station_charge_to": W1.STATION_CHARGE_TO, "est_margin": W1.EST_MARGIN,
                   "on_time_s": W1.ON_TIME_S, "drones": F.N_PER_POD, "land_s": F.LAND_S,
                   "params": W1.Paper1World().P, "wall_s": round(time.time() - t0, 1),
                   "frozen_rules": "pod: fleet_rules.rule_pod; launch height: medium; hover: nearest "
                                   "feasible charger below 20 %"}, f, indent=1)
    print("[paper1] %s: %d episodes in %.0f s -> %s" % (a.method, len(jobs), time.time() - t0, a.out))


if __name__ == "__main__":
    main()
