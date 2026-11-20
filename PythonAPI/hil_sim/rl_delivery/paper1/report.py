"""All Paper 1 data for the dashboard's results page (/paper1), read locally.

``build()`` reads rl_delivery/research/paper1/ (episode results, training progress,
tuning and crash-cause files) and returns one JSON-ready dict: per method / seed /
scenario means, seed-averaged paired differences with 95 % intervals, training
curves, the B4 / B5 tuning runs, the world-design checks and crash causes. Plain
Python (no numpy), cached on the files' modification times, so new runs appear the
next time the page loads. Nothing leaves the machine.

Paper 1 の全データ（結果ページ用）。研究フォルダのファイルを読み、平均・対応のある差（95 %
区間）・学習曲線・調整・世界設計の確認・墜落原因をまとめる。ファイルが変われば再計算する。
"""
import csv
import glob
import json
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
RES = os.path.join(os.path.dirname(HERE), "research", "paper1")
SCEN = ["S1_normal", "S2_rush_hour", "S3_pod_delay", "S4_station_outage", "S5_high_wind",
        "S6_burst_returns", "S7_unseen_customers"]
METRICS = ["delivered_per_h", "drone_per_h", "pod_per_h", "ontime15_per_h", "ontime30_per_h",
           "ready_time_mean_s", "ready_time_p95_s", "recovery_failure_rate", "recovery_failures",
           "crashes", "flights", "pod_stranded", "waiting_orders_end", "to_pod", "to_next", "to_station",
           "charger_conflicts", "pad_conflicts", "wait_drone_min", "wh_per_parcel", "infeasible_picks",
           "fallbacks", "pred_ready_mae_s", "latency_mean_ms"]
DIFF_METRICS = ["delivered_per_h", "ontime30_per_h", "ready_time_mean_s", "recovery_failure_rate", "crashes"]
WORLDS = {
    "v2": {"B0": ["B0_v2"], "POD": ["POD_v2"], "B1": ["B1_s%d_v2" % s for s in range(3)],
           "B2": ["B2_s%d_v2" % s for s in range(3)], "B3": ["B3_s%d_v2" % s for s in range(3)],
           "B4": ["B4_s%d_v2" % s for s in range(3)], "B4conf": ["B4conf_s%d_v2" % s for s in range(3)],
           "B5": ["B5_v2"]},
    "v1": {"B0": ["B0"], "B1": ["B1_s%d" % s for s in range(3)], "B2": ["B2_s%d" % s for s in range(3)],
           "B3": ["B3_s%d" % s for s in range(3)]},
}
PAIRS = {"v2": [("B4", "B3"), ("B4", "B2"), ("B4", "B5"), ("B4", "B4conf"), ("B4conf", "B3"), ("B5", "B3"),
                ("B5", "B2"), ("B3", "B2"), ("B2", "B1")],
         "v1": [("B3", "B2"), ("B2", "B1")]}
CHECKS = [   # world-design checks (docs/PAPER1_BASELINES.md 3b) / 世界設計の確認
    {"id": "v1", "label": "Original world (v1)", "detail": "zone 1, drone-only, 16 orders/h, 3 pads, placeholder hardware",
     "runs": {"B0": "B0"}},
    {"id": "pad1", "label": "Typical hardware + 1 pad", "detail": "zone 1, drone-only, 16 orders/h",
     "runs": {"B0": "B0_pad1_typical", "POD": "POD_pad1_typical"}},
    {"id": "pressure", "label": "Busier world", "detail": "zones 1+3, 25 orders/h, typical hardware, 1 pad",
     "runs": {"B0": "B0_pressure", "POD": "POD_pressure"}},
    {"id": "meet", "label": "Fixed meeting points", "detail": "busier world; here / middle / next / best per launch",
     "runs": {"here": "B0_pressure_meet0", "middle": "B0_pressure_meet1", "next": "B0_pressure_meet2",
              "best": "B0_pressure_meetbest"}},
    {"id": "podwork", "label": "Pod delivers 40 % itself", "detail": "busier world + locker deliveries by the pod",
     "runs": {"B0": "B0_podwork_rule", "POD": "POD_podwork_rule", "here": "B0_podwork_meet0",
              "next": "B0_podwork_meet2", "best": "B0_podwork_meetbest"}},
    {"id": "v2", "label": "World v2 (adopted)", "detail": "pod deliveries + charging fix + next-stop option",
     "runs": {"B0": "B0_v2", "POD": "POD_v2"}},
]
T975 = {29: 2.045, 209: 1.971}
_cache = {"sig": None, "data": None}


def _num(v):
    try:
        x = float(v)
        return x if math.isfinite(x) else None
    except (TypeError, ValueError):
        return None


def _load(tag):
    path = os.path.join(RES, "results_episodes_%s.csv" % tag)
    if not os.path.exists(path):
        return None
    with open(path) as f:
        return {(r["scenario"], r["seed"]): r for r in csv.DictReader(f)}


def _mean(vals):
    v = [x for x in vals if x is not None]
    return sum(v) / len(v) if v else None


def _summ(rows):
    """Means of every metric over episode rows, plus crashes per 100 flights. / 平均"""
    out = {k: _mean([_num(r.get(k)) for r in rows]) for k in METRICS}
    fl = sum(_num(r.get("flights")) or 0.0 for r in rows)
    out["crashes_per100"] = 100.0 * sum(_num(r.get("crashes")) or 0.0 for r in rows) / fl if fl else None
    out["n"] = len(rows)
    return out


def _ci(d):
    d = [x for x in d if x is not None]
    n = len(d)
    if n < 2:
        return None
    mu = sum(d) / n
    sd = math.sqrt(sum((x - mu) ** 2 for x in d) / (n - 1))
    h = T975.get(n - 1, 1.96) * sd / math.sqrt(n)
    return {"diff": mu, "lo": mu - h, "hi": mu + h, "sig": abs(mu) > h, "n": n}


def _signature():
    files = sorted(glob.glob(os.path.join(RES, "*.csv")) + glob.glob(os.path.join(RES, "*.json")))
    return tuple((os.path.basename(p), os.path.getmtime(p)) for p in files)


def build():
    sig = _signature()
    if _cache["sig"] == sig and _cache["data"] is not None:
        return _cache["data"]
    results, diffs, missing = [], [], []
    for world, methods in WORLDS.items():
        loaded = {}
        for m, tags in methods.items():
            runs = [(t, _load(t)) for t in tags]
            missing += [t for t, d in runs if d is None]
            loaded[m] = [d for _t, d in runs if d is not None]
            for i, (t, d) in enumerate(runs):
                if d is None:
                    continue
                seed = i if len(tags) > 1 else None
                for sc in SCEN + ["ALL"]:
                    rows = [r for (s, _), r in d.items() if sc == "ALL" or s == sc]
                    results.append(dict(_summ(rows), world=world, method=m, seed=seed, scenario=sc, tag=t))
            if len(loaded[m]) > 1:                       # seed average / シード平均
                for sc in SCEN + ["ALL"]:
                    rows = [r for d in loaded[m] for (s, _), r in d.items() if sc == "ALL" or s == sc]
                    results.append(dict(_summ(rows), world=world, method=m, seed="avg", scenario=sc, tag=m + "_avg"))
        base = loaded.get("B0", [None])[0]
        pairs = [(m, "B0") for m in methods if m != "B0"] + PAIRS[world]
        for a, b in pairs:
            A, B = loaded.get(a), loaded.get(b)
            if not A or not B or base is None:
                continue
            keys = [k for k in base if all(k in d for d in A + B)]
            for sc in SCEN + ["ALL"]:
                kk = [k for k in keys if sc == "ALL" or k[0] == sc]
                for met in DIFF_METRICS:
                    d = []
                    for k in kk:
                        va = _mean([_num(x[k].get(met)) for x in A])
                        vb = _mean([_num(x[k].get(met)) for x in B])
                        d.append(None if va is None or vb is None else va - vb)
                    c = _ci(d)
                    if c:
                        diffs.append(dict(c, world=world, a=a, b=b, metric=met, scenario=sc))
    training = {}
    for path in sorted(glob.glob(os.path.join(RES, "train_B?_s?*.csv"))):
        name = os.path.basename(path)[6:-4]              # B3_s0_v2 / B3_s0
        parts = name.split("_")
        world = "v2" if parts[-1] == "v2" else "v1"
        with open(path) as f:
            training["%s|%s|%s" % (world, parts[0], parts[1][1:])] = [
                {k: _num(v) for k, v in r.items()} for r in csv.DictReader(f)]
    checks = []
    for c in CHECKS:
        runs = {}
        for role, tag in c["runs"].items():
            d = _load(tag)
            if d is not None:
                runs[role] = _summ(list(d.values()))
        pair = None
        if "B0" in c["runs"] and "POD" in c["runs"]:
            A, B = _load(c["runs"]["B0"]), _load(c["runs"]["POD"])
            if A and B:
                pair = {}
                for met in ("delivered_per_h", "ontime30_per_h", "ready_time_mean_s"):
                    pair[met] = _ci([(_num(A[k].get(met)) or 0.0) - (_num(B[k].get(met)) or 0.0) for k in A if k in B])
        checks.append(dict(c, summary=runs, b0_minus_pod=pair))

    def _json(name):
        p = os.path.join(RES, name)
        try:
            with open(p) as f:
                return json.load(f)
        except (OSError, ValueError):
            return None
    # CARLA comparisons (carla_ab_run.py), the first B4's diagnosis / CARLA の比較と診断
    carla = []
    for path in sorted(glob.glob(os.path.join(RES, "carla_ab_*.json"))):
        d = _json(os.path.basename(path)) or {}
        st = d.get("settings", {})
        name = os.path.basename(path)
        carla.append({"file": name, "hardware": st.get("hardware", "carla"),
                      "b4": "B4conf" if "20261117" in name else "B4", "minutes": st.get("minutes"),
                      "runs": {m: {k: v for k, v in r.items() if k != "kpi"} for m, r in (d.get("runs") or {}).items()}})
    data = {"scenarios": SCEN, "results": results, "diffs": diffs, "training": training, "checks": checks,
            "tuning": _json("tuning_rollouts_v2.json"), "tuning2": _json("tuning_b4_revised_v2.json"),
            "carla": carla, "diagnosis": _json("carla_diagnosis_20261118.json"),
            "causes": _json("crash_causes_B0_v2.json"),
            "missing": missing, "files": len(sig), "updated": max((m for _n, m in sig), default=0.0)}
    _cache["sig"], _cache["data"] = sig, data
    return data


if __name__ == "__main__":
    d = build()
    print(len(d["results"]), "result rows,", len(d["diffs"]), "differences,", len(d["training"]), "curves,",
          "missing:", d["missing"])
