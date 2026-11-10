"""Compare the runs of one experiment (tag): table + learning-curve chart.

  python3 -m rl_delivery.compare --tag nav_algo_compare
  python3 -m rl_delivery.compare --tag nav_algo_compare --metric landed_or_reached

Reads research/exp/<tag>/<run>/ (config.json + <brain>_progress.csv), groups
runs by algorithm, and writes to research/exp/<tag>/:
- summary.csv   per run: algorithm, seed, steps, final metrics (mean of the
                last 3 evaluations)
- table.csv     per algorithm: mean, std and n over seeds for each metric
- table.md      the same as a Markdown table (paste into the paper draft)
- curves.png    the main metric vs training steps, mean +- std over seeds

実験（タグ）ごとの比較表と学習曲線。
"""
import argparse
import glob
import json
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))

MAIN = {"nav": "reached", "drive": "reached", "fleet": "deliveries_per_h", "flight": "reached",
        "mission": "deliveries"}
METRICS = {
    "nav": ["reached", "delivered", "crash", "crash_obstacle", "crash_nofly", "crash_battery", "timeout", "dist"],
    "drive": ["reached", "kerb", "collision", "stop_violations", "red_violations", "speed", "route_len", "traffic"],
    "fleet": ["deliveries_per_h", "crashes_per_100", "stranded", "hover_s_per_h", "lead_mean_s", "waiting"],
}


def load_runs(tag):
    import pandas as pd
    runs = []
    for d in sorted(glob.glob(os.path.join(HERE, "research", "exp", tag, "*"))):
        cfg_p = os.path.join(d, "config.json")
        if not os.path.exists(cfg_p):
            continue
        cfg = json.load(open(cfg_p))
        brain = cfg["args"]["brain"]
        prog = os.path.join(d, "%s_progress.csv" % brain)
        if not os.path.exists(prog):
            continue
        df = pd.read_csv(prog)
        if df.empty:
            continue
        runs.append({"run": os.path.basename(d), "brain": brain, "algo": cfg["args"].get("algo") or "default",
                     "seed": cfg["args"].get("seed", 0), "df": df})
    return runs


def main():
    import pandas as pd
    ap = argparse.ArgumentParser()
    ap.add_argument("--tag", required=True)
    ap.add_argument("--metric", default="")
    a = ap.parse_args()
    runs = load_runs(a.tag)
    if not runs:
        raise SystemExit("no finished runs with progress under research/exp/%s" % a.tag)
    out = os.path.join(HERE, "research", "exp", a.tag)
    brain = runs[0]["brain"]
    metrics = [m for m in METRICS.get(brain, []) if m in runs[0]["df"].columns]
    main_m = a.metric or MAIN.get(brain, metrics[0])
    rows = []
    for r in runs:
        tail = r["df"].tail(3)
        row = {"run": r["run"], "algo": r["algo"], "seed": r["seed"], "steps": int(r["df"]["steps"].iloc[-1])}
        row.update({m: float(tail[m].mean()) for m in metrics})
        rows.append(row)
    summ = pd.DataFrame(rows)
    summ.to_csv(os.path.join(out, "summary.csv"), index=False)
    agg = summ.groupby("algo")[metrics + ["steps"]].agg(["mean", "std", "count"])
    agg.to_csv(os.path.join(out, "table.csv"))
    lines = ["| algorithm | n | " + " | ".join(metrics) + " |", "|---|---|" + "---|" * len(metrics)]
    for algo, g in summ.groupby("algo"):
        cells = ["%.3g ± %.2g" % (g[m].mean(), g[m].std() if len(g) > 1 else 0.0) for m in metrics]
        lines.append("| %s | %d | %s |" % (algo, len(g), " | ".join(cells)))
    with open(os.path.join(out, "table.md"), "w") as f:
        f.write("Experiment `%s` (%s): mean ± std over seeds, last 3 evaluations per run.\n\n" % (a.tag, brain))
        f.write("\n".join(lines) + "\n")
    # learning curves: mean +- std over seeds on a common step grid / 学習曲線
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(7, 4), dpi=150)
    for algo in sorted(summ["algo"].unique()):
        group = [r for r in runs if r["algo"] == algo]
        top = min(int(r["df"]["steps"].iloc[-1]) for r in group)
        grid = np.linspace(0, top, 60)
        ys = np.array([np.interp(grid, r["df"]["steps"], r["df"][main_m]) for r in group])
        m, sd = ys.mean(0), ys.std(0)
        ax.plot(grid, m, label="%s (n=%d)" % (algo, len(group)))
        ax.fill_between(grid, m - sd, m + sd, alpha=0.2)
    ax.set_xlabel("training steps")
    ax.set_ylabel(main_m)
    ax.set_title("%s: %s" % (a.tag, main_m))
    ax.grid(alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(out, "curves.png"))
    print("\n".join(lines))
    print("-> %s/{summary.csv, table.csv, table.md, curves.png}" % out)


if __name__ == "__main__":
    main()
