"""
Generates publication-grade figures and LaTeX tables for Paper 1 (FA-RAMPPO).

Outputs (saved to rl_delivery/research/paper1_results/):
- fig1_cumulative_reward.png: Cumulative event reward across Scenarios S1-S5
- fig2_decision_latency.png: Sub-second latency vs 1000ms real-time limit
- table1_paper1_summary.md / .tex: Publication benchmark comparison table
"""

import json
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

RESULTS_DIR = Path(__file__).resolve().parent / "paper1_results"
JSON_PATH = RESULTS_DIR / "paper1_baseline_comparison.json"


def generate_paper1_deliverables():
    if not JSON_PATH.exists():
        print(f"Error: {JSON_PATH} does not exist.")
        return

    with open(JSON_PATH) as f:
        data = json.load(f)

    scenarios = list(data.keys())
    baselines = list(data[scenarios[0]].keys())
    clean_b_labels = [
        "B0: Heuristic",
        "B1: Std PPO",
        "B2: Maskable PPO",
        "B3: Forecast PPO",
        "B4: FA-RAMPPO (Ours)",
        "B5: Rollout Heur",
    ]

    # Set publication styling
    plt.style.use("seaborn-v0_8-whitegrid" if "seaborn-v0_8-whitegrid" in plt.style.available else "default")
    plt.rcParams.update({
        "font.family": "sans-serif",
        "font.size": 11,
        "axes.labelsize": 12,
        "axes.titlesize": 13,
        "xtick.labelsize": 10,
        "ytick.labelsize": 10,
        "legend.fontsize": 10,
        "figure.titlesize": 14,
    })

    # -------------------------------------------------------------
    # Figure 1: Cumulative Event Return (Reward) across Scenarios
    # -------------------------------------------------------------
    fig, ax = plt.subplots(figsize=(10, 5), dpi=300)
    x = np.arange(len(scenarios))
    width = 0.13
    colors = ["#7f7f7f", "#d62728", "#ff7f0e", "#1f77b4", "#2ca02c", "#9467bd"]

    for i, b in enumerate(baselines):
        means = [data[s][b]["mean_reward"] for s in scenarios]
        stds = [data[s][b]["std_reward"] for s in scenarios]
        pos = x + (i - len(baselines)/2 + 0.5) * width
        bars = ax.bar(pos, means, width, yerr=stds, label=clean_b_labels[i], color=colors[i], capsize=3, alpha=0.9)
        # Highlight our proposed method (B4)
        if b == "B4_FA_RAMPPO":
            for bar in bars:
                bar.set_edgecolor("black")
                bar.set_linewidth(1.5)

    ax.set_ylabel("Expected Event Return ($r_k$)")
    ax.set_title("Figure 1: Fleet Recovery Return across Operating Scenarios (Paper 1)")
    ax.set_xticks(x)
    ax.set_xticklabels(["S1: Normal", "S2: Surge", "S3: Traffic", "S4: Weather", "S5: Held-Out"])
    ax.set_ylim(5.75, 6.15)
    ax.legend(frameon=True, loc="upper right", ncol=2)
    plt.tight_layout()
    fig1_path = RESULTS_DIR / "fig1_cumulative_reward.png"
    plt.savefig(fig1_path)
    plt.close()
    print(f"[Plot] Saved: {fig1_path}")

    # -------------------------------------------------------------
    # Figure 2: Decision Latency vs Sub-1-Second Execution Budget
    # -------------------------------------------------------------
    fig, ax = plt.subplots(figsize=(9, 4.8), dpi=300)
    s_normal = data["S1_Normal"]
    latencies = [s_normal[b]["mean_latency_ms"] for b in baselines]

    y_pos = np.arange(len(baselines))
    bars = ax.barh(y_pos, latencies, color=colors, alpha=0.85, height=0.6)
    # Highlight B4
    bars[4].set_edgecolor("black")
    bars[4].set_linewidth(1.8)

    # Budget threshold line at 1000 ms
    ax.axvline(1000.0, color="red", linestyle="--", linewidth=1.8, label="Decision Budget Limit (1000 ms)")
    ax.set_xscale("log")
    ax.set_xlim(0.01, 2000.0)
    ax.set_yticks(y_pos)
    ax.set_yticklabels(clean_b_labels)
    ax.invert_yaxis()
    ax.set_xlabel("Decision Latency (ms, log scale)")
    ax.set_title("Figure 2: Computational Decision Latency vs Real-Time Budget Limit")

    for i, v in enumerate(latencies):
        ax.text(v * 1.25, i, f"{v:.2f} ms", va="center", fontweight="bold" if i == 4 else "normal")

    ax.legend(loc="lower right")
    plt.tight_layout()
    fig2_path = RESULTS_DIR / "fig2_decision_latency.png"
    plt.savefig(fig2_path)
    plt.close()
    print(f"[Plot] Saved: {fig2_path}")

    # -------------------------------------------------------------
    # Table 1: Markdown & LaTeX Summary Tables
    # -------------------------------------------------------------
    md_lines = [
        "# Table 1: Comprehensive Baseline Comparison across Scenarios (Paper 1)",
        "",
        "| Scenario | Baseline Method | Return ($r_k$) | Feasibility (%) | Latency (ms) |",
        "|---|---|---|---|---|",
    ]
    tex_lines = [
        "\\begin{table*}[t]",
        "\\centering",
        "\\caption{Performance Comparison of FA-RAMPPO (B4) against Matched Baselines (B0--B5) across Operational Scenarios S1--S5.}",
        "\\label{tab:paper1_benchmark}",
        "\\begin{tabular}{llccc}",
        "\\toprule",
        "\\textbf{Scenario} & \\textbf{Method} & \\textbf{Return ($r_k$)} & \\textbf{Feasibility (\\%)} & \\textbf{Latency (ms)} \\\\",
        "\\midrule",
    ]

    for s in scenarios:
        s_clean = s.replace("_", " ")
        first_row = True
        for b, clean_name in zip(baselines, clean_b_labels):
            m = data[s][b]
            rew_str = f"{m['mean_reward']:.3f} $\\pm$ {m['std_reward']:.3f}"
            rew_md = f"{m['mean_reward']:.3f} ± {m['std_reward']:.3f}"
            scen_col = s_clean if first_row else ""
            scen_tex = f"\\multirow{{6}}{{*}}{{{s_clean}}}" if first_row else ""

            # Bold our method
            if b == "B4_FA_RAMPPO":
                clean_name_md = f"**{clean_name}**"
                clean_name_tex = f"\\textbf{{{clean_name}}}"
                rew_md = f"**{rew_md}**"
                rew_str = f"\\textbf{{{rew_str}}}"
            else:
                clean_name_md = clean_name
                clean_name_tex = clean_name

            md_lines.append(f"| {scen_col} | {clean_name_md} | {rew_md} | {m['feasibility_rate']*100:.1f}% | {m['mean_latency_ms']:.2f} |")
            tex_lines.append(f"{scen_tex} & {clean_name_tex} & {rew_str} & {m['feasibility_rate']*100:.1f}\\% & {m['mean_latency_ms']:.2f} \\\\")
            first_row = False
        tex_lines.append("\\midrule")

    tex_lines[-1] = "\\bottomrule"
    tex_lines.extend(["\\end{tabular}", "\\end{table*}"])

    md_path = RESULTS_DIR / "table1_paper1_summary.md"
    with open(md_path, "w") as f:
        f.write("\n".join(md_lines) + "\n")
    print(f"[Table] Saved Markdown: {md_path}")

    tex_path = RESULTS_DIR / "table1_paper1_summary.tex"
    with open(tex_path, "w") as f:
        f.write("\n".join(tex_lines) + "\n")
    print(f"[Table] Saved LaTeX: {tex_path}")


if __name__ == "__main__":
    generate_paper1_deliverables()
