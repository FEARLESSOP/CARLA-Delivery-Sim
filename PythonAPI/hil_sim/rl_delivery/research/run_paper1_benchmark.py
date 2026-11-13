"""
Paper 1 Experimental Benchmark Runner: FA-RAMPPO vs. Baselines (B0 to B5).

Evaluates the 6 matched baselines across Scenarios S1-S5:
    S1: Normal daytime operations
    S2: Train arrival demand surge (2.5x order rate)
    S3: Severe traffic congestion & delays
    S4: Adverse weather (turbulent wind gusts)
    S5: Unseen held-out disruptions

Produces publication-grade metrics and comparison tables:
    - Expected Cumulative Fleet Return (r_k)
    - On-Time Delivery Share (%)
    - 95th-Percentile Lead Time (s)
    - Drone Unavailable Time (minutes)
    - Total Fleet Energy Consumption (Wh/delivery)
    - Decision Latency (ms)
"""

import json
import os
import sys
import time
from pathlib import Path

# Add project root to path
HIL_SIM = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(HIL_SIM))

import numpy as np
from rl_delivery.ramppo import (
    FARAMPPOPlanner,
    BaselineType,
    evaluate_scenario_benchmark,
)

RESULTS_DIR = HIL_SIM / "rl_delivery" / "research" / "paper1_results"


def run_full_paper1_campaign(num_episodes: int = 10, seed: int = 42):
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    scenarios = [
        ("S1_Normal", "Normal Daytime Demand"),
        ("S2_Surge", "Train Arrival Demand Surge"),
        ("S3_Traffic", "Severe Traffic Congestion"),
        ("S4_Weather", "High Wind & Adverse Weather"),
        ("S5_HeldOut", "Unseen Held-Out Disruptions"),
    ]

    all_results = {}
    print("=" * 80)
    print("  PAPER 1 EXPERIMENTAL CAMPAIGN: FA-RAMPPO (B4) vs. BASELINES (B0-B5)")
    print("=" * 80)

    for s_key, s_name in scenarios:
        print(f"\n>>> Running Scenario {s_key}: {s_name} ({num_episodes} episodes/baseline)...")
        t0 = time.time()
        s_res = evaluate_scenario_benchmark(
            scenario_name=s_key,
            num_episodes=num_episodes,
            seed=seed,
        )
        elapsed = time.time() - t0
        all_results[s_key] = s_res
        print(f"    Completed in {elapsed:.1f}s")

        # Print quick summary table
        print(f"    {'Baseline':<25} | {'Reward (mean±std)':<20} | {'Feasibility':<12} | {'Latency (ms)':<12}")
        print("    " + "-" * 75)
        for b_name, m in s_res.items():
            rew_str = f"{m['mean_reward']:.2f} ± {m['std_reward']:.2f}"
            print(f"    {b_name:<25} | {rew_str:<20} | {m['feasibility_rate']*100:.1f}%{'':<6} | {m['mean_latency_ms']:.2f} ms")

    # Save to disk
    out_json = RESULTS_DIR / "paper1_baseline_comparison.json"
    with open(out_json, "w") as f:
        json.dump(all_results, f, indent=2)
    print(f"\n[OK] Results saved to: {out_json}")

    return all_results


if __name__ == "__main__":
    episodes = int(sys.argv[1]) if len(sys.argv) > 1 else 5
    run_full_paper1_campaign(num_episodes=episodes)
