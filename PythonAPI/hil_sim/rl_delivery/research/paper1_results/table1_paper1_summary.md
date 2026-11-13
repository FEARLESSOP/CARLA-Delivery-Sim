# Table 1: Comprehensive Baseline Comparison across Scenarios (Paper 1)

| Scenario | Baseline Method | Return ($r_k$) | Feasibility (%) | Latency (ms) |
|---|---|---|---|---|
| S1 Normal | B0: Heuristic | 5.980 ± 0.044 | 100.0% | 0.03 |
|  | B1: Std PPO | 5.990 ± 0.036 | 100.0% | 0.03 |
|  | B2: Maskable PPO | 5.990 ± 0.036 | 100.0% | 0.03 |
|  | B3: Forecast PPO | 5.990 ± 0.036 | 100.0% | 0.02 |
|  | **B4: FA-RAMPPO (Ours)** | **5.990 ± 0.036** | 100.0% | 1.86 |
|  | B5: Rollout Heur | 5.991 ± 0.034 | 100.0% | 1.83 |
| S2 Surge | B0: Heuristic | 5.891 ± 0.056 | 100.0% | 0.02 |
|  | B1: Std PPO | 5.881 ± 0.077 | 100.0% | 0.02 |
|  | B2: Maskable PPO | 5.881 ± 0.077 | 100.0% | 0.02 |
|  | B3: Forecast PPO | 5.881 ± 0.077 | 100.0% | 0.02 |
|  | **B4: FA-RAMPPO (Ours)** | **5.881 ± 0.077** | 100.0% | 1.76 |
|  | B5: Rollout Heur | 5.898 ± 0.049 | 100.0% | 1.75 |
| S3 Traffic | B0: Heuristic | 5.891 ± 0.056 | 100.0% | 0.02 |
|  | B1: Std PPO | 5.881 ± 0.077 | 100.0% | 0.02 |
|  | B2: Maskable PPO | 5.881 ± 0.077 | 100.0% | 0.02 |
|  | B3: Forecast PPO | 5.881 ± 0.077 | 100.0% | 0.02 |
|  | **B4: FA-RAMPPO (Ours)** | **5.881 ± 0.077** | 100.0% | 1.75 |
|  | B5: Rollout Heur | 5.898 ± 0.049 | 100.0% | 1.93 |
| S4 Weather | B0: Heuristic | 5.891 ± 0.056 | 100.0% | 0.02 |
|  | B1: Std PPO | 5.881 ± 0.077 | 100.0% | 0.02 |
|  | B2: Maskable PPO | 5.881 ± 0.077 | 100.0% | 0.02 |
|  | B3: Forecast PPO | 5.881 ± 0.077 | 100.0% | 0.02 |
|  | **B4: FA-RAMPPO (Ours)** | **5.881 ± 0.077** | 100.0% | 1.76 |
|  | B5: Rollout Heur | 5.898 ± 0.049 | 100.0% | 1.77 |
| S5 HeldOut | B0: Heuristic | 5.891 ± 0.056 | 100.0% | 0.02 |
|  | B1: Std PPO | 5.881 ± 0.077 | 100.0% | 0.02 |
|  | B2: Maskable PPO | 5.881 ± 0.077 | 100.0% | 0.02 |
|  | B3: Forecast PPO | 5.881 ± 0.077 | 100.0% | 0.02 |
|  | **B4: FA-RAMPPO (Ours)** | **5.881 ± 0.077** | 100.0% | 1.76 |
|  | B5: Rollout Heur | 5.898 ± 0.049 | 100.0% | 1.76 |
