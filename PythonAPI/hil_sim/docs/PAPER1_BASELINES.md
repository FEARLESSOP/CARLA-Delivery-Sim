# Paper 1 baselines B0–B5: definitions, status, data, comparison protocol

Everything needed to run and judge the Paper 1 comparison in one place: what
each baseline is, what is built, the data that exists (and why it cannot be
used), and the protocol, metrics and file formats for a fair comparison.
Written 2026-11-16 from the guide (`~/Downloads/Rollout_Assisted_Maskable_PPO_Oita_Paper1_Guide.docx`,
IWN Lab method note of 2026-11-09) and from the code as it is now.
Paths are relative to `~/Downloads/CARLA_0.9.16/PythonAPI/hil_sim/`.

日本語の要約: Paper 1（ドローン回収先の選択）で比較する 6 手法 B0〜B5 の定義・実装状況・
既存データ・比較手順をまとめた。B0 と B4/B5 の骨組みはあるが、B1〜B3 は学習済み
ネットワークが無く固定の好みで選んでおり、既存の結果（2026-11-12）は比較に使えない。
正しい比較には回収判断の学習環境、B1〜B3 の学習、7 つのシナリオ、対応のあるシード・
信頼区間が必要。

> **Status (2026-11-16) / 状況: B0 is complete, with results (section 3a).** B1–B5
> are not yet. The 2026-11-12 numbers in section 4 are not valid evidence; section
> 7 is the work still to do. / B0 は完成し結果あり（3a 章）。B1〜B5 は未完成。

---

## 1. The comparison in one paragraph / 比較の要点

**Research question (guide):** when one of four drones finishes a parcel
delivery, does evaluating short predicted futures (rollouts) make its
**recovery decision** better for the **whole fleet** than a learned PPO policy
alone?

**The decision:** a drone has just dropped its parcel. Choose where it recovers:
the LMP-4 mobile hub at an approved *stopped* location, fixed charger A, fixed
charger B (… K = 2–5 options), or a safe hold if one is modelled.

**Scope (Paper 1 only):** customers are served only by drones; the LMP-4 is a
parcel-and-recovery hub (no MiniPods); parcel dispatch to free drones is
**frozen** (a fixed rule) so that only the recovery choice differs between
methods. No landing on a moving vehicle.

**The contrast that answers the question:** **B4 vs B3** (rollouts vs giving
the policy the same forecasts as inputs). The others isolate the pieces:

| Contrast | What it shows |
|---|---|
| B4 vs B3 | value of decision-time lookahead beyond forecast features (**primary**) |
| B2 vs B1 | value of hard feasibility masking |
| B3 vs B2 | value of forecast features in the observation |
| B4 vs B5 | whether the learned PPO (prior + critic) is needed for rollout success |
| all vs B0 | everything vs a simple transparent rule (sanity floor) |

---

## 2. Common setup every baseline must share / 全手法で共通の条件

A comparison is only fair if all six see the **same world, same states and same
rules**:

| Item | Requirement |
|---|---|
| Environment | one recovery-decision environment (Gymnasium), used for training B1–B3 and for evaluating all six |
| Events | a decision event = a drone finished its delivery; other events (pod stops, charger frees, new orders) advance the clock |
| Action set | finite list a = (recovery destination, arrival slot); same list for all methods |
| Mask | the same feasibility engine for B0, B2–B5 (B1 deliberately unmasked; its infeasible choices are counted, not prevented) |
| Fleet | 4 drones, 1 LMP-4 with roof pad(s), ≥ 1–2 fixed chargers with capacity and queues, parcel stock on the pod |
| Dispatch | the same frozen rule assigns parcels to free drones in every method |
| Disturbances | traffic delay on the pod, wind, charger queue / outage, order arrivals — drawn from **paired seeds** (same seed list for every method) |
| Reward | one reward function for training and scoring (section 5.2) |
| Episode | a full operating period (e.g. 1–3 h of sim time) with many decisions, not one decision |
| Scoring | outcomes measured in the **execution** environment, never in the rollout model used by B4 / B5 |
| Compute | B4 / B5 decision deadline (target < 1 s); on timeout fall back to B0 and count it |

**Decided 2026-11-16 (user): Paper 1 follows the guide** — no-fly zones, battery
reserve, unsafe landing and reservation conflicts are **hard constraints** (mask +
safety supervisor) for B0, B2–B5; B1 is the unmasked PPO. Paper 1 is a separate
study: the live fleet brains stay pure RL (`docs/RL_DELIVERY.md` section 2). Better
alternatives may be proposed to the user. / ユーザー決定：Paper 1 はガイドどおりハード制約を使う。

---

## 3. The six baselines / 6 つの手法

Status key: ✅ done · 🟡 partial · ❌ not built.

### B0 — Nearest feasible + deterministic queue score / 最寄りの実行可能先＋待ち行列スコア ✅
- **What it is:** a non-learning, transparent operational rule. Among feasible
  recovery options, pick the one with the lowest *distance/flight time + expected
  queue wait* (from the reservation calendar). No learning, no lookahead.
- **Decision rule:** `a* = argmin_{a feasible} [ flight_time(a) + expected_wait(a) ]`
- **Isolates:** the floor every learned method must beat.
- **Code (2026-11-16):** `rl_delivery/paper1/methods.py`, `B0NearestFeasibleQueue`,
  using `Paper1World.estimate()` (`rl_delivery/paper1/world.py`). For each option
  (pod at its meeting point, charger 0 / 1 / 2) it estimates, without any random
  draw: flight time and energy round the no-fly zones; the wait (pod arrival at the
  meeting point, landings queued on the roof pad, the charger's queue); the charge
  time (in the pod 0.08 %/s up to 45 %; on a deck 1.5 %/s up to 90 %); for a charger
  also the empty flight back to the pod. Infeasible: < 15 % battery left on arrival
  (estimated energy × 1.2), charger out of service, pod stranded. It picks the
  lowest estimated drop-off → ready time; if nothing is feasible it falls back to
  the charger with the most battery on arrival (counted). The old branch in
  `rl_delivery/ramppo.py` (distance only) is superseded.
- **Status:** ✅ complete; tests `tests/test_paper1_b0.py` (7 checks).

### B1 — Standard PPO (no mask) / 標準 PPO（マスクなし）
- **What it is:** a PPO policy trained on the recovery environment that sees the
  state and outputs a recovery option, **without** action masking.
- **Decision rule:** `a* = argmax_a π_θ(a|s)` (deterministic at evaluation).
- **Isolates:** policy learning without explicit feasibility filtering. Its
  infeasible picks are executed as failures (or replaced by a fallback) and counted.
- **Code now:** branch `B1_STANDARD_PPO` — **no network**: argmax of a fixed
  preference vector (hub 1.8, chargers 1.0), mask ignored.
- **Status:** ✅ trained and evaluated, 3 seeds, worlds v1 and v2 (section 3b).

### B2 — Maskable PPO / マスク付き PPO
- **What it is:** PPO with invalid-action masking (sb3-contrib `MaskablePPO`):
  infeasible options get zero probability during training and evaluation.
- **Decision rule:** `a* = argmax_{a feasible} π_θ(a|s)`
- **Isolates:** the benefit of hard feasibility filtering (vs B1).
- **Code now:** branch `B2_MASKABLE_PPO` — **same fixed preference** as B1 with
  the mask applied (that is why B1–B3 give identical numbers).
- **Status:** ✅ trained and evaluated, 3 seeds, v1 and v2; its actor π_θ and critic V_θ
  (`checkpoints/paper1/v2/B2/s<seed>/final.zip`) are what B4 needs.

### B3 — Maskable PPO + forecast features / 予測特徴付きマスク PPO
- **What it is:** B2 whose observation also contains the forecasts the rollout
  would use: pod ETA to each stop (with uncertainty), expected queue at each
  pad/charger, charger availability, near-term order backlog.
- **Decision rule:** `a* = argmax_{a feasible} π_θ(a | s, forecasts)`
- **Isolates:** whether *giving* the policy predictions is enough without
  decision-time search — **the key comparison partner of B4**.
- **Code now:** branch `B3_FORECAST_PPO` — **identical to B2** (no forecast
  features exist).
- **Status:** ✅ trained and evaluated, 3 seeds, v1 and v2 (section 3b).
**Built 2026-11-16 (B1–B3 shared):** training env `rl_delivery/paper1/env.py`
(`Paper1RecoveryEnv`: one step = one recovery decision; Discrete(4) = pod / charger
0–2; `action_masks()` = B0's feasibility; observation `base` (30 inputs: drone,
pod, other drones, orders, wind, time, per-charger distance / occupancy / outage)
or `forecast` (+28: per option flight / wait / charge / return / ready time,
battery on arrival, feasible); trained on S1–S6 with seeds ≥ 10^6, S7 never seen),
trainer `rl_delivery/paper1/train.py` (PPO / MaskablePPO, net 128×128, n_steps
256 × 6 envs, batch 512, 10 epochs, lr 3e-4, γ 0.99, λ 0.95, ent 0.01, clip 0.2,
300 k steps, validation every 30 k on S1/S3/S5 seeds 2000–2003), evaluation via
`rl_delivery/paper1/methods.py` `PolicyMethod`. Running: seeds 0, 1, 2 for each,
then the 7 × 30 evaluation (`research/paper1/run_b1_b3.sh`, log `run_b1_b3.log`).


### B4 — Rollout-assisted Maskable PPO (proposed, "FA-RAMPPO") / 提案手法
- **What it is:** at each recovery decision, for every feasible candidate (or the
  policy's top-K), clone the compact state, simulate N disturbance scenarios H
  decision events ahead with **common random numbers**, add the critic at the
  horizon, combine with the policy prior, reserve and re-check.
- **Decision rule:**
  `Q̂_H(s,a) = (1/N) Σ_n [ Σ_{k=0}^{H−1} γ^k r_k^(n) + γ^H V_θ(s_H^(n)) ]`
  `a* = argmax_{a feasible} [ Q̂_H(s,a) + β log π_θ(a|s) ]`
  then `safety_supervisor.revalidate_and_reserve(a*)`.
- **Isolates:** the incremental value of simulating alternatives (vs B3).
- **Code now:** branch `B4_FA_RAMPPO` + `delivery/compact_simulator.py`
  `CompactSimulator.evaluate_candidate_action`:
  - mask ✅; rollout loop ✅ (N = 8, H = 4, γ = 0.95, CRN seeds `base_seed + n·10007`) ✅;
  - **V_θ is a hand-written score** (`_heuristic_terminal_value`), not the PPO critic;
  - **π_θ is the fixed preference** (hub 1.8 / chargers 1.0); in the live CARLA
    bridge (`delivery/rl_bridge.py`, `FleetRecoveryBridge`) the "prior" is the
    *fleet* drone brain's action (a different action space), queried with a zero
    observation and mapped arbitrarily to hub/charger;
  - other drones do nothing during rollouts (no continuation policy);
  - reservation booking ✅ (`PadReservationManager.book_slot`).
- **Status:** 🟡
- **To finish:** use B2's real π_θ (action probabilities) and V_θ; continuation of
  the other drones by the trained policy (or the frozen dispatch rule); top-K
  candidate selection; decision deadline with B0 fallback; log every predicted
  Q̂, the actual outcome, mask reasons and fallbacks.

### B5 — Rollout search with a fixed (non-learned) continuation / 学習なしロールアウト
- **What it is:** the same rollouts as B4 but **no learned parts**: no prior, a
  heuristic terminal value, other drones follow a fixed rule.
- **Decision rule:** `a* = argmax_{a feasible} Q̂_H^{fixed}(s,a)`
- **Isolates:** whether PPO is needed at all for rollout success (vs B4).
- **Code now:** branch `B5_ROLLOUT_HEURISTIC` — rollouts + argmax ✅, no prior ✅;
  but **no continuation rule runs** (other drones idle).
- **Status:** 🟡
- **To finish:** a fixed continuation rule for the other drones (e.g. B0 for their
  recoveries + the frozen dispatch), same N / H / γ / seeds as B4.

### Summary / まとめ

| ID | Learned? | Mask | Forecast inputs | Rollouts | Status |
|---|---|---|---|---|---|
| B0 | no | yes | — | no | ✅ complete, results in 3a |
| B1 | PPO | **no** | no | no | ✅ 3 seeds, v1 + v2 |
| B2 | MaskablePPO | yes | no | no | ✅ 3 seeds, v1 + v2 |
| B3 | MaskablePPO | yes | **yes** | no | ✅ 3 seeds, v1 + v2 |
| B4 | MaskablePPO prior + critic | yes | (in rollouts) | **yes** | ✅ 3 seeds, v2 (beats B3) |
| B5 | no | yes | (in rollouts) | **yes** | ✅ v2 (beats B0, below B4) |

---

## 3a. B0 results (2026-11-16) / B0 の結果

Run: `../../carla-venv/bin/python -m rl_delivery.paper1.evaluate --method B0 --seeds 30`
(210 episodes, 33 s). 7 scenarios × 30 paired seeds (1000–1029) × 3 h of sim time
each, in the Paper 1 world (section 6). Mean per episode with the 95 % confidence
interval (t, n = 30). Files: `rl_delivery/research/paper1/results_episodes_B0.csv`
(one row per episode), `decisions_B0.jsonl` (7,326 recovery decisions: mask,
reasons, every option's estimate, choice, latency, predicted and actual ready
time), `summary_B0.csv`, `config_B0.json`.

| Metric | S1 normal | S2 rush hour | S3 pod delay | S4 outage | S5 high wind | S6 burst | S7 unseen |
|---|---|---|---|---|---|---|---|
| Delivered parcels / h | 13.4 [12.2, 14.5] | 11.8 [10.5, 13.2] | 11.2 [9.7, 12.8] | 13.2 [12.0, 14.4] | 8.1 [6.8, 9.5] | 11.3 [9.8, 12.8] | 12.3 [10.9, 13.7] |
| On-time ≤ 15 min / h | 12.0 [10.9, 13.1] | 9.3 [8.2, 10.4] | 8.2 [6.9, 9.4] | 11.9 [10.8, 13.0] | 7.0 [5.7, 8.4] | 9.6 [8.1, 11.1] | 11.0 [9.7, 12.3] |
| On-time ≤ 30 min / h | 13.2 [12.0, 14.4] | 11.6 [10.1, 13.0] | 10.4 [8.9, 11.9] | 13.2 [11.9, 14.4] | 7.8 [6.4, 9.2] | 11.0 [9.4, 12.6] | 12.2 [10.7, 13.6] |
| On-time ≤ 60 min / h | 13.3 [12.1, 14.5] | 11.8 [10.4, 13.2] | 11.2 [9.6, 12.7] | 13.2 [12.0, 14.4] | 8.0 [6.6, 9.4] | 11.2 [9.7, 12.7] | 12.3 [10.9, 13.7] |
| Lead time mean (s) | 408 [343, 472] | 564 [493, 634] | 700 [624, 776] | 390 [331, 449] | 538 [380, 697] | 534 [386, 681] | 380 [351, 409] |
| Lead time p95 (s) | 1271 [918, 1624] | 1561 [1273, 1850] | 1967 [1682, 2252] | 1171 [854, 1489] | 1685 [1117, 2253] | 1555 [1004, 2106] | 1073 [970, 1175] |
| **Drop-off → ready mean (s)** | 79 [66, 92] | 99 [87, 111] | 99 [87, 110] | 78 [67, 90] | 89 [77, 102] | 105 [95, 116] | 83 [73, 94] |
| Drop-off → ready p95 (s) | 230 [173, 288] | 306 [248, 364] | 294 [235, 353] | 226 [181, 270] | 273 [214, 331] | 282 [234, 329] | 238 [197, 279] |
| Recovery failure rate | 0.048 [0.030, 0.066] | 0.070 [0.047, 0.093] | 0.066 [0.041, 0.092] | 0.052 [0.032, 0.072] | 0.122 [0.093, 0.152] | 0.067 [0.048, 0.086] | 0.057 [0.035, 0.080] |
| Roof-pad conflicts / ep | 6.3 [5.0, 7.6] | 7.0 [5.3, 8.7] | 8.3 [6.0, 10.6] | 6.4 [5.1, 7.7] | 2.2 [1.2, 3.2] | 9.1 [6.8, 11.4] | 5.7 [4.4, 7.0] |
| Charger conflicts / ep | 0.0 [0.0, 0.0] | 0.0 [0.0, 0.0] | 0.0 [0.0, 0.0] | 0.0 [0.0, 0.0] | 0.0 [0.0, 0.0] | 0.0 [0.0, 0.0] | 0.0 [0.0, 0.0] |
| Waiting drone-min / ep | 7.0 [5.8, 8.2] | 8.0 [6.6, 9.4] | 10.2 [7.5, 12.8] | 6.5 [5.5, 7.5] | 3.6 [2.5, 4.7] | 6.4 [5.1, 7.8] | 5.4 [4.6, 6.3] |
| Energy Wh / parcel | 175 [157, 193] | 211 [192, 230] | 224 [203, 245] | 176 [158, 194] | 233 [206, 259] | 237 [214, 261] | 189 [169, 209] |
| Empty-return km / ep | 6.7 [5.9, 7.5] | 7.7 [6.6, 8.7] | 8.2 [6.9, 9.5] | 6.7 [5.9, 7.5] | 4.5 [3.6, 5.5] | 9.0 [7.8, 10.3] | 6.3 [5.3, 7.2] |
| Recoveries to a charger / ep | 4.7 [3.6, 5.8] | 5.8 [4.6, 7.0] | 5.9 [4.8, 7.1] | 4.4 [3.5, 5.4] | 4.2 [3.1, 5.2] | 7.0 [5.7, 8.2] | 4.3 [3.1, 5.5] |
| Fallbacks / ep | 1.1 [0.7, 1.5] | 1.1 [0.8, 1.5] | 1.0 [0.5, 1.5] | 1.2 [0.8, 1.6] | 1.0 [0.6, 1.4] | 1.3 [0.9, 1.7] | 0.8 [0.4, 1.1] |
| Decision latency p95 (ms) | 0.41 [0.35, 0.47] | 0.39 [0.36, 0.41] | 0.42 [0.39, 0.45] | 0.40 [0.38, 0.42] | 0.38 [0.35, 0.40] | 0.37 [0.35, 0.39] | 0.37 [0.35, 0.39] |
| Ready-time prediction error, MAE (s) | 30 [21, 39] | 43 [35, 51] | 43 [34, 52] | 28 [21, 36] | 34 [26, 42] | 39 [30, 48] | 32 [25, 39] |

Orders per episode ≈ 48–51 (16 per hour); crashes per episode 2.3 (S1) to 4.1 (S5);
no pod was stranded.

**What the numbers say / 結果の読み方**
- The scenarios work as intended: rush hour and pod delays lengthen lead and ready
  times; high wind doubles crashes and recovery failures and cuts deliveries ~40 %;
  bursts raise roof-pad conflicts and empty-return distance.
- B0 sends ~85–90 % of recoveries to the pod, ~4–7 per episode to a charger.
- Its ready-time estimate is off by 28–43 s on average (MAE): it underestimates
  the pod's arrival and the landing queue. That is the gap a rollout method (B4)
  should close.

**Limits to state in the paper / 論文に書くべき制約**
- **Charger conflicts are 0 in every scenario:** 4 drones and 3 stations × 3 deck
  pads never fill a station, so the charger-queue part of B0 is never exercised
  and the station outage (S4) barely differs from S1 (it removes one of three
  stations). Test 1 pad per station as the guide's pad-count ablation.
- **Fallbacks ≈ 1 per episode:** no option keeps the 15 % reserve; most of these
  end as recovery failures.
- **World assumptions (same for every method):** launch safety assumed (no canopy
  take-off crashes); repaired drones rejoin the pod after the wreck (30 min) and
  repair (2 h) time; rain off; in-flight crashes (wind, battery, collision) kept.
- The energy model is the fleet world's placeholder (0.25 % / s at hover on a
  500 Wh drone): use Wh/parcel for comparison between methods, not as a real value.

---

## 3b. Does the recovery choice matter? (2026-11-16) / 回収判断は結果を左右するか

Diagnostic: B0 against **always the pod** (`methods.py` `AlwaysPodDiagnostic`, tag
`POD_*`), paired seeds 1000-1029, all 7 scenarios. If they tie, no learned method can
beat B0 on this decision either. New world options (defaults unchanged, checked: the
default reproduces the saved B0 episodes exactly): `--pads N` (charging pads per
station), `--profile typical` (500 Wh, 25 min hover, station 2C = 30 min, pod 1C =
60 min; user choice "typical values"), `--area 1,3` (hub serves zones 1 + 3),
`--rate R` (orders/h).
診断：B0 と「常にポッド」を同じシードで比較。差が無ければ学習手法も B0 に勝てない。

| World | Delivered/h B0 / POD (S1) | Charger trips per episode (B0) | Paired differences |
|---|---|---|---|
| v1 world, 3 pads (B1-B3 s0 trained here) | 13.4 / — | 4-7 | B1-B3 all choose the pod; ties within the CIs |
| typical, 1 pad, zone 1, 16/h | 15.5 / 15.6 | 0.1-0.4 | none significant except trip counts |
| typical, 1 pad, zones 1+3, 25/h ("pressure") | 20.1 / 20.5 | 0.2-0.6 | none significant except trip counts (S7) |

Finding: in this hub design the pod dominates. A drone only needs 45 % (READY_PCT)
to be ready, it rarely drops below that on a sortie, and the pod comes to it, so the
pod is ready sooner than a charger in almost every state; zero charger conflicts
even with 1 pad. With the v1 placeholders (deck charge 0→100 % in 67 s) chargers
looked useful only because they were unrealistically fast. The pod-versus-charger
choice therefore cannot separate B0-B5 as specified; the decision must change
(section 7).

**v1 final (3 seeds, 2026-11-16 16:23; mean over 7 scenarios × 30 days, seed range):**
B0 11.63 delivered/h, ready 90 s, 5.2 charger trips; B1 12.06 (12.06-12.06), ready 109 s,
0.01 trips, 1.67 infeasible picks; B2 11.74 (11.72-11.76), ready 101 s, 1.9 trips; B3
11.74 (11.73-11.76), ready 101 s, 1.7 trips. All three seeds learn the same policy
("back to the pod"); the forecast inputs change nothing. This is the supporting v1 table.

**Meeting point (same day).** `--meet 0|1|2|best` overrides the pod rule's meeting
point at a launch (here / middle / towards the next stop / shortest flights back).
All four gave identical episodes (middle reproduced the earlier run 210/210)
because the three candidates are the same street node at every launch (74/74
checked): the pod drives to the node nearest the orders' centre, launches there,
"middle" is the node nearest that same centre, and "next" falls back to middle
because Paper 1 customers are drone-only (no locker stops). The pod has no other
work while its drones are out, so it simply waits for them. A meeting-point
decision only exists if the pod has competing work (locker / face-to-face stops,
or moving on to the next order cluster) so that "here" and "later, further on"
are different places.
合流地点：3 候補は毎回同じ地点（74/74）。ポッドに他の仕事が無く、ドローンを待つだけなので判断が存在しない。

**With pod deliveries (same day).** `--pod-share 0.4`: 40 % of orders are delivered
by the pod itself at the nearest locker (fleet-world f2f orders), so the pod drives
off while its drones are out; result files gain `drone_per_h` / `pod_per_h`
(`delivered_per_h` = both). World: typical profile, 1 pad, zones 1+3, 25 orders/h.
"Here" and "next stop" now differ at 27 of 34 launches checked. Paired against B0
with the rule's meeting point (30 seeds, tags `*_podwork_*`):

| Variant | Delivered/h S1 / S2 / S3 / S7 | Drop → ready S1 / S3 |
|---|---|---|
| B0, rule (meet where the pod launched) | 20.0 / 18.3 / 16.3 / 19.6 | 302 s / 524 s |
| always the pod | -0.1 / +0.3 / -0.2 / +1.3 | +10 s / 0 s |
| meet at the pod's next stop | +0.9 / -1.0 / -0.7 / +0.8 | -53 s / -63 s |
| per-launch shortest flights | +0.2 / -0.3 / +0.2 / -0.0 | +3 s / -47 s |

Finding: now the choice matters and **its best answer depends on the situation**:
meeting at the next stop gains about 1 delivery/h in S1, S4, S7 and loses 0.6-1.0/h
in S2, S3, S5, and cuts drop → ready by 10-75 s; ready times are 3-5 times longer
than without pod work (drones wait for a busy pod). No fixed rule wins everywhere,
which is the room a state-dependent (learned / rollout) method needs. Most single
differences are not yet significant with 30 seeds (high variance). Noise to fix
first: the pod strands in 3-17 % of episodes, because the frozen pod rule only
charges when no drone is out.
ポッド配達ありでは合流地点の選択が結果を左右し、最良の選択は状況（シナリオ）で変わる。学習・ロールアウトの余地がある。

**World v2 (adopted 2026-11-16, user go-ahead).** `--world v2` (`world.py` `WORLDS`):
the pod-deliveries world above, plus (1) the pod charging fix (`Paper1World.pod_rule`:
below 40 % with drones out no more locker trips, the pod gathers them; below 40 % with
all home it drives to the nearest charger) and (2) the meeting point inside the
recovery choice: 5 options = the pod's current meeting point, **the pod's next stop**
(moves the meeting point; drones already heading to the pod follow, paying only the
part already flown), charger 0-2. Observation +3 inputs (next stop exists, distances).
v1 is unchanged (B0 reproduces 3/3; `tests/test_paper1_b0.py` passes; v1
checkpoints still load). Paired B0 v2 against always-the-current-meeting-point
(30 seeds, tags `B0_v2`, `POD_v2`): the pod strands 1 time in 210 episodes (was 3-17 %);

| B0 − POD | S1 | S2 | S3 | S4 | S5 | S6 | S7 |
|---|---|---|---|---|---|---|---|
| delivered/h | -0.03 | +0.62 | +0.69 | -0.18 | +0.21 | +0.62 | +0.37 |
| drop → ready [s] | +18 | **-71** | **-113** | +31 | -27 | -1 | -32 |

(bold: 95 % CI excludes 0). B0 uses the next stop 4-8 times per episode; it helps in
congestion and pod delays and costs time in normal traffic: situation-dependent, so a
learned / rollout method has room. B1-B3 in v2: `research/paper1/run_v2.sh`
(starts after the v1 run; checkpoints `checkpoints/paper1/v2/<M>/s<seed>/`, tags
`<M>_s<seed>_v2`).
世界 v2 を採用：ポッド配達・充電修正・「次の停車地で合流」を含む 5 択。B0 と常にポッドの差は状況で正負が変わる。

**v2 seed 0 (2026-11-16 17:50; 7 scenarios × 30 paired days; mean over all 210, paired
difference against B0 with 95 % CI, * = excludes 0):**

| | Delivered/h | On time (30 min)/h | Drop → ready | Next-stop picks | Infeasible picks |
|---|---|---|---|---|---|
| B0 | 20.00 | 17.88 | 333 s | 6.0 | — |
| always pod | 19.67 | 17.66 | 361 s | 0 | — |
| B1 | 19.84 (-0.16) | 17.94 (+0.07) | 337 s (+4) | 8.3 | 4.4 |
| B2 | 20.27 (+0.27) | 18.38 (**+0.50***) | 319 s (-13) | 11.1 | 0 |
| B3 | 20.23 (+0.23) | 18.50 (**+0.62***) | 278 s (**-55***) | 14.7 | 0 |

B3 is significantly faster to ready in S2, S5, S7 and overall (−16 %); B2 and B3 deliver
significantly more on time overall; B1 does not beat B0 and makes 4.4 infeasible picks
per episode. One seed only: seeds 1-2 running.
v2 シード 0：B3 は準備完了まで 55 秒（16 %）短縮、B2/B3 は時間内配達が有意に増加。B1 は B0 と同等。

**v2 final, 3 seeds (2026-11-16 20:07).** Mean over 7 scenarios × 30 paired days;
learners: mean over seeds [seed range]; "vs B0" = seed-averaged policy per day minus
B0, 95 % CI (* = excludes 0). Tags `B0_v2`, `POD_v2`, `<M>_s<seed>_v2`.

| | Delivered/h | On time (30 min)/h | Drop → ready | Next-stop picks | Charger trips | Infeasible picks |
|---|---|---|---|---|---|---|
| B0 | 20.00 | 17.88 | 333 s | 6.0 | 1.9 | — |
| always pod | 19.67 | 17.66 | 361 s | 0 | 0.3 | — |
| B1 | 20.04 [19.84-20.17] | 18.15 [17.94-18.31] | 337 s [337-337] | 8.9 | 0.5 | 3.0 [0.8-4.4] |
| B2 | 20.37 [20.27-20.44] | 18.40 [18.27-18.54] | 315 s [308-319] | 10.7 | 0.5 | 0 |
| B3 | 20.22 [20.18-20.26] | 18.38 [18.27-18.50] | 297 s [278-314] | 11.9 | 1.1 | 0 |
| **B1 vs B0** | +0.04 ± 0.38 | +0.27 ± 0.38 | +4 ± 19 s | | | |
| **B2 vs B0** | +0.37 ± 0.38 | **+0.52 ± 0.38*** | −17 ± 19 s | | | |
| **B3 vs B0** | +0.22 ± 0.37 | **+0.51 ± 0.38*** | **−36 ± 18 s*** | | | |
| **B3 vs B2** | −0.15 ± 0.20 | −0.01 ± 0.20 | **−19 ± 12 s*** | | | |

Per scenario (seed-averaged, vs B0): B3 is significantly faster to ready in S1 (−47 s),
S4 (−51 s), S7 (−70 s); B2 delivers significantly more on time in S3 (+1.3/h) and is
faster in S7 (−41 s); nothing is significantly worse than B0 anywhere.

Findings: (1) the masked learners beat B0 on punctuality (+0.5 parcels/h on time,
about +3 %), consistently over seeds; (2) the forecast inputs (B3) make drones ready
significantly sooner than B2 (−19 s) and than B0 (−36 s, −11 %), but not more on time;
(3) B3 varies more between seeds (278-314 s) than B2; (4) without the mask (B1) there
is no gain and 0.8-4.4 infeasible picks per episode; (5) throughput is unchanged
(about 20/h for all). B3 is the bar for B4 (the main comparison, B4 vs B3). Next:
B5, then B4 (user, 2026-11-16: start tomorrow).
v2 最終（3 シード）：マスク付き学習手法は B0 より時間内配達が約 +0.5 件/時（有意）。B3 は準備完了が B0 より 36 秒、B2 より 19 秒短い（有意）。B1 は改善なし。次は B5、B4。

**B4 built (2026-11-17).** `rl_delivery/paper1/rollout.py` (`RolloutMethod`,
`clone_world`): at each recovery decision the world is copied (static map shared,
moving state deep-copied, arrival callbacks re-bound to the copy), each copy gets fresh
random seeds, a re-drawn next order time and no pre-drawn crash outcomes (no peeking at
the real future); B2's top-3 options are rolled out 10 min (4 samples, common random
numbers), other drones by B2, the end scored by B2's critic (γ = 0.99 per decision);
deadline 20 s → B2's choice. Checks: rollouts never change the real episode (B0 with
all B4 rollouts run = B0 exactly, 2 scenarios); B4 is reproducible; ~1.3 s per decision.
The first version (argmax of the rollout scores) chose chargers too often (B2's critic
is over-optimistic in states B2 rarely visits): B4 now **keeps B2's choice unless a
candidate is better by more than 2 standard errors** of the paired difference.
Settings were chosen on **validation days only** (seeds 2020-2029, 7 situations, 70
days; a first look at test days 0-4 was discarded as biased):

| B4 version (B2 s0) | on time/h vs B2 | drop → ready vs B2 |
|---|---|---|
| argmax, 10 min, 4 samples | −0.28 ± 0.94 | −15 ± 46 s |
| **confident override, 10 min, 4 samples (chosen)** | **+0.42 ± 0.58** | **−57 ± 35 s** |
| confident, 2 decisions, 8 samples | +0.39 ± 0.81 | −44 ± 42 s |

Full test: `research/paper1/run_b4.sh` (B2 seeds 0-2, tags `B4_s<seed>_v2`).

**B4 setting revised (2026-11-18), after the CARLA diagnosis (section 3c).** Two options in
`RolloutMethod`: `fixed_time` (every option's look-ahead ends at the same time; the critic
scores the state then, from the deciding drone's view; the world loop takes an optional
`t_stop`) and `prior_gate` (keep B2's choice without rollouts when B2 gives it ≥ p). Chosen on
the same 70 validation days (`tuning_b4_validation_v2.log`), paired against B2:

| B4 version | on time/h vs B2 | drop → ready vs B2 | chargers in the 22 CARLA situations |
|---|---|---|---|
| confident override (previous, now `B4conf`) | +0.42 ± 0.58 | −57 ± 35 s | 10 |
| + equal look-ahead | +0.44 ± 0.62 | −41 ± 37 s | 10 |
| + B2 gate 95 % | +0.09 ± 0.55 | −39 ± 33 s | 0 |
| + B2 gate 99 % | +0.15 ± 0.52 | −45 ± 34 s | – |
| **+ equal look-ahead + B2 gate 95 % (chosen)** | **+0.49 ± 0.61** | **−45 ± 36 s** | **0** |

The previous results are kept as `B4conf_s<seed>_v2`; the new B4 is re-tested under the
`B4_s<seed>_v2` tags (`run_b4_v2.log`), then compared again in CARLA.

**Revised B4, test results (2026-11-18 10:51; 3 seeds, 7 × 30 test days; seed-averaged
paired differences, * = 95 % CI excludes 0).** Per seed: on time 18.95 / 19.04 / 18.90 per h,
ready 279 / 280 / 266 s, 0.58-0.72 s per decision (B4conf: 1.36-1.40 s).

| new B4 minus | On time/h | Drop → ready | Delivered/h |
|---|---|---|---|
| **B3 (primary)** | **+0.58 ± 0.22*** | **−22 ± 13 s*** | **+0.45 ± 0.21*** |
| B2 | +0.57 ± 0.21* | −41 ± 12 s* | +0.30 ± 0.20* |
| B5 | +0.34 ± 0.32* | −41 ± 19 s* | +0.37 ± 0.33* |
| B0 | +1.09 ± 0.37* | −58 ± 19 s* | +0.67 ± 0.36* |
| B4conf (previous) | −0.22 ± 0.21* | +12 ± 11 s* | −0.15 ± 0.20 |

The revised B4 still beats every baseline on every main measure, at half the computing time,
but gives up about a quarter of the previous version's lead (−0.22 parcels/h on time, +12 s
ready vs B4conf, both significant): the price of trusting B2 when it is very sure, which is
what keeps B4 sensible in CARLA (section 3c). Report both in the paper.
B4 を実装：世界を複製して B2 の上位 3 案を 10 分先まで再生。B2 の選択を覆すのは 2 標準誤差以上良い場合だけ（検証日のみで決定）。

**B4 results, 3 seeds (2026-11-17 11:19; B4 on each B2 seed, 7 scenarios × 30 paired
test days, tags `B4_s<seed>_v2`).**

| | Delivered/h | On time/h | Drop → ready | Recovery failures | Next-stop picks | Charger trips |
|---|---|---|---|---|---|---|
| B4 s0 / s1 / s2 | 20.55 / 21.03 / 20.87 | 18.95 / 19.47 / 19.11 | 270 / 263 / 256 s | 1.9 / 1.5 / 1.6 % | 12.3 / 11.8 / 11.6 | 2.1 / 2.0 / 1.9 |

Seed-averaged paired differences (210 days, 95 % CI, * = excludes 0):

| B4 minus | On time/h | Drop → ready | Delivered/h | Failure rate |
|---|---|---|---|---|
| **B3 (primary)** | **+0.80 ± 0.23*** | **−34 ± 13 s*** | **+0.59 ± 0.23*** | −0.006 ± 0.003* |
| B2 | +0.78 ± 0.23* | −52 ± 12 s* | +0.44 ± 0.22* | −0.005 ± 0.003* |
| B0 | +1.30 ± 0.38* | −70 ± 19 s* | +0.82 ± 0.38* | −0.006 ± 0.006* |

Per seed pair, B4 beats B3 on punctuality in all three (+0.45*, +1.10*, +0.84*) and B2
on punctuality and ready time in all three. Per scenario (B4 − B3, seed-averaged):
on time +0.70* (S1), +0.83* (S2), +1.26* (S3), +0.27 (S4), +0.61 (S5), +1.36* (S6),
+0.56 (S7); ready −87 s* in S3 and −39 s* in S6; never worse. Decision time 1.4 s
(mean). Finding: decision-time lookahead on top of the learned policy beats the
forecast-input policy (B3) and its own base policy (B2) on punctuality, ready time and
throughput; gains are largest with pod delays and order bursts. Still open: B5 (is
the learned part needed), rollout calibration.
B4 の結果（3 シード）：B3 より時間内配達 +0.80 件/時、準備完了 −34 秒、配達数 +0.59 件/時（すべて有意）。ポッド遅延と注文集中で最大の効果。

**B5 (2026-11-17 12:32).** Same engine, no learned part: every feasible option is a
candidate, the other drones follow B0, no critic. Settings chosen on the same 70
validation days as B4 (10 / 20 min look-ahead × argmax / confident): argmax versions
over-use chargers (11-12 per day) and lose ready time (+73 / +102 s vs B0); chosen:
**20 min look-ahead, keep B0's choice unless 2 standard errors better** (+1.11 ± 0.70
on time/h, −45 ± 30 s vs B0 on validation). Test (7 × 30 days, tag `B5_v2`): delivered
20.30/h, on time 18.62/h, ready 316 s, failures 2.1 %, 7.8 next-stop picks, 4.1 charger
trips, 2.9 s per decision.

| B5 minus | On time/h | Drop → ready | Delivered/h |
|---|---|---|---|
| B4 (3-seed avg) | **−0.55 ± 0.30*** | **+53 ± 18 s*** | **−0.52 ± 0.32*** |
| B3 (avg) | +0.24 ± 0.34 | +19 ± 19 s | +0.07 ± 0.34 |
| B2 (avg) | +0.23 ± 0.33 | +0 ± 20 s | −0.08 ± 0.34 |
| B0 | **+0.75 ± 0.36*** | −17 ± 21 s | +0.30 ± 0.36 |

Each B4 seed is faster to ready than B5 (−46*, −53*, −60* s) and more punctual
(+0.32, +0.85*, +0.49*); per scenario B4 is never worse, with the largest margins in
S3 (+1.18*/h, −108* s) and S7 (+0.98*/h). Finding: **the learned PPO is needed** — search
with a fixed continuation beats the rule (B0) but only matches the learners without
lookahead (B2, B3); with B2's prior, continuation and critic the same search is
clearly better and needs half the compute (1.4 s vs 2.9 s per decision).
B5：学習なしの探索は B0 には勝つが B2/B3 と同等。B4 は B5 より有意に良く、計算も半分。PPO は必要。
結論：このハブ構成ではポッドが常に有利で、ポッドか充電器かの判断では手法の差が出ない。

## 3c. Transfer to CARLA: discussion point for the paper / CARLA への移行（考察）

**What was tested.** B4 (seed 0) ran inside the CARLA delivery simulation through the bridge
(`delivery/paper1_bridge.py`): OitaKosen, 3 LastMile pods × 4 drones, 90 % drone orders,
same order seed, against the simulation's own recovery rule, once with CARLA's drone battery
and once with the "typical" battery the methods learned on (section 7, item 8).

**What happened.** Throughput was about equal to the sim's rule (58.6 vs 57.4 and 60.1 vs
62.1 deliveries/h) and orders waited 2.7-3.4 min less with B4 in both comparisons; crashes and
failures were flight collisions, outside the recovery choice. But B4 sent 40-50 % of drones to
a charger (practice world: about 15 %), even drones still at ~97 % battery, while B0 and B2
replayed on the same saved situations always chose the pod.

**Why (diagnosis on 22 saved decisions).** Not the bridge: replayed offline, B4 reproduces 21
of the 22 live decisions, and calibrating the practice world to the live pods (workload,
charging rules) changes nothing. The charger choices come with low pod batteries and come
from B4 itself: CARLA's drops are minutes apart, so the look-ahead is long and uneven; the
reward prices a 30 min charge at about 0.6 on-time parcels (w_r), so freeing the pod looks
worth it; and B2's critic is unreliable in states that were rare in training.

**For the paper.** (1) Decision-time lookahead gave the largest gains in the practice world,
but it amplifies any error in the value estimate when the deployment world differs: the
rollouts are only as good as the model and the critic. (2) A learned prior is a useful
safeguard: B2 was 98.8-99.7 % sure of "pod" in every one of these CARLA situations, and
keeping its choice when it is that sure removes all of the odd charger choices (validation of
this safeguard in the practice world: section 3b). (3) Sim-to-sim transfer should be checked
with the deployment simulator before claiming field benefits; one run per arm is not enough
for firm conclusions.

**With the revised B4 (2026-11-18,** equal look-ahead + B2 gate 95 %, typical battery, same
settings, same CARLA session, `carla_ab_typical_20261118_1223.json`):

| | Delivered/h | Drone/h | Failed | Crashes | Charger trips | Order wait | Drop → landed (n) |
|---|---|---|---|---|---|---|---|
| revised B4 | **70.6** | **65.2** | 2 | 3 | 1 | **642 s** | 55 s (46) |
| sim's rule | 66.9 | 60.2 | 5 | 5 | 0 | 700 s | 52 s (45) |

B4's choices: 40 pod, 1 charger, 0 next stop, 7 late "pod" answers not used. The odd charger
choices are gone, and B4 is ahead on deliveries (+3.8/h), drone deliveries (+5.0/h), failures
and order waiting (−57 s); drop → landed is about equal and now comparable (both arms return
almost every drone). One run per arm: indicative, not conclusive.
CARLA では処理量はほぼ同等、注文の待ち時間は B4 が約 3 分短い。一方 B4 は満充電に近いドローンも充電器へ送った（B0/B2 は送らない）。原因は橋渡しではなく、先読みと評価関数の誤差。B2 が確信している時はその選択を保つと解消する。

**Repeat, seed 8 (2026-11-20,** same settings, other orders, `carla_ab_typical_20261120_1051_seed8.json`;
the planned seeds 9 and 10 were cancelled by the user):

| | Sim minutes | Delivered/h | Drone/h | Failed | Crashes | Charger trips | Order wait | Drop → landed (n) |
|---|---|---|---|---|---|---|---|---|
| revised B4 | 41.5 | **57.9** | **53.6** | 2 | 5 | 1 | **330 s** | 63 s (32) |
| sim's rule | 38.3 | 48.6 | 40.7 | 2 | 3 | 0 | 549 s | 53 s (25) |

B4 ahead again on deliveries (+9.4/h) and order wait (−219 s); more crashes (5 vs 3) and a
slower drop → landed (+10 s). Caveat: the Unreal editor was opened on the same GPU from about
09:58 (end of the B4 run) through the whole rule run; the rule run got fewer sim minutes
(38.3 vs 41.5) and may have been slowed. Two days (seeds 7, 8): B4 +3.8 and +9.4 deliveries/h,
order wait −57 and −219 s: same direction both days, still too few runs for a significance test.
seed 8 の再試行でも B4 が配達数・待ち時間で上回った（ただし規則側の走行中に UE エディタを同じ GPU で開いていた）。

## 4. Existing data (2026-11-12) and why it is not valid / 既存データと無効な理由

Produced by Google Antigravity with `rl_delivery/research/run_paper1_benchmark.py`
→ `rl_delivery/ramppo.py::evaluate_scenario_benchmark`. Files in
`rl_delivery/research/paper1_results/`: `paper1_baseline_comparison.json`,
`table1_paper1_summary.md/.tex`, `fig1_cumulative_reward.png`,
`fig2_decision_latency.png`. Episodes per cell: not stored in the file (script
default 5; Antigravity's log says 20).

### 4.1 How the numbers were made
- **State:** `CompactFleetState(current_time_s=100, wind_speed_ms=w)` with
  defaults: 4 drones **all docked at the hub, 500 Wh each, no orders**; hub at
  (0, 0) with stops (0,0)/(100,100)/(200,0); chargers station_1 (80, 50) and
  station_2 (150, −50), capacity 2, 7.5 Wh/s.
- **Decision:** one query per episode — drone_1 at a random (x, y) in ±100 m at
  30 m altitude, battery 60–150 Wh; targets hub + 2 chargers.
- **Scenarios:** S1 wind 3.0 m/s; S2–S5 wind 9.5 m/s (no other difference).
- **Scoring:** one `CompactSimulator(H=5, N=2)` rollout of the chosen action — the
  same simplified model B4/B5 plan with.
- **Planner (B4/B5):** N = 8, H = 4, γ = 0.95, β = 0.5.
- **Reward:** `r_k = 2·first_deliveries + 2·on_time − 5·missed_deadlines −
  unavailable_drone_minutes/10 − energy_Wh/100` (all delivery terms are 0 here:
  there are no orders).

### 4.2 The numbers (mean ± std of the score, feasibility, latency)

| Scenario | B0 | B1 | B2 | B3 | B4 | B5 |
|---|---|---|---|---|---|---|
| S1 Normal | 5.980 ± 0.044 | 5.990 ± 0.036 | 5.990 ± 0.036 | 5.990 ± 0.036 | 5.990 ± 0.036 | 5.991 ± 0.034 |
| S2 Surge | 5.891 ± 0.056 | 5.881 ± 0.077 | 5.881 ± 0.077 | 5.881 ± 0.077 | 5.881 ± 0.077 | 5.898 ± 0.049 |
| S3 Traffic | 5.891 ± 0.056 | 5.881 ± 0.077 | 5.881 ± 0.077 | 5.881 ± 0.077 | 5.881 ± 0.077 | 5.898 ± 0.049 |
| S4 Weather | 5.891 ± 0.056 | 5.881 ± 0.077 | 5.881 ± 0.077 | 5.881 ± 0.077 | 5.881 ± 0.077 | 5.898 ± 0.049 |
| S5 Held-out | 5.891 ± 0.056 | 5.881 ± 0.077 | 5.881 ± 0.077 | 5.881 ± 0.077 | 5.881 ± 0.077 | 5.898 ± 0.049 |
| Feasibility (all) | 100 % | 100 % | 100 % | 100 % | 100 % | 100 % |
| Latency, ms (S1) | 0.033 | 0.028 | 0.026 | 0.020 | 1.86 | 1.83 |

### 4.3 Why it cannot be used as evidence
1. B1, B2, B3 contain **no trained network** (fixed preference) → identical rows.
2. B4's prior is the same fixed preference and its V_θ is hand-written → B4 equals
   B1–B3 in every scenario.
3. **No fleet effect possible:** no orders, the other three drones idle, the
   deciding drone's position is not used inside the simulator.
4. S2–S5 differ only in wind 9.5 vs 3.0 m/s → four identical scenario rows.
5. **One decision per episode**, scored in the planner's own model (favours B4/B5).
6. No paired-seed statistics, no confidence intervals, none of the guide's
   metrics (on-time parcels/h, drop-off→ready time, pad conflicts, Wh/parcel, p95
   latency), no check of the rollout model against the execution simulator.
7. Read literally, B5 and B0 are *best* in S2–S5 and the proposed B4 never wins.

Use these files only as a smoke test of the code path, not in the paper.
/ これらはコードの動作確認としてのみ使い、論文には使わない。

---

## 5. Comparison protocol to follow / 比較の手順

### 5.1 Scenarios (from the guide) / シナリオ
Each must be a concrete parameter set (fill in before running), run on the same
paired seed list for every method; tune H, N, β on a **validation** set only.

| ID | Scenario | Parameters (set 2026-11-16, `rl_delivery/paper1/world.py` `SCENARIOS`) |
|---|---|---|
| S1 | Normal demand | 16 orders/h (the CARLA rate 47/h for 3 pods, per pod); wind up to 12 m/s mean, 15 m/s gusts |
| S2 | Rush-hour road congestion | S1 + every pod drive takes ×1.6 |
| S3 | Unexpected LMP-4 delay | S1 + 30 % of pod drives get an extra 60–300 s |
| S4 | Station outage | S1 + the station nearest the zone centre is out from 1 h to 2 h |
| S5 | High wind (within the flight envelope) | S1 with wind up to 15 m/s mean, 18 m/s gusts |
| S6 | Burst of simultaneous drone returns | S1 rate, orders arrive 6 at a time |
| S7 | Unseen customer patterns (held out) | S1 with the other half of the zone's addresses (S1–S6 use one fixed half) |

### 5.2 Reward (one definition for training and scoring) / 報酬
Guide form, per event: `r_t = w_d·(new on-time parcels) − w_l·tardiness_min −
w_r·(extra next-ready drone-minutes) − w_e·energy_Wh − w_q·queue_conflicts −
w_f·recovery_failures`, with event-time discounting `γ^(Δt/τ)` when event
intervals vary. Chosen 2026-11-16 (first values, `rl_delivery/paper1/env.py` `REWARD`; one decision
interval = from one recovery decision to the next): `w_d = 1.0` per parcel delivered
within 30 min, `w_l = 0.05` per minute later than 30 min, `w_r = 0.02` per drone-minute
not ready after its drop (any drone), `w_e = 0.002` per Wh, `w_q = 0.1` per roof-pad or
charger conflict, `w_f = 5.0` per recovery failure; γ = 0.99 per decision (no
event-time discount). Scale: one on-time parcel = 50 not-ready drone-minutes = 500 Wh.
Collisions, prohibited flights, unsafe landing and low reserve: hard constraints
(guide) — or penalties if the pure-RL rule is chosen (section 2).

### 5.3 Metrics / 評価指標
| Metric | Definition | Primary? |
|---|---|---|
| On-time parcels per hour | parcels delivered before their deadline ÷ operating hours | **primary** |
| Drop-off → ready time | from parcel drop until the drone is ready to collect its next parcel (includes recovery flight, wait, charge, empty return, reload), mean and p95 | **primary** |
| Recovery failure rate | recoveries that could not be completed (no slot, battery, mask emptied) ÷ recoveries | secondary |
| Pad / charger conflicts | double bookings or arrivals at a full resource | secondary |
| Waiting drone-minutes | time drones spend waiting for a pad/charger/pod | secondary |
| Energy per parcel | Wh ÷ parcels delivered | secondary |
| Empty return distance | km flown without a parcel to get back to stock | secondary |
| Infeasible picks (B1) | choices that violate the mask | secondary |
| Decision latency | mean and p95 wall-clock per decision; fallbacks to B0 | secondary |

### 5.4 Statistics / 統計
- ≥ 3 training seeds per learned method; ≥ 30 evaluation episodes per scenario,
  the **same seeds** for all methods (paired).
- Report mean and 95 % confidence interval (paired bootstrap of per-episode
  differences, or paired t-test), per scenario.
- Primary claim = B4 − B3 on the two primary metrics; secondary contrasts as in
  section 1.
- Ablations: H ∈ {1, 3, 5}, N ∈ {4, 8, 16}, β ∈ {0, 0.5, 1, 2}, forecast error
  (calibrated vs biased vs noisy), landing pads ∈ {1, 2}.
- Hypotheses to test (not assert): **H1** B4 reduces fleet next-ready delay under
  uncertain traffic; **H2** B4 stays safe under mask enforcement; **H3** the gain
  disappears or reverses with a badly calibrated forecast or too small a compute
  budget. Report failures (model bias, short horizon, latency, reward gaming).

### 5.5 Result files (templates) / 結果ファイル
Per episode — `rl_delivery/research/paper1/results_episodes.csv`:
```
method,scenario,train_seed,eval_seed,ontime_per_h,ready_time_mean_s,ready_time_p95_s,
recovery_failures,pad_conflicts,wait_drone_min,wh_per_parcel,empty_return_km,
infeasible_picks,latency_mean_ms,latency_p95_ms,fallbacks,decisions,sim_hours
```
Per decision — `rl_delivery/research/paper1/decisions.jsonl` (one JSON per line):
```
{"method","scenario","eval_seed","t","drone","state_summary","mask":[...],
 "mask_reasons":[...],"candidates":[...],"q_hat":[...],"prior":[...],
 "chosen","fallback":false,"latency_ms","actual_ready_s","predicted_ready_s"}
```
Summary — `rl_delivery/research/paper1/summary.csv`: method, scenario, metric,
mean, ci_low, ci_high, n. Plus a model-calibration file comparing predicted vs
actual ready time (B4/B5 rollouts vs execution).

---

## 6. Paper 1 world and parameters / Paper 1 の世界とパラメータ

**The execution world (2026-11-16):** `rl_delivery/paper1/world.py`, `Paper1World`, a
subclass of the fleet world (`rl_delivery/env_fleet.py`) on the real OitaKosen map
(roads, buildings, no-fly zones, height map, hidden wind). Scope: zone 1 (891
addresses, ~1 km × 0.6 km), one LMP-4, 4 drones (500 Wh), 3 charging stations
(3 deck pads each, 307–498 m from the zone centre), drone-only customers, no
standby drones, rain off. Added to the fleet world: charger capacity and queues,
the empty flight back from a charger, one landing at a time on the roof pad
(15 s), the scenarios and the measurements. Frozen for every method: the pod
(`fleet_rules.rule_pod`: where to stop, launch, its own charging), launch height
(medium), hover rule (back to the nearest feasible charger below 20 %).

| Setting (Paper 1 world) | Value | Source |
|---|---|---|
| Normal order rate | 16 / h | CARLA delivery sim, 47 / h for 3 pods |
| "Ready" | docked in the pod with ≥ 45 % | fleet rule's launch threshold |
| Battery reserve | 15 % on arrival, estimated energy × 1.2 | existing mask |
| Station charge target | 90 %, then fly back | chosen (deck charges at 1.5 %/s) |
| On-time thresholds | 15 / 30 / 60 min reported together | no deadline exists in the system |
| Episode | 3 h sim time, step 5 s | fleet world |
| Seeds | 1000–1029, paired across methods | — |

Parameters of the older compact model (Antigravity's code, used by its B4/B5):

| Parameter | Value | Where |
|---|---|---|
| Battery reserve | 15 Wh | `delivery/feasibility_engine.py` `FeasibilityEngine(reserve_wh=15.0)` |
| Docking duration | 20 s | `FeasibilityEngine(docking_duration_s=20.0)` |
| Policy-prior weight β | 0.5 | `rl_delivery/ramppo.py` `FARAMPPOPlanner(beta_policy_weight=0.5)` |
| Horizon H / scenarios N / γ | 4 / 8 / 0.95 | `FARAMPPOPlanner` → `CompactSimulator` |
| CRN seeds | `base_seed + n·10007` | `CompactSimulator.evaluate_candidate_action` |
| Drones | 4 × 500 Wh | `CompactFleetState` defaults |
| Hub pad | 1 (`hub_roof_pad`) | `CompactFleetState` |
| Chargers | 2, capacity 2, 7.5 Wh/s | `CompactFleetState` defaults |
| Guide starting ranges | K 2–5, N 8–16, H 3–5, decision < 1 s | guide section 05 |

These are placeholders, not calibrated values; calibrate against the delivery
sim (`delivery_sim.py`, `delivery/station.py` battery model) before the study.

---

## 7. Work still to do, in order / 残りの作業

**Paper structure (user decision 2026-11-16, may be revised):** both worlds are in
the paper. **World v2 is the main experiment** (every main table, B4 vs B3, all
methods vs B0). **World v1 is a supporting result** (one paragraph + one table,
discussion or appendix): with drone-only customers the pod just waits and every
method ties, even "always the pod", so smart recovery pays off only when the pod has
its own work. The intermediate checks of section 3b stay in this document only.
Tell the supervisor that the main experiment moved from the guide's setup to v2,
with v1 as the evidence why.
論文構成：v2 を主実験、v1 を補足結果（全手法が同等）として両方載せる（後で外してもよい）。

1. Decide hard constraints vs pure RL (section 2) — needed for B1–B5, not for B0.
2. ✅ **Execution world and harness** (`rl_delivery/paper1/`, 2026-11-16). Still
   needed for training: a Gymnasium view of `Paper1World` (observation, reward,
   `action_masks()` from `estimate()`), acting only at recovery decisions.
3. ✅ **Train B1, B2, B3** (3 seeds each, done 2026-11-16 20:07) in both worlds: v1 (checkpoints
   `rl_delivery/checkpoints/paper1/<method>/s<seed>/`, tags `<method>_s<seed>`) and v2
   (`.../paper1/v2/<method>/s<seed>/`, tags `<method>_s<seed>_v2`, `run_v2.sh`).
4. ✅ **B0** done in both worlds (tags `B0`, `B0_v2`). **B5**: a fixed continuation rule.
5. **Finish B4** (in v2, with the v2 B2): B2's π_θ and V_θ, continuation policy for the other drones, top-K,
   deadline + fallback, decision logging.
6. **Evaluation harness**: the 7 scenarios, full episodes, paired seeds, metrics
   and files of section 5, ablations.
7. **Calibrate** the compact rollout model against the execution simulator
   (predicted vs actual ready time on held-out scenarios).
8. 🟡 **CARLA validation:** the bridge is built (2026-11-17; `delivery/paper1_bridge.py`,
   `rl_delivery/paper1/snapshot.py`, launch option "Paper 1 recovery", 3 pods × 4 drones,
   docs/RL_CARLA_BRIDGE.md) and tested offline (`tests/test_paper1_bridge.py`: rebuilt
   snapshots choose like the original world 24/25 (B0), 22/25 (B2); every choice carried
   out on stand-in pods). **First live run 2026-11-17** (OitaKosen, 3 LastMile pods, B4 seed 0,
   delivery run `data/delivery/20261117_135707`): every decision logged (`paper1_decision`);
   in 20 min 3 decisions (few drone deliveries: most orders went to the pods' lockers),
   decided in 0.2 s; one "meet at the next stop" carried out end to end (the pod drove on at
   once, the drone waited at the next stop and landed on the pod there at 52 %), two "back
   to the pod"; no errors, no crashes from the bridge. A B4-vs-code comparison needs long
   runs with more drone orders (e.g. delivery args `--drone-prob 0.9`).
   **B4 vs the sim's own rule in CARLA (2026-11-17,** `rl_delivery/research/paper1/carla_ab_run.py`,
   OitaKosen, 3 pods × 4 drones, `--drone-prob 0.9 --seed 7`, 4 random stations, 45 min wall
   each; one run per arm, so differences may be chance):

   | | Delivered/h | Drone/h | Failed | Crashes | Charger trips | Order wait | Drop → landed (n) |
   |---|---|---|---|---|---|---|---|
   | CARLA battery, B4 | 58.6 | 55.5 | 0 | 0 | 23 | 679 s | 48 s (13) |
   | CARLA battery, code | 57.4 | 53.1 | 1 | 2 | 0 | 884 s | 55 s (34) |
   | typical battery, B4 | 60.1 | 55.7 | 1 | 3 | 16 | 736 s | 92 s (20) |
   | typical battery, code | 62.1 | 53.8 | 2 | 4 | 0 | 896 s | 53 s (37) |

   (files `carla_ab_carla_20261117_1555.json`, assembled by hand because the script picked the
   B4 folder twice (fixed), and `carla_ab_typical_20261117_1726.json`). Throughput about equal;
   orders wait 2.7-3.4 min less with B4 in both; B4 sends 40-50 % of drones to a charger (16-19
   of 36-38 decisions; practice world: ~15 %), also with the typical battery, so the battery
   gap is not the main cause: still to diagnose (save each snapshot, replay offline). Crashes
   are flight collisions (not controlled by the recovery choice). "Drop → landed" counts only
   drones that returned to a pod, so it is not comparable between the arms.
   **Diagnosis (2026-11-18,** run `data/delivery/20261118_091543`, typical battery, 22 decisions
   with snapshots, `research/paper1/carla_diagnose.py`): B4 chose a charger 12 times, B0 and B2
   replayed on the same snapshots never (drones were at ~97 % after the drop). Charger choices
   go with a low pod battery (median 30 % vs 54 %); replayed with the pod at 60-90 % B4 charges
   6/22. Cause, from splitting B4's scores: in the imagined futures built from CARLA's
   situations the pod drives far more than CARLA's pods do (about 30 % of its battery in
   10-15 min, CARLA about 8 %), so a returning drone keeps the pod busy and a low pod strands
   (reward -5, critic 0), while sending the drone away frees the pod (it charges or keeps
   delivering); the cost of losing the drone for a 30 min charge mostly falls beyond the
   10 min look-ahead and B2's critic, trained where this was rare, does not price it. The
   pod charging thresholds were aligned with CARLA's (`pod_low_out` / `pod_low_home` from the
   pod's PodEnergyModel; practice world unchanged): no effect on this. Fix options: calibrate
   the bridge's pod driving / energy to CARLA, or a more conservative B4 (would need re-tuning
   on validation days).
   **Correction + calibration (same day).** The bridge now passes the live pod's workload
   (`rate_per_h`: orders it received per hour, `pod_share` = 1 − drone_prob) and charging
   rules into the practice world. Measured on the 22 snapshots, the imagined pods drive
   *less* than CARLA's (4.6-4.8 km/h vs 8.2 km/h), so the "pods drive 4× more" reading above
   was wrong (large battery drops came from long rollouts and charging); the calibration does
   not change B4's choices (12 pod / 10 charger before and after). Remaining causes: drops
   are minutes apart in CARLA, so a 10 min look-ahead often runs much longer and unevenly per
   option; the reward prices a 30 min charge at only ~0.6 on-time parcels (w_r = 0.02 per
   drone-minute), so freeing the pod looks worth it; and B2's critic is uncalibrated in
   these states. These are properties of B4 / the reward, not of the bridge.

---

## 8. Files / ファイル

| File | What |
|---|---|
| `~/Downloads/Rollout_Assisted_Maskable_PPO_Oita_Paper1_Guide.docx` | the method guide (source of the definitions) |
| `rl_delivery/paper1/world.py` | **Paper 1 execution world** (`Paper1World`), scenarios, `estimate()`, measurements |
| `rl_delivery/paper1/methods.py` | recovery methods (B0 done; B1–B5 to add, same interface) |
| `rl_delivery/paper1/evaluate.py` | evaluation harness: scenarios × paired seeds → CSV / JSONL / summary |
| `rl_delivery/research/paper1/` | **results**: `results_episodes_<M>.csv`, `decisions_<M>.jsonl`, `summary_<M>.csv`, `config_<M>.json` |
| `tests/test_paper1_b0.py` | tests of the Paper 1 world and B0 |
| `rl_delivery/ramppo.py` | `FARAMPPOPlanner` (B0–B5 branches), `evaluate_scenario_benchmark` |
| `delivery/compact_simulator.py` | compact fleet state, event stepping, rollouts, reward, heuristic terminal value |
| `delivery/feasibility_engine.py` | masks: reserve, no-fly, pad capacity, stopped hub |
| `delivery/reservations.py` | pad / charger slot calendar (`PadReservationManager`) |
| `delivery/ledger.py` | energy / event ledger |
| `delivery/rl_bridge.py` | `FleetRecoveryBridge` (live CARLA use; prior not valid yet) |
| `rl_delivery/research/run_paper1_benchmark.py`, `plot_paper1_figures.py` | the 2026-11-12 runner and plots |
| `rl_delivery/research/paper1_results/` | the 2026-11-12 outputs (smoke test only) |
| `tests/test_ramppo.py`, `test_compact_simulator.py`, `test_feasibility_engine.py`, `test_ledger_reservations.py` | unit tests of the pieces |
| `docs/archive/antigravity_2026-11/` | Antigravity's plan, spec and log (claims not verified) |
