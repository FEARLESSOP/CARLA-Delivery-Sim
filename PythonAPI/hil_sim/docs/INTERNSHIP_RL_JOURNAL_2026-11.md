# Reinforcement learning for an autonomous delivery fleet: November 2026

Source notes for the internship monthly journal (period 2026-11-09 to 2026-11-19).
Everything below is taken from the project's own records (`docs/RL_DELIVERY.md`,
`docs/PAPER1_BASELINES.md`, `docs/RL_CARLA_BRIDGE.md`, the result files in
`rl_delivery/research/` and the git history). Numbers are from the evaluation runs.

日本語の要約：2026 年 11 月、CARLA 上の配達シミュレーター（ポッド＋ドローン）に強化学習を導入した。
高速な練習世界で学習し、ダッシュボードで観察し、CARLA で検証した。主な成果は、ドローンの回収判断に
関する研究（Paper 1）で、先読み付きの学習手法 B4 が他の全手法を上回った（30 分以内の配達 +0.58 件/時、
準備完了まで 22 秒短縮）。CARLA でも既定ルールより配達数が多かった。ドローン航法の飛行禁止区域回避は
まだ未解決で、原因を調査した。

---

## 1. Summary

In November I added reinforcement learning (RL) to a human-in-the-loop driving and
delivery simulator built on CARLA 0.9.16. The simulator runs an autonomous last-mile
delivery fleet: LastMile pods (electric delivery vehicles) that each carry four
delivery drones. The RL agents learn in a fast "practice world" written in Python
(no CARLA needed), are watched on a web dashboard, and are then tested inside CARLA.

The main research result is **Paper 1**, a controlled comparison of six methods for one
decision: *where should a drone go after it drops a parcel?* The proposed method
(**B4**, a trained policy plus decision-time lookahead) beat every baseline in the
practice world, and also beat the simulator's own rule in a first CARLA test. A second
line of work improved the drone navigation brain and made the CARLA drones move and
look like real quadcopters.

## 2. The project and the setup

- **Simulator:** CARLA 0.9.16 (Unreal Engine 4.26) on Ubuntu 24.04, RTX 5080. The rig
  (`PythonAPI/hil_sim/`) runs the simulator, traffic, a React dashboard, camera viewers
  and a driving window for a Logitech G923 wheel.
- **Delivery simulation:** 3 zones on the OitaKosen map, one pod per zone, 4 drones per
  pod, charging stations with spare drones, parcel lockers for orders the pod delivers
  itself, no-fly zones, wind.
- **RL software:** Python 3.12, Gymnasium environments, Stable-Baselines3 and
  sb3-contrib (PPO, MaskablePPO, RecurrentPPO, SAC), PyTorch on CPU.
- **Development assistance:** I worked with AI coding assistants (Claude Code and Google
  Antigravity) for writing code, running experiments and writing documentation; the
  design decisions (what is RL, the rules, which experiments, which changes to accept)
  were mine.

## 3. What is learned by RL

| Agent ("brain") | What it decides | Algorithm | Status (2026-11-19) |
|---|---|---|---|
| Drone navigation | thrust, tilt and yaw 20 times a second: fly to the drop, lower the parcel, fly to the meeting point, land on the pod | RecurrentPPO (LSTM memory, to feel the hidden wind) | 66.8 M steps; ~56–69 % of test trips completed; no-fly-zone avoidance still open |
| Fleet: pod | where to stop, how many drones to launch, meeting point, lockers, charging | MaskablePPO | needs retraining (behaviour masks removed) |
| Fleet: drone | launch height, after the drop (meeting point or station), hover or leave | MaskablePPO | needs retraining |
| Pod driving | steering and pedals | SAC | **retired** 2026-11-16: the pod is driven by code (CARLA's BasicAgent) |
| Paper 1 recovery methods (B1–B4) | the after-drop decision only | PPO / MaskablePPO (+ rollouts for B4) | done, 3 seeds each |

**My rules for the RL** (decided during the month):
- *Pure RL* for the drones and fleet decisions: every behaviour is learned; scripted or
  rule-based code exists only as a **baseline to beat**, never in control.
- No safety layer (no geofence, no emergency stop): failures are allowed in training,
  penalised and counted.
- Practice world first, CARLA for testing. Wind is realistic and hidden from the agents.
- Changes are made to the *world* or the *reward*, never by steering the agent.
- Paper 1 is a separate study that follows its own guide (it uses feasibility masks).

## 4. Timeline

| Date | What happened |
|---|---|
| 11-09 | Practice world for the pods and drones, trainer, watch mode (replays on the dashboard), KPIs; two-leg drone flights (drop, then land on the pod) |
| 11-10 | Fleet world with pod and drone brains trained in turns; pod driving brain; research record and experiment queue |
| 11-11 – 11-13 | (Antigravity) masks and reward changes; the navigation brain **collapsed** after a reward change with exploration set to zero; my review found the claimed benchmark had no trained networks |
| 11-13 | Navigation fix: restored the potential-based progress reward, exploration floor, health alarms that stop training automatically; RL bridge to CARLA; drone loop sped up from 4 Hz to ~30 Hz |
| 11-13 | Found a units bug: speed limits read as km/h were m/s (×3.6), data patched |
| 11-16 | Decided the RL scope (drones + fleet decisions; pod driving by code); removed the behaviour masks from the fleet world; started Paper 1: B0 rule baseline, B1–B3 trained (3 seeds) |
| 11-16 | Paper 1 world v1 showed no method could win (the recovery choice did not matter); designed world v2 where it matters |
| 11-17 | B4 (proposed) and B5 built, tuned on validation days, tested; results page on the dashboard; CARLA bridge for the Paper 1 methods; first live CARLA runs |
| 11-17 – 11-18 | CARLA comparisons B4 vs the simulator's rule; diagnosis of why the first B4 over-used chargers; revised B4 |
| 11-18 – 11-19 | RL drone flight inside CARLA's LastMile missions; navigation retraining for no-fly zones (three runs); realistic drone motion and camera behaviour in CARLA |

## 5. Paper 1: where should a drone go after its drop?

### 5.1 The question
After a drone drops its parcel it can (a) fly back to the pod's meeting point, (b) meet the
pod at its next stop, or (c) recharge at one of three stations. The choice affects how soon
the drone is ready again and how many parcels arrive on time. The guide's main question:
**does decision-time lookahead (rollouts) improve a learned policy?**

### 5.2 The six methods
| | Method | Learned? | Lookahead? |
|---|---|---|---|
| B0 | rule: the feasible option with the shortest estimated time to ready | no | no |
| B1 | PPO without the feasibility mask | yes | no |
| B2 | MaskablePPO (infeasible options masked) | yes | no |
| B3 | MaskablePPO + forecast inputs (per-option estimates) | yes | no |
| **B4** | **proposed:** B2's policy + imagining each option 10 min ahead (B2 runs the other drones, B2's critic scores the end) | yes | yes |
| B5 | the same imagining with no network (B0's rule inside) | no | yes |

### 5.3 Protocol
- Practice world with 7 situations (normal, rush hour, pod delays, station outage, high
  wind, order bursts, unseen customers), **30 paired test days** each, the same days for
  every method; learned methods trained with **3 seeds**.
- Separate **validation days** for every tuning decision (never the test days).
- Paired differences with 95 % confidence intervals.

### 5.4 The world had to be redesigned
In the first world (**v1**: drone-only customers, the pod waits for its drones) every
method tied, even "always fly back to the pod". I tested why step by step (one charging pad,
realistic batteries, a busier area, fixed meeting points) and found the decision only
matters when **the pod has its own work**. World **v2**: the pod delivers 40 % of orders
itself at lockers, two zones, 25 orders/h, typical drone hardware (25 min flight, 30 min
fast charge), one pad per station, and a "meet at the pod's next stop" option. v2 is the
main experiment; v1 is a supporting result.

### 5.5 Results (world v2, 3 seeds × 7 situations × 30 days)
| B4 minus | On time (≤30 min) per hour | Drone ready again | Delivered per hour |
|---|---|---|---|
| **B3 (main comparison)** | **+0.58 ± 0.22** | **22 s sooner** | +0.45 |
| B2 | +0.57 | 41 s sooner | +0.30 |
| B5 | +0.34 | 41 s sooner | +0.37 |
| B0 | +1.09 | 58 s sooner | +0.67 |

All differences are significant (the 95 % interval excludes zero). Findings:
1. Lookahead improves the learned policy (B4 > B3).
2. The learned network is needed: the same lookahead without it (B5) only matches the
   learners without lookahead, at twice the computing time.
3. Feasibility masks matter: without them (B1) the policy makes 1–4 impossible choices per
   day and gains nothing.
4. In v1 no method helps: smart recovery pays off only when the pod has its own work.

### 5.6 Revising B4 after CARLA
The first B4 had a bigger lead (+0.80 on time per hour vs B3) but behaved oddly in CARLA
(see 6.2). The fix, chosen on validation days only: every option's lookahead ends at the
same time, and B2's choice is kept when B2 is at least 95 % sure. The revised B4 keeps a
significant lead over every baseline, gives up about a quarter of the first version's lead,
and needs half the computing time (0.6–0.7 s per decision instead of 1.4 s). Both versions
are reported.

## 6. Bringing RL into CARLA

### 6.1 The bridge
At each drone drop in CARLA, the bridge (`delivery/paper1_bridge.py`) takes a snapshot of
that pod (pod, 4 drones, orders, meeting point, next stop, wind), rebuilds the situation in
the practice world in a separate process, asks the chosen method, and carries out the answer
one tick later (move the meeting point, or reserve a charging pad). Tested offline: rebuilt
snapshots give the same decision as the original world in 24/25 (B0) and 22/25 (B2) cases.
Selectable on the dashboard's Launch page ("Drone after drop: code or AI").

### 6.2 CARLA results (3 LastMile pods × 4 drones, 90 % drone orders, ~44 min each, one run per arm)
| Drone battery | Method | Deliveries/h | Failed | Crashes | Charger trips | Order wait |
|---|---|---|---|---|---|---|
| CARLA's own | first B4 | 58.6 | 0 | 0 | 23 | 679 s |
| CARLA's own | simulator rule | 57.4 | 1 | 2 | 0 | 884 s |
| typical | first B4 | 60.1 | 1 | 3 | 16 | 736 s |
| typical | simulator rule | 62.1 | 2 | 4 | 0 | 896 s |
| typical | **revised B4** | **70.6** | **2** | **3** | **1** | **642 s** |
| typical | simulator rule (same session) | 66.9 | 5 | 5 | 0 | 700 s |

**Diagnosis of the first B4's charger choices** (22 decisions saved with their situation):
it sent drones still at ~97 % battery to chargers, while B0 and B2 on the same situations
never did. Ruled out: the bridge (offline replay reproduced 21/22 decisions), pod charging
rules and workload. Causes: CARLA's decisions are minutes apart, so the lookahead ran long
and unevenly; the reward priced a 30-minute charge at about 0.6 on-time parcels; and B2's
critic was unreliable in situations rare in training. The revised B4 fixed it (1 charger
trip). Discussion point for the paper: lookahead amplifies errors in the value estimate
when the deployment world differs; a learned prior is a useful safeguard.

### 6.3 The navigation brain in CARLA
I connected the RL flight to the LastMile pods' missions: a scripted vertical take-off
(clear of trees and walls next to the pod), the brain flies to the drop and to the meeting
point, and waiting and landing on the pod stay scripted. On 30 real CARLA trips about half
ended in a **no-fly zone**: a zone lies between pod 1 and its drops and the brain flies
straight. Three retraining attempts:
1. **More practice crossing zones** (half the trips): no change (crossing trips: 19/32 into
   the zone vs 20/32 before). Reason: the reward measured progress as straight-line
   distance, so going round a zone looked like losing ground.
2. **Progress measured round the zones** (shortest legal path): the brain started diving
   onto the pod and training stopped itself after 1 hour (health alarms). Analysis showed
   the new distance **jumped** up to 63 m in one step (a bug at the zone margins), making
   the reward noisy. Fixed and tested (now ≤ 1.9 m per step).
3. **Fixed version, 4 hours:** healthy training, landings back to normal, but still 23/32
   crossing trips into the zone. Likely limit: the brain sees zones only through 8 distance
   sensors and its goal as a straight-line direction, so it cannot tell which way round a
   200 m zone is shorter. Open question: give it route information as an input.

### 6.4 Realistic drone motion and cameras
The scripted drones started and stopped instantly and snapped to new headings. I changed
the flight logic to accelerate and brake (3 m/s² horizontal, 2 m/s² vertical, braking to
stop exactly on the target) and added a display layer: limited yaw rate (90°/s), pitch and
roll from the motion (forward lean ~15° in cruise, nose up when braking, banking sideways).
The downward delivery camera always points straight down, the chase and orbit views stay
level, and the camera viewers show a tilt indicator.

## 7. Tools I built for watching and analysing

- **Dashboard AI training page:** trainer status, learning curves, health alarms,
  replays of any checkpoint.
- **Paper 1 replay tab:** two methods side by side on the same practice day on the 2D map.
- **Paper 1 results page** (`/paper1`, opens in a new tab): every result with cursor-
  responsive charts, practice world and CARLA tests kept separate, CSV download; data read
  locally from the result files.
- **Automatic health checks** for training (stop on collapse), hourly logs, and
  before/after comparison scripts that run when training ends.

## 8. Lessons learned

1. **Reward hacking is normal:** the agents climbed to 1,770 m, hovered until the time
   limit, circled the target, idled until stranded and crawled. Each was fixed with a world
   rule or a cost, never with scripted behaviour.
2. **Doing nothing** appears whenever failure costs outweigh early progress; time-outs and
   idling must cost as much as a failure.
3. **Potential-based shaping** (reward for getting closer that is taken back when moving
   away) stopped circling and hovering.
4. **Curriculum** (short trips first) was needed for navigation.
5. **Success criteria matter:** an over-strict landing rule gave 0 % landings for 2.6 M
   steps; a realistic one gave 56 % within about 1 M steps.
6. **An experiment must be able to show a difference:** in Paper 1's first world every
   method tied, because the decision did not matter. Diagnostic baselines ("always the
   pod") revealed this early.
7. **Tune on validation days only**, never on test days, or the comparison is biased.
8. **Check the reward signal itself:** a reward that looked correct had 60 m jumps that
   broke training; measuring it per step found the bug.
9. **Environment bugs look like learning failures** (a broken trip picker, a battery draw
   100× too high, a units bug): replays and traces find them, curves do not.
10. **Sim-to-sim transfer needs testing:** a method that is best in the practice world can
    behave oddly in CARLA; diagnosing with saved situations and offline replay was decisive.
11. **Report honestly:** several attempts failed (navigation detours, the first B4 in
    CARLA); the failures and their causes are part of the result.

## 9. Next steps

- Paper 1: repeat the CARLA comparison (3 runs per method), check the rollouts' predicted
  ready times against reality, write the paper.
- Navigation: decide whether route information may be an input, then retrain for no-fly
  zones; reduce obstacle crashes.
- Fleet brains: retrain without the removed behaviour masks.
- Later: Mini Pods and porter robots, other maps.

## 10. Where things are

| What | File |
|---|---|
| RL handbook (rules, brains, rewards, findings) | `docs/RL_DELIVERY.md` |
| Paper 1 (setup, every result, tuning, CARLA discussion) | `docs/PAPER1_BASELINES.md` |
| CARLA bridge | `docs/RL_CARLA_BRIDGE.md`, `delivery/paper1_bridge.py` |
| Paper 1 code | `rl_delivery/paper1/` (world, methods, rollout, evaluation, snapshot) |
| Result files | `rl_delivery/research/paper1/` |
| Navigation training | `rl_delivery/env_nav.py`, `rl_delivery/train.py`, `rl_delivery/results/` |
| Drone motion and cameras | `delivery/drone_motion.py`, `delivery/drone.py`, `drone_window.py`, `drone_viewer.py`, `drone_hud.py` |
