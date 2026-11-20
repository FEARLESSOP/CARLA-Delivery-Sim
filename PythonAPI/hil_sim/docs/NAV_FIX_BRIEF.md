# Brief: KPIs for the drone navigation brain, and fixing it

> **Status 2026-11-13 / 状況:** Argon answered; Claude Code built a corrected version (KPIs: `docs/RL_DELIVERY.md` section 13; fix and run `nav_v7_fix`: `rl_delivery/HANDOFF.md`, `rl_delivery/research/changes.csv`). Kept as the record of what was asked. / 回答を受けて実装済み。依頼の記録として保管。

For an AI assistant (e.g. Argon) or a person picking this up. Self-contained:
everything needed is below, including the relevant code as it is on
2026-11-13. If you can read the repository, the paths are given too
(relative to `~/Downloads/CARLA_0.9.16/PythonAPI/hil_sim/`).

日本語の要約: ドローン航法ブレイン（RecurrentPPO）が学習中に崩壊した（観測に関係なくほぼ
一定の小さな指令を出し、最初の建物に衝突する）。(A) 崩壊や不正を早期に見つける KPI を設計し、
(B) 2400 万ステップの正常なチェックポイントから学習をやり直して直すこと。ユーザーの規則
（すべて学習で、固定ルールなし・安全装置なし）を守ること。

---

## 1. The task

**A. Design KPIs for the navigation brain** that would have caught this
failure early and that show whether a fix works. Two kinds:
1. *Outcome KPIs*: is the drone doing the job (deliver, land on the pod,
   safely, efficiently)?
2. *Training-health KPIs*: is the learning itself healthy (still exploring,
   still reacting to what it senses, not exploiting the reward)? With an alarm
   threshold for each.

For every KPI give: name, exact definition / formula, the data it needs (from
the list in section 6), how often to compute it, target direction, an alarm
threshold, and what to do when it fires.

**B. Fix the brain**: diagnose (confirm or reject the hypotheses in section
4), change the world / reward / training settings, restart from the good
checkpoint, train, and show with the KPIs from A that it works.

Deliver: the KPI list (a Markdown table), the code changes (diffs), the
evaluation numbers before / after, and the entries for the research record
(section 8).

## 2. Rules you must follow (the user's, non-negotiable)

- **Nothing hardcoded.** Every behaviour is learned. Rule / scripted / PID code
  exists only as a baseline to beat, never in control. A helper may be world
  physics, an option list or a reward, never behaviour (no steering, planning,
  choosing or vetoing for the brain).
- **Pure RL, no safety layer.** No geofence for no-fly zones, no emergency
  stop. Breaches are possible; they are penalised and counted.
- **Pretend world first** (this env), CARLA only for testing later.
- Agents are **allowed to fail** in training (crashes, flat batteries).
- **Wind is realistic and hidden**: never in the observation.
- Fix stuck or exploiting brains with **world rules or rewards**, never by
  steering. Check training **every 30 minutes**.
- **Do not commit or push** git. **Do not touch the running CARLA pipeline**;
  any CARLA test needs the user's OK first.
- Exactly **one trainer per brain**. Stop with `touch rl_delivery/results/STOP_nav`
  (it saves first), never kill by process name.
- Docs and code comments are **English + Japanese**; figure text English only.

## 3. What the brain is

A delivery drone takes off from a pod roof, flies to a drop point (hover 7 m
above ground, almost stopped, = "delivered"), then flies to a meeting point and
lands on the pod roof (= "reached"/"landed"). Map: OitaKosen (a real campus +
town, buildings, trees, poles from the map meshes on a 2 m height grid; no-fly
zones). Algorithm: **RecurrentPPO** (sb3-contrib, `MlpLstmPolicy`, LSTM 192,
net [256, 256]), 20 Hz, episodes up to 480 s.

### Action (4 numbers, each -1..1) — `rl_delivery/drone_dyn.py`
roll, pitch (× 30° max tilt), thrust (`1 + a`, so 0..2× hover), yaw rate
(× 90°/s). Lagged response (attitude τ 0.15 s, thrust τ 0.10 s), quadratic
drag, wind pushes it. Battery drain per step `0.25 %/s × thrust^1.5`.

### Observation (54 numbers) — `rl_delivery/env_nav.py`, `_obs()`
In the drone's yaw frame, with sensor noise (GPS 0.3 m, velocity 0.1 m/s):
goal vector /100 (3), velocity /15 (3), acceleration /10 (3), sin roll, sin
pitch (2), horizontal goal distance /1000 and bearing /π (2), height above
ground /50, above whatever is below /20, goal height difference /30 (3), 16
range readings around it out to 40 m (3), 8 readings 5 m lower (8), distance
to no-fly zones in 8 directions /150 (8), last action (4), battery /100 (1),
phase 0 = to drop / 1 = to pod (1).

### Reward per step (current code)
```python
d = distance(goal, drone)
if d < self.d_best:                       # progress only when beating the best so far
    r += 0.3 * (self.d_best - d); self.d_best = d
# circling cost within 30 m of the goal: sideways speed round it
r -= 0.02 * |tangential v| * (1 - hd/30 + 0.3)        (if 0.5 < hd < 30)
r -= 0.005                                  # time
r -= 0.003 * (thrust - 1)**2 + 0.003 * sum((act - last_act)**2)
if clearance < 3: r -= 0.02 * (3 - clearance)          # too close to things
if height_above_ground > 60: r -= 0.002 * (agl - 60)   # needlessly high
r -= 0.02 * battery_drop                    # battery is a cost
# terminal:
crash (pod / obstacle / ground / no-fly / ceiling 150 m / battery 0):  r -= 50, end
landed on pod:  r += 100 + 20 * battery/100, end
delivered (phase 0, within 2.5 m horizontal, 3 m vertical, speed < 1.5): r += 60, phase -> 1
time-out 480 s: r -= 100, end
```
Landing = phase 1, within 1.3 m of the pod centre, < 0.6 m above the roof,
sinking < 1.5 m/s, drifting < 1.2 m/s, tilt ≤ 12°. Hitting the pod harder or
lower = crash "pod".

**Goal height in phase 1 (`_update_goal`)**: the goal's height follows a fixed
cone (cruise at max(drop ground, pod ground) + 40 m until 25 m from the pod,
then linear down to the roof). **This is a hand-written descent profile and
breaks the "nothing hardcoded" rule: remove it** (the goal should be the pod
roof; the brain learns its own descent).

### Training loop — `rl_delivery/train.py`, `train_nav`
6-12 parallel envs. Every 100 k steps: evaluate 16 fixed trips
(deterministic) at the current distance cap and wind level, write a row to
`rl_delivery/results/nav_progress.csv`, save
`rl_delivery/checkpoints/nav/step_<steps>.zip` + `.json`. Curriculum: when
landed ≥ 70 %, trip distance cap +60 m (below 200 m) or +150 m, up to 900 m;
then wind level +0.2 up to 1.0. `--resume` loads the newest checkpoint and
its distance / wind; `--dist` overrides the distance.
Settings: `n_steps 512, batch 2048, n_epochs 5, lr 3e-4, gamma 0.995,
gae_lambda 0.95, ent_coef 0.0, clip 0.2`.
**Trap:** `RecurrentPPO.load()` keeps the saved hyper-parameters; to change
`ent_coef` (or anything else) on resume, set it on the loaded model
(`model.ent_coef = ...`) before `learn()`.

Commands (from `hil_sim/`, `PY=../../carla-venv/bin/python`):
```bash
$PY -W ignore -m rl_delivery.train --brain nav --hours 8 --envs 12 --eval-every 100000 [--resume] [--dist 150]
touch rl_delivery/results/STOP_nav                    # stop (saves first)
ps -eo pid,etime,args | grep "[r]l_delivery.train"    # one trainer per brain
```
`--resume` takes the *newest* checkpoint (the broken one): to start from the
good one, move the later checkpoints aside (don't delete them) or add an
option to load a given checkpoint.

## 4. What went wrong (evidence, measured 2026-11-13)

### Training history (`nav_progress.csv`, 16 test trips per row)
| Steps | Trip cap | Delivered | Landed | Crash | Notes |
|---|---|---|---|---|---|
| ~22-24 M | 900 m | 69 % | 50 % | 50 % | the good period |
| ~30 M | 900 m | 63 % | 6 % | 94 % | after a restart with reward changes |
| ~36-45 M | 900 m | 0-50 % | 0 % | 100 % | battery crashes rise to 50-70 % |
| 45-100 M | 150 m (forced) | 0 % | 0 % | 100 % | ~55 M more steps, nobody acted |

Training was restarted (resumed) at ~28.6 M and ~34.7 M steps after another
assistant changed the reward (the old one gave `phi(d) - phi(d_last)`, a
closeness score that also *lost* reward when moving away; the new one pays
only for beating the best distance so far), added the goal-height cone, the
12° tilt limit for landing, and raised the time-out penalty from -50 to -100.

### Replays (16 trips each, deterministic)
| Checkpoint | World | Trips | Delivered | Landed | How trips end |
|---|---|---|---|---|---|
| step_023992572 (24 M) | current | 150 m | 94 % | 69 % | 11 landed, 2 obstacle, 3 pod |
| step_023992572 | current | 600 m | 75 % | 56 % | 9 landed, 5 no-fly, 1 obstacle, 1 pod |
| step_023992572 | before the changes | 150 m | 94 % | 38 % | 6 landed, 8 obstacle, 2 pod |
| step_099754348 (latest) | current | 150 m | 0 % | 0 % | 11 obstacle, 5 battery; mean 132 s |
| step_099754348 | current | 600 m | 0 % | 0 % | 8 obstacle, 7 battery, 1 no-fly |

The good checkpoint works in both world versions, so the world change is not
what broke it; the brain degraded through further training.

### What the latest brain does
Every step it outputs almost the same command, about `[0.03, 0, 0, 0.02]`,
whatever it senses: it creeps forward at ~1 m/s, rises slowly and flies into
the first building at ~4 m height (or drifts until the battery is flat).

### Policy collapse, measured
Same 300 real observations (from the good brain's flights) fed to each
checkpoint; std of the deterministic action across observations, mean |action|,
and the policy's log_std:

| Checkpoint | Action std over obs (roll, pitch, thrust, yaw) | mean \|a\| | log_std |
|---|---|---|---|
| 20 M | 0.74 0.46 0.41 0.38 | 0.53 | -0.7 .. -1.2 |
| 24 M | 0.50 0.41 0.31 0.48 | 0.38 | -0.8 .. -1.3 |
| 30.5 M | 0.37 0.34 0.34 0.41 | 0.34 | -1.1 .. -1.6 |
| 40 M | 0.22 0.29 0.36 0.45 | 0.29 | -1.4 .. -1.9 |
| 45 M | 0.19 0.40 0.23 0.12 | 0.22 | -1.8 .. -2.2 |
| 50 M | 0.09 0.10 0.05 0.04 | 0.06 | -2.2 .. -2.7 |
| 70 M | 0.02 0.05 0.03 0.01 | 0.04 | -3.5 .. -4.2 |
| 99.7 M | 0.003 0.003 0.002 0.001 | 0.01 | -5.6 .. -6.5 |

### Hypotheses to confirm or reject
1. **No exploration pressure**: `ent_coef = 0`, so log_std is free to shrink;
   once it is tiny the brain can't discover anything new.
2. **"Do nothing" is the least-bad strategy under this reward**: every step
   costs (time, thrust deviation, action change, battery), and with the
   best-so-far progress reward, moving away costs nothing and wandering earns
   nothing, so the remaining gradient mostly says "move less". Check by
   computing the expected return of "zero action" vs. the good brain on the
   same trips.
3. Restarts with reward changes on a resumed policy (value function trained on
   a different reward) may have started the slide.
4. The forced 150 m cap after the collapse did not help (nothing left to
   explore with).

## 5. Suggested fix (verify before relying on it)

1. Start from `step_023992572.zip` (keep the later checkpoints, moved aside).
2. Remove the goal-height cone (rule violation); goal in phase 1 = the pod roof.
3. Keep exploration alive: a small `ent_coef` (e.g. 0.001-0.01) and/or a floor
   on log_std; alarm when log_std or action sensitivity falls (KPIs).
4. Make sure "do nothing" never beats trying: e.g. a potential-based progress
   term again (it also penalises moving away), and per-step costs small next
   to progress. Prove it with the zero-action return check.
5. Keep the real fixes: landing measured against the pod position, goal
   bookkeeping per phase. Keep the tilt limit / -100 time-out only if a short
   A/B shows they help.
6. One change at a time where possible, a short run per change, compare on the
   same fixed test trips; then a long run with 30-minute checks.

Acceptance (suggestion; refine in your KPI design): on 900 m trips, no wind,
≥ 80 % delivered, ≥ 70 % landed, crash ≤ 20 %, no KPI alarm for 10 M steps;
then wind levels up to 1.0. Baseline to beat: the scripted pilot (19 / 20
delivered and landed on trips ≤ 200 m).

## 6. Data available for KPIs

Per test trip (`info["kpi"]` at the end of an episode): `reached` (landed),
`delivered`, `deliver_t`, `crash` (`pod` / `obstacle` / `ground` / `nofly` /
`ceiling` / `battery` / `timeout` / empty), `time_s`, `battery_used` (%),
`min_clear` (closest approach to anything, m), `start_dist`, `end_dist`.
Per step (`env.log` list when set): time, position, wind, command, clearance,
phase. From the model: the action for any observation (deterministic or
sampled), `model.policy.log_std`, the value estimate, the LSTM state;
SB3 training stats (`train/entropy_loss`, `approx_kl`, `clip_fraction`,
`explained_variance`, `value_loss`) if logged; episode rewards
(`model.ep_info_buffer`). The fixed 300-observation probe from section 4 is a
cheap, comparable "is it still reacting" measure.

Existing fleet-level drone KPIs (for consistency of names; these are for the
whole delivery sim, not this brain): `docs/DELIVERY_KPI.md` section 2
(deliveries, success rate, lead time, crashes by cause, distance per delivery,
landing battery).

## 7. Files

| File | What |
|---|---|
| `rl_delivery/env_nav.py` | the navigation world: `NavEnv`, observation, reward, termination, `ScriptedPilot` baseline |
| `rl_delivery/drone_dyn.py` | quadcopter physics, battery |
| `rl_delivery/wind.py` | hidden wind model |
| `rl_delivery/train.py` | `train_nav`, `evaluate_nav`, `new_model`, curriculum, progress CSV |
| `rl_delivery/results/nav_progress.csv` | evaluation rows during training |
| `rl_delivery/checkpoints/nav/` | `step_<n>.zip` + `.json` (eval at save time) |
| `rl_delivery/research/` | research record: `changes.csv`, `runs.csv`, `runs/<id>/config.json` |
| `docs/RL_DELIVERY.md` | the RL handbook (rules, brains, rewards, traps) |
| `delivery/rl_bridge.py` | `RLNavFlight`: the brain in CARLA (same 54-number observation) |

Python: `../../carla-venv/bin/python` (sb3-contrib, torch CPU). Set
`torch.set_num_threads(1)` in evaluation scripts.

## 8. Record what you do

Add a line to `rl_delivery/research/changes.csv` for every change
(`when_jst,brain,type,change,reason,evidence`; type = world / reward /
curriculum / algorithm / evaluation / bug), note the run in `runs.csv`, update
`docs/RL_DELIVERY.md` (sections 7, 10, 11) and `rl_delivery/HANDOFF.md`. Report
honestly what was verified and what was not, with numbers.

## Appendix: the two diagnostic checks used above

```python
# Replay: N trips of a checkpoint in NavEnv at a distance cap; counts how trips end.
import numpy as np, torch; torch.set_num_threads(1)
from sb3_contrib import RecurrentPPO
from rl_delivery.env_nav import NavEnv
m = RecurrentPPO.load(CKPT, device="cpu")
env = NavEnv("OitaKosen", wind_level=0.0, seed=4242); env.curriculum_dist = DIST
for k in range(16):
    obs, _ = env.reset(seed=4242 + k); st, start = None, np.ones((1,), bool)
    while True:
        a, st = m.predict(obs, state=st, episode_start=start, deterministic=True)
        start = np.zeros((1,), bool)
        obs, r, te, tr, info = env.step(a)
        if te or tr: print(info["kpi"]); break

# Sensitivity: collect 300 observations from the good brain's flights (every 10th step,
# trips seed 900..905, distance cap 300 m), then for each checkpoint:
A = np.array([m.predict(o, state=None, episode_start=np.ones((1,), bool), deterministic=True)[0] for o in O])
print(A.std(0), np.abs(A).mean(), m.policy.log_std.detach().numpy())
```
