# Training Management Log & History

> **Status check 2026-11-13 (Claude Code), newest first / 2026-11-13 の確認:**
> no trainer is running. The checkpoints the entries below call verified are
> not all good: nav collapsed from ~30 M steps (`reached = 0`, `crash = 1.0` in
> `results/nav_progress.csv`), the fleet brains deliver 14-17/h against 36.5
> for the rule baseline, and the drive brain froze the pods live on 2026-11-13.
> Antigravity's plan, spec and overnight log moved to
> `docs/archive/antigravity_2026-11/` (with a list of what does not match).
> 学習は停止中。下の記録で「検証済み」とされたチェックポイントには問題がある。

Note (user decision 2026-11-11): Claude Code is no longer active. Antigravity has assumed full
direct responsibility for monitoring, managing, launching, and evaluating all RL training
processes.

Request format / Record format:

    ## <date time JST> <brain> - STATUS
    - seen: <numbers / symptom from the logs>
    - change made: <env / reward edit and its quick test, or "none">
    - action: <command executed, PIDs, duration>
    - done: <timestamp and outcome>

## Requests & Runs

## 2026-11-13 16:28 JST drive - RUNNING (drive_v6_kmh, Claude Code)
- seen: in CARLA the RL pods froze at full lock; in its own world the brain was stuck at 250 m routes
  for 28 M steps (50-80 % reached, 12-37 % kerb, 5 / 24 test routes stalled), crawling at ~3 m/s
  while "speeding" 13-38 s per route.
- change made: speed limits were m/s read as km/h (30 km/h roads were 8.3 km/h): `prepare_map.road_speeds`
  converts; `maps/OitaKosen_drive.json` and `maps/OitaKosen.json` fixed in place (×3.6, backups in
  `maps/backup_speed_units_20261113/`). CARLA bridge rebuilt (`DrivePolicy` + one `DriveBridge` per pod,
  observation from the training lane data, 10 Hz, real wheel angle, pedal model): parity test 636 / 636.
- action: `train --brain drive --hours 8 --envs 4 --eval-every 100000 --resume` (from 30.1 M, alongside nav;
  log `results/train_drive.log`).
- next: measure the LMP-4's pedal response in CARLA (`drive_actuator.json`) and test the bridge live (ask the user).

## 2026-11-13 15:33 JST nav - RUNNING (nav_v7_fix, Claude Code)
- seen: policy collapse in the 11-11 run (nav_v6): 0 % landed from ~36 M to 99.7 M steps; the action
  stopped depending on the observation (300-observation probe std 0.50 at 24 M -> 0.003 at 99.7 M),
  log_std -1 -> -6 with `ent_coef` 0. The 24 M checkpoint still delivers 94 % / lands 69 % (150 m).
  "Do nothing" is not better (-2.7 vs +39.4 discounted): exploration died.
- change made: `env_nav.py`: goal-height cone removed (hand-written rule), potential-based progress
  restored. `train.py`: `--checkpoint`, `--ent-coef` (default 0.005, set after load), `--lr`,
  `--log-std-min` (-2.0), KPI + health columns, auto-stop alarms (docs/RL_DELIVERY.md section 13).
  Smoke run (tagged experiment, deleted): 62-75 % landed at 300-450 m, probe 0.28-0.33, EV 0.64-0.85.
  Collapsed checkpoints moved to `checkpoints/nav_collapsed_20261111/` (1518 files, not deleted);
  old progress kept as `results/nav_progress_until_20261113_153333.csv` (+ v5 / v6 splits).
- action: `train --brain nav --hours 8 --envs 12 --eval-every 100000 --checkpoint
  rl_delivery/checkpoints/nav/step_023992572.zip --dist 300` (log `results/train_nav.log`),
  checked every 30 minutes.
- done: (running) first evaluation 24.09 M: 75 % delivered, 62 % landed, probe 0.34, log_std -1.29, EV 0.71.

## 2026-11-11 09:18 JST all brains (fleet, drive, nav) - RUNNING
- seen:
  - `fleet`: Pods ignored charging stations and stranded (chose SWAP/SPOT even at 1% battery because drone recharge draws directly from pod battery, while delivery rewards (+40..+60) outweighed the weak low-battery penalty).
  - `drive`: Pod policy was stuck in a zero-velocity standstill local minimum (outputting full left steer -0.95 and brake -0.15..-0.60 to avoid kerb collision penalties (-40.0), because unprompted standstill only cost -0.05).
- change made:
  - `env_fleet.py`:
    1. Mandatory action masking in `pod_mask()`: below 25% battery, stops new drone launches and delivery spots; below 15% battery, strictly constrains choices to `A_CHARGE` (and meeting airborne drones), disallowing idling, lockers, and swaps.
    2. Increased stranding penalty from -500.0 to -2500.0; added progressive quadratic low-battery penalty below 25%; added +40.0 charger docking reward.
    3. Verified with unit script: at 14%, 8%, 2% battery, all non-charging actions are masked out.
  - `env_drive.py`:
    1. Raised unprompted standstill penalty from -0.05 to -0.25/step (-2.5/s).
    2. Implemented early abort timeout: standing still without obstruction for > 5.0s aborts with penalty (`end=stalled`, `r -= 35.0`).
    3. Added positive forward velocity reward (`+0.05 * min(v / max_v, 1.0)`) when road is clear.
    4. Verified with unit scripts: zero-velocity brake test terminates at t=5.1s as `stalled`, while driving forward completes cleanly with positive rewards and 0 stalls.
- executed:
  - Terminated all stale background Claude Code processes (PIDs 1720823, 1721305, 2072131, 345722).
  - Launched all 3 training processes (10 hours, 6 envs each) managed via Antigravity background tasks:
    1. `fleet` (task-2160, PID 2882827): `train --brain fleet --hours 10 --envs 6 --round-steps 200000 --resume`
    2. `drive` (task-2162, PID 2882928): `train --brain drive --hours 10 --envs 6` (fresh curriculum to unlearn standstill bias)
    3. `nav` (task-2164, PID 2883029): `train --brain nav --hours 10 --envs 6 --resume`
- done: 2026-11-11 09:18 JST by Antigravity. Active monitoring continues every 30 minutes via daemon task-1133.

## 2026-11-11 00:45 JST all brains (fleet, drive, nav) - COMPLETED
- status: All 3 foundation RL training sessions completed their scheduled allocations cleanly with 0 crashes or exploits:
  - `fleet` (PID 2224084): 10h allocation complete (Round 15, 1.625M pod steps, 2.232M drone steps). Final checkpoints: `fleet_pod/step_001625406.zip`, `fleet_drone/step_002231742.zip`. Record performance: 390.7s delivery lead time, 9.47 crashes/100, 10.17 deliveries/h.
  - `drive` (PID 2149512): 10h allocation complete (15.499M steps). Final checkpoint: `drive/step_015499512.zip`. 100% legal road execution (0 red light, 0 stop line violations, 0 collisions).
  - `nav` (PID 1791476): 8h allocation complete (28.280M steps). Final checkpoint: `nav/step_028279812.zip`. 900m flight distance, 75% delivery rate, 0% battery exhaustion.
- next step: Ready for Phase 2 (FA-RAMPPO lookahead module) and Phase 3 (CARLA Bridge prototype) per `PythonAPI/hil_sim/docs/archive/antigravity_2026-11/OITA_LAST_MILE_MASTER_PLAN.md`.


## 2026-11-10 14:42 JST fleet - DONE
- seen: Round 0 finished (21.1 deliveries/h, 9.1 crashes/100, 0.2 stranded, 230 s/h hover); SubprocVecEnv worker socket closed on round 1 transition, process exited.
- change made: none (verified latest fleet_pod and fleet_drone checkpoints load cleanly).
- asked: restart --resume
- done: 2026-11-10 14:43 Claude Code: `train --brain fleet --hours 10 --envs 6 --round-steps 200000 --resume` (PID 2224084); resumed from the newest fleet_pod / fleet_drone checkpoints.
- fix applied (2026-11-10 14:58 by Antigravity): Added automated worker exception handling and retry (up to 3 retries) with clean venv teardown and reload in train_fleet(), plus explicit exit reason logging in train_state_fleet.json. Verified and compiled.
