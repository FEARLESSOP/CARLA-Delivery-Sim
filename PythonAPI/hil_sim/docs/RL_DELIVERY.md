# RL for the LastMile pods and drones (`rl_delivery/`)

The handbook for this work, for any agent (Claude Code, Google Antigravity) or
person picking it up. Read it fully before touching `rl_delivery/`, the AI
training page or a running trainer. KPIs: [DELIVERY_KPI.md](DELIVERY_KPI.md). Getting the
brains into CARLA: [RL_CARLA_BRIDGE.md](RL_CARLA_BRIDGE.md).
State as of 2026-11-10 13:30 JST (see section 11 for the latest numbers).

## 1. What is being built

Reinforcement learning for the delivery fleet on the OitaKosen map: pods and
drones learn **every** behaviour themselves in a fast pretend world (no CARLA
needed), are watched on the dashboard, and are later tested in CARLA.

The user's delivery flow (the world's rules, not scripts):
1. Three zones, **one pod per zone**, **4 drones per pod**. The pod carries all
   parcels of its zone.
2. **Charging stations** (LastMile charge docks) charge pods (pod bay, 50 kW)
   and hold **standby drones** (3-pad drone deck + overflow pads).
3. Two order types: **face-to-face (F2F)** = the pod drops the parcel in a
   **parcel locker**; **drone** = a drone flies it.
4. Drone round: the pod finds a stop near the middle of the current drone
   deliveries, **under open sky** (a tree above can crash a drone at take-off),
   stops and launches as many drones as it decides (one parcel each).
5. While drones fly, the pod drives to lockers for F2F drops (one or more),
   then to the **meeting point**, timing it so hovering drones' batteries last.
6. Drones finish their drop, fly to the meeting point and **hover until the
   pod is there**, land in it and charge from the pod's battery.
7. **Standby swap**: a drone too low now, or that won't be charged enough in
   time for its next trip, is replaced by a standby drone flying out from a
   station; the tired one flies to the station and becomes a standby.
8. **Crash before the drop**: a standby drone flies to the crash site, picks
   the parcel up and delivers it. **Crash after the drop**: a standby replaces
   it. A human collects wrecks after 30 min; after a 2 h repair they are
   standbys again.
9. Pods are EVs: driving, idling and charging drones drain the 10 kWh battery;
   a flat pod is stranded. Pods must fit charging into the work.
10. Pods only enter other zones to pass through to their own or to a charger.
11. Pods must not mount kerbs and must obey the road (止まれ stop signs,
    signals, speed limits). Drones must find their own best path and never
    hit buildings or trees, and avoid no-fly zones.

## 2. The user's rules for the RL (non-negotiable)

- **What is RL (user decision 2026-11-16):** pure RL = **the drones** (flying and
  path finding, `env_nav.py`) and **the whole fleet system** (`env_fleet.py`: every pod
  and drone decision — where to stop, launches, meeting points, charging, drone
  recovery). **Only the pod's driving is code** (CARLA's BasicAgent with the Japan
  rules, `--policy-drive off`); the pod-driving RL brain (`env_drive.py`,
  `checkpoints/drive`) is retired — do not train it again. Paper 1 (a separate
  study, `docs/PAPER1_BASELINES.md`) follows its guide, hard masks included.
  ユーザー決定 2026-11-16：純粋 RL はドローン（飛行・経路）と艦隊の全判断。ポッドの運転だけ
  コード（BasicAgent）で、運転 RL は廃止。Paper 1 はガイドどおり（ハードマスクあり）。
- 2026-11-16: the behaviour masks added to `env_fleet.py` on 11-11 were removed
  (charging blocked above 50 %, swap cool-down / cap, no launches below 25 %, only
  charging below 15 %, the drone after-drop reserve mask, which also crashed on a
  missing field). The fleet checkpoints were trained with them: retrain before use.
- **Nothing hardcoded** inside the RL parts: their behaviour is learned. Rule-based /
  scripted / PID code exists there **only as baselines to beat**, never in control.
  Before adding any helper that steers, plans, chooses or vetoes, check it is
  world physics, an option list or a reward, not behaviour.
- **Pure RL, no safety layer**: no geofence for no-fly zones, no emergency stop
  for the pod. Breaches are possible; they are penalised and counted.
- **Pretend world first**, CARLA for testing.
- The agents must be **allowed to fail** in training (flat batteries, crashes,
  strandings): that is how they learn battery management.
- **Wind is realistic and hidden**: no brain ever sees it.
- Train on **OitaKosen** first; Mini Pods and porter bots come later.
- The user wants: a check of all trainings **every 30 minutes** (tweak if
  stuck or exploiting), pictures / replays of progress, plain-English reports.

## 3. The brains (agents)

| Brain | Decides | World | Algorithm | Checkpoints |
|---|---|---|---|---|
| **Pod (fleet)** | where to stop for a drone round (6 spots round the middle of the drone deliveries, with how open the sky is), how many drones to launch (1–4) + which meeting point (3), which locker (4), go to the meeting point, call a standby swap, go charge (3 stations), wait | `env_fleet.py` PodView | MaskablePPO (masked discrete, 28 actions) | `checkpoints/fleet_pod` |
| **Drone (fleet)** | cruise height at launch, after the drop: meeting point or home to a station, while hovering: keep waiting or go home, standby: accept or decline a task | `env_fleet.py` DroneView | MaskablePPO (Discrete 3, masked per decision) | `checkpoints/fleet_drone` |
| Pod driving (retired 2026-11-16: the pod is driven by code) | steering + pedal 10×/s | `env_drive.py` | SAC | `checkpoints/drive` |
| **Drone navigation** | thrust / tilt / yaw 20×/s; finds its own way to the drop, then to the meeting point and lands on the pod | `env_nav.py` | RecurrentPPO (LSTM, to feel the hidden wind) | `checkpoints/nav` |
| Drone mission (retired) | old one-pod world, replaced by the fleet brains | `env_mission.py` | | `checkpoints/mission` |
| Drone flight (retired) | followed a given route; replaced by navigation | `env_flight.py` | | `checkpoints/flight` |

A navigation flight has two legs, like the delivery sim: every flight is new
(random take-off road in a pod zone, random building out of 1,204, random
meeting point near the drop in that zone, new wind; tests use unseen seeds).
1. Pod roof -> drop: hover 4–10 m above the ground within 2.5 m, slower than
   1.5 m/s = parcel lowered (+60).
2. Drop -> meeting point: land on the pod roof (2.6 m above the road) within
   1.3 m of its centre, sinking < 1.5 m/s, drifting < 1.2 m/s (+100 + battery
   bonus); any other touch of the pod = crash "pod". Time limit 480 s.
Touching anything else, entering a no-fly zone, the 150 m ceiling, a flat
battery and the time limit all end the flight as failures.

The pod and drone fleet brains train **in turns** (`train.py --brain fleet`):
each round trains one for `--round-steps` against the other's newest checkpoint
(the rules at first). One brain per type is shared by every unit of that type.

Joining is still to do: inside the fleet world the pod's driving and the
drones' flying are quick stand-ins (road-graph travel time; a flight estimate
from length, height, hidden wind, rain and the height map). Next: the driving
brain drives and the navigation brain flies inside the fleet world, then a
top-up training, then CARLA.

## 4. Files

All under `~/Downloads/CARLA_0.9.16/PythonAPI/hil_sim/rl_delivery/` (run as
`python3 -m rl_delivery.<module>` from `hil_sim`, carla-venv python):

| File | What |
|---|---|
| `prepare_map.py` | road graph (offline `carla.Map` from the xodr), buildings, zones -> `maps/<Map>.json` |
| `prepare_heights.py` | 2 m height map from the map builder's meshes: ground + top of buildings, trees, poles -> `maps/<Map>_heights.npz` |
| `prepare_sites.py` | charging stations + lockers from the last live run (`data/delivery_live/map.json`) -> `maps/<Map>_sites.json` |
| `prepare_drive.py` | lane-level data every 2 m (width, edge kind kerb / lane / oncoming found geometrically), 止まれ lines, signals grouped by junction -> `maps/<Map>_drive.json` |
| `wind.py` | realistic hidden wind: log profile, Dryden turbulence, 1-cosine gusts, building wake |
| `drone_dyn.py` | quadcopter physics (thrust, tilt lag, drag, battery) |
| `env_fleet.py` | the fleet world (+ PodView / DroneView) |
| `fleet_rules.py` | rule version of the flow (baseline only) |
| `env_drive.py` | pod driving world (+ `ScriptedDriver` baseline) |
| `env_nav.py` | drone navigation world (+ `ScriptedPilot` baseline) |
| `train.py` | trainer: `--brain fleet / drive / nav` (+ old mission / flight), curricula, checkpoints, progress CSVs |
| `watch.py` | records one episode of any brain for the dashboard's watch mode |
| `baseline.py` | today's rule fleet in a headless CARLA (5 seeds) -> `results/baseline_OitaKosen.json` |

Git-ignored (generated): `checkpoints/`, `results/`, `maps/`. Rebuild maps
with the prepare_* commands (sites need one live delivery_sim run with
`--charge-stations N`).

## 5. Running and watching

```bash
cd ~/Downloads/CARLA_0.9.16/PythonAPI/hil_sim
PY=../../carla-venv/bin/python
$PY -m rl_delivery.prepare_map --map OitaKosen        # after a map rebuild
$PY -m rl_delivery.prepare_heights --map OitaKosen
$PY -m rl_delivery.prepare_sites --map OitaKosen
$PY -m rl_delivery.prepare_drive --map OitaKosen
$PY -W ignore -m rl_delivery.train --brain fleet --hours 10 --envs 6 --round-steps 200000
$PY -W ignore -m rl_delivery.train --brain drive --hours 10 --envs 6 --eval-every 100000
$PY -W ignore -m rl_delivery.train --brain nav --hours 8 --envs 12 --eval-every 100000
$PY -W ignore -m rl_delivery.train --brain nav ... --resume   # carry on from the newest checkpoint
touch rl_delivery/results/STOP_<brain>                     # stop (saves first): STOP_fleet / STOP_drive / STOP_nav
$PY -W ignore -m rl_delivery.watch --brain fleet --checkpoint latest --drone-checkpoint latest --seed 7
```

Logs: `results/train_<brain>.log`, progress `results/<brain>_progress.csv`,
status `results/train_state_<brain>.json`. Dashboard (http://localhost:5000)
**AI training** page: a tab per brain, Start / Resume / Stop, learning chart
with the baseline to beat, and **watch mode** (Record & play any checkpoint or
baseline): fleet = whole-map hour with pods, drones, standbys, parcels, events;
driving = route with kerbs (red), centre line (yellow), stop lines, lights,
cars; navigation = height map, trail, side view. Routes: `/rl/state`,
`/rl/train`, `/rl/watch` in `dashboard.py`.

Run trainers with the Bash tool's background mode (not `nohup ... &`), keep
**exactly one trainer per brain** (`ps -eo pid,etime,args | grep "[r]l_delivery.train"`),
kill duplicates by PID only. Trainers die when the agent session ends:
check after a restart and resume.

### Research record and experiments (for the paper)

`rl_delivery/research/` (git-tracked): `runs.csv` (every run), `changes.csv`
(every change with reason and evidence), progress curves, baselines; every new
run records its config (constants, git commit) automatically. Experiments:
`train.py --algo --seed --tag`, the queue `research/queue.csv` run by
`queue_runner.py` (service `systemctl --user start rl-queue`), comparison
tables / charts with `compare.py`. Details: `rl_delivery/research/README.md`.

## 6. Baselines (scores to beat)

| What | Numbers |
|---|---|
| Today's rule fleet in CARLA (5 seeds × 30 min, 3 pods) | pods 23.6 deliveries/h, lead 10.3 min; drones 23.6/h, 5.2 crashes / 100 flights, lowest landing 66 % |
| Rule flow in the fleet world (`fleet_rules.py`, `results/fleet_rule_baseline.json`) | 41.5 deliveries/h, 15.3 crashes / 100, 0 pods stranded, ~830 s/h drones hovering |
| Scripted driver | 31 / 40 routes reached, 0 stop / red violations, 3 kerb strikes, 2.4 m/s |
| Scripted pilot (navigation, both legs, ≤ 200 m) | 19 / 20 delivered and landed |

## 7. Rewards, and the tricks the brains found (and the fixes)

RL agents exploit any gap. Every one so far, fixed with world rules or costs,
never with steering:

| Brain | Trick | Fix |
|---|---|---|
| nav | climbed to ~1,770 m to avoid every obstacle | 150 m ceiling above ground (航空法) ends the flight; cost for needless height; battery is a running cost |
| nav | hovered until the time limit | time-out = failure with the full crash penalty |
| nav | circled the drop | reward rises as it gets closer and falls equally when it moves away (potential-based, steeper in the last ~15 m); sideways speed round the target costs |
| nav | (bug) 41 % of practice trips were a made-up fallback with the drop inside a building | trip picker searches the zone, never falls back |
| mission | crashes while hovering were free; crash too cheap vs deliveries | every crash costs, everywhere; crash −150; low-battery landings cost |
| fleet pod | waited until all pods were flat | waiting parcels cost 0.05 / parcel / min; delivery +20; stranded −500 |
| fleet (eval) | "always wait" was only the deterministic test's artefact | tests sample actions like training |
| drive | stood still (time-out cheaper than a kerb) | time-out = −40 like a kerb; standing still without a reason (no stop line / red / car ahead) costs |
| drive | PPO outputs pinned at full brake | switched to SAC |
| nav | delivered 88 % but never landed (~2.6 M steps) | landing tolerance 1.0 m / 1.0 / 0.8 -> 1.3 m / 1.5 / 1.2 m/s (realistic touchdown): 0 -> 56 % landed in one check |
| fleet pod | never charged: all pods stranded | low pod battery costs below 25 % (the stranding alone came too late to learn from) |
| fleet pod | waited 328 / 359 decisions | per-decision credit made a 30 s wait look cheaper than minutes-long drives: waiting while there is work costs −3 (else −0.2); fresh pod brain |
| drive | crawled at 0.6–0.9 m/s | progress 0.1 -> 0.3 per metre |
| drive | (design) never got moving with SAC: 0.08 m/s | pedal was +2.5 / −6.0 m/s²: random exploration braked more than it accelerated; now symmetric ±3 m/s² |
| nav | (11-11 → 11-13) **policy collapse**: after a reward change (progress only for beating the best distance so far) on a resumed brain with `ent_coef` 0, the action stopped depending on the observation; the drone crept at ~1 m/s into the first building; 0 % landed for ~55 M steps | restart from the last healthy checkpoint (24 M), potential-based progress again, `ent_coef` 0.005 + log_std floor −2.0, goal-height cone removed (a hand-written rule), and **health alarms that stop training** (section 13). "Do nothing" was *not* a better strategy (−2.7 vs +39.4 discounted): exploration died, so the brain could not get back |

Watch for new tricks: a failure type (time-out, battery, ceiling, standing
still) rising while the success number stays flat.

### Nav: flying round no-fly zones (2026-11-18/19)

In CARLA the nav brain flew LastMile missions (new `RLMission`, delivery/rl_bridge.py: scripted
vertical take-off clear of everything within 15 m, the brain to the drop and on to the meeting
point, scripted hold and landing on the pod). On 30 real trips 13 ended in a no-fly zone: pod
1's drops lie behind the zone x 115-310, y −384..−280, and the brain flies straight. Random
practice trips almost never cross a zone (2/32), and on 32 crossing trips the 50 M brain hit
the zone 20 times. **Detour practice** (`--detour-share 0.5`: half the trips cross a zone,
env_nav `detour_share`) for 7.7 M steps changed nothing (19/32; CARLA trips 12/30), because the
progress reward measured the straight-line distance: going round a zone looks like losing
ground, cutting through costs one −50. **Progress round the zones** (`--geo-progress`, env_nav
`geo_progress`): the horizontal distance in the potential is the shortest legal path round the
no-fly zones (10 m margin, visibility graph over the zones' corners) plus the height
difference; still pure RL (only the reward changes). Run started 2026-11-19 from the 57.8 M
brain; check: `rl_delivery/research/nav_detour_check.py` (CARLA trips, crossing trips, normal
trips) → `nav_detour_check.json`.
航法ブレインが飛行禁止区域を直進して突入する問題：迂回練習だけでは改善せず、進捗報酬を「区域を回る最短距離」で測るように変更して再学習。

**First geo run failed (2026-11-19 09:00-10:01, auto-stopped: crash ≥ 80 % six times).** The
brain started diving onto the pod (touch-down −4 m/s vs −0.5) and stopped landing; checkpoints
moved to `checkpoints/nav/rejected_geo_20261119/` so `latest` stays the healthy 57.8 M brain.
Analysis on the same flights (57.8 M brain): the episode totals barely change with geo progress
(+3 per trip), but the distance **jumped** up to 63 m in one step (reward ±19): the visibility
check used the corners' 10 m margin, so near a zone no corner was visible and the distance fell
back to the straight line, and paths flipped between corners 10 m out; noisy reward spikes
plausibly drowned the small careful-flight terms. Fixed: visibility against the zones
themselves (0.2 m inside the edge), corners 1 m out, fallback via the nearest corner, no
progress on the step that enters a zone; the per-step change is now ≤ 1.9 m on 16 crossing
trips (straight line: 1.1 m). Second geo run started 2026-11-19 from the 57.8 M brain.
最初の geo 学習は着陸が崩れ自動停止。原因は距離の跳び（最大 63 m/ステップ）。修正して再学習。

**Second geo run (2026-11-19 10:32-14:32, 57.8 → 66.8 M steps, healthy: no collapse, landings
56-69 %, pod crashes 0-6 % in the last hours).** Check (`nav_detour_check.json`):

| brain | CARLA trips (30): completed / no-fly / obstacle | crossing trips (32): completed / no-fly | normal (32): completed |
|---|---|---|---|
| 50.1 M | 11 / 14 / 5 | 6 / 20 | 20 |
| 57.8 M (detour practice) | 8 / 13 / 8 | 5 / 19 | 20 |
| 66.8 M (geo progress) | 10 / 10 / 10 | 6 / 23 | 21 |

Not solved: after about 16 M steps of detour practice (half of them with progress measured
round the zones) the brain still enters the zone on about 70 % of the crossing trips; on
the CARLA trips the no-fly crashes fell a little (10 vs 13-14) but obstacle crashes rose.
Likely limit: the brain sees the zones only through 8 distance rays (150 m) and its goal as a
straight-line vector, so it cannot tell which way round a 200 m zone is shorter; going round
needs route information or far longer training. `latest` is the 66.8 M brain (healthy, as good
as before on normal trips).
2 回目の geo 学習は健全だが迂回は改善せず。原因はおそらく観測（距離レイと直線方向のみ）。

## 8. Curricula

- nav: trips 80 m, +60 m steps to 200 m, then +150 m to 900 m, each when 70 %
  of test flights land back on the pod; then wind in 20 % steps.
- drive: routes 150 m, +100 m to 800 m, then traffic +1 car up to 6, each when
  75 % reach the end with ~0 violations.
- fleet: none yet (full world from the start).

## 9. Traps

- carla-venv's PyTorch is the CPU build: training runs on the 20 CPU threads.
  `OMP_NUM_THREADS=1` per world process (set in train.py) or worlds fight over
  threads.
- Deterministic tests mislead early masked-discrete brains: sample.
- numpy bools break `json.dump` (watch.py converts them).
- netconvert maps make each direction its own road: CARLA reports no "left
  lane" across the centre line; prepare_drive finds neighbours geometrically.
  xodr lane headings are off at piece ends: env_drive resamples routes at 1 m.
- Dashboard helpers must not call CARLA while its port is closed (a failed
  connect holds Python's lock and freezes every page for 5–25 s).
- Mesh coordinates in `maps/build/<Map>/meshes` have y = north; CARLA y = south.
- `RecurrentPPO.load()` / `PPO.load()` keep the saved settings (`ent_coef`,
  learning rate): train.py now sets them after loading (`--ent-coef`, `--lr`).
  `--resume` takes the **newest file by time**; to start from a given brain use
  `--checkpoint FILE`. Resuming a brain across a reward change is a new run:
  watch explained variance and the health probe (section 13).
- OpenDRIVE `<speed max>` is in **m/s** unless it has a `unit`: until 2026-11-13
  prepare_map / prepare_drive stored it as km/h, so the driving brain learnt
  8.3 km/h limits on 30 km/h roads. `road_speeds()` converts now.
- `research_log.py` copies each run's progress from the file named in `RUNS`
  every time it runs: when a trainer starts a fresh progress file (no
  `--resume`), first freeze the old run's rows in their own file
  (`results/<brain>_progress_<run>.csv`) and point the run there.
- A success number at 0 % for a whole evaluation series means stop and look,
  not "keep training": the 11-11 nav run went ~55 M steps like that.
- The CARLA bridge exists (`delivery/rl_bridge.py`, `--policy-nav`,
  `--policy-drive`, `--policy-recovery`; status in `docs/RL_CARLA_BRIDGE.md`).
  Testing needs the demo stopped or restarted: ask the user.

## 10. Findings so far (material for the paper)

Lessons that hold across the brains (evidence: `rl_delivery/research/changes.csv`,
`runs.csv`, progress curves):
1. **Reward hacking is the norm, not the exception**: 7 distinct tricks in 2
   days (climbing, hovering until time-out, circling, idling until stranded,
   standing still, waiting, crawling). Each fix was a world rule or a cost, never
   scripted behaviour. Detect them by a failure type rising while success stays flat.
2. **Avoidance-by-inaction** (do nothing = no crash) appeared in every brain
   whose failure costs outweighed early progress; costing time-outs / idling
   as much as a failure removed it each time.
3. **Potential-based shaping** (closeness reward that falls equally when moving
   away) stopped circling and hovering and took navigation from 0 % to ~70 %
   reached (one leg).
4. **Curriculum** (short trips first) was needed: 0 % reached on 200 m trips
   from scratch vs 69 % after starting at 80 m.
5. **Success criteria matter**: an over-strict landing rule gave 0 % for 2.6 M
   steps; a realistic one gave 56 % within ~1 M steps.
6. **Semi-MDP credit**: in an event-driven world, per-decision rewards made
   short actions (waits) look cheap; costs must reflect time, or idle-with-work
   must be priced.
7. **Action design matters for exploration**: an asymmetric pedal kept SAC at
   standstill; PPO's clipped Gaussian saturated at the action bounds.
8. **Evaluation protocol matters**: deterministic (argmax) tests of an early
   masked policy showed "always wait" while its sampled behaviour delivered 29/h.
9. **Environment bugs look like learning failures**: a trip-picker fallback
   (41 % broken trips) and a 100× battery-draw bug were found only by looking at
   replays / traces, not curves.
10. Algorithm notes: RecurrentPPO (memory) learnt wind compensation for route
    following (beat PID: 100 % vs 90 % finished, 0.72 vs 1.38 m, 0 vs 10 %
    crashes in full wind); SAC for continuous driving after PPO saturation.

To make these claims in a paper: rerun each as a controlled comparison with
3–5 seeds through the experiment queue (`rl_delivery/research/README.md`),
e.g. with / without shaping, curriculum, landing tolerance, symmetric pedal,
idle cost; RecurrentPPO vs PPO vs SAC for navigation; SAC vs PPO vs TD3 for driving.

## 11. Status (2026-11-13 15:40 JST)

| Brain | Latest (test set) | Stage |
|---|---|---|
| Drone navigation | run nav_v7_fix from the 24 M brain (15:33): first evaluation 75 % delivered, 62 % landed at 300 m, health normal | re-climbing the distance curriculum with alarms on |
| Fleet | 14–17 deliveries/h (pod / drone brains, 11-12) vs 36 for the rule flow at the CARLA rate | needs work |
| Pod driving | retired 2026-11-16 (the pod is driven by code); last runs: resumed 32.5 M 58 % reached / 31 % kerb, fresh experiment 83 % / 10 % on 150 m routes | — |
| Drone mission | retired | — |

## 12. Next

1. Get each brain to its milestone in the pretend world (navigation: ≥ 70 %
   landed at ≥ 300 m with wind; driving: ≥ 75 % of 400 m+ routes with traffic,
   lawful; fleet: beat the rule flow).
2. Driving brain inside the fleet world (pod driving), navigation brain inside
   it (drone flying), top-up training of the fleet brains.
3. Driving world: pedestrians at crossings, cross traffic at junctions.
4. CARLA bridge (`delivery_sim --policy`, `docs/RL_CARLA_BRIDGE.md`), tests with the user's go-ahead.
5. Paper experiments through the queue once a brain's design is stable
   (algorithms, ablations, 3–5 seeds), tables / charts with `compare.py`.
6. Mini Pods and porter bots as more agent types (same fleet world design).
7. Optional: CUDA PyTorch for the RTX 5080 (faster network updates).

## 13. Navigation KPIs and health alarms (2026-11-13)

Computed by `train.py` at every evaluation (16 fixed test trips, deterministic)
and written to `results/nav_progress.csv` and each checkpoint's `.json`.
The design started from a proposal by Argon (`docs/NAV_FIX_BRIEF.md` was its
brief); thresholds are calibrated on the 2026-11 collapse (one run: treat them
as first values). / 評価ごとに計算する航法の KPI と、崩壊を検知して学習を止める警報。

**Outcome / 成果**

| Column | Definition | Target |
|---|---|---|
| `delivered`, `reached` | share of trips that hovered at the drop / landed on the pod | ≥ 80 % / ≥ 70 % at 900 m |
| `crash`, `crash_<cause>`, `timeout` | share of trips ending that way (pod, obstacle, no-fly, ceiling, battery) | crash ≤ 20 %; no cause rising alone |
| `ret_mean` | mean undiscounted return of the trips | ↑ |
| `zero_gap` | `ret_mean` minus the "do nothing" policy's return on the same trips (cached per world version / wind / distance in `results/nav_zero_baseline.json`) | ≥ 15 (alarm below) |
| `progress_drop`, `progress_pod` | per leg: (start − end horizontal distance) ÷ start; the pod leg only on delivered trips | → 1 |
| `speed_useful` | horizontal distance closed on both legs ÷ trip time (m/s) | ≥ 2 (the collapsed brain: ~1) |
| `landing_batt` | battery left on landed trips (%) | ↑ |
| `min_clear_p10` | 10th percentile of each trip's closest approach to anything (m) | ≥ 0.8 |

**Training health / 学習の健全性**

| Column | Definition | Healthy (24 M) | Collapsed |
|---|---|---|---|
| `probe_std_min` | smallest per-command spread of the deterministic action over 300 fixed observations (`results/nav_probe_obs.npy`, from the 24 M brain's flights) | 0.31 | 0.003 (alarm < 0.12) |
| `probe_abs_mean` | mean \|action\| over the same observations | 0.38 | 0.01 (alarm < 0.15) |
| `log_std_min/max` | the policy's exploration noise (floored at `--log-std-min`, default −2.0) | −1.3 | −6.5 |
| `dact_mean` | mean command change per step in the test trips | — | ~0 |
| `lstm_std` | spread of the LSTM state over a trip (memory in use) | — | — |
| `explained_var`, `approx_kl` | PPO update statistics averaged since the last evaluation | EV 0.6–0.85 | — |
| `alarm` | `collapse` (probe below its limits), `crash` (≥ 80 %), `zero_gap` | empty | — |

**Automatic stop / 自動停止:** two `collapse` alarms in a row, or three
evaluations with crash ≥ 80 %, write `results/STOP_nav` with the reason (also in
`train_state_nav.json` → `alarm`); roll back to the last healthy checkpoint
(`--checkpoint`) and fix the world or reward. The curriculum does not advance
while any alarm is on.
