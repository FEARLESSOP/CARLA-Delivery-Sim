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

- **Nothing hardcoded.** Every behaviour is learned: pod driving, pod
  decisions, drone flying and path finding, drone decisions. Rule-based /
  scripted / PID code exists **only as baselines to beat**, never in control.
  Before adding any helper that steers, plans, chooses or vetoes, check it is
  world physics, an option list or a reward, not behaviour.
- **Pure RL, no safety layer**: no geofence for no-fly zones, no emergency stop
  for the pod. Breaches are possible; they are penalised and counted.
- **Pretend world first**, CARLA for testing (pod driving may fine-tune there).
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
| **Pod driving** | steering + pedal (±3 m/s², symmetric) 10×/s | `env_drive.py` | SAC (PPO pinned its outputs at full brake) | `checkpoints/drive` |
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

Watch for new tricks: a failure type (time-out, battery, ceiling, standing
still) rising while the success number stays flat.

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
- No CARLA bridge yet (plan and how-to: `docs/RL_CARLA_BRIDGE.md`): `delivery_sim.py` has no `--policy` hook. Order to
  build it: navigation (drones are props it moves), driving (`apply_control`
  with observations rebuilt from live waypoints / lights / actors), fleet
  (replaces the dispatcher). Testing needs the demo stopped: ask the user.

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

## 11. Status (2026-11-10 13:30 JST)

| Brain | Latest (test set) | Stage |
|---|---|---|
| Drone navigation | 7.8 M steps: 88 % delivered, 50–62 % landed on the pod, 25 % obstacle, 6 % no-fly | 200 m trips, calm; needs 70 % landed for longer trips, then wind |
| Fleet | fresh pod brain (12:15) still in its first round; before: ~9 deliveries/h, pods stranded (rule flow 41.5/h, 0 stranded) | fixing idling / charging |
| Pod driving | restarted 13:0x with the symmetric pedal: 0.1 M steps, 0 % reached, 0.3 m/s | early |
| Drone mission | retired; stopped (an accidental dashboard start ran 2 h) | — |

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
