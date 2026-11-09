# RL for the LastMile pod and the drones (`rl_delivery/`)

Reinforcement learning for the delivery fleet on OitaKosen: the drones (and
next the pod) learn every behaviour themselves in a fast pretend world, are
watched on the dashboard, and are tested in CARLA. KPIs: [DELIVERY_KPI.md](DELIVERY_KPI.md).
Mini Pods and porter bots come later.

## The rules (user decisions, 2026-11-09)

- **Nothing hardcoded.** Every behaviour is RL: pod driving (kerbs, stop signs,
  signals), pod dispatch and charging, drone path finding round buildings,
  trees and no-fly zones, drone mission decisions (launch, height, meeting
  point, replacement drones, charging-station trips). Rule-based / PID / scripted
  code exists only as baselines to beat, never in control.
- **Pure RL, no safety layer.** No geofence for no-fly zones, no emergency stop.
  Breaches are possible; they are penalised and counted.
- **Pretend world first**, CARLA for testing (pod driving: fine-tune in CARLA).
- **Zones:** a pod and its drones (standby drones too) deliver and wait only in
  their own zone; other zones only to pass through to it or to a charger.
- **Wind is realistic and hidden:** no brain ever sees it.
- The agents must be allowed to fail in training (flat batteries, crashes).

## The brains

| Brain | Decides | Status | Code |
|---|---|---|---|
| Drone navigation | thrust / tilt / turn 20× a second; finds its own way to the drop, then to the meeting point and lands on the pod | training | `env_nav.py` |
| Drone mission | launch now (low / medium / high) or wait; which meeting point to return to | training | `env_mission.py` |
| Drone flight (retired) | followed a given route in wind | done, replaced by navigation | `env_flight.py` |
| Pod mission | orders, stop order, charging | to build | |
| Pod driving | steering / speed, kerbs, signs, signals, traffic | to build | |

## The pretend world

- **Map** (`prepare_map.py`, no CARLA needed): the road graph from the map's
  xodr (offline `carla.Map`), buildings.csv, `data/delivery_zones.json`.
- **Height map** (`prepare_heights.py`): the map builder's own meshes on a 2 m
  grid: ground + the top of anything on it (buildings, tree trunks and
  canopies, poles, lamps, signs). Ground matches CARLA's roads within 0.15 m.
- **Wind** (`wind.py`): mean wind with a log height profile, Dryden turbulence
  (MIL-F-8785C), smooth 1-cosine gusts, calmer air behind buildings, faster
  between them. Mean at 30 m up to ~11.5 m/s; brief peaks 17–20 m/s on the
  worst days.
- **Drone physics** (`drone_dyn.py`): thrust on a tilted body, drag on the
  air-relative velocity, attitude and motor lag (overshoot after gusts),
  battery drain from thrust.
- **Navigation senses** (like a real drone): noisy GPS / IMU, the drop as a
  vector, height above ground and above what is below, a 16-ray range ring at
  its height + 8 rays 5 m lower (40 m), known no-fly zones as 8 distances.
- **Mission world**: one pod zone, the pod on a rule round of stops (until the
  pod brain exists), 4 drones, orders, battery swaps in the pod, crashed
  drones replaced after 15 min, rain and wind; the wind the brain sees is an
  anemometer at the pod. Flights use a quick stand-in (route round no-fly zones,
  time / battery / crash risk from length, height, hidden wind, rain, building
  heights) until the navigation brain replaces it (option B: train both now,
  top-up later).

## A navigation flight (two legs, like the delivery sim)

Every flight is new: random take-off road in a pod zone, random building
(1,204) to deliver to, random meeting point on another road of that zone, new
wind. Tests use seeds it never practised on.
1. Pod roof -> drop: hover 4-10 m above the ground within 2.5 m, slower than
   1.5 m/s = parcel lowered (+60).
2. Drop -> meeting point: land on the pod roof (2.6 m above the road) within
   1 m of its centre, sinking < 1 m/s, drifting < 0.8 m/s (+100 + battery
   bonus). Any other touch of the pod is a crash ("pod"). Time limit 480 s.
The closer-is-better reward and the circling cost switch to the pod after the
drop; the brain senses which leg it is on. Scripted baseline at <= 200 m:
19 / 20 delivered and landed.

## Rewards and the tricks they had to stop

Every trick the brains found, and the fix (world rules / costs, never steering):

| Brain | Trick | Fix |
|---|---|---|
| Navigation | climbed to ~1,770 m to avoid every obstacle until the battery died | 150 m ceiling above ground (Japan's 航空法) ends the flight; cost for needless height; battery is a running cost |
| Navigation | hovered safely until the 240 s time limit | time-out is a failure with the full crash penalty |
| Navigation | circled the drop | reward rises as it gets closer and falls the same when it moves away (steeper in the last ~15 m, potential-based so hovering earns nothing); sideways speed round the drop within 30 m costs |
| Mission | crashes while hovering at a meeting point or chasing the pod cost nothing | every crash costs, wherever it happens |
| Mission | a crash (−40) was cheaper than four deliveries (+10 each): launched on 35 % battery | crash cost −150; growing cost for landing under 20 % battery |

Watch for new tricks: a failure type (time-out, battery, ceiling) rising while
"reached" stays flat. The training logs and charts split failures by type.

## Curriculum

- Navigation: trips (start -> drop and drop -> meeting point) of 80 m, then +60 m steps to 200 m, then +150 m steps to
  900 m, each when 70 % of test flights land back on the pod; then wind in 20 % steps to full.
- Flight (retired): wind in 20 % steps when 80 % of flights finished within 3 m.

## Baselines (the scores to beat)

| What | Source | Numbers |
|---|---|---|
| Today's rule fleet in CARLA (5 seeds × 30 min, 3 pods, 9 cars, 20 walkers) | `rl_delivery/baseline.py` → `results/baseline_OitaKosen.json` | pods 23.6 deliveries/h, lead 10.3 min; drones 23.6/h, 5.2 crashes / 100 flights, lowest landing battery 66 % |
| Rule mission in the pretend world | `RuleMission` | 11.5 deliveries/h, 4.2 crashes / 100, 15.5 min lead, lowest landing 47 % |
| Scripted pilot (climb 45 m, straight, descend; no avoidance) | `ScriptedPilot` | reached 65 %, hit obstacles 30 %, no-fly 5 % |
| PID autopilot (route following) | `PidPilot` | full wind: 90 % finished, 1.38 m off path, 10 % crashes |

## Running it

```bash
cd ~/Downloads/CARLA_0.9.16/PythonAPI/hil_sim
python3 -m rl_delivery.prepare_map --map OitaKosen       # once per map (or after a rebuild)
python3 -m rl_delivery.prepare_heights --map OitaKosen
python3 -m rl_delivery.train --brain nav --hours 6       # nav | mission | flight
python3 -m rl_delivery.train --brain nav --resume        # carry on from the latest checkpoint
touch rl_delivery/results/STOP_nav                       # stop (saves first); STOP for flight
python3 -m rl_delivery.watch --brain nav --checkpoint latest --wind 0.5 --seed 7
python3 -m rl_delivery.baseline --seeds 1 2 3 4 5        # CARLA baseline (demo must be stopped)
```

Or the dashboard **AI training** page: pick a brain, Start / Resume / Stop,
learning chart, and watch mode (Record & play any checkpoint or baseline:
top-down map, side view / wind close-up, decision log, per-flight KPIs).

Generated data is git-ignored: `rl_delivery/checkpoints/`, `results/`, `maps/`.

## Traps

- carla-venv's PyTorch is the CPU build; training runs on the 20 CPU threads.
- One process per world: `OMP_NUM_THREADS=1` (set in train.py), or the worlds
  fight over threads and training crawls.
- Check `ps` for duplicate trainers after a restart; two trainers on one brain
  mix their checkpoints.
- numpy bools break `json.dump` (watch.py converts them).
- The dashboard's CARLA helpers must not call CARLA while its port is closed:
  a failed connect holds Python's lock and freezes every page for 5–25 s.

## Next

1. Mission world: replacement drones flying out to the pod, tired drones to
   the charging station; all as RL decisions.
2. Pod mission brain (orders, stop order, going to charge).
3. Pod driving world and brain (lanes, kerbs, stop signs, signals, traffic).
4. Navigation brain inside the mission world (replaces the flight stand-in), top-up.
5. All agents together; checkpoints in CARLA (`delivery_sim --policy`).
