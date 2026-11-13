# Putting the trained brains into CARLA (the bridge)

How the RL brains from `rl_delivery/` (see [RL_DELIVERY.md](RL_DELIVERY.md))
get to drive the pods and fly the drones in the real CARLA simulation.

> **Status (checked 2026-11-13) / 状況（2026-11-13 確認）: built, not working well.**
> The code below exists and its unit tests pass (`tests/test_rl_bridge.py`), but
> the brains it loads are not good enough to use; both RL options are `off` in
> the saved launch options. / コードとテストはあるが、読み込む脳が使える水準にない。
> - **Drone navigation** (`--policy-nav`, `RLNavFlight`): the latest checkpoint
>   collapsed in training (`results/nav_progress.csv`: `reached = 0`, `crash = 1.0`
>   from ~30 M steps). An earlier note called it "verified, 0 crashes"; the logs
>   do not support that. Since 2026-11-13 the flight is stepped at the trained
>   0.05 s even when the client loop is slower, and torch runs on one thread
>   (it slowed the delivery loop to ~4 Hz and made the drones jump).
> - **Pod driving** (`--policy-drive`, `DriveBridge`, SAC): on 2026-11-13 live it
>   froze all three LastMile pods at full left lock, so no drone could launch.
> - **Fleet recovery** (`--policy-recovery`, `FleetRecoveryBridge`): wires
>   `rl_delivery/ramppo.py` + `delivery/feasibility_engine.py` into the drop
>   (`D_AFTER`). Its hard-coded feasibility rules go against "nothing hardcoded"
>   ([RL_DELIVERY.md](RL_DELIVERY.md)), and its benchmark is not valid (see
>   [archive/antigravity_2026-11/README.md](archive/antigravity_2026-11/README.md)).
> - Checkpoint names `latest` / `auto` resolve to the newest file.
> - Tests: `tests/test_rl_bridge.py` (no CARLA), `tests/test_bridge_live.py` (needs CARLA).

## 1. The idea

A checkpoint is a small neural network (a `.zip` in
`rl_delivery/checkpoints/<brain>/`). It takes an **observation** (a fixed list
of numbers) and returns an **action**. The pretend world builds those numbers
from its own state; in CARLA the bridge must build **the same numbers, in the
same order and scale**, from the live simulation, and turn the action into
CARLA commands. Get the observation wrong and a perfect brain drives badly:
**observation parity is the whole job**.

Every brain keeps its checkpoint format; nothing is retrained for CARLA except
an optional fine-tune (pod driving) at the end.

## 2. When to import (milestones in the pretend world first)

Import a brain only when it does its job in the pretend world; otherwise CARLA
just shows the same failures slower.

| Brain | Ready for CARLA when (pretend world, test set) |
|---|---|
| Drone navigation | lands on the pod in ≥ 70 % of two-leg flights at ≥ 300 m, with wind on |
| Pod driving | reaches ≥ 75 % of 400 m+ routes with traffic, 0 stop / red violations, kerb strikes ≈ 0 |
| Fleet (pod + drone decisions) | beats the rule flow: more deliveries/h, fewer crashes/100, 0 pods stranded |

## 3. Command line (planned)

```bash
cd ~/Downloads/CARLA_0.9.16/PythonAPI/hil_sim
# one brain at a time, the others stay rule-based
python3 run_pipeline.py --map OitaKosen --delivery 3 \
    --delivery-args "--policy-nav rl_delivery/checkpoints/nav/step_XXXXXXXXX.zip"
python3 delivery_sim.py ... --policy-drive rl_delivery/checkpoints/drive/latest
python3 delivery_sim.py ... --policy-fleet-pod <ckpt> --policy-fleet-drone <ckpt>
# "latest" = the newest checkpoint of that brain
```

Dashboard (planned): a checkpoint picker per brain on the AI training page
("Test in CARLA"), which relaunches the pipeline with these flags. Launching
or stopping CARLA stops the user's demo: **always ask the user first**.

## 4. Brain by brain

### 4.1 Drone navigation (build first, simplest)

- In `delivery_sim` the drones are **props it moves itself** (kinematic), so
  the brain can fly them: every tick, step `rl_delivery/drone_dyn.Drone` with
  the brain's action and the wind, then set the prop's transform (the same
  `ApplyTransform` batch the sim uses now).
- **Wind**: CARLA has no wind on props; use `rl_delivery/wind.Wind` (the same
  hidden model), so CARLA tests the same physics the brain learnt. Seed it from
  the dashboard's weather (mean wind) if wanted.
- **Observation** (must match `env_nav.NavEnv._obs`, 54 numbers):
  noisy position / velocity / acceleration / attitude rotated into the drone's
  yaw frame; vector to the current goal (drop hover point, then the pod roof);
  height above ground and above what is below; the 16-ray range ring at its
  height and 8 rays 5 m lower (40 m); 8 no-fly distances (150 m); last action;
  battery; leg flag (0 to the drop, 1 to the pod).
  - Ranges and heights: from the same height map
    (`rl_delivery/maps/<Map>_heights.npz`, matches CARLA's ground within
    0.15 m) or from CARLA ray casts (`world.cast_ray`); start with the height
    map (identical to training), then compare against ray casts.
  - Goal: the drop = the order's delivery point at 7 m above the ground; the
    pod roof = the pod vehicle's live position, 2.6 m up (the pod may move:
    the brain was trained with a parked pod, so the fleet side should keep the
    pod parked until its drones land).
- **Done** when the drone hovers within 2.5 m at the drop (lower the parcel
  with the existing winch animation) and when it touches down on the pod roof
  (dock it with the existing hatch / crane animation).
- **Mapping out**: crash rules as in training (touching anything, no-fly,
  ceiling 150 m, battery) -> the sim's existing crash handling.

### 4.2 Pod driving

- The pod is a real CARLA vehicle (`vehicle.delivery.lastmile`): the action
  (steer, throttle/brake in -1..1) becomes `carla.VehicleControl`
  (steer = a[0], throttle = max(a[1], 0), brake = max(-a[1], 0)), applied every
  0.1 s (the brain's step; the sim ticks at 30 Hz: hold the control for 3
  ticks, or retrain at 30 Hz).
- **Observation** (must match `env_drive.DriveEnv._obs`, 30 numbers), rebuilt
  from CARLA each step:
  - speed, current steer; lateral offset and heading error to the route's lane
    centre; lane width; edge kinds each side (kerb / lane / oncoming) - use the
    same `rl_delivery/maps/<Map>_drive.json` lane data along the route the
    fleet (or the rule dispatcher) asks for, matched by position;
  - bends ahead at 5/10/20/30/40 m;
  - the next stop line: distance, kind (止まれ / signal), the **live** light
    colour from `carla.TrafficLight.get_state()` of the light controlling that
    lane, whether the pod already stopped there;
  - the car ahead on the route (gap, closing speed) from the world's vehicle
    actors projected onto the route;
  - speed limit (lane limit, `lane_limits.py`), distance to the route's end,
    last action.
- **Differences to expect** from the pretend world: real vehicle dynamics
  (the pretend world uses a kinematic bicycle), real traffic that also crosses
  junctions, pedestrians. Plan a **fine-tune in CARLA** (a few hours of SAC
  with the same rewards, CARLA in synchronous mode) after the first tests.
- **Safety**: none added (user rule: pure RL). Record kerb strikes (the
  existing off-road guard in `delivery_sim` detects them; count, don't fix)
  and violations, and compare with the scripted / rule-based pod.

### 4.3 Fleet (pod + drone decisions) - biggest

- Replaces `delivery_sim`'s dispatcher decisions (`Fleet`, `Van.assign`,
  `lmp_van`): at each pod decision point build `env_fleet.Fleet.pod_obs` (96
  numbers) and the action mask from the live sim (pod battery, docked drones'
  batteries, drones out / hovering, waiting orders, lockers with parcels,
  meeting point, stations), ask the pod brain, and turn the choice into the
  sim's existing moves (drive to a spot / locker / meeting point / station,
  launch n drones with a meeting point, request a standby swap, wait).
- Drone decisions likewise (`drone_obs`, 16 numbers): cruise height at launch,
  meeting point or station after the drop, keep hovering or go home, standby
  accept / decline.
- The option lists must be built **exactly** like `Fleet.pod_options` (6
  dispatch spots round the middle of the drone deliveries, 4 lockers by route
  distance, 3 meeting points, 3 stations), or the brain's choices mean
  something else.
- Needs in `delivery_sim`: standby drones flying out to a pod and tired drones
  flying to a station (swap), crash-site parcel recovery by a standby, the
  human wreck collection timer - parts of this exist in `delivery/station.py`
  (standby dispatch, recovery); the rest is added with the bridge.

## 5. How to test (once a part is built)

1. Ask the user to stop the demo (or test when it is stopped).
2. Run the CARLA baseline for comparison if it is old:
   `python3 -m rl_delivery.baseline --seeds 1 2 3 4 5` (rule fleet, KPIs in
   `results/baseline_OitaKosen.json`).
3. Run the same seeds with the brain(s) plugged in (`--policy-...`), same
   traffic and duration; `delivery_sim` writes the KPI block to each run's
   `summary.json` (`delivery/kpi.py`).
4. Compare per KPI (docs/DELIVERY_KPI.md): deliveries/h, lead time, crashes
   per 100 flights, lowest landing battery, pods stranded, kerb strikes,
   stop / red violations, no-fly breaches, drops outside the zone.
5. Watch it: camera Q / E cycles pods and drones in the CARLA window; the
   dashboard's Drones / Delivery pages show them live. Record a clip or
   screenshots for the user.
6. If CARLA is much worse than the pretend world: check observation parity
   first (log the observation vector in both worlds for the same situation
   and diff them), then dynamics (fine-tune), never by adding scripted help.

## 6. Order of work

1. Navigation bridge (drone props, height-map observations) -> test.
2. Driving bridge (vehicle control, live lights and cars) -> test -> CARLA
   fine-tune.
3. Fleet bridge (dispatcher replaced, standby swaps / recovery in the sim)
   -> test.
4. All three together -> KPI comparison against the rule fleet.
