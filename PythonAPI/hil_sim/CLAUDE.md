# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A human-in-the-loop driving-study rig on CARLA, driven with a Logitech G923
wheel (or the keyboard). `run_pipeline.py` boots the simulator, traffic, a
telemetry dashboard, a multi-camera viewer and the driving window as one
supervised process tree. The collected human-error telemetry then feeds the
RL layer in `rl/`.

`AGENTS.md` beside this file is the same guidance for Google Antigravity (with
the user's rules, the gates and the traps in `../../AGENTS.md`): when you change
this file, change it too.

Docs, comments and user-facing strings are bilingual **EN + JA**. Match that
when editing them — every table row, help line and error message in this repo
carries both.

## Placement is load-bearing

This folder must sit at `<CARLA_ROOT>/PythonAPI/hil_sim`. `run_pipeline.py`
derives everything from its own location:

```python
CARLA_ROOT = BASE_DIR.parent.parent      # run_pipeline.py:31
VENV_PY    = CARLA_ROOT / "carla-venv" / "bin" / "python"
```

and it launches every child process with `VENV_PY` and the simulator via
`CARLA_ROOT/CarlaUE4.sh`. Maps are loaded by `load_world_map()` (a throwaway
client: 120 s, then it checks the loaded map's name). It replaced
`PythonAPI/util/config.py`, whose 10 s wait made every client time out on the
big extended OitaStation (2026-11-09; `tests/test_map_switch.py`,
`tests/test_pipeline_control.py`).
Nested one level too deep, `CARLA_ROOT` silently resolves to `PythonAPI/`
and the run dies at `run_pipeline.py:153` with `ERROR: venv python not found`.

The scripts also expect a client library importable as `carla` — either the
egg in `<CARLA_ROOT>/PythonAPI/carla/dist` (the `glob` at the top of each
script) or a pip-installed `carla` wheel in `carla-venv`. The client version
must match the running server.

## Commands

```bash
# the launcher: only the dashboard starts; pick map/traffic/pods/car there and Launch
# (launcher.py; live control while running; Stop returns to the Launch page)
python3 run_pipeline.py

# full rig with flags (Ubuntu has no `python`, only `python3`)
python3 run_pipeline.py --map Town02 --vehicles 60 --walkers 15
python3 run_pipeline.py --keyboard          # W/A/S/D instead of the wheel
python3 run_pipeline.py --no-sim            # reuse a CARLA already on port 2000
python3 run_pipeline.py --map Town05 --delivery 3 --ffb   # + 3 delivery vans + force feedback

# individual components, each against a running server
<CARLA_ROOT>/carla-venv/bin/python dashboard.py --dashboard-port 5000
<CARLA_ROOT>/carla-venv/bin/python sensor_viewer.py
<CARLA_ROOT>/carla-venv/bin/python "manual_control_steeringwheel G923.py" --keyboard
```

Note the space in `manual_control_steeringwheel G923.py` — always quote it.

The dashboard UI is a React + TypeScript app in `web/` (rules: `web/DEV.md`).
`dashboard.py` is the JSON backend and serves the committed build `web/dist`.
After any change in `web/src` rebuild it (Node 22 is in `~/.local/node`, not on
the PATH):

```bash
export PATH=~/.local/node/bin:$PATH && cd web && npm run build   # tsc -b + vite build
```

Tests (no CARLA server required; needs `pygame` + `carla` in the venv):

```bash
<CARLA_ROOT>/carla-venv/bin/python tests/test_keyboard_control.py
<CARLA_ROOT>/carla-venv/bin/python tests/test_delivery.py   # delivery planner + drone sortie
```

Every `tests/test_*.py` runs alone without a server:
`for t in tests/test_*.py; do <CARLA_ROOT>/carla-venv/bin/python $t || echo FAIL $t; done`.

`test_keyboard_control.py` drives the real `DualControl` with synthetic key state and fake actors,
asserting on the `carla.VehicleControl` actually applied. There is no test
runner and no lint config.

RL layer (see `docs/RL_DESIGN.md` §6). Use the venv python; `stable-baselines3`
and CPU `torch` are installed there:

```bash
python -m rl.extract_scenarios      # mine human errors from data/ (offline)
                                    # -> data/scenarios/map_profiles.json (arm B traffic)
python -m rl.plot_town_events       # Town02 human event map + metric battery (offline)
python -m rl.smoke_test             # env sanity check (needs CARLA)
python -m rl.run_arm --arm A --seed 0 --timesteps 60000   # study arm, end to end
python -m rl.run_arm --arm B --seed 0 --timesteps 60000   # (needs CARLA, headless is fine)
python -m rl.run_arm --arm A --seed 0 --resume            # continue after a crash / freeze
python -m rl.plot_progress          # -> rl/results/figures/{progress,results}.png, results_table.md
python -m rl.plot_improvement       # -> rl/results/figures/improvement_*.png + improvement_table.md
```

`run_arm.py` owns the tick and the TrafficManager and starts
`generate_traffic.py --passive` underneath it; never run it while a human
session (`run_pipeline.py`) is using the same server.

**The study map is one constant.** `rl/config.py` holds `STUDY_MAP` (and
`EVAL_TRAFFIC`); every `--map` / `--eval-traffic` default reads it, so
switching the town for training and analysis is a one-line edit there.

**All RL output lives in `rl/results/`** (`rl/paths.py`: `models/`, `tb/`,
`eval_history.csv`, `figures/`, `logs/`). Only its README is tracked.
`run_arm.py` also protects the machine: on 2026-09-14 a training run hard-
locked the whole workstation (NVIDIA modeset kernel thread, see
`docs/TROUBLESHOOTING.md`), so it now refuses to start when the kernel log
of this boot already shows `NVRM: Xid` or lockups (`--allow-faults`), writes
a `logs/*.health.json` snapshot and a line-flushed `.log` from the first
second, runs the world in `no_rendering_mode` (the env has no camera;
`--render` to disable), caps torch at `--torch-threads 4`, nices itself,
and saves the replay buffer with each checkpoint so `--resume` continues a
run that was cut short.

## Architecture

**Actor discovery, not IPC.** Components never address each other directly.
They poll the CARLA world for a vehicle by `role_name`:

- `hero` — the human-driven car, spawned by the G923 script (or by
  `generate_traffic.py`). `dashboard.py` and `sensor_viewer.py` search for it
  and re-attach their sensors if it dies or is replaced.
- `ego` — an autonomous agent sharing the same world (`agent_drive.py`,
  `rl/carla_env.py`), so human and agent can be benchmarked side by side.

Other roles: `timetable_bus` (`bus_schedule.py`, OitaStation's GTFS buses),
`parked` (`parked_cars.py`, physics-off cars on a map's parking spots),
`bus_live_test` (`maps/bus_live.py`). generate_traffic.py runs both helpers
and removes their cars on exit; a killed run's leftovers are cleared at start.

Consequence: startup order barely matters, and a dashboard showing "waiting
for hero" is a normal transient state, not a failure. Run the agent as
`hero` to score it with the same dashboard metrics as a human session.

**Launcher and live control.** `launch_options.py` defines every launch option
(flag, type, default, and whether a change is `live`, a `map` switch or a
`restart`). `launcher.py` (no-argument `run_pipeline.py`) runs the pipeline as
a child with `--external-dashboard`. The dashboard writes requests through
`pipeline_control.py`: `data/launch_request.json` (launch / stop / relaunch, read
by the launcher) and `data/pipeline_control.json` (traffic / delivery / map /
vehicle / helper / stop, read by the pipeline's supervision loop). The pipeline
reports `data/pipeline_state.json` each loop. Traffic changes restart
`generate_traffic.py` (the tick master: restarted, never stopped).

**JSON side-channels.** Live coupling between the driving window and the
dashboard happens through small files next to the scripts, polled by mtime —
not through sockets:

- `pedal_tuning.json` — pedal strength/curve. The dashboard's Settings > Driving > Pedal Tuning
  card writes it; `DualControl._load_tuning` re-reads it ~1×/s while driving,
  as do the `[` / `]` keys. Mapping is `output = strength × press^curve`.
- `keybinds.json` / `g923_buttons.json` — key and wheel-button bindings and
  the G923 button names (`keybinds.py`; dashboard Settings > Keybinds). The
  control script re-reads them ~1×/s; it reports each wheel press in
  `wheel_last.json` and ignores wheel presses while `keybind_learn.json` says
  the dashboard is identifying buttons. Never show raw button numbers in a UI.
- `turn_signal.json` — blinker state written by the control script. While a
  signal is on, the dashboard scores a broken-line crossing as a legal lane
  change rather than a lane invasion.
- `traffic_heartbeat.json` — written by `generate_traffic.py` on every tick,
  polled by `run_pipeline.py`'s freeze watchdog. A stale mtime means the
  world stopped advancing, which `proc.poll()` cannot see.

- `map_request.json` — F5 / Shift+F5 in the driving window (`map_switch.py`).
  Under the pipeline the window never calls `load_world` (that aborted
  libcarla in every other client and took the rig down): it writes the target
  map + current car here and exits with code 75; `run_pipeline.py` stops the
  clients (traffic first), loads the map with CarlaUE4 still up, and restarts
  them all with the same options (`tests/test_map_switch.py`).
- `pipeline_control.json` / `pipeline_status.json` — the dashboard's
  Windows card (`windows_control.py`). The dashboard merges
  `{window: {action: open|close, t}}` into the control file (the pipeline
  never writes it, only clears it at start, and remembers the last `t` per
  window); `run_pipeline.py` starts/stops that one viewer with its launch
  command and rewrites the status file (~1 s; removed at shutdown). Only the
  read-only viewers (`sensor_viewer`, `drone_cams`) are offered. A viewer
  exiting with code 0 (its close button / Esc / Q) is a user close — no
  restart; any other code is a crash and is restarted. The open/closed state,
  not the launch flags, decides what `start_clients()` reopens after F5.
  Viewers catch SIGTERM so their sensors are destroyed (`tests/test_windows_control.py`).

All of these are gitignored and regenerated on first run.

**One tick master.** With traffic enabled the world runs in *synchronous*
mode, and a synchronous CARLA server advances only when a client calls
`world.tick()`. `generate_traffic.py` is the only thing in this rig that
does: the driving window, `dashboard.py` and `sensor_viewer.py` are all
passive readers. So the tick master is a single point of failure — if it
dies or wedges, everything freezes at once, and a driver sees a still frame
rather than an error. (RL training inverts the roles: `rl/run_arm.py`
ticks and owns the TrafficManager and starts `generate_traffic.py
--passive` under it — a synchronous TrafficManager only advances from the
process that owns it, so the ticking process must be the TM server.)

Three things keep that recoverable, and all three matter:

- `generate_traffic.py` never trusts `settings.synchronous_mode` to mean
  "someone is ticking". It probes for a real tick and takes over if none
  arrives, so a restart revives a frozen world instead of joining it as a
  second passive waiter.
- It restores asynchronous mode on the way out, including on SIGTERM (the
  pipeline's stop signal). Skipping that leaves the *server* synchronous
  with no master, which freezes every later session that reuses it — the
  failure survives process death and only clears when CarlaUE4 is killed.
  `run_pipeline.py` also forces async mode whenever it reuses a running
  CARLA, to clear that state after an unclean exit.
- `run_pipeline.py` watches the heartbeat (`--stall-timeout`, `--stall-grace`)
  and restarts the tick master when the world stops advancing.

A frozen world is therefore a bug in one of those three, not something to
work around elsewhere.

**Study arms are a traffic switch, nothing else.** `rl/run_arm.py` runs
one arm of `docs/RESEARCH_PROTOCOL.md` §3: arm A spawns default
TrafficManager traffic, arm B passes `--driver-profile
data/scenarios/map_profiles.json` so every NPC drives with the km-weighted
error statistics mined from the human sessions on that map
(`generate_traffic.py::apply_driver_profile`). Map, SAC settings, reward,
budget and seed list are identical; both arms are *evaluated* in the same
traffic, so rows of `rl/results/eval_history.csv` compare across arms only
when `eval_traffic` matches. `rl/plot_progress.py` reports CARLA
Leaderboard scores (RC, IS, DS = RC × IS) and infractions per km, mean ±
std over seeds, because that is the form the CARLA RL literature uses.
Figure text is English only; everything else stays EN + JA.

**One definition of "bad driving".** `rl/rules.py` holds `ALERT_RULES`
(thresholds) and `REWARD_RULES` + `step_reward` (penalties). The G923 script
imports `ALERT_RULES` from it, falling back to an inline copy only if `rl/`
isn't importable. `rl/carla_env.py` uses the same module for rewards. Change
a threshold in `rl/rules.py` and both the driver's HUD alert and the agent's
reward move together — that coupling is deliberate (`docs/RL_DESIGN.md` §3).

**Input handling.** `DualControl` (in the G923 script) serves wheel *and*
keyboard, and runs exactly one of them per frame: `_parse_vehicle_wheel()`
when a G923/G29 was detected, otherwise `_parse_vehicle_keys()`. Detection
failure falls back to the keyboard rather than raising. Both paths funnel
through the same `pedal_tuning.json` strength caps so telemetry stays
comparable across input devices.

**Data.** Every run writes `data/<YYYY-MM-DD>/<HH-MM-SS>/` containing
`telemetry.csv` (~10 Hz, 30 columns) and `summary.json` (rewritten every 5 s).
The dashboard's Statistics tab is built by scanning those folders, and
`rl/extract_scenarios.py` mines them into `data/scenarios/` as human-error
clips and driver profiles used to parameterise training traffic.

**Process supervision.** `run_pipeline.py` starts each child in its own
process group so the whole tree can be killed on exit, waits on the driving
window, and restarts crashed helpers a bounded number of times. It detects a
CARLA already holding port 2000 and reuses it instead of starting a second
simulator (two competing for the GPU is the usual cause of signal-11 at
startup).

## Reference docs

- `docs/RL_DESIGN.md` — observation/action spaces, reward formula (§3.1),
  scenario library, training recipe, run/evaluate workflow (§6).
- `docs/TROUBLESHOOTING.md` — what a healthy run's CPU/GPU numbers look like,
  the NVRM Xid GPU crash that masquerades as a CARLA bug (and why
  `--spawn-chunk` exists), PRIME GPU selection, empty `telemetry.csv`.
- `docs/RESEARCH_PROTOCOL.md` — the experimental design (hypothesis,
  conditions, metrics). Its decisions were fixed in a grilling session and
  are meant to change only by explicit revision, not silently.

## Delivery mission

`delivery_sim.py` + `delivery/` run a car + drone delivery scenario on any
map with `--vans N` (docs/DELIVERY.md); `planning.Dispatcher` gives every
batch to exactly one van. Keep `delivery/planning.py`, `delivery/drone.py` and
`delivery/zones.py` free of `carla` imports so `tests/test_delivery.py` and
`tests/test_zones.py` run without a server. Pod zones / no-fly zones / meeting points come from
`data/delivery_zones.json` (dashboard City Map, per map); `zones.ACTIVE` is the airspace
every drone flight routes round.

Inside `run_pipeline.py --delivery N` it runs `--passive`: generate_traffic.py
owns the synchronous tick, so delivery_sim must never tick or change world
settings (two tick masters double the sim speed). With `--no-traffic` the
pipeline drops `--passive` and delivery_sim becomes the tick master. Standalone
it defaults to sync at 20 Hz with real-time pacing (`--speedup`); async makes
BasicAgent weave off the road. The `airsim` backend targets CarlaAir and has
never run on this machine.
