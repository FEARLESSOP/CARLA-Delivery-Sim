# AGENTS.md — the rig folder `hil_sim` (for Google Antigravity and other agents)

The Antigravity counterpart of `CLAUDE.md` in this folder (same facts, adapted). Antigravity
loads this file when the workspace is `hil_sim` (or below), and the parent
`/home/inouelab/Downloads/CARLA_0.9.16/AGENTS.md` (the git root) with it. If your workspace is
the CARLA root, this file is NOT loaded automatically: read it. **Read the root AGENTS.md first:
it holds the user's rules, the working method, the gates, the checks and the traps.**

日本語の要約: この `hil_sim` は CARLA の `PythonAPI/hil_sim` に置く必要がある（パスはここから導出）。
ドキュメント・コメント・表示文字列は英日併記。ティックするのは generate_traffic.py だけ。
パイプラインの停止・再起動は必ずユーザーに確認し、ランチャー経由で行う。

## What this is

A human-in-the-loop driving-study rig on CARLA 0.9.16, driven with a Logitech G923 wheel (or
the keyboard). `run_pipeline.py` boots the simulator, traffic, the dashboard, viewers and the
driving window as one supervised process tree; the telemetry feeds the RL layer in `rl/`.
**Where is what:** `FILES.md` (every important file by topic). **How to run / every feature:**
`README.md`. **Every change + rebuild + open items:** `../../CARLA_RIG_SETUP_FROM_SCRATCH.md`.

**Bilingual EN + JA:** docs, comments, help lines, error messages and UI strings carry both
(match it in every table row you add). Result figures are English only. `FILES.md`,
`CLAUDE.md` and this file are English-led.

## Placement is load-bearing

This folder must be `<CARLA_ROOT>/PythonAPI/hil_sim`. `run_pipeline.py` derives
`CARLA_ROOT = BASE_DIR.parent.parent`, starts every child with `<CARLA_ROOT>/carla-venv/bin/python`,
the simulator with `<CARLA_ROOT>/CarlaUE4.sh` (PRIME offload env on this hybrid GPU), and loads
maps with `load_world_map()` (a throwaway client, 120 s + a check of the loaded map name; the old
`PythonAPI/util/config.py` gave up after 10 s and every client timed out on the big OitaStation).
Nested one level deeper, the run dies with `ERROR: venv python not found`.

## Commands

```bash
cd ~/Downloads/CARLA_0.9.16/PythonAPI/hil_sim
python3 run_pipeline.py                         # the launcher: only the dashboard (:5000); Launch there
python3 run_pipeline.py --map Town02 --vehicles 60 --walkers 15 --keyboard   # flag form
python3 run_pipeline.py --no-sim                # reuse a CARLA already on port 2000
../../carla-venv/bin/python "manual_control_steeringwheel G923.py" --keyboard   # note the space: quote it
```

**Restarting (ask the user first):** dashboard Stop → Launch, or
`../../carla-venv/bin/python -c "import pipeline_control as pc; pc.request(pc.LAUNCH_PATH, 'relaunch')"`.
Live changes without a restart (map switch, traffic counts, pods, car, helpers on/off) go through
`pc.request(pc.CONTROL_PATH ...)` from the dashboard; prefer the dashboard. To load new code into
one helper, SIGTERM that child only (the pipeline restarts it, 3 times per run); for
`carla_ffb.py` SIGTERM the `--child` worker (the user runs its supervisor).

**Dashboard UI** = React 19 + TypeScript in `web/` (rules `web/DEV.md`); `dashboard.py` is the
JSON backend and serves the committed `web/dist`. After a change in `web/src`:
`export PATH=~/.local/node/bin:$PATH && cd web && npm run build`.

**Tests** (no server; carla-venv): `for t in tests/test_*.py; do ../../carla-venv/bin/python $t || echo FAIL $t; done`.
There is no runner and no lint config; each test runs alone. `tests/test_keyboard_control.py`
section 1 fails while a G923 is plugged in (expected). New features get a `tests/test_<x>.py`
that runs without CARLA (fake world / shimmed `carla`).

## Architecture (what not to break)

- **Actor discovery, not IPC.** Components find vehicles by `role_name` (`hero` = the human,
  `ego` = an agent; also `timetable_bus`, `parked`, `bus_live_test`). Start order barely matters.
- **One tick master.** With traffic on, the world is synchronous and only `generate_traffic.py`
  calls `world.tick()` (30 Hz). Everything else (driving window, dashboard, delivery_sim
  `--passive`, overlays, live checks) is passive. A helper that ticks doubles the sim speed. It
  probes for a real tick, restores async on exit (also SIGTERM), writes `traffic_heartbeat.json`;
  the pipeline restarts it when the heartbeat goes stale. RL inverts this (`rl/run_arm.py` ticks).
  Never run RL training on the server a human session uses.
- **A passive process cannot step a synchronous TM:** a live test that needs its own TM uses
  another `--tm-port` in async (e.g. `maps/bus_live.py spawn --tm-port 8040`). The hero's
  autopilot uses tm-port+1.
- **Launcher and live control:** `launch_options.py` (every option: flag, default, `live` / `map`
  / `restart`), `launcher.py` (runs `run_pipeline.py ... --external-dashboard`, log
  `data/pipeline.log`), `pipeline_control.py` (`data/launch_request.json`,
  `data/pipeline_control.json`, `data/pipeline_state.json`). The map list is the installed levels
  (`map_switch.build_cycle`): a newly installed map appears by itself.
- **JSON side channels** next to the scripts, polled by mtime: `pedal_tuning.json`,
  `keybinds.json`, `turn_signal.json`, `traffic_heartbeat.json`, `map_request.json` (F5: the
  window exits 75, the pipeline loads the map and restarts the clients), `ffb_tuning.json`,
  `vehicle_tuning.json`, `season.json`, `weather_cycle.json`, `data/road_overlay.json`,
  `data/bus_status.json`, `data/delivery_live/`. All git-ignored.
- **Helpers started by the pipeline:** `weather_cycle.py` (every map), `road_overlay.py` (every
  map, idle until O / the dashboard card), `signal_timing.py apply --watch --only-map` (maps.json
  `signal_timing`), `traction_fx.py`, optional `carla_ffb.py`, `delivery_sim.py --passive`,
  viewers. `generate_traffic.py` also runs the Japan profile (`traffic_profile.py`),
  `hokousha.py`, `parked_cars.py` (maps with `maps/build/<Map>/parked_cars.json`) and
  `bus_schedule.py` (maps.json `bus_schedule`).
- **One definition of "bad driving":** `rl/rules.py` (alerts + rewards together).
- **Delivery:** keep `delivery/planning.py`, `drone.py`, `zones.py` free of `carla` imports.
  The `airsim` backend has never run.
- **Data:** `data/<date>/<time>/telemetry.csv` + `summary.json` per session (study data: never
  delete).

## The maps (all in `maps/`, skill carla-map-generator)

- `maps/maps.json`: one entry per map. `oita_station` (OitaStation, extended: the user's v8 model
  as an embedded `core` inside a generated city to ±640 m) must stay **before** the rollback
  `oita_station_v8` (same name; the first match wins). `oita_kosen` (OitaKosen, hilly, landmarks,
  JARTIC signs, police signal timing), `pasir_ris`, `marina_bay`, `spa_francorchamps`.
- `maps/make_osm_city.sh <key> --lefthand [--no-import]`: the whole chain (road logic → lane fit
  and kept-wall gate for a core → model → signs, speeds, pedestrian heads → core merge →
  lane-change marks → shop fronts → check_roads → surface grid → FBX → core join → import).
  Read the script: every step is commented. Build output `maps/build/<Name>/` (git-ignored, but
  needed at run time for seasons, parked cars and delivery addresses).
- `maps/build_map.sh <key> <ABSOLUTE build dir>`: import, cook, install (steps 2b foliage, 2c
  navmesh, 2d-2g Japanese signals / speed posts / pedestrian heads / lamps). Stops CARLA's map:
  ask before running it.
- Live checks (read-only, CARLA running): `maps/stop_live.py`, `hokousha_live.py`,
  `jp_sokudo_live.py`, `bus_live.py`, `road_overlay_live.py`, `spa_live.py`.
- Knowledge: `~/Downloads/CARLA_0.9.16/map_guide/README.md` (chapters 01-39). Embedded core:
  ch. 35; buses ch. 36; overlay ch. 37; lane-change marks ch. 39; change log ch. 38.

## Own cars (`cars/`, skill carla-car-importer)

`cars/cars.json` (12 vehicles: rx7, supra, imiev, csl, nsx, soarer, sileighty, rb16, sf90, dawn,
s800, and the NPC-only Oita bus `oitabus` = `vehicle.oitabus.erga`). `bash cars/build_cars.sh`
with no keys (the install replaces the whole UserCars package; CARLA stopped, ask first).
`own_cars.py`: `npc_only` vehicles never park, never join random traffic, are never "Your car".
Knowledge: `~/.claude/skills/carla-car-importer/car_guide/README.md`.

## Reference docs

`docs/RL_DESIGN.md`, `docs/RESEARCH_PROTOCOL.md` (decisions change only by explicit revision),
`docs/TROUBLESHOOTING.md` (Xid GPU crash, PRIME, empty telemetry), `docs/DELIVERY.md`,
`web/DEV.md`, `data/oita_bus/sources.md` (bus data credit line, CC BY 4.0: keep it wherever the
buses are shown).
