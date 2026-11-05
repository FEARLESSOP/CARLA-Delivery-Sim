# Where is what (hil_sim)

A map of the important files, by topic. Paths are relative to
`~/Downloads/CARLA_0.9.16/PythonAPI/hil_sim`. Python for CARLA clients:
`../../carla-venv/bin/python`; map model steps: `maps/.venv/bin/python`.

## Start here

| File | What it is |
|---|---|
| `README.md` | how to run the rig, every feature, options |
| `FILES.md` | this map |
| `../../CARLA_RIG_SETUP_FROM_SCRATCH.md` | every change and how to rebuild the rig on a new PC |
| `../../map_guide/` | map building guide, numbered chapters (lives in the carla-map-generator skill) |
| `docs/TROUBLESHOOTING.md` | known problems and fixes |
| `CLAUDE.md` | instructions for Claude Code in this folder |

## Running the rig

| File | What it is |
|---|---|
| `run_pipeline.py` | starts CARLA and every client; restarts helpers; F5 map switch |
| `manual_control_steeringwheel G923.py` | the driving window (G923 wheel or keyboard) |
| `map_switch.py` | the F5 map change, done by the pipeline |
| `windows_control.py` | dashboard Windows card: open / close sensor viewer and drone cams |
| `dashboard.py` | Flask dashboard on :5000 (telemetry, FFB, keybinds, seasons, windows) |
| `keybinds.py` | keyboard and wheel bindings (`keybinds.json` is made on first use) |
| `carla_ffb.py`, `ffb_settings.py` | force feedback; tuning in `ffb_tuning.json`, `ffb_presets.json` |
| `vehicle_tuning.py` | vehicle performance and drivetrains (`vehicle_tuning.json`) |
| `traction_fx.py` | skid marks |
| `sensor_viewer.py`, `drone_viewer.py` | optional viewer windows |
| `seasons.py`, `season.json` | tree seasons and wind |

## Traffic, rules and signals (Japan maps)

| File | What it is |
|---|---|
| `generate_traffic.py` | NPC cars, cyclists, walkers; applies the country profile |
| `traffic_profile.py`, `traffic_profiles.json` | Japan driving rules: 止まれ full stop, yield to walkers, speeds (guide ch. 23) |
| `hokousha.py` | Japanese pedestrian signals: phase from the car lights, walkers obey (ch. 26) |
| `signal_timing.py`, `data/signal_timing_oita.json` | real police signal timing (JARTIC type C); auto-started on maps with maps.json `"signal_timing"` (ch. 18) |
| `lane_spawns.py` | spawn points on lanes |

## Delivery simulation

| File | What it is |
|---|---|
| `delivery_sim.py` | the delivery sim (vans, pods, drones) |
| `delivery/` | its modules; `delivery/rules.py` = Japan rules for the vans |
| `docs/DELIVERY.md` | how it works |

## Maps (carla-map-generator skill)

| File | What it is |
|---|---|
| `maps/maps.json` | one entry per map (package, OSM, keep_left, road_style, signal_timing, ...) |
| `maps/make_osm_city.sh` | the whole chain: OSM → road logic → model → FBX |
| `maps/build_map.sh` | import, cook, install (steps 2d/2e/2f: Japanese signals, speed posts, pedestrian heads) |
| `maps/osm_city/` | generators: `roadnet.py`, `lanes.py`, `layout.py`, `markings.py`, `buildings*.py`, `props.py`, `terrain.py`, `elevate.py` |
| `maps/osm_city/jp_signs.py`, `signs.py`, `speed_spot.py`, `lht_poles.py` | road signs, sign positions, signal poles |
| `maps/osm_city/jp_shingo.py` + `maps/ue_jp_shingo*.py` | Japanese traffic signal model BP_JpShingo (ch. 24) |
| `maps/osm_city/jp_sokudo.py` + `maps/ue_jp_sokudo*.py` | blue-number speed posts BP_JpSokudo (ch. 25) |
| `maps/osm_city/hokousha_crossings.py`, `jp_hokousha*.py` + `maps/ue_jp_hokousha.py` | pedestrian signal heads (ch. 26) |
| `maps/osm_city/check_roads.py`, `check_signal_poles.py` | checks: nothing on the road, poles where CARLA puts them |
| `maps/build/<Map>/` | build output (git-ignored): xodr, FBX, photos, logs |
| `maps/queue/` | queued map requests and handoffs |

## Live checks (CARLA running, read-only)

| File | What it checks |
|---|---|
| `maps/hokousha_live.py` | pedestrian signal phase vs car lights, frames per second |
| `maps/jp_sokudo_live.py` | speed posts match CARLA's limits, camera shots |
| `maps/stop_live.py` | cars (or `--role delivery`) stop at 止まれ |
| `maps/spa_live.py` | Spa-Francorchamps map and the RB16 |

## Own cars (carla-car-importer skill)

| File | What it is |
|---|---|
| `cars/cars.json` | one entry per car |
| `cars/build_cars.sh` | build and install every car |
| `cars/prepare_car.py`, `make_glass.py`, `make_lights.py`, `drivetrain.py` | the build steps |
| `cars/car_views.json`, `car_views.py` | driver eye and steering wheel per car |

## Tests

`tests/test_*.py`, each runs alone without a server:
`for t in tests/test_*.py; do ../../carla-venv/bin/python $t || echo FAIL $t; done`.
UE-side tests (`maps/ue_jp_*_test.py`) run in the editor commandlet; see their headers.

## Research / RL

`rl/` (SAC agent, evaluation, plots), `agent_drive.py`, `docs/RL_DESIGN.md`, `docs/RESEARCH_PROTOCOL.md`.
