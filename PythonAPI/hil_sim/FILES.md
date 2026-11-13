# Where is what (hil_sim)

A map of the important files, by topic. Paths are relative to
`~/Downloads/CARLA_0.9.16/PythonAPI/hil_sim`. Python for CARLA clients:
`../../carla-venv/bin/python`; map model steps: `maps/.venv/bin/python`.

## Start here

| File | What it is |
|---|---|
| `README.md` | how to run the rig, every feature, options |
| `FILES.md` | this map |
| `docs/README.md` | index of every document, by topic (2026-11-13) |
| `../../CARLA_RIG_SETUP_FROM_SCRATCH.md` | every change and how to rebuild the rig on a new PC |
| `../../map_guide/` | map building guide, numbered chapters (lives in the carla-map-generator skill) |
| `docs/TROUBLESHOOTING.md` | known problems and fixes |
| `CLAUDE.md` | instructions for Claude Code in this folder |
| `AGENTS.md`, `../../AGENTS.md` | the same for Google Antigravity: the user's rules, working method, map/car workflow with gates, checks and pass criteria, traps (root file); `../../.agents/skills/` points Antigravity at the two Claude skills |

## Running the rig

| File | What it is |
|---|---|
| `run_pipeline.py` | starts CARLA and every client; restarts helpers; F5 map switch; loads maps with `load_world_map` (120 s + check; `util/config.py` timed out on big maps) |
| `launcher.py` | `python3 run_pipeline.py` with no arguments: the dashboard's Launch page starts / stops / restarts the pipeline |
| `launch_options.py` | every launch option (flag, type, default, live / map / restart), maps and cars to pick |
| `pipeline_control.py` | request / state files between the dashboard, the launcher and the running pipeline |
| `web/` | the dashboard UI: React + TypeScript (Vite), `web/DEV.md`; `web/dist` is the build Flask serves |
| `manual_control_steeringwheel G923.py` | the driving window (G923 wheel or keyboard) |
| `map_switch.py` | the F5 map change, done by the pipeline |
| `windows_control.py` | dashboard Windows card: open / close sensor viewer and drone cams |
| `dashboard.py` | Flask backend on :5000: every JSON route, serves `web/dist` |
| `keybinds.py` | keyboard and wheel bindings (`keybinds.json` is made on first use) |
| `carla_ffb.py`, `ffb_settings.py` | force feedback; tuning in `ffb_tuning.json`, `ffb_presets.json` |
| `vehicle_tuning.py` | vehicle performance and drivetrains (`vehicle_tuning.json`) |
| `traction_fx.py` | skid marks |
| `road_overlay.py` | road logic overlay (O key, Drive tab card "Road logic overlay"): the OpenDRIVE map the AI uses drawn around the hero and around the CarlaUE4 window's camera with dim debug lines: road edges, direction, junctions, lane types, rules (signs, lights + stop bars, lane limits), lane change, ids, raw OSM ways; `data/road_overlay.json`, `tests/test_road_overlay.py`, live `maps/road_overlay_live.py` (map guide ch. 37) |
| `sensor_viewer.py`, `drone_viewer.py` | optional viewer windows |
| `seasons.py`, `season.json` | tree seasons and wind |
| `weather_cycle.py` | day/night cycle (24 h in 5 min), dynamic weather, street lamps on/off; rain grounds the drones (`weather_cycle.json`, dashboard card "Day & weather") |

## Traffic, rules and signals (Japan maps)

| File | What it is |
|---|---|
| `generate_traffic.py` | NPC cars, cyclists, walkers; applies the country profile |
| `traffic_profile.py`, `traffic_profiles.json` | Japan driving rules: 止まれ full stop, yield to walkers, speeds (guide ch. 23) |
| `hokousha.py` | Japanese pedestrian signals: phase from the car lights, walkers obey (ch. 26) |
| `parked_cars.py` | CARLA + imported cars parked on a map's parking spots (maps/build/<Map>/parked_cars.json), physics off; generate_traffic `--parked-cars` (ch. 35) |
| `bus_schedule.py` | timetabled Oita buses on OitaStation (GTFS-JP, `data/oita_bus/`): lane planning on the road logic, spawn times, TM driving with stops / dwell / station hold, `data/bus_status.json`; generate_traffic `--bus-schedule` (guide ch. 36); `maps/bus_live.py` live check; `tests/test_bus_schedule.py` |
| `data/oita_bus/` | the bus timetable: `sources.md` (sources, CC BY 4.0 credit, gaps), `stops.json`, `routes.json`, `timetable.json`, `shapes.json` (route shapes, `tools/shapes.py`), `tools/` |
| `signal_timing.py`, `data/signal_timing_oita.json` | real police signal timing (JARTIC type C); auto-started on maps with maps.json `"signal_timing"` (ch. 18) |
| `lane_spawns.py` | spawn points on lanes |
| `lane_limits.py` | the speed limit of the lane a car is on (dashboard, driving window) |

## Delivery simulation

| File | What it is |
|---|---|
| `delivery_sim.py` | the delivery sim (vans, pods, drones) |
| `delivery/` | its modules; `delivery/rules.py` = Japan rules for the vans |
| `docs/DELIVERY.md` | how it works |
| `delivery/station.py` | drone fleet: charging, standby, wind and rain grounding |
| `delivery/loop_prof.py` | main-loop timing: the `[loop]` lines in the pipeline log every 20 s (Hz, ms per phase, blocking CARLA calls with file:line) |
| `delivery/backends.py` | drones in CARLA (`DroneUnit`), sensor probe thread, `RayWorker` (collision rays off the main loop) |
| `delivery/rl_bridge.py` | trained brains in CARLA: `RLNavFlight`, `DriveBridge`, `FleetRecoveryBridge` (`docs/RL_CARLA_BRIDGE.md`) |
| `delivery/feasibility_engine.py`, `delivery/compact_simulator.py`, `delivery/reservations.py`, `delivery/ledger.py`, `rl_delivery/ramppo.py` | Antigravity's FA-RAMPPO recovery planner (hard-coded rules; benchmark not valid: `docs/archive/antigravity_2026-11/README.md`) |
| `triple_screen.py` | the driving window across three monitors (monitor detection, cameras, fullscreen span) |
| `recovery.py` | stuck NPC cars / pods / the hero reverse out of a wedge; still walkers get a new route |
| `hero_autopilot.py` | the smooth autopilot behind P in the driving window (lane limits, bends, lights, pedals) |
| `delivery/zones.py` | pod delivery zones, no-fly zones and the drone detours (`data/delivery_zones.json`) |
| `delivery/drone_sensors.py` | each drone's sensor suite for the dashboard's Drones page (GNSS/RTK, IMU, baro, lidar, flow, obstacle, wind, battery, motors, winch, link, Remote ID, parachute; sources in the docstring), camera specs, the damage model (which parts a crash / failure breaks); no CARLA |
| `drone_window.py` | Drones page "Open CARLA view": a CARLA window for one drone (chase camera following the drone body, orbit, onboard camera strip, HUD with damage warnings); never leaks a sensor |
| `drone_windows.py` | starts / focuses / replaces / closes the per-drone `drone_window.py` processes for `dashboard.py` (`/drones/window`), pid files in `data/drone_windows/`; no CARLA |
| `delivery/assets/make_drone_web.py` | splits `LastMilePod_Drone.glb` into named parts (arm/motor/ESC/rotor 1..4, shell, battery, winch, ...) -> `web/src/views/drones/three/lmp_drone_parts.glb` |
| `delivery/assets/make_drone_svg.py` | the part classification `make_drone_web.py` uses (it can still write the old line drawing; the page no longer uses it) |
| `web/src/views/drones/` | the Drones page: tabs, overview, component close-ups, damage report, 3D drone (`three/`, three.js, lazy chunk), flight data, mini map, sensor cards |
| `tests/test_drone_sensors.py` | sensor maths, damage model, raycast budget, snapshot (fake world) |
| `delivery/lmh_rig.py` | LastMile Hub (LMH-3) + Mini Pods, no CARLA: the hub timeline ported from the site's `mini.js` (scan, divert, lift, load, release, roll out, dock), part motions from `delivery/assets/lmh_parts.json` (stub until it exists), kerb-side paths, A*, obstacle sweep, distribution / meeting point choice, the pod state machine (front drop, delivery box, stuck recovery, faults + rescue) and the hub's crew |
| `delivery/lmh_hub.py` | the hub in CARLA (mixed into a Van): lane-graph router on the kerb side, obstacle cache, ray thread for the pods, props (hub parts, pod bodies; LMP-4 + box stand-ins until the LMH assets are imported), events, published state |
| `delivery/minipod_sensors.py` | each Mini Pod's sensor suite (lidar, 6 cameras, ultrasonic, bumper, GNSS/RTK, IMU, wheels/motors, battery, lid/shutter/flag, LTE, e-stop; sources in the docstring); no CARLA |
| `tests/test_lmh.py` | no-fly orders -> hubs, timeline, kerb paths, pod state machine, dist / meet points, published state; no CARLA |
| `tests/lmh_live.py` | live check: one hub + pods in the running world for one cycle (never ticks), cleans up |
| `minipod_window.py` | Mini Pods page "Open CARLA view": a CARLA window for one mini pod (chase camera behind the pod body, orbit, six onboard cameras from `lmh_parts.json` mounts, HUD with damage / faults; on the hub vehicle while docked; `--actor` follows any actor for tests); never leaks a sensor (`hil_podwin_<id>`) |
| `minipod_windows.py` | starts / focuses / replaces / closes the per-pod `minipod_window.py` processes for `dashboard.py` (`/minipods/window`), pid files in `data/minipod_windows/`; no CARLA |
| `web/src/views/minipods/` | the Mini Pods page: pod tabs, overview, component close-ups, hub card, damage report, 3D pod (`three/`, the real `lmh_minipod.glb`, lazy chunk), drive data, mini map, sensor cards from the self-describing sensor dict |
| `web/public/models/lmh_minipod.glb` | the Mini Pod model the page loads (copied from `~/Downloads/LastMileHub_FBX_Maya_Unreal/LastMileHub_MiniPod.glb`; node names `MiniPod_P1_Wheel1..6`, `_Lid`, `_Shutter`, `_FlagMast`, `_Flag`), served by `dashboard.py` at `/models/` |
| `tests/test_drone_window.py` | window processes (open / focus / replace / close), camera following / re-attach / pod / crash (fake world), `/drones/window` |

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
| `maps/osm_city/core_merge.py`, `core_fbx.py`, `core_cars.py`, `core_materials.py`, `core_facades.py`, `topo_buildings.py` | extending an existing model (maps.json "core"): cut + join, parking spots, core ground re-materialled, survey facades, topoexport buildings (ch. 35) |
| `maps/osm_city/fit_core_widths.py`, `check_keep_walls.py` | embedded core: lanes fitted to the model's asphalt (seam targets, bulge rule; kept walls a hard limit) and the gate that stops the chain when a driving lane runs within 0.3 m of the kept station / viaduct walls (ch. 35) |
| `maps/osm_city/core_patch.py`, `layout.py` (`add_seam_fill`) | embedded core: drive patch (ground paved where the lanes leave the model's asphalt: check_drive 0) and the seam wedges between the model and the generated carriageway (ch. 35) |
| `maps/osm_city/core_props.py` | the model's shop-front props left out in the core_fbx JOIN: bicycles, menu boards, door planters / pots, vending machines (v8 wall feet cached in `core/v8_walls.npz`; ch. 35) |
| `maps/osm_city/shopfront_jp.py` + `test_shopfront_jp.py` | Japan maps: shop entrances (自動ドア, 格子戸 + 暖簾) drawn by buildings_jp and recorded (`shopfronts.json`), then vending machines + recycling box, A-frame / stand kanban, pots, planters, のぼり in front of them after core_merge (`Terrain_JpShopfront_*`, `shopfront_props.json`; ch. 10) |
| `maps/osm_city/jp_speeds.py` | speed limits: ring 100, real limits, legal 60/30 (ch. 29) |
| `maps/osm_city/lane_change_jp.py` + `tests/test_lane_change_jp.py` | Japan maps: every xodr `<roadMark>` (type, colour, laneChange) from the painted lines (marking meshes, core tile included): yellow solid between lanes = no lane change, white solid / broken = allowed, solid kerb / centre = none; only roadMark lines change (`.xodr.prelanechange`, `lane_change.json`); make_osm_city.sh after core_merge (map guide ch. 39) |
| `maps/osm_city/gaito.py`, `jp_gaito.py` + `maps/ue_jp_gaito.py` | street lamps that light up, BP_JpGaito (ch. 28) |
| `maps/osm_city/check_roads.py`, `check_signal_poles.py` | checks: nothing on the road, poles where CARLA puts them |
| `maps/osm_city/check_drive.py`, `check_paths.py`, `check_markings.py`, `check_stop_poles.py` | checks: built road vs road logic (0 spots in every class, ch. 30), walkable network (ch. 31), markings (0 problems, ch. 32) |
| `maps/osm_city/test_*.py`, `landmarks/test_*.py` | model-side tests (`maps/.venv/bin/python`) |
| `maps/build/<Map>/` | build output (git-ignored): xodr, FBX, photos, logs |
| `maps/queue/` | queued map requests and handoffs |

## Live checks (CARLA running, read-only)

| File | What it checks |
|---|---|
| `maps/hokousha_live.py` | pedestrian signal phase vs car lights, frames per second |
| `maps/jp_sokudo_live.py` | speed posts match CARLA's limits, camera shots |
| `maps/stop_live.py` | cars (or `--role delivery`) stop at 止まれ |
| `maps/spa_live.py` | Spa-Francorchamps map and the RB16 |
| `maps/bus_live.py` | timetabled buses: `watch` (read-only) or `spawn --time 08:00 --n 5 --tm-port 8040` (own role / TM, removes its buses); door vs pole per stop (ch. 36) |
| `maps/road_overlay_live.py` | road logic overlay on for 20 s near the hero: off / on camera shots, server FPS, shapes and draw time |

## Own cars (carla-car-importer skill)

| File | What it is |
|---|---|
| `cars/cars.json` | one entry per car |
| `cars/build_cars.sh` | build and install every car |
| `cars/prepare_car.py`, `make_glass.py`, `make_lights.py`, `drivetrain.py` | the build steps (heavy NPC vehicles: `steering_wheel` none, `max_texture`; build_vehicle.py brakes, physics box, base_type; ch. 16) |
| `cars/car_views.json`, `car_views.py` | driver eye and steering wheel per car |
| `own_cars.py` | cars.json blueprint ids; `npc_only` vehicles (the Oita bus) kept out of parking, random traffic, "Your car" and the hero cycle (car guide ch. 16) |
| `tests/test_heavy_npc.py` | npc_only filtering, heavy-vehicle build options (brakes, physics box, base_type, no steering prop); no CARLA, no UE |

## Tests

`tests/test_*.py`, each runs alone without a server:
`for t in tests/test_*.py; do ../../carla-venv/bin/python $t || echo FAIL $t; done`.
UE-side tests (`maps/ue_jp_*_test.py`) run in the editor commandlet; see their headers.

## Research / RL

`rl/` (SAC agent, evaluation, plots), `agent_drive.py`, `docs/RL_DESIGN.md`, `docs/RESEARCH_PROTOCOL.md`.
