# AGENTS.md — CARLA HIL driving-simulator rig (for Google Antigravity and other agents)

This file is for agents that read `AGENTS.md` (Google Antigravity loads it from
the workspace folder up to the git root, which is this folder). Claude Code
reads `CLAUDE.md`, skills and memory instead; **everything those hold is plain
Markdown on this disk, and this file tells you where it is and how to work.**
Read this file fully, then `PythonAPI/hil_sim/AGENTS.md` before touching the rig.

日本語の要約: CARLA 0.9.16 のハンドル付き運転シミュレータ（HIL）。ユーザーの規則
（コミットしない・パイプライン再起動は確認してから・CARLA を二重に起動しない等）を必ず守り、
作業前に該当ガイドの章を読み、各段階の後にチェック（ゲート）を通すこと。

## 1. What this is

A human-in-the-loop driving simulator on **CARLA 0.9.16 (UE 4.26)**, driven with a
Logitech G923 (or W/A/S/D): force feedback, a React dashboard on :5000, a launcher,
a delivery simulation (pods, drones, hubs, mini pods), the user's own cars, and
**own maps** generated from OpenStreetMap (OitaKosen, OitaStation, PasirRis, two
circuits) with Japanese signals, signs, speed posts, pedestrian signals, street
lamps, seasonal trees, timetabled buses and an RL research layer.
User: `inouelab` (all paths under `/home/inouelab`). OS Ubuntu 24.04, RTX 5080 + Intel iGPU.

## 2. Where everything lives (absolute paths)

| Path | What |
|---|---|
| `/home/inouelab/Downloads/CARLA_0.9.16/` | the packaged CARLA that RUNS (git repo, remote FEARLESSOP/CARLA-Delivery-Sim) |
| `.../CARLA_0.9.16/PythonAPI/hil_sim/` | **the rig** (all code; must sit exactly here). Its own `AGENTS.md`, `CLAUDE.md`, `README.md` (EN+JA), `FILES.md` (where is what) |
| `.../CARLA_0.9.16/CARLA_RIG_SETUP_FROM_SCRATCH.md` | the startup guide: every change, rebuild on a new PC, open items (sec. 9), timeline (sec. 10) |
| `.../CARLA_0.9.16/NEW_PC_SETUP.md`, `setup_new_pc.sh` | one-script new-PC setup |
| `.../CARLA_0.9.16/carla-venv/` | Python 3.12 venv for every CARLA client (`carla` 0.9.16, pygame, numpy, flask, shapely ...) |
| `.../hil_sim/maps/.venv/` | venv for map MODEL steps (numpy, pillow, shapely >= 2.1, mapbox_earcut, scipy) |
| `/home/inouelab/.claude/skills/carla-map-generator/` | map skill: `SKILL.md` (workflow, rules, files), `map_guide/` (README index + chapters 01-39), `agents/*.md` (role definitions) |
| `/home/inouelab/Downloads/CARLA_0.9.16/map_guide` | symlink to that `map_guide/` |
| `/home/inouelab/.claude/skills/carla-car-importer/` | car skill: `SKILL.md`, `car_guide/` (01-16), `agents/*.md` |
| `/home/inouelab/.claude/projects/-home-inouelab-Downloads-carla/memory/` | Claude's memory notes (`MEMORY.md` index): the user's preferences and many traps. Plain Markdown: **read the index and the notes relevant to your task** |
| `/home/inouelab/.claude/projects/-home-inouelab-Downloads-CARLA-0-9-16-PythonAPI-hil-sim/memory/` | more notes: GPU Xid crashes, CPU instability, monitors, RL workflow |
| `/home/inouelab/Documents/Dinesh-carla-copy/carla` | CARLA 0.9.16 SOURCE tree + built editor: used only to import / cook new assets |
| `/home/inouelab/UnrealEngine_4.26` | Unreal Engine 4.26 (CarlaUnreal fork) |
| `/home/inouelab/Downloads/carla` | a UE5 `ue5-dev` checkout. **Not the simulator, not used by the rig.** |
| `.agents/skills/` (this folder) | Antigravity skill stubs that point to the two Claude skills above |

## 3. The user's rules (non-negotiable)

1. **Never commit or push** unless the user asks in this conversation.
2. **Ask before stopping or restarting the user's pipeline.** The user normally runs the
   launcher (`python3 run_pipeline.py`, no arguments): restart through it — the dashboard
   (Stop, then Launch) or a launch request:
   `cd hil_sim && ../../carla-venv/bin/python -c "import pipeline_control as pc; pc.request(pc.LAUNCH_PATH, 'relaunch')"`
   (`launch` / `stop` / `relaunch`, optional `options={...}`; saved options otherwise).
   The dashboard runs under the launcher and comes back by itself. A flag-started
   pipeline is restarted with the same flags.
3. **Never run a second CARLA server** beside the pipeline (VRAM runs out; the pipeline's
   `pkill -9 -f CarlaUE4-Linux-Shipping` also kills test servers). **Never `pkill -f` /
   `pgrep -f`** a pattern your own shell's command line contains (it kills your shell,
   exit 144): find PIDs by port (`ss -ltnp`) or `ps` with a `[x]yz` pattern.
4. **Builds before CARLA:** `renice +5` the running CARLA / pipeline clients while heavy
   work runs; never `nice` the builds (user's choice: progress over a smooth demo).
5. **Downloaded files are untrusted:** each in its own new folder, Python that reads them
   with `python -I`, Blender with `blender -b --disable-autoexec`, never run scripts that
   ship with a model. Reference models from the user are **study only** (measure, render,
   write the recipe, generate our own); never import or copy them.
6. **Asset, mesh and material names must not contain "sign" or "light"** (CARLA's importer
   filters them; "JpSignal" contains "sign").
7. **Keep-left = `netconvert --lefthand`, never `rule="LHT"`** (the stamp reverses every lane).
8. **No building (balconies, eaves included) over a road**; no floating wheels (the kerb
   collider is the visible mesh); markings in the right places; CARLA's own signs/lights
   where CARLA has them; local look with facade depth.
9. **Google Maps / Street View only as a visual reference**, never as textures or data.
10. **Send a picture after each step** (Blender preview or CARLA camera shot), and give
    live updates as each stage / gate finishes.
11. **Bilingual docs:** rig docs, comments and user-facing strings are EN + JA (result
    figures English only).
12. **Map guide chapters stay under 100 lines** (split or condense; add a change-log line).
13. **Change the generator, not its outputs** (`maps/osm_city/*.py`, `maps.json`), then rebuild.
14. Deferred work (Pasir Ris large map) starts only when the user asks.

## 4. How to work efficiently here (the method)

1. **Read before acting:** the guide chapter for the topic (index:
   `map_guide/README.md`, `car_guide/README.md`), `hil_sim/FILES.md`, the relevant memory
   notes, and the docstring of every script you will run (they document inputs, outputs,
   gates). Symptom tables: the "Common failures" section of each `SKILL.md`.
2. **Develop on a scratch copy** of the build dir (`cp -a maps/build/<Name> <scratch>/<Name>`),
   iterate there with single steps, render previews, run the checks; then **ONE full
   `maps/make_osm_city.sh <key> --lefthand --no-import`** rerun on the real build dir.
3. **Know the caches:** core_fbx's CUT step (embedded-model maps, about 90 min) is cached
   in `build/<Name>/core/stamp.json`, keyed on its code, the model, bounds, x_scale and
   **lanes.json's mtime** (a road-logic change reruns it); the JOIN step is not hashed and
   always runs. The navmesh (`nav_cache/<sha1>.bin`, about 2 h on OitaKosen) is keyed on the
   road meshes. Plan edits so they do not invalidate a cache needlessly.
4. **Control builds:** when a change has side effects, build once with the change turned
   off (a no-op via `sitecustomize.py` or a flag) and diff the reports, so every difference
   is attributed.
5. **Render previews for every step:** `blender -b -P maps/osm_city/blender_build.py -- build/<Name> --render name:x,y,z:tx,ty,tz`
   (map frame, y north; aerial, driver's eye on an approach, kerb close-up at 0.45 m).
   Look at the images yourself and send them to the user.
6. **Long runs in the background**, logged to a file, watched by PID (`kill -0 <pid>`),
   never by name. Don't poll in tight loops.
7. **Import only after asking** (it stops CARLA): `bash maps/build_map.sh <key> <ABSOLUTE build dir>`;
   check `Success - 0 error(s)` and `grep -c "already exists"` = 0. It clears
   `~/carlaCache/0.9.16/<Pkg>/Maps/<Map>` (clients cache xodr/navmesh/TM and only check
   that a file exists). **xodr-only change:** copy `<Name>.xodr` to
   `CarlaUE4/Content/<Pkg>/Maps/<Name>/OpenDrive/` (and the source tree's Content), delete the
   carlaCache entry, reload the map: no import, no cook. **BP-only change:** build_map steps
   3-4 (cook + install, about 1 min).
8. **After an import:** relaunch through the launcher (ask first), then live-check
   (traffic moving, no wrong-way, parked cars, buses, signals, walkers; camera shots)
   with the read-only scripts in `hil_sim/maps/*_live.py`.
9. **Record every lesson:** the right guide chapter + its change log (ch. 38 for maps,
   car_guide ch. 11 for cars), the startup guide when a feature changes, and a memory
   note for user preferences/facts (same frontmatter format; add it to `MEMORY.md`).
10. Report honestly what was verified (with evidence) and what was not.

## 5. The map workflow (skill carla-map-generator)

Authoritative: `/home/inouelab/.claude/skills/carla-map-generator/SKILL.md` (read it fully).
Stages, **each followed by the gate** (agent-checker: APPROVED / REJECTED):

1. Main session: inputs (OSM ≤ ~1 × 2 km, driving side, package, CamelCase name), `maps.json`
   entry, flat baseline build, offline validation of the road logic.
2. road-builder (photo-getter on call) → gate.
3. road-painter → gate; road-sign-creator → gate; signal-builder and traffic-rules-builder → gate.
4. general-builder → general-painter → gate.
5. Per landmark (biggest first, then the rest in parallel scratch copies): photo-getter →
   gate → building-builder → gate.
6. road-checker → gate; charge-station-builder → gate.
7. Main session: FBX, import (ask), live check, guide + skill update.

**Roles.** Each role is defined in a Markdown file (frontmatter `name`/`description`, then the
role's instructions and checklist) under `/home/inouelab/.claude/skills/carla-map-generator/agents/`:

| Role (file `<role>.md`) | Job / checklist highlights |
|---|---|
| road-builder | road logic (netconvert from OSM), lane counts/widths vs reality, terrain elevation, bridges, dividers, bus bays |
| road-painter | markings per country (ch. 4, 32), asphalt/kerb/pavement materials |
| general-builder | non-landmark buildings in the country's style (ch. 10, 13, 14), on terrain, off roads |
| general-painter | per-building textures/colours from Street View samples, weathering |
| photo-getter | browser: zoom-20 satellite tiles with scale, Street View every side, Google Earth orbits, owner's plans; saves an index |
| building-builder | landmark trace `<site>_trace.json` → `osm_city/landmarks/<site>.py`, register in build.py, previews |
| road-sign-creator | real signs: JARTIC type D (lon/lat) + OSM + Street View; CARLA 206/205/274 in the xodr, Japanese plates modelled; type C signal timing (ch. 18) |
| traffic-rules-builder | NPCs follow the country's rules via the per-country TM profile, only on that country's maps (ch. 23) |
| signal-builder | the country's signal model as a CARLA TrafficLight BP (BP_JpShingo, ch. 24) |
| road-checker | `check_roads.py` site + whole map, loops/driveways clear; PASS / FAIL with the code responsible |
| charge-station-builder | surface grid, LastMile charging-station spots, live charge checks (ch. 34) |
| agent-checker | **the gate** |

**Playing a role in Antigravity:** read the role file and do its job yourself, or start a
sub-agent (`invoke_subagent` / `define_subagent`) whose prompt is the role file's text plus
the task, the build dir, and what was already ruled out (sub-agents do not inherit your
conversation). Do not copy Claude's `tools:` lists into Antigravity agent files (unknown tool
names can hang a sub-agent).

**The gate is explicit:** after each stage, run agent-checker's checklist
(`agents/agent-checker.md`) **against the files themselves**, not against the builder's
summary: open the images, rerun the checks, measure. Output `APPROVED`, or `REJECTED` with a
numbered list (file / line / image, what must change). Nothing moves on without APPROVED.
Prefer an independent reviewer (a fresh sub-agent) over self-review.

## 6. The car workflow (skill carla-car-importer)

`/home/inouelab/.claude/skills/carla-car-importer/SKILL.md`; roles in its `agents/`:
**car-surveyor** (unpack safely, survey in Blender, cars.json entry, `prepare_car.py` dry-run
into scratch, previews) → **car-checker** gate (survey + dry-run) → **car-builder**
(`bash cars/build_cars.sh` with NO keys: the install replaces the whole UserCars package; CARLA
stopped, ask first; check every step's `OK` and the cook's `Success - 0 error(s)`; live checks
with camera shots, eye tuning) → **car-checker** gate (live evidence). Heavy NPC vehicles
(buses): car_guide ch. 16.

## 7. Checks and their pass criteria

Run from `hil_sim/maps/` (`V=.venv/bin/python`, `C=../../../carla-venv/bin/python`):

| Check | Command | Pass |
|---|---|---|
| nothing on/over a road | `$V osm_city/check_roads.py <osm> build/<Name> [--site NAME]` | PASS (exit 0; the chain stops otherwise); build.py prints `0 low triangles over the carriageway` |
| drivability | `$V osm_city/check_drive.py build/<Name> --json build/<Name>/check_drive.json` | 0 spots in **every** class: steps, above, below, tilt, holes, sheets, blocked |
| walkable network | `$V osm_city/check_paths.py build/<Name>` | every crossing end on the network (exit 0); few pieces |
| markings | `$V osm_city/check_markings.py <osm> build/<Name>` (+ `check_stop_poles.py`) | 0 problems |
| kept walls (embedded core) | `$V osm_city/check_keep_walls.py build/<Name>` | no driving lane within 0.3 m (exit 0) |
| signal poles | `$C osm_city/check_signal_poles.py build/<Name>` | all within 0.05 m of carla.Map |
| road logic offline | `$C -c "import carla; m=carla.Map('X', open('build/<Name>/<Name>.xodr').read()); ..."` | waypoints load, runaway segments 0 (SafeRoutePlanner), every light tied to a junction, keep-left side checked geometrically (ch. 3) |
| buses | from `hil_sim/`: `../../carla-venv/bin/python bus_schedule.py report --xodr maps/build/OitaStation/OitaStation.xodr` | 0 unmatched trips; read the remarks (passed stops, loops) |
| lane-change marks | `build/<Name>/lane_change.json`, `tests/test_lane_change_jp.py` | marks per class as painted |

**Tests** (no CARLA needed): `cd hil_sim && for t in tests/test_*.py; do ../../carla-venv/bin/python $t || echo FAIL $t; done`
(`test_keyboard_control.py` section 1 fails while a G923 is plugged in: expected). Model-side
tests: `maps/.venv/bin/python maps/osm_city/test_*.py` and `maps/osm_city/landmarks/test_*.py`.
UE-side tests `maps/ue_jp_*_test.py` run in the editor commandlet (see their headers).
**Venvs:** `carla-venv` for anything that imports `carla` (road logic, lanes, live checks,
tests); `maps/.venv` for model steps. Ubuntu has `python3`, no `python`.

## 8. Traps that cost hours (learn them)

- **Stale caches:** a TM cache `Content/<Pkg>/Maps/<Map>/TM/*.bin` from an older xodr segfaults
  every TM client (none is installed now); clients' `~/carlaCache/0.9.16/...` keeps old
  xodr/navmesh/TM files — clear it on every re-import.
- **Two maps.json entries with the same `name`: the FIRST wins** (`oita_station` must stay before
  the rollback `oita_station_v8`).
- **Blender:** a bake leaves `matrix_parent_inverse` (a child became 100× too big): set it and
  `matrix_basis` to identity, `view_layer.update()` after `data.transform()`. Build rigs in cm.
- **A duplicate module constant** (a second `KEEP =` in core_merge) silently dropped every core
  building. One definition per name.
- **A raster no-build mask stair-stepped lot edges** on angled streets: use exact polygon unions.
- **Map load timeout:** `PythonAPI/util/config.py` waits only 10 s; big maps need
  `run_pipeline.load_world_map` (120 s + confirm), else every client times out.
- **Debug lines are self-lit and bloom:** send channel values ~1-3 (or pure black); 255 whites
  out the camera. `draw_string` never shows in a camera image.
- **Sync worlds:** a new client sees no actors until `world.wait_for_tick()`; a just-spawned
  vehicle reads (0,0,0) until the next tick. Never call `world.tick()` from a helper (one tick
  master: generate_traffic.py).
- **netconvert writes no `laneChange`** (CARLA reads Both everywhere): `lane_change_jp.py` fixes
  Japan maps. netconvert lights float 5 m up (`fix_xodr_signals.py`); service roads exclude cars;
  `access=no` ways are dropped.
- **UE commandlets segfault on exit after saving:** judge them by their own "saved"/"done"/`OK`
  log lines, not the exit code. `EditorLevelLibrary.spawn_actor_*` segfaults without a viewport.
- **Import.py's `get_xodr_crosswalks` traceback** (`No module named 'carla'`) is harmless.
- FBX2OBJ is missing: the navmesh is made by `nav_obj.py` + RecastBuilder (build_map step 2c);
  0 walkers with no error = no `Nav/<Map>.bin`. In the OBJ write all vertices before `usemtl`.
- PrepareAssetsForCooking crashes at random ("Cast of nullptr to OpenDriveActor"): rerun.
- Road and pavement both raycast as label NONE on our maps: find kerbs by the height step or the
  surface grid. CARLA's sign-based speed limit stays 30 on keep-left roads (`lane_limits.py`).
- `carla.Transform.transform(loc)` mutates `loc`. CARLA's GlobalRoutePlanner runs away on our
  maps: use `delivery/route.py` SafeRoutePlanner; test new map code under `ulimit -v`.
- A bare `./CarlaUE4.sh` renders on the Intel iGPU: always use PRIME offload (run_pipeline does).
- Helpers get 3 restarts per pipeline run; don't burn them.

More: map guide ch. 35 (embedded core), ch. 7 (import), ch. 8 (live traps), and the memory notes.

## 9. Current state (2026-11-09)

OitaStation extended map imported 2026-11-09 (v8 core inside a generated city to ±640 m); all
12 own cars rebuilt incl. the NPC-only Oita bus `vehicle.oitabus.erga`. Open items and the
timeline: `CARLA_RIG_SETUP_FROM_SCRATCH.md` sections 9.2 and 10.

## 10. RL for the pods and drones (`hil_sim/rl_delivery/`)

Read `PythonAPI/hil_sim/docs/RL_DELIVERY.md` fully before touching it (skill stub:
`.agents/skills/rl-delivery`). Essentials:
- Brains: pod + drone decision brains (fleet world `env_fleet.py`, trained in turns), pod
  driving (`env_drive.py`, SAC), drone navigation (`env_nav.py`, RecurrentPPO). Mission and
  flight brains are retired.
- User rules: **nothing hardcoded** (every behaviour learned; rule/scripted/PID code only as
  baselines), **pure RL, no geofence / emergency stop**, pretend world first, agents may fail,
  hidden realistic wind, OitaKosen first, check the trainings every 30 min and fix stuck or
  exploiting brains with world rules / rewards (never steering).
- Roles (user decision, 2026-11-11): **Claude Code is no longer running. Antigravity manages both
  the 30-minute checks and trainer execution directly** (start, stop, monitor, evaluate).
- One trainer per brain (`ps -eo pid,etime,args | grep "[r]l_delivery.train"`); stop with
  `touch rl_delivery/results/STOP_<brain>`, carry on with `--resume`; trainers die with the
  agent session. Dashboard AI training page = status, charts, watch-mode replays.
- No CARLA bridge yet (plan: `PythonAPI/hil_sim/docs/RL_CARLA_BRIDGE.md`); CARLA tests need the demo stopped:
  ask the user.
