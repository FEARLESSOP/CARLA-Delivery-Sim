# 03: OitaKosen dead-end U-turn loops: handoff to finish (fix3)

**Status: COMPLETED & IMPORTED 2026-11-05 17:30.**
Code, scratch build, review defect fixes (smooth heads j291, j296/j298, j308, j310; dropped lamps 13; guide ch. 21 & 22; speed signs verified), FBX, and Unreal Engine cook/import are completed and live in CARLA.

All paths are on the user's PC (Ubuntu, user `inouelab`).
- `$H` = `~/Downloads/CARLA_0.9.16/PythonAPI/hil_sim`
- `$M` = `$H/maps`
- `$S` = the old Claude scratch folder
  `/tmp/claude-1000/-home-inouelab-Downloads-carla/2fc4bb2b-7fe4-48d7-a21a-167a8395953c/scratchpad`.
  It may be gone after a reboot; nothing essential lives only there except the review renders and scripts noted below.

---

## 1. Background: what the rig is

- CARLA 0.9.16, packaged, at `~/Downloads/CARLA_0.9.16`. A driving-simulator rig with a Logitech G923 wheel lives in `$H`.
- Generated city map **OitaKosen** (Japan, keep-left). It is built from OpenStreetMap by the generator in `$M/osm_city/`, through these steps:
  - netconvert road logic (OpenDRIVE `.xodr`);
  - a 3-D model as numpy meshes;
  - Blender FBX export;
  - Unreal 4.26 import, cook and install (`$M/build_map.sh`).
- The full manual for the rig is `~/Downloads/CARLA_0.9.16/CARLA_RIG_SETUP_FROM_SCRATCH.md`.
- The map knowledge base is `~/.claude/skills/carla-map-generator/map_guide/`. Read `README.md`, then ch. 03, 04, 07, 08, 11, 12 and **21 (road edges, dead ends, guardrail)**.
- The skill rules are in `~/.claude/skills/carla-map-generator/SKILL.md`.

### User's rules (must hold)
1. No building, balcony, canopy or pole over a road (`check_roads.py` must PASS).
2. No floating wheels, and no drops or folds at paving edges. Kerbs are smooth.
   **The user hates "random squares", notches and jagged edges at road ends.**
3. Correct markings. Japanese plates and JARTIC signs stay as they are now.
4. Keep-left (`--lefthand`; never stamp `rule="LHT"`).
5. Mesh and material names must never contain "sign" or "light": CARLA's importer drops the whole mesh.
6. Never commit or push unless the user asks.
   Ask the user before stopping CARLA / `run_pipeline.py`.
   Never run a second CARLA server.
   Never use `pkill -f` / `pgrep -f`; find PIDs with `ps -eo pid,args | grep "[x]yz"`.
7. The user wants build work to run at **higher** priority than CARLA. Do not `nice` builds; renice the CARLA processes to +5 instead, if needed.

---

## 2. The problem this task fixes

- Traffic Manager (TM) cars got stuck at dead ends.
- Cause: every netconvert dead-end turnaround connector was drawn **inside out**.
  - It started about 2.8 m sideways from the street lane, heading about 151° reversed, and looped back over the street.
  - TM steered into the kerb. A plain full-lock turn fitted, but TM couldn't follow the lane.
- Fix (built in this task):
  - a forward **three-arc loop** per dead end: swing-out radius ≥ 4.6 m, loop radius 5 / 4.5 / 4 m, swing-back;
  - G1-continuous with the street lanes;
  - a turning "head" paved around the loop.

### History so far (already live in CARLA, keep it)
- Queue 02 (U-turn ears removed with `--no-turnarounds.except-deadend`, ring guardrail) and fix2 (round heads, nothing paved outside the ring) are imported.
- The live build dir is `$M/build/OitaKosen`, which was fix2.
- Previous builds are kept as `build/OitaKosen_pre_edges` and `build/OitaKosen_pre_fix2`.

---

## 3. Current state of fix3 (scratch build `$M/build/OitaKosen_fix3`)

### Code (shared modules, modified, uncommitted)
Originals are in `$S/fix3/osm_city_orig/` and `$S/fix3/maps.json.orig`.

| File | What fix3 changed |
|---|---|
| `osm_city/turnloops.py` (new) | Called by `roadnet.py` on `road_fixes` maps. Replaces each dead-end connector with a forward loop and searches its position and radius. It has a `--refine` branch that moves loops away from lanes at another height (`APART_DZ`) to keep `layout.HEAD_CLEAR`. It writes `meta.json` `dead_end_loops` and `dead_end_walled`. |
| `osm_city/roadnet.py` | Calls turnloops. |
| `osm_city/layout.py` | `dead_end_heads`: the head is the loop plus a 2 m margin, rounded, plus the street's last 10 m. Other-height roads are clipped at `HEAD_CLEAR`. "Walled" heads keep `WALLED_MARGIN` 1.2 m of paving down to `HEAD_CLEAR_MIN` 2.0 m from the other road, with a guardrail. Inside landmark sites (+3.7 m) the head stays the plain old hull. |
| `osm_city/props.py` | Lamps near heads (`LAMP_HEAD_NEAR` 3 m) are placed `LAMP_HEAD_CLEAR` 1.0 m behind the kerb, otherwise slid or dropped. Seasonal-tree road checks. |
| `osm_city/buildings_jp.py` | The shed fallback applies only to blocks beside a head (`NEAR_HEAD` 6 m, < 150 m²). |
| `osm_city/build.py` | Log lines (lamp moves). |
| `maps/maps.json` | `oita_kosen.osm_fixes`: **9 stubs with no room for a loop retagged as driveways** (ways 192089559, 192089561, 192089569, 192089598 (split), 197011617, 197011634, 197011653, 202926863, 202926868 (split)). This renumbers every road and junction id. |

### Scratch outputs
Latest, all 11-05:

| File | Time |
|---|---|
| `OitaKosen.xodr` | 14:34 |
| `build.log` | 14:32 |
| `check_roads.log` | 14:37: **PASS** |
| `signs.log` | 14:34 |
| `OitaKosen.fbx` (474 MB) | 14:43 |

- Renders are in `previews/fix3b_*.png`.
- **These outputs match the current code.** Nothing changed after 14:43.

### Already approved by the reviewer (don't break)
- 74 loops all forward: join angles ≤ 11.6°, no lane end without a successor, the road graph strongly connected (2942/2942 lanes).
- JARTIC: 74 stop landmarks (type 206), 24 lights, `signal_phases.conflicts() == []`, `jp_signs` 0 plate problems, 1 post behind the guardrail.
- Building types back to normal (mansion 11, shed 29, apaato 15). 34 houses lost to heads: accepted.
- Landmark "cut back" lines the same as the live build or better.
- Clean heads: j336 (≈ map (274,-89)), j131, j273, j175, j279, j205, j224.

---

## 4. Remaining work (the reviewer's REJECT items): do these

Head ids below are fix3 ids; positions are map frame (x east, y north). The CARLA frame is the same x with y negated.

### 4.1 Jagged drops and folds at head edges (BLOCKER)
| Head | Map pos | Defect (render) |
|---|---|---|
| j291 | (-293, 476) | Spiky folded terrain wedge behind the head kerb, with a grey spike on the kerb (`previews/fix3b_j291.png`). Marked "refined" in turnloops, but the ground isn't smoothed. |
| j308 | (46, 506) | Same terrain fold behind the head; the footpath runs into it (`fix3b_j308.png`). |
| j296 + j298 | ~(215–223, 451–463) | Two heads touch at **different heights**: a torn zig-zag kerb seam with dark faceted triangles on the asphalt (`fix3b_j296.png`, `fix3b_j298.png`). Neither is in the refine list or `dead_end_walled`, because the refine check doesn't treat a neighbouring head as "a road at another height". |
| j310 | (40, 392), walled | A wavy kerb strip runs across the paving where the head meets the road above. The guardrail is on one side only (`fix3b_j310.png`). |

Do:
1. In `turnloops.py` (refine) and `layout.py` (head clipping), treat **other dead-end heads/loops at a different height (> 1 m)** as obstacles, just like other-height roads. Either separate them (shift a loop sideways / pick another radius) or make one a walled head.
2. Behind every head kerb, make the ground a **smooth graded verge**: no spikes, no folds. If the height difference can't be graded within the space, use a **kerb + retaining wall (+ guardrail if the drop > 1 m)**. The terrain draping code is `osm_city/terrain.py` (Drape) plus the ground step in `osm_city/ground.py`. See map_guide ch. 11, 12, 15 and 21.
3. j310: the walled side needs a continuous wall/guardrail along the whole drop, and the head paving must not cross the upper road's kerb line.
4. Render **driver-eye and aerial views** of j291, j296/j298, j308 and j310 (see section 5). Show no fold, spike or torn seam.

### 4.2 Lamps
- 36 lamps are left out (live build 17; target < 30). In `props.py`, before dropping a lamp, slide it along the kerb beyond 8 m or put it on the opposite verge.

### 4.3 Guide ch. 21 must match what is built
- `~/.claude/skills/carla-map-generator/map_guide/21_road_edges_and_barriers.md` is out of date: it says "19 dropped", and "walled heads clipped at the full HEAD_CLEAR".
- Rewrite it to describe:
  - the forward loops;
  - `HEAD_CLEAR`, `WALLED_MARGIN` and `HEAD_CLEAR_MIN`;
  - the neighbour-head rule;
  - the lamp rule;
  - the 9 driveway retags;
  - the numbers.
- Keep **every chapter under 100 lines**: split if needed, update `README.md` and the `20_change_log_2.md` entry.

### 4.4 Speed signs (type 274)
- The live build has 19; fix3 has 20.
  - New: a 40 km/h sign at map (142.3, 163.2) [CARLA (142.3,-163.2)] on road 3631.
  - Moved: one sign went from CARLA (27.6,-136.0) to (12.0,-166.5).
- Probably a side effect of the stub retags (roads merged or split). Check each against the JARTIC data:
  - `build/OitaKosen_fix3/jartic_d.json`
  - the `jartic_log` in `jp_plates.json`
  - OSM `maxspeed`
- Keep only signs backed by data, and document the reason.

---

## 5. Commands

### Python environments
- Model steps: `$M/.venv/bin/python` (numpy, shapely ≥ 2.1, pillow, mapbox_earcut).
- Road logic and offline CARLA map checks: `~/Downloads/CARLA_0.9.16/carla-venv/bin/python`.
- Blender is 4.x (`blender`).
- OSM file: `"/home/inouelab/Downloads/oita kosen/map_kosen_extended.osm"`.

### Rebuild the scratch build (from the road logic, because turnloops/roadnet changed)
**Do NOT run `make_osm_city.sh oita_kosen` as it is.**
- It always writes to `build/OitaKosen` (the LIVE build dir, line 36).
- Without `--no-import`, it also runs `build_map.sh`, which stops nothing but imports into CARLA.

Run its steps by hand into the scratch dir. This is the same order as `make_osm_city.sh` lines ~38-70:
```bash
cd ~/Downloads/CARLA_0.9.16/PythonAPI/hil_sim/maps
CPY=../../../carla-venv/bin/python; VPY=.venv/bin/python
OSM="/home/inouelab/Downloads/oita kosen/map_kosen_extended.osm"
OUT=$PWD/build/OitaKosen_fix3          # absolute path
# 1 road logic (OitaKosen: keep-left, road_fixes, road_style japan)
$CPY osm_city/roadnet.py "$OSM" $OUT OitaKosen --lefthand --road-fixes --road-style=japan
$CPY osm_city/lht_poles.py $OUT/OitaKosen.xodr $OUT/OitaKosen.xodr
$CPY osm_city/lanes.py $OUT/OitaKosen.xodr $OUT/lanes.json
# 2 terrain + elevation (GLB list and --sw come from maps.json oita_kosen
#   "terrain_glb" / "terrain_sw"; copy the exact python3 -c snippets from
#   make_osm_city.sh lines 32 and 53 to build GLBS and SWS)
$VPY osm_city/terrain.py $OUT --glb "${GLBS[@]}" --osm "$OSM" --sw "${SWS[@]}"
$VPY osm_city/elevate.py $OUT
$CPY osm_city/turnloops.py $OUT/OitaKosen.xodr --refine      # the dead-end loops
$CPY osm_city/lanes.py $OUT/OitaKosen.xodr $OUT/lanes.json
# 3 model, signs, checks, FBX
$VPY osm_city/build.py "$OSM" $OUT
$VPY osm_city/signs.py $OUT
$VPY osm_city/jp_signs.py "$OSM" $OUT
$VPY osm_city/check_roads.py "$OSM" $OUT                     # must print "road check: PASS"
blender -b -P osm_city/blender_build.py -- $OUT --fbx
```
- If only model code changed (layout/props/terrain/ground/buildings), you can start at step 3.
- If only `turnloops.py` changed, re-run from step 1, or at least from `elevate.py`. The xodr has to be regenerated before turnloops runs again.
- Read `make_osm_city.sh` once to confirm these flags still match.

### Renders
```bash
blender -b -P osm_city/blender_build.py -- $OUT --render name:x,y,z:tx,ty,tz
# map frame, y north; camera position then target. Driver-eye: z = ground + 1.2
```

### Offline road-logic checks (carla-venv)
```python
import carla
m = carla.Map("OitaKosen", open(OUT + "/OitaKosen.xodr").read())
wps = m.generate_waypoints(1.0)
# every lane has a successor; follow each dead-end loop with wp.next(1.0)
# from the street lane into the opposite lane; heading step <= ~20 deg/m
m.get_all_landmarks_of_type("206")   # expect 74 stops
m.get_all_landmarks_of_type("274")   # speed signs
```
The reviewer's loop check script is `$S/chk/loops.py` (if `$S` still exists).
`osm_city/signal_phases.py` provides `conflicts()`, which must return `[]`.

### Acceptance checklist (all must pass before import)
- `check_roads.py` PASS; `build.log` says `0 low triangles over the carriageway`.
- 74 forward loops, no dead lane ends, one strongly connected component.
- 74 stops, 24 lights, `conflicts() == []`; `jp_signs` 0 problems; no post within 0.2 m of the guardrail; the guardrail continuous (`ring guardrail: 4436 m, 0 openings`).
- No drop, fold, spike or torn seam at any head (renders of the 4.1 heads, plus spot checks).
- Lamps left out < 30; building types unchanged vs the live build (mansion 11, apaato 15–17, shed ≈ 29).
- Landmark "cut back" lines in `build.log` the same as `build/OitaKosen/build.log` or better.
- Every speed sign backed by data.
- Guide chapters < 100 lines and matching the build.

---

## 6. Import into CARLA (only after the user OKs stopping CARLA)

1. **Promote the scratch build to the main build dir:**
   ```bash
   cd $M/build
   mv OitaKosen OitaKosen_pre_fix3 && mv OitaKosen_fix3 OitaKosen
   for d in dorm_photos estate_photos kosen_photos road_photos seven_area_photos seven_photos sign_photos nav_cache; do
     [ -e OitaKosen/$d ] || cp -al OitaKosen_pre_fix3/$d OitaKosen/$d; done
   cp -a OitaKosen_pre_fix3/kosen_trace_src.py OitaKosen/ 2>/dev/null
   ```
2. **Navmesh:** the road meshes changed, so the cache key changes and `build_map.sh` step 2c would recast for about 2 h.
   - To keep the import fast, compute the key with the same hash code as `build_map.sh` (lines ~91–102: sha1 of the Road_*/Terrain_Street npz V/I arrays).
   - Copy the closest existing bin to `OitaKosen/nav_cache/<key>.bin` (the newest in `nav_cache/`), and note it is stale.
   - Later, run the exact recast: `osm_city/nav_crosswalks.py`, then `nav_obj.py`, then `RecastBuilder` (`~/Documents/Dinesh-carla-copy/carla/Build/recast-c10-install/bin/RecastBuilder`). Install the bin at the next restart.
3. **Stop the pipeline:** `kill -INT <pid of run_pipeline.py>`, then wait for `CarlaUE4-Linux-Shipping` to exit.
4. **Import:** `cd $M && bash build_map.sh oita_kosen "$M/build/OitaKosen"`.
   - Expect `Success - 0 error(s)` and `[ue_foliage] N leaf meshes`.
   - The "No module named carla" and "BaseMap.umap failed to save" lines are harmless.
   - `build_map.sh` no longer installs the TM cache. The road ids changed, and an installed TM cache segfaults every TM client (rc -11). Make sure `CarlaUE4/Content/Japan/Maps/OitaKosen/TM/` is empty.
5. **Restart with the user's settings:**
   ```bash
   cd $H && DISPLAY=:1 PYTHONUNBUFFERED=1 setsid nohup python3 run_pipeline.py --map OitaKosen \
     --vehicles 10 --walkers 10 --delivery 3 --ffb --no-sensor-viewer \
     --start-vehicle vehicle.mazda.rx7 --res 1080p --drone-cams > /tmp/pipeline_kosen.log 2>&1 < /dev/null & disown
   ```
   Wait for `spawned 10 vehicles`. If `generate_traffic`/`manual_control` die with rc -11, move `TM/*.bin` aside and restart.
   This restart also activates the new **dashboard Keybinds page** (`http://localhost:5000/#settings-keybinds`), which is already built. The user must press each wheel button once in identify mode.
6. **Live checks** (the world is synchronous: call `world.wait_for_tick()` before `get_actors()`):
   - Spawn test cars with their own `role_name`: the scratch unstick watchdog teleports "autopilot"/"hero" cars.
   - TM-drive a car through 4–5 dead ends, including j243 (live j245, ≈ map (167,-373)), j336 and j279; it must complete the U-turn.
   - Watch for unsticks over 10 min.
   - Spot-check heads from driver eye.
7. **Record:** map_guide ch. 21 plus the change log, `maps/queue/03_*.md` status, and `CARLA_RIG_SETUP_FROM_SCRATCH.md` (a short note). Commit or push only if the user asks. Repos: `FEARLESSOP/CARLA-Delivery-Sim` (rig code; cooked content stays out of git) and `FEARLESSOP/carla-skills` (the git root is `~/.claude/skills`).

---

## 7. If you need to roll back fix3
- Restore `osm_city/` files from `$S/fix3/osm_city_orig/` and `maps.json` from `$S/fix3/maps.json.orig`, or use `git diff`/`git checkout` in `~/Downloads/CARLA_0.9.16`. The last rig commit is `e911cb0`, which is pre-fix3.
- The live CARLA map stays fix2 until an import.
