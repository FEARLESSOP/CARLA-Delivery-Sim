# Build brief: Pasir Ris (large) — Singapore

Status: **DEFERRED** (the user took it off the queue on 2026-11-04: "i want to
do that another time"). Start only when the user asks. This file is the
starting point then; the original request is in `01_pasir_ris_large.md`
(same folder). Use the skill `carla-map-generator` and its agent team.

## 1. What the user asked for (2026-11-02, summary; verbatim text in the queue file)
- A big, **flat** map of Pasir Ris, Singapore, keep-left, as a test of how far
  the skill can go. Use everything learnt so far; update the skill and the
  guide with anything new.
- Detailed landmarks: **Downtown East**, **White Sands** shopping mall, **all
  of Pasir Ris Street 11, especially blocks 130-136**, **Coral Primary
  School** (demolished: rebuild it from 2016-2018 photos for the exterior
  colours).
- **OneMap** (https://www.onemap.gov.sg/) 3-D buildings for **shapes only**
  (footprints, heights, massing) — never download/rip its models or textures;
  infer textures, add detail. Landmarks: OneMap + Google Earth + Street View +
  any images.
- The user will be AFK while it builds: answer questions yourself, finish it,
  they review and ask for changes after.

## 2. Area and data on disk
- Bbox NW 1.381728, 103.947458 / SE 1.361690, 103.965429: 2.00 km E-W x
  2.23 km N-S (~4.5 km², ~2.6x the existing PasirRis map and ~3x OitaKosen).
- OSM (downloaded 2026-11-03 via the OSM API 0.6 `map` call; Overpass answered
  406 without a User-Agent): `~/Downloads/pasir_ris_large_osm/map_pasir_ris_large.osm`,
  10 MB: 31k nodes, 4.3k ways, 946 buildings; highways: 1009 footway, 929
  service, 224 residential, 197 tertiary, 139 primary, 136 steps, 113
  cycleway, 17 secondary, 13 motorway (TPE) + 10 links; 74 bridge, 169 layer
  tags; **65 `highway=construction` (Loyang Viaduct) — leave out (rule 6)**.
  OSM may be months old by then: re-download if the user agrees.
- Found in the OSM: Pasir Ris Street 11 (39 ways), HDB blocks 109-146 and
  180-187 with car parks 135A, 142A, 180A; `Downtown East`, `D'Resort @
  Downtown East`; `White Sands` (retail); **`Coral Primary School` (school
  polygon still mapped: check what stands there now — the old building is
  gone)**; White Sands Primary; Pasir Ris Crest Sec; Loyang View Sec; Loyang
  Point; Loyang Bus Depot; Sungei Loyang; Civil Service Club @ Loyang.
- Do **not** overwrite `build/PasirRis` (maps.json key `pasir_ris`, OSM
  `~/Downloads/map (1).osm`, lat0 1.370325 lon0 103.959805, 0.9 x 1.9 km). The
  new map lies over almost all of it: new key e.g. `pasir_ris_large`, name
  `PasirRisLarge` (CamelCase, no "sign"/"light"), package `Singapore`,
  `keep_left: true`, `speed_kmh` 50.

## 3. Reuse from the existing PasirRis map
- Landmark modules `osm_city/landmarks/prcs.py` (Pasir Ris Crest Sec) and
  `lvss.py` (Loyang View Sec) with their traces `build/PasirRis/prcs_trace.json`,
  `lvss_trace.json` and photo folders: traces are stored in z20 tile pixels
  (lat/lon tiles), so they carry over to the new frame — copy them into the
  new build dir, rebuild, check positions.
- `colour_notes.json`, `site_features.json`, `colour_samples.json` (Street
  View colour samples per precinct, traced courts/pitches) — copy, then
  extend to the new area.
- Singapore textures/markings/props (HDB void decks, covered linkways, bus
  shelters, Belisha beacons, noise barrier on the TPE): already in the
  generator (guide ch. 4-6).

## 4. Lessons from OitaKosen to apply (guide chapters)
- Flat map: no terrain/drape steps (ch. 11-12 not needed); still run the
  `0 low triangles over the carriageway` and road checks.
- Keep-left via `--lefthand` only, never `rule="LHT"` (ch. 3). Traffic
  hybrid physics **off** on keep-left maps (`keep_left` key in maps.json;
  ch. 8). Hero on `--autopilot` uses its own TM port (ch. 8).
- **TM cars wedge at tight junctions** (OitaKosen open item, ch. 8): pave at
  least 1.5 m past junction lanes, no lamps/trees on islands < ~4 m wide or
  within 1.5 m of a junction lane; measure with a TM fleet before import.
  Gate it behind a maps.json flag so PasirRis stays byte-identical.
- Walker navmesh: build_map.sh step 2c makes it (nav_crosswalks + nav_obj +
  RecastBuilder; ch. 7); check `Nav/<Map>.bin` exists and walkers spawn.
- Pass an absolute build path to build_map.sh (step 2b runs elsewhere).
- Trees: tropical evergreens (rain tree, angsana, palms) — no season swap;
  they can still sway: give real-tree leaves their own objects and the
  foliage material like the maples (ch. 19), budget permitting.
- Landmark workflow: photo-getter → agent-checker → building-builder →
  agent-checker per landmark, biggest first, the rest in parallel scratch
  copies (memory `landmark-build-order`); a user photo beats GE (ch. 17).
- Reference models from the user are study-only (ch. 19).
- Road signs: road-sign-creator (ch. 18). JARTIC is Japan-only; for Singapore
  use OSM `traffic_sign`/`highway=stop|give_way`, Street View, LTA practice
  (check whether LTA DataMall has usable sign/signal data before assuming).

## 5. Landmark notes to start from
| Landmark | Notes / sources |
|---|---|
| Downtown East / D'Resort | large leisure complex (E!hub mall, D'Resort chalets/hotel blocks); OneMap massing, GE, SV round the site; the drop-off loops count as road (road-checker) |
| White Sands | mall by Pasir Ris MRT; massing from OneMap; SV of each face; the taxi stand (OSM `White Sands Taxi Stand`) and bus interchange nearby |
| Pasir Ris St 11, blocks 130-136 (+ whole street 109-146) | HDB slab/point blocks: storeys from OneMap/SV, block numbers on gables, void decks, linkways, MSCPs (135A, 142A), playgrounds, fitness corners |
| Coral Primary School | demolished: Street View history / Google Earth history / photos 2016-2018 for colours and form; OSM polygon gives the site; say clearly in the report what is inferred |

## 6. Risks
- Size: ~2.6x PasirRis (3.3 M triangles there) → expect ~8-9 M triangles,
  FBX ~0.3 GB+, longer import/cook; watch the 30 GB memory cap (route
  planner OOM history: `SafeRoutePlanner`, ch. 3). Consider dropping detail
  outside the landmarks (merge per tile, trees LOD) if over budget.
- TPE motorway + links and bridges/layers (74/169): check grade separation
  in the road logic (ch. 3).
- OneMap: view and measure only; no scraping of model/texture files.

## 7. Checklist when the user says go
1. Re-read this file, the queue file, SKILL.md, guide README; install the
   agents (`cp -n` from the skill's agents/).
2. Ask whether to refresh the OSM download (or use the 2026-11-03 file).
3. maps.json entry (new key), flat baseline build, road logic validated.
4. Agents in order: road-builder → road-painter → road-sign-creator →
   general-builder → general-painter → landmarks → road-checker, each passing
   agent-checker; live updates to the user after each agent.
5. FBX, import (ask before stopping their pipeline), navmesh present, live
   verification (traffic moving, no wrong-way, walkers, pods, landmarks shots).
6. Update the guide (new lessons, change log ch. 20), SKILL.md, memory.
Estimate (2026-11 pace): 12-20 h of agent work, landmarks most of it.
