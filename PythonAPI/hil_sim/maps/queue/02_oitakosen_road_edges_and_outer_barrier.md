# 02: OitaKosen road-edge "squares" + outer barrier

**Status: DONE (imported and verified live 2026-11-04 23:00; follow-ups: junction 245 pinch, east-ring outside stubs).** Was queued: Start automatically **after the car import
(Ferrari SF90 / Honda S800 / Rolls-Royce Dawn, LHD→RHD) is built, verified
and reported**. Use the carla-map-generator skill and its agents.

## User's request (verbatim)

> ive notice on the roads there seem to be some random squares on the side
> of each end of the road. i have placed the screenshots of what im talking
> about in /home/inouelab/Pictures/Screenshots/claudeview
>
> i want to know what it is, and remove them, i dont like it. the road edges
> should be smooth without random stuffs like that
>
> also i want you to add a barrier to the outter road edge. so that the cars
> cant go the the huge grass field on the outter edge

## Evidence

Screenshots in `/home/inouelab/Pictures/Screenshots/claudeview/` (5 files,
14:57-15:02):
- Kerb and grass lines step out and in with **rectangular notches/bulges**
  where a road meets a junction, at stop lines, crossings and the 止まれ
  mouths.
- Seen at junction ends, both sides, and around the roundabout/central
  island. The kerb and verge follow a square outline instead of a smooth
  edge.

First guesses, to confirm before changing anything:
- netconvert junction-polygon corners or short stub roads (the 0.2 m stubs
  seen at cluster 54/55/252), with the kerb (lanes.py / ground.py) tracing
  the junction outline;
- crossing or stop-line aprons, or the drive band (layout.py) squaring the
  edge;
- bus-bay / dropped-kerb cut-outs.

Find which code makes them. Reproduce them on a Blender preview at one
screenshot spot first.

## Tasks

1. **Explain** to the user what the squares are (with a before render).
2. **Remove them.** Road edges, kerbs and verges must be smooth through
   junction ends:
   - fix the generator (kerb and outline smoothing, stub handling), not the
     outputs;
   - keep road clearance, markings and crossings correct;
   - road-checker must PASS, and agent-checker gates the stage.
3. **Outer barrier:** add a continuous barrier (Japanese guardrail style,
   ガードレール: white steel beam on posts, or the local style seen in
   imagery) along the **outer edge of the ring road**, so cars cannot leave
   onto the big grass field outside.
   - It needs collision in CARLA: check that the barrier mesh gets collision
     (not a decal-only kind).
   - Openings only where real roads leave the map, if any. Nothing on the
     carriageway (road-checker).
4. **Rebuild** with the model chain, then preview. Ask the user before the
   import (it closes CARLA for about 25 min). Import, verify live with shots
   at the screenshot spots, and a car driven at the outer edge must be
   stopped by the barrier.
5. **Record** it in map_guide (new lesson plus the change log) and in memory.

## Notes
- The walker navmesh is stale since the akeno import. If the road/kerb
  meshes change, the navmesh hash changes: let build_map.sh rebuild it
  (about 2 h), or use the background recast in scratch `recast_akeno/`
  after a rehash.
- The floating spawn points (about 17/973, invisible Terrain collision above
  the road) may be related to the same edge/terrain code: look at them too.
