# Queued map: Pasir Ris (large, flat) — start ONLY after OitaKosen is done

Full build brief: pasir_ris_large_BRIEF.md (same folder).

Queued 2026-11-02 ~17:05 by the user (in chat). Status: DEFERRED (2026-11-04: the user took it off the queue, "i want to do that another time"; start only when they ask again).

## Area (from /home/inouelab/Pictures/Screenshots/Screenshot from 2026-11-02 16-46-04.png)
- NW corner 1.381728, 103.947458; SE corner 1.361690, 103.965429
- 2.00 km (E-W) x 2.23 km (N-S), ~4.5 km2 (about 3x OitaKosen); flat; Singapore, keep-left
- No OSM file given: download the bbox from the OSM API / Overpass (own empty folder, untrusted data)
- Overlaps the existing PasirRis map (build/PasirRis): build as a NEW map, do not overwrite PasirRis

## The user's request (verbatim)
this is a very big map. i want you to use all the knowledge you have so far in building this. the key landmarks i want you do make as detailed as possible is

Downtown East
White Sands Shopping mall
the entirety of pasir ris street 11, especially the blocks 130 to 136 area
Coral primary school (this building doesnt exist anymore as it was demolished. i want you to look at older photos and build this school. use images from 2016 to 2018 for the exterior colours)

i want you to use the onemap Singapore
https://www.onemap.gov.sg/
as this has all of the 3d models for all the buildings in Singapore and its fairly accurate. i only want you to use it for the shape of each of the building models, you should not use it for textures or just rip it from the site. the shape, yes use that and add details, but textures and such infer from it. but the land marks, i want you to use the one map, earth, street view and any images you can find to build and add details to it. i want this map to be flat

all of these should use the CARLA map generator skill

this entire map is a test to see how far the skill can work. if anything new is learn from this, or you want to reinforce the knowledge, update the skill and the md file

also i will be afk, any question you have try to answer it yourself, just get the map done, i will comeback to view the complete product and make changes if i see any i want to change

## Notes for whoever picks this up
- OneMap 3D: shapes (footprints, heights, massing) only, as a reference to measure from; never download/rip its
  models or textures. Landmarks: OneMap + Google Earth 3D + Street View (+ historic imagery for Coral Primary,
  2016-2018) via photo-getter.
- Size: build steps may hit the 30 GB cap; check memory early (roadnet/route planner OOM history, guide ch. 3).
- 2026-11-03 01:54: OSM downloaded (OSM API 0.6 map call; Overpass /api/map answered 406 without a User-Agent):
  ~/Downloads/pasir_ris_large_osm/map_pasir_ris_large.osm (10 MB: 31k nodes, 4.3k ways, 946 buildings,
  3.5k highway tags, 74 bridge, 169 layer tags). Not built yet.
