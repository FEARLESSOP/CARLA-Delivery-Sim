"""Road check: does anything stand on or over a road? (maps/.venv)

  python check_roads.py <osm> <build dir> [--site NAME] [--json out.json]

Reads the built meshes (build.py) and the road network (lanes.json via
layout.py) and reports, per object and material, the triangles whose
centroid lies over the road surface (carriageway + kerb):
  buildings (SM_*)  none over carriageway or kerb at any height (no balcony,
                    canopy or eave over a road)
  everything else   none over the carriageway below 4.5 m (paint and the road
                    itself excepted;
                    tree crowns, lamp arms above 4.5 m are allowed)
On Japanese maps (maps.json "road_style": "japan") buildings and their lots
are also tested against footways, cycleways and drives (user's rule 1 covers
footways); other maps keep the check above unchanged.
On hilly maps (terrain_field.npz) every height is measured from the
road surface / ground below the vertex (terrain.Drape), not from z = 0.
Every triangle is tested whole (vertical faces by their base edge), not
just its centre. With --site, only the area of that OSM-named site (+30 m)
is checked and its own driveways and car-park aisles count as road. Exit status 1 if anything fails.

道路の上・中に建物や物体が無いか確認する（建物は高さに関係なく不可）。
"""

import argparse
import json
import os
import sys

import numpy as np
import shapely

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import layout  # noqa: E402
import terrain  # noqa: E402

CLEAR = 4.5
GROUND = 0.2          # kerb top 0.15 m + paint / 縁石天端＋塗装


def road_style(name):
    """maps.json "road_style" of the map called ``name``. / 道路様式。"""
    try:
        for v in json.load(open(os.path.join(HERE, "..", "maps.json"))).values():
            if isinstance(v, dict) and v.get("name") == name:
                return v.get("road_style")
    except (OSError, ValueError):
        pass
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("osm")
    ap.add_argument("out")
    ap.add_argument("--site")
    ap.add_argument("--json")
    a = ap.parse_args()
    meta = json.load(open(os.path.join(a.out, "meta.json")))
    L = layout.Layout(meta, os.path.join(a.out, "lanes.json"), a.osm)
    # hilly maps: heights count from the road surface / ground below / 起伏のある地図では路面・地面からの高さ
    D = terrain.drape_for(a.out, L)
    road_b = L.road_all.buffer(-0.02)        # buildings: carriageway and kerb / 建物: 車道＋縁石
    if road_style(meta.get("name")) == "japan":
        # Japan: and footways, cycleways, drives / 日本: 歩道・自転車道・私道も
        road_b = shapely.union_all([road_b, L.footpaths.buffer(-0.02), L.cycleways.buffer(-0.02),
                                    L.driveways.buffer(-0.02)])
        print("buildings tested against carriageway, kerb, footways, cycleways and drives")
    road = L.carriageway.buffer(-0.02)       # the rest: carriageway / その他: 車道
    area = None
    if a.site:
        site = [p for p, t in L.osm.areas(lambda t: t.get("name") == a.site)]
        if not site:
            sys.exit("no OSM area named %r" % a.site)
        area = site[0].buffer(30.0)
        road = road.intersection(area)
        road_b = road_b.intersection(area)
        # the site's own driveways and car-park aisles count as road there
        # (buildings over them at any height, others below 4.5 m) / 敷地内の車路も道路扱い
        drive = L.driveways.intersection(site[0].buffer(5.0)).buffer(-0.05)
        road = road.union(drive)
        road_b = road_b.union(drive)
    man = json.load(open(os.path.join(a.out, "meshes", "manifest.json")))
    fails = []
    for o in man["objects"]:
        if o["kind"] in ("Road_Road", "Road_Marking", "Road_Curb"):
            continue
        building = o["name"].startswith("SM_")
        d = np.load(os.path.join(a.out, "meshes", o["name"] + ".npz"))
        for k in d.files:
            if not k.endswith("|V"):
                continue
            mat = k[:-2]
            V, I = d[k], d[mat + "|I"]
            T = V[I]
            cen = T.mean(axis=1)
            Tz = T[:, :, 2] if D is None else T[:, :, 2] - D.z(T[:, :, 0], T[:, :, 1])
            sel = np.ones(len(cen), bool) if building else Tz.min(axis=1) < CLEAR
            # ground lying no higher than the kerb cannot block anything (paving,
            # grass edges; build.py checks ground over the carriageway) / 地面は対象外
            sel &= Tz.max(axis=1) > GROUND
            if area is not None:
                sel &= shapely.contains_xy(area, cen[:, 0], cen[:, 1])
            if not sel.any():
                continue
            # the whole triangle, not just its centre: a big roof triangle can
            # reach over a road with its centre inside the plot / 三角形全体で判定
            tri = T[sel]
            polys_ = shapely.polygons(np.concatenate([tri[:, :, :2], tri[:, :1, :2]], axis=1))
            flat = shapely.area(polys_) < 1e-6          # vertical faces: test their base edge / 鉛直面は底辺
            if flat.any():
                k = np.argsort(tri[flat][:, :, 2], axis=1)[:, :2]
                e = np.take_along_axis(tri[flat][:, :, :2], k[:, :, None], axis=1)
                polys_[flat] = shapely.linestrings(e)
            hit = shapely.intersects(polys_, road_b if building else road)
            if hit.any():
                pts = cen[sel][hit]
                fails.append({"object": o["name"], "material": mat, "triangles": int(hit.sum()),
                              "building": building, "example": [round(float(v), 2) for v in pts[0]],
                              "z_min": round(float(Tz[sel][hit].min()), 2)})
    for f in fails:
        print("FAIL %(object)s %(material)s: %(triangles)d triangles over the road, lowest z %(z_min)s, e.g. %(example)s"
              % f)
    print("road check: %s (%d problems)" % ("PASS" if not fails else "FAIL", len(fails)))
    if a.json:
        json.dump({"pass": not fails, "problems": fails}, open(a.json, "w"), indent=1)
    sys.exit(1 if fails else 0)


if __name__ == "__main__":
    main()
