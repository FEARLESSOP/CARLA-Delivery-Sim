"""Build the map's meshes from the OSM file and the road logic (maps/.venv).

  python build.py <osm> <out_dir>        (after roadnet.py and lanes.py)

Writes <out_dir>/meshes/*.npz (one per object: per material vertices,
normals, UVs, triangles), <out_dir>/meshes/manifest.json, the textures,
<out_dir>/buildings.csv (for delivery_sim: the map's buildings are
tagged Terrain by CARLA's importer) and a plan preview.

OSM と道路情報から地図のメッシュを作る。
"""

import csv
import json
import os
import pickle
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import buildings  # noqa: E402
import extras  # noqa: E402
import sitefeatures  # noqa: E402
from landmarks import akeno, dorm, kosen, lvss, prcs, seven  # noqa: E402
from shapely.geometry import Point  # noqa: E402
from shapely.ops import unary_union  # noqa: E402
import ground  # noqa: E402
import layout  # noqa: E402
import markings  # noqa: E402
import jtrees  # noqa: E402
import props  # noqa: E402
import terrain  # noqa: E402
import textures  # noqa: E402
from mesh import Mesh  # noqa: E402

PLINTH_OUT = 0.1          # hilly maps: plinth face this far outside the footprint / 基壇の張り出し


def plinth(poly, zlow, ztop, mat="concrete"):
    """A building's plinth / retaining wall on sloping ground: from below the
    lowest ground to just above the highest (where the building stands).
    傾斜地の建物の基壇（擁壁）: 最低地盤の下から最高地盤の少し上まで。"""
    m = Mesh()
    ring = poly.buffer(PLINTH_OUT, join_style="mitre")
    m.prism(mat, ring, zlow, ztop, top=False)
    m.flat(mat, ring.difference(poly), ztop, 1.0)
    return m


def save(mesh_dir, name, m, manifest, kind):
    arrs = m.arrays()
    if not arrs:
        return
    data = {}
    for mat, (V, N, UV, I) in arrs.items():
        data[mat + "|V"], data[mat + "|N"], data[mat + "|UV"], data[mat + "|I"] = V, N, UV, I
    np.savez_compressed(os.path.join(mesh_dir, name + ".npz"), **data)
    manifest["objects"].append({"name": name, "kind": kind, "materials": sorted(arrs),
                                "tris": int(sum(len(a[3]) for a in arrs.values()))})


def road_style(name):
    """maps.json "road_style" of the map called ``name`` (e.g. "japan"), None
    for the default (Singapore) practice. / maps.json の道路様式。"""
    try:
        for v in json.load(open(os.path.join(HERE, "..", "maps.json"))).values():
            if isinstance(v, dict) and v.get("name") == name:
                return v.get("road_style")
    except (OSError, ValueError):
        pass
    return None


def main():
    osm, out = sys.argv[1:3]
    t0 = time.time()
    meta = json.load(open(os.path.join(out, "meta.json")))
    name = meta["name"]
    style = road_style(name)
    if style:
        meta["road_style"] = style
    L = layout.Layout(meta, os.path.join(out, "lanes.json"), osm)
    print(L.summary(), flush=True)
    # hilly maps: the ground follows the elevated road logic and the terrain
    # 起伏のある地図: 地面は標高付きの道路情報と地形に沿う
    D = terrain.drape_for(out, L)
    if D is not None:
        print("terrain: road surface vs terrain %.1f..%.1f m (cut < 0 < fill)" % D.cut_fill, flush=True)
    # hand-modelled landmarks replace the generic buildings on their sites
    # 詳細モデルの建物は敷地内の汎用建物を置き換える
    # each landmark module: NAME, TAG, TRACE (its trace file in the build dir),
    # the class, write_textures() / 各詳細モデル: 名前・タグ・トレース・クラス
    marks = []
    for mod, cls in ((prcs, prcs.PRCS), (lvss, lvss.LVSS), (kosen, kosen.KOSEN), (seven, seven.SEVEN),
                     (dorm, dorm.DORM), (akeno, akeno.AKENO)):
        # a module with enabled(): switched on by its maps.json entry (akeno:
        # "area_features"), the others by their trace file / maps.json で有効化するモジュール
        if (mod.enabled(out, meta) if hasattr(mod, "enabled") else os.path.exists(os.path.join(out, mod.TRACE))):
            lm = cls(L, out)
            lm.mod = mod
            lm.drape = D             # landmarks/onterrain.py modules stand on the terrain / 地形に載せる
            marks.append(lm)
    # landmarks that draw their own ground and trees (onterrain): the generic
    # stages see the layout exactly as without them; their site is cleared only
    # for the ground and taken out of the props afterwards, so nothing outside
    # the site changes (props.py spends one random stream over the whole map)
    # 自前で地面・樹木を描く詳細モデル: 汎用処理には元の配置を見せ、敷地は地面だけ消す
    draped = [lm for lm in marks if hasattr(lm, "ground_taken")]
    marks_old = [lm for lm in marks if not hasattr(lm, "ground_taken")]
    # the car-park aisles inside landmark sites stay reserved (no lamps, trees,
    # fences on them) after their generic paving is cleared below
    # 敷地内の駐車通路は舗装を消した後も占有扱い（街灯・樹木・柵を置かない）
    L.reserved = layout.clean(unary_union([L.driveways.intersection(lm.site.buffer(5.0)) for lm in marks_old]), 0.5) \
        if marks_old else layout.Polygon()
    for lm in marks_old:
        L.buildings = [(p, t) for p, t in L.buildings if not lm.site.contains(p.representative_point())]
        L.building_area = L.building_area.union(lm.footprint)
        inner = lm.inner()
        for nm in ("parking", "driveways", "footpaths", "cycleways", "aprons", "parks", "forest", "pitches",
                   "playgrounds", "pools"):
            setattr(L, nm, layout.clean(getattr(L, nm).difference(inner), 0.05))
        print("landmark: %s (%d blocks)" % (lm.mod.NAME, len(lm.blocks)), flush=True)
    surfaces = None
    if style == "japan":
        # Japanese markings and road surfaces / 日本式の標示と路面
        import markings_jp
        import surfaces_jp
        from shapely import area as shapely_area
        M = markings_jp.MarkingsJP(L, os.path.join(out, name + ".xodr"))
        print("markings (japan): %s" % M.report(), flush=True)
        for q in M.stop_log:
            print("  stop: junction %(junction)s road %(road)s way %(way)s at (%(x)s, %(y)s): %(why)s" % q)
        for r, j, w in M.no_stop_log:
            print("  no stop (road_features no_stop): junction %s road %s way %s" % (j, r, w))
        for q in getattr(M, "jartic_log", []):
            print("  jartic: %s" % ", ".join("%s %s" % kv for kv in q.items()))
        for r, j, why in M.dropped_stops:
            print("  stop dropped: junction %s road %s (%s): short link, no road outside the cluster" % (j, r, why))
        surfaces = surfaces_jp.partition(L, M)
        print("road surfaces: %s" % ", ".join("%s %.0f m2" % (k, float(np.sum(shapely_area(v))))
                                               for k, v in surfaces.items()), flush=True)
    else:
        M = markings.Markings(L)
    print("markings: %d pieces, crossings %d" % (len(M.out), len(M.crossings)), flush=True)
    # what imagery adds: colours, courts, fences, playgrounds, boards / 画像から読んだ情報
    S = sitefeatures.Sites(out, meta)
    lanes_data = json.load(open(os.path.join(out, "lanes.json")))
    X = extras.Extras(L, S, lanes_data["lanes"], lanes_data["approaches"])
    for lm in marks_old:             # the landmark has its own courts and fences / 独自のコート・柵
        X.courts = [c for c in X.courts if not lm.site.contains(Point(c["cx"], c["cy"]))]
        X.fences = [f for f in X.fences if not lm.site.buffer(2.0).contains(f.interpolate(0.5, normalized=True))]
    X.apply_to_layout()
    print("courts %d, fence runs %d, ships %d, direction boards %d, colour observations %d"
          % (len(X.courts), len(X.fences), len(X.ships), len(X.boards), len(S.obs)), flush=True)
    B = buildings.Builder(L, S)
    if B.jp:
        # Japanese maps: lots and terraces first, so the ground leaves them out
        # and trees stay off them (buildings_jp) / 日本: 敷地の造成を先に（地面・樹木が避ける）
        B.drape = D
        B.keep_clear = unary_union([X.court_area().buffer(0.5)] + [f.buffer(0.4) for f in X.fences]
                                   # lots keep off landmark features that are not OSM areas (akeno)
                                   # OSM に無い地物（akeno）を敷地が避ける
                                   + [lm.lot_clear() for lm in marks if hasattr(lm, "lot_clear")])
        B.jp_styles().plan()
        J = B.jp_styles()
        print("lots: %d terraces, %.0f m2 (ground cut away beneath)" % (len(J.items), L.terraces.area), flush=True)
    if draped:
        names = ("parking", "driveways", "footpaths", "cycleways", "aprons", "parks", "forest", "pitches",
                 "playgrounds", "pools")
        before = {nm: getattr(L, nm) for nm in names}
        for lm in draped:
            inner = lm.inner()
            for nm in names:
                # piece by piece: pieces clear of the site stay the same polygons / 敷地に掛かる部分だけ切る
                keep_p = []
                for g in layout.polys(getattr(L, nm)):
                    keep_p += [q for q in layout.polys(g.difference(inner)) if q.area >= 0.05] \
                        if g.intersects(inner) else [g]
                setattr(L, nm, layout.MultiPolygon(keep_p))
            print("landmark: %s (%d blocks)" % (lm.mod.NAME, len(lm.blocks)), flush=True)
        cleared = {nm: getattr(L, nm) for nm in names}
        lot_terraces = getattr(L, "terraces", None)
        L.terraces = unary_union(([lot_terraces] if lot_terraces is not None else [])
                                 + [lm.ground_taken() for lm in draped])
    G = ground.Ground(L, M.out, M.crossings, drape=D, surfaces=surfaces)
    if draped:
        L.terraces = lot_terraces                # only the grass base leaves the sites out / 芝の除外のみ
        for nm, v in before.items():
            setattr(L, nm, v)
        if hasattr(L, "footpaths_before"):
            # props see the default pavement band too (layout "footpaths": "osm") / 既定歩道帯も見せる
            L.footpaths = L.footpaths_before
    X.drape = D
    if D is not None and marks_old:
        print("WARNING: landmarks are not draped on terrain yet: %s" % [lm.mod.NAME for lm in marks_old])
    keep = [X.court_area().buffer(1.0)]
    for lm in marks_old:
        keep.append(lm.keepout())
    if getattr(L, "terraces", None) is not None:
        keep.append(L.terraces.buffer(0.3))
    P = props.Props(L, M.crossings, keepout=unary_union(keep))
    P.drape = D
    # Japan-style maps draw every tree as a seasonal Japanese species (jtrees.py)
    # 日本式のマップは全ての木を季節の樹木で描く
    P.jtrees = style == "japan"
    if draped:
        for nm, v in cleared.items():
            setattr(L, nm, v)
        for lm in draped:
            # its own courts and fences replace the generic ones (dropped only now:
            # the layout and lots were made with them, as without the landmark)
            # 汎用のコート・柵は最後に除く（配置・造成は元のまま）
            X.courts = [c for c in X.courts if not lm.site.contains(Point(c["cx"], c["cy"]))]
            X.fences = [f for f in X.fences if not lm.site.buffer(2.0).contains(f.interpolate(0.5, normalized=True))]
            ko = lm.keepout()
            P.trees = [t for t in P.trees if not ko.contains(Point(t[0], t[1]))]
            if hasattr(lm, "tree_clear"):
                # its own tree rows replace the generic trees along them / 自前の樹列が汎用の木に代わる
                tc = lm.tree_clear
                P.trees = [t for t in P.trees if not tc.contains(Point(t[0], t[1]))] + lm.trees()
            P.items = [it for it in P.items if not (ko.intersects(it[1]) if hasattr(it[1], "geom_type")
                                                    else ko.contains(Point(it[1][0], it[1][1])))]
    print("trees %d, furniture %d" % (len(P.trees), len(P.items)), flush=True)
    mesh_dir = os.path.join(out, "meshes")
    os.makedirs(mesh_dir, exist_ok=True)
    for f in os.listdir(mesh_dir):
        os.remove(os.path.join(mesh_dir, f))
    manifest = {"name": name, "objects": [], "materials": {k: list(v[:1]) + [list(v[1]) if v[1] else None]
                                                           + list(v[2:]) for k, v in textures.materials(style).items()}}
    # buildings, grouped into the ground tiles / 建物はタイルごとにまとめる
    btile = {}
    rows = []
    plinths = []
    for i, (poly, t) in enumerate(L.buildings):
        m = B.build(poly, t, i)
        if m is None:
            continue
        zb = 0.0
        if B.jp:
            # on its terrace (the lot, retaining walls included in the mesh)
            # 造成面の上（擁壁はメッシュに含む）
            it = B.jp_styles().items[i]
            zb = it.zb
            m.shift_z(zb)
            if it.drop > 0.05:
                plinths.append(it.drop)
        elif D is not None:
            # on the highest ground under it, a plinth down to the lowest
            # 敷地の最高地盤に置き、最低地盤まで基壇
            zlo, zhi = D.footprint(poly)
            zb = zhi
            m.shift_z(zb)
            if zhi - zlo > 0.05:
                m.merge(plinth(poly, zlo - 0.3, zhi + ground.RAISED_Z))
                plinths.append(zhi - zlo)
        c = poly.centroid
        tile = next((tn for tn, tb in ground.tiles(meta) if tb.contains(c)), None) or "00"
        btile.setdefault(tile, Mesh()).merge(m)
        mr = poly.minimum_rotated_rectangle
        cc = list(mr.exterior.coords)
        e0 = np.hypot(cc[1][0] - cc[0][0], cc[1][1] - cc[0][1])
        e1 = np.hypot(cc[2][0] - cc[1][0], cc[2][1] - cc[1][1])
        yaw = np.degrees(np.arctan2(cc[1][1] - cc[0][1], cc[1][0] - cc[0][0]))
        lv = buildings.levels_of(t, 2)
        st = buildings.style_of(t, poly, lv)
        if B.jp:
            st, lv = it.style, it.levels
        height = {"hdb": buildings.VOID + (lv - 1) * buildings.FLOOR + 1.2, "condo": 4.5 + (lv - 1) * 3.1 + 1.2,
                  "carpark": lv * 3.0 + 1.0, "house": min(lv, 3) * 3.2 + 2.5, "retail": 5.0 + (lv - 1) * 4.2 + 1.0,
                  "school": lv * 3.5 + 1.0, "worship": max(2, lv) * 3.8 + 4.0}.get(st, lv * 3.6 + 1.0)
        if B.jp:
            height = getattr(it, "height", height)
        rows.append({"id": "b%d" % i, "name": t.get("name") or ("Blk %s" % t["addr:housenumber"]
                                                               if t.get("addr:housenumber") else ""),
                     "height_m": round(height, 1), "x_east_m": round(c.x, 2), "y_north_m": round(c.y, 2),
                     "footprint_m2": round(poly.area, 1), "style": st, "levels": lv,
                     "length_m": round(e0, 2), "width_m": round(e1, 2), "yaw_deg": round(yaw, 1)})
        if D is not None:
            rows[-1]["z_base_m"] = round(zb, 2)
    print("buildings built: %s" % B.count, flush=True)
    if B.jp:
        J = B.jp_styles()
        st = sorted(J.stats.items())
        print("roofs: %s" % ", ".join("%s %d" % kv for kv in st), flush=True)
        wm = [it.wall_max for it in J.items.values() if getattr(it, "wall_max", 0) > 0.3]
        print("retaining walls: %d terraces with walls > 0.3 m (%d stone ishigaki), highest %.1f m"
              % (len(wm), sum(1 for it in J.items.values() if getattr(it, "stone", False)), max(wm) if wm else 0),
              flush=True)
    if plinths:
        print("plinths: %d buildings on slopes, height %.1f m median, %.1f m max"
              % (len(plinths), float(np.median(plinths)), max(plinths)), flush=True)
    for tn, tb in ground.tiles(meta):
        for kind, m in G.build(tb).items():
            save(mesh_dir, "%s_%s_%s" % (kind, name, tn), m, manifest, kind)
        # leaf objects are shared by the generic trees and the landmarks' own
        # trees: merged per tile, saved once (a second save overwrote the first
        # and left those trees bare) / 葉は汎用と詳細モデルで共有: タイルごとに合わせて 1 回保存
        leafs = {}
        for kind, m in P.build(tb).items():
            if kind in jtrees.KIND.values():
                leafs[kind] = m
                continue
            save(mesh_dir, "%s_%s_%s" % (kind, name, tn), m, manifest, kind)
        if tn in btile:
            save(mesh_dir, "SM_Blocks_%s_%s" % (name, tn), btile[tn], manifest, "buildings")
        save(mesh_dir, "Terrain_Courts_%s_%s" % (name, tn), X.ground_mesh(tb, ground.RAISED_Z), manifest, "courts")
        for lm in marks:
            tag = lm.mod.TAG
            save(mesh_dir, "Terrain_%s_%s_%s" % (tag, name, tn), lm.ground_mesh(tb, ground.RAISED_Z), manifest, "courts")
            save(mesh_dir, "SM_%s_%s_%s" % (tag, name, tn), lm.buildings_mesh(tb), manifest, "buildings")
            for kind, xm in (lm.extra_meshes(tb) if hasattr(lm, "extra_meshes") else {}).items():
                if kind in jtrees.KIND.values():
                    if kind in leafs:
                        leafs[kind].merge(xm)
                    else:
                        leafs[kind] = xm
                    continue
                save(mesh_dir, "%s_%s_%s" % (kind, name, tn), xm, manifest, "trees")
        for kind, lm_ in leafs.items():
            save(mesh_dir, "%s_%s_%s" % (kind, name, tn), lm_, manifest, kind)
        save(mesh_dir, "Terrain_Sites_%s_%s" % (name, tn), X.props_mesh(tb), manifest, "sites")
    tex_dir = textures.write_all(out, style)
    for lm in marks:
        for k, v in lm.mod.write_textures(tex_dir).items():
            manifest["materials"][k] = [v[0], None, v[2], v[3], v[4]]
    for k, v in X.write_board_textures(tex_dir).items():
        manifest["materials"][k] = [v[0], None, v[2], v[3], v[4]]
    if style == "japan":
        for k, v in jtrees.write_textures(tex_dir).items():
            manifest["materials"][k] = [v[0], None, v[2], v[3], v[4]]
    json.dump(manifest, open(os.path.join(mesh_dir, "manifest.json"), "w"), indent=0)
    # CARLA's PrepareAssetsForCooking drops a whole mesh when its name or ANY of
    # its material names contains "light" or "sign" (ValidateStaticMesh): a
    # material "kosen_curtain_light" silently removed a campus tile (2026-11-04)
    # 名前に light / sign を含むメッシュ・マテリアルは CARLA が丸ごと捨てる
    bad = sorted({n for o in manifest["objects"] for n in [o["name"]] + o["materials"]
                  if "light" in n.lower() or "sign" in n.lower()})
    if bad:
        raise SystemExit("names CARLA's importer would drop (contain light/sign): %s" % ", ".join(bad))
    # check: nothing raised may lie over the carriageway (a floating wheel)
    # 確認: 車道の上に盛り上がった面が無いこと
    import shapely
    road = L.carriageway.buffer(-0.02)
    bad = 0
    for o in manifest["objects"]:
        if o["kind"] in ("Road_Road", "Road_Marking"):
            continue
        d = np.load(os.path.join(mesh_dir, o["name"] + ".npz"))
        for k in d.files:
            if not k.endswith("|V"):
                continue
            V, I = d[k], d[k[:-2] + "|I"]
            cen = V[I].mean(axis=1)
            # surfaces a wheel could touch: below 1 m over the road surface there
            # 車輪が触れ得る面: その場の路面から 1 m 未満
            low = (cen[:, 2] - (D.z(cen[:, 0], cen[:, 1]) if D is not None else 0.0)) < 1.0
            if low.any():
                bad += int(shapely.contains_xy(road, cen[low, 0], cen[low, 1]).sum())
    print("check: %d low triangles over the carriageway (must be 0)" % bad, flush=True)
    # buildings.csv: x east, y north (delivery_sim turns y to CARLA's south) / x 東, y 北
    with open(os.path.join(out, "buildings.csv"), "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    layout.preview(L, os.path.join(out, "plan.png"), 1.0)
    extra = {"crossings": M.crossings, "giveway": M.giveway}
    if getattr(M, "stops", None) is not None:
        extra["stop"] = M.stops            # Japan: 止まれ approaches get stop signs / 止まれの進入路に一時停止標識
    if hasattr(M, "speed_signs"):
        extra["speed"] = M.speed_signs()   # Japan: speed signs where the limit changes / 制限変化点のみ
    if getattr(M, "jartic", None) is not None:
        # maps.json "jartic_d": JARTIC speeds for signs.py, the log for jp_signs.py / JARTIC の速度と記録
        extra["speed_set"] = M.jartic_speed
        extra["jartic_log"] = M.jartic_log
    pickle.dump(extra, open(os.path.join(out, "crossings.pkl"), "wb"))
    tot = sum(o["tris"] for o in manifest["objects"])
    print("objects %d, triangles %d (%.0f s)" % (len(manifest["objects"]), tot, time.time() - t0))


if __name__ == "__main__":
    main()
