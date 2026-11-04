#!/usr/bin/env python3
"""Seasons and wind for the generated maps' trees.

The map builder (maps/osm_city/jtrees.py) draws every tree as a Japanese
species (maple, cherry, ginkgo, zelkova, camphor) and puts each species'
leaves in objects of their own (``Terrain_<Species>Leaves_<Map>_<tile>``) on
CARLA's foliage master material, with one leaf texture per species and season
beside the map's textures (``maps/build/<Map>/textures/<species>_leaves_<season>.png``).
This swaps each species' leaf texture live (``World.apply_color_texture_to_objects``) and
sets the weather's wind, which the foliage material sways with. The choice is
kept in ``season.json`` so the dashboard can apply it again after a map
(re)load, when the swapped texture is gone.

    python seasons.py --season autumn --wind 40
    python seasons.py --season auto          # from today's date (Kyushu)

マップの樹木の季節と風。各モミジの葉は独立オブジェクト（CARLA の植生マテリアル）
にあり、季節ごとの葉テクスチャを実行時に差し替え、天候の風で揺らす。
"""

import argparse
import datetime
import glob
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
STATE_PATH = os.path.join(HERE, "season.json")
SEASONS = ("hanami", "spring", "summer", "autumn_early", "autumn", "winter")
LABELS = {"hanami": ("Cherry blossom", "花見"), "spring": ("Spring", "春"), "summer": ("Summer", "夏"),
          "autumn_early": ("Early autumn", "初秋"), "autumn": ("Autumn leaves", "紅葉"), "winter": ("Winter", "冬"),
          "auto": ("By date", "日付で自動")}
# species -> object-name part the builder uses (jtrees.KIND) / 樹種と葉オブジェクト名
SPECIES = {"maple": "MapleLeaves", "sakura": "SakuraLeaves", "ginkgo": "GinkgoLeaves",
           "zelkova": "ZelkovaLeaves", "camphor": "CamphorLeaves"}


def season_for(date=None):
    """The trees' look on a date in Kyushu (Oita): cherry blossom late March to
    early April, new leaves to May, green June to mid-October, turning late
    October to mid-November, 紅葉 to mid-December, bare (camphor green) after.
    九州（大分）の日付から季節。"""
    d = date or datetime.date.today()
    md = d.month * 100 + d.day
    if 325 <= md < 411:
        return "hanami"
    if 411 <= md < 601:
        return "spring"
    if 601 <= md < 1020:
        return "summer"
    if 1020 <= md < 1115:
        return "autumn_early"
    if 1115 <= md < 1215:
        return "autumn"
    return "winter"


def load_state():
    try:
        with open(STATE_PATH) as fh:
            s = json.load(fh)
    except Exception:
        s = {}
    s.setdefault("season", "auto")
    s.setdefault("wind", 30.0)
    return s


def save_state(s):
    tmp = STATE_PATH + ".tmp"
    with open(tmp, "w") as fh:
        json.dump(s, fh, indent=1)
    os.replace(tmp, STATE_PATH)


def texture_path(map_name, species, season):
    """The map's leaf texture for a species and season, or None. / 種と季節の葉テクスチャ。"""
    short = map_name.split("/")[-1]
    name = "%s_leaves_%s.png" % (species, season)
    p = os.path.join(HERE, "maps", "build", short, "textures", name)
    if os.path.exists(p):
        return p
    hits = glob.glob(os.path.join(HERE, "maps", "build", "*", "textures", name))
    return hits[0] if hits else None


def leaf_objects(world, species=None):
    """Leaf objects in the world: {species: [names]} (or one species' list).
    ワールド内の葉オブジェクト。"""
    names = world.get_names_of_all_objects()
    out = {sp: sorted({n for n in names if key in n}) for sp, key in SPECIES.items()}
    out = {sp: v for sp, v in out.items() if v}
    return out.get(species, []) if species else out


def _texture(path, max_size=256):
    import carla
    from PIL import Image
    im = Image.open(path).convert("RGBA")
    if im.size[0] > max_size:
        # 256 px is plenty for a 0.75 m card and keeps the RPC small (each
        # texel goes over as 4 bytes) / 256 px で十分、RPC も小さく
        im = im.resize((max_size, max_size), Image.LANCZOS)
    w, h = im.size
    tex = carla.TextureColor(w, h)
    px = im.load()
    for y in range(h):
        for x in range(w):
            r, g, b, a = px[x, y]
            tex.set(x, y, carla.Color(r, g, b, a))
    return tex


def apply(world, season=None, wind=None):
    """Apply a season ('auto' = by date) and/or a wind intensity (0-100).
    Returns a status dict. / 季節と風を適用。"""
    import carla
    out = {"ok": True}
    if season is not None:
        eff = season_for() if season == "auto" else season
        if eff not in SEASONS:
            return {"ok": False, "error": "unknown season %r" % season}
        groups = leaf_objects(world)
        out.update({"season": season, "effective": eff, "objects": sum(len(v) for v in groups.values()),
                    "species": {sp: len(v) for sp, v in groups.items()}})
        if not groups:
            out.update({"ok": False, "error": "no seasonal trees on this map / このマップに季節の樹木がありません"})
        missing = []
        for sp, names in groups.items():
            # one texture per species, sent once for all its objects / 種ごとに 1 回
            path = texture_path(world.get_map().name, sp, eff)
            if path is None:
                missing.append(sp)
                continue
            world.apply_color_texture_to_objects(names, carla.MaterialParameter.Diffuse, _texture(path))
        if missing:
            out.update({"ok": False, "error": "no %s texture for %s / テクスチャなし" % (eff, ", ".join(missing))})
    if wind is not None:
        w = world.get_weather()
        w.wind_intensity = max(0.0, min(100.0, float(wind)))
        world.set_weather(w)
        out["wind"] = w.wind_intensity
    return out


def main():
    ap = argparse.ArgumentParser(description="Tree seasons and wind / 樹木の季節と風")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--season", choices=SEASONS + ("auto",), help="leaf colour / 葉の色")
    ap.add_argument("--wind", type=float, help="wind intensity 0-100 / 風の強さ")
    ap.add_argument("--list", action="store_true", help="list the leaf objects / 葉オブジェクト一覧")
    a = ap.parse_args()
    import carla
    c = carla.Client(a.host, a.port)
    c.set_timeout(30.0)
    world = c.get_world()
    if a.list:
        for sp, names in leaf_objects(world).items():
            for n in names:
                print(sp, n)
        return 0
    s = load_state()
    if a.season:
        s["season"] = a.season
    if a.wind is not None:
        s["wind"] = a.wind
    r = apply(world, a.season or s["season"], a.wind if a.wind is not None else s["wind"])
    save_state(s)
    print(json.dumps(r, ensure_ascii=False))
    return 0 if r.get("ok") else 1


if __name__ == "__main__":
    sys.exit(main())
