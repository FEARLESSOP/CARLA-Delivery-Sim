"""Per-building facades for an embedded core (General Painter): the model
author's building survey (maps.json "buildings_csv", e.g. the Oita Station v8
buildings.csv: id, kind, height_m, facade_style, x_east_m / y_north_m) turned
into colour_notes.json "facades" entries that buildings_jp honours (style,
storeys, wall / accent / glass / facade type), with Street View corrections
from colour_notes.json "facades_sv" (keyed by the csv id) merged on top.

  python core_facades.py build/<Name>          (maps/.venv or any python3)

Only buildings inside maps.json "core" bounds are written: the generated band
round it keeps its own colour observations. The csv is read as data only.
Seeded per csv id: the same survey gives the same city.

埋め込みコアの建物ごとの外装: 作者の建物表（facade_style・高さ）を
buildings_jp の様式・階数・外壁色へ変換し、Street View の補正を重ねる。
"""

import csv
import json
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))

# v8 facade_style -> (style, wall mix, accent mix, facade-type mix); the colour
# names follow the v8 style names, the shares the Street View frames of the
# core (photos/core_C1..C5, 2026-11-08): white / light grey grid offices,
# blue glass, greige panels, red-brown and dark brown brick tile, beige and
# white tile mansions / v8 の外装名 -> 様式・外壁・差し色・外装型
OFFICE_FRONTS = [("punched", 4), ("ribbon", 4), ("curtain", 1)]
MAP = {
    "office_white": ("office", [("jp_panel_white", 3), ("jp_tile_white", 2)],
                     [("jp_panel_grey", 2), ("jp_panel_darkgrey", 1), ("jp_panel_white", 1)], OFFICE_FRONTS),
    "office_grey": ("office", [("jp_panel_grey", 3), ("jp_panel_darkgrey", 1), ("jp_tile_white", 1)],
                    [("jp_panel_darkgrey", 2), ("jp_panel_white", 1)], OFFICE_FRONTS),
    "office_beige": ("office", [("jp_panel_beige", 3), ("jp_tile_beige", 2)],
                     [("jp_panel_brown", 1), ("jp_panel_white", 1), ("jp_panel_darkgrey", 1)], OFFICE_FRONTS),
    "office_brown": ("office", [("jp_panel_brown", 2), ("jp_tile_brown", 2), ("jp_tile_darkbrown", 1)],
                     [("jp_panel_beige", 2), ("jp_panel_darkgrey", 1)], [("punched", 3), ("ribbon", 2)]),
    "curtain_blue": ("office", [("jp_panel_white", 2), ("jp_panel_grey", 1)], [("jp_panel_grey", 1)],
                     [("curtain", 1)]),
    "curtain_green": ("office", [("jp_panel_grey", 2), ("jp_panel_white", 1)], [("jp_panel_darkgrey", 1)],
                      [("curtain", 1)]),
    "tile_white": ("office", [("jp_tile_white", 3), ("jp_panel_white", 1)],
                   [("jp_panel_grey", 1), ("jp_tile_darkbrown", 1)], [("punched", 3), ("ribbon", 1)]),
    "tile_beige": ("office", [("jp_tile_beige", 3), ("jp_panel_beige", 1)],
                   [("jp_tile_darkbrown", 1), ("jp_panel_white", 1)], [("punched", 3), ("ribbon", 1)]),
    "tile_brown": ("office", [("jp_tile_brown", 2), ("jp_panel_brown", 1), ("jp_tile_darkbrown", 1)],
                   [("jp_panel_beige", 1), ("jp_tile_white", 1)], [("punched", 3), ("ribbon", 1)]),
    "render_grey": ("office", [("jp_panel_grey", 2), ("jp_wall_grey_render", 1)],
                    [("jp_panel_white", 1), ("jp_panel_darkgrey", 1)], [("punched", 2), ("ribbon", 1)]),
    "retail_a": ("shop", [("jp_tile_beige", 2), ("jp_wall_white_render", 1), ("jp_tile_white", 2),
                          ("jp_panel_beige", 1)], [("jp_panel_grey", 1), ("jp_tile_darkbrown", 1)],
                 [("punched", 2), ("ribbon", 1)]),
    "retail_b": ("shop", [("jp_wall_grey_render", 1), ("jp_panel_grey", 2), ("jp_tile_brown", 1),
                          ("jp_panel_white", 1), ("jp_panel_darkgrey", 1)],
                 [("jp_panel_white", 1), ("jp_panel_darkgrey", 1)], [("punched", 1), ("ribbon", 2)]),
    "apt_white": ("mansion", [("jp_tile_white", 3), ("jp_concrete_white", 1)],
                  [("jp_concrete_white", 1), ("jp_tile_beige", 1), ("jp_panel_grey", 1)], None),
    "apt_beige": ("mansion", [("jp_tile_beige", 3)], [("jp_concrete_white", 1), ("jp_tile_brown", 1)], None),
    "apt_brown": ("mansion", [("jp_tile_brown", 2), ("jp_tile_darkbrown", 1)],
                  [("jp_tile_beige", 1), ("jp_concrete_white", 1)], None),
    "apt_grey": ("mansion", [("jp_panel_grey", 2), ("jp_tile_white", 1)],
                 [("jp_concrete_white", 1), ("jp_panel_darkgrey", 1)], None),
    "house_cream": ("house", [("jp_wall_ivory", 2), ("jp_wall_ivory_render", 1)], None, None),
    "house_grey": ("house", [("jp_wall_grey", 2), ("jp_wall_grey_render", 1)], None, None),
    "house_brown": ("house", [("jp_wall_brown", 2), ("jp_wall_brown_tile", 1)], None, None),
    "parking": ("carpark", [("jp_panel_grey", 2), ("jp_panel_white", 1)], [("jp_panel_white", 1)], None),
    "south_parking": ("carpark", [("jp_panel_grey", 1)], [("jp_panel_white", 1)], None),
    "canopy": ("shed", None, None, None),
    "warehouse": ("shed", None, None, None),
    "school": ("school", None, None, None),
}
# a low building of a house colour in the city centre is a narrow shop
# building (雑居ビル), not a detached house / 都心の低層は雑居ビル
HOUSE_AS_SHOP = {"house_cream": "jp_wall_ivory_render", "house_grey": "jp_wall_grey_render",
                 "house_brown": "jp_tile_brown"}
KANBAN = [("jp_kanban_white", 3), ("jp_kanban_red", 2), ("jp_kanban_blue", 2), ("jp_kanban_green", 1),
          ("jp_kanban_yellow", 1)]


def pick(rng, mix):
    return rng.choices([k for k, _ in mix], [w for _, w in mix])[0] if mix else None


def storeys(style, h, front=None):
    """Storeys from a surveyed height (m) for buildings_jp's storey heights
    (office: 4.65 m ground floor + 3.6, hotel 3.2; mansion 3.0 + plinth and
    lift room; shop 3.6 + 2.9; car park decks 2.9). / 高さから階数。"""
    if style == "office":
        return max(2, int(round((h - 1.1 - 4.65) / (3.2 if front == "hotel" else 3.6))) + 1)
    if style == "mansion":
        return max(3, int(round((h - 0.55 - (4.2 if h > 15 else 1.0)) / 3.0)))
    if style == "shop":
        return max(1, int(round((h - 0.8 - 3.7) / 2.9)) + 1)
    if style == "house":
        return 1 if h < 5.0 else (2 if h < 8.5 else 3)
    if style == "carpark":
        return max(1, int(round((h - 1.1 - 0.15) / 2.9)))
    return 1


def entry(r, sx):
    """One csv row -> a facades entry, or None. / 1 行を外装の項目に。"""
    fs = r.get("facade_style", "")
    if fs not in MAP:
        return None
    rng = random.Random("fac:" + r["id"])
    style, walls, accs, fronts = MAP[fs]
    try:
        h = float(r.get("height_m") or 0.0)
        x, y = float(r["x_east_m"]) * sx, float(r["y_north_m"])
        area = float(r.get("footprint_m2") or 0.0)
    except ValueError:
        return None
    kind = r.get("kind", "")
    e = {"xy": [round(x, 2), round(y, 2)], "area": round(area, 1), "id": r["id"], "v8": fs, "kind": kind,
         "h": h, "hsrc": r.get("height_source", "")}
    if r.get("name"):
        e["name"] = r["name"]
    front = pick(rng, fronts)
    if kind == "hotel" and style == "office" and front != "curtain":
        front = "hotel"
    if kind == "apartment" and style == "office":
        style, front = "mansion", None
    lv = storeys(style, h, front) if h > 0 else None
    if style == "house" and (lv is None or lv > 3 or area > 250 or h > 10.5):
        style = "shop"
        walls = [(HOUSE_AS_SHOP[fs], 1)]
        lv = storeys("shop", h) if h > 0 else 2
    if style == "shop" and lv and lv > 4:
        # taller than a shop building: an office block with a shop podium
        # 店舗棟より高い: 1 階が店舗の事務所ビル
        style = "office"
        front = front or pick(rng, OFFICE_FRONTS)
        lv = storeys("office", h, front)
    if style == "office" and lv and lv < 2:
        style, lv = "shop", 1
    e["style"] = style
    if lv:
        e["levels"] = lv
    w, a = pick(rng, walls), pick(rng, accs)
    if w:
        e["wall"] = w
    if a and style in ("office", "mansion", "carpark"):
        e["acc"] = a
    if style == "office":
        e["front"] = front or "punched"
        e["podium"] = "shop" if (kind == "retail" or fs.startswith("retail") or rng.random() < 0.35) else "office"
        e["kanban"] = pick(rng, KANBAN)
        e["ribbon"] = rng.choice("ab")
        if e["front"] == "curtain":
            e["glass"] = {"curtain_green": "jp_curtain_green"}.get(fs, rng.choice(
                ["jp_curtain_blue", "jp_curtain_blue", "jp_curtain_dark", "jp_curtain_silver"]))
    if style == "mansion":
        e["bal"] = rng.choice(["solid", "solid", "glass", "rail"])
    return e


def main(bd):
    cfg = json.load(open(os.path.join(HERE, "..", "maps.json")))
    meta = json.load(open(os.path.join(bd, "meta.json")))
    ent = next(v for v in cfg.values() if isinstance(v, dict) and v.get("name") == meta["name"])
    core = ent.get("core")
    if not core or not ent.get("buildings_csv"):
        raise SystemExit("maps.json entry has no core / buildings_csv")
    x0, y0, x1, y1 = core["bounds"]
    sx = float(core.get("x_scale", 1.0))
    notes_p = os.path.join(bd, "colour_notes.json")
    notes = json.load(open(notes_p))
    sv = {k: v for k, v in (notes.get("facades_sv") or {}).items() if not k.startswith("_")}
    out, styles = [], {}
    with open(os.path.expanduser(ent["buildings_csv"]), encoding="utf-8") as fh:
        for r in csv.DictReader(fh):
            e = entry(r, sx)
            if e is None or not (x0 <= e["xy"][0] <= x1 and y0 <= e["xy"][1] <= y1):
                continue
            if r["id"] in sv:
                fix = {k: v for k, v in sv[r["id"]].items() if k not in ("photo", "note")}
                if "style" in fix and "levels" not in fix and e.get("h"):
                    fix["levels"] = storeys(fix["style"], e["h"], fix.get("front", e.get("front")))
                e.update(fix)
                e["sv"] = sv[r["id"]].get("photo", True)
            out.append(e)
            styles[e["style"]] = styles.get(e["style"], 0) + 1
    notes["facades"] = out
    notes["_facades_doc"] = (
        "core_facades.py (General Painter, 2026-11-08): one entry per building of the v8 author's survey "
        "(maps.json buildings_csv) inside the core bounds, xy in this map frame (v8 x * x_scale). style / levels "
        "/ wall / acc / glass / front (punched, ribbon, curtain, hotel) / podium (shop, office) / kanban / bal "
        "are buildings_jp names; buildings_jp.JP.facade_of gives an entry to the generated footprint that "
        "contains its point (biggest surveyed footprint first, canopies last) or lies within 5 m. levels from "
        "the csv height (shadow / OSM levels; the topoexport's 6.0 m / 2 storeys is a placeholder). "
        "facades_sv: Street View corrections by csv id (photos/core_C*.jpg), merged on top. Rerun the script "
        "after editing facades_sv.")
    json.dump(notes, open(notes_p, "w"), ensure_ascii=False, indent=1)
    print("facades: %d entries in the core (%s)" % (len(out), ", ".join("%s %d" % kv for kv in sorted(styles.items()))))


if __name__ == "__main__":
    main(sys.argv[1])
