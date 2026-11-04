"""What OSM does not say, read off imagery by hand (maps/.venv).

Two files in the build directory, written while looking at Google Maps
satellite view and Street View (2020-2026 imagery; construction sites are
ignored):

  site_features.json   courts, pitches, parade squares, playgrounds traced
                       on satellite screenshots: per screenshot its centre
                       (lat, lon) and zoom, per feature a pixel box/centre,
                       size, angle (degrees clockwise on screen) and colour
  colour_notes.json    building colours seen in Street View: the observed
                       building's centroid (map frame) and its wall, accent
                       and second accent paints

Screenshot scale: the browser viewport is 1854 px wide and the screenshot
1568, so a screenshot pixel is (156543.03 cos(lat) / 2^zoom) / 0.8457
metres; the map centre falls at (784, 355) in the screenshot (calibrated on
two OSM features).

画像（衛星写真・ストリートビュー）から手で読み取った情報を読み込む。
"""

import json
import math
import os

from shapely.affinity import rotate
from shapely.geometry import Point, box

import geo

SHOT_W, VIEW_W = 1568.0, 1854.0
CENTRE = (784.0, 355.0)


class Sites(object):
    def __init__(self, out_dir, meta):
        self.fr = geo.Frame(meta)
        self.features = []
        self.obs = []
        p = os.path.join(out_dir, "site_features.json")
        if os.path.exists(p):
            for name, shot in json.load(open(p))["shots"].items():
                z = shot.get("zoom", 18)
                mpp = 156543.03392 * math.cos(math.radians(shot["lat"])) / 2 ** z / (SHOT_W / VIEW_W)
                cx, cy = self.fr.xy(shot["lon"], shot["lat"])
                for f in shot["features"]:
                    self.features.append(self._feature(f, cx, cy, mpp, name))
        self.notes = {}             # the whole colour_notes.json (areas_jp for buildings_jp) / 全体
        p = os.path.join(out_dir, "colour_notes.json")
        if os.path.exists(p):
            self.notes = json.load(open(p))
            self.obs = self.notes.get("obs", [])

    def _feature(self, f, cx, cy, mpp, shot):
        out = dict(f)
        out["shot"] = shot
        P = lambda px, py: (cx + (px - CENTRE[0]) * mpp, cy - (py - CENTRE[1]) * mpp)
        ang = -math.radians(f.get("angle", 0.0))                 # clockwise on screen -> CCW / 時計回り→反時計回り
        if "box" in f:
            x0, y0, x1, y1 = f["box"]
            c, size = ((x0 + x1) / 2, (y0 + y1) / 2), (x1 - x0, y1 - y0)
        elif "center" in f:
            c, size = f["center"], f.get("size", (2 * f.get("r", 5), 2 * f.get("r", 5)))
        else:
            c, size = f.get("near", CENTRE), (10, 10)
        x, y = P(*c)
        w, h = size[0] * mpp, size[1] * mpp
        out["xy"] = (x, y)
        out["angle_rad"] = ang
        if "r" in f:
            out["poly"] = Point(x, y).buffer(f["r"] * mpp, quad_segs=8)
        else:
            out["poly"] = rotate(box(x - w / 2, y - h / 2, x + w / 2, y + h / 2), ang, origin=(x, y), use_radians=True)
        # a court's long axis and its direction / コートの長辺と向き
        out["length"], out["width"] = max(w, h), min(w, h)
        out["axis"] = ang + (math.pi / 2 if h > w else 0.0)
        return out

    def of_type(self, *types):
        return [f for f in self.features if f["type"] in types]

    def colours(self, poly, style, max_dist=260.0):
        """The colours seen on the nearest observed building of a like kind
        (HDB-like or condo-like), or None. / 最寄りの観測建物の配色。"""
        c = poly.centroid
        kinds = {"hdb": ("hdb", "carpark"), "carpark": ("hdb", "carpark"), "condo": ("condo",),
                 "worship": ("worship",), "house": ("house", "condo")}.get(style, (style,))
        best, bd = None, max_dist
        for o in self.obs:
            if o.get("style", "hdb") not in kinds:
                continue
            d = math.hypot(o["xy"][0] - c.x, o["xy"][1] - c.y)
            if d < bd:
                best, bd = o, d
        return best
