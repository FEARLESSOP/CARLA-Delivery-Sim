"""The map's frame: metres east (x) and north (y) of the OSM box's centre.

The same equirectangular projection netconvert is given (proj4 string
below), so the road logic and the 3-D model line up exactly. CARLA's own
frame is x east, y SOUTH (OpenDRIVE y north is flipped on load); the model
is built in this x east / y north frame and exported from Blender, whose
FBX export makes the same flip.

地図座標: OSM 範囲の中心から東 (x)・北 (y) へのメートル。
"""

import math
import xml.etree.ElementTree as ET

R = 6367449.0
KY = R * math.pi / 180.0        # metres per degree of latitude / 緯度 1 度あたり


def frame_from_osm(path):
    """Centre and half sizes of the OSM file's <bounds>. / OSM 範囲から座標系を作る。"""
    for ev, el in ET.iterparse(path, events=("start",)):
        if el.tag == "bounds":
            la0, la1 = float(el.get("minlat")), float(el.get("maxlat"))
            lo0, lo1 = float(el.get("minlon")), float(el.get("maxlon"))
            lat0, lon0 = (la0 + la1) / 2, (lo0 + lo1) / 2
            kx = KY * math.cos(math.radians(lat0))
            return {"lat0": lat0, "lon0": lon0, "half_x": (lo1 - lo0) / 2 * kx,
                    "half_y": (la1 - la0) / 2 * KY}
    raise ValueError("no <bounds> in %s" % path)


class Frame(object):
    def __init__(self, meta):
        self.lat0, self.lon0 = meta["lat0"], meta["lon0"]
        self.kx = KY * math.cos(math.radians(self.lat0))

    def xy(self, lon, lat):
        return ((lon - self.lon0) * self.kx, (lat - self.lat0) * KY)

    def lonlat(self, x, y):
        return (self.lon0 + x / self.kx, self.lat0 + y / KY)

    def proj4(self):
        return ("+proj=eqc +lat_ts=%.9f +lat_0=%.9f +lon_0=%.9f +x_0=0 +y_0=0 +R=%.1f +units=m +no_defs"
                % (self.lat0, self.lat0, self.lon0, R))
