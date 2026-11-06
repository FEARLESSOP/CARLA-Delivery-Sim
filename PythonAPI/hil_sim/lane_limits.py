"""The posted speed limit of the lane a car is on, for your own maps.

CARLA's Vehicle.get_speed_limit() only changes when the car drives through a
274 speed sign, and keeps it until the next one. Japan posts no sign where
the legal limit applies (60 km/h with a centre line, 30 without), so after
leaving the 100 km/h ring the old value would stay. On the maps in
maps/maps.json the lanes carry their limit in the xodr (<speed>,
maps/osm_city/jp_speeds.py), so the dashboard and the driving window read
that; any other map keeps get_speed_limit().

  ll = LaneLimits()
  kmh = ll.limit(world_map, vehicle)       # None: use vehicle.get_speed_limit()

自作マップでは車線の最高速度（xodr）を表示する（標識の無い法定速度の道路でも正しい）。
"""

import json
import os

from traffic_profile import lane_speeds_from_xodr

HERE = os.path.dirname(os.path.abspath(__file__))


def own_maps(path=os.path.join(HERE, "maps", "maps.json")):
    try:
        return {m["name"] for m in json.load(open(path)).values() if isinstance(m, dict) and m.get("name")}
    except (OSError, ValueError):
        return set()


class LaneLimits(object):
    def __init__(self, maps=None):
        self.maps = own_maps() if maps is None else set(maps)
        self.name = None
        self.by_section, self.by_lane = {}, {}

    def _load(self, wmap):
        name = wmap.name.split("/")[-1]
        if name != self.name:
            self.name = name
            self.by_section, self.by_lane = ({}, {}) if name not in self.maps else \
                lane_speeds_from_xodr(wmap.to_opendrive())

    def limit(self, wmap, actor):
        """km/h of the lane under ``actor`` on your own maps, else None.
        自作マップでは車線の最高速度、それ以外は None。"""
        self._load(wmap)
        if not self.by_section:
            return None
        wp = wmap.get_waypoint(actor.get_location())
        if wp is None:
            return None
        kmh = self.by_section.get((wp.road_id, wp.section_id, wp.lane_id))
        if kmh is None:
            kmh = self.by_lane.get((wp.road_id, wp.lane_id))
        return round(kmh) if kmh else None
