"""A GlobalRoutePlanner that cannot run away on imported maps.

CARLA's planner samples every road segment by following ``next()`` from the
segment's entry until it comes within one step of the segment's exit. On
maps converted from OpenStreetMap (Oita Station: 170 of 3172 segments) the
walk never arrives -- it leaves the segment and circles the city, appending
a waypoint every step until the machine runs out of memory (60 GB, then
the out-of-memory killer).

This version follows the successor that stays on the segment's road (and
heads for its exit) where a lane forks, caps each walk at a few times the
segment's own length, and
returns an empty route instead of raising when there is no path. Where it
cannot label a turn (a junction on such a map can give it a missing
vector), that step is lane-following and the route is kept. A
segment that hits the cap keeps no sampled path: the graph still links its
entry to its exit, so routes through it are found, just without the
in-between waypoints.

OSM から変換したマップで、区間のサンプリングが出口に着かず無限に伸びて
メモリを使い切る問題を防ぐ GlobalRoutePlanner。
"""

import numpy as np
from agents.navigation.global_route_planner import GlobalRoutePlanner
from agents.navigation.local_planner import RoadOption


class SafeRoutePlanner(GlobalRoutePlanner):
    def _build_topology(self):
        self._topology = []
        self.runaway_segments = 0
        res = self._sampling_resolution
        for wp1, wp2 in self._wmap.get_topology():
            l1, l2 = wp1.transform.location, wp2.transform.location
            if l1.distance(l2) < 1e-3:
                # a zero-length junction connector (netconvert): CARLA's graph build
                # cannot take its direction; entry and exit are one node anyway
                # 長さ 0 の交差点内道路は除く（入口と出口は同じ節点）
                continue
            x1, y1, z1, x2, y2, z2 = np.round([l1.x, l1.y, l1.z, l2.x, l2.y, l2.z], 0)
            wp1.transform.location, wp2.transform.location = l1, l2
            seg = {"entry": wp1, "exit": wp2, "entryxyz": (x1, y1, z1), "exitxyz": (x2, y2, z2),
                   "path": []}
            if l1.distance(l2) > res:
                # a sane walk is about distance / res steps; allow plenty for curves
                # 正常なら 距離/刻み 歩程度。曲線分の余裕を持たせる
                limit = base_limit = int(4 * l1.distance(l2) / res) + 50
                first = wp1.next(res)
                if not first:
                    # a junction stub whose successor starts metres away (netconvert,
                    # PasirRisTampines 2026-11-20): link entry -> exit only
                    # 後続が離れた所から始まる交差点内の短い道路: 入口と出口だけを結ぶ
                    self._topology.append(seg)
                    continue
                w = first[0]
                best = float("inf")
                while w.transform.location.distance(l2) > res:
                    d = w.transform.location.distance(l2)
                    if d > best and best < 2 * res:
                        break                            # passed the exit just wide of it / 出口のすぐ脇を通過
                    best = min(best, d)
                    seg["path"].append(w)
                    if len(seg["path"]) > limit and limit == base_limit:
                        # a long curved road (a loop whose ends are close): allow its own
                        # length once (PasirRisTampines 2026-11-20: 600 m estate loops)
                        # 両端が近い長い曲線道路（ループ）は道路長まで許す
                        L = self._road_length(wp1.road_id)
                        if L > l1.distance(l2):
                            limit = max(limit, int(1.5 * L / res) + 50)
                    if len(seg["path"]) > limit:
                        seg["path"] = []                 # runaway: link entry -> exit only / 暴走
                        self.runaway_segments += 1
                        break
                    nxt = w.next(res)
                    if not nxt:
                        break
                    # stay on the segment's own road where the lane forks
                    # (CARLA takes the first successor, which can leave it)
                    # 分岐では区間自身の道路に留まる
                    same = [q for q in nxt if q.road_id in (wp1.road_id, wp2.road_id)] or nxt
                    w = min(same, key=lambda q: q.transform.location.distance(l2))
            else:
                nxt = wp1.next(res)
                if not nxt:
                    continue
                seg["path"].append(nxt[0])
            self._topology.append(seg)

    def _road_length(self, road_id):
        """A road's length from the map's OpenDRIVE (read once, when first needed).
        道路の長さ（OpenDRIVE から、必要になった時に一度だけ読む）。"""
        if getattr(self, "_road_len", None) is None:
            import xml.etree.ElementTree as ET
            self._road_len = {int(r.get("id")): float(r.get("length"))
                              for r in ET.fromstring(self._wmap.to_opendrive()).iter("road")}
        return self._road_len.get(road_id, 0.0)

    def trace_route(self, origin, destination):
        """[] when there is no way there (one-way streets, the map's edge)
        instead of raising: callers fall back to straight-line distance, and a
        van that cannot route uses its usual stuck recovery.
        経路が無い場合は例外ではなく空リストを返す。"""
        import networkx as nx
        try:
            return super().trace_route(origin, destination)
        except (nx.NetworkXNoPath, nx.NodeNotFound):
            return []

    def _turn_decision(self, *args, **kwargs):
        """LANEFOLLOW where CARLA's turn labelling fails on a missing vector.
        曲がり判定が失敗する箇所は直進扱いにする。"""
        try:
            return super()._turn_decision(*args, **kwargs)
        except (ValueError, TypeError, KeyError):
            return RoadOption.LANEFOLLOW
