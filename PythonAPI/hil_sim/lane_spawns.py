"""Spawn points that lie on a lane of the road logic in use.

A map's spawn points are baked into the level when it is imported, from the
road logic of that time. When the road logic (.xodr) is replaced afterwards
-- Oita Station: streets cut at the ring road, the ring road added -- some
of them stand where there is no road any more (outside the map, or between
lanes), and a car spawned there falls off the map or sits off road; others
face the old lane direction (Oita Station's were made from road logic whose
keep-left lanes ran backwards). This keeps the points on a driving lane
outside junctions, moved onto the lane's centre line and turned the way the
lane runs. If too few are left (a map whose spawn points were never on its
lanes), all of them are returned unchanged.

マップのスポーン地点はインポート時の道路情報で焼き込まれている。道路情報を後から
差し替えた場合、道路の無い所・交差点内の地点を除き、残りを車線中心と車線の向きに合わせる。
"""

import carla


def lane_spawn_points(wmap, min_share=0.3):
    points = wmap.get_spawn_points()
    out = []
    for sp in points:
        wp = wmap.get_waypoint(sp.location, project_to_road=False, lane_type=carla.LaneType.Driving)
        if wp is None or wp.is_junction:
            continue
        tf = wp.transform                 # the lane's centre and heading / 車線の中心と向き
        tf.location.z = sp.location.z     # keep the level's spawn height / 高さは元のまま
        out.append(tf)
    return out if len(out) >= min_share * len(points) else points
