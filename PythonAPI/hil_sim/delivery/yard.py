"""Off-road EV charging yards for the pods (no carla import).

CARLA's route planner only knows the driving lanes, so a yard off the road
(a courtyard behind a building) is reached by a scripted manoeuvre instead:
the pod drives on the road to the yard's entry, then follows a fixed path
in -- a turn off the road, a lane through the yard, and a sideways slide
into its bay (the pod has four-wheel steering, so it can crab and turn on
the spot) -- and later a fixed path back out. Only one pod moves in a yard
at a time.

Yards are surveyed per map by raycast (passage width, ceiling height,
lamps, trees); maps without one keep road-side chargers.

ポッド用の路外 EV 充電ヤード（carla 非依存）。ルート計画は走行車線しか知らない
ため、ヤードの入口までは道路を走り、そこから決まった経路（道路からの右左折・
ヤード内の通路・四輪操舵による横移動でベイへ）で入り、別の道路へ出る。
ヤード内を動くポッドは同時に 1 台。
"""

import math

# Town05: the courtyard behind the building next to the drone charging
# station. Surveyed by raycast 2026-10-27: a ground-level tunnel through the
# building (x 43-57, y 34.0-40.5, 5.6 m ceiling -- the 4 m pod fits), then an
# open courtyard (x 57-71: lawns, lamps at x 62.1-64, trees at (63, 30.5) and
# (63, 44)). The far tunnel ends at a hedge, so pods leave the way they came:
# they turn on the spot in the far paved band (x 65.5-70.5, just wider than
# the pod's 5.1 m turning circle) and drive back out. The near band is too
# shallow for a 4.6 m pod (building overhang at 57.5, a lamp at 62.1).
# Town05：ドローン充電所の隣の建物の中庭。建物を貫く通路（幅 6.5 m・天井 5.6 m）
# の先に中庭。反対側の通路は生け垣で塞がっているため、奥の舗装帯でその場旋回
# して同じ通路から戻る。手前の帯はポッド（4.6 m）には浅すぎる。
YARDS = {
    "Town05": {
        "name": "courtyard behind the drone station",
        "entry": (35.1, 46.0),            # road route ends here (curb lane, heading -y)
        "lane_y": 37.25,                  # the lane through both tunnels and the yard
        "turn_in": ((39.1, 41.25), 4.0, 180.0, 270.0),    # off the south road
        "pivot": (68.0, 0.0, 180.0),      # turn on the spot at this x: yaw 0 -> 180
        "turn_out": ((39.1, 33.25), 4.0, 90.0, 180.0),    # back onto the south road
        "exit": (35.1, 29.0),             # handed back to the road here (heading -y)
        # either side of the lane, clear of the turning circle (radius 2.55 m)
        "bays": [(68.0, 33.2), (68.0, 41.3)],
        "speeds": {"drive": 3.0, "turn": 2.0, "crab": 0.8, "pivot": 0.8},   # m/s
    },
}


def yard_for(map_name):
    """The yard for a CARLA map name like 'Carla/Maps/Town05', or None."""
    return YARDS.get(map_name.split("/")[-1])


def _line(a, b, speed, yaw=None, step=0.1):
    """Points a -> b; heading along the motion, or fixed ``yaw`` (crabbing)."""
    n = max(1, int(math.hypot(b[0] - a[0], b[1] - a[1]) / step))
    h = math.degrees(math.atan2(b[1] - a[1], b[0] - a[0])) if yaw is None else yaw
    return [(a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n, h, speed)
            for k in range(1, n + 1)]


def _arc(turn, speed, step=0.1):
    """Points around ``turn`` = (centre, radius, from deg, to deg); heading is
    the tangent (angle + 90 when the angle grows)."""
    (cx, cy), r, a0, a1 = turn
    n = max(1, int(abs(math.radians(a1 - a0)) * r / step))
    sign = 1.0 if a1 > a0 else -1.0
    out = []
    for k in range(1, n + 1):
        a = math.radians(a0 + (a1 - a0) * k / n)
        out.append((cx + r * math.cos(a), cy + r * math.sin(a),
                    math.degrees(a) + 90.0 * sign, speed))
    return out


def _pivot(x, y, yaw0, yaw1, speed, reach=2.55, step=0.1):
    """Turn on the spot from ``yaw0`` to ``yaw1``; points spaced so the pod's
    corners (``reach`` m out) move ``step`` m each. / その場旋回。"""
    n = max(1, int(abs(math.radians(yaw1 - yaw0)) * reach / step))
    return [(x, y, yaw0 + (yaw1 - yaw0) * k / n, speed) for k in range(1, n + 1)]


def path_in(yard, bay, start):
    """From the car at ``start`` (x, y) on the road into bay index ``bay``.

    [(x, y, yaw, speed)] every 10 cm. / 道路上の ``start`` からベイまでの経路。
    """
    sp = yard["speeds"]
    (cx, cy), r, a0, _ = yard["turn_in"]
    arc_start = (cx + r * math.cos(math.radians(a0)), cy + r * math.sin(math.radians(a0)))
    bx, by = yard["bays"][bay]
    lane = yard["lane_y"]
    pts = _line(start, arc_start, sp["drive"])
    pts += _arc(yard["turn_in"], sp["turn"])
    pts += _line(pts[-1][:2], (bx, lane), sp["drive"])
    pts += _line((bx, lane), (bx, by), sp["crab"], yaw=0.0)
    return pts


def path_out(yard, bay):
    """From bay index ``bay`` back out of the yard onto the road: slide into
    the lane, turn on the spot, drive out and turn onto the road."""
    sp = yard["speeds"]
    (cx, cy), r, a0, _ = yard["turn_out"]
    arc_start = (cx + r * math.cos(math.radians(a0)), cy + r * math.sin(math.radians(a0)))
    bx, by = yard["bays"][bay]
    lane = yard["lane_y"]
    px, yaw0, yaw1 = yard["pivot"]
    pts = _line((bx, by), (bx, lane), sp["crab"], yaw=yaw0)
    if abs(px - bx) > 0.05:
        pts += _line((bx, lane), (px, lane), sp["crab"], yaw=yaw0)
    pts += _pivot(px, lane, yaw0, yaw1, sp["pivot"])
    pts += _line((px, lane), arc_start, sp["drive"])
    pts += _arc(yard["turn_out"], sp["turn"])
    pts += _line(pts[-1][:2], yard["exit"], sp["drive"])
    return pts


class Mover(object):
    """Walks a pose list at each point's speed. ``step(dt)`` -> (x, y, yaw), done."""

    def __init__(self, points):
        self.points = points
        self.i = 0
        self._carry = 0.0

    @property
    def done(self):
        return self.i >= len(self.points) - 1

    def step(self, dt):
        if not self.done:
            # points are ~10 cm apart: advance by distance at this point's speed
            self._carry += self.points[self.i][3] * dt
            while self._carry >= 0.1 and not self.done:
                self._carry -= 0.1
                self.i += 1
        x, y, yaw, _ = self.points[self.i]
        return (x, y, yaw), self.done
