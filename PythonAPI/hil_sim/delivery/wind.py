"""Wind and how it pushes a drone that fights it (no carla import).

The wind is a base breeze that slowly wanders, a little turbulence, and
gusts: random ones (a Poisson process) or ones asked for from the
dashboard. Directions are the way the wind blows TOWARD, as a CARLA yaw in
degrees (0 = +x, 90 = +y).

A drone holds its position against the wind, but its controller only learns
about a change in the wind after a lag and can only push back so hard. So a
sudden gust shoves it sideways for a moment (``Drift``), and wind stronger
than the drone can fight blows it away steadily: into a building, or so far
off course that it is lost.

風と、それに抗うドローンの流され方（carla 非依存）。風はゆっくり変化する
基本風・乱流・突風（ランダムまたはダッシュボードから）。向きは風が吹いて
いく方向（CARLA の yaw [度]）。ドローンの制御は風の変化に遅れて反応し、
押し返せる力にも上限があるため、突風では一時的に流され、限界を超える風では
流され続ける（建物への衝突やロスト）。
"""

import math

# drift controller: tuned so a gust the drone can fight pushes it ~1-2 m,
# and one beyond its limit keeps pushing (see tests)
# ドリフト制御：抗える突風で約 1〜2 m、限界を超えると流され続けるよう調整
REACT_S = 0.8           # how fast the controller notices a wind change [s]
HOLD_GAIN = 1.2         # pull back towards the planned path [1/s]


def _vec(speed, direction):
    r = math.radians(direction)
    return (speed * math.cos(r), speed * math.sin(r))


class Wind(object):
    """Base wind + turbulence + gusts. ``step(t, dt)`` then ``vector(t)``."""

    def __init__(self, rng, base_speed=3.0, base_dir=None, gust_rate=0.5,
                 gust_min=5.0, gust_max=14.0, gust_s=(3.0, 9.0)):
        self.rng = rng
        self.base_speed = base_speed
        self.base_dir = rng.uniform(0.0, 360.0) if base_dir is None else base_dir
        self.gust_rate = gust_rate          # random gusts per minute / 毎分の突風回数
        self.gust_min, self.gust_max, self.gust_s = gust_min, gust_max, gust_s
        self.gusts = []                     # dicts: t0, dur, peak, dir, manual
        self.history = []                   # finished and running gusts, for the log

    def step(self, t, dt):
        # the breeze wanders a few degrees a second / 基本風は毎秒数度ゆらぐ
        self.base_dir = (self.base_dir + self.rng.gauss(0.0, 2.0) * math.sqrt(dt)) % 360.0
        self.gusts = [g for g in self.gusts if t < g["t0"] + g["dur"]]
        if self.gust_rate > 0 and self.rng.random() < self.gust_rate / 60.0 * dt:
            self.gust(t, self.rng.uniform(self.gust_min, self.gust_max),
                      (self.base_dir + self.rng.uniform(-40.0, 40.0)) % 360.0,
                      self.rng.uniform(*self.gust_s), manual=False)

    def gust(self, t, speed, direction=None, duration=6.0, manual=True):
        """Start a gust of ``speed`` m/s at its peak. / 突風を起こす。"""
        g = {"t0": t, "dur": max(duration, 0.5), "peak": max(speed, 0.0),
             "dir": self.base_dir if direction is None else direction % 360.0,
             "manual": manual}
        self.gusts.append(g)
        self.history.append(g)
        return g

    def set_base(self, speed=None, direction=None):
        if speed is not None:
            self.base_speed = max(speed, 0.0)
        if direction is not None:
            self.base_dir = direction % 360.0

    def calm(self):
        self.base_speed = 0.0
        self.gusts = []

    @staticmethod
    def _envelope(g, t):
        u = (t - g["t0"]) / g["dur"]
        return math.sin(math.pi * u) ** 2 if 0.0 <= u <= 1.0 else 0.0

    def vector(self, t):
        """Wind velocity (wx, wy) in m/s at time ``t``."""
        turb = 1.0 + 0.10 * math.sin(0.7 * t) + 0.07 * math.sin(1.9 * t + 1.0)
        wx, wy = _vec(self.base_speed * turb, self.base_dir)
        for g in self.gusts:
            gx, gy = _vec(g["peak"] * self._envelope(g, t), g["dir"])
            wx, wy = wx + gx, wy + gy
        return wx, wy

    def state(self, t):
        wx, wy = self.vector(t)
        speed = math.hypot(wx, wy)
        live = [g for g in self.gusts if self._envelope(g, t) > 0.0]
        return {"speed": round(speed, 2),
                "dir": round(math.degrees(math.atan2(wy, wx)) % 360.0, 1),
                "base_speed": round(self.base_speed, 2), "base_dir": round(self.base_dir, 1),
                "gust": bool(live),
                "gust_peak": round(max([g["peak"] for g in live] or [0.0]), 1),
                "gust_rate": self.gust_rate}


class Drift(object):
    """How far the wind has pushed one drone off its planned path (x, y) [m].

    The controller's idea of the wind lags the real wind by ``REACT_S``; it
    pushes back with that estimate plus a pull towards the path, but never
    harder than ``max_wind`` m/s. ``assist`` (precision landing into the pod)
    pulls the drone straight back onto its path.
    制御が推定する風は実際の風から ``REACT_S`` 遅れる。推定風＋経路への引き戻し
    で押し返すが、``max_wind`` m/s を超えては押し返せない。
    """

    def __init__(self, max_wind=11.0):
        self.max_wind = max_wind
        self.d = (0.0, 0.0)
        self.w_est = None
        self.saturated = False      # the wind is stronger than the drone / 限界超え

    def reset(self):
        self.d, self.w_est, self.saturated = (0.0, 0.0), None, False

    @property
    def offset(self):
        return math.hypot(*self.d)

    def step(self, dt, wind, assist=False):
        if self.w_est is None:
            self.w_est = tuple(wind)    # took off already knowing the breeze
        k = min(dt / REACT_S, 1.0)
        self.w_est = tuple(e + (w - e) * k for e, w in zip(self.w_est, wind))
        cx = self.w_est[0] + HOLD_GAIN * self.d[0]
        cy = self.w_est[1] + HOLD_GAIN * self.d[1]
        mag = math.hypot(cx, cy)
        self.saturated = mag > self.max_wind
        if self.saturated:
            cx, cy = cx * self.max_wind / mag, cy * self.max_wind / mag
        dx = self.d[0] + (wind[0] - cx) * dt
        dy = self.d[1] + (wind[1] - cy) * dt
        if assist:
            keep = math.exp(-4.0 * dt)
            dx, dy = dx * keep, dy * keep
        self.d = (dx, dy)
        return self.d
