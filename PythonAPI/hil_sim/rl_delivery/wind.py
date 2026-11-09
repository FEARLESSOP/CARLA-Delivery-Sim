"""Realistic wind for the pretend world (hidden from every AI brain).

Three layers, the way flight simulators model low-altitude wind:
1. Mean wind: a speed at 10 m that drifts slowly, a direction that wanders,
   and a log-law profile (stronger higher up, z0 for a suburb).
2. Dryden turbulence (MIL-F-8785C, low altitude): random buffeting whose
   strength and length scales follow the mean wind and the height.
3. Discrete gusts: smooth "1-cosine" bumps with random start, length,
   strength and direction (they never jump).
Plus shelter: calmer, more turbulent air in a building's wake, and a little
speed-up between buildings (simple, from a building height grid).

Limits follow small delivery drones: ~12 m/s mean, gust peaks ~15 m/s.

現実的な風（平均風・Dryden 乱流・1-cos 突風・建物の影響）。AI には見せない。
"""
import math

import numpy as np

FT = 0.3048
Z0 = 0.5                  # roughness length of a suburb [m] / 郊外の粗度長
REF_AGL = 30.0            # the limits hold at this flight height / 上限はこの高度で
PROFILE = math.log(REF_AGL / Z0) / math.log(10.0 / Z0)


class BuildingGrid(object):
    """Max building top above ground on a coarse grid, for shelter lookups."""

    def __init__(self, buildings, bounds, cell=10.0):
        x0, y0, x1, y1 = bounds
        self.x0, self.y0, self.cell = x0, y0, cell
        self.nx, self.ny = int((x1 - x0) / cell) + 2, int((y1 - y0) / cell) + 2
        self.h = np.zeros((self.nx, self.ny), np.float32)
        for b in buildings:
            r = max(b["len"], b["wid"]) / 2.0
            i0, i1 = int((b["x"] - r - x0) / cell), int((b["x"] + r - x0) / cell)
            j0, j1 = int((b["y"] - r - y0) / cell), int((b["y"] + r - y0) / cell)
            i0, j0 = max(i0, 0), max(j0, 0)
            i1, j1 = min(i1, self.nx - 1), min(j1, self.ny - 1)
            if i0 <= i1 and j0 <= j1:
                self.h[i0:i1 + 1, j0:j1 + 1] = np.maximum(self.h[i0:i1 + 1, j0:j1 + 1], b["h"])

    def top(self, x, y):
        i, j = int((x - self.x0) / self.cell), int((y - self.y0) / self.cell)
        if 0 <= i < self.nx and 0 <= j < self.ny:
            return float(self.h[i, j])
        return 0.0


class Wind(object):
    """One episode's wind. ``step(dt)`` then ``at(x, y, agl)`` -> (vx, vy, vz) m/s."""

    def __init__(self, rng, max_mean=12.0, max_gust=15.0, level=1.0, grid=None):
        self.rng = rng
        self.grid = grid
        self.max_mean, self.max_gust = max_mean, max_gust
        self.set_level(level)
        self.t = 0.0
        self.dir = rng.uniform(0, 2 * math.pi)
        self.turb = np.zeros(3)
        self.gusts = []          # [start, length, peak, dir] / 突風
        self.next_gust = self._gap()

    def set_level(self, level):
        """0 = calm, 1 = the full range (curriculum). / 風の強さの範囲"""
        self.level = max(0.0, min(1.0, level))
        self.u10_max = self.max_mean / PROFILE      # mean at 30 m <= max_mean
        self.u10_target = self.rng.uniform(0.0, self.u10_max * self.level)
        self.u10 = self.u10_target * self.rng.uniform(0.7, 1.0)

    def _gap(self):
        if self.level <= 0:
            return 1e9
        rate = 0.02 + 0.08 * self.u10 / max(self.u10_max, 1e-6)   # gusts per second
        return self.t + self.rng.exponential(1.0 / rate)

    def step(self, dt, agl=30.0):
        self.t += dt
        # slow drift of the mean / 平均風の緩やかな変化
        self.u10 += (self.u10_target - self.u10) * dt / 120.0 + self.rng.normal(0, 0.03) * math.sqrt(dt)
        self.u10 = max(0.0, min(self.u10_max, self.u10))
        self.dir += self.rng.normal(0, 0.01) * math.sqrt(dt)
        if self.rng.random() < dt / 300.0:
            self.u10_target = self.rng.uniform(0.0, self.u10_max * self.level)
        # Dryden turbulence, first-order shaping filters (MIL-F-8785C)
        h_ft = max(agl, 3.0) / FT
        u20 = self.u10 * math.log(20.0 / Z0) / math.log(10.0 / Z0) / FT
        sw = 0.1 * u20
        su = sw / (0.177 + 0.000823 * h_ft) ** 0.4
        lw = h_ft
        lu = h_ft / (0.177 + 0.000823 * h_ft) ** 1.2
        v = max(self.mean_speed(agl), 1.0) / FT
        for k, (s, L) in enumerate(((su, lu), (su, lu), (sw, lw))):
            a = min(v * dt / L, 1.0)
            self.turb[k] = (1 - a) * self.turb[k] + s * FT * math.sqrt(2 * a) * self.rng.normal()
        # gusts / 突風
        if self.t >= self.next_gust:
            m30 = self.u10 * PROFILE
            room = max(0.0, self.max_gust - m30)
            peak = self.rng.uniform(0.3, 1.0) * min(room, 0.6 * m30 + 2.0)
            self.gusts.append([self.t, self.rng.uniform(2.0, 12.0), peak,
                               self.dir + self.rng.normal(0, 0.5)])
            self.next_gust = self._gap()
        self.gusts = [g for g in self.gusts if self.t < g[0] + g[1]]

    def mean_speed(self, agl):
        return self.u10 * math.log(max(agl, Z0 * 2) / Z0) / math.log(10.0 / Z0)

    def at(self, x, y, agl):
        """Wind vector at a point (agl = height above ground). / その地点の風"""
        u = min(self.mean_speed(agl), self.max_mean * 1.1)
        c, s = math.cos(self.dir), math.sin(self.dir)
        turb = self.turb.copy()
        if self.grid is not None and u > 0.5:
            # wake: a building upwind taller than us shelters us / 風上の建物の陰
            shelter = 0.0
            for d in (8.0, 16.0, 30.0, 50.0):
                top = self.grid.top(x - c * d, y - s * d)
                if top > agl and d < 5.0 * top:
                    shelter = max(shelter, 1.0 - d / (5.0 * top))
            if shelter > 0:
                u *= 1.0 - 0.6 * shelter
                turb *= 1.0 + 0.8 * shelter
            elif self.grid.top(x - s * 12, y + c * 12) > agl and self.grid.top(x + s * 12, y - c * 12) > agl:
                u *= 1.2            # channelled between two buildings / 建物間の加速
        vx, vy = u * c + turb[0] * c - turb[1] * s, u * s + turb[0] * s + turb[1] * c
        for g0, glen, peak, gd in self.gusts:
            ph = (self.t - g0) / glen
            if 0.0 <= ph <= 1.0:
                a = 0.5 * peak * (1 - math.cos(2 * math.pi * ph))
                vx += a * math.cos(gd)
                vy += a * math.sin(gd)
        return np.array([vx, vy, turb[2]])
