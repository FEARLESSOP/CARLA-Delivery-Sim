"""Delivery zones and no-fly zones (server-free; tests/test_zones.py).

Drawn on the dashboard's City Map, saved per map in data/delivery_zones.json:

  {"maps": {"OitaKosen": {
      "pods":  {"1": {"x0": .., "y0": .., "x1": .., "y1": ..}, ...},
      "nofly": [{"id": "nf1", "x0": .., "y0": .., "x1": .., "y1": ..}, ...],
      "meet":  {"1": [{"x": .., "y": ..}, ...], ...},
      "dist":  {"4": [{"x": .., "y": ..}, ...], ...},
      "charge": [{"x": .., "y": .., "yaw": ..}, ...]}}}

Rectangles are axis-aligned in CARLA world metres (x0 < x1, y0 < y1).

  - A pod with a zone only takes orders whose drop point is inside it; pods
    without a zone take the orders outside every pod zone.
  - Meeting points (per pod, picked on the map, snapped to a kerb point the
    pod can stop at): a pod that has any launches its drones and takes them
    back only there. Pods without any choose their stops themselves.
  - Distribution points (per LastMile hub, ``dist``, same rules as meeting
    points): a hub that has any parks only at the one best for its batch and
    sends its Mini Pods out from there; its meeting points are where the pods
    come back to. No-fly zones do not apply to the pods (ground robots), so
    distribution points inside one are fine.
  - Nothing flies over a no-fly zone: drone orders inside one are delivered
    face to face, drone stops (hubs) are never inside one, and every drone
    leg is routed around them (shortest path over the corners of the zones
    grown by a safety margin).
  - Charging stations (``charge``, up to MAX_CHARGE): where the LastMile
    charging stations (the web viewer's charge dock) stand, picked on the map
    before a launch. ``yaw`` is optional (degrees; the sim turns the station
    to face the nearest kerb lane when it is missing). The sim checks each
    spot (free ground, no building, off the road, flat) and reports the ones
    it cannot use; with the launch option "random" it places them itself.

配送ゾーンと飛行禁止区域（サーバー不要）。ダッシュボードの地図で描き、地図ごとに
data/delivery_zones.json に保存する。ゾーンを持つポッドはその中の注文だけを担当し、
ゾーン無しのポッドはどのゾーンにも入らない注文を担当する。飛行禁止区域の上は飛ばない
（中の注文は対面配達、ハブは置かない、飛行経路は区域の角を回って迂回）。
"""

import heapq
import json
import math
import os

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ZONES_PATH = os.path.join(HERE, "data", "delivery_zones.json")
MARGIN = 6.0          # metres kept between a drone and a no-fly zone / 区域との間隔
MIN_SIDE = 5.0        # smaller rectangles are dropped / これより小さい矩形は無効
MAX_NOFLY = 32
MAX_MEET = 24         # meeting points per pod / ポッドごとの合流地点の上限
MAX_DIST = 24         # distribution points per hub / ハブごとの配布地点の上限
MAX_CHARGE = 12       # charging stations per map / 地図ごとの充電ステーション上限
MEET_SNAP = 15.0      # a stop this close to a meeting point counts as one / この距離以内は合流地点
LIMIT = 1.0e5         # |coordinate| sanity bound / 座標の上限


def map_key(name):
    """'Japan/Maps/OitaKosen/OitaKosen' -> 'OitaKosen'."""
    return str(name or "").replace("\\", "/").rstrip("/").split("/")[-1]


def clean_rect(r):
    """A rect dict with sorted, finite corners, or None. / 正規化した矩形か None。"""
    if not isinstance(r, dict):
        return None
    try:
        x0, y0, x1, y1 = (float(r[k]) for k in ("x0", "y0", "x1", "y1"))
    except (KeyError, TypeError, ValueError):
        return None
    if not all(math.isfinite(v) and abs(v) < LIMIT for v in (x0, y0, x1, y1)):
        return None
    x0, x1 = min(x0, x1), max(x0, x1)
    y0, y1 = min(y0, y1), max(y0, y1)
    if x1 - x0 < MIN_SIDE or y1 - y0 < MIN_SIDE:
        return None
    return {"x0": round(x0, 1), "y0": round(y0, 1), "x1": round(x1, 1), "y1": round(y1, 1)}


def clean(entry):
    """Validate one map's {"pods", "nofly", "meet", "dist", "charge"}. Unknown keys are dropped."""
    entry = entry if isinstance(entry, dict) else {}
    pods = {}
    for k, r in (entry.get("pods") or {}).items():
        try:
            pid = int(k)
        except (TypeError, ValueError):
            continue
        c = clean_rect(r)
        if c is not None and 1 <= pid <= 99:
            pods[str(pid)] = c
    nofly, seen = [], set()
    for i, r in enumerate((entry.get("nofly") or [])[:MAX_NOFLY]):
        c = clean_rect(r)
        if c is None:
            continue
        nid = str((r.get("id") if isinstance(r, dict) else None) or "nf%d" % (i + 1))[:24]
        while nid in seen:
            nid += "_"
        seen.add(nid)
        c["id"] = nid
        if isinstance(r.get("name"), str) and r["name"].strip():
            c["name"] = r["name"].strip()[:40]
        nofly.append(c)
    return {"pods": pods, "nofly": nofly, "meet": _clean_points(entry.get("meet"), MAX_MEET),
            "dist": _clean_points(entry.get("dist"), MAX_DIST),
            "charge": clean_charge(entry.get("charge"))}


def clean_charge(pts):
    """[{"x", "y"[, "yaw"]}] charging-station spots: finite, at most MAX_CHARGE,
    yaw wrapped to (-180, 180]. / 充電ステーション地点を検証。"""
    out = []
    for p in (pts if isinstance(pts, list) else [])[:MAX_CHARGE]:
        try:
            x, y = float(p["x"]), float(p["y"])
        except (KeyError, TypeError, ValueError):
            continue
        if not (math.isfinite(x) and math.isfinite(y) and abs(x) < LIMIT and abs(y) < LIMIT):
            continue
        q = {"x": round(x, 1), "y": round(y, 1)}
        try:
            yaw = float(p.get("yaw"))
            if math.isfinite(yaw):
                q["yaw"] = round((yaw + 180.0) % 360.0 - 180.0, 1) or 0.0
        except (TypeError, ValueError):
            pass
        out.append(q)
    return out


def _clean_points(table, limit):
    """{"<vid>": [{"x", "y"}]} with valid ids (1..99), finite points, at most
    ``limit`` per vehicle. / 車両ごとの点のリストを検証。"""
    out = {}
    for k, pts in (table if isinstance(table, dict) else {}).items():
        try:
            pid = int(k)
        except (TypeError, ValueError):
            continue
        keep = []
        for p in (pts if isinstance(pts, list) else [])[:limit]:
            try:
                x, y = float(p["x"]), float(p["y"])
            except (KeyError, TypeError, ValueError):
                continue
            if math.isfinite(x) and math.isfinite(y) and abs(x) < LIMIT and abs(y) < LIMIT:
                keep.append({"x": round(x, 1), "y": round(y, 1)})
        if keep and 1 <= pid <= 99:
            out[str(pid)] = keep
    return out


def load_all(path=None):
    try:
        with open(path or ZONES_PATH) as fh:
            data = json.load(fh)
        return data if isinstance(data, dict) and isinstance(data.get("maps"), dict) else {"maps": {}}
    except (OSError, ValueError):
        return {"maps": {}}


def load(map_name, path=None):
    """This map's zones (empty when none are saved). / この地図のゾーン。"""
    return clean(load_all(path)["maps"].get(map_key(map_name)))


def save(map_name, entry, path=None):
    """Store one map's zones (atomic write); returns the cleaned entry."""
    path = path or ZONES_PATH
    data = load_all(path)
    entry = clean(entry)
    data["maps"][map_key(map_name)] = entry
    data["_about"] = ("Pod delivery zones and no-fly zones per map (CARLA world metres), "
                      "drawn on the dashboard's City Map. / 地図ごとの配送ゾーンと飛行禁止区域")
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w") as fh:
        json.dump(data, fh, indent=1)
    os.replace(tmp, path)
    return entry


# -- geometry / 幾何
def inside(r, p, grow=0.0):
    return r["x0"] - grow <= p[0] <= r["x1"] + grow and r["y0"] - grow <= p[1] <= r["y1"] + grow


def _seg_hits_rect(a, b, r):
    """Does segment a-b pass through the open interior of rect r? (Liang-Barsky)"""
    x0, y0, x1, y1 = r["x0"], r["y0"], r["x1"], r["y1"]
    dx, dy = b[0] - a[0], b[1] - a[1]
    t0, t1 = 0.0, 1.0
    for p, q in ((-dx, a[0] - x0), (dx, x1 - a[0]), (-dy, a[1] - y0), (dy, y1 - a[1])):
        if p == 0.0:
            if q <= 0.0:
                return False      # parallel and outside (or on the edge) / 平行で外側
            continue
        t = q / p
        if p < 0.0:
            t0 = max(t0, t)
        else:
            t1 = min(t1, t)
        if t0 >= t1:
            return False
    return t1 - t0 > 1e-6


def _grown(r, m):
    return {"x0": r["x0"] - m, "y0": r["y0"] - m, "x1": r["x1"] + m, "y1": r["y1"] + m}


class Airspace(object):
    """The no-fly zones of one map and the detours around them.

    ``path(a, b)`` returns the horizontal waypoints from a to b (b included,
    a not), going round every no-fly zone with ``margin`` to spare. A zone
    that contains a or b itself is ignored for that leg (a drone already
    inside leaves by the straight line; zones are kept off the drop points and
    hubs by the planner, so that only happens when a zone is drawn live).
    飛行禁止区域と迂回経路。``path(a, b)`` は a→b の水平経由点（b を含む）。
    """

    def __init__(self, nofly=(), margin=MARGIN):
        self.margin = margin
        self.zones = [clean_rect(r) or r for r in nofly]
        # the corners the paths turn at sit a bit outside the margin, so a
        # corner-to-corner line along an edge does not count as a hit
        # 経由点は余裕より少し外側（辺に沿う線を交差扱いしない）
        self._walls = [_grown(r, margin) for r in self.zones]
        self._corners = []
        for r in self.zones:
            g = _grown(r, margin * 1.05 + 0.5)
            self._corners += [(g["x0"], g["y0"]), (g["x1"], g["y0"]),
                              (g["x1"], g["y1"]), (g["x0"], g["y1"])]

    def __bool__(self):
        return bool(self.zones)

    def blocked(self, p, grow=None):
        """Is point p inside a zone (grown by ``grow``, default the margin)?"""
        g = self.margin if grow is None else grow
        return any(inside(r, p, g) for r in self.zones)

    def _clear(self, a, b, walls):
        return not any(_seg_hits_rect(a, b, w) for w in walls)

    def path(self, a, b):
        a2, b2 = (a[0], a[1]), (b[0], b[1])
        walls = [w for w in self._walls if not inside(w, a2) and not inside(w, b2)]
        if not walls or self._clear(a2, b2, walls):
            return [b2]
        nodes = [a2, b2] + [c for c in self._corners if not any(inside(w, c) for w in walls)]
        # Dijkstra over the visibility graph / 可視グラフ上の最短経路
        best, prev, heap = {0: 0.0}, {}, [(0.0, 0)]
        while heap:
            d, i = heapq.heappop(heap)
            if i == 1:
                break
            if d > best.get(i, 1e18):
                continue
            for j in range(1, len(nodes)):
                if j == i:
                    continue
                nd = d + math.dist(nodes[i], nodes[j])
                if nd < best.get(j, 1e18) and self._clear(nodes[i], nodes[j], walls):
                    best[j], prev[j] = nd, i
                    heapq.heappush(heap, (nd, j))
        if 1 not in prev:
            return [b2]           # boxed in: fly straight / 経路なし：直行
        out, i = [], 1
        while i != 0:
            out.append(nodes[i])
            i = prev[i]
        return out[::-1]

    def length(self, a, b):
        pts, here, total = self.path(a, b), (a[0], a[1]), 0.0
        for p in pts:
            total += math.dist(here, p)
            here = p
        return total


class Zones(object):
    """One map's pod zones + airspace, as the delivery sim uses them."""

    def __init__(self, entry=None, margin=MARGIN):
        e = clean(entry)
        self.pods = {int(k): r for k, r in e["pods"].items()}
        self.nofly = e["nofly"]
        self.meet = {int(k): [(p["x"], p["y"]) for p in pts] for k, pts in e["meet"].items()}
        self.dist = {int(k): [(p["x"], p["y"]) for p in pts] for k, pts in e["dist"].items()}
        self.charge = e["charge"]           # charging-station spots (read at start) / 充電所
        self.air = Airspace(self.nofly, margin)

    def zone_of(self, vid):
        return self.pods.get(vid)

    def owner(self, p, vids):
        """The pod (of ``vids``) whose zone holds p; smallest zone wins an
        overlap. None: outside every pod zone. / p を担当するポッド。"""
        cands = [v for v in vids if self.pods.get(v) is not None and inside(self.pods[v], p)]
        if not cands:
            return None
        area = lambda v: (self.pods[v]["x1"] - self.pods[v]["x0"]) * (self.pods[v]["y1"] - self.pods[v]["y0"])
        return min(cands, key=lambda v: (area(v), v))

    def meet_of(self, vid):
        """Pod ``vid``'s meeting points, those clear of the no-fly zones."""
        return [p for p in self.meet.get(vid, ()) if not self.air.blocked(p)]

    def meet_all(self, vid):
        """Hub ``vid``'s meeting points, no-fly zones ignored (Mini Pods are
        ground robots). / ハブの合流地点（飛行禁止区域は無関係）。"""
        return list(self.meet.get(vid, ()))

    def dist_of(self, vid):
        """Hub ``vid``'s distribution points (no-fly zones do not apply to the
        Mini Pods). / ハブの配布地点。"""
        return list(self.dist.get(vid, ()))

    def summary(self):
        pts = lambda table: {str(k): [{"x": x, "y": y} for x, y in v] for k, v in sorted(table.items())}
        return {"pods": {str(k): r for k, r in sorted(self.pods.items())}, "nofly": self.nofly,
                "meet": pts(self.meet), "dist": pts(self.dist), "charge": self.charge}


# The airspace the drones fly in right now (set by delivery_sim when the zones
# change; ``ver`` tells flights in the air to re-plan their detours).
# 現在の空域（ゾーン変更時に delivery_sim が設定。ver で飛行中の機体が再計画）
ACTIVE = {"air": None, "ver": 0}


def set_active(air):
    ACTIVE["air"] = air if air else None
    ACTIVE["ver"] += 1


def air_dist(a, b):
    """Horizontal flight distance a -> b, detours included. / 迂回込みの水平距離。"""
    air = ACTIVE["air"]
    return air.length(a, b) if air else math.hypot(b[0] - a[0], b[1] - a[1])


def air_path(a, b):
    air = ACTIVE["air"]
    return air.path(a, b) if air else [(b[0], b[1])]
