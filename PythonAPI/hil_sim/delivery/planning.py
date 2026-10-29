"""Server-free planning logic for the delivery mission.

Everything here works on plain (x, y, z) tuples in CARLA world metres, so it
can be unit-tested without a CARLA server (tests/test_delivery.py).

配送ミッションの計画ロジック（サーバー不要）。座標はすべて CARLA ワールド
座標 [m] の (x, y, z) タプルで、CARLA サーバーなしでテストできる。
"""

import itertools
import math
import random
from dataclasses import dataclass, field

FACE_TO_FACE = "face_to_face"
DRONE = "drone"
CHARGE = "charge"     # a stop at an EV charger / EV 充電器への停車
ORDER_KINDS = (FACE_TO_FACE, DRONE)

# brute-force tours up to this many points, nearest-neighbour + 2-opt above
# 全探索する最大点数（これを超えると最近傍法 + 2-opt）
EXACT_TOUR_LIMIT = 8


def dist2d(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def centroid(points):
    n = float(len(points))
    return tuple(sum(p[i] for p in points) / n for i in range(3))


@dataclass
class Building(object):
    """A deliverable building: where the drone drops and where the car parks.

    配送先の建物。ドローンの投下地点と車の停車地点を持つ。
    """
    id: int
    name: str
    drop: tuple          # sidewalk point in front of the building / 建物前の歩道
    curb: tuple          # driving-lane point the car stops at / 車の停車地点


@dataclass
class Order(object):
    id: int
    kind: str
    building: Building
    created_t: float
    batch: int
    delivered_t: float = None
    van: int = None          # which van owns it once dispatched / 担当車
    failed_t: float = None
    fail_reason: str = None  # why it could not be delivered / 配達失敗の理由

    @property
    def drop(self):
        return self.building.drop


@dataclass
class OrderGenerator(object):
    """Releases a batch of 3-5 orders at random times.

    Each batch arrives a random ``min_gap..max_gap`` seconds (sim time) after
    the previous one, divided by ``fleet_size``: twice the vans, twice the
    orders. ``prompt`` brings the next batch forward (an idle van with nothing
    queued). Drone orders in one batch are drawn within ``drone_radius`` of
    each other so one drone sortie can serve them all.

    ランダムな時刻に 3〜5 件の注文バッチを発生させる。間隔は
    ``min_gap..max_gap`` 秒を ``fleet_size`` で割った値（車が倍なら注文も倍）。
    ``prompt`` で次のバッチを前倒しする（待機中の車がいて注文が無いとき）。
    """
    buildings: list
    rng: random.Random
    min_gap: float = 20.0
    max_gap: float = 90.0
    min_batch: int = 3
    max_batch: int = 5
    drone_prob: float = 0.5
    drone_radius: float = 150.0
    fleet_size: int = 1      # gaps shrink with the number of vans / 車の台数で間隔を縮める
    next_t: float = None
    _next_order_id: int = 1
    _next_batch_id: int = 1
    busy_buildings: set = field(default_factory=set)
    skipped: int = 0         # batches dropped because every building was busy

    def schedule_from(self, t, immediate=False):
        gap = self.rng.uniform(self.min_gap, self.max_gap) / max(self.fleet_size, 1)
        self.next_t = t if immediate else t + gap

    def prompt(self, t, delay):
        """Have the next batch arrive within ``delay`` seconds. / 次を前倒し。"""
        if self.next_t is None or self.next_t > t + delay:
            self.next_t = t + delay

    def poll(self, t):
        """Return a new batch (list of Order) if one is due at sim time t, else [].

        時刻 t に到着するバッチがあれば Order のリストを、なければ [] を返す。
        """
        if self.next_t is None:
            self.schedule_from(t)
        if t < self.next_t:
            return []
        self.schedule_from(t)
        return self.make_batch(t)

    def make_batch(self, t):
        n = self.rng.randint(self.min_batch, self.max_batch)
        kinds = [DRONE if self.rng.random() < self.drone_prob else FACE_TO_FACE
                 for _ in range(n)]
        free = [b for b in self.buildings if b.id not in self.busy_buildings]
        if not free:
            # every building already has an undelivered order (the vans are
            # far behind): drop this batch rather than double-book a building
            # 全建物に未配達の注文がある（車が大幅に遅れている）：同じ建物に
            # 二重に注文を入れず、このバッチを見送る
            self.skipped += 1
            return []
        chosen = []
        n_drone = kinds.count(DRONE)
        if n_drone:
            chosen += self._pick_cluster(free, n_drone)
        rest = [b for b in free if b not in chosen]
        chosen += self.rng.sample(rest, min(n - len(chosen), len(rest)))
        if len(chosen) < n:     # map too small / all buildings busy
            kinds = kinds[:len(chosen)]
        # cluster first, so drone kinds line up with the clustered buildings
        kinds.sort(key=lambda k: k != DRONE)
        batch_id = self._next_batch_id
        self._next_batch_id += 1
        orders = []
        for kind, b in zip(kinds, chosen):
            orders.append(Order(self._next_order_id, kind, b, t, batch_id))
            self._next_order_id += 1
            self.busy_buildings.add(b.id)
        return orders

    def release(self, order):
        self.busy_buildings.discard(order.building.id)

    def _pick_cluster(self, free, k):
        seeds = list(free)
        self.rng.shuffle(seeds)
        for seed in seeds:
            near = [b for b in free
                    if b is not seed and dist2d(b.drop, seed.drop) <= self.drone_radius]
            if len(near) >= k - 1:
                return [seed] + self.rng.sample(near, k - 1)
        # no cluster big enough: fall back to the k closest to a random seed
        seed = seeds[0]
        return sorted(free, key=lambda b: dist2d(b.drop, seed.drop))[:k]


class Dispatcher(object):
    """Hands each batch of orders to exactly one van.

    Batches wait in arrival order. Whenever vans are free, the oldest batch
    goes to the free van nearest to it, then the next oldest to the nearest
    of the remaining free vans, and so on. A batch leaves the queue the moment
    it is assigned, so two vans can never work the same order.

    注文バッチを必ず 1 台の車だけに割り当てる。バッチは到着順に待ち、空いて
    いる車があれば、最も古いバッチをそれに最も近い空車へ、次のバッチを残りの
    空車のうち最も近い車へ…と割り当てる。割り当てた時点でキューから外すため、
    2 台が同じ注文を担当することはない。
    """

    def __init__(self):
        self.queue = []

    def add(self, batch):
        if batch:
            self.queue.append(list(batch))

    def push_front(self, batch):
        """A batch a van gave back (it must charge first): served next."""
        if batch:
            for o in batch:
                o.van = None
            self.queue.insert(0, list(batch))

    @property
    def orders(self):
        return [o for b in self.queue for o in b]

    def assign(self, free_vans):
        """``free_vans``: list of (van_id, (x, y, z)). Returns [(van_id, batch)].

        空車のリスト [(車 ID, 位置)] を受け取り、[(車 ID, バッチ)] を返す。
        """
        free = list(free_vans)
        out = []
        while self.queue and free:
            batch = self.queue.pop(0)
            c = centroid([o.drop for o in batch])
            pick = min(free, key=lambda v: dist2d(v[1], c))
            free.remove(pick)
            for o in batch:
                o.van = pick[0]
            out.append((pick[0], batch))
        return out


def tour_length(start, points, order, end=None, dist=dist2d):
    """Length of start -> points[order...] -> end (end defaults to start)."""
    end = start if end is None else end
    path = [start] + [points[i] for i in order] + [end]
    return sum(dist(path[i], path[i + 1]) for i in range(len(path) - 1))


def best_tour(start, points, end=None, dist=dist2d):
    """Shortest visiting order of ``points`` from ``start`` back to ``end``.

    Exact for small inputs (a delivery batch is at most 5), heuristic above.
    Returns (order, length) where order indexes into points.

    ``start`` から全 ``points`` を回って ``end`` に戻る最短順序を返す。
    """
    n = len(points)
    if n == 0:
        return [], (0.0 if end is None else dist(start, end))
    if n <= EXACT_TOUR_LIMIT:
        best = min(itertools.permutations(range(n)),
                   key=lambda o: tour_length(start, points, o, end, dist))
        return list(best), tour_length(start, points, best, end, dist)
    # nearest neighbour, then 2-opt
    left, order, cur = set(range(n)), [], start
    while left:
        nxt = min(left, key=lambda i: dist(cur, points[i]))
        order.append(nxt)
        left.discard(nxt)
        cur = points[nxt]
    improved = True
    while improved:
        improved = False
        for i in range(n - 1):
            for j in range(i + 1, n):
                cand = order[:i] + order[i:j + 1][::-1] + order[j + 1:]
                if tour_length(start, points, cand, end, dist) < \
                        tour_length(start, points, order, end, dist) - 1e-9:
                    order, improved = cand, True
    return order, tour_length(start, points, order, end, dist)


def choose_hub(drop_points, candidates, search_radius=120.0, max_candidates=200):
    """Pick the road point the car parks at to launch the drone.

    Starts from the centroid of the drone orders, then among the road
    candidates near it picks the one whose round-trip drone tour
    (hub -> every drop -> hub) is shortest. Returns (hub, order, length).

    ドローンを発進させる停車地点を選ぶ。ドローン注文の重心を起点に、その
    周辺の道路候補点の中から、往復ツアー（ハブ→全投下点→ハブ）が最短に
    なる点を選ぶ。(hub, 順序, 距離) を返す。
    """
    if not drop_points:
        raise ValueError("choose_hub needs at least one drop point")
    if not candidates:
        raise ValueError("choose_hub needs at least one road candidate")
    c = centroid(drop_points)
    by_dist = sorted(candidates, key=lambda p: dist2d(p, c))
    near = [p for p in by_dist if dist2d(p, c) <= search_radius] or by_dist[:30]
    best = None
    for hub in near[:max_candidates]:
        order, length = best_tour(hub, drop_points)
        if best is None or length < best[2]:
            best = (hub, order, length)
    return best


def order_stops(start, stops, dist):
    """Visit order for the car's stops (open path from ``start``).

    ``dist(a, b)`` should be the road distance; stops are opaque objects
    with a ``.location`` tuple. Exact search — a batch has at most 6 stops.

    車の停車順を決める（``start`` からの片道）。``dist`` は道路距離。
    """
    if not stops:
        return []
    pts = [s.location for s in stops]
    cache = {}

    def d(a, b):
        key = (a, b)
        if key not in cache:
            cache[key] = dist(a, b)
        return cache[key]

    def path_len(o):
        path = [start] + [pts[i] for i in o]
        return sum(d(path[k], path[k + 1]) for k in range(len(o)))

    best = min(itertools.permutations(range(len(stops))), key=path_len)
    return [stops[i] for i in best]


@dataclass
class Stop(object):
    """One place the car drives to: a face-to-face handover or the drone hub.

    車が向かう 1 か所：対面受け渡し、またはドローン発進ハブ。
    """
    kind: str               # FACE_TO_FACE, DRONE or CHARGE (an EV charger, no orders)
    location: tuple
    orders: list
    drone_order: list = None   # drop visiting order for a drone stop

    @property
    def batch(self):
        return self.orders[0].batch if self.orders else None


def plan_batch(orders, car_location, hub_candidates, road_dist, hub_radius=120.0):
    """Turn a batch of orders into an ordered list of car stops.

    Face-to-face orders become one stop each (at the building's curb); all
    drone orders share a single hub stop chosen by :func:`choose_hub`.

    注文バッチを車の停車リストに変換する。対面注文は 1 件 1 停車（建物前の
    路肩）、ドローン注文はまとめて :func:`choose_hub` で選んだハブ 1 か所。
    """
    stops = [Stop(FACE_TO_FACE, o.building.curb, [o])
             for o in orders if o.kind == FACE_TO_FACE]
    drone_orders = [o for o in orders if o.kind == DRONE]
    if drone_orders:
        drops = [o.drop for o in drone_orders]
        hub, visit, _ = choose_hub(drops, hub_candidates, hub_radius)
        stops.append(Stop(DRONE, hub, drone_orders,
                          [drone_orders[i] for i in visit]))
    return order_stops(car_location, stops, road_dist)


@dataclass
class Obstacle(object):
    """Footprint + top of something the drone must fly over (building, tree...).

    ドローンが越えるべき物体（建物・樹木など）の平面形状と上端高さ。
    """
    x: float
    y: float
    ex: float            # half extent along the box's own x / 自身の x 方向半幅
    ey: float
    yaw: float           # degrees / 度
    top: float           # world z of its highest point / 最高点のワールド z


def _seg_point_dist(ax, ay, bx, by, px, py):
    dx, dy = bx - ax, by - ay
    n2 = dx * dx + dy * dy
    u = 0.0 if n2 == 0.0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / n2))
    return math.hypot(ax + u * dx - px, ay + u * dy - py)


def _seg_box_dist(a, b, o):
    """Horizontal distance from segment a-b to obstacle o's oriented footprint."""
    c, s = math.cos(math.radians(o.yaw)), math.sin(math.radians(o.yaw))

    def local(p):           # into the box frame / 箱の座標系へ
        dx, dy = p[0] - o.x, p[1] - o.y
        return (dx * c + dy * s, -dx * s + dy * c)

    (ax, ay), (bx, by) = local(a), local(b)
    # segment crosses the box? clip it (Liang-Barsky)
    t0, t1, dx, dy = 0.0, 1.0, bx - ax, by - ay
    inside = True
    for p, q in ((-dx, ax + o.ex), (dx, o.ex - ax), (-dy, ay + o.ey), (dy, o.ey - ay)):
        if p == 0.0:
            if q < 0.0:
                inside = False
                break
        else:
            r = q / p
            if p < 0.0:
                t0 = max(t0, r)
            else:
                t1 = min(t1, r)
            if t0 > t1:
                inside = False
                break
    if inside:
        return 0.0
    # otherwise the closest pair involves a segment end or a box corner
    corners = [(sx * o.ex, sy * o.ey) for sx in (-1, 1) for sy in (-1, 1)]

    def box_point_dist(px, py):
        return math.hypot(max(abs(px) - o.ex, 0.0), max(abs(py) - o.ey, 0.0))

    return min([box_point_dist(ax, ay), box_point_dist(bx, by)] +
               [_seg_point_dist(ax, ay, bx, by, qx, qy) for qx, qy in corners])


def leg_altitude(a, b, obstacles, corridor=5.0, margin=8.0, min_alt=15.0, surface=None,
                 end_clear=6.0, path_width=1.0):
    """Cruise z for flying level from a to b.

    ``margin`` above the tallest obstacle whose footprint comes within
    ``corridor`` of the straight path, but never below ``min_alt`` above the
    higher endpoint. Within ``end_clear`` of either end only obstacles the
    path itself crosses (``path_width``) count: the ends are sidewalk spots
    right beside the delivery building, which the drone passes vertically,
    not over -- counting its facade made a 5 m hop climb over a 47 m tower. ``surface``, if given, is the highest ground/roof z
    measured along the path (hills are not in the obstacle list) and is
    cleared by ``margin`` too. Returns (z, tallest obstacle or None).

    a から b へ水平飛行する巡航高度。直線経路から ``corridor`` 以内にある
    最も高い障害物の ``margin`` 上。ただし高い方の端点から ``min_alt`` 未満には
    しない。(z, 最も高い障害物 または None) を返す。
    """
    floor = max(a[2], b[2]) + min_alt
    length = dist2d(a, b)
    if length > 2.0 * end_clear:
        u = end_clear / length
        mid = ((a[0] + u * (b[0] - a[0]), a[1] + u * (b[1] - a[1]), a[2]),
               (b[0] - u * (b[0] - a[0]), b[1] - u * (b[1] - a[1]), b[2]))
    else:
        mid = None                  # too short: only what the path crosses
    lo_x, hi_x = min(a[0], b[0]), max(a[0], b[0])
    lo_y, hi_y = min(a[1], b[1]), max(a[1], b[1])
    worst = None
    for o in obstacles:
        if worst is not None and o.top <= worst.top:
            continue
        r = math.hypot(o.ex, o.ey) + corridor          # cheap bounding-box reject
        if o.x + r < lo_x or o.x - r > hi_x or o.y + r < lo_y or o.y - r > hi_y:
            continue
        if (_seg_box_dist(a, b, o) <= path_width or
                (mid is not None and _seg_box_dist(mid[0], mid[1], o) <= corridor)):
            worst = o
    z = floor if worst is None else max(floor, worst.top + margin)
    if surface is not None:
        z = max(z, surface + margin)
    return z, worst


def sortie_altitudes(home, drops, obstacles, surface_fn=None, **kw):
    """Cruise z for each round trip home -> drop -> home (one per drop).

    The way out and the way back are the same straight line, so one height
    serves both. ``surface_fn(a, b)`` returns the highest surface z under
    that line (or None).
    各往復 home → drop → home の巡航高度（投下地点ごとに 1 つ）。往路と復路は
    同じ直線なので高さは共通。``surface_fn(a, b)`` はその下の最も高い面の z。
    """
    return [leg_altitude(home, d, obstacles,
                         surface=surface_fn(home, d) if surface_fn else None, **kw)[0]
            for d in drops]
