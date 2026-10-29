"""Background traffic for the delivery mission: autopilot cars and walkers.

Spawned from inside delivery_sim.py rather than generate_traffic.py, because
the mission owns the synchronous tick and the Traffic Manager has to follow
that same tick.

配送ミッションの背景交通（自動運転車と歩行者）。ミッションが同期 tick を
持ち、Traffic Manager もその tick に従う必要があるため、generate_traffic.py
ではなく delivery_sim.py 内から生成する。
"""

import random

import carla

SpawnActor = carla.command.SpawnActor
SetAutopilot = carla.command.SetAutopilot
FutureActor = carla.command.FutureActor
DestroyActor = carla.command.DestroyActor


class Traffic(object):
    def __init__(self, client, world, tm_port=8000, seed=None, sync=True):
        self.client, self.world = client, world
        self.sync = sync
        self.rng = random.Random(seed)
        self.tm = client.get_trafficmanager(tm_port)
        self.tm_port = self.tm.get_port()
        self.tm.set_synchronous_mode(sync)
        self.tm.set_global_distance_to_leading_vehicle(2.5)
        if seed is not None:
            self.tm.set_random_device_seed(seed)
            world.set_pedestrians_seed(seed)
        self.vehicles, self.walkers, self.controllers = [], [], []

    def _tick(self):
        self.world.tick() if self.sync else self.world.wait_for_tick()

    def spawn_vehicles(self, n, avoid=()):
        """Autopilot cars at free spawn points, at least 10 m from ``avoid``.

        ``avoid`` の 10 m 以内を避けた空きスポーン地点に自動運転車を出す。
        """
        lib = self.world.get_blueprint_library()
        # four-wheel cars only: bikes are hard for BasicAgent to see
        # 4 輪車のみ（二輪車は BasicAgent が検知しにくい）
        bps = [b for b in lib.filter("vehicle.*")
               if b.get_attribute("number_of_wheels").as_int() == 4
               and (not b.has_attribute("base_type")
                    or b.get_attribute("base_type").as_str() in ("car", "van", "truck", ""))]
        points = [sp for sp in self.world.get_map().get_spawn_points()
                  if all(sp.location.distance(a) > 10.0 for a in avoid)]
        self.rng.shuffle(points)
        batch = []
        for sp in points[:n]:
            bp = self.rng.choice(bps)
            if bp.has_attribute("color"):
                bp.set_attribute("color", self.rng.choice(
                    bp.get_attribute("color").recommended_values))
            bp.set_attribute("role_name", "autopilot")
            batch.append(SpawnActor(bp, sp).then(SetAutopilot(FutureActor, True, self.tm_port)))
        for r in self.client.apply_batch_sync(batch, self.sync):
            if not r.error:
                self.vehicles.append(r.actor_id)
        return len(self.vehicles)

    def spawn_walkers(self, n, cross=0.0, run_share=0.1):
        """Pedestrians walking between random navigation points.

        ``cross`` is the share that cross roads (0 keeps them on sidewalks;
        BasicAgent does not brake for pedestrians).
        ``cross`` は道路を横断する歩行者の割合（0 なら歩道のみ。BasicAgent は
        歩行者に対してブレーキをかけない）。
        """
        lib = self.world.get_blueprint_library()
        bps = list(lib.filter("walker.pedestrian.*"))
        batch, speeds = [], []
        for _ in range(n):
            loc = self.world.get_random_location_from_navigation()
            if loc is None:
                continue
            bp = self.rng.choice(bps)
            if bp.has_attribute("is_invincible"):
                bp.set_attribute("is_invincible", "false")
            speed = bp.get_attribute("speed").recommended_values if bp.has_attribute(
                "speed") else ["0", "1.4", "2.5"]
            speeds.append(float(speed[2] if self.rng.random() < run_share else speed[1]))
            batch.append(SpawnActor(bp, carla.Transform(loc)))
        ok_speeds = []
        for r, sp in zip(self.client.apply_batch_sync(batch, self.sync), speeds):
            if not r.error:
                self.walkers.append(r.actor_id)
                ok_speeds.append(sp)
        ctrl_bp = lib.find("controller.ai.walker")
        batch = [SpawnActor(ctrl_bp, carla.Transform(), w) for w in self.walkers]
        for r in self.client.apply_batch_sync(batch, self.sync):
            if not r.error:
                self.controllers.append(r.actor_id)
        # controllers need one tick before they can be started
        # コントローラは開始前に 1 tick 必要
        self._tick()
        self.world.set_pedestrians_cross_factor(cross)
        for ctrl, sp in zip(self.world.get_actors(self.controllers), ok_speeds):
            ctrl.start()
            ctrl.go_to_location(self.world.get_random_location_from_navigation())
            ctrl.set_max_speed(sp)
        return len(self.walkers)

    def destroy(self):
        for ctrl in self.world.get_actors(self.controllers):
            ctrl.stop()
        ids = self.vehicles + self.controllers + self.walkers
        self.client.apply_batch([DestroyActor(i) for i in ids])
        self.vehicles, self.walkers, self.controllers = [], [], []
        if self.sync:
            self.tm.set_synchronous_mode(False)
