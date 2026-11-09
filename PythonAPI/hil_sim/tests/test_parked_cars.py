#!/usr/bin/env python3
"""parked_cars.py: blueprint pool, size fit, spot transform. No CARLA.

  ../../carla-venv/bin/python tests/test_parked_cars.py
"""
import os
import random
import sys
import types

HIL = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HIL)

import parked_cars as pc  # noqa: E402

FAILS = []


def check(name, cond, detail=""):
    print(("  PASS  " if cond else "  FAIL  ") + name + (" | " + str(detail) if detail != "" else ""))
    if not cond:
        FAILS.append(name)


print("=== pool ===")
ids = ["vehicle.toyota.prius", "vehicle.dodge.charger_police", "vehicle.delivery.pod", "vehicle.honda.nsx_r",
       "vehicle.redbull.rb16", "vehicle.kawasaki.ninja", "vehicle.carlamotors.firetruck", "vehicle.mini.cooper_s"]
base = {"vehicle.toyota.prius": "car", "vehicle.dodge.charger_police": "car", "vehicle.delivery.pod": "car",
        "vehicle.kawasaki.ninja": "motorcycle", "vehicle.carlamotors.firetruck": "truck", "vehicle.mini.cooper_s": "car"}
user = {"vehicle.honda.nsx_r", "vehicle.redbull.rb16"}
pool = pc.pool_ids(ids, base, user)
check("CARLA cars and user cars park", "vehicle.toyota.prius" in pool and "vehicle.honda.nsx_r" in pool, pool)
check("no police, pods, F1, bikes, trucks", not any(s in " ".join(pool) for s in ("police", "delivery", "rb16", "ninja", "firetruck")), pool)
check("user cars from cars.json read", len(pc.user_car_ids()) >= 1, sorted(pc.user_car_ids())[:3])
check("npc_only vehicles (the bus) never in the user pool", not (pc.user_car_ids() & pc.own_cars.npc_only_ids()))
check("npc ids never park", "vehicle.honda.nsx_r" not in pc.pool_ids(ids, base, user, npc={"vehicle.honda.nsx_r"}))

print("=== size fit ===")
spot = {"x": 0, "y": 0, "z": 0, "yaw": 0, "length": 4.0, "width": 2.0}
check("a car 0.6 m longer than its spot still fits", pc.fits(4.6, spot) and not pc.fits(4.7, spot))
check("unknown length: try it", pc.fits(None, spot))
rng = random.Random(1)
ext = {"vehicle.toyota.prius": 4.5, "vehicle.mini.cooper_s": 3.8, "vehicle.honda.nsx_r": 5.2}
picks = {pc.choose(spot, pool, ext, rng) for _ in range(200)}
check("never a car known to be too long", "vehicle.honda.nsx_r" not in picks, picks)
check("all too long -> None", pc.choose({"length": 2.0}, ["vehicle.honda.nsx_r"], ext, rng) is None)
check("avoid list honoured", pc.choose(spot, ["vehicle.mini.cooper_s"], ext, rng, avoid={"vehicle.mini.cooper_s"}) is None)

check("a car 0.5 m wider than its spot still fits", pc.fits(4.0, spot, 2.5) and not pc.fits(4.0, spot, 2.6))
ext2 = {"vehicle.toyota.prius": [4.5, 1.8], "vehicle.mini.cooper_s": [3.8, 2.7]}
picks2 = {pc.choose(spot, ["vehicle.toyota.prius", "vehicle.mini.cooper_s"], ext2, rng) for _ in range(100)}
check("never a car known to be too wide; [L, W] and L-only entries", picks2 == {"vehicle.toyota.prius"}
      and pc.size(4.2) == (4.2, None) and pc.size([4.2, 1.9]) == (4.2, 1.9), picks2)

print("=== ground check ===")
class _P(object):
    def __init__(self, z): self.location = types.SimpleNamespace(z=z)
class _W(object):
    def __init__(self, dz): self.dz = dz
    def ground_projection(self, loc, dist): return _P(loc.z - 2.0 + self.dz)
fakec = types.SimpleNamespace(Location=lambda x, y, z: types.SimpleNamespace(x=x, y=y, z=z))
sp = [{"x": i, "y": 0, "z": 2.2} for i in range(10)]
check("spots on this level's ground kept", len(pc.on_ground(_W(0.1), fakec, sp, log=lambda m: None)) == 10)
check("level 2.2 m lower (old installed map): no spots", pc.on_ground(_W(-2.2), fakec, sp, log=lambda m: None) == [])

print("=== transform ===")
class _L(object):
    def __init__(self, x, y, z): self.x, self.y, self.z = x, y, z
class _R(object):
    def __init__(self, pitch=0.0, yaw=0.0, roll=0.0): self.yaw = yaw
fake = types.SimpleNamespace(Location=_L, Rotation=_R, Transform=lambda l, r: (l, r))
l, r = pc.spot_transform({"x": 10.0, "y": 20.0, "z": 2.2, "yaw": 30.0, "length": 4, "width": 2}, False, fake)
check("map frame -> CARLA (y south, yaw negated), lifted", (l.x, l.y, round(l.z, 2), r.yaw) == (10.0, -20.0, 2.5, -30.0), (l.x, l.y, l.z, r.yaw))
l2, r2 = pc.spot_transform({"x": 0, "y": 0, "z": 0, "yaw": 30.0}, True, fake)
check("nose either way: flip adds 180", abs(r2.yaw - 150.0) < 1e-9, r2.yaw)
check("heading_ok both ways", pc.heading_ok(30.0, -30.0) and pc.heading_ok(30.0, 150.0) and not pc.heading_ok(30.0, 60.0))
check("data path per map", pc.data_path("/Game/Japan/Maps/OitaStation/OitaStation").endswith("maps/build/OitaStation/parked_cars.json"))

print("\n%d failure(s)" % len(FAILS))
sys.exit(1 if FAILS else 0)
