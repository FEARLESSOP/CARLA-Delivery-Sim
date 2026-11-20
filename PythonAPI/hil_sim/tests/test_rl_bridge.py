"""Unit tests for delivery/rl_bridge.py.

Tests:
1. Checkpoint resolution for nav, drive, fleet_pod, fleet_drone.
2. NavBridge: loads checkpoint, builds observation (54 floats), steps RecurrentPPO, updates physics.
3. DriveBridge: loads checkpoint, builds observation (30 floats), steps SAC, generates vehicle control.
"""
import math
import os
import sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
HIL_SIM = os.path.dirname(HERE)
sys.path.insert(0, HIL_SIM)

from delivery.rl_bridge import resolve_checkpoint, NavBridge, DriveBridge, RLNavFlight, FleetRecoveryBridge


def test_resolve_checkpoints():
    print("Testing resolve_checkpoint...")
    p_nav = resolve_checkpoint("nav", "latest")
    assert os.path.exists(p_nav), "Nav checkpoint does not exist: %s" % p_nav
    print("  Nav latest:", p_nav)

    p_drive = resolve_checkpoint("drive", "latest")
    assert os.path.exists(p_drive), "Drive checkpoint does not exist: %s" % p_drive
    print("  Drive latest:", p_drive)

    p_pod = resolve_checkpoint("fleet_pod", "latest")
    assert os.path.exists(p_pod), "Fleet pod checkpoint does not exist: %s" % p_pod
    print("  Fleet pod latest:", p_pod)

    p_drone = resolve_checkpoint("fleet_drone", "latest")
    assert os.path.exists(p_drone), "Fleet drone checkpoint does not exist: %s" % p_drone
    print("  Fleet drone latest:", p_drone)
    print("PASS: resolve_checkpoint\n")


def test_nav_bridge():
    print("Testing NavBridge & RLNavFlight...")
    bridge = NavBridge(checkpoint_path="latest", map_name="OitaKosen", wind_level=3.0)
    assert bridge.model is not None, "Nav model should be loaded"
    assert bridge.heights is not None, "Heights should be loaded"

    # Mock van with dock_pos
    class MockVan:
        def dock_pos(self):
            return (150.0, 250.0, 30.0)

    van = MockVan()
    hover_point = (180.0, 270.0, 25.0)
    flight = bridge.create_flight(van, hover_point=hover_point)

    assert isinstance(flight, RLNavFlight)
    assert flight.loaded is True
    assert flight.done is False
    assert len(flight.drops) == 1

    # Check observation shape
    obs = flight._obs()
    assert obs.shape == (54,), "Observation shape should be (54,), got %s" % str(obs.shape)
    assert not np.isnan(obs).any(), "Observation contains NaN"

    # Step flight for 20 ticks (1 second)
    dt = 0.05
    for tick in range(20):
        evs = flight.update(dt)
        assert len(flight.pos) == 3
        assert isinstance(flight.roll_deg, float)
        assert isinstance(flight.pitch_deg, float)
        assert isinstance(flight.yaw_deg, float)

    print("  Flight pos after 1s:", flight.pos)
    print("  Attitude: roll=%.1f deg, pitch=%.1f deg, yaw=%.1f deg" %
          (flight.roll_deg, flight.pitch_deg, flight.yaw_deg))
    print("PASS: NavBridge & RLNavFlight\n")


def test_drive_bridge():
    """DriveBridge builds the same route and the same 30-number observation as
    rl_delivery/env_drive.py for the same pod state (parity), from lane pieces
    and from CARLA-like waypoints; one bridge per pod; controls in range.
    DriveBridge が env_drive と同じ経路・同じ観測を作ること（観測の一致）を確認。"""
    import numpy as np
    from rl_delivery.env_drive import DriveEnv
    from delivery.rl_bridge import DrivePolicy
    print("Testing DriveBridge (parity with env_drive)...")
    policy = DrivePolicy("latest", "OitaKosen")
    bridge = DriveBridge(policy=policy, map_name="OitaKosen")
    other = DriveBridge(policy=policy, map_name="OitaKosen")
    assert bridge.model is policy.model and other.model is policy.model, "brain loaded once"

    class Loc:
        def __init__(self, x, y, z=0.0):
            self.x, self.y, self.z = x, y, z

    class Rot:
        def __init__(self, yaw):
            self.yaw = yaw

    class Tf:
        def __init__(self, x, y, yaw_deg):
            self.location, self.rotation = Loc(x, y), Rot(yaw_deg)

    class WP:
        def __init__(self, x, y, road, lane):
            self.transform = Tf(x, y, 0.0)
            self.road_id, self.lane_id = road, lane

    class Car:
        id = 999

        def __init__(self, x, y, yaw, v):
            self.tf, self.v = Tf(x, y, math.degrees(yaw)), Loc(v * math.cos(yaw), v * math.sin(yaw))

        def get_transform(self):
            return self.tf

        def get_velocity(self):
            return self.v

    env = DriveEnv("OitaKosen", seed=8080)
    env.route_len, env.traffic = 250.0, 0
    checked = 0
    for seed in range(8080, 8086):
        obs, _ = env.reset(seed=seed)
        edges = env._route.__self__ and None
        # rebuild the env route from its lane pieces: the edges are not stored, so
        # take the route env just built and recover its edges from the points
        # 環境の経路を車線片から再構成
        env2 = DriveEnv("OitaKosen", seed=seed)
        env2.route_len, env2.traffic = 250.0, 0
        env2.rng = np.random.default_rng(seed)
        edges = env2._route()
        env2._build(edges)
        bridge.build_from_edges(edges)
        assert np.allclose(bridge.P, env2.P) and np.allclose(bridge.S, env2.S), "route points differ"
        assert [round(m[0], 6) for m in bridge.marks] == [round(m[0], 6) for m in env2.marks], "stop lines differ"
        # the same route from CARLA-like waypoints (road / lane ids) / 経路点からも同じ
        wps = []
        for ei in edges:
            L = policy.lanes[ei]
            wps += [WP(q[0], q[1], L["road"], L["lane"]) for q in L["pts"][::3] + L["pts"][-1:]]
        other.build_from_waypoints(wps)
        # same geometry (duplicate lane pieces in the data are interchangeable) / 同じ形状
        n = min(len(other.P), len(env2.P))
        assert abs(len(other.P) - len(env2.P)) <= 1 and np.allclose(other.P[:n, :2], env2.P[:n, :2], atol=0.05), \
            "route from waypoints differs: %s vs %s" % (other.edges[:8], edges[:8])
        # drive env2 with the brain and compare observations along the way / 走行中の観測を比較
        env2.reset(seed=seed)
        env2._build(edges)
        q = env2.P[0]
        env2.x, env2.y, env2.yaw, env2.v, env2.steer, env2.k, env2.s = q[0], q[1], q[2], 0.0, 0.0, 0, 0.0
        bridge.build_from_edges(edges)
        bridge.k, bridge.s = 0, 0.0
        o_env = env2._obs()
        for step in range(120):
            a, _ = policy.model.predict(o_env, deterministic=True)
            o_env, _r, te, tr, _i = env2.step(a)
            if te or tr:
                break
            # bridge state = env state before this observation / ブリッジの状態を合わせる
            bridge.steer, bridge.last_act = env2.steer, np.asarray(env2.last, np.float32)
            for mb, me in zip(bridge.marks, env2.marks):
                mb[3], mb[4] = me[3], me[4]
            o_br, _v = bridge.observe(Car(env2.x, env2.y, env2.yaw, env2.v))
            mask = np.ones(30, bool)
            mask[20:23] = False                           # light colour: CARLA's lights, not env's timer / 信号色は除外
            assert np.allclose(o_br[mask], o_env[mask], atol=1e-4), (
                "observation differs at step %d: %s" % (step, np.nonzero(~np.isclose(o_br, o_env, atol=1e-4))[0]))
            checked += 1
    print("  %d observations identical to env_drive (6 routes)" % checked)
    assert checked > 100
    ctrl = bridge.run_step(Car(env2.x, env2.y, env2.yaw, env2.v))
    print("  Control returned: steer=%.3f, throttle=%.3f, brake=%.3f" % (ctrl.steer, ctrl.throttle, ctrl.brake))
    assert -1.0 <= ctrl.steer <= 1.0 and 0.0 <= ctrl.throttle <= 1.0 and 0.0 <= ctrl.brake <= 1.0
    print("PASS: DriveBridge\n")

def test_fleet_recovery_bridge():
    print("Testing FleetRecoveryBridge...")
    bridge = FleetRecoveryBridge(checkpoint_path="latest", baseline="b4_fa_ramppo")
    assert bridge.planner is not None
    assert bridge.feasibility is not None

    class MockBattery:
        level = 85.0

    class MockDrone:
        id = 1
        pos = (150.0, 200.0, 25.0)
        battery = MockBattery()

    class MockVan:
        vid = 1
        status = "spot"
        meets = {}
        def roof(self):
            return (155.0, 210.0, 2.5)

    drone = MockDrone()
    van = MockVan()
    station_pads = [
        (100.0, 100.0, 0.0),
        (300.0, 300.0, 0.0),
    ]

    # Test stopped pod recovery
    chosen_target, decision = bridge.evaluate_recovery(
        drone_unit=drone,
        van=van,
        station_pads=station_pads,
        current_wind=(2.0, 0.0),
        current_t=100.0,
    )
    print("  Case 1 (Stopped Pod): Chosen:", chosen_target.target_id, "Type:", chosen_target.target_type,
          "Feasible:", decision.feasible, "Latency (ms): %.2f" % decision.decision_latency_ms)
    assert chosen_target.target_type == "hub"
    assert decision.feasible is True

    # Test moving pod recovery -> diversion to station
    van.status = "driving"
    chosen_target2, decision2 = bridge.evaluate_recovery(
        drone_unit=drone,
        van=van,
        station_pads=station_pads,
        current_wind=(2.0, 0.0),
        current_t=110.0,
    )
    print("  Case 2 (Moving Pod -> Divert): Chosen:", chosen_target2.target_id, "Type:", chosen_target2.target_type,
          "Feasible:", decision2.feasible, "Latency (ms): %.2f" % decision2.decision_latency_ms)
    assert chosen_target2.target_type == "station"
    assert decision2.feasible is True
    print("PASS: FleetRecoveryBridge\n")


if __name__ == "__main__":
    test_resolve_checkpoints()
    test_nav_bridge()
    test_drive_bridge()
    test_fleet_recovery_bridge()
    print("ALL BRIDGE UNIT TESTS PASSED!")
