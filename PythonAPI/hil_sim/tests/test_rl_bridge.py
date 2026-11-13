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
    print("Testing DriveBridge...")
    bridge = DriveBridge(checkpoint_path="latest", map_name="OitaKosen")
    assert bridge.model is not None, "Drive model should be loaded"

    # Mock CARLA waypoints along a 100m straight line
    class MockTransform:
        def __init__(self, x, y, z, yaw):
            self.location = MockLoc(x, y, z)
            self.rotation = MockRot(yaw)

    class MockLoc:
        def __init__(self, x, y, z):
            self.x, self.y, self.z = x, y, z
        def distance(self, other):
            return math.sqrt((self.x - other.x)**2 + (self.y - other.y)**2 + (self.z - other.z)**2)

    class MockRot:
        def __init__(self, yaw):
            self.yaw = yaw

    class MockWP:
        def __init__(self, x, y, z, yaw=0.0):
            self.transform = MockTransform(x, y, z, yaw)
            self.lane_width = 3.5
            self.is_junction = False

    wps = [MockWP(float(i), 0.0, 10.0, yaw=0.0) for i in range(0, 100, 5)]
    bridge.build_from_waypoints(wps)
    assert bridge.P is not None and len(bridge.P) > 0

    # Mock car actor
    class MockCar:
        id = 999
        def get_transform(self):
            return MockTransform(10.0, 0.2, 10.0, yaw=1.0)
        def get_velocity(self):
            return MockLoc(5.0, 0.0, 0.0)

    car = MockCar()
    ctrl = bridge.run_step(car)
    print("  Control returned: steer=%.3f, throttle=%.3f, brake=%.3f" %
          (ctrl.steer, ctrl.throttle, ctrl.brake))
    assert -1.0 <= ctrl.steer <= 1.0
    assert 0.0 <= ctrl.throttle <= 1.0
    assert 0.0 <= ctrl.brake <= 1.0
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
