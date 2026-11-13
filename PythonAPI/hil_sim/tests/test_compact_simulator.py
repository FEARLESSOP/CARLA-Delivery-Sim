"""
Unit tests for Fast Compact Simulator (Paper 1 Method).
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import unittest
import time
from delivery.compact_simulator import (
    CompactFleetState,
    CompactSimulator,
    SimDrone,
    SimHub,
    SimStation,
    SimOrder,
    DroneState,
    EventIntervalMetrics,
)
from delivery.feasibility_engine import RecoveryTarget


class TestCompactSimulator(unittest.TestCase):
    def setUp(self):
        self.state = CompactFleetState(current_time_s=100.0)
        self.sim = CompactSimulator(horizon_events=3, num_scenarios=4)

    def test_state_cloning_speed_and_independence(self):
        t0 = time.perf_counter()
        cloned = self.state.clone()
        elapsed_us = (time.perf_counter() - t0) * 1e6

        # Fast cloning requirement (< 500 us)
        self.assertLess(elapsed_us, 500.0)

        # Deep copy independence check
        cloned.hub.position = (999.0, 999.0, 999.0)
        self.assertNotEqual(self.state.hub.position, cloned.hub.position)

    def test_event_reward_computation(self):
        # Case 1: 1 on-time delivery, 0 misses, 0.5 unavailable min, 10 Wh energy
        # r = 2(1) + 2(1) - 5(0) - (0.5/10) - (10/100) = 4 - 0.05 - 0.10 = 3.85
        m = EventIntervalMetrics(
            first_deliveries=1,
            on_time_deliveries=1,
            missed_deadlines=0,
            unavailable_drone_minutes=0.5,
            total_energy_wh=10.0,
        )
        self.assertAlmostEqual(m.compute_reward(), 3.85, places=3)

        # Case 2: 1 missed delivery, 10 unavailable min
        # r = 2(1) + 2(0) - 5(1) - 1.0 - 0 = -4.0
        m_late = EventIntervalMetrics(
            first_deliveries=1,
            on_time_deliveries=0,
            missed_deadlines=1,
            unavailable_drone_minutes=10.0,
            total_energy_wh=0.0,
        )
        self.assertAlmostEqual(m_late.compute_reward(), -4.0, places=3)

    def test_rollout_candidate_evaluation(self):
        target_hub = RecoveryTarget(
            target_id="hub",
            target_type="hub",
            position=(0.0, 0.0, 2.0),
            resource_id="hub_roof_pad",
            is_stopped_at_bay=True,
        )
        target_station = RecoveryTarget(
            target_id="station_1",
            target_type="station",
            position=(80.0, 50.0, 0.0),
            resource_id="station_1",
        )

        q_hub = self.sim.evaluate_candidate_action(
            initial_state=self.state,
            drone_id="drone_1",
            target=target_hub,
            flight_time_s=15.0,
            base_seed=42,
        )

        q_station = self.sim.evaluate_candidate_action(
            initial_state=self.state,
            drone_id="drone_1",
            target=target_station,
            flight_time_s=25.0,
            base_seed=42,
        )

        # Both should return finite float expected values
        self.assertTrue(isinstance(q_hub, float))
        self.assertTrue(isinstance(q_station, float))
        # Re-running with same seed should yield identical result (CRN)
        q_hub_repeat = self.sim.evaluate_candidate_action(
            initial_state=self.state,
            drone_id="drone_1",
            target=target_hub,
            flight_time_s=15.0,
            base_seed=42,
        )
        self.assertEqual(q_hub, q_hub_repeat)


if __name__ == "__main__":
    unittest.main()
