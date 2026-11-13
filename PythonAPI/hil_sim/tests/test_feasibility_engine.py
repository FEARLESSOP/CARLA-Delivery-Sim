"""
Unit tests for FeasibilityEngine (Paper 1 Method).
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import unittest
import numpy as np

from delivery.feasibility_engine import (
    FeasibilityEngine,
    DroneRecoveryQuery,
    RecoveryTarget,
)
from delivery.reservations import PadReservationManager
from delivery.zones import Airspace


class TestFeasibilityEngine(unittest.TestCase):
    def setUp(self):
        self.pad_mgr = PadReservationManager()
        self.pad_mgr.register_resource("hub_pad_01", capacity=1)
        self.pad_mgr.register_resource("station_pad_01", capacity=2)

        # Single no-fly zone around (200, 200)
        nofly = [{"id": "nf1", "x0": 180.0, "y0": 180.0, "x1": 220.0, "y1": 220.0}]
        self.airspace = Airspace(nofly=nofly, margin=5.0)

        self.engine = FeasibilityEngine(
            pad_manager=self.pad_mgr,
            airspace=self.airspace,
            reserve_wh=15.0,
            recovery_mode="stopped_bay",
            docking_duration_s=25.0,
        )

    def test_battery_reserve_constraint(self):
        # Drone with very low battery (5 Wh) cannot reach target 500m away
        query = DroneRecoveryQuery(
            drone_id="drone_1",
            position=(0.0, 0.0, 30.0),
            battery_wh=5.0,  # Below reserve + flight energy
            current_time_s=100.0,
        )
        target = RecoveryTarget(
            target_id="hub_01",
            target_type="hub",
            position=(500.0, 500.0, 2.0),
            resource_id="hub_pad_01",
            is_stopped_at_bay=True,
        )
        res = self.engine.evaluate_target(query, target)
        self.assertFalse(res.feasible)
        self.assertIn("insufficient_battery_reserve", res.reason)

    def test_airspace_nofly_constraint(self):
        query = DroneRecoveryQuery(
            drone_id="drone_1",
            position=(0.0, 0.0, 30.0),
            battery_wh=300.0,
            current_time_s=100.0,
        )
        # Target located inside the no-fly zone (200, 200)
        target = RecoveryTarget(
            target_id="station_nf",
            target_type="station",
            position=(200.0, 200.0, 0.0),
            resource_id="station_pad_01",
        )
        res = self.engine.evaluate_target(query, target)
        self.assertFalse(res.feasible)
        self.assertEqual(res.reason, "target_inside_nofly_zone")

    def test_stopped_bay_recovery_mode(self):
        query = DroneRecoveryQuery(
            drone_id="drone_1",
            position=(100.0, 100.0, 30.0),
            battery_wh=200.0,
            current_time_s=100.0,
        )
        # Target hub is moving and has no future stop planned
        target_moving = RecoveryTarget(
            target_id="hub_moving",
            target_type="hub",
            position=(120.0, 100.0, 2.0),
            resource_id="hub_pad_01",
            is_stopped_at_bay=False,
            future_stop_time_s=None,
        )
        res = self.engine.evaluate_target(query, target_moving)
        self.assertFalse(res.feasible)
        self.assertIn("hub_not_stopped_at_bay", res.reason)

        # But fixed station does not have moving bay restriction
        target_station = RecoveryTarget(
            target_id="station_01",
            target_type="station",
            position=(120.0, 100.0, 0.0),
            resource_id="station_pad_01",
            is_stopped_at_bay=False,
        )
        res_st = self.engine.evaluate_target(query, target_station)
        self.assertTrue(res_st.feasible)

    def test_pad_reservation_conflict(self):
        query = DroneRecoveryQuery(
            drone_id="drone_1",
            position=(50.0, 50.0, 30.0),
            battery_wh=200.0,
            current_time_s=100.0,
        )
        target = RecoveryTarget(
            target_id="hub_01",
            target_type="hub",
            position=(60.0, 50.0, 2.0),
            resource_id="hub_pad_01",
            is_stopped_at_bay=True,
        )

        # Pre-book the pad for another drone at the estimated arrival time (~105s to 130s)
        self.pad_mgr.book_slot("hub_pad_01", "drone_2", start_s=100.0, duration_s=30.0)

        res = self.engine.evaluate_target(query, target)
        self.assertFalse(res.feasible)
        self.assertIn("pad_slot_conflict", res.reason)

    def test_action_mask_generation(self):
        query = DroneRecoveryQuery(
            drone_id="drone_1",
            position=(0.0, 0.0, 30.0),
            battery_wh=100.0,
            current_time_s=100.0,
        )
        # Target 0: Feasible hub
        t0 = RecoveryTarget(
            target_id="hub_ok",
            target_type="hub",
            position=(50.0, 0.0, 2.0),
            resource_id="hub_pad_01",
            is_stopped_at_bay=True,
        )
        # Target 1: Infeasible station (inside no-fly)
        t1 = RecoveryTarget(
            target_id="station_nofly",
            target_type="station",
            position=(200.0, 200.0, 0.0),
            resource_id="station_pad_01",
        )
        mask, evals = self.engine.compute_action_mask(query, [t0, t1])
        self.assertEqual(len(mask), 2)
        self.assertTrue(mask[0])
        self.assertFalse(mask[1])


if __name__ == "__main__":
    unittest.main()
