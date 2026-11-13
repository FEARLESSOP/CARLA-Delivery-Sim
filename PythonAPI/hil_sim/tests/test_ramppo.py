"""
Unit tests for FA-RAMPPO Planner and Paper 1 Baselines (B0 to B5).
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import unittest
from delivery.compact_simulator import CompactFleetState
from delivery.feasibility_engine import DroneRecoveryQuery, RecoveryTarget
from rl_delivery.ramppo import (
    FARAMPPOPlanner,
    BaselineType,
    evaluate_scenario_benchmark,
)


class TestFARAMPPO(unittest.TestCase):
    def setUp(self):
        self.planner = FARAMPPOPlanner(
            beta_policy_weight=0.5,
            horizon_events=3,
            num_scenarios=4,
        )
        self.state = CompactFleetState(current_time_s=200.0)
        self.query = DroneRecoveryQuery(
            drone_id="drone_1",
            position=(50.0, 50.0, 30.0),
            battery_wh=120.0,
            current_time_s=200.0,
        )
        self.targets = [
            RecoveryTarget("hub", "hub", (0.0, 0.0, 2.0), "hub_roof_pad", is_stopped_at_bay=True),
            RecoveryTarget("station_1", "station", (80.0, 50.0, 0.0), "station_1"),
            RecoveryTarget("station_2", "station", (150.0, -50.0, 0.0), "station_2"),
        ]

    def test_all_baselines_execute_successfully(self):
        baselines = [
            BaselineType.B0_HEURISTIC,
            BaselineType.B1_STANDARD_PPO,
            BaselineType.B2_MASKABLE_PPO,
            BaselineType.B3_FORECAST_PPO,
            BaselineType.B4_FA_RAMPPO,
            BaselineType.B5_ROLLOUT_HEURISTIC,
        ]

        for b in baselines:
            res = self.planner.select_recovery_action(
                query=self.query,
                targets=self.targets,
                fleet_state=self.state,
                baseline=b,
                commit_reservation=False,
            )
            self.assertIsNotNone(res.selected_target_id)
            self.assertIn(res.selected_action_idx, [0, 1, 2])
            # Sub-1-second budget check (< 1000 ms)
            self.assertLess(res.decision_latency_ms, 1000.0)

    def test_fa_ramppo_decision_rule_and_reservation(self):
        # B4: FA-RAMPPO with reservation commit
        res = self.planner.select_recovery_action(
            query=self.query,
            targets=self.targets,
            fleet_state=self.state,
            baseline=BaselineType.B4_FA_RAMPPO,
            commit_reservation=True,
        )
        self.assertTrue(res.feasible)
        self.assertEqual(res.baseline_used, BaselineType.B4_FA_RAMPPO)
        self.assertIsNotNone(res.booking_id)

        # Verify pad reservation manager actually contains the booking
        chosen_target = self.targets[res.selected_action_idx]
        active = self.planner.feasibility.pad_manager.get_active_booking(
            chosen_target.resource_id,
            res.diagnostics["arrival_time_s"] + 5.0,
        )
        self.assertEqual(len(active), 1)
        self.assertEqual(active[0].booking_id, res.booking_id)

    def test_benchmark_runner_s1(self):
        # Test matched benchmark evaluation across 2 episodes
        res = evaluate_scenario_benchmark(scenario_name="S1_Normal", num_episodes=2, seed=123)
        self.assertEqual(len(res), 6)
        for b_key, metrics in res.items():
            self.assertIn("mean_reward", metrics)
            self.assertIn("feasibility_rate", metrics)
            self.assertIn("mean_latency_ms", metrics)
            self.assertGreaterEqual(metrics["feasibility_rate"], 0.0)
            self.assertLessEqual(metrics["feasibility_rate"], 1.0)


if __name__ == "__main__":
    unittest.main()
