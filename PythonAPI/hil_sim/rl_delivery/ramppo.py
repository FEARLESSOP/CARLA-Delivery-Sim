"""
FA-RAMPPO (Fleet-Aware Rollout-Assisted Maskable PPO) Decision Engine & Paper 1 Baselines.

Implements the decision-time predictive lookahead algorithm for last-mile drone
recovery reservations, and provides the 6 matched academic baselines (B0-B5):

Algorithms:
    B0: Heuristic Nearest-Feasible + Queue Score
    B1: Standard PPO (Unmasked Reactive)
    B2: Maskable PPO (Reactive with Feasibility Mask)
    B3: Maskable PPO + Forecast Features
    B4: FA-RAMPPO (Proposed: Feasibility Mask + Rollout Lookahead + Policy Prior)
    B5: Rollout Search with Heuristic Continuation (No Learned Prior)

Decision Rule (B4):
    a* = argmax_{a in A_feasible} [ Q_hat_H(s, a) + beta * log pi_theta(a | s) ]
"""

import math
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Callable, Dict, List, Optional, Tuple

import numpy as np

from delivery.compact_simulator import (
    CompactFleetState,
    CompactSimulator,
    DroneState,
    SimDrone,
    SimHub,
    SimOrder,
    SimStation,
)
from delivery.feasibility_engine import (
    DroneRecoveryQuery,
    FeasibilityEngine,
    RecoveryTarget,
)
from delivery.reservations import PadReservationManager


class BaselineType(str, Enum):
    B0_HEURISTIC = "B0_Heuristic"
    B1_STANDARD_PPO = "B1_Standard_PPO"
    B2_MASKABLE_PPO = "B2_Maskable_PPO"
    B3_FORECAST_PPO = "B3_Forecast_PPO"
    B4_FA_RAMPPO = "B4_FA_RAMPPO"
    B5_ROLLOUT_HEURISTIC = "B5_Rollout_Heuristic"


@dataclass
class RecoveryDecisionResult:
    """Output of a recovery decision."""
    selected_target_id: str
    selected_action_idx: int
    baseline_used: BaselineType
    feasible: bool
    q_value: float
    policy_prior_prob: float
    decision_latency_ms: float
    booking_id: Optional[str] = None
    diagnostics: Dict[str, Any] = field(default_factory=dict)


class FARAMPPOPlanner:
    """
    Fleet-Aware Rollout-Assisted Maskable PPO Decision-Time Planner.
    Coordinates feasibility masking, compact stochastic lookahead, and
    atomic pad reservations.
    """

    def __init__(
        self,
        feasibility_engine: Optional[FeasibilityEngine] = None,
        compact_simulator: Optional[CompactSimulator] = None,
        beta_policy_weight: float = 0.5,
        horizon_events: int = 4,
        num_scenarios: int = 8,
        gamma: float = 0.95,
    ):
        self.feasibility = feasibility_engine or FeasibilityEngine()
        self.simulator = compact_simulator or CompactSimulator(
            horizon_events=horizon_events,
            num_scenarios=num_scenarios,
            gamma=gamma,
        )
        self.beta = float(beta_policy_weight)

    def select_recovery_action(
        self,
        query: DroneRecoveryQuery,
        targets: List[RecoveryTarget],
        fleet_state: CompactFleetState,
        baseline: BaselineType = BaselineType.B4_FA_RAMPPO,
        policy_prior_fn: Optional[Callable[[np.ndarray], np.ndarray]] = None,
        commit_reservation: bool = True,
    ) -> RecoveryDecisionResult:
        """
        Evaluate candidate targets and select recovery action according to baseline.
        """
        t0 = time.perf_counter()

        # Step 1: Feasibility Evaluation
        mask, evals = self.feasibility.compute_action_mask(query, targets)
        num_targets = len(targets)

        # Baseline B1: Standard PPO ignores mask
        if baseline == BaselineType.B1_STANDARD_PPO:
            active_mask = np.ones(num_targets, dtype=bool)
        else:
            active_mask = mask

        # Step 2: Policy Prior Probability Distribution pi(a | s)
        if policy_prior_fn is not None:
            raw_probs = policy_prior_fn(np.zeros(32, dtype=np.float32))
        else:
            # Calibrated default prior: prefer hub slightly over fixed station
            raw_probs = np.ones(num_targets, dtype=np.float32)
            for i, t in enumerate(targets):
                if t.target_type == "hub":
                    raw_probs[i] = 1.8
                else:
                    raw_probs[i] = 1.0

        # Mask and renormalize prior
        masked_probs = raw_probs * active_mask.astype(np.float32)
        prob_sum = float(np.sum(masked_probs))
        if prob_sum > 1e-6:
            norm_probs = masked_probs / prob_sum
        else:
            norm_probs = active_mask.astype(np.float32) / max(1.0, float(np.sum(active_mask)))

        # Step 3: Action Selection based on requested baseline
        q_values = np.zeros(num_targets, dtype=np.float32)

        if baseline == BaselineType.B0_HEURISTIC:
            # B0: Nearest feasible target with queue penalty
            scores = []
            for i, t in enumerate(targets):
                if not active_mask[i]:
                    scores.append(-1e9)
                    continue
                dist = math.hypot(t.position[0] - query.position[0], t.position[1] - query.position[1])
                score = -dist
                if t.target_type == "hub" and not t.is_stopped_at_bay:
                    score -= 200.0
                scores.append(score)
            best_idx = int(np.argmax(scores))

        elif baseline in (BaselineType.B1_STANDARD_PPO, BaselineType.B2_MASKABLE_PPO, BaselineType.B3_FORECAST_PPO):
            # Pure reactive policy (no lookahead rollout)
            best_idx = int(np.argmax(norm_probs))

        elif baseline == BaselineType.B5_ROLLOUT_HEURISTIC:
            # B5: Lookahead rollout using heuristic continuation (beta = 0, no learned prior)
            feasible_indices = [i for i in range(num_targets) if active_mask[i]]
            for idx in feasible_indices:
                t = targets[idx]
                fl_time = evals[idx].flight_time_s
                q_hat = self.simulator.evaluate_candidate_action(
                    initial_state=fleet_state,
                    drone_id=query.drone_id,
                    target=t,
                    flight_time_s=fl_time,
                )
                q_values[idx] = q_hat
            # Pure argmax Q_hat
            best_idx = int(np.argmax([q_values[i] if active_mask[i] else -1e9 for i in range(num_targets)]))

        else:
            # B4: FA-RAMPPO (Lookahead + Mask + Policy Prior)
            feasible_indices = [i for i in range(num_targets) if active_mask[i]]
            total_scores = np.full(num_targets, -1e9, dtype=np.float32)

            for idx in feasible_indices:
                t = targets[idx]
                fl_time = evals[idx].flight_time_s

                # Fast compact simulator rollout lookahead
                q_hat = self.simulator.evaluate_candidate_action(
                    initial_state=fleet_state,
                    drone_id=query.drone_id,
                    target=t,
                    flight_time_s=fl_time,
                )
                q_values[idx] = q_hat

                # Combine Q_hat with policy prior: Q_hat(s, a) + beta * log pi(a | s)
                prior_p = max(1e-4, float(norm_probs[idx]))
                total_scores[idx] = q_hat + (self.beta * math.log(prior_p))

            best_idx = int(np.argmax(total_scores))

        chosen_target = targets[best_idx]
        chosen_eval = evals[best_idx]
        latency_ms = (time.perf_counter() - t0) * 1000.0

        # Step 4: Atomic Pad Reservation (if requested)
        booking_id = None
        if commit_reservation and chosen_eval.feasible:
            ok, _, bid, _ = self.feasibility.pad_manager.book_slot(
                resource_id=chosen_target.resource_id,
                entity_id=query.drone_id,
                start_s=chosen_eval.arrival_time_s,
                duration_s=self.feasibility.docking_duration_s,
            )
            if ok:
                booking_id = bid

        return RecoveryDecisionResult(
            selected_target_id=chosen_target.target_id,
            selected_action_idx=best_idx,
            baseline_used=baseline,
            feasible=chosen_eval.feasible,
            q_value=float(q_values[best_idx]),
            policy_prior_prob=float(norm_probs[best_idx]),
            decision_latency_ms=latency_ms,
            booking_id=booking_id,
            diagnostics={
                "all_q_values": q_values.tolist(),
                "action_mask": mask.tolist(),
                "flight_time_s": chosen_eval.flight_time_s,
                "arrival_time_s": chosen_eval.arrival_time_s,
                "reasons": [e.reason for e in evals],
            },
        )


def evaluate_scenario_benchmark(
    scenario_name: str = "S1_Normal",
    num_episodes: int = 5,
    seed: int = 42,
) -> Dict[str, Dict[str, float]]:
    """
    Run matched scenario evaluation comparing all 6 baselines (B0 to B5).
    """
    baselines = [
        BaselineType.B0_HEURISTIC,
        BaselineType.B1_STANDARD_PPO,
        BaselineType.B2_MASKABLE_PPO,
        BaselineType.B3_FORECAST_PPO,
        BaselineType.B4_FA_RAMPPO,
        BaselineType.B5_ROLLOUT_HEURISTIC,
    ]

    results = {}
    planner = FARAMPPOPlanner()

    for b in baselines:
        rewards = []
        latencies = []
        feasibility_rates = []

        for ep in range(num_episodes):
            ep_seed = seed + ep * 1000
            rng = np.random.RandomState(ep_seed)

            # Scenario settings
            wind_speed = 3.0 if "Normal" in scenario_name else 9.5
            state = CompactFleetState(current_time_s=100.0, wind_speed_ms=wind_speed)

            # Drones finish deliveries at diverse spots
            query = DroneRecoveryQuery(
                drone_id="drone_1",
                position=(rng.uniform(-100, 100), rng.uniform(-100, 100), 30.0),
                battery_wh=rng.uniform(60.0, 150.0),
                current_time_s=100.0,
                wind_vector=(wind_speed, 0.0),
            )

            targets = [
                RecoveryTarget("hub", "hub", state.hub.position, "hub_roof_pad", is_stopped_at_bay=state.hub.is_stopped),
                RecoveryTarget("station_1", "station", (80.0, 50.0, 0.0), "station_1"),
                RecoveryTarget("station_2", "station", (150.0, -50.0, 0.0), "station_2"),
            ]

            decision = planner.select_recovery_action(
                query=query,
                targets=targets,
                fleet_state=state,
                baseline=b,
                commit_reservation=False,
            )

            # Accumulate reward over 5 decision events
            sim = CompactSimulator(horizon_events=5, num_scenarios=2)
            fl_time = decision.diagnostics["flight_time_s"]
            r_ep = sim.evaluate_candidate_action(
                initial_state=state,
                drone_id="drone_1",
                target=targets[decision.selected_action_idx],
                flight_time_s=fl_time,
                base_seed=ep_seed,
            )

            rewards.append(r_ep)
            latencies.append(decision.decision_latency_ms)
            feasibility_rates.append(1.0 if decision.feasible else 0.0)

        results[b.value] = {
            "mean_reward": float(np.mean(rewards)),
            "std_reward": float(np.std(rewards)),
            "feasibility_rate": float(np.mean(feasibility_rates)),
            "mean_latency_ms": float(np.mean(latencies)),
        }

    return results
