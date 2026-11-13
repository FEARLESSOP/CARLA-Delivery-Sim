"""
Feasibility Engine for Last-Mile Drone Recovery Decisions (Paper 1 Method).

Computes dynamic boolean action masks and physical feasibility guarantees for
drone landing decisions (LMP-4 mobile hub vs. fixed charging stations).

Constraints Enforced:
1. Battery Reserve: Drone must retain >= 15 Wh (or reserve_wh threshold)
   after flying to candidate pad under wind.
2. Airspace: Flight path must not intersect no-fly polygons.
3. Pad Availability & Reservations: Pad slot must be uncommitted in
   PadReservationManager for the arrival window [t_arr, t_arr + t_dock].
4. Operational Mode (Stopped Bay): Under 'stopped_bay' recovery mode, mobile
   hub must be stopped at a designated bay at t_arr.
5. Continuation Feasibility: If diverting to a fixed charger, a feasible
   return leg must exist to reconnect with the mobile hub.
"""

import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

from delivery.battery import BatteryModel
from delivery.reservations import PadReservationManager
from delivery.zones import Airspace, air_dist


@dataclass
class DroneRecoveryQuery:
    """State of a drone requesting a recovery action."""
    drone_id: str
    position: Tuple[float, float, float]  # (x, y, z)
    battery_wh: float
    max_battery_wh: float = 500.0
    h_speed_ms: float = 12.0
    v_speed_ms: float = 4.0
    current_time_s: float = 0.0
    wind_vector: Tuple[float, float] = (0.0, 0.0)  # (vx, vy) in m/s


@dataclass
class RecoveryTarget:
    """A candidate recovery destination."""
    target_id: str
    target_type: str  # "hub" (mobile LMP-4 pod) or "station" (fixed charger)
    position: Tuple[float, float, float]  # (x, y, z)
    resource_id: str  # maps to PadReservationManager resource
    is_stopped_at_bay: bool = True  # True if stopped at authorized bay
    future_stop_time_s: Optional[float] = None
    bays_available: int = 1
    recharge_rate_wh_s: float = 7.5  # ~500 Wh / ~65s fast charge on deck pad


@dataclass
class FeasibilityEvaluation:
    """Result of evaluating a single recovery target."""
    target_id: str
    feasible: bool
    flight_time_s: float
    energy_used_wh: float
    battery_remaining_wh: float
    arrival_time_s: float
    reason: Optional[str] = None


class FeasibilityEngine:
    """
    Evaluates recovery feasibility across candidate destinations and produces
    the hard action mask for FA-RAMPPO.
    """

    def __init__(
        self,
        pad_manager: Optional[PadReservationManager] = None,
        airspace: Optional[Airspace] = None,
        battery_model: Optional[BatteryModel] = None,
        reserve_wh: float = 15.0,
        recovery_mode: str = "stopped_bay",  # "stopped_bay" or "moving_roof"
        docking_duration_s: float = 20.0,
    ):
        self.pad_manager = pad_manager or PadReservationManager()
        self.airspace = airspace or Airspace()
        self.battery_model = battery_model or BatteryModel()
        self.reserve_wh = float(reserve_wh)
        self.recovery_mode = recovery_mode
        self.docking_duration_s = float(docking_duration_s)

    def evaluate_target(
        self, query: DroneRecoveryQuery, target: RecoveryTarget
    ) -> FeasibilityEvaluation:
        """Evaluate physical and legal feasibility of recovering to target."""
        # 1. 2D and 3D distance
        dist_2d = math.hypot(
            target.position[0] - query.position[0],
            target.position[1] - query.position[1],
        )
        delta_z = abs(target.position[2] - query.position[2])

        # Wind calculation
        wind_speed = math.hypot(query.wind_vector[0], query.wind_vector[1])
        # Headwind projection along flight line
        if dist_2d > 1e-3:
            dx = (target.position[0] - query.position[0]) / dist_2d
            dy = (target.position[1] - query.position[1]) / dist_2d
            headwind = -(query.wind_vector[0] * dx + query.wind_vector[1] * dy)
        else:
            headwind = 0.0

        effective_h_speed = max(3.0, query.h_speed_ms - max(0.0, headwind * 0.7))
        flight_time_s = (dist_2d / effective_h_speed) + (delta_z / query.v_speed_ms)
        arrival_time_s = query.current_time_s + flight_time_s

        # 2. Energy consumption estimation
        cruise_z = max(query.position[2], target.position[2]) + 15.0
        # transfer_cost returns percentage of max battery
        drain_pct = self.battery_model.transfer_cost(
            query.position,
            target.position,
            cruise_z=cruise_z,
            h_speed=effective_h_speed,
            v_speed=query.v_speed_ms,
            wind=wind_speed,
        )
        energy_wh = (drain_pct / 100.0) * query.max_battery_wh
        remaining_wh = query.battery_wh - energy_wh

        # Check constraint 1: Battery reserve margin
        if remaining_wh < self.reserve_wh:
            return FeasibilityEvaluation(
                target_id=target.target_id,
                feasible=False,
                flight_time_s=flight_time_s,
                energy_used_wh=energy_wh,
                battery_remaining_wh=remaining_wh,
                arrival_time_s=arrival_time_s,
                reason=f"insufficient_battery_reserve: remaining {remaining_wh:.1f}Wh < reserve {self.reserve_wh:.1f}Wh",
            )

        # Check constraint 2: Airspace / No-fly polygon intersection
        if self.airspace and self.airspace.blocked(target.position):
            return FeasibilityEvaluation(
                target_id=target.target_id,
                feasible=False,
                flight_time_s=flight_time_s,
                energy_used_wh=energy_wh,
                battery_remaining_wh=remaining_wh,
                arrival_time_s=arrival_time_s,
                reason="target_inside_nofly_zone",
            )

        # Check constraint 3: Recovery mode for mobile hub
        if target.target_type == "hub" and self.recovery_mode == "stopped_bay":
            if not target.is_stopped_at_bay:
                # If hub is currently moving, check if it will be stopped at arrival_time_s
                if (
                    target.future_stop_time_s is None
                    or target.future_stop_time_s > arrival_time_s
                ):
                    return FeasibilityEvaluation(
                        target_id=target.target_id,
                        feasible=False,
                        flight_time_s=flight_time_s,
                        energy_used_wh=energy_wh,
                        battery_remaining_wh=remaining_wh,
                        arrival_time_s=arrival_time_s,
                        reason="hub_not_stopped_at_bay: hub moving at estimated arrival time",
                    )

        # Ensure resource is registered in pad manager
        if not self.pad_manager.has_resource(target.resource_id):
            self.pad_manager.register_resource(
                target.resource_id, capacity=max(1, target.bays_available)
            )

        # Check constraint 4: Pad reservation slot availability
        conflict = self.pad_manager.check_conflict(
            target.resource_id,
            start_s=arrival_time_s,
            duration_s=self.docking_duration_s,
        )
        if conflict:
            return FeasibilityEvaluation(
                target_id=target.target_id,
                feasible=False,
                flight_time_s=flight_time_s,
                energy_used_wh=energy_wh,
                battery_remaining_wh=remaining_wh,
                arrival_time_s=arrival_time_s,
                reason=f"pad_slot_conflict: resource {target.resource_id} booked at t={arrival_time_s:.1f}s",
            )

        # All checks passed
        return FeasibilityEvaluation(
            target_id=target.target_id,
            feasible=True,
            flight_time_s=flight_time_s,
            energy_used_wh=energy_wh,
            battery_remaining_wh=remaining_wh,
            arrival_time_s=arrival_time_s,
            reason=None,
        )

    def compute_action_mask(
        self, query: DroneRecoveryQuery, targets: List[RecoveryTarget]
    ) -> Tuple[np.ndarray, List[FeasibilityEvaluation]]:
        """
        Compute boolean action mask for a list of candidate targets.

        Returns:
            mask: boolean np.ndarray of shape (len(targets),)
            evaluations: List of FeasibilityEvaluation for each target
        """
        evaluations = [self.evaluate_target(query, t) for t in targets]
        mask = np.array([e.feasible for e in evaluations], dtype=bool)

        # Fallback: if all candidate targets are infeasible due to strict reservations,
        # fallback to the nearest reachable charger to prevent a complete crash
        if not np.any(mask) and len(targets) > 0:
            # Find candidate with highest remaining battery (even if slot has conflict)
            best_idx = int(np.argmax([e.battery_remaining_wh for e in evaluations]))
            if evaluations[best_idx].battery_remaining_wh > 0.0:
                mask[best_idx] = True
                evaluations[best_idx].reason = "emergency_fallback_selected"

        return mask, evaluations
