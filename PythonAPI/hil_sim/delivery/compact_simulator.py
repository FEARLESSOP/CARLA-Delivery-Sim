"""
Fast Compact Simulator for Decision-Time Lookahead (Paper 1 Method).

Provides lightweight, sub-millisecond cloneable state rollouts without CARLA
rendering overhead. Simulates H future decision events across N disturbance
scenarios using Common Random Numbers (CRN).

Event-Interval Reward Formulation (Paper 1 & Master Plan):
    r_k = 2 * D_k + 2 * O_k - 5 * M_k - (U_k / 10.0) - (E_k / 100.0)
where:
    D_k: First deliveries completed in interval k
    O_k: Deliveries completed on-time (within deadline)
    M_k: Deliveries that missed their customer deadline
    U_k: Fleet drone-minutes unavailable (empty deadhead or waiting for recharge)
    E_k: Total system energy consumed in Wh
"""

import copy
import heapq
import math
import random
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Callable, Dict, List, Optional, Tuple

import numpy as np

from delivery.battery import BatteryModel
from delivery.feasibility_engine import FeasibilityEngine, RecoveryTarget
from delivery.reservations import PadReservationManager


class DroneState(str, Enum):
    DOCKED = "docked"
    OUTBOUND_DELIVERY = "outbound_delivery"
    RECOVERING = "recovering"  # flying back to hub or station
    CHARGING = "charging"
    DEADHEAD_RETURN = "deadhead_return"  # flying empty from fixed charger back to hub


@dataclass
class SimDrone:
    drone_id: str
    position: Tuple[float, float, float]
    battery_wh: float
    max_battery_wh: float = 500.0
    state: DroneState = DroneState.DOCKED
    assigned_order_id: Optional[str] = None
    target_id: Optional[str] = None
    eta_s: float = 0.0
    total_unavailable_s: float = 0.0
    total_energy_wh: float = 0.0


@dataclass
class SimOrder:
    order_id: str
    destination: Tuple[float, float, float]
    created_at_s: float
    deadline_s: float
    weight_kg: float = 1.0
    delivered: bool = False
    delivered_at_s: Optional[float] = None
    missed: bool = False


@dataclass
class SimHub:
    hub_id: str
    position: Tuple[float, float, float]
    speed_ms: float = 8.0
    is_stopped: bool = True
    current_stop_idx: int = 0
    route_stops: List[Tuple[float, float, float]] = field(default_factory=list)
    dwell_time_remaining_s: float = 60.0
    battery_kwh: float = 10.0


@dataclass
class SimStation:
    station_id: str
    position: Tuple[float, float, float]
    capacity: int = 2
    occupied: int = 0
    recharge_rate_wh_s: float = 7.5  # fast charge deck pad


@dataclass
class EventIntervalMetrics:
    """Reward components for interval k."""
    first_deliveries: int = 0
    on_time_deliveries: int = 0
    missed_deadlines: int = 0
    unavailable_drone_minutes: float = 0.0
    total_energy_wh: float = 0.0

    def compute_reward(self) -> float:
        """Paper 1 Event Reward: r_k = 2*D + 2*O - 5*M - U/10 - E/100."""
        return (
            2.0 * self.first_deliveries
            + 2.0 * self.on_time_deliveries
            - 5.0 * self.missed_deadlines
            - (self.unavailable_drone_minutes / 10.0)
            - (self.total_energy_wh / 100.0)
        )


class CompactFleetState:
    """
    Lightweight, cloneable state representation of the fleet, mobile hub,
    fixed chargers, and customer orders.
    """

    def __init__(
        self,
        current_time_s: float = 0.0,
        hub: Optional[SimHub] = None,
        drones: Optional[Dict[str, SimDrone]] = None,
        stations: Optional[Dict[str, SimStation]] = None,
        orders: Optional[Dict[str, SimOrder]] = None,
        pad_manager: Optional[PadReservationManager] = None,
        battery_model: Optional[BatteryModel] = None,
        wind_speed_ms: float = 3.0,
    ):
        self.current_time_s = float(current_time_s)
        self.hub = hub or SimHub(
            hub_id="lmp4_pod_1",
            position=(0.0, 0.0, 2.0),
            route_stops=[(0.0, 0.0, 2.0), (100.0, 100.0, 2.0), (200.0, 0.0, 2.0)],
        )
        self.drones = drones or {
            f"drone_{i+1}": SimDrone(
                drone_id=f"drone_{i+1}",
                position=self.hub.position,
                battery_wh=500.0,
                state=DroneState.DOCKED,
            )
            for i in range(4)
        }
        self.stations = stations or {
            "station_1": SimStation("station_1", (80.0, 50.0, 0.0), capacity=2),
            "station_2": SimStation("station_2", (150.0, -50.0, 0.0), capacity=2),
        }
        self.orders = orders or {}
        self.pad_manager = pad_manager or PadReservationManager()
        self.battery_model = battery_model or BatteryModel()
        self.wind_speed_ms = float(wind_speed_ms)

        # Initialize pad reservations
        if not self.pad_manager.has_resource("hub_roof_pad"):
            self.pad_manager.register_resource("hub_roof_pad", capacity=1)
        for s_id, s in self.stations.items():
            if not self.pad_manager.has_resource(s_id):
                self.pad_manager.register_resource(s_id, capacity=s.capacity)

    def clone(self) -> "CompactFleetState":
        """Fast deepcopy clone for lookahead rollouts."""
        new_state = CompactFleetState(
            current_time_s=self.current_time_s,
            hub=copy.deepcopy(self.hub),
            drones=copy.deepcopy(self.drones),
            stations=copy.deepcopy(self.stations),
            orders=copy.deepcopy(self.orders),
            pad_manager=copy.deepcopy(self.pad_manager),
            battery_model=self.battery_model,
            wind_speed_ms=self.wind_speed_ms,
        )
        return new_state

    def step_to_next_decision_event(
        self,
        disturbance_rng: random.Random,
        traffic_delay_factor: float = 1.0,
        wind_jitter_ms: float = 0.0,
    ) -> Tuple[float, EventIntervalMetrics]:
        """
        Advance simulation to the next discrete decision event.
        Returns:
            dt: seconds advanced
            metrics: EventIntervalMetrics for the step
        """
        metrics = EventIntervalMetrics()
        # Find next earliest event time across active entities
        next_event_time = float("inf")

        # 1. Drone events
        for d in self.drones.values():
            if d.state != DroneState.DOCKED and d.eta_s > self.current_time_s:
                if d.eta_s < next_event_time:
                    next_event_time = d.eta_s

        # 2. Hub events
        if self.hub.dwell_time_remaining_s > 0:
            hub_next = self.current_time_s + self.hub.dwell_time_remaining_s
            if hub_next < next_event_time:
                next_event_time = hub_next

        # If no active events, advance by default step (e.g. 15s)
        if math.isinf(next_event_time) or next_event_time <= self.current_time_s:
            next_event_time = self.current_time_s + 15.0

        dt = max(0.5, next_event_time - self.current_time_s)
        self.current_time_s = next_event_time
        effective_wind = max(0.0, self.wind_speed_ms + wind_jitter_ms)

        # Update Hub status
        if self.hub.is_stopped:
            self.hub.dwell_time_remaining_s -= dt
            if self.hub.dwell_time_remaining_s <= 0.0 and self.hub.route_stops:
                # Hub departs for next stop
                self.hub.is_stopped = False
                self.hub.current_stop_idx = (self.hub.current_stop_idx + 1) % len(self.hub.route_stops)
                next_stop = self.hub.route_stops[self.hub.current_stop_idx]
                leg_dist = math.hypot(next_stop[0] - self.hub.position[0], next_stop[1] - self.hub.position[1])
                # Traffic stochastic delay
                travel_s = (leg_dist / self.hub.speed_ms) * traffic_delay_factor
                self.hub.dwell_time_remaining_s = travel_s
        elif self.hub.route_stops:
            self.hub.dwell_time_remaining_s -= dt
            if self.hub.dwell_time_remaining_s <= 0.0:
                # Arrived at stop
                self.hub.position = self.hub.route_stops[self.hub.current_stop_idx]
                self.hub.is_stopped = True
                self.hub.dwell_time_remaining_s = 60.0  # standard 60s dwell

        # Update Drones
        for d in self.drones.values():
            if d.state in (DroneState.RECOVERING, DroneState.DEADHEAD_RETURN):
                # Accumulate unavailable drone minutes
                metrics.unavailable_drone_minutes += (dt / 60.0)
                d.total_unavailable_s += dt

                # Drone flight energy drain
                p_rate_pct_s = self.battery_model.rate(loaded=False, climbing=False, wind=effective_wind)
                energy_wh = (p_rate_pct_s / 100.0) * d.max_battery_wh * dt
                d.battery_wh = max(0.0, d.battery_wh - energy_wh)
                metrics.total_energy_wh += energy_wh
                d.total_energy_wh += energy_wh

                # Check if arrived at destination
                if self.current_time_s >= d.eta_s:
                    if d.target_id == "hub":
                        # Dock onto mobile hub
                        d.position = self.hub.position
                        d.state = DroneState.DOCKED
                        d.battery_wh = d.max_battery_wh  # fast battery swap on hub
                        d.target_id = None
                    else:
                        # Arrived at fixed charging station
                        st = self.stations.get(d.target_id)
                        if st:
                            d.position = st.position
                            d.state = DroneState.CHARGING
                            st.occupied += 1
                            # Compute charge duration to 100%
                            needed_wh = d.max_battery_wh - d.battery_wh
                            charge_s = needed_wh / st.recharge_rate_wh_s
                            d.eta_s = self.current_time_s + charge_s
                        else:
                            d.state = DroneState.DOCKED

            elif d.state == DroneState.CHARGING:
                # Accumulate unavailable drone minutes
                metrics.unavailable_drone_minutes += (dt / 60.0)
                d.total_unavailable_s += dt

                st = self.stations.get(d.target_id)
                charge_rate = st.recharge_rate_wh_s if st else 7.5
                d.battery_wh = min(d.max_battery_wh, d.battery_wh + charge_rate * dt)

                if self.current_time_s >= d.eta_s:
                    # Fully charged -> must fly empty deadhead return back to hub
                    if st:
                        st.occupied = max(0, st.occupied - 1)
                    d.state = DroneState.DEADHEAD_RETURN
                    d.target_id = "hub"
                    return_dist = math.hypot(self.hub.position[0] - d.position[0], self.hub.position[1] - d.position[1])
                    return_s = max(5.0, return_dist / 12.0)
                    d.eta_s = self.current_time_s + return_s

            elif d.state == DroneState.OUTBOUND_DELIVERY:
                # Flying with parcel to customer
                p_rate_pct_s = self.battery_model.rate(loaded=True, climbing=False, wind=effective_wind)
                energy_wh = (p_rate_pct_s / 100.0) * d.max_battery_wh * dt
                d.battery_wh = max(0.0, d.battery_wh - energy_wh)
                metrics.total_energy_wh += energy_wh
                d.total_energy_wh += energy_wh

                if self.current_time_s >= d.eta_s:
                    # Drop completed!
                    if d.assigned_order_id and d.assigned_order_id in self.orders:
                        ord_obj = self.orders[d.assigned_order_id]
                        ord_obj.delivered = True
                        ord_obj.delivered_at_s = self.current_time_s
                        metrics.first_deliveries += 1
                        if self.current_time_s <= ord_obj.deadline_s:
                            metrics.on_time_deliveries += 1
                        else:
                            metrics.missed_deadlines += 1
                            ord_obj.missed = True

                    # Drone now needs a recovery decision (or follows continuation)
                    d.assigned_order_id = None
                    d.state = DroneState.RECOVERING

        # Check deadlines for any uncompleted orders
        for ord_obj in self.orders.values():
            if not ord_obj.delivered and not ord_obj.missed and self.current_time_s > ord_obj.deadline_s:
                ord_obj.missed = True
                metrics.missed_deadlines += 1

        return dt, metrics

    def apply_recovery_decision(
        self,
        drone_id: str,
        target_id: str,  # "hub" or station_id
        target_position: Tuple[float, float, float],
        flight_time_s: float,
    ) -> bool:
        """Apply a recovery decision for a drone completing a delivery."""
        if drone_id not in self.drones:
            return False
        d = self.drones[drone_id]
        d.state = DroneState.RECOVERING
        d.target_id = target_id
        d.eta_s = self.current_time_s + flight_time_s
        return True


class CompactSimulator:
    """
    Simulates H future decision events across N disturbance scenarios to estimate
    the rollout return Q_hat(s, a).
    """

    def __init__(
        self,
        horizon_events: int = 4,  # H in [3, 4, 5]
        num_scenarios: int = 8,   # N in [8, 16]
        gamma: float = 0.95,
        decision_budget_s: float = 0.9,
    ):
        self.H = horizon_events
        self.N = num_scenarios
        self.gamma = gamma
        self.decision_budget_s = decision_budget_s

    def evaluate_candidate_action(
        self,
        initial_state: CompactFleetState,
        drone_id: str,
        target: RecoveryTarget,
        flight_time_s: float,
        continuation_policy: Optional[Callable[[CompactFleetState], None]] = None,
        base_seed: int = 42,
    ) -> float:
        """
        Simulate N disturbance rollouts over horizon H for candidate action target.
        Returns estimated rollout value Q_hat(s, target).
        """
        scenario_returns = []

        for n in range(self.N):
            # Seed for Common Random Numbers (CRN)
            crn_seed = base_seed + (n * 10007)
            rng = random.Random(crn_seed)

            # Clone world state
            sim_state = initial_state.clone()

            # Apply candidate recovery decision
            sim_state.apply_recovery_decision(
                drone_id=drone_id,
                target_id="hub" if target.target_type == "hub" else target.target_id,
                target_position=target.position,
                flight_time_s=flight_time_s,
            )

            cumulative_return = 0.0
            discount = 1.0

            for h in range(self.H):
                # Sample stochastic disturbances
                traffic_delay = rng.lognormvariate(0.0, 0.15)  # traffic travel time jitter
                wind_jitter = rng.gauss(0.0, 1.2)             # wind speed fluctuation

                # Advance simulation by one decision event
                _, metrics = sim_state.step_to_next_decision_event(
                    disturbance_rng=rng,
                    traffic_delay_factor=traffic_delay,
                    wind_jitter_ms=wind_jitter,
                )

                # Step reward: r_k = 2D + 2O - 5M - U/10 - E/100
                r_k = metrics.compute_reward()
                cumulative_return += discount * r_k
                discount *= self.gamma

                # Optional continuation policy for other drones
                if continuation_policy:
                    continuation_policy(sim_state)

            # Terminal heuristic value estimate V(s_H)
            terminal_value = self._heuristic_terminal_value(sim_state)
            cumulative_return += discount * terminal_value
            scenario_returns.append(cumulative_return)

        # Return mean expected return Q_hat(s, a)
        return float(np.mean(scenario_returns))

    def _heuristic_terminal_value(self, state: CompactFleetState) -> float:
        """Heuristic terminal value function approximating V_theta(s_H)."""
        val = 0.0
        # Positive value for docked, high-battery ready drones
        for d in state.drones.values():
            if d.state == DroneState.DOCKED:
                val += 2.0 * (d.battery_wh / d.max_battery_wh)
            elif d.state == DroneState.CHARGING:
                val += 0.5 * (d.battery_wh / d.max_battery_wh)
            elif d.state == DroneState.DEADHEAD_RETURN:
                val -= 1.0  # penalty for remaining in deadhead leg
        # Negative value for outstanding pending orders close to deadline
        for o in state.orders.values():
            if not o.delivered:
                remaining_time = o.deadline_s - state.current_time_s
                if remaining_time < 0.0:
                    val -= 5.0
                elif remaining_time < 120.0:
                    val -= 2.0
        return val
