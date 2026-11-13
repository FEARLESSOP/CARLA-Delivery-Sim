"""
Custody Ledger for Multi-Modal Autonomous Last-Mile Delivery.
荷物管理台帳：すべての荷物の所有権（保管場所）を厳密に一元管理し、二重配送や紛失を防止。

Authoritative state machine guaranteeing:
1. Every parcel ID has exactly ONE custodian at any millisecond.
2. Custody transitions are atomic, validated, and chronologically monotonic.
3. Every parcel is credited at most once upon final customer delivery.
"""

from dataclasses import dataclass, field
from enum import Enum
import time
from typing import Dict, List, Optional, Set, Tuple


class CustodyEntity(Enum):
    DEPOT = "depot"
    LMP4_HUB = "lmp4_hub"
    MINIPOD_HUB = "minipod_hub"
    DRONE = "drone"
    TRANSFER_LOCKER = "transfer_locker"
    MINIPOD = "minipod"
    CUSTOMER = "customer"


class ParcelStatus(Enum):
    PENDING = "pending"
    LOADED_HUB = "loaded_hub"
    IN_FLIGHT = "in_flight"
    DEPOSITED_LOCKER = "deposited_locker"
    OUT_FOR_DELIVERY = "out_for_delivery"
    DELIVERED = "delivered"
    CANCELLED = "cancelled"
    RETURNED_DEPOT = "returned_depot"


@dataclass
class CustodyEvent:
    from_entity: str
    to_entity: str
    timestamp_s: float
    status: ParcelStatus
    location: Optional[Tuple[float, float, float]] = None
    note: str = ""


@dataclass
class ParcelRecord:
    order_id: str
    origin: Tuple[float, float, float]
    destination: Tuple[float, float, float]
    created_at_s: float
    deadline_s: float
    weight_kg: float = 1.0
    current_custodian: str = "depot"
    status: ParcelStatus = ParcelStatus.PENDING
    history: List[CustodyEvent] = field(default_factory=list)
    delivered_at_s: Optional[float] = None


class CustodyLedger:
    """
    Transactional ledger tracking parcel custody across Depot, LMP-4 Hubs,
    Drones, Lockers, MiniPods, and Customers.
    """

    def __init__(self):
        self.parcels: Dict[str, ParcelRecord] = {}
        # Fast lookup: custodian entity_id -> Set[order_id]
        self._entity_holdings: Dict[str, Set[str]] = {}

    def register_order(
        self,
        order_id: str,
        origin: Tuple[float, float, float],
        destination: Tuple[float, float, float],
        created_at_s: float,
        deadline_s: float,
        weight_kg: float = 1.0,
        initial_custodian: str = "depot",
    ) -> ParcelRecord:
        """Register a new parcel in the ledger at the depot."""
        if order_id in self.parcels:
            raise ValueError(f"Order '{order_id}' already registered in CustodyLedger.")
        rec = ParcelRecord(
            order_id=order_id,
            origin=origin,
            destination=destination,
            created_at_s=created_at_s,
            deadline_s=deadline_s,
            weight_kg=weight_kg,
            current_custodian=initial_custodian,
            status=ParcelStatus.PENDING,
        )
        rec.history.append(
            CustodyEvent(
                from_entity="none",
                to_entity=initial_custodian,
                timestamp_s=created_at_s,
                status=ParcelStatus.PENDING,
                location=origin,
                note="Order registered",
            )
        )
        self.parcels[order_id] = rec
        self._entity_holdings.setdefault(initial_custodian, set()).add(order_id)
        return rec

    def transfer_custody(
        self,
        order_id: str,
        from_entity: str,
        to_entity: str,
        timestamp_s: float,
        new_status: ParcelStatus,
        location: Optional[Tuple[float, float, float]] = None,
        note: str = "",
    ) -> bool:
        """
        Atomically transfers parcel custody from one entity to another.
        Enforces single-custodian invariant and chronological ordering.
        """
        if order_id not in self.parcels:
            raise KeyError(f"Order '{order_id}' does not exist in ledger.")
        rec = self.parcels[order_id]

        if rec.status in (ParcelStatus.DELIVERED, ParcelStatus.CANCELLED):
            raise RuntimeError(
                f"Cannot transfer order '{order_id}': already in terminal state '{rec.status.value}'."
            )

        if rec.current_custodian != from_entity:
            raise PermissionError(
                f"Custody violation for '{order_id}': entity '{from_entity}' attempted to release "
                f"parcel, but current custodian is '{rec.current_custodian}'."
            )

        if rec.history and timestamp_s < rec.history[-1].timestamp_s:
            raise ValueError(
                f"Chronological violation for '{order_id}': transfer at t={timestamp_s:.2f}s "
                f"is earlier than last event at t={rec.history[-1].timestamp_s:.2f}s."
            )

        # Atomic state update
        self._entity_holdings.get(from_entity, set()).discard(order_id)
        self._entity_holdings.setdefault(to_entity, set()).add(order_id)

        rec.current_custodian = to_entity
        rec.status = new_status
        if new_status == ParcelStatus.DELIVERED:
            rec.delivered_at_s = timestamp_s

        rec.history.append(
            CustodyEvent(
                from_entity=from_entity,
                to_entity=to_entity,
                timestamp_s=timestamp_s,
                status=new_status,
                location=location,
                note=note,
            )
        )
        return True

    def get_custodian(self, order_id: str) -> str:
        """Returns the current custodian entity ID of the parcel."""
        if order_id not in self.parcels:
            raise KeyError(f"Order '{order_id}' not found.")
        return self.parcels[order_id].current_custodian

    def get_holdings(self, entity_id: str) -> Set[str]:
        """Returns all order IDs currently held by an entity."""
        return set(self._entity_holdings.get(entity_id, set()))

    def audit_integrity(self) -> Dict[str, any]:
        """
        Audits ledger invariants across all registered parcels:
        1. No parcel held by multiple entities.
        2. Non-terminal parcels accounted for in entity holdings.
        3. Monotonic timestamps in event histories.
        """
        errors = []
        delivered_count = 0
        active_count = 0

        for oid, rec in self.parcels.items():
            if rec.status == ParcelStatus.DELIVERED:
                delivered_count += 1
                if rec.delivered_at_s is None:
                    errors.append(f"Parcel {oid} is DELIVERED but missing delivered_at_s.")
            else:
                active_count += 1
                holder = rec.current_custodian
                if oid not in self._entity_holdings.get(holder, set()):
                    errors.append(
                        f"Parcel {oid} custodian is '{holder}' but not in holdings index."
                    )

            # Monotonic history check
            last_t = -1.0
            for ev in rec.history:
                if ev.timestamp_s < last_t:
                    errors.append(
                        f"Parcel {oid} has non-monotonic timestamp {ev.timestamp_s} < {last_t}."
                    )
                last_t = ev.timestamp_s

        return {
            "valid": len(errors) == 0,
            "total_parcels": len(self.parcels),
            "delivered": delivered_count,
            "active": active_count,
            "errors": errors,
        }

    def compute_kpi(self, current_time_s: float) -> Dict[str, float]:
        """Computes academic paper KPIs over all parcels."""
        total = len(self.parcels)
        if total == 0:
            return {
                "total_orders": 0,
                "delivered": 0,
                "on_time_rate": 0.0,
                "missed_deadlines": 0,
                "lead_mean_s": 0.0,
                "lead_p95_s": 0.0,
                "max_order_age_s": 0.0,
            }

        delivered = [p for p in self.parcels.values() if p.status == ParcelStatus.DELIVERED]
        lead_times = [p.delivered_at_s - p.created_at_s for p in delivered if p.delivered_at_s]
        on_time = [p for p in delivered if p.delivered_at_s <= p.deadline_s]

        # Calculate ages of unfulfilled orders
        unfulfilled = [p for p in self.parcels.values() if p.status != ParcelStatus.DELIVERED]
        ages = [current_time_s - p.created_at_s for p in unfulfilled]
        missed = sum(
            1
            for p in self.parcels.values()
            if (p.delivered_at_s and p.delivered_at_s > p.deadline_s)
            or (not p.delivered_at_s and current_time_s > p.deadline_s)
        )

        lead_times_sorted = sorted(lead_times)
        p95_idx = int(0.95 * len(lead_times_sorted)) if lead_times_sorted else 0
        lead_p95 = lead_times_sorted[p95_idx] if lead_times_sorted else 0.0

        return {
            "total_orders": total,
            "delivered": len(delivered),
            "delivered_rate": len(delivered) / total,
            "on_time_count": len(on_time),
            "on_time_rate": len(on_time) / max(len(delivered), 1),
            "missed_deadlines": missed,
            "lead_mean_s": sum(lead_times) / max(len(lead_times), 1),
            "lead_p95_s": lead_p95,
            "max_order_age_s": max(ages) if ages else 0.0,
        }
