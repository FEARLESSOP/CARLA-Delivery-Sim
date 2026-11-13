"""
Atomic Versioned Pad and Resource Reservation Manager.
パッド・充電器・ロッカーの容量制約予約マネージャー：
同時着陸や充電ポート衝突を防ぎ、アトミックな予約スワップ（ロールバック付き）を提供。

Implements Paper 1 Dynamic Aerial Recovery Calendar:
- Single-capacity LMP-4 mobile roof pad
- Multi-capacity fixed drone charging stations
- Atomic slot swaps with rollback
- Optimistic concurrency control via calendar versioning
"""

from dataclasses import dataclass, field
import uuid
from typing import Dict, List, Optional, Tuple


@dataclass
class SlotBooking:
    booking_id: str
    resource_id: str
    entity_id: str
    start_s: float
    end_s: float
    created_at_s: float
    note: str = ""

    @property
    def duration_s(self) -> float:
        return self.end_s - self.start_s


@dataclass
class ResourceCalendar:
    resource_id: str
    capacity: int = 1
    version: int = 1
    bookings: List[SlotBooking] = field(default_factory=list)


class PadReservationManager:
    """
    Coordinates access to landing pads, charging docks, and locker compartments
    with atomic versioning and conflict detection.
    """

    def __init__(self):
        self.calendars: Dict[str, ResourceCalendar] = {}

    def register_resource(self, resource_id: str, capacity: int = 1) -> None:
        """Register a shared physical resource (e.g. 'lmp4_roof_pad', 'station_dock_alpha')."""
        if capacity < 1:
            raise ValueError(f"Resource capacity must be >= 1, got {capacity}.")
        self.calendars[resource_id] = ResourceCalendar(
            resource_id=resource_id, capacity=capacity, version=1, bookings=[]
        )

    def get_version(self, resource_id: str) -> int:
        """Get the current calendar version for optimistic concurrency checks."""
        if resource_id not in self.calendars:
            raise KeyError(f"Resource '{resource_id}' is not registered.")
        return self.calendars[resource_id].version

    def has_resource(self, resource_id: str) -> bool:
        """Check if resource is registered."""
        return resource_id in self.calendars

    def check_conflict(
        self,
        resource_id: str,
        start_s: float,
        duration_s: float,
        exclude_booking_id: Optional[str] = None,
    ) -> bool:
        """Public method to query if a proposed interval has a conflict."""
        if resource_id not in self.calendars:
            return False
        cal = self.calendars[resource_id]
        return self._check_conflict(
            cal, start_s, start_s + duration_s, exclude_booking_id
        )

    def _check_conflict(
        self,
        cal: ResourceCalendar,
        start_s: float,
        end_s: float,
        exclude_booking_id: Optional[str] = None,
    ) -> bool:
        """Returns True if booking (start_s, end_s) exceeds resource capacity."""
        # Check concurrency at discrete event points within interval
        overlapping = [
            b
            for b in cal.bookings
            if b.booking_id != exclude_booking_id
            and not (b.end_s <= start_s or b.start_s >= end_s)
        ]
        if len(overlapping) < cal.capacity:
            return False

        # If overlapping >= capacity, check peak concurrency within window
        check_points = sorted(
            list(
                set(
                    [start_s, end_s]
                    + [b.start_s for b in overlapping]
                    + [b.end_s for b in overlapping]
                )
            )
        )
        for i in range(len(check_points) - 1):
            t_mid = 0.5 * (check_points[i] + check_points[i + 1])
            if start_s <= t_mid <= end_s:
                active = sum(1 for b in overlapping if b.start_s <= t_mid < b.end_s)
                if active >= cal.capacity:
                    return True
        return False

    def book_slot(
        self,
        resource_id: str,
        entity_id: str,
        start_s: float,
        duration_s: float,
        expected_version: Optional[int] = None,
        now_s: float = 0.0,
        note: str = "",
    ) -> Tuple[bool, int, Optional[str], str]:
        """
        Attempts to reserve a slot on the given resource.
        Returns: (success, new_or_current_version, booking_id, status_message)
        """
        if resource_id not in self.calendars:
            return False, 0, None, f"Unknown resource '{resource_id}'"
        cal = self.calendars[resource_id]

        if expected_version is not None and cal.version != expected_version:
            return False, cal.version, None, f"stale_version (current={cal.version}, expected={expected_version})"

        end_s = start_s + duration_s
        if duration_s <= 0:
            return False, cal.version, None, "Invalid duration <= 0"

        if self._check_conflict(cal, start_s, end_s):
            return False, cal.version, None, "slot_conflict"

        # Commit reservation
        bid = f"book_{uuid.uuid4().hex[:8]}"
        booking = SlotBooking(
            booking_id=bid,
            resource_id=resource_id,
            entity_id=entity_id,
            start_s=start_s,
            end_s=end_s,
            created_at_s=now_s,
            note=note,
        )
        cal.bookings.append(booking)
        cal.version += 1
        return True, cal.version, bid, "ok"

    def swap_slot(
        self,
        resource_id: str,
        booking_id: str,
        new_start_s: float,
        new_duration_s: float,
        expected_version: Optional[int] = None,
        now_s: float = 0.0,
    ) -> Tuple[bool, int, str]:
        """
        Atomically shifts an existing booking to a new time window.
        If the new slot conflicts or version is stale, the old booking is preserved.
        Returns: (success, current_version, status_message)
        """
        if resource_id not in self.calendars:
            return False, 0, f"Unknown resource '{resource_id}'"
        cal = self.calendars[resource_id]

        if expected_version is not None and cal.version != expected_version:
            return False, cal.version, "stale_version"

        target_b = next((b for b in cal.bookings if b.booking_id == booking_id), None)
        if not target_b:
            return False, cal.version, f"Booking '{booking_id}' not found"

        new_end_s = new_start_s + new_duration_s
        if self._check_conflict(cal, new_start_s, new_end_s, exclude_booking_id=booking_id):
            return False, cal.version, "slot_conflict"

        # Atomic commit
        target_b.start_s = new_start_s
        target_b.end_s = new_end_s
        cal.version += 1
        return True, cal.version, "swapped"

    def cancel_booking(self, resource_id: str, booking_id: str) -> bool:
        """Cancels a booking and increments calendar version."""
        if resource_id not in self.calendars:
            return False
        cal = self.calendars[resource_id]
        orig_len = len(cal.bookings)
        cal.bookings = [b for b in cal.bookings if b.booking_id != booking_id]
        if len(cal.bookings) < orig_len:
            cal.version += 1
            return True
        return False

    def clean_expired(self, current_time_s: float, grace_period_s: float = 60.0) -> int:
        """Removes bookings that ended before (current_time_s - grace_period_s)."""
        removed = 0
        cutoff = current_time_s - grace_period_s
        for cal in self.calendars.values():
            active = []
            for b in cal.bookings:
                if b.end_s < cutoff:
                    removed += 1
                else:
                    active.append(b)
            if len(active) < len(cal.bookings):
                cal.bookings = active
                cal.version += 1
        return removed

    def get_active_booking(
        self, resource_id: str, timestamp_s: float
    ) -> List[SlotBooking]:
        """Returns bookings active at timestamp_s."""
        if resource_id not in self.calendars:
            return []
        return [
            b
            for b in self.calendars[resource_id].bookings
            if b.start_s <= timestamp_s < b.end_s
        ]
