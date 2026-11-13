"""
Unit tests for CustodyLedger and PadReservationManager.
"""

import sys
import unittest

sys.path.insert(0, ".")
from delivery.ledger import CustodyLedger, ParcelStatus
from delivery.reservations import PadReservationManager


class TestCustodyLedger(unittest.TestCase):
    def setUp(self):
        self.ledger = CustodyLedger()

    def test_order_registration(self):
        rec = self.ledger.register_order(
            order_id="ord_001",
            origin=(10.0, 20.0, 0.0),
            destination=(100.0, 200.0, 0.0),
            created_at_s=100.0,
            deadline_s=700.0,
            weight_kg=1.5,
        )
        self.assertEqual(rec.order_id, "ord_001")
        self.assertEqual(rec.current_custodian, "depot")
        self.assertEqual(rec.status, ParcelStatus.PENDING)
        self.assertIn("ord_001", self.ledger.get_holdings("depot"))

    def test_custody_transfer_lifecycle(self):
        self.ledger.register_order("ord_002", (0, 0, 0), (50, 50, 0), 10.0, 500.0)

        # Depot -> LMP-4 Hub
        self.ledger.transfer_custody(
            "ord_002", "depot", "lmp4_pod_1", 20.0, ParcelStatus.LOADED_HUB
        )
        self.assertEqual(self.ledger.get_custodian("ord_002"), "lmp4_pod_1")
        self.assertNotIn("ord_002", self.ledger.get_holdings("depot"))
        self.assertIn("ord_002", self.ledger.get_holdings("lmp4_pod_1"))

        # LMP-4 Hub -> Drone
        self.ledger.transfer_custody(
            "ord_002", "lmp4_pod_1", "drone_1", 30.0, ParcelStatus.IN_FLIGHT
        )
        self.assertEqual(self.ledger.get_custodian("ord_002"), "drone_1")

        # Drone -> Customer (Delivery)
        self.ledger.transfer_custody(
            "ord_002", "drone_1", "customer_42", 65.0, ParcelStatus.DELIVERED
        )
        self.assertEqual(self.ledger.get_custodian("ord_002"), "customer_42")

        # Audit integrity
        audit = self.ledger.audit_integrity()
        self.assertTrue(audit["valid"])
        self.assertEqual(audit["delivered"], 1)

        # KPI check
        kpi = self.ledger.compute_kpi(current_time_s=100.0)
        self.assertEqual(kpi["delivered"], 1)
        self.assertEqual(kpi["on_time_count"], 1)
        self.assertEqual(kpi["lead_mean_s"], 55.0)  # 65 - 10 = 55s

    def test_custody_violations(self):
        self.ledger.register_order("ord_003", (0, 0, 0), (10, 10, 0), 0.0, 300.0)
        # Attempt to transfer from drone_1 when depot holds it
        with self.assertRaises(PermissionError):
            self.ledger.transfer_custody(
                "ord_003", "drone_1", "customer", 10.0, ParcelStatus.DELIVERED
            )

        # Non-monotonic timestamp
        self.ledger.transfer_custody("ord_003", "depot", "drone_1", 50.0, ParcelStatus.IN_FLIGHT)
        with self.assertRaises(ValueError):
            self.ledger.transfer_custody("ord_003", "drone_1", "customer", 40.0, ParcelStatus.DELIVERED)


class TestPadReservationManager(unittest.TestCase):
    def setUp(self):
        self.mgr = PadReservationManager()
        self.mgr.register_resource("lmp4_pad", capacity=1)
        self.mgr.register_resource("station_charger", capacity=2)

    def test_single_pad_reservation(self):
        # First booking succeeds
        ok, v1, b1, msg = self.mgr.book_slot("lmp4_pad", "drone_1", start_s=100.0, duration_s=30.0)
        self.assertTrue(ok)
        self.assertIsNotNone(b1)
        self.assertEqual(v1, 2)

        # Overlapping booking fails
        ok2, v2, b2, msg2 = self.mgr.book_slot("lmp4_pad", "drone_2", start_s=110.0, duration_s=30.0)
        self.assertFalse(ok2)
        self.assertEqual(msg2, "slot_conflict")

        # Non-overlapping booking succeeds
        ok3, v3, b3, msg3 = self.mgr.book_slot("lmp4_pad", "drone_2", start_s=130.0, duration_s=20.0)
        self.assertTrue(ok3)

    def test_optimistic_concurrency_versioning(self):
        v = self.mgr.get_version("lmp4_pad")
        # Agent A books slot
        ok_a, v_a, _, _ = self.mgr.book_slot("lmp4_pad", "drone_1", 200.0, 20.0, expected_version=v)
        self.assertTrue(ok_a)

        # Agent B had the old version v, should fail with stale_version
        ok_b, _, _, msg_b = self.mgr.book_slot("lmp4_pad", "drone_2", 230.0, 20.0, expected_version=v)
        self.assertFalse(ok_b)
        self.assertIn("stale_version", msg_b)

    def test_atomic_swap_with_rollback(self):
        # Book slot 1 and slot 2
        ok1, _, b1, _ = self.mgr.book_slot("lmp4_pad", "drone_1", start_s=300.0, duration_s=30.0)
        ok2, _, b2, _ = self.mgr.book_slot("lmp4_pad", "drone_2", start_s=340.0, duration_s=30.0)
        self.assertTrue(ok1 and ok2)

        # Drone 1 attempts to shift into Drone 2's slot (conflict)
        ok_swap, _, msg = self.mgr.swap_slot("lmp4_pad", b1, new_start_s=350.0, new_duration_s=20.0)
        self.assertFalse(ok_swap)
        self.assertEqual(msg, "slot_conflict")

        # Verify Drone 1 still has its original slot intact
        active = self.mgr.get_active_booking("lmp4_pad", 310.0)
        self.assertEqual(len(active), 1)
        self.assertEqual(active[0].booking_id, b1)

    def test_multi_capacity_resource(self):
        # Charger capacity 2
        ok1, _, _, _ = self.mgr.book_slot("station_charger", "drone_1", 100.0, 50.0)
        ok2, _, _, _ = self.mgr.book_slot("station_charger", "drone_2", 120.0, 50.0)
        self.assertTrue(ok1 and ok2)

        # 3rd simultaneous booking fails
        ok3, _, _, msg = self.mgr.book_slot("station_charger", "drone_3", 130.0, 20.0)
        self.assertFalse(ok3)
        self.assertEqual(msg, "slot_conflict")


if __name__ == "__main__":
    unittest.main()
