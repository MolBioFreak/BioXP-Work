"""Run the native pickup sequence and transport with inert pump/motion leaves."""
import unittest

from bioxp.pipette.oem_calibration import (
    CalibrationBindings, CalibrationExecutionError, ManualTipLoadRequest, manual_load_tip,
)
from bioxp.pipette.transport import CanPipetteTransport, FourPipetteTransport


class TestSparsePickup(unittest.TestCase):
    def pickup(self, mounted, *, failed=None, keep=(0,)):
        self.ejected = []
        self.moves = []
        self.accounted = []
        case = self

        class Driver:
            def __init__(self, channel):
                self.channel = channel

            def query_tip_status(self):
                return {"ok": True, "semantic_ok": True, "hardware_truth_level": "hardware_query",
                        "tip_loaded": self.channel in mounted, "source_return": 1, "reader_generation": 1}

            def pipette_eject_tip(self, **kwargs):
                case.ejected.append(self.channel)
                if self.channel not in mounted or self.channel == failed:
                    return {"ok": False}
                return {"ok": True, "outcome": "completed", "completion_verified": True}

        channels = [CanPipetteTransport(driver_factory=lambda c=c: Driver(c), pipette_id=c)
                    for c in range(4)]
        for channel in channels:
            channel._initialized = True
        transport = FourPipetteTransport(channels, sleep=lambda _: None)

        def finite(name, inputs):
            self.moves.append(name)
            return {"ok": True}

        def account(transition, channels):
            self.accounted.append((transition, channels))
            return {}

        bindings = CalibrationBindings(
            finite=finite, native=lambda name, call, inputs: call(),
            pipette=lambda name, call: call(transport), facts=dict, tray_location=lambda _: 4,
            lower_pipette=lambda *_: {"ok": True, "target_position_steps": 141531},
            z_low=None, set_z_speed=None, detection_move=None, stop_z=None, pickup_effect=account,
        )
        return manual_load_tip(ManualTipLoadRequest(tray=4, well="B5", lift_z=True, keep_channels=keep), bindings)

    def test_p1_alone_does_not_issue_any_eject_or_release_lift(self):
        result = self.pickup({0})
        self.assertTrue(result["ok"])
        self.assertEqual(self.ejected, [])
        self.assertNotIn("pipette_move_z", self.moves)
        self.assertIn("pipette_home", self.moves)
        self.assertEqual(self.accounted, [("remove", [0])])

    def test_only_verified_mounted_unneeded_tips_are_ejected_and_restored(self):
        result = self.pickup({0, 1, 2})
        self.assertEqual(self.ejected, [1, 2])
        self.assertLess(self.moves.index("pipette_move_z"), self.moves.index("pipette_home"))
        self.assertEqual(result["released_channels"], [1, 2])
        self.assertEqual(self.accounted, [("remove", [0, 1, 2]), ("restore", [1, 2])])

    def test_eject_fault_on_mounted_channel_is_not_ignored(self):
        with self.assertRaises(CalibrationExecutionError) as raised:
            self.pickup({0, 3}, failed=3)
        self.assertEqual(raised.exception.evidence["failed_step"], "releaseUnneededTips")
        self.assertEqual(raised.exception.evidence["release_unverified_channels"], [3])
        self.assertEqual(self.ejected, [3])
        self.assertNotIn("pipette_home", self.moves)

    def test_full_group_keep_one_still_ejects_the_other_three(self):
        self.pickup({0, 1, 2, 3})
        self.assertEqual(self.ejected, [1, 2, 3])

    def test_ordinary_pickup_does_not_eject(self):
        self.pickup({0, 1, 2, 3}, keep=None)
        self.assertEqual(self.ejected, [])
