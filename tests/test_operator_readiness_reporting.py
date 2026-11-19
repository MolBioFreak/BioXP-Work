import asyncio
import threading
import unittest
from unittest.mock import patch

from fastapi import HTTPException
from bioxp.hardware_status import HardwareStateOwner
from bioxp.lifecycle_state import CanonicalLifecycleOwner
from bioxp.operator_controls import _bounded_telemetry, _OperatorPollCache, _provider_z_motion_readiness


class ReadinessReport(unittest.TestCase):
    def test_projection_does_not_change_gate(self):
        state = {"ownership_generation": 2, "ownership": {"CAN_READY": None},
                 "maintenance": {"motion_blocked": False, "recovery_required": False},
                 "serial206_initialization_provider": {"bound": True, "z_authority": {"state": "ready"}},
                 "lifecycle": {"operation_state": "stopped"}}
        for can, blocked, reason, label in [(None, False, None, "unknown"),
                (None, True, "operator_motion_off", "off"), (True, True, "operator_motion_off", "off"),
                (True, True, "maintenance", "blocked"), (True, False, None, "on")]:
            state["ownership"]["CAN_READY"] = can
            state["maintenance"].update(motion_blocked=blocked, recovery_required=blocked, block_reason=reason)
            for catalog in [False, True]:
                motion = _bounded_telemetry(state, catalog=catalog)["motion"]
                self.assertEqual(label, motion.get("state"))
                self.assertEqual(motion["enabled"], _provider_z_motion_readiness(state)["enabled"])

    def test_same_epoch_promotion_fences_warm_views_and_old_worker(self):
        async def exercise():
            owner = HardwareStateOwner()
            lifecycle = CanonicalLifecycleOwner()
            self.enterContext(patch("bioxp.hardware_status.lifecycle_state", lifecycle))
            epoch = owner.change_ownership(reason="reset", transport="owned", usb="service", router="running")
            cache = _OperatorPollCache(lambda: owner.ownership_epoch, lambda: lifecycle.revision)
            started, release = threading.Event(), threading.Event()
            hold = False
            @cache.wrap
            async def control_catalog_v2(view):
                observed = owner.ownership_projection()["ownership"]["CAN_READY"]
                if hold:
                    started.set()
                    assert release.wait(2)
                return {"ownership_generation": epoch, "observed": observed}
            try:
                for view in ["full", "assessment"]:
                    self.assertIsNone((await control_catalog_v2(view=view))["observed"])
                hold = True
                await control_catalog_v2(view="full")
                self.assertTrue(started.wait(1))
                owner.collect(["transport", "boards"], {"transport": lambda _: {}, "boards": lambda _: {}})
                revision = lifecycle.revision
                self.assertTrue(owner.publish_can_ready_from_snapshot(
                    snapshot_id=owner.completed_snapshot()["snapshot_id"], reason="fixture evidence")["published"])
                self.assertEqual(epoch, owner.ownership_epoch)
                self.assertGreater(lifecycle.revision, revision)
                with self.assertRaises(HTTPException) as warming:
                    await control_catalog_v2(view="assessment")
                self.assertEqual(503, warming.exception.status_code)
                old = cache._pending
                release.set()
                with self.assertRaises(HTTPException):
                    await asyncio.wrap_future(old)
                for view in ["assessment", "full"]:
                    self.assertIs((await control_catalog_v2(view=view))["observed"], True)
            finally:
                release.set()
                cache.close()
        asyncio.run(exercise())
