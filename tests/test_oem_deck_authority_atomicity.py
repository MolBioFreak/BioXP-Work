from __future__ import annotations

from dataclasses import replace
import sys
import types

import pytest

from bioxp.oem_deck_catalog import DeckCatalog, configured_location_names
from bioxp import oem_deck_movement
from bioxp.oem_deck_movement import DeckAuthoritySnapshot, NamedLocationIntent, compile_named_location
from bioxp.oem_compat.position_table import PositionTable

if "usb" not in sys.modules:
    usb_package = types.ModuleType("usb")
    usb_core = types.ModuleType("usb.core")
    usb_util = types.ModuleType("usb.util")
    usb_package.core = usb_core
    usb_package.util = usb_util
    sys.modules.update({"usb": usb_package, "usb.core": usb_core, "usb.util": usb_util})

from bioxp.usb_driver import BioXpTester


def fixtures():
    table = PositionTable.from_rows([{"name": n, "x": i, "y": i, "zLow": 60000, "zDelta": 10000, "inc_factor": 0} for i, n in enumerate(configured_location_names())])
    authority = DeckAuthoritySnapshot(1, "owner", 2, 3, table.digest, 1, {"x": 1, "y": 1, "z": 1, "g": 1}, {"global": 0, "x": 0, "y": 0, "z": 0}, "l", "p", 1.0, 0, 0, 65000, "LOC_MS", 0, False, False, -1, True, None, 65000, "BIOXP", True, True)
    return table, authority


class ExplodingProvider:
    writes = 0
    def moveTo(self, **kwargs):
        self.writes += 1
        raise AssertionError("physical write must not occur")


def test_authority_drift_before_first_write_yields_zero_provider_io() -> None:
    table, authority = fixtures(); provider = ExplodingProvider()
    plan = compile_named_location(NamedLocationIntent("LOC_OC"), DeckCatalog.from_position_table(table), table, authority)
    executor = oem_deck_movement.DeckMovementExecutor(provider, lambda: replace(authority, machine_state_revision=2))
    with pytest.raises(oem_deck_movement.MovementAuthorityChanged, match="before_first_tx"):
        executor.execute(plan)
    assert provider.writes == 0


def test_authority_digest_ignores_capture_timestamp_but_detects_authority_drift() -> None:
    _table, authority = fixtures()
    assert replace(authority, captured_at=2.0).digest == authority.digest
    assert replace(authority, machine_state_revision=2).digest != authority.digest


def test_successful_unlock_latches_host_status_false_and_lock_cannot_reset_it() -> None:
    tester = BioXpTester.__new__(BioXpTester)
    tester._oem_latch_status = True
    tester._oem_latch_status_generation = 0
    tester._send_motor = lambda *_args, **_kwargs: {"status": 100, "value": 0}
    tester._tmcl_success = lambda ack: ack is not None and ack.get("status") == 100
    tester.activate_boards = lambda **_kwargs: None
    tester.send_tmcl_retry = lambda *_args, **_kwargs: {"status": 100, "value": 0}
    tester.send_tmcl = lambda *_args, **_kwargs: None
    tester.io_snapshot = lambda *_args, **_kwargs: {3: 1}

    unrelated = tester.deck_io_set_type(2, 0)
    before_diagnostic = tester.read_oem_latch_status()
    unlocked = tester.latch_oem(False)
    after_unlock = tester.read_oem_latch_status()
    locked = tester.latch_oem(True)
    after_lock = tester.read_oem_latch_status()

    assert unrelated["ok"] is True
    assert before_diagnostic["value"] is True
    assert unlocked["ack"]["status"] == 100 and locked["ack"]["status"] == 100
    assert after_unlock["value"] is False
    assert after_lock == after_unlock
