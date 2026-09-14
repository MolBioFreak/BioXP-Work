"""Passive deck cache compares current owners, not its own saved epochs."""
import pytest
import time

def wait_enabled(app, expected):
    deadline = time.monotonic() + 5
    row = None
    while time.monotonic() < deadline:
        row = catalog_action(app)
        if row["enabled"] is expected:
            return row
        time.sleep(.05)
    raise AssertionError(row)
from tests.test_deck_scoped_authority import retained_rig, qualify_test_references
from tests.test_deck_scoped_integration import installed_retained, catalog_action


@pytest.mark.parametrize('key', ['ownership_generation', 'board_epoch_4', 'board_epoch_5'])
@pytest.mark.parametrize('replacement', ['missing', 'changed'])
def test_passive_catalog_rejects_independently_changed_owner(installed_retained, monkeypatch, key, replacement):
    from bioxp import api
    app, provider, primitive, references, root = installed_retained
    assert catalog_action(app)['enabled'] is False
    qualify_test_references(references)
    collected = api._collect_and_publish_hardware_snapshot(['axes','latch'], reason='isolated-cache-fence')
    assert collected['deck_authority']['enabled'] is True
    assert wait_enabled(app, True)['enabled'] is True
    stamps = provider.deck_owner_authority_stamps()
    # Fault injection at the independent current-owner read, not the cached
    # authority or the UI response. No controller reads or writes are allowed.
    stale_owner = dict(stamps)
    if replacement == 'missing':
        stale_owner.pop(key)
    else:
        stale_owner[key] += 1
    calls = list(primitive.calls)
    monkeypatch.setattr(provider, 'deck_owner_authority_stamps', lambda: dict(stale_owner))
    assert wait_enabled(app, False)['enabled'] is False
    assert primitive.calls == calls
