"""Addressed native Stop reaches receiver during named final publication."""
import threading

from tests.test_deck_scoped_authority import retained_rig
from tests.test_deck_near_terminal import named_rig, run_named
from tests.test_z_stop_conflict_evidence import producer


def test_stop_reaches_native_receiver_while_named_finalizer_holds_writer(retained_rig, producer, monkeypatch):
    provider, observations, runtime, references, store, root = retained_rig
    stop_provider, tester, hardware = producer
    leaf, raw, execute = named_rig(retained_rig, monkeypatch, (0, 0))
    # Same named provider/lifecycle/store, real native double-Stop adapter and
    # router. Only the existing HardwareEndpoints controller leaf is doubled.
    monkeypatch.setattr(observations, 'z_stop', stop_provider.primitives.z_stop, raising=False)
    holding = threading.Event()
    release = threading.Event()
    delivered = threading.Event()
    real_commit, real_write = store.commit_deck_success, hardware.write
    errors, results = [], {}
    def write(*args, **kwargs):
        result = real_write(*args, **kwargs)
        if hardware.stop_writes == 2:
            delivered.set()
        return result
    monkeypatch.setattr(hardware, 'write', write)
    def commit(*args, **kwargs):
        with provider.deck_owner_authority_scope(), store._transaction():
            # Preserve the existing finalizer, including all authority checks.
            result = real_commit(*args, **kwargs)
            holding.set()
            assert release.wait(4)
            return result
    monkeypatch.setattr(store, 'commit_deck_success', commit)
    def move():
        try:
            results['move'] = run_named(retained_rig, execute, 'LOC_OC', 'stop-during-finalization')
        except BaseException as exc:
            errors.append(exc)
    def stop():
        try:
            results['stop'] = provider.execute_z_stop_interrupt(inputs={'command_id': 'next-stop'},
                expected_generation=3, idempotency_key='next-stop')
        except BaseException as exc:
            errors.append(exc)
    mover = threading.Thread(target=move, daemon=True)
    stopper = threading.Thread(target=stop, daemon=True)
    mover.start()
    try:
        assert holding.wait(4)
        stopper.start()
        assert delivered.wait(1), 'addressed Stop blocked behind named final publication'
        assert mover.is_alive() and stopper.is_alive()
        assert 'stop' not in results, 'Stop recording must still await the held provider/writer'
    finally:
        release.set()
        mover.join(8)
        if stopper.ident is not None:
            stopper.join(8)
    assert not mover.is_alive() and not stopper.is_alive()
    assert not errors, errors
    assert hardware.stop_writes == 2
    assert results['stop']['result']['controller_command_acknowledged'] is True
    assert results['stop']['result']['controller_terminal_state_verified'] is False
    saved = runtime.read_serial206_receipt('z', 'next-stop')
    assert saved['controller_command_acknowledged'] is True
    assert saved['controller_terminal_state_verified'] is False
