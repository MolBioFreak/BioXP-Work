"""Record-only invalidation at standalone command dispatch, never motor leaves.

The mutation joins an existing dispatch transaction; it creates no transaction,
writer thread, controller work, retry, or new motion prerequisite. A failed
bookkeeping write retains a process-local UNKNOWN projection until a genuine
named arrival replaces it. SQLite remains the durable source.
"""
from __future__ import annotations


import json
import logging
from pathlib import Path
import threading
import time
import uuid

_LOG = logging.getLogger(__name__)
_LOCK = threading.Lock()
_UNKNOWN_ROOTS: set[str] = set()

# Closed roster: don't infer motion from prefixes (profiles and Stops share them).
STANDALONE_ACTIONS = frozenset({
    'oem.x.move_steps', 'oem.x.move_absolute', 'oem.x.manual_panel_home',
    'oem.x.diagnostic_home_axis', 'oem.x.startup_home',
    'oem.x.move_to_origin_home', 'oem.x.caught_plate_recovery_home', 'oem.x.set_home',
    'oem.y.move_steps', 'oem.y.move_absolute', 'oem.y.manual_panel_home',
    'oem.xy.home', 'oem.xy.home_xy', 'oem.xy.move_absolute', 'oem.xy.move_xy',
    'oem.z.manual_home', 'oem.z.diagnostic_home_axis', 'oem.z.move_steps',
    'oem.z.move_absolute', 'oem.z.set_home', 'oem.z.clear', 'oem.z.move_z_home',
    'oem.z.move_gz', 'oem.z.home_gz', 'oem.z.lower_pipette', 'oem.z.lift_pipette',
    'oem.z.self_test', 'oem.z.resume_after_abort',
})


def standalone_xyz_route(path: str, inputs: dict) -> bool:
    if path in {
        '/motion/oem/y/internal/acceleration_overload',
        '/motion/oem/y/internal/board_test_my', '/motion/oem/y/set_home',
        '/motion/oem/move_xy', '/motion/oem/home_xy',
    }:
        return True
    if path in {'/motion/oem/manual/home', '/motion/oem/manual/sethome',
                '/motion/oem/manual/relative', '/motion/oem/manual/absolute',
                '/motion/axis/relative', '/motion/axis/absolute', '/motion/axis/zero',
                '/motion/axis/home'}:
        return str(inputs.get('axis', '')).lower() in {'x', 'y', 'z'}
    return False


def location_is_invalidated(root) -> bool:
    with _LOCK:
        return str(Path(root).resolve()) in _UNKNOWN_ROOTS


def named_location_published(root) -> None:
    with _LOCK:
        _UNKNOWN_ROOTS.discard(str(Path(root).resolve()))


def invalidate_at_dispatch(conn, *, root, command_id: str) -> None:
    """Caller owns its existing transaction and authority-write scope.

    Preserve all fields except CurrentLocation, including well/tray/tips/custody
    and controller/home records. Restored machine_status projects this durable
    UNKNOWN until the next named arrival; old immutable snapshots stay intact.
    Savepoint failures are observations, not refusals or a rollback of dispatch.
    """
    with _LOCK:
        _UNKNOWN_ROOTS.add(str(Path(root).resolve()))
    conn.execute('SAVEPOINT standalone_location_record')
    try:
        current = conn.execute(
            'SELECT * FROM operator_plane_deck_semantic_state WHERE singleton=1').fetchone()
        if current is not None and current['current_location'] != 'UNKNOWN':
            now = time.time()
            producer = command_id
            if conn.execute('SELECT 1 FROM operator_plane_commands WHERE command_id=?',
                            (producer,)).fetchone() is None:
                # Legacy dispatch has a canonical command but no plane mirror.
                # This record is bookkeeping only, never an executable command.
                producer = str(uuid.uuid4())
                sequence = conn.execute(
                    'SELECT COALESCE(MAX(stream_sequence),0)+1 FROM operator_plane_commands').fetchone()[0]
                payload = _json({'source_command_id': command_id, 'delivery_attempted': False})
                conn.execute(
                    'INSERT INTO operator_plane_commands(command_id,stream_sequence,action_id,requested_json,effective_json,status,version,ownership_generation,queued_at,dispatched_at,finished_at,terminal_json,updated_at) VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?)',
                    (producer, sequence, 'oem.deck.semantic_state_publication', payload, payload,
                     'completed', 1, int(current['ownership_generation'] or 0), now, now, now, payload, now))
            before = int(current['semantic_state_revision'])
            provenance = json.loads(current['transition_provenance_json'] or '{}')
            provenance.update(source_operation='standalone_axis_motion', command_id=producer,
                              upstream_source_command_id=command_id, before_revision=before,
                              after_revision=before + 1, updates={'current_location': 'UNKNOWN'})
            encoded = _json(provenance)
            conn.execute(
                "UPDATE operator_plane_deck_semantic_state SET current_location='UNKNOWN',semantic_state_revision=?,producer_operation='standalone_axis_motion',producer_command_id=?,transition_provenance_json=?,updated_at=? WHERE singleton=1",
                (before + 1, producer, encoded, now))
            conn.execute(
                'INSERT INTO operator_plane_deck_semantic_transitions(command_id,source_operation,before_revision,after_revision,transition_json,created_at) VALUES(?,?,?,?,?,?)',
                (producer, 'standalone_axis_motion', before, before + 1, encoded, now))
        conn.execute('RELEASE standalone_location_record')
    except Exception:
        conn.execute('ROLLBACK TO standalone_location_record')
        conn.execute('RELEASE standalone_location_record')
        _LOG.exception('Standalone XYZ location invalidation could not be persisted; location remains UNKNOWN in this process')


def _json(value) -> str:
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(',', ':'), allow_nan=False)
