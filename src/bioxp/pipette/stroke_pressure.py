"""Bounded, host-receive-time evidence for finite, completion-waited strokes.

Unmarked workflows remain passive; explicit opt-in scopes transient stream control. The executor
owns attribution, the service owns retention, and the router owns ingress.
"""
from collections.abc import Mapping
from contextlib import contextmanager
from contextvars import ContextVar
from functools import wraps

_attempt = ContextVar('stroke_attempt', default=None)
_claim = ContextVar('stroke_claim', default=None)
MAX_STROKES = 32
MAX_FRAMES = 256
MAX_MS = 10000
STROKES = {'aspirate', 'dispense', 'dispense_all', 'aspirate air', 'cavro_reaspirate'}


@contextmanager
def attempt_scope(job, result, entered_id, document, fence):
    refs, pending = [], []
    identity = dict(job_id=job, source_occurrence_id=result['source_occurrence_id'],
                    native_action_id=result['action_id'], native_attempt_id=entered_id,
                    attempt=result.get('attempt'))
    method = document.metadata.get('bms_method')
    marker = method.get('workflow_pressure_streaming') if isinstance(method, Mapping) else None
    active = marker is True
    authored = False
    if active:
        from .workflow_stream import authored_control
        authored = any(authored_control(a.params) for stage in document.stages for a in stage.actions)
    if active and not identity['attempt']:
        identity['attempt'] = 1
    token = _attempt.set((identity, refs, pending, active, authored, fence)
                         if job and marker is not False and identity['attempt'] else None)
    try:
        yield
    finally:
        _attempt.reset(token)
        # Persist after the entire handler (including follow/lift/cleanup), not
        # between sends, while a pump is active, or in the USB receiver.
        for retain, ref in pending:
            try:
                retain()
            except Exception as exc:
                ref['observation_error'] = str(exc)[:256]
        if refs:
            result['pressure_traces'] = refs


def receipt_scope(store, claim, generation, call):
    attempt = _attempt.get()
    if attempt is None or not claim:
        return call()
    token = _claim.set((attempt, store, claim, generation))
    try:
        return call()
    finally:
        _claim.reset(token)


def observe_stroke(run):
    @wraps(run)
    def wrapped(group, operation, channels, callback, **kwargs):
        owner = _claim.get()
        if owner is None or operation not in STROKES or not kwargs.get('defer_completion', True):
            return run(group, operation, channels, callback, **kwargs)
        (identity, refs, pending, active, authored, fence), store, claim, generation = owner
        if len(refs) >= MAX_STROKES:
            if len(refs) == MAX_STROKES:
                refs.append({**identity, 'observation_error': 'attempt stroke capture limit reached'})
            return run(group, operation, channels, callback, **kwargs)
        ref = {**identity, 'stroke': operation, 'channels': list(channels)}
        capture = router = None
        epoch = group._interrupt_epoch
        control = None
        def send(channel, transport, defer):
            nonlocal capture, router, control
            if channel == channels[0]:
                refs.append(ref)
                try:
                    # Use only an already-bound reader; never construct a driver.
                    router = transport._driver.bus.router
                    if not all(group._transports[c]._driver.bus.router is router for c in channels):
                        raise ValueError('selected channels do not share an active pressure reader')
                    capture = router.begin_scientific_pressure_capture(dict(channels=list(channels),
                        max_frames=MAX_FRAMES, duration_ms=MAX_MS))
                    if active:
                        from .workflow_stream import StreamControl
                        control = StreamControl(group, channels, router, epoch, authored, fence, ref)
                        control.enable()
                        # Exclude frames from queries and setup, including frames queued
                        # on the receiver before this exact host-time boundary.
                        with capture.lock:
                            capture.started_at = router._clock()
                            capture.deadline = capture.started_at + MAX_MS / 1000
                            capture.frames.clear()
                            capture.lost_frames = 0
                except Exception as exc:
                    ref['observation_error'] = str(exc)[:256]
            if control is not None:
                control.check_authority()
            value = callback(channel, transport, defer)
            if control is not None:
                control.stroke_revisions[channel] = getattr(transport._driver, '_pipette_buffer_revision', None)
            return value
        outcome = None
        try:
            outcome = run(group, operation, channels, send, **kwargs)
            return outcome
        finally:
            if capture is not None:
                try:
                    finished = router.detach_scientific_pressure_capture(capture)
                    interrupted = group._interrupt_epoch != epoch
                    returned = outcome is not None
                    state = 'interrupted' if interrupted else 'completed' if returned and outcome.get('ok') else 'failed'
                    def retain():
                        from .receipts import PipetteReceiptStore
                        raw = capture.finish()
                        raw.update(finished_at=finished, stroke_identity=dict(ref),
                            acquisition='scoped_group_send_through_wait' if control else 'passive_group_send_through_wait', duration_limit_reached=finished >= capture.deadline,
                            interrupted=interrupted, ok=state=='completed', source_return_completed=returned)
                        writer = PipetteReceiptStore(store.root)
                        try:
                            retained = writer.retain_scientific_pressure_capture(raw,
                                command_id=claim['command_id'], pipette_operation_id=claim['pipette_operation_id'],
                                source_generation=generation, terminal_state=state)
                        finally:
                            writer.connection.close()
                        ref['stream_session_id'] = retained['stream_session_id']
                        if retained.get('observation_error'):
                            ref['observation_error'] = retained['observation_error'][:256]
                        elif not any(row['samples'] for row in raw['channels']):
                            ref['observation_error'] = 'no selected-channel pressure samples received during stroke'
                    pending.append((retain, ref))
                except Exception as exc:
                    ref['observation_error'] = str(exc)[:256]
            if control is not None:
                control.restore(outcome)
    return wrapped
