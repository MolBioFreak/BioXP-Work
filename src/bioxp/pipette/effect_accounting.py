"""Completed command evidence, never liquid metrology or an admission gate.

Two independent orifice-first compartment inventories: commanded pneumatic
travel and authored liquid intent. Corrected displacement is never apportioned
to liquid. Unknown initial state keeps a known suffix/prefix and signed deltas.
"""
from collections.abc import Mapping
from copy import deepcopy
from decimal import Decimal, localcontext
from typing import Literal
from pydantic import Field
from .cavro_geometry import GeometryInput, VesselGeometry, vessel_state, decimal_work_precision
from .cavro_commands import Number
from contextvars import ContextVar
from contextlib import contextmanager


application_accounting_context: ContextVar[tuple[str, Mapping] | None] = ContextVar('application_accounting_context', default=None)


@contextmanager
def accounting_occurrence(scope, context):
    """Bind optional evidence to the inline occurrence, including device paths.

    Reset on every exit; subsequent ordinary commands must never inherit a
    declared initial state or vessel context from a finished application.
    """
    token = application_accounting_context.set((scope, context) if context is not None else None)
    try:
        yield
    finally:
        application_accounting_context.reset(token)


class Compartment(GeometryInput):
    kind: Literal['front_air', 'rear_air', 'liquid', 'unknown']
    volume_ul: Number
    label: str | None = None


class InitialChannelState(GeometryInput):
    channel: int = Field(ge=0, le=3)
    # None is unknown; [] explicitly declares empty. These are independent.
    pneumatic: list[Compartment] | None = None
    liquid_intent: list[Compartment] | None = None
    provenance: str


class AccountingContext(GeometryInput):
    context_id: str
    initial_channels: list[InitialChannelState] = Field(default_factory=list)
    vessels: list[VesselGeometry] = Field(default_factory=list)


class StrokeIntent(GeometryInput):
    role: Literal['sample', 'conditioning', 'excess', 'reaspiration', 'front_air', 'rear_air', 'authored']
    # Endpoint per physical pump, not channel-list position.
    channel_vessels: dict[str, str] = Field(default_factory=dict)
    label: str | None = None


class _Inventory:
    def __init__(self, initial=None):
        self.known = initial is not None and all(Decimal(str(s['volume_ul'])) >= 0 for s in initial)
        self.initially_known = self.known
        self.unknown_effect = False
        self.segments: list[dict] = [] if not self.known else [dict(s, volume_ul=Decimal(str(s['volume_ul']))) for s in (initial or [])]
        self.delta = Decimal(0)
        self.relative_peak = Decimal(0)
        self.travel = Decimal(0)
        self.peak = self.total() if self.known else None

    def total(self):
        with localcontext() as ctx:
            ctx.prec = decimal_work_precision(self.segments)
            return sum((s['volume_ul'] for s in self.segments), Decimal(0))

    def apply(self, signed, kind, label=None, *, selected_kind_only=False):
        transitions = []
        if signed is None:
            self.known = False
            self.unknown_effect = True
            self.segments = []
            return {'unknown_effect': True, 'crossed': [], 'unassigned_ul': None}
        self.delta += signed
        self.relative_peak = max(self.relative_peak, self.delta)
        self.travel += abs(signed)
        if signed >= 0:
            if signed:
                self.segments.insert(0, {'kind': kind, 'volume_ul': signed, 'label': label})
        else:
            left = -signed
            index = 0
            while left and index < len(self.segments):
                segment = self.segments[index]
                if selected_kind_only and segment['kind'] != kind:
                    # A liquid target does not author removal of intervening
                    # air. Report that interface, do not invent an air stroke.
                    transitions.append({'kind': segment['kind'], 'volume_ul': '0',
                                        'interface_crossed': False, 'removal_not_authored': True})
                    index += 1
                    continue
                amount = min(left, segment['volume_ul'])
                left -= amount
                segment['volume_ul'] -= amount
                transitions.append({'kind': segment['kind'], 'volume_ul': format(amount, "f"),
                                    'label': segment.get('label'), 'interface_crossed': segment['volume_ul'] == 0})
                if not segment['volume_ul']:
                    self.segments.pop(index)
                else:
                    index += 1
            if left:
                self.known = False
            return {'unknown_effect': False, 'crossed': transitions, 'unassigned_ul': format(left, "f")}
        if self.known:
            self.peak = max(self.peak or Decimal(0), self.total())
        return {'unknown_effect': False, 'crossed': transitions, 'unassigned_ul': '0'}

    def snapshot(self):
        return {'current_state_known': self.known, 'occupancy_ul': format(self.total(), "f") if self.known else None,
                'peak_occupancy_ul': format(self.peak, "f") if self.known and self.peak is not None else None,
                'known_prefix_peak_occupancy_ul': None if self.peak is None else format(self.peak, "f"),
                'relative_peak_known_delta_ul': format(self.relative_peak, "f"),
                'cumulative_travel_complete': not self.unknown_effect,
                'known_delta_ul': format(self.delta, "f"), 'cumulative_travel_ul': format(self.travel, "f"),
                'orifice_first_compartments': [dict(s, volume_ul=format(s['volume_ul'], "f")) for s in self.segments],
                'unknown_initial_or_effect': not self.known}


def application_accounting(events, context, *, incomplete):
    """Reduce real completed per-channel snapshots, never reapply effects."""
    channels = {}
    def visit(value):
        if isinstance(value, Mapping):
            body = value.get('result')
            if isinstance(value.get('channel'), int) and isinstance(body, Mapping) and isinstance(body.get('accounting'), Mapping):
                channels[str(value['channel'])] = deepcopy(body['accounting'])
            for item in value.values():
                visit(item)
        elif isinstance(value, (list, tuple)):
            for item in value:
                visit(item)
    for event in events:
        visit(event.get('result'))
        visit(event.get('partial'))
    vessels = []
    with localcontext() as ctx:
        ctx.prec = decimal_work_precision(channels, context)
        for vessel in context.get('vessels', []):
            states = [v for a in channels.values() for v in a['occupancy']['vessels'] if v['vessel_id'] == vessel['vessel_id']]
            delta = sum((Decimal(v['planned_delta_ul']) for v in states), Decimal(0))
            unknown = any(v.get('unknown_completed_liquid_effect') for v in states)
            vessels.append(dict(vessel_state(vessel if not unknown else dict(vessel, fill_ul=None), delta),
                                unknown_completed_liquid_effect=unknown))
    return {'channels': channels, 'vessels': vessels, 'completed_prefix_only': True,
            'incomplete_application': incomplete, 'uncompleted_strokes_not_estimated': True,
            'actual_delivery_ul': None, 'basis': 'completed_command_intent_not_measured_liquid'}


class CompletedEffectAccount:
    def __init__(self, initial=None, vessels=(), *, track_inventory=True):
        self.track_inventory = track_inventory
        self.pneumatic_delta = Decimal(0)
        self.intended_delta = Decimal(0)
        self.unknown_liquid_effect = False
        self.unknown_pneumatic_effect = False
        self.initial = deepcopy(initial)
        self.pneumatic = _Inventory((initial or {}).get('pneumatic'))
        self.intent = _Inventory((initial or {}).get('liquid_intent'))
        self.vessels = {v['vessel_id']: deepcopy(v) for v in vessels}
        self.vessel_deltas = {key: Decimal(0) for key in self.vessels}
        self.unknown_vessels = set()
        self.roles = {}
        self.completed_strokes = 0

    def apply(self, operation: str, result: Mapping):
        # Exact decimal authority; no binary float scientific accumulation.
        with localcontext() as ctx:
            ctx.prec = decimal_work_precision(result, self.pneumatic_delta, self.intended_delta,
                self.initial, self.pneumatic.segments, self.intent.segments, self.roles, self.vessel_deltas,
                self.pneumatic.travel, self.intent.travel)
            return self._apply(operation, result)

    def _apply(self, operation, result):
        volume = result.get('volume_ul')
        incoming = operation in {'aspirate', 'aspirate_air', 'cavro_reaspirate'}
        outgoing = operation in {'dispense', 'dispense_air'}
        pneumatic = (Decimal(str(volume)) * (1 if incoming else -1)
                     if volume is not None and (incoming or outgoing) else None)
        air = operation in {'aspirate_air', 'cavro_reaspirate'} or (
            outgoing and result.get('dispense_type') in (1, 2))
        target = result.get('target_liquid_ul')
        intended = (Decimal(0) if air else Decimal(str(target)) * (1 if incoming else -1)
                    if target is not None and (incoming or outgoing) else None)
        unknown = operation in {'dispense_all', 'mix'}
        if operation == 'cavro_device_command':
            requested = result.get('requested') or {}
            unknown = requested.get('command') in {
                'plunger_absolute', 'plunger_aspirate', 'plunger_dispense', 'initialize',
                'eject_tip', 'begin_lld', 'stored_program_run', 'diagnostic'} or 'commands' in requested
        transitions = None
        if incoming or outgoing or unknown:
            self.completed_strokes += 1
            if pneumatic is not None:
                self.pneumatic_delta += pneumatic
            else:
                self.unknown_pneumatic_effect = True
            if intended is not None:
                self.intended_delta += intended
            else:
                self.unknown_liquid_effect = True
            if self.track_inventory:
                context = result.get('accounting_intent') or {}
                role = context.get('role') or result.get('liquid_phase') or operation
                front = (result.get('dispense_type') == 1 if outgoing else
                         result.get('front_air', True) and operation != 'cavro_reaspirate')
                kind = ('front_air' if front else 'rear_air') if air else 'liquid'
                # Pneumatic 'liquid' means a stroke labelled liquid, not actual liquid.
                p = self.pneumatic.apply(pneumatic, kind, context.get('label'))
                # Air interfaces in the intent inventory use explicit air commands,
                # but liquid traversals use only target_liquid_ul, never volume_ul.
                intent_amount = pneumatic if air else intended
                i = self.intent.apply(intent_amount, kind, context.get('label'), selected_kind_only=True)
                transitions = {'pneumatic': p, 'authored_liquid_intent': i}
                totals = self.roles.setdefault(role, {'count': 0, 'pneumatic_travel_ul': Decimal(0), 'liquid_intent_travel_ul': Decimal(0)})
                totals['count'] += 1
                totals['pneumatic_travel_ul'] += abs(pneumatic or Decimal(0))
                totals['liquid_intent_travel_ul'] += abs(intended or Decimal(0))
                endpoint = result.get('accounting_vessel_id')
                if endpoint in self.vessel_deltas and intended is not None:
                    self.vessel_deltas[endpoint] -= intended
                elif endpoint in self.vessel_deltas:
                    self.unknown_vessels.add(endpoint)
        return {
            'basis': 'completed_command_intent_not_measured_liquid',
            'initial_fill_known': self.pneumatic.initially_known,
            # Retain legacy numeric projection; exact strings live below.
            'pneumatic_displacement_delta_ul': None if pneumatic is None else float(pneumatic),
            'intended_liquid_delta_ul': None if intended is None else float(intended),
            'known_pneumatic_delta_ul': float(self.pneumatic_delta),
            'known_intended_liquid_delta_ul': float(self.intended_delta),
            'unknown_liquid_effect': self.unknown_liquid_effect,
            'unknown_pneumatic_effect': self.unknown_pneumatic_effect,
            'measured_liquid_ul': None,
            **({'occupancy': {'pneumatic': self.pneumatic.snapshot(), 'authored_liquid_intent': self.intent.snapshot(),
                'initial_state': deepcopy(self.initial),
                'transitions': transitions, 'completed_strokes': self.completed_strokes,
                'roles': {k: {f: format(v, "f") if isinstance(v, Decimal) else v for f, v in row.items()} for k, row in self.roles.items()},
                'vessels': [dict(vessel_state(v if k not in self.unknown_vessels else dict(v, fill_ul=None), self.vessel_deltas[k]),
                                 unknown_completed_liquid_effect=k in self.unknown_vessels,
                                 scope='per_channel_endpoint_intent_not_aggregate_vessel') for k, v in self.vessels.items()],
                'physical_delivery_verified': False, 'capacity_enforced': False}} if self.track_inventory else {}),
        }
