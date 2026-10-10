"""Completed controller displacement is not delivered-liquid metrology.

Legacy counters remain compatible. This separate evidence projection never
changes admission and never assumes an empty tip at construction.
"""
from collections.abc import Mapping


class CompletedEffectAccount:
    def __init__(self):
        self.pneumatic_delta = 0.0
        self.intended_delta = 0.0
        self.unknown_liquid_effect = False
        self.unknown_pneumatic_effect = False

    def apply(self, operation: str, result: Mapping):
        volume = result.get('volume_ul')
        incoming = operation in {'aspirate', 'aspirate_air', 'cavro_reaspirate'}
        outgoing = operation in {'dispense', 'dispense_air'}
        pneumatic = (float(volume) * (1 if incoming else -1)
                     if volume is not None and (incoming or outgoing) else None)
        air = operation in {'aspirate_air', 'cavro_reaspirate'} or (
            outgoing and result.get('dispense_type') in (1, 2))
        target = result.get('target_liquid_ul')
        intended = (0.0 if air else float(target) * (1 if incoming else -1)
                    if target is not None and (incoming or outgoing) else None)
        if pneumatic is not None:
            self.pneumatic_delta += pneumatic
        elif operation in {'dispense_all', 'mix', 'cavro_device_command'}:
            requested = result.get('requested') or {}
            if operation != 'cavro_device_command' or requested.get('command') in {
                'plunger_absolute', 'plunger_aspirate', 'plunger_dispense', 'initialize',
                'eject_tip', 'begin_lld', 'stored_program_run', 'diagnostic'} or 'commands' in requested:
                self.unknown_pneumatic_effect = True
                self.unknown_liquid_effect = True
        if intended is not None:
            self.intended_delta += intended
        if operation in {'aspirate', 'dispense', 'dispense_air', 'dispense_all', 'mix'} and intended is None:
            self.unknown_liquid_effect = True
        return {
            'basis': 'completed_command_intent_not_measured_liquid',
            'initial_fill_known': False,
            'pneumatic_displacement_delta_ul': pneumatic,
            'intended_liquid_delta_ul': intended,
            'known_pneumatic_delta_ul': self.pneumatic_delta,
            'known_intended_liquid_delta_ul': self.intended_delta,
            'unknown_liquid_effect': self.unknown_liquid_effect,
            'unknown_pneumatic_effect': self.unknown_pneumatic_effect,
            'measured_liquid_ul': None,
        }
