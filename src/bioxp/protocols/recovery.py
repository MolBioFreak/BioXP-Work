"""Attempt-local authored action scope. Never construct a plan from control input."""
from dataclasses import replace
from .models import ProtocolActionKind as K


def supported(action):
    return (action.kind in (K.PIPETTE_ASPIRATE, K.PIPETTE_DISPENSE, K.PIPETTE_MIX)
            or (action.kind == K.PIPETTE_MANUAL_PHYSICAL and
                action.params.get('operation') == 'cavro_liquid_recipe' and
                action.params.get('recipe', {}).get('mode') in ('single', 'multi')))


def _offset(recipe, side, value):
    multi = recipe.get('multi')
    if side == 'source':
        if recipe.get('liquid_detection') or recipe.get('aspiration_tracking'):
            raise ValueError('Source offset requires a calibrated source lower')
        hooks = [recipe['before_liquid']]
        if multi and multi['conditioning_back_to_source_count']:
            hooks.append(multi['conditioning_before'])
    else:
        hooks = ([row['before'] for row in multi['aliquots']] if multi else [recipe['before_dispense']])
    if not hooks:
        raise ValueError('No positioned endpoint')
    for steps in hooks:
        lowers = [step['positioning'] for step in steps if step.get('operation') == 'position'
                  and step['positioning'].get('operation') == 'lower' and not step.get('rack_transform')]
        if len(lowers) != 1:
            raise ValueError('Offset requires one unambiguous calibrated lower per endpoint')
        lowers[0]['offset_steps'] = value


def adjusted(action, overrides):
    if not isinstance(overrides, dict):
        raise ValueError('Recovery overrides require an object')
    if not overrides:
        return action
    if not supported(action) or action.params.get('operation') != 'cavro_liquid_recipe':
        raise ValueError('No supported adjustments for this action')
    params = action.to_payload()['params']
    recipe = params['recipe']
    from ..pipette.cavro_application import PRESSURE_PARAMETERS
    for key, value in overrides.items():
        if type(value) is not int:
            raise ValueError('Recovery adjustments require native integers')
        if key in ('source_offset_steps', 'destination_offset_steps'):
            _offset(recipe, key.split('_')[0], value)
            continue
        if value < 0:
            raise ValueError('Recovery adjustment requires nonnegative native integer')
        if key in ('aspiration_delay_ms', 'dispense_delay_ms'):
            recipe[key] = value
        elif key in PRESSURE_PARAMETERS and recipe.get('liquid_detection'):
            recipe['liquid_detection'].setdefault('pressure_settings', {})[key] = value
        elif (key in ('on_contact_delay_ms', 'submerge_steps') and recipe.get('liquid_detection')
              and ((key == 'on_contact_delay_ms') == (recipe['liquid_detection'].get('mode') == 'on_contact'))):
            recipe['liquid_detection'][key] = value
        else:
            raise ValueError('Unsupported recovery adjustment')
    from ..manual_pipetting import manual_physical_plan
    manual_physical_plan(params)
    return replace(action, params=params)
