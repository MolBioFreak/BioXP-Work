"""Attempt-local authored action scope. Never construct a plan from control input."""
from dataclasses import replace
from .models import ProtocolActionKind as K


def supported(action):
    return (action.kind in (K.PIPETTE_ASPIRATE, K.PIPETTE_DISPENSE, K.PIPETTE_MIX)
            or (action.kind == K.PIPETTE_MANUAL_PHYSICAL and
                action.params.get('operation') == 'cavro_liquid_recipe' and
                action.params.get('recipe', {}).get('mode') == 'single'))


def adjusted(action, overrides):
    if not isinstance(overrides, dict):
        raise ValueError('Recovery overrides require an object')
    if not overrides:
        return action
    if action.params.get('operation') != 'cavro_liquid_recipe':
        raise ValueError('No supported adjustments for this action')
    params = action.to_payload()['params']
    recipe = params['recipe']
    for key, value in overrides.items():
        if type(value) is not int or value < 0:
            raise ValueError('Recovery adjustment requires nonnegative native integer')
        if key == 'aspiration_delay_ms':
            recipe[key] = value
        elif (key in ('on_contact_delay_ms', 'submerge_steps') and recipe.get('liquid_detection')
              and (key != 'on_contact_delay_ms' or recipe['liquid_detection'].get('mode') == 'on_contact')):
            recipe['liquid_detection'][key] = value
        else:
            raise ValueError('Unsupported recovery adjustment')
    from ..manual_pipetting import manual_physical_plan
    manual_physical_plan(params)
    return replace(action, params=params)
