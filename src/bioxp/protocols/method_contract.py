"""Disconnected method compiler contract. Native validators remain authoritative.

The schema describes ordinary (non-OEM-opcode) params. Custom semantic checks
and source enum/location geometry remain native parser responsibilities.
"""
from ..manual_pipetting import ManualPipettingRequest
from ..pipette.cavro_application import ApplicationRequest, capability_catalog
from ..pipette.cavro_liquid import Recipe


def method_contract():
    def obj(properties, required=()):
        return {'type': 'object', 'additionalProperties': False,
                'properties': properties, 'required': list(required)}
    number = {'type': 'number'}
    nonnegative = {'type': 'number', 'minimum': 0}
    boolean = {'type': 'boolean'}
    timer = {'type': 'string', 'minLength': 1}
    color = {'type': 'integer', 'minimum': 0, 'maximum': 255}
    thermal = {'bank': {'enum': ['nest', 'lid', 'pedestal']}, 'target_temp_c': number,
               'fan_speed': color, 'cool_rate_c_s': {'type': 'number', 'minimum': -2, 'maximum': 0},
               'heat_rate_c_s': {'type': 'number', 'minimum': 0, 'maximum': 2}}
    rates = {'dependentRequired': {'cool_rate_c_s': ['heat_rate_c_s'], 'heat_rate_c_s': ['cool_rate_c_s']},
             'allOf': [{'if': {'properties': {'bank': {'const': 'pedestal'}}},
                        'then': {'not': {'anyOf': [{'required': ['cool_rate_c_s']}, {'required': ['heat_rate_c_s']}]}}}]}
    setpoint = {**obj(thermal, ('bank', 'target_temp_c')), **rates}
    hold = {**obj({**thermal, 'duration_s': nonnegative, 'start': {'enum': ['dispatch', 'attainment']},
                  'tolerance_c': {}, 'timeout_s': {}}, ('bank', 'target_temp_c', 'duration_s', 'start')), **rates}
    hold['allOf'] = [*rates['allOf'], {
        'if': {'properties': {'start': {'const': 'attainment'}}},
        'then': {'required': ['tolerance_c', 'timeout_s'],
                 'properties': {'tolerance_c': nonnegative, 'timeout_s': nonnegative}}}]
    params = {
        'park': obj({'rehome': boolean}),
        'led': obj(dict.fromkeys(('red', 'green', 'blue'), color), ('red', 'green', 'blue')),
        'seal_separate': obj({}),
        'wait': obj({'seconds': nonnegative}, ('seconds',)),
        'timer_start': obj({'timer_id': timer, 'seconds': nonnegative}, ('timer_id', 'seconds')),
        'timer_wait': obj({'timer_id': timer}, ('timer_id',)),
        'thermal_setpoint': setpoint, 'thermal_hold': hold,
        'thermal_profile': obj({'segments': {'type': 'array', 'minItems': 1, 'items': hold},
                                'repeat': {'type': 'integer', 'minimum': 0}}, ('segments', 'repeat')),
        'chiller_setpoint': obj({'bank': {'enum': ['rc', 'oc']}, 'target_temp_c': number}, ('bank', 'target_temp_c')),
        'snapshot': obj({}),
        'camera_illumination': obj({'channel': {'type': 'integer', 'enum': [1, 2, 3]}, 'on': boolean}, ('channel', 'on')),
        'barcode_read': obj({'mode': {'enum': ['stationary', 'job_id', 'reagent_id']}}, ('mode',)),
        'pipette_pierce': obj({'plate': {'type': 'integer'}, 'well': {'type': 'string'},
                              'pattern': {'enum': ['d', 'r', 'h', 't']}}, ('plate', 'well', 'pattern')),
    }
    return {'schema_version': 'bioxp.method_binding_contract.v1',
            'scope': 'ordinary native method additions; native validators remain authoritative',
            'action_params': params,
            'manual_pipetting_request': ManualPipettingRequest.model_json_schema(),
            'cavro_application': ApplicationRequest.model_json_schema(),
            'cavro_liquid_recipe': Recipe.model_json_schema(),
            'cavro_capabilities': capability_catalog(),
            'bindings': {
                'cavro_liquid_recipe': {'kind': 'pipette_manual_physical', 'operation': 'cavro_liquid_recipe',
                    'field': 'recipe', 'compiler': 'bioxp.pipette.cavro_liquid.compile_liquid_recipe',
                    'finite_operation': 'cavro_application', 'finite_child': 'sourceCavroApplication'},
                'park': {'finite_operation': 'park_gantry', 'finite_child': 'parkGantry',
                    'omitted_rehome': False, 'source': 'ControlLib.parkGantry:7071-7122',
                    'side_effects': ['source tip ejection/query when TipExist', 'optional authored rehome', 'source Park location publication']},
                'led': {'owner': 'BioXpTester.strip_set_rgb', 'camera_illumination': False},
                'seal_separate': {'source': 'ControlLib.scriptInterpretor.default SS',
                    'execution': 'source_noop', 'physical_seal_separation': False,
                    'operator_completion_verified': None},
                'pipette_pierce': {'owner': 'ControlLib.movExecution',
                    'pattern_plate_ordinals': {'d': [0, 1, 2, 7, 8, 9, 10], 'r': [0, 1, 2, 7, 8, 9, 10], 'h': [2], 't': [0]}}},
            'physical_qualification': False}
