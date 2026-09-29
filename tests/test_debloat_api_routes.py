"""A6 retired refusal wrappers must not shadow supported replacements."""
from bioxp import api


def test_retired_wrappers_absent_and_replacements_present():
    routes = {(method, route.path) for route in api.app.routes for method in getattr(route, 'methods', ())}
    retired = {
        ('POST', '/motion/interlock/prepare'),
        ('POST', '/motion/oem/z/live_right_reference'),
        ('POST', '/motion/oem/z/abort'),
        *(('GET', path) for path in (
            '/liquid/data', '/liquid/fluid-detection/{channel}/timestamp',
            '/liquid/pressure', '/liquid/condition', '/liquid/status/readback')),
    }
    assert not routes & retired
    assert {('POST', path) for method, path in retired if method == 'GET'} <= routes
    assert ('GET', '/motion/oem/z/status') in routes
    assert ('POST', '/motion/oem/prepare_without_motion') in routes
