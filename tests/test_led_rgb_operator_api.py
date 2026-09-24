"""Camera-box RGB writes can preserve the existing controller ownership."""
import asyncio
from types import SimpleNamespace

import pytest
from bioxp import api


@pytest.mark.parametrize('flags,expected', [
    ({}, {'reconnect_first': True, 'activate_first': True}),
    ({'reconnect_first': False, 'activate_first': False},
     {'reconnect_first': False, 'activate_first': False}),
])
@pytest.mark.parametrize('ok', [True, False])
def test_rgb_api_passes_owner_flags_and_exact_result(monkeypatch, flags, expected, ok):
    calls = []
    result = {'ok': ok, 'rgb': (255, 0, 0), 'acks': {'r': None}}
    def strip_set_rgb(*rgb, **kwargs):
        calls.append((rgb, kwargs))
        return result
    async def run(label, fn, *, timeout_s):
        assert label == 'LED RGB' and timeout_s == 20.0
        return fn()
    monkeypatch.setattr(api, '_get_tester', lambda: SimpleNamespace(strip_set_rgb=strip_set_rgb))
    monkeypatch.setattr(api, '_run_blocking', run)
    request = api.LedRgbRequest(r=255, g=0, b=0, **flags)
    assert asyncio.run(api.led_rgb(request)) is result
    assert calls == [((255, 0, 0), expected)]
