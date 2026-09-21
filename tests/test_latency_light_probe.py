"""Harness produces compact GET measurements, never command or collect POSTs."""
import io
import json
from urllib.error import HTTPError

import pytest
from scripts import bioxp_latency_probe as probe


class Response(io.BytesIO):
    code = 200


def test_sample_issues_only_get_and_discards_evidence():
    def opener(request, **kwargs):
        assert request.get_method() == 'GET'
        return Response(json.dumps({'status': 'completed', 'terminal': True,
                                    'provider_evidence': 'PRIVATE'}).encode())
    result = probe.sample('http://example.invalid/status', timeout=1, opener=opener)
    assert result['status'] == 'completed'
    assert 'PRIVATE' not in json.dumps(result)
    assert result['elapsed_s'] >= 0


def test_oversized_response_is_bounded():
    result = probe.sample('http://example.invalid/status', timeout=1,
                          opener=lambda *a, **k: Response(b'x' * (probe.MAX_RESPONSE_BYTES + 2)))
    assert result['error'] == 'response_too_large'
    assert result['response_bytes'] == probe.MAX_RESPONSE_BYTES + 1


def test_cli_only_polls_existing_command_compactly(monkeypatch, capsys):
    seen = []
    monkeypatch.setattr(probe, 'sample', lambda url, **kw: seen.append(url) or {'http_status': 200, 'elapsed_s': .1})
    assert probe.main(['--base-url', 'http://example.invalid', '--command', 'command-1', '--count', '1']) == 0
    assert seen == ['http://example.invalid/operator/v2/actions/receipts/command-1?detail=false']
    assert json.loads(capsys.readouterr().out)['median_s'] == .1


@pytest.mark.parametrize('url', ['http://user:secret@example.invalid', 'file:///tmp/db', 'https://example.invalid?token=secret'])
def test_credentials_and_non_http_targets_rejected(url):
    with pytest.raises(SystemExit): probe.main(['--base-url', url, '--count', '1'])
