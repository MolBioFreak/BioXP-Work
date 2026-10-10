"""Report decoding, separate from transport correlation and execution success."""
import math
import re


def decode_report(compiled, data):
    raw = bytes(data)
    reduced = compiled['ascii'] == '#'
    body = raw if reduced else raw[2:]
    result = {'report_kind': compiled.get('report_kind'), 'units': compiled.get('units'),
              'raw_bytes': list(raw), 'raw_ascii': None, 'value': None, 'parsed': False}
    if not reduced and (len(raw) < 2 or list(raw[:2]) != [32, 96]):
        return result
    try:
        text = body.decode('ascii')
    except UnicodeDecodeError:
        return result
    result['raw_ascii'] = text
    if reduced:
        if not re.fullmatch('[0-9A-Fa-f]{4}', text):
            return result
        value = int(text, 16)
        result.update(value=value - 65536 if value >= 32768 else value, parsed=True)
    else:
        kind = compiled.get('report_kind')
        numeric = compiled.get('units') is not None or kind in {
            'high_current_setting', 'low_current_setting', 'active_current_level',
            'pump_busy', 'digital_input', 'tip_present', 'lld_direction', 'buffer_nonempty'}
        if kind in {'status', 'extended_status', 'error_log'}:
            result.update(value=list(body), parsed=False)
            # Full original error/status definitions are source-limited; raw
            # payload is retained without claiming interpreted semantic status.
            return result
        if numeric:
            if text and text.strip() == text:
                try:
                    value = float(text)
                    if math.isfinite(value):
                        result.update(value=value, parsed=True)
                except ValueError:
                    pass
        else:
            # Firmware/configuration/log strings retain exact returned spelling.
            result.update(value=text, parsed=bool(text))
    return result
