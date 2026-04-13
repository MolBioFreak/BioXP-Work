# TMCL trace fixtures

This directory is the starter corpus for BioXP TMCL request/reply decode work.

## Layout

- `fixtures/*.jsonl`: line-oriented capture records that mirror the output shape of `scripts/capture_tmcl_session.py`
- each line is a JSON object with at least:
  - `timestamp`
  - `timestamp_ns`
  - `direction` (`tx`, `rx`, or `event`)
  - `frame` (byte array)
  - `hex`
  - `length`

## Decode usage

Decode a stored session log into correlated transactions:

```bash
python scripts/decode_tmcl_frame.py --session --input testdata/tmcl/fixtures/manual_gap_and_gio_session.jsonl
```

Decode a single captured line or raw frame:

```bash
python scripts/decode_tmcl_frame.py '{"direction":"tx","frame":[126,0,0,0,4,8,138,0,1,0,0,0,0,0,151,126]}'
```

## Capture usage

The live capture helper writes JSONL in the same format:

```bash
python scripts/capture_tmcl_session.py command --board 0x04 --command 138 --motor 1 --output testdata/tmcl/fixtures/live_query_motor_stop.jsonl
```

## Starter fixtures

- `manual_query_motor_stop_session.jsonl`
  - one motor-stop query transaction on the head board
- `manual_gap_and_gio_session.jsonl`
  - one motor parameter query plus one deck IO query
- `manual_thermal_sensor_session.jsonl`
  - one thermal sensor temperature read transaction

These fixtures are intentionally small and sanitized. They are suitable for decoder regression tests and as templates for adding future real-world captures.
