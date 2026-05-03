# BioXP3200 OEM Phase1 Transport Progress — 2026-05-03

## Scope

Phase1 objective is transport and reply compliance: command/reply correlation, async-event separation, dry-run/shadow/live safety boundaries, and fail-closed behavior when controller replies are missing, ambiguous, or non-success.

## Implemented in this slice

- Strict TMCL reply parsing remains anchored to the robot driver response shape:
  - board ID: byte 6
  - status: byte 7
  - command: byte 8
  - signed value: bytes 9:13
- `match_tmcl_reply(...)` now fails closed when:
  - no matching board/command reply exists
  - more than one matching board/command reply exists
  - the matching reply has non-success status while success is required
- Added explicit `AmbiguousReply` and `MutatingCommandBlocked` exceptions.
- Added `ShadowTransport` query-only seam:
  - `opened_usb=true` is allowed in shadow mode
  - physical motion remains forbidden by the shared safety contract
  - only read/status TMCL commands are allowed through to the adapter
  - mutating frames are blocked before reaching the adapter

## Current shadow read whitelist

- TMCL `GAP` command 6: get axis parameter / status reads
- TMCL command 138: existing robot query-stop/status command

This is intentionally narrow. Expand only with source-backed, non-motion/status-only commands.

## Tests

Validated with:

```text
python3 -m pytest tests/test_oem_compat_transport.py tests/test_oem_compat_api.py tests/test_oem_oracle_extractor.py tests/test_oem_binding_loader.py -q
29 passed in 0.25s
```

Compile check:

```text
python3 -m py_compile src/bioxp/oem_compat/transport.py src/bioxp/oem_compat/frames.py src/bioxp/oem_compat/api.py
```

## Claim boundary

This is still workstation-safe control-plane/transport compliance. It does not claim live robot motion or full OEM transport replacement yet. Live mode must still require operator acknowledgement, artifact roots, and no success on unmatched/ambiguous/non-success replies.
