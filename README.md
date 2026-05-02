# BioXP 3200 Linux Control Stack

Direct Linux control and reverse engineering of a BioXP 3200 via the Atmel Novo USB-to-CAN adapter.

## Canonical runtime

The live control contract in this repo is:

- `src/bioxp/usb_driver.py` — direct USB/TMCL runtime and operator truth
- `src/bioxp/api.py` — robot-local HTTP wrapper used by BMS and other callers

When the docs and older reverse-engineering notes disagree, treat the live Python runtime and API as authoritative.

## Operating model

The current intended deployment model is:

- the robot owns the BioXP runtime locally
- external systems such as BMS link to and proxy the robot-local HTTP API
- BMS is not the normal place to start or stop the robot daemon
- controller telemetry is useful, but it is not independent proof of physical motion

## Current runtime surface

The current stack exposes meaningful live surfaces for:

- USB/TMCL transport through the Novo adapter
- board activation, reconnect, and status flows
- latch and LED control
- motion status plus relative, absolute, and home actions
- motion arm / interlock / reference-state endpoints
- thermal and chiller board control
- camera device, control, snapshot, stream, and recovery paths
- vision inspection and barcode helper endpoints
- liquid-handling service endpoints (`/liquid/*`)
- protocol compile / execute / review endpoints (`/protocol/*`)

There is also a growing service/test surface under:

- `src/bioxp/services/`
- `tests/`

That said, API existence does not automatically mean OEM-equivalent or physically proven behavior.

## Practical status

The stack is no longer just a raw reverse-engineering sandbox, but it is also not close to full OEM parity.

Current grounding from the repo docs and live runtime surfaces:

- practical usable OEM parity is still roughly in the 15–25% range when physical proof and operator safety are counted
- low-level primitive coverage is much better than that, roughly in the 40–50% range
- the biggest remaining gaps are still:
  - semantic deck / location / well modeling
  - stronger pipette and liquid-handling parity
  - richer vision / inspection semantics
  - protocol/job semantics that are validated on real hardware
  - better transport observability, trace capture, and proof artifacts

## Reliability note

The current reliability picture is still unresolved. Live evidence currently includes repeatable camera/UVC control-query failures plus historically suspicious USB reset behavior around the Novo adapter, but this repo does not treat that as a closed "bad componentry" verdict.

The current runtime intentionally performs reconnect and recovery work such as:

- `set_configuration()`
- `claim_interface()`
- reconnect flows that can call `dev.reset()`
- camera recovery paths that can release owners or force USB re-enumeration

So repeated resets, reconnect churn, or camera-control failures may reflect some combination of transport instability, recovery behavior, permission/driver issues, and hardware weakness. Novo USB/CAN remains a serious suspect, but software reconnect/reset behavior is still a major confounder. Until those signals are cleanly separated and correlated with physical inspection, describe the problem as unresolved transport/recovery instability rather than proven blanket hardware failure.

## Running the stack

From the project root, interactive operator/runtime truth is:

```bash
sudo .venv/bin/python src/bioxp/usb_driver.py
```

Legacy standalone 24V diagnostic:

```bash
sudo .venv/bin/python src/bioxp/diagnostic_24v.py
```

Typical robot-local HTTP API launch:

```bash
PYTHONPATH=src .venv/bin/uvicorn bioxp.api:app --host 0.0.0.0 --port 8123
```

## Documentation map

Start with:

- `docs/README.md`

Most useful current docs:

- `docs/ARCHITECTURE_AND_CONTROL_PLANE.md`
- `docs/SUBSYSTEMS_AND_OPERATIONS.md`
- `docs/RUNBOOK.md`
- `docs/LIVE_HOMING_RUNBOOK_2026-04-13.md`
- `docs/VENDOR_PARITY_SCORECARD.md`
- `docs/MOTION_RELIABILITY_ROADMAP_2026-04-12.md`
- `docs/DECODE_AND_GAP_ASSESSMENT_2026-04-12.md`
- `docs/MODULE_REFERENCE.md`
- `docs/REVERSE_ENGINEERING_TRACEABILITY.md`

Historical reverse-engineering notes and extracted string artifacts are still retained in `docs/`, but they should be treated as reference material rather than live behavioral truth.
