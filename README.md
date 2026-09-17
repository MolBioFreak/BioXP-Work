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

The current reliability picture should not be framed as a proven bad-component verdict. The robot is treated as functional under OEM control; current native-Linux work is about completing OEM-compatible control semantics, transport ownership, and BMS/runtime truth alignment.

Historical Linux-side evidence included camera/UVC control-query failures and suspicious USB reset behavior around the Novo adapter. That evidence remains useful for transport/recovery diagnostics, but the current documentation posture is:

- do not present the robot as generally broken hardware
- separate native-Linux implementation gaps from OEM-proven mechanical/electrical function
- describe Linux issues as transport/recovery/OEM-parity work unless a new, isolated live test proves a component fault
- keep BMS as a thin operator/proxy surface; robot-local runtime and artifacts own hardware truth

The runtime still performs reconnect and recovery work such as:

- `set_configuration()`
- `claim_interface()`
- reconnect flows that can call `dev.reset()`
- camera recovery paths that can release owners or force USB re-enumeration

So repeated resets, reconnect churn, or camera-control failures may reflect transport instability, recovery behavior, permission/driver issues, or other Linux-stack parity gaps. They should not be summarized as blanket hardware failure unless a new isolated test proves the specific component fault while controlling for software ownership/recovery effects.

## Running the stack

From the project root, interactive operator/runtime truth is:

```bash
sudo .venv/bin/python src/bioxp/usb_driver.py
```

Legacy standalone 24V diagnostic:

```bash
sudo .venv/bin/python src/bioxp/diagnostic_24v.py
```

The robot-local HTTP API has one owner. Do not launch `uvicorn`, a generic
container runner, a watchdog fallback, or a user recovery unit. Install the
immutable packet described in `release/README.md`, then control only the
canonical unit:

```bash
scripts/bioxp_handlerctl.py status
scripts/bioxp_handlerctl.py start
scripts/bioxp_handlerctl.py restart
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
