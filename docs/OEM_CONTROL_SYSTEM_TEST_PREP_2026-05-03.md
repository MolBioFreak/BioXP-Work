# BioXP3200 OEM Control-System Test Prep — 2026-05-03

## Purpose

This handoff records workstation-side prep completed for end-to-end native BioXP3200 control-system testing. The robot hardware is treated as functional; this work is about Linux/OEM semantic parity and BMS/native control readiness.

## Phase2 prep closed for testing

Added a source-anchored motion profile matrix exposed by `src/bioxp/oem_compat/boards.py::axis_profile_matrix()`.

The matrix covers:
- axis board/motor assignment for X/Y/Z/gripper/door
- speed/acc/current/stall/switch-mask parameters
- startup-home speeds
- manual-home speeds
- gripper version-one home speed
- thermal-door `doorSearchHome` method identity
- source anchor: `BioXPControlLib/ClassControlInterface.cs initializeMotors, initializeMotorsWithoutMotion, btnHome*_Click`

Existing Phase2 tests still verify startup/homing order, API OEM-home routing, prepared-reuse hardening, and supervised helper operator gates.

## Phase3 prep implemented

Added `src/bioxp/oem_compat/position_table.py` with:
- `PositionTable.from_rows(...)`
- `PositionTable.resolve(...)`
- `PositionTable.compile_move_to(...)`
- `PositionTarget` payloads with base coordinates, offsets, final coordinates, and source anchors

This gives the next test run a semantic deck/location target resolver instead of raw coordinate guessing.

## Phase4 prep advanced

Expanded OEM XML/script semantic coverage for testing-critical liquid commands:
- `MT` now compiles as an OEM liquid-transfer semantic action
- `FP` now compiles as an OEM fluid-prep semantic action
- parsed metadata includes source/destination location IDs, well IDs, volume, raw tokens, and `requires_ack_readback=true`

`lifetest.xml` coverage improved:
- supported command count: 95 -> 160
- unsupported command count: 83 -> 18
- `MT`: 62/62 now supported
- `FP`: 3/3 now supported

Remaining unsupported lifetest verbs are now the smaller tail for subsequent exact semantic work.

## Phase5 prep advanced

`src/bioxp/oem_compat/pipette.py` plans now carry explicit testing contracts:
- `ack_policy=ack_and_status_readback_required`
- `requires_readback=true`
- `preflight.requires_reference_axes=[x,y,z]`
- tip/deck/pressure state requirements
- `fail_closed_without_readback=true`

Dry-run plans still never report hardware execution.

## Phase6 prep advanced

`src/bioxp/oem_compat/vision.py` dry-run results now include artifacted failure contracts:
- image required for barcode/inspection style semantic checks
- failure is semantic, not merely transport-level
- confidence/raw-frame preservation requirements
- location ID passthrough for test artifacts

## Verification

Targeted prep suite:

```text
python3 -m pytest tests/test_oem_xml_import.py tests/test_oem_compat_phases.py tests/test_oem_compat_end_to_end.py tests/test_oem_control_system_prep.py -q
23 passed in 0.05s
```

Broad workstation suite excluding the known direct-pyusb collection test:

```text
python3 -m pytest tests -q --ignore=tests/test_usb_reconnect.py
153 passed in 1.99s
```

Compile check:

```text
python3 -m py_compile \
  src/bioxp/oem_compat/boards.py \
  src/bioxp/oem_compat/position_table.py \
  src/bioxp/oem_compat/scripts.py \
  src/bioxp/oem_compat/pipette.py \
  src/bioxp/oem_compat/vision.py \
  src/bioxp/protocols/oem_xml_import.py
```

Known full-suite caveat:
- `python3 -m pytest tests -q` still collects `tests/test_usb_reconnect.py`, which imports `usb.core` directly and fails on workstations without `pyusb` installed. This is environmental, not from this slice.

## Claim boundary

This is testing prep, not a claim that the full native Linux control system is live-complete. The important prep gaps closed here are semantic dry-run coverage, source-anchored motion profile reporting, PositionTable/moveTo scaffolding, MT/FP lifetest compilation, pipette ACK/readback contracts, and artifacted vision failure semantics.
