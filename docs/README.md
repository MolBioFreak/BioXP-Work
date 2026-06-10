# BioXP 3200 Documentation Hub

This directory is the centralized documentation surface for the current BioXP Linux control stack.

## Runtime truth

The primary live code surfaces are:

- `src/bioxp/usb_driver.py` — direct USB/TMCL runtime and operator truth
- `src/bioxp/api.py` — robot-local HTTP wrapper for BMS and other callers
- `src/bioxp/services/` — service-layer implementation for motion, artifacts, pipette, protocol, reference, and vision helpers
- `tests/` — current automated checks for service/runtime behavior

If an older reverse-engineering note conflicts with the live runtime, trust the current Python code first.

## Operating interpretation rules

Keep these rules in mind while using the docs in this folder:

1. The robot-local runtime is the intended owner of BioXP control.
2. BMS is a linkage/proxy client, not the normal runtime supervisor.
3. Controller telemetry and API success are not independent proof of physical displacement.
4. Repeatable camera/UVC control-query failures and historical Novo USB reset behavior are serious transport warnings, but not yet a proven blanket hardware-failure diagnosis.
5. Software reconnect/reset behavior is a major confounder when interpreting those failures.

## Documentation map

### Core runtime and operator docs

1. `ARCHITECTURE_AND_CONTROL_PLANE.md`
   - overall runtime model
   - USB/TMCL transport and recovery behavior
   - board topology and command surface
2. `SUBSYSTEMS_AND_OPERATIONS.md`
   - latch, LED, camera, motor, chiller, and thermal surfaces
   - subsystem-specific control and recovery paths
3. `RUNBOOK.md`
   - day-to-day operator usage
   - common recovery actions and menu flows
4. `LIVE_HOMING_RUNBOOK_2026-04-13.md`
   - supervised homing / re-reference session guidance
   - current live API base assumptions and proxy limitations

### Parity, reliability, and planning docs

5. `VENDOR_PARITY_SCORECARD.md`
   - practical parity framing versus raw primitive coverage
   - current blockers to operator-safe OEM-equivalent behavior
6. `MOTION_RELIABILITY_ROADMAP_2026-04-12.md`
   - motion-proof-first roadmap
   - controller-truth versus physical-truth framing
7. `DECODE_AND_GAP_ASSESSMENT_2026-04-12.md`
   - TMCL decoder status
   - OEM gap assessment and missing semantic layers
8. `REVERSE_ENGINEERING_TRACEABILITY.md`
   - mapping from OEM/decompiled evidence to Linux implementations
9. `MODULE_REFERENCE.md`
   - module-by-module reference for `src/bioxp/`

## Current vs historical material

The files below are still useful, but they are supporting reference artifacts rather than the live product contract:

- `reverse_engineering_report.md`
- `bioxp_ssd_report.md`
- `RCA_2025-03-01.md`
- `FIX_PLAN_2025-03-01.md`
- `PROGRESS_2026-03-02.md`
- `*_strings.txt` extraction files

Use them for provenance, DLL/string evidence, and older reasoning trails. Use the runtime docs above for current behavior.
