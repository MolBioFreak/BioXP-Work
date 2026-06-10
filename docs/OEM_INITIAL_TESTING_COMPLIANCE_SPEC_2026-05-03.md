# BioXP3200 OEM Initial Testing / Compliance Spec — 2026-05-03

> Scope: Linux-native `bioxp-oem-compat` control-system parity work in this repository. BMS is intentionally out of scope except as a later thin operator surface.

## Goal

Reach the first defensible robot-testing gate: a workstation-proven, source-anchored OEM compatibility runtime that can consume the current OEM script corpus without unsupported commands, maintain deterministic dry-run semantic state, emit trace/replay artifacts, and fail closed anywhere OEM behavior is not yet implemented.

This is not a claim of proven live 100% OEM equivalence. It is the minimum theoretical compliance gate before live testing is honest.

## Definitions

- **OEM-native compliance:** Linux-native behavior source/trace-anchored to decompiled OEM code, SSD XML/config assets, and captured/oracle traces. It does not require WPF UI parity.
- **Initial testing compliance:** the first robot-testable milestone: the selected OEM corpus parses and dry-runs through the compatibility runtime with no unsupported commands, no placeholder-success paths, explicit source anchors, and complete artifacts.
- **Good-faith testing:** live/shadow robot testing where the code under test is the intended OEM-equivalent control path, not an acknowledged scaffold pretending to be complete.

## Non-negotiable gates before live motion/liquid testing

1. Dry-run remains default and never opens USB.
2. Shadow mode is query/status-only and blocks mutating commands.
3. Live mode requires `operator_ack` and an artifact root.
4. Script corpus coverage for the selected OEM XML set is 100%: zero unsupported commands.
5. Every supported script macro produces structured semantic metadata and either deterministic state transitions or an explicit fail-closed protocol gate.
6. `ClassVirtualBioXP` equivalent state exists for materials, wells, zones, tips, gantry, plates, and volume accounting.
7. `ClassBioXPScriptHandler` equivalent parser/expander exists for the selected corpus.
8. `ControlLib.scriptInterpretor` equivalent command dispatch exists for the expanded command set used by the selected corpus.
9. Pipette commands have ACK/readback/state contracts, not TX-only success.
10. Vision/barcode/inspection commands are implemented or explicitly excluded from the selected first test protocol.
11. Trace artifacts include frames, semantic actions, state transitions, expected replies, source anchors, and replay validation.
12. A source coverage matrix classifies primary OEM methods as implemented, intentionally excluded, blocked, or missing.

## Initial code phases

### Phase 0 — Compliance gates and corpus truth

Files:
- `tests/test_oem_initial_compliance.py`
- `src/bioxp/protocols/oem_xml_import.py`
- `src/bioxp/oem_compat/scripts.py`
- `docs/OEM_INITIAL_TESTING_COMPLIANCE_SPEC_2026-05-03.md`

Acceptance:
- The real SSD script corpus under `BioXP_SSD_Backup/Scripts` imports with `command_nodes_total == 2933` and `unsupported_command_count == 0`.
- The six currently missing macros (`RT`, `SA`, `ST`, `SW`, `TT`, `ZW`) are parsed into structured metadata.
- Existing fixture tests are updated so lifetest is no longer expected to report unsupported commands.

### Phase 1 — VirtualBioXP state kernel

Files to add:
- `src/bioxp/oem_compat/virtual_bioxp.py`
- `src/bioxp/oem_compat/material_state.py`
- `src/bioxp/oem_compat/well_state.py`
- `src/bioxp/oem_compat/tip_state.py`

Acceptance:
- Materials can be loaded from OEM inventory sections.
- Zones resolve to deterministic well sets.
- Source/destination well searches are deterministic.
- Fluid levels update/reduce and cannot go negative silently.
- Tip state tracks T50/T200 load/eject/keep semantics.

### Phase 2 — ScriptHandler parser/expander

Files to add:
- `src/bioxp/oem_compat/script_handler.py`
- `src/bioxp/oem_compat/script_commands.py`
- `src/bioxp/oem_compat/script_expander.py`

Acceptance:
- OEM macros expand into lower-level semantic commands matching `ClassBioXPScriptHandler` for the selected corpus.
- Option groups parse: AP/AO, DP/DAO, MO, DB/DBO, source/destination selectors, volume, tips, zones, repeat/wash counts.
- Loop/dwell/delaypoint semantics are represented explicitly.

### Phase 3 — ControlLib command dispatcher

Files to add:
- `src/bioxp/oem_compat/script_interpreter.py`
- `src/bioxp/oem_compat/control_runtime.py`
- `src/bioxp/oem_compat/job_lifecycle.py`

Acceptance:
- Expanded commands dispatch to motion/pipette/thermal/vision facades.
- Pause/resume/abort/cleanup are modeled with fail-closed state transitions.
- Dry-run artifacts show command-by-command state transitions.

### Phase 4 — Pipette protocol parity

Files to add:
- `src/bioxp/oem_compat/pipette_channel.py`
- `src/bioxp/oem_compat/pipette_collection.py`
- `src/bioxp/oem_compat/pipette_protocol.py`
- `src/bioxp/oem_compat/pipette_errors.py`

Acceptance:
- ClassPipette/ClassPipetteCollection operations have per-channel state, command formatting, ACK/readback expectations, error parsing, pressure query/stream contracts, and completion waits.
- Aspirate/dispense/mix/load/eject/tip-query/fluid-detect paths cannot return fake success.

### Phase 5 — High-level motion/deck/thermal/vision gates

Files to add/extend:
- `src/bioxp/oem_compat/motion_controller.py`
- `src/bioxp/oem_compat/plate_handler.py`
- `src/bioxp/oem_compat/gripper.py`
- `src/bioxp/oem_compat/thermal.py`
- `src/bioxp/oem_compat/chiller.py`
- `src/bioxp/oem_compat/inspection.py`

Acceptance:
- `moveTo`, `scriptmoveTo`, G/Z coupled moves, door, catch/release/press/cut operations are source-mapped.
- Thermal/chiller calls used by the selected corpus have payload/reply contracts.
- Vision-dependent commands either produce real artifacts or block the selected protocol.

## First implementation slice

Start with Phase 0 using strict TDD:
1. Add failing tests that require full OEM corpus coverage and parsed metadata for RT/SA/ST/SW/TT/ZW.
2. Implement minimal macro parser support in `oem_xml_import.py` and `scripts.py`.
3. Update existing tests that expected lifetest unsupported counts.
4. Run targeted OEM tests.

## Initial testing signoff artifact

Before any live run, produce an artifact directory containing:
- imported protocol payloads for all selected scripts
- aggregate coverage report
- expanded dry-run semantic command list
- initial/final virtual machine state
- frame/reply plan
- replay result
- source coverage matrix

Suggested root:
`/mnt/BioModStack/bms_results/bioxp_validation/<timestamp>/`
