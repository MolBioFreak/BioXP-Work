# BioXP3200 OEM Runtime Parity Completion Spec — 2026-05-03

## Scope
This spec turns the remaining source-system RE path into explicit implementation gates inside the Linux-native OEM compatibility control plane:

`ClassBioXPScriptHandler -> ClassVirtualBioXP -> ControlLib job lifecycle -> ClassPipetteCollection/ClassPipette -> CVisionLib inspection -> shadow/live proof`

The goal is not WPF UI parity. The goal is source-shaped runtime behavior with fail-closed evidence before any physical execution.

## Non-negotiable constraints
- Dry-run remains default and must never open USB or claim physical motion.
- Shadow mode may query/status only; it must not actuate.
- Live mode must require operator acknowledgement, artifact root, zero unsupported source coverage, source anchors, startup/reference proof, pipette readback proof, and vision proof or an explicit protocol gate.
- Hardware is treated as functional under OEM control; Linux failures are parity/transport/recovery issues unless isolated proof says otherwise.

## Layer 1 — ClassBioXPScriptHandler parity
Implement a source-shaped script execution plan layer that converts imported OEM protocol actions into ordered execution steps.

Required behavior:
- Preserve OEM verb, raw command, source node, action kind, and source position.
- Classify each action into an OEM-equivalent layer target:
  - `script_handler`
  - `virtual_bioxp`
  - `control_lib`
  - `pipette_collection`
  - `vision_facade`
  - `operator_gate`
- Expand pipette/macro actions into preflight + planned device operations instead of only recording generic state.
- Emit explicit blockers for live/shadow execution if an action still requires unavailable physical semantics.

Target files:
- `src/bioxp/oem_compat/script_handler.py`
- `src/bioxp/oem_compat/control_lib.py`
- `tests/test_oem_runtime_parity_pipeline.py`

## Layer 2 — ClassVirtualBioXP parity
VirtualBioXP is the deterministic semantic state ledger. It must keep deck/material state and attach state deltas to execution steps.

Required behavior:
- Track materials, zones, volumes, plate/covers, thermal/chiller settings, tip state, vision requirements, operator gates.
- Apply every imported action in dry-run.
- Never mark physical motion true.

Target files:
- `src/bioxp/oem_compat/state.py`
- `tests/test_oem_runtime_parity_pipeline.py`

## Layer 3 — ControlLib job lifecycle parity
ControlLib coordinates the runtime lifecycle and enforces compliance gates.

Required behavior:
- Lifecycle must include: `created`, `preflighted`, `script_interpreted`, `planned`, `state_applied`, `proof_gated`, `complete` for successful dry-runs.
- Preflight fail-closed on unsupported coverage, unsupported summaries, missing OEM source anchors.
- Dry-run result must include execution plan, virtual state, proof ladder, artifact, and no-USB/no-motion safety flags.

Target files:
- `src/bioxp/oem_compat/control_lib.py`
- `tests/test_oem_runtime_parity_pipeline.py`

## Layer 4 — Pipette collection/device behavior
The dry-run pipette layer must represent OEM ClassPipetteCollection/ClassPipette contracts without fake success.

Required behavior:
- For pipette actions, generate command plans with:
  - required reference axes
  - required tip state
  - required deck state
  - required pressure state
  - ack/readback policy
  - source/destination/material/volume metadata
- Live execution remains blocked until ACK/readback is implemented and validated.

Target files:
- `src/bioxp/oem_compat/pipette.py`
- `src/bioxp/oem_compat/script_handler.py`
- `tests/test_oem_runtime_parity_pipeline.py`

## Layer 5 — Vision/inspection behavior
Vision must be explicit, source-shaped, and fail-closed. Camera availability is not equivalent to OEM inspection parity.

Required behavior:
- Vision actions must create artifact contracts for image/raw frame/confidence/result preservation.
- Unsupported live vision should block live execution, not silently pass.
- Dry-run may record a required vision proof gate.

Target files:
- `src/bioxp/oem_compat/vision.py`
- `src/bioxp/oem_compat/script_handler.py`
- `tests/test_oem_runtime_parity_pipeline.py`

## Layer 6 — Shadow/live proof ladder
Before live mode, the runtime must produce a proof ladder and fail closed unless all preconditions are satisfied.

Required proof ladder fields:
- `mode`
- `dry_run_clean`
- `source_anchored`
- `zero_unsupported_commands`
- `transport_reply_contract_ready`
- `virtual_state_complete`
- `pipette_ack_readback_ready`
- `vision_artifact_contract_ready`
- `operator_ack_required`
- `artifact_root_required`
- `live_allowed`
- `blockers`

Initial implementation acceptance:
- Dry-run over `lifetest.xml` returns a full plan with script-handler, virtual-state, pipette, vision/operator/proof classification and no live permission.
- Shadow/live remain blocked until robot validation artifacts exist.
- Full test suite remains green.

## Verification commands
- `python3 -m pytest tests/test_oem_runtime_parity_pipeline.py -q`
- `python3 -m pytest tests/test_oem_initial_compliance.py tests/test_oem_virtual_job_compliance.py tests/test_oem_runtime_parity_pipeline.py tests/test_oem_compat_api.py -q`
- `python3 -m pytest tests -q`
