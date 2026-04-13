# BioXP Vendor-Parity Execution Backlog (2026-04-12)

This backlog converts the strategic roadmap into concrete PR-sized chunks.

Working rules:
- Keep PRs narrow enough to review in one sitting.
- Do not mix low-level reliability fixes with broad feature additions.
- Every PR must either increase truth, reduce ambiguity, or unlock the next semantic layer.
- BMS should stay a thin operator surface; machine semantics belong in `bioxp_re`.

## PR-01 — Program framing and parity baselines

Objective
- Create the scorecard, flexibility requirements, and target architecture docs so every later change has a named acceptance target.

Files
- Create: `docs/VENDOR_PARITY_SCORECARD.md`
- Create: `docs/FLEXIBILITY_REQUIREMENTS.md`
- Create: `docs/ARCHITECTURE_TARGET_2026-04.md`
- Modify: `docs/DECODE_AND_GAP_ASSESSMENT_2026-04-12.md`

Validation
- Docs render cleanly.
- Every major subsystem has a parity row and a next gate.

Dependencies
- none

## PR-02 — Motion prep policy hardening + truth metadata

Objective
- Remove silent prepared-fast-path behavior from normal moves and expose explicit truth semantics in motion responses.

Files
- Modify: `src/bioxp/api.py`
- Create: `tests/test_motion_phase1.py`
- Modify: `/home/dalab/biomodstack/biomodstack/platform/frontend/src/lib/bioxpClient.ts`
- Modify: `/home/dalab/biomodstack/biomodstack/platform/frontend/src/components/BioXpCockpit.tsx`

Deliverables
- `reuse_prepared` becomes debug-only compatibility behavior.
- Board activation is never skipped.
- Axis prep is only skipped when an explicit debug env flag is enabled and strict-arm/live state is healthy.
- Motion responses include `prep_policy` and `motion_truth`.
- Cockpit shows controller-only truth and prep-policy notes.

Validation
- `uv run --with fastapi --with pydantic --with starlette --with pyusb --with pytest python -m pytest tests/test_motion_phase1.py -q`
- `npm run build` in frontend

Dependencies
- PR-01 recommended but not required

## PR-03 — Proof-bundle artifact scaffolding

Objective
- Add the first artifact path and metadata shape for supervised motion validation bundles.

Files
- Create: `src/bioxp/services/artifact_service.py`
- Modify: `src/bioxp/api.py`
- Create: `tests/test_motion_artifacts.py`
- Modify: `/home/dalab/biomodstack/biomodstack/platform/frontend/src/components/BioXpCockpit.tsx`

Deliverables
- Canonical artifact root under `/mnt/BioModStack/bms_results/bioxp_validation/`
- bundle metadata schema
- placeholder operator note support
- save request/response JSON plus snapshot references

Validation
- backend tests for bundle directory creation and metadata serialization
- one supervised dry-run bundle created without hardware writes

Dependencies
- PR-02

## PR-04 — Transaction-aware TMCL decoding

Objective
- Upgrade the decoder from single-frame interpretation to request/reply transaction decode.

Files
- Modify: `scripts/decode_tmcl_frame.py`
- Create: `scripts/capture_tmcl_session.py`
- Create: `tests/test_tmcl_decode.py`
- Create: `testdata/tmcl/README.md`
- Create: `testdata/tmcl/fixtures/`

Deliverables
- request/reply correlation mode
- transaction JSON output
- starter trace-corpus structure

Validation
- fixture-based decoder tests
- manual decode of one captured transaction log

Dependencies
- none

## PR-05 — Domain skeleton and semantic deck model

Objective
- Create the typed domain layer that breaks the runtime out of raw axis-only control.

Files
- Create: `src/bioxp/domain/__init__.py`
- Create: `src/bioxp/domain/capabilities.py`
- Create: `src/bioxp/domain/deck.py`
- Create: `src/bioxp/domain/locations.py`
- Create: `src/bioxp/domain/labware.py`
- Create: `tests/test_deck_model.py`
- Create: `config/deck/default_layout.yaml`

Deliverables
- semantic location IDs
- deck/labware config loader
- capability registry skeleton

Validation
- domain tests for location resolution and config parsing

Dependencies
- PR-01

## PR-06 — Motion/reference services

Objective
- Pull motion/reference behavior out of route handlers and into reusable service functions.

Files
- Create: `src/bioxp/services/__init__.py`
- Create: `src/bioxp/services/motion_service.py`
- Create: `src/bioxp/services/reference_service.py`
- Modify: `src/bioxp/api.py`
- Create: `tests/test_motion_service.py`
- Create: `tests/test_reference_service.py`

Deliverables
- typed service calls for relative/absolute/home/reference
- explicit desync/reference state model
- safer route wrappers with thinner control flow

Validation
- backend service tests
- route smoke tests with fakes

Dependencies
- PR-02, PR-05

## PR-07 — Pipette transport and typed liquid actions

Objective
- Replace `501` liquid endpoints with a real pipette abstraction boundary.

Files
- Modify: `src/bioxp/can_driver.py`
- Create: `src/bioxp/pipette/__init__.py`
- Create: `src/bioxp/pipette/transport.py`
- Create: `src/bioxp/pipette/models.py`
- Create: `src/bioxp/services/pipette_service.py`
- Modify: `src/bioxp/api.py`
- Create: `tests/test_pipette_service.py`

Deliverables
- pipette transport boundary
- init/status/tip/aspirate/dispense/mix APIs
- typed pipette errors

Validation
- fake-transport backend tests
- explicit coverage for aspirate/dispense request validation

Dependencies
- PR-05

## PR-08 — Vision/barcode service layer

Objective
- Convert camera transport into typed inspection and barcode operations.

Files
- Create: `src/bioxp/vision/__init__.py`
- Create: `src/bioxp/vision/barcode.py`
- Create: `src/bioxp/vision/inspection.py`
- Create: `src/bioxp/services/vision_service.py`
- Modify: `src/bioxp/api.py`
- Create: `tests/test_vision_service.py`

Deliverables
- barcode read surface
- snapshot-backed inspection result objects
- capability-gated vision operations

Validation
- unit tests with canned image metadata / mocked capture outputs

Dependencies
- PR-03, PR-05

## PR-09 — Protocol model and executor skeleton

Objective
- Introduce the normalized action model that both OEM XML and native workflows will compile into.

Files
- Create: `src/bioxp/protocols/__init__.py`
- Create: `src/bioxp/protocols/models.py`
- Create: `src/bioxp/protocols/compiler.py`
- Create: `src/bioxp/protocols/executor.py`
- Create: `src/bioxp/protocols/runtime_state.py`
- Create: `src/bioxp/protocols/validators.py`
- Create: `tests/test_protocol_compiler.py`
- Create: `tests/test_protocol_executor.py`

Deliverables
- normalized action schema
- dry-run executor mode
- stage state and pause/review markers

Validation
- compile and execute one tiny native protocol in simulation mode

Dependencies
- PR-05, PR-06

## PR-10 — OEM XML importer first slice

Objective
- Map OEM script constructs into the normalized protocol model with explicit coverage accounting.

Files
- Create: `src/bioxp/protocols/oem_xml_import.py`
- Create: `tests/test_oem_xml_import.py`
- Create: `testdata/oem_xml/demo.xml`
- Create: `testdata/oem_xml/lifetest.xml`
- Modify: `docs/VENDOR_PARITY_SCORECARD.md`

Deliverables
- importer for a first supported subset
- unsupported-verb reporting
- source-node traceability in compiled output

Validation
- fixture imports of `demo.xml` and `lifetest.xml`
- coverage report generated from test fixtures

Dependencies
- PR-09

## PR-11 — BMS semantic operator surface

Objective
- Upgrade BMS from primitive control panel to semantic protocol/artifact operator UI.

Files
- Modify: `/home/dalab/biomodstack/biomodstack/platform/api/routers/bioxp.py`
- Modify: `/home/dalab/biomodstack/biomodstack/platform/frontend/src/lib/bioxpClient.ts`
- Modify: `/home/dalab/biomodstack/biomodstack/platform/frontend/src/components/BioXpCockpit.tsx`
- Create: `/home/dalab/biomodstack/biomodstack/platform/frontend/src/components/BioXpProtocolRunner.tsx`
- Create: `/home/dalab/biomodstack/biomodstack/platform/frontend/src/components/BioXpArtifactsPanel.tsx`
- Create: `/home/dalab/biomodstack/biomodstack/platform/api/tests/test_bioxp_router.py`
- Create: `/home/dalab/biomodstack/biomodstack/platform/api/tests/test_bioxp_protocol_jobs.py`

Deliverables
- protocol submit/status/review/resume UI
- artifact list panel
- debug-vs-operator separation in cockpit

Validation
- BMS backend tests
- frontend build

Dependencies
- PR-03, PR-09

## PR-12 — Release gates and parity validation

Objective
- Turn the scorecard into a release-readiness surface backed by evidence.

Files
- Modify: `docs/VENDOR_PARITY_SCORECARD.md`
- Create: `docs/VALIDATION_PROTOCOLS.md`
- Create: `docs/RELEASE_CRITERIA.md`
- Create: `docs/SERVICE_WORKFLOWS.md`

Deliverables
- named validation tracks for motion, pipette, thermal, vision, and protocol execution
- explicit release gates
- documented unsupported fringe features

Validation
- scorecard updated with real evidence links
- release criteria checklist reviewed after each major subsystem lands

Dependencies
- PR-06 through PR-11

## Recommended execution order

1. PR-01
2. PR-02
3. PR-03
4. PR-04
5. PR-05
6. PR-06
7. PR-07
8. PR-08
9. PR-09
10. PR-10
11. PR-11
12. PR-12

## Immediate next merge targets

If work starts right now, merge in this order:
- PR-01: framing docs
- PR-02: motion prep policy + truth metadata
- PR-03: proof-bundle scaffolding

That preserves the right sequence:
- define parity
- harden truth
- start artifact capture
- then expand semantics and subsystems
