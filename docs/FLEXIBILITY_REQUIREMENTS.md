# BioXP Flexibility Requirements

Date: 2026-04-12

Goal
- Define what “utter flexibility” means for the Linux/BMS BioXP stack without collapsing into an untyped, unsafe free-for-all.

Core principle
- Flexibility must come from typed schemas, capability flags, and compilation into normalized actions.
- It must not come from freeform raw parameter dictionaries or UI-only machine logic.

## Requirement groups

## A. Machine capability and deployment flexibility

FR-01 — Capability-gated subsystems
- The runtime must describe which capabilities a given machine/profile supports.
- Examples: motion, pipette, barcode, thermal, chiller, inspection, protocol execution.
- Missing capabilities must degrade gracefully and visibly.

FR-02 — Machine profile configuration
- A machine profile must be loadable from data/config rather than hardcoded per deployment.
- Profile data should cover board/layout variants, subsystem availability, and calibration references.

FR-03 — Safe partial deployment
- The control plane must remain usable when only a subset of subsystems is ready.
- Example: motion + thermal may be available before pipette/vision.

## B. Deck, labware, and location flexibility

FR-04 — Data-driven deck layouts
- Deck slot, plate, rack, and location definitions must live in config files.
- Layout changes should not require source edits for common variants.

FR-05 — Semantic location addressing
- Workflows must target `location_id`, `well_id`, `slot`, or named semantic positions rather than raw axis coordinates.

FR-06 — Calibration-aware transforms
- Offsets and calibration transforms must be layered on top of semantic locations, not hand-coded in protocol steps.

## C. Protocol and workflow flexibility

FR-07 — Native protocol format
- The runtime must support a first-class YAML/JSON protocol format that is versioned and validated.

FR-08 — OEM XML as importer, not runtime core
- OEM XML must compile into the same normalized action model as native workflows.
- OEM XML support must not define the internal runtime abstraction.

FR-09 — Composable workflow model
- Protocols must support loops, subprotocols, review gates, waits, and reusable templates.

FR-10 — Dry-run/simulation mode
- Protocols must run in simulation mode without hardware side effects.
- Simulation must produce state/log/artifact output compatible with the real executor where possible.

## D. Operator and API flexibility

FR-11 — API-first launch surface
- Typed semantic actions and full protocols must be launchable through an API without requiring browser-first control.

FR-12 — Thin UI / thick runtime rule
- BMS may present and validate workflows, but machine semantics must live in the BioXP runtime/service layer.

FR-13 — Review/pause/resume/retry controls
- Long workflows must expose explicit execution-state controls rather than forcing users to clone or improvise jobs.

## E. Evidence and safety flexibility

FR-14 — Artifact-backed truth
- Every safety-critical or validation-relevant execution path must be able to emit artifacts.
- For motion, that includes request/response/state plus snapshot/operator evidence.

FR-15 — Debug-only fast paths
- Risky optimizations like prepared-path reuse must be compatibility/debug features guarded by explicit flags.
- They must never masquerade as the normal operator-safe path.

FR-16 — Controller truth versus physical truth labeling
- The system must always distinguish between controller-reported deltas and independently confirmed physical displacement.

## F. Extensibility requirements

FR-17 — Additive action families
- New action families should be addable to the normalized protocol model without rewriting the executor architecture.

FR-18 — Import/export traceability
- Imported OEM steps should retain source-location metadata so unsupported semantics are visible and auditable.

FR-19 — Backward-compatible evolution
- Schema changes should use versioned models and explicit migrations where needed.

## Non-requirements

The following are explicitly not required for flexibility:
- cloning the OEM GUI layout exactly
- preserving OEM XML as the only authoring format
- exposing raw board commands as the main end-user workflow surface
- embedding machine semantics in frontend components
- allowing arbitrary freeform JSON for complex protocol launches

## Design consequences

These requirements imply:
1. A typed domain layer for capabilities, deck, labware, motion, pipetting, thermal, and vision.
2. A service layer that owns machine semantics.
3. A normalized protocol compiler/executor model.
4. Artifact services and truth labeling as first-class features.
5. BMS acting as a thin operator/client surface over typed backend behavior.
