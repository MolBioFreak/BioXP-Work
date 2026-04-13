# BioXP Target Architecture (2026-04)

Goal
- Reach OEM-equivalent core machine functionality while building a more declarative, API-first, extensible control stack than the original vendor application.

Design thesis
- Keep direct hardware transport deterministic and narrow.
- Move machine semantics into typed domain and service layers.
- Compile both OEM XML and native YAML/JSON protocols into one normalized execution model.
- Keep BMS as the operator/control plane, not the source of machine truth.

## Layered target model

## Layer A — Hardware adapters

Primary files
- `src/bioxp/usb_driver.py`
- `src/bioxp/can_driver.py`
- `scripts/decode_tmcl_frame.py`

Responsibilities
- USB/TMCL transport
- board activation/status
- motion parameter reads/writes
- thermal/chiller primitive control
- pipette transport hooks
- camera transport hooks
- low-level recovery/pacing

Rules
- No protocol/job semantics here.
- No deck/location abstractions here.
- No frontend-specific logic here.

## Layer B — Typed domain model

Target package
- `src/bioxp/domain/`

Responsibilities
- capabilities
- deck and semantic locations
- labware and consumables
- typed motion/pipette/thermal/vision request/response models
- normalized errors and state enums

Rules
- This layer defines nouns and states, not hardware I/O.

## Layer C — Machine services

Target package
- `src/bioxp/services/`

Responsibilities
- initialization/startup state
- safe motion prep and reference flows
- semantic movement resolution
- pipette orchestration
- thermal/chiller recipes
- vision/barcode operations
- artifact capture
- machine state transitions

Rules
- Route handlers should be thin wrappers over services.
- Safety policy lives here, not in UI code.

## Layer D — Protocol engine

Target package
- `src/bioxp/protocols/`

Responsibilities
- normalized action schema
- validators and compiler
- executor/runtime state
- pause/resume/retry/review gates
- OEM XML importer
- native YAML/JSON importer

Rules
- OEM XML is an input format, not the internal execution model.
- Simulation mode should use the same normalized action graph.

## Layer E — Operator/API surfaces

Primary files
- `src/bioxp/api.py`
- `/home/dalab/biomodstack/biomodstack/platform/api/routers/bioxp.py`
- `/home/dalab/biomodstack/biomodstack/platform/frontend/src/lib/bioxpClient.ts`
- `/home/dalab/biomodstack/biomodstack/platform/frontend/src/components/BioXpCockpit.tsx`

Responsibilities
- expose typed actions and protocol controls
- present operator review and artifact panels
- separate debug primitives from safe operator workflows

Rules
- BMS should never be the only place where machine semantics exist.
- Browser UI is a client, not the machine brain.

## Normalized execution flow

1. Operator or API client submits a typed semantic action or protocol.
2. API layer validates input and calls the correct service or protocol compiler.
3. Service/protocol layer resolves semantic intent into machine actions.
4. Hardware adapter executes deterministic low-level commands.
5. Runtime captures state, logs, and artifacts.
6. API/BMS presents controller truth, artifact links, and review state without pretending that raw telemetry equals physical proof.

## Invariants

1. Board activation may not be silently skipped on normal motion flows.
2. Controller-reported movement is not the same thing as physical proof.
3. Raw primitive endpoints may exist, but typed semantic services are the preferred path.
4. Complex workflows must launch from validated templates or normalized protocol models.
5. Capability flags, not code forks, should determine which workflows are available on a machine.

## Migration plan

### Step 1 — Stabilize truth and motion safety
- harden prep policy
- expose truth metadata
- start artifact capture

### Step 2 — Add typed semantics
- create domain models for deck/location/labware/capabilities
- extract motion/reference services

### Step 3 — Add missing core subsystems
- pipette service layer
- vision/barcode service layer

### Step 4 — Add workflow execution
- normalized protocol model
- executor
- OEM XML importer
- native YAML/JSON support

### Step 5 — Upgrade operator surface
- protocol runner
- artifact browser
- review/pause/resume controls

## Repo-boundary guidance

`bioxp_re`
- owns hardware control, typed semantics, and protocol/runtime logic

BMS backend/frontend
- owns proxying, job/control-plane integration, and operator presentation
- should remain thin wherever possible

## Success condition

The target architecture succeeds when:
- core machine behavior is reliable and artifact-backed
- semantic deck/pipette/thermal/vision actions exist as typed services
- OEM XML and native workflows run through the same executor
- BMS can operate the system without becoming the place where machine logic is duplicated
