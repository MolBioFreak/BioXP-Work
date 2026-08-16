# BioXP Pipette OEM Parity Execution Plan

> **For Hermes:** Use `subagent-driven-development` and strict RED-GREEN slices. Preserve one implementation writer per worktree.

**Goal:** Repair the identified pipette parity defects through BioXP, shared Novo/CAN transport, application orchestration, API receipts, BioModStack, and the operator UI. Finish at read-only live acceptance.

**Architecture:** The robot remains the only command and state authority. `NovoRouter` remains the only USB endpoint reader and serialized CAN transaction owner. BioModStack remains a typed relay and operator surface. Physical pipette commands stay disabled for this tranche.

**Authorized repositories:**

- BioXP: `/home/dalab/worktrees/bioxp-pipette-parity-full-20260816`
- BioModStack: `/home/dalab/worktrees/bms-pipette-parity-full-20260816`
- Governing branches: `origin/test`

**Prohibited in this tranche:** tip ejection, tip loading, aspirate, dispense, mix, air operations, fluid detection, pressure-stream activation, initialization, group initiation, termination, gantry/Z/deck movement, dry commissioning, and wet commissioning on robot 206.

---

## Authority map

| Concern | Current owner | Required owner | Proof |
|---|---|---|---|
| OEM methods and wire literals | Lock-pinned Controlsuite source | Unchanged | Source hash and row anchor |
| USB endpoint and frame dispatch | `NovoRouter` | `NovoRouter` only | Ownership and listener evidence |
| Per-channel command formatting | `BioXpCanDriver` | `BioXpCanDriver` | Golden command vectors |
| Channel state | `CanPipetteTransport` | Controller-derived state plus explicit host shadow | Behavioral fixtures |
| Four-channel semantics | `FourPipetteTransport` | Source-shaped collection | Collection fixtures |
| Machine workflow | Serial-206 application provider | Robot application layer | No-motion planner and admission tests |
| Receipts | Robot receipt store | Controller-event-derived phases | Schema and provenance tests |
| BMS contract | BMS strict Pydantic models | Typed robot projection | API round trip |
| Operator workflow | BioXP cockpit | Dedicated four-channel pane | Mounted frontend tests and passive browser proof |

## Gate sequence

1. **Pre-flight:** exact clean worktree, remote ancestry, authority hashes, and physical-mutation prohibition.
2. **WP0:** complete denominator and acceptance matrix.
3. **WP1:** transport, response, and completion truth.
4. **WP2:** per-channel and collection semantics.
5. **WP3:** no-motion Controlsuite application wiring and physical workflow plans.
6. **WP4:** BioXP API, BMS contract, and operator UI.
7. **WP5:** focused regression and independent specification review.
8. **WP6:** commit, push, managed Development deployment, and passive live acceptance.
9. **Abort gate:** any observed robot mutation, transport TX outside an explicitly approved read-only query, competing endpoint reader, unowned dirty source, or ambiguous runtime ownership stops the tranche.

## Acceptance matrix

| ID | Identified issue | Required correction | Focused evidence | Passive live evidence | Physical gate |
|---|---|---|---|---|---|
| PIP-001 | Denominator claims zero unclassified rows while application call sites remain grouped | Generate row-level methods, properties, commands, state fields, and application call sites with explicit classifications | Matrix invariant tests compare generated source inventory with matrix rows | Published matrix and source hashes match deployed revision | Closed |
| PIP-002 | Speed and volume encoding are not proven across the OEM input domain | Use explicit OEM conversion helpers with bounded accepted domains; preserve literal strings in receipts | Decimal boundary golden vectors | No command sent; OpenAPI exposes the bounded domain | Closed |
| PIP-003 | Multipart equivalence is unresolved | Keep observed framing separate from unverified decompiler masks; reject unproven fragmented commands unless fixture authority exists | TX/RX order, malformed order, channel/function, and final-frame tests | Router status reports multipart authority state | Closed |
| PIP-004 | ACK can be inferred from TX-only `ok` | Add explicit evidence phase and require semantic pipette ACK data | TX-only negative test; immediate ACK and delayed completion tests | Read-only receipts never claim controller ACK without evidence | Closed |
| PIP-005 | Delayed completion is not bound to the admitted command | Bind completion owner to channel, command family, owner token, generation, and registration time | Late prior completion rejection; wrong-family and generation-change tests | Passive status exposes current owner and no stale completion | Closed |
| PIP-006 | Q1 and asynchronous error state are incomplete | Separate synchronous reply, queued error bytes, initialization counter, clear semantics, and callback event | Clean, error, clear, stale, malformed, and no-reply fixtures | Read-only Q1 receipts display exact phases | Closed |
| PIP-007 | Host precondition query can become postcondition proof | Rename and separate precondition, controller completion, post-readback, and physical-effect fields | Receipt truth-table tests | Dashboard labels each field separately | Closed |
| PIP-008 | `TipLocation` and default channel selection differ from OEM behavior | Make the collection consume one explicit reconciled tip location; require explicit channel sets at public APIs | No tip, one tip, kept tip, partial tips, explicit subset fixtures | Read-only status shows source and freshness for selection | Closed |
| PIP-009 | Speed, pressure, and diagnostics ignore OEM tip-presence rules | Filter the OEM-equivalent paths by controller-derived tip state while preserving explicit diagnostic refusal | Tip-present and tip-missing channel fixtures | Status shows per-channel eligibility without sending commands | Closed |
| PIP-010 | `getData()` is exposed as individual queries | Add the fixed aggregate OEM data sweep and preserve each subreceipt | Exact query order, result attribution, short/null parser tests | Read-only firmware/data sweep on all channels when admitted | Closed |
| PIP-011 | Group, eject, mix, air, fluid, and state semantics remain incomplete | Implement source-shaped collection behavior behind mutation admission; keep controller and host effects distinct | Fake-controller branch fixtures and rejected mutation tests | Mutation endpoints remain unavailable | Closed |
| PIP-012 | Physical tip load and fluid detection lack machine orchestration | Implement typed plans and coupled provider interfaces for gantry, Z, deck, waste, plate, and strip dependencies | Planner and fail-closed admission tests with zero transport calls | UI displays plans and blockers only | Closed |
| PIP-013 | Constructor, BoardTest, `initializeMotion`, and `initPipette` policies are merged or incomplete | Implement distinct call-site policies and retry ledgers without invoking them live | Distinct retry, early-return, cleanup, and callback-order fixtures | Read-only lifecycle reports `not_run` unless an owner executed it | Closed |
| PIP-014 | Compatibility artifacts may be unreachable | Route production callers through one canonical collection and retire or mark non-authoritative facades | Import/call-path tests | Source identity reports canonical class path | Closed |
| PIP-015 | BMS strict contract returns HTTP 502 | Replace pipette dictionaries with closed typed models that accept the exact robot payload and reject malformed truth | Robot payload round trip and hostile-schema tests | Catalog and dashboard return HTTP 200 | Closed |
| PIP-016 | Dedicated four-channel operator workflow is absent | Add a pipette tab with four channels, selection source, tip/init state, pressure/data reads, receipt phases, and disabled mutation controls | Mounted component tests and typed client tests | Real Development browser shows the pane and no console errors | Closed |
| PIP-017 | Live constructor/readback is inactive | Expose truthful lifecycle and a bounded read-only query path; do not auto-run initialization | Admission and no-TX status tests | Passive owner proof plus explicitly admitted query-only reads | Closed |
| PIP-018 | Physical acceptance is absent | Preserve explicit pending states for dry and wet acceptance | Mutation-denial regression | UI says physical acceptance is pending | Separate authorization required |

## WP0: Denominator closure

**Files:**

- Modify: `docs/specs/2026-08-02-pipette-oem-parity-matrix.json`
- Create: `scripts/build_pipette_oem_parity_matrix.py`
- Modify: `tests/test_pipette_wp0_wp4_contracts.py`

**RED:** Add a test that extracts current OEM and Linux rows and fails when any `ControlLib` pipette call site lacks an exact matrix row. Add an invariant that prohibits `unclassified_row_count == 0` when any row carries an open denominator statement.

**GREEN:** Generate deterministic row IDs for every authoritative method/property/call site and bind each to one status. Keep unresolved rows `blocked`.

**Gate:** No production runtime change begins until this test passes.

## WP1: Transport and response lifecycle

**Files:**

- Modify: `src/bioxp/novo_router.py`
- Modify: `src/bioxp/novo_usb_can.py`
- Modify: `src/bioxp/can_driver.py`
- Modify: `src/bioxp/pipette/transport.py`
- Modify: `src/bioxp/pipette/receipts.py`
- Modify: `tests/test_pipette_wp0_wp4_contracts.py`
- Modify: `tests/test_pipette_oem_continuation_contracts.py`
- Modify: `tests/test_pipette_can_shadow_queries.py`

**RED slices:**

1. TX-only success must leave `controller_acknowledged` false.
2. Immediate semantic ACK must set ACK true and completion false.
3. Delayed completion must match command owner and generation.
4. A late completion from an expired owner must be rejected.
5. Wrong channel, function, family, generation, or multipart phase must be rejected.
6. Precondition readback must never set postcondition or physical-effect truth.
7. Q1 synchronous and asynchronous state must remain separately inspectable.

**GREEN:** Add explicit `delivery`, `controller_ack`, `controller_completion`, `post_readback`, and `physical_effect` phases. Never derive one phase from another.

## WP2: Four-channel collection semantics

**Files:**

- Modify: `src/bioxp/pipette/models.py`
- Modify: `src/bioxp/pipette/transport.py`
- Modify: `src/bioxp/oem_pipette_collection.py`
- Modify: `src/bioxp/services/pipette_service.py`
- Modify: focused pipette tests

**RED slices:**

1. Default routing follows reconciled `TipLocation` and fails when ambiguous.
2. Speed, pressure, and diagnostics use controller-derived tip eligibility.
3. Aggregate data retrieval uses the fixed OEM query order.
4. Host air/fluid/tip state updates only after the specified controller phase.
5. Eject, group, mix, air, fluid, and terminate remain mutation-gated.

**GREEN:** Keep the canonical production collection in `FourPipetteTransport`. Compatibility classes must delegate to it or be marked non-production.

## WP3: No-motion application orchestration

**Files:**

- Modify: `src/bioxp/oem_serial206_initialization.py`
- Modify: `src/bioxp/operator_controls.py`
- Modify or create: robot application-level pipette orchestration module
- Modify: constructor and application-lifecycle tests

**RED slices:**

1. Constructor, BoardTest, diagnostic init, `initializeMotion`, and `initPipette` each retain distinct retry policies.
2. Physical tip-load and fluid-detection requests produce typed plans only while commissioning is closed.
3. Every plan requires exact gantry, Z, deck, tip, waste, strip, and ownership dependencies.
4. Planning and status paths make zero CAN writes and zero motion calls.

**GREEN:** Wire source-shaped planners and durable blocked-stage receipts. Do not auto-run or simulate physical success.

## WP4: API, BMS, and operator UI

**BioXP files:**

- Modify: `src/bioxp/api.py`
- Modify: `src/bioxp/operator_controls.py`
- Modify: `src/bioxp/pipette/receipts.py`
- Modify: route and catalog tests

**BMS files:**

- Modify: `platform/api/services/bioxp/operator_models.py`
- Modify: `platform/api/routers/bioxp/operator_controls.py`
- Modify: `platform/api/tests/test_bioxp_operator_controls.py`
- Modify: `platform/frontend/src/lib/bioxpClient.ts`
- Modify: `platform/frontend/src/components/BioXpOperatorControlTabs.tsx`
- Modify: `platform/frontend/src/components/BioXpQuickDashboard.tsx`
- Modify: `platform/frontend/tests/bioxpOperatorControlTabs.test.ts`
- Add mounted pipette tests under `platform/frontend/tests/vitest/`

**RED slices:**

1. The exact robot dashboard payload validates through BMS.
2. Malformed or phase-collapsed receipt payloads fail closed.
3. All four channels render with truth source, freshness, initialization, tip, pressure, error, and receipt phases.
4. Mutation controls remain present but disabled with the robot-owned blocker.
5. The browser sends only fixed BMS routes and typed inputs.

**GREEN:** Use closed models. Keep arbitrary raw CAN, paths, and runtime controls absent from the browser.

## WP5: Focused verification

**BioXP commands:**

```bash
PYTHONDONTWRITEBYTECODE=1 pytest -p no:cacheprovider \
  tests/test_pipette_wp0_wp4_contracts.py \
  tests/test_pipette_oem_continuation_contracts.py \
  tests/test_pipette_can_shadow_queries.py \
  tests/test_pipette_service.py \
  tests/test_oem_constructor_pipette_stage.py \
  tests/test_oem_startup_pipette_remediation.py -q
```

**BMS API command:**

```bash
cd platform/api
PYTHONDONTWRITEBYTECODE=1 uv run --frozen --group dev python -m pytest \
  tests/test_bioxp_operator_controls.py \
  tests/test_bioxp_robot_client.py \
  tests/test_bioxp_compact_api.py -q -p no:cacheprovider
```

**BMS frontend commands:**

```bash
cd platform/frontend
pnpm exec tsx --test tests/bioxpOperatorControlTabs.test.ts
pnpm exec vitest run --config vitest.md.config.ts tests/vitest/bioxpOperatorPipetteMounted.test.tsx
pnpm exec tsc -b --pretty false
```

Run `git diff --check` in both repositories after the last edit. An independent reviewer must compare the exact tree with this matrix before integration.

## WP6: Development release and passive live acceptance

1. Fetch current `origin/test` in both repositories.
2. Rebase the isolated branches if required.
3. Rerun the focused tests after the final rebase.
4. Commit scoped files only.
5. Push fast-forward updates to `test` in dependency order: robot first, BMS second.
6. Use the managed release path for robot API and BMS Development. Preserve old releases for rollback.
7. Prove managed owner, PID/cgroup, working directory, listener, commit, tree, and clean release.
8. Query only health, OpenAPI, catalog, dashboard, pipette status, firmware/data/status routes explicitly classified as read-only.
9. Open the actual Development BioXP route. Verify the dedicated pipette pane, four channels, receipt phases, disabled mutation controls, and browser console.
10. Compare command/receipt history before and after passive acceptance. Require zero physical pipette mutation.

## Completion boundary

This tranche completes only when PIP-001 through PIP-017 have focused source evidence and passive Development evidence. PIP-018 remains pending by design. Full Controlsuite parity, physical dry acceptance, and wet commissioning remain prohibited claims until Christian gives a separate physical authorization and those gates pass.
