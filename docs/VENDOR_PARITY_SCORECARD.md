# BioXP Vendor-Parity Scorecard

Date: 2026-04-12

Purpose
- Track core OEM-equivalent capability in a way that distinguishes raw primitive coverage from usable, validated machine behavior.
- Keep the program honest about what is real, what is partial, and what still lacks physical proof.

Status scale
- Not started: no meaningful new-runtime implementation
- Primitive only: low-level controls exist but no semantic/operator-safe layer
- Partial: some semantic or operator-safe behavior exists, but not enough for dependable core use
- Core-ready: feature exists in a typed/operator-safe form and has validation evidence

Practical parity estimate right now
- Overall practical OEM parity: about 15-25%
- Primitive-only low-level coverage: about 40-50%

## Core parity rows

| Area | OEM evidence | Current Linux/runtime status | Score | What blocks core-ready | Required evidence/gate |
|---|---|---|---|---|---|
| USB/TMCL transport | OEM CAN/board control assemblies and live control paths | Direct USB runtime exists in `src/bioxp/usb_driver.py` | Partial | Need transaction-level observability and trace corpus | Request/reply decode fixtures plus captured traces |
| Board activation/status | `activateBoard()` / firmware/status flows | Implemented and usable | Partial | Reliability still coupled to manual recovery | Repeated restart/reconnect validation |
| Motion primitives | TMCL `MVP/SAP/GAP/RFS`, OEM motor flows | Relative/absolute/home endpoints exist | Partial | Physical truth still not independently proven | 3+ supervised proof bundles per canned move |
| Motion prep/interlock policy | OEM startup/init and live-gate semantics | Exists, but historically allowed risky reuse path | Partial | Need hard policy and operator-visible truth semantics | Debug-only gating + tests + operator messaging |
| Reference/homing workflow | OEM startup and reference sequences | Guarded home exists per axis | Primitive only | No start-of-day reference procedure/state model | Documented single-axis supervised reference procedure |
| Deck/location semantics | `PositionTable`, `locationID`, `wellID`, `scriptmoveTo(...)` | Not implemented | Not started | No deck/labware/location model | Typed location resolution + config-driven layouts |
| Pipette/tip/liquid subsystem | `ClassPipetteCollection`, `ClassPipette`, liquid verbs | `/liquid/aspirate` and `/liquid/dispense` still `501` | Not started | Missing pipette transport and service layer | Validated aspirate/dispense workflow |
| Thermal control | OEM thermal board classes and scripts | Meaningful baseline/setpoint/fan/PWM/rates control exists | Partial | Needs integration into protocol engine and stronger validation | Repeated thermal program validation |
| Chiller control | OEM chiller board classes | Meaningful baseline/setpoint/fan/PWM/rates control exists | Partial | Needs protocol integration and fault/trace evidence | Repeated chiller program validation |
| Vision/barcode/inspection | CVision stack, zbar/OpenCV dependencies, inspection semantics | Camera transport/control exists; no semantic vision layer | Primitive only | Missing barcode and typed inspection services | Barcode + one inspection flow validated |
| Script/protocol engine | `ClassBioXPScriptHandler`, XML scripts | No normalized workflow runtime yet | Not started | Missing protocol model/compiler/executor | One native multi-stage workflow runs end-to-end |
| OEM XML compatibility | `demo.xml`, `lifetest.xml`, script handlers | Assets exist, importer absent | Not started | Need XML importer with coverage accounting | Fixture imports with explicit unsupported-verb report |
| Artifact-backed traceability | OEM logs/state, modern operator need | Manual notes only; no canonical proof bundle yet | Not started | Need artifact service + bundle schema | Motion/protocol artifact bundles on disk |
| Operator control plane | OEM GUI/service tooling | BMS cockpit exists and proxy works | Partial | Still primitive-first, not semantic/protocol-first | Protocol launch/review/resume + artifact panels |
| Diagnostics/service workflows | OEM service-oriented init/reset flows | Several manual diagnostics exist | Primitive only | No consolidated documented service workflows | Runbooks + typed service endpoints + validation |

## Flexibility rows beyond strict OEM parity

| Area | Current status | Target outcome |
|---|---|---|
| Capability gating | Not implemented | Machine profile can enable/disable subsystems without code forks |
| Config-driven deck/labware | Not implemented | Layouts change by YAML/data, not source edits |
| Native protocol format | Not implemented | YAML/JSON protocols compile into normalized actions |
| Simulation/dry-run | Not implemented | Protocols can run against a simulator before hardware execution |
| Custom workflow composition | Not implemented | Subprotocols, loops, review gates, and templates are reusable |
| API-first execution | Primitive-only | Typed semantic actions and protocols can launch without UI poking |

## Current top blockers

1. Motion truth is still controller-centric.
2. No semantic deck/location/well layer exists.
3. Pipette/liquid handling is still missing.
4. Vision semantics are missing even though camera transport exists.
5. There is no normalized protocol engine.
6. There is no OEM XML importer or coverage report.

## Current acceptance policy

Do not mark a row Core-ready unless at least one of these is true:
- automated tests exist for the relevant typed/service surface
- supervised proof bundles exist for the physical behavior
- both, for safety-critical subsystems

## Immediate scorecard moves planned now

- Lock down motion prep reuse semantics
- Expose controller-only truth labeling in motion responses and UI
- Create the first canonical execution backlog and architecture target docs
- Start artifact/bundle scaffolding next
