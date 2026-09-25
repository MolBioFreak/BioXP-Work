# Manual well-addressed pipetting contract

`bioxp.manual_pipetting` supplies `ManualPipettingRequest` (Pydantic, JSON schema),
`compile_manual_pipetting(request) -> ProtocolDocument`,
`manual_position_plan(params)`, and `bind_manual_position_handler(...)`.

## API integration (route/catalog edits intentionally owned separately)

Add one entry to `api._protocol_live_handlers()`:

```python
ProtocolActionKind.PIPETTE_POSITION: bind_manual_position_handler(
    command_store=_protocol_command_store(),
    execute_plan=app.state.oem_workflow_plan_executor,
    require_motion_ready=_require_motion_route_ready,
),
```

Import the binder from `.manual_pipetting`. Bind after the ordinary command plane
exists; do not make a new queue, thread pool or controller connection. Existing
`pipette_aspirate` and `pipette_dispense` handler entries remain unchanged.

For a compile route, accept `ManualPipettingRequest` and return
`compile_manual_pipetting(req).to_payload()`. To execute, wrap that document with
`source_type="native"` in the existing `ProtocolExecuteRequest` and call the
existing `protocol_execute` / `create_protocol_job` path, retaining its caller
idempotency and live-intent fields. Do not call the handler directly from HTTP.
Do not set `input_mode="oem_prepared"` or use `oem_operation` for these buttons.
The existing protocol dry run remains nonphysical. Catch compiler `ValueError`
as request validation, as the ordinary protocol route already does.

### Physical pickup / measurement binder

Import `bind_manual_physical_handler` from `.manual_pipetting` and add:

```python
ProtocolActionKind.PIPETTE_MANUAL_PHYSICAL: bind_manual_physical_handler(
    command_store=_protocol_command_store(),
    execute_plan=app.state.oem_workflow_plan_executor,
    require_motion_ready=_require_motion_route_ready,
    provider_getter=lambda: _serial206_oem_initialization_provider,
    receipt_store_getter=lambda: _pipette_receipts,
),
```

Both operations compile to ordinary `pipette_manual_physical` actions, not OEM
metadata. Each action submits ONE finite compound (`manual_load_tip` or
`measure_fluid_height`). Source finite subcalls execute inline in that provider
child's existing owner, preserving nested receipts, `z`, `source_return`, partial
failures and occurrence-scoped semantic publications. The BR completion callback
starts nonblocking Z before the shared pipette wait; no nested queue submission.
Pipette calls use the real receipt service and shared collection inline, with
`caller_class=protocol_manual`, distinct per-call receipt identities and the
finite command recorded as parent (never reused as the pipette claim ID).

Pickup reads location from the provider's persisted source machine object's
`machine_status.constructed_tip_trays[index].location`; this is the existing
ClassMachineStatus projection, not a new model/occupancy constructor. It reads
no occupancy, timestamp, or tip-history admission evidence. Measurement captures
`config_sections.offsets.m_Z_MOTOR_MAX_CURRENT_DOWN` from the active source machine
snapshot; no numeric fallback. Existing source positioning reads the canonical
location/well and active PositionTable.

Results retain the complete source `steps` under the compound child result and
promote actual `position_steps`, `source_return`, `timing`, `fluid_timestamps`, or
pickup lost-step fields through the finite response. Success is not physical
verification or persisted calibration. No lifecycle prep, sweep or Park is added.
The repeated `zOffset` scan remains unsupported: source loadTips and modeled
volume dependencies have not been supplied here; no misleading scan button.

Offline qualification: `tests/test_manual_physical_native.py` exercises ordinary
native dispatch, real OperatorCommandStore and PipetteReceiptStore, production
provider/motion adapters and real collection/channel transports, replacing only
native IO and captured source inputs. Repeated operations verify distinct durable
identities; missing fluid timestamps still succeed; native failure stops without
invented cleanup; BR -> owned nonblocking Z -> wait ordering is asserted.
Combined qualification: 191 passed, 1 deselected across manual/native calibration,
provider, native protocol, binding closure, OEM air/stream and composite suites.
The deselection is the documented unchanged RGB baseline expectation below.

## Typed authoring request

Top-level fields: `protocol_id: nonempty string`, `steps: nonempty array`.
Every step rejects extra fields and has one of these exact shapes:

| operation | Required fields | Meaning |
|---|---|---|
| `load_tip` | `tray: int (1..5)`, `well: A1..B12`, `overpress: bool = false`, `lift_z: bool = false` | Physical source manual pickup; no fabricated TipLoaded/type/inventory mutation |
| `measure_fluid_height` | `speed: int = 300` | Source fluid-height measurement at current well, no durable calibration Apply |
| `move` | `location_id: int`, `well: string or int`, `position_flag: 0 or 1 or 2` | Actual calibrated XY plus source Z routing, then location publication |
| `lower` | `location_id: int` | In-place source `lowerTo`: calibrated `zLow` |
| `lift` | `location_id: int`, `height_steps: int or null` | In-place source `liftTo`: null selects `zHigh`; integer selects `zLow - height_steps` |
| `aspirate` | `channels: int[]`, `volume_ul: number`, `speed: number` | Existing explicit-channel/speed liquid owner, in place |
| `dispense` | `channels: int[]`, `volume_ul: number`, `speed: number` | Existing explicit-channel/speed liquid owner, in place |
| `mix` | `channels: int[]`, `volume_ul: number`, `aspirate_speed: number`, `dispense_speed: number`, `cycles: int` | Explicit repeated aspiration/dispense strokes, in place |

Location IDs are the canonical OEM `locationID` values, not plateName ordinals;
use the selected block's canonical location. Wells use existing `A1..H12` /
`0..95` decoding. Position flag 0 uses current source pseudo-home, 1 calibrated
high, 2 calibrated low. No XY, Z calibration, speed or volume is fabricated.
The existing `moveZ` primitive retains its source pseudo-home clamping behavior.
The finite enum/grid does not establish that every enumerated well is physically
usable at every station. Channel IDs are 0..3; existing command validation owns
volume/speed/channel constraints. Mix cycles are 1..50, matching the existing
native mix request bound. Mix here is repeated explicit strokes, **not** the
OEM `mmix` scientific body or `mixAll` controller command.

### Exact channel / well semantics

`move` uses the real source machine `TipLocation`, not the plunger checkbox list.
At ordinary liquid stations, effective row is `row - 2 * TipLocation`, with -1
(the all-tip reference) treated as 0. Racks 7..10, waste 6 and trough 16 are
source exceptions and do not subtract that offset. With four tips, the selected
well is the source head reference; other channels retain their fixed spacing.
Choosing a single liquid channel does **not** cause an extra gantry move and
must not publish `tip_location`, `tip_loaded`, or a fake inventory. Display this
reference distinction in the UI. Existing tip-loading/state ownership supplies
the source tip alignment; these controls do not silently load or select tips.

### Scope

Author a transfer as the explicit ordered move/lower/aspirate/lift/move/lower/
dispense/lift steps the operator requested. A single button can submit a
single-step program. Lower/lift and liquid buttons do not infer XY from a well
label. A well change takes effect only via an explicit Move. No automatic
initialization, loading, fluid detection, aspiration air, piercing, cleanup,
withdrawal on failure, tip sweep, lid operation or Park is appended. Existing
routing/interlocks and workflow ownership remain authoritative; missing history
or diagnostic observations are not new admission gates.

## Source / implementation chains

Retained SSD `decompiled_src/BioXPControlLib/`:

- `ControlLib.cs:6812-6887` (`movExecution`) resolves destination/well, calls
  `scriptmoveTo`, then `updateLocation`. The manual finite wrapper supplies an
  explicit location/well and uses just these existing children, no piercing.
- `ClassControlInterface.cs:3718-3847` decodes well / PositionTable XY / selected
  tip compensation / Z flags; subsequent `scriptmoveTo` body retains loaded-tip
  and carried-cover routing. Port: provider `scriptmoveTo` -> production
  `oem_scriptmove_to` -> `OemPathPlanner` -> `_execute_oem_steps_live` -> native
  axes. No replacement geometry implementation.
- `ClassControlInterface.cs:4138-4162`: `lowerTo`, both `liftTo` overloads. Port:
  existing `pipette_lower` / `pipette_lift` -> `wp8_pipette_source_leaf` -> live
  PositionTable -> existing `moveZ`.
- `ClassPipetteCollection.cs:844-882,1055-1093`: explicit channel/speed Aspirate
  and Dispense. Port: ordinary protocol pipette handler -> pipette service ->
  `FourPipetteTransport` -> `CanPipetteTransport` -> existing driver. All stroke
  volumes/channels/speeds are operator-authored; the compiler reuses native
  command validators and does not alter collection custody.

## Offline qualification

`tests/test_manual_pipetting.py` exercises typed validation, round-trip native
compilation, real generic protocol execution, SQLite finite-command admission /
child receipts / canonical location publication, real movement provider and
production movement adapter against synthetic native I/O, calibrated lower/lift,
all source tip compensation values and exempt locations, explicit liquid owner
execution for one/four channels, ordered transfer + repeated mix strokes, and
move/aspirate/dispense failure stopping without lifecycle side effects.

Fixtures explicitly provide synthetic calibration and initial state; none are
installed as runtime defaults. Physical liquid accuracy, installed calibration,
and actual tip presence are not established by these tests. No robot contacted,
no deploy, no push, no restart.

Qualification command:

```sh
PYTHONPATH=src:. /home/dalab/.cache/bioxp-oem-cv-venv/bin/python -m pytest \
  tests/test_manual_pipetting.py tests/test_protocol_oem_provider_bindings.py \
  tests/test_protocol_oem_native.py tests/test_protocol_v1_binding_closure.py \
  -q -k 'not test_current_stall_and_rgb_source_contract'
```

Result: **93 passed, 1 deselected**. The deselected existing RGB test expects
one call after two `setColor` occurrences; current source makes both calls.
It also fails when the provider module is loaded directly from baseline
`a65a176` with `git show`, so this unrelated expectation was not changed.
