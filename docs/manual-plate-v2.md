# Public manual plate custody (V2)

Explicit button presses use `POST /operator/v2/actions/oem.deck.catch_plate` and
`POST /operator/v2/actions/oem.deck.release_plate`. These are public normal
canonical commands, not internal admissions, workflow documents, or motion plans
submitted by the browser. Each has its own root request/command identity and the
existing persisted finite OEM child ledger.

Both accept the unchanged `bioxp.operator_action_request.v2` envelope:
`schema_version`, `expected_ownership_generation`,
`expected_board_epoch_by_board`, `idempotency_key`, and `inputs`.

- Catch inputs: required strict integer `plate`; optional strict boolean
  `run_in_parallel` (native default true).
- Release inputs: required strict integer `destination`; optional strict boolean
  `press_plate` (native default false) and `run_in_parallel` (native default true).
- Unknown fields and non-native types are rejected. Omitted defaults remain
  omitted in canonical request identity; the existing finite compiler applies
  them. The metadata/full V2 catalog exports `inputs` with source enum values,
  required flags, types and defaults. Source enum membership does not promise
  an active Serial-206 PositionTable row: the existing native missing-row
  refusal remains authoritative.

Reuse the existing method author's `custody.objects` entries
`{token,label,ordinal}` and `custody.destinations`
`{token,label,plate_destination}`. Usual objects are PL_POOL=0, PL_OUTPUT=1,
PL_REAGENT=2, CV_OUTPUT=4, CV_REAGENT=5. Plate placements at OC/TC/MS use native
21/23/25, not named head-travel 1/2/0. Do not reinterpret generic head movement's
`destination_options` as placement options. No second placement catalog or
protocol schema is introduced.

The existing finite worker resolves current machine/custody state at dispatch,
compiles `catch_plate` or `release_plate`, and executes inline under that public
root's owner. It never enqueues a child behind itself. Existing resource leases,
board/owner checks and Stop fences cover both public IDs. Source door operations,
state-dependent branches, offsets, speeds, gripper locking, clearance and custody
publication remain in the unchanged OEM compiler/provider/adapter. No historical
custody-proof gate or sensor-proof admission requirement is added.

Optional compact terminal `plate_operation` is:

```
{operation: "catch_plate" | "release_plate",
 source_return_ok: boolean | null,
 exception_suppressed: boolean,
 failed_child: string | null}
```

It is copied from the actual finite source response, never inferred from command
status or physical placement. It can be absent before a source response exists
(e.g. compile failure or a winning Stop). The normal command status remains
canonical. `physical_effect_verified` remains separate and false without
independent physical evidence. Full controller/per-step outcomes remain in
receipt detail and the existing finite ledger; successful software custody
publication is not independently verified grasp or placement. No automatic
pickup/dropoff, replay, initialization or Home is authorized on UI mount.
