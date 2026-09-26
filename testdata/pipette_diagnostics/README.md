# PIP-C3 robot/BMS contract fixtures

`schema.json` is generated from `ManualDiagnosticPipette.model_json_schema()`.
`requests.json` contains every action's manual-authoring request and the corresponding native `ProtocolDocument` for the existing `/protocol/execute` path. These are examples, **not requests to execute**. No new endpoint, queue, firmware console or controller was introduced.

The manual step is:

```json
{"operation":"diagnostic_pipette","diagnostic":{"action":"aspirate","channels":[0,2],"volume_ul":10.25,"speed":100}}
```

The native action kind is `pipette_manual_physical`; its `params` is that complete step. It compiles to one `diagnostic_pipette` finite operation containing `sourceDiagnosticPipette`, with nested receipts under the same existing finite owner.

## Closed action union

- `aspirate`, `dispense`: required `channels` (unique zero-based integers 0..3, empty allowed), finite nonnegative `volume_ul`, positive integer `speed`.
- `eject`: required `channels` with the same numbering/uniqueness rules.
- `plunger_up`, `plunger_down`: required nonnegative integer `steps` (maximum signed 32-bit magnitude). Up negates the magnitude; down retains it.
- `dispense_all`, `diagnoses`, `initialize`, `get_data`, `last_error`: no additional arguments.
- Unknown actions/fields, opcodes, diagnostic test numbers and Last Error byte selectors are rejected by the typed contract. No channel normalization from 1-based values.

## Results

The finite result is in `completed_children[0].result`; failures preserve the partial result in the executor's `provider_results`. Existing workflow/dispatcher wrapping still applies. The result includes `action`, `source_anchor`, `events`, `ok`, `completed`, `physical_effect_verified:false`; events have distinct nested pipette receipt identities. Physical accuracy is not established by these results.

Action-specific presentation:

- Liquid: `selected_channels`, `lost_tip_channels`, `cached_tip_channels`, and `stroke` when volume is positive. Aspirate retains lost-tip selection; Dispense removes only lost cached-tip selections. Tip queries happen only for cached tipped channels. Explicit speed setters and group waits are reused.
- Dispense All: `cached_tip_channels` and `dispense_all`; no TipLocation selection or fresh-tip gate.
- Diagnoses: ordered `tests` 0/1/2, each with `label`, four `channels` (`diagnosis`, `display`), and raw group `result`. Cached tipped eligibility only.
- Initialize: `attempts`, at most initial + retry; group then status, retry only on first false status. No constructor firmware/condition sweep.
- Eject: `ejected_channels`; every channel is queried in order and only selected query=1 channels are sent eject, without travel or group ejection.
- Get Data: four `channels`, each with `part_number`, `revision`, `firmware`, original `information` and `data`. Malformed part/revision replies preserve the OEM indexing exception and completed earlier channels. No obsolete log file.
- Last Error: four `channels`, each with `error`, hexadecimal `display`, and original `result`; raw query byte is fixed to 1. OEM null-reply default remains distinguishable in raw result.
- Plunger: current31 then signed relative Z `events`, without current restoration.

For the source void liquid/Dispense All/Diagnoses/Initialize callers, `ok`/`source_return_completed` means that source caller returned, not that all controller observations succeeded. `controller_outcome_ok` and the individual events retain false waits/status. The source ignores these particular wait Booleans; Initialize uses first status only to choose its one retry. Real exceptions and addressed Stop remain failures. Other primitive API admission is unchanged. Do not render source return as physical success.

## Source and intentional UI difference

Retained SSD `decompiled_src/BioXPControlLib/ClassPipetteCollection.cs`: 418–531 (liquid buttons), 533–555 (Dispense All), 557–569 (Last Error), 571–582 + 261–283 (Diagnoses), 584–604 (Initialize), 606–624 (Eject), 640–659 + 302–309 (Get Data), 750–775 (cached speed), 844–882 (explicit Aspirate). `ControlLib.cs:1417–1433`: current31/relative Z.

The OEM Dispense widget reads the Aspirate textboxes at Collection:523. Typed Dispense deliberately uses its own `volume_ul` and `speed`; this is not literal reproduction of that widget bug.

The tests connect the actual manual compiler/handler, finite provider/executor, collection, OperatorCommandStore and PipetteReceiptStore, replacing native IO only. Fixtures include source-state/owner authority setup, not real device observations. Network sockets are blocked (asyncio AF_UNIX is allowed). Offline tests do not qualify hardware, BMS rendering, mounted routes or deployment.
