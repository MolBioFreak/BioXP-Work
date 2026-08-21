# BioXP 3200 serial-206 four-channel pipette OEM gap-rectification and release specification

**Revision:** 1.0
**Status:** FINAL SPECIFICATION — OEM-vetted contract; implementation-ready parity is rejected until unresolved rows close
**Finalized:** 2026-08-03 UTC
**Machine:** BioXP 3200 serial 206
**Canonical path:** `docs/specs/2026-08-02-pipette-oem-gap-rectification-spec.md`
**Scope owner:** Christian

## 1. Purpose and claim boundary

This document is the final, evidence-bound specification for rectifying and accepting the serial-206 four-channel pipette subsystem. It defines what must be implemented, how departures from literal OEM behavior must be classified, and which evidence is required before any parity or release claim. It is a final specification, not an implementation-ready OEM equivalence approval.

Finalizing this document does **not** establish that the Linux/BioModStack implementation is:

- source-complete;
- OEM-runtime-equivalent;
- deployed from the reviewed source;
- physically accepted;
- commissioned for dry or wet liquid handling; or
- release-ready.

No pipette completion percentage is authorized by this specification. A percentage may be reported only after the complete denominator in Sections 14–16 exists and every counted item has row-level evidence.

This revision corrects material errors in the provisional draft, including:

1. `ControlLib.initializeMotion()` initializes the pipette group only in the stale-tip branch; the no-tip branch only clears host `TipLoaded` state.
2. Constructor initialization, `initializeMotion`, diagnostic initialization, and `initPipette()` are distinct OEM call sites with distinct retry/failure semantics.
3. OEM wake uses a special global-ID transport path with no correlated device ACK; the caller logs null/non-space response information, then unconditionally delays 100 ms and sends `WR`.
4. OEM `ClassPipetteCollection.waitforcompletion()` is apartment-dependent; it is not universally one shared deadline.
5. OEM `NovoEncoding.Decode()` does not validate delimiters, checksum, or DLC even though encoding appends an additive checksum and CAN records contain DLC.
6. The pinned `NovoCANUSBLib.dll` contains a real unreachable `ID & 7 == 259/260` branch; this is a binary defect, not a decompiler typo.
7. OEM `mixAll()` passes `m_TipType` as the mix volume and ignores its `vol` argument.
8. The application aggregate is larger than `ControlLib`; `BioXPMainWindow` owns startup gating, the serialized motion worker, event-subscription order, early returns, and terminal status writes.
9. The current Linux implementation has advanced beyond several old “missing” claims, but it still lacks complete parity and physical acceptance.

## 2. Controlling authority

### 2.1 Sealed evidence lock

The controlling evidence lock is:

`/home/dalab/Desktop/ROBOT/BioXP 3200 Development Work/reports/bioxp_historical_run_forensics_20260717/derived/oem_runtime_parity_spec_20260719/oem_machine_runtime_bundle_serial206/OEM_EVIDENCE_LOCK.json`

- lock SHA-256: `a69454df24e9348fd34d8c89f2a2e089576587152bdcc20754f9d700ecbaf03c`
- schema: `4`
- acquisition: `20260719T024740Z-4a7fe6783205846c`
- target: `BioXP-206`

The lock’s conflict order controls:

1. hash-pinned captured binary and direct binary IL;
2. hash-pinned decompiled source mapped to that binary;
3. hash-pinned serial-206 machine/configuration artifacts;
4. captured runtime traces and logs from the same machine/version;
5. Linux replacement source, tests, and documents as implementation evidence only.

A later abstraction, test, comment, or document may not override conflicting OEM evidence.

### 2.2 Pinned binaries

The relevant lock-pinned binaries are:

| Artifact | SHA-256 |
|---|---|
| `GenBotApp.exe` | `d7cb7c94ebcfc386bc0a4acebb92f13d8463bb2b675f5439adf43be35198c6d9` |
| `BioXPControlLib.dll` | `163db8f7835cecbc87da4d14734a8224d79ea1e2ccc77bbb299998fa31bf14ed` |
| `ClassCanLib.dll` | `7d91d8279fa967c20d006679c1c2d6545451b898fdeca9028224ef6d0c46dac1` |
| `NovoCANUSBLib.dll` | `6856ba3bda0033b938483ea7deeb664478cf446dabc93d1048fbb843d3cda3c3` |
| `Novo.Devices.dll` | `971efffd20c753de10f0750193cefb437ac64aac6ae6b6c22788d5bc5f2e1729` |
| `BioXPCommonLib.dll` | `c0e430426a5af34c4fcadf067d2598aa98392569fd0175661d6478f18833382e` |

### 2.3 Pinned source projections

The audited source projections match the lock’s decompile-anchor hashes:

| Source | SHA-256 | Role |
|---|---|---|
| `decompiled_src/BioXPControlLib/ControlLib.cs` | `f69b3529dcb9723c705ac55ecb3f035010cc294d3891de096c165bb20116f6c2` | application lifecycle and workflow call sites |
| `decompiled_src_genbotapp/GenBotApp/BioXPMainWindow.cs` | `b288a45e2de54cd2c8d30a4498a343cd6f423aff7e88a78847076bfbfb4e904c` | application/startup aggregate, motion worker, startup gates, event binding, terminal status |
| `decompiled_src/BioXPControlLib/ClassPipetteCollection.cs` | `ffe3729fa35642d04ef6fe45501e52200dd7c2977a70902aa20e46fb26d4011e` | four-channel collection |
| `decompiled_src_can/BioXPControlLib/ClassPipette.cs` | `681f959cf527b060cece17b3cf7ff59c1ba1f5ead99fea53520d09486ac0c957` | per-channel command/state machine |
| `decompiled_src_can/ClassCanLib/ClassNovo.cs` | `11293074caec278076723666e69022b547c43f32b5fa886c99f75d5b60043d06` | CAN façade and pipette event forwarding |
| `decompiled_src_novo/NovoCANUSBLib/ClassNovoCANUSB.cs` | `e4cf1c311ed5ae79e9490564a48947bced5f46af36890cf4a77120a3b50ffb06` | transaction, routing, 60 s pipette wait |
| `decompiled_src_novodevices/Novo/Devices/CanInterfaceBoard.cs` | `9603dfd57f8c56e0423da651280c3a6a70b38a2902662588a0d359827f477724` | raw USB-CAN record transport |
| `decompiled_src_novodevices/Novo/Devices/NovoEncoding.cs` | `2cd04c7cf947028161cd65320f2bc501a6c184de1d7984d895bcf91621cc68bc` | framing, escaping, checksum generation |
| `decompiled_src_novodevices/Novo/Devices/CAN/Interfaces/CanPacket.cs` | `07e359eb6479388925ffd94b3689010321b13eee183c55b3689947eb8cf97181` | CAN packet/DLC/ID value object |
| `decompiled_src_novodevices/Novo/Devices/CAN/Interfaces/ICanPacket.cs` | `b98c3bbb458e9ba0e012f8abfd45fe1488bf413a104fd02a5cfa36c22249fc91` | CAN packet interface |
| `decompiled_src_novodevices/Novo/Devices/TecanADP.cs` | `53b0ca7f3504a80a1c35ba84849ef8211401831f0a065697daaaa291f97a5c59` | pipette command/response simulator and ordering evidence |
| `decompiled_src_novodevices/WinUsb/WinUsbCommunications.cs` | `d4423e07690f5029692fcb2ae13c11dd990f2c23e0ca1f12a1bf1059d08a07ec` | WinUSB transfer boundary |

A source claim without a path and line/method anchor is non-authoritative.

The application authority is not exhausted by `ControlLib.cs`: `BioXPMainWindow.cs:253-329,375-821,973-1026,1046-1341,2030-2100` owns `MainWindowInitialize`, the serialized motion worker, `ControlLib` construction, `errorEvent` subscription order, `initializeEnvironment`, CAN/door/latch startup gates, `initializeSystem` queue consumption, early returns, caught/suppressed failures, and terminal status writes. `ClassMachineStatus.cs` is also a transitive authority for `TipLoaded`, `TipDirty`, `TipLocation`, location/well, and gantry-load state effects; it must be included in the method/call-site denominator even where its individual hash is not listed in the lock anchor table.

### 2.4 Serial-206 machine artifacts

The selected machine files are lock-pinned:

| Artifact | SHA-256 | Pipette-relevant facts |
|---|---|---|
| `appdata/config.xml` | `33aadf87f631cf33f2e0b4c86948c92be3b21412ca5477ea8fa8bc7848cbf475` | serial 206; calibrated; liquid calibration true; camera installed/calibrated; `OutlierRangeFactor=4`; tip collections TypeA capacity 10 and TypeB capacity 25 |
| `appdata/Operation_parameters.xml` | `d032d58c08312706892a2d5c7a9a319f817a6d9ef73e75d30dd933ae110c9685` | `CheckForStaticTipLoss=False`; `LogPressure=False`; `PressureFailTipRestore=False`; `CheckCamera=True` |
| `appdata/pressurebuffer.txt` | `96213f97bdf9b429642e0e1e8ed86500c22bd45f17dd96a546b3b37362295c42` | captured four-pipette pressure calibration/reference data |

Credentials and secret-bearing fields in the raw configuration are evidence but must never be copied into source, receipts, logs, fixtures, or this specification.

## 3. Evidence classes and divergence policy

Every parity row shall carry one of these classifications:

- **OEM-LITERAL:** exact binary/source behavior, including defects.
- **OEM-OBSERVED:** captured behavior from the pinned runtime or serial-206 wire trace.
- **SAFETY-HARDENING:** intentional fail-closed behavior stricter than OEM.
- **PRODUCT-EXTENSION:** Linux/BMS capability with no OEM claim.
- **UNRESOLVED:** source/binary ambiguity or missing runtime evidence.

A `SAFETY-HARDENING` or `PRODUCT-EXTENSION` row may be accepted, but it may not be described as literal OEM parity. Where literal OEM behavior is unsafe or defective, the implementation must expose both the OEM fact and the approved divergence.

## 4. Authoritative physical and ownership model

### PIP-001 — Four independent pipettes

The subsystem is exactly four independent ADP pipettes with zero-based IDs/channels `0`, `1`, `2`, and `3`, coordinated by one collection. The collection constructor creates all four unconditionally. No invented `PipettesInstalled` configuration gate may suppress a channel.

### PIP-002 — Single shared USB owner

Production-equivalent operation requires:

- one service-owned Novo USB interface;
- VID `0x03eb`, PID `0x2423`;
- WinUSB interface alternate setting `1`;
- one bulk-IN reader;
- one router/demultiplexer;
- serialized registration of response/completion owners; and
- no independent pipette endpoint reader or SocketCAN production fallback.

A client may release its references but may not stop or close the shared owner.

### PIP-003 — Per-channel CAN IDs

For channel `p` in `0..3`:

| Family | TX ID |
|---|---|
| control | `0x100 | (p << 3)` |
| command | `0x101 | (p << 3)` |
| first multipart | `0x103 | (p << 3)` |
| middle multipart | `0x104 | (p << 3)` |
| report/query | `0x106 | (p << 3)` |

The corresponding normal synchronous receive ID is `TX ID | 0x400`.

The OEM synchronous pipette matcher contains exactly 20 normal IDs: decimal `{1280,1281,1283,1284,1286,1288,1289,1291,1292,1294,1296,1297,1299,1300,1302,1304,1305,1307,1308,1310}`. This set is not the entire pipette receive surface. Pressure-stream samples use a separate four-ID asynchronous set `{1285,1293,1301,1309}` and must route only to the pressure parser. Unknown traffic outside both sets is unmatched/rejected. An arbitrary frame, a frame for the wrong channel, or a frame for the wrong function is not success.

### PIP-004 — Raw USB-CAN record and framing

The raw-framing authority additionally includes `CanInterfaceBoard.cs`, `NovoEncoding.cs`, `CAN/Interfaces/CanPacket.cs`, `ICanPacket.cs`, `TecanADP.cs`, and `WinUsbCommunications.cs`; `ClassNovoCANUSB` begins above this byte-framing layer.

Before framing, the record is:

1. four-byte big-endian module/CAN ID;
2. one-byte DLC;
3. exactly DLC data bytes.

`NovoEncoding.Encode()` then:

1. computes the eight-bit additive sum of every record byte;
2. appends that checksum byte;
3. escapes `0x7e` and `0x7d` as `0x7d` followed by byte XOR `0x20`;
4. wraps the encoded body in `0x7e` delimiters.

OEM `NovoEncoding.Decode()` strips framing/escaping/checksum position but returns `true` unconditionally and does not verify delimiter, checksum, declared DLC, malformed escapes, or minimum length. Linux delimiter/checksum/DLC/escape/length validation is required as `SAFETY-HARDENING`; it must be fixture-validated against valid OEM frames and separately test malformed rejection.

The WinUSB pipe timeout is configured as 2000 ms. This is distinct from pipette transaction and collection-completion timeouts.

## 5. OEM transaction and completion contract

### PIP-005 — Synchronous transaction owner

OEM registers a response event/waiter before ordinary pipette TX, but recovered completion correlation is channel plus mutable `m_LastCMD`, not a transaction ID. A Linux owner shall register an explicit expected transaction, correlate channel/function/reply shape, and reset before TX as `SAFETY-HARDENING`.

For ordinary non-pipette families, the caller-supplied timeout is used. For the OEM pipette transaction path, `ClassNovoCANUSB.sendCommand()` uses a hard-coded 60,000 ms wait; its `timeout` argument does not control that pipette branch. `ClassNovo.TransmitMessage()` also serializes the wrapper transaction under one global lock and inserts approximately 1 ms before and 10 ms after the transaction. Those delays are distinct from the 2,000 ms WinUSB pipe timeout and collection-level waits.

### PIP-006 — Immediate versus delayed response

A pipette command can produce:

1. an immediate function-family response consumed by the synchronous transaction; and
2. a later command-completion/error frame consumed by `ClassPipette.processMessage()` and collection wait handles.

Receipt of the immediate response is not automatically proof of final command completion. TX completion is never physical-effect proof.

Initialization (`WR`) is the critical two-stage case: in the pinned simulator/order, a DLC-zero function-1 frame is the immediate response; a later function-1 frame carrying status/error data drives channel completion.

Literal matcher behavior must also be retained: function 1 with DLC zero returns matcher result 0 and removes the synchronous waiter; function 1 with nonzero DLC returns result 3, signals the synchronous response event, and does not remove that waiter. The pinned `TecanADP` simulator emits DLC zero first and `[0x20,0x60]` later, producing the intended immediate-then-asynchronous flow. A serial-206 trace is still required before declaring that ordering universal.

### PIP-007 — Multipart behavior and pinned binary defect

Commands longer than eight bytes are split across first/middle multipart ID families. Reports may arrive across first/middle/report families and must be reassembled in arrival order for the same transaction/channel. OEM evidence is weaker than this target contract: RX multipart accumulation is keyed by module ID, `m_multFrame` is global rather than channel-scoped, and per-command builder loops disagree (`Aspirate` starts at chunk index 1 while `Dispense` and `setTopSpeed` start at index 2). Linux channel/transaction-scoped assembly is safety hardening, not OEM fact.

The pinned `NovoCANUSBLib.dll` contains this literal IL behavior in `sendCommand()`:

- `ID & 7`;
- compare to decimal `259` and `260`.

Those comparisons are unreachable. The later `transmitCommand()` path masks with decimal `263` (`0x107`) and compares to `259/260` as intended. Therefore “multipart TX is no-wait” is not established by source intention alone. Acceptance requires a pinned binary execution trace or captured serial-206 wire trace for at least one fragmented command and one fragmented report. Until then this row is `UNRESOLVED`, and Linux fragment normalization is not an OEM-equivalence claim.

### PIP-008 — Collection completion waits

`ClassPipetteCollection.waitforcompletion(job, timeout)` behaves differently by apartment state:

- STA: waits each of four handles separately with the full timeout and logs each timeout;
- non-STA: performs one `WaitAll` with one timeout.

The Linux runtime must either reproduce the apartment state and literal behavior or explicitly adopt one bounded policy as `SAFETY-HARDENING`. A single shared 10-second deadline must not be called universally “exact OEM.”

The method logs lost completion/time anomalies and returns Boolean. Callers do not consistently honor that Boolean; each call site must preserve or explicitly harden its own behavior. OEM delayed completion is interpreted against the channel’s mutable `m_LastCMD`, not a transport transaction ID; Linux pre-registration, transaction IDs, and reset-before-TX ordering are safety hardening requirements.

## 6. Complete per-channel OEM command surface

The per-channel implementation denominator is every public/internal property and method in the pinned `ClassPipette`, not only startup commands.

### 6.1 Channel state

Each channel owns at least:

- pipette ID and five CAN address families;
- `PipetteStatusCode` (`NOT_DETERMINED=0`, `STATUS_FALSE=1`, `STATUS_TRUE=2`);
- current `PipetteCommands` category;
- last error/status code;
- queued error vector;
- command-completed state and wait-event signal;
- tip-loaded state;
- top speed;
- pressure and fluid level;
- fluid-detection timestamp;
- diagnosis result; and
- initialization counter.

### 6.2 Error enum

The complete `PipetteErrorCode` sequence is:

| Value | Name |
|---:|---|
| 32 | `NO_ERROR` |
| 33 | `INITIALIZATION_ERROR` |
| 34 | `INVALID_COMMAND` |
| 35 | `INVALID_OPERAND` |
| 36 | `PRESSURE_ERROR` |
| 37 | `OVER_PRESSURE` |
| 38 | `LIQUIE_LEVEL_DETECT_FAILURE` |
| 39 | `DEVICE_NOT_INITIALIZED` |
| 40 | `TIP_EJECT_FAILURE` |
| 41 | `PLUNGER_OVERLOAD` |
| 42 | `TIP_LOST` |
| 43 | `NOT_USED` |
| 44 | `EXTENDED_ERROR` |
| 45 | `NVMEM_ACCESS_FAILURE` |
| 46 | `COMMAND_BUFFFER_EMPTY` |
| 47 | `COMMAND_BUFFER_OVERFLOW` |

Names retain OEM spelling.

### 6.3 Command ledger

| OEM method/family | Literal payload/address | Required semantics |
|---|---|---|
| wake before first `initiate()`/`getData()` | global CAN `0x080`; `[0x20|p, 0x20|p]` | synchronously transmit; log null/non-`0x20`; do not claim OEM gates continuation |
| `initiate()` | `WR` on command ID after 100 ms | reset fluid level; last command `Init` |
| `initiateGroup()` | `WR` on command ID, no wake | set initialized undetermined; clear error collection before and after send |
| `DispenseAll()` | `A0R` | command category `A0` |
| `Mix(volume,count)` | composite: for `count-1` cycles, `Aspirate(volume)`, sleep 1500 ms, `Dispense(volume,0)`, sleep 1500 ms; final `Aspirate(volume)`, sleep 1500 ms | no dedicated Mix wire command; command category follows the constituent calls |
| `ejectTip()` | `E1R` when initialized, otherwise `E0R` | command category `Eject_Tip` |
| `terminateCommand()` | `TR` on control ID | command category `Terminate_Command` |
| `setTopSpeed(v)` | `V{v},1R` | update cached speed after send |
| `Dispense(v)` | `D{v},1R` | decrement channel fluid level |
| `DispenseAir(v,type)` | `D{v},{type}R` | update front/rear air or fluid bookkeeping according to `dispenseType` |
| `Aspirate(v)` / `AspirateAir(v)` | `P{v},1R` | increment fluid level only for non-air aspirate |
| `QueryStatus()` | `Q1` on report ID | asynchronous/query error-vector behavior must be trace reconciled |
| `QueryErrorLog(n)` | bytes `Q : <raw-byte n> 1` on report ID | returns first reply byte; source initializes the null-reply result to `0` |
| `ExecuteDiagnoses(n)` | `d<ASCII digit>R` | waits 1500 ms; returns stored diagnosis string |
| `QueryTipStatus()` | `?31` on report ID | non-null response returns 1 only when `array[2]` is ASCII `1`; non-null other bytes and null both set cached tip false and return 2; short non-null data can throw |
| `QueryFirmware(n)` | `&<ASCII digit>` on report ID | returns reply string or null |
| `QueryPressure()` | `?57` on report ID | converts reply substring after two-byte prefix to `double` |
| `startFluidDetection()` | `BR` | command category `Detect_Fluid_Level` |
| pressure setup | `b15R` | sent before enabling stream |
| pressure stream on/off | `o0,1R` / `o0,0R` | command category `Set_Pressure_stream` |
| heartbeat enable/disable | `U60R` / `U61R` | command category `Set_Heartbeat` |

The diagnostic `getData()` report family is also mandatory: `?40`, `?41`, `?42`, `?44`, `?45`, `?47`, `?51`, `?52`, `?53`, `?54`, `?55`, `?58`, and `?59`, with their pinned labels/units.

The per-channel denominator additionally includes `SendCommand1`, both `SendCommand` overloads, `getData`, `initiate`, `initiateGroup`, `DispenseAll`, composite `Mix`, `ejectTip`, `terminateCommand`, `setTopSpeed`, `Dispense`, `DispenseAir`, `AspirateAir`, `Aspirate`, `QueryStatus`, `QueryErrorLog`, `ExecuteDiagnoses`, `QueryTipStatus`, `QueryFirmware`, `QueryPressure`, `startFluidDetection`, `processMessage`, `GetErrorCode`, pressure-stream setup, heartbeat, and error logging. Each method requires literal payload, address family, completion class, parser, state mutation, and error behavior.

There is no OEM `ClassPipette.setVolume` method in this binary. Volume is embedded directly in `P...` and `D...` commands. A “volume setter/profile setter” claim is invalid unless bound to different pinned OEM evidence.

### 6.3.1 Literal volume encoding and local state mutation

For per-channel `Aspirate` and `Dispense`, OEM rounds volumes at or above 100 to the nearest integer, otherwise formats as `F0` or `F1` depending on whether a fractional value remains, then builds `[P|D, ASCII-volume, ',', '1', 'R']`. Payloads longer than the single-frame limit use the first/middle/report families, subject to the unresolved source/binary chunk-index discrepancies above. `setTopSpeed` uses the analogous `V{velocity},1R` form and updates cached speed after the send. The source mutates local fluid/air state after the send even when the lower send result is null/error: aspirate adds fluid, dispense updates fluid/front-air/rear-air according to `dispenseType`, `DispenseAll` clears fluid and both air levels, and `AspirateAir` updates only the selected front/rear air compartment. `AspirateAir` alone returns `-1` without TX when the channel is not initialized. These are OEM-local-state facts, not controller or physical-effect evidence; Linux must retain a separate semantic receipt and fail-closed validity. `QueryPressure()` directly parses the reply after its two-byte prefix; null, short, or nonnumeric replies can throw or otherwise invalidate the result. `QueryFirmware()` returns null on a null response but short malformed data is not a source-validated success case. The `getData()` query family likewise requires per-query fixture parsers and labels; a transport reply alone is not a valid diagnostic/result artifact.

### 6.4 Message-state semantics

`processMessage(dlc,msg)` shall be represented explicitly, including these literal behaviors:

- `msg[2]` becomes the channel error code;
- `0x20` marks initialized true;
- DLC 2 completion signals only the enumerated command categories;
- fluid-detection completion records a timestamp;
- query-error frames mutate the error queue, with `0x40` clearing it;
- query-tip status updates cached tip state from ASCII `1` at the OEM offset;
- diagnosis frames update the diagnosis string;
- `TIP_LOST` during `Eject_Tip` is suppressed in the returned event error but remains the channel error field.

The Q1 synchronous/asynchronous routing is not source-closed: `QueryStatus()` clears the queue, sends `Q1`, and discards the synchronous return; the shown `processMessage()` path is what fills the queue, while `checkedPipetteStatus()` checks only the queue head through `GetErrorCode()`. A null/no-reply or cleared queue can therefore pass the OEM aggregate check. Exact IL/binary or authoritative trace fixtures are required before claiming Q1 semantic parity. A Linux reply-presence/validity check and fail-closed parser are `SAFETY-HARDENING`.

## 7. Complete collection contract

The collection owns exactly four channels, one wait handle per channel, tip selection/state, tip type, pressure operations, and application event forwarding.

The denominator includes:

- `initiateGroup`, `reinitializePipette`, `checkedPipetteCondition`, `checkedPipetteStatus`;
- `SetTopSpeed`;
- all-channel and subset `Aspirate`/`Dispense`;
- `AspirateAir`, `DispenseAir`, `DispenseAll`, `mixAll`;
- `readPressure`, `ReadPressure`, pressure-stream control;
- `ejectAllTips`, `ejectTip`, `KeepTip`, `verifyEjectTip`;
- `queryTipStatus`, `queryIndividualTipStatus`;
- `terminatecommands`;
- `detectfluid`, `getFluidTimeStamp`;
- `loadTip` tip-type state;
- diagnostics, firmware/data retrieval, error-log access, heartbeat disable; and
- `waitforcompletion` and error logging.

The collection denominator also includes every overload of `Aspirate`, `Dispense`, and `DispenseAll`; `SendCommand1`; pressure enable/read and offset; diagnostics/error/info/data methods; both initialization methods; status/condition/speed; air and liquid routing; tip/ejection/keep/verify; termination; aggregate and individual tip queries; fluid detection/timestamps; heartbeat disable; host-only `loadTip`; apartment-dependent waiting; and all application event/error forwarding. Collection rows must preserve selection predicates, timeout formula, `AllowtoStop`, ignored return values, partial-target behavior, and host-state mutations.

### PIP-009 — `initiateGroup()` literal flow

For each channel, only when prior `CommandCompleted` is true:

1. set it false;
2. reset that channel wait handle;
3. call per-channel `initiate(false)`, which may wake if channel state is not initialized, then always sleeps 100 ms and sends `WR`.

After the loop OEM:

1. calls `waitforcompletion("Reinitialize pipette",10000)` and ignores its Boolean;
2. enables pressure stream serially on all four;
3. sleeps 1000 ms once after the serial enables;
4. disables pressure stream serially on all four;
5. calculates pressure offsets.

This is not one simultaneous, exactly 1.000-second four-channel epoch: channel windows are staggered and may differ. A Linux epoch ID, per-channel sample attribution, and stale-sample exclusion are `SAFETY-HARDENING`. A Linux fail-closed return on a missing immediate response or completion is also stricter than OEM and must be labeled accordingly.

### PIP-010 — `reinitializePipette()` is separate

This path:

1. abort-checks `ControlLib.forceabort`;
2. sends direct group `WR` on all four without wake;
3. waits 10 seconds;
4. requires every channel `ErrorCode == 32` for true;
5. resets collection tip type to enum value 201 (`UNKNOWN`).

It must not be merged with `initiateGroup()`.

### PIP-011 — Status/condition checks

`checkedPipetteCondition()` calls `QueryFirmware(1)` on every channel and throws on null.

`checkedPipetteStatus()`:

1. throws immediately when `ControlLib.forceabort` is true;
2. calls `QueryStatus()` on channels 0..3 with a 30 ms sleep after every call;
3. sleeps another 1 ms;
4. logs every channel whose `GetErrorCode()` is nonzero and returns false if any is nonzero.

The source does not visibly validate reply presence before this queue-head check; an empty/cleared queue can pass. Linux semantic strengthening is allowed but must retain the literal source result, `reply_received`, and replacement safety validity separately.

### PIP-012 — Tip status and ejection

`queryTipStatus(-1)` performs four hardware `?31` reads and returns the count of channel results equal to per-channel OEM return value 1. A single-channel query returns 1 only for loaded, otherwise 0; per-channel null/non-`1` replies are collapsed to the OEM no-tip result 2 before the collection maps them to 0. `queryIndividualTipStatus()` returns four Booleans. This collapse is an OEM fact, not a validity proof; Linux must retain independent `reply_received`/`validity` fields.

`ejectAllTips(checkMissingTip,wait)`:

- derives the first-pass candidate array from `MachineStatus.TipLocation` or all four channels;
- performs a fresh aggregate hardware tip readback;
- when `TipLocation==-1` and fewer than four tips are reported, or when zero tips are reported, clears candidates whose per-channel cached `TipLoaded` is not 1; a kept-tip/single-location path with a nonzero total count does not perform that same per-channel suppression, and a selected tipless target can still be commanded;
- sends `ejectTip()` to the remaining candidate channels without proving that the candidate set equals the freshly loaded set;
- optionally waits 8000 ms, but does not honor the returned Boolean;
- when `wait=false`, sleeps 500 ms;
- resets `m_TipType` to enum value 201 during the waiting path, calls `verifyEjectTip()`, and sets `TipLocation=-1`.

`verifyEjectTip()` performs hardware readback. In all-tip mode, if any tip remains, OEM sends a second ejection pass to all four, waits 6000 ms while ignoring the returned Boolean, re-queries, unlocks the door, and throws if any remain. Keep-one-tip mode has different targeting and verification: `KeepTip(tip)` queries, ejects every channel except the retained index, waits 6000 ms, verifies the retained channel, and sets `TipLocation` to the retained channel.

Startup stale-tip rectification must preserve `ejectAllTips(false,true)` call arguments and the literal two-pass behavior. Any operator authorization, exact-set equality, per-channel ACK gate, or invalid-readback fail-stop rule is a Linux safety intercept and must not be represented as literal OEM aggregate behavior.

### PIP-013 — Liquid routing and state

All-channel/subset selection, `m_PipetteHasTip`, `TipLocation`, `m_TipType`, `TipLoaded`, `TipDirty`, fluid name/level, current location/well, `AllowtoStop`, speed, and pressure policy are authoritative dependencies.

Important literal behaviors include:

- all-channel and subset `Aspirate`/`Dispense` use different selection and speed paths; subset overloads set a requested speed first, while ordinary all-channel overloads use cached channel speed;
- liquid operations compute bounded wait estimates from volume and selected/current speed, set `AllowtoStop=false`, and often sleep another 1000 ms and return after an ignored completion timeout rather than throw;
- pressure-stream placement differs: aspirate enables pressure per channel with 200 ms sleeps, while ordinary dispense enables the stream once before the channel loop; stream disable follows the collection wait;
- lost-tip checks use fresh aggregate `?31` counts plus cached `TipLoaded`, may raise `TIP_LOST` through the collection callback, and do not create a universal per-channel validity model;
- air operations use the same `P`/`D` command families but different host-fluid bookkeeping and `frontAir`/rear-air dispense types;
- `DispenseAll` sends `A0R` to channels marked as having tips and the per-channel method clears fluid/front-air/rear-air state;
- `mixAll(count,vol,vigrous)` ignores `vol`, derives volume from `m_TipType`, scales/restores each channel’s speed, calls the composite per-channel `Mix`, and sleeps 100 ms between channels;
- `loadTip()` only updates tip-type host state; physical loading is gantry/deck motion plus hardware readback, not a pipette actuator command.

Arbitrary Linux pressure profiles, `air_gap_ul`, and `blow_out` are not OEM-backed by this class surface. They must be rejected, mapped to exact OEM workflow operations, or classified as product extensions. Merely returning `blow_out=true` without sending `A0R` is not implementation.

### PIP-014 — Fluid detection and terminate

`detectfluid()` starts `BR` on all four; the pinned source resets wait handles after command transmission when `wait=true`, creating a literal race. The application moves Z while waiting up to 15 seconds; on completion it stops Z and reads timestamps/position. On timeout it calls `terminatecommands()` and throws.

Linux shall register completion owners before transmission as `SAFETY-HARDENING`, while preserving the OEM command, timestamps, Z-motion coupling, timeout, terminate, and error result.

### PIP-015 — Abort and heartbeat

`ControlLib.forceabort` checks and user-stop exceptions are part of OEM collection admission. `TR` is a real per-channel stop primitive and must be available independently of HTTP task cancellation. `ClassPipetteCollection.pipetteError(channel,code)` and `ControlLib.errorEvent(string)` are distinct callback channels: the application binds the latter before startup, while the collection binds the former only after successful normal initialization. Shutdown/application call sites that disable heartbeat must be inventoried and reproduced or explicitly retired with evidence.

## 8. Application lifecycle contracts

### 8.0 Application/startup aggregate authority

`ControlLib` is not the complete startup owner. `BioXPMainWindow` constructs the aggregate, starts the serialized motion worker, subscribes `ControlLib.errorEvent` before startup admission, calls `initializeEnvironment`, gates startup on CAN/door/latch state, queues `initializeSystem`, consumes that queue command, and writes terminal status after the method returns. Its early-return and exception paths include in-motion, ShipMode PARK, unexpected saved-status, and caught/suppressed initialization branches. A normal `initializeSystem()` return is therefore not proof that every initialization stage ran or succeeded.

`ClassPipetteCollection` construction is passive with respect to hardware: it stores dependencies, creates four `ClassPipette` instances, subscribes the CAN receive handler, and configures (but does not start) its timer. The later `ControlLib` constructor-owned CAN-ready branch is the hardware-commanding aggregate initialization and must not be called collection-constructor-equivalent.

### 8.1 `ControlLib` constructor-owned startup

After CAN service readiness, `ControlLib` performs hardware commands before `initializeMotion()`:

- Board-test branch: only when the collection exists and `pipetteExist` is true, call `initiateGroup()` then `ReadPressure()`.
- Normal branch:
  1. `initiateGroup()`;
  2. `checkedPipetteCondition()` — four firmware/condition queries, throwing on a null response;
  3. first `checkedPipetteStatus()`;
  4. if false, one more `initiateGroup()` and one more `checkedPipetteStatus()`;
  5. ignore the second Boolean and continue to `initializeMotorsWithoutMotion()`.

`initializeMotorsWithoutMotion()` still performs controller/heater/chiller/LED writes. “Non-motion” describes the call-site name only; source does not establish that this branch is hardware-passive. A production Linux fail-closed constructor is allowed and preferable, but it is a named safety divergence. The receipt must carry both `oem_literal_outcome` and `admitted_safe_outcome`.

### 8.2 Exact `ControlLib.initializeMotion()` pipette branch

Source: `ControlLib.cs:8797-8856`.

The exact sequence is:

1. set `m_stopScripts=true`;
2. set `forceabort=false`;
3. call full `initializeMotors()`;
4. set host `ThermalDoorOpen=false`;
5. call `queryTipStatus(-1)`;
6. sleep 500 ms;
7. branch on `TipExist`.

If no tip exists:

8. set host `TipLoaded=false` (which also invokes the machine-status gantry-load clear); do not clear `TipDirty`;
9. exit successfully; **do not call `initiateGroup()` here**.

If any tip exists:

8. call `openThermalDoor()` and ignore its Boolean return (the primitive can return false on serials above 9 when open/closed sensors disagree);
9. unconditionally set host `ThermalDoorOpen=true`;
10. set host `TipLoaded=true` (gantry-load state mutation);
11. call the exact `scriptmoveTo` route for location 28/well 0 to location 6/column 0/row 0;
12. update host location to location 6, well 0;
13. call `ejectAllTips(false,true)`; this includes its fresh query, ignored 8-second wait, tip-type reset, verification, conditional all-four second pass, ignored 6-second wait, and `TipLocation=-1` side effect;
14. call OEM `moveZ(80000)` and `moveX(79000)` wrappers, including board-null guards, clamps, Z-current mutation, and default wait-for-stop behavior;
15. call `queryTipStatus(-1)`;
16. sleep 100 ms;
17. if any tip remains: set `m_pauseScripts=true`, emit inner error event `Eject tip failed`, and throw when the event is bound; otherwise return from the inner branch if no event is bound. If the callback itself throws, the outer catch path applies.
18. set host `TipDirty=false`;
19. set host `TipLoaded=false`;
20. sleep 2 ms;
21. call `initiateGroup()`;
22. call `checkedPipetteStatus()`;
23. only if false, call one additional `initiateGroup()` and one additional `checkedPipetteStatus()`;
24. if the retry status is false, invoke `errorEvent("Eject tip failed")` without a null guard, then throw `Eject tip failed`;
25. outer catch emits `errorEvent(ex.Message)` and rethrows with the OEM `throw ex` behavior only when the event is bound; with no delegate bound, the outer catch swallows the exception and returns. With a normal handler, the explicit failure and outer catch can emit the same failure twice.

The `scriptmoveTo` call is not a generic location label. Exact parity requires the resolved overload/IL and all transitive route predicates: `PositionTable`, X/Y increments, pseudo-Z home, current XYZ, `TipLoaded`, `TipDirty`, `TipLocation`, gripper confirmation, `cleanPath`, midpoint routing, `DeviceType`, `XHighLimit`, location-19 Y, default position/run-parallel arguments, and `updateLocation` side effects. The decompiled overload selection remains a pinned-IL resolution item.

The final `moveZ(80000)`/`moveX(79000)` calls are wrappers, not raw writes: X clamps values at or below 60 and no-ops if the X board is absent; Z clamps below `PSUDO_Z_HOME`, mutates the Z current position, and no-ops if the Z board is absent; both default to blocking wait-for-stop.

There are at most two group/status method-call attempts in this method, and only in the stale-tip branch. Because `initiateGroup()` dispatch is guarded by each channel’s pre-existing `CommandCompleted` state, this does not prove two complete four-channel physical attempts. Retry counts from `initPipette()` or diagnostic UI must not be imported.

This section closes only the pipette subgraph. Full `initializeMotion()` also unconditionally calls `initializeMotors()`; its motor, door, chiller, gripper, UI, status, and configuration predicates require separate transitive source/runtime closure before full-method parity can be claimed.

The current Linux provider’s literal acknowledgement mismatch is a blocker: it invokes startup ejection with `operator_ack="EJECT"`, while the transport accepts only `EJECT_STALE_STARTUP_TIPS`.

### 8.3 `initPipette()` retry loop

`ControlLib.initPipette(bool ini=false)` calls `reinitializePipette()` initially. With `ini=true`, it performs no retry and returns `0` or `-1`. Otherwise it retries while false and `num < 3`, for at most three retries and four total attempts, with error logging, tip query/ejection cleanup, and related state/motion behavior. It returns `0` on success and throws `could not initialize pipette` after final failure. This is distinct from `initializeMotion()` and must have its own row-level call-site specification before full application parity. Diagnostic UI and BoardTest callers have additional policies and must not be merged into this loop.

### 8.4 Full application call-site closure

Full parity requires an inventory of every `m_PipetteControl` call in `ControlLib.cs`, including:

- constructor and board-test startup;
- initialization and recovery;
- script interpreter liquid commands;
- air handling, purge, mix, split-volume, calibration, and fluid-level detection;
- tip loading, kept-tip routing, tip cleanup loops, and abort cleanup;
- diagnostics and data export;
- heartbeat/shutdown; and
- every machine-status/location/fluid update coupled to those calls.

A per-channel driver plus `/liquid/*` routes is not full OEM application parity.

## 9. Current Linux/BMS baseline

This section records source-level implementation status only. It is not deployment or hardware evidence.

### 9.1 Confirmed represented behavior

Current source contains:

- one shared `BioXpTester`/`NovoRouter` ownership path;
- four `CanPipetteTransport` channels under `FourPipetteTransport`;
- raw Novo framing/escaping/additive checksum generation;
- stricter checksum, delimiter, and DLC validation;
- normal 20-ID synchronous pipette receive classification plus separate pressure-stream routing;
- channel/function semantic matching;
- delayed initialization completion registration;
- `WR`, `Q1`, `&1`, `?31`, `?57`, `b15R`, pressure stream on/off, `E1R`, `P...`, and `D...` source paths;
- four-channel group/condition/status primitives;
- pressure epoch → serial stream enable → one 1000 ms sleep → serial stream disable → offset order;
- four-channel hardware tip readback;
- a startup-only ejection path with preauthorization and post-readback;
- a durable staged `initializeMotion` provider whose no-tip branch exits without group init;
- `/liquid/status`, `/liquid/init`, `/liquid/tip`, `/liquid/aspirate`, `/liquid/dispense`, and `/liquid/mix` routes;
- generic operator catalog/UI exposure; and
- a quick-dashboard pipette status surface.

### 9.2 Confirmed source-level deviations/gaps

Current source also shows:

- wake uses the special global-ID TX path and does not capture a correlated device ACK;
- group initialization requires an immediate response and uses a single shared 10-second deadline, both stricter/normalized versus literal OEM;
- multipart command transmission is absent from `BioXpCanDriver`; commands over eight bytes are rejected;
- ejection always sends `E1R`; uninitialized `E0R` is absent;
- startup ejection omits OEM second-pass behavior and has the `EJECT` versus `EJECT_STALE_STARTUP_TIPS` integration mismatch;
- `TR`, `BR`, heartbeat, diagnostic, full firmware/data, error-log, air, `A0R`, kept-tip, and complete collection methods are absent;
- Q1 error-vector handling is normalized and lacks captured-runtime reconciliation;
- API models allow unproven pressure profiles, `air_gap_ul`, and `blow_out` semantics;
- low-level aspirate/dispense/mix implementations exist, but the canonical `FourPipetteTransport` blocks all tip/liquid mutation; its mix surface is not an OEM-accepted physical workflow;
- `/liquid/status` is a cache/projection read; no live query is performed by that route;
- generic UI exposure exists, but no dedicated pipette workflow has been accepted;
- robot receipts now carry `requested_inputs` and `controller_acknowledged`, but controller-ACK detection recognizes TMCL status 100 and does not recognize pipette `ack.outcome="ack"`;
- physical-effect truth is incomplete and sometimes depends on later human assessment; and
- no live runtime, deployed SHA, physical dry-cycle, wet-cycle, commissioning, or release evidence was reviewed for this specification.

## 10. Rectification ledger

Every row is required. “Partial” does not count as complete.

| ID | Priority | Gap / required rectification | Acceptance evidence |
|---|---:|---|---|
| G-001 | P0 | Bind runtime to the exact evidence-lock identity and expose lock/config/binary/source hashes in immutable receipts. | Hash verification plus runtime identity receipt. |
| G-002 | P0 | Complete method/property/call-site denominator for `BioXPMainWindow`, `ControlLib`, `ClassPipette`, collection, Novo, constructor, `initializeMotion`, `initPipette`, shutdown, scripts, and workflows. | Machine-readable parity matrix with zero unclassified rows. |
| G-003 | P0 | Resolve the pinned multipart mask defect with binary execution or captured serial-206 traces. | Fragmented TX and RX trace fixtures tied to binary hash. |
| G-004 | P0 | Implement OEM wake TX observation/logging without inventing a correlated device ACK, and explicitly classify any stronger fail-closed gate. | Four-channel first-init traces including special-ID wake path and `WR`. |
| G-005 | P0 | Resolve collection completion policy against actual OEM apartment state; stop calling one shared deadline universally exact. | Thread/apartment evidence and timeout tests/receipts. |
| G-006 | P0 | Reconcile immediate versus delayed response semantics, special global-ID wake handling, mutable-command completion ownership, and per-command completion owners. | Command-family transaction/completion matrix and traces. |
| G-007 | P0 | Reconcile Q1 synchronous/asynchronous routing, queue offsets, `0x40` clear semantics, stale-error behavior, reply presence, and the evidence deficit between synchronous return bytes and `processMessage()`. | Pinned Q1 traces for clean and each error class. |
| G-008 | P0 | Correct application-owner, constructor, BoardTest, diagnostic-init, `initializeMotion`, and `initPipette` call-site semantics without merged retries; include callback subscription/early-return/suppression behavior. | Lifecycle receipts proving each distinct path. |
| G-009 | P0 | Fix startup-ejection acknowledgment mismatch; implement OEM `TipLocation` targeting quirks, ignored 8/6-second waits, second-pass all-four verification, `E0R/E1R`, exact `ejectAllTips(false,true)` behavior, and host-state updates. Keep authorization-set equality and invalid-readback gates classified as Linux safety intercepts. | Empty/partial/full/kept-tip trace matrix and post-readback. |
| G-010 | P0 | Implement first-class terminate `TR`, abort propagation, timeout termination, and latched ambiguous-outcome handling. | Stop/abort/timeout fault-injection evidence. |
| G-011 | P0 | Prove one deployed canonical service owns USB/router/pipettes and no competing reader or SocketCAN fallback exists. | Process/listener/USB owner/router/source-SHA receipt. |
| G-012 | P0 | Preserve wet mutation disabled until dry acceptance, deck/reference/interlock checks, and separate wet authorization. | Admission evidence and rejected premature requests. |
| G-013 | P1 | Complete pressure stream setup, separate four-ID pressure routing, serial enable/disable timing, epoch isolation, sample attribution, offset calculation, serial-206 pressurebuffer identity, and malformed pressure handling. | Four-channel pressure artifacts and tolerance acceptance. |
| G-014 | P1 | Implement diagnostics, `Q:<n>1`, `d<n>R`, firmware `&n`, all `getData()` queries, exact null/short parser behavior, and error/result surfaces. | Offline fixtures plus read-only live receipts. |
| G-015 | P1 | Implement heartbeat enable/disable and shutdown call-site behavior. | Lifecycle and reconnect evidence. |
| G-016 | P1 | Implement fluid detection `BR`, reset-after-send race classification, timestamps, DLC-dependent completion, Z coupling, 15-second timeout, four-channel `TR`, and fail-closed race hardening. | Dry detection simulation then supervised physical evidence. |
| G-017 | P1 | Implement exact speed, subset/all-channel, `AllowtoStop`, pressure placement, timeout formulas, air, `A0R`, composite mix, tip-type, kept-tip, and host fluid-state semantics. | Method-level fixtures and staged physical acceptance. |
| G-018 | P1 | Reject or explicitly classify arbitrary pressure profiles, `air_gap_ul`, `blow_out`, and product metadata not mapped to OEM behavior. | API schema/admission matrix. |
| G-019 | P1 | Make `/liquid/status` and dashboard/UI visibly label cache, live query, controller ACK, completion, and physical-effect truth separately. | Contract snapshots and UI acceptance. |
| G-020 | P1 | Make pipette ACKs set `controller_acknowledged` only from semantic pipette evidence; preserve requested versus effective inputs and stage receipts. | Robot/BMS schema round trip. |
| G-021 | P1 | Add dedicated operator-safe pipette controls or formally accept generic catalog UX with channel/state/precondition clarity. | Browser acceptance and receipt linkage. |
| G-022 | P1 | Implement the complete robot-authoritative runtime audit, migration, retention, reporting, BMS relay, and cockpit contract in `docs/specs/2026-08-20-bioxp-runtime-audit-storage-reporting-spec.md`. This includes a durable claim before every transport-producing pipette entrypoint, one canonical `bioxp_runtime.db`, a generated closed-world OEM/robot/protocol/lifecycle/operator/BMS/UI denominator, typed command/pipette/channel/CAN/event/pressure identities, indefinite compact metadata, five-year full evidence, backup-first migration from JSONL, crash-safe evidence lifecycle, bounded snapshot-consistent reports, governed exports, and retirement of the unused workstation command database. | Every RA-0 through RA-14 gate and RAQ-001 through RAQ-017 in the runtime audit specification. Source-only implementation does not satisfy this row. |
| G-023 | P2 | Execute staged read-only shadow, dry no-liquid, stale-tip, air, and separately authorized wet commissioning. | Signed acceptance packet for each stage. |
| G-024 | P2 | Prove release branch, deployed SHA, managed service ownership, BMS/API/frontend compatibility, and rollback readiness. | Release evidence bundle and independent review. |

## 11. Ordered work packages

Work packages are sequential unless an explicit dependency review allows parallel offline work.

### WP0 — Authority and denominator closure

- freeze exact binary/source/config hashes;
- generate full method, property, command, state, and call-site matrix;
- classify every literal defect and proposed divergence;
- resolve or explicitly block unresolved source/IL claims.

Exit: G-001 and G-002 complete; every later row has exact source anchors.

### WP1 — Transport and trace-equivalence layer

- raw framing fixtures;
- valid and malformed parser fixtures;
- normal 20-ID synchronous routing plus separate four-ID pressure routing;
- channel/function correlation;
- immediate/delayed completion and special wake path;
- multipart trace resolution;
- timeout/apartment policy;
- shared ownership and reconnect semantics.

Exit: G-003–G-007 and G-011 complete offline; no hardware mutation required.

### WP2 — Complete per-channel and collection surface

- command/state/error families;
- init/reinit/status/pressure;
- tip/eject/keep/verify;
- terminate/heartbeat/diagnostics/data;
- fluid detection;
- speed/air/liquid/mix/subset routing;
- host-state synchronization.

Exit: G-009–G-018 source-complete with fixtures; wet paths remain admission-blocked.

### WP3 — Application lifecycle integration

- constructor and BoardTest branches;
- exact application-owner/startup gates and callback subscription order;
- exact stale-tip/no-tip `initializeMotion` branches;
- exact scriptmove/motion-wrapper dependency closure;
- `initPipette` retry loop;
- script/workflow call sites;
- abort, cleanup, rehome, and persistent receipts governed by the runtime audit storage specification.

Exit: G-008, G-010, G-016, G-017, and G-022 integrated without parity overclaims.

### WP4 — API, BMS, and operator plane

- route schemas and admissions;
- cache/live/ACK/completion/physical truth;
- pipette controller acknowledgement;
- requested/effective inputs;
- robot-authoritative report APIs, strict BMS relay, cockpit filters/detail/export, and durable history governed by the runtime audit storage specification.

Exit: G-019–G-021 complete in development runtime. G-022 remains open until every applicable RA gate passes.

### WP5 — Read-only shadow and dry commissioning

- exact runtime/deployed-source identity;
- passive four-channel RX/queries;
- constructor-owned hardware path with explicit `oem_literal_outcome` versus `admitted_safe_outcome`;
- pressure and diagnostic reads;
- supervised no-liquid/stale-tip/air operations only when separately authorized.

Exit: all read-only and dry acceptance evidence complete; wet remains disabled.

### WP6 — Wet commissioning

Requires separate explicit authorization. Establish liquid-specific volumes, speeds, tips, deck locations, disposal, containment, stop conditions, and acceptance tolerances before any wet command.

Exit: signed wet acceptance or explicit rejection. No automatic progression.

### WP7 — Release

- independent review;
- exact branch/SHA promotion authorization;
- managed deployment;
- API/BMS/frontend compatibility;
- rollback drill and post-deploy evidence.

Exit: G-024 complete and release owner acceptance recorded.

## 12. Safety and admission

Before any pipette TX, the authoritative gate shall prove as applicable:

- exact serial-206 machine/evidence identity;
- one current ownership generation;
- shared USB/router healthy and uncontested;
- request freshness and idempotency;
- channel set and command family valid;
- required completion owner registered;
- controller/reference/interlock state appropriate;
- exact tip hardware state from `?31` **and independent reply validity**;
- deck/location/well and host machine state coherent;
- no active conflicting motion/liquid command;
- abort/terminate path available;
- operator authorization literal and scope valid;
- liquid mutation class permitted by commissioning state.

A stale authorization, generation change, malformed reply, unknown completion, process timeout, or restart during an admitted physical stage fails closed and latches ambiguity. It may not be retried automatically.

Wet aspirate, dispense, mix, fluid detection, purge, and pressure-based liquid inference remain disabled until WP6 authorization and evidence.

## 13. Required verification matrix

Implementation verification must include, at minimum:

### Offline/static

- source-hash and evidence-lock checks;
- CAN ID formulas for channels 0..3;
- raw encode/decode golden vectors, escaped checksum vectors, malformed vectors;
- all 20 normal synchronous RX IDs, separate four pressure-stream IDs, and wrong-channel/wrong-function rejection;
- immediate/delayed completion ordering and special wake behavior;
- all timeout and apartment policies;
- fragment TX/RX vectors from observed evidence;
- every command literal and query parser;
- complete error enum, Q1 queue cases, reply-presence evidence, and the unresolved synchronous-return/`processMessage()` relationship;
- no-tip/stale-tip/full/partial/kept-tip branches;
- constructor versus `initializeMotion` versus `initPipette` retries;
- pressure epoch isolation;
- abort/terminate/restart ambiguity;
- API/receipt/schema round trips;
- durable claim before transport, failure and ambiguity persistence, exact operator-to-pipette-to-CAN correlation, versioned migration, five-year evidence lifecycle, backup/restore, one-snapshot reports, strict BMS models, cockpit drill-down, and governed export under the runtime audit storage specification;
- cache/live/physical truth labels.

### Read-only live

- deployed SHA and owner identity;
- exactly one USB reader/router;
- four-channel firmware, Q1, `?31`, `?57`, and diagnostic reads;
- no unexpected TX outside the authorized query set;
- artifact persistence and BMS/UI display.

### Physical dry

Only under separate watched authorization:

- initialization and delayed completion;
- pressure stream/offset;
- selected tip ejection with post-readback and retry behavior;
- terminate/abort under bounded conditions;
- air-only operations if explicitly admitted;
- recovery/rehome and persistent host-state coherence.

### Wet

Only under WP6 authorization:

- bounded single-channel first liquid operation;
- calibrated volume/tolerance checks;
- all-channel and subset routing;
- split-volume/air-gap/mix workflows only after exact mapping;
- contamination/disposal controls;
- stop/abort/recovery;
- final commissioning packet.

## 14. Acceptance ladder

| Gate | Acceptance condition |
|---|---|
| A0 | Specification hash, evidence lock, binaries, sources, and machine artifacts frozen. |
| A1 | Complete source/IL/method/call-site parity matrix; no unclassified row. |
| A2 | Offline transport and parser fixtures pass, including observed multipart evidence. |
| A3 | Complete per-channel and collection source surface with mutation still blocked. |
| A4 | Exact constructor/`initializeMotion`/`initPipette` integration and durable failure semantics, including a durable claim before pipette side effects. |
| A5 | API/BMS/receipt/UI contracts and all development-applicable RA gates accepted in development. |
| A6 | Deployed runtime identity and single-owner proof. |
| A7 | Read-only live shadow accepted on all four channels. |
| A8 | Separately authorized dry physical acceptance completed. |
| A9 | Separately authorized wet commissioning completed, if wet release is in scope. |
| A10 | Independent regression/safety review accepted. |
| A11 | Release owner authorizes promotion/deployment and post-deploy evidence passes. |

A later gate cannot compensate for a failed earlier gate. A test passing against mocks cannot satisfy a live or physical gate.

## 15. Completion denominator and reporting

The completion denominator consists of:

1. all 24 gap rows;
2. every method/property/command row from the pinned per-channel and collection classes;
3. every pipette application call site;
4. every accepted intentional divergence;
5. all A0–A11 gates in release scope.

Each denominator row must record:

- ID and exact source/binary/config anchor;
- OEM-literal behavior;
- observed runtime behavior, if any;
- Linux implementation path and commit;
- classification;
- test/trace/artifact IDs;
- deployment identity;
- physical acceptance state;
- reviewer and date;
- blocker/defer reason.

Allowed progress labels are:

- `not_started`;
- `source_implemented`;
- `offline_verified`;
- `runtime_bound`;
- `live_read_only_accepted`;
- `physical_dry_accepted`;
- `wet_commissioned`;
- `release_accepted`;
- `blocked`;
- `deferred_out_of_scope`.

No row may skip levels or borrow evidence from a different subsystem, binary version, machine, worktree, service, or Z-axis audit.

## 16. Final specification verdict

This document is **final as the source-vetted OEM rectification specification and release contract**. `docs/specs/2026-08-20-bioxp-runtime-audit-storage-reporting-spec.md` is the controlling companion for G-022 storage and reporting work. Independent review confirms that the recovered source is sufficiently bounded to specify the work, but rejects the artifact as an implementation-ready exact OEM contract until the open source, IL, runtime, and RA rows close.

The current implementation is **not** declared full OEM parity, audit-complete, physically accepted, commissioned, or release-ready. Material blockers include the application-owner and constructor call-site closure, unresolved binary/runtime fragment semantics, unresolved Q1 synchronous/asynchronous error-vector correlation, incomplete command and collection surfaces, literal-versus-hardened lifecycle differences, the startup-ejection acknowledgement mismatch, incomplete controller-ack truth, unresolved runtime-audit RA gates, no deployed-runtime proof, and no physical acceptance evidence.

The next permitted implementation tranche is WP0 unless Christian explicitly authorizes a different bounded work package. Finalization of this specification authorizes no code changes, tests, USB/CAN commands, pipette actions, robot motion, deployment, commit, promotion, or commissioning.
