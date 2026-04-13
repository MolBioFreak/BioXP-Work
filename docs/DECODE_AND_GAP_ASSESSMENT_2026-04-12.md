# BioXP TMCL Decode + OEM Gap Assessment (2026-04-12)

## What changed today

### Decoder improvements landed
Updated `scripts/decode_tmcl_frame.py` to add OEM thermal/chiller fault status coverage and a missing motion parameter label.

Added TMCL statuses:
- `8` = temperature above maximum allowed
- `9` = temperature below minimum allowed
- `10` = nest RTD delta greater than allowed
- `11` = current exceeded maximum allowed
- `12` = temperature diverged from reference
- `13` = voltage drop
- `14` = max current reference deviation exceeded
- `133` = RTD failure

Added motor parameter:
- `1` = `ACTUAL_POSITION`

Quick verification:
- `READ_SENSOR_TEMPERATURE` reply with status `133` now decodes as `RTD failure`
- `GAP` request with type `1` now decodes as `ACTUAL_POSITION`

## Files inspected

### Current Linux/runtime surfaces
- `scripts/decode_tmcl_frame.py`
- `src/bioxp/usb_driver.py`
- `src/bioxp/api.py`
- `docs/TMCL_VENDOR_CORE_CONTROL_MAP.md`
- `docs/REVERSE_ENGINEERING_TRACEABILITY.md`
- `docs/PROGRESS_2026-03-02.md`
- `docs/ARCHITECTURE_AND_CONTROL_PLANE.md`
- `docs/MODULE_REFERENCE.md`

### OEM assets
- `.deploy/Application Files/GenBotApp_6_3_0_1/*`
- `decompiled_src/BioXPControlLib/ClassControlInterface.cs`
- `decompiled_src/BioXPControlLib/ControlLib.cs`
- `decompiled_src/BioXPControlLib/ClassPipetteCollection.cs`
- `decompiled_src_can/BioXPControlLib/ClassPipette.cs`
- `decompiled_src_can/ClassCanLib/StatusCode.cs`
- `decompiled_src_vision/CVisionLib/ClassFrameGrabber.cs`
- `decompiled_src_bioxpcommon/BioXPCommonLib/ClassBioXPScriptHandler.cs`
- `Scripts/demo.xml`
- `Scripts/lifetest.xml`

### BMS/operator surfaces
- `/home/dalab/biomodstack/biomodstack/platform/api/routers/bioxp.py`
- `/home/dalab/biomodstack/biomodstack/platform/frontend/src/lib/bioxpClient.ts`
- `/home/dalab/biomodstack/biomodstack/platform/frontend/src/components/BioXpCockpit.tsx`

## Current TMCL decoder status

### What the decoder already does well
The current decoder is good at single-frame structural decode:
- request vs reply/event framing
- checksum validation
- board IDs
- command names
- motion axes / thermal targets / chiller banks / sensor axes
- deck IO query types
- LED mask/channel semantics
- thermal/chiller GP parameter labels
- basic reply status interpretation

It already correctly labels important BioXP-specific commands like:
- motion (`MVP`, `SAP`, `GAP`, `RFS`)
- IO (`SIO`, `GIO`)
- LED (`SET_LED`)
- board lifecycle (`ACTIVATE_BOARD`)
- thermal/chiller (`SET_TARGET_TEMPERATURE`, `SET_HEATSINK_FAN`, `READ_SENSOR_TEMPERATURE`, `SET_PWM_OVERRIDE`, `DUMP_TC_HISTORY`)
- firmware query (`QUERY_FIRMWARE_VERSION`)

### Highest-value decode gaps still open
1. The decoder is still a single-frame decoder, not a transaction decoder.
   - TMCL reply frames do not carry enough context to fully interpret many replies by themselves.
   - The next useful step is a request/reply correlating mode so replies can inherit the originating request’s `type`, motor/bank context, and parameter semantics.

2. There is still no committed real trace corpus in the repo.
   - No saved USB/TMCL session logs
   - No pcap/usbmon dump set
   - No committed observed request/reply JSON corpus

3. `DUMP_TC_HISTORY` is only identified, not semantically decoded.

4. Some lower-priority commands are still named but not meaningfully interpreted:
   - `STAP`, `RSAP`, `STGP`, `RSGP`
   - `SCO`, `GCO`, `CCO`, `AAP`

### Highest-value next decode targets
1. Build request/reply correlation into the decoder.
2. Capture real thermal/chiller fault traffic and decode statuses `8-14` and `133` in context.
3. Capture and decode `cmd 153` thermal history traffic.
4. Save a reusable real-world TMCL trace corpus into the repo.

## Bottom-line OEM parity estimate

### Practical estimate right now
Given the current state of motion bring-up and the fact that physical truth is still only partly proven:
- about `15-25%` usable OEM parity overall in practical terms

### More generous low-level estimate if you count raw primitive coverage only
If ignoring reliability and semantic layers and just counting exposed low-level control primitives:
- about `40-50%` of low-level board-control surface

### Why the practical number stays low
The current Linux stack has meaningful progress in:
- USB/TMCL transport
- board activation/status
- raw motion primitives
- latch/interlock control
- LED control
- thermal/chiller primitives
- generic camera device/snapshot/stream handling

But OEM parity is still far away because the missing problem is no longer mainly “can we send a board command?”
The missing problem is:
- unstable / not fully proven low-level execution
- missing semantic middle layer
- missing pipette subsystem
- missing vision semantics
- missing job/protocol engine

## OEM has major semantic layers that Linux still lacks

### 1. Script/job orchestration layer
OEM clearly has a real workflow engine, not just a GUI.
Evidence:
- `ClassBioXPScriptHandler.cs` loads/interprets XML jobs
- `demo.xml` and `lifetest.xml` contain high-level verbs like:
  - `PP`, `TCD`, `MC`, `MP`, `LED`, `WAIT`, `DELAYPOINT`
  - `CC`, `SS`, `FP`, `MT`, `DB`, `ET`, `SP`, `DWELL`, `LOOP`, `LA`

Current Linux runtime has no equivalent XML/job interpreter in `src/bioxp`.

### 2. Deck/location/well/material semantics
OEM motion is location-aware and well-aware.
Evidence:
- `PositionTable`
- `locationID`
- `wellID`
- `scriptmoveTo(locationID, wellID, ...)`

Current Linux runtime is still mostly expressed as:
- axis names (`x`, `y`, `z`, `g`, `door`)
- thermal banks (`nest`, `lid`, `pedestal`)
- chiller banks (`rc`, `oc`)

That means the semantic deck model is still missing.

### 3. Pipette/liquid-handling subsystem
OEM has a real pipette protocol and multi-pipette orchestration.
Evidence:
- `ClassPipetteCollection.cs`
- `ClassPipette.cs`
- pipette verbs for aspirate/dispense/mix/eject/detectfluid/pressure handling
- script-layer liquid-transfer options in `ClassBioXPScriptHandler.cs`

Current Linux runtime explicitly does not export this yet:
- `/liquid/aspirate` -> `501`
- `/liquid/dispense` -> `501`

### 4. Vision/inspection/barcode semantics
OEM includes a substantial vision stack:
- barcode scanning
- focus adjustment
- pool plate checks
- output plate checks
- strip-handle finding
- inspection/error semantics around tips/covers/plates/barcodes

Current Linux runtime has camera transport/control, but not OEM-equivalent semantic CV.

### 5. Integrated multi-subsystem orchestration
OEM combines:
- motion
- pipetting
- vision
- thermal/chiller programs
- consumable state
- job execution

Current Linux runtime mostly exposes primitive device-control endpoints rather than a machine-semantic orchestrator.

## OEM deployment inventory confirms more than a board-control app
The ClickOnce payload includes major assemblies for:
- BioXP control
- common/job/script layers
- CAN transport
- Novo device/runtime support
- substantial CVision stack
- native OpenCV / zbar / QR dependencies
- cloud/upload support (`AWSSDK.*`)

Decompiled roots present on disk include:
- `decompiled_src`
- `decompiled_src_bioxpcommon`
- `decompiled_src_can`
- `decompiled_src_commonlib`
- `decompiled_src_communication`
- `decompiled_src_genbotapp`
- `decompiled_src_novo`
- `decompiled_src_novodevices`
- `decompiled_src_novoutilities`
- `decompiled_src_vision`

Conclusion: this is a broad automation platform, not just motor-control code.

## BMS/operator status and immediate implications

### Good news
The current operator-facing BMS cockpit is already using the safer relative-move path by default:
- `reuse_prepared` defaults to `false`
- hold-jog explicitly sends `reuse_prepared: false`
- absolute/home paths do not expose `reuse_prepared`

### Important remaining risk
The risky fast path still exists robot-side in `src/bioxp/api.py`.
That means the operator UI is safer than the daemon’s full surface, but the daemon still permits a path that can skip parts of the fresh-prep flow.

### Test/validation reality
There is no meaningful automated test suite covering this BioXP integration.
Current validation is mostly manual/operator-driven.
That means the roadmap should not pretend the proof problem is solved.

## Priority order from here
Because motion truth is still only partly proven and the interface can overload, the priority order should be:

1. control-plane reliability and overload hardening
2. motion correctness with independent physical verification
3. request/reply-correlating TMCL decode + real trace capture corpus
4. semantic deck/location model
5. pipette subsystem
6. higher-level workflow engine and vision semantics

## Concrete next actions

### Near-term decode work
1. Add a transaction-aware decode mode to `scripts/decode_tmcl_frame.py`
2. Capture real TMCL traces from supervised runs and commit a small corpus
3. Decode `cmd 153` thermal history payloads
4. Add richer reply-side interpretation for `GAP`, `GGP`, `GIO`, and `QUERY_MOTOR_STOP`

### Near-term gap-closing work
1. Freeze the safe motion baseline and remove or lock down risky prepared fast paths for non-debug use
2. Add proof-bundle capture for motion validation:
   - request/response JSON
   - before/after image snapshots
   - operator note / confirmation
   - artifact directory under `/mnt/BioModStack/bms_results/`
3. Build the semantic deck/location model before attempting broad automation
4. Port pipette behavior from OEM classes into the Python runtime instead of leaving `501` stubs

## Program framing updates added after this assessment
The follow-on planning pass created four concrete execution/control documents in this repo:
- `docs/VENDOR_PARITY_SCORECARD.md`
- `docs/FLEXIBILITY_REQUIREMENTS.md`
- `docs/ARCHITECTURE_TARGET_2026-04.md`
- `docs/EXECUTION_BACKLOG_2026-04-12.md`

These documents translate the gap assessment into:
- named parity rows
- explicit flexibility requirements
- a target layered architecture
- prioritized PR-sized implementation chunks

They should now be treated as the planning/control layer above this gap note.

## Bottom line
Low-level BioXP control is partly there.
Safety hardening in some areas may already be more explicit than OEM.
But the overall gap to OEM is still large because OEM capability depends on semantic machine models, pipette logic, vision semantics, and a real workflow engine.

The immediate next milestone is not “more features.”
It is:
- reliable and physically proven motion/control
- better TMCL decode from real traces
- then the missing semantic middle layer
