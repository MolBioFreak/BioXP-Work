# BioXP3200 live-console validation checklist — 2026-04-26

Purpose: verify the newly added live protocol contract and reset-provenance behavior on the robot-local runtime without confusing software recovery with proven bad componentry.

Artifact root for this session:

```text
/mnt/BioModStack/bms_results/bioxp_validation/2026-04-26T19-46-01Z/
```

## Scope and safety guardrails

- Required operator posture: one person physically at the BioXP console, watching the gantry/camera/deck.
- Start with GET/status and dry-run checks only.
- Do not run aspirate/dispense/mix or any live protocol until x/y/z reference state is physically confirmed and the deck manifest matches the actual deck.
- Treat reset provenance as evidence of which recovery path was taken, not proof that a hardware component is good or bad.
- If a command times out or a physical state disagrees with telemetry, stop and capture logs before retrying.

## 1. Runtime and baseline observability

Run from the workstation or BMS host:

```bash
mkdir -p /mnt/BioModStack/bms_results/bioxp_validation/2026-04-26T19-46-01Z
BASE=http://100.124.140.56:8123
curl -sS "$BASE/status" | tee /mnt/BioModStack/bms_results/bioxp_validation/2026-04-26T19-46-01Z/status.json
curl -sS "$BASE/motion/power/status" | tee /mnt/BioModStack/bms_results/bioxp_validation/2026-04-26T19-46-01Z/motion_power_status.json
curl -sS "$BASE/motion/reference/status" | tee /mnt/BioModStack/bms_results/bioxp_validation/2026-04-26T19-46-01Z/motion_reference_status.json
curl -sS "$BASE/liquid/status" | tee /mnt/BioModStack/bms_results/bioxp_validation/2026-04-26T19-46-01Z/liquid_status.json
curl -sS "$BASE/camera/stream_state" | tee /mnt/BioModStack/bms_results/bioxp_validation/2026-04-26T19-46-01Z/camera_stream_state_before.json
```

Pass criteria:

- `/status` responds.
- `/motion/power/status` and `/motion/reference/status` are captured separately; do not collapse power/armed status into reference truth.
- `/liquid/status` is captured as transport/status evidence only, not proof of OEM-equivalent pipetting.
- `/camera/stream_state` shows current stream state before any reset.

## 2. Camera reset provenance check

This check is intentionally a software/process reset, not a USB hardware reset.

```bash
curl -sS -X POST "$BASE/camera/reset" \
  -H 'Content-Type: application/json' \
  -d '{"device":"/dev/video0"}' \
  | tee /mnt/BioModStack/bms_results/bioxp_validation/2026-04-26T19-46-01Z/camera_reset.json
curl -sS "$BASE/camera/stream_state" \
  | tee /mnt/BioModStack/bms_results/bioxp_validation/2026-04-26T19-46-01Z/camera_stream_state_after_reset.json
```

Expected payload evidence:

- `reset_provenance.schema_version == "bioxp.reset_provenance.v1"`
- `reset_provenance.subsystem == "camera"`
- `reset_provenance.source == "camera_reset_local"`
- `reset_provenance.reset_scope == "ffmpeg_process_and_stream_lock"`
- `reset_provenance.hardware_usb_reset_performed == false`
- `reset_provenance.software_recovery == true`
- `reset_provenance.hardware_component_fault_proven == false`

If this clears a camera hang, record it as software/UVC-path recovery. Do not call it proof of a bad camera or bad USB component without kernel log correlation and physical inspection.

## 3. USB reconnect provenance check

Only perform this under physical supervision. This can rebind or reset the USB runtime path.

Recommended evidence capture on the robot:

```bash
ssh molbiofreak@robot 'journalctl -u bioxp-api.service --since "20 minutes ago" --no-pager' \
  | tee /mnt/BioModStack/bms_results/bioxp_validation/2026-04-26T19-46-01Z/bioxp_api_journal_before_usb_reconnect.log
```

Then use the narrowest available API/maintenance path that calls `BioXpTester.reconnect(hard_reset=false)` first. If a hard reset is explicitly required, record the reason in the artifact notes before running it.

Expected provenance from the runtime method:

- `reset_provenance.schema_version == "bioxp.reset_provenance.v1"`
- `reset_provenance.subsystem == "usb_runtime"`
- `reset_provenance.source == "BioXpTester.reconnect"`
- `reset_provenance.reset_scope == "usb_rebind_and_optional_device_reset"`
- `reset_provenance.requested_hard_reset` matches the operator request.
- `reset_provenance.hardware_usb_reset_performed` is true only if the hard-reset attempt was actually used.
- `reset_provenance.transport_recovery_state_reset == true` on success.
- `attempts[]` shows soft-first behavior unless hard reset was explicitly requested.

Afterward:

```bash
ssh molbiofreak@robot 'journalctl -u bioxp-api.service --since "20 minutes ago" --no-pager' \
  | tee /mnt/BioModStack/bms_results/bioxp_validation/2026-04-26T19-46-01Z/bioxp_api_journal_after_usb_reconnect.log
```

## 4. Live protocol contract negative check

This should fail closed. It validates that `dry_run=false` cannot silently execute without operator/deck/preflight/artifact contract evidence.

```bash
curl -sS -i -X POST "$BASE/protocol/execute" \
  -H 'Content-Type: application/json' \
  -d '{"source_type":"native","document":{"steps":[]},"dry_run":false}' \
  | tee /mnt/BioModStack/bms_results/bioxp_validation/2026-04-26T19-46-01Z/protocol_live_missing_contract.http
```

Pass criteria:

- HTTP 409.
- Error payload includes `live_protocol_contract_failed`.
- No motion, liquid, or inspection action occurs.

## 5. Protocol dry-run bundle check

Dry-run should still work without live artifact gating.

```bash
curl -sS -X POST "$BASE/protocol/execute" \
  -H 'Content-Type: application/json' \
  -d '{"source_type":"native","document":{"steps":[]},"dry_run":true}' \
  | tee /mnt/BioModStack/bms_results/bioxp_validation/2026-04-26T19-46-01Z/protocol_dry_run.json
```

Pass criteria:

- Job is created.
- Operator bundle path is returned or discoverable from the job payload.
- No live action occurs.

## 6. Live protocol contract positive check — only after physical setup

Only proceed when all are true:

- Operator is physically at the machine.
- x/y/z reference state has been physically verified.
- Deck manifest matches the actual loaded deck.
- Required preflight screenshots/snapshots/artifacts have been captured.
- The protocol document has been reviewed and dry-run output is clean.

Minimum required live request fields:

- `dry_run: false`
- `live_execution_ack: true`
- `operator_id`
- `physical_console_verified: true`
- `deck_manifest`
- `preflight`
- `artifact_refs` and/or `snapshot_refs`

Pass criteria:

- Live contract is persisted in the operator bundle under `execution.live_contract`.
- `preflight.json` is written in the job directory.
- Any live action result includes enough artifact references to reconstruct what was physically verified.

## 7. Motor functionality testing sequence

Use the robot-local API first for motor testing. BMS proxy/UI can be checked afterward, but the first motion evidence should come from the robot daemon itself.

Recommended artifact root:

```bash
export BIOXP_BASE_URL=http://100.124.140.56:8123
export BIOXP_LOG_ROOT=/mnt/BioModStack/bms_results/bioxp_validation/2026-04-26T19-46-01Z/motor
```

Initial no-motion readiness snapshot:

```bash
scripts/bioxp_motion_readiness_snapshot.sh | tee /mnt/BioModStack/bms_results/bioxp_validation/2026-04-26T19-46-01Z/motor/readiness_initial.log
```

If motion is not armed, run strict startup without homing first:

```bash
scripts/bioxp_supervised_strict_startup.sh
```

Only proceed to movement if:

- `motion_arm.armed == true`
- `motion_arm.reason == "strict_init_pass"`
- `rail_24v.no24v == false`
- door/latch/solenoid are in the permissive locked state
- selected axis speed is zero
- operator is physically watching the robot

First supervised relative probes:

```bash
scripts/bioxp_supervised_relative_move.sh z -500
scripts/bioxp_supervised_relative_move.sh x 500
scripts/bioxp_supervised_relative_move.sh y 500
```

Interpretation rules:

- `reuse_prepared=false` is forced by the helper; do not debug the prepared fast path during baseline motor validation.
- A successful API response is controller-side evidence only. The operator must confirm physical motion, sound, and absence of binding.
- If `reported_delta` differs from requested steps, or post-move speed is not zero, stop.
- If physical motion disagrees with telemetry, stop and capture journal/kernel logs before retrying.
- If reference state is `unknown`, avoid blind absolute moves. Use supervised relative micro-moves or homing/re-reference under direct observation.

Only after micro-moves look sane should homing be tested, one axis at a time:

```bash
scripts/bioxp_supervised_home_axis.sh x
scripts/bioxp_supervised_home_axis.sh y
scripts/bioxp_supervised_home_axis.sh z --timeout 25.0
scripts/bioxp_supervised_home_axis.sh g
scripts/bioxp_supervised_home_axis.sh door
```

Do not start full startup homing until individual-axis behavior is understood. Telemetry plus physical observation are both required.

## 8. BMS proxy/UI validation

On BMS, capture proxy parity and frontend behavior:

```bash
BMS=http://127.0.0.1:8000
curl -sS "$BMS/api/bioxp/capabilities" | tee /mnt/BioModStack/bms_results/bioxp_validation/2026-04-26T19-46-01Z/bms_bioxp_capabilities.json
curl -sS "$BMS/api/bioxp/camera/stream_state" | tee /mnt/BioModStack/bms_results/bioxp_validation/2026-04-26T19-46-01Z/bms_camera_stream_state.json
curl -sS "$BMS/api/bioxp/motion/reference/status" | tee /mnt/BioModStack/bms_results/bioxp_validation/2026-04-26T19-46-01Z/bms_motion_reference_status.json
curl -sS "$BMS/api/bioxp/liquid/status" | tee /mnt/BioModStack/bms_results/bioxp_validation/2026-04-26T19-46-01Z/bms_liquid_status.json
```

Pass criteria:

- BMS capability matrix says the relevant robot-local routes are proxied.
- Cockpit displays runtime reachability as linkage/proxy truth, not SSH/admin ownership.
- Reference status, liquid status, and camera stream state are visible without implying physical proof beyond what the robot reports.

## 9. Stop conditions

Stop immediately and preserve artifacts if any of the following happen:

- Physical gantry/deck state disagrees with telemetry.
- Camera reset returns `hardware_component_fault_proven: true` unexpectedly or omits provenance.
- USB reconnect requires repeated hard resets.
- Kernel logs show repeated USB resets closely correlated with API reconnect attempts.
- `/protocol/execute dry_run=false` does not fail closed when contract evidence is missing.
- Liquid operations pass despite missing x/y/z reference truth.
