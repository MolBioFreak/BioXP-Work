#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  bioxp_supervised_relative_move.sh <x|y|z|g|door> <steps> [--timeout SEC] [--base-url URL] [--log-root DIR] [--yes]

What it does:
  - requires an explicit operator confirmation unless --yes is supplied
  - captures pre-move status/power/latch/axis snapshots
  - attempts reference-status checks when the route exists
  - executes one supervised /motion/axis/relative request with reuse_prepared=false
  - captures post-move status/power/latch/axis snapshots
  - saves every payload into a timestamped log directory

Defaults:
  BIOXP_BASE_URL=http://127.0.0.1:8000/api/bioxp
  BIOXP_RELATIVE_TIMEOUT_S=12.0
  BIOXP_LOG_ROOT=/tmp/bioxp-live-runs

Examples:
  BIOXP_BASE_URL=http://100.124.140.56:8123 ./scripts/bioxp_supervised_relative_move.sh z -500
  BIOXP_BASE_URL=http://127.0.0.1:8123 ./scripts/bioxp_supervised_relative_move.sh x 500 --timeout 12.0

Safety notes:
  - Prefer robot-local URL for first motor testing.
  - Use one axis at a time.
  - Keep reuse_prepared=false unless explicitly debugging the prepared fast path elsewhere.
  - Telemetry is controller-side evidence only; the operator must physically watch the axis.
EOF
}

BASE_URL="${BIOXP_BASE_URL:-http://127.0.0.1:8000/api/bioxp}"
TIMEOUT_S="${BIOXP_RELATIVE_TIMEOUT_S:-12.0}"
LOG_ROOT="${BIOXP_LOG_ROOT:-/tmp/bioxp-live-runs}"
AXIS=""
STEPS=""
ASSUME_YES=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    x|y|z|g|door)
      if [[ -n "$AXIS" ]]; then
        echo "Axis already set to ${AXIS}; got another axis ${1}." >&2
        exit 2
      fi
      AXIS="$1"
      shift
      ;;
    --timeout)
      TIMEOUT_S="$2"
      shift 2
      ;;
    --base-url)
      BASE_URL="$2"
      shift 2
      ;;
    --log-root)
      LOG_ROOT="$2"
      shift 2
      ;;
    --yes|-y)
      ASSUME_YES=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    --*)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
    *)
      if [[ -z "$STEPS" ]]; then
        STEPS="$1"
        shift
      else
        echo "Unexpected positional argument: $1" >&2
        usage >&2
        exit 2
      fi
      ;;
  esac
done

if [[ -z "$AXIS" || -z "$STEPS" ]]; then
  echo "Missing axis or steps." >&2
  usage >&2
  exit 2
fi

python3 - "$AXIS" "$STEPS" "$TIMEOUT_S" <<'PY' >/dev/null
import sys
axis, steps, timeout_s = sys.argv[1], sys.argv[2], sys.argv[3]
if axis not in {"x", "y", "z", "g", "door"}:
    raise SystemExit(f"invalid axis: {axis}")
int(steps)
value = float(timeout_s)
if not (0.1 < value <= 60.0):
    raise SystemExit("timeout must be >0.1 and <=60.0")
PY

if [[ "$ASSUME_YES" -ne 1 ]]; then
  echo "About to command a LIVE BioXP relative move: axis=${AXIS}, steps=${STEPS}, base=${BASE_URL}"
  echo "Operator must be physically watching the robot with stop/power access."
  printf 'Type MOVE to proceed: '
  read -r reply
  if [[ "$reply" != "MOVE" ]]; then
    echo "Aborted."
    exit 1
  fi
fi

STAMP="$(date +%Y%m%d_%H%M%S)"
RUN_DIR="${LOG_ROOT}/${STAMP}_relative_${AXIS}_${STEPS}"
mkdir -p "$RUN_DIR"

json_body="$(python3 - "$AXIS" "$STEPS" "$TIMEOUT_S" <<'PY'
import json
import sys
axis, steps, timeout_s = sys.argv[1], int(sys.argv[2]), float(sys.argv[3])
print(json.dumps({"axis": axis, "steps": steps, "wait_timeout_s": timeout_s, "reuse_prepared": False}, separators=(",", ":")))
PY
)"
printf '%s\n' "$json_body" > "$RUN_DIR/relative_request.json"

call_get() {
  local label="$1"
  local path="$2"
  local out="$RUN_DIR/${label}.json"
  echo
  echo "=== ${label} ==="
  curl -fsS "${BASE_URL}${path}" | tee "$out"
  echo
}

maybe_get() {
  local label="$1"
  local path="$2"
  local out="$RUN_DIR/${label}.json"
  echo
  echo "=== ${label} ==="
  if curl -fsS "${BASE_URL}${path}" > "$out"; then
    cat "$out"
  else
    python3 - "$path" <<'PY' > "$out"
import json
import sys
print(json.dumps({"available": False, "path": sys.argv[1]}, separators=(",", ":")))
PY
    cat "$out"
  fi
  echo
}

call_post() {
  local label="$1"
  local path="$2"
  local body="$3"
  local out="$RUN_DIR/${label}.json"
  echo
  echo "=== ${label} ==="
  curl -fsS -X POST "${BASE_URL}${path}" \
    -H 'Content-Type: application/json' \
    -d "$body" | tee "$out"
  echo
}

echo "BioXP supervised relative-move helper"
echo "BASE_URL=${BASE_URL}"
echo "AXIS=${AXIS}"
echo "STEPS=${STEPS}"
echo "TIMEOUT_S=${TIMEOUT_S}"
echo "RUN_DIR=${RUN_DIR}"

call_get "status_before" "/status"
call_get "power_before" "/motion/power/status"
call_get "latch_before" "/latch/status"
call_get "axis_before" "/motion/axis/${AXIS}/status"
maybe_get "reference_before" "/motion/reference/status?axes=${AXIS}"
call_post "relative" "/motion/axis/relative" "$json_body"
call_get "power_after" "/motion/power/status"
call_get "latch_after" "/latch/status"
call_get "axis_after" "/motion/axis/${AXIS}/status"
maybe_get "reference_after" "/motion/reference/status?axes=${AXIS}"

python3 - "$RUN_DIR" "$AXIS" "$STEPS" <<'PY'
import json
import os
import sys

run_dir, axis, requested_steps = sys.argv[1], sys.argv[2], int(sys.argv[3])

def load(name):
    path = os.path.join(run_dir, f"{name}.json")
    with open(path, "r", encoding="utf-8") as fh:
        return json.load(fh)

axis_before = load("axis_before")
axis_after = load("axis_after")
relative = load("relative")
power_after = load("power_after")
latch_after = load("latch_after")
ref_after = load("reference_after")

before_status = axis_before.get("status", {}) if isinstance(axis_before, dict) else {}
after_status = axis_after.get("status", {}) if isinstance(axis_after, dict) else {}
before_pos = before_status.get("position", {}).get("position")
after_pos = after_status.get("position", {}).get("position")
after_speed = after_status.get("speed", {}).get("speed")
reported_delta = None if before_pos is None or after_pos is None else after_pos - before_pos
move = relative.get("move", {}) if isinstance(relative, dict) else {}
wait = relative.get("wait", {}) if isinstance(relative, dict) else {}
truth = relative.get("motion_truth", {}) if isinstance(relative, dict) else {}
prep_policy = relative.get("prep_policy", {}) if isinstance(relative, dict) else {}
armed = power_after.get("motion_arm", {}).get("armed")
arm_reason = power_after.get("motion_arm", {}).get("reason")
rail_ok = power_after.get("rail_24v", {}).get("no24v") is False
latch_ok = latch_after.get("door_sensor") == 1 and latch_after.get("solenoid_state") == 1 and latch_after.get("latch_sensor") == 1
ref_state = ref_after.get("rows", {}).get(axis, {}).get("state") if isinstance(ref_after.get("rows"), dict) else None

print()
print("=== summary ===")
print(f"axis={axis}")
print(f"requested_steps={requested_steps}")
print(f"position_before={before_pos}")
print(f"position_after={after_pos}")
print(f"reported_delta={reported_delta}")
print(f"reported_speed_after={after_speed}")
print(f"move_ok={move.get('ok')}")
print(f"wait_ok={wait.get('ok')} stopped={wait.get('stopped')}")
print(f"motion_arm_after={armed} reason={arm_reason}")
print(f"rail_24v_ok={rail_ok}")
print(f"latch_bundle_ok={latch_ok}")
print(f"reference_state_after={ref_state}")
print(f"prep_policy={prep_policy}")
print(f"motion_truth_evidence={truth.get('evidence_level')} physical_confirmed={truth.get('physical_motion_confirmed')}")
print(f"logs={run_dir}")
print()
if reported_delta != requested_steps:
    print("WARNING: controller-reported delta did not match requested steps. Stop and inspect before another move.")
if after_speed not in (0, None):
    print("WARNING: axis speed after move is not zero. Stop and inspect before another move.")
print("Operator job: confirm the axis physically moved as expected. Telemetry is controller-side evidence, not physical proof.")
PY
