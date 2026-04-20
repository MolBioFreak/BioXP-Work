#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  bioxp_supervised_strict_startup.sh [--homing] [--base-url URL] [--log-root DIR]

What it does:
  - captures pre-startup status/power/latch/axes snapshots
  - runs POST /motion/arm/strict_startup
  - captures post-startup status/power/latch/axes snapshots
  - attempts reference-status checks when the route exists
  - saves every payload into a timestamped log directory

Flags:
  --homing       run OEM startup homing sequence (run_homing=true)

Defaults:
  BIOXP_BASE_URL=http://127.0.0.1:8000/api/bioxp
  BIOXP_LOG_ROOT=/tmp/bioxp-live-runs
  BIOXP_AXES=x,y,z,g,door

Examples:
  ./scripts/bioxp_supervised_strict_startup.sh
  ./scripts/bioxp_supervised_strict_startup.sh --homing
  BIOXP_BASE_URL=http://127.0.0.1:8123 ./scripts/bioxp_supervised_strict_startup.sh --homing
EOF
}

BASE_URL="${BIOXP_BASE_URL:-http://127.0.0.1:8000/api/bioxp}"
LOG_ROOT="${BIOXP_LOG_ROOT:-/tmp/bioxp-live-runs}"
AXES="${BIOXP_AXES:-x,y,z,g,door}"
RUN_HOMING=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --homing)
      RUN_HOMING=1
      shift
      ;;
    --base-url)
      BASE_URL="$2"
      shift 2
      ;;
    --log-root)
      LOG_ROOT="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

STAMP="$(date +%Y%m%d_%H%M%S)"
MODE="strict_startup"
if [[ "$RUN_HOMING" -eq 1 ]]; then
  MODE="strict_startup_homing"
fi
RUN_DIR="${LOG_ROOT}/${STAMP}_${MODE}"
mkdir -p "$RUN_DIR"

json_body="$(python3 - "$RUN_HOMING" <<'PY'
import json
import sys
print(json.dumps({"run_homing": bool(int(sys.argv[1]))}, separators=(",", ":")))
PY
)"
printf '%s\n' "$json_body" > "$RUN_DIR/strict_startup_request.json"

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

echo "BioXP supervised strict-startup helper"
echo "BASE_URL=${BASE_URL}"
echo "RUN_HOMING=${RUN_HOMING}"
echo "AXES=${AXES}"
echo "RUN_DIR=${RUN_DIR}"

call_get "status_before" "/status"
call_get "power_before" "/motion/power/status"
call_get "latch_before" "/latch/status"
call_get "axes_before" "/motion/axes/status?axes=${AXES}"
maybe_get "reference_before" "/motion/reference/status?axes=${AXES}"
call_post "strict_startup" "/motion/arm/strict_startup" "$json_body"
call_get "status_after" "/status"
call_get "power_after" "/motion/power/status"
call_get "latch_after" "/latch/status"
call_get "axes_after" "/motion/axes/status?axes=${AXES}"
maybe_get "reference_after" "/motion/reference/status?axes=${AXES}"

python3 - "$RUN_DIR" "$RUN_HOMING" <<'PY'
import json
import os
import sys

run_dir = sys.argv[1]
run_homing = bool(int(sys.argv[2]))

def load(name):
    path = os.path.join(run_dir, f"{name}.json")
    with open(path, "r", encoding="utf-8") as fh:
        return json.load(fh)

result = load("strict_startup")
power_after = load("power_after")
latch_after = load("latch_after")
ref_after = load("reference_after")

armed = power_after.get("motion_arm", {}).get("armed")
arm_reason = power_after.get("motion_arm", {}).get("reason")
rail_ok = power_after.get("rail_24v", {}).get("no24v") is False
latch_ok = latch_after.get("door_sensor") == 1 and latch_after.get("solenoid_state") == 1 and latch_after.get("latch_sensor") == 1
checks = result.get("checks", []) if isinstance(result.get("checks"), list) else []
failed = [row.get("name") for row in checks if isinstance(row, dict) and not row.get("ok")]
ref_rows = ref_after.get("rows") if isinstance(ref_after, dict) else None

print()
print("=== summary ===")
print(f"run_homing={run_homing}")
print(f"result_ok={result.get('ok')}")
print(f"motion_arm_after={armed} reason={arm_reason}")
print(f"rail_24v_ok={rail_ok}")
print(f"latch_bundle_ok={latch_ok}")
print(f"failed_checks={failed}")
if isinstance(ref_rows, dict):
    compact = {axis: row.get('state') for axis, row in ref_rows.items() if isinstance(row, dict)}
    print(f"reference_states_after={compact}")
else:
    print("reference_states_after=None")
print(f"logs={run_dir}")
print()
if run_homing:
    print("Operator job: watch the whole startup homing sequence, verify each axis physically behaves as expected, and interrupt only if motion looks unsafe.")
else:
    print("Operator job: none beyond verifying interlocks stayed healthy; no axis motion should have been requested in this mode.")
PY
