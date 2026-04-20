#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  bioxp_supervised_home_axis.sh <x|y|z|g|door> [--timeout SEC] [--speed STEPS_PER_S] [--base-url URL] [--log-root DIR]

What it does:
  - captures pre-home status/power/latch/axis snapshots
  - executes one OEM-style /motion/axis/home request
  - captures post-home status/power/latch/axis snapshots
  - attempts reference-status checks when the route exists
  - saves every payload into a timestamped log directory

Defaults:
  BIOXP_BASE_URL=http://127.0.0.1:8000/api/bioxp
  BIOXP_HOME_TIMEOUT_S=20.0
  BIOXP_LOG_ROOT=/tmp/bioxp-live-runs

Examples:
  BIOXP_BASE_URL=http://127.0.0.1:8000/api/bioxp ./scripts/bioxp_supervised_home_axis.sh x
  BIOXP_BASE_URL=http://127.0.0.1:8123 ./scripts/bioxp_supervised_home_axis.sh z --timeout 25.0
  ./scripts/bioxp_supervised_home_axis.sh g --speed 200
EOF
}

BASE_URL="${BIOXP_BASE_URL:-http://127.0.0.1:8000/api/bioxp}"
TIMEOUT_S="${BIOXP_HOME_TIMEOUT_S:-20.0}"
LOG_ROOT="${BIOXP_LOG_ROOT:-/tmp/bioxp-live-runs}"
AXIS=""
SPEED=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    x|y|z|g|door)
      AXIS="$1"
      shift
      ;;
    --timeout)
      TIMEOUT_S="$2"
      shift 2
      ;;
    --speed)
      SPEED="$2"
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

if [[ -z "$AXIS" ]]; then
  echo "Missing axis." >&2
  usage >&2
  exit 2
fi

STAMP="$(date +%Y%m%d_%H%M%S)"
RUN_DIR="${LOG_ROOT}/${STAMP}_home_${AXIS}"
mkdir -p "$RUN_DIR"

json_body="$(python3 - "$AXIS" "$TIMEOUT_S" "$SPEED" <<'PY'
import json
import sys

axis = sys.argv[1]
timeout_s = float(sys.argv[2])
speed_raw = sys.argv[3]
body = {"axis": axis, "timeout_s": timeout_s}
if speed_raw:
    body["speed"] = int(speed_raw)
print(json.dumps(body, separators=(",", ":")))
PY
)"
printf '%s\n' "$json_body" > "$RUN_DIR/home_request.json"

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

echo "BioXP supervised home helper"
echo "BASE_URL=${BASE_URL}"
echo "AXIS=${AXIS}"
echo "TIMEOUT_S=${TIMEOUT_S}"
echo "SPEED=${SPEED:-<OEM default>}"
echo "RUN_DIR=${RUN_DIR}"

call_get "status_before" "/status"
call_get "power_before" "/motion/power/status"
call_get "latch_before" "/latch/status"
call_get "axis_before" "/motion/axis/${AXIS}/status"
maybe_get "reference_before" "/motion/reference/status?axes=${AXIS}"
call_post "home" "/motion/axis/home" "$json_body"
call_get "power_after" "/motion/power/status"
call_get "latch_after" "/latch/status"
call_get "axis_after" "/motion/axis/${AXIS}/status"
maybe_get "reference_after" "/motion/reference/status?axes=${AXIS}"

python3 - "$RUN_DIR" "$AXIS" <<'PY'
import json
import os
import sys

run_dir, axis = sys.argv[1], sys.argv[2]

def load(name):
    path = os.path.join(run_dir, f"{name}.json")
    with open(path, "r", encoding="utf-8") as fh:
        return json.load(fh)

axis_before = load("axis_before")
axis_after = load("axis_after")
home = load("home")
power_after = load("power_after")
latch_after = load("latch_after")
ref_after = load("reference_after")

before_pos = axis_before.get("status", {}).get("position", {}).get("position")
after_pos = axis_after.get("status", {}).get("position", {}).get("position")
after_speed = axis_after.get("status", {}).get("speed", {}).get("speed")
armed = power_after.get("motion_arm", {}).get("armed")
arm_reason = power_after.get("motion_arm", {}).get("reason")
rail_ok = power_after.get("rail_24v", {}).get("no24v") is False
latch_ok = latch_after.get("door_sensor") == 1 and latch_after.get("solenoid_state") == 1 and latch_after.get("latch_sensor") == 1
ref_state = ref_after.get("rows", {}).get(axis, {}).get("state") if isinstance(ref_after.get("rows"), dict) else None
home_outer = home.get("home") if isinstance(home, dict) else None
home_inner = home_outer.get("home") if isinstance(home_outer, dict) else None
home_ok = None
if isinstance(home_inner, dict):
    home_ok = home_inner.get("ok")
elif isinstance(home_outer, dict):
    home_ok = home_outer.get("ok")
elif isinstance(home, dict):
    home_ok = home.get("ok")

print()
print("=== summary ===")
print(f"axis={axis}")
print(f"position_before={before_pos}")
print(f"position_after={after_pos}")
print(f"reported_delta={None if before_pos is None or after_pos is None else after_pos - before_pos}")
print(f"reported_speed_after={after_speed}")
print(f"motion_arm_after={armed} reason={arm_reason}")
print(f"rail_24v_ok={rail_ok}")
print(f"latch_bundle_ok={latch_ok}")
print(f"reference_state_after={ref_state}")
print(f"home_ok={home_ok}")
print(f"logs={run_dir}")
print()
print("Operator job: confirm the axis physically moved to the expected end position and nothing sounded wrong.")
PY
