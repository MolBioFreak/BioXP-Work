#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
BASE_URL="${BIOXP_BASE_URL:-http://127.0.0.1:8123}"
LOG_ROOT="${BIOXP_LOG_ROOT:-/tmp/bioxp-live-runs}"
STEP_TIMEOUT="${BIOXP_STEP_TIMEOUT:-35}"
STAMP="$(date +%Y%m%d_%H%M%S)"
RUN_DIR="${LOG_ROOT}/${STAMP}_OEM_STARTUP_HOMING_STEPWISE"
mkdir -p "$RUN_DIR"

usage() {
  cat <<'EOF'
Usage: bioxp_supervised_oem_startup_homing_stepwise.sh [--start-at STEP]

OEM startup homing, supervised one physical step at a time. This replaces the
blocked monolithic strict_startup --homing path.

Steps:
  strict-startup  z-home  gripper-clear  gripper-home  x-home  x-park-6000  y-home  door-home  y-set-home

Every motion step requires typing STEP, logs before/after payloads, and traps
Ctrl-C/timeout to run bioxp_emergency_motor_kill.sh, which preempts the API first.
EOF
}
START_AT="strict-startup"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --start-at) START_AT="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown arg: $1" >&2; usage >&2; exit 2 ;;
  esac
done

echo "BioXP OEM startup homing STEPWISE helper"
echo "BASE_URL=$BASE_URL"
echo "RUN_DIR=$RUN_DIR"
echo "STEP_TIMEOUT=$STEP_TIMEOUT"
echo "START_AT=$START_AT"
echo "Emergency software stop: ./scripts/bioxp_emergency_motor_kill.sh"
echo "NOTE: Z-home is temporarily reversed for live direction retest (cmd=1/move_right instead of OEM-source cmd=2/move_left)."

KILLING=0
emergency_trap() {
  local rc=$?
  if [[ "$KILLING" -eq 0 ]]; then
    KILLING=1
    echo
    echo "INTERRUPTED/TIMEOUT: running emergency motor kill now..." >&2
    ./scripts/bioxp_emergency_motor_kill.sh || true
  fi
  exit "$rc"
}
trap emergency_trap INT TERM

call_get() {
  local label="$1"
  local path="$2"
  local out="$RUN_DIR/${label}.json"
  echo
  echo "=== $label ==="
  timeout --foreground 12 curl -fsS "${BASE_URL}${path}" | tee "$out"
  echo
}
call_post() {
  local label="$1"
  local path="$2"
  local body="$3"
  local out="$RUN_DIR/${label}.json"
  echo
  echo "=== $label ==="
  timeout --foreground "$STEP_TIMEOUT" curl -fsS -X POST "${BASE_URL}${path}" -H 'Content-Type: application/json' -d "$body" | tee "$out"
  echo
}
confirm_step() {
  local step="$1" desc="$2"
  echo
  echo "NEXT PHYSICAL STEP: $step"
  echo "$desc"
  echo "Type STEP to run this step, SKIP to skip, or ABORT to stop:"
  read -r ans
  case "$ans" in
    STEP) return 0 ;;
    SKIP) return 1 ;;
    ABORT) echo "Operator aborted." ; exit 130 ;;
    *) echo "Expected STEP/SKIP/ABORT; aborting." >&2; exit 130 ;;
  esac
}
inspect_after_step() {
  local step="$1"
  call_get "${step}_power_after" "/motion/power/status" || true
  call_get "${step}_axes_after" "/motion/axes/status?axes=x,y,z,g,door" || true
  echo
  echo "Observe the machine. If motion was wrong, run/keep running emergency kill and do NOT continue."
  echo "Type OK to continue, ABORT to stop:"
  read -r obs
  [[ "$obs" == "OK" ]] || exit 130
}

STEPS=(strict-startup z-home gripper-clear gripper-home x-home x-park-6000 y-home door-home y-set-home)
run_step=0
for s in "${STEPS[@]}"; do
  [[ "$s" == "$START_AT" ]] && run_step=1
  [[ "$run_step" -eq 1 ]] || continue
  case "$s" in
    strict-startup)
      call_get "status_before" "/status"
      call_get "power_before" "/motion/power/status"
      call_get "latch_before" "/latch/status"
      call_get "axes_before" "/motion/axes/status?axes=x,y,z,g,door"
      call_post "strict_startup_no_homing" "/motion/arm/strict_startup" '{"run_homing":false}'
      inspect_after_step "strict_startup"
      ;;
    z-home)
      if confirm_step "z-home" "OEM initializeMotors first motion: Z axisSearchHome at OEM speed 1791. This is expected to clear the head/pin-hole geometry before gripper/X/Y."; then
        call_post "z_home" "/motion/oem/startup_step" '{"step":"z-home","timeout_s":25.0}'
        inspect_after_step "z_home"
      fi
      ;;
    gripper-clear)
      if confirm_step "gripper-clear" "OEM setGripperCurrent(31) then MotorGrip moveSteps +10000. This should NOT be the first head-clear step anymore; Z-home already ran."; then
        call_post "gripper_clear" "/motion/oem/startup_step" '{"step":"gripper-clear","timeout_s":12.0}'
        inspect_after_step "gripper_clear"
      fi
      ;;
    gripper-home)
      if confirm_step "gripper-home" "OEM gripper axisSearchHome/goHome."; then
        call_post "gripper_home" "/motion/oem/startup_step" '{"step":"gripper-home","timeout_s":25.0}'
        inspect_after_step "gripper_home"
      fi
      ;;
    x-home)
      if confirm_step "x-home" "OEM X axisSearchHome at startup speed 250."; then
        call_post "x_home" "/motion/oem/startup_step" '{"step":"x-home","timeout_s":25.0}'
        inspect_after_step "x_home"
      fi
      ;;
    x-park-6000)
      if confirm_step "x-park-6000" "OEM setHome(X), setSpeed(X)=1700, Thread.Sleep(40), moveX(6000)."; then
        call_post "x_park_6000" "/motion/oem/startup_step" '{"step":"x-park-6000","timeout_s":12.0}'
        inspect_after_step "x_park_6000"
      fi
      ;;
    y-home)
      if confirm_step "y-home" "OEM Y axisSearchHome at startup speed 250."; then
        call_post "y_home" "/motion/oem/startup_step" '{"step":"y-home","timeout_s":25.0}'
        inspect_after_step "y_home"
      fi
      ;;
    door-home)
      if confirm_step "door-home" "OEM thermal doorSearchHome. Skip if the door geometry is not installed/safe."; then
        call_post "door_home" "/motion/oem/startup_step" '{"step":"door-home","timeout_s":25.0}'
        inspect_after_step "door_home"
      fi
      ;;
    y-set-home)
      if confirm_step "y-set-home" "OEM final setHome(Y) after doorSearchHome. This should not command physical motion."; then
        call_post "y_set_home" "/motion/oem/startup_step" '{"step":"y-set-home","timeout_s":5.0}'
        inspect_after_step "y_set_home"
      fi
      ;;
  esac
done

echo
echo "STEPWISE OEM startup homing helper completed. logs=$RUN_DIR"
