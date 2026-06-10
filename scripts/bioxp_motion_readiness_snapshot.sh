#!/usr/bin/env bash
set -euo pipefail

BASE_URL="${BIOXP_BASE_URL:-http://127.0.0.1:8000/api/bioxp}"
AXES="${BIOXP_AXES:-x,y,z,g,door}"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

call() {
  local label="$1"
  local path="$2"
  echo
  echo "=== ${label} ==="
  curl -sS "${BASE_URL}${path}"
  echo
}

echo "BioXP readiness snapshot"
echo "BASE_URL=${BASE_URL}"
echo "AXES=${AXES}"

call "status" "/status"
call "motion power" "/motion/power/status"
call "latch" "/latch/status"
call "axes" "/motion/axes/status?axes=${AXES}"

echo
cat <<EOF
Next actions if needed:
  1. If motion_arm.armed=false, use the strict-startup helper:
     BIOXP_BASE_URL="$BIOXP_BASE_URL" "${SCRIPT_DIR}/bioxp_supervised_strict_startup.sh"

  2. Home one axis at a time with logged before/after snapshots:
     BIOXP_BASE_URL="$BIOXP_BASE_URL" "${SCRIPT_DIR}/bioxp_supervised_home_axis.sh" x

  3. Re-run this readiness snapshot after each major step.

Tip:
  Prefer BIOXP_BASE_URL=http://100.124.140.56:8123 from this workstation, or http://127.0.0.1:8123 on the robot host.
EOF
