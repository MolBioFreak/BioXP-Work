#!/usr/bin/env bash
# Supervised non-motion ControlLib.initializeMotion diagnostic launcher.
set -euo pipefail

BIOXP_BASE_URL="${BIOXP_BASE_URL:-http://127.0.0.1:8123}"
BIOXP_LOG_ROOT="${BIOXP_LOG_ROOT:-/tmp/bioxp-live-runs}"
BIOXP_AXES="${BIOXP_AXES:-x,y,z,g,door}"
BIOXP_TIMEOUT_S="${BIOXP_TIMEOUT_S:-90}"
BIOXP_OPERATOR_ACK="${BIOXP_OPERATOR_ACK:-}"
SESSION_TAG="$(date +%Y%m%d_%H%M%S)_OEM_INITIALIZE_MOTION_DIAGNOSTIC"
ARTIFACT_ROOT="${BIOXP_ARTIFACT_ROOT:-${BIOXP_LOG_ROOT}/${SESSION_TAG}}"

if [[ -z "${BIOXP_OPERATOR_ACK}" ]]; then
  printf 'Type INITIALIZE to run the supervised no-homing diagnostic: '
  read -r BIOXP_OPERATOR_ACK
fi
if [[ "${BIOXP_OPERATOR_ACK}" != "INITIALIZE" ]]; then
  echo "Refusing: operator acknowledgement must be INITIALIZE" >&2
  exit 2
fi

mkdir -p "${ARTIFACT_ROOT}"
printf 'base_url=%s\nartifact_root=%s\nmode=no-homing diagnostic\n' "${BIOXP_BASE_URL}" "${ARTIFACT_ROOT}"

snapshot() {
  local phase="$1"
  local failures=0
  local endpoint filename
  local -a endpoints=(
    "/status|${phase}_status.json"
    "/motion/power/status|${phase}_motion_power.json"
    "/latch/status|${phase}_latch.json"
    "/motion/axes/status?axes=${BIOXP_AXES}|${phase}_axes.json"
  )
  for row in "${endpoints[@]}"; do
    endpoint="${row%%|*}"
    filename="${row#*|}"
    if ! curl -fsS "${BIOXP_BASE_URL}${endpoint}" >"${ARTIFACT_ROOT}/${filename}" 2>"${ARTIFACT_ROOT}/${filename}.stderr"; then
      failures=1
    fi
  done
  return "${failures}"
}

if ! snapshot pre; then
  echo "Refusing: pre-attempt passive snapshot was incomplete; no initialization request sent" >&2
  exit 1
fi

python3 - "${ARTIFACT_ROOT}/initialize_motion_request.json" "${BIOXP_TIMEOUT_S}" <<'PY'
import json
import sys

path, timeout_s = sys.argv[1:]
payload = {
    "operator_ack": "INITIALIZE",
    "run_homing": False,
    "include_tip_pipette_cleanup": False,
    "timeout_s": int(timeout_s),
}
with open(path, "w", encoding="utf-8") as handle:
    json.dump(payload, handle, indent=2, sort_keys=True)
PY

post_rc=0
if curl --fail-with-body -sS -X POST "${BIOXP_BASE_URL}/motion/oem/initialize_motion" \
  -H 'Content-Type: application/json' \
  --data-binary "@${ARTIFACT_ROOT}/initialize_motion_request.json" \
  >"${ARTIFACT_ROOT}/initialize_motion_response.json" \
  2>"${ARTIFACT_ROOT}/initialize_motion_post.stderr"; then
  :
else
  post_rc=$?
fi

post_snapshot_rc=0
if snapshot post; then
  :
else
  post_snapshot_rc=$?
fi

if [[ "${post_rc}" -ne 0 ]]; then
  echo "Refusing success: initialization request failed (curl exit ${post_rc}); post-attempt snapshot retained" >&2
  exit 1
fi
if [[ "${post_snapshot_rc}" -ne 0 ]]; then
  echo "Refusing success: post-attempt passive snapshot was incomplete" >&2
  exit 1
fi

python3 - "${ARTIFACT_ROOT}" <<'PY'
import json
import sys
from pathlib import Path

root = Path(sys.argv[1])
response = json.loads((root / "initialize_motion_response.json").read_text())
result = response.get("result") if isinstance(response, dict) else None
if not isinstance(response, dict) or response.get("ok") is not True:
    raise SystemExit("refusing success: API response did not prove ok=true")
if not isinstance(result, dict):
    raise SystemExit("refusing success: API response did not include a result object")
if result.get("physical_motion_commanded") is not False:
    raise SystemExit("refusing success: result.physical_motion_commanded was not literally false")
(root / "summary.json").write_text(json.dumps({
    "ok": True,
    "physical_motion_commanded": False,
    "mode": "supervised_no_homing_diagnostic",
    "request": "initialize_motion_request.json",
    "response": "initialize_motion_response.json",
}, indent=2, sort_keys=True) + "\n")
PY

echo "PASS: no-homing diagnostic artifacted at ${ARTIFACT_ROOT}"
