#!/usr/bin/env bash
set -euo pipefail

BIOXP_BASE_URL="${BIOXP_BASE_URL:-http://127.0.0.1:8123}"
BIOXP_LOG_ROOT="${BIOXP_LOG_ROOT:-/tmp/bioxp-live-runs}"
MODE="${BIOXP_OEM_STARTUP_MODE:-live}"
RUN_HOMING="${BIOXP_OEM_STARTUP_RUN_HOMING:-true}"
REQUIRE_CONFIG="${BIOXP_OEM_STARTUP_REQUIRE_CONFIG:-true}"
TIMEOUT_S="${BIOXP_OEM_STARTUP_TIMEOUT_S:-300}"
SESSION_TAG="$(date +%Y%m%d_%H%M%S)_OEM_APP_STARTUP_SEQUENCE"
ARTIFACT_ROOT="${BIOXP_OEM_STARTUP_ARTIFACT_ROOT:-${BIOXP_LOG_ROOT}/${SESSION_TAG}}"

mkdir -p "${ARTIFACT_ROOT}"

echo "=== OEM app startup mirror ==="
echo "base_url=${BIOXP_BASE_URL}"
echo "mode=${MODE}"
echo "artifact_root=${ARTIFACT_ROOT}"
echo "run_homing=${RUN_HOMING} require_config=${REQUIRE_CONFIG}"

ACK=""
if [[ "${MODE}" == "live" ]]; then
  echo "Type INITIALIZE to request live OEM startup program. This does not bypass door/latch/interlocks."
  read -r ACK
  if [[ "${ACK}" != "INITIALIZE" ]]; then
    echo "Refusing live startup: operator acknowledgement did not match INITIALIZE" >&2
    exit 2
  fi
fi

REQ="${ARTIFACT_ROOT}/startup_request_body.json"
RESP="${ARTIFACT_ROOT}/startup_request_response.json"
HTTP="${ARTIFACT_ROOT}/startup_request.http"
python3 - "${REQ}" "${MODE}" "${ACK}" "${ARTIFACT_ROOT}" "${RUN_HOMING}" "${REQUIRE_CONFIG}" "${TIMEOUT_S}" <<'PY'
import json, sys
path, mode, ack, artifact_root, run_homing, require_config, timeout_s = sys.argv[1:]
payload = {
    "mode": mode,
    "artifact_root": artifact_root,
    "require_config": require_config.lower() == "true",
    "door_policy": "wait_for_closed",
    "run_homing": run_homing.lower() == "true",
    "run_post_home": True,
    "timeout_s": float(timeout_s),
}
if ack:
    payload["operator_ack"] = ack
open(path, "w").write(json.dumps(payload, indent=2, sort_keys=True))
PY

code="$(curl -sS -m 30 -o "${RESP}" -w '%{http_code}' -X POST "${BIOXP_BASE_URL}/oem/startup/request" -H 'Content-Type: application/json' --data-binary "@${REQ}" || true)"
printf 'HTTP_CODE:%s\n' "${code}" | tee "${HTTP}"
cat "${RESP}" || true
printf '\n'
if [[ "${code}" != "200" ]]; then
  echo "OEM startup request failed; artifact_root=${ARTIFACT_ROOT}" >&2
  exit 1
fi

SESSION_ID="$(python3 - "${RESP}" <<'PY'
import json, sys
print(json.load(open(sys.argv[1])).get('session_id',''))
PY
)"
if [[ -z "${SESSION_ID}" ]]; then
  echo "No session_id in response" >&2
  exit 1
fi

echo "session_id=${SESSION_ID}"
echo "Polling status. Ctrl-C is safe; use emergency kill script for unsafe motion."
for _ in $(seq 1 120); do
  STATUS="${ARTIFACT_ROOT}/status_${SESSION_ID}.json"
  curl -sS -m 10 "${BIOXP_BASE_URL}/oem/startup/status/${SESSION_ID}" -o "${STATUS}" || true
  state="$(python3 - "${STATUS}" <<'PY'
import json, sys
try:
    print(json.load(open(sys.argv[1])).get('state','unknown'))
except Exception:
    print('unreadable')
PY
)"
  echo "state=${state}"
  case "${state}" in
    ready|failed_closed|aborted) break ;;
    waiting_for_door_close) echo "Waiting for enclosure door/latch close per OEM startup gate." ;;
  esac
  sleep 2
done

echo "Final status artifact root: ${ARTIFACT_ROOT}"
