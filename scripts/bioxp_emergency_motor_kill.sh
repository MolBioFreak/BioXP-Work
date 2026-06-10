#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_ROOT="${BIOXP_LOG_ROOT:-/tmp/bioxp-live-runs}"
mkdir -p "$LOG_ROOT"
LOG="$LOG_ROOT/${STAMP}_EMERGENCY_MOTOR_KILL.log"
RESTART_API=1
PREEMPT_API=1
while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-restart-api) RESTART_API=0; shift ;;
    --no-preempt-api) PREEMPT_API=0; shift ;;
    -h|--help)
      cat <<'EOF'
Usage: bioxp_emergency_motor_kill.sh [--no-restart-api] [--no-preempt-api]

Best-effort software emergency stop for BioXP motion. By default it first
preempts the uvicorn BioXP API process so a hung motion request cannot keep the
USB handle busy, then opens USB directly, sends MST stops, zeros run/standby
currents, deactivates boards 4/5/6, and restarts the API in a disarmed state.
Physical power/E-stop/unplug still wins if motion is unsafe.
EOF
      exit 0 ;;
    *) echo "Unknown argument: $1" >&2; exit 2 ;;
  esac
done

API_PID_FILE=/tmp/bioxp_api_8123.pid
API_LOG=/tmp/bioxp_api_8123.log
find_api_pids() {
  pgrep -f '[u]vicorn bioxp\.api:app' || true
}
preempt_api() {
  [[ "$PREEMPT_API" -eq 1 ]] || return 0
  local pids
  pids="$(find_api_pids | tr '\n' ' ')"
  if [[ -n "${pids// }" ]]; then
    echo "preempt_api_pids=${pids}"
    kill ${pids} 2>/dev/null || true
    sleep 0.8
    pids="$(find_api_pids | tr '\n' ' ')"
    if [[ -n "${pids// }" ]]; then
      echo "force_kill_api_pids=${pids}"
      kill -9 ${pids} 2>/dev/null || true
      sleep 0.5
    fi
  else
    echo "preempt_api_pids=none"
  fi
}
restart_api() {
  [[ "$RESTART_API" -eq 1 ]] || return 0
  local pids
  pids="$(find_api_pids | tr '\n' ' ')"
  if [[ -n "${pids// }" ]]; then
    echo "restart_api_kill_existing=${pids}"
    kill ${pids} 2>/dev/null || true
    sleep 0.8
    pids="$(find_api_pids | tr '\n' ' ')"
    if [[ -n "${pids// }" ]]; then
      echo "restart_api_force_kill_existing=${pids}"
      kill -9 ${pids} 2>/dev/null || true
      sleep 0.5
    fi
  fi
  echo "restart_api=true"
  nohup env PYTHONPATH=src .venv/bin/uvicorn bioxp.api:app --host 0.0.0.0 --port 8123 >"$API_LOG" 2>&1 &
  echo $! > "$API_PID_FILE"
  sleep 1.5
  echo "restart_api_pid=$(cat "$API_PID_FILE" 2>/dev/null || true)"
}
{
  echo "BioXP emergency motor kill"
  echo "time=$(date -Is)"
  echo "pwd=$PWD"
  echo "This preempts a hung API if needed, sends MST stop to known motion channels, drops run/standby currents to 0, then deactivates motion boards."
  preempt_api
  set +e
  PYTHONPATH=src python3 - <<'PY'
import json, time, sys
from bioxp.usb_driver import BioXpTester

def emit(row):
    print(json.dumps(row, separators=(",", ":"), default=str), flush=True)

def safe(label, fn):
    try:
        out = fn()
        emit({"label": label, "ok": True, "out": out})
        return out
    except Exception as exc:
        emit({"label": label, "ok": False, "error": repr(exc)})
        return None

try:
    t = BioXpTester()
except Exception as exc:
    emit({"label":"open_usb","ok":False,"error":repr(exc),"note":"If this says Resource busy, the API preemption failed; use physical power/E-stop/unplug."})
    raise SystemExit(2)

safe("drain_before", lambda: t.drain(max_reads=20, timeout_ms=4))
channels = [(5,0,"x"),(4,0,"y"),(4,1,"z"),(4,2,"g"),(6,0,"door")]
for round_idx in range(3):
    for board, motor, axis in channels:
        safe(f"{axis}:MST_stop_{round_idx+1}", lambda b=board,m=motor: t.motor_stop(b, motor=m))
    time.sleep(0.03)
for board, motor, axis in channels:
    safe(f"{axis}:run_current_param6_zero", lambda b=board,m=motor: t.motor_set_axis_param(b, 6, 0, motor=m))
    safe(f"{axis}:standby_current_param7_zero", lambda b=board,m=motor: t.motor_set_axis_param(b, 7, 0, motor=m))
for board, motor, axis in channels:
    safe(f"{axis}:speed_after", lambda b=board,m=motor: t.motor_get_speed(b, motor=m))
for board, label in [(4,"HEAD/YZG"),(5,"DECK/X"),(6,"DOOR")]:
    safe(f"board_{board}:{label}:deactivate_cmd64_value0", lambda b=board: t._motor_send_noreply_burst(b, 64, 0, 0, 0, repeats=5))
time.sleep(0.10)
for board, motor, axis in channels:
    safe(f"{axis}:speed_final", lambda b=board,m=motor: t.motor_get_speed(b, motor=m))
safe("rail_24v_final", lambda: t.motor_query_24v_sensor())
try:
    t._disconnect()
    emit({"label":"disconnect","ok":True,"out":"released"})
except Exception as exc:
    emit({"label":"disconnect","ok":False,"error":repr(exc)})
emit({"done": True, "physical_motion_commanded": False, "note": "If unsafe motion continues, use the physical power/E-stop/unplug route immediately."})
PY
  py_rc=$?
  restart_api
  exit "$py_rc"
} | tee "$LOG"
rc=${PIPESTATUS[0]}
echo "log=$LOG"
exit "$rc"
