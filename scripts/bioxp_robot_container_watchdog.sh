#!/usr/bin/env bash
set -euo pipefail
PORT="${BIOXP_CONTAINER_PORT:-8123}"
BASE="http://127.0.0.1:${PORT}"
LOG_DIR="${BIOXP_CONTAINER_LOG_DIR:-$HOME/.local/state/bioxp-container}"
RUN_SCRIPT="${BIOXP_RUN_SCRIPT:-$HOME/bioxp_re/scripts/bioxp_robot_container_run.sh}"
LOCK="$LOG_DIR/watchdog-${PORT}.lock"
mkdir -p "$LOG_DIR"

# Production is managed by bioxp-api.service. The watchdog must never spawn a
# second standalone API listener on the same port; that was the failure mode
# that left controls talking to a stale host process while systemd crash-looped
# the container handler. Keep the old standalone fallback only for systems that
# do not have the service installed.
if systemctl cat bioxp-api.service >/dev/null 2>&1; then
  if ! curl -fsS --max-time 2 "$BASE/openapi.json" >/dev/null 2>&1; then
    echo "[bioxp-watchdog] bioxp-api.service owns lifecycle; not spawning standalone listener for $BASE" >>"$LOG_DIR/watchdog-${PORT}.log"
  fi
  exit 0
fi

if curl -fsS --max-time 2 "$BASE/openapi.json" >/dev/null 2>&1; then exit 0; fi
(
  flock -n 9 || exit 0
  if curl -fsS --max-time 2 "$BASE/openapi.json" >/dev/null 2>&1; then exit 0; fi
  pkill -f "BIOXP_RUNTIME_OWNER=udocker-container.*--port $PORT" 2>/dev/null || true
  pkill -f "uvicorn bioxp.api:app --host 0.0.0.0 --port $PORT" 2>/dev/null || true
  sleep 1
  nohup setsid env BIOXP_CONTAINER_PORT="$PORT" "$RUN_SCRIPT" >>"$LOG_DIR/bioxp-container-${PORT}.log" 2>>"$LOG_DIR/bioxp-container-${PORT}.err" </dev/null &
) 9>"$LOCK"
