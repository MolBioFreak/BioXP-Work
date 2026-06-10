#!/usr/bin/env bash
set -euo pipefail
UDOCKER_BIN="${UDOCKER_BIN:-$HOME/.local/share/bioxp-container-tools/venv/bin/udocker}"
CONTAINER_NAME="${BIOXP_CONTAINER_NAME:-bioxp-robot-handler-prod}"
APP_DIR="${BIOXP_APP_DIR:-$HOME/bioxp_re}"
PORT="${BIOXP_CONTAINER_PORT:-8123}"
LOG_DIR="${BIOXP_CONTAINER_LOG_DIR:-$HOME/.local/state/bioxp-container}"
mkdir -p "$LOG_DIR"

# If the production systemd unit exists, this runner is only allowed to execute
# inside that unit's invocation. This prevents cron/manual watchdog fallbacks from
# creating a second, unmanaged API owner on the hardware port.
if systemctl cat bioxp-api.service >/dev/null 2>&1 && [[ -z "${INVOCATION_ID:-}" ]]; then
  echo "[bioxp-guard] refusing standalone launch because bioxp-api.service owns lifecycle" >&2
  exit 97
fi

# Safety guard: the systemd service must be the sole owner of the robot API port.
# A stale host-venv uvicorn on :8123 can accept motion commands while the managed
# udocker handler crash-loops behind it. Kill only that exact stale same-user
# host process; refuse to start over any other listener.
listener_pids() {
  ss -H -ltnp "sport = :$PORT" 2>/dev/null | sed -nE 's/.*pid=([0-9]+),.*/\1/p' | sort -u || true
}
for pid in $(listener_pids); do
  [[ -r "/proc/$pid/cmdline" ]] || continue
  cmd=$(tr '\0' ' ' <"/proc/$pid/cmdline" 2>/dev/null || true)
  env=$(tr '\0' '\n' <"/proc/$pid/environ" 2>/dev/null || true)
  if [[ "$cmd" == *"python -m uvicorn bioxp.api:app"* && "$cmd" == *"--port $PORT"* && "$env" != *"BIOXP_RUNTIME_OWNER=udocker-container"* ]]; then
    echo "[bioxp-guard] killing stale host uvicorn pid=$pid on port $PORT: $cmd" >&2
    kill "$pid" 2>/dev/null || true
    for _ in {1..20}; do
      kill -0 "$pid" 2>/dev/null || break
      sleep 0.25
    done
  else
    echo "[bioxp-guard] refusing to start: port $PORT already owned by pid=$pid cmd=$cmd" >&2
    exit 98
  fi
done

if [[ -n "$(listener_pids)" ]]; then
  echo "[bioxp-guard] refusing to start: port $PORT still occupied after stale-listener cleanup" >&2
  exit 98
fi

exec "$UDOCKER_BIN" run \
  --user=root \
  --volume="$APP_DIR:/app" \
  --volume=/dev:/dev \
  --volume=/run/udev:/run/udev \
  --workdir=/app \
  "$CONTAINER_NAME" \
  /bin/sh -lc "PYTHONPATH=/app/src BIOXP_RUNTIME_OWNER=udocker-container exec python -m uvicorn bioxp.api:app --host 0.0.0.0 --port $PORT"
