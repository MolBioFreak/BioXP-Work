#!/usr/bin/env -S -i PATH=/usr/sbin:/usr/bin:/sbin:/bin /bin/bash --noprofile --norc
# Canonical systemd-owned container launcher for a committed BioXP release worktree.
# Application source, immutable lock evidence, and writable runtime state are distinct mounts.
set -euo pipefail

PATH=/usr/sbin:/usr/bin:/sbin:/bin
readonly PATH
READLINK=/usr/bin/readlink
DIRNAME=/usr/bin/dirname
GIT=/usr/bin/git
GREP=/usr/bin/grep
STAT=/usr/bin/stat
SS=/usr/bin/ss
SED=/usr/bin/sed
SORT=/usr/bin/sort
UDOCKER_BIN=/home/molbiofreak/.local/share/bioxp-container-tools/venv/bin/udocker
CONTAINER_NAME=bioxp-robot-handler-prod
PORT=8123
LOCK_DIR=/var/lib/bioxp-oem-authority
LOCK_FILE=/var/lib/bioxp-oem-authority/OEM_EVIDENCE_LOCK.json
STATE_DIR=/var/lib/bioxp-oem-runtime

SCRIPT_PATH="$($READLINK -f -- "${BASH_SOURCE[0]}")"
APP_DIR="$(cd "$($DIRNAME "$SCRIPT_PATH")/.." && /bin/pwd -P)"
EXPECTED_SCRIPT="$APP_DIR/scripts/bioxp_release_container_run.sh"

if [[ "$SCRIPT_PATH" != "$EXPECTED_SCRIPT" ]]; then
  echo "[bioxp-guard] release launcher identity does not match its repository path" >&2
  exit 99
fi

if ! $GREP -Fqx '0::/system.slice/bioxp-api.service' /proc/self/cgroup; then
  echo "[bioxp-guard] release launcher requires exact system-manager bioxp-api.service cgroup ownership" >&2
  exit 97
fi

if [[ ! -x "$UDOCKER_BIN" ]]; then
  echo "[bioxp-guard] required udocker launcher is unavailable" >&2
  exit 93
fi

if ! $GIT -C "$APP_DIR" rev-parse --is-inside-work-tree >/dev/null 2>&1 \
  || [[ "$($GIT -C "$APP_DIR" rev-parse --show-toplevel)" != "$APP_DIR" ]] \
  || [[ -n "$($GIT -C "$APP_DIR" status --porcelain)" ]]; then
  echo "[bioxp-guard] release source must be a clean Git worktree" >&2
  exit 96
fi
RELEASE_SHA="$($GIT -C "$APP_DIR" rev-parse HEAD)"
RELEASE_TREE="$($GIT -C "$APP_DIR" rev-parse HEAD^{tree})"

if [[ -L "$LOCK_FILE" || ! -f "$LOCK_FILE" || -w "$LOCK_FILE" \
  || "$($STAT -c '%U:%G:%a' "$LOCK_FILE")" != 'root:root:444' \
  || -L "$LOCK_DIR" || ! -d "$LOCK_DIR" \
  || "$($STAT -c '%U:%G:%a' "$LOCK_DIR")" != 'root:root:555' ]]; then
  echo "[bioxp-guard] OEM lock must be canonical, root-owned, non-writable, and non-symlinked" >&2
  exit 95
fi

if [[ ! -d "$STATE_DIR" || ! -w "$STATE_DIR" ]]; then
  echo "[bioxp-guard] runtime state root must be the existing writable directory" >&2
  exit 94
fi

listener_pids() {
  $SS -H -ltnp "sport = :$PORT" 2>/dev/null | $SED -nE 's/.*pid=([0-9]+),.*/\1/p' | $SORT -u || true
}

if [[ -n "$(listener_pids)" ]]; then
  echo "[bioxp-guard] refusing to start: canonical port $PORT already owned" >&2
  exit 98
fi

exec "$UDOCKER_BIN" run \
  --user=root \
  --volume="$APP_DIR:/app" \
  --volume="$LOCK_DIR:/app/.oem_lock" \
  --volume="$STATE_DIR:/app/.oem_runtime_state" \
  --volume=/dev:/dev \
  --volume=/run/udev:/run/udev \
  --workdir=/app \
  "$CONTAINER_NAME" \
  /bin/sh -lc "PYTHONPATH=/app/src BIOXP_RUNTIME_OWNER=udocker-container BIOXP_RELEASE_SHA=$RELEASE_SHA BIOXP_RELEASE_TREE=$RELEASE_TREE BIOXP_RELEASE_SOURCE_MOUNT=/app BIOXP_OEM_MACHINE_BUNDLE_LOCK=/app/.oem_lock/OEM_EVIDENCE_LOCK.json BIOXP_PHYSICAL_LABEL_SERIAL=206 BIOXP_OEM_RUNTIME_ROOT=/app/.oem_runtime_state BIOXP_OEM_RUNTIME_STATE_ROOT=/app/.oem_runtime_state exec python -m uvicorn bioxp.api:app --host 0.0.0.0 --port $PORT"
