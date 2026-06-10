#!/usr/bin/env bash
set -euo pipefail

missing=0
for cmd in ffmpeg ps v4l2-ctl fuser; do
  if ! command -v "$cmd" >/dev/null 2>&1; then
    echo "missing required camera runtime dependency: $cmd" >&2
    missing=1
  fi
done
if [ "$missing" -ne 0 ]; then
  exit 11
fi

python - <<'PY'
import shutil
import usb.core
import fastapi
import uvicorn

missing = [name for name in ["ffmpeg", "ps", "v4l2-ctl", "fuser"] if shutil.which(name) is None]
if missing:
    raise SystemExit(f"missing helpers: {missing}")
print("container_camera_runtime_contract_ok")
PY
