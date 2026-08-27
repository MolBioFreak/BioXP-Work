#!/usr/bin/env bash
set -euo pipefail

printf '%s\n' \
  '[bioxp-watchdog] standalone recovery is permanently disabled.' \
  '[bioxp-watchdog] bioxp-api.service is the sole runtime owner; inspect or restart that canonical unit.' >&2
exit 97
