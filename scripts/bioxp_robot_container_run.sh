#!/usr/bin/env bash
set -euo pipefail

printf '%s\n' \
  '[bioxp-guard] generic container launch is permanently disabled.' \
  '[bioxp-guard] bioxp-api.service is the sole runtime owner; use scripts/bioxp_handlerctl.py start|restart|status.' >&2
exit 97
