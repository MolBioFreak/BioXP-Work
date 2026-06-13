# BioXP API reload/connection RCA — 2026-06-13

## Incident
During full-init deployment/testing, the normal robot API on port 8123 could not be restarted via `systemctl restart bioxp-api.service` because the SSH user lacked non-interactive polkit/sudo authorization. A workaround attempted to run the current repo code through the existing user-level udocker service on port 8124, then suppress the stale 8123 process so 8124 could access the USB device. This was not a valid deployment path.

## Evidence
- `systemctl cat bioxp-api.service` / `systemctl show` says the system unit should run host venv uvicorn on port 8123:
  - `ExecStart=/home/molbiofreak/bioxp_re/.venv/bin/uvicorn bioxp.api:app --host 0.0.0.0 --port 8123`
  - `User=molbiofreak`
  - `WorkingDirectory=/home/molbiofreak/bioxp_re`
- The observed running process before suppression was actually udocker/proot on port 8123:
  - `udocker run ... --volume=/home/molbiofreak/bioxp_re:/app ... bioxp-robot-handler-prod ... uvicorn ... --port 8123`
- `systemctl restart bioxp-api.service` from SSH failed:
  - `Interactive authentication required`
  - `sudo -n` failed with `sudo: a password is required`
- Starting the user-level service on 8124 succeeded and loaded current repo code, but while 8123 still existed it returned USB busy:
  - HTTP 503 `[Errno 16] Resource busy`
- TERM-suppressing same-user 8123 processes caused the system unit to restart repeatedly and then enter:
  - `failed (Result: start-limit-hit)`
- The user-level 8124 service is not a production replacement for 8123. It is disabled, port-shifted, and tied to the user systemd manager/session behavior. It could be used only as a temporary diagnostic process, not as a stable deployment target.

## Root cause
There were two separate deployment/control-plane issues:

1. **Service definition vs observed runtime mismatch**
   The system unit metadata describes host-venv uvicorn, but the actual long-running production owner was udocker/proot. This means restart expectations were stale; the service/run-script relationship must be reconciled before remote deployment is reliable.

2. **No authorized non-interactive restart path**
   The SSH user can inspect the service and kill same-user processes, but cannot reset/restart the system unit through systemd without polkit/sudo auth. Killing processes under an always-restart unit is not a safe reload mechanism; it can hit systemd start-limit and leave the production service down.

## Corrective rule
Do **not** use process-kill suppression, SIGTERM fallback, or 8124 sidecar as a deployment/reload mechanism for production BioXP API code.

Acceptable reload paths are only:
- interactive/local operator performs `sudo systemctl reset-failed bioxp-api.service && sudo systemctl restart bioxp-api.service`, then Hermes verifies OpenAPI/build info/status; or
- install a narrowly scoped sudoers/polkit rule for `molbiofreak` to run exactly:
  - `systemctl reset-failed bioxp-api.service`
  - `systemctl restart bioxp-api.service`
  - optionally `systemctl status bioxp-api.service`
  with no password; or
- convert the production service to a user-owned systemd service that owns port 8123 and USB lifecycle intentionally, then disable the conflicting system unit.

Until one of those is implemented, code can be committed and tested offline, but Hermes must not claim the live 8123 API has loaded it.

## Current required recovery
Because the system service is in start-limit-hit, recover with privileged/local command:

```bash
sudo systemctl reset-failed bioxp-api.service
sudo systemctl restart bioxp-api.service
systemctl status bioxp-api.service --no-pager -l
```

Then verify:

```bash
curl -fsS http://127.0.0.1:8123/openapi.json >/tmp/openapi.json
curl -fsS http://127.0.0.1:8123/motion/power/status | python3 -m json.tool
```
