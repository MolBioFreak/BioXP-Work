#!/usr/bin/env bash
set -euo pipefail

if [[ ${EUID} -ne 0 ]]; then
  echo "ERROR: run this installer as root" >&2
  exit 77
fi

UNIT=/etc/systemd/system/bioxp-api.service
RULE=/etc/polkit-1/rules.d/49-bioxp-api-management.rules
DROPIN_DIR=/etc/systemd/system/bioxp-api.service.d
OVERRIDE=${DROPIN_DIR}/override.conf
EXPECTED_EXEC='/home/molbiofreak/bioxp_re/.venv/bin/uvicorn bioxp.api:app --host 0.0.0.0 --port 8123'

[[ -f ${UNIT} ]] || { echo "ERROR: ${UNIT} is absent" >&2; exit 78; }
grep -Fq "ExecStart=${EXPECTED_EXEC}" "${UNIT}" || {
  echo "ERROR: refusing unexpected bioxp-api.service ExecStart" >&2
  exit 78
}
id molbiofreak >/dev/null 2>&1 || { echo "ERROR: molbiofreak account absent" >&2; exit 78; }
command -v systemctl >/dev/null
[[ -d /etc/polkit-1/rules.d ]] || { echo "ERROR: polkit rules directory absent" >&2; exit 78; }

rule_tmp=$(mktemp)
override_tmp=$(mktemp)
trap 'rm -f "$rule_tmp" "$override_tmp"' EXIT

cat >"${rule_tmp}" <<'RULE'
/* Exact-unit authorization for the BioXP handler operator.
 * No sudo, shell, unit-file editing, daemon-reload, reboot, or other unit access.
 */
polkit.addRule(function(action, subject) {
    if (action.id !== "org.freedesktop.systemd1.manage-units") {
        return polkit.Result.NOT_HANDLED;
    }
    if (subject.user !== "molbiofreak") {
        return polkit.Result.NOT_HANDLED;
    }
    if (action.lookup("unit") !== "bioxp-api.service") {
        return polkit.Result.NOT_HANDLED;
    }
    var verb = action.lookup("verb");
    if (verb === "start" || verb === "stop" || verb === "restart" || verb === "reset-failed") {
        return polkit.Result.YES;
    }
    return polkit.Result.NOT_HANDLED;
});
RULE

cat >"${override_tmp}" <<'OVERRIDE'
[Unit]
StartLimitIntervalSec=60
StartLimitBurst=3

[Service]
Restart=on-failure
RestartSec=3s
TimeoutStopSec=20s
KillMode=control-group
SendSIGKILL=yes
OVERRIDE

install -o root -g root -m 0644 "${rule_tmp}" "${RULE}"
install -d -o root -g root -m 0755 "${DROPIN_DIR}"
install -o root -g root -m 0644 "${override_tmp}" "${OVERRIDE}"

systemctl daemon-reload
systemd-analyze verify bioxp-api.service

printf '%s\n' \
  'INSTALLED: exact-unit BioXP Polkit authorization' \
  'INSTALLED: hardened bioxp-api.service restart/termination policy'
systemctl show bioxp-api.service \
  -p FragmentPath -p DropInPaths -p Restart -p RestartUSec \
  -p TimeoutStopUSec -p KillMode -p SendSIGKILL -p StartLimitBurst
