#!/usr/bin/env bash
set -euo pipefail

if [[ ${EUID} -ne 0 ]]; then
  echo "ERROR: run this installer as root" >&2
  exit 77
fi

UNIT=/etc/systemd/system/bioxp-api.service
LAUNCHER=/usr/local/libexec/bioxp-release-container-run
IDENTITY_DIR=/etc/bioxp
RECEIPT=${IDENTITY_DIR}/release-identity.json
SOURCE_MANIFEST=${IDENTITY_DIR}/source-manifest.json
IMAGE_INSPECTION=${IDENTITY_DIR}/image-inspection.json
IMAGE_INSPECTOR=/usr/local/libexec/bioxp-udocker-image-inspector
RULE=/etc/polkit-1/rules.d/49-bioxp-api-management.rules
UDOCKER_ROOT=/opt/bioxp/udocker-runtime
UDOCKER_BIN=${UDOCKER_ROOT}/venv/bin/udocker

for identity in "${RECEIPT}" "${SOURCE_MANIFEST}" "${IMAGE_INSPECTION}" "${IMAGE_INSPECTOR}"; do
  [[ -f ${identity} && ! -L ${identity} ]] || { echo "ERROR: external immutable identity file is required: ${identity}" >&2; exit 78; }
  [[ $(stat -c '%u:%g' "${identity}") == 0:0 ]] || { echo "ERROR: external release identity files must be root-owned" >&2; exit 78; }
  mode=$(stat -c '%a' "${identity}")
  (( (8#${mode} & 8#022) == 0 )) || { echo "ERROR: identity file is group/world writable: ${identity}" >&2; exit 78; }
done
[[ -x ${IMAGE_INSPECTOR} ]] || { echo "ERROR: external image inspector is not executable" >&2; exit 78; }

[[ -d ${UDOCKER_ROOT} && ! -L ${UDOCKER_ROOT} && -x ${UDOCKER_BIN} && ! -L ${UDOCKER_BIN} ]] \
  || { echo "ERROR: immutable udocker runtime is absent or symlinked" >&2; exit 78; }
/usr/bin/python3 - "${UDOCKER_ROOT}" <<'PY'
import stat
import sys
from pathlib import Path

root = Path(sys.argv[1])
for path in (root, *root.rglob("*")):
    info = path.lstat()
    if info.st_uid != 0 or info.st_gid != 0 or info.st_mode & 0o022:
        raise SystemExit(f"udocker runtime is mutable or non-root-owned: {path}")
    if stat.S_ISLNK(info.st_mode):
        target = path.resolve(strict=True)
        target_info = target.stat()
        if target_info.st_uid != 0 or target_info.st_gid != 0 or target_info.st_mode & 0o022:
            raise SystemExit(f"udocker runtime symlink resolves to mutable authority: {path}")
PY

packet=$(mktemp)
rule_tmp=$(mktemp)
trap 'rm -f "$packet" "$rule_tmp"' EXIT
/usr/bin/python3 - "${RECEIPT}" "${SOURCE_MANIFEST}" "${IMAGE_INSPECTION}" "${IMAGE_INSPECTOR}" >"${packet}" <<'PY'
import hashlib, json, re, sys
from pathlib import Path
receipt_path, manifest_path, inspection_path, inspector_path = map(Path, sys.argv[1:])
receipt_raw = receipt_path.read_bytes()
manifest_raw = manifest_path.read_bytes()
inspection_raw = inspection_path.read_bytes()
receipt = json.loads(receipt_raw)
manifest = json.loads(manifest_raw)
inspection = json.loads(inspection_raw)
hex40 = re.compile(r"^[0-9a-f]{40}$")
hex64 = re.compile(r"^[0-9a-f]{64}$")
def need(value, message):
    if not value:
        raise SystemExit(message)
need(receipt.get("schema") == "bioxp.release.identity.v1" and receipt.get("status") == "verified", "receipt is not verified v1")
need(manifest.get("schema") == "bioxp.release.source_manifest.v1" and manifest.get("status") == "verified", "manifest is not verified v1")
need(inspection.get("schema") == "bioxp.release.image_inspection.v1" and inspection.get("status") == "verified", "image inspection is not verified v1")
source = receipt.get("source")
image = receipt.get("image")
binding = receipt.get("binding")
need(isinstance(source, dict) and isinstance(image, dict) and isinstance(binding, dict), "receipt source/image/binding missing")
commit = source.get("commit")
root = source.get("host_path")
need(isinstance(commit, str) and hex40.fullmatch(commit), "commit is not canonical")
need(source.get("mode") == "exact_commit_materialization", "installer accepts only exact_commit_materialization")
need(root == f"/opt/bioxp/releases/{commit}", "materialization path is not canonical")
manifest_sha = source.get("manifest_sha256")
need(isinstance(manifest_sha, str) and hex64.fullmatch(manifest_sha), "manifest digest is not canonical")
need(hashlib.sha256(manifest_raw).hexdigest() == manifest_sha, "manifest byte digest mismatch")
need(manifest.get("commit") == commit and manifest.get("tree") == source.get("tree") and manifest.get("root") == "/app", "manifest source identity mismatch")
image_id = image.get("id")
inspection_sha = image.get("inspection_receipt_sha256")
need(isinstance(image_id, str) and re.fullmatch(r"sha256:[0-9a-f]{64}", image_id), "image ID is not immutable")
need(isinstance(inspection_sha, str) and hex64.fullmatch(inspection_sha), "image inspection digest is not canonical")
need(hashlib.sha256(inspection_raw).hexdigest() == inspection_sha, "image inspection byte digest mismatch")
need(inspection.get("requested_image_id") == image_id and inspection.get("resolved_local_image_id") == image_id and inspection.get("source_bytes_verified") is True, "image inspection identity mismatch")
verifier = inspection.get("verifier")
need(isinstance(verifier, dict) and verifier.get("path") == str(inspector_path), "image inspector path mismatch")
need(isinstance(verifier.get("sha256"), str) and hashlib.sha256(inspector_path.read_bytes()).hexdigest() == verifier["sha256"], "image inspector digest mismatch")
need(binding.get("unit_path") == "/etc/systemd/system/bioxp-api.service", "unit path mismatch")
need(binding.get("launcher_path") == "/usr/local/libexec/bioxp-release-container-run", "launcher path mismatch")
unit_sha = binding.get("unit_sha256")
launcher_sha = binding.get("launcher_sha256")
need(isinstance(unit_sha, str) and hex64.fullmatch(unit_sha), "unit digest is not canonical")
need(isinstance(launcher_sha, str) and hex64.fullmatch(launcher_sha), "launcher digest is not canonical")
for value in (root, unit_sha, launcher_sha, image_id):
    sys.stdout.buffer.write(value.encode() + b"\0")
PY
mapfile -d '' MATERIALIZATION <"${packet}"
[[ ${#MATERIALIZATION[@]} -eq 4 ]] || { echo "ERROR: verified materialization packet incomplete" >&2; exit 78; }
REPOSITORY_ROOT=${MATERIALIZATION[0]}
EXPECTED_UNIT_SHA256=${MATERIALIZATION[1]}
EXPECTED_LAUNCHER_SHA256=${MATERIALIZATION[2]}
EXPECTED_IMAGE_ID=${MATERIALIZATION[3]}
UNIT_SOURCE=${REPOSITORY_ROOT}/systemd/bioxp-api.service
LAUNCHER_SOURCE=${REPOSITORY_ROOT}/scripts/bioxp_release_container_run.sh

[[ -d ${REPOSITORY_ROOT} && ! -L ${REPOSITORY_ROOT} ]] || { echo "ERROR: exact materialization is absent or symlinked" >&2; exit 78; }
[[ -f ${UNIT_SOURCE} && ! -L ${UNIT_SOURCE} ]] || { echo "ERROR: canonical unit template absent" >&2; exit 78; }
[[ -f ${LAUNCHER_SOURCE} && ! -L ${LAUNCHER_SOURCE} ]] || { echo "ERROR: canonical release launcher absent" >&2; exit 78; }
/usr/bin/python3 - "${REPOSITORY_ROOT}" <<'PY'
import stat, sys
from pathlib import Path
root = Path(sys.argv[1])
for path in (root, *root.rglob("*")):
    info = path.lstat()
    if stat.S_ISLNK(info.st_mode) or info.st_uid != 0 or info.st_gid != 0 or info.st_mode & 0o222:
        raise SystemExit(f"materialization is mutable, non-root-owned, or symlinked: {path}")
PY
/usr/bin/python3 - "${REPOSITORY_ROOT}" "${SOURCE_MANIFEST}" <<'PY'
import hashlib
import json
import os
import stat
import sys
from pathlib import Path

root = Path(sys.argv[1])
manifest = json.loads(Path(sys.argv[2]).read_bytes())
rows = manifest.get("files")
if not isinstance(rows, list) or manifest.get("file_count") != len(rows):
    raise SystemExit("source manifest inventory is incomplete")
expected = {}
for row in rows:
    if not isinstance(row, dict) or set(row) != {"path", "size", "sha256"}:
        raise SystemExit("source manifest row is malformed")
    relative = row["path"]
    if (
        not isinstance(relative, str)
        or not relative
        or relative.startswith("/")
        or ".." in Path(relative).parts
        or relative in expected
        or type(row["size"]) is not int
        or row["size"] < 0
        or not isinstance(row["sha256"], str)
        or len(row["sha256"]) != 64
    ):
        raise SystemExit("source manifest row identity is invalid")
    expected[relative] = (row["size"], row["sha256"])
observed = {}
for directory, dirnames, filenames in os.walk(root, topdown=True, followlinks=False):
    base = Path(directory)
    if base == root:
        dirnames[:] = [name for name in dirnames if name not in {".git", ".bioxp-release"}]
    for name in [*dirnames, *filenames]:
        candidate = base / name
        if stat.S_ISLNK(candidate.lstat().st_mode):
            raise SystemExit("materialized source contains a symlink")
    for name in filenames:
        candidate = base / name
        info = candidate.lstat()
        if not stat.S_ISREG(info.st_mode):
            raise SystemExit("materialized source contains a non-regular file")
        relative = candidate.relative_to(root).as_posix()
        observed[relative] = (info.st_size, hashlib.sha256(candidate.read_bytes()).hexdigest())
if observed != expected:
    raise SystemExit("materialized source bytes differ from the deterministic manifest")
PY
/usr/bin/python3 "${REPOSITORY_ROOT}/scripts/bioxp_source_manifest.py" verify \
  --root "${REPOSITORY_ROOT}" --manifest "${SOURCE_MANIFEST}" >/dev/null
"${IMAGE_INSPECTOR}" verify \
  --receipt "${IMAGE_INSPECTION}" \
  --source-manifest "${SOURCE_MANIFEST}" \
  --image-id "${EXPECTED_IMAGE_ID}" >/dev/null
[[ $(sha256sum "${UNIT_SOURCE}" | cut -d' ' -f1) == "${EXPECTED_UNIT_SHA256}" ]] || { echo "ERROR: materialized unit digest mismatch" >&2; exit 78; }
[[ $(sha256sum "${LAUNCHER_SOURCE}" | cut -d' ' -f1) == "${EXPECTED_LAUNCHER_SHA256}" ]] || { echo "ERROR: materialized launcher digest mismatch" >&2; exit 78; }

id molbiofreak >/dev/null 2>&1 || { echo "ERROR: molbiofreak account absent" >&2; exit 78; }
command -v systemctl >/dev/null
[[ -d /etc/polkit-1/rules.d ]] || { echo "ERROR: polkit rules directory absent" >&2; exit 78; }

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

install -d -o root -g root -m 0755 /usr/local/libexec
install -o root -g root -m 0755 "${LAUNCHER_SOURCE}" "${LAUNCHER}"
install -o root -g root -m 0644 "${UNIT_SOURCE}" "${UNIT}"
install -o root -g root -m 0644 "${rule_tmp}" "${RULE}"

systemctl daemon-reload
systemd-analyze verify bioxp-api.service
fragment=$(systemctl show bioxp-api.service -p FragmentPath --value)
dropins=$(systemctl show bioxp-api.service -p DropInPaths --value)
exec_start=$(systemctl show bioxp-api.service -p ExecStart --value)
[[ ${fragment} == "${UNIT}" ]] || { echo "ERROR: loaded FragmentPath is not canonical: ${fragment}" >&2; exit 78; }
[[ -z ${dropins} ]] || { echo "ERROR: loaded unit has forbidden DropInPaths: ${dropins}" >&2; exit 78; }
/usr/bin/python3 - "${exec_start}" "${LAUNCHER}" <<'PY'
import sys
value, launcher = sys.argv[1:]
fields = {}
for part in value.strip().strip("{}").split(";"):
    key, sep, selected = part.strip().partition("=")
    if sep:
        fields[key.strip()] = selected.strip()
if fields.get("path") != launcher or fields.get("argv[]") != launcher:
    raise SystemExit(f"loaded ExecStart is not exact canonical launcher: {value}")
PY
[[ $(sha256sum "${UNIT}" | cut -d' ' -f1) == "${EXPECTED_UNIT_SHA256}" ]] || { echo "ERROR: installed unit digest mismatch after daemon-reload" >&2; exit 78; }
[[ $(sha256sum "${LAUNCHER}" | cut -d' ' -f1) == "${EXPECTED_LAUNCHER_SHA256}" ]] || { echo "ERROR: installed launcher digest mismatch after daemon-reload" >&2; exit 78; }

printf '%s\n' \
  'INSTALLED: exact verified materialization unit bytes' \
  'INSTALLED: exact verified materialization launcher bytes' \
  'VERIFIED: loaded FragmentPath/ExecStart with no drop-ins' \
  'INSTALLED: exact-unit BioXP Polkit authorization' \
  'NOT STARTED: release start remains gated by full image/source/deployment identities'
sha256sum "${UNIT}" "${LAUNCHER}" "${SOURCE_MANIFEST}" "${RECEIPT}"
