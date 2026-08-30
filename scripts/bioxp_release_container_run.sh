#!/bin/bash
# Sole launcher admitted by the canonical bioxp-api.service unit.
if [[ "${BIOXP_LAUNCHER_SANITIZED:-}" != "1" ]]; then
  readonly INCOMING_INVOCATION_ID=${INVOCATION_ID:-}
  if [[ -z "$INCOMING_INVOCATION_ID" ]]; then
    printf '%s\n' "[bioxp-release] launcher requires a systemd invocation" >&2
    exit 78
  fi
  exec /usr/bin/env -i \
    PATH=/usr/sbin:/usr/bin:/sbin:/bin \
    INVOCATION_ID="$INCOMING_INVOCATION_ID" \
    BIOXP_LAUNCHER_SANITIZED=1 \
    /bin/bash --noprofile --norc "$0" "$@"
fi
unset BIOXP_LAUNCHER_SANITIZED
set -euo pipefail

PATH=/usr/sbin:/usr/bin:/sbin:/bin
readonly PATH
readonly SERVICE_UNIT=bioxp-api.service
readonly INSTALLED_LAUNCHER=/usr/local/libexec/bioxp-release-container-run
readonly INSTALLED_UNIT=/etc/systemd/system/bioxp-api.service
readonly RECEIPT_FILE=/etc/bioxp/release-identity.json
readonly SOURCE_MANIFEST_FILE=/etc/bioxp/source-manifest.json
readonly IMAGE_INSPECTION_FILE=/etc/bioxp/image-inspection.json
readonly IMAGE_INSPECTOR=/usr/local/libexec/bioxp-udocker-image-inspector
readonly RUNTIME_DIR=/run/bioxp-release
readonly RUNTIME_BINDING_FILE=/run/bioxp-release/runtime-binding.json
readonly RELEASE_MODE_MARKER=/run/bioxp-release/release-mode
readonly OEM_LOCK_DIR=/var/lib/bioxp-oem-authority
readonly OEM_LOCK_FILE=/var/lib/bioxp-oem-authority/OEM_EVIDENCE_LOCK.json
readonly STATE_DIR=/var/lib/bioxp-oem-runtime
readonly UDOCKER_ROOT=/opt/bioxp/udocker-runtime
readonly UDOCKER_BIN=/opt/bioxp/udocker-runtime/venv/bin/udocker
readonly PORT=8123

fail() {
  printf '%s\n' "[bioxp-release] $*" >&2
  exit 78
}

protected_root_file() {
  local path=$1
  [[ -f "$path" && ! -L "$path" ]] || fail "required root-owned file is absent or not regular: $path"
  [[ "$(/usr/bin/stat -c '%u:%g' "$path")" == "0:0" ]] || fail "required file is not root-owned: $path"
  local mode
  mode=$(/usr/bin/stat -c '%a' "$path")
  (( (8#$mode & 8#022) == 0 )) || fail "required file is group/world writable: $path"
}

sha256_file() {
  /usr/bin/sha256sum "$1" | /usr/bin/cut -d' ' -f1
}

SCRIPT_PATH=$(/usr/bin/readlink -f -- "${BASH_SOURCE[0]}")
[[ "$SCRIPT_PATH" == "$INSTALLED_LAUNCHER" ]] || fail "launcher must be installed at $INSTALLED_LAUNCHER"
[[ -n "${INVOCATION_ID:-}" ]] || fail "launcher requires a systemd invocation"
/usr/bin/grep -Eq '^[0-9]+:[^:]*:/system.slice/bioxp-api\.service$|^0::/system.slice/bioxp-api\.service$' /proc/self/cgroup \
  || fail "launcher is not owned by the exact system-manager $SERVICE_UNIT cgroup"

for protected in "$RECEIPT_FILE" "$SOURCE_MANIFEST_FILE" "$IMAGE_INSPECTION_FILE" "$INSTALLED_UNIT" "$INSTALLED_LAUNCHER" "$IMAGE_INSPECTOR" "$UDOCKER_BIN"; do
  protected_root_file "$protected"
done
[[ -x "$IMAGE_INSPECTOR" ]] || fail "required external robot image inspector is not executable"
[[ -x "$UDOCKER_BIN" ]] || fail "required udocker launcher is unavailable"
[[ -d "$UDOCKER_ROOT" && ! -L "$UDOCKER_ROOT" ]] || fail "immutable udocker runtime root is absent or symlinked"
UDOCKER_SHA256=$(sha256_file "$UDOCKER_BIN")
UDOCKER_TREE_SHA256=$(/usr/bin/python3 - "$UDOCKER_ROOT" <<'PY'
import hashlib
import os
import stat
import sys
from pathlib import Path

root = Path(sys.argv[1])
container_state = root / "store" / "containers"
aggregate = hashlib.sha256()
paths = [
    path for path in (root, *root.rglob("*"))
    if path != container_state and container_state not in path.parents
]
for path in sorted(paths, key=lambda item: item.as_posix().encode("utf-8")):
    info = path.lstat()
    if info.st_uid != 0 or info.st_gid != 0 or (not stat.S_ISLNK(info.st_mode) and info.st_mode & 0o022):
        raise SystemExit(f"mutable or non-root-owned udocker runtime path: {path}")
    relative = "." if path == root else path.relative_to(root).as_posix()
    if stat.S_ISREG(info.st_mode):
        kind = "file"
        identity = hashlib.sha256(path.read_bytes()).hexdigest()
    elif stat.S_ISDIR(info.st_mode):
        kind = "directory"
        identity = ""
    elif stat.S_ISLNK(info.st_mode):
        target = path.resolve(strict=True)
        target_info = target.stat()
        if target_info.st_uid != 0 or target_info.st_gid != 0 or target_info.st_mode & 0o022:
            raise SystemExit(f"udocker runtime symlink resolves to mutable authority: {path}")
        kind = "symlink"
        identity = os.readlink(path)
    else:
        raise SystemExit(f"unsupported udocker runtime file type: {path}")
    aggregate.update(relative.encode() + b"\0" + kind.encode() + b"\0" + str(info.st_size).encode() + b"\0" + identity.encode() + b"\n")
print(aggregate.hexdigest())
PY
) || fail "immutable udocker runtime verification failed"
[[ "$UDOCKER_TREE_SHA256" =~ ^[0-9a-f]{64}$ ]] || fail "udocker runtime aggregate is invalid"

packet_file=$(/usr/bin/mktemp "$RUNTIME_DIR/.identity-packet.XXXXXX")
trap '/bin/rm -f "$packet_file"' EXIT
if ! /usr/bin/python3 - "$RECEIPT_FILE" "$SOURCE_MANIFEST_FILE" "$IMAGE_INSPECTION_FILE" "$IMAGE_INSPECTOR" >"$packet_file" <<'PY'
import hashlib
import json
import re
import sys
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
image_id_pattern = re.compile(r"^sha256:[0-9a-f]{64}$")

def need(condition, message):
    if not condition:
        raise SystemExit(f"[bioxp-release] {message}")

def text(mapping, key):
    value = mapping.get(key)
    need(isinstance(value, str) and bool(value), f"missing string field: {key}")
    return value

def digest_file(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def canonical(value):
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode("utf-8")

need(receipt.get("schema") == "bioxp.release.identity.v1" and receipt.get("status") == "verified", "deployment receipt is not verified v1")
need(manifest.get("schema") == "bioxp.release.source_manifest.v1" and manifest.get("status") == "verified", "source manifest is not verified v1")
need(inspection.get("schema") == "bioxp.release.image_inspection.v1" and inspection.get("status") == "verified", "external image inspection is not verified v1")
source = receipt.get("source")
image = receipt.get("image")
deployment = receipt.get("deployment")
binding = receipt.get("binding")
need(all(isinstance(value, dict) for value in (source, image, deployment, binding)), "receipt sections are incomplete")
commit = text(source, "commit")
tree = text(source, "tree")
mode = text(source, "mode")
root = text(source, "root")
manifest_sha = text(source, "manifest_sha256")
aggregate_sha = text(source, "aggregate_sha256")
image_id = text(image, "id")
inspection_sha = text(image, "inspection_receipt_sha256")
need(hex40.fullmatch(commit) is not None and hex40.fullmatch(tree) is not None, "source commit/tree is not full 40-hex")
need(mode == "exact_commit_materialization", "source materialization mode must be exact_commit_materialization")
need(root == "/app", "runtime source root must be /app")
need(all(hex64.fullmatch(value) is not None for value in (manifest_sha, aggregate_sha, inspection_sha)), "release digest is not full SHA-256")
need(image_id_pattern.fullmatch(image_id) is not None, "image ID must be full immutable sha256:<64-hex>")
need(hashlib.sha256(manifest_raw).hexdigest() == manifest_sha, "source manifest byte digest mismatch")
need(hashlib.sha256(inspection_raw).hexdigest() == inspection_sha, "image inspection receipt byte digest mismatch")
need(manifest.get("commit") == commit and manifest.get("tree") == tree and manifest.get("root") == root, "source manifest commit/tree/root mismatch")
need(manifest.get("aggregate_sha256") == aggregate_sha, "source aggregate digest mismatch")
files = manifest.get("files")
need(
    isinstance(files, list)
    and files
    and type(manifest.get("file_count")) is int
    and manifest.get("file_count") == len(files)
    and type(manifest.get("total_size")) is int
    and manifest.get("total_size") >= 0,
    "source manifest inventory is incomplete",
)
need(files == sorted(files, key=lambda row: str(row.get("path", "")).encode("utf-8")), "source manifest paths are not bytewise sorted")
aggregate = hashlib.sha256()
total = 0
seen = set()
for row in files:
    need(isinstance(row, dict) and set(row) == {"path", "size", "sha256"}, "source inventory row is malformed")
    path = row.get("path")
    size = row.get("size")
    digest = row.get("sha256")
    need(isinstance(path, str) and path and not path.startswith("/") and ".." not in Path(path).parts and path not in seen, "source inventory path is unsafe or duplicate")
    need(type(size) is int and size >= 0 and isinstance(digest, str) and hex64.fullmatch(digest), "source inventory size/digest is malformed")
    seen.add(path)
    total += size
    aggregate.update(path.encode("utf-8") + b"\0" + str(size).encode("ascii") + b"\0" + digest.encode("ascii") + b"\n")
need(total == manifest.get("total_size") and aggregate.hexdigest() == aggregate_sha, "source inventory aggregate is not canonical")
need(binding.get("service_unit") == "bioxp-api.service" and binding.get("unit_path") == "/etc/systemd/system/bioxp-api.service", "service unit binding mismatch")
need(binding.get("launcher_path") == "/usr/local/libexec/bioxp-release-container-run", "launcher path binding mismatch")
unit_sha = text(binding, "unit_sha256")
launcher_sha = text(binding, "launcher_sha256")
config_sha = text(binding, "configuration_sha256")
oem_lock_sha = text(binding, "oem_lock_sha256")
need(all(hex64.fullmatch(value) is not None for value in (unit_sha, launcher_sha, config_sha, oem_lock_sha)), "binding digest is not full SHA-256")
need(binding.get("oem_lock_path") == "/var/lib/bioxp-oem-authority/OEM_EVIDENCE_LOCK.json", "OEM lock path binding mismatch")
configuration = binding.get("configuration")
expected_configuration = {
    "declared_listener": {"host": "0.0.0.0", "port": 8123},
    "database_root": "/app/.oem_runtime_state",
    "working_directory": "/app",
    "argv": ["python", "-m", "uvicorn", "bioxp.api:app", "--host", "0.0.0.0", "--port", "8123"],
    "environment": {
        "BIOXP_OEM_MACHINE_BUNDLE_LOCK": "/app/.oem_lock/OEM_EVIDENCE_LOCK.json",
        "BIOXP_OEM_RUNTIME_ROOT": "/app/.oem_runtime_state",
        "BIOXP_OEM_RUNTIME_STATE_ROOT": "/app/.oem_runtime_state",
        "BIOXP_PHYSICAL_LABEL_SERIAL": "206",
        "PYTHONPATH": "/app/src",
    },
}
need(configuration == expected_configuration, "runtime configuration is not canonical")
need(hashlib.sha256(canonical(configuration)).hexdigest() == config_sha, "canonical runtime configuration digest mismatch")
need(inspection.get("requested_image_id") == image_id and inspection.get("resolved_local_image_id") == image_id, "external inspector did not resolve the exact immutable local image ID")
need(inspection.get("source_bytes_verified") is True, "external inspector did not verify image source bytes")
verifier = inspection.get("verifier")
need(isinstance(verifier, dict) and verifier.get("path") == str(inspector_path), "external image verifier path mismatch")
verifier_sha = text(verifier, "sha256")
need(hex64.fullmatch(verifier_sha) is not None and digest_file(inspector_path) == verifier_sha, "external image verifier executable digest mismatch")
labels = inspection.get("oci_labels")
embedded = inspection.get("embedded_source_manifest")
need(isinstance(labels, dict) and isinstance(embedded, dict), "external image inspection labels/manifest are incomplete")
need(labels.get("org.opencontainers.image.revision") == commit and labels.get("com.bioxp.source.tree") == tree, "OCI revision/tree labels mismatch")
need(labels.get("com.bioxp.source.manifest.sha256") == manifest_sha, "OCI source-manifest label mismatch")
need(embedded.get("path") == "/usr/share/bioxp-release/source-manifest.json" and embedded.get("sha256") == manifest_sha, "embedded source manifest digest mismatch")
need(embedded.get("aggregate_sha256") == aggregate_sha, "embedded source aggregate mismatch")
release_id = text(receipt, "release_id")
receipt_id = text(deployment, "receipt_id")
installed_at = text(deployment, "installed_at")
host_source = source.get("host_path")
need(host_source == f"/opt/bioxp/releases/{commit}", "exact materialization path must be /opt/bioxp/releases/<commit>")
values = [release_id, receipt_id, installed_at, commit, tree, mode, root, str(host_source), manifest_sha, aggregate_sha, image_id, inspection_sha, unit_sha, launcher_sha, config_sha, oem_lock_sha, hashlib.sha256(receipt_raw).hexdigest()]
for value in values:
    sys.stdout.buffer.write(str(value).encode("utf-8") + b"\0")
PY
then
  fail "release packet validation failed"
fi
mapfile -d '' PACKET <"$packet_file"
/bin/rm -f "$packet_file"
trap - EXIT
[[ ${#PACKET[@]} -eq 17 ]] || fail "release identity parser returned an incomplete packet"
RELEASE_ID=${PACKET[0]}
DEPLOYMENT_RECEIPT_ID=${PACKET[1]}
INSTALLED_AT=${PACKET[2]}
SOURCE_COMMIT=${PACKET[3]}
SOURCE_TREE=${PACKET[4]}
SOURCE_MODE=${PACKET[5]}
SOURCE_ROOT=${PACKET[6]}
HOST_SOURCE=${PACKET[7]}
SOURCE_MANIFEST_SHA256=${PACKET[8]}
SOURCE_AGGREGATE_SHA256=${PACKET[9]}
IMAGE_ID=${PACKET[10]}
IMAGE_INSPECTION_SHA256=${PACKET[11]}
UNIT_SHA256=${PACKET[12]}
LAUNCHER_SHA256=${PACKET[13]}
CONFIGURATION_SHA256=${PACKET[14]}
OEM_LOCK_SHA256=${PACKET[15]}
CANONICAL_RECEIPT_SHA256=${PACKET[16]}

[[ "$(sha256_file "$INSTALLED_UNIT")" == "$UNIT_SHA256" ]] || fail "installed canonical unit bytes do not match the deployment receipt"
[[ "$(sha256_file "$INSTALLED_LAUNCHER")" == "$LAUNCHER_SHA256" ]] || fail "installed release launcher bytes do not match the deployment receipt"
"$IMAGE_INSPECTOR" verify \
  --receipt "$IMAGE_INSPECTION_FILE" \
  --source-manifest "$SOURCE_MANIFEST_FILE" \
  --image-id "$IMAGE_ID" >/dev/null \
  || fail "live local image store no longer matches the sealed image inspection"

[[ "$SOURCE_MODE" == exact_commit_materialization ]] || fail "source mode is not exact_commit_materialization"
[[ "$HOST_SOURCE" == "/opt/bioxp/releases/$SOURCE_COMMIT" ]] || fail "materialized source path is not canonical"
[[ -d "$HOST_SOURCE" && ! -L "$HOST_SOURCE" ]] || fail "exact commit materialization is absent or symlinked"
if ! /usr/bin/python3 - "$HOST_SOURCE" <<'PY'
import os, stat, sys
from pathlib import Path
root = Path(sys.argv[1])
for path in [root, *root.rglob("*")]:
    info = path.lstat()
    if stat.S_ISLNK(info.st_mode) or info.st_uid != 0 or info.st_gid != 0 or info.st_mode & 0o222:
        raise SystemExit(f"mutable, non-root-owned, or symlinked release source: {path}")
PY
then
  fail "exact commit materialization is not recursively root-owned and non-writable"
fi
if ! /usr/bin/python3 - "$HOST_SOURCE" "$SOURCE_MANIFEST_FILE" <<'PY'
import hashlib, json, os, stat, sys
from pathlib import Path
root = Path(sys.argv[1])
manifest_path = Path(sys.argv[2])
manifest = json.loads(manifest_path.read_bytes())
rows = manifest.get("files")
if not isinstance(rows, list) or manifest.get("file_count") != len(rows):
    raise SystemExit("source manifest inventory is incomplete")
expected = {}
for row in rows:
    if not isinstance(row, dict) or set(row) != {"path", "size", "sha256"}:
        raise SystemExit("source manifest row is malformed")
    relative = row["path"]
    if not isinstance(relative, str) or not relative or relative.startswith("/") or ".." in Path(relative).parts or relative in expected:
        raise SystemExit("source manifest path is unsafe or duplicate")
    expected[relative] = (row["size"], row["sha256"])
observed = {}
for directory, dirnames, filenames in os.walk(root, topdown=True, followlinks=False):
    base = Path(directory)
    if base == root:
        dirnames[:] = [name for name in dirnames if name not in {".git", ".bioxp-release"}]
    for name in [*dirnames, *filenames]:
        candidate = base / name
        info = candidate.lstat()
        if stat.S_ISLNK(info.st_mode):
            raise SystemExit("mounted source contains a symlink")
    for name in filenames:
        candidate = base / name
        info = candidate.lstat()
        if not stat.S_ISREG(info.st_mode):
            raise SystemExit("mounted source contains a non-regular file")
        relative = candidate.relative_to(root).as_posix()
        observed[relative] = (info.st_size, hashlib.sha256(candidate.read_bytes()).hexdigest())
if observed != expected:
    raise SystemExit("mounted source bytes differ from the deterministic manifest")
PY
then
  fail "mounted source bytes do not match the deterministic manifest"
fi

[[ -d "$OEM_LOCK_DIR" && ! -L "$OEM_LOCK_DIR" ]] || fail "OEM authority directory is unavailable"
protected_root_file "$OEM_LOCK_FILE"
[[ "$(sha256_file "$OEM_LOCK_FILE")" == "$OEM_LOCK_SHA256" ]] || fail "OEM authority lock bytes do not match the deployment receipt"
[[ -d "$STATE_DIR" && -w "$STATE_DIR" && ! -L "$STATE_DIR" ]] || fail "canonical durable state root is unavailable or not writable"
[[ -d "$RUNTIME_DIR" && -w "$RUNTIME_DIR" && ! -L "$RUNTIME_DIR" ]] || fail "systemd runtime directory is unavailable"

if /usr/bin/ss -H -ltn "sport = :$PORT" 2>/dev/null | /usr/bin/grep -q .; then
  fail "canonical listener port $PORT is already owned"
fi

LAUNCHER_CGROUP=$(/usr/bin/cat /proc/self/cgroup)
/usr/bin/python3 - "$RUNTIME_BINDING_FILE" "$LAUNCHER_CGROUP" "$$" "$RELEASE_ID" "$DEPLOYMENT_RECEIPT_ID" "$INSTALLED_AT" "$CANONICAL_RECEIPT_SHA256" "$SOURCE_MANIFEST_SHA256" "$SOURCE_AGGREGATE_SHA256" "$IMAGE_ID" "$IMAGE_INSPECTION_SHA256" "$UNIT_SHA256" "$LAUNCHER_SHA256" "$CONFIGURATION_SHA256" "$OEM_LOCK_SHA256" "$INVOCATION_ID" "$UDOCKER_BIN" "$UDOCKER_SHA256" "$UDOCKER_TREE_SHA256" <<'PY'
import hashlib, json, os, sys, time
from pathlib import Path
(path_raw, cgroup, launcher_pid, release_id, deployment_receipt_id, installed_at, canonical_receipt_sha256, source_manifest_sha256, source_aggregate_sha256, image_id, image_inspection_sha256, unit_sha256, launcher_sha256, configuration_sha256, oem_lock_sha256, invocation_id, udocker_path, udocker_sha256, udocker_tree_sha256) = sys.argv[1:]
payload = {
    "schema": "bioxp.release.runtime_binding.v1",
    "status": "verified",
    "release_id": release_id,
    "deployment_receipt_id": deployment_receipt_id,
    "installed_at": installed_at,
    "canonical_receipt_sha256": canonical_receipt_sha256,
    "source_manifest_sha256": source_manifest_sha256,
    "source_aggregate_sha256": source_aggregate_sha256,
    "image_id": image_id,
    "image_inspection_receipt_sha256": image_inspection_sha256,
    "udocker_path": udocker_path,
    "udocker_sha256": udocker_sha256,
    "udocker_tree_sha256": udocker_tree_sha256,
    "service_unit": "bioxp-api.service",
    "unit_sha256": unit_sha256,
    "launcher_sha256": launcher_sha256,
    "configuration_sha256": configuration_sha256,
    "oem_lock_sha256": oem_lock_sha256,
    "launcher_pid": int(launcher_pid),
    "launcher_cgroup": cgroup,
    "launcher_cgroup_sha256": hashlib.sha256((cgroup.strip() + "\n").encode()).hexdigest(),
    "systemd_invocation_id": invocation_id,
    "declared_listener": {"host": "0.0.0.0", "port": 8123},
    "observed_listener": None,
    "database_root": "/app/.oem_runtime_state",
    "binding_created_at": time.time(),
}
path = Path(path_raw)
temporary = path.with_name(path.name + ".tmp")
temporary.write_text(json.dumps(payload, sort_keys=True, separators=(",", ":")) + "\n", encoding="utf-8")
os.chmod(temporary, 0o444)
os.replace(temporary, path)
PY
printf 'release-mode\n' >"$RELEASE_MODE_MARKER"
/bin/chmod 0444 "$RELEASE_MODE_MARKER"

IMAGE_REF=$(/usr/bin/python3 - "$UDOCKER_ROOT/store" "$IMAGE_ID" <<'PY'
import hashlib, sys
from pathlib import Path
store = Path(sys.argv[1]).resolve()
digest = sys.argv[2].removeprefix("sha256:")
matches = []
for tag in sorted((store / "repos").glob("*/*")):
    config = tag / "container.json"
    if not config.is_file():
        config = tag / f"{digest}.layer"
    if config.is_file() and hashlib.sha256(config.read_bytes()).hexdigest() == digest:
        matches.append(f"{tag.parent.name}:{tag.name}")
if len(matches) != 1:
    raise SystemExit("immutable image ID does not resolve to exactly one local uDocker reference")
print(matches[0])
PY
) || fail "immutable local uDocker image reference resolution failed"

SOURCE_VOLUME=()
[[ "$SOURCE_MODE" == exact_commit_materialization ]] || fail "unsupported source mode"
SOURCE_VOLUME=(--volume="$HOST_SOURCE:/app:ro")

exec "$UDOCKER_BIN" --repo="$UDOCKER_ROOT/store" run \
  --pull=never \
  --user=root \
  --env="BIOXP_RELEASE_ID=$RELEASE_ID" \
  --env="BIOXP_RELEASE_IMAGE_ID=$IMAGE_ID" \
  --env="BIOXP_RELEASE_SOURCE_COMMIT=$SOURCE_COMMIT" \
  --env="BIOXP_RELEASE_UDOCKER_SHA256=$UDOCKER_SHA256" \
  --env="BIOXP_RELEASE_UDOCKER_TREE_SHA256=$UDOCKER_TREE_SHA256" \
  "${SOURCE_VOLUME[@]}" \
  --volume="$RUNTIME_DIR:/run/bioxp-release:ro" \
  --volume="$OEM_LOCK_DIR:/app/.oem_lock:ro" \
  --volume="$STATE_DIR:/app/.oem_runtime_state" \
  --volume=/dev:/dev \
  --volume=/run/udev:/run/udev:ro \
  --workdir=/app \
  "$IMAGE_REF" \
  /bin/sh -lc 'PYTHONPATH=/app/src BIOXP_OEM_MACHINE_BUNDLE_LOCK=/app/.oem_lock/OEM_EVIDENCE_LOCK.json BIOXP_PHYSICAL_LABEL_SERIAL=206 BIOXP_OEM_RUNTIME_ROOT=/app/.oem_runtime_state BIOXP_OEM_RUNTIME_STATE_ROOT=/app/.oem_runtime_state exec python -m uvicorn bioxp.api:app --host 0.0.0.0 --port 8123'
