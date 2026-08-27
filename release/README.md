# BioXP canonical immutable release packet

The repository owns exactly one production unit template: `systemd/bioxp-api.service`. The installed unit calls only `/usr/local/libexec/bioxp-release-container-run`. Generic container runners, watchdogs, host-venv recovery units, cron jobs, and manual `uvicorn` processes are not runtime owners.

A release installer must create three root-owned, non-symlink, non-writable files before the unit can start:

- `/etc/bioxp/source-manifest.json`, conforming to `release/source-manifest.schema.json`;
- `/etc/bioxp/release-identity.json`, conforming to `release/release-identity.schema.json`;
- `/etc/bioxp/image-inspection.json`, conforming to `release/image-inspection.schema.json` and produced by the external root-owned `/usr/local/libexec/bioxp-udocker-image-inspector` authority.

The same installer must provide a root-owned, recursively non-writable uDocker runtime at `/opt/bioxp/udocker-runtime`, with its launcher at `/opt/bioxp/udocker-runtime/venv/bin/udocker`. The external image inspector must support this launch-time verification contract:

```text
/usr/local/libexec/bioxp-udocker-image-inspector verify --receipt /etc/bioxp/image-inspection.json --source-manifest /etc/bioxp/source-manifest.json --image-id sha256:<64-hex>
```

The command exits zero only when the current local image store still resolves that full image ID to the inspected source bytes and embedded manifest.

The packet must contain identities measured after the final source commit and image import:

1. exact 40-hex BioXP source commit and tree;
2. a full immutable local image ID in `sha256:<64 lowercase hex>` form (a tag or short udocker ID is forbidden);
3. source mode `exact_commit_materialization`, with a root-owned read-only exact commit mounted at `/app` from the required `host_path`;
4. SHA-256 of the exact source-manifest bytes;
5. SHA-256 of the external image-inspection receipt and its independently installed verifier executable, with exact OCI commit/tree/manifest labels and embedded source-manifest bytes verified;
6. SHA-256 of the installed canonical unit bytes and installed launcher bytes;
7. SHA-256 of the root-owned OEM authority lock and canonical runtime configuration;
8. a unique deployment receipt ID and installation timestamp;
9. the canonical listener `0.0.0.0:8123` and container database root `/app/.oem_runtime_state`.

The launcher checks those identities, the external inspector executable digest, exact cgroup ownership, listener vacancy, source materialization, OEM lock, and writable durable state before executing the immutable image. It creates a per-invocation runtime binding under `/run/bioxp-release` and mounts the canonical receipt, source manifest, image-inspection receipt, and runtime binding read-only. The application loads only that packet. Raw environment release labels are never source or deployment authority.

If `/run/bioxp-release/release-mode` is present, malformed, missing, short, tag-only, contradictory, writable, or non-canonical identity evidence aborts startup. Outside release mode the application preserves an explicit `unverified` identity instead of manufacturing deployment proof.

## Repository-owned producers

The immutable packet is produced offline, in this order:

1. `scripts/bioxp_source_manifest.py materialize` creates the exact-commit source tree and canonical source-manifest bytes.
2. `scripts/bioxp_udocker_image_import.py` imports a normalized image bundle into a staging store, invokes `bioxp_udocker_image_inspector.py`, and publishes the store plus inspection receipt only after verification. The required bundle layout is `oci-config.json` plus `rootfs/`; the full image ID is exactly `sha256(oci-config.json)` and the embedded manifest is `rootfs/usr/share/bioxp-release/source-manifest.json`. Tags and shortened IDs are rejected.
3. `scripts/bioxp_release_seal.py` recomputes the source inventory aggregate, source-manifest byte digest, image-inspection byte digest and cross-bindings, inspector digest, unit/launcher/OEM-lock digests, and canonical runtime-configuration digest before emitting `release-identity.json`. Output JSON is sorted, compact, UTF-8, newline-terminated, and deterministic when the explicit release ID, deployment receipt ID, installation time, and inspection time are unchanged.
4. `scripts/bootstrap_bioxp_immutable_release.py --candidate PACKET --publication-root ROOT` copies and revalidates one recursively owner-bound, non-writable, symlink-free packet, stages it under `ROOT`, and replaces `ROOT/current` using same-filesystem renames. Any exception or supported fault injection restores the complete previous packet; no mixed old/new packet is retained.

The image inspector's `verify` command defaults to the canonical normalized store at `/opt/bioxp/udocker-runtime/store`, so the installer and launcher contract remains:

```text
bioxp-udocker-image-inspector verify --receipt RECEIPT --source-manifest MANIFEST --image-id sha256:<64-hex>
```

Producer inputs and published authority default to UID 0. The importer and bootstrap expose `--required-uid` solely so hardware-free tests can exercise the same ownership checks in temporary user-owned trees. Production must not override the default.
