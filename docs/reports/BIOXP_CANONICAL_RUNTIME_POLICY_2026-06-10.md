# BioXP Canonical Runtime Policy

Date: 2026-06-10

## Canonical reference

The canonical BioXP runtime is the running robot handler checkout at:

```text
/home/molbiofreak/bioxp_re
```

The live service bind-mounts that checkout into the robot handler container as `/app` and runs:

```text
PYTHONPATH=/app/src BIOXP_RUNTIME_OWNER=udocker-container BIOXP_OEM_MACHINE_CONFIG_DIR=/app/config/oem/original_ssd_appdata_20260610 python -m uvicorn bioxp.api:app --host 0.0.0.0 --port 8123
```

All future movement/runtime iteration must start from this checkout and the committed runtime state after the movement-readiness work.

## Deprecated / removed

The following are not valid runtime references and were removed from the live repo because they can be mistaken for active code:

- `src/bioxp/*.py.bak-*`
- `tests/*.py.bak-*`
- `backup-bin/` implementation snapshots and quarantine copies
- `.hermes/backups/*` code backups inside this repo
- backup-only doc pointing to deleted `backup-bin` content

Do not resurrect those backup implementation files as runtime inputs. Use git history for archaeology.

## Preserved intentionally

The OEM decompiled/source evidence and original SSD-derived config are preserved as reference material, not active Python runtime code. Examples:

- original SSD AppData-derived config under `config/oem/original_ssd_appdata_20260610`
- OEM compatibility/planning modules under `src/bioxp/oem_compat/`
- decompiled OEM source outside this repo used as evidence

## Active movement-test baseline

The current reference includes:

- OEM machine config + PositionTable read-only binding
- OEM dry-run path planning comparison routes
- movement readiness comparison route
- strict no-homing readiness gate result documentation
- gripper idle-current safety invariant
- controller-only motion truth classification

Policy: controller counters are not physical proof. The next iteration starts from supervised relative proof moves only after live no-motion gates pass.
