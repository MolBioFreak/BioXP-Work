# OEM Parity Fresh Attempt Phase 0 Backup — 2026-06-09

Phase 0 completed before fresh OEM parity scaffold work.

## Backup artifact

```text
/home/molbiofreak/bioxp_re/backup-bin/oem_parity_fresh_attempt_20260609T181148Z
```

## Scope

- No robot motion performed.
- No USB/CAN/camera commands issued by the backup capture.
- Existing Linux/OEM-ish homing paths remain quarantined as `legacy_partial_guarded_reconstruction`.
- Backup includes git status, HEAD, tracked diff, untracked file list, OpenAPI snapshot, and copies of `src/bioxp/api.py`, `src/bioxp/usb_driver.py`, and `src/bioxp/oem_homing_model.py` when present.

## Next permitted work

Proceed only with no-USB/no-motion Tranche A:

1. source oracle extraction;
2. fresh no-USB spec modules;
3. dry-run artifact runtime;
4. robot-local dry-run route listing.

Stop before user/live testing.
