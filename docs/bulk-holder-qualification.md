# Backend candidate qualification

Base: `064f8d3d75d9cab2f7e467057a65b301e170afb1` (installed native candidate).
Branch/worktree: `fix/bulk-holder-xy`, `/home/dalab/robot/wt-bulk-holder-xy` on BigBoy.

Executed offline on BigBoy:
```
TMPDIR=/root/workstation-migration/bulk-holder-integration PYTHONPATH=src \
/root/workstation-migration/bulk-holder-integration/venv/bin/python -m pytest -q \
 tests/test_bulk_holder_xy.py tests/test_debloat_core.py tests/test_movement_modes.py \
 --basetemp=/root/workstation-migration/bulk-holder-integration/pytest-tmp \
 --junitxml=/root/workstation-migration/bulk-holder-integration/combined-junit.xml
```
Result: **52 passed, 1 warning in 137.35s**. Warning: existing Starlette TestClient/httpx deprecation. JUnit readback: tests=52, failures=0, errors=0, skipped=0.

New focused tests exercise actual calibration PATCH/GET functions through local FastAPI TestClient, canonical SQLite save and cold reopen, exact hash-verified captured Serial-206 snapshot, unchanged OEM rows and legacy station-16 inc_factor/heights, no Z fields or alignment-Z persistence, strict rejection of holder Z inputs, all four row anchors and sixteen PCR well coordinates, channel-set metadata, actual named compiler/executor/provider/native adapter over existing inert wire fixture (all 20 named extensions), existing movement-mode route/worker with row/well target sequence and single ordinary travel-height raise, and manual workflow compiler acceptance of the same named target. Existing core movement and workflow-sequence regression tests also pass.

Test setup initially lacked pytest in system Python; created an isolated qualification venv under this evidence directory. New tests reuse existing untracked retained test support copied from wt-fast-root-fifo; only the new test is added to the candidate commit. Initial test expectation incorrectly demanded a Y frame when OEM correctly omitted a zero-distance Y move; corrected to verify terminal emulated axis positions. No production behavior was changed for that test adjustment.

No live API POST/PATCH, hardware collection, reconnect, homing, motion, deploy or service restart occurred. No mobile/APK/frontend files changed. The separate holder definition contains XY and confirmed PCR pitch only. No lock/XML evidence or OEM station values were changed. Micro rows have anchor-level addressing only; individual micro-tube pitch and all pipetting depths remain unmeasured and are not inferred.

Parent must coordinate any deployment and later save through the existing calibration API. The payload in native-contract.md has not been applied to the live robot.
