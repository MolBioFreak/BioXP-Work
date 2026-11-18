# Saved-method launch tip handling

The BMS saved-run route (`routers/bioxp/methods.py`, `saved_run` → `_submit`) attaches `metadata.bms_method_run` with schema `bms.bioxp-method-run.v1` and `saved.id`/`revision`. Compiler-only `bms_method` metadata is not a launch. Quick runs have no saved revision. A recovery snapshot does not select the prefix. Dry runs, review continuation and idempotent replay do not enter it.

On a selected launch the native worker enters only the tip prefix before the first authored action, not the prepared-script lifecycle. Its short-lived child claim uses the same canonical SQLite writer as direct claims. It is acquired after workflow dispatch, released on unwind or return, and does not extend the parent resource footprint. Historical terminal claims are not admission conditions. Stop and its existing fences remain independent.

`ControlLib.cs:5094–5112` queries the collection, waits 500 ms and, if tipped, moves to waste (6, offsets 0/0, nonparallel), ejects, queries again and waits 100 ms. A still-loaded final query skips method entry with the manual-removal result. `ClassPipetteCollection.cs:1176–1235,1265–1323` initially selects by TipLocation and observations, waits up to 8 s, then verifies all channels. If any remain, verification retries all four with a shared 6 s wait; persistent tips throw before the caller’s final query. No Home is part of this prefix.

Offline acceptance: `tests/test_new_method_launch_tips.py` (HTTP, worker, native plan executor, both SQLite owners and inert controller leaves; plus exact helper wire and delay assertions) and `tests/test_workflow_launch_pipette_exclusion.py` (both resource-claim orders, nested lineage and exceptional release). These are not physical qualification.
