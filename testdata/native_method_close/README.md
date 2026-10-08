# Native method closeout — qualified scope and remaining decision

Implementation: `ae006a9a854c2959ab313aac994b396217ac12f6` (parent `e58d3e5`).

## Delivered

- Native `cavro_liquid_recipe` now validates and compiles with `compile_liquid_recipe`, then runs through exactly one existing finite `cavro_application` / `sourceCavroApplication` child. Real dispatcher, command store, receipt store, source application, CAN/group transport and ASGI readback are exercised. No second physical owner or registration-only shortcut.
- F02: missing adapter outcome (`{}` / `ok:null`) is `unknown`, not a failed event followed by success. Source-return completion remains distinct; `has_unknown_outcomes` survives, without a new gate. Explicit failure still fails. Real source board-null/no-command success preserves false ACK.
- F01 source repair: restored source false-wait pump termination, preserved post-Z partial effects and primary detector exceptions, prevented fabricated final Z position/settlement, and retained Stop/ownership fences. Regression covers timeout, channel error, detector exception, explicit Stop, owner loss and secondary termination exception against actual native provider/group shape.
- Ordinary Park runs existing finite `parkGantry`, optional authored `rehome` defaults false, including source tip query/ejection. RGB status strip uses native `strip_set_rgb`, distinct from camera illumination. SS is explicitly source no-op, not a fabricated actuator or operator-completion claim. Barcode empty, successful and BLACK branches use the real decoder/source owners. A source-authored Z current change now updates the existing verifier's expected parameter without an extra hardware write.
- Published generated native schema/contract: `native-close-contract.json`, `native-close-manual-schema.json`, `native-close-recipe-schema.json`; generator `bioxp.protocols.method_contract.method_contract()`.

## Verified evidence

`testdata/native_method_close/manifest.json` pins source/doc/artifact hashes and exact roster. **20 modules, 337 tests, zero failures/errors/skips** in the final selected per-module results. This includes the prior 14-file finish roster and new regression/recipe/BMS tests. Tests run in isolated native-image containers with physical leaves replaced; not hardware qualification.

All **69** unchanged BMS documents parse. **49 unique documents** executed through native runtime and exported (including eight full bound scientific examples); remaining **20** whole-document executions are listed explicitly in the manifest. Underlying action owners also have native connected tests; do not mistake parser coverage for all-document execution coverage.

Compressed actual full readbacks, preserving embedded BMS document/metadata, are under `bms/`. In particular:

- `bms-46-success.json.gz`: actual recipe success.
- `bms-46-aspirate.json.gz`, `bms-46-dispense.json.gz`: actual mid-channel partial failures.
- `bms-0-error-hold.json.gz`: authored error_hold, ordinary Continue rejected, explicit Abort settlement, held snapshot retained.
- `bms-11-success.json.gz` through `bms-18-success.json.gz`: full bound examples and review/progress snapshots.
- `native/plld-*.json.gz`: actual post-Z fault reproductions, not fabricated receiver data.

Original BMS snapshot SHA256: `95dca7bccc05596d7e7a6b1c0ce2a7308839c247255ad55a1acd45ad1b0683c7`.

## Not closed / limits

**Do not mark automatic-Z-Stop F01 closed.** Recovered ControlLib 7887–7902 stops Z only on successful detection; false wait terminates pumps then throws. Neither this method nor its applicable outer catch establishes an unconditional finally Z Stop. See `native-close-source-decision.md`. Adding such Stop (especially after owner loss) is a behavior change requiring Christian's exact approval; no automatic lift/retract/home/Stop was invented. The source-aligned lifecycle/evidence defect is repaired, but unconditional automatic settlement remains an explicit policy decision.

No classifier/threshold was invented. No physical seal-separation mechanism was inferred from SS. No live hardware, deployment or push. BMS receiving-side report/progress/browser proof remains the receiving owner's responsibility using these actual exports.

The initial single-process 155-test combined run exhausted file descriptors in the retained-store fixture workload and had two missing relative-Z fixture leaves. Its failure JUnit is retained under `prior/`; it is not counted as passing. Final qualification uses fresh per-module processes, extends only the physical relative-Z fixture, and corrects the pre-existing fake-LED custody test interception after LED gained a real binding. The corrected workflow rerun is explicitly selected in the manifest; the superseded failed result is also retained. Do not claim a green monolithic suite.
