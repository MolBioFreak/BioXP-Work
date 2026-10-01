# Retained tooling and manual recovery interfaces

The historical supervised shell workflow, refusal-only generic container runners,
one-off Git bootstrap, RGB cycling experiment, synthetic latency microbenchmark,
and generated runtime audit denominator are retired (A7). This does not change
controller behavior or add a new execution requirement.

## Supported interfaces remain

- The robot-owned operator catalog selects current X/Y/Z operations. Historical
  generic home/relative wrappers are not substitutes for those actions.
- `POST /motion/axis/home` and `POST /motion/axis/relative` retain their supported
  manual gripper (`g`) and thermal-door (`door`) branches, request validation and
  controller semantics. These are explicit live operations, not passive probes.
- `POST /motion/arm/strict_startup` with `{"run_homing":false}` remains the
  no-homing maintenance recovery operation. Removal of the shell wrapper does
  not remove this route or turn it into automatic startup/homing.
- `GET /status`, `GET /motion/power/status`, `GET /latch/status`, and
  `GET /motion/axes/status` retain observation capability. No observation here
  authorizes a move or creates a new missing-proof refusal.
- RGB control remains in the API/USB driver; only the standalone cycling
  experiment is retired. Camera illumination is a separate capability.
- `scripts/bioxp_emergency_motor_kill.sh` remains the exclusive-owner emergency
  tool. Normal operator addressed Stop remains in the command plane.
- `scripts/bioxp_latency_probe.py` retains GET-only HTTP timing. It is not an
  equivalent replacement for the retired synthetic state microbenchmark.
- `release/README.md` and the immutable-release tools remain the release path;
  the canonical launcher is `scripts/bioxp_release_container_run.sh`.

No live command was executed to qualify this cleanup. Existing OEM/controller
interlocks, explicit request semantics and truthful partial outcomes are unchanged.

## Canonical retained inputs

`testdata/oem_xml/demo.xml` and `testdata/oem_xml/lifetest.xml` retain the exact
bytes previously duplicated in `scripts/`. The other named OEM XML inputs and
live movement-registry JSON remain untouched.

The duplicate Windows updater remains at:
`/home/dalab/Desktop/ROBOT/BioXP 3200 Development Work/BioXP_SSD_Backup/Scripts/BioXpUpdateMonitor.ps1`
(SHA-256 `d77617e3527062077acba667231c954ed93a708719ddaf8f5a07abd5df7120ad`).

The redundant ControlLib string dump is reproducible with system `strings` on:
`/home/dalab/Desktop/ROBOT/BioXP 3200 Development Work/BioXP_SSD_Backup/BioXPControlLib.dll`
(DLL SHA-256 `163db8f7835cecbc87da4d14734a8224d79ea1e2ccc77bbb299998fa31bf14ed`).
The output SHA-256 is
`3509a7b04b0aafb2a5c706d845aec9fbfff857b3e437fe796cfeb94932842187`.
The CVision string dump is retained because matching retained source provenance
has not been established. No SSD access or original-source deletion was performed.
