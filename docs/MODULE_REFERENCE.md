# Module Reference

This reference describes modules under `src/bioxp/` and their operational status.

## 1. `src/bioxp/usb_driver.py`
Status: Active primary runtime.

Purpose:
- Direct USB/TMCL control plane
- Full interactive menu for latch, LED, camera, motor, chiller, thermal
- Integrated recovery and diagnostic tooling

Key Class:
- `BioXpTester`

Key Entrypoints:
- script `__main__` menu loop
- `run_latch_menu`
- `run_led_menu`
- `run_camera_menu` (+ advanced)
- `run_motor_menu`
- `run_chiller_menu`
- `run_thermal_cycler_menu`

Outputs/Artifacts:
- LED persisted state: `output/led_state.json`
- camera captures: `output/camera/`

Primary dependencies:
- `usb.core`, `usb.util` (PyUSB)
- stdlib (`ctypes`, `fcntl`, `subprocess`, `signal`, `json`, etc.)

## 2. `src/bioxp/diagnostic_24v.py`
Status: Legacy targeted diagnostic utility.

Purpose:
- Earlier focused 24V/motor-power probing and broad IO enable attempts
- Lower-level experiment script outside current structured menu runtime

When to use:
- only for exploratory diagnostics not already covered by `usb_driver.py`

## 3. `src/bioxp/can_driver.py`
Status: Legacy/prototype SocketCAN driver.

Purpose:
- Early direct CAN abstraction for thermal/motor/pipette message packing
- Useful conceptual reference for payload packing and board enums

Current limitation:
- not the active production control path in this repo

## 4. `src/bioxp/api.py`
Status: Active API wrapper around the canonical USB runtime.

Purpose:
- FastAPI surface around `BioXpTester`
- BMS-safe HTTP entrypoint for motion, thermal, chiller, camera, and lock-clearance actions

Current limitation:
- liquid handling routes are not exported yet and return `501`
- runtime availability still depends on direct USB access and hardware presence

## 5. `src/bioxp/__init__.py`
Status: package export surface.

Purpose:
- exports `BioXpTester` as the primary runtime class
- preserves legacy `can_driver` symbols as optional compatibility imports

## 6. Scripts Directory (`scripts/`)
Purpose:
- OEM XML workflow artifacts and historical test scripts
- useful for command-sequence context and reverse-engineering validation

Examples:
- `demo.xml`
- `lifetest.xml`
- `TP015 48 HOUR SYSTEM BURN-IN.xml`

## 7. Documentation and Data Dependencies
Primary docs are in `docs/` (see `docs/README.md`).

Historical reverse-engineering inputs include:
- decompiled string extracts in docs
- backup/decompiled trees under `BioXP_Original_Backup/`

## 8. Recommended Runtime Selection
For active hardware control and operations:

- use `src/bioxp/usb_driver.py`
- use `src/bioxp/api.py` when integrating external orchestration systems such as BMS

For reference only:

- `can_driver.py`, `diagnostic_24v.py`
