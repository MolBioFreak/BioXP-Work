# BioXP 3200 Linux Driver

Reverse-engineering and control of a BioXP 3200 on Linux via the Atmel Novo USB-to-CAN adapter.

## Canonical Runtime
The active control runtime is:

- `src/bioxp/usb_driver.py`

Documentation in this repository now treats that script as source of truth for current behavior.

## Current State
Near full control has been achieved in the interactive USB driver:

1. Latch control
2. LED control (main strip path)
3. Camera system
4. Motor control
5. Chiller system
6. Thermal cycler

Primary remaining work is comm robustness and recovery-path parity, mainly in the motor path.

## Run
From project root:

```bash
sudo .venv/bin/python src/bioxp/usb_driver.py
```

Legacy standalone diagnostic:

```bash
sudo .venv/bin/python src/bioxp/diagnostic_24v.py
```

API wrapper for external systems such as BMS:

```bash
PYTHONPATH=src .venv/bin/uvicorn bioxp.api:app --host 0.0.0.0 --port 8000
```

## Documentation
Central documentation hub:

- `docs/README.md`

Primary docs:

- `docs/ARCHITECTURE_AND_CONTROL_PLANE.md`
- `docs/SUBSYSTEMS_AND_OPERATIONS.md`
- `docs/MODULE_REFERENCE.md`
- `docs/REVERSE_ENGINEERING_TRACEABILITY.md`
- `docs/RUNBOOK.md`

Historical context docs are retained in `docs/` and referenced from `docs/README.md`.
