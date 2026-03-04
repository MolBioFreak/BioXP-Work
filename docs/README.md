# BioXP 3200 Documentation Hub

This directory is the centralized, current documentation for the Linux control stack.

## Source of Truth
The canonical runtime source is:

- `src/bioxp/usb_driver.py`

All operational behavior described here is derived from that script.

## Documentation Map
1. `ARCHITECTURE_AND_CONTROL_PLANE.md`
   - End-to-end system architecture
   - TMCL transport model
   - board topology, command surface, reliability wrappers
2. `SUBSYSTEMS_AND_OPERATIONS.md`
   - Latch, LED, Camera, Motor, Chiller, Thermal control surfaces
   - What each subsystem does, how it is controlled, and real use cases
3. `MODULE_REFERENCE.md`
   - Purpose and status of each Python module in `src/bioxp/`
   - Dependencies and output artifacts
4. `REVERSE_ENGINEERING_TRACEABILITY.md`
   - Mapping from Windows/decompiled origins to Linux implementations
   - Rationale for retained behavior and hardening choices
5. `RUNBOOK.md`
   - Practical operator workflow
   - startup checks, common procedures, and recovery paths

## Current vs Legacy Docs
The files below are retained as historical analysis and progress context:

- `reverse_engineering_report.md`
- `bioxp_ssd_report.md`
- `RCA_2025-03-01.md`
- `FIX_PLAN_2025-03-01.md`
- `PROGRESS_2026-03-02.md`
- `*_strings.txt` extraction files

They are useful reference artifacts, but behavior and menu truth should be taken from `usb_driver.py` and this documentation hub.
