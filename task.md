# Exploring BioXP 3200 SSD

- [x] Locate the BioXP Windows 10 SSD and identify its filesystem structure.
- [x] Search `Program Files`, `Program Files (x86)`, `ProgramData` and the root for proprietary control software related to BioXP and liquid robotics.
- [x] Analyze found software directories, specifically looking for CAN bus communication executables, configs, or scripts.
- [x] Report the findings to the user with a summary of the controller software and pathways for reuse.

## Phase 2: Reverse Engineering Deep Dive
- [x] Copy BioXP control files (DLLs and Scripts) from the SSD to a local scratch workspace (`/home/dalab/.gemini/antigravity/scratch/bioxp_re`).
- [x] Analyze the XML protocol files (`demo.xml`, `lifetest.xml`, etc.) to understand macro command syntax and robotic parameters.
- [x] Inspect the .NET DLLs (`BioXPControlLib.dll`, `NovoCANUSBLib.dll`, `ClassCanLib.dll`) to map the API surface and CAN bus interaction layers.
- [x] Generate a comprehensive senior engineer-level reverse engineering report detailing the control system architecture and a concrete Linux migration path.

## Phase 3: Q&A and Architecture Planning
- [x] Move the local workspace `/home/dalab/.gemini/antigravity/scratch/bioxp_re` to the user's Desktop.
- [x] Answer User Q1 (Hardware Reporting & Remote Hub Feasibility) and update the report.
- [x] Answer User Q3 (Physical Mechanism Replication for CAN bus) and update the report.

## Phase 4: Generalizing the Bioreactor Vision
- [x] Outline the software architecture required to turn the BioXP 3200 into a generalized workstation (Full Gibson, RE Digest, Cell-Free DNA Synthesis).

## Phase 5: Analyze Computer Vision and Barcode Subsystem
- [x] Extract strings from `CVisionLib.dll` and map its computer vision capabilities (Barcode reading, volume checking, alignment).
- [x] Document findings in the reverse engineering report.

## Phase 6: Evaluate Native Windows 10 Iteration (The 'Flip Side')
- [x] Analyze the friction and benefits of keeping the Windows 10 OS and wrapping the native `.NET 4.5.2` DLLs.
