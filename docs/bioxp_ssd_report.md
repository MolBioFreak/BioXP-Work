# BioXP 3200 SSD Analysis Report

I have thoroughly scoured the 120GB Kingston SSD (`/dev/sda`, mounted at `/media/dalab/10CC3A60CC3A3FF0`) from your BioXP 3200 system. Here is a summary of the proprietary control software and the best pathways for repurposing the instrument as a personal liquid handler.

## Software Architecture Discovered

The primary control software for the BioXP 3200 is a .NET Framework 4.5.2 Windows Presentation Foundation (WPF) application named **GenBotApp**. 

Instead of being installed in the standard `Program Files` directories, the software is deployed via Microsoft ClickOnce. The active deployment was found at the root of the drive in:
`/media/dalab/10CC3A60CC3A3FF0/.deploy/Application Files/GenBotApp_6_3_0_1/`

### Key Proprietary Control Libraries
Inside the deployment folder, I identified several critical .NET Dynamic Link Libraries (DLLs) that act as the interface between the CAN bus, the liquid robotics, and the computer components:

1. **`NovoCANUSBLib.dll` & `ClassCanLib.dll`**: These libraries handle the direct, low-level CAN bus communication.
2. **`BioXPControlLib.dll` & `BioXPCommonLib.dll`**: These contain the high-level business logic, state machines, and API for controlling the Liquid Robotics, thermal cyclers, and magnetic stations.
3. **`LabJack.LJM.dll`**: Indicates the system uses LabJack data acquisition hardware for specific analog/digital I/O (e.g., sensors or heaters).
4. **Computer Vision Stack (`CVisionLib.dll`)**: A highly sophisticated OpenCV (C++) based vision system, alongside `Gma.QrCodeNet.Encoding.dll`. It handles reagent barcode traceability, blob detection for volumetric meniscus verification (`FindValidWells`), and closed-loop positional calibration to compensate for gantry slop.

### Protocol Scripts and Maintenance Tools
In the `/Scripts/` directory at the root of the SSD, I discovered XML-based job and protocol definitions:
- `TP015 48 HOUR SYSTEM BURN-IN.xml`
- `lifetest.xml`
- `demo.xml`

Additionally, `BioXpUpdateMonitor.ps1` is a PowerShell script that runs in the background, listening for USB drives labeled `BIOXPUPDATE` to automatically flash new firmware/software.

## Pathways for Reuse and Repurposing

Since your goal is to use the BioXP 3200 as a custom, personal liquid handling instrument, you have an excellent, straightforward path forward because the control stack is written in **managed .NET (C#)**.

### 1. The "Native Wrapper" Approach (Easiest)
Because `BioXPControlLib.dll` and `NovoCANUSBLib.dll` are managed assemblies, you can simply create a new C# console application or Python script (using `pythonnet`) and add these DLLs as references. 
- **Action**: You can decompile these DLLs using free tools like **ILSpy** or **JetBrains dotPeek** to view the exact class names and methods.
- **Benefit**: You don't need to reverse-engineer the raw CAN frames. You can just call the high-level `Dispense()`, `Aspirate()`, or `MoveTo()` methods already defined in `BioXPControlLib.dll`.

### 2. The "Raw CAN Bus" Approach (Most Flexible)
If you want to strip the Windows 10 machine entirely and control the CAN bus via a Raspberry Pi or Linux machine (e.g., with python-can):
- **Action**: Decompile `ClassCanLib.dll` to study the message framing. 
- **Benefit**: This allows you to write a clean, cross-platform driver.

### 3. XML Protocol Reverse Engineering
You can analyze `demo.xml` and `lifetest.xml` to understand the macro-level G-code/command structure the BioXP expects. If the `GenBotApp` supports a headless or CLI mode, you might simply be able to feed it custom XML files to execute liquid handling protocols without writing any new drivers.

> [!TIP]
> **Next Steps:** I highly recommend copying the `.deploy/Application Files/GenBotApp_6_3_0_1/` directory and the `Scripts/` directory to your main workstation. You can then use a .NET decompiler to uncover the API surface area of `BioXPControlLib.dll`. If you'd like, I can write a small script to copy these files over for you.
