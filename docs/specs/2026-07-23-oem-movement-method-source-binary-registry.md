# BioXP3200 serial-206 OEM movement method/source/binary registry

**Date:** 2026-07-23
**Schema:** `bioxp.oem_movement_method_source_binary_registry.v1`
**Authority:** recovered OEM source/binaries and the canonical evidence lock only
**Physical action:** none; this is a static evidence registry

## 1. Purpose and claim boundary

Complete OEM startup movement envelope: application admission and worker; no-motion motor setup; initializeMotors; initializeMotion stale-tip remediation; optional self-test movement; camera/cover movement; park; door/interlocks; terminal startup branches. General post-admission job execution is excluded.

**Claim boundary:** Static source/binary/configuration registry only; no deployment, transport, controller, or physical parity claim.

This registry closes source-discovery for the startup movement envelope. `present_hash_locked_and_line_bounded` means the source bytes, captured binary, and inclusive source lines are pinned; it does **not** mean the Linux implementation or physical machine behavior has been accepted.

The canonical corpus must be searched before any absence statement. `unreviewed`, `not yet line-locked`, `not implemented`, and `not physically validated` are not synonyms for `absent`.

## 2. Controlling evidence

- Evidence lock: `/home/dalab/Desktop/ROBOT/BioXP 3200 Development Work/reports/bioxp_historical_run_forensics_20260717/derived/oem_runtime_parity_spec_20260719/OEM_EVIDENCE_LOCK.json`
- Lock schema: `bioxp.oem_evidence_lock.v4`
- Acquisition: `20260719T024740Z-4a7fe6783205846c`
- Frozen SSD manifest SHA-256: `ce5104c91f5149374910b0f4fca2aef1c49f42270d1c35ccd788da3ec63e818a`
- Machine: BioXP3200 serial `206`
- Registry JSON: `docs/specs/2026-07-23-oem-movement-method-source-binary-registry.json`

## 3. Captured OEM binaries

| Binary | SHA-256 | Bytes | Exact path |
|---|---|---:|---|
| `BioXPCommonLib.dll` | `c0e430426a5af34c4fcadf067d2598aa98392569fd0175661d6478f18833382e` | 316928 | `/home/dalab/Desktop/ROBOT/BioXP 3200 Development Work/reports/bioxp_historical_run_forensics_20260717/raw/live_oem_windows_phase0_extension/20260719T024740Z-4a7fe6783205846c/source_roots/profile_3_clickonce_genbotapp_1/C/Users/BioXp PC/AppData/Local/Apps/2.0/95WM87ZT.NKJ/M077NK28.8QY/genb..tion_0000000000000000_0006.0003_153bce31295c21ca/BioXPCommonLib.dll` |
| `BioXPControlLib.dll` | `163db8f7835cecbc87da4d14734a8224d79ea1e2ccc77bbb299998fa31bf14ed` | 914944 | `/home/dalab/Desktop/ROBOT/BioXP 3200 Development Work/reports/bioxp_historical_run_forensics_20260717/raw/live_oem_windows_phase0_extension/20260719T024740Z-4a7fe6783205846c/source_roots/profile_3_clickonce_genbotapp_1/C/Users/BioXp PC/AppData/Local/Apps/2.0/95WM87ZT.NKJ/M077NK28.8QY/genb..tion_0000000000000000_0006.0003_153bce31295c21ca/BioXPControlLib.dll` |
| `CVisionLib.dll` | `84ab0c851f1bb418289035efd9fa84420e9ff82ea0a69ec0c00bb5d401e750f2` | 541184 | `/home/dalab/Desktop/ROBOT/BioXP 3200 Development Work/reports/bioxp_historical_run_forensics_20260717/raw/live_oem_windows_phase0_extension/20260719T024740Z-4a7fe6783205846c/source_roots/profile_3_clickonce_genbotapp_1/C/Users/BioXp PC/AppData/Local/Apps/2.0/95WM87ZT.NKJ/M077NK28.8QY/genb..tion_0000000000000000_0006.0003_153bce31295c21ca/CVisionLib.dll` |
| `ClassCanLib.dll` | `7d91d8279fa967c20d006679c1c2d6545451b898fdeca9028224ef6d0c46dac1` | 116224 | `/home/dalab/Desktop/ROBOT/BioXP 3200 Development Work/reports/bioxp_historical_run_forensics_20260717/raw/live_oem_windows_phase0_extension/20260719T024740Z-4a7fe6783205846c/source_roots/profile_3_clickonce_genbotapp_1/C/Users/BioXp PC/AppData/Local/Apps/2.0/95WM87ZT.NKJ/M077NK28.8QY/genb..tion_0000000000000000_0006.0003_153bce31295c21ca/ClassCanLib.dll` |
| `CommonLib.dll` | `100eee43654be4db1ce666e5d1e283c0ef16ccae478ba106db4edf6be5b449c6` | 64512 | `/home/dalab/Desktop/ROBOT/BioXP 3200 Development Work/reports/bioxp_historical_run_forensics_20260717/raw/live_oem_windows_phase0_extension/20260719T024740Z-4a7fe6783205846c/source_roots/profile_3_clickonce_genbotapp_1/C/Users/BioXp PC/AppData/Local/Apps/2.0/95WM87ZT.NKJ/M077NK28.8QY/genb..tion_0000000000000000_0006.0003_153bce31295c21ca/CommonLib.dll` |
| `GenBotApp.exe` | `d7cb7c94ebcfc386bc0a4acebb92f13d8463bb2b675f5439adf43be35198c6d9` | 1907200 | `/home/dalab/Desktop/ROBOT/BioXP 3200 Development Work/reports/bioxp_historical_run_forensics_20260717/raw/live_oem_windows_phase0_extension/20260719T024740Z-4a7fe6783205846c/source_roots/profile_3_clickonce_genbotapp_1/C/Users/BioXp PC/AppData/Local/Apps/2.0/95WM87ZT.NKJ/M077NK28.8QY/genb..tion_0000000000000000_0006.0003_153bce31295c21ca/GenBotApp.exe` |
| `Novo.Devices.dll` | `971efffd20c753de10f0750193cefb437ac64aac6ae6b6c22788d5bc5f2e1729` | 148992 | `/home/dalab/Desktop/ROBOT/BioXP 3200 Development Work/reports/bioxp_historical_run_forensics_20260717/raw/live_oem_windows_phase0_extension/20260719T024740Z-4a7fe6783205846c/source_roots/profile_3_clickonce_genbotapp_1/C/Users/BioXp PC/AppData/Local/Apps/2.0/95WM87ZT.NKJ/M077NK28.8QY/genb..tion_0000000000000000_0006.0003_153bce31295c21ca/Novo.Devices.dll` |
| `NovoCANUSBLib.dll` | `6856ba3bda0033b938483ea7deeb664478cf446dabc93d1048fbb843d3cda3c3` | 28160 | `/home/dalab/Desktop/ROBOT/BioXP 3200 Development Work/reports/bioxp_historical_run_forensics_20260717/raw/live_oem_windows_phase0_extension/20260719T024740Z-4a7fe6783205846c/source_roots/profile_3_clickonce_genbotapp_1/C/Users/BioXp PC/AppData/Local/Apps/2.0/95WM87ZT.NKJ/M077NK28.8QY/genb..tion_0000000000000000_0006.0003_153bce31295c21ca/NovoCANUSBLib.dll` |

## 4. Recovered source-to-binary map

| Source ID | Recovered source | SHA-256 | Lines | Captured binary | Map status |
|---|---|---|---:|---|---|
| `base` | `decompiled_src_can/ClassCanLib/ClassBaseBoard.cs` | `2622aee1810b9a8a52bed54a49196d50e50751465608c8420dd1e5077d95dd5e` | 283 | `ClassCanLib.dll` | `registry_extension_from_captured_assembly` |
| `cci` | `decompiled_src/BioXPControlLib/ClassControlInterface.cs` | `86093e5270c82ea2e45cb4de449076372ca79d9485ba6de9565d5eb255811e6e` | 5558 | `BioXPControlLib.dll` | `canonical_evidence_lock` |
| `chiller` | `decompiled_src_can/ClassCanLib/ClassChillerBoard.cs` | `c048e2cdfcecc58c97f857e2c2bad85eb9766c2196981fa8fde67df7334981fd` | 595 | `ClassCanLib.dll` | `registry_extension_from_captured_assembly` |
| `control` | `decompiled_src/BioXPControlLib/ControlLib.cs` | `f69b3529dcb9723c705ac55ecb3f035010cc294d3891de096c165bb20116f6c2` | 11220 | `BioXPControlLib.dll` | `canonical_evidence_lock` |
| `deck` | `decompiled_src_can/ClassCanLib/ClassDeckBoard.cs` | `bcae5ce4662cb3208dbc2fdf6e4c7ad0e23774307b633fc7a9ce5b727318698f` | 604 | `ClassCanLib.dll` | `canonical_evidence_lock` |
| `defaults` | `decompiled_src_bioxpcommon/BioXPCommonLib/DefaultParameters.cs` | `04f53c129317f8ae508d6971d2cf9fb534e1bc8307b192f8db395cbbcdf64fd2` | 85 | `BioXPCommonLib.dll` | `registry_extension_from_captured_assembly` |
| `head` | `decompiled_src_can/ClassCanLib/ClassHeadBoard.cs` | `342a9b2f09731002194b67e37f1d4e866ecbfb3c25effd85b3cd609e8cbdd1ea` | 560 | `ClassCanLib.dll` | `registry_extension_from_captured_assembly` |
| `ican` | `decompiled_src_can/ClassCanLib/InterfaceCAN.cs` | `aed90411d2966ae45142f8a988a2c6757011845c3ab2c5823c98090783419da1` | 32 | `ClassCanLib.dll` | `registry_extension_from_captured_assembly` |
| `io` | `decompiled_src_can/ClassCanLib/ClassIOControl.cs` | `cb684f0afc788503601ce4b71babacaf83bb57bdccd3cbfba20c9b49c1fa2b1f` | 205 | `ClassCanLib.dll` | `canonical_evidence_lock` |
| `main` | `decompiled_src_genbotapp/GenBotApp/BioXPMainWindow.cs` | `b288a45e2de54cd2c8d30a4498a343cd6f423aff7e88a78847076bfbfb4e904c` | 4355 | `GenBotApp.exe` | `canonical_evidence_lock` |
| `motor` | `decompiled_src_can/ClassCanLib/ClassMotor.cs` | `9fb1b4bec771165053a82b4fe95510615d6ed9beda1a041280584ceb4ab7fe99` | 799 | `ClassCanLib.dll` | `registry_extension_from_captured_assembly` |
| `novo` | `decompiled_src_can/ClassCanLib/ClassNovo.cs` | `11293074caec278076723666e69022b547c43f32b5fa886c99f75d5b60043d06` | 232 | `ClassCanLib.dll` | `canonical_evidence_lock` |
| `novousb` | `decompiled_src_novo/NovoCANUSBLib/ClassNovoCANUSB.cs` | `e4cf1c311ed5ae79e9490564a48947bced5f46af36890cf4a77120a3b50ffb06` | 1157 | `NovoCANUSBLib.dll` | `canonical_evidence_lock` |
| `pipette` | `decompiled_src_can/BioXPControlLib/ClassPipette.cs` | `681f959cf527b060cece17b3cf7ff59c1ba1f5ead99fea53520d09486ac0c957` | 821 | `BioXPControlLib.dll` | `canonical_evidence_lock` |
| `pipettes` | `decompiled_src/BioXPControlLib/ClassPipetteCollection.cs` | `ffe3729fa35642d04ef6fe45501e52200dd7c2977a70902aa20e46fb26d4011e` | 1481 | `BioXPControlLib.dll` | `canonical_evidence_lock` |
| `settings` | `decompiled_src_bioxpcommon/BioXPCommonLib/ClassBioXPSettings.cs` | `08155dc24602bc12cf25af745c74cc478f33e0f2675fc0d4b6f2e1ba917d8d41` | 7076 | `BioXPCommonLib.dll` | `canonical_evidence_lock` |
| `thermal` | `decompiled_src_can/ClassCanLib/ClassThermalBoard.cs` | `23d50725da200044422fde56b00611df708b514cef0f3637b2ad8d19e0b23f26` | 1006 | `ClassCanLib.dll` | `registry_extension_from_captured_assembly` |
| `thermalctl` | `decompiled_src_can/ClassCanLib/ClassThermalControl.cs` | `fdefde9b38c70b9fec47e6ffdd4929ad8f25bc90c57c43baf03722addbe6cf25` | 1257 | `ClassCanLib.dll` | `registry_extension_from_captured_assembly` |
| `vision` | `decompiled_src_vision/CVisionLib/ClassFrameGrabber.cs` | `6eec22f02eae5b4738a6d857b57691009dacd5b1b9bb615d41e9d8a8e4d28501` | 14587 | `CVisionLib.dll` | `registry_extension_from_captured_assembly` |

`registry_extension_from_captured_assembly` does not mean uncertain binary ownership. It means the source was recovered after the original evidence-lock `decompile_binary_map` was authored; this registry pins it to the already captured assembly. Extending the canonical lock with these records is a pre-implementation acceptance gate.

## 5. Selected serial-206 configuration provenance

- `config.xml`: `33aadf87f631cf33f2e0b4c86948c92be3b21412ca5477ea8fa8bc7848cbf475`
- `Operation_parameters.xml`: `d032d58c08312706892a2d5c7a9a319f817a6d9ef73e75d30dd933ae110c9685`
- `InspectionSettings.xml`: `d38220177e7e01b3d6d50892e0ffbbe27b1eb46087c4623cd6ca4757cc80b2d7`

| Field | Selected value | Artifact | Exact line(s) | Selector |
|---|---|---|---|---|
| `SerialNumber` | `206` | `config.xml` | `4` | `/BioXPCommonLib/GenBot/SerialNumber/@GenBot` |
| `ConfigVersion` | `3` | `config.xml` | `5` | `/BioXPCommonLib/GenBot/Config/@Version` |
| `GripperVersion` | `1` | `config.xml` | `5` | `/BioXPCommonLib/GenBot/Config/@GripperVersion` |
| `TroughVersion` | `1` | `config.xml` | `5` | `/BioXPCommonLib/GenBot/Config/@TroughVersion` |
| `Calibrated` | `1` | `config.xml` | `6` | `/BioXPCommonLib/GenBot/Calibration/@Calibrated` |
| `CameraInstalled` | `1` | `config.xml` | `7` | `/BioXPCommonLib/GenBot/CameraInstalled/@Camera` |
| `CameraCalibrated` | `true` | `config.xml` | `7` | `/BioXPCommonLib/GenBot/CameraInstalled/@Cameracalibrated` |
| `TCDoorOpen` | `18500` | `config.xml` | `15` | `/BioXPCommonLib/CalibrationFactors/Offsets/@m_TCDoorOpen` |
| `TCDoorStallGuardThreshold` | `6` | `config.xml` | `15` | `/BioXPCommonLib/CalibrationFactors/Offsets/@m_TCDoorStallGuardThreshold` |
| `TC_DOOR_VELOCITY` | `50` | `config.xml` | `15` | `/BioXPCommonLib/CalibrationFactors/Offsets/@m_TC_DOOR_VELOCITY` |
| `TC_DOOR_ACCELERATION` | `20` | `config.xml` | `15` | `/BioXPCommonLib/CalibrationFactors/Offsets/@m_TC_DOOR_ACCELERATION` |
| `TC_DOOR_MAX_CURRENT` | `31` | `config.xml` | `15` | `/BioXPCommonLib/CalibrationFactors/Offsets/@m_TC_DOOR_MAX_CURRENT` |
| `Z_MOTOR_MAX_CURRENT_DOWN` | `25` | `config.xml` | `15` | `/BioXPCommonLib/CalibrationFactors/Offsets/@m_Z_MOTOR_MAX_CURRENT_DOWN` |
| `Z_MOTOR_MAX_CURRENT_UP` | `31` | `config.xml` | `15` | `/BioXPCommonLib/CalibrationFactors/Offsets/@m_Z_MOTOR_MAX_CURRENT_UP` |
| `Z_MOTOR_STALL_GUARD_THRESHOLD` | `3` | `config.xml` | `15` | `/BioXPCommonLib/CalibrationFactors/Offsets/@m_Z_MOTOR_STALL_GUARD_THRESHOLD` |
| `ReagentChiller` | `{"FeedForward": 400, "MaxCoolingCurrent": -5, "MaxHeatingCurrent": 5, "Proportional": 1.5}` | `config.xml` | `17` | `/BioXPCommonLib/CalibrationFactors/ReagentChiller` |
| `OutputChiller` | `{"FeedForward": 400, "MaxCoolingCurrent": -5, "MaxHeatingCurrent": 5, "Proportional": 1.5}` | `config.xml` | `18` | `/BioXPCommonLib/CalibrationFactors/OutputChiller` |
| `X_limits` | `[0, 90263]` | `config.xml` | `25` | `/BioXPCommonLib/AxisLimits/X_limit` |
| `Y_limits` | `[0, 102956]` | `config.xml` | `26` | `/BioXPCommonLib/AxisLimits/Y_limit` |
| `Z_limits` | `[0, 160000]` | `config.xml` | `27` | `/BioXPCommonLib/AxisLimits/Z_limit` |
| `G_limits` | `[0, 15000]` | `config.xml` | `28` | `/BioXPCommonLib/AxisLimits/G_limit` |
| `PositionTable` | `"29 entries"` | `config.xml` | `[30, 60]` | `/BioXPCommonLib/PositionTable/*` |
| `CameraOffset` | `{"x": 3499, "y": -7744, "zDelta": 6842, "zLow": 3145}` | `config.xml` | `59` | `/BioXPCommonLib/PositionTable/CAMERA_OFFSET` |
| `Mode` | `"WebMode"` | `Operation_parameters.xml` | `4` | `/OperationParameters/Mode/@Mode` |
| `DeckInspection` | `true` | `Operation_parameters.xml` | `8` | `/OperationParameters/DeckInspection/@DeckInspection` |
| `InspectionLogOnly` | `false` | `Operation_parameters.xml` | `10` | `/OperationParameters/InspectionLogOnly/@InspectionLogOnly` |
| `SelfTest` | `true` | `Operation_parameters.xml` | `11` | `/OperationParameters/SelfTest/@SelfTest` |
| `CheckCamera` | `true` | `Operation_parameters.xml` | `17` | `/OperationParameters/CheckCamera/@CheckCamera` |
| `InspectionSettings3200.CoverInspection` | `"selected camera-control block"` | `InspectionSettings.xml` | `[59, 68]` | `CameraSettings/Settings3200/*[Key=CoverInspection]` |

The selected `PositionTable` is exactly lines 30–60 of `config.xml`: 29 entries, including all startup, cover, camera, press, waste, and park coordinates. It is data, not a future acquisition gap.

## 6. Exact method/member registry

Total line-bounded methods/members: **299**.

### 6.1 `decompiled_src_can/ClassCanLib/ClassBaseBoard.cs`

Source SHA-256: `2622aee1810b9a8a52bed54a49196d50e50751465608c8420dd1e5077d95dd5e`
Captured binary: `ClassCanLib.dll`

| Method/member ID | Inclusive lines | Declaration/member | Literal sleeps | Decompiler warning lines |
|---|---:|---|---|---|
| `ClassBaseBoard.ClassBaseBoard@45` | `45–51` | `public ClassBaseBoard(InterfaceCAN cancontrol, ushort canID)` | — | — |
| `ClassBaseBoard.deactivateBoard@53` | `53–71` | `public virtual void deactivateBoard()` | L56=1ms | — |
| `ClassBaseBoard.handleReturnMessageNovo@132` | `132–134` | `public virtual void handleReturnMessageNovo(byte[] msg)` | — | — |
| `ClassBaseBoard.activateBoard@136` | `136–138` | `public virtual void activateBoard()` | — | — |
| `ClassBaseBoard.forceAbortMotion@140` | `140–142` | `public virtual void forceAbortMotion()` | — | — |
| `ClassBaseBoard.setStallGuardThreshold@144` | `144–146` | `public virtual void setStallGuardThreshold(int axis, int threshold)` | — | — |
| `ClassBaseBoard.doorSearchHome@148` | `148–150` | `public virtual void doorSearchHome(int axis, int speed, int stallThread)` | — | — |
| `ClassBaseBoard.goHome@152` | `152–155` | `public virtual int goHome(bool rehome, int axis, int speed, bool waitforstop = true)` | — | — |
| `ClassBaseBoard.moveToAbs@157` | `157–160` | `public virtual int moveToAbs(int axis, int position, bool waitforstop = true, bool stallRecover = false, bool gripperRecover = false)` | — | — |
| `ClassBaseBoard.setSpeedAcc@162` | `162–164` | `public virtual void setSpeedAcc(int axis, int speed, int acc)` | — | — |
| `ClassBaseBoard.setSpeed@166` | `166–168` | `public virtual void setSpeed(int axis, int velocity)` | — | — |
| `ClassBaseBoard.setMaxCurrent@170` | `170–172` | `public virtual void setMaxCurrent(int axis, int current)` | — | — |
| `ClassBaseBoard.disableRightSwitch@174` | `174–176` | `public virtual void disableRightSwitch(int axis)` | — | — |
| `ClassBaseBoard.setAxisLimits@178` | `178–180` | `public virtual void setAxisLimits(int axis, int minLimit, int maxLimit)` | — | — |
| `ClassBaseBoard.stopMotor@182` | `182–184` | `public virtual void stopMotor(int axis, bool waitforstop = true)` | — | — |
| `ClassBaseBoard.setRdivPdiv@186` | `186–188` | `public virtual void setRdivPdiv(int axis, int rdiv, int pdiv)` | — | — |
| `ClassBaseBoard.disableLeftSwitch@190` | `190–192` | `public virtual void disableLeftSwitch(int axis)` | — | — |
| `ClassBaseBoard.queryHome@194` | `194–197` | `public virtual bool queryHome(int axis)` | — | — |
| `ClassBaseBoard.queryRightSensor@199` | `199–202` | `public virtual bool queryRightSensor(int axis)` | — | — |
| `ClassBaseBoard.axisSearchHome@204` | `204–207` | `public virtual int axisSearchHome(int axis, int speed)` | — | — |
| `ClassBaseBoard.setHome@209` | `209–211` | `public virtual void setHome(int axis)` | — | — |
| `ClassBaseBoard.readMaxCurrent@213` | `213–216` | `public virtual int readMaxCurrent(int axis)` | — | — |
| `ClassBaseBoard.moveSteps@218` | `218–221` | `public virtual int moveSteps(int axis, int steps, bool waitforstop = true)` | — | — |
| `ClassBaseBoard.GetWaitEventhandle@223` | `223–226` | `public virtual AutoResetEvent GetWaitEventhandle(int axis)` | — | — |
| `ClassBaseBoard.setMaxAcc@228` | `228–230` | `public virtual void setMaxAcc(int axis, int acc)` | — | — |
| `ClassBaseBoard.startFanService@232` | `232–234` | `public virtual void startFanService()` | — | — |
| `ClassBaseBoard.stopFanService@236` | `236–238` | `public virtual void stopFanService()` | — | — |

### 6.2 `decompiled_src/BioXPControlLib/ClassControlInterface.cs`

Source SHA-256: `86093e5270c82ea2e45cb4de449076372ca79d9485ba6de9565d5eb255811e6e`
Captured binary: `BioXPControlLib.dll`

| Method/member ID | Inclusive lines | Declaration/member | Literal sleeps | Decompiler warning lines |
|---|---:|---|---|---|
| `ClassControlInterface.m_AxisIODesignater` | `29–111` | `Logical axis/board/axis designator map` | — | — |
| `ClassControlInterface.ClassControlInterface@170` | `170–556` | `public ClassControlInterface(ControlLib controls, InterfaceCAN cancontrol, ClassBioXPSettings settings)` | — | 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255, 256, 257, 258, 259, 260, 261, 262, 263, 264, 265, 266, 267, 268, 269, 270, 271, 272, 273, 274, 275, 276, 277, 278, 279, 280, 281, 282, 283, 284, 285, 286, 287, 288, 289, 290, 291, 292, 293, 294, 295, 296, 297, 298, 299, 300, 301, 302, 303, 304, 305, 306, 307, 308, 309, 310, 311, 312, 313, 314, 315, 316, 317, 318, 319, 320, 321, 322, 323, 324, 325, 326, 327, 328, 329, 330, 331, 332, 333, 334, 335, 336, 337, 338, 339, 340, 341, 342, 343, 344, 345, 346, 347, 348, 349, 350, 351, 352, 353, 354, 355, 356, 357, 358, 359, 360, 361, 362, 363, 364, 365, 366, 367, 368, 369, 370, 371, 372, 373, 374, 375, 376, 377, 378, 379, 380, 381, 382 |
| `ClassControlInterface.openThermalDoor@2651` | `2651–2676` | `public bool openThermalDoor()` | — | — |
| `ClassControlInterface.closeThermalDoor@2678` | `2678–2712` | `public bool closeThermalDoor()` | — | 2680 |
| `ClassControlInterface.confirmAxis@2714` | `2714–2763` | `public bool confirmAxis(string axis)` | — | — |
| `ClassControlInterface.setColor@2765` | `2765–2799` | `public void setColor(int R, int G, int B)` | — | 2767 |
| `ClassControlInterface.GetThermalTemperature@2832` | `2832–2840` | `public double GetThermalTemperature()` | — | — |
| `ClassControlInterface.getNestTemp@2875` | `2875–2883` | `public double getNestTemp(int nest)` | — | — |
| `ClassControlInterface.GetLidTemperature@2885` | `2885–2894` | `public double GetLidTemperature()` | — | 2887 |
| `ClassControlInterface.GetTCPedestalTemp@2896` | `2896–2905` | `public double GetTCPedestalTemp()` | — | 2898 |
| `ClassControlInterface.SetTemperature@3007` | `3007–3014` | `public void SetTemperature(double Temperature, int duration, bool wait)` | — | 3009 |
| `ClassControlInterface.turnOffHeater@3057` | `3057–3067` | `public void turnOffHeater()` | — | 3059, 3060 |
| `ClassControlInterface.setChillerPWM@3078` | `3078–3117` | `public void setChillerPWM(string chiller = null, int pwm = 0)` | — | 3080, 3081, 3082, 3083 |
| `ClassControlInterface.readRCTemperature@3143` | `3143–3152` | `public double readRCTemperature()` | — | 3145 |
| `ClassControlInterface.readOCTemperature@3154` | `3154–3163` | `public double readOCTemperature()` | — | 3156 |
| `ClassControlInterface.initializeMotorsWithoutMotion@3181` | `3181–3265` | `public void initializeMotorsWithoutMotion()` | L3186=1ms, L3190=2ms, L3192=2ms, L3194=2ms, L3199=2ms, L3201=2ms, L3203=2ms, L3205=2ms, L3210=2ms, L3212=2ms, L3215=2ms, L3227=2ms, L3231=2ms, L3237=2ms, L3240=2ms, L3242=2ms, L3247=2ms, L3249=2ms, L3251=2ms, L3253=2ms, L3255=2ms | — |
| `ClassControlInterface.initializeMotors@3348` | `3348–3421` | `public void initializeMotors()` | L3370=20ms, L3373=40ms | — |
| `ClassControlInterface.setChillerCoolRate@3423` | `3423–3438` | `public void setChillerCoolRate(string chiller, double rate = -0.025)` | — | 3425, 3426 |
| `ClassControlInterface.activateBoard@3474` | `3474–3494` | `public void activateBoard()` | L3491=10ms | — |
| `ClassControlInterface.deactivateBoard@3496` | `3496–3505` | `public void deactivateBoard()` | — | — |
| `ClassControlInterface.CloseGripper@3569` | `3569–3593` | `public int CloseGripper(bool resetSpeed = true, bool resetStallGThre = true, int stallGuardThre = 5)` | — | — |
| `ClassControlInterface.getCurrentPosition@3644` | `3644–3661` | `public void getCurrentPosition(out int x, out int y, out int z)` | — | — |
| `ClassControlInterface.moveTo@3663` | `3663–3689` | `public int moveTo(locationID locationid, int column, int row, bool Tip10, bool highPos = true, bool runInParallel = true)` | — | 3665, 3666, 3667, 3668, 3669, 3670, 3671, 3672, 3673, 3674, 3675, 3676, 3677, 3678 |
| `ClassControlInterface.moveTo@3691` | `3691–3716` | `public int moveTo(locationID locationid, int offsetX, int offsetY, bool runInParallel = true)` | — | 3693, 3694, 3695, 3696, 3697, 3698 |
| `ClassControlInterface.scriptmoveTo@3718` | `3718–3732` | `public int scriptmoveTo(locationID currentLoc, wellID currentWell, locationID locationid, wellID well, int positionflag = 0)` | — | 3720, 3721, 3722, 3723, 3724, 3725, 3726, 3727, 3728 |
| `ClassControlInterface.scriptmoveTo@3734` | `3734–4015` | `public int scriptmoveTo(locationID currentLoc, wellID currentWell, locationID locationid, int column, int row, int positionflag = 0, bool runInParallel = true)` | L3872=50ms | 3736, 3737, 3738, 3739, 3740, 3741, 3742, 3743, 3744, 3745, 3746, 3747, 3748, 3749, 3750, 3751, 3752, 3753, 3754, 3755, 3756, 3757, 3758, 3759, 3760, 3761, 3762, 3763, 3764, 3765, 3766, 3767, 3768, 3769, 3770, 3771, 3772, 3773, 3774, 3775, 3776, 3777, 3778, 3779, 3780, 3781, 3782, 3783, 3784, 3785, 3786, 3787, 3788, 3789, 3790, 3791, 3792, 3793, 3794, 3795, 3796, 3797, 3798, 3799, 3800, 3801, 3802, 3803, 3804, 3805, 3806, 3807, 3808, 3809, 3810, 3811, 3812, 3813, 3814, 3815, 3816, 3817, 3818, 3819, 3820, 3821, 3822, 3823, 3824, 3825, 3826, 3827 |
| `ClassControlInterface.moveSteps@4165` | `4165–4204` | `public int moveSteps(string axis, int steps)` | — | — |
| `ClassControlInterface.moveX@4206` | `4206–4220` | `public void moveX(int x, int acc, bool waitforstop = true, bool appAdjustment = true, ManualResetEvent moveStarted = null)` | — | — |
| `ClassControlInterface.moveY@4222` | `4222–4231` | `public void moveY(int y, int acc, bool waitforstop = true, bool appAdjustment = true, ManualResetEvent moveStarted = null)` | — | — |
| `ClassControlInterface.moveX@4233` | `4233–4244` | `public void moveX(int x, bool waitforstop = true, bool appAdjustment = true)` | — | — |
| `ClassControlInterface.moveY@4246` | `4246–4252` | `public void moveY(int y, bool waitforstop = true, bool appAdjustment = true)` | — | — |
| `ClassControlInterface.moveZ@4254` | `4254–4266` | `public void moveZ(int z, int motorCurrent = 31, bool appAdjustment = true, bool waitforstop = true)` | — | — |
| `ClassControlInterface.moveTo@4463` | `4463–4621` | `public int moveTo(int x, int y, int z, bool runInParalel)` | L4538=1ms, L4559=600ms, L4600=300ms, L4612=1ms, L4613=1ms | 4465, 4466, 4467, 4468, 4469, 4470, 4471, 4472 |
| `ClassControlInterface.MoveZHome@4623` | `4623–4632` | `public int MoveZHome(bool rehome = true)` | — | — |
| `ClassControlInterface.query24voltage@4944` | `4944–4953` | `internal int query24voltage()` | — | 4946 |
| `ClassControlInterface.querydoorsensor@4955` | `4955–4964` | `internal int querydoorsensor()` | — | 4957 |
| `ClassControlInterface.querylatchsensor@4966` | `4966–4975` | `internal int querylatchsensor()` | — | 4968 |
| `ClassControlInterface.querySolenoidControl@4977` | `4977–4986` | `internal int querySolenoidControl()` | — | 4979 |
| `ClassControlInterface.setSolenoidControl@4988` | `4988–4995` | `internal void setSolenoidControl(int onoff)` | — | 4990 |
| `ClassControlInterface.HomeAxis@4997` | `4997–5052` | `internal int HomeAxis(string axis)` | — | — |
| `ClassControlInterface.HomeXY@5054` | `5054–5070` | `internal int[] HomeXY()` | — | — |
| `ClassControlInterface.moveAxis@5072` | `5072–5093` | `internal void moveAxis(string axis, int pos)` | — | — |
| `ClassControlInterface.setGripperCurrent@5392` | `5392–5398` | `internal void setGripperCurrent(int current)` | — | — |

### 6.3 `decompiled_src_can/ClassCanLib/ClassChillerBoard.cs`

Source SHA-256: `c048e2cdfcecc58c97f857e2c2bad85eb9766c2196981fa8fde67df7334981fd`
Captured binary: `ClassCanLib.dll`

| Method/member ID | Inclusive lines | Declaration/member | Literal sleeps | Decompiler warning lines |
|---|---:|---|---|---|
| `ClassChillerBoard.ClassChillerBoard@33` | `33–54` | `public ClassChillerBoard(InterfaceCAN cancontrol, ushort canID, interfaceBaseSettings settings)` | — | — |
| `ClassChillerBoard.activateBoard@56` | `56–74` | `public override void activateBoard()` | L59=1ms | — |
| `ClassChillerBoard.setChillerTemp@76` | `76–113` | `public int setChillerTemp(int bank, double tempinC)` | — | — |
| `ClassChillerBoard.chillerFan@115` | `115–121` | `public override void chillerFan(int bank, int speed = 190)` | — | — |
| `ClassChillerBoard.readChillerFanSpeed@123` | `123–131` | `public int readChillerFanSpeed(int bank)` | — | — |
| `ClassChillerBoard.readChillerTemperature@133` | `133–160` | `public override double readChillerTemperature(int bank)` | — | — |
| `ClassChillerBoard.readChillerPedestalTemperature@162` | `162–175` | `public double readChillerPedestalTemperature(int bank)` | — | — |
| `ClassChillerBoard.setChillerCoolRate@177` | `177–191` | `public void setChillerCoolRate(int bank, double rate)` | — | — |
| `ClassChillerBoard.setChillerHeatRate@193` | `193–207` | `public void setChillerHeatRate(int bank, double rate)` | — | — |
| `ClassChillerBoard.readChillerCoolRate@209` | `209–225` | `public double readChillerCoolRate(ChillerBank chiller)` | — | — |
| `ClassChillerBoard.readChillerHeatRate@227` | `227–243` | `public double readChillerHeatRate(ChillerBank chiller)` | — | — |
| `ClassChillerBoard.readChillerCurrent@245` | `245–248` | `public double readChillerCurrent(ChillerBank chiller)` | — | — |
| `ClassChillerBoard.readChillerNestTemp@250` | `250–266` | `public double readChillerNestTemp(ChillerBank chiller)` | — | — |
| `ClassChillerBoard.setChillerPWM@268` | `268–274` | `public void setChillerPWM(int chiller, int pwm = 0)` | — | — |
| `ClassChillerBoard.setChillerPedTemp@276` | `276–282` | `public void setChillerPedTemp(int chiller, double temp)` | — | — |
| `ClassChillerBoard.setChillerMaxCurrent@284` | `284–295` | `public void setChillerMaxCurrent(ChillerBank bank, double maxHeatingCurrent, double maxCoolingCurrent)` | — | — |
| `ClassChillerBoard.setChillerProportional@297` | `297–303` | `public void setChillerProportional(ChillerItem sel, double gain)` | — | — |
| `ClassChillerBoard.setChillerFeedForward@305` | `305–311` | `public void setChillerFeedForward(ChillerItem sel, double gain)` | — | — |
| `ClassChillerBoard.handleReturnMessageNovo@414` | `414–564` | `public override void handleReturnMessageNovo(byte[] msg)` | — | 416 |
| `ClassChillerBoard.startFanService@566` | `566–570` | `public override void startFanService()` | — | — |
| `ClassChillerBoard.stopFanService@572` | `572–576` | `public override void stopFanService()` | — | — |
| `ClassChillerBoard.SetTimerInterval@578` | `578–582` | `public override void SetTimerInterval(int interval)` | — | — |
| `ClassChillerBoard.resumeTempSetting@584` | `584–594` | `public override void resumeTempSetting()` | — | — |

### 6.4 `decompiled_src/BioXPControlLib/ControlLib.cs`

Source SHA-256: `f69b3529dcb9723c705ac55ecb3f035010cc294d3891de096c165bb20116f6c2`
Captured binary: `BioXPControlLib.dll`

| Method/member ID | Inclusive lines | Declaration/member | Literal sleeps | Decompiler warning lines |
|---|---:|---|---|---|
| `ControlLib.AdjustCamera@1883` | `1883–1920` | `private void AdjustCamera(CameraControlParameter cameraControlParameter)` | — | — |
| `ControlLib.AllLEDOff@1922` | `1922–1927` | `private void AllLEDOff()` | — | — |
| `ControlLib.CheckCamera@1929` | `1929–1960` | `public bool CheckCamera()` | — | — |
| `ControlLib.inspectCover@3663` | `3663–3768` | `public ErrorStatus inspectCover()` | — | 3665, 3666, 3667, 3668, 3669, 3670, 3671 |
| `ControlLib.InspectOutputLocation@3778` | `3778–3823` | `private int InspectOutputLocation(int expectedResult)` | L3790=700ms | 3780, 3781, 3782 |
| `ControlLib.checkRCCover@3825` | `3825–3848` | `private bool checkRCCover()` | L3835=700ms | 3827 |
| `ControlLib.checkCoverStorage@3850` | `3850–3889` | `private unsafe bool checkCoverStorage(locationID location)` | L3879=700ms | 3852, 3853, 3854, 3855, 3856, 3857, 3858 |
| `ControlLib.checkChillerCover@3891` | `3891–3942` | `private unsafe bool checkChillerCover(locationID location)` | L3922=400ms, L3923=500ms | 3893, 3894, 3895, 3896, 3897, 3898, 3899 |
| `ControlLib.setChilerTemperature@7056` | `7056–7069` | `public int setChilerTemperature(Chiller chiller, int temp)` | — | — |
| `ControlLib.parkGantry@7071` | `7071–7122` | `public void parkGantry(bool rehome = false)` | L7088=100ms | 7073, 7074, 7075, 7076, 7077, 7078 |
| `ControlLib.releasePlate@8116` | `8116–8230` | `public void releasePlate(locationID location, bool press_plate = false, bool runInParallel = true)` | L8156=5ms, L8170=200ms, L8179=200ms, L8212=1000ms, L8215=100ms | 8118, 8119, 8120, 8121, 8122, 8123, 8124, 8125, 8126, 8127, 8128, 8129, 8130, 8131, 8132, 8133, 8134, 8135, 8136, 8137, 8138, 8139, 8140, 8141, 8142, 8143, 8144, 8145 |
| `ControlLib.catchPlate@8262` | `8262–8410` | `public void catchPlate(plateName plate, bool runInParallel = true)` | L8365=1000ms, L8368=100ms, L8374=100ms | 8264, 8265, 8266, 8267, 8268, 8269, 8270, 8271, 8272, 8273, 8274, 8275, 8276, 8277, 8278, 8279, 8280, 8281, 8282, 8283, 8284, 8285, 8286, 8287, 8288, 8289, 8290, 8291, 8292, 8293, 8294, 8295, 8296, 8297, 8298, 8299, 8300, 8301, 8302, 8303, 8304, 8305, 8306 |
| `ControlLib.doorOpen@8541` | `8541–8610` | `public bool doorOpen(bool bopen, bool scriptRunning = false)` | — | 8543, 8544 |
| `ControlLib.setThermalTemperature@8653` | `8653–8656` | `private void setThermalTemperature(double temperature, int duration, double rate)` | — | — |
| `ControlLib.setLIDTemperature@8658` | `8658–8668` | `private void setLIDTemperature(double temperature, int duration, double rate, string wait)` | — | — |
| `ControlLib.checkDoorStatus@8670` | `8670–8726` | `public bool checkDoorStatus()` | L8673=500ms, L8679=800ms, L8703=300ms | — |
| `ControlLib.initialCheck@8728` | `8728–8760` | `public bool initialCheck()` | L8734=200ms, L8749=50ms | — |
| `ControlLib.initializeMotion@8797` | `8797–8856` | `public void initializeMotion()` | L8806=500ms, L8818=100ms, L8831=2ms | — |
| `ControlLib.SnapshotImage@8951` | `8951–8994` | `public void SnapshotImage(string condition)` | — | — |
| `ControlLib.unlockDoor@10451` | `10451–10459` | `public void unlockDoor()` | — | — |
| `ControlLib.setLEDColor@10559` | `10559–10562` | `public void setLEDColor(int Red, int Green, int Blue)` | — | — |
| `ControlLib.forceAbortMotion@10564` | `10564–10610` | `public void forceAbortMotion(bool waitforpipette = false)` | — | — |
| `ControlLib.selftest@10688` | `10688–10786` | `public bool selftest()` | L10717=100ms, L10731=100ms | — |
| `ControlLib.TCSelfTest@10788` | `10788–10865` | `private void TCSelfTest(bool selftest = true)` | — | — |
| `ControlLib.RCSelfTest@10867` | `10867–10932` | `private void RCSelfTest(bool selftest = true)` | L10905=1ms | — |
| `ControlLib.OCSelfTest@10934` | `10934–10999` | `private void OCSelfTest(bool selftest = true)` | L10972=1ms | — |
| `ControlLib.waitforcompletion@11001` | `11001–11022` | `public bool waitforcompletion(int timeout = 4000)` | — | — |

### 6.5 `decompiled_src_can/ClassCanLib/ClassDeckBoard.cs`

Source SHA-256: `bcae5ce4662cb3208dbc2fdf6e4c7ad0e23774307b633fc7a9ce5b727318698f`
Captured binary: `ClassCanLib.dll`

| Method/member ID | Inclusive lines | Declaration/member | Literal sleeps | Decompiler warning lines |
|---|---:|---|---|---|
| `ClassDeckBoard.ClassDeckBoard@39` | `39–54` | `public ClassDeckBoard(InterfaceCAN cancontrol, ushort canID, interfaceBaseSettings settings)` | — | — |
| `ClassDeckBoard.goHome@73` | `73–132` | `public override int goHome(bool rehome, int axis, int speed, bool waitforstop = true)` | — | — |
| `ClassDeckBoard.setHome@134` | `134–137` | `public override void setHome(int axis)` | — | — |
| `ClassDeckBoard.setStallGuardThreshold@139` | `139–142` | `public override void setStallGuardThreshold(int axis, int threshold)` | — | — |
| `ClassDeckBoard.setMaxCurrent@144` | `144–147` | `public override void setMaxCurrent(int axis, int current)` | — | — |
| `ClassDeckBoard.readMaxCurrent@149` | `149–152` | `public override int readMaxCurrent(int axis)` | — | — |
| `ClassDeckBoard.setSpeedAcc@154` | `154–158` | `public override void setSpeedAcc(int axis, int speed, int acc)` | — | — |
| `ClassDeckBoard.setRdivPdiv@165` | `165–169` | `public override void setRdivPdiv(int axis, int rdiv, int pdiv)` | — | — |
| `ClassDeckBoard.moveToAbs@171` | `171–227` | `public override int moveToAbs(int axis, int position, bool waitforstop = true, bool stallRecover = false, bool gripperRecover = false)` | L196=1ms | — |
| `ClassDeckBoard.moveSteps@229` | `229–284` | `public override int moveSteps(int axis, int steps, bool waitforstop = true)` | — | — |
| `ClassDeckBoard.stopMotor@302` | `302–316` | `public override void stopMotor(int axis, bool waitforstop = true)` | — | — |
| `ClassDeckBoard.axisSearchHome@362` | `362–381` | `public override int axisSearchHome(int axis, int speed)` | L377=500ms | — |
| `ClassDeckBoard.queryHome@383` | `383–395` | `public override bool queryHome(int axis)` | — | — |
| `ClassDeckBoard.queryRightSensor@397` | `397–409` | `public override bool queryRightSensor(int axis)` | — | — |
| `ClassDeckBoard.checkMotorStopped@411` | `411–427` | `public bool checkMotorStopped(int axis, bool reset = false)` | — | — |
| `ClassDeckBoard.setAxisLimits@429` | `429–432` | `public override void setAxisLimits(int axis, int minLimit, int maxLimit)` | — | — |
| `ClassDeckBoard.setSpeed@439` | `439–442` | `public override void setSpeed(int axis, int velocity)` | — | — |
| `ClassDeckBoard.disableRightSwitch@444` | `444–447` | `public override void disableRightSwitch(int axis)` | — | — |
| `ClassDeckBoard.disableLeftSwitch@449` | `449–452` | `public override void disableLeftSwitch(int axis)` | — | — |
| `ClassDeckBoard.setColor@465` | `465–470` | `public void setColor(byte r, byte g, byte b)` | — | — |

### 6.6 `decompiled_src_bioxpcommon/BioXPCommonLib/DefaultParameters.cs`

Source SHA-256: `04f53c129317f8ae508d6971d2cf9fb534e1bc8307b192f8db395cbbcdf64fd2`
Captured binary: `BioXPCommonLib.dll`

| Method/member ID | Inclusive lines | Declaration/member | Literal sleeps | Decompiler warning lines |
|---|---:|---|---|---|
| `DefaultParameters.ForceToHighHome@81` | `81–84` | `public static void ForceToHighHome()` | — | — |

### 6.7 `decompiled_src_can/ClassCanLib/ClassHeadBoard.cs`

Source SHA-256: `342a9b2f09731002194b67e37f1d4e866ecbfb3c25effd85b3cd609e8cbdd1ea`
Captured binary: `ClassCanLib.dll`

| Method/member ID | Inclusive lines | Declaration/member | Literal sleeps | Decompiler warning lines |
|---|---:|---|---|---|
| `ClassHeadBoard.ClassHeadBoard@23` | `23–39` | `public ClassHeadBoard(InterfaceCAN cancontrol, ushort canID, ClassBioXPSettings settings)` | — | — |
| `ClassHeadBoard.goHome@60` | `60–119` | `public override int goHome(bool rehome, int axis, int speed, bool waitforstop = true)` | — | — |
| `ClassHeadBoard.setHome@121` | `121–124` | `public override void setHome(int axis)` | — | — |
| `ClassHeadBoard.setStallGuardThreshold@126` | `126–129` | `public override void setStallGuardThreshold(int axis, int threshold)` | — | — |
| `ClassHeadBoard.setMaxCurrent@131` | `131–134` | `public override void setMaxCurrent(int axis, int current)` | — | — |
| `ClassHeadBoard.readMaxCurrent@136` | `136–139` | `public override int readMaxCurrent(int axis)` | — | — |
| `ClassHeadBoard.setSpeedAcc@141` | `141–145` | `public override void setSpeedAcc(int axis, int speed, int acc)` | — | — |
| `ClassHeadBoard.setRdivPdiv@152` | `152–156` | `public override void setRdivPdiv(int axis, int rdiv, int pdiv)` | — | — |
| `ClassHeadBoard.moveToAbs@158` | `158–229` | `public override int moveToAbs(int axis, int position, bool waitforstop = true, bool stallRecover = false, bool gripperRecover = false)` | L183=1ms, L187=10000ms, L197=5000ms | — |
| `ClassHeadBoard.moveSteps@231` | `231–286` | `public override int moveSteps(int axis, int steps, bool waitforstop = true)` | — | — |
| `ClassHeadBoard.stopMotor@304` | `304–318` | `public override void stopMotor(int axis, bool waitforstop = true)` | — | — |
| `ClassHeadBoard.axisSearchHome@368` | `368–387` | `public override int axisSearchHome(int axis, int speed)` | L383=500ms | — |
| `ClassHeadBoard.queryHome@389` | `389–401` | `public override bool queryHome(int axis)` | — | — |
| `ClassHeadBoard.queryRightSensor@403` | `403–415` | `public override bool queryRightSensor(int axis)` | — | — |
| `ClassHeadBoard.checkMotorStopped@417` | `417–433` | `public bool checkMotorStopped(int axis, bool reset = false)` | — | — |
| `ClassHeadBoard.setAxisLimits@435` | `435–438` | `public override void setAxisLimits(int axis, int minLimit, int maxLimit)` | — | — |
| `ClassHeadBoard.setSpeed@508` | `508–511` | `public override void setSpeed(int axis, int velocity)` | — | — |
| `ClassHeadBoard.disableRightSwitch@513` | `513–516` | `public override void disableRightSwitch(int axis)` | — | — |
| `ClassHeadBoard.disableLeftSwitch@518` | `518–521` | `public override void disableLeftSwitch(int axis)` | — | — |

### 6.8 `decompiled_src_can/ClassCanLib/InterfaceCAN.cs`

Source SHA-256: `aed90411d2966ae45142f8a988a2c6757011845c3ab2c5823c98090783419da1`
Captured binary: `ClassCanLib.dll`

| Method/member ID | Inclusive lines | Declaration/member | Literal sleeps | Decompiler warning lines |
|---|---:|---|---|---|
| `InterfaceCAN` | `3–32` | `Interface and events` | — | — |
| `InterfaceCAN.CAN_READY` | `5–5` | `Transport readiness property` | — | — |
| `InterfaceCAN.TransmitMessage` | `19–19` | `Serialized CAN request/reply contract` | — | — |
| `InterfaceCAN.startService` | `21–21` | `Transport service start` | — | — |
| `InterfaceCAN.trigDoorOpenEvent` | `23–23` | `Door event projection` | — | — |
| `InterfaceCAN.trigBoardErrorEvent` | `25–25` | `Board error projection` | — | — |
| `InterfaceCAN.trigLatchEvent` | `27–27` | `Latch event projection` | — | — |
| `InterfaceCAN.calculatePressureoffset` | `29–29` | `Pipette pressure offset dispatch` | — | — |
| `InterfaceCAN.close` | `31–31` | `Transport close` | — | — |

### 6.9 `decompiled_src_can/ClassCanLib/ClassIOControl.cs`

Source SHA-256: `cb684f0afc788503601ce4b71babacaf83bb57bdccd3cbfba20c9b49c1fa2b1f`
Captured binary: `ClassCanLib.dll`

| Method/member ID | Inclusive lines | Declaration/member | Literal sleeps | Decompiler warning lines |
|---|---:|---|---|---|
| `ClassIOControl.ClassIOControl@25` | `25–31` | `public ClassIOControl(InterfaceCAN control, ushort canID)` | — | — |
| `ClassIOControl.setLED@54` | `54–90` | `public int setLED(byte RGBmask, int intensity)` | — | — |
| `ClassIOControl.query24VSensor@92` | `92–111` | `public int query24VSensor()` | — | — |
| `ClassIOControl.querDoorSensor@113` | `113–132` | `public int querDoorSensor()` | — | — |
| `ClassIOControl.querySolenoidControl@134` | `134–153` | `public int querySolenoidControl()` | — | — |
| `ClassIOControl.querLatchSensor@155` | `155–174` | `public int querLatchSensor()` | — | — |
| `ClassIOControl.setSolenoidControl@176` | `176–204` | `public int setSolenoidControl(int onoff)` | — | — |

### 6.10 `decompiled_src_genbotapp/GenBotApp/BioXPMainWindow.cs`

Source SHA-256: `b288a45e2de54cd2c8d30a4498a343cd6f423aff7e88a78847076bfbfb4e904c`
Captured binary: `GenBotApp.exe`

| Method/member ID | Inclusive lines | Declaration/member | Literal sleeps | Decompiler warning lines |
|---|---:|---|---|---|
| `BioXPMainWindow.BioXPMainWindow@253` | `253–292` | `public BioXPMainWindow()` | — | 255, 256, 257, 258 |
| `BioXPMainWindow.BioXPMainWindow@294` | `294–345` | `public BioXPMainWindow(string bixo_dir, ClassBioXPSettings settings, ClassMachineStatus machinestatus, ClassVirtualBioXP VirtualXP)` | — | 296, 297, 298, 299, 300, 301 |
| `BioXPMainWindow.initializeEnvironment@973` | `973–1027` | `private void initializeEnvironment()` | — | 975 |
| `BioXPMainWindow.initializeSystem@1046` | `1046–1342` | `private void initializeSystem(bool skipInitializeMotion = false)` | — | 1048, 1049, 1050, 1051, 1052, 1053, 1054, 1055, 1056, 1057, 1058, 1059, 1060, 1061, 1062, 1063, 1064, 1065, 1066, 1067, 1068, 1069, 1070, 1071, 1072, 1073, 1074, 1075, 1076, 1077, 1078, 1079, 1080, 1081, 1082, 1083, 1084, 1085, 1086, 1087, 1088, 1089, 1090, 1091, 1092, 1093 |
| `BioXPMainWindow.SelfTest@1562` | `1562–1565` | `private bool SelfTest()` | — | — |
| `BioXPMainWindow.PrepareToRunJob@1567` | `1567–1586` | `private ErrorStatus PrepareToRunJob(out string jobid_barcode, out string reagent_barcode)` | — | 1569, 1570, 1571, 1572, 1573 |
| `BioXPMainWindow.PrepareToRunJob@1588` | `1588–1809` | `private ErrorStatus PrepareToRunJob()` | — | 1590, 1591, 1592, 1593, 1594, 1595, 1596, 1597, 1598, 1599, 1600, 1601, 1602, 1603, 1604, 1605, 1606, 1607, 1608, 1609, 1610, 1611, 1612, 1613, 1614 |
| `BioXPMainWindow.motion_thread_process@2030` | `2030–2101` | `private void motion_thread_process()` | — | 2032, 2033, 2034, 2035, 2036, 2037, 2038 |
| `BioXPMainWindow.m_pageWarning_buttonclicked@2611` | `2611–2844` | `private void m_pageWarning_buttonclicked(int button, WarningSituation STATUS)` | — | 2613, 2614 |
| `BioXPMainWindow.UpdateCheck@4264` | `4264–4309` | `private bool UpdateCheck()` | — | — |

### 6.11 `decompiled_src_can/ClassCanLib/ClassMotor.cs`

Source SHA-256: `9fb1b4bec771165053a82b4fe95510615d6ed9beda1a041280584ceb4ab7fe99`
Captured binary: `ClassCanLib.dll`

| Method/member ID | Inclusive lines | Declaration/member | Literal sleeps | Decompiler warning lines |
|---|---:|---|---|---|
| `ClassMotor.ClassMotor@60` | `60–72` | `public ClassMotor(InterfaceCAN cancontrol, ushort canID, byte axis)` | — | — |
| `ClassMotor.MoveLeft@74` | `74–116` | `public int MoveLeft(int speed = 200)` | — | — |
| `ClassMotor.MoveRight@118` | `118–159` | `public int MoveRight(int speed = 200)` | — | — |
| `ClassMotor.StopMotor@161` | `161–183` | `public int StopMotor()` | — | — |
| `ClassMotor.disableRightLimitSwitch@185` | `185–206` | `public int disableRightLimitSwitch()` | — | — |
| `ClassMotor.disableLeftLimitSwitch@208` | `208–229` | `public int disableLeftLimitSwitch()` | — | — |
| `ClassMotor.MovetoRelPosition@231` | `231–261` | `public int MovetoRelPosition(int steps)` | — | — |
| `ClassMotor.MoveHome@263` | `263–288` | `public int MoveHome()` | — | — |
| `ClassMotor.moveToAbs@290` | `290–340` | `public int moveToAbs(int pos)` | — | — |
| `ClassMotor.setMaxSpeed@342` | `342–378` | `public int setMaxSpeed(int speed)` | — | — |
| `ClassMotor.setMaxAcc@380` | `380–416` | `public int setMaxAcc(int acceleration)` | — | — |
| `ClassMotor.setMaxCurrent@418` | `418–462` | `public int setMaxCurrent(int current)` | — | — |
| `ClassMotor.readMaxCurrent@464` | `464–490` | `public int readMaxCurrent()` | — | — |
| `ClassMotor.setHome@492` | `492–517` | `public int setHome()` | — | — |
| `ClassMotor.setRampMode@519` | `519–540` | `public int setRampMode(int mode)` | — | — |
| `ClassMotor.setStallGuardThreshold@542` | `542–563` | `public int setStallGuardThreshold(int threshold)` | — | — |
| `ClassMotor.queryActualPosition@565` | `565–591` | `public int queryActualPosition()` | — | — |
| `ClassMotor.queryMotorSpeed@593` | `593–621` | `public int queryMotorSpeed()` | — | — |
| `ClassMotor.queryReachedPosition@623` | `623–639` | `public int queryReachedPosition()` | — | — |
| `ClassMotor.queryLeftSwitchStatus@641` | `641–664` | `public int queryLeftSwitchStatus()` | — | — |
| `ClassMotor.queryRightSwitchStatus@666` | `666–689` | `public int queryRightSwitchStatus()` | — | — |
| `ClassMotor.queryMotorStop@691` | `691–720` | `public int queryMotorStop(int axis = -1)` | — | — |
| `ClassMotor.setPDIV@722` | `722–743` | `public int setPDIV(int pdiv)` | — | — |
| `ClassMotor.setRDIV@745` | `745–766` | `public int setRDIV(int rdiv)` | — | — |
| `ClassMotor.setLimits@768` | `768–772` | `public void setLimits(int lowLimits, int highLimits)` | — | — |
| `ClassMotor.beyondLimit@774` | `774–788` | `public bool beyondLimit(int pos)` | — | — |
| `ClassMotor.atTarget@790` | `790–798` | `internal bool atTarget(int position)` | — | — |

### 6.12 `decompiled_src_can/ClassCanLib/ClassNovo.cs`

Source SHA-256: `11293074caec278076723666e69022b547c43f32b5fa886c99f75d5b60043d06`
Captured binary: `ClassCanLib.dll`

| Method/member ID | Inclusive lines | Declaration/member | Literal sleeps | Decompiler warning lines |
|---|---:|---|---|---|
| `ClassNovo.ClassNovo@47` | `47–60` | `public ClassNovo()` | — | — |
| `ClassNovo.ClassNovo@62` | `62–76` | `public ClassNovo(interfaceBaseSettings settings)` | — | — |
| `ClassNovo.startService@84` | `84–89` | `public void startService()` | — | 86, 87 |
| `ClassNovo.close@91` | `91–94` | `public void close()` | — | — |
| `ClassNovo.MessageProcessingThread@146` | `146–162` | `private void MessageProcessingThread()` | — | — |
| `ClassNovo.ClassNovo_GotMessage@164` | `164–167` | `private void ClassNovo_GotMessage(TrafficPacket packet)` | — | — |
| `ClassNovo.GotMessageProcess@169` | `169–192` | `private void GotMessageProcess(TrafficPacket packet)` | — | 171, 172 |
| `ClassNovo.TransmitMessage@194` | `194–226` | `public byte[] TransmitMessage(byte SIDH, byte SIDL, byte[] CMD, string cmdMessage = "", int timeout = 60000)` | L199=1ms, L212=10ms | — |
| `ClassNovo.calculatePressureoffset@228` | `228–231` | `public void calculatePressureoffset()` | — | — |

### 6.13 `decompiled_src_novo/NovoCANUSBLib/ClassNovoCANUSB.cs`

Source SHA-256: `e4cf1c311ed5ae79e9490564a48947bced5f46af36890cf4a77120a3b50ffb06`
Captured binary: `NovoCANUSBLib.dll`

| Method/member ID | Inclusive lines | Declaration/member | Literal sleeps | Decompiler warning lines |
|---|---:|---|---|---|
| `ClassNovoCANUSB.ClassNovoCANUSB@243` | `243–288` | `public ClassNovoCANUSB()` | — | 245, 246, 247, 248 |
| `ClassNovoCANUSB.ClassNovoCANUSB@290` | `290–335` | `public ClassNovoCANUSB(interfaceBaseSettings settings)` | — | 292, 293, 294, 295 |
| `ClassNovoCANUSB.UpdateDevices@357` | `357–417` | `private void UpdateDevices()` | — | — |
| `ClassNovoCANUSB.OnReplyPacketAvailable@424` | `424–428` | `private void OnReplyPacketAvailable(object sender, CanPacketEventArgs e)` | — | — |
| `ClassNovoCANUSB.SendData@430` | `430–446` | `private void SendData()` | — | 432, 433 |
| `ClassNovoCANUSB.SendRaw@448` | `448–463` | `private void SendRaw()` | — | 450, 451 |
| `ClassNovoCANUSB.sendCommand@474` | `474–563` | `public byte[] sendCommand(int ID, int dlc, List<byte> data, string cmdMessage = "", int timeout = 60000)` | — | — |
| `ClassNovoCANUSB.transmitCommand@565` | `565–729` | `private TrafficPacket transmitCommand(int ID, int dlc, List<byte> data, string cmdMessage = "")` | L589=0ms, L620=0ms, L631=0ms | 567, 568, 569, 570, 571, 572, 573, 574 |
| `ClassNovoCANUSB.ProcessReceivedTrafficPacket@787` | `787–977` | `private void ProcessReceivedTrafficPacket(TrafficPacket packet)` | — | — |
| `ClassNovoCANUSB.IsAMatch@979` | `979–990` | `private bool IsAMatch(ICanPacket sentPacket, ICanPacket receivedPacket)` | — | — |
| `ClassNovoCANUSB.IsAMatchPipette@992` | `992–1050` | `private int IsAMatchPipette(TrafficPacket sentPacket, TrafficPacket receivedPacket)` | — | — |
| `ClassNovoCANUSB.IsAMatchPipetteUIM@1052` | `1052–1066` | `private int IsAMatchPipetteUIM(TrafficPacket sentPacket, TrafficPacket receivedPacket)` | — | — |
| `ClassNovoCANUSB.filterMassage@1121` | `1121–1133` | `private bool filterMassage(TrafficPacket packet)` | — | — |
| `ClassNovoCANUSB.calculatePressureoffset@1135` | `1135–1151` | `public void calculatePressureoffset()` | — | — |
| `ClassNovoCANUSB.closeDevice@1153` | `1153–1156` | `protected void closeDevice()` | — | — |

### 6.14 `decompiled_src_can/BioXPControlLib/ClassPipette.cs`

Source SHA-256: `681f959cf527b060cece17b3cf7ff59c1ba1f5ead99fea53520d09486ac0c957`
Captured binary: `BioXPControlLib.dll`

| Method/member ID | Inclusive lines | Declaration/member | Literal sleeps | Decompiler warning lines |
|---|---:|---|---|---|
| `ClassPipette.TipLoaded` | `117–127` | `Per-channel tip state` | — | — |
| `ClassPipette.CommandCompleted` | `129–139` | `Per-channel completion state` | — | — |
| `ClassPipette.ClassPipette@143` | `143–163` | `public ClassPipette(InterfaceCAN cancontrol, ushort id)` | — | — |
| `ClassPipette.SendCommand@174` | `174–177` | `private void SendCommand(PipetteCommands cmdCat, byte[] address, byte[] cmd, bool checkInitialized = true, string cmdMessage = "")` | — | — |
| `ClassPipette.SendCommand@179` | `179–192` | `private byte[] SendCommand(byte[] address, byte[] cmd, bool checkInitialized = true, string cmdMessage = "")` | — | — |
| `ClassPipette.initiate@194` | `194–209` | `public void initiate(bool forceInitialize = false)` | L204=100ms | — |
| `ClassPipette.ejectTip@301` | `301–310` | `public void ejectTip()` | — | — |
| `ClassPipette.QueryStatus@528` | `528–535` | `public void QueryStatus()` | — | — |
| `ClassPipette.QueryTipStatus@571` | `571–597` | `public int QueryTipStatus()` | — | — |
| `ClassPipette.processMessage@638` | `638–745` | `public PipetteErrorCode processMessage(int dlc, byte[] msg)` | — | — |
| `ClassPipette.enablePressureStream@757` | `757–772` | `public void enablePressureStream(bool enable)` | — | — |

### 6.15 `decompiled_src/BioXPControlLib/ClassPipetteCollection.cs`

Source SHA-256: `ffe3729fa35642d04ef6fe45501e52200dd7c2977a70902aa20e46fb26d4011e`
Captured binary: `BioXPControlLib.dll`

| Method/member ID | Inclusive lines | Declaration/member | Literal sleeps | Decompiler warning lines |
|---|---:|---|---|---|
| `ClassPipetteCollection.TipExist` | `90–101` | `Four-channel tip predicate` | — | — |
| `ClassPipetteCollection.ClassPipetteCollection@153` | `153–233` | `public ClassPipetteCollection(ControlLib controls, InterfaceCAN cancontrol, ClassBioXPSettings settings, ClassMachineStatus status)` | — | 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186 |
| `ClassPipetteCollection.enablePressureStream@240` | `240–246` | `public void enablePressureStream(bool on)` | — | — |
| `ClassPipetteCollection.initiateGroup@677` | `677–693` | `public void initiateGroup()` | L690=1000ms | — |
| `ClassPipetteCollection.checkedPipetteStatus@726` | `726–748` | `public bool checkedPipetteStatus()` | L736=30ms, L738=1ms | — |
| `ClassPipetteCollection.ejectAllTips@1176` | `1176–1235` | `public void ejectAllTips(bool checkMissingTip = true, bool wait = true)` | L1231=500ms | 1178 |
| `ClassPipetteCollection.ejectTip@1237` | `1237–1240` | `public void ejectTip(int tip)` | — | — |
| `ClassPipetteCollection.KeepTip@1242` | `1242–1263` | `public void KeepTip(int tip)` | — | — |
| `ClassPipetteCollection.verifyEjectTip@1265` | `1265–1323` | `public void verifyEjectTip(int tip = -1)` | — | — |
| `ClassPipetteCollection.terminatecommands@1325` | `1325–1334` | `public void terminatecommands()` | — | — |
| `ClassPipetteCollection.queryTipStatus@1336` | `1336–1358` | `public int queryTipStatus(int pipette = -1)` | — | — |
| `ClassPipetteCollection.queryIndividualTipStatus@1360` | `1360–1371` | `public bool[] queryIndividualTipStatus()` | — | — |
| `ClassPipetteCollection.waitforcompletion@1425` | `1425–1461` | `internal bool waitforcompletion(string job, int timeout = 7000)` | — | — |

### 6.16 `decompiled_src_bioxpcommon/BioXPCommonLib/ClassBioXPSettings.cs`

Source SHA-256: `08155dc24602bc12cf25af745c74cc478f33e0f2675fc0d4b6f2e1ba917d8d41`
Captured binary: `BioXPCommonLib.dll`

| Method/member ID | Inclusive lines | Declaration/member | Literal sleeps | Decompiler warning lines |
|---|---:|---|---|---|
| `ClassBioXPSettings.ClassBioXPSettings@1411` | `1411–1489` | `public ClassBioXPSettings(ResourceManager rsManager, string appDir)` | — | 1413, 1414, 1415, 1416, 1417, 1418, 1419, 1420, 1421, 1422, 1423, 1424, 1425, 1426, 1427, 1428, 1429, 1430, 1431, 1432, 1433, 1434, 1435, 1436, 1437, 1438, 1439, 1440, 1441, 1442, 1443, 1444, 1445, 1446, 1447, 1448, 1449, 1450, 1451, 1452, 1453, 1454, 1455, 1456, 1457, 1458, 1459, 1460, 1461, 1462, 1463, 1464, 1465, 1466, 1467, 1468, 1469, 1470, 1471, 1472, 1473, 1474, 1475, 1476, 1477, 1478 |
| `ClassBioXPSettings.loadOperationParameters@2516` | `2516–2709` | `public void loadOperationParameters()` | — | 2518, 2519 |
| `ClassBioXPSettings.loadConfig@2798` | `2798–3530` | `public virtual void loadConfig()` | — | 2800, 2801, 2802, 2803, 2804, 2805, 2806, 2807, 2808, 2809, 2810, 2811, 2812, 2813, 2814, 2815, 2816, 2817, 2818, 2819, 2820, 2821, 2822, 2823, 2824, 2825, 2826, 2827, 2828, 2829, 2830, 2831, 2832, 2833, 2834, 2835, 2836, 2837, 2838, 2839, 2840, 2841, 2842 |

### 6.17 `decompiled_src_can/ClassCanLib/ClassThermalBoard.cs`

Source SHA-256: `23d50725da200044422fde56b00611df708b514cef0f3637b2ad8d19e0b23f26`
Captured binary: `ClassCanLib.dll`

| Method/member ID | Inclusive lines | Declaration/member | Literal sleeps | Decompiler warning lines |
|---|---:|---|---|---|
| `ClassThermalBoard.ClassThermalBoard@46` | `46–64` | `public ClassThermalBoard(InterfaceCAN cancontrol, ushort canID, ClassBioXPSettings settings)` | — | — |
| `ClassThermalBoard.setHome@179` | `179–182` | `public override void setHome(int axis)` | — | — |
| `ClassThermalBoard.setStallGuardThreshold@184` | `184–187` | `public override void setStallGuardThreshold(int axis, int threshold)` | — | — |
| `ClassThermalBoard.setMaxCurrent@189` | `189–192` | `public override void setMaxCurrent(int axis, int current)` | — | — |
| `ClassThermalBoard.readMaxCurrent@194` | `194–197` | `public override int readMaxCurrent(int axis)` | — | — |
| `ClassThermalBoard.setSpeedAcc@199` | `199–203` | `public override void setSpeedAcc(int axis, int speed, int acc)` | — | — |
| `ClassThermalBoard.setRdivPdiv@210` | `210–214` | `public override void setRdivPdiv(int axis, int rdiv, int pdiv)` | — | — |
| `ClassThermalBoard.moveToAbs@216` | `216–272` | `public override int moveToAbs(int axis, int position, bool waitforstop = true, bool stallRecover = false, bool gripperRecover = false)` | L241=1ms | — |
| `ClassThermalBoard.moveSteps@274` | `274–330` | `public override int moveSteps(int axis, int steps, bool waitforstop = true)` | — | — |
| `ClassThermalBoard.stopMotor@348` | `348–362` | `public override void stopMotor(int axis, bool waitforstop = true)` | — | — |
| `ClassThermalBoard.doorSearchHome@364` | `364–410` | `public override void doorSearchHome(int axis, int speed, int stallThread)` | L389=50ms | — |
| `ClassThermalBoard.axisSearchHome@412` | `412–431` | `public override int axisSearchHome(int axis, int speed)` | L427=500ms | — |
| `ClassThermalBoard.queryHome@433` | `433–445` | `public override bool queryHome(int axis)` | — | — |
| `ClassThermalBoard.queryRightSensor@447` | `447–459` | `public override bool queryRightSensor(int axis)` | — | — |
| `ClassThermalBoard.checkMotorStopped@461` | `461–477` | `public bool checkMotorStopped(int axis, bool reset = false)` | — | — |
| `ClassThermalBoard.setAxisLimits@479` | `479–482` | `public override void setAxisLimits(int axis, int minLimit, int maxLimit)` | — | — |
| `ClassThermalBoard.setSpeed@489` | `489–492` | `public override void setSpeed(int axis, int velocity)` | — | — |
| `ClassThermalBoard.disableRightSwitch@494` | `494–497` | `public override void disableRightSwitch(int axis)` | — | — |
| `ClassThermalBoard.disableLeftSwitch@499` | `499–502` | `public override void disableLeftSwitch(int axis)` | — | — |

### 6.18 `decompiled_src_can/ClassCanLib/ClassThermalControl.cs`

Source SHA-256: `fdefde9b38c70b9fec47e6ffdd4929ad8f25bc90c57c43baf03722addbe6cf25`
Captured binary: `ClassCanLib.dll`

| Method/member ID | Inclusive lines | Declaration/member | Literal sleeps | Decompiler warning lines |
|---|---:|---|---|---|
| `ClassThermalControl.ClassThermalControl@118` | `118–135` | `public ClassThermalControl(InterfaceCAN cancontrol, ushort canID, byte axis)` | — | — |
| `ClassThermalControl.setChillerTemperature@259` | `259–264` | `public void setChillerTemperature(double temp, int duration)` | — | — |
| `ClassThermalControl.setChillerTemperature@271` | `271–326` | `public int setChillerTemperature(double temp)` | — | — |
| `ClassThermalControl.turnChillerFanOnOff@364` | `364–382` | `public int turnChillerFanOnOff(int speed = 204)` | — | — |
| `ClassThermalControl.readChillerTemperature@384` | `384–464` | `public double readChillerTemperature(int axis)` | — | — |
| `ClassThermalControl.setGP@505` | `505–542` | `public int setGP(int command, int bank, int value)` | — | — |
| `ClassThermalControl.readGP@544` | `544–580` | `public double readGP(int command, int bank)` | — | — |
| `ClassThermalControl.getCoolRampRate@589` | `589–592` | `public double getCoolRampRate()` | — | — |
| `ClassThermalControl.getHeatRampRate@594` | `594–597` | `public double getHeatRampRate()` | — | — |
| `ClassThermalControl.getTECCurrent@599` | `599–602` | `public double getTECCurrent(int bank)` | — | — |
| `ClassThermalControl.readNestTemp@736` | `736–739` | `public double readNestTemp(int nest)` | — | — |
| `ClassThermalControl.getLidHeatRampRate@964` | `964–967` | `public double getLidHeatRampRate()` | — | — |
| `ClassThermalControl.startFanService@1064` | `1064–1067` | `internal void startFanService()` | — | — |

### 6.19 `decompiled_src_vision/CVisionLib/ClassFrameGrabber.cs`

Source SHA-256: `6eec22f02eae5b4738a6d857b57691009dacd5b1b9bb615d41e9d8a8e4d28501`
Captured binary: `CVisionLib.dll`

| Method/member ID | Inclusive lines | Declaration/member | Literal sleeps | Decompiler warning lines |
|---|---:|---|---|---|
| `ClassFrameGrabber.ClassFrameGrabber@28` | `28–34` | `public unsafe ClassFrameGrabber()` | — | — |
| `ClassFrameGrabber.checkLabel@9284` | `9284–9859` | `public unsafe bool checkLabel(string fname)` | — | — |
| `ClassFrameGrabber.matchPattern@12605` | `12605–12956` | `public unsafe double[] matchPattern(string fname, string templateImage, int match_method)` | — | — |

## 7. Decompiler/binary-review boundary

Decompiler warning lines are recorded per method instead of being hidden. Before implementing any warning-bearing authority method, the tranche must resolve the affected predicate/control flow against the pinned captured binary or preserve a fail-closed blocker. Native `CVisionLib` projections must be tested against the captured DLL and templates; they must not be rewritten from apparent decompiler output alone.

## 8. Registry acceptance rules

The registry is accepted only when automated validation proves:

1. every source and binary hash matches the file at the exact recorded path;
2. every method declaration exists at its recorded start line and the end line remains brace-bounded;
3. every source references a known captured binary;
4. all required startup call-graph anchors are present;
5. the exact 19-file live machine corpus remains closed-world and hash-valid;
6. serial-206 configuration values match the recorded XML lines/selectors;
7. no method is promoted from static authority to implemented, controller-validated, or physically validated without separate evidence.

## 9. Explicitly closed false gaps

- Lower `ClassPipette` command/reply bodies: present and line-bounded.
- `InterfaceCAN`, `ClassNovo`, and `ClassNovoCANUSB` routing: present and line-bounded.
- Board and motor primitives: present and line-bounded.
- Route bodies and the selected 29-entry `PositionTable`: present and line-bounded/hash-locked.
- Camera/cover callers, native vision binary, inspection settings, and templates: present/hash-locked.
- Chiller/thermal transport and calibration: present and line-bounded/hash-locked.
- Serial-206 configuration selection: closed-world and hash-locked.

The remaining work is implementation, semantic equivalence testing, controller validation, interlock validation, and separately authorized physical acceptance—not source discovery.
