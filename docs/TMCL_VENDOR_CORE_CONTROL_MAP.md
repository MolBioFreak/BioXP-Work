# BioXP TMCL vendor core control map

This note captures the remaining non-standard BioXP TMCL control surface that matters for operator-facing reverse engineering and low-level trace decoding.

Source files used:
- `src/bioxp/usb_driver.py`
- `decompiled_src_can/ClassCanLib/ClassMotor.cs`
- `decompiled_src_can/ClassCanLib/ClassIOControl.cs`
- `decompiled_src_can/ClassCanLib/ClassThermalControl.cs`
- `decompiled_src_can/ClassCanLib/ClassThermalBoard.cs`
- `decompiled_src_can/ClassCanLib/ClassChillerBoard.cs`
- `decompiled_src_can/ClassCanLib/ClassBaseBoard.cs`

## Board IDs
- `0x04` = head board
- `0x05` = deck board
- `0x06` = thermal controller
- `0x07` = chiller board

## Core vendor / product-specific commands

### Board lifecycle
- `cmd 64` = board lifecycle toggle
  - OEM `activateBoard()` uses request pattern `cmd=64 type=0 motor=0 value=1`
  - OEM `deactivateBoard()` reuses the same command with `value=0`
  - chiller deactivation appears to tolerate an `Invalid command` status in OEM code while still treating the board as deinitialized
- `cmd 173` = query firmware version
  - request pattern: `cmd=173 type=0 motor=0 value=0`
  - reply returns firmware bytes in the 4-byte value field

### Deck IO / latch / 24V / LEDs
- `cmd 14 type 2` on deck board = latch solenoid write
  - OEM `ClassIOControl.setIOPort(2, value)` and Python `latch_oem()`
- `cmd 15 type 0` on deck board = 24V-present query
  - OEM `query24VSensor()`
  - non-zero means 24V dropped / absent
- `cmd 15 type 1` on deck board = door sensor query
- `cmd 15 type 2` on deck board = solenoid state query
- `cmd 15 type 3` on deck board = latch sensor query
- `cmd 14 type 0/1` with `motor=2` on deck board = ESM 24V relay control
  - Python reverse-engineered path for deck/head motor power relays
- `cmd 50` = LED intensity write
  - OEM `ClassIOControl.setLED()`
  - `motor` is LED mask/channel, not a motion axis

### Motion / motor control
Standard TMCL motion commands are still used for the gantry and door axes:
- `cmd 1` = ROR
- `cmd 2` = ROL
- `cmd 3` = MST
- `cmd 4` = MVP
- `cmd 5` = SAP
- `cmd 6` = GAP
- `cmd 13` = RFS

Important BioXP/OEM motion details:
- GAP param `1` = actual position
- GAP param `3` = actual speed
- GAP params `9/10` = left/right switch states
- GAP/SAP param `138` = ramp mode / stop query-related config in OEM motor code
- GAP/SAP param `153` = RDIV
- GAP/SAP param `154` = PDIV
- GAP/SAP param `205` = stall-guard threshold used by OEM code

Vendor query used immediately before motion:
- `cmd 138 type 0`
  - OEM `ClassMotor.queryMotorStop()`
  - Python `motor_query_motor_stop()`
  - request is usually scoped either by `motor=<axis>` or by a bitmask in the value field

### Thermal controller (`board 0x06`)

High-level vendor commands:
- `cmd 140` = set target temperature
  - `motor=0` nest / TC target
  - `motor=1` lid target
  - `motor=2` pedestal target
  - value is signed millidegrees C
- `cmd 141` = set heatsink fan speed
  - value is raw 0..255
- `cmd 143` = read temperature sensor
  - Python reverse-engineered path uses axis `2` for pedestal temp reads
  - reply value is signed millidegrees C
- `cmd 144` = set PWM override
  - `motor=0` nest, `motor=1` lid
  - value is percent 0..100
- `cmd 153` = dump thermal history
  - OEM `dumpTCHistory()`

Thermal GP / GGP-SGP parameters used by OEM and Python:
- `4` = current temperature (millideg C)
  - OEM `readTemperature()` / `readLidTemperature()` uses `cmd 10 type 4 bank {0|1}`
- `7` = heat ramp (mC/s)
- `8` = cool ramp (mC/s)
- `9` = proportional gain x1000
- `10` = integral gain x1000
- `11` = derivative gain x1000
- `12` = feed-forward gain x1000
- `13` = TEC current (mA scaled by 1000 in software)
- `14` = current reference
- `19` = temperature gain raw calibration value
- `20` = temperature offset raw calibration value
- `21` = fan speed raw
  - confirmed by OEM `readTCFanSpeed()` / Python fan verify path
- `22` = temperature reference / target (millideg C)
  - confirmed by OEM target setters and Python verify path
- `23` = PWM override percent
  - confirmed by OEM `queryPWM()` and Python PWM verify path

### Chiller board (`board 0x07`)

Control-bank writes:
- `cmd 140` = set chiller target temperature
  - `motor=0` reagent chiller controller
  - `motor=1` output chiller controller
  - value is signed millidegrees C
- `cmd 141` = set chiller fan speed
  - same controller-bank mapping as above
- `cmd 144` = set PWM override
  - same controller-bank mapping as above

Sensor reads:
- `cmd 143` reply value is signed millidegrees C
- reverse-engineered sensor axis map used by `usb_driver.py`:
  - `0` = reagent chiller temp
  - `1` = reagent pedestal temp
  - `3` = output chiller temp
  - `4` = output pedestal temp

Chiller GP params confirmed from OEM + Python:
- `2` = max heating current (mA x1000 in software)
- `3` = max cooling current (mA x1000 in software)
- `4` = current temperature (millideg C)
- `7` = heat ramp (mC/s)
- `8` = cool ramp (mC/s)
- `9` = proportional gain x1000
- `10` = integral gain x1000
- `11` = derivative gain x1000
- `12` = feed-forward gain x1000
- `13` = TEC current
- `14` = current reference
- `19` = temperature gain raw calibration value
- `20` = temperature offset raw calibration value
- `21` = fan speed raw
- `22` = temperature reference / target (millideg C)
- `23` = PWM override percent

Useful extra chiller-bank finding:
- gain/offset calibration accesses are multiplexed beyond banks `0/1`
- OEM `ClassChillerBoard` routes params `19/20` for calibration banks `0/3/6/7` back onto controller instances `0` or `1`
- so bank semantics for calibration are broader than the simple controller-bank map used for setpoint/fan/PWM writes

## Current decoder coverage
`/home/dalab/Desktop/bioxp_re/scripts/decode_tmcl_frame.py` now labels:
- commands `138`, `140`, `141`, `143`, `144`, `153`
- thermal/chiller GP param `4` as current temperature
- thermal/chiller GP params `19/20` as gain/offset calibration rather than PWM
- board-specific request interpretations for thermal/chiller target banks and sensor axes
- temperature scaling for successful `cmd 143` replies

## Remaining ambiguity / caution
The chiller board reuses the `motor` field differently across commands:
- for setpoint/fan/PWM writes, `motor` behaves like controller bank `0/1`
- for temperature reads, `motor` behaves like a sensor axis (`0/1/3/4`)

So decoders and tooling must interpret the `motor` field in command context, not as a single global axis map.

One OEM anomaly to treat carefully:
- `ClassThermalControl.readNestTemp(int nest)` decompiles to `readGP(143, nest)`, which looks inconsistent with the rest of the thermal path
- the higher-level `ClassThermalBoard.readNestTemp(int nest)` calls `readThermalTemperature(nest)` instead, i.e. real `cmd 143`
- until packet traces prove otherwise, treat the `readGP(143, nest)` helper as likely dead or misleading decompiler output rather than a confirmed alternate protocol
