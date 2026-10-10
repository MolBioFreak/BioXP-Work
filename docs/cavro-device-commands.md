# Original Cavro ADP typed codec contract

Producer: `bioxp.pipette.cavro_commands` (pure; no device, store or application IO).
This is original ADP, **not ADP Detect**. `DeviceCommand` is a closed Pydantic
union discriminated by `command`; `DEVICE_COMMAND_ADAPTER` validates it.
`DeviceProgram` / `DEVICE_PROGRAM_ADAPTER` define typed buffer authoring.

## Integration

- `compile_device_command(payload)` returns exactly `ascii`, `can_command_type`,
  `wait_mode`, `report_kind`, `units`, `persistent`, `warnings`, `requested`.
- `compile_device_program(payload)` returns those fields plus `program`,
  `command_effects`, `buffer_action`, `effects_deferred`.
- Data-only, fresh JSON-compatible exports: `device_command_schema()`,
  `device_program_schema()`, `device_command_catalog()`, `device_command_examples()`.
- `command_effects(payload)` describes authored effects, **never observed effects**.
  Do not treat numeric plunger displacement as delivered liquid or update liquid
  counters blindly from these declarations.
- Application embeds `DeviceCommand` in its existing instruction union and calls
  the existing finite transport with the compiled mapping. No new executor.
- Transport uses frame 6 for reports, frame 1 for actions/settings/controls.
  Documented `T` is emitted without R (frame 1; frame 0 is also documented).
  Existing legacy TR and error-log encoders are untouched.
- `wait_mode=report` never means actuator completion; `ack` only means buffer
  append acknowledgment (including standalone g/G markers, which have no R).
  `control` includes R/X/C/T/reset/H/e and programs with
  halt/endless loop/control transfer: READY alone cannot establish completion.
- Diagnostics use frame 1 with R, `completion` and a non-null diagnostic
  `report_kind`; transport must retain the diagnostic answer as well as lifecycle.
- `persistent=true` means this instruction explicitly requests a documented
  NVRAM write, not that write/readback was verified. Unknown preloaded/stored
  program effects are not inferred from `false`. U commands, b, i, > and s differ
  from working settings v/V/c/L/K/p and nonpersistent o. No automatic i/reset.

## Examples

```python
compile_device_command({"command": "report", "selector": 57})
# ?57, frame 6, report, pressure, psi
compile_device_command({"command": "plunger_absolute", "value": "1.175", "unit": "ul"})
# A1.175,1R; arbitrary absolute displacement, not delivered volume
compile_device_command({"command": "begin_lld", "start_mode": 1})
# B1R; starts at current position, distinct from B0R / legacy BR
compile_device_command({"command": "configuration", "setting": "output_general_purpose"})
# U73R; persistent, no implicit reset
compile_device_program({"mode": "append", "commands": [
    {"command": "loop_start"}, {"command": "delay", "milliseconds": 20},
    {"command": "loop_end", "repetitions": 2}]})
# gM20G2; frame 1, ack, effects deferred; no execute or automatic clear
compile_device_command({"command": "buffer_execute"})  # R
compile_device_command({"command": "buffer_replay"})   # X (not resume)
compile_device_command({"command": "buffer_clear"})    # C
```

## Numeric and program boundaries

Use decimal strings for exact authored spelling (including trailing zeros).
Python Decimal input is preserved as a fixed-point string, never routed through
binary float. `speed_default` explicitly emits V/v/c with omitted parameters;
these restore manufacturer increment-unit defaults, not inferred installed values.
Finite JSON numbers are supported; their shortest round-trip decimal is emitted
without silent clamping, unit conversion or rounding. Structural ASCII-injection,
unknown command/selector, nonfinite values and nonembeddable commands are rejected
only in this new typed surface. Numeric manufacturer range/precision/length and
applicability diagnostics are **nonblocking** and source/page linked. The catalog's
`numeric_domains` exposes numeric bounds/precision as documentary data, not schema
admission limits. Legacy paths
are unchanged. Missing A/P/D action headings do not justify universal bounds.

Programs preserve order and firmware g/G loops without host expansion. Loops must
balance within the typed string; reports and immediate controls remain separate.
A stored string contains typed commands; `stored_program_write` writes sN,...R but
does not execute those stored commands. e transfers control, not a subroutine
return. Host commands after e receive unreachable diagnostics; stored e must be
last. Effects after a host e retain authored fields but are marked unreachable in
that string; the target stored program's effects remain unknown. An empty execute
is R with control semantics, not proof of a finite program. A prior device buffer
is unknown. R can append and execute it; C is never
inserted automatically. H reports READY while halted. M is device timing, not
host sleep; source p40 defines R/T/X differences. Errors erase the executed buffer.

## Coverage boundary

The catalog accounts for **all 29 reviewed families** CF-01..CF-29, preserving
prior ADP IDs, source pages/line ranges, semantics, implementation disposition and
named missing source. CF-06's h setter and CF-29's physical/address/checksum/boot
protocol details are source-unresolved, not fabricated endpoints. Documented
subsets of CF-01/02/07/18/26/27 are implemented while their complete variants,
movement domains, post-LLD trajectory, error tables and CAN common commands remain
unresolved. Reported current-setting units are not inferred from diagnostic d0.

This codec does not certify device firmware, transport, application, UI, stored
method receiving, metrology, physical motion or persistence. Parent owns that
integration and qualification. No IO, driver, application or API changes belong
in this lane.
