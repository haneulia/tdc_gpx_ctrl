# TDC-GPX Unified CSR Adapter

## Boundary

`tdc_gpx_unified_csr_adapter` replaces the two local CSR banks only at the
configuration/status boundary. It does not enter the 150 MHz AXIS data path or
the 200 MHz GPX bus/readout path. `shot_start` and `stop_tdc` remain direct from
Laser Controller, and the existing `config_ctrl` AXIS-to-TDC CDC remains the
owner of physical-chip timing.

## Reduced register profile

The adapter consumes nine unified CTLs:

- four dedicated chip settings;
- three pipeline settings;
- one indexed image command and one image data word.

The 16 x 32-bit image is still present internally. Software stages one word at
a time and receives an 8-bit image-write acknowledgement in STAT27[31:24]. A
changed common `CFG_EPOCH` moves the complete image and all TDC settings in one
755-bit CDC snapshot. It is a low-rate configuration transfer, not a datapath
bus, so the width does not add per-hit or per-beat arithmetic.

All software commands share CTL25's opcode and 8-bit command epoch. This
one-deep serialized command path replaces START/STOP/recovery/register-access
edge bits that could otherwise collide. STAT27[23:16] acknowledges the emitted
command. Invalid opcode 7 is consumed, rejected, and never reaches downstream
logic.

## Compatibility rules

- Requested chip masks are intersected with the synthesized chip mask; an
  empty result selects the first present chip.
- Stops per chip, columns, GPX bus divider/ticks, and maximum Return count use
  the same clamps as the local CSR implementation.
- `RESET_EPOCH` generates one soft-reset pulse and preserves accepted config.
- A config epoch waits while `i_cfg_ready=0`; active config and image change
  together before the one-cycle `o_cmd_cfg_write` pulse.
- A command waits while `i_cmd_ready=0`; a second command cannot overwrite the
  one pending slot.

## Status and IRQ

STAT23..26 retain one address+28-bit result per chip. STAT27 retains legacy
pipeline status in its low 16 bits and adds command/image acknowledgement.
STAT28 and STAT29 preserve legacy STATUS_EXT/STATUS_EXT2 packing exactly.

Unified IRQ21..27 report register done, pipeline fault, chip error, timeout,
sequence/protocol error, configuration/image rejection, and command rejection.
Sticky levels are edge-detected independently so one cause cannot mask another.

## Gate

Run `scripts/run_tdc_gpx_unified_adapter_regression.ps1`. The focused test uses
a 100 MHz unified CSR clock and 150 MHz AXIS clock, stages all 16 image words,
checks deferred atomic apply, all six commands, four independent GPX results,
legacy status packing, and all seven interrupt identities. The maintained
150 MHz AXIS / 200 MHz TDC integration regression remains the Stage 4 exit gate.

Stage 4 closed in commit `c1b7130` with all three gates passing:

- `TDC_GPX_UNIFIED_ADAPTER_REGRESSION_PASS`;
- internal encoder plus synthetic Echo: 4 shots, 96 rise/96 fall beats,
  1,513 processing clocks of point margin;
- external encoder plus physical 16-channel Return-7 Echo: 9 shots, 828
  rise/828 fall beats, 2,016 raw 28-bit I-Mode comparisons, and 216 processing
  clocks of point margin.

The integration harness uses Echo Receiver package revision 7's indexed delay
protocol. Direct writes to the retired Echo CTL1..16 delay map are not backward
compatible and must not be used by new software or testbenches.
