# TDC-GPX Local/Unified CSR Mode Closure

## Scope

This result closes the TDC-GPX portion of unified-CSR migration Stage 5. The
build-time generic is:

```vhdl
g_ENABLE_LOCAL_CSR : boolean := true
```

`true` preserves both legacy AXI-Lite banks. `false` removes those CSR owners
and selects the coherent unified adapter. The data path, TDC pins, shot/stop
timing inputs, and two VDMA AXI streams are unchanged.

## Implemented contract

- Local and unified control planes are mutually exclusive RTL generate paths.
- Disabled local AXI outputs remain quiescent and return the `SLVERR` response
  code where a response value is exposed.
- The unified path accepts nine TDC CTL words plus System control/apply words.
- The 16-word GPX image uses an indexed command/data staging window and is
  committed atomically with the remaining configuration.
- Seven TDC STAT words and seven IRQ identities return through one named
  `tdc_unified_csr` interface.
- IP-XACT hides both local AXI interfaces, their clock/reset, and local IRQs in
  unified mode; it hides the unified interface and its clock/reset in local
  mode.
- Local and unified control clocks inherit parent metadata. AXIS and TDC clock
  metadata remain dependent on `g_AXIS_CLK_MHZ` and `g_TDC_CLK_MHZ`.

## Verification gates

| Gate | Result |
|---|---|
| Focused unified adapter to `config_ctrl` | PASS |
| Coherent configuration accepted exactly once | PASS |
| Indexed image Reg6 and controller-owned Reg7 | PASS |
| Disabled local AXI quiescent/`SLVERR` contract | PASS |
| Unified STAT5/STAT6/STAT7-equivalent packing | PASS |
| Unified diagnostic IRQ causes | PASS |
| Default local C06 regression | PASS |
| Packager integrity | `TDC_GPX_IP_PACKAGER_DRC=1` |
| Static package contract | `TDC_GPX_IP_PACKAGE_STATIC_PASS` |
| Block Design mode visibility | `TDC_GPX_IP_PACKAGE_BD_MODE_PASS` |
| OOC local mode, 32-bit, AXIS/TDC 150/200 MHz | PASS, no black boxes |
| OOC unified mode, 32-bit, AXIS/TDC 150/200 MHz | PASS, no black boxes |

The OOC hierarchy checks prove that local mode contains both local CSR owners
and no unified adapter. Unified mode contains the adapter and neither local CSR
owner. The synthesized top-level totals were 16,995 LUT / 21,810 FF in local
mode and 16,305 LUT / 23,675 FF in unified mode. The higher unified FF count is
the cost of coherent wide configuration/status snapshots; it is a later
optimization target, not a functional failure.

## Maintained integration evidence

Both runs use 32-bit output, AXIS 150 MHz, and TDC 200 MHz.

| Scenario | Shots | Rise/Fall beats | Raw 28-bit checks | 17-bit checks per slope | Point margin |
|---|---:|---:|---:|---:|---:|
| Internal encoder, synthetic Echo | 4 | 96 / 96 | 16 | 8 / 8 | 1,513 clocks |
| External encoder, 16 APD, Return-7 | 9 | 828 / 828 | 2,016 | 1,008 / 1,008 | 216 clocks |

Archived evidence:

- `sim_results/vivado_xsim/sessions/260728212300_stage5_local_internal_system_integration_smoke`
- `sim_results/vivado_xsim/sessions/260728212400_stage5_local_return7_system_integration_smoke`

## Package result

The production package at `C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/ip_repo` was
rebuilt from the canonical source manifest. It contains source-level CSR RTL,
the two unified interface-definition XML files, and no XCI or external `../`
source reference. The packaged `tdc_gpx_top.vhd` is byte-identical to the
temporary package used by the dual-mode OOC gate.

Vivado still reports a machine-local corrupted per-user Xilinx Tcl Store and
falls back to the installation store. The package scripts explicitly load the
installation app initializer, and all package gates pass. This environment
warning is not a TDC-GPX IP integrity failure.

## Remaining boundary

This checkpoint does not close Stage 5 for Echo Receiver, Motor Decoder, or
Laser Controller. It also does not claim unified-system functional equivalence.
Those gates precede the single `my_axil_csr32_top` owner in Stage 6 and the
local-versus-unified end-to-end comparison in Stage 7.
