# C09 CDC source-register hardening checkpoint

## 1. Scope

This checkpoint removes all post-synthesis `CDC-10` findings from the
`tdc_gpx_top` OOC baseline while preserving behavior across the existing C06
integration suite.

Reference configuration:

- Device: `xc7z020clg484-2`
- `g_OUTPUT_WIDTH = 32`
- `g_AXIS_CLK_MHZ = 150`
- `g_TDC_CLK_MHZ = 200`
- slope topology: `DEDICATED_2X2`
- stream mode: `ASYNC`

## 2. Changes

| Signal class | Previous crossing | Updated crossing | Reason |
|---|---|---|---|
| CSR handshake-idle reductions | combinational reduction into 2-FF synchronizer | AXI-Lite source-domain register into 2-FF synchronizer | Remove combinational CDC source and make the sampled level explicit |
| Per-chip `busy` and raw-overflow sticky | `xpm_cdc_single`, unregistered source | `xpm_cdc_single`, `SRC_INPUT_REG=1` | Register the source inside the XPM primitive |
| Command-collision vector sticky | cross-chip combinational OR into 2-FF synchronizer | TDC source-domain registered OR into 2-FF synchronizer | Preserve sticky-level semantics and remove the combinational CDC source |
| Drain-timeout and sequence-error event | level synchronizer sampling a one-TDC-clock pulse | `xpm_cdc_pulse` | Prevent pulse loss when `g_AXIS_CLK_MHZ < g_TDC_CLK_MHZ` |

The last row is a functional correction, not only a reporting cleanup. Both
events are generated as one-cycle pulses in `tdc_gpx_chip_ctrl`; a level
synchronizer could miss them at 50/100/125/150 MHz AXIS rates when the TDC
clock is 200 MHz. The XPM toggle-based pulse crossing is appropriate because
each event can fire at most once per chip transaction/shot and therefore has
ample spacing relative to the destination synchronization latency.

## 3. Verification

Functional regression:

- Command: `scripts/run_c06_v002_regression.ps1 -Stamp 260717_cdc10_src_reg`
- Result: all C06 scenarios PASS
- Covered widths: 32/64/128 bits
- Covered split clock case: AXIS 150 MHz / TDC 200 MHz
- Covered behaviors: range ticks, face sequencing, status aggregation, lane
  masks, top integration, bounded backpressure, and STAT sticky/soft-clear
- Archive: `sim_results/vivado_xsim/sessions/260717_cdc10_src_reg_c06_v002_regression`

OOC synthesis/CDC check:

- Session: `signoff_results/sessions/260717_cdc10_check_w32_a150_t200_dedicated_2x2_synth`
- Vivado: 2025.2.1
- Black boxes: 0
- Unconstrained internal endpoints: 0

## 4. Before/after result

| Metric | Baseline | Updated | Judgment |
|---|---:|---:|---|
| `CDC-10` Critical | 14 | 0 | Closed |
| `TIMING-10` missing synchronizer property | 1 | 0 | Closed |
| `CDC-4` Critical | 4 | 4 | Open: register-read bundled data crossing |
| `CDC-6` Warning | 23 | 24 | Expected XPM-recognized multi-bit crossing; inspect in final waiver audit |
| `CDC-15` Warning | 1567 | 1567 | Unchanged XPM handshake structures |
| Total LUT | 28,521 | 28,542 | +21 (+0.07%) |
| Total FF | 43,536 | 43,573 | +37 (+0.08%) |
| AXIS WNS | +1.074 ns | +1.074 ns | No regression |
| TDC WNS | -1.765 ns | -1.765 ns | Unchanged; raw FIFO path remains open |
| TDC failing endpoints | 690 | 690 | Unchanged |

## 5. Checkpoint judgment

The CDC source-register direction is correct and verified. This checkpoint is
safe to retain as an independent commit. It does not constitute final sign-off:

1. Replace the four `CDC-4` per-chip register-read bundled-data crossings with
   an atomic data-plus-valid transfer.
2. Restructure the TDC-domain raw FIFO path that currently misses 200 MHz by
   1.765 ns.
3. Re-run the clock matrix, including the 50 MHz AXIS / 200 MHz TDC corner,
   after the remaining CDC and timing changes.

