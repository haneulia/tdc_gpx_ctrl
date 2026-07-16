# C08-S14 CSR Geometry / Status Contract Alignment

| Item | Value |
|---|---|
| Baseline HTML | `C08_HDL_HTML_Alignment_260715_Packed_VDMA_Geometry_Closure_Simulator_v013.html` |
| Result HTML | `C08_HDL_HTML_Alignment_260717_CSR_Geometry_Status_Contract_Simulator_v014.html` |
| RTL target | `tdc_gpx_pkg`, `tdc_gpx_csr_pipeline`, `tdc_gpx_cell_pipe`, `tdc_gpx_top` |
| Simulator | Vivado 2025.2.1 xsim |

## 1. Contract decision

`c_CELL_SIZE_BYTES=32` is retained only as the legacy power-of-two internal
cell allocation. It must not be reported as the serialized VDMA cell size.
The published pipeline CSR now uses the packed canonical contract:

```text
STAT3 CELL_SIZE max
  = (ceil(7 hits / 2 hit slots per 32-bit word) + 1 metadata word) * 4 B
  = 20 B

STAT4 MAX_HSIZE
  = 48 B prefix + align16(32 full-mask cells * 20 B)
  = 688 B
```

`STAT4` is a conservative compile-time capacity maximum. Runtime lane geometry
continues to use `o_vdma_hsize_bytes_rise/fall`; for example, a dedicated
16-cell lane at `max_hits=7` is `48 + align16(16 * 20) = 368 B`.

## 2. CSR address and status policy

- Published read-only status window: `0x40..0x5C`.
- Generated-IP native `0x20..0x3C` alias: hidden and reads zero.
- `0x60..0x7F`: reserved and reads zero.
- Writes: only `0x00..0x1C` CTL0..7 reach the generated IP.
- `STAT7[15] masked_slope_drop_any`: set by a hit on a disabled slope lane,
  retained through cmd_stop/abort, cleared by `CTL2[1] err_soft_clear`.

The RTL packing now uses the bit constants in `tdc_gpx_cfg_pkg` for every
STAT6/7 field. A simulation-only assertion rejects any X/U in STAT5/6/7 before
the CDC handshake, preventing the previous exact-zero false PASS pattern.

## 3. HTML changes

Stage 14 keeps the Stage 13 packed VDMA calculation and adds time-ordered D04
rows for:

1. Published CSR status window.
2. STAT3 canonical maximum cell bytes.
3. STAT4 full-mask maximum HSIZE.
4. Runtime rise/fall HSIZE source and distinction from STAT4.
5. Masked-slope sticky set, retain, and clear lifecycle.
6. Shot-reset FIFO guard (`16` RAM entries + `2` XPM FWFT elastic beats,
   reset only at zero interface-outstanding beats).

The Stage 14 integrity verdict includes the exact `32 rows / 20 B / 688 B /
0x40..0x5C / 18 outstanding beats` contract. Inline JavaScript syntax
validation passes.

## 4. Verification evidence

`run_c06_v002_regression.ps1` now includes lane-mask lifecycle and masked-slope
STAT wrapper scenarios. The final release passed all scenarios for
32/64/128-bit, 150/200 MHz split clocks, and bounded backpressure. Exact CSR
reads observed:

```text
0x40 HW_VERSION = 0x00010000
0x48 MAX_ROWS   = 0x00000020
0x4C CELL_SIZE  = 0x00000014
0x50 MAX_HSIZE  = 0x000002B0
0x20 reserved   = 0x00000000
0x60 reserved   = 0x00000000
STAT7 before soft clear = 0x00008000
STAT7 after  soft clear = 0x00000000
```

The output-stage stress probe also filled the XPM AXIS path to 18 outstanding
beats under a shot-boundary stall. The FIFO then backpressured normally and
preserved 88/88 beats, two TLASTs, one SOF, and frame_done.

Final archives:

- `260717_contract_hardening_output_final_shot_bp_regression` (35 artifacts)
- `260717_contract_hardening_release_p0_shot_stall_top_int` (50 artifacts)
