# TDC-GPX CSR Register Map (SW driver reference)

Concrete register addresses and bitfields for the AXI4-Lite CSR interface
exposed by `tdc_gpx_config_ctrl` (via `tdc_gpx_csr_pipeline` and
`tdc_gpx_csr_chip`). Fields reflect the post Round 5 / 6 / 7 / 8 state.

Clock: `s_axi_aclk` (PS domain). All register accesses cross to / from the
runtime domains via `xpm_cdc_handshake` — see sections CDC notes below.

## csr_pipeline — pipeline control / status (8 CTL + 8 STAT, 7-bit addr)

### Control registers (R/W)

| Offset | Name       | Fields                                                |
|:------:|------------|-------------------------------------------------------|
| 0x00   | MAIN_CTRL  | `[3:0] active_chip_mask`, `[4] packet_scope`, `[6:5] hit_store_mode`, `[9:7] dist_scale`, `[10] drain_mode`, `[11] pipeline_en`, `[14:12] n_faces`, `[18:15] stops_per_chip`, `[22:19] n_drain_cap`, `[27:23] stopdis_override`, `[31:28] COMMAND` |
| 0x04   | RANGE_COLS | `[15:0] max_range_clks`, `[31:16] cols_per_face`      |
| 0x08–0x1C | reserved  | —                                                  |

`COMMAND` (CTL0[31:28]) bit assignments:
- `[28] cmd_start`
- `[29] cmd_stop`
- `[30] cmd_soft_reset`
- `[31] cmd_cfg_write`
- Pulse-only; hardware edge-detects and clears.

### Status registers (R/O)

| Offset | Name         | Meaning                                               |
|:------:|--------------|-------------------------------------------------------|
| 0x40   | HW_VERSION   | `[31:0]` compile-time constant (default `0x00010000`) |
| 0x44   | HW_CONFIG    | Packed generics: `[3:0] N_CHIPS`, `[7:4] MAX_STOPS_PER_CHIP`, `[11:8] MAX_HITS_PER_STOP`, `[16:12] HIT_SLOT_DATA_WIDTH`, `[24:17] TDATA_WIDTH`, `[27:25] CELL_FMT` |
| 0x48   | MAX_ROWS     | `c_MAX_ROWS_PER_FACE`                                 |
| 0x4C   | CELL_SIZE    | `c_CELL_SIZE_BYTES`                                   |
| 0x50   | MAX_HSIZE    | `MAX_ROWS × beats_per_cell × (TDATA/8) + hdr prefix`  |
| 0x54   | **STATUS**   | See STATUS (STAT5) layout below                       |
| 0x58   | **STATUS_EXT** | See STATUS_EXT (STAT6) layout below — Round 5/6/7   |
| 0x5C   | reserved     | —                                                     |

### STATUS (STAT5 @ 0x54) bit layout

| Bit(s)  | Field               | Type      | Source                              |
|:-------:|---------------------|-----------|-------------------------------------|
| `[0]`   | `busy`              | level     | status_agg OR of every busy FSM     |
| `[1]`   | `pipeline_overrun`  | level     | rise OR fall face_assembler overrun |
| `[2]`   | `err_fatal`         | sticky    | err_handler recovery-retry exhausted|
| `[3]`   | reserved            | —         | —                                   |
| `[7:4]` | `chip_error_mask`   | level     | per-chip chip_error merged          |
| `[11:8]`| `drain_timeout_mask`| sticky    | per-chip chip_run drain timeout     |
| `[15:12]`| `sequence_error_mask` | sticky | per-chip stop_tdc sequence error    |
| `[31:16]`| reserved           | —         | —                                   |

Clear semantic: `chip_error_mask` tracks live status; the sticky masks
(`drain_timeout_mask`, `sequence_error_mask`, `err_fatal`) clear on
`i_rst_n` or SW-initiated `i_err_soft_clear` pulse.

### STATUS_EXT (STAT6 @ 0x58) bit layout — Round 5/6/7

| Bit(s)   | Field                     | Type      | Source                      |
|:--------:|---------------------------|-----------|-----------------------------|
| `[0]`    | `err_read_timeout`        | sticky    | err_handler ST_WAIT_READ watchdog |
| `[1]`    | `reg_rejected`            | sticky    | cmd_arb overlap queue full  |
| `[2]`    | `reg_zero_mask`           | sticky    | cmd_arb zero-mask request   |
| `[3]`    | `shot_flush_drop_rise`    | sticky    | rise face_assembler shot_start flush on non-empty FIFO |
| `[4]`    | `shot_flush_drop_fall`    | sticky    | fall face_assembler shot_start flush on non-empty FIFO |
| `[7:5]`  | reserved                  | —         | —                           |
| `[15:8]` | `shot_overrun_count_rise` | wrap cnt  | rise face_assembler blank-fill count (8-bit) |
| `[23:16]`| `shot_overrun_count_fall` | wrap cnt  | fall face_assembler blank-fill count (8-bit) |
| `[27:24]`| `err_reg_overflow_mask`   | sticky    | per-chip chip_reg 3rd-pulse queue overflow |
| `[31:28]`| `run_drain_complete_mask` | sticky    | per-chip chip_run internal drain-complete (Round 8: cleared on next shot_start) |

Clear semantic notes:
- `err_read_timeout`, `reg_rejected`, `reg_zero_mask`, `err_reg_overflow_mask`: cleared on `i_rst_n` or `i_err_soft_clear` pulse
  (Round 7 B-5 threaded soft_clear through chip_reg).
- `shot_flush_drop_*`: cleared only on `i_rst_n` (no soft_clear path yet).
- `shot_overrun_count_*`: 8-bit wrapping counter; treat as liveness /
  rough event density, NOT an exact event tally (Round 7 B-3 — fast-
  fire events can collapse under in-flight STAT6 handshake).
- `run_drain_complete_mask[i]`: set when chip[i]'s drain completes;
  cleared on the next `shot_start` for that chip. Read as "this shot's
  drain finished" per chip.

### STATUS_EXT counter semantics (read/observe caveats)

STAT6 crosses clock domains via `xpm_cdc_handshake` (32-bit payload).
The handshake auto-retriggers on any source change, so SW sees:
- Every change in the pipeline-wide bits `[7:0]` → visible within one
  handshake latency (~8 dest cycles).
- Counters `[23:8]` — if multiple increments fall inside one in-flight
  handshake, they collapse to the final value. SW should treat counters
  as "≥ N events" indicators, not strict counts.

## csr_chip — per-chip configuration / register access (32 CTL + 32 STAT, 9-bit addr)

### Control registers

| Offset | Name       | Fields |
|:------:|------------|--------|
| 0x04   | BUS_TIMING | `[5:0] bus_clk_div`, `[8:6] bus_ticks`, `[13:10] reg_target_addr`, `[15:14] reg_target_chip`, `[19:16] reg_target_chip_mask`, `[30] reg_read_trigger`, `[31] reg_write_trigger` |
| 0x0C   | START_OFF1 | `[17:0]` |
| 0x10   | CFG_REG7   | `[31:0]` |
| 0x14–0x50 | CFG_IMAGE[0..15] | 16 × 32-bit TDC-GPX chip register mirror |
| 0x54   | SCAN_TIMEOUT | `[15:0] max_scan_clks`, `[18:16] max_hits_cfg` |
| others | reserved / unused (ctl0, ctl2, ctl22–31 owned by csr_pipeline) |

### Status registers

| Offset | Name       | Fields |
|:------:|------------|--------|
| 0x80   | STAT0_CHIP0_RESULT | `[27:0] rdata`, `[31:28] addr` — reg read result for chip 0 |
| 0x84   | STAT1_CHIP1_RESULT | same layout, chip 1 |
| 0x88   | STAT2_CHIP2_RESULT | same layout, chip 2 |
| 0x8C   | STAT3_CHIP3_RESULT | same layout, chip 3 |
| 0x90–0xFC | reserved (STAT4–31) | — |

## Command / status clocking

- **Command path** (`s_axi_aclk` → runtime):
  `CTL writes` → `xpm_cdc_handshake` → AXI-Stream domain → `xpm_cdc_pulse`
  → TDC domain (per-chip). Latency 4–8 dest cycles per CDC stage.
- **Status path** (runtime → `s_axi_aclk`):
  Status signals sticky-latched on their source clock → single CDC hop
  via `xpm_cdc_handshake` for STAT5 / STAT6 (32 bits each) →
  `s_axi_aclk` CSR read.
- **CDC reset**: AXI-Stream `i_axis_aresetn` is synchronized into TDC
  domain via `xpm_cdc_async_rst` (Round 5 #8). All TDC-clocked
  submodules use the synchronized `s_tdc_aresetn`.

## SW usage notes

### Power-up sequence
1. Write config: CFG_IMAGE[0..15], BUS_TIMING, START_OFF1, CFG_REG7.
2. Write MAIN_CTRL with desired pipeline config + `cmd_cfg_write` bit.
3. Poll STATUS.busy until 0 — configuration programmed to chips.
4. Write RANGE_COLS, SCAN_TIMEOUT.
5. Write MAIN_CTRL with `cmd_start` pulse to begin measurement loop.

### Error recovery
- `STATUS.err_fatal` indicates err_handler exhausted retries.
- `STATUS_EXT.err_read_timeout`, `err_reg_overflow_mask` — observability
  for specific failure modes.
- Issue `cmd_soft_reset` pulse to reset hardware without losing config.
- Issue `i_err_soft_clear` (external input) to clear stickies without reset.

### Measurement loop
- `STATUS.busy` goes high during each shot.
- `STATUS_EXT.shot_overrun_count_*` increments if shots arrive too fast.
- `STATUS_EXT.run_drain_complete_mask[i]` set per chip when the current
  shot's drain finishes. Poll to confirm per-chip completion before
  the next shot, or let hardware handle shot pacing via shot_start.

## Version history
- Round 5 #10/#12/#13/#17: pipeline-wide stickies exposed (STAT6 bits 0–2)
- Round 5 #15/#16: per-slope face_assembler stickies (STAT6 bits 3–4, 23:8)
- Round 6 B1: per-chip stickies (STAT6 bits 31:24)
- Round 7 A-3: `run_drain_complete_mask` per-shot clear semantic
- Round 7 B-5: `err_reg_overflow_mask` joins soft_clear policy
- Round 8 C-1: `err_reg_overflow_mask` clear priority matches peers

## Source references

- `tdc_gpx_cfg_pkg.vhd` — bit constants for STAT5, STAT6 (`c_STAT_*`, `c_STAT6_*`), register addresses
- `tdc_gpx_pkg.vhd` — `t_tdc_status` record with all SW-visible fields
- `tdc_gpx_csr_pipeline.vhd` — STAT5 / STAT6 packing + handshake CDC
- `tdc_gpx_csr_chip.vhd` — per-chip CSR + chip-result STAT0–3
- `tdc_gpx_config_ctrl.vhd` — soft_clear, reset sync, status flow
- `tdc_gpx_top.vhd` — `s_status.*` record assignment (wiring source → status)
