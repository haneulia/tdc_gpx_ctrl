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
| 0x04   | RANGE_COLS | `[15:0] max_range_5ns_ticks`, `[31:16] cols_per_face` |
| 0x08   | AUX_CMD    | `[0] force_reinit` (rising edge), `[1] err_soft_clear` (rising edge), `[31:2] reserved` |
| 0x0C–0x1C | reserved  | —                                                  |

`COMMAND` (CTL0[31:28]) bit assignments:
- `[28] cmd_start`
- `[29] cmd_stop`
- `[30] cmd_soft_reset`
- `[31] cmd_cfg_write`
- Pulse-only; hardware edge-detects and clears.

`AUX_CMD` (CTL2) bit assignments:
- `[0] force_reinit` — SW writes 1→0 to escape `PH_RESP_DRAIN` quarantine
  after externally flushing the bus. Single-shot recovery attempt.
- `[1] err_soft_clear` — SW writes 1→0 to acknowledge per-run error
  history (SOFT-CLEAR category stickies: `err_chip_mask`, `err_cause`,
  `err_reg_overflow`, `chip_reg req_overflow`, `stop_id_error_mask`,
  `masked_slope_drop_any`, etc.). Does NOT clear HISTORICAL-category stickies.
- Both are rising-edge detected in the i_axis_aclk domain.

`max_range_5ns_ticks` and `max_scan_5ns_ticks` are always encoded in the
200 MHz reference timebase: one tick is 5 ns regardless of
`g_TDC_CLK_MHZ` or `g_AXIS_CLK_MHZ`. RTL uses ceiling conversion into each
consuming clock domain, so the represented physical window is never shortened.
Range is consumed in both TDC and AXIS domains; scan timeout is consumed only
in the AXIS domain.

### Status registers (R/O)

| Offset | Name         | Meaning                                               |
|:------:|--------------|-------------------------------------------------------|
| 0x40   | HW_VERSION   | `[31:0]` compile-time constant (default `0x00010000`) |
| 0x44   | HW_CONFIG    | Selected build profile: `[3:0] popcount(g_PRESENT_CHIP_MASK)`, `[7:4] g_MAX_STOPS_PER_CHIP`, `[11:8] g_MAX_HITS_PER_STOP`, `[16:12] HIT_SLOT_DATA_WIDTH`, `[24:17] TDATA_WIDTH`, `[27:25] CELL_FMT`, `[28] HAS_FALLING`, `[31:29] 0` |
| 0x48   | MAX_ROWS     | Build maximum rows: `popcount(g_PRESENT_CHIP_MASK) × g_MAX_STOPS_PER_CHIP` (default `32`) |
| 0x4C   | CELL_SIZE    | Build maximum canonical cell bytes from `g_MAX_HITS_PER_STOP` (default `20 B`) |
| 0x50   | MAX_HSIZE    | Build full-mask packed maximum: `48 + align16(MAX_ROWS × CELL_SIZE)` (default `688 B`) |
| 0x54   | **STATUS**   | See STATUS (STAT5) layout below                       |
| 0x58   | **STATUS_EXT** | See STATUS_EXT (STAT6) layout below — Round 5/6/7   |
| 0x5C   | **STATUS_EXT2** | See STATUS_EXT2 (STAT7) layout below — Round 11 Cat C |

Address contract: only `0x40..0x5C` is the published status window. The
generated IP's native `0x20..0x3C` status placement is hidden by the wrapper,
and `0x60..0x7F` remains reserved. Writes are accepted only for `0x00..0x1C`.
`MAX_ROWS`, `CELL_SIZE`, and `MAX_HSIZE` describe the selected compile-time
build profile, not necessarily the package's absolute ABI capacities. The actual
slope-specific runtime line sizes are exported by
`o_vdma_hsize_bytes_rise/fall` and may be smaller (for example `368 B` for
16 cells at `max_hits=7`).

Build-profile enforcement on CSR writes:
- `active_chip_mask` is ANDed with `g_PRESENT_CHIP_MASK`; an empty result falls
  back to the lowest-index present chip.
- `stops_per_chip` is clamped to `2..g_MAX_STOPS_PER_CHIP`.
- `max_hits_cfg=0` aliases to `g_MAX_HITS_PER_STOP`; larger values clamp to it.

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
| `[5]`    | `rise_hdr_drain_timeout`  | sticky    | rise header drain watchdog escape |
| `[6]`    | `fall_hdr_drain_timeout`  | sticky    | fall header drain watchdog escape |
| `[7]`    | `err_frame_wait_escape`   | sticky    | err_handler frame-done wait escape |
| `[11:8]` | `shot_overrun_count_rise[3:0]` | wrap cnt | rise blank-fill count low nibble |
| `[15:12]`| `shot_flush_drop_mask`     | sticky    | per-chip OR of rise/fall non-empty shot flush |
| `[19:16]`| `shot_overrun_count_fall[3:0]` | wrap cnt | fall blank-fill count low nibble |
| `[23:20]`| `cmd_collision_mask`       | sticky    | per-chip PH_IDLE command collision |
| `[27:24]`| `err_reg_overflow_mask`   | sticky    | per-chip chip_reg 3rd-pulse queue overflow |
| `[31:28]`| `run_drain_complete_mask` | sticky    | per-chip chip_run internal drain-complete (Round 8: cleared on next shot_start) |

Clear semantic notes:
- `err_read_timeout`, `reg_rejected`, `reg_zero_mask`, `err_reg_overflow_mask`: cleared on `i_rst_n` or `i_err_soft_clear` pulse
  (Round 7 B-5 threaded soft_clear through chip_reg).
- `shot_flush_drop_*`, `shot_flush_drop_mask`, and header drain timeout:
  HISTORICAL evidence, cleared only on `i_rst_n`.
- `shot_overrun_count_*`: the CSR exposes the low 4-bit wrapping value; treat as liveness /
  rough event density, NOT an exact event tally (Round 7 B-3 — fast-
  fire events can collapse under in-flight STAT6 handshake).
- `run_drain_complete_mask[i]`: set when chip[i]'s drain completes;
  cleared on the next `shot_start` for that chip. Read as "this shot's
  drain finished" per chip.

### STATUS_EXT2 (STAT7 @ 0x5C) bit layout — Round 11 Category C

| Bit(s)   | Field                           | Type   | Source |
|:--------:|---------------------------------|--------|--------|
| `[3:0]`  | `reg_timeout_mask`              | sticky | cmd_arb per-chip reg transaction timeout |
| `[7:4]`  | `stop_id_error_mask`            | sticky | cell_builder per-chip stop_id out-of-range (rise OR fall) |
| `[10:8]` | `run_timeout_cause_last`        | latched | chip_run last 3-bit cause: 001=raw_busy, 010=ef1_rsp, 011=ef2_rsp, 100=burst_rsp, 101=flush_rsp, 110=overrun_flush, 111=capture_stop_fallback |
| `[14:11]`| `quarantine_escape_mask`        | sticky | per-chip cell_builder quarantine hard-cap escape |
| `[15]`   | `masked_slope_drop_any`         | sticky | any hit addressed a disabled chip/slope lane |
| `[19:16]`| `rise_face_start_collapsed[3:0]`| wrap   | rise non-IDLE face_start coalesce low nibble |
| `[23:20]`| `mono_violation_mask`           | sticky | stop-count monotonic violation per chip |
| `[27:24]`| `fall_face_start_collapsed[3:0]`| wrap   | fall non-IDLE face_start coalesce low nibble |
| `[31:28]`| `init_cfg_coalesced_mask`       | sticky | chip_init cfg-write coalesce per chip |

Clear semantics:
- `reg_timeout_mask`: cleared on normal transaction completion OR on i_err_soft_clear (via cmd_arb's own reset path).
- `stop_id_error_mask`: cleared on i_rst_n or i_err_soft_clear (top-level sticky aggregate).
- `masked_slope_drop_any`: survives cmd_stop/abort for post-run read and
  clears on i_rst_n or `i_err_soft_clear`.
- `quarantine_escape_mask`, `mono_violation_mask`, and
  `init_cfg_coalesced_mask`: HISTORICAL, hard-reset clear.
- `run_timeout_cause_last`: latches the most-recent cause on any chip's run_timeout pulse; never auto-cleared.
- Face_start collapsed counters: CSR exposes each low nibble; not an exact event tally.

### STATUS_EXT counter semantics (read/observe caveats)

STAT6 crosses clock domains via `xpm_cdc_handshake` (32-bit payload).
The handshake auto-retriggers on any source change, so SW sees:
- Every change in the pipeline-wide bits `[7:0]` → visible within one
  handshake latency (~8 dest cycles).
- Counter fields — if multiple increments fall inside one in-flight
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
| 0x54   | SCAN_TIMEOUT | `[15:0] max_scan_5ns_ticks`, `[18:16] max_hits_cfg`, `[19] falling_enable`; reset `0x00080000` |
| others | reserved / unused (ctl0, ctl2, ctl22–31 owned by csr_pipeline) |

`max_scan_5ns_ticks` uses the same fixed timebase as range. RTL computes
`ceil(max_scan_5ns_ticks * g_AXIS_CLK_MHZ / 200)` before `face_assembler`.
A value of zero disables the programmable deadline but is not an infinite
wait: `face_assembler` still forces blank completion at its 16-bit hard cap
(`0xFFFF`), approximately 1.31 ms / 655 us / 524 us / 437 us / 328 us at
50 / 100 / 125 / 150 / 200 MHz respectively.

`falling_enable=1` applies the compile-time `g_RISE_CHIP_MASK` and
`g_FALL_CHIP_MASK`. `falling_enable=0` routes every active/present chip to the
Rise lane and holds the Fall lane idle. A build with `g_FALL_CHIP_MASK=0000`
reports `HW_CONFIG.HAS_FALLING=0`; in that build the Fall processing chain is
removed at elaboration and software must leave `falling_enable=0`. In a
Fall-capable build, clearing this bit only idles the existing Fall hardware.

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
4. Write RANGE_COLS and SCAN_TIMEOUT, including `falling_enable` according to
   the static capability reported by `HW_CONFIG[28]`.
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
