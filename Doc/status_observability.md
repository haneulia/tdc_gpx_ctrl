# TDC-GPX Status Observability Map

## Currently Exposed in t_tdc_status (SW-visible via CSR)

| Field | Source | Type | Description |
|-------|--------|------|-------------|
| busy | chip_ctrl phase | flag | '1' when any phase active |
| pipeline_overrun | face_seq | flag | shot overrun detected |
| err_fatal | err_handler | sticky | recovery retry exhausted |
| chip_error_mask | top glue | 4-bit | unmasked chip error flags |
| drain_timeout_mask | chip_ctrl | 4-bit sticky | per-chip drain timeout |
| sequence_error_mask | chip_ctrl | 4-bit sticky | per-chip stop_tdc sequence error |
| shot_seq_current | face_seq | 16-bit | current shot sequence number |
| vdma_frame_count | face_seq | 32-bit | VDMA frame counter |
| error_cycle_count | status_agg | 32-bit | accumulated error cycle count |
| shot_drop_count | face_seq | 16-bit | deferred-shot overflow drops |
| frame_abort_count | face_seq | 16-bit | frames discarded by abort |
| err_active | err_handler | flag | FSM recovery in progress |
| err_chip_mask | err_handler | 4-bit | chips under recovery |
| err_cause | err_handler | 3-bit | [0]=HFifoFull [1]=IFifoFull [2]=NotLocked |
| rsp_mismatch_mask | chip_ctrl | 4-bit sticky | bus response tuser mismatch |

## Currently Available but NOT in t_tdc_status

| Signal | Source | Description | Recommendation |
|--------|--------|-------------|----------------|
| o_shot_dropped | cell_builder×8 | shot dropped (no free buffer) | Aggregate to status |
| o_hit_dropped_any | cell_builder×8 | cell hit overflow | Aggregate to status |
| o_slice_timeout | cell_builder×8 | IFIFO2 timeout truncation | Aggregate to status |
| o_cfg_rejected | face_seq | cmd_start rejected (invalid config) | Wire to status |
| o_stop_id_error | decode_pipe×4 | out-of-range stop_id | Already in top glue |
| s_raw_hold_busy | chip_ctrl×4 | raw hold stall | Add counter |
| o_timeout (init) | chip_init×4 | bus response timeout | Aggregate to status |
| o_timeout (reg) | chip_reg×4 | bus response timeout | Aggregate to status |

## Recommended Future Additions

1. `init_timeout_mask` — per-chip init timeout sticky
2. `reg_timeout_mask` — per-chip reg timeout sticky
3. `raw_stall_count` — aggregate raw hold full stall cycles
4. `cfg_rejected_count` — config validation rejection count
5. `unexpected_restart_count` — header_inserter face_start in non-IDLE
6. `late_beat_flushed_count` — face_assembler input FIFO flush on shot_start

## Design Policy

- **Clamp vs Reject**: face_seq now validates config before accepting cmd_start.
  Invalid geometry (active_mask=0, stops_per_chip<2) is rejected with cfg_rejected pulse.
  Downstream modules still have local clamps for defense-in-depth but these should
  not normally fire in the integrated system.

- **Pulse vs Counter vs Sticky**: 
  - Pulses (1-cycle) for events that SW polls or IRQ catches immediately
  - Sticky bits for conditions that must survive until SW reads status
  - Counters for events that may repeat and need accumulation
