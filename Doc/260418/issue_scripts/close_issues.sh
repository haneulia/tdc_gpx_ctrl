#!/bin/bash
set -e
GH="/c/Program Files/GitHub CLI/gh.exe"
R="haneulia/tdc_gpx_ctrl"

close_issue() {
  local num="$1"; local sha="$2"; local label="$3"
  "$GH" -R "$R" issue close "$num" --comment "Resolved in [${sha}](https://github.com/${R}/commit/${sha}) — ${label}"
}

# P1 critical
close_issue  1 0c6697c "chip_run: separate 'response arrived' (i_bus_rsp_pending) from 'response consumed' (i_bus_rsp_valid = fire)"
close_issue  2 0c6697c "ST_DRAIN_FLUSH / ST_OVERRUN_FLUSH completion now checks i_bus_rsp_pending='0' instead of i_bus_rsp_valid='0'"
close_issue  3 af735f7 "raw-axis FIFO refactored to array-based shift register, depth raised 2 -> 3"
close_issue  4 af735f7 "depth-3 buffer covers the worst-case control-beat insertion — no beat (raw or control) is dropped"
close_issue  5 97d8d40 "DRAIN-state overrun override now preserves same-cycle fired beat (no drop, accounting consistent)"
close_issue  6 4fc2bca "cell_builder: new ST_C_QUARANTINE absorbs late stale beats after DROP timeout until drain marker or abort"
close_issue  7 e17e3de "config_ctrl: 2-FF synchronizer pipeline with ASYNC_REG on every multi-bit cfg bundle"
close_issue  8 e17e3de "xpm_cdc_async_rst generates s_tdc_aresetn; bus_phy, sk_brsp, chip_ctrl all wired through it"
close_issue  9 3858ebd "PH_RESP_DRAIN hard cap now quarantines (stays in drain) when bus is still active — no stale leak"
close_issue 10 37a14c6 "cmd_arb: 1-depth overlap queue + o_reg_rejected sticky for second-overlap rejection"

# P2 important
close_issue 11 70369a7 "chip_ctrl: new o_run_drain_complete pulse separates internal completion from o_drain_done handshake"
close_issue 12 9642f2f "chip_reg s_err_req_overflow_r sticky exposed via o_err_req_overflow and chip_ctrl.o_err_reg_overflow"
close_issue 13 9642f2f "err_handler s_err_read_timeout_r sticky exposed via o_err_read_timeout"
close_issue 14 1bcdedc "chip_init: busy-state i_cfg_write_req now latched into s_cfg_write_pending_r, consumed on ST_OFF re-entry"
close_issue 15 8cede61 "face_assembler: o_shot_flush_drop sticky records shot_start flushes that dropped non-empty input FIFO data"
close_issue 16 8cede61 "face_assembler: o_shot_overrun_count 8-bit wrap counter for blank-fill invocations"
close_issue 17 9642f2f "cmd_arb: o_reg_zero_mask runtime sticky (previously sim-only assert)"

# P3 maintenance
close_issue 18 219be08 "chip_run: port comment for i_expected_ififo1/2 rewritten to match actual (sampled-at-ST_DRAIN_LATCH) behavior"
close_issue 19 219be08 "face_assembler: o_face_abort comment upgraded to DEPRECATED note (permanently stuck at '0' since Round 4)"
close_issue 20 219be08 "face_seq: s_shot_overrun_r dead latch removed (set/cleared but never read inside the module)"
close_issue 21 219be08 "header_inserter: comment updated to reflect actual pending-latch behavior for non-IDLE face_start"
close_issue 22 219be08 "cell_pipe: concurrent rising_edge asserts moved into a clocked process (standard synchronous form)"
close_issue 23 219be08 "err_handler: i_reg11_* ports upgraded to NAMING DEBT note — actually carry Reg12 reads; rename deferred for backward compat"

echo "--- 23 issues closed ---"
