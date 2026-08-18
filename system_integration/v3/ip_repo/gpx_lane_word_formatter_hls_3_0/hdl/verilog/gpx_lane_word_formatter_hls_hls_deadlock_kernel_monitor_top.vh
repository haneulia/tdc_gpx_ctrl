
wire kernel_monitor_reset;
wire kernel_monitor_clock;
wire kernel_monitor_report;
assign kernel_monitor_reset = ~ap_rst_n;
assign kernel_monitor_clock = ap_clk;
assign kernel_monitor_report = 1'b0;
wire [5:0] axis_block_sigs;
wire [6:0] inst_idle_sigs;
wire [0:0] inst_block_sigs;
wire kernel_block;

assign axis_block_sigs[0] = ~ordered_cell_or_face_close_in_TDATA_blk_n;
assign axis_block_sigs[1] = ~formatter_control_out_TDATA_blk_n;
assign axis_block_sigs[2] = ~grp_p_anonymous_namespace_emit_missing_shot_line_fu_900.grp_p_anonymous_namespace_emit_missing_shot_line_Pipeline_VITIS_LOOP_474_1_fu_68.canonical_line_word_out_TDATA_blk_n;
assign axis_block_sigs[3] = ~grp_p_anonymous_namespace_emit_ordered_cell_fu_911.grp_p_anonymous_namespace_emit_real_shot_metadata_fu_86.canonical_line_word_out_TDATA_blk_n;
assign axis_block_sigs[4] = ~grp_p_anonymous_namespace_emit_ordered_cell_fu_911.grp_p_anonymous_namespace_emit_ordered_cell_Pipeline_VITIS_LOOP_639_1_fu_98.canonical_line_word_out_TDATA_blk_n;
assign axis_block_sigs[5] = ~grp_p_anonymous_namespace_emit_footer_line_fu_921.canonical_line_word_out_TDATA_blk_n;

assign inst_block_sigs[0] = 1'b0;

assign inst_idle_sigs[0] = 1'b0;
assign inst_idle_sigs[1] = grp_p_anonymous_namespace_emit_missing_shot_line_fu_900.ap_idle;
assign inst_idle_sigs[2] = grp_p_anonymous_namespace_emit_missing_shot_line_fu_900.grp_p_anonymous_namespace_emit_missing_shot_line_Pipeline_VITIS_LOOP_474_1_fu_68.ap_idle;
assign inst_idle_sigs[3] = grp_p_anonymous_namespace_emit_ordered_cell_fu_911.ap_idle;
assign inst_idle_sigs[4] = grp_p_anonymous_namespace_emit_ordered_cell_fu_911.grp_p_anonymous_namespace_emit_real_shot_metadata_fu_86.ap_idle;
assign inst_idle_sigs[5] = grp_p_anonymous_namespace_emit_ordered_cell_fu_911.grp_p_anonymous_namespace_emit_ordered_cell_Pipeline_VITIS_LOOP_639_1_fu_98.ap_idle;
assign inst_idle_sigs[6] = grp_p_anonymous_namespace_emit_footer_line_fu_921.ap_idle;

gpx_lane_word_formatter_hls_hls_deadlock_idx0_monitor gpx_lane_word_formatter_hls_hls_deadlock_idx0_monitor_U (
    .clock(kernel_monitor_clock),
    .reset(kernel_monitor_reset),
    .axis_block_sigs(axis_block_sigs),
    .inst_idle_sigs(inst_idle_sigs),
    .inst_block_sigs(inst_block_sigs),
    .block(kernel_block)
);


always @ (kernel_block or kernel_monitor_reset) begin
    if (kernel_block == 1'b1 && kernel_monitor_reset == 1'b0) begin
        find_kernel_block = 1'b1;
    end
    else begin
        find_kernel_block = 1'b0;
    end
end
