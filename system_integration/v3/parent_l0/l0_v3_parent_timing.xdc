# TDC-GPX LiDAR V3 Stage-L0 timing/CDC contract.
#
# Clock domains:
#   clk_fpga_0                                      = CSR/AXI-Lite 100 MHz
#   clk_out1_design_1_lidar_ctrl_v3_proc_clk_wiz_0 = Processing 150 MHz
#   clk_fpga_2                                      = TDC bus/acquisition 200 MHz
#
# Do not replace these endpoint constraints with set_clock_groups. XPM FIFOs
# and handshakes own point-to-point max-delay and bus-skew constraints; a
# clock-wide false path would override that physical protection.
#
# This file deliberately contains only XDC-supported constraint commands.
# Endpoint-count assertions and reviewed CDC waivers live in the Parent
# sign-off Tcl, where normal Tcl control flow is supported.

# Explicit 2-FF synchronizers: exclude only the metastability-facing first
# stage. The meta-to-sync stage remains timed by the destination clock.
set l0_v3_meta_d [get_pins -quiet -hier -regexp \
    {^.*/tdc_gpx_lidar_ctrl_v3_0/inst/.*_meta_r_reg.*\/D$}]
set_false_path -to $l0_v3_meta_d

# The physical encoder uses a four-stage shift synchronizer whose first stage
# is vector bit 0 rather than a separately named *_meta_r register.
set l0_encoder_meta_d [get_pins -quiet -hier -regexp \
    {^.*/tdc_gpx_lidar_ctrl_v3_0/inst/.*phy_[abz]_sync_r_reg\[0\]\/D$}]
set_false_path -to $l0_encoder_meta_d

# Coherent configuration payloads are held stable by four-phase request/ACK
# protocols. The 215-bit payload is wider than the range for which Xilinx XPM
# applies set_bus_skew; per-path 5 ns max-delay already bounds every bit before
# the destination's explicit settle/prepare phase.
set l0_cfg_src [get_cells -quiet -hier -regexp \
    {^.*/tdc_gpx_lidar_ctrl_v3_0/inst/u_csr_config/u_config/u_manager/candidate_r_reg.*$}]
set l0_cfg_proc_dst [get_cells -quiet -hier -regexp \
    {^.*/tdc_gpx_lidar_ctrl_v3_0/inst/u_csr_config/u_config/u_proc_gateway/prepared_cfg_r_reg.*$}]
set l0_cfg_tdc_dst [get_cells -quiet -hier -regexp \
    {^.*/tdc_gpx_lidar_ctrl_v3_0/inst/u_csr_config/u_config/u_tdc_gateway/prepared_cfg_r_reg.*$}]
set_max_delay 5.000 -datapath_only -from $l0_cfg_src -to $l0_cfg_proc_dst
set_max_delay 5.000 -datapath_only -from $l0_cfg_src -to $l0_cfg_tdc_dst

# The 16x28-bit GPX register image uses a separate acknowledged activation
# transaction. Its candidate image remains stable until the TDC-domain copy
# is committed. As above, a per-path max-delay is used for this wide payload.
set l0_image_src [get_cells -quiet -hier -regexp \
    {^.*/tdc_gpx_lidar_ctrl_v3_0/inst/u_csr_config/u_gpx_image_transaction/candidate_image_r_reg.*$}]
set l0_image_dst [get_cells -quiet -hier -regexp \
    {^.*/tdc_gpx_lidar_ctrl_v3_0/inst/u_csr_config/gen_tdc_external_apply\.u_gpx_activation/image_r_reg.*$}]
set_max_delay 5.000 -datapath_only -from $l0_image_src -to $l0_image_dst

# Diagnostic mailbox request/response words are registered and held until the
# synchronized toggle completes. The extra destination settle state is the
# functional atomicity guard. These 8-bit and 33-bit payloads receive both a
# flight-time and an inter-bit-skew bound in each clock-domain direction.
set l0_proc_diag_req_src [get_cells -quiet -hier -regexp \
    {^.*/tdc_gpx_lidar_ctrl_v3_0/inst/u_status_snapshot/u_proc_mailbox/request_payload_r_reg.*$}]
set l0_proc_diag_req_dst [get_cells -quiet -hier -regexp \
    {^.*/tdc_gpx_lidar_ctrl_v3_0/inst/u_status_snapshot/u_proc_mailbox/domain_request_r_reg.*$}]
set l0_proc_diag_rsp_src [get_cells -quiet -hier -regexp \
    {^.*/tdc_gpx_lidar_ctrl_v3_0/inst/u_status_snapshot/u_proc_mailbox/domain_response_r_reg.*$}]
set l0_proc_diag_rsp_dst [get_cells -quiet -hier -regexp \
    {^.*/tdc_gpx_lidar_ctrl_v3_0/inst/u_status_snapshot/u_proc_mailbox/source_response_r_reg.*$}]
set l0_tdc_diag_req_src [get_cells -quiet -hier -regexp \
    {^.*/tdc_gpx_lidar_ctrl_v3_0/inst/u_status_snapshot/u_tdc_mailbox/request_payload_r_reg.*$}]
set l0_tdc_diag_req_dst [get_cells -quiet -hier -regexp \
    {^.*/tdc_gpx_lidar_ctrl_v3_0/inst/u_status_snapshot/u_tdc_mailbox/domain_request_r_reg.*$}]
set l0_tdc_diag_rsp_src [get_cells -quiet -hier -regexp \
    {^.*/tdc_gpx_lidar_ctrl_v3_0/inst/u_status_snapshot/u_tdc_mailbox/domain_response_r_reg.*$}]
set l0_tdc_diag_rsp_dst [get_cells -quiet -hier -regexp \
    {^.*/tdc_gpx_lidar_ctrl_v3_0/inst/u_status_snapshot/u_tdc_mailbox/source_response_r_reg.*$}]
set_max_delay 5.000 -datapath_only -from $l0_proc_diag_req_src -to $l0_proc_diag_req_dst
set_bus_skew 5.000 -from $l0_proc_diag_req_src -to $l0_proc_diag_req_dst
set_max_delay 5.000 -datapath_only -from $l0_proc_diag_rsp_src -to $l0_proc_diag_rsp_dst
set_bus_skew 5.000 -from $l0_proc_diag_rsp_src -to $l0_proc_diag_rsp_dst
set_max_delay 5.000 -datapath_only -from $l0_tdc_diag_req_src -to $l0_tdc_diag_req_dst
set_bus_skew 5.000 -from $l0_tdc_diag_req_src -to $l0_tdc_diag_req_dst
set_max_delay 5.000 -datapath_only -from $l0_tdc_diag_rsp_src -to $l0_tdc_diag_rsp_dst
set_bus_skew 5.000 -from $l0_tdc_diag_rsp_src -to $l0_tdc_diag_rsp_dst

# GPX D[27:0] is an asynchronous parallel response bus. The RTL does not use a
# one-cycle timing relationship: RDN is asserted first, then Runtime TDC-GPX
# bus timing (BUS_CLK_DIV/BUS_TICKS) guarantees a board-safe capture window of
# at least 25 ns at this 200 MHz Parent profile before the IOB input FF is
# enabled. Exclude only pad-to-capture paths; the capture-window legality is
# checked by RTL assertions and dedicated bus timing regressions.
set l0_gpx_data_ports [get_ports -quiet {io_tdc_d[*]}]
set l0_gpx_capture_d [get_pins -quiet -hier -regexp \
    {^.*/tdc_gpx_lidar_ctrl_v3_0/inst/.*u_proven_bus_phy/s_d_in_ff_r_reg\[[0-9]+\]\/D$}]
set_false_path -from $l0_gpx_data_ports -to $l0_gpx_capture_d

# Registered external outputs have protocol-cycle timing owners rather than a
# returned device clock. Keep a finite FPGA register-to-pad budget instead of
# declaring those output paths false. The 8 ns TDC budget includes the 7-series
# OBUF/OBUFT delay and still leaves at least 17 ns of pin-stable margin inside the
# separately verified >=25 ns Runtime BUS_CLK_DIV/BUS_TICKS protocol window.
# Laser service pins use one 150 MHz Processing period.
set l0_tdc_output_ports [get_ports -quiet [list \
    {io_tdc_d[*]} {o_tdc_adr[*]} {o_tdc_csn[*]} {o_tdc_rdn[*]} \
    {o_tdc_wrn[*]} {o_tdc_stopdis[*]} {o_tdc_alutrigger[*]} \
    {o_tdc_puresn[*]}]]
set l0_proc_output_ports [get_ports -quiet \
    {o_fire_pulse o_start_tdc o_stop_tdc}]
set_max_delay 8.000 -datapath_only \
    -from [get_clocks -quiet clk_fpga_2] -to $l0_tdc_output_ports
set_max_delay 6.667 -datapath_only \
    -from [get_clocks -quiet -of_objects \
        [get_pins -quiet design_1_lidar_ctrl_v3_i/proc_clk_wiz/inst/mmcm_adv_inst/CLKOUT0]] \
    -to $l0_proc_output_ports
