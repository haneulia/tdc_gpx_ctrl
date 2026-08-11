-- ============================================================================
-- H6-A Parent data subsystem OOC implementation harness
--
-- 합성 최대 부하는 4개 TDC-GPX Chip 모두 Rising/Falling, Chip당 8 STOP,
-- STOP당 최대 7 Return이다. Runtime 설정, 물리 GPX 버스, AXI 출력 및 진단
-- 레코드를 Top port까지 보존하여 HLS/RTL/CDC 경로가 최적화로 제거되지 않게 한다.
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_build_pkg.all;
use work.lidar_config_types_pkg.all;
use work.lidar_event_types_pkg.all;
use work.lidar_gpx_pkg.all;
use work.lidar_gpx_image_pkg.all;
use work.lidar_gpx_event_pkg.all;
use work.lidar_gpx_data_pkg.all;
use work.lidar_gpx_vdma_pkg.all;
use work.lidar_v3_hls_contract_pkg.all;

entity lidar_gpx_hls_parent_data_subsystem_impl is
    generic (
        G_PROC_CLK_MHZ : positive := 150;
        G_TDC_CLK_MHZ  : positive := 200;
        G_OUTPUT_WIDTH : positive := 32
    );
    port (
        i_proc_clk          : in std_logic;
        i_proc_rst_n        : in std_logic;
        i_proc_abort        : in std_logic;
        i_proc_clear_status : in std_logic;
        i_active_valid      : in std_logic;
        i_active_config     : in lidar_active_config_t;
        i_proc_shot         : in shot_start_event_t;
        o_proc_shot_ready   : out std_logic;
        i_proc_stop_tdc     : in std_logic;
        i_face_close_event  : in face_close_event_t;
        o_face_close_ready  : out std_logic;
        i_rise_profile      : in gpx_vdma_lane_profile_t;
        i_fall_profile      : in gpx_vdma_lane_profile_t;

        o_rise_tdata  : out std_logic_vector(G_OUTPUT_WIDTH - 1 downto 0);
        o_rise_tkeep  : out std_logic_vector(G_OUTPUT_WIDTH / 8 - 1 downto 0);
        o_rise_tstrb  : out std_logic_vector(G_OUTPUT_WIDTH / 8 - 1 downto 0);
        o_rise_tuser  : out std_logic_vector(0 downto 0);
        o_rise_tvalid : out std_logic;
        o_rise_tlast  : out std_logic;
        i_rise_tready : in std_logic;
        o_fall_tdata  : out std_logic_vector(G_OUTPUT_WIDTH - 1 downto 0);
        o_fall_tkeep  : out std_logic_vector(G_OUTPUT_WIDTH / 8 - 1 downto 0);
        o_fall_tstrb  : out std_logic_vector(G_OUTPUT_WIDTH / 8 - 1 downto 0);
        o_fall_tuser  : out std_logic_vector(0 downto 0);
        o_fall_tvalid : out std_logic;
        o_fall_tlast  : out std_logic;
        i_fall_tready : in std_logic;

        o_shot_done          : out std_logic;
        o_shot_done_context  : out shot_start_event_t;
        o_frame_output_done  : out std_logic;
        o_proc_idle          : out std_logic;
        o_outstanding_shots  : out unsigned(15 downto 0);
        o_decoder_inflight   : out unsigned(7 downto 0);
        o_rise_emitted_lines : out unsigned(16 downto 0);
        o_fall_emitted_lines : out unsigned(16 downto 0);
        o_context_fault      : out std_logic;

        i_tdc_clk           : in std_logic;
        i_tdc_rst_n         : in std_logic;
        i_register_image    : in gpx_register_image_t;
        i_config_apply      : in std_logic;
        o_config_ready      : out std_logic;
        o_config_done       : out std_logic;
        i_run_enable        : in std_logic;
        i_soft_reset        : in std_logic;
        i_force_reinit      : in std_logic;
        i_tdc_clear_status  : in std_logic;
        o_tdc_safe          : out std_logic;
        o_tdc_shot_complete : out std_logic;
        i_register_read     : in gpx_register_read_request_t;
        o_register_read_ready : out std_logic;
        o_register_read_response : out gpx_register_read_response_t;
        i_register_read_response_ready : in std_logic;
        o_cdc_reset_busy    : out std_logic;

        o_adr        : out gpx_bus_address_array_t;
        o_csn        : out chip_mask_t;
        o_rdn        : out chip_mask_t;
        o_wrn        : out chip_mask_t;
        o_oen        : out chip_mask_t;
        i_d          : in gpx_bus_data_array_t;
        o_d          : out gpx_bus_data_array_t;
        o_d_tri      : out gpx_bus_data_array_t;
        i_ef1        : in chip_mask_t;
        i_ef2        : in chip_mask_t;
        i_lf1        : in chip_mask_t;
        i_lf2        : in chip_mask_t;
        i_irflag     : in chip_mask_t;
        i_errflag    : in chip_mask_t;
        o_stopdis    : out chip_mask_t;
        o_alutrigger : out chip_mask_t;
        o_puresn     : out chip_mask_t;

        o_active_mask   : out chip_mask_t;
        o_terminal_mask : out chip_mask_t;
        o_tdc_status    : out gpx_lane_status_array_t;
        o_tdc_faults    : out gpx_lane_faults_array_t;
        o_shot_drop     : out std_logic;
        o_stop_drop     : out std_logic;
        o_hit_fault     : out gpx_hit_decoder_faults_t;
        o_cell_fault    : out gpx_cell_collector_faults_t;
        o_frame_fault   : out gpx_frame_assembler_faults_t;
        o_rise_formatter_fault : out lidar_gpx_word_formatter_faults_t;
        o_fall_formatter_fault : out lidar_gpx_word_formatter_faults_t
    );
end entity lidar_gpx_hls_parent_data_subsystem_impl;

architecture rtl of lidar_gpx_hls_parent_data_subsystem_impl is
    constant C_BUILD_CONFIG : lidar_build_config_t := (
        proc_clk_mhz           => G_PROC_CLK_MHZ,
        tdc_clk_mhz            => G_TDC_CLK_MHZ,
        stream_clock_mode      => STREAM_CLOCK_ASYNC,
        num_chips              => 4,
        stops_per_chip         => 8,
        max_returns_per_stop   => 7,
        rise_capability_mask   => "1111",
        fall_capability_mask   => "1111",
        output_width           => G_OUTPUT_WIDTH,
        num_faces              => 4,
        enable_echo_receiver   => false,
        enable_echo_simulation => false
    );
begin
    assert G_PROC_CLK_MHZ /= G_TDC_CLK_MHZ
        report "V3-H6-IMPL-001 OOC profiles must exercise asynchronous clocks"
        severity failure;
    assert G_OUTPUT_WIDTH = 32 or G_OUTPUT_WIDTH = 64
        report "V3-H6-IMPL-002 output width must be 32 or 64"
        severity failure;

    u_h6_parent_data : entity work.lidar_gpx_hls_parent_data_subsystem
        generic map (
            G_BUILD_CONFIG => C_BUILD_CONFIG,
            G_OEN_MODE => "DYNAMIC_CONNECTED"
        )
        port map (
            i_proc_clk => i_proc_clk,
            i_proc_rst_n => i_proc_rst_n,
            i_proc_abort => i_proc_abort,
            i_proc_clear_status => i_proc_clear_status,
            i_proc_active_valid => i_active_valid,
            i_proc_active_config => i_active_config,
            i_proc_shot => i_proc_shot,
            o_proc_shot_ready => o_proc_shot_ready,
            i_proc_stop_tdc => i_proc_stop_tdc,
            i_face_close_event => i_face_close_event,
            o_face_close_ready => o_face_close_ready,
            i_rise_active_profile => i_rise_profile,
            i_fall_active_profile => i_fall_profile,
            o_rise_tdata => o_rise_tdata,
            o_rise_tkeep => o_rise_tkeep,
            o_rise_tstrb => o_rise_tstrb,
            o_rise_tuser => o_rise_tuser,
            o_rise_tvalid => o_rise_tvalid,
            o_rise_tlast => o_rise_tlast,
            i_rise_tready => i_rise_tready,
            o_fall_tdata => o_fall_tdata,
            o_fall_tkeep => o_fall_tkeep,
            o_fall_tstrb => o_fall_tstrb,
            o_fall_tuser => o_fall_tuser,
            o_fall_tvalid => o_fall_tvalid,
            o_fall_tlast => o_fall_tlast,
            i_fall_tready => i_fall_tready,
            o_shot_done => o_shot_done,
            o_shot_done_context => o_shot_done_context,
            o_frame_output_done => o_frame_output_done,
            o_processing_idle => open,
            o_axis_output_idle => open,
            o_proc_idle => o_proc_idle,
            o_outstanding_shots => o_outstanding_shots,
            o_decoder_inflight => o_decoder_inflight,
            o_rise_emitted_lines => o_rise_emitted_lines,
            o_fall_emitted_lines => o_fall_emitted_lines,
            o_context_fault_sticky => o_context_fault,
            i_tdc_clk => i_tdc_clk,
            i_tdc_rst_n => i_tdc_rst_n,
            i_tdc_active_valid => i_active_valid,
            i_tdc_active_config => i_active_config,
            i_tdc_register_image => i_register_image,
            i_tdc_config_apply => i_config_apply,
            o_tdc_config_ready => o_config_ready,
            o_tdc_config_done => o_config_done,
            i_tdc_run_enable => i_run_enable,
            i_tdc_soft_reset => i_soft_reset,
            i_tdc_force_reinit => i_force_reinit,
            i_tdc_clear_status => i_tdc_clear_status,
            o_tdc_safe => o_tdc_safe,
            o_tdc_shot_complete => o_tdc_shot_complete,
            i_tdc_register_read => i_register_read,
            o_tdc_register_read_ready => o_register_read_ready,
            o_tdc_register_read_response => o_register_read_response,
            i_tdc_register_read_response_ready =>
                i_register_read_response_ready,
            o_cdc_reset_busy => o_cdc_reset_busy,
            o_adr => o_adr,
            o_csn => o_csn,
            o_rdn => o_rdn,
            o_wrn => o_wrn,
            o_oen => o_oen,
            i_d => i_d,
            o_d => o_d,
            o_d_tri => o_d_tri,
            i_ef1 => i_ef1,
            i_ef2 => i_ef2,
            i_lf1 => i_lf1,
            i_lf2 => i_lf2,
            i_irflag => i_irflag,
            i_errflag => i_errflag,
            o_stopdis => o_stopdis,
            o_alutrigger => o_alutrigger,
            o_puresn => o_puresn,
            o_active_mask => o_active_mask,
            o_terminal_mask => o_terminal_mask,
            o_tdc_status => o_tdc_status,
            o_tdc_faults => o_tdc_faults,
            o_shot_drop_sticky => o_shot_drop,
            o_stop_drop_sticky => o_stop_drop,
            o_hit_fault_sticky => o_hit_fault,
            o_cell_fault_sticky => o_cell_fault,
            o_frame_fault_sticky => o_frame_fault,
            o_rise_formatter_fault_sticky => o_rise_formatter_fault,
            o_fall_formatter_fault_sticky => o_fall_formatter_fault
        );
end architecture rtl;
