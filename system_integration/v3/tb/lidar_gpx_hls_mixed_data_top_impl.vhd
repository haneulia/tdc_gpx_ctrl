-- ============================================================================
-- H5 OOC implementation harness
--
-- 4개 TDC-GPX Chip 모두 Rising/Falling 양 Edge, Chip당 8 STOP, 물리 최대
-- 7 Return 조건을 고정한다. H1~H4 HLS와 retained 32/64-bit AXI packer를
-- 함께 배치·배선하여 Processing 150/200 MHz timing과 자원을 확인한다.
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_build_pkg.all;
use work.lidar_config_types_pkg.all;
use work.lidar_config_reference_pkg.all;
use work.lidar_event_types_pkg.all;
use work.lidar_gpx_event_pkg.all;
use work.lidar_gpx_data_pkg.all;
use work.lidar_gpx_vdma_pkg.all;
use work.lidar_v3_hls_contract_pkg.all;

entity lidar_gpx_hls_mixed_data_top_impl is
    generic (
        G_PROC_CLK_MHZ : positive := 150;
        G_OUTPUT_WIDTH : positive := 32
    );
    port (
        i_clk          : in std_logic;
        i_rst_n        : in std_logic;
        i_abort        : in std_logic;
        i_clear_sticky : in std_logic;
        i_active_version : in std_logic_vector(15 downto 0);
        i_raw_payload  : in std_logic_vector(
            C_GPX_RAW_EVENT_PAYLOAD_WIDTH - 1 downto 0);
        i_raw_valid    : in std_logic;
        o_raw_ready    : out std_logic;
        i_face_close_event : in face_close_event_t;
        o_face_close_ready : out std_logic;
        o_rise_tdata : out std_logic_vector(G_OUTPUT_WIDTH - 1 downto 0);
        o_rise_tkeep : out std_logic_vector(G_OUTPUT_WIDTH / 8 - 1 downto 0);
        o_rise_tstrb : out std_logic_vector(G_OUTPUT_WIDTH / 8 - 1 downto 0);
        o_rise_tuser : out std_logic_vector(0 downto 0);
        o_rise_tvalid : out std_logic;
        o_rise_tlast : out std_logic;
        i_rise_tready : in std_logic;
        o_fall_tdata : out std_logic_vector(G_OUTPUT_WIDTH - 1 downto 0);
        o_fall_tkeep : out std_logic_vector(G_OUTPUT_WIDTH / 8 - 1 downto 0);
        o_fall_tstrb : out std_logic_vector(G_OUTPUT_WIDTH / 8 - 1 downto 0);
        o_fall_tuser : out std_logic_vector(0 downto 0);
        o_fall_tvalid : out std_logic;
        o_fall_tlast : out std_logic;
        i_fall_tready : in std_logic;
        o_idle : out std_logic;
        o_decoder_inflight : out unsigned(7 downto 0);
        o_fault_summary : out std_logic_vector(30 downto 0)
    );
end entity lidar_gpx_hls_mixed_data_top_impl;

architecture rtl of lidar_gpx_hls_mixed_data_top_impl is
    constant C_PLANNED_SHOTS : positive := 1800;
    constant C_BUILD_CONFIG : lidar_build_config_t := (
        proc_clk_mhz           => G_PROC_CLK_MHZ,
        tdc_clk_mhz            => 200,
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

    function fn_active return lidar_active_config_t is
        variable runtime_v : lidar_runtime_config_t :=
            fn_default_runtime_config(C_BUILD_CONFIG);
        variable result : lidar_active_config_t;
    begin
        runtime_v.tdc.active_chip_mask := "1111";
        runtime_v.tdc.falling_enable := '1';
        runtime_v.tdc.max_hits_per_stop := to_unsigned(7, 3);
        result.version := (others => '0');
        result.source := runtime_v;
        result.derived := fn_derive_runtime_config(
            C_BUILD_CONFIG, runtime_v);
        result.derived.columns_per_face := to_unsigned(
            C_PLANNED_SHOTS, 16);
        return result;
    end function fn_active;

    function fn_profile return gpx_vdma_lane_profile_t is
        variable result : gpx_vdma_lane_profile_t :=
            C_GPX_VDMA_LANE_PROFILE_IDLE;
        variable hsize_v : natural;
    begin
        hsize_v := fn_gpx_vdma_shot_hsize_bytes(32, 7, G_OUTPUT_WIDTH);
        result.valid := '1';
        result.enabled := '1';
        result.slot_count := to_unsigned(32, result.slot_count'length);
        result.visible_returns := to_unsigned(
            7, result.visible_returns'length);
        result.cell_word_count := to_unsigned(
            fn_gpx_vdma_cell_word_count(7),
            result.cell_word_count'length);
        result.planned_shots := to_unsigned(
            C_PLANNED_SHOTS, result.planned_shots'length);
        result.raw_line_words := to_unsigned(
            fn_gpx_vdma_shot_raw_hsize_bytes(32, 7) / 4,
            result.raw_line_words'length);
        result.hsize_words := to_unsigned(
            hsize_v / 4, result.hsize_words'length);
        result.hsize_bytes := to_unsigned(
            hsize_v, result.hsize_bytes'length);
        result.footer_lines := to_unsigned(
            fn_gpx_vdma_footer_lines(hsize_v),
            result.footer_lines'length);
        result.vsize_lines := to_unsigned(
            fn_gpx_vdma_vsize_lines(C_PLANNED_SHOTS, hsize_v),
            result.vsize_lines'length);
        result.stride_bytes := to_unsigned(
            fn_gpx_vdma_stride_bytes(32, 7, G_OUTPUT_WIDTH),
            result.stride_bytes'length);
        return result;
    end function fn_profile;

    constant C_ACTIVE_BASE : lidar_active_config_t := fn_active;
    constant C_PROFILE : gpx_vdma_lane_profile_t := fn_profile;
    signal active_s : lidar_active_config_t := C_ACTIVE_BASE;
    signal raw_s : gpx_raw_event_t := C_GPX_RAW_EVENT_IDLE;
    signal hit_fault_s : gpx_hit_decoder_faults_t;
    signal cell_fault_s : gpx_cell_collector_faults_t;
    signal frame_fault_s : gpx_frame_assembler_faults_t;
    signal rise_fault_s : lidar_gpx_word_formatter_faults_t;
    signal fall_fault_s : lidar_gpx_word_formatter_faults_t;
begin
    assert G_OUTPUT_WIDTH = 32 or G_OUTPUT_WIDTH = 64
        report "V3-H5-IMPL-001 output width must be 32 or 64"
        severity failure;

    p_active : process (all)
        variable value_v : lidar_active_config_t;
    begin
        value_v := C_ACTIVE_BASE;
        value_v.version := unsigned(i_active_version);
        active_s <= value_v;
    end process p_active;

    p_raw : process (all)
        variable value_v : gpx_raw_event_t;
    begin
        value_v := fn_unpack_raw_event(i_raw_payload);
        value_v.valid := i_raw_valid;
        raw_s <= value_v;
    end process p_raw;

    -- 모든 진단을 Top port까지 보존해 최대 topology 로직 trim을 막는다.
    o_fault_summary <= rise_fault_s & fall_fault_s &
        frame_fault_s.masked_payload_drop &
        frame_fault_s.column_gap & frame_fault_s.geometry_error &
        frame_fault_s.missing_cell & frame_fault_s.duplicate_terminal &
        frame_fault_s.duplicate_cell & frame_fault_s.unexpected_cell &
        frame_fault_s.context_mismatch &
        cell_fault_s.hit_capacity_drop &
        cell_fault_s.start_number_nonzero &
        cell_fault_s.return_overflow & cell_fault_s.context_mismatch &
        hit_fault_s.slope_role_error & hit_fault_s.stop_index_error &
        hit_fault_s.chip_index_error;

    u_h5 : entity work.lidar_gpx_hls_mixed_data_top
        generic map (G_BUILD_CONFIG => C_BUILD_CONFIG)
        port map (
            i_clk => i_clk,
            i_rst_n => i_rst_n,
            i_abort => i_abort,
            i_clear_sticky => i_clear_sticky,
            i_active_valid => '1',
            i_active_config => active_s,
            i_rise_active_profile => C_PROFILE,
            i_fall_active_profile => C_PROFILE,
            i_raw_event => raw_s,
            o_raw_ready => o_raw_ready,
            i_face_close_event => i_face_close_event,
            o_face_close_ready => o_face_close_ready,
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
            o_shot_done => open,
            o_shot_done_context => open,
            o_frame_output_done => open,
            o_idle => o_idle,
            o_decoder_inflight => o_decoder_inflight,
            o_rise_emitted_lines => open,
            o_fall_emitted_lines => open,
            o_hit_fault_sticky => hit_fault_s,
            o_cell_fault_sticky => cell_fault_s,
            o_frame_fault_sticky => frame_fault_s,
            o_rise_formatter_fault_sticky => rise_fault_s,
            o_fall_formatter_fault_sticky => fall_fault_s
        );
end architecture rtl;
