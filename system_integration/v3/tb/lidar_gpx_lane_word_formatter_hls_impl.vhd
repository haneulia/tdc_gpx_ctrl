-- ============================================================================
-- 구현 타이밍 점검용 래퍼
--
-- 검증 목적:
--   * 4개 TDC-GPX 칩, 칩당 8 STOP, 최대 7 Return 합성 조건을 고정한다.
--   * Rising/Falling H4 HLS formatter와 최종 AXI4-Stream Word packer를
--     모두 포함하여 실제 배치·배선 후 150/200 MHz 타이밍을 확인한다.
--   * 합성 시 결정되는 출력 폭은 32-bit 또는 64-bit로 각각 검증한다.
--
-- 이 래퍼는 기능 시뮬레이션용 Top이 아니다. 입력과 출력 포트를 의도적으로
-- 보존하여 합성기가 HLS 데이터 경로를 제거하지 못하게 하는 OOC 구현 전용 Top이다.
-- 실행 스크립트: scripts/run_v3_gpx_lane_word_formatter_impl.ps1
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_build_pkg.all;
use work.lidar_gpx_event_pkg.all;
use work.lidar_gpx_data_pkg.all;
use work.lidar_gpx_vdma_pkg.all;
use work.lidar_v3_hls_contract_pkg.all;

entity lidar_gpx_lane_word_formatter_hls_impl is
    generic (
        G_PROC_CLK_MHZ : positive := 150;
        G_OUTPUT_WIDTH : positive := 32
    );
    port (
        i_clk          : in  std_logic;
        i_rst_n        : in  std_logic;
        i_abort        : in  std_logic;
        i_clear_sticky : in  std_logic;

        i_active_version : in unsigned(15 downto 0);
        i_rise_profile   : in gpx_vdma_lane_profile_t;
        i_fall_profile   : in gpx_vdma_lane_profile_t;

        i_rise_cell       : in  gpx_frame_cell_event_t;
        o_rise_cell_ready : out std_logic;
        i_fall_cell       : in  gpx_frame_cell_event_t;
        o_fall_cell_ready : out std_logic;

        i_frame_close_event : in  gpx_frame_close_event_t;
        o_rise_close_ready  : out std_logic;
        o_fall_close_ready  : out std_logic;

        o_m_axis_rise_tdata  : out std_logic_vector(
            G_OUTPUT_WIDTH - 1 downto 0);
        o_m_axis_rise_tkeep  : out std_logic_vector(
            G_OUTPUT_WIDTH / 8 - 1 downto 0);
        o_m_axis_rise_tstrb  : out std_logic_vector(
            G_OUTPUT_WIDTH / 8 - 1 downto 0);
        o_m_axis_rise_tuser  : out std_logic_vector(0 downto 0);
        o_m_axis_rise_tvalid : out std_logic;
        o_m_axis_rise_tlast  : out std_logic;
        i_m_axis_rise_tready : in  std_logic;

        o_m_axis_fall_tdata  : out std_logic_vector(
            G_OUTPUT_WIDTH - 1 downto 0);
        o_m_axis_fall_tkeep  : out std_logic_vector(
            G_OUTPUT_WIDTH / 8 - 1 downto 0);
        o_m_axis_fall_tstrb  : out std_logic_vector(
            G_OUTPUT_WIDTH / 8 - 1 downto 0);
        o_m_axis_fall_tuser  : out std_logic_vector(0 downto 0);
        o_m_axis_fall_tvalid : out std_logic;
        o_m_axis_fall_tlast  : out std_logic;
        i_m_axis_fall_tready : in  std_logic;

        o_rise_footer_emitted : out std_logic;
        o_fall_footer_emitted : out std_logic;
        o_rise_emitted_lines  : out unsigned(16 downto 0);
        o_fall_emitted_lines  : out unsigned(16 downto 0);
        o_rise_line_done      : out std_logic;
        o_fall_line_done      : out std_logic;
        o_rise_frame_done     : out std_logic;
        o_fall_frame_done     : out std_logic;
        o_rise_idle           : out std_logic;
        o_fall_idle           : out std_logic;
        o_rise_fault_pulse    : out lidar_gpx_word_formatter_faults_t;
        o_fall_fault_pulse    : out lidar_gpx_word_formatter_faults_t;
        o_rise_fault_sticky   : out lidar_gpx_word_formatter_faults_t;
        o_fall_fault_sticky   : out lidar_gpx_word_formatter_faults_t
    );
end entity lidar_gpx_lane_word_formatter_hls_impl;

architecture rtl of lidar_gpx_lane_word_formatter_hls_impl is

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

    signal rise_line_word_s       : gpx_vdma_line_word_event_t;
    signal rise_line_word_ready_s : std_logic;
    signal fall_line_word_s       : gpx_vdma_line_word_event_t;
    signal fall_line_word_ready_s : std_logic;
    signal rise_formatter_idle_s  : std_logic;
    signal fall_formatter_idle_s  : std_logic;
    signal rise_packer_idle_s     : std_logic;
    signal fall_packer_idle_s     : std_logic;

begin

    assert G_OUTPUT_WIDTH = 32 or G_OUTPUT_WIDTH = 64
        report "V3-HLS-H4-IMPL-001 output width must be 32 or 64"
        severity failure;

    o_rise_idle <= rise_formatter_idle_s and rise_packer_idle_s;
    o_fall_idle <= fall_formatter_idle_s and fall_packer_idle_s;

    u_rise_formatter : entity work.lidar_gpx_lane_word_formatter_hls_adapter
        generic map (
            G_BUILD_CONFIG => C_BUILD_CONFIG,
            G_LANE_RISE    => true
        )
        port map (
            i_clk => i_clk,
            i_rst_n => i_rst_n,
            i_abort => i_abort,
            i_clear_sticky => i_clear_sticky,
            i_active_profile => i_rise_profile,
            i_active_version => i_active_version,
            i_cell_event => i_rise_cell,
            o_cell_ready => o_rise_cell_ready,
            i_frame_close_event => i_frame_close_event,
            o_frame_close_ready => o_rise_close_ready,
            o_line_word => rise_line_word_s,
            i_line_word_ready => rise_line_word_ready_s,
            o_footer_emitted => o_rise_footer_emitted,
            o_emitted_line_count => o_rise_emitted_lines,
            o_idle => rise_formatter_idle_s,
            o_fault_pulse => o_rise_fault_pulse,
            o_fault_sticky => o_rise_fault_sticky
        );

    u_rise_axis_packer : entity work.lidar_gpx_axis_word_packer
        generic map (G_OUTPUT_WIDTH => G_OUTPUT_WIDTH)
        port map (
            i_clk => i_clk,
            i_rst_n => i_rst_n,
            i_abort => i_abort,
            i_line_word => rise_line_word_s,
            o_line_word_ready => rise_line_word_ready_s,
            o_m_axis_tdata => o_m_axis_rise_tdata,
            o_m_axis_tkeep => o_m_axis_rise_tkeep,
            o_m_axis_tstrb => o_m_axis_rise_tstrb,
            o_m_axis_tuser => o_m_axis_rise_tuser,
            o_m_axis_tvalid => o_m_axis_rise_tvalid,
            o_m_axis_tlast => o_m_axis_rise_tlast,
            i_m_axis_tready => i_m_axis_rise_tready,
            o_line_done => o_rise_line_done,
            o_frame_done => o_rise_frame_done,
            o_idle => rise_packer_idle_s
        );

    u_fall_formatter : entity work.lidar_gpx_lane_word_formatter_hls_adapter
        generic map (
            G_BUILD_CONFIG => C_BUILD_CONFIG,
            G_LANE_RISE    => false
        )
        port map (
            i_clk => i_clk,
            i_rst_n => i_rst_n,
            i_abort => i_abort,
            i_clear_sticky => i_clear_sticky,
            i_active_profile => i_fall_profile,
            i_active_version => i_active_version,
            i_cell_event => i_fall_cell,
            o_cell_ready => o_fall_cell_ready,
            i_frame_close_event => i_frame_close_event,
            o_frame_close_ready => o_fall_close_ready,
            o_line_word => fall_line_word_s,
            i_line_word_ready => fall_line_word_ready_s,
            o_footer_emitted => o_fall_footer_emitted,
            o_emitted_line_count => o_fall_emitted_lines,
            o_idle => fall_formatter_idle_s,
            o_fault_pulse => o_fall_fault_pulse,
            o_fault_sticky => o_fall_fault_sticky
        );

    u_fall_axis_packer : entity work.lidar_gpx_axis_word_packer
        generic map (G_OUTPUT_WIDTH => G_OUTPUT_WIDTH)
        port map (
            i_clk => i_clk,
            i_rst_n => i_rst_n,
            i_abort => i_abort,
            i_line_word => fall_line_word_s,
            o_line_word_ready => fall_line_word_ready_s,
            o_m_axis_tdata => o_m_axis_fall_tdata,
            o_m_axis_tkeep => o_m_axis_fall_tkeep,
            o_m_axis_tstrb => o_m_axis_fall_tstrb,
            o_m_axis_tuser => o_m_axis_fall_tuser,
            o_m_axis_tvalid => o_m_axis_fall_tvalid,
            o_m_axis_tlast => o_m_axis_fall_tlast,
            i_m_axis_tready => i_m_axis_fall_tready,
            o_line_done => o_fall_line_done,
            o_frame_done => o_fall_frame_done,
            o_idle => fall_packer_idle_s
        );

end architecture rtl;
