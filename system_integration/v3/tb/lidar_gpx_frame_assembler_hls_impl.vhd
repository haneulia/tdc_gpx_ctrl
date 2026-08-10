-- ============================================================================
-- 테스트 자산 목적:
--   H3 HLS Frame assembler와 RTL Adapter의 최대 4-Chip 양 Edge 구현
--   자원 및 150/200 MHz 실제 배치배선 timing을 확인한다.
-- 관련 RTL: lidar_gpx_frame_lane_assembler_hls_adapter.
-- 실행 회귀: scripts/run_v3_gpx_frame_assembler_impl.ps1
-- 유지보수 주의: 포트를 제거하면 합성기가 Cell RAM/FIFO를 trim할 수 있다.
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_build_pkg.all;
use work.lidar_event_types_pkg.all;
use work.lidar_gpx_event_pkg.all;
use work.lidar_gpx_data_pkg.all;

entity lidar_gpx_frame_assembler_hls_impl is
    generic (
        G_PROC_CLK_MHZ : positive := 150
    );
    port (
        i_clk          : in  std_logic;
        i_rst_n        : in  std_logic;
        i_abort        : in  std_logic;
        i_clear_sticky : in  std_logic;
        i_active_version   : in unsigned(15 downto 0);
        i_active_rise_mask : in chip_mask_t;
        i_active_fall_mask : in chip_mask_t;
        i_columns_per_face : in unsigned(15 downto 0);
        i_cell_event : in  gpx_cell_event_t;
        o_cell_ready : out std_logic;
        i_face_close_event : in face_close_event_t;
        o_face_close_ready : out std_logic;
        o_rise_event : out gpx_frame_cell_event_t;
        i_rise_ready : in  std_logic;
        o_fall_event : out gpx_frame_cell_event_t;
        i_fall_ready : in  std_logic;
        o_frame_close_event : out gpx_frame_close_event_t;
        i_frame_close_ready : in std_logic;
        o_rise_line_done : out std_logic;
        o_fall_line_done : out std_logic;
        o_shot_done      : out std_logic;
        o_shot_done_context : out shot_start_event_t;
        o_idle           : out std_logic;
        o_fault_pulse  : out gpx_frame_assembler_faults_t;
        o_fault_sticky : out gpx_frame_assembler_faults_t
    );
end entity lidar_gpx_frame_assembler_hls_impl;

architecture rtl of lidar_gpx_frame_assembler_hls_impl is
    constant C_BUILD_CONFIG : lidar_build_config_t := (
        proc_clk_mhz           => G_PROC_CLK_MHZ,
        tdc_clk_mhz            => 200,
        stream_clock_mode      => STREAM_CLOCK_ASYNC,
        num_chips              => 4,
        stops_per_chip         => 8,
        max_returns_per_stop   => 7,
        rise_capability_mask   => "1111",
        fall_capability_mask   => "1111",
        output_width           => 32,
        num_faces              => 4,
        enable_echo_receiver   => false,
        enable_echo_simulation => false
    );
begin
    u_assembler : entity work.lidar_gpx_frame_lane_assembler_hls_adapter
        generic map (G_BUILD_CONFIG => C_BUILD_CONFIG)
        port map (
            i_clk => i_clk,
            i_rst_n => i_rst_n,
            i_abort => i_abort,
            i_clear_sticky => i_clear_sticky,
            i_active_version => i_active_version,
            i_active_rise_mask => i_active_rise_mask,
            i_active_fall_mask => i_active_fall_mask,
            i_columns_per_face => i_columns_per_face,
            i_cell_event => i_cell_event,
            o_cell_ready => o_cell_ready,
            i_face_close_event => i_face_close_event,
            o_face_close_ready => o_face_close_ready,
            o_rise_event => o_rise_event,
            i_rise_ready => i_rise_ready,
            o_fall_event => o_fall_event,
            i_fall_ready => i_fall_ready,
            o_frame_close_event => o_frame_close_event,
            i_frame_close_ready => i_frame_close_ready,
            o_rise_line_done => o_rise_line_done,
            o_fall_line_done => o_fall_line_done,
            o_shot_done => o_shot_done,
            o_shot_done_context => o_shot_done_context,
            o_idle => o_idle,
            o_fault_pulse => o_fault_pulse,
            o_fault_sticky => o_fault_sticky
        );
end architecture rtl;
