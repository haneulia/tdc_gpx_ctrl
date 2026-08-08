-- ============================================================================
-- 테스트 자산 목적: VDMA profile 계산과 Face Footer 생성 경로의 구현 timing을 점검한다.
-- 핵심 검증 계약: sequential geometry 계산과 Footer 출력이 목표 clock에서 구현된다.
-- 관련 RTL: lidar_gpx_vdma_profile_manager, lidar_gpx_face_footer_builder.
-- 실행 회귀: scripts/run_v2_gpx_face_footer.ps1
-- 유지보수 주의: field/geometry 기능은 대응 두 단위 TB가 소유한다.
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_build_pkg.all;
use work.lidar_event_types_pkg.all;
use work.lidar_gpx_data_pkg.all;
use work.lidar_gpx_vdma_pkg.all;

-- OOC implementation wrapper for the complete J7/J8 registered path.
entity lidar_gpx_face_footer_builder_impl is
    generic (
        G_CLK_MHZ      : positive := 150;
        G_OUTPUT_WIDTH : positive := 32
    );
    port (
        i_clk   : in std_logic;
        i_rst_n : in std_logic;
        i_abort : in std_logic;

        i_profile_request_valid : in std_logic;
        o_profile_request_ready : out std_logic;
        i_lane_chip_mask        : in chip_mask_t;
        i_visible_returns       : in unsigned(2 downto 0);
        i_planned_shots         : in shot_index_t;
        o_profile_rejected      : out std_logic;
        i_profile_activate_valid : in std_logic;
        o_profile_activate_ready : out std_logic;
        i_datapath_idle          : in std_logic;
        o_vdma_cfg_valid         : out std_logic;
        i_vdma_cfg_ready         : in std_logic;
        o_vdma_cfg_enable        : out std_logic;
        o_vdma_cfg_hsize         : out gpx_vdma_geometry_value_t;
        o_vdma_cfg_vsize         : out gpx_vdma_geometry_value_t;
        o_vdma_cfg_stride        : out gpx_vdma_geometry_value_t;
        o_active_profile         : out gpx_vdma_lane_profile_t;
        o_profile_activated      : out std_logic;
        o_profile_pending        : out std_logic;

        i_line_word       : in  gpx_vdma_line_word_event_t;
        o_line_word_ready : out std_logic;
        i_frame_close_event : in  gpx_frame_close_event_t;
        o_frame_close_ready : out std_logic;
        o_line_word       : out gpx_vdma_line_word_event_t;
        i_line_word_ready : in  std_logic;
        o_active_hsize_bytes : out gpx_vdma_geometry_value_t;
        o_active_vsize_lines : out gpx_vdma_geometry_value_t;
        o_stride_bytes       : out gpx_vdma_geometry_value_t;
        o_footer_emitted     : out std_logic;
        o_idle               : out std_logic
    );
end entity lidar_gpx_face_footer_builder_impl;

architecture rtl of lidar_gpx_face_footer_builder_impl is

    function fn_build_config return lidar_build_config_t is
        variable result : lidar_build_config_t := C_DEFAULT_BUILD_CONFIG;
    begin
        result.proc_clk_mhz := G_CLK_MHZ;
        result.tdc_clk_mhz := 200;
        result.stream_clock_mode := STREAM_CLOCK_ASYNC;
        result.output_width := G_OUTPUT_WIDTH;
        return result;
    end function fn_build_config;

    constant C_BUILD_CONFIG : lidar_build_config_t := fn_build_config;
    signal active_profile_s : gpx_vdma_lane_profile_t;

begin

    o_active_profile <= active_profile_s;

    u_profile : entity work.lidar_gpx_vdma_profile_manager
        generic map (
            G_BUILD_CONFIG => C_BUILD_CONFIG,
            G_LANE_RISE    => true
        )
        port map (
            i_clk                   => i_clk,
            i_rst_n                 => i_rst_n,
            i_abort                 => i_abort,
            i_request_valid         => i_profile_request_valid,
            o_request_ready         => o_profile_request_ready,
            i_lane_chip_mask        => i_lane_chip_mask,
            i_visible_returns       => i_visible_returns,
            i_planned_shots         => i_planned_shots,
            o_request_rejected      => o_profile_rejected,
            i_activate_valid        => i_profile_activate_valid,
            o_activate_ready        => o_profile_activate_ready,
            i_datapath_idle         => i_datapath_idle,
            o_vdma_cfg_valid        => o_vdma_cfg_valid,
            i_vdma_cfg_ready        => i_vdma_cfg_ready,
            o_vdma_cfg_enable       => o_vdma_cfg_enable,
            o_vdma_hsize_bytes      => o_vdma_cfg_hsize,
            o_vdma_vsize_lines      => o_vdma_cfg_vsize,
            o_vdma_stride_bytes     => o_vdma_cfg_stride,
            o_active_profile        => active_profile_s,
            o_profile_activated     => o_profile_activated,
            o_pending_valid         => o_profile_pending,
            o_busy                  => open
        );

    u_footer : entity work.lidar_gpx_face_footer_builder
        generic map (
            G_BUILD_CONFIG => C_BUILD_CONFIG,
            G_LANE_RISE    => true
        )
        port map (
            i_clk                => i_clk,
            i_rst_n              => i_rst_n,
            i_abort              => i_abort,
            i_active_profile     => active_profile_s,
            i_line_word          => i_line_word,
            o_line_word_ready    => o_line_word_ready,
            i_frame_close_event  => i_frame_close_event,
            o_frame_close_ready  => o_frame_close_ready,
            o_line_word          => o_line_word,
            i_line_word_ready    => i_line_word_ready,
            o_active_hsize_bytes => o_active_hsize_bytes,
            o_active_vsize_lines => o_active_vsize_lines,
            o_stride_bytes       => o_stride_bytes,
            o_footer_emitted     => o_footer_emitted,
            o_idle               => o_idle
        );

end architecture rtl;
