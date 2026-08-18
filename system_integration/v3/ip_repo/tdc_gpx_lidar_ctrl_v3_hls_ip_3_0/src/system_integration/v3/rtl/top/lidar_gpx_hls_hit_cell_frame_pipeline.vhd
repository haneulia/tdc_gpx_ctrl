library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_build_pkg.all;
use work.lidar_config_types_pkg.all;
use work.lidar_event_types_pkg.all;
use work.lidar_gpx_event_pkg.all;
use work.lidar_gpx_data_pkg.all;

-- H5 Processing clock 데이터 pipeline이다.
--
-- H1은 외부 TDC-GPX I-Mode 28-bit Word를 해석하고, H2는 한 STOP 채널의
-- 물리 Return을 Cell로 수집하며, H3는 Cell을 독립 Rise/Fall lane 순서로
-- 정렬한다. Motor/Laser 제어, 물리 fire_done 승인부터 측정 시작 기준시점
-- (T0), GPX bus PHY, CDC와 AXI packer는 이 블록 밖의 유지 RTL이 소유한다.
entity lidar_gpx_hls_hit_cell_frame_pipeline is
    generic (
        G_BUILD_CONFIG : lidar_build_config_t := C_DEFAULT_BUILD_CONFIG
    );
    port (
        i_clk           : in  std_logic;
        i_rst_n         : in  std_logic;
        i_abort         : in  std_logic;
        i_clear_sticky  : in  std_logic;
        i_active_valid  : in  std_logic;
        i_active_config : in  lidar_active_config_t;

        i_raw_event : in  gpx_raw_event_t;
        o_raw_ready : out std_logic;

        i_face_close_event : in  face_close_event_t;
        o_face_close_ready : out std_logic;

        o_rise_event : out gpx_frame_cell_event_t;
        i_rise_ready : in  std_logic;
        o_fall_event : out gpx_frame_cell_event_t;
        i_fall_ready : in  std_logic;
        o_frame_close_event : out gpx_frame_close_event_t;
        i_frame_close_ready : in  std_logic;

        o_rise_line_done : out std_logic;
        o_fall_line_done : out std_logic;
        o_shot_done : out std_logic;
        o_shot_done_context : out shot_start_event_t;
        o_idle : out std_logic;
        o_decoder_inflight : out unsigned(7 downto 0);

        o_hit_fault_pulse  : out gpx_hit_decoder_faults_t;
        o_hit_fault_sticky : out gpx_hit_decoder_faults_t;
        o_cell_fault_pulse  : out gpx_cell_collector_faults_t;
        o_cell_fault_sticky : out gpx_cell_collector_faults_t;
        o_frame_fault_pulse  : out gpx_frame_assembler_faults_t;
        o_frame_fault_sticky : out gpx_frame_assembler_faults_t
    );
end entity lidar_gpx_hls_hit_cell_frame_pipeline;

architecture rtl of lidar_gpx_hls_hit_cell_frame_pipeline is
    signal hit_event_s  : gpx_hit_event_t := C_GPX_HIT_EVENT_IDLE;
    signal hit_ready_s  : std_logic;
    signal cell_event_s : gpx_cell_event_t := C_GPX_CELL_EVENT_IDLE;
    signal cell_ready_s : std_logic;
    signal decoder_idle_s   : std_logic;
    signal collector_idle_s : std_logic;
    signal assembler_idle_s : std_logic;
begin
    assert fn_validate_build_config(G_BUILD_CONFIG) = CFG_OK
        report "V3-H5-PIPE-001 invalid build configuration"
        severity failure;

    o_idle <= decoder_idle_s and collector_idle_s and assembler_idle_s;

    u_h1_hit_decoder : entity work.lidar_gpx_hit_decoder_hls_adapter
        generic map (G_BUILD_CONFIG => G_BUILD_CONFIG)
        port map (
            i_clk => i_clk,
            i_rst_n => i_rst_n,
            i_abort => i_abort,
            i_clear_sticky => i_clear_sticky,
            i_active_rise_mask =>
                i_active_config.derived.active_rise_mask,
            i_active_fall_mask =>
                i_active_config.derived.active_fall_mask,
            i_raw_event => i_raw_event,
            o_raw_ready => o_raw_ready,
            o_hit_event => hit_event_s,
            i_hit_ready => hit_ready_s,
            o_idle => decoder_idle_s,
            o_inflight_count => o_decoder_inflight,
            o_fault_pulse => o_hit_fault_pulse,
            o_fault_sticky => o_hit_fault_sticky
        );

    u_h2_cell_collector : entity work.lidar_gpx_cell_collector_hls_adapter
        generic map (G_BUILD_CONFIG => G_BUILD_CONFIG)
        port map (
            i_clk => i_clk,
            i_rst_n => i_rst_n,
            i_abort => i_abort,
            i_clear_sticky => i_clear_sticky,
            i_active_version => i_active_config.version,
            -- Runtime 직렬화(전시) Return 슬롯 수다. 물리 IFIFO는 이 값과
            -- 무관하게 EF 완료까지 Drain하며 8번째 물리 Return만 overflow다.
            i_max_hits_per_stop =>
                i_active_config.source.tdc.max_hits_per_stop,
            i_active_rise_mask =>
                i_active_config.derived.active_rise_mask,
            i_active_fall_mask =>
                i_active_config.derived.active_fall_mask,
            i_hit_event => hit_event_s,
            o_hit_ready => hit_ready_s,
            o_cell_event => cell_event_s,
            i_cell_ready => cell_ready_s,
            o_idle => collector_idle_s,
            o_fault_pulse => o_cell_fault_pulse,
            o_fault_sticky => o_cell_fault_sticky
        );

    u_h3_frame_assembler :
        entity work.lidar_gpx_frame_lane_assembler_hls_adapter
        generic map (G_BUILD_CONFIG => G_BUILD_CONFIG)
        port map (
            i_clk => i_clk,
            i_rst_n => i_rst_n,
            i_abort => i_abort,
            i_clear_sticky => i_clear_sticky,
            i_active_version => i_active_config.version,
            i_active_rise_mask =>
                i_active_config.derived.active_rise_mask,
            i_active_fall_mask =>
                i_active_config.derived.active_fall_mask,
            i_columns_per_face =>
                i_active_config.derived.columns_per_face,
            i_cell_event => cell_event_s,
            o_cell_ready => cell_ready_s,
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
            o_idle => assembler_idle_s,
            o_fault_pulse => o_frame_fault_pulse,
            o_fault_sticky => o_frame_fault_sticky
        );

    -- synthesis translate_off
    p_contract : process (i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '1' and i_abort = '0' then
                if i_raw_event.valid = '1' or
                   i_face_close_event.valid = '1' then
                    assert i_active_valid = '1'
                        report "V3-H5-PIPE-002 event without Active config"
                        severity failure;
                end if;
            end if;
        end if;
    end process p_contract;
    -- synthesis translate_on
end architecture rtl;
