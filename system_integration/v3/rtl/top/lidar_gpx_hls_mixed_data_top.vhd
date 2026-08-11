library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_build_pkg.all;
use work.lidar_config_types_pkg.all;
use work.lidar_event_types_pkg.all;
use work.lidar_gpx_event_pkg.all;
use work.lidar_gpx_data_pkg.all;
use work.lidar_gpx_vdma_pkg.all;
use work.lidar_v3_hls_contract_pkg.all;

-- H5 혼합 RTL/HLS 데이터 경로 Top이다.
--
-- 이후 Parent 통합에서 V2 Processing-domain 데이터 경로를 교체하는 단위다.
-- TDC-domain 비동기 결과 FIFO 뒤에서 시작하여 Rise/Fall AXI4-Stream에서
-- 끝난다. 따라서 TDC-GPX 물리 Pin timing, CDC, Motor/Laser 제어와 물리
-- fire_done 승인부터 측정 시작 기준시점 (T0)까지의 경로를 중복하지 않는다.
entity lidar_gpx_hls_mixed_data_top is
    generic (
        G_BUILD_CONFIG : lidar_build_config_t := C_DEFAULT_BUILD_CONFIG
    );
    port (
        i_clk           : in std_logic;
        i_rst_n         : in std_logic;
        i_abort         : in std_logic;
        i_clear_sticky  : in std_logic;
        i_active_valid  : in std_logic;
        i_active_config : in lidar_active_config_t;
        i_rise_active_profile : in gpx_vdma_lane_profile_t;
        i_fall_active_profile : in gpx_vdma_lane_profile_t;

        i_raw_event : in gpx_raw_event_t;
        o_raw_ready : out std_logic;
        i_face_close_event : in face_close_event_t;
        o_face_close_ready : out std_logic;

        o_rise_tdata : out std_logic_vector(
            G_BUILD_CONFIG.output_width - 1 downto 0);
        o_rise_tkeep : out std_logic_vector(
            G_BUILD_CONFIG.output_width / 8 - 1 downto 0);
        o_rise_tstrb : out std_logic_vector(
            G_BUILD_CONFIG.output_width / 8 - 1 downto 0);
        o_rise_tuser : out std_logic_vector(0 downto 0);
        o_rise_tvalid : out std_logic;
        o_rise_tlast : out std_logic;
        i_rise_tready : in std_logic;
        o_fall_tdata : out std_logic_vector(
            G_BUILD_CONFIG.output_width - 1 downto 0);
        o_fall_tkeep : out std_logic_vector(
            G_BUILD_CONFIG.output_width / 8 - 1 downto 0);
        o_fall_tstrb : out std_logic_vector(
            G_BUILD_CONFIG.output_width / 8 - 1 downto 0);
        o_fall_tuser : out std_logic_vector(0 downto 0);
        o_fall_tvalid : out std_logic;
        o_fall_tlast : out std_logic;
        i_fall_tready : in std_logic;

        o_shot_done : out std_logic;
        o_shot_done_context : out shot_start_event_t;
        o_frame_output_done : out std_logic;
        o_idle : out std_logic;
        o_decoder_inflight : out unsigned(7 downto 0);
        o_rise_emitted_lines : out unsigned(16 downto 0);
        o_fall_emitted_lines : out unsigned(16 downto 0);
        o_hit_fault_sticky : out gpx_hit_decoder_faults_t;
        o_cell_fault_sticky : out gpx_cell_collector_faults_t;
        o_frame_fault_sticky : out gpx_frame_assembler_faults_t;
        o_rise_formatter_fault_sticky :
            out lidar_gpx_word_formatter_faults_t;
        o_fall_formatter_fault_sticky :
            out lidar_gpx_word_formatter_faults_t
    );
end entity lidar_gpx_hls_mixed_data_top;

architecture rtl of lidar_gpx_hls_mixed_data_top is
    signal h3_rise_event_s : gpx_frame_cell_event_t :=
        C_GPX_FRAME_CELL_EVENT_IDLE;
    signal h3_fall_event_s : gpx_frame_cell_event_t :=
        C_GPX_FRAME_CELL_EVENT_IDLE;
    signal rise_event_r : gpx_frame_cell_event_t :=
        C_GPX_FRAME_CELL_EVENT_IDLE;
    signal fall_event_r : gpx_frame_cell_event_t :=
        C_GPX_FRAME_CELL_EVENT_IDLE;
    signal frame_close_s : gpx_frame_close_event_t :=
        C_GPX_FRAME_CLOSE_EVENT_IDLE;
    signal h3_rise_ready_s : std_logic;
    signal h3_fall_ready_s : std_logic;
    signal rise_ready_s    : std_logic;
    signal fall_ready_s    : std_logic;
    signal frame_close_ready_s : std_logic;
    signal pipeline_idle_s : std_logic;
    signal output_idle_s : std_logic;
begin
    -- H3의 폭이 큰 Cell record가 FIFO 출력 선택 MUX를 거쳐 H4 입력까지
    -- 한 Cycle에 이동하지 않도록 Rise/Fall 각각에 순차 경계를 둔다.
    -- 이 1-entry elastic register는 데이터 순서를 바꾸지 않으며, H4가
    -- 멈추면 현재 record를 그대로 보존한다.
    h3_rise_ready_s <= '1' when
        rise_event_r.valid = '0' or rise_ready_s = '1' else '0';
    h3_fall_ready_s <= '1' when
        fall_event_r.valid = '0' or fall_ready_s = '1' else '0';

    p_h3_h4_pipeline : process (i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' or i_abort = '1' then
                rise_event_r <= C_GPX_FRAME_CELL_EVENT_IDLE;
                fall_event_r <= C_GPX_FRAME_CELL_EVENT_IDLE;
            else
                if h3_rise_ready_s = '1' then
                    if h3_rise_event_s.valid = '1' then
                        rise_event_r <= h3_rise_event_s;
                    else
                        rise_event_r <= C_GPX_FRAME_CELL_EVENT_IDLE;
                    end if;
                end if;
                if h3_fall_ready_s = '1' then
                    if h3_fall_event_s.valid = '1' then
                        fall_event_r <= h3_fall_event_s;
                    else
                        fall_event_r <= C_GPX_FRAME_CELL_EVENT_IDLE;
                    end if;
                end if;
            end if;
        end if;
    end process p_h3_h4_pipeline;

    assert fn_validate_build_config(G_BUILD_CONFIG) = CFG_OK
        report "V3-H5-TOP-001 invalid build configuration"
        severity failure;
    assert G_BUILD_CONFIG.output_width = 32 or
           G_BUILD_CONFIG.output_width = 64
        report "V3-H5-TOP-002 V3 output width must be 32 or 64"
        severity failure;

    o_idle <= pipeline_idle_s and output_idle_s;

    u_h1_h3_pipeline : entity work.lidar_gpx_hls_hit_cell_frame_pipeline
        generic map (G_BUILD_CONFIG => G_BUILD_CONFIG)
        port map (
            i_clk => i_clk,
            i_rst_n => i_rst_n,
            i_abort => i_abort,
            i_clear_sticky => i_clear_sticky,
            i_active_valid => i_active_valid,
            i_active_config => i_active_config,
            i_raw_event => i_raw_event,
            o_raw_ready => o_raw_ready,
            i_face_close_event => i_face_close_event,
            o_face_close_ready => o_face_close_ready,
            o_rise_event => h3_rise_event_s,
            i_rise_ready => h3_rise_ready_s,
            o_fall_event => h3_fall_event_s,
            i_fall_ready => h3_fall_ready_s,
            o_frame_close_event => frame_close_s,
            i_frame_close_ready => frame_close_ready_s,
            o_rise_line_done => open,
            o_fall_line_done => open,
            o_shot_done => o_shot_done,
            o_shot_done_context => o_shot_done_context,
            o_idle => pipeline_idle_s,
            o_decoder_inflight => o_decoder_inflight,
            o_hit_fault_pulse => open,
            o_hit_fault_sticky => o_hit_fault_sticky,
            o_cell_fault_pulse => open,
            o_cell_fault_sticky => o_cell_fault_sticky,
            o_frame_fault_pulse => open,
            o_frame_fault_sticky => o_frame_fault_sticky
        );

    u_h4_axis_output : entity work.lidar_gpx_hls_axis_output_subsystem
        generic map (G_BUILD_CONFIG => G_BUILD_CONFIG)
        port map (
            i_clk => i_clk,
            i_rst_n => i_rst_n,
            i_abort => i_abort,
            i_clear_sticky => i_clear_sticky,
            i_active_version => i_active_config.version,
            i_rise_active_profile => i_rise_active_profile,
            i_fall_active_profile => i_fall_active_profile,
            i_rise_event => rise_event_r,
            o_rise_ready => rise_ready_s,
            i_fall_event => fall_event_r,
            o_fall_ready => fall_ready_s,
            i_frame_close_event => frame_close_s,
            o_frame_close_ready => frame_close_ready_s,
            o_frame_output_done => o_frame_output_done,
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
            o_rise_line_done => open,
            o_fall_line_done => open,
            o_rise_frame_done => open,
            o_fall_frame_done => open,
            o_rise_emitted_lines => o_rise_emitted_lines,
            o_fall_emitted_lines => o_fall_emitted_lines,
            o_idle => output_idle_s,
            o_rise_fault_pulse => open,
            o_fall_fault_pulse => open,
            o_rise_fault_sticky => o_rise_formatter_fault_sticky,
            o_fall_fault_sticky => o_fall_formatter_fault_sticky
        );
end architecture rtl;
