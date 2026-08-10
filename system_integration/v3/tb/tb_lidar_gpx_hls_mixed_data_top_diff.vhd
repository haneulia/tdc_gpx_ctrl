-- ============================================================================
-- H5 종단 간 차분 테스트벤치
--
-- 목적:
--   같은 외부 TDC-GPX I-Mode Raw 28-bit 사건을 V2 Golden 데이터 경로와
--   V3 H1~H4 혼합 RTL/HLS Top에 동시에 입력하고 최종 AXI Beat를 비교한다.
--
-- 검증 범위:
--   * 4개 TDC-GPX Chip, Chip당 8 STOP 채널
--   * Dedicated Rise2/Fall2와 모든 4-Chip 양 Edge topology
--   * Runtime 직렬화(전시) Return 슬롯 1/3/7, 물리 Return은 항상 7개 Drain
--   * 합성 시 결정되는 32/64-bit AXI4-Stream 폭과 150/200 MHz
--   * Rise/Fall 독립 backpressure, PACKED17, Shot Metadata, Face Footer
--   * 부분 Shot 도중 abort 후 stale 데이터 제거와 다음 Face 정상 복구
--   * H1 accepted Raw 사건 미완료 개수와 실제 통합 idle
--
-- 유지보수 주의:
--   두 경로의 latency는 달라도 된다. 먼저 도착한 출력은 정지시키고 두 경로
--   모두 TVALID일 때만 같은 TREADY로 수락하여 payload와 경계를 비교한다.
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.env.all;

use work.lidar_build_pkg.all;
use work.lidar_config_types_pkg.all;
use work.lidar_config_reference_pkg.all;
use work.lidar_event_types_pkg.all;
use work.lidar_gpx_event_pkg.all;
use work.lidar_gpx_data_pkg.all;
use work.lidar_gpx_vdma_pkg.all;
use work.lidar_v3_hls_contract_pkg.all;

entity tb_lidar_gpx_hls_mixed_data_top_diff is
    generic (
        G_CLK_MHZ      : positive := 150;
        G_OUTPUT_WIDTH : positive := 32;
        G_ALL_DUAL     : boolean := false;
        G_FALL_ENABLE  : boolean := true;
        G_RETURN_SLOTS : positive range 1 to 7 := 1
    );
end entity tb_lidar_gpx_hls_mixed_data_top_diff;

architecture sim of tb_lidar_gpx_hls_mixed_data_top_diff is
    constant C_CHIPS : positive := 4;
    constant C_STOPS : positive := 8;
    constant C_PHYSICAL_RETURNS : positive := 7;
    constant C_PLANNED_SHOTS : positive := 2;
    constant C_ACTIVE_VERSION : unsigned(15 downto 0) := x"0055";
    constant C_CLK_PERIOD : time := 1 us / G_CLK_MHZ;

    function fn_rise_mask return chip_mask_t is
    begin
        if G_ALL_DUAL or not G_FALL_ENABLE then
            return "1111";
        end if;
        return "0011";
    end function fn_rise_mask;

    function fn_fall_mask return chip_mask_t is
    begin
        if not G_FALL_ENABLE then
            return "0000";
        end if;
        if G_ALL_DUAL then
            return "1111";
        end if;
        return "1100";
    end function fn_fall_mask;

    function fn_popcount(value : chip_mask_t) return natural is
        variable result : natural := 0;
    begin
        for index in value'range loop
            if value(index) = '1' then
                result := result + 1;
            end if;
        end loop;
        return result;
    end function fn_popcount;

    function fn_build_config return lidar_build_config_t is
        variable result : lidar_build_config_t := C_DEFAULT_BUILD_CONFIG;
    begin
        result.proc_clk_mhz := G_CLK_MHZ;
        result.tdc_clk_mhz := 200;
        result.stream_clock_mode := STREAM_CLOCK_ASYNC;
        result.num_chips := C_CHIPS;
        result.stops_per_chip := C_STOPS;
        result.max_returns_per_stop := C_PHYSICAL_RETURNS;
        result.rise_capability_mask := fn_rise_mask;
        result.fall_capability_mask := fn_fall_mask;
        result.output_width := G_OUTPUT_WIDTH;
        result.num_faces := 4;
        result.enable_echo_receiver := false;
        result.enable_echo_simulation := false;
        return result;
    end function fn_build_config;

    constant C_BUILD_CONFIG : lidar_build_config_t := fn_build_config;

    function fn_active_config return lidar_active_config_t is
        variable runtime_v : lidar_runtime_config_t :=
            fn_default_runtime_config(C_BUILD_CONFIG);
        variable result : lidar_active_config_t;
    begin
        runtime_v.tdc.active_chip_mask := "1111";
        if G_FALL_ENABLE then
            runtime_v.tdc.falling_enable := '1';
        else
            runtime_v.tdc.falling_enable := '0';
        end if;
        runtime_v.tdc.max_hits_per_stop := to_unsigned(
            G_RETURN_SLOTS, runtime_v.tdc.max_hits_per_stop'length);
        result.version := C_ACTIVE_VERSION;
        result.source := runtime_v;
        result.derived := fn_derive_runtime_config(
            C_BUILD_CONFIG, runtime_v);
        result.derived.columns_per_face := to_unsigned(
            C_PLANNED_SHOTS, result.derived.columns_per_face'length);
        return result;
    end function fn_active_config;

    constant C_ACTIVE_CONFIG : lidar_active_config_t := fn_active_config;

    function fn_profile(lane_rise : boolean)
        return gpx_vdma_lane_profile_t is
        variable result : gpx_vdma_lane_profile_t :=
            C_GPX_VDMA_LANE_PROFILE_IDLE;
        variable slot_count_v : natural;
        variable hsize_v : natural;
    begin
        if lane_rise then
            slot_count_v := fn_popcount(
                C_ACTIVE_CONFIG.derived.active_rise_mask) * C_STOPS;
        else
            slot_count_v := fn_popcount(
                C_ACTIVE_CONFIG.derived.active_fall_mask) * C_STOPS;
        end if;
        hsize_v := fn_gpx_vdma_shot_hsize_bytes(
            slot_count_v, G_RETURN_SLOTS, G_OUTPUT_WIDTH);
        result.valid := '1';
        if slot_count_v /= 0 then
            result.enabled := '1';
        end if;
        result.slot_count := to_unsigned(
            slot_count_v, result.slot_count'length);
        result.visible_returns := to_unsigned(
            G_RETURN_SLOTS, result.visible_returns'length);
        result.cell_word_count := to_unsigned(
            fn_gpx_vdma_cell_word_count(G_RETURN_SLOTS),
            result.cell_word_count'length);
        result.planned_shots := to_unsigned(
            C_PLANNED_SHOTS, result.planned_shots'length);
        result.raw_line_words := to_unsigned(
            fn_gpx_vdma_shot_raw_hsize_bytes(
                slot_count_v, G_RETURN_SLOTS) / 4,
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
            fn_gpx_vdma_stride_bytes(
                C_CHIPS * C_STOPS, C_PHYSICAL_RETURNS, G_OUTPUT_WIDTH),
            result.stride_bytes'length);
        return result;
    end function fn_profile;

    constant C_RISE_PROFILE : gpx_vdma_lane_profile_t := fn_profile(true);
    constant C_FALL_PROFILE : gpx_vdma_lane_profile_t := fn_profile(false);

    function fn_shot_context(shot_column : natural)
        return shot_start_event_t is
        variable result : shot_start_event_t := C_SHOT_START_EVENT_IDLE;
    begin
        result.valid := '1';
        result.request.valid := '1';
        result.request.face_index := to_unsigned(1, 3);
        result.request.position := to_unsigned(
            2000 + shot_column * 5, result.request.position'length);
        result.request.direction := DIRECTION_CW;
        result.request.shot_index := to_unsigned(
            shot_column, result.request.shot_index'length);
        if shot_column + 1 = C_PLANNED_SHOTS then
            result.request.last_in_face := '1';
        end if;
        result.request.source_sim := '0';
        result.request.source_latency_clks := to_unsigned(
            7 + shot_column, result.request.source_latency_clks'length);
        result.request.source_latency_valid := '1';
        result.request.active_version := C_ACTIVE_VERSION;
        result.fire_to_t0_clks := to_unsigned(
            13 + shot_column, result.fire_to_t0_clks'length);
        -- 측정 시작 기준시점 (T0): 동기화된 fire_done 승인과 start_tdc
        -- 발생 사건. Face 기준 절대 시각과 Shot별 delta의 기준이 된다.
        result.t0_timestamp_ticks := to_unsigned(
            16#100000# + shot_column * 16#123#,
            result.t0_timestamp_ticks'length);
        result.t0_timestamp_valid := '1';
        result.t0_time_sync_valid := '1';
        return result;
    end function fn_shot_context;

    function fn_raw_data(
        chip_index  : natural;
        stop_index  : natural;
        slope_value : gpx_slope_t;
        return_index : natural;
        shot_column : natural)
        return gpx_raw_event_t is
        variable result : gpx_raw_event_t := C_GPX_RAW_EVENT_IDLE;
        variable hit_value : natural;
    begin
        result.valid := '1';
        result.kind := GPX_RAW_DATA;
        result.chip_index := to_unsigned(
            chip_index, result.chip_index'length);
        if stop_index >= 4 then
            result.ififo_id := '1';
        end if;
        result.raw_word(
            C_GPX_RAW_CHACODE_HI downto C_GPX_RAW_CHACODE_LO) :=
            std_logic_vector(to_unsigned(stop_index mod 4, 2));
        result.raw_word(C_GPX_RAW_START_HI downto C_GPX_RAW_START_LO) :=
            (others => '0');
        result.raw_word(C_GPX_RAW_SLOPE_BIT) :=
            fn_gpx_slope_to_bit(slope_value);
        hit_value := (shot_column * 16#3000# + chip_index * 16#700# +
            stop_index * 16#40# + return_index * 3 + 1) mod 16#20000#;
        if slope_value = GPX_SLOPE_FALL then
            hit_value := (hit_value + 16#10000#) mod 16#20000#;
        end if;
        result.raw_word(C_GPX_RAW_HIT_HI downto C_GPX_RAW_HIT_LO) :=
            std_logic_vector(to_unsigned(hit_value, C_GPX_HIT_WIDTH));
        result.shot_context := fn_shot_context(shot_column);
        result.chip_shot_seq := to_unsigned(
            shot_column, result.chip_shot_seq'length);
        return result;
    end function fn_raw_data;

    function fn_raw_done(chip_index : natural; shot_column : natural)
        return gpx_raw_event_t is
        variable result : gpx_raw_event_t := C_GPX_RAW_EVENT_IDLE;
    begin
        result.valid := '1';
        result.kind := GPX_RAW_DRAIN_DONE;
        result.chip_index := to_unsigned(
            chip_index, result.chip_index'length);
        result.ififo_id := '1';
        result.shot_context := fn_shot_context(shot_column);
        result.chip_shot_seq := to_unsigned(
            shot_column, result.chip_shot_seq'length);
        return result;
    end function fn_raw_done;

    function fn_close return face_close_event_t is
        variable result : face_close_event_t := C_FACE_CLOSE_EVENT_IDLE;
    begin
        result.valid := '1';
        result.face_frame_id := x"00000155";
        result.face_index := to_unsigned(1, result.face_index'length);
        result.direction := DIRECTION_CW;
        result.source_sim := '0';
        result.active_version := C_ACTIVE_VERSION;
        result.columns_per_face := to_unsigned(
            C_PLANNED_SHOTS, result.columns_per_face'length);
        return result;
    end function fn_close;

    signal clk : std_logic := '0';
    signal rst_n : std_logic := '0';
    signal abort_event : std_logic := '0';
    signal clear_sticky : std_logic := '0';
    signal source_raw : gpx_raw_event_t := C_GPX_RAW_EVENT_IDLE;
    signal source_raw_ready : std_logic;
    signal source_close : face_close_event_t := C_FACE_CLOSE_EVENT_IDLE;
    signal source_close_ready : std_logic;

    signal v2_raw : gpx_raw_event_t := C_GPX_RAW_EVENT_IDLE;
    signal v3_raw : gpx_raw_event_t := C_GPX_RAW_EVENT_IDLE;
    signal v2_raw_ready : std_logic;
    signal v3_raw_ready : std_logic;
    signal v2_close : face_close_event_t := C_FACE_CLOSE_EVENT_IDLE;
    signal v3_close : face_close_event_t := C_FACE_CLOSE_EVENT_IDLE;
    signal v2_close_ready : std_logic;
    signal v3_close_ready : std_logic;

    signal v2_rise_event : gpx_frame_cell_event_t;
    signal v2_fall_event : gpx_frame_cell_event_t;
    signal v2_frame_close : gpx_frame_close_event_t;
    signal v2_rise_ready : std_logic;
    signal v2_fall_ready : std_logic;
    signal v2_frame_close_ready : std_logic;
    signal v2_pipeline_idle : std_logic;
    signal v2_axis_idle : std_logic;
    signal v2_frame_output_done : std_logic;

    signal v2_rise_tdata : std_logic_vector(G_OUTPUT_WIDTH - 1 downto 0);
    signal v2_rise_tkeep : std_logic_vector(G_OUTPUT_WIDTH / 8 - 1 downto 0);
    signal v2_rise_tstrb : std_logic_vector(G_OUTPUT_WIDTH / 8 - 1 downto 0);
    signal v2_rise_tuser : std_logic_vector(0 downto 0);
    signal v2_rise_tvalid : std_logic;
    signal v2_rise_tlast : std_logic;
    signal v2_rise_tready : std_logic;
    signal v2_fall_tdata : std_logic_vector(G_OUTPUT_WIDTH - 1 downto 0);
    signal v2_fall_tkeep : std_logic_vector(G_OUTPUT_WIDTH / 8 - 1 downto 0);
    signal v2_fall_tstrb : std_logic_vector(G_OUTPUT_WIDTH / 8 - 1 downto 0);
    signal v2_fall_tuser : std_logic_vector(0 downto 0);
    signal v2_fall_tvalid : std_logic;
    signal v2_fall_tlast : std_logic;
    signal v2_fall_tready : std_logic;

    signal v3_rise_tdata : std_logic_vector(G_OUTPUT_WIDTH - 1 downto 0);
    signal v3_rise_tkeep : std_logic_vector(G_OUTPUT_WIDTH / 8 - 1 downto 0);
    signal v3_rise_tstrb : std_logic_vector(G_OUTPUT_WIDTH / 8 - 1 downto 0);
    signal v3_rise_tuser : std_logic_vector(0 downto 0);
    signal v3_rise_tvalid : std_logic;
    signal v3_rise_tlast : std_logic;
    signal v3_rise_tready : std_logic;
    signal v3_fall_tdata : std_logic_vector(G_OUTPUT_WIDTH - 1 downto 0);
    signal v3_fall_tkeep : std_logic_vector(G_OUTPUT_WIDTH / 8 - 1 downto 0);
    signal v3_fall_tstrb : std_logic_vector(G_OUTPUT_WIDTH / 8 - 1 downto 0);
    signal v3_fall_tuser : std_logic_vector(0 downto 0);
    signal v3_fall_tvalid : std_logic;
    signal v3_fall_tlast : std_logic;
    signal v3_fall_tready : std_logic;
    signal v3_idle : std_logic;
    signal v3_frame_output_done : std_logic;
    signal v3_decoder_inflight : unsigned(7 downto 0);

    signal sink_cycle_r : natural range 0 to 31 := 0;
    signal sink_ready_r : std_logic := '0';
    signal rise_beats_r : natural := 0;
    signal fall_beats_r : natural := 0;
    signal rise_lines_r : natural := 0;
    signal fall_lines_r : natural := 0;
    signal rise_sof_r : natural := 0;
    signal fall_sof_r : natural := 0;
    signal v2_frame_done_count_r : natural := 0;
    signal v3_frame_done_count_r : natural := 0;
    signal max_decoder_inflight_r : natural := 0;
begin
    assert G_OUTPUT_WIDTH = 32 or G_OUTPUT_WIDTH = 64
        report "V3-H5-DIFF output width must be 32 or 64"
        severity failure;

    clk <= not clk after C_CLK_PERIOD / 2;

    -- 두 DUT가 동일한 사건을 정확히 한 번씩 수락하도록 입력을 묶는다.
    p_gate_input : process (all)
        variable v2_raw_v : gpx_raw_event_t;
        variable v3_raw_v : gpx_raw_event_t;
        variable v2_close_v : face_close_event_t;
        variable v3_close_v : face_close_event_t;
    begin
        v2_raw_v := source_raw;
        v3_raw_v := source_raw;
        v2_raw_v.valid := source_raw.valid and v3_raw_ready;
        v3_raw_v.valid := source_raw.valid and v2_raw_ready;
        v2_raw <= v2_raw_v;
        v3_raw <= v3_raw_v;
        source_raw_ready <= v2_raw_ready and v3_raw_ready;

        v2_close_v := source_close;
        v3_close_v := source_close;
        v2_close_v.valid := source_close.valid and v3_close_ready;
        v3_close_v.valid := source_close.valid and v2_close_ready;
        v2_close <= v2_close_v;
        v3_close <= v3_close_v;
        source_close_ready <= v2_close_ready and v3_close_ready;
    end process p_gate_input;

    -- 먼저 출력한 DUT를 정지시켜 latency 차이를 제거하고 Beat만 비교한다.
    v2_rise_tready <= sink_ready_r and v3_rise_tvalid;
    v3_rise_tready <= sink_ready_r and v2_rise_tvalid;
    v2_fall_tready <= sink_ready_r and v3_fall_tvalid;
    v3_fall_tready <= sink_ready_r and v2_fall_tvalid;

    u_v2_pipeline : entity work.lidar_gpx_hit_cell_frame_pipeline
        generic map (G_BUILD_CONFIG => C_BUILD_CONFIG)
        port map (
            i_clk => clk,
            i_rst_n => rst_n,
            i_abort => abort_event,
            i_clear_sticky => clear_sticky,
            i_active_valid => '1',
            i_active_config => C_ACTIVE_CONFIG,
            i_raw_event => v2_raw,
            o_raw_ready => v2_raw_ready,
            i_face_close_event => v2_close,
            o_face_close_ready => v2_close_ready,
            o_rise_event => v2_rise_event,
            i_rise_ready => v2_rise_ready,
            o_fall_event => v2_fall_event,
            i_fall_ready => v2_fall_ready,
            o_frame_close_event => v2_frame_close,
            i_frame_close_ready => v2_frame_close_ready,
            o_rise_line_done => open,
            o_fall_line_done => open,
            o_shot_done => open,
            o_shot_done_context => open,
            o_idle => v2_pipeline_idle,
            o_hit_fault_pulse => open,
            o_hit_fault_sticky => open,
            o_cell_fault_pulse => open,
            o_cell_fault_sticky => open,
            o_frame_fault_pulse => open,
            o_frame_fault_sticky => open
        );

    u_v2_axis : entity work.lidar_gpx_axis_output_subsystem
        generic map (G_BUILD_CONFIG => C_BUILD_CONFIG)
        port map (
            i_clk => clk,
            i_rst_n => rst_n,
            i_abort => abort_event,
            i_rise_active_profile => C_RISE_PROFILE,
            i_fall_active_profile => C_FALL_PROFILE,
            i_rise_event => v2_rise_event,
            o_rise_ready => v2_rise_ready,
            i_fall_event => v2_fall_event,
            o_fall_ready => v2_fall_ready,
            i_frame_close_event => v2_frame_close,
            o_frame_close_ready => v2_frame_close_ready,
            o_frame_output_done => v2_frame_output_done,
            o_rise_tdata => v2_rise_tdata,
            o_rise_tkeep => v2_rise_tkeep,
            o_rise_tstrb => v2_rise_tstrb,
            o_rise_tuser => v2_rise_tuser,
            o_rise_tvalid => v2_rise_tvalid,
            o_rise_tlast => v2_rise_tlast,
            i_rise_tready => v2_rise_tready,
            o_fall_tdata => v2_fall_tdata,
            o_fall_tkeep => v2_fall_tkeep,
            o_fall_tstrb => v2_fall_tstrb,
            o_fall_tuser => v2_fall_tuser,
            o_fall_tvalid => v2_fall_tvalid,
            o_fall_tlast => v2_fall_tlast,
            i_fall_tready => v2_fall_tready,
            o_rise_line_done => open,
            o_fall_line_done => open,
            o_rise_frame_done => open,
            o_fall_frame_done => open,
            o_idle => v2_axis_idle
        );

    u_v3_mixed : entity work.lidar_gpx_hls_mixed_data_top
        generic map (G_BUILD_CONFIG => C_BUILD_CONFIG)
        port map (
            i_clk => clk,
            i_rst_n => rst_n,
            i_abort => abort_event,
            i_clear_sticky => clear_sticky,
            i_active_valid => '1',
            i_active_config => C_ACTIVE_CONFIG,
            i_rise_active_profile => C_RISE_PROFILE,
            i_fall_active_profile => C_FALL_PROFILE,
            i_raw_event => v3_raw,
            o_raw_ready => v3_raw_ready,
            i_face_close_event => v3_close,
            o_face_close_ready => v3_close_ready,
            o_rise_tdata => v3_rise_tdata,
            o_rise_tkeep => v3_rise_tkeep,
            o_rise_tstrb => v3_rise_tstrb,
            o_rise_tuser => v3_rise_tuser,
            o_rise_tvalid => v3_rise_tvalid,
            o_rise_tlast => v3_rise_tlast,
            i_rise_tready => v3_rise_tready,
            o_fall_tdata => v3_fall_tdata,
            o_fall_tkeep => v3_fall_tkeep,
            o_fall_tstrb => v3_fall_tstrb,
            o_fall_tuser => v3_fall_tuser,
            o_fall_tvalid => v3_fall_tvalid,
            o_fall_tlast => v3_fall_tlast,
            i_fall_tready => v3_fall_tready,
            o_shot_done => open,
            o_shot_done_context => open,
            o_frame_output_done => v3_frame_output_done,
            o_idle => v3_idle,
            o_decoder_inflight => v3_decoder_inflight,
            o_rise_emitted_lines => open,
            o_fall_emitted_lines => open,
            o_hit_fault_sticky => open,
            o_cell_fault_sticky => open,
            o_frame_fault_sticky => open,
            o_rise_formatter_fault_sticky => open,
            o_fall_formatter_fault_sticky => open
        );

    p_backpressure : process (clk)
    begin
        if rising_edge(clk) then
            if rst_n = '0' or abort_event = '1' then
                sink_cycle_r <= 0;
                sink_ready_r <= '0';
            else
                if sink_cycle_r = 12 then
                    sink_cycle_r <= 0;
                else
                    sink_cycle_r <= sink_cycle_r + 1;
                end if;
                if sink_cycle_r = 2 or sink_cycle_r = 3 or
                   sink_cycle_r = 8 or sink_cycle_r = 11 then
                    sink_ready_r <= '0';
                else
                    sink_ready_r <= '1';
                end if;
            end if;
        end if;
    end process p_backpressure;

    p_compare : process (clk)
        variable inflight_v : natural;
    begin
        if rising_edge(clk) then
            if rst_n = '0' or abort_event = '1' then
                rise_beats_r <= 0;
                fall_beats_r <= 0;
                rise_lines_r <= 0;
                fall_lines_r <= 0;
                rise_sof_r <= 0;
                fall_sof_r <= 0;
                v2_frame_done_count_r <= 0;
                v3_frame_done_count_r <= 0;
                max_decoder_inflight_r <= 0;
            else
                inflight_v := to_integer(v3_decoder_inflight);
                if inflight_v > max_decoder_inflight_r then
                    max_decoder_inflight_r <= inflight_v;
                end if;
                if v2_rise_tvalid = '1' and v3_rise_tvalid = '1' and
                   sink_ready_r = '1' then
                    assert v2_rise_tdata = v3_rise_tdata and
                           v2_rise_tkeep = v3_rise_tkeep and
                           v2_rise_tstrb = v3_rise_tstrb and
                           v2_rise_tuser = v3_rise_tuser and
                           v2_rise_tlast = v3_rise_tlast
                        report "V3-H5-DIFF Rise AXI Beat mismatch"
                        severity failure;
                    rise_beats_r <= rise_beats_r + 1;
                    if v2_rise_tlast = '1' then
                        rise_lines_r <= rise_lines_r + 1;
                    end if;
                    if v2_rise_tuser(0) = '1' then
                        rise_sof_r <= rise_sof_r + 1;
                    end if;
                end if;
                if v2_fall_tvalid = '1' and v3_fall_tvalid = '1' and
                   sink_ready_r = '1' then
                    assert v2_fall_tdata = v3_fall_tdata and
                           v2_fall_tkeep = v3_fall_tkeep and
                           v2_fall_tstrb = v3_fall_tstrb and
                           v2_fall_tuser = v3_fall_tuser and
                           v2_fall_tlast = v3_fall_tlast
                        report "V3-H5-DIFF Fall AXI Beat mismatch"
                        severity failure;
                    fall_beats_r <= fall_beats_r + 1;
                    if v2_fall_tlast = '1' then
                        fall_lines_r <= fall_lines_r + 1;
                    end if;
                    if v2_fall_tuser(0) = '1' then
                        fall_sof_r <= fall_sof_r + 1;
                    end if;
                end if;
                if v2_frame_output_done = '1' then
                    v2_frame_done_count_r <= v2_frame_done_count_r + 1;
                end if;
                if v3_frame_output_done = '1' then
                    v3_frame_done_count_r <= v3_frame_done_count_r + 1;
                end if;
            end if;
        end if;
    end process p_compare;

    p_stimulus : process
        procedure wait_clocks(count : positive) is
        begin
            for index in 1 to count loop
                wait until rising_edge(clk);
            end loop;
            wait for 1 ps;
        end procedure wait_clocks;

        procedure send_raw(value : gpx_raw_event_t) is
        begin
            source_raw <= value;
            loop
                wait until rising_edge(clk);
                exit when source_raw_ready = '1';
            end loop;
            wait for 1 ps;
            source_raw <= C_GPX_RAW_EVENT_IDLE;
        end procedure send_raw;

        procedure send_shot(shot_column : natural) is
        begin
            for chip_index in 0 to C_CHIPS - 1 loop
                for stop_index in 0 to C_STOPS - 1 loop
                    if C_ACTIVE_CONFIG.derived.active_rise_mask(
                            chip_index) = '1' then
                        for return_index in 0 to C_PHYSICAL_RETURNS - 1 loop
                            send_raw(fn_raw_data(
                                chip_index, stop_index, GPX_SLOPE_RISE,
                                return_index, shot_column));
                        end loop;
                    end if;
                    if C_ACTIVE_CONFIG.derived.active_fall_mask(
                            chip_index) = '1' then
                        for return_index in 0 to C_PHYSICAL_RETURNS - 1 loop
                            send_raw(fn_raw_data(
                                chip_index, stop_index, GPX_SLOPE_FALL,
                                return_index, shot_column));
                        end loop;
                    end if;
                end loop;
                send_raw(fn_raw_done(chip_index, shot_column));
            end loop;
        end procedure send_shot;

        procedure send_close is
        begin
            source_close <= fn_close;
            loop
                wait until rising_edge(clk);
                exit when source_close_ready = '1';
            end loop;
            wait for 1 ps;
            source_close <= C_FACE_CLOSE_EVENT_IDLE;
        end procedure send_close;

        variable timeout_v : natural;
        variable expected_rise_lines_v : natural;
        variable expected_fall_lines_v : natural;
    begin
        wait_clocks(8);
        rst_n <= '1';
        wait_clocks(6);

        -- 완성되지 않은 Shot을 만든 뒤 abort하여 모든 HLS generation과
        -- retained RTL packer가 stale 상태를 제거하는지 확인한다.
        for return_index in 0 to 4 loop
            send_raw(fn_raw_data(
                0, 0, GPX_SLOPE_RISE, return_index, 15));
        end loop;
        wait_clocks(3);
        abort_event <= '1';
        wait_clocks(3);
        abort_event <= '0';
        clear_sticky <= '1';
        wait_clocks(1);
        clear_sticky <= '0';

        timeout_v := 0;
        while not (v2_pipeline_idle = '1' and v2_axis_idle = '1' and
                   v3_idle = '1') loop
            wait_clocks(1);
            timeout_v := timeout_v + 1;
            assert timeout_v < 2000
                report "V3-H5-DIFF idle timeout after abort"
                severity failure;
        end loop;

        for shot_column in 0 to C_PLANNED_SHOTS - 1 loop
            send_shot(shot_column);
        end loop;
        send_close;

        timeout_v := 0;
        while v2_frame_done_count_r = 0 or
              v3_frame_done_count_r = 0 loop
            wait_clocks(1);
            timeout_v := timeout_v + 1;
            assert timeout_v < 200000
                report "V3-H5-DIFF Face output timeout"
                severity failure;
        end loop;
        wait_clocks(10);

        expected_rise_lines_v := C_PLANNED_SHOTS +
            to_integer(C_RISE_PROFILE.footer_lines);
        expected_fall_lines_v := C_PLANNED_SHOTS +
            to_integer(C_FALL_PROFILE.footer_lines);
        assert rise_lines_r = expected_rise_lines_v and
               rise_sof_r = 1 and rise_beats_r > 0 and
               ((G_FALL_ENABLE and
                 fall_lines_r = expected_fall_lines_v and
                 fall_sof_r = 1 and fall_beats_r > 0) or
                (not G_FALL_ENABLE and
                 fall_lines_r = 0 and fall_sof_r = 0 and
                 fall_beats_r = 0)) and
               v2_frame_done_count_r = 1 and
               v3_frame_done_count_r = 1 and
               v2_pipeline_idle = '1' and v2_axis_idle = '1' and
               v3_idle = '1' and v3_decoder_inflight = 0
            report "V3-H5-DIFF completion/geometry/idle mismatch"
            severity failure;

        report "LIDAR_V3_H5_MIXED_DATA_TOP_DIFF_PASS clk_mhz=" &
            integer'image(G_CLK_MHZ) & " width=" &
            integer'image(G_OUTPUT_WIDTH) & " return_slots=" &
            integer'image(G_RETURN_SLOTS) & " all_dual=" &
            boolean'image(G_ALL_DUAL) & " fall_enable=" &
            boolean'image(G_FALL_ENABLE) & " decoder_inflight_max=" &
            integer'image(max_decoder_inflight_r)
            severity note;
        stop;
        wait;
    end process p_stimulus;
end architecture sim;

library ieee;
use ieee.std_logic_1164.all;

entity tb_lidar_gpx_hls_mixed_data_top_diff_dedicated_32_150 is
end entity;
architecture sim of tb_lidar_gpx_hls_mixed_data_top_diff_dedicated_32_150 is
begin
    u_test : entity work.tb_lidar_gpx_hls_mixed_data_top_diff
        generic map (
            G_CLK_MHZ => 150,
            G_OUTPUT_WIDTH => 32,
            G_ALL_DUAL => false,
            G_RETURN_SLOTS => 1
        );
end architecture;

library ieee;
use ieee.std_logic_1164.all;

entity tb_lidar_gpx_hls_mixed_data_top_diff_dedicated_64_200 is
end entity;
architecture sim of tb_lidar_gpx_hls_mixed_data_top_diff_dedicated_64_200 is
begin
    u_test : entity work.tb_lidar_gpx_hls_mixed_data_top_diff
        generic map (
            G_CLK_MHZ => 200,
            G_OUTPUT_WIDTH => 64,
            G_ALL_DUAL => false,
            G_RETURN_SLOTS => 3
        );
end architecture;

library ieee;
use ieee.std_logic_1164.all;

entity tb_lidar_gpx_hls_mixed_data_top_diff_all_dual_32_150 is
end entity;
architecture sim of tb_lidar_gpx_hls_mixed_data_top_diff_all_dual_32_150 is
begin
    u_test : entity work.tb_lidar_gpx_hls_mixed_data_top_diff
        generic map (
            G_CLK_MHZ => 150,
            G_OUTPUT_WIDTH => 32,
            G_ALL_DUAL => true,
            G_RETURN_SLOTS => 7
        );
end architecture;

library ieee;
use ieee.std_logic_1164.all;

entity tb_lidar_gpx_hls_mixed_data_top_diff_all_dual_64_200 is
end entity;
architecture sim of tb_lidar_gpx_hls_mixed_data_top_diff_all_dual_64_200 is
begin
    u_test : entity work.tb_lidar_gpx_hls_mixed_data_top_diff
        generic map (
            G_CLK_MHZ => 200,
            G_OUTPUT_WIDTH => 64,
            G_ALL_DUAL => true,
            G_RETURN_SLOTS => 7
        );
end architecture;

library ieee;
use ieee.std_logic_1164.all;

entity tb_lidar_gpx_hls_mixed_data_top_diff_rise_only_32_200 is
end entity;
architecture sim of tb_lidar_gpx_hls_mixed_data_top_diff_rise_only_32_200 is
begin
    u_test : entity work.tb_lidar_gpx_hls_mixed_data_top_diff
        generic map (
            G_CLK_MHZ => 200,
            G_OUTPUT_WIDTH => 32,
            G_ALL_DUAL => false,
            G_FALL_ENABLE => false,
            G_RETURN_SLOTS => 7
        );
end architecture;
