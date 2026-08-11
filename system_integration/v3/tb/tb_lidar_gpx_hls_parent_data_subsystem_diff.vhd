-- ============================================================================
-- H6-A Parent 데이터 경계 차분 테스트벤치
--
-- 목적:
--   실제 TDC-GPX I-Mode 28-bit 병렬 버스 모델에서 시작하여 EF 기반 IFIFO
--   전체 Drain, TDC/Processing 비동기 FIFO, H1~H4, 최종 32/64-bit AXI까지
--   V2 Golden과 V3 HLS Parent 데이터 경계를 독립 실행·비교한다.
--
-- 검증 범위:
--   * 4개 TDC-GPX Chip, Chip당 8 STOP, 물리 Return 7개 전체 Drain
--   * Rise 2/Fall 2 전용 및 4개 Chip 모두 Rising/Falling 양 Edge
--   * Runtime 직렬화(전시) Return 슬롯 1/7
--   * Processing:TDC clock 1:4, 4:1, 1:1, 150:200, 200:150
--   * Rise/Fall AXI backpressure와 Face Footer 완료 전 Face ACK 금지
--   * V2/V3 최종 TDATA/TKEEP/TSTRB/TUSER/TLAST 파일 단위 차분
--
-- 유지보수 주의:
--   이 TB는 한 실행에서 V2 또는 V3 한 경로만 물리 GPX 모델에 연결한다.
--   실행 스크립트가 동일 Profile의 두 캡처 파일을 비교한다. latency와 버스
--   제어 cycle은 달라도 되지만 승인된 AXI Beat 순서와 내용은 같아야 한다.
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.textio.all;
use ieee.std_logic_textio.all;
use std.env.all;

use work.lidar_build_pkg.all;
use work.lidar_config_types_pkg.all;
use work.lidar_config_reference_pkg.all;
use work.lidar_event_types_pkg.all;
use work.lidar_gpx_pkg.all;
use work.lidar_gpx_image_pkg.all;
use work.lidar_gpx_event_pkg.all;
use work.lidar_gpx_data_pkg.all;
use work.lidar_gpx_vdma_pkg.all;
use work.lidar_v3_hls_contract_pkg.all;
use work.tdc_gpx_pkg.all;

entity tb_lidar_gpx_hls_parent_data_subsystem_diff is
    generic (
        G_USE_V3         : boolean := true;
        G_PROC_CLK_MHZ    : positive := 150;
        G_TDC_CLK_MHZ     : positive := 200;
        G_OUTPUT_WIDTH    : positive := 32;
        G_ALL_DUAL        : boolean := false;
        G_VISIBLE_RETURNS : positive range 1 to 7 := 1
    );
end entity tb_lidar_gpx_hls_parent_data_subsystem_diff;

architecture sim of tb_lidar_gpx_hls_parent_data_subsystem_diff is

    constant C_CHIPS : positive := 4;
    constant C_STOPS_PER_CHIP : positive := 8;
    constant C_PHYSICAL_RETURNS : positive := 7;
    constant C_COLUMNS_PER_FACE : positive := 4;
    -- 7 Return 최대 부하를 정상 완료시키는 제품 기본 Drain 여유시간이다.
    -- 100 ns 같은 축소값은 처리 경로 비교가 아니라 의도적 timeout 시험에서만 쓴다.
    constant C_DRAIN_MARGIN_TIME_NS : positive := 6000;
    constant C_PROC_PERIOD : time := 1 us / G_PROC_CLK_MHZ;
    constant C_TDC_PERIOD : time := 1 us / G_TDC_CLK_MHZ;

    function fn_stream_clock_mode return stream_clock_mode_t is
    begin
        if G_PROC_CLK_MHZ = G_TDC_CLK_MHZ then
            return STREAM_CLOCK_SYNC;
        end if;
        return STREAM_CLOCK_ASYNC;
    end function fn_stream_clock_mode;

    function fn_slopes_per_chip return positive is
    begin
        if G_ALL_DUAL then
            return 2;
        end if;
        return 1;
    end function fn_slopes_per_chip;

    function fn_lane_slot_count return positive is
    begin
        if G_ALL_DUAL then
            return C_CHIPS * C_STOPS_PER_CHIP;
        end if;
        return 2 * C_STOPS_PER_CHIP;
    end function fn_lane_slot_count;

    function fn_rise_capability return chip_mask_t is
    begin
        if G_ALL_DUAL then
            return "1111";
        end if;
        return "0011";
    end function fn_rise_capability;

    function fn_fall_capability return chip_mask_t is
    begin
        if G_ALL_DUAL then
            return "1111";
        end if;
        return "1100";
    end function fn_fall_capability;

    constant C_BUILD_CONFIG : lidar_build_config_t := (
        proc_clk_mhz           => G_PROC_CLK_MHZ,
        tdc_clk_mhz            => G_TDC_CLK_MHZ,
        stream_clock_mode      => fn_stream_clock_mode,
        num_chips              => C_CHIPS,
        stops_per_chip         => C_STOPS_PER_CHIP,
        max_returns_per_stop   => C_PHYSICAL_RETURNS,
        rise_capability_mask   => fn_rise_capability,
        fall_capability_mask   => fn_fall_capability,
        output_width           => G_OUTPUT_WIDTH,
        num_faces              => 4,
        enable_echo_receiver   => false,
        enable_echo_simulation => false
    );

    function fn_active_config return lidar_active_config_t is
        variable runtime_v : lidar_runtime_config_t :=
            fn_default_runtime_config(C_BUILD_CONFIG);
        variable result : lidar_active_config_t;
    begin
        runtime_v.tdc.active_chip_mask := "1111";
        runtime_v.tdc.falling_enable := '1';
        runtime_v.tdc.max_hits_per_stop := to_unsigned(
            G_VISIBLE_RETURNS,
            runtime_v.tdc.max_hits_per_stop'length);
        result.version := to_unsigned(21, 16);
        result.source := runtime_v;
        result.derived := fn_derive_runtime_config(
            C_BUILD_CONFIG, runtime_v);
        result.derived.columns_per_face := to_unsigned(
            C_COLUMNS_PER_FACE, 16);
        return result;
    end function fn_active_config;

    constant C_ACTIVE_CONFIG : lidar_active_config_t := fn_active_config;
    constant C_LANE_SLOT_COUNT : positive := fn_lane_slot_count;
    constant C_WORDS_PER_IFIFO : positive :=
        4 * C_PHYSICAL_RETURNS * fn_slopes_per_chip;
    constant C_HSIZE_BYTES : positive := fn_gpx_vdma_shot_hsize_bytes(
        C_LANE_SLOT_COUNT, G_VISIBLE_RETURNS, G_OUTPUT_WIDTH);
    constant C_FOOTER_LINES : positive :=
        fn_gpx_vdma_footer_lines(C_HSIZE_BYTES);
    constant C_EXPECTED_LINES_PER_LANE : positive :=
        2 * (C_COLUMNS_PER_FACE + C_FOOTER_LINES);
    constant C_EXPECTED_BEATS_PER_LANE : positive :=
        C_EXPECTED_LINES_PER_LANE * C_HSIZE_BYTES /
        (G_OUTPUT_WIDTH / 8);

    function fn_lane_profile return gpx_vdma_lane_profile_t is
        variable result : gpx_vdma_lane_profile_t :=
            C_GPX_VDMA_LANE_PROFILE_IDLE;
    begin
        result.valid := '1';
        result.enabled := '1';
        result.slot_count := to_unsigned(
            C_LANE_SLOT_COUNT, result.slot_count'length);
        result.visible_returns := to_unsigned(
            G_VISIBLE_RETURNS, result.visible_returns'length);
        result.cell_word_count := to_unsigned(
            fn_gpx_vdma_cell_word_count(G_VISIBLE_RETURNS),
            result.cell_word_count'length);
        result.planned_shots := to_unsigned(
            C_COLUMNS_PER_FACE, result.planned_shots'length);
        result.raw_line_words := to_unsigned(
            fn_gpx_vdma_shot_raw_hsize_bytes(
                C_LANE_SLOT_COUNT, G_VISIBLE_RETURNS) / 4,
            result.raw_line_words'length);
        result.hsize_words := to_unsigned(
            C_HSIZE_BYTES / 4, result.hsize_words'length);
        result.hsize_bytes := to_unsigned(
            C_HSIZE_BYTES, result.hsize_bytes'length);
        result.footer_lines := to_unsigned(
            C_FOOTER_LINES, result.footer_lines'length);
        result.vsize_lines := to_unsigned(
            C_COLUMNS_PER_FACE + C_FOOTER_LINES,
            result.vsize_lines'length);
        result.stride_bytes := to_unsigned(
            fn_gpx_vdma_stride_bytes(
                C_CHIPS * C_STOPS_PER_CHIP,
                C_PHYSICAL_RETURNS, G_OUTPUT_WIDTH),
            result.stride_bytes'length);
        return result;
    end function fn_lane_profile;

    constant C_LANE_PROFILE : gpx_vdma_lane_profile_t :=
        fn_lane_profile;

    function fn_shot(column_index : natural) return shot_start_event_t is
        variable result : shot_start_event_t := C_SHOT_START_EVENT_IDLE;
    begin
        result.valid := '1';
        result.request.valid := '1';
        result.request.face_index := to_unsigned(2, 3);
        result.request.position := to_unsigned(
            3000 + column_index, C_POSITION_WIDTH);
        result.request.direction := DIRECTION_CW;
        result.request.shot_index := to_unsigned(column_index, 16);
        result.request.last_in_face := '0';
        result.request.source_sim := '0';
        result.request.source_latency_clks := to_unsigned(8, 8);
        result.request.source_latency_valid := '1';
        result.request.active_version := to_unsigned(21, 16);
        result.fire_to_t0_clks := to_unsigned(4, 32);
        -- 측정 시작 기준시점 (T0)은 동기화된 fire_done 승인과 start_tdc
        -- 발생 사건이다. 이 값은 TDC Drain 이후에도 같은 Shot identity로 간다.
        result.t0_timestamp_ticks := to_unsigned(
            16#123456#, result.t0_timestamp_ticks'length);
        result.t0_timestamp_valid := '1';
        result.t0_time_sync_valid := '1';
        return result;
    end function fn_shot;

    function fn_close(face_number : natural) return face_close_event_t is
        variable result : face_close_event_t := C_FACE_CLOSE_EVENT_IDLE;
    begin
        result.valid := '1';
        result.face_frame_id := to_unsigned(100 + face_number, 32);
        result.face_index := to_unsigned(face_number, 3);
        result.direction := DIRECTION_CW;
        result.source_sim := '0';
        result.active_version := to_unsigned(21, 16);
        result.columns_per_face := to_unsigned(C_COLUMNS_PER_FACE, 16);
        return result;
    end function fn_close;

    function fn_raw_word(
        chip_index : natural;
        ififo_id   : std_logic;
        word_index : natural
    ) return gpx_bus_data_t is
        variable result : gpx_bus_data_t := (others => '0');
        variable hit_value : natural;
        variable channel_index : natural;
    begin
        if G_ALL_DUAL then
            channel_index := (word_index / 2) mod 4;
        else
            channel_index := word_index mod 4;
        end if;
        result(c_RAW_CHACODE_HI downto c_RAW_CHACODE_LO) :=
            std_logic_vector(to_unsigned(channel_index, 2));
        result(c_RAW_STARTNUM_HI downto c_RAW_STARTNUM_LO) :=
            (others => '0');
        if G_ALL_DUAL and word_index mod 2 = 0 then
            result(c_RAW_SLOPE_BIT) := '1';
        elsif G_ALL_DUAL then
            result(c_RAW_SLOPE_BIT) := '0';
        elsif chip_index < 2 then
            result(c_RAW_SLOPE_BIT) := '1';
        else
            result(c_RAW_SLOPE_BIT) := '0';
        end if;
        hit_value := chip_index * 16#2000# + word_index;
        if ififo_id = '1' then
            hit_value := hit_value + 16#0800#;
        end if;
        result(c_RAW_HIT_HI downto c_RAW_HIT_LO) :=
            std_logic_vector(to_unsigned(hit_value, c_RAW_HIT_WIDTH));
        return result;
    end function fn_raw_word;

    function fn_all_initialized(value : gpx_lane_status_array_t)
        return boolean is
    begin
        for index in 0 to C_CHIPS - 1 loop
            if value(index).initialized /= '1' then
                return false;
            end if;
        end loop;
        return true;
    end function fn_all_initialized;

    function fn_all_running(value : gpx_lane_status_array_t)
        return boolean is
    begin
        for index in 0 to C_CHIPS - 1 loop
            if value(index).run_active /= '1' then
                return false;
            end if;
        end loop;
        return true;
    end function fn_all_running;

    function fn_all_shot_outstanding(value : gpx_lane_status_array_t)
        return boolean is
    begin
        for index in 0 to C_CHIPS - 1 loop
            if value(index).shot_outstanding /= '1' then
                return false;
            end if;
        end loop;
        return true;
    end function fn_all_shot_outstanding;

    function fn_capture_file_name(lane_rise : boolean) return string is
    begin
        if G_USE_V3 and lane_rise then
            return "v3_rise_capture.txt";
        elsif G_USE_V3 then
            return "v3_fall_capture.txt";
        elsif lane_rise then
            return "v2_rise_capture.txt";
        end if;
        return "v2_fall_capture.txt";
    end function fn_capture_file_name;

    type natural_array_t is array (0 to C_CHIPS - 1) of
        natural range 0 to 65535;
    type logic_array_t is array (0 to C_CHIPS - 1) of std_logic;

    signal proc_clk : std_logic := '0';
    signal tdc_clk : std_logic := '0';
    signal tdc_clk_async : std_logic := '0';
    signal proc_rst_n : std_logic := '0';
    signal tdc_rst_n : std_logic := '0';
    signal simulation_done : boolean := false;

    signal proc_shot : shot_start_event_t := C_SHOT_START_EVENT_IDLE;
    signal proc_shot_ready : std_logic;
    signal proc_stop_tdc : std_logic := '0';
    signal face_close : face_close_event_t := C_FACE_CLOSE_EVENT_IDLE;
    signal face_close_ready : std_logic;

    signal rise_tdata : std_logic_vector(G_OUTPUT_WIDTH - 1 downto 0);
    signal rise_tkeep : std_logic_vector(G_OUTPUT_WIDTH / 8 - 1 downto 0);
    signal rise_tstrb : std_logic_vector(G_OUTPUT_WIDTH / 8 - 1 downto 0);
    signal rise_tuser : std_logic_vector(0 downto 0);
    signal rise_tvalid : std_logic;
    signal rise_tlast : std_logic;
    signal rise_tready : std_logic := '0';
    signal fall_tdata : std_logic_vector(G_OUTPUT_WIDTH - 1 downto 0);
    signal fall_tkeep : std_logic_vector(G_OUTPUT_WIDTH / 8 - 1 downto 0);
    signal fall_tstrb : std_logic_vector(G_OUTPUT_WIDTH / 8 - 1 downto 0);
    signal fall_tuser : std_logic_vector(0 downto 0);
    signal fall_tvalid : std_logic;
    signal fall_tlast : std_logic;
    signal fall_tready : std_logic := '0';

    signal shot_done : std_logic;
    signal shot_done_context : shot_start_event_t;
    signal frame_output_done : std_logic;
    signal proc_idle : std_logic;
    signal outstanding_shots : unsigned(15 downto 0);
    signal decoder_inflight : unsigned(7 downto 0) := (others => '0');

    signal config_apply : std_logic := '0';
    signal config_ready : std_logic;
    signal config_done : std_logic;
    signal run_enable : std_logic := '0';
    signal safe : std_logic;
    signal cdc_reset_busy : std_logic;

    signal adr : gpx_bus_address_array_t;
    signal csn : chip_mask_t;
    signal rdn : chip_mask_t;
    signal wrn : chip_mask_t;
    signal oen : chip_mask_t;
    signal d_from_dut : gpx_bus_data_array_t;
    signal d_tri : gpx_bus_data_array_t;
    signal d_bus : gpx_bus_data_array_t := (others => (others => 'Z'));
    signal chip_d_out : gpx_bus_data_array_t := (others => (others => '0'));
    signal chip_d_oe : chip_mask_t := (others => '0');
    signal ef1 : chip_mask_t;
    signal ef2 : chip_mask_t;
    signal lf1 : chip_mask_t;
    signal lf2 : chip_mask_t;
    signal irflag : chip_mask_t := (others => '0');
    signal errflag : chip_mask_t := (others => '0');
    signal stopdis : chip_mask_t;
    signal alutrigger : chip_mask_t;
    signal puresn : chip_mask_t;
    signal active_mask : chip_mask_t;
    signal terminal_mask : chip_mask_t;
    signal tdc_status : gpx_lane_status_array_t;
    signal tdc_faults : gpx_lane_faults_array_t;

    signal shot_drop_sticky : std_logic;
    signal stop_drop_sticky : std_logic;
    signal context_fault_sticky : std_logic;
    signal hit_fault_sticky : gpx_hit_decoder_faults_t;
    signal cell_fault_sticky : gpx_cell_collector_faults_t;
    signal frame_fault_sticky : gpx_frame_assembler_faults_t;
    signal rise_formatter_fault_sticky :
        lidar_gpx_word_formatter_faults_t :=
            C_LIDAR_GPX_WORD_FORMATTER_FAULTS_CLEAR;
    signal fall_formatter_fault_sticky :
        lidar_gpx_word_formatter_faults_t :=
            C_LIDAR_GPX_WORD_FORMATTER_FAULTS_CLEAR;

    signal fifo1_fill : natural_array_t := (others => 0);
    signal fifo2_fill : natural_array_t := (others => 0);
    signal fifo1_read_index : natural_array_t := (others => 0);
    signal fifo2_read_index : natural_array_t := (others => 0);
    signal fifo_load : std_logic := '0';

    signal sink_cycle_r : natural range 0 to 31 := 0;
    signal force_sink_stall : std_logic := '0';
    signal rise_beat_count_r : natural := 0;
    signal fall_beat_count_r : natural := 0;
    signal rise_line_count_r : natural := 0;
    signal fall_line_count_r : natural := 0;
    signal rise_sof_count_r : natural := 0;
    signal fall_sof_count_r : natural := 0;
    signal shot_done_count_r : natural := 0;
    signal frame_done_count_r : natural := 0;
    signal max_decoder_inflight_r : natural := 0;

    signal v2_rise_event : gpx_frame_cell_event_t;
    signal v2_fall_event : gpx_frame_cell_event_t;
    signal v2_frame_close_event : gpx_frame_close_event_t;
    signal v2_rise_ready : std_logic;
    signal v2_fall_ready : std_logic;
    signal v2_frame_close_ready : std_logic;
    signal v2_proc_idle : std_logic;
    signal v2_axis_idle : std_logic;

    file rise_capture_file : text open write_mode is
        fn_capture_file_name(true);
    file fall_capture_file : text open write_mode is
        fn_capture_file_name(false);

begin

    assert G_OUTPUT_WIDTH = 32 or G_OUTPUT_WIDTH = 64
        report "V3-H6-TB-001 output width must be 32 or 64"
        severity failure;

    proc_clk <= not proc_clk after C_PROC_PERIOD / 2
        when not simulation_done;

    gen_async_clocks : if G_PROC_CLK_MHZ /= G_TDC_CLK_MHZ generate
        tdc_clk_async <= not tdc_clk_async after C_TDC_PERIOD / 2
            when not simulation_done;
        tdc_clk <= tdc_clk_async;
    end generate gen_async_clocks;

    gen_sync_clocks : if G_PROC_CLK_MHZ = G_TDC_CLK_MHZ generate
        -- STREAM_CLOCK_SYNC 계약은 두 포트가 같은 물리 clock net을 쓴다.
        tdc_clk <= proc_clk;
    end generate gen_sync_clocks;

    gen_pin_models : for index in 0 to C_CHIPS - 1 generate
        ef1(index) <= '1' when fifo1_fill(index) = 0 else '0';
        ef2(index) <= '1' when fifo2_fill(index) = 0 else '0';
        lf1(index) <= '1' when fifo1_fill(index) >= 2 else '0';
        lf2(index) <= '1' when fifo2_fill(index) >= 2 else '0';

        d_bus(index) <= d_from_dut(index)
            when d_tri(index)(0) = '0' else (others => 'Z');
        d_bus(index) <= chip_d_out(index)
            when chip_d_oe(index) = '1' else (others => 'Z');
    end generate gen_pin_models;

    gen_v2_golden : if not G_USE_V3 generate
        signal v2_frame_output_done_s : std_logic;
    begin
        decoder_inflight <= (others => '0');
        rise_formatter_fault_sticky <=
            C_LIDAR_GPX_WORD_FORMATTER_FAULTS_CLEAR;
        fall_formatter_fault_sticky <=
            C_LIDAR_GPX_WORD_FORMATTER_FAULTS_CLEAR;
        proc_idle <= v2_proc_idle and v2_axis_idle;
        frame_output_done <= v2_frame_output_done_s;

        u_v2_b5_b8 : entity work.lidar_gpx_b5_b8_subsystem
            generic map (
                G_BUILD_CONFIG => C_BUILD_CONFIG,
                G_OEN_MODE => "DYNAMIC_CONNECTED",
                G_DRAIN_MARGIN_TIME_NS => C_DRAIN_MARGIN_TIME_NS
            )
            port map (
                i_proc_clk => proc_clk,
                i_proc_rst_n => proc_rst_n,
                i_proc_abort => '0',
                i_proc_clear_status => '0',
                i_proc_active_valid => '1',
                i_proc_active_config => C_ACTIVE_CONFIG,
                i_proc_shot => proc_shot,
                o_proc_shot_ready => proc_shot_ready,
                i_proc_stop_tdc => proc_stop_tdc,
                i_face_close_event => face_close,
                o_face_close_ready => face_close_ready,
                o_rise_event => v2_rise_event,
                i_rise_ready => v2_rise_ready,
                o_fall_event => v2_fall_event,
                i_fall_ready => v2_fall_ready,
                o_frame_close_event => v2_frame_close_event,
                i_frame_close_ready => v2_frame_close_ready,
                i_frame_output_done => v2_frame_output_done_s,
                o_rise_line_done => open,
                o_fall_line_done => open,
                o_shot_done => shot_done,
                o_shot_done_context => shot_done_context,
                o_proc_idle => v2_proc_idle,
                o_outstanding_shots => outstanding_shots,
                o_context_fault_sticky => context_fault_sticky,
                i_tdc_clk => tdc_clk,
                i_tdc_rst_n => tdc_rst_n,
                i_tdc_active_valid => '1',
                i_tdc_active_config => C_ACTIVE_CONFIG,
                i_tdc_register_image => C_GPX_REGISTER_IMAGE_DEFAULT,
                i_tdc_config_apply => config_apply,
                o_tdc_config_ready => config_ready,
                o_tdc_config_done => config_done,
                i_tdc_run_enable => run_enable,
                o_tdc_safe => safe,
                o_tdc_shot_complete => open,
                i_tdc_register_read =>
                    C_GPX_REGISTER_READ_REQUEST_IDLE,
                o_tdc_register_read_ready => open,
                o_tdc_register_read_response => open,
                i_tdc_register_read_response_ready => '1',
                o_cdc_reset_busy => cdc_reset_busy,
                o_adr => adr,
                o_csn => csn,
                o_rdn => rdn,
                o_wrn => wrn,
                o_oen => oen,
                i_d => d_bus,
                o_d => d_from_dut,
                o_d_tri => d_tri,
                i_ef1 => ef1,
                i_ef2 => ef2,
                i_lf1 => lf1,
                i_lf2 => lf2,
                i_irflag => irflag,
                i_errflag => errflag,
                o_stopdis => stopdis,
                o_alutrigger => alutrigger,
                o_puresn => puresn,
                o_active_mask => active_mask,
                o_terminal_mask => terminal_mask,
                o_tdc_status => tdc_status,
                o_tdc_faults => tdc_faults,
                o_shot_drop_sticky => shot_drop_sticky,
                o_stop_drop_sticky => stop_drop_sticky,
                o_hit_fault_pulse => open,
                o_hit_fault_sticky => hit_fault_sticky,
                o_cell_fault_pulse => open,
                o_cell_fault_sticky => cell_fault_sticky,
                o_frame_fault_pulse => open,
                o_frame_fault_sticky => frame_fault_sticky
            );

        u_v2_axis : entity work.lidar_gpx_axis_output_subsystem
            generic map (
                G_BUILD_CONFIG => C_BUILD_CONFIG
            )
            port map (
                i_clk => proc_clk,
                i_rst_n => proc_rst_n,
                i_abort => '0',
                i_rise_active_profile => C_LANE_PROFILE,
                i_fall_active_profile => C_LANE_PROFILE,
                i_rise_event => v2_rise_event,
                o_rise_ready => v2_rise_ready,
                i_fall_event => v2_fall_event,
                o_fall_ready => v2_fall_ready,
                i_frame_close_event => v2_frame_close_event,
                o_frame_close_ready => v2_frame_close_ready,
                o_frame_output_done => v2_frame_output_done_s,
                o_rise_tdata => rise_tdata,
                o_rise_tkeep => rise_tkeep,
                o_rise_tstrb => rise_tstrb,
                o_rise_tuser => rise_tuser,
                o_rise_tvalid => rise_tvalid,
                o_rise_tlast => rise_tlast,
                i_rise_tready => rise_tready,
                o_fall_tdata => fall_tdata,
                o_fall_tkeep => fall_tkeep,
                o_fall_tstrb => fall_tstrb,
                o_fall_tuser => fall_tuser,
                o_fall_tvalid => fall_tvalid,
                o_fall_tlast => fall_tlast,
                i_fall_tready => fall_tready,
                o_rise_line_done => open,
                o_fall_line_done => open,
                o_rise_frame_done => open,
                o_fall_frame_done => open,
                o_idle => v2_axis_idle
            );
    end generate gen_v2_golden;

    gen_v3_hls : if G_USE_V3 generate
    begin
        u_v3_parent_data : entity work.lidar_gpx_hls_parent_data_subsystem
            generic map (
                G_BUILD_CONFIG => C_BUILD_CONFIG,
                G_OEN_MODE => "DYNAMIC_CONNECTED",
                G_DRAIN_MARGIN_TIME_NS => C_DRAIN_MARGIN_TIME_NS
            )
            port map (
                i_proc_clk => proc_clk,
                i_proc_rst_n => proc_rst_n,
                i_proc_abort => '0',
                i_proc_clear_status => '0',
                i_proc_active_valid => '1',
                i_proc_active_config => C_ACTIVE_CONFIG,
                i_proc_shot => proc_shot,
                o_proc_shot_ready => proc_shot_ready,
                i_proc_stop_tdc => proc_stop_tdc,
                i_face_close_event => face_close,
                o_face_close_ready => face_close_ready,
                i_rise_active_profile => C_LANE_PROFILE,
                i_fall_active_profile => C_LANE_PROFILE,
                o_rise_tdata => rise_tdata,
                o_rise_tkeep => rise_tkeep,
                o_rise_tstrb => rise_tstrb,
                o_rise_tuser => rise_tuser,
                o_rise_tvalid => rise_tvalid,
                o_rise_tlast => rise_tlast,
                i_rise_tready => rise_tready,
                o_fall_tdata => fall_tdata,
                o_fall_tkeep => fall_tkeep,
                o_fall_tstrb => fall_tstrb,
                o_fall_tuser => fall_tuser,
                o_fall_tvalid => fall_tvalid,
                o_fall_tlast => fall_tlast,
                i_fall_tready => fall_tready,
                o_shot_done => shot_done,
                o_shot_done_context => shot_done_context,
                o_frame_output_done => frame_output_done,
                o_proc_idle => proc_idle,
                o_outstanding_shots => outstanding_shots,
                o_decoder_inflight => decoder_inflight,
                o_rise_emitted_lines => open,
                o_fall_emitted_lines => open,
                o_context_fault_sticky => context_fault_sticky,
                i_tdc_clk => tdc_clk,
                i_tdc_rst_n => tdc_rst_n,
                i_tdc_active_valid => '1',
                i_tdc_active_config => C_ACTIVE_CONFIG,
                i_tdc_register_image => C_GPX_REGISTER_IMAGE_DEFAULT,
                i_tdc_config_apply => config_apply,
                o_tdc_config_ready => config_ready,
                o_tdc_config_done => config_done,
                i_tdc_run_enable => run_enable,
                o_tdc_safe => safe,
                o_tdc_shot_complete => open,
                i_tdc_register_read =>
                    C_GPX_REGISTER_READ_REQUEST_IDLE,
                o_tdc_register_read_ready => open,
                o_tdc_register_read_response => open,
                i_tdc_register_read_response_ready => '1',
                o_cdc_reset_busy => cdc_reset_busy,
                o_adr => adr,
                o_csn => csn,
                o_rdn => rdn,
                o_wrn => wrn,
                o_oen => oen,
                i_d => d_bus,
                o_d => d_from_dut,
                o_d_tri => d_tri,
                i_ef1 => ef1,
                i_ef2 => ef2,
                i_lf1 => lf1,
                i_lf2 => lf2,
                i_irflag => irflag,
                i_errflag => errflag,
                o_stopdis => stopdis,
                o_alutrigger => alutrigger,
                o_puresn => puresn,
                o_active_mask => active_mask,
                o_terminal_mask => terminal_mask,
                o_tdc_status => tdc_status,
                o_tdc_faults => tdc_faults,
                o_shot_drop_sticky => shot_drop_sticky,
                o_stop_drop_sticky => stop_drop_sticky,
                o_hit_fault_sticky => hit_fault_sticky,
                o_cell_fault_sticky => cell_fault_sticky,
                o_frame_fault_sticky => frame_fault_sticky,
                o_rise_formatter_fault_sticky =>
                    rise_formatter_fault_sticky,
                o_fall_formatter_fault_sticky =>
                    fall_formatter_fault_sticky
            );
    end generate gen_v3_hls;

    p_chip_models : process (tdc_clk)
        variable rdn_previous_v : logic_array_t := (others => '1');
    begin
        if rising_edge(tdc_clk) then
            chip_d_oe <= (others => '0');
            if tdc_rst_n = '0' then
                fifo1_fill <= (others => 0);
                fifo2_fill <= (others => 0);
                fifo1_read_index <= (others => 0);
                fifo2_read_index <= (others => 0);
                rdn_previous_v := (others => '1');
            else
                for index in 0 to C_CHIPS - 1 loop
                    if fifo_load = '1' then
                        fifo1_fill(index) <= C_WORDS_PER_IFIFO;
                        fifo2_fill(index) <= C_WORDS_PER_IFIFO;
                        fifo1_read_index(index) <= 0;
                        fifo2_read_index(index) <= 0;
                    end if;

                    if oen(index) = '0' and rdn(index) = '0' then
                        chip_d_oe(index) <= '1';
                        if adr(index) = c_TDC_REG8_IFIFO1 then
                            chip_d_out(index) <= fn_raw_word(
                                index, '0', fifo1_read_index(index));
                        elsif adr(index) = c_TDC_REG9_IFIFO2 then
                            chip_d_out(index) <= fn_raw_word(
                                index, '1', fifo2_read_index(index));
                        else
                            chip_d_out(index) <= (others => '0');
                        end if;
                    end if;

                    if rdn(index) = '1' and rdn_previous_v(index) = '0' then
                        if adr(index) = c_TDC_REG8_IFIFO1 and
                           fifo1_fill(index) > 0 then
                            fifo1_fill(index) <= fifo1_fill(index) - 1;
                            fifo1_read_index(index) <=
                                fifo1_read_index(index) + 1;
                        elsif adr(index) = c_TDC_REG9_IFIFO2 and
                              fifo2_fill(index) > 0 then
                            fifo2_fill(index) <= fifo2_fill(index) - 1;
                            fifo2_read_index(index) <=
                                fifo2_read_index(index) + 1;
                        end if;
                    end if;
                    rdn_previous_v(index) := rdn(index);
                end loop;
            end if;
        end if;
    end process p_chip_models;

    p_axis_ready : process (proc_clk)
    begin
        if rising_edge(proc_clk) then
            if proc_rst_n = '0' then
                sink_cycle_r <= 0;
                rise_tready <= '0';
                fall_tready <= '0';
            else
                if sink_cycle_r = 15 then
                    sink_cycle_r <= 0;
                else
                    sink_cycle_r <= sink_cycle_r + 1;
                end if;
                if force_sink_stall = '1' then
                    rise_tready <= '0';
                    fall_tready <= '0';
                else
                    if sink_cycle_r = 2 or sink_cycle_r = 3 or
                       sink_cycle_r = 11 then
                        rise_tready <= '0';
                    else
                        rise_tready <= '1';
                    end if;
                    if sink_cycle_r = 5 or sink_cycle_r = 8 or
                       sink_cycle_r = 9 then
                        fall_tready <= '0';
                    else
                        fall_tready <= '1';
                    end if;
                end if;
            end if;
        end if;
    end process p_axis_ready;

    p_capture : process (proc_clk)
        variable output_line_v : line;
        variable rise_hold_valid_v : boolean := false;
        variable fall_hold_valid_v : boolean := false;
        variable rise_hold_data_v : std_logic_vector(
            G_OUTPUT_WIDTH - 1 downto 0);
        variable fall_hold_data_v : std_logic_vector(
            G_OUTPUT_WIDTH - 1 downto 0);
        variable rise_hold_keep_v : std_logic_vector(
            G_OUTPUT_WIDTH / 8 - 1 downto 0);
        variable fall_hold_keep_v : std_logic_vector(
            G_OUTPUT_WIDTH / 8 - 1 downto 0);
        variable rise_hold_user_v : std_logic_vector(0 downto 0);
        variable fall_hold_user_v : std_logic_vector(0 downto 0);
        variable rise_hold_last_v : std_logic;
        variable fall_hold_last_v : std_logic;
        variable inflight_v : natural;
    begin
        if rising_edge(proc_clk) then
            if proc_rst_n = '0' then
                rise_beat_count_r <= 0;
                fall_beat_count_r <= 0;
                rise_line_count_r <= 0;
                fall_line_count_r <= 0;
                rise_sof_count_r <= 0;
                fall_sof_count_r <= 0;
                shot_done_count_r <= 0;
                frame_done_count_r <= 0;
                max_decoder_inflight_r <= 0;
                rise_hold_valid_v := false;
                fall_hold_valid_v := false;
            else
                inflight_v := to_integer(decoder_inflight);
                if inflight_v > max_decoder_inflight_r then
                    max_decoder_inflight_r <= inflight_v;
                end if;

                if shot_done = '1' then
                    assert shot_done_context = fn_shot(1)
                        report "V3-H6-TB-002 Shot identity mismatch"
                        severity failure;
                    shot_done_count_r <= shot_done_count_r + 1;
                end if;
                if frame_output_done = '1' then
                    frame_done_count_r <= frame_done_count_r + 1;
                end if;

                if rise_hold_valid_v then
                    assert rise_tvalid = '1' and
                           rise_tdata = rise_hold_data_v and
                           rise_tkeep = rise_hold_keep_v and
                           rise_tstrb = rise_hold_keep_v and
                           rise_tuser = rise_hold_user_v and
                           rise_tlast = rise_hold_last_v
                        report "V3-H6-TB-003 Rise payload changed under stall"
                        severity failure;
                end if;
                if fall_hold_valid_v then
                    assert fall_tvalid = '1' and
                           fall_tdata = fall_hold_data_v and
                           fall_tkeep = fall_hold_keep_v and
                           fall_tstrb = fall_hold_keep_v and
                           fall_tuser = fall_hold_user_v and
                           fall_tlast = fall_hold_last_v
                        report "V3-H6-TB-004 Fall payload changed under stall"
                        severity failure;
                end if;

                rise_hold_valid_v := rise_tvalid = '1' and
                    rise_tready = '0';
                if rise_hold_valid_v then
                    rise_hold_data_v := rise_tdata;
                    rise_hold_keep_v := rise_tkeep;
                    rise_hold_user_v := rise_tuser;
                    rise_hold_last_v := rise_tlast;
                end if;
                fall_hold_valid_v := fall_tvalid = '1' and
                    fall_tready = '0';
                if fall_hold_valid_v then
                    fall_hold_data_v := fall_tdata;
                    fall_hold_keep_v := fall_tkeep;
                    fall_hold_user_v := fall_tuser;
                    fall_hold_last_v := fall_tlast;
                end if;

                if rise_tvalid = '1' and rise_tready = '1' then
                    assert rise_tkeep = (rise_tkeep'range => '1') and
                           rise_tstrb = (rise_tstrb'range => '1')
                        report "V3-H6-TB-005 Rise byte qualifier mismatch"
                        severity failure;
                    write(output_line_v, string'("DATA="));
                    hwrite(output_line_v, rise_tdata);
                    write(output_line_v, string'(" KEEP="));
                    hwrite(output_line_v, rise_tkeep);
                    write(output_line_v, string'(" USER="));
                    hwrite(output_line_v, rise_tuser);
                    write(output_line_v, string'(" LAST="));
                    write(output_line_v, rise_tlast);
                    writeline(rise_capture_file, output_line_v);
                    rise_beat_count_r <= rise_beat_count_r + 1;
                    if rise_tlast = '1' then
                        rise_line_count_r <= rise_line_count_r + 1;
                    end if;
                    if rise_tuser(0) = '1' then
                        rise_sof_count_r <= rise_sof_count_r + 1;
                    end if;
                end if;

                if fall_tvalid = '1' and fall_tready = '1' then
                    assert fall_tkeep = (fall_tkeep'range => '1') and
                           fall_tstrb = (fall_tstrb'range => '1')
                        report "V3-H6-TB-006 Fall byte qualifier mismatch"
                        severity failure;
                    write(output_line_v, string'("DATA="));
                    hwrite(output_line_v, fall_tdata);
                    write(output_line_v, string'(" KEEP="));
                    hwrite(output_line_v, fall_tkeep);
                    write(output_line_v, string'(" USER="));
                    hwrite(output_line_v, fall_tuser);
                    write(output_line_v, string'(" LAST="));
                    write(output_line_v, fall_tlast);
                    writeline(fall_capture_file, output_line_v);
                    fall_beat_count_r <= fall_beat_count_r + 1;
                    if fall_tlast = '1' then
                        fall_line_count_r <= fall_line_count_r + 1;
                    end if;
                    if fall_tuser(0) = '1' then
                        fall_sof_count_r <= fall_sof_count_r + 1;
                    end if;
                end if;
            end if;
        end if;
    end process p_capture;

    p_stimulus : process
        procedure wait_proc_clocks(count : positive) is
        begin
            for index in 1 to count loop
                wait until rising_edge(proc_clk);
            end loop;
            wait for 1 ps;
        end procedure wait_proc_clocks;

        procedure wait_tdc_clocks(count : positive) is
        begin
            for index in 1 to count loop
                wait until rising_edge(tdc_clk);
            end loop;
            wait for 1 ps;
        end procedure wait_tdc_clocks;

        procedure send_close(face_number : natural) is
        begin
            face_close <= fn_close(face_number);
            for timeout in 0 to 600000 loop
                wait_proc_clocks(1);
                exit when face_close_ready = '1';
            end loop;
            assert face_close_ready = '1'
                report "V3-H6-TB-007 Face-close acknowledgement timeout"
                severity failure;
            wait_proc_clocks(1);
            face_close <= C_FACE_CLOSE_EVENT_IDLE;
            wait_proc_clocks(2);
        end procedure send_close;

        variable frame_done_before_v : natural;
    begin
        proc_rst_n <= '0';
        tdc_rst_n <= '0';
        wait_proc_clocks(8);
        wait_tdc_clocks(8);
        proc_rst_n <= '1';
        tdc_rst_n <= '1';

        for timeout in 0 to 200000 loop
            wait_tdc_clocks(1);
            exit when fn_all_initialized(tdc_status) and
                cdc_reset_busy = '0';
        end loop;
        assert fn_all_initialized(tdc_status) and cdc_reset_busy = '0'
            report "V3-H6-TB-008 initialization timeout"
            severity failure;

        for timeout in 0 to 20000 loop
            wait_tdc_clocks(1);
            exit when config_ready = '1';
        end loop;
        assert config_ready = '1'
            report "V3-H6-TB-009 config-ready timeout"
            severity failure;
        config_apply <= '1';
        wait_tdc_clocks(1);
        config_apply <= '0';
        for timeout in 0 to 20000 loop
            wait_tdc_clocks(1);
            exit when config_done = '1';
        end loop;
        assert config_done = '1'
            report "V3-H6-TB-010 config-done timeout"
            severity failure;

        run_enable <= '1';
        for timeout in 0 to 200000 loop
            wait_tdc_clocks(1);
            exit when fn_all_running(tdc_status);
        end loop;
        assert fn_all_running(tdc_status)
            report "V3-H6-TB-011 run arm timeout"
            severity failure;

        fifo_load <= '1';
        wait_tdc_clocks(1);
        fifo_load <= '0';
        wait_tdc_clocks(4);

        for timeout in 0 to 10000 loop
            exit when proc_shot_ready = '1';
            wait_proc_clocks(1);
        end loop;
        assert proc_shot_ready = '1'
            report "V3-H6-TB-012 Shot ingress timeout"
            severity failure;
        proc_shot <= fn_shot(1);
        wait_proc_clocks(1);
        proc_shot <= C_SHOT_START_EVENT_IDLE;

        for timeout in 0 to 20000 loop
            wait_tdc_clocks(1);
            exit when fn_all_shot_outstanding(tdc_status);
        end loop;
        assert fn_all_shot_outstanding(tdc_status)
            report "V3-H6-TB-013 Shot was not broadcast to four Chips"
            severity failure;

        irflag <= (others => '1');
        wait_tdc_clocks(6);
        irflag <= (others => '0');
        proc_stop_tdc <= '1';
        wait_proc_clocks(2);
        proc_stop_tdc <= '0';

        for timeout in 0 to 1000000 loop
            wait_proc_clocks(1);
            exit when shot_done_count_r = 1 and
                outstanding_shots = 0;
        end loop;
        assert shot_done_count_r = 1 and outstanding_shots = 0
            report "V3-H6-TB-014 physical Drain/H5 Shot closure timeout"
            severity failure;

        -- Footer가 AXI에서 승인되기 전에는 Face-close ACK가 나가면 안 된다.
        force_sink_stall <= '1';
        wait_proc_clocks(2);
        frame_done_before_v := frame_done_count_r;
        face_close <= fn_close(2);
        wait_proc_clocks(20);
        assert face_close_ready = '0' and
               frame_done_count_r = frame_done_before_v
            report "V3-H6-TB-015 Face ACK escaped AXI backpressure"
            severity failure;
        force_sink_stall <= '0';
        for timeout in 0 to 1000000 loop
            wait_proc_clocks(1);
            exit when face_close_ready = '1';
        end loop;
        assert face_close_ready = '1'
            report "V3-H6-TB-016 stalled Face did not complete"
            severity failure;
        wait_proc_clocks(1);
        face_close <= C_FACE_CLOSE_EVENT_IDLE;
        wait_proc_clocks(2);

        -- Shot이 전혀 없는 다음 Face도 Hole Line과 Footer로 닫혀야 한다.
        send_close(3);

        for timeout in 0 to 1000000 loop
            wait_proc_clocks(1);
            exit when frame_done_count_r = 2 and proc_idle = '1';
        end loop;
        wait_proc_clocks(4);

        assert frame_done_count_r = 2 and proc_idle = '1' and
               rise_line_count_r = C_EXPECTED_LINES_PER_LANE and
               fall_line_count_r = C_EXPECTED_LINES_PER_LANE and
               rise_beat_count_r = C_EXPECTED_BEATS_PER_LANE and
               fall_beat_count_r = C_EXPECTED_BEATS_PER_LANE and
               rise_sof_count_r = 2 and fall_sof_count_r = 2
            report "V3-H6-TB-017 AXI geometry/completion mismatch"
            severity failure;
        assert shot_drop_sticky = '0' and stop_drop_sticky = '0' and
               context_fault_sticky = '0'
            report "V3-H6-TB-018 ordering or CDC drop fault"
            severity failure;
        assert hit_fault_sticky = C_GPX_HIT_DECODER_FAULTS_CLEAR and
               cell_fault_sticky = C_GPX_CELL_COLLECTOR_FAULTS_CLEAR
            report "V3-H6-TB-019 Hit/Cell fault mismatch"
            severity failure;
        assert frame_fault_sticky.context_mismatch = '0' and
               frame_fault_sticky.unexpected_cell = '0' and
               frame_fault_sticky.duplicate_cell = '0' and
               frame_fault_sticky.duplicate_terminal = '0' and
               frame_fault_sticky.missing_cell = '0' and
               frame_fault_sticky.geometry_error = '0' and
               frame_fault_sticky.column_gap = '1' and
               frame_fault_sticky.masked_payload_drop = '0'
            report "V3-H6-TB-020 Frame diagnostic mismatch"
            severity failure;
        assert rise_formatter_fault_sticky =
                   C_LIDAR_GPX_WORD_FORMATTER_FAULTS_CLEAR and
               fall_formatter_fault_sticky =
                   C_LIDAR_GPX_WORD_FORMATTER_FAULTS_CLEAR
            report "V3-H6-TB-021 H4 formatter fault mismatch"
            severity failure;
        assert active_mask = "1111" and terminal_mask = "1111"
            report "V3-H6-TB-022 active/terminal mask mismatch"
            severity failure;

        report "LIDAR_V3_H6_PARENT_DATA_DIFF_CAPTURE_PASS use_v3=" &
            boolean'image(G_USE_V3) & " proc_mhz=" &
            integer'image(G_PROC_CLK_MHZ) & " tdc_mhz=" &
            integer'image(G_TDC_CLK_MHZ) & " width=" &
            integer'image(G_OUTPUT_WIDTH) & " all_dual=" &
            boolean'image(G_ALL_DUAL) & " visible_returns=" &
            integer'image(G_VISIBLE_RETURNS) & " decoder_inflight_max=" &
            integer'image(max_decoder_inflight_r)
            severity note;
        simulation_done <= true;
        stop;
        wait;
    end process p_stimulus;

end architecture sim;

-- 1:4 극단 비동기: Processing 50 MHz, TDC 200 MHz.
library ieee;
use ieee.std_logic_1164.all;
entity tb_lidar_gpx_h6_parent_v2_p50_t200_w32 is end entity;
architecture sim of tb_lidar_gpx_h6_parent_v2_p50_t200_w32 is begin
    u : entity work.tb_lidar_gpx_hls_parent_data_subsystem_diff
        generic map (false, 50, 200, 32, false, 1);
end architecture;
library ieee;
use ieee.std_logic_1164.all;
entity tb_lidar_gpx_h6_parent_v3_p50_t200_w32 is end entity;
architecture sim of tb_lidar_gpx_h6_parent_v3_p50_t200_w32 is begin
    u : entity work.tb_lidar_gpx_hls_parent_data_subsystem_diff
        generic map (true, 50, 200, 32, false, 1);
end architecture;

-- 4:1 극단 비동기: Processing 200 MHz, TDC 50 MHz.
library ieee;
use ieee.std_logic_1164.all;
entity tb_lidar_gpx_h6_parent_v2_p200_t50_w64 is end entity;
architecture sim of tb_lidar_gpx_h6_parent_v2_p200_t50_w64 is begin
    u : entity work.tb_lidar_gpx_hls_parent_data_subsystem_diff
        generic map (false, 200, 50, 64, true, 7);
end architecture;
library ieee;
use ieee.std_logic_1164.all;
entity tb_lidar_gpx_h6_parent_v3_p200_t50_w64 is end entity;
architecture sim of tb_lidar_gpx_h6_parent_v3_p200_t50_w64 is begin
    u : entity work.tb_lidar_gpx_hls_parent_data_subsystem_diff
        generic map (true, 200, 50, 64, true, 7);
end architecture;

-- 1:1 동기: Processing/TDC가 같은 150 MHz 물리 clock net을 공유한다.
library ieee;
use ieee.std_logic_1164.all;
entity tb_lidar_gpx_h6_parent_v2_p150_t150_w32 is end entity;
architecture sim of tb_lidar_gpx_h6_parent_v2_p150_t150_w32 is begin
    u : entity work.tb_lidar_gpx_hls_parent_data_subsystem_diff
        generic map (false, 150, 150, 32, true, 7);
end architecture;
library ieee;
use ieee.std_logic_1164.all;
entity tb_lidar_gpx_h6_parent_v3_p150_t150_w32 is end entity;
architecture sim of tb_lidar_gpx_h6_parent_v3_p150_t150_w32 is begin
    u : entity work.tb_lidar_gpx_hls_parent_data_subsystem_diff
        generic map (true, 150, 150, 32, true, 7);
end architecture;

-- 제품 비동기 조합: Processing 150 MHz, TDC 200 MHz.
library ieee;
use ieee.std_logic_1164.all;
entity tb_lidar_gpx_h6_parent_v2_p150_t200_w32 is end entity;
architecture sim of tb_lidar_gpx_h6_parent_v2_p150_t200_w32 is begin
    u : entity work.tb_lidar_gpx_hls_parent_data_subsystem_diff
        generic map (false, 150, 200, 32, false, 1);
end architecture;
library ieee;
use ieee.std_logic_1164.all;
entity tb_lidar_gpx_h6_parent_v3_p150_t200_w32 is end entity;
architecture sim of tb_lidar_gpx_h6_parent_v3_p150_t200_w32 is begin
    u : entity work.tb_lidar_gpx_hls_parent_data_subsystem_diff
        generic map (true, 150, 200, 32, false, 1);
end architecture;

-- 역 제품 비동기 조합: Processing 200 MHz, TDC 150 MHz.
library ieee;
use ieee.std_logic_1164.all;
entity tb_lidar_gpx_h6_parent_v2_p200_t150_w64 is end entity;
architecture sim of tb_lidar_gpx_h6_parent_v2_p200_t150_w64 is begin
    u : entity work.tb_lidar_gpx_hls_parent_data_subsystem_diff
        generic map (false, 200, 150, 64, true, 7);
end architecture;
library ieee;
use ieee.std_logic_1164.all;
entity tb_lidar_gpx_h6_parent_v3_p200_t150_w64 is end entity;
architecture sim of tb_lidar_gpx_h6_parent_v3_p200_t150_w64 is begin
    u : entity work.tb_lidar_gpx_hls_parent_data_subsystem_diff
        generic map (true, 200, 150, 64, true, 7);
end architecture;
