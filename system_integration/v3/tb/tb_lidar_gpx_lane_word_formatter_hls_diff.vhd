-- ============================================================================
-- 검증 목적:
--   V2 RTL의 Cell serializer -> Shot Metadata -> Hole Line -> Face Footer ->
--   AXI Word packer 결과와 V3 H4 HLS formatter 결과를 Beat 단위로 비교한다.
--
-- 핵심 검증 범위:
--   * 합성 시 결정되는 32/64-bit AXI4-Stream 폭
--   * Runtime 직렬화(전시) Return 슬롯 수 1~7과 실제 유효 Return 수가 더 적은 Cell
--   * 최대 4 TDC-GPX Chip x 8 STOP = slope당 32 Cell slot
--   * leading/interior/trailing/all-hole Shot Line과 Face Footer
--   * PACKED17 Hit 하위 16 bit, Hit[16], Cell Metadata의 정확한 위치
--   * AXI backpressure 중 데이터/Line/Frame 경계 보존
--   * Line 중간 abort 후 Reset Epoch 복구 및 stale Word 제거
--
-- 유지보수 규칙:
--   두 경로의 Pipeline latency는 달라도 된다. 한쪽 출력이 먼저 도착하면
--   해당 출력을 정지시키고, 두 출력 Beat가 모두 준비된 시점에만 비교한다.
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.env.all;

use work.lidar_build_pkg.all;
use work.lidar_event_types_pkg.all;
use work.lidar_gpx_event_pkg.all;
use work.lidar_gpx_data_pkg.all;
use work.lidar_gpx_vdma_pkg.all;
use work.lidar_v3_hls_contract_pkg.all;

entity tb_lidar_gpx_lane_word_formatter_hls_diff is
    generic (
        G_CLK_MHZ      : positive := 150;
        G_OUTPUT_WIDTH : positive := 32;
        G_SLOT_COUNT   : positive range 1 to 32 := 1;
        G_RETURN_SLOTS : positive range 1 to 7 := 1;
        G_LANE_RISE    : boolean := true
    );
end entity tb_lidar_gpx_lane_word_formatter_hls_diff;

architecture sim of tb_lidar_gpx_lane_word_formatter_hls_diff is

    constant C_PLANNED_SHOTS : positive := 4;
    constant C_ACTIVE_VERSION : unsigned(15 downto 0) := x"0032";

    function fn_build_config return lidar_build_config_t is
        variable result : lidar_build_config_t := C_DEFAULT_BUILD_CONFIG;
    begin
        result.proc_clk_mhz := G_CLK_MHZ;
        result.tdc_clk_mhz := 200;
        result.stream_clock_mode := STREAM_CLOCK_ASYNC;
        result.num_chips := 4;
        result.stops_per_chip := 8;
        result.max_returns_per_stop := 7;
        result.rise_capability_mask := "1111";
        result.fall_capability_mask := "1111";
        result.output_width := G_OUTPUT_WIDTH;
        result.num_faces := 4;
        result.enable_echo_receiver := false;
        result.enable_echo_simulation := false;
        return result;
    end function fn_build_config;

    function fn_active_profile return gpx_vdma_lane_profile_t is
        variable result : gpx_vdma_lane_profile_t :=
            C_GPX_VDMA_LANE_PROFILE_IDLE;
        variable hsize_bytes : natural;
    begin
        hsize_bytes := fn_gpx_vdma_shot_hsize_bytes(
            G_SLOT_COUNT, G_RETURN_SLOTS, G_OUTPUT_WIDTH);
        result.valid := '1';
        result.enabled := '1';
        result.slot_count := to_unsigned(
            G_SLOT_COUNT, result.slot_count'length);
        result.visible_returns := to_unsigned(
            G_RETURN_SLOTS, result.visible_returns'length);
        result.cell_word_count := to_unsigned(
            fn_gpx_vdma_cell_word_count(G_RETURN_SLOTS),
            result.cell_word_count'length);
        result.planned_shots := to_unsigned(
            C_PLANNED_SHOTS, result.planned_shots'length);
        result.raw_line_words := to_unsigned(
            fn_gpx_vdma_shot_raw_hsize_bytes(
                G_SLOT_COUNT, G_RETURN_SLOTS) / 4,
            result.raw_line_words'length);
        result.hsize_bytes := to_unsigned(
            hsize_bytes, result.hsize_bytes'length);
        result.hsize_words := to_unsigned(
            hsize_bytes / 4, result.hsize_words'length);
        result.footer_lines := to_unsigned(
            fn_gpx_vdma_footer_lines(hsize_bytes),
            result.footer_lines'length);
        result.vsize_lines := to_unsigned(
            fn_gpx_vdma_vsize_lines(C_PLANNED_SHOTS, hsize_bytes),
            result.vsize_lines'length);
        result.stride_bytes := to_unsigned(
            fn_gpx_vdma_stride_bytes(32, 7, G_OUTPUT_WIDTH),
            result.stride_bytes'length);
        return result;
    end function fn_active_profile;

    function fn_context(
        shot_column : natural;
        face_index  : natural)
        return shot_start_event_t is
        variable result : shot_start_event_t := C_SHOT_START_EVENT_IDLE;
    begin
        result.valid := '1';
        result.request.valid := '1';
        result.request.face_index := to_unsigned(
            face_index, result.request.face_index'length);
        result.request.position := to_unsigned(
            1000 + shot_column * 3, result.request.position'length);
        result.request.direction := DIRECTION_CW;
        result.request.shot_index := to_unsigned(
            shot_column, result.request.shot_index'length);
        if shot_column + 1 = C_PLANNED_SHOTS then
            result.request.last_in_face := '1';
        end if;
        result.request.source_sim := '0';
        result.request.source_latency_clks := to_unsigned(
            5 + shot_column, result.request.source_latency_clks'length);
        result.request.source_latency_valid := '1';
        result.request.active_version := C_ACTIVE_VERSION;
        result.fire_to_t0_clks := to_unsigned(
            11 + shot_column, result.fire_to_t0_clks'length);
        result.t0_timestamp_ticks := to_unsigned(
            16#120000# + shot_column * 16#101#,
            result.t0_timestamp_ticks'length);
        result.t0_timestamp_valid := '1';
        result.t0_time_sync_valid := '1';
        return result;
    end function fn_context;

    function fn_cell_event(
        shot_column : natural;
        slot_index  : natural;
        gap_before  : natural;
        face_index  : natural)
        return gpx_frame_cell_event_t is
        variable result : gpx_frame_cell_event_t :=
            C_GPX_FRAME_CELL_EVENT_IDLE;
        variable visible_returns : positive range 1 to 7;
        variable hit_value : natural;
    begin
        visible_returns := G_RETURN_SLOTS;
        if G_RETURN_SLOTS > 1 and slot_index mod 3 = 0 then
            visible_returns := G_RETURN_SLOTS - 1;
        end if;

        result.valid := '1';
        result.cell.valid := '1';
        result.cell.kind := GPX_CELL_DATA;
        result.cell.chip_index := to_unsigned(
            slot_index / 8, result.cell.chip_index'length);
        result.cell.stop_index := to_unsigned(
            slot_index mod 8, result.cell.stop_index'length);
        if slot_index mod 8 >= 4 then
            result.cell.ififo_id := '1';
        end if;
        if G_LANE_RISE then
            result.cell.slope := GPX_SLOPE_RISE;
        else
            result.cell.slope := GPX_SLOPE_FALL;
        end if;
        result.cell.hit_count := to_unsigned(
            visible_returns, result.cell.hit_count'length);
        result.cell.max_hits := to_unsigned(
            G_RETURN_SLOTS, result.cell.max_hits'length);
        for return_index in 0 to visible_returns - 1 loop
            hit_value := (shot_column * 16384 + slot_index * 257 +
                          return_index * 17 + 1) mod (2 ** C_GPX_HIT_WIDTH);
            if (slot_index + return_index) mod 2 = 1 then
                hit_value := (hit_value mod 16#10000#) + 16#10000#;
            end if;
            result.cell.hits(return_index) := to_unsigned(
                hit_value, result.cell.hits(return_index)'length);
        end loop;
        if shot_column = 2 and slot_index = 1 then
            result.cell.error_fill := '1';
            result.cell.hit_dropped := '1';
            result.cell.return_overflow := '1';
            result.cell.faulted := '1';
            result.cell.timeout_cause := "101";
        end if;
        result.cell.shot_context := fn_context(shot_column, face_index);
        result.cell.chip_shot_seq := to_unsigned(
            16#20# + shot_column, result.cell.chip_shot_seq'length);

        result.slot_index := to_unsigned(
            slot_index, result.slot_index'length);
        result.slot_count := to_unsigned(
            G_SLOT_COUNT, result.slot_count'length);
        if slot_index = 0 then
            result.line_start := '1';
            result.gap_before := to_unsigned(
                gap_before, result.gap_before'length);
        end if;
        if slot_index + 1 = G_SLOT_COUNT then
            result.line_end := '1';
        end if;
        if shot_column = 0 then
            result.first_column := '1';
        end if;
        if shot_column + 1 = C_PLANNED_SHOTS then
            result.last_column := '1';
        end if;
        if shot_column = 2 then
            result.line_faulted := '1';
        end if;
        return result;
    end function fn_cell_event;

    function fn_close_event(
        frame_id    : natural;
        face_index  : natural;
        trailing_gap : natural;
        all_hole    : boolean)
        return gpx_frame_close_event_t is
        variable result : gpx_frame_close_event_t :=
            C_GPX_FRAME_CLOSE_EVENT_IDLE;
    begin
        result.valid := '1';
        result.face_frame_id := to_unsigned(
            frame_id, result.face_frame_id'length);
        result.face_index := to_unsigned(
            face_index, result.face_index'length);
        result.direction := DIRECTION_CW;
        result.source_sim := '0';
        result.active_version := C_ACTIVE_VERSION;
        result.columns_per_face := to_unsigned(
            C_PLANNED_SHOTS, result.columns_per_face'length);
        result.trailing_gap := to_unsigned(
            trailing_gap, result.trailing_gap'length);
        if all_hole then
            result.all_hole := '1';
        end if;
        return result;
    end function fn_close_event;

    constant C_BUILD_CONFIG : lidar_build_config_t := fn_build_config;
    constant C_ACTIVE_PROFILE : gpx_vdma_lane_profile_t :=
        fn_active_profile;
    constant C_CLK_PERIOD : time := 1 us / G_CLK_MHZ;
    constant C_TOTAL_LINES : positive :=
        2 * (C_PLANNED_SHOTS +
             to_integer(C_ACTIVE_PROFILE.footer_lines));
    constant C_BEATS_PER_LINE : positive :=
        to_integer(C_ACTIVE_PROFILE.hsize_bytes) / (G_OUTPUT_WIDTH / 8);
    constant C_EXPECTED_BEATS : positive :=
        C_TOTAL_LINES * C_BEATS_PER_LINE;

    signal clk : std_logic := '0';
    signal rst_n : std_logic := '0';
    signal abort_event : std_logic := '0';
    signal clear_sticky : std_logic := '0';

    signal source_cell : gpx_frame_cell_event_t :=
        C_GPX_FRAME_CELL_EVENT_IDLE;
    signal source_cell_ready : std_logic;
    signal source_close : gpx_frame_close_event_t :=
        C_GPX_FRAME_CLOSE_EVENT_IDLE;
    signal source_close_ready : std_logic;

    signal v2_cell : gpx_frame_cell_event_t :=
        C_GPX_FRAME_CELL_EVENT_IDLE;
    signal hls_cell : gpx_frame_cell_event_t :=
        C_GPX_FRAME_CELL_EVENT_IDLE;
    signal v2_cell_ready : std_logic;
    signal hls_cell_ready : std_logic;
    signal v2_close : gpx_frame_close_event_t :=
        C_GPX_FRAME_CLOSE_EVENT_IDLE;
    signal hls_close : gpx_frame_close_event_t :=
        C_GPX_FRAME_CLOSE_EVENT_IDLE;
    signal v2_close_ready : std_logic;
    signal hls_close_ready : std_logic;

    signal hls_line_word : gpx_vdma_line_word_event_t :=
        C_GPX_VDMA_LINE_WORD_EVENT_IDLE;
    signal hls_line_word_ready : std_logic;
    signal hls_formatter_idle : std_logic;
    signal hls_packer_idle : std_logic;
    signal hls_footer_emitted : std_logic;
    signal hls_emitted_lines : unsigned(16 downto 0);
    signal hls_fault_pulse : lidar_gpx_word_formatter_faults_t;
    signal hls_fault_sticky : lidar_gpx_word_formatter_faults_t;

    signal v2_tdata : std_logic_vector(G_OUTPUT_WIDTH - 1 downto 0);
    signal v2_tkeep : std_logic_vector(G_OUTPUT_WIDTH / 8 - 1 downto 0);
    signal v2_tstrb : std_logic_vector(G_OUTPUT_WIDTH / 8 - 1 downto 0);
    signal v2_tuser : std_logic_vector(0 downto 0);
    signal v2_tvalid : std_logic;
    signal v2_tlast : std_logic;
    signal v2_tready : std_logic;
    signal v2_line_done : std_logic;
    signal v2_frame_done : std_logic;
    signal v2_footer_emitted : std_logic;
    signal v2_idle : std_logic;

    signal hls_tdata : std_logic_vector(G_OUTPUT_WIDTH - 1 downto 0);
    signal hls_tkeep : std_logic_vector(G_OUTPUT_WIDTH / 8 - 1 downto 0);
    signal hls_tstrb : std_logic_vector(G_OUTPUT_WIDTH / 8 - 1 downto 0);
    signal hls_tuser : std_logic_vector(0 downto 0);
    signal hls_tvalid : std_logic;
    signal hls_tlast : std_logic;
    signal hls_tready : std_logic;
    signal hls_line_done : std_logic;
    signal hls_frame_done : std_logic;

    signal force_output_stall : std_logic := '0';
    signal sink_ready_r : std_logic := '0';
    signal sink_cycle_r : natural range 0 to 31 := 0;

    signal compared_beats : natural := 0;
    signal compared_lines : natural := 0;
    signal compared_sof : natural := 0;
    signal v2_frames : natural := 0;
    signal hls_frames : natural := 0;
    signal v2_footers : natural := 0;
    signal hls_footers : natural := 0;

begin

    assert G_OUTPUT_WIDTH = 32 or G_OUTPUT_WIDTH = 64
        report "V3-H4-DIFF output width must be 32 or 64"
        severity failure;

    clk <= not clk after C_CLK_PERIOD / 2;

    -- 한 경로만 입력을 먼저 소비하지 않도록 두 ready가 모두 참일 때만
    -- 각 DUT의 valid를 활성화한다.
    p_gate_source : process (all)
        variable v2_cell_v : gpx_frame_cell_event_t;
        variable hls_cell_v : gpx_frame_cell_event_t;
        variable v2_close_v : gpx_frame_close_event_t;
        variable hls_close_v : gpx_frame_close_event_t;
    begin
        v2_cell_v := source_cell;
        hls_cell_v := source_cell;
        v2_cell_v.valid := source_cell.valid and hls_cell_ready;
        hls_cell_v.valid := source_cell.valid and v2_cell_ready;
        v2_close_v := source_close;
        hls_close_v := source_close;
        v2_close_v.valid := source_close.valid and hls_close_ready;
        hls_close_v.valid := source_close.valid and v2_close_ready;
        v2_cell <= v2_cell_v;
        hls_cell <= hls_cell_v;
        v2_close <= v2_close_v;
        hls_close <= hls_close_v;
    end process p_gate_source;

    source_cell_ready <= v2_cell_ready and hls_cell_ready;
    source_close_ready <= v2_close_ready and hls_close_ready;

    -- 먼저 도착한 출력은 정지시켜 Pipeline latency가 아니라 실제 Beat의
    -- 데이터와 경계만 비교한다.
    v2_tready <= sink_ready_r and hls_tvalid;
    hls_tready <= sink_ready_r and v2_tvalid;

    u_v2_reference : entity work.lidar_gpx_axis_lane_pipeline
        generic map (
            G_BUILD_CONFIG => C_BUILD_CONFIG,
            G_LANE_RISE    => G_LANE_RISE
        )
        port map (
            i_clk => clk,
            i_rst_n => rst_n,
            i_abort => abort_event,
            i_active_profile => C_ACTIVE_PROFILE,
            i_cell_event => v2_cell,
            o_cell_ready => v2_cell_ready,
            i_frame_close_event => v2_close,
            o_frame_close_ready => v2_close_ready,
            o_m_axis_tdata => v2_tdata,
            o_m_axis_tkeep => v2_tkeep,
            o_m_axis_tstrb => v2_tstrb,
            o_m_axis_tuser => v2_tuser,
            o_m_axis_tvalid => v2_tvalid,
            o_m_axis_tlast => v2_tlast,
            i_m_axis_tready => v2_tready,
            o_line_done => v2_line_done,
            o_frame_done => v2_frame_done,
            o_footer_emitted => v2_footer_emitted,
            o_idle => v2_idle
        );

    u_h4_formatter : entity work.lidar_gpx_lane_word_formatter_hls_adapter
        generic map (
            G_BUILD_CONFIG => C_BUILD_CONFIG,
            G_LANE_RISE    => G_LANE_RISE
        )
        port map (
            i_clk => clk,
            i_rst_n => rst_n,
            i_abort => abort_event,
            i_clear_sticky => clear_sticky,
            i_active_profile => C_ACTIVE_PROFILE,
            i_active_version => C_ACTIVE_VERSION,
            i_cell_event => hls_cell,
            o_cell_ready => hls_cell_ready,
            i_frame_close_event => hls_close,
            o_frame_close_ready => hls_close_ready,
            o_line_word => hls_line_word,
            i_line_word_ready => hls_line_word_ready,
            o_footer_emitted => hls_footer_emitted,
            o_emitted_line_count => hls_emitted_lines,
            o_idle => hls_formatter_idle,
            o_fault_pulse => hls_fault_pulse,
            o_fault_sticky => hls_fault_sticky
        );

    u_h4_axis_packer : entity work.lidar_gpx_axis_word_packer
        generic map (
            G_OUTPUT_WIDTH => G_OUTPUT_WIDTH
        )
        port map (
            i_clk => clk,
            i_rst_n => rst_n,
            i_abort => abort_event,
            i_line_word => hls_line_word,
            o_line_word_ready => hls_line_word_ready,
            o_m_axis_tdata => hls_tdata,
            o_m_axis_tkeep => hls_tkeep,
            o_m_axis_tstrb => hls_tstrb,
            o_m_axis_tuser => hls_tuser,
            o_m_axis_tvalid => hls_tvalid,
            o_m_axis_tlast => hls_tlast,
            i_m_axis_tready => hls_tready,
            o_line_done => hls_line_done,
            o_frame_done => hls_frame_done,
            o_idle => hls_packer_idle
        );

    p_sink_backpressure : process (clk)
    begin
        if rising_edge(clk) then
            if rst_n = '0' or force_output_stall = '1' then
                sink_cycle_r <= 0;
                sink_ready_r <= '0';
            else
                if sink_cycle_r = 10 then
                    sink_cycle_r <= 0;
                else
                    sink_cycle_r <= sink_cycle_r + 1;
                end if;
                if sink_cycle_r = 3 or sink_cycle_r = 4 or
                   sink_cycle_r = 9 then
                    sink_ready_r <= '0';
                else
                    sink_ready_r <= '1';
                end if;
            end if;
        end if;
    end process p_sink_backpressure;

    p_compare : process (clk)
    begin
        if rising_edge(clk) then
            if rst_n = '0' or abort_event = '1' then
                compared_beats <= 0;
                compared_lines <= 0;
                compared_sof <= 0;
                v2_frames <= 0;
                hls_frames <= 0;
                v2_footers <= 0;
                hls_footers <= 0;
            else
                if v2_tvalid = '1' and hls_tvalid = '1' then
                    assert v2_tdata = hls_tdata
                        report "V3-H4-DIFF AXIS TDATA mismatch"
                        severity failure;
                    assert v2_tkeep = hls_tkeep and
                           v2_tstrb = hls_tstrb
                        report "V3-H4-DIFF AXIS byte qualifier mismatch"
                        severity failure;
                    assert v2_tuser = hls_tuser and
                           v2_tlast = hls_tlast
                        report "V3-H4-DIFF AXIS Line/Frame boundary mismatch"
                        severity failure;
                end if;
                if v2_tvalid = '1' and hls_tvalid = '1' and
                   sink_ready_r = '1' then
                    compared_beats <= compared_beats + 1;
                    if v2_tlast = '1' then
                        compared_lines <= compared_lines + 1;
                    end if;
                    if v2_tuser(0) = '1' then
                        compared_sof <= compared_sof + 1;
                    end if;
                end if;
                if v2_frame_done = '1' then
                    v2_frames <= v2_frames + 1;
                end if;
                if hls_frame_done = '1' then
                    hls_frames <= hls_frames + 1;
                end if;
                if v2_footer_emitted = '1' then
                    v2_footers <= v2_footers + 1;
                end if;
                if hls_footer_emitted = '1' then
                    hls_footers <= hls_footers + 1;
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
        end procedure wait_clocks;

        procedure send_cell(value : gpx_frame_cell_event_t) is
            variable timeout : natural := 0;
        begin
            source_cell <= value;
            loop
                wait until rising_edge(clk);
                exit when source_cell_ready = '1';
                timeout := timeout + 1;
                assert timeout < 10000
                    report "V3-H4-DIFF Cell input timeout"
                    severity failure;
            end loop;
            source_cell <= C_GPX_FRAME_CELL_EVENT_IDLE;
        end procedure send_cell;

        procedure send_shot(
            shot_column : natural;
            gap_before  : natural;
            face_index  : natural) is
        begin
            for slot_index in 0 to G_SLOT_COUNT - 1 loop
                send_cell(fn_cell_event(
                    shot_column, slot_index, gap_before, face_index));
            end loop;
        end procedure send_shot;

        procedure send_close(value : gpx_frame_close_event_t) is
            variable timeout : natural := 0;
        begin
            source_close <= value;
            loop
                wait until rising_edge(clk);
                exit when source_close_ready = '1';
                timeout := timeout + 1;
                assert timeout < 10000
                    report "V3-H4-DIFF Face close input timeout"
                    severity failure;
            end loop;
            source_close <= C_GPX_FRAME_CLOSE_EVENT_IDLE;
        end procedure send_close;

        procedure wait_paths_idle(limit : positive) is
        begin
            for cycle_index in 1 to limit loop
                wait until rising_edge(clk);
                exit when v2_idle = '1' and hls_formatter_idle = '1' and
                          hls_packer_idle = '1';
                if cycle_index = limit then
                    assert false
                        report "V3-H4-DIFF path idle timeout" &
                            " v2_idle=" & std_logic'image(v2_idle) &
                            " hls_formatter_idle=" &
                            std_logic'image(hls_formatter_idle) &
                            " hls_packer_idle=" &
                            std_logic'image(hls_packer_idle) &
                            " compared_beats=" &
                            integer'image(compared_beats) &
                            " v2_frames=" & integer'image(v2_frames) &
                            " hls_frames=" & integer'image(hls_frames)
                        severity failure;
                end if;
            end loop;
        end procedure wait_paths_idle;

        procedure wait_frames(count : positive; limit : positive) is
        begin
            for cycle_index in 1 to limit loop
                wait until rising_edge(clk);
                exit when v2_frames = count and hls_frames = count;
                if cycle_index = limit then
                    assert false
                        report "V3-H4-DIFF frame completion timeout"
                        severity failure;
                end if;
            end loop;
        end procedure wait_frames;

    begin
        rst_n <= '0';
        force_output_stall <= '1';
        wait_clocks(8);
        rst_n <= '1';
        wait_clocks(5);

        -- 출력 정지 상태에서 한 Cell을 승인한 뒤 abort하여 두 경로의
        -- 부분 Line과 HLS 내부 상태가 모두 폐기되는지 확인한다.
        send_cell(fn_cell_event(0, 0, 0, 0));
        wait_clocks(30);
        abort_event <= '1';
        wait_clocks(2);
        abort_event <= '0';
        force_output_stall <= '0';
        wait_paths_idle(3000);
        clear_sticky <= '1';
        wait_clocks(1);
        clear_sticky <= '0';

        -- Face 0: Shot 0, 누락 Shot 1, Shot 2, trailing 누락 Shot 3.
        send_shot(0, 0, 0);
        send_shot(2, 1, 0);
        send_close(fn_close_event(16#1001#, 0, 1, false));
        wait_frames(1, 50000);

        -- Face 1: 실제 Shot이 하나도 없는 all-hole Face.
        send_close(fn_close_event(
            16#1002#, 1, C_PLANNED_SHOTS, true));
        wait_frames(2, 50000);
        wait_paths_idle(5000);
        wait_clocks(5);

        assert compared_beats = C_EXPECTED_BEATS
            report "V3-H4-DIFF compared Beat count mismatch"
            severity failure;
        assert compared_lines = C_TOTAL_LINES
            report "V3-H4-DIFF compared Line count mismatch"
            severity failure;
        assert compared_sof = 2
            report "V3-H4-DIFF SOF count mismatch"
            severity failure;
        assert v2_frames = 2 and hls_frames = 2
            report "V3-H4-DIFF Frame completion count mismatch"
            severity failure;
        assert v2_footers = 2 and hls_footers = 2
            report "V3-H4-DIFF Footer completion count mismatch"
            severity failure;
        assert hls_fault_sticky =
               C_LIDAR_GPX_WORD_FORMATTER_FAULTS_CLEAR
            report "V3-H4-DIFF unexpected H4 structural fault"
            severity failure;

        report "LIDAR_V3_GPX_LANE_WORD_FORMATTER_DIFF_PASS clk_mhz=" &
            integer'image(G_CLK_MHZ) & " width=" &
            integer'image(G_OUTPUT_WIDTH) & " slots=" &
            integer'image(G_SLOT_COUNT) & " returns=" &
            integer'image(G_RETURN_SLOTS)
            severity note;
        finish;
        wait;
    end process p_stimulus;

end architecture sim;

library ieee;
use ieee.std_logic_1164.all;
entity tb_lidar_gpx_lane_word_formatter_diff_min32_150 is end entity;
architecture sim of tb_lidar_gpx_lane_word_formatter_diff_min32_150 is
begin
    u_test : entity work.tb_lidar_gpx_lane_word_formatter_hls_diff
        generic map (
            G_CLK_MHZ => 150, G_OUTPUT_WIDTH => 32,
            G_SLOT_COUNT => 1, G_RETURN_SLOTS => 1,
            G_LANE_RISE => true);
end architecture;

library ieee;
use ieee.std_logic_1164.all;
entity tb_lidar_gpx_lane_word_formatter_diff_max32_200 is end entity;
architecture sim of tb_lidar_gpx_lane_word_formatter_diff_max32_200 is
begin
    u_test : entity work.tb_lidar_gpx_lane_word_formatter_hls_diff
        generic map (
            G_CLK_MHZ => 200, G_OUTPUT_WIDTH => 32,
            G_SLOT_COUNT => 32, G_RETURN_SLOTS => 7,
            G_LANE_RISE => true);
end architecture;

library ieee;
use ieee.std_logic_1164.all;
entity tb_lidar_gpx_lane_word_formatter_diff_mid64_150 is end entity;
architecture sim of tb_lidar_gpx_lane_word_formatter_diff_mid64_150 is
begin
    u_test : entity work.tb_lidar_gpx_lane_word_formatter_hls_diff
        generic map (
            G_CLK_MHZ => 150, G_OUTPUT_WIDTH => 64,
            G_SLOT_COUNT => 16, G_RETURN_SLOTS => 3,
            G_LANE_RISE => false);
end architecture;

library ieee;
use ieee.std_logic_1164.all;
entity tb_lidar_gpx_lane_word_formatter_diff_max64_200 is end entity;
architecture sim of tb_lidar_gpx_lane_word_formatter_diff_max64_200 is
begin
    u_test : entity work.tb_lidar_gpx_lane_word_formatter_hls_diff
        generic map (
            G_CLK_MHZ => 200, G_OUTPUT_WIDTH => 64,
            G_SLOT_COUNT => 32, G_RETURN_SLOTS => 7,
            G_LANE_RISE => true);
end architecture;
