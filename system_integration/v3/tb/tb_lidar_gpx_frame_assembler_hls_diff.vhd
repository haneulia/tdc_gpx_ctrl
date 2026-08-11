-- ============================================================================
-- 검증 목적:
--   V2 RTL Frame lane assembler와 V3 HLS Frame assembler를 동일한 Cell 및
--   Face 종료 입력으로 구동하여 관측 가능한 계약이 같은지 차등 검증한다.
--
-- 검증 범위:
--   * 전용 Rise/Fall Chip, 한 Chip 양 Edge, Fall 비활성, 축소 구성,
--     네 Chip 모두 Rise/Fall 구성
--   * Runtime 직렬화(전시) Return 슬롯 수 1~7이 섞인 Cell payload와 canonical slot 순서
--   * Rise/Fall 독립 Backpressure, 누락 Cell blank-fill, 중복/마스크 오류
--   * 내부 출력이 남은 상태의 Abort, Reset Epoch 복구 및 주소 재사용
--   * Face 내부 column gap, trailing gap, all-hole Face 종료
--   * 150 MHz 및 200 MHz Processing/AXIS clock
--
-- 유지보수 규칙:
--   입력은 두 DUT가 모두 ready일 때만 동시에 승인한다. 출력은 두 DUT의
--   같은 Lane valid가 모두 모일 때 함께 소비한다. 따라서 Pipeline latency는
--   비교 대상이 아니며, event 내용과 순서 및 외부 소비 완료 의미만 비교한다.
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.env.all;

use work.lidar_build_pkg.all;
use work.lidar_event_types_pkg.all;
use work.lidar_gpx_event_pkg.all;
use work.lidar_gpx_data_pkg.all;

entity tb_lidar_gpx_frame_assembler_hls_diff is
    generic (
        G_CLK_MHZ  : positive := 150;
        G_SCENARIO : natural range 0 to 4 := 0
    );
end entity tb_lidar_gpx_frame_assembler_hls_diff;

architecture sim of tb_lidar_gpx_frame_assembler_hls_diff is

    function fn_build_config return lidar_build_config_t is
        variable result : lidar_build_config_t := C_DEFAULT_BUILD_CONFIG;
    begin
        result.proc_clk_mhz           := G_CLK_MHZ;
        result.tdc_clk_mhz            := 200;
        result.output_width           := 32;
        result.num_faces              := 4;
        result.max_returns_per_stop   := 7;
        result.enable_echo_receiver   := false;
        result.enable_echo_simulation := false;

        case G_SCENARIO is
            when 0 =>
                result.num_chips            := 4;
                result.stops_per_chip       := 8;
                result.rise_capability_mask := "0011";
                result.fall_capability_mask := "1100";
            when 1 =>
                result.num_chips            := 1;
                result.stops_per_chip       := 8;
                result.rise_capability_mask := "0001";
                result.fall_capability_mask := "0001";
            when 2 =>
                result.num_chips            := 3;
                result.stops_per_chip       := 6;
                result.rise_capability_mask := "0111";
                result.fall_capability_mask := "0000";
            when 3 =>
                result.num_chips            := 3;
                result.stops_per_chip       := 6;
                result.rise_capability_mask := "0011";
                result.fall_capability_mask := "0100";
            when others =>
                result.num_chips            := 4;
                result.stops_per_chip       := 8;
                result.rise_capability_mask := "1111";
                result.fall_capability_mask := "1111";
        end case;
        return result;
    end function fn_build_config;

    function fn_rise_mask return chip_mask_t is
    begin
        case G_SCENARIO is
            when 0      => return "0011";
            when 1      => return "0001";
            when 2      => return "0111";
            when 3      => return "0011";
            when others => return "1111";
        end case;
    end function fn_rise_mask;

    function fn_fall_mask return chip_mask_t is
    begin
        case G_SCENARIO is
            when 0      => return "1100";
            when 1      => return "0001";
            when 2      => return "0000";
            when 3      => return "0100";
            when others => return "1111";
        end case;
    end function fn_fall_mask;

    function fn_columns return natural is
    begin
        if G_SCENARIO = 0 then
            return 3;
        end if;
        return 1;
    end function fn_columns;

    function fn_context(
        shot_index : natural;
        columns    : natural;
        face_index : natural := 1
    ) return shot_start_event_t is
        variable result : shot_start_event_t := C_SHOT_START_EVENT_IDLE;
    begin
        result.valid := '1';
        result.request.valid := '1';
        result.request.face_index := to_unsigned(face_index, 3);
        result.request.position := to_unsigned(
            100 + shot_index, result.request.position'length);
        result.request.direction := DIRECTION_CW;
        result.request.shot_index := to_unsigned(shot_index, 16);
        if shot_index + 1 >= columns then
            result.request.last_in_face := '1';
        end if;
        result.request.source_sim := '0';
        result.request.source_latency_clks := to_unsigned(
            3 + shot_index, result.request.source_latency_clks'length);
        result.request.source_latency_valid := '1';
        result.request.active_version := to_unsigned(9, 16);
        result.fire_to_t0_clks := to_unsigned(
            7 + shot_index, result.fire_to_t0_clks'length);
        result.t0_timestamp_ticks := to_unsigned(
            16#10000# + shot_index,
            result.t0_timestamp_ticks'length);
        result.t0_timestamp_valid := '1';
        result.t0_time_sync_valid := '1';
        return result;
    end function fn_context;

    function fn_data_cell(
        shot_index  : natural;
        columns     : natural;
        chip_index  : natural;
        stop_index  : natural;
        slope_value : gpx_slope_t
    ) return gpx_cell_event_t is
        variable result : gpx_cell_event_t := C_GPX_CELL_EVENT_IDLE;
        variable count_value : natural;
        variable slope_offset : natural := 0;
    begin
        if slope_value = GPX_SLOPE_FALL then
            slope_offset := 10000;
        end if;
        count_value := 1 + ((shot_index + chip_index + stop_index) mod 7);

        result.valid := '1';
        result.kind := GPX_CELL_DATA;
        result.chip_index := to_unsigned(chip_index, 2);
        result.stop_index := to_unsigned(stop_index, 3);
        if stop_index >= 4 then
            result.ififo_id := '1';
        end if;
        result.slope := slope_value;
        result.hit_count := to_unsigned(count_value, 3);
        result.max_hits := to_unsigned(7, 3);
        for hit_index in 0 to count_value - 1 loop
            result.hits(hit_index) := to_unsigned(
                (shot_index * 20000 + slope_offset +
                 chip_index * 1000 + stop_index * 10 + hit_index + 1)
                    mod (2 ** C_GPX_HIT_WIDTH),
                C_GPX_HIT_WIDTH);
        end loop;
        result.shot_context := fn_context(shot_index, columns);
        result.chip_shot_seq := to_unsigned(10 + shot_index, 16);
        return result;
    end function fn_data_cell;

    function fn_terminal(
        shot_index : natural;
        columns    : natural;
        chip_index : natural
    ) return gpx_cell_event_t is
        variable result : gpx_cell_event_t := C_GPX_CELL_EVENT_IDLE;
    begin
        result.valid := '1';
        result.kind := GPX_CELL_DRAIN_DONE;
        result.chip_index := to_unsigned(chip_index, 2);
        result.max_hits := to_unsigned(7, 3);
        result.shot_context := fn_context(shot_index, columns);
        result.chip_shot_seq := to_unsigned(10 + shot_index, 16);
        return result;
    end function fn_terminal;

    function fn_face_close(
        frame_id  : natural;
        face_index : natural;
        columns   : natural
    ) return face_close_event_t is
        variable result : face_close_event_t := C_FACE_CLOSE_EVENT_IDLE;
    begin
        result.valid := '1';
        result.face_frame_id := to_unsigned(
            frame_id, result.face_frame_id'length);
        result.face_index := to_unsigned(face_index, 3);
        result.direction := DIRECTION_CW;
        result.source_sim := '0';
        result.active_version := to_unsigned(9, 16);
        result.columns_per_face := to_unsigned(columns, 16);
        return result;
    end function fn_face_close;

    function fn_fault_or(
        left_value  : gpx_frame_assembler_faults_t;
        right_value : gpx_frame_assembler_faults_t
    ) return gpx_frame_assembler_faults_t is
        variable result : gpx_frame_assembler_faults_t;
    begin
        result.context_mismatch :=
            left_value.context_mismatch or right_value.context_mismatch;
        result.unexpected_cell :=
            left_value.unexpected_cell or right_value.unexpected_cell;
        result.duplicate_cell :=
            left_value.duplicate_cell or right_value.duplicate_cell;
        result.duplicate_terminal :=
            left_value.duplicate_terminal or right_value.duplicate_terminal;
        result.missing_cell :=
            left_value.missing_cell or right_value.missing_cell;
        result.geometry_error :=
            left_value.geometry_error or right_value.geometry_error;
        result.column_gap :=
            left_value.column_gap or right_value.column_gap;
        result.masked_payload_drop :=
            left_value.masked_payload_drop or
            right_value.masked_payload_drop;
        return result;
    end function fn_fault_or;

    constant C_BUILD_CONFIG : lidar_build_config_t := fn_build_config;
    constant C_RISE_MASK : chip_mask_t := fn_rise_mask;
    constant C_FALL_MASK : chip_mask_t := fn_fall_mask;
    constant C_COLUMNS : natural := fn_columns;
    constant C_CLK_PERIOD : time := 1 us / G_CLK_MHZ;

    signal clk          : std_logic := '0';
    signal rst_n        : std_logic := '0';
    signal abort_event  : std_logic := '0';
    signal clear_sticky : std_logic := '0';

    signal cell_event : gpx_cell_event_t := C_GPX_CELL_EVENT_IDLE;
    signal v2_cell_ready  : std_logic;
    signal hls_cell_ready : std_logic;
    signal face_close_event : face_close_event_t := C_FACE_CLOSE_EVENT_IDLE;
    signal v2_face_close_ready  : std_logic;
    signal hls_face_close_ready : std_logic;

    signal v2_rise_event  : gpx_frame_cell_event_t;
    signal hls_rise_event : gpx_frame_cell_event_t;
    signal v2_fall_event  : gpx_frame_cell_event_t;
    signal hls_fall_event : gpx_frame_cell_event_t;
    signal v2_rise_ready  : std_logic;
    signal hls_rise_ready : std_logic;
    signal v2_fall_ready  : std_logic;
    signal hls_fall_ready : std_logic;

    signal v2_close_event  : gpx_frame_close_event_t;
    signal hls_close_event : gpx_frame_close_event_t;
    signal v2_close_ready  : std_logic;
    signal hls_close_ready : std_logic;

    signal v2_shot_done  : std_logic;
    signal hls_shot_done : std_logic;
    signal v2_shot_context  : shot_start_event_t;
    signal hls_shot_context : shot_start_event_t;
    signal v2_idle  : std_logic;
    signal hls_idle : std_logic;

    signal v2_fault_pulse  : gpx_frame_assembler_faults_t;
    signal hls_fault_pulse : gpx_frame_assembler_faults_t;
    signal v2_fault_sticky  : gpx_frame_assembler_faults_t;
    signal hls_fault_sticky : gpx_frame_assembler_faults_t;
    signal v2_fault_seen  : gpx_frame_assembler_faults_t :=
        C_GPX_FRAME_ASSEMBLER_FAULTS_CLEAR;
    signal hls_fault_seen : gpx_frame_assembler_faults_t :=
        C_GPX_FRAME_ASSEMBLER_FAULTS_CLEAR;

    signal force_output_stall : std_logic := '0';
    signal ready_counter : natural := 0;
    signal consumer_rise_ready : std_logic := '0';
    signal consumer_fall_ready : std_logic := '0';
    signal consumer_close_ready : std_logic := '0';

    signal v2_shot_done_count  : natural := 0;
    signal hls_shot_done_count : natural := 0;
    signal rise_cell_count  : natural := 0;
    signal fall_cell_count  : natural := 0;
    signal close_count      : natural := 0;

begin

    clk <= not clk after C_CLK_PERIOD / 2;

    -- A Lane is consumed only when both implementations present the same
    -- logical position. This removes latency from the comparison.
    v2_rise_ready <= consumer_rise_ready and hls_rise_event.valid;
    hls_rise_ready <= consumer_rise_ready and v2_rise_event.valid;
    v2_fall_ready <= consumer_fall_ready and hls_fall_event.valid;
    hls_fall_ready <= consumer_fall_ready and v2_fall_event.valid;
    v2_close_ready <= consumer_close_ready and hls_close_event.valid;
    hls_close_ready <= consumer_close_ready and v2_close_event.valid;

    u_v2 : entity work.lidar_gpx_frame_lane_assembler
        generic map (G_BUILD_CONFIG => C_BUILD_CONFIG)
        port map (
            i_clk => clk,
            i_rst_n => rst_n,
            i_abort => abort_event,
            i_clear_sticky => clear_sticky,
            i_active_version => to_unsigned(9, 16),
            i_active_rise_mask => C_RISE_MASK,
            i_active_fall_mask => C_FALL_MASK,
            i_columns_per_face => to_unsigned(C_COLUMNS, 16),
            i_cell_event => cell_event,
            o_cell_ready => v2_cell_ready,
            i_face_close_event => face_close_event,
            o_face_close_ready => v2_face_close_ready,
            o_rise_event => v2_rise_event,
            i_rise_ready => v2_rise_ready,
            o_fall_event => v2_fall_event,
            i_fall_ready => v2_fall_ready,
            o_frame_close_event => v2_close_event,
            i_frame_close_ready => v2_close_ready,
            o_rise_line_done => open,
            o_fall_line_done => open,
            o_shot_done => v2_shot_done,
            o_shot_done_context => v2_shot_context,
            o_idle => v2_idle,
            o_fault_pulse => v2_fault_pulse,
            o_fault_sticky => v2_fault_sticky
        );

    u_hls : entity work.lidar_gpx_frame_lane_assembler_hls_adapter
        generic map (G_BUILD_CONFIG => C_BUILD_CONFIG)
        port map (
            i_clk => clk,
            i_rst_n => rst_n,
            i_abort => abort_event,
            i_clear_sticky => clear_sticky,
            i_active_version => to_unsigned(9, 16),
            i_active_rise_mask => C_RISE_MASK,
            i_active_fall_mask => C_FALL_MASK,
            i_columns_per_face => to_unsigned(C_COLUMNS, 16),
            i_cell_event => cell_event,
            o_cell_ready => hls_cell_ready,
            i_face_close_event => face_close_event,
            o_face_close_ready => hls_face_close_ready,
            o_rise_event => hls_rise_event,
            i_rise_ready => hls_rise_ready,
            o_fall_event => hls_fall_event,
            i_fall_ready => hls_fall_ready,
            o_frame_close_event => hls_close_event,
            i_frame_close_ready => hls_close_ready,
            o_rise_line_done => open,
            o_fall_line_done => open,
            o_shot_done => hls_shot_done,
            o_shot_done_context => hls_shot_context,
            o_idle => hls_idle,
            o_fault_pulse => hls_fault_pulse,
            o_fault_sticky => hls_fault_sticky
        );

    p_ready : process (clk)
    begin
        if rising_edge(clk) then
            if rst_n = '0' or force_output_stall = '1' then
                ready_counter <= 0;
                consumer_rise_ready <= '0';
                consumer_fall_ready <= '0';
                consumer_close_ready <= '0';
            else
                ready_counter <= ready_counter + 1;
                if ready_counter mod 5 = 1 or ready_counter mod 5 = 2 then
                    consumer_rise_ready <= '0';
                else
                    consumer_rise_ready <= '1';
                end if;
                if ready_counter mod 7 = 3 or ready_counter mod 7 = 4 or
                   ready_counter mod 7 = 5 then
                    consumer_fall_ready <= '0';
                else
                    consumer_fall_ready <= '1';
                end if;
                consumer_close_ready <= '1';
            end if;
        end if;
    end process p_ready;

    p_compare : process (clk)
        variable v2_rise_held : boolean := false;
        variable hls_rise_held : boolean := false;
        variable v2_fall_held : boolean := false;
        variable hls_fall_held : boolean := false;
        variable v2_rise_value : gpx_frame_cell_event_t :=
            C_GPX_FRAME_CELL_EVENT_IDLE;
        variable hls_rise_value : gpx_frame_cell_event_t :=
            C_GPX_FRAME_CELL_EVENT_IDLE;
        variable v2_fall_value : gpx_frame_cell_event_t :=
            C_GPX_FRAME_CELL_EVENT_IDLE;
        variable hls_fall_value : gpx_frame_cell_event_t :=
            C_GPX_FRAME_CELL_EVENT_IDLE;
    begin
        if rising_edge(clk) then
            if rst_n = '0' or abort_event = '1' then
                v2_rise_held := false;
                hls_rise_held := false;
                v2_fall_held := false;
                hls_fall_held := false;
            else
                if v2_rise_held then
                    assert v2_rise_event = v2_rise_value
                        report "V3-H3-DIFF V2 Rise changed under backpressure"
                        severity failure;
                end if;
                if hls_rise_held then
                    assert hls_rise_event = hls_rise_value
                        report "V3-H3-DIFF HLS Rise changed under backpressure"
                        severity failure;
                end if;
                if v2_fall_held then
                    assert v2_fall_event = v2_fall_value
                        report "V3-H3-DIFF V2 Fall changed under backpressure"
                        severity failure;
                end if;
                if hls_fall_held then
                    assert hls_fall_event = hls_fall_value
                        report "V3-H3-DIFF HLS Fall changed under backpressure"
                        severity failure;
                end if;

                v2_rise_held := v2_rise_event.valid = '1' and
                    v2_rise_ready = '0';
                hls_rise_held := hls_rise_event.valid = '1' and
                    hls_rise_ready = '0';
                v2_fall_held := v2_fall_event.valid = '1' and
                    v2_fall_ready = '0';
                hls_fall_held := hls_fall_event.valid = '1' and
                    hls_fall_ready = '0';
                if v2_rise_held then v2_rise_value := v2_rise_event; end if;
                if hls_rise_held then hls_rise_value := hls_rise_event; end if;
                if v2_fall_held then v2_fall_value := v2_fall_event; end if;
                if hls_fall_held then hls_fall_value := hls_fall_event; end if;

                if v2_rise_event.valid = '1' and v2_rise_ready = '1' then
                    assert v2_rise_event = hls_rise_event
                        report "V3-H3-DIFF Rise Cell mismatch"
                        severity failure;
                    rise_cell_count <= rise_cell_count + 1;
                end if;
                if v2_fall_event.valid = '1' and v2_fall_ready = '1' then
                    assert v2_fall_event = hls_fall_event
                        report "V3-H3-DIFF Fall Cell mismatch"
                        severity failure;
                    fall_cell_count <= fall_cell_count + 1;
                end if;
                if v2_close_event.valid = '1' and v2_close_ready = '1' then
                    assert v2_close_event = hls_close_event
                        report "V3-H3-DIFF Face close mismatch"
                        severity failure;
                    close_count <= close_count + 1;
                end if;
            end if;
        end if;
    end process p_compare;

    p_status : process (clk)
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                v2_shot_done_count <= 0;
                hls_shot_done_count <= 0;
                v2_fault_seen <= C_GPX_FRAME_ASSEMBLER_FAULTS_CLEAR;
                hls_fault_seen <= C_GPX_FRAME_ASSEMBLER_FAULTS_CLEAR;
            else
                if clear_sticky = '1' then
                    v2_fault_seen <= C_GPX_FRAME_ASSEMBLER_FAULTS_CLEAR;
                    hls_fault_seen <= C_GPX_FRAME_ASSEMBLER_FAULTS_CLEAR;
                else
                    v2_fault_seen <= fn_fault_or(
                        v2_fault_seen, v2_fault_pulse);
                    hls_fault_seen <= fn_fault_or(
                        hls_fault_seen, hls_fault_pulse);
                end if;
                if v2_shot_done = '1' then
                    assert v2_shot_context.valid = '1'
                        report "V3-H3-DIFF V2 Shot done context invalid"
                        severity failure;
                    v2_shot_done_count <= v2_shot_done_count + 1;
                end if;
                if hls_shot_done = '1' then
                    assert hls_shot_context.valid = '1'
                        report "V3-H3-DIFF HLS Shot done context invalid"
                        severity failure;
                    hls_shot_done_count <= hls_shot_done_count + 1;
                end if;
            end if;
        end if;
    end process p_status;

    p_watchdog : process
    begin
        wait for 2 ms;
        assert false
            report "V3-H3-DIFF timeout"
            severity failure;
    end process p_watchdog;

    p_stimulus : process
        procedure wait_clocks(count : natural) is
        begin
            for index in 1 to count loop
                wait until rising_edge(clk);
            end loop;
        end procedure wait_clocks;

        procedure send_cell(value : gpx_cell_event_t) is
        begin
            loop
                wait until falling_edge(clk);
                exit when v2_cell_ready = '1' and hls_cell_ready = '1';
            end loop;
            cell_event <= value;
            wait until rising_edge(clk);
            cell_event <= C_GPX_CELL_EVENT_IDLE;
        end procedure send_cell;

        procedure send_close(value : face_close_event_t) is
        begin
            loop
                wait until falling_edge(clk);
                exit when v2_face_close_ready = '1' and
                    hls_face_close_ready = '1';
            end loop;
            face_close_event <= value;
            wait until rising_edge(clk);
            face_close_event <= C_FACE_CLOSE_EVENT_IDLE;
        end procedure send_close;

        procedure send_shot(
            shot_index : natural;
            inject_faults : boolean := false
        ) is
            variable expected_mask : chip_mask_t;
            variable fault_cell : gpx_cell_event_t;
        begin
            for chip_index in C_MAX_CHIPS - 1 downto 0 loop
                if C_RISE_MASK(chip_index) = '1' then
                    for stop_index in C_BUILD_CONFIG.stops_per_chip - 1
                        downto 0 loop
                        if not (inject_faults and chip_index = 1 and
                                stop_index =
                                    C_BUILD_CONFIG.stops_per_chip - 1) then
                            send_cell(fn_data_cell(
                                shot_index, C_COLUMNS, chip_index,
                                stop_index, GPX_SLOPE_RISE));
                        end if;
                    end loop;
                end if;
                if C_FALL_MASK(chip_index) = '1' then
                    for stop_index in C_BUILD_CONFIG.stops_per_chip - 1
                        downto 0 loop
                        send_cell(fn_data_cell(
                            shot_index, C_COLUMNS, chip_index,
                            stop_index, GPX_SLOPE_FALL));
                    end loop;
                end if;
            end loop;

            if inject_faults then
                -- Existing Rise Cell duplicate.
                send_cell(fn_data_cell(
                    shot_index, C_COLUMNS, 0, 0, GPX_SLOPE_RISE));
                -- Chip 2 Rise is disabled in scenario 3; payload is consumed
                -- and diagnosed without entering the canonical Rise line.
                send_cell(fn_data_cell(
                    shot_index, C_COLUMNS, 2, 0, GPX_SLOPE_RISE));

                -- Same open Shot with a different Active version.
                fault_cell := fn_data_cell(
                    shot_index, C_COLUMNS, 0, 0, GPX_SLOPE_RISE);
                fault_cell.shot_context.request.active_version :=
                    to_unsigned(10, 16);
                send_cell(fault_cell);

                -- Scenario 3 builds three Chips, so logical Chip 3 is outside
                -- the active build range and must be diagnosed explicitly.
                send_cell(fn_data_cell(
                    shot_index, C_COLUMNS, 3, 0, GPX_SLOPE_RISE));
            end if;

            expected_mask := C_RISE_MASK or C_FALL_MASK;
            for chip_index in C_MAX_CHIPS - 1 downto 0 loop
                if expected_mask(chip_index) = '1' then
                    send_cell(fn_terminal(
                        shot_index, C_COLUMNS, chip_index));
                    if inject_faults and chip_index = 2 then
                        -- Duplicate while the same Shot still waits for the
                        -- remaining Chip terminals.
                        send_cell(fn_terminal(
                            shot_index, C_COLUMNS, chip_index));
                    end if;
                end if;
            end loop;
        end procedure send_shot;

        procedure wait_both_idle(max_clocks : natural) is
        begin
            -- The caller may have completed an input handshake on the current
            -- rising edge. Allow both DUT state machines one full cycle to
            -- leave their previously observed idle state before polling it.
            wait until rising_edge(clk);
            for index in 0 to max_clocks - 1 loop
                if v2_idle = '1' and hls_idle = '1' then
                    return;
                end if;
                wait until rising_edge(clk);
            end loop;
            assert false
                report "V3-H3-DIFF DUTs did not become idle"
                severity failure;
        end procedure wait_both_idle;

        variable expected_shots : natural;
        variable bad_close : face_close_event_t;
    begin
        rst_n <= '0';
        force_output_stall <= '1';
        wait_clocks(6);
        rst_n <= '1';
        wait_clocks(4);

        -- Abort a complete Shot while both external Lane consumers are
        -- stalled. No stale Cell or completion may escape after recovery.
        send_shot(0, false);
        wait_clocks(100);
        assert (v2_rise_event.valid = '1' or v2_fall_event.valid = '1') and
               (hls_rise_event.valid = '1' or hls_fall_event.valid = '1')
            report "V3-H3-DIFF pre-abort buffered output was not reached"
            severity failure;
        abort_event <= '1';
        wait_clocks(2);
        abort_event <= '0';
        force_output_stall <= '0';
        wait_both_idle(200);

        clear_sticky <= '1';
        wait_clocks(1);
        clear_sticky <= '0';
        wait_clocks(3);

        send_shot(0, G_SCENARIO = 3);
        if G_SCENARIO = 0 then
            send_shot(2, false);
            expected_shots := 2;
        else
            expected_shots := 1;
        end if;
        wait_both_idle(1000);

        -- First close checks normal/trailing history. A second Face close with
        -- no Shot checks the explicit all-hole Face contract.
        send_close(fn_face_close(16#101#, 1, C_COLUMNS));
        wait_both_idle(200);
        send_close(fn_face_close(16#102#, 2, C_COLUMNS));
        wait_both_idle(200);
        bad_close := fn_face_close(16#103#, 1, C_COLUMNS);
        bad_close.active_version := to_unsigned(10, 16);
        send_close(bad_close);
        wait_both_idle(200);
        wait_clocks(5);

        assert v2_shot_done_count = expected_shots and
               hls_shot_done_count = expected_shots
            report "V3-H3-DIFF Shot completion count mismatch: expected=" &
                integer'image(expected_shots) &
                " V2=" & integer'image(v2_shot_done_count) &
                " HLS=" & integer'image(hls_shot_done_count)
            severity failure;
        assert close_count = 3
            report "V3-H3-DIFF Face close count mismatch"
            severity failure;
        assert rise_cell_count > 0
            report "V3-H3-DIFF no Rise output was compared"
            severity failure;
        if C_FALL_MASK = "0000" then
            assert fall_cell_count = 0
                report "V3-H3-DIFF Fall-off emitted a Fall Cell"
                severity failure;
        else
            assert fall_cell_count > 0
                report "V3-H3-DIFF no Fall output was compared"
                severity failure;
        end if;

        assert v2_fault_seen = hls_fault_seen and
               v2_fault_sticky = hls_fault_sticky and
               v2_fault_seen = v2_fault_sticky
            report "V3-H3-DIFF fault pulse/sticky mismatch"
            severity failure;
        assert v2_fault_sticky.column_gap = '1'
            report "V3-H3-DIFF all-hole/gap diagnostic missing"
            severity failure;
        if G_SCENARIO = 3 then
            assert v2_fault_sticky.duplicate_cell = '1' and
                   v2_fault_sticky.duplicate_terminal = '1' and
                   v2_fault_sticky.missing_cell = '1' and
                   v2_fault_sticky.masked_payload_drop = '1' and
                   v2_fault_sticky.context_mismatch = '1' and
                   v2_fault_sticky.unexpected_cell = '1'
                report "V3-H3-DIFF injected fault coverage incomplete"
                severity failure;
        end if;
        assert v2_fault_sticky.geometry_error = '1'
            report "V3-H3-DIFF Face geometry diagnostic missing"
            severity failure;

        report "LIDAR_V3_GPX_FRAME_ASSEMBLER_DIFF_PASS clk_mhz=" &
            integer'image(G_CLK_MHZ) & " scenario=" &
            integer'image(G_SCENARIO)
            severity note;
        finish;
        wait;
    end process p_stimulus;

end architecture sim;

library ieee;
use ieee.std_logic_1164.all;
entity tb_lidar_gpx_frame_assembler_hls_diff_dedicated_150 is end entity;
architecture sim of tb_lidar_gpx_frame_assembler_hls_diff_dedicated_150 is
begin
    u_test : entity work.tb_lidar_gpx_frame_assembler_hls_diff
        generic map (G_CLK_MHZ => 150, G_SCENARIO => 0);
end architecture;

library ieee;
use ieee.std_logic_1164.all;
entity tb_lidar_gpx_frame_assembler_hls_diff_dedicated_200 is end entity;
architecture sim of tb_lidar_gpx_frame_assembler_hls_diff_dedicated_200 is
begin
    u_test : entity work.tb_lidar_gpx_frame_assembler_hls_diff
        generic map (G_CLK_MHZ => 200, G_SCENARIO => 0);
end architecture;

library ieee;
use ieee.std_logic_1164.all;
entity tb_lidar_gpx_frame_assembler_hls_diff_one_dual_150 is end entity;
architecture sim of tb_lidar_gpx_frame_assembler_hls_diff_one_dual_150 is
begin
    u_test : entity work.tb_lidar_gpx_frame_assembler_hls_diff
        generic map (G_CLK_MHZ => 150, G_SCENARIO => 1);
end architecture;

library ieee;
use ieee.std_logic_1164.all;
entity tb_lidar_gpx_frame_assembler_hls_diff_one_dual_200 is end entity;
architecture sim of tb_lidar_gpx_frame_assembler_hls_diff_one_dual_200 is
begin
    u_test : entity work.tb_lidar_gpx_frame_assembler_hls_diff
        generic map (G_CLK_MHZ => 200, G_SCENARIO => 1);
end architecture;

library ieee;
use ieee.std_logic_1164.all;
entity tb_lidar_gpx_frame_assembler_hls_diff_fall_off_150 is end entity;
architecture sim of tb_lidar_gpx_frame_assembler_hls_diff_fall_off_150 is
begin
    u_test : entity work.tb_lidar_gpx_frame_assembler_hls_diff
        generic map (G_CLK_MHZ => 150, G_SCENARIO => 2);
end architecture;

library ieee;
use ieee.std_logic_1164.all;
entity tb_lidar_gpx_frame_assembler_hls_diff_fall_off_200 is end entity;
architecture sim of tb_lidar_gpx_frame_assembler_hls_diff_fall_off_200 is
begin
    u_test : entity work.tb_lidar_gpx_frame_assembler_hls_diff
        generic map (G_CLK_MHZ => 200, G_SCENARIO => 2);
end architecture;

library ieee;
use ieee.std_logic_1164.all;
entity tb_lidar_gpx_frame_assembler_hls_diff_reduced_150 is end entity;
architecture sim of tb_lidar_gpx_frame_assembler_hls_diff_reduced_150 is
begin
    u_test : entity work.tb_lidar_gpx_frame_assembler_hls_diff
        generic map (G_CLK_MHZ => 150, G_SCENARIO => 3);
end architecture;

library ieee;
use ieee.std_logic_1164.all;
entity tb_lidar_gpx_frame_assembler_hls_diff_reduced_200 is end entity;
architecture sim of tb_lidar_gpx_frame_assembler_hls_diff_reduced_200 is
begin
    u_test : entity work.tb_lidar_gpx_frame_assembler_hls_diff
        generic map (G_CLK_MHZ => 200, G_SCENARIO => 3);
end architecture;

library ieee;
use ieee.std_logic_1164.all;
entity tb_lidar_gpx_frame_assembler_hls_diff_all_dual_150 is end entity;
architecture sim of tb_lidar_gpx_frame_assembler_hls_diff_all_dual_150 is
begin
    u_test : entity work.tb_lidar_gpx_frame_assembler_hls_diff
        generic map (G_CLK_MHZ => 150, G_SCENARIO => 4);
end architecture;

library ieee;
use ieee.std_logic_1164.all;
entity tb_lidar_gpx_frame_assembler_hls_diff_all_dual_200 is end entity;
architecture sim of tb_lidar_gpx_frame_assembler_hls_diff_all_dual_200 is
begin
    u_test : entity work.tb_lidar_gpx_frame_assembler_hls_diff
        generic map (G_CLK_MHZ => 200, G_SCENARIO => 4);
end architecture;
