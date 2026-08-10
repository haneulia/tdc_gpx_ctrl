-- ============================================================================
-- 검증 목적:
--   V2 RTL Hit-to-Cell 수집기와 V3 HLS 수집기를 gpx_cell_event_t 전체
--   계약 경계에서 동일 입력으로 비교한다. Pipeline 지연 자체가 아니라 Cell의
--   내용, 순서, 오류 분류 및 Abort 뒤 복구 결과가 같은지를 검증한다.
--
-- 검증 범위:
--   * 물리 Return 용량 7에서 Runtime 직렬화(전시) Return 슬롯 수 1~7 전체
--   * 전용 Rise/Fall Chip, 한 Chip 양 Edge, 축소 구성, 네 Chip 전체 양 Edge
--   * IFIFO1 하위 STOP 완료, 전체 Drain 완료, Timeout/Error-fill
--   * START number, 8번째 물리 Return, topology/context 오류 진단
--   * 출력 Backpressure와 Abort 뒤 같은 Chip/STOP 주소 재사용
--   * 150 MHz 및 200 MHz Processing/AXIS clock
--
-- 유지보수 규칙:
--   두 구현이 모두 ready일 때만 같은 Hit를 제공하므로 같은 Clock edge에서
--   입력이 승인된다. 출력은 양쪽 valid가 모일 때까지 각각 보관하여 비교한다.
--   따라서 서로 다른 Pipeline latency는 오류가 아니며, 값이나 순서 차이만
--   실패로 판정한다. 시나리오를 추가할 때 이 입력/출력 동기화 규칙을 지킨다.
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.env.all;

use work.lidar_build_pkg.all;
use work.lidar_event_types_pkg.all;
use work.lidar_gpx_event_pkg.all;
use work.lidar_gpx_data_pkg.all;

entity tb_lidar_gpx_cell_collector_hls_diff is
    generic (
        G_CLK_MHZ  : positive := 150;
        G_SCENARIO : natural range 0 to 3 := 0
    );
end entity tb_lidar_gpx_cell_collector_hls_diff;

architecture sim of tb_lidar_gpx_cell_collector_hls_diff is

    function fn_build_config return lidar_build_config_t is
        variable result : lidar_build_config_t := C_DEFAULT_BUILD_CONFIG;
    begin
        result.proc_clk_mhz           := G_CLK_MHZ;
        result.tdc_clk_mhz            := 200;
        result.output_width           := 32;
        result.num_faces              := 4;
        result.enable_echo_receiver   := false;
        result.enable_echo_simulation := false;
        result.max_returns_per_stop   := 7;

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

    function fn_shot_context(
        shot_index     : natural;
        active_version : natural;
        face_index     : natural := 1;
        source_sim     : std_logic := '0'
    ) return shot_start_event_t is
        variable result : shot_start_event_t := C_SHOT_START_EVENT_IDLE;
    begin
        result.valid                         := '1';
        result.request.valid                 := '1';
        result.request.face_index            := to_unsigned(face_index, 3);
        result.request.position              := to_unsigned(
            100 + shot_index, result.request.position'length);
        result.request.shot_index            := to_unsigned(shot_index, 16);
        result.request.active_version        := to_unsigned(active_version, 16);
        result.request.source_sim            := source_sim;
        result.request.source_latency_valid  := '1';
        result.request.source_latency_clks   := to_unsigned(
            3 + shot_index mod 5,
            result.request.source_latency_clks'length);
        result.fire_to_t0_clks               := to_unsigned(
            7 + shot_index, result.fire_to_t0_clks'length);
        result.t0_timestamp_ticks            := to_unsigned(
            16#10000# + shot_index,
            result.t0_timestamp_ticks'length);
        result.t0_timestamp_valid            := '1';
        result.t0_time_sync_valid            := '1';
        return result;
    end function fn_shot_context;

    function fn_hit_data(
        chip_index     : natural;
        stop_index     : natural;
        slope_value    : gpx_slope_t;
        hit_value      : natural;
        start_number   : natural;
        shot_index     : natural;
        active_version : natural;
        faulted        : std_logic := '0'
    ) return gpx_hit_event_t is
        variable result : gpx_hit_event_t := C_GPX_HIT_EVENT_IDLE;
    begin
        result.valid         := '1';
        result.kind          := GPX_HIT_DATA;
        result.chip_index    := to_unsigned(chip_index, 2);
        if stop_index >= 4 then
            result.ififo_id := '1';
        end if;
        result.channel_code  := to_unsigned(stop_index mod 4, 2);
        result.stop_index    := to_unsigned(stop_index, 3);
        result.start_number  := to_unsigned(start_number, 8);
        result.slope         := slope_value;
        result.hit           := to_unsigned(hit_value, C_GPX_HIT_WIDTH);
        result.faulted       := faulted;
        result.shot_context  := fn_shot_context(
            shot_index, active_version);
        result.chip_shot_seq := to_unsigned(shot_index, 16);
        return result;
    end function fn_hit_data;

    function fn_hit_control(
        kind_value     : gpx_hit_event_kind_t;
        chip_index     : natural;
        ififo_id       : std_logic;
        shot_index     : natural;
        active_version : natural;
        faulted        : std_logic := '0';
        timeout_cause  : std_logic_vector(2 downto 0) := "000"
    ) return gpx_hit_event_t is
        variable result : gpx_hit_event_t := C_GPX_HIT_EVENT_IDLE;
    begin
        result.valid         := '1';
        result.kind          := kind_value;
        result.chip_index    := to_unsigned(chip_index, 2);
        result.ififo_id      := ififo_id;
        result.faulted       := faulted;
        result.timeout_cause := timeout_cause;
        result.shot_context  := fn_shot_context(
            shot_index, active_version);
        result.chip_shot_seq := to_unsigned(shot_index, 16);
        return result;
    end function fn_hit_control;

    constant C_BUILD_CONFIG : lidar_build_config_t := fn_build_config;
    constant C_CLK_PERIOD   : time := 1 us / G_CLK_MHZ;

    signal clk            : std_logic := '0';
    signal rst_n          : std_logic := '0';
    signal abort_event    : std_logic := '0';
    signal clear_sticky   : std_logic := '0';
    signal monitor_clear  : std_logic := '0';
    signal active_version : unsigned(15 downto 0) := to_unsigned(5, 16);
    signal max_hits       : unsigned(2 downto 0) := to_unsigned(3, 3);
    signal hit_event      : gpx_hit_event_t := C_GPX_HIT_EVENT_IDLE;
    signal cell_ready     : std_logic := '0';

    signal ref_hit_ready    : std_logic;
    signal ref_cell_event   : gpx_cell_event_t;
    signal ref_fault_pulse  : gpx_cell_collector_faults_t;
    signal ref_fault_sticky : gpx_cell_collector_faults_t;

    signal hls_hit_ready    : std_logic;
    signal hls_cell_event   : gpx_cell_event_t;
    signal hls_fault_pulse  : gpx_cell_collector_faults_t;
    signal hls_fault_sticky : gpx_cell_collector_faults_t;

    signal ref_pulse_seen : gpx_cell_collector_faults_t :=
        C_GPX_CELL_COLLECTOR_FAULTS_CLEAR;
    signal hls_pulse_seen : gpx_cell_collector_faults_t :=
        C_GPX_CELL_COLLECTOR_FAULTS_CLEAR;

    signal simulation_done : boolean := false;

begin

    clk <= not clk after C_CLK_PERIOD / 2 when not simulation_done;

    u_v2_reference : entity work.lidar_gpx_cell_collector
        generic map (
            G_BUILD_CONFIG => C_BUILD_CONFIG
        )
        port map (
            i_clk               => clk,
            i_rst_n             => rst_n,
            i_abort             => abort_event,
            i_clear_sticky      => clear_sticky,
            i_active_version    => active_version,
            i_max_hits_per_stop => max_hits,
            i_active_rise_mask  => C_BUILD_CONFIG.rise_capability_mask,
            i_active_fall_mask  => C_BUILD_CONFIG.fall_capability_mask,
            i_hit_event         => hit_event,
            o_hit_ready         => ref_hit_ready,
            o_cell_event        => ref_cell_event,
            i_cell_ready        => cell_ready,
            o_fault_pulse       => ref_fault_pulse,
            o_fault_sticky      => ref_fault_sticky
        );

    u_v3_hls : entity work.lidar_gpx_cell_collector_hls_adapter
        generic map (
            G_BUILD_CONFIG => C_BUILD_CONFIG
        )
        port map (
            i_clk               => clk,
            i_rst_n             => rst_n,
            i_abort             => abort_event,
            i_clear_sticky      => clear_sticky,
            i_active_version    => active_version,
            i_max_hits_per_stop => max_hits,
            i_active_rise_mask  => C_BUILD_CONFIG.rise_capability_mask,
            i_active_fall_mask  => C_BUILD_CONFIG.fall_capability_mask,
            i_hit_event         => hit_event,
            o_hit_ready         => hls_hit_ready,
            o_cell_event        => hls_cell_event,
            i_cell_ready        => cell_ready,
            o_fault_pulse       => hls_fault_pulse,
            o_fault_sticky      => hls_fault_sticky
        );

    p_fault_monitor : process (clk)
    begin
        if rising_edge(clk) then
            if rst_n = '0' or monitor_clear = '1' then
                ref_pulse_seen <= C_GPX_CELL_COLLECTOR_FAULTS_CLEAR;
                hls_pulse_seen <= C_GPX_CELL_COLLECTOR_FAULTS_CLEAR;
            else
                ref_pulse_seen.context_mismatch <=
                    ref_pulse_seen.context_mismatch or
                    ref_fault_pulse.context_mismatch;
                ref_pulse_seen.return_overflow <=
                    ref_pulse_seen.return_overflow or
                    ref_fault_pulse.return_overflow;
                ref_pulse_seen.start_number_nonzero <=
                    ref_pulse_seen.start_number_nonzero or
                    ref_fault_pulse.start_number_nonzero;
                ref_pulse_seen.hit_capacity_drop <=
                    ref_pulse_seen.hit_capacity_drop or
                    ref_fault_pulse.hit_capacity_drop;

                hls_pulse_seen.context_mismatch <=
                    hls_pulse_seen.context_mismatch or
                    hls_fault_pulse.context_mismatch;
                hls_pulse_seen.return_overflow <=
                    hls_pulse_seen.return_overflow or
                    hls_fault_pulse.return_overflow;
                hls_pulse_seen.start_number_nonzero <=
                    hls_pulse_seen.start_number_nonzero or
                    hls_fault_pulse.start_number_nonzero;
                hls_pulse_seen.hit_capacity_drop <=
                    hls_pulse_seen.hit_capacity_drop or
                    hls_fault_pulse.hit_capacity_drop;
            end if;
        end if;
    end process p_fault_monitor;

    p_stimulus : process
        procedure wait_clocks(count : natural) is
        begin
            for index in 1 to count loop
                wait until rising_edge(clk);
            end loop;
            wait for 1 ps;
        end procedure wait_clocks;

        procedure wait_both_input_ready(
            message_text : string;
            limit         : positive := 1200
        ) is
            variable ready_seen : boolean := false;
        begin
            hit_event <= C_GPX_HIT_EVENT_IDLE;
            for timeout in 0 to limit loop
                wait until falling_edge(clk);
                if ref_hit_ready = '1' and hls_hit_ready = '1' then
                    ready_seen := true;
                    exit;
                end if;
            end loop;
            assert ready_seen
                report message_text & " input-ready timeout"
                severity failure;
        end procedure wait_both_input_ready;

        procedure offer_both(
            value        : gpx_hit_event_t;
            message_text : string
        ) is
        begin
            wait_both_input_ready(message_text);
            hit_event <= value;
            wait until rising_edge(clk);
            assert ref_hit_ready = '1' and hls_hit_ready = '1'
                report message_text & " was not accepted together"
                severity failure;
            wait for 1 ps;
            hit_event <= C_GPX_HIT_EVENT_IDLE;
        end procedure offer_both;

        procedure compare_next_cell(
            message_text : string;
            variable observed : out gpx_cell_event_t;
            stall_clocks : natural := 0
        ) is
            variable both_valid : boolean := false;
            variable held_ref   : gpx_cell_event_t;
            variable held_hls   : gpx_cell_event_t;
        begin
            cell_ready <= '0';
            for timeout in 0 to 1600 loop
                wait until falling_edge(clk);
                if ref_cell_event.valid = '1' and
                   hls_cell_event.valid = '1' then
                    both_valid := true;
                    exit;
                end if;
            end loop;
            assert both_valid
                report message_text & " output timeout"
                severity failure;
            assert ref_cell_event = hls_cell_event
                report message_text & " V2/HLS Cell mismatch"
                severity failure;

            held_ref := ref_cell_event;
            held_hls := hls_cell_event;
            observed := ref_cell_event;
            for index in 1 to stall_clocks loop
                wait until rising_edge(clk);
                wait until falling_edge(clk);
                assert ref_cell_event = held_ref and
                       hls_cell_event = held_hls
                    report message_text & " changed under backpressure"
                    severity failure;
            end loop;

            cell_ready <= '1';
            wait until rising_edge(clk);
            wait for 1 ps;
            cell_ready <= '0';
        end procedure compare_next_cell;

        procedure compare_cell_count(
            count        : positive;
            message_text : string;
            stall_first  : natural := 0
        ) is
            variable observed : gpx_cell_event_t;
        begin
            for index in 0 to count - 1 loop
                if index = 0 then
                    compare_next_cell(
                        message_text & " index=" & integer'image(index),
                        observed,
                        stall_first);
                else
                    compare_next_cell(
                        message_text & " index=" & integer'image(index),
                        observed);
                end if;
            end loop;
        end procedure compare_cell_count;

        procedure clear_fault_state is
        begin
            clear_sticky  <= '1';
            monitor_clear <= '1';
            wait_clocks(1);
            clear_sticky  <= '0';
            monitor_clear <= '0';
            wait_clocks(2);
            assert ref_fault_sticky =
                       C_GPX_CELL_COLLECTOR_FAULTS_CLEAR and
                   hls_fault_sticky =
                       C_GPX_CELL_COLLECTOR_FAULTS_CLEAR and
                   ref_pulse_seen =
                       C_GPX_CELL_COLLECTOR_FAULTS_CLEAR and
                   hls_pulse_seen =
                       C_GPX_CELL_COLLECTOR_FAULTS_CLEAR
                report "fault clear mismatch"
                severity failure;
        end procedure clear_fault_state;

        procedure assert_fault_equivalence(message_text : string) is
        begin
            wait_both_input_ready(message_text);
            wait_clocks(2);
            assert ref_fault_sticky = hls_fault_sticky
                report message_text & " sticky mismatch"
                severity failure;
            assert ref_pulse_seen = hls_pulse_seen
                report message_text & " accumulated pulse mismatch"
                severity failure;
        end procedure assert_fault_equivalence;

        procedure abort_pending_output(message_text : string) is
            variable both_valid : boolean := false;
            variable recovered  : boolean := false;
        begin
            cell_ready <= '0';
            for timeout in 0 to 1600 loop
                wait until falling_edge(clk);
                if ref_cell_event.valid = '1' and
                   hls_cell_event.valid = '1' then
                    both_valid := true;
                    exit;
                end if;
            end loop;
            assert both_valid
                report message_text & " did not create pending output"
                severity failure;

            abort_event <= '1';
            wait_clocks(1);
            abort_event <= '0';

            for timeout in 0 to 1800 loop
                wait until falling_edge(clk);
                assert ref_cell_event.valid = '0' and
                       hls_cell_event.valid = '0'
                    report message_text & " exposed stale output after abort"
                    severity failure;
                if ref_hit_ready = '1' and hls_hit_ready = '1' then
                    recovered := true;
                    exit;
                end if;
            end loop;
            assert recovered
                report message_text & " did not recover input readiness"
                severity failure;
        end procedure abort_pending_output;

        variable observed       : gpx_cell_event_t;
        variable expected_faults : gpx_cell_collector_faults_t;
        variable start_value    : natural;
    begin
        wait_clocks(8);
        rst_n <= '1';
        wait_clocks(6);

        assert fn_validate_build_config(C_BUILD_CONFIG) = CFG_OK
            report "test profile build configuration is illegal"
            severity failure;
        clear_fault_state;

        case G_SCENARIO is
            when 0 =>
                -- Physical Drain always accepts seven Returns. Runtime
                -- visibility changes only the number retained in the Cell.
                for visible_count in 1 to 7 loop
                    max_hits <= to_unsigned(visible_count, 3);
                    wait_clocks(1);
                    for return_index in 0 to 6 loop
                        offer_both(
                            fn_hit_data(
                                0, 0, GPX_SLOPE_RISE,
                                16#10000# + visible_count * 16 +
                                    return_index,
                                0, visible_count, 5),
                            "Return sweep data");
                    end loop;
                    offer_both(
                        fn_hit_control(
                            GPX_HIT_DRAIN_DONE, 0, '1',
                            visible_count, 5),
                        "Return sweep Drain");

                    for cell_index in 0 to 8 loop
                        if cell_index = 0 then
                            compare_next_cell(
                                "Return sweep Cell visible=" &
                                    integer'image(visible_count) &
                                    " index=" & integer'image(cell_index),
                                observed,
                                4);
                        else
                            compare_next_cell(
                                "Return sweep Cell visible=" &
                                    integer'image(visible_count) &
                                    " index=" & integer'image(cell_index),
                                observed);
                        end if;
                        if cell_index = 0 then
                            assert observed.kind = GPX_CELL_DATA and
                                   observed.slope = GPX_SLOPE_RISE and
                                   to_integer(observed.stop_index) = 0 and
                                   to_integer(observed.hit_count) =
                                       visible_count and
                                   to_integer(observed.max_hits) =
                                       visible_count
                                report "Return sweep semantic mismatch"
                                severity failure;
                            for hit_index in 0 to 6 loop
                                if hit_index < visible_count then
                                    assert observed.hits(hit_index) =
                                        to_unsigned(
                                            16#10000# +
                                            visible_count * 16 + hit_index,
                                            C_GPX_HIT_WIDTH)
                                        report "Return sweep Hit mismatch"
                                        severity failure;
                                else
                                    assert observed.hits(hit_index) = 0
                                        report "filtered Return was exposed"
                                        severity failure;
                                end if;
                            end loop;
                        elsif cell_index = 8 then
                            assert observed.kind = GPX_CELL_DRAIN_DONE
                                report "Return sweep terminal mismatch"
                                severity failure;
                        end if;
                    end loop;
                end loop;
                assert_fault_equivalence("Return sweep faults");
                assert ref_fault_sticky =
                           C_GPX_CELL_COLLECTOR_FAULTS_CLEAR
                    report "intentional Return filtering raised a fault"
                    severity failure;

                -- Dedicated Fall Chip ordering is also part of the contract.
                max_hits <= to_unsigned(3, 3);
                wait_clocks(1);
                offer_both(
                    fn_hit_data(
                        2, 3, GPX_SLOPE_FALL, 16#12345#, 0, 20, 5),
                    "dedicated Fall data");
                offer_both(
                    fn_hit_control(
                        GPX_HIT_DRAIN_DONE, 2, '1', 20, 5),
                    "dedicated Fall Drain");
                compare_cell_count(9, "dedicated Fall output", 3);

            when 1 =>
                max_hits <= to_unsigned(7, 3);
                wait_clocks(1);
                for return_index in 0 to 1 loop
                    offer_both(
                        fn_hit_data(
                            0, 2, GPX_SLOPE_RISE,
                            16#10010# + return_index,
                            0, 30, 5),
                        "one-Chip Rise lower");
                    offer_both(
                        fn_hit_data(
                            0, 2, GPX_SLOPE_FALL,
                            16#00020# + return_index,
                            0, 30, 5),
                        "one-Chip Fall lower");
                end loop;
                offer_both(
                    fn_hit_control(
                        GPX_HIT_IFIFO1_DONE, 0, '0', 30, 5),
                    "one-Chip IFIFO1 done");
                compare_cell_count(9, "one-Chip lower output", 5);

                offer_both(
                    fn_hit_data(
                        0, 5, GPX_SLOPE_RISE, 16#15555#, 0, 30, 5),
                    "one-Chip Rise upper");
                offer_both(
                    fn_hit_data(
                        0, 5, GPX_SLOPE_FALL, 16#05555#, 0, 30, 5),
                    "one-Chip Fall upper");
                offer_both(
                    fn_hit_control(
                        GPX_HIT_DRAIN_DONE, 0, '1', 30, 5),
                    "one-Chip Drain done");
                compare_cell_count(9, "one-Chip upper output");

                offer_both(
                    fn_hit_data(
                        0, 1, GPX_SLOPE_RISE, 16#10101#, 0, 31, 5),
                    "one-Chip timeout data");
                offer_both(
                    fn_hit_control(
                        GPX_HIT_TIMEOUT, 0, '1', 31, 5, '1', "101"),
                    "one-Chip timeout");
                compare_cell_count(17, "one-Chip timeout output", 2);

                -- Abort with a Cell held by backpressure, then reuse the same
                -- Chip/STOP. The old Hit must not survive the epoch change.
                offer_both(
                    fn_hit_data(
                        0, 0, GPX_SLOPE_RISE, 16#0AAAA#, 0, 32, 5),
                    "abort stale data");
                offer_both(
                    fn_hit_control(
                        GPX_HIT_DRAIN_DONE, 0, '1', 32, 5),
                    "abort stale Drain");
                abort_pending_output("one-Chip abort");

                offer_both(
                    fn_hit_data(
                        0, 1, GPX_SLOPE_RISE, 16#15555#, 0, 33, 5),
                    "abort recovery data");
                offer_both(
                    fn_hit_control(
                        GPX_HIT_DRAIN_DONE, 0, '1', 33, 5),
                    "abort recovery Drain");
                for cell_index in 0 to 16 loop
                    compare_next_cell(
                        "abort recovery Cell index=" &
                            integer'image(cell_index),
                        observed);
                    if cell_index = 0 then
                        assert observed.hit_count = 0 and
                               observed.hits(0) = 0
                            report "aborted stale Hit survived"
                            severity failure;
                    elsif cell_index = 1 then
                        assert observed.hit_count = 1 and
                               observed.hits(0) =
                                   to_unsigned(16#15555#, 17)
                            report "abort recovery Hit mismatch"
                            severity failure;
                    end if;
                end loop;
                assert_fault_equivalence("one-Chip final faults");

            when 2 =>
                max_hits <= to_unsigned(2, 3);
                wait_clocks(1);
                for return_index in 0 to 7 loop
                    if return_index = 0 then
                        start_value := 1;
                    else
                        start_value := 0;
                    end if;
                    offer_both(
                        fn_hit_data(
                            0, 1, GPX_SLOPE_RISE,
                            16#10030# + return_index,
                            start_value,
                            40, 5),
                        "reduced overflow data");
                end loop;
                offer_both(
                    fn_hit_data(
                        0, 1, GPX_SLOPE_FALL,
                        16#00041#, 0, 40, 5),
                    "reduced unsupported slope");
                offer_both(
                    fn_hit_control(
                        GPX_HIT_TIMEOUT, 0, '1', 40, 5, '1', "110"),
                    "reduced timeout");
                compare_cell_count(7, "reduced timeout output", 4);
                assert_fault_equivalence("reduced fault summary");

                expected_faults := C_GPX_CELL_COLLECTOR_FAULTS_CLEAR;
                expected_faults.context_mismatch := '1';
                expected_faults.return_overflow := '1';
                expected_faults.start_number_nonzero := '1';
                assert ref_fault_sticky = expected_faults and
                       ref_pulse_seen = expected_faults
                    report "reduced expected fault summary mismatch"
                    severity failure;

                clear_fault_state;
                max_hits <= to_unsigned(3, 3);
                wait_clocks(1);
                offer_both(
                    fn_hit_data(
                        2, 5, GPX_SLOPE_FALL, 16#16666#, 0, 41, 5),
                    "reduced valid Fall data");
                offer_both(
                    fn_hit_control(
                        GPX_HIT_DRAIN_DONE, 2, '1', 41, 5),
                    "reduced valid Fall Drain");
                compare_cell_count(7, "reduced valid Fall output");
                assert_fault_equivalence("reduced clean summary");
                assert ref_fault_sticky =
                           C_GPX_CELL_COLLECTOR_FAULTS_CLEAR
                    report "reduced valid path raised a fault"
                    severity failure;

            when others =>
                max_hits <= to_unsigned(4, 3);
                wait_clocks(1);
                for chip_index in 0 to 3 loop
                    offer_both(
                        fn_hit_data(
                            chip_index, chip_index, GPX_SLOPE_RISE,
                            16#10000# + chip_index,
                            0, 50 + chip_index, 5),
                        "all-dual Rise data");
                    offer_both(
                        fn_hit_data(
                            chip_index, chip_index, GPX_SLOPE_FALL,
                            16#01000# + chip_index,
                            0, 50 + chip_index, 5),
                        "all-dual Fall data");
                    offer_both(
                        fn_hit_control(
                            GPX_HIT_DRAIN_DONE, chip_index, '1',
                            50 + chip_index, 5),
                        "all-dual Drain");

                    for cell_index in 0 to 16 loop
                        if cell_index = 0 then
                            compare_next_cell(
                                "all-dual Cell chip=" &
                                    integer'image(chip_index) &
                                    " index=" & integer'image(cell_index),
                                observed,
                                2);
                        else
                            compare_next_cell(
                                "all-dual Cell chip=" &
                                    integer'image(chip_index) &
                                    " index=" & integer'image(cell_index),
                                observed);
                        end if;
                        if cell_index = chip_index then
                            assert observed.slope = GPX_SLOPE_RISE and
                                   observed.hit_count = 1 and
                                   observed.hits(0) = to_unsigned(
                                       16#10000# + chip_index, 17)
                                report "all-dual Rise semantic mismatch"
                                severity failure;
                        elsif cell_index = 8 + chip_index then
                            assert observed.slope = GPX_SLOPE_FALL and
                                   observed.hit_count = 1 and
                                   observed.hits(0) = to_unsigned(
                                       16#01000# + chip_index, 17)
                                report "all-dual Fall semantic mismatch"
                                severity failure;
                        elsif cell_index = 16 then
                            assert observed.kind = GPX_CELL_DRAIN_DONE
                                report "all-dual terminal mismatch"
                                severity failure;
                        end if;
                    end loop;
                end loop;
                assert_fault_equivalence("all-dual faults");
                assert ref_fault_sticky =
                           C_GPX_CELL_COLLECTOR_FAULTS_CLEAR
                    report "all-dual valid path raised a fault"
                    severity failure;
        end case;

        report "LIDAR_V3_GPX_CELL_COLLECTOR_DIFF_PASS clk_mhz=" &
            integer'image(G_CLK_MHZ) & " scenario=" &
            integer'image(G_SCENARIO)
            severity note;
        simulation_done <= true;
        wait;
    end process p_stimulus;

end architecture sim;

library ieee;
use ieee.std_logic_1164.all;

entity tb_lidar_gpx_cell_collector_hls_diff_dedicated_150 is
end entity;
architecture sim of tb_lidar_gpx_cell_collector_hls_diff_dedicated_150 is
begin
    u_test : entity work.tb_lidar_gpx_cell_collector_hls_diff
        generic map (G_CLK_MHZ => 150, G_SCENARIO => 0);
end architecture;

library ieee;
use ieee.std_logic_1164.all;

entity tb_lidar_gpx_cell_collector_hls_diff_dedicated_200 is
end entity;
architecture sim of tb_lidar_gpx_cell_collector_hls_diff_dedicated_200 is
begin
    u_test : entity work.tb_lidar_gpx_cell_collector_hls_diff
        generic map (G_CLK_MHZ => 200, G_SCENARIO => 0);
end architecture;

library ieee;
use ieee.std_logic_1164.all;

entity tb_lidar_gpx_cell_collector_hls_diff_one_dual_150 is
end entity;
architecture sim of tb_lidar_gpx_cell_collector_hls_diff_one_dual_150 is
begin
    u_test : entity work.tb_lidar_gpx_cell_collector_hls_diff
        generic map (G_CLK_MHZ => 150, G_SCENARIO => 1);
end architecture;

library ieee;
use ieee.std_logic_1164.all;

entity tb_lidar_gpx_cell_collector_hls_diff_one_dual_200 is
end entity;
architecture sim of tb_lidar_gpx_cell_collector_hls_diff_one_dual_200 is
begin
    u_test : entity work.tb_lidar_gpx_cell_collector_hls_diff
        generic map (G_CLK_MHZ => 200, G_SCENARIO => 1);
end architecture;

library ieee;
use ieee.std_logic_1164.all;

entity tb_lidar_gpx_cell_collector_hls_diff_reduced_150 is
end entity;
architecture sim of tb_lidar_gpx_cell_collector_hls_diff_reduced_150 is
begin
    u_test : entity work.tb_lidar_gpx_cell_collector_hls_diff
        generic map (G_CLK_MHZ => 150, G_SCENARIO => 2);
end architecture;

library ieee;
use ieee.std_logic_1164.all;

entity tb_lidar_gpx_cell_collector_hls_diff_reduced_200 is
end entity;
architecture sim of tb_lidar_gpx_cell_collector_hls_diff_reduced_200 is
begin
    u_test : entity work.tb_lidar_gpx_cell_collector_hls_diff
        generic map (G_CLK_MHZ => 200, G_SCENARIO => 2);
end architecture;

library ieee;
use ieee.std_logic_1164.all;

entity tb_lidar_gpx_cell_collector_hls_diff_all_dual_150 is
end entity;
architecture sim of tb_lidar_gpx_cell_collector_hls_diff_all_dual_150 is
begin
    u_test : entity work.tb_lidar_gpx_cell_collector_hls_diff
        generic map (G_CLK_MHZ => 150, G_SCENARIO => 3);
end architecture;

library ieee;
use ieee.std_logic_1164.all;

entity tb_lidar_gpx_cell_collector_hls_diff_all_dual_200 is
end entity;
architecture sim of tb_lidar_gpx_cell_collector_hls_diff_all_dual_200 is
begin
    u_test : entity work.tb_lidar_gpx_cell_collector_hls_diff
        generic map (G_CLK_MHZ => 200, G_SCENARIO => 3);
end architecture;
