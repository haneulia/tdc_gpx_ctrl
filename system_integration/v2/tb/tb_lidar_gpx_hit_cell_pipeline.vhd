-- ============================================================================
-- 테스트 자산 목적: B6 Hit decoder와 B7 Cell collector 직접 연결 계약을 검증한다.
-- 핵심 검증 계약: context/ready pipeline, Return 순서, backpressure와 무손실 전달이다.
-- 관련 RTL: lidar_gpx_hit_decoder, lidar_gpx_cell_collector.
-- 실행 회귀: scripts/run_v2_gpx_cell_collector.ps1
-- 유지보수 주의: 두 블록 사이 register stage 변경 시 latency와 payload identity를 같이 본다.
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.env.all;

use work.lidar_build_pkg.all;
use work.lidar_event_types_pkg.all;
use work.lidar_gpx_event_pkg.all;
use work.lidar_gpx_data_pkg.all;

entity tb_lidar_gpx_hit_cell_pipeline is
    generic (
        G_CLK_MHZ : positive := 150
    );
end entity tb_lidar_gpx_hit_cell_pipeline;

architecture sim of tb_lidar_gpx_hit_cell_pipeline is

    function fn_build_config return lidar_build_config_t is
        variable result : lidar_build_config_t := C_DEFAULT_BUILD_CONFIG;
    begin
        result.proc_clk_mhz := G_CLK_MHZ;
        result.tdc_clk_mhz := 200;
        result.stream_clock_mode := STREAM_CLOCK_ASYNC;
        result.num_chips := 1;
        result.stops_per_chip := 8;
        result.max_returns_per_stop := 7;
        result.rise_capability_mask := "0001";
        result.fall_capability_mask := "0001";
        result.output_width := 32;
        result.num_faces := 4;
        result.enable_echo_receiver := false;
        result.enable_echo_simulation := false;
        return result;
    end function fn_build_config;

    function fn_shot_context(
        shot_index : natural
    ) return shot_start_event_t is
        variable result : shot_start_event_t := C_SHOT_START_EVENT_IDLE;
    begin
        result.valid := '1';
        result.request.valid := '1';
        result.request.face_index := to_unsigned(1, 3);
        result.request.position := to_unsigned(
            100 + shot_index, result.request.position'length);
        result.request.shot_index := to_unsigned(shot_index, 16);
        result.request.source_sim := '0';
        result.request.active_version := to_unsigned(5, 16);
        result.fire_to_t0_clks := to_unsigned(7, 32);
        return result;
    end function fn_shot_context;

    function fn_raw_data(
        stop_index   : natural;
        slope_value  : gpx_slope_t;
        hit_value    : natural;
        start_number : natural;
        shot_index   : natural
    ) return gpx_raw_event_t is
        variable result : gpx_raw_event_t := C_GPX_RAW_EVENT_IDLE;
    begin
        result.valid := '1';
        result.kind := GPX_RAW_DATA;
        if stop_index >= 4 then
            result.ififo_id := '1';
        end if;
        result.raw_word(
            C_GPX_RAW_CHACODE_HI downto C_GPX_RAW_CHACODE_LO) :=
            std_logic_vector(to_unsigned(stop_index mod 4, 2));
        result.raw_word(C_GPX_RAW_START_HI downto C_GPX_RAW_START_LO) :=
            std_logic_vector(to_unsigned(start_number, 8));
        result.raw_word(C_GPX_RAW_SLOPE_BIT) :=
            fn_gpx_slope_to_bit(slope_value);
        result.raw_word(C_GPX_RAW_HIT_HI downto C_GPX_RAW_HIT_LO) :=
            std_logic_vector(to_unsigned(hit_value, C_GPX_HIT_WIDTH));
        result.shot_context := fn_shot_context(shot_index);
        result.chip_shot_seq := to_unsigned(shot_index, 16);
        return result;
    end function fn_raw_data;

    function fn_raw_control(
        kind_value : gpx_raw_event_kind_t;
        shot_index : natural
    ) return gpx_raw_event_t is
        variable result : gpx_raw_event_t := C_GPX_RAW_EVENT_IDLE;
    begin
        result.valid := '1';
        result.kind := kind_value;
        result.ififo_id := '1';
        result.shot_context := fn_shot_context(shot_index);
        result.chip_shot_seq := to_unsigned(shot_index, 16);
        return result;
    end function fn_raw_control;

    constant C_BUILD_CONFIG : lidar_build_config_t := fn_build_config;
    constant C_CLK_PERIOD : time := 1 us / G_CLK_MHZ;

    signal clk : std_logic := '0';
    signal rst_n : std_logic := '0';
    signal abort_event : std_logic := '0';
    signal clear_sticky : std_logic := '0';
    signal raw_event : gpx_raw_event_t := C_GPX_RAW_EVENT_IDLE;
    signal raw_ready : std_logic;
    signal hit_event : gpx_hit_event_t;
    signal hit_ready : std_logic;
    signal cell_event : gpx_cell_event_t;
    signal cell_ready : std_logic := '0';
    signal decoder_fault_pulse : gpx_hit_decoder_faults_t;
    signal decoder_fault_sticky : gpx_hit_decoder_faults_t;
    signal collector_fault_pulse : gpx_cell_collector_faults_t;
    signal collector_fault_sticky : gpx_cell_collector_faults_t;
    signal collector_pulse_seen : gpx_cell_collector_faults_t :=
        C_GPX_CELL_COLLECTOR_FAULTS_CLEAR;

begin

    clk <= not clk after C_CLK_PERIOD / 2;

    u_decoder : entity work.lidar_gpx_hit_decoder
        generic map (
            G_BUILD_CONFIG => C_BUILD_CONFIG
        )
        port map (
            i_clk          => clk,
            i_rst_n        => rst_n,
            i_abort        => abort_event,
            i_clear_sticky => clear_sticky,
            i_raw_event    => raw_event,
            o_raw_ready    => raw_ready,
            o_hit_event    => hit_event,
            i_hit_ready    => hit_ready,
            o_fault_pulse  => decoder_fault_pulse,
            o_fault_sticky => decoder_fault_sticky
        );

    u_collector : entity work.lidar_gpx_cell_collector
        generic map (
            G_BUILD_CONFIG => C_BUILD_CONFIG
        )
        port map (
            i_clk               => clk,
            i_rst_n             => rst_n,
            i_abort             => abort_event,
            i_clear_sticky      => clear_sticky,
            i_active_version    => to_unsigned(5, 16),
            i_max_hits_per_stop => to_unsigned(3, 3),
            i_hit_event         => hit_event,
            o_hit_ready         => hit_ready,
            o_cell_event        => cell_event,
            i_cell_ready        => cell_ready,
            o_fault_pulse       => collector_fault_pulse,
            o_fault_sticky      => collector_fault_sticky
        );

    p_pulse_monitor : process (clk)
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                collector_pulse_seen <=
                    C_GPX_CELL_COLLECTOR_FAULTS_CLEAR;
            else
                collector_pulse_seen.context_mismatch <=
                    collector_pulse_seen.context_mismatch or
                    collector_fault_pulse.context_mismatch;
                collector_pulse_seen.return_overflow <=
                    collector_pulse_seen.return_overflow or
                    collector_fault_pulse.return_overflow;
                collector_pulse_seen.start_number_nonzero <=
                    collector_pulse_seen.start_number_nonzero or
                    collector_fault_pulse.start_number_nonzero;
                collector_pulse_seen.hit_capacity_drop <=
                    collector_pulse_seen.hit_capacity_drop or
                    collector_fault_pulse.hit_capacity_drop;
            end if;
        end if;
    end process p_pulse_monitor;

    p_stimulus : process
        variable expected_hits : gpx_hit_value_array_t :=
            (others => (others => '0'));

        procedure wait_clocks(count : positive) is
        begin
            for index in 1 to count loop
                wait until rising_edge(clk);
            end loop;
            wait for 1 ps;
        end procedure wait_clocks;

        procedure send_raw(value : gpx_raw_event_t) is
        begin
            raw_event <= value;
            loop
                wait until rising_edge(clk);
                exit when raw_ready = '1';
            end loop;
            wait for 1 ps;
            raw_event <= C_GPX_RAW_EVENT_IDLE;
        end procedure send_raw;

        procedure expect_data(
            stop_index    : natural;
            slope_value   : gpx_slope_t;
            hit_count     : natural;
            hits          : gpx_hit_value_array_t;
            hit_dropped   : std_logic;
            faulted       : std_logic;
            stall_cycles  : natural := 0;
            return_overflow : std_logic := '0'
        ) is
            variable held_value : gpx_cell_event_t;
        begin
            loop
                wait until falling_edge(clk);
                exit when cell_event.valid = '1';
            end loop;
            held_value := cell_event;

            assert cell_event.kind = GPX_CELL_DATA and
                   to_integer(cell_event.stop_index) = stop_index and
                   cell_event.slope = slope_value and
                   to_integer(cell_event.hit_count) = hit_count and
                   cell_event.hits = hits and
                   cell_event.hit_dropped = hit_dropped and
                   cell_event.return_overflow = return_overflow and
                   cell_event.faulted = faulted
                report "V2-B6B7-TB Cell data mismatch"
                severity failure;

            for index in 1 to stall_cycles loop
                wait until rising_edge(clk);
                wait until falling_edge(clk);
                assert cell_event = held_value
                    report "V2-B6B7-TB Cell changed under backpressure"
                    severity failure;
            end loop;

            cell_ready <= '1';
            wait until rising_edge(clk);
            cell_ready <= '0';
        end procedure expect_data;

        procedure expect_control(
            faulted : std_logic
        ) is
        begin
            loop
                wait until falling_edge(clk);
                exit when cell_event.valid = '1';
            end loop;
            assert cell_event.kind = GPX_CELL_DRAIN_DONE and
                   cell_event.faulted = faulted
                report "V2-B6B7-TB terminal control mismatch"
                severity failure;
            cell_ready <= '1';
            wait until rising_edge(clk);
            cell_ready <= '0';
        end procedure expect_control;

    begin
        wait_clocks(5);
        rst_n <= '1';
        wait_clocks(3);

        for return_number in 0 to 7 loop
            if return_number = 3 then
                send_raw(fn_raw_data(
                    2, GPX_SLOPE_RISE, 16#10020# + return_number,
                    1, 1));
            else
                send_raw(fn_raw_data(
                    2, GPX_SLOPE_RISE, 16#10020# + return_number,
                    0, 1));
            end if;
        end loop;
        send_raw(fn_raw_data(
            2, GPX_SLOPE_FALL, 16#00041#, 0, 1));
        send_raw(fn_raw_data(
            2, GPX_SLOPE_FALL, 16#00042#, 0, 1));
        send_raw(fn_raw_control(GPX_RAW_DRAIN_DONE, 1));

        for slope_value in GPX_SLOPE_RISE downto GPX_SLOPE_FALL loop
            for stop_value in 0 to 7 loop
                expected_hits := (others => (others => '0'));
                if stop_value = 2 and slope_value = GPX_SLOPE_RISE then
                    expected_hits(0) := to_unsigned(16#10020#, 17);
                    expected_hits(1) := to_unsigned(16#10021#, 17);
                    expected_hits(2) := to_unsigned(16#10022#, 17);
                    expect_data(stop_value, slope_value, 3,
                        expected_hits, '0', '1', 4, '1');
                elsif stop_value = 2 and slope_value = GPX_SLOPE_FALL then
                    expected_hits(0) := to_unsigned(16#00041#, 17);
                    expected_hits(1) := to_unsigned(16#00042#, 17);
                    expect_data(stop_value, slope_value, 2,
                        expected_hits, '0', '1');
                else
                    expect_data(stop_value, slope_value, 0,
                        expected_hits, '0', '1');
                end if;
            end loop;
        end loop;
        expect_control('1');

        assert decoder_fault_sticky = C_GPX_HIT_DECODER_FAULTS_CLEAR
            report "V2-B6B7-TB B6 unexpectedly rejected a valid raw event"
            severity failure;
        assert collector_fault_sticky.context_mismatch = '0' and
               collector_fault_sticky.return_overflow = '1' and
               collector_fault_sticky.start_number_nonzero = '1' and
               collector_fault_sticky.hit_capacity_drop = '0' and
               collector_pulse_seen.context_mismatch = '0' and
               collector_pulse_seen.return_overflow = '1' and
               collector_pulse_seen.start_number_nonzero = '1' and
               collector_pulse_seen.hit_capacity_drop = '0'
            report "V2-B6B7-TB B7 ownership diagnostics mismatch"
            severity failure;

        clear_sticky <= '1';
        wait_clocks(1);
        clear_sticky <= '0';
        wait_clocks(1);

        -- Shot 2 proves that the per-Chip sequential scrub hides all Shot 1
        -- metadata without resetting the seven 17-bit payload banks.
        send_raw(fn_raw_data(
            0, GPX_SLOPE_RISE, 16#05555#, 0, 2));
        send_raw(fn_raw_control(GPX_RAW_DRAIN_DONE, 2));
        for slope_value in GPX_SLOPE_RISE downto GPX_SLOPE_FALL loop
            for stop_value in 0 to 7 loop
                expected_hits := (others => (others => '0'));
                if stop_value = 0 and slope_value = GPX_SLOPE_RISE then
                    expected_hits(0) := to_unsigned(16#05555#, 17);
                    expect_data(stop_value, slope_value, 1,
                        expected_hits, '0', '0');
                else
                    expect_data(stop_value, slope_value, 0,
                        expected_hits, '0', '0');
                end if;
            end loop;
        end loop;
        expect_control('0');

        report "LIDAR_V2_GPX_HIT_CELL_PIPELINE_PASS clk_mhz=" &
            integer'image(G_CLK_MHZ)
            severity note;
        stop;
        wait;
    end process p_stimulus;

end architecture sim;

library ieee;
use ieee.std_logic_1164.all;

entity tb_lidar_gpx_hit_cell_pipeline_150 is
end entity tb_lidar_gpx_hit_cell_pipeline_150;

architecture sim of tb_lidar_gpx_hit_cell_pipeline_150 is
begin
    u_test : entity work.tb_lidar_gpx_hit_cell_pipeline
        generic map (G_CLK_MHZ => 150);
end architecture sim;

library ieee;
use ieee.std_logic_1164.all;

entity tb_lidar_gpx_hit_cell_pipeline_200 is
end entity tb_lidar_gpx_hit_cell_pipeline_200;

architecture sim of tb_lidar_gpx_hit_cell_pipeline_200 is
begin
    u_test : entity work.tb_lidar_gpx_hit_cell_pipeline
        generic map (G_CLK_MHZ => 200);
end architecture sim;
