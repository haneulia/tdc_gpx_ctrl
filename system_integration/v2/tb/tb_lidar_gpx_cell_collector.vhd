library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.env.all;

use work.lidar_build_pkg.all;
use work.lidar_event_types_pkg.all;
use work.lidar_gpx_event_pkg.all;
use work.lidar_gpx_data_pkg.all;

entity tb_lidar_gpx_cell_collector is
    generic (
        G_CLK_MHZ  : positive := 150;
        G_SCENARIO : natural range 0 to 2 := 0
    );
end entity tb_lidar_gpx_cell_collector;

architecture sim of tb_lidar_gpx_cell_collector is

    function fn_build_config return lidar_build_config_t is
        variable result : lidar_build_config_t := C_DEFAULT_BUILD_CONFIG;
    begin
        result.proc_clk_mhz          := G_CLK_MHZ;
        result.tdc_clk_mhz           := 200;
        result.output_width          := 32;
        result.num_faces             := 4;
        result.enable_echo_receiver  := false;
        result.enable_echo_simulation := false;

        if G_SCENARIO = 0 then
            result.num_chips            := 4;
            result.rise_capability_mask := "0011";
            result.fall_capability_mask := "1100";
        else
            result.num_chips            := 1;
            result.rise_capability_mask := "0001";
            result.fall_capability_mask := "0001";
        end if;
        return result;
    end function fn_build_config;

    function fn_shot_context(
        shot_index     : natural;
        active_version : natural
    ) return shot_start_event_t is
        variable result : shot_start_event_t := C_SHOT_START_EVENT_IDLE;
    begin
        result.valid                        := '1';
        result.request.valid                := '1';
        result.request.face_index           := to_unsigned(1, 3);
        result.request.position             := to_unsigned(100 + shot_index, 15);
        result.request.shot_index            := to_unsigned(shot_index, 16);
        result.request.active_version        := to_unsigned(active_version, 16);
        result.request.source_latency_valid := '1';
        result.request.source_latency_clks  := to_unsigned(
            3, result.request.source_latency_clks'length);
        result.fire_to_t0_clks              := to_unsigned(7, 32);
        return result;
    end function fn_shot_context;

    function fn_hit_data(
        chip_index     : natural;
        stop_index     : natural;
        slope_value    : gpx_slope_t;
        return_index   : natural;
        hit_value      : natural;
        start_number   : natural;
        shot_index     : natural;
        active_version : natural
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
        result.return_index  := to_unsigned(return_index, 3);
        result.hit           := to_unsigned(hit_value, C_GPX_HIT_WIDTH);
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
    signal active_version : unsigned(15 downto 0) := to_unsigned(5, 16);
    signal max_hits       : unsigned(2 downto 0) := to_unsigned(3, 3);
    signal hit_event      : gpx_hit_event_t := C_GPX_HIT_EVENT_IDLE;
    signal hit_ready      : std_logic;
    signal cell_event     : gpx_cell_event_t;
    signal cell_ready     : std_logic := '0';
    signal fault_pulse    : gpx_cell_collector_faults_t;
    signal fault_sticky   : gpx_cell_collector_faults_t;
    signal pulse_seen     : gpx_cell_collector_faults_t :=
        C_GPX_CELL_COLLECTOR_FAULTS_CLEAR;

begin

    clk <= not clk after C_CLK_PERIOD / 2;

    u_dut : entity work.lidar_gpx_cell_collector
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
            i_hit_event         => hit_event,
            o_hit_ready         => hit_ready,
            o_cell_event        => cell_event,
            i_cell_ready        => cell_ready,
            o_fault_pulse       => fault_pulse,
            o_fault_sticky      => fault_sticky
        );

    p_pulse_monitor : process (clk)
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                pulse_seen <= C_GPX_CELL_COLLECTOR_FAULTS_CLEAR;
            else
                pulse_seen.context_mismatch <=
                    pulse_seen.context_mismatch or
                    fault_pulse.context_mismatch;
                pulse_seen.return_sequence_error <=
                    pulse_seen.return_sequence_error or
                    fault_pulse.return_sequence_error;
                pulse_seen.start_number_nonzero <=
                    pulse_seen.start_number_nonzero or
                    fault_pulse.start_number_nonzero;
                pulse_seen.hit_capacity_drop <=
                    pulse_seen.hit_capacity_drop or
                    fault_pulse.hit_capacity_drop;
            end if;
        end if;
    end process p_pulse_monitor;

    p_stimulus : process
        variable expected_hits : gpx_hit_value_array_t :=
            (others => (others => '0'));

        procedure wait_clocks(count : natural) is
        begin
            for index in 1 to count loop
                wait until rising_edge(clk);
            end loop;
        end procedure wait_clocks;

        procedure send_event(value : gpx_hit_event_t) is
        begin
            loop
                wait until falling_edge(clk);
                exit when hit_ready = '1';
            end loop;
            hit_event <= value;
            wait until rising_edge(clk);
            hit_event <= C_GPX_HIT_EVENT_IDLE;
        end procedure send_event;

        procedure expect_data(
            chip_index    : natural;
            stop_index    : natural;
            slope_value   : gpx_slope_t;
            hit_count     : natural;
            hits          : gpx_hit_value_array_t;
            hit_dropped   : std_logic := '0';
            error_fill    : std_logic := '0';
            faulted       : std_logic := '0';
            stall_cycles  : natural := 0
        ) is
            variable held_value : gpx_cell_event_t;
        begin
            loop
                wait until falling_edge(clk);
                exit when cell_event.valid = '1';
            end loop;
            held_value := cell_event;

            assert cell_event.kind = GPX_CELL_DATA and
                   to_integer(cell_event.chip_index) = chip_index and
                   to_integer(cell_event.stop_index) = stop_index and
                   cell_event.slope = slope_value and
                   to_integer(cell_event.hit_count) = hit_count and
                   cell_event.hit_dropped = hit_dropped and
                   cell_event.error_fill = error_fill and
                   cell_event.faulted = faulted
                report "V2-B7-TB Cell metadata mismatch"
                severity failure;

            for hit_index in 0 to C_MAX_RETURNS_PER_STOP - 1 loop
                assert cell_event.hits(hit_index) = hits(hit_index)
                    report "V2-B7-TB Cell Hit mismatch at index " &
                        integer'image(hit_index)
                    severity failure;
            end loop;

            for index in 1 to stall_cycles loop
                wait until rising_edge(clk);
                wait until falling_edge(clk);
                assert cell_event = held_value
                    report "V2-B7-TB Cell changed under backpressure"
                    severity failure;
            end loop;

            cell_ready <= '1';
            wait until rising_edge(clk);
            cell_ready <= '0';
        end procedure expect_data;

        procedure expect_control(
            kind_value     : gpx_cell_event_kind_t;
            chip_index     : natural;
            error_fill     : std_logic := '0';
            faulted        : std_logic := '0';
            timeout_cause  : std_logic_vector(2 downto 0) := "000"
        ) is
        begin
            loop
                wait until falling_edge(clk);
                exit when cell_event.valid = '1';
            end loop;

            assert cell_event.kind = kind_value and
                   to_integer(cell_event.chip_index) = chip_index and
                   cell_event.error_fill = error_fill and
                   cell_event.faulted = faulted and
                   cell_event.timeout_cause = timeout_cause
                report "V2-B7-TB control Cell mismatch"
                severity failure;

            cell_ready <= '1';
            wait until rising_edge(clk);
            cell_ready <= '0';
        end procedure expect_control;

        procedure expect_blank_range(
            chip_index   : natural;
            slope_value  : gpx_slope_t;
            first_stop   : natural;
            last_stop    : natural;
            error_fill   : std_logic := '0';
            faulted      : std_logic := '0'
        ) is
            variable blank_hits : gpx_hit_value_array_t :=
                (others => (others => '0'));
        begin
            for stop_value in first_stop to last_stop loop
                expect_data(
                    chip_index  => chip_index,
                    stop_index  => stop_value,
                    slope_value => slope_value,
                    hit_count   => 0,
                    hits        => blank_hits,
                    error_fill  => error_fill,
                    faulted     => faulted);
            end loop;
        end procedure expect_blank_range;

    begin
        wait_clocks(4);
        rst_n <= '1';
        wait_clocks(2);

        case G_SCENARIO is
            when 0 =>
                max_hits <= to_unsigned(3, 3);
                send_event(fn_hit_data(
                    0, 0, GPX_SLOPE_RISE, 0, 16#10001#, 0, 1, 5));
                send_event(fn_hit_data(
                    0, 0, GPX_SLOPE_RISE, 1, 16#00002#, 0, 1, 5));
                send_event(fn_hit_data(
                    0, 0, GPX_SLOPE_RISE, 2, 16#10003#, 0, 1, 5));
                send_event(fn_hit_data(
                    0, 0, GPX_SLOPE_RISE, 3, 16#00004#, 0, 1, 5));
                send_event(fn_hit_data(
                    0, 2, GPX_SLOPE_RISE, 0, 16#05555#, 0, 1, 5));
                send_event(fn_hit_control(
                    GPX_HIT_IFIFO1_DONE, 0, '0', 1, 5));

                expected_hits := (others => (others => '0'));
                expected_hits(0) := to_unsigned(16#10001#, 17);
                expected_hits(1) := to_unsigned(16#00002#, 17);
                expected_hits(2) := to_unsigned(16#10003#, 17);
                expect_data(0, 0, GPX_SLOPE_RISE, 3,
                    expected_hits, '1', '0', '0', 3);
                expected_hits := (others => (others => '0'));
                expect_data(0, 1, GPX_SLOPE_RISE, 0, expected_hits);
                expected_hits(0) := to_unsigned(16#05555#, 17);
                expect_data(0, 2, GPX_SLOPE_RISE, 1, expected_hits);
                expected_hits := (others => (others => '0'));
                expect_data(0, 3, GPX_SLOPE_RISE, 0, expected_hits);
                expect_control(GPX_CELL_IFIFO1_DONE, 0);

                send_event(fn_hit_data(
                    0, 4, GPX_SLOPE_RISE, 0, 16#1ABCD#, 0, 1, 5));
                send_event(fn_hit_control(
                    GPX_HIT_DRAIN_DONE, 0, '1', 1, 5));
                expected_hits(0) := to_unsigned(16#1ABCD#, 17);
                expect_data(0, 4, GPX_SLOPE_RISE, 1, expected_hits);
                expect_blank_range(0, GPX_SLOPE_RISE, 5, 7);
                expect_control(GPX_CELL_DRAIN_DONE, 0);

                send_event(fn_hit_data(
                    2, 0, GPX_SLOPE_FALL, 0, 16#12345#, 0, 2, 5));
                send_event(fn_hit_control(
                    GPX_HIT_IFIFO1_DONE, 2, '0', 2, 5));
                expected_hits := (others => (others => '0'));
                expected_hits(0) := to_unsigned(16#12345#, 17);
                expect_data(2, 0, GPX_SLOPE_FALL, 1, expected_hits);
                expect_blank_range(2, GPX_SLOPE_FALL, 1, 3);
                expect_control(GPX_CELL_IFIFO1_DONE, 2);
                send_event(fn_hit_control(
                    GPX_HIT_DRAIN_DONE, 2, '1', 2, 5));
                expect_blank_range(2, GPX_SLOPE_FALL, 4, 7);
                expect_control(GPX_CELL_DRAIN_DONE, 2);

                assert fault_sticky.hit_capacity_drop = '1' and
                       pulse_seen.hit_capacity_drop = '1'
                    report "V2-B7-TB capacity drop diagnostic missing"
                    severity failure;

            when 1 =>
                max_hits <= to_unsigned(7, 3);
                send_event(fn_hit_data(
                    0, 2, GPX_SLOPE_RISE, 0, 16#10011#, 0, 3, 5));
                send_event(fn_hit_data(
                    0, 2, GPX_SLOPE_FALL, 0, 16#00021#, 0, 3, 5));
                send_event(fn_hit_data(
                    0, 2, GPX_SLOPE_RISE, 1, 16#10012#, 0, 3, 5));
                send_event(fn_hit_data(
                    0, 2, GPX_SLOPE_FALL, 1, 16#00022#, 0, 3, 5));
                send_event(fn_hit_control(
                    GPX_HIT_IFIFO1_DONE, 0, '0', 3, 5));

                for slope_value in GPX_SLOPE_RISE downto GPX_SLOPE_FALL loop
                    for stop_value in 0 to 3 loop
                        expected_hits := (others => (others => '0'));
                        if stop_value = 2 then
                            if slope_value = GPX_SLOPE_RISE then
                                expected_hits(0) := to_unsigned(16#10011#, 17);
                                expected_hits(1) := to_unsigned(16#10012#, 17);
                            else
                                expected_hits(0) := to_unsigned(16#00021#, 17);
                                expected_hits(1) := to_unsigned(16#00022#, 17);
                            end if;
                            expect_data(0, stop_value, slope_value, 2,
                                expected_hits);
                        else
                            expect_data(0, stop_value, slope_value, 0,
                                expected_hits);
                        end if;
                    end loop;
                end loop;
                expect_control(GPX_CELL_IFIFO1_DONE, 0);

                send_event(fn_hit_control(
                    GPX_HIT_DRAIN_DONE, 0, '1', 3, 5));
                expect_blank_range(0, GPX_SLOPE_RISE, 4, 7);
                expect_blank_range(0, GPX_SLOPE_FALL, 4, 7);
                expect_control(GPX_CELL_DRAIN_DONE, 0);

            when others =>
                max_hits <= to_unsigned(2, 3);
                send_event(fn_hit_data(
                    0, 1, GPX_SLOPE_RISE, 0, 16#10031#, 1, 10, 5));
                send_event(fn_hit_data(
                    0, 1, GPX_SLOPE_RISE, 2, 16#00032#, 0, 10, 5));
                send_event(fn_hit_data(
                    0, 1, GPX_SLOPE_RISE, 1, 16#10033#, 0, 10, 5));
                send_event(fn_hit_data(
                    0, 1, GPX_SLOPE_RISE, 2, 16#00034#, 0, 10, 5));
                send_event(fn_hit_data(
                    0, 1, GPX_SLOPE_FALL, 0, 16#10041#, 0, 10, 6));
                send_event(fn_hit_control(
                    GPX_HIT_TIMEOUT, 0, '1', 10, 5, '1', "101"));

                for slope_value in GPX_SLOPE_RISE downto GPX_SLOPE_FALL loop
                    for stop_value in 0 to 7 loop
                        expected_hits := (others => (others => '0'));
                        if stop_value = 1 and
                           slope_value = GPX_SLOPE_RISE then
                            expected_hits(0) := to_unsigned(16#10031#, 17);
                            expected_hits(1) := to_unsigned(16#10033#, 17);
                            expect_data(0, stop_value, slope_value, 2,
                                expected_hits, '1', '1', '1');
                        elsif stop_value = 1 and
                              slope_value = GPX_SLOPE_FALL then
                            expected_hits(0) := to_unsigned(16#10041#, 17);
                            expect_data(0, stop_value, slope_value, 1,
                                expected_hits, '0', '1', '1');
                        else
                            expect_data(0, stop_value, slope_value, 0,
                                expected_hits, '0', '1', '1');
                        end if;
                    end loop;
                end loop;
                expect_control(
                    GPX_CELL_TIMEOUT, 0, '1', '1', "101");

                assert fault_sticky.context_mismatch = '1' and
                       fault_sticky.return_sequence_error = '1' and
                       fault_sticky.start_number_nonzero = '1' and
                       fault_sticky.hit_capacity_drop = '1' and
                       pulse_seen.context_mismatch = '1' and
                       pulse_seen.return_sequence_error = '1' and
                       pulse_seen.start_number_nonzero = '1' and
                       pulse_seen.hit_capacity_drop = '1'
                    report "V2-B7-TB fault pulse/sticky summary mismatch"
                    severity failure;

                clear_sticky <= '1';
                wait_clocks(1);
                clear_sticky <= '0';
                wait_clocks(1);
                assert fault_sticky =
                       C_GPX_CELL_COLLECTOR_FAULTS_CLEAR
                    report "V2-B7-TB sticky clear failed"
                    severity failure;

                send_event(fn_hit_data(
                    0, 0, GPX_SLOPE_RISE, 0, 16#10055#, 0, 11, 5));
                send_event(fn_hit_control(
                    GPX_HIT_IFIFO1_DONE, 0, '0', 11, 5));
                loop
                    wait until falling_edge(clk);
                    exit when cell_event.valid = '1';
                end loop;
                abort_event <= '1';
                wait until rising_edge(clk);
                abort_event <= '0';
                wait until falling_edge(clk);
                assert cell_event.valid = '0' and hit_ready = '1'
                    report "V2-B7-TB abort did not clear pending Cell"
                    severity failure;
        end case;

        report "LIDAR_V2_GPX_CELL_COLLECTOR_PASS clk_mhz=" &
            integer'image(G_CLK_MHZ) & " scenario=" &
            integer'image(G_SCENARIO)
            severity note;
        stop;
        wait;
    end process p_stimulus;

end architecture sim;

library ieee;
use ieee.std_logic_1164.all;

entity tb_lidar_gpx_cell_collector_dedicated_150 is
end entity tb_lidar_gpx_cell_collector_dedicated_150;

architecture sim of tb_lidar_gpx_cell_collector_dedicated_150 is
begin
    u_test : entity work.tb_lidar_gpx_cell_collector
        generic map (G_CLK_MHZ => 150, G_SCENARIO => 0);
end architecture sim;

library ieee;
use ieee.std_logic_1164.all;

entity tb_lidar_gpx_cell_collector_dedicated_200 is
end entity tb_lidar_gpx_cell_collector_dedicated_200;

architecture sim of tb_lidar_gpx_cell_collector_dedicated_200 is
begin
    u_test : entity work.tb_lidar_gpx_cell_collector
        generic map (G_CLK_MHZ => 200, G_SCENARIO => 0);
end architecture sim;

library ieee;
use ieee.std_logic_1164.all;

entity tb_lidar_gpx_cell_collector_dual_150 is
end entity tb_lidar_gpx_cell_collector_dual_150;

architecture sim of tb_lidar_gpx_cell_collector_dual_150 is
begin
    u_test : entity work.tb_lidar_gpx_cell_collector
        generic map (G_CLK_MHZ => 150, G_SCENARIO => 1);
end architecture sim;

library ieee;
use ieee.std_logic_1164.all;

entity tb_lidar_gpx_cell_collector_dual_200 is
end entity tb_lidar_gpx_cell_collector_dual_200;

architecture sim of tb_lidar_gpx_cell_collector_dual_200 is
begin
    u_test : entity work.tb_lidar_gpx_cell_collector
        generic map (G_CLK_MHZ => 200, G_SCENARIO => 1);
end architecture sim;

library ieee;
use ieee.std_logic_1164.all;

entity tb_lidar_gpx_cell_collector_faults_150 is
end entity tb_lidar_gpx_cell_collector_faults_150;

architecture sim of tb_lidar_gpx_cell_collector_faults_150 is
begin
    u_test : entity work.tb_lidar_gpx_cell_collector
        generic map (G_CLK_MHZ => 150, G_SCENARIO => 2);
end architecture sim;

library ieee;
use ieee.std_logic_1164.all;

entity tb_lidar_gpx_cell_collector_faults_200 is
end entity tb_lidar_gpx_cell_collector_faults_200;

architecture sim of tb_lidar_gpx_cell_collector_faults_200 is
begin
    u_test : entity work.tb_lidar_gpx_cell_collector
        generic map (G_CLK_MHZ => 200, G_SCENARIO => 2);
end architecture sim;
