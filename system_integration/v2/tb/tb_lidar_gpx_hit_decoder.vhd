library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.env.all;

use work.lidar_build_pkg.all;
use work.lidar_config_types_pkg.all;
use work.lidar_event_types_pkg.all;
use work.lidar_gpx_pkg.all;
use work.lidar_gpx_event_pkg.all;
use work.lidar_gpx_data_pkg.all;

entity tb_lidar_gpx_hit_decoder is
    generic (
        G_CLK_MHZ  : positive := 150;
        G_SCENARIO : natural range 0 to 2 := 0
    );
end entity tb_lidar_gpx_hit_decoder;

architecture sim of tb_lidar_gpx_hit_decoder is

    function fn_build_config return lidar_build_config_t is
        variable result : lidar_build_config_t := C_DEFAULT_BUILD_CONFIG;
    begin
        result.proc_clk_mhz := G_CLK_MHZ;
        result.tdc_clk_mhz := 200;
        result.stream_clock_mode := STREAM_CLOCK_ASYNC;
        result.output_width := 32;
        result.num_faces := 4;
        result.enable_echo_receiver := false;
        result.enable_echo_simulation := false;

        case G_SCENARIO is
            when 0 =>
                result.num_chips := 4;
                result.stops_per_chip := 8;
                result.max_returns_per_stop := 7;
                result.rise_capability_mask := "0011";
                result.fall_capability_mask := "1100";
            when 1 =>
                result.num_chips := 1;
                result.stops_per_chip := 8;
                result.max_returns_per_stop := 7;
                result.rise_capability_mask := "0001";
                result.fall_capability_mask := "0001";
            when others =>
                result.num_chips := 3;
                result.stops_per_chip := 6;
                result.max_returns_per_stop := 7;
                result.rise_capability_mask := "0011";
                result.fall_capability_mask := "0100";
        end case;
        return result;
    end function fn_build_config;

    constant C_BUILD_CONFIG : lidar_build_config_t := fn_build_config;
    constant C_CLK_PERIOD : time := 1 us / G_CLK_MHZ;

    signal clk             : std_logic := '0';
    signal rst_n           : std_logic := '0';
    signal abort_event     : std_logic := '0';
    signal clear_sticky    : std_logic := '0';
    signal raw_event       : gpx_raw_event_t := C_GPX_RAW_EVENT_IDLE;
    signal raw_ready       : std_logic;
    signal hit_event       : gpx_hit_event_t;
    signal hit_ready       : std_logic := '1';
    signal fault_pulse     : gpx_hit_decoder_faults_t;
    signal fault_sticky    : gpx_hit_decoder_faults_t;
    signal simulation_done : boolean := false;

    function fn_shot_context return shot_start_event_t is
        variable result : shot_start_event_t := C_SHOT_START_EVENT_IDLE;
    begin
        result.valid := '1';
        result.request.valid := '1';
        result.request.face_index := to_unsigned(2, 3);
        result.request.position := to_unsigned(1234, C_POSITION_WIDTH);
        result.request.direction := DIRECTION_CW;
        result.request.shot_index := to_unsigned(37, 16);
        result.request.last_in_face := '0';
        result.request.source_sim := '0';
        result.request.source_latency_clks := to_unsigned(
            8, result.request.source_latency_clks'length);
        result.request.source_latency_valid := '1';
        result.request.active_version := to_unsigned(21, 16);
        result.fire_to_t0_clks := to_unsigned(
            19, result.fire_to_t0_clks'length);
        return result;
    end function fn_shot_context;

    function fn_raw_data(
        chip_index   : natural;
        ififo_index  : natural;
        channel_code : natural;
        start_number : natural;
        slope        : gpx_slope_t;
        hit_value    : natural;
        chip_seq     : natural
    ) return gpx_raw_event_t is
        variable result : gpx_raw_event_t := C_GPX_RAW_EVENT_IDLE;
    begin
        result.valid := '1';
        result.kind := GPX_RAW_DATA;
        result.chip_index := to_unsigned(chip_index, result.chip_index'length);
        if ififo_index = 1 then
            result.ififo_id := '1';
        end if;
        result.raw_word(C_GPX_RAW_CHACODE_HI downto C_GPX_RAW_CHACODE_LO) :=
            std_logic_vector(to_unsigned(channel_code, 2));
        result.raw_word(C_GPX_RAW_START_HI downto C_GPX_RAW_START_LO) :=
            std_logic_vector(to_unsigned(start_number, 8));
        result.raw_word(C_GPX_RAW_SLOPE_BIT) := fn_gpx_slope_to_bit(slope);
        result.raw_word(C_GPX_RAW_HIT_HI downto C_GPX_RAW_HIT_LO) :=
            std_logic_vector(to_unsigned(hit_value, C_GPX_HIT_WIDTH));
        result.shot_context := fn_shot_context;
        result.chip_shot_seq := to_unsigned(chip_seq, 16);
        return result;
    end function fn_raw_data;

    function fn_raw_control(
        kind          : gpx_raw_event_kind_t;
        chip_index    : natural;
        ififo_index   : natural;
        chip_seq      : natural;
        faulted       : std_logic := '0';
        timeout_cause : std_logic_vector(2 downto 0) := "000"
    ) return gpx_raw_event_t is
        variable result : gpx_raw_event_t := C_GPX_RAW_EVENT_IDLE;
    begin
        result.valid := '1';
        result.kind := kind;
        result.chip_index := to_unsigned(chip_index, result.chip_index'length);
        if ififo_index = 1 then
            result.ififo_id := '1';
        end if;
        result.faulted := faulted;
        result.timeout_cause := timeout_cause;
        result.shot_context := fn_shot_context;
        result.chip_shot_seq := to_unsigned(chip_seq, 16);
        return result;
    end function fn_raw_control;

    function fn_expected_data(
        chip_index   : natural;
        ififo_index  : natural;
        channel_code : natural;
        start_number : natural;
        slope        : gpx_slope_t;
        hit_value    : natural;
        chip_seq     : natural
    ) return gpx_hit_event_t is
        variable result : gpx_hit_event_t := C_GPX_HIT_EVENT_IDLE;
    begin
        result.valid := '1';
        result.kind := GPX_HIT_DATA;
        result.chip_index := to_unsigned(chip_index, result.chip_index'length);
        if ififo_index = 1 then
            result.ififo_id := '1';
            result.stop_index := to_unsigned(channel_code + 4, 3);
        else
            result.stop_index := to_unsigned(channel_code, 3);
        end if;
        result.channel_code := to_unsigned(channel_code, 2);
        result.start_number := to_unsigned(start_number, 8);
        result.slope := slope;
        result.hit := to_unsigned(hit_value, C_GPX_HIT_WIDTH);
        result.shot_context := fn_shot_context;
        result.chip_shot_seq := to_unsigned(chip_seq, 16);
        return result;
    end function fn_expected_data;

    function fn_expected_control(
        kind          : gpx_hit_event_kind_t;
        chip_index    : natural;
        ififo_index   : natural;
        chip_seq      : natural;
        faulted       : std_logic := '0';
        timeout_cause : std_logic_vector(2 downto 0) := "000"
    ) return gpx_hit_event_t is
        variable result : gpx_hit_event_t := C_GPX_HIT_EVENT_IDLE;
    begin
        result.valid := '1';
        result.kind := kind;
        result.chip_index := to_unsigned(chip_index, result.chip_index'length);
        if ififo_index = 1 then
            result.ififo_id := '1';
        end if;
        result.faulted := faulted;
        result.timeout_cause := timeout_cause;
        result.shot_context := fn_shot_context;
        result.chip_shot_seq := to_unsigned(chip_seq, 16);
        return result;
    end function fn_expected_control;

begin

    clk <= not clk after C_CLK_PERIOD / 2 when not simulation_done;

    u_dut : entity work.lidar_gpx_hit_decoder
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
            o_fault_pulse  => fault_pulse,
            o_fault_sticky => fault_sticky
        );

    p_stimulus : process
        procedure wait_clocks(count : positive) is
        begin
            for index in 1 to count loop
                wait until rising_edge(clk);
            end loop;
            wait for 1 ps;
        end procedure wait_clocks;

        procedure offer_raw(value : gpx_raw_event_t) is
        begin
            raw_event <= value;
            loop
                wait until rising_edge(clk);
                exit when raw_ready = '1';
            end loop;
            wait for 1 ps;
            raw_event <= C_GPX_RAW_EVENT_IDLE;
        end procedure offer_raw;

        procedure send_and_expect(
            raw_value      : gpx_raw_event_t;
            expected_value : gpx_hit_event_t;
            message_text   : string
        ) is
        begin
            offer_raw(raw_value);
            loop
                wait until falling_edge(clk);
                exit when hit_event.valid = '1';
            end loop;
            assert hit_event = expected_value
                report message_text
                severity failure;
            assert fault_pulse = C_GPX_HIT_DECODER_FAULTS_CLEAR
                report message_text & " produced an unexpected fault"
                severity failure;
            wait_clocks(1);
            assert hit_event.valid = '0'
                report message_text & " did not retire"
                severity failure;
        end procedure send_and_expect;

        procedure send_and_expect_fault(
            raw_value      : gpx_raw_event_t;
            expected_pulse : gpx_hit_decoder_faults_t;
            message_text   : string
        ) is
        begin
            offer_raw(raw_value);
            wait until rising_edge(clk);
            wait for 1 ps;
            assert hit_event.valid = '0'
                report message_text & " unexpectedly emitted a Hit"
                severity failure;
            assert fault_pulse = expected_pulse
                report message_text & " fault pulse mismatch"
                severity failure;
            wait_clocks(1);
            assert fault_pulse = C_GPX_HIT_DECODER_FAULTS_CLEAR
                report message_text & " fault pulse was not one clock"
                severity failure;
        end procedure send_and_expect_fault;

        procedure check_backpressure(
            raw_value      : gpx_raw_event_t;
            expected_value : gpx_hit_event_t
        ) is
            variable held_value : gpx_hit_event_t;
        begin
            hit_ready <= '0';
            offer_raw(raw_value);
            loop
                wait until falling_edge(clk);
                exit when hit_event.valid = '1';
            end loop;
            assert hit_event = expected_value
                report "V2-B6-TB backpressure initial payload mismatch"
                severity failure;
            held_value := hit_event;
            for index in 1 to 5 loop
                wait until rising_edge(clk);
                wait for 1 ps;
                assert hit_event = held_value
                    report "V2-B6-TB payload changed under backpressure"
                    severity failure;
            end loop;
            hit_ready <= '1';
            wait_clocks(1);
            assert hit_event.valid = '0'
                report "V2-B6-TB held payload did not retire"
                severity failure;
        end procedure check_backpressure;

        variable raw_value      : gpx_raw_event_t;
        variable expected_value : gpx_hit_event_t;
        variable expected_fault : gpx_hit_decoder_faults_t;
        variable slope_value    : gpx_slope_t;
        variable hit_value      : natural;
        variable start_value    : natural;
    begin
        wait_clocks(5);
        rst_n <= '1';
        wait_clocks(3);

        assert fault_sticky = C_GPX_HIT_DECODER_FAULTS_CLEAR
            report "V2-B6-TB sticky fault was not clear after reset"
            severity failure;

        case G_SCENARIO is
            when 0 =>
                raw_value := fn_raw_data(
                    0, 0, 0, 16#40#, GPX_SLOPE_RISE, 16#10000#, 1);
                expected_value := fn_expected_data(
                    0, 0, 0, 16#40#, GPX_SLOPE_RISE, 16#10000#, 1);
                check_backpressure(raw_value, expected_value);

                for chip in 0 to 3 loop
                    if chip < 2 then
                        slope_value := GPX_SLOPE_RISE;
                    else
                        slope_value := GPX_SLOPE_FALL;
                    end if;

                    for ififo in 0 to 1 loop
                        for channel in 0 to 3 loop
                            for return_id in 0 to 6 loop
                                if chip = 0 and ififo = 0 and channel = 0 and
                                   return_id = 0 then
                                    next;
                                end if;
                                start_value := 16#40# + chip * 8 +
                                    ififo * 4 + channel;
                                hit_value := chip * 16#2000# +
                                    ififo * 16#0800# + channel * 16 +
                                    return_id;
                                if (return_id mod 2) = 0 then
                                    hit_value := hit_value + 16#10000#;
                                end if;
                                send_and_expect(
                                    fn_raw_data(chip, ififo, channel,
                                        start_value, slope_value, hit_value, 1),
                                    fn_expected_data(chip, ififo, channel,
                                        start_value, slope_value, hit_value, 1),
                                    "V2-B6-TB dedicated data mismatch");
                            end loop;
                        end loop;

                        if ififo = 0 then
                            send_and_expect(
                                fn_raw_control(GPX_RAW_IFIFO1_DONE,
                                    chip, 0, 1),
                                fn_expected_control(GPX_HIT_IFIFO1_DONE,
                                    chip, 0, 1),
                                "V2-B6-TB IFIFO1 control mismatch");
                        else
                            send_and_expect(
                                fn_raw_control(GPX_RAW_DRAIN_DONE,
                                    chip, 1, 1),
                                fn_expected_control(GPX_HIT_DRAIN_DONE,
                                    chip, 1, 1),
                                "V2-B6-TB terminal control mismatch");
                        end if;
                    end loop;
                end loop;

                send_and_expect(
                    fn_raw_data(0, 0, 0, 16#55#, GPX_SLOPE_RISE,
                        16#1ABCD#, 2),
                    fn_expected_data(0, 0, 0, 16#55#, GPX_SLOPE_RISE,
                        16#1ABCD#, 2),
                    "V2-B6-TB post-terminal decode mismatch");

            when 1 =>
                for return_id in 0 to 6 loop
                    send_and_expect(
                        fn_raw_data(0, 0, 2, 16#31#,
                            GPX_SLOPE_RISE, 16#10020# + return_id, 4),
                        fn_expected_data(0, 0, 2, 16#31#,
                            GPX_SLOPE_RISE, 16#10020# + return_id, 4),
                        "V2-B6-TB dual-edge Rise decode mismatch");
                    send_and_expect(
                        fn_raw_data(0, 0, 2, 16#32#,
                            GPX_SLOPE_FALL, 16#0020# + return_id, 4),
                        fn_expected_data(0, 0, 2, 16#32#,
                            GPX_SLOPE_FALL, 16#0020# + return_id, 4),
                        "V2-B6-TB dual-edge Fall decode mismatch");
                end loop;

                send_and_expect(
                    fn_raw_data(0, 0, 2, 0, GPX_SLOPE_RISE, 1, 4),
                    fn_expected_data(0, 0, 2, 0,
                        GPX_SLOPE_RISE, 1, 4),
                    "V2-B6-TB eighth Rise pass-through");
                send_and_expect(
                    fn_raw_data(0, 0, 2, 0, GPX_SLOPE_FALL, 2, 4),
                    fn_expected_data(0, 0, 2, 0,
                        GPX_SLOPE_FALL, 2, 4),
                    "V2-B6-TB eighth Fall pass-through");

                send_and_expect(
                    fn_raw_control(GPX_RAW_DRAIN_DONE, 0, 1, 4),
                    fn_expected_control(GPX_HIT_DRAIN_DONE, 0, 1, 4),
                    "V2-B6-TB dual-edge terminal mismatch");
                send_and_expect(
                    fn_raw_data(0, 0, 2, 7, GPX_SLOPE_RISE, 3, 5),
                    fn_expected_data(0, 0, 2, 7, GPX_SLOPE_RISE, 3, 5),
                    "V2-B6-TB dual-edge Rise post-terminal mismatch");
                send_and_expect(
                    fn_raw_data(0, 0, 2, 8, GPX_SLOPE_FALL, 4, 5),
                    fn_expected_data(0, 0, 2, 8, GPX_SLOPE_FALL, 4, 5),
                    "V2-B6-TB dual-edge Fall post-terminal mismatch");

                abort_event <= '1';
                wait_clocks(1);
                assert raw_ready = '0' and hit_event.valid = '0'
                    report "V2-B6-TB abort did not close the input boundary"
                    severity failure;
                abort_event <= '0';
                send_and_expect(
                    fn_raw_data(0, 0, 2, 9, GPX_SLOPE_RISE, 5, 6),
                    fn_expected_data(0, 0, 2, 9, GPX_SLOPE_RISE, 5, 6),
                    "V2-B6-TB abort recovery mismatch");

            when others =>
                expected_fault := C_GPX_HIT_DECODER_FAULTS_CLEAR;
                expected_fault.chip_index_error := '1';
                send_and_expect_fault(
                    fn_raw_data(3, 0, 0, 0, GPX_SLOPE_RISE, 1, 1),
                    expected_fault,
                    "V2-B6-TB absent Chip");

                expected_fault := C_GPX_HIT_DECODER_FAULTS_CLEAR;
                expected_fault.slope_role_error := '1';
                send_and_expect_fault(
                    fn_raw_data(0, 0, 0, 0, GPX_SLOPE_FALL, 2, 1),
                    expected_fault,
                    "V2-B6-TB unsupported Fall role");
                send_and_expect_fault(
                    fn_raw_data(2, 0, 0, 0, GPX_SLOPE_RISE, 3, 1),
                    expected_fault,
                    "V2-B6-TB unsupported Rise role");

                expected_fault := C_GPX_HIT_DECODER_FAULTS_CLEAR;
                expected_fault.stop_index_error := '1';
                send_and_expect_fault(
                    fn_raw_data(0, 1, 2, 0, GPX_SLOPE_RISE, 4, 1),
                    expected_fault,
                    "V2-B6-TB stop range");

                send_and_expect(
                    fn_raw_data(2, 1, 1, 16#7E#,
                        GPX_SLOPE_FALL, 16#1FFFF#, 3),
                    fn_expected_data(2, 1, 1, 16#7E#,
                        GPX_SLOPE_FALL, 16#1FFFF#, 3),
                    "V2-B6-TB valid reduced topology data");
                send_and_expect(
                    fn_raw_control(GPX_RAW_TIMEOUT, 2, 1, 3, '1', "101"),
                    fn_expected_control(GPX_HIT_TIMEOUT,
                        2, 1, 3, '1', "101"),
                    "V2-B6-TB timeout context mismatch");
                send_and_expect(
                    fn_raw_data(2, 1, 1, 16#7F#,
                        GPX_SLOPE_FALL, 16#10001#, 4),
                    fn_expected_data(2, 1, 1, 16#7F#,
                        GPX_SLOPE_FALL, 16#10001#, 4),
                    "V2-B6-TB post-timeout decode mismatch");

                assert fault_sticky.chip_index_error = '1' and
                       fault_sticky.stop_index_error = '1' and
                       fault_sticky.slope_role_error = '1'
                    report "V2-B6-TB identity sticky summary mismatch"
                    severity failure;
                clear_sticky <= '1';
                wait_clocks(1);
                clear_sticky <= '0';
                assert fault_sticky = C_GPX_HIT_DECODER_FAULTS_CLEAR
                    report "V2-B6-TB sticky clear failed"
                    severity failure;
        end case;

        report "LIDAR_V2_GPX_HIT_DECODER_PASS clk_mhz=" &
            integer'image(G_CLK_MHZ) & " scenario=" &
            integer'image(G_SCENARIO)
            severity note;
        simulation_done <= true;
        stop;
        wait;
    end process p_stimulus;

end architecture sim;

library ieee;
use ieee.std_logic_1164.all;

entity tb_lidar_gpx_hit_decoder_dedicated_150 is
end entity tb_lidar_gpx_hit_decoder_dedicated_150;

architecture sim of tb_lidar_gpx_hit_decoder_dedicated_150 is
begin
    u_test : entity work.tb_lidar_gpx_hit_decoder
        generic map (G_CLK_MHZ => 150, G_SCENARIO => 0);
end architecture sim;

library ieee;
use ieee.std_logic_1164.all;

entity tb_lidar_gpx_hit_decoder_dedicated_200 is
end entity tb_lidar_gpx_hit_decoder_dedicated_200;

architecture sim of tb_lidar_gpx_hit_decoder_dedicated_200 is
begin
    u_test : entity work.tb_lidar_gpx_hit_decoder
        generic map (G_CLK_MHZ => 200, G_SCENARIO => 0);
end architecture sim;

library ieee;
use ieee.std_logic_1164.all;

entity tb_lidar_gpx_hit_decoder_dual_150 is
end entity tb_lidar_gpx_hit_decoder_dual_150;

architecture sim of tb_lidar_gpx_hit_decoder_dual_150 is
begin
    u_test : entity work.tb_lidar_gpx_hit_decoder
        generic map (G_CLK_MHZ => 150, G_SCENARIO => 1);
end architecture sim;

library ieee;
use ieee.std_logic_1164.all;

entity tb_lidar_gpx_hit_decoder_dual_200 is
end entity tb_lidar_gpx_hit_decoder_dual_200;

architecture sim of tb_lidar_gpx_hit_decoder_dual_200 is
begin
    u_test : entity work.tb_lidar_gpx_hit_decoder
        generic map (G_CLK_MHZ => 200, G_SCENARIO => 1);
end architecture sim;

library ieee;
use ieee.std_logic_1164.all;

entity tb_lidar_gpx_hit_decoder_faults_150 is
end entity tb_lidar_gpx_hit_decoder_faults_150;

architecture sim of tb_lidar_gpx_hit_decoder_faults_150 is
begin
    u_test : entity work.tb_lidar_gpx_hit_decoder
        generic map (G_CLK_MHZ => 150, G_SCENARIO => 2);
end architecture sim;

library ieee;
use ieee.std_logic_1164.all;

entity tb_lidar_gpx_hit_decoder_faults_200 is
end entity tb_lidar_gpx_hit_decoder_faults_200;

architecture sim of tb_lidar_gpx_hit_decoder_faults_200 is
begin
    u_test : entity work.tb_lidar_gpx_hit_decoder
        generic map (G_CLK_MHZ => 200, G_SCENARIO => 2);
end architecture sim;
