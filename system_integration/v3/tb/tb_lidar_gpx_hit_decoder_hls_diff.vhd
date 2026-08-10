-- ============================================================================
-- Test purpose: prove that the V3 HLS Raw28-to-Hit17 decoder preserves the
-- V2 registered decoder contract at the record boundary.
--
-- 검증 목적:
--   V2 RTL을 Golden 기준으로 삼아 V3 HLS 디코더가 동일한 Raw28 입력을
--   동일한 Hit17 이벤트와 오류 상태로 변환하는지 Clock 단위로 비교한다.
--
-- Covered contracts:
--   * all I-Mode fields, Hit[16], Shot context and sequence are bit exact;
--   * dedicated, one-Chip dual-edge, reduced STOP and four-Chip dual-edge;
--   * AXI backpressure holds the complete event;
--   * invalid Chip/STOP/slope events are dropped with equal pulse/sticky bits;
--   * abort flushes both implementations and recovery accepts a new event.
-- ============================================================================
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

entity tb_lidar_gpx_hit_decoder_hls_diff is
    generic (
        G_CLK_MHZ  : positive := 200;
        G_SCENARIO : natural range 0 to 3 := 0
    );
end entity tb_lidar_gpx_hit_decoder_hls_diff;

architecture sim of tb_lidar_gpx_hit_decoder_hls_diff is

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
                result.rise_capability_mask := "0011";
                result.fall_capability_mask := "1100";
            when 1 =>
                result.num_chips := 1;
                result.stops_per_chip := 8;
                result.rise_capability_mask := "0001";
                result.fall_capability_mask := "0001";
            when 2 =>
                result.num_chips := 3;
                result.stops_per_chip := 6;
                result.rise_capability_mask := "0011";
                result.fall_capability_mask := "0100";
            when others =>
                result.num_chips := 4;
                result.stops_per_chip := 8;
                result.rise_capability_mask := "1111";
                result.fall_capability_mask := "1111";
        end case;
        return result;
    end function fn_build_config;

    function fn_shot_context(
        event_sequence : natural
    ) return shot_start_event_t is
        variable result : shot_start_event_t := C_SHOT_START_EVENT_IDLE;
    begin
        result.valid := '1';
        result.request.valid := '1';
        result.request.face_index := to_unsigned(event_sequence mod 4, 3);
        result.request.position := to_unsigned(
            16#1200# + event_sequence, C_POSITION_WIDTH);
        if (event_sequence mod 2) = 0 then
            result.request.direction := DIRECTION_CCW;
        else
            result.request.direction := DIRECTION_CW;
        end if;
        result.request.shot_index := to_unsigned(16#30# + event_sequence, 16);
        result.request.last_in_face := '0';
        result.request.source_sim := '0';
        result.request.source_latency_clks := to_unsigned(
            5 + event_sequence, result.request.source_latency_clks'length);
        result.request.source_latency_valid := '1';
        result.request.active_version := to_unsigned(16#40# + event_sequence, 16);
        result.fire_to_t0_clks := to_unsigned(
            11 + event_sequence, result.fire_to_t0_clks'length);
        result.t0_timestamp_ticks := to_unsigned(
            16#12340# + event_sequence, result.t0_timestamp_ticks'length);
        result.t0_timestamp_valid := '1';
        result.t0_time_sync_valid := '1';
        return result;
    end function fn_shot_context;

    function fn_raw_data(
        chip_index   : natural;
        ififo_index  : natural;
        channel_code : natural;
        start_number : natural;
        slope        : gpx_slope_t;
        hit_value    : natural;
        event_sequence : natural
    ) return gpx_raw_event_t is
        variable result : gpx_raw_event_t := C_GPX_RAW_EVENT_IDLE;
    begin
        result.valid := '1';
        result.kind := GPX_RAW_DATA;
        result.chip_index := to_unsigned(chip_index, 2);
        if ififo_index /= 0 then
            result.ififo_id := '1';
        end if;
        result.raw_word(C_GPX_RAW_CHACODE_HI downto C_GPX_RAW_CHACODE_LO) :=
            std_logic_vector(to_unsigned(channel_code, 2));
        result.raw_word(C_GPX_RAW_START_HI downto C_GPX_RAW_START_LO) :=
            std_logic_vector(to_unsigned(start_number, 8));
        result.raw_word(C_GPX_RAW_SLOPE_BIT) := fn_gpx_slope_to_bit(slope);
        result.raw_word(C_GPX_RAW_HIT_HI downto C_GPX_RAW_HIT_LO) :=
            std_logic_vector(to_unsigned(hit_value, C_GPX_HIT_WIDTH));
        result.faulted := '0';
        result.timeout_cause := std_logic_vector(to_unsigned(event_sequence mod 8, 3));
        result.shot_context := fn_shot_context(event_sequence);
        result.chip_shot_seq := to_unsigned(event_sequence, 16);
        return result;
    end function fn_raw_data;

    function fn_raw_control(
        kind          : gpx_raw_event_kind_t;
        chip_index    : natural;
        ififo_index   : natural;
        event_sequence : natural;
        faulted       : std_logic := '0';
        timeout_cause : std_logic_vector(2 downto 0) := "000"
    ) return gpx_raw_event_t is
        variable result : gpx_raw_event_t := C_GPX_RAW_EVENT_IDLE;
    begin
        result.valid := '1';
        result.kind := kind;
        result.chip_index := to_unsigned(chip_index, 2);
        if ififo_index /= 0 then
            result.ififo_id := '1';
        end if;
        result.faulted := faulted;
        result.timeout_cause := timeout_cause;
        result.shot_context := fn_shot_context(event_sequence);
        result.chip_shot_seq := to_unsigned(event_sequence, 16);
        return result;
    end function fn_raw_control;

    function fn_faults(
        chip_error  : std_logic;
        stop_error  : std_logic;
        slope_error : std_logic
    ) return gpx_hit_decoder_faults_t is
        variable result : gpx_hit_decoder_faults_t :=
            C_GPX_HIT_DECODER_FAULTS_CLEAR;
    begin
        result.chip_index_error := chip_error;
        result.stop_index_error := stop_error;
        result.slope_role_error := slope_error;
        return result;
    end function fn_faults;

    constant C_BUILD_CONFIG : lidar_build_config_t := fn_build_config;
    constant C_CLK_PERIOD : time := 1 us / G_CLK_MHZ;

    signal clk          : std_logic := '0';
    signal rst_n        : std_logic := '0';
    signal abort_event  : std_logic := '0';
    signal clear_sticky : std_logic := '0';
    signal raw_event    : gpx_raw_event_t := C_GPX_RAW_EVENT_IDLE;
    signal hit_ready    : std_logic := '0';

    signal ref_raw_ready    : std_logic;
    signal ref_hit_event    : gpx_hit_event_t;
    signal ref_fault_pulse  : gpx_hit_decoder_faults_t;
    signal ref_fault_sticky : gpx_hit_decoder_faults_t;

    signal hls_raw_ready    : std_logic;
    signal hls_hit_event    : gpx_hit_event_t;
    signal hls_fault_pulse  : gpx_hit_decoder_faults_t;
    signal hls_fault_sticky : gpx_hit_decoder_faults_t;

    signal simulation_done : boolean := false;

begin

    clk <= not clk after C_CLK_PERIOD / 2 when not simulation_done;

    u_v2_reference : entity work.lidar_gpx_hit_decoder
        generic map (
            G_BUILD_CONFIG => C_BUILD_CONFIG
        )
        port map (
            i_clk              => clk,
            i_rst_n            => rst_n,
            i_abort            => abort_event,
            i_clear_sticky     => clear_sticky,
            i_active_rise_mask => C_BUILD_CONFIG.rise_capability_mask,
            i_active_fall_mask => C_BUILD_CONFIG.fall_capability_mask,
            i_raw_event        => raw_event,
            o_raw_ready        => ref_raw_ready,
            o_hit_event        => ref_hit_event,
            i_hit_ready        => hit_ready,
            o_fault_pulse      => ref_fault_pulse,
            o_fault_sticky     => ref_fault_sticky
        );

    u_v3_hls : entity work.lidar_gpx_hit_decoder_hls_adapter
        generic map (
            G_BUILD_CONFIG => C_BUILD_CONFIG
        )
        port map (
            i_clk              => clk,
            i_rst_n            => rst_n,
            i_abort            => abort_event,
            i_clear_sticky     => clear_sticky,
            i_active_rise_mask => C_BUILD_CONFIG.rise_capability_mask,
            i_active_fall_mask => C_BUILD_CONFIG.fall_capability_mask,
            i_raw_event        => raw_event,
            o_raw_ready        => hls_raw_ready,
            o_hit_event        => hls_hit_event,
            i_hit_ready        => hit_ready,
            o_fault_pulse      => hls_fault_pulse,
            o_fault_sticky     => hls_fault_sticky
        );

    p_stimulus : process
        procedure wait_clocks(count : positive) is
        begin
            for index in 1 to count loop
                wait until rising_edge(clk);
            end loop;
            wait for 1 ps;
        end procedure wait_clocks;

        procedure offer_both(value : gpx_raw_event_t) is
            variable ready_seen : boolean := false;
        begin
            -- AXI-style source rule: hold the complete record stable until
            -- both implementations accept it on the same rising edge.  A
            -- TREADY value sampled while TVALID is low is not an acceptance.
            raw_event <= value;
            for timeout in 0 to 40 loop
                wait until rising_edge(clk);
                if ref_raw_ready = '1' and hls_raw_ready = '1' then
                    ready_seen := true;
                    exit;
                end if;
            end loop;
            assert ready_seen
                report "V3-HLS-DIFF input ready timeout"
                severity failure;

            wait for 1 ps;
            raw_event <= C_GPX_RAW_EVENT_IDLE;
        end procedure offer_both;

        procedure send_and_compare(
            value        : gpx_raw_event_t;
            message_text : string;
            hold_clocks  : natural := 0
        ) is
            variable both_valid : boolean := false;
            variable held_ref   : gpx_hit_event_t;
            variable held_hls   : gpx_hit_event_t;
        begin
            hit_ready <= '0';
            offer_both(value);
            for timeout in 0 to 40 loop
                wait until falling_edge(clk);
                if ref_hit_event.valid = '1' and
                   hls_hit_event.valid = '1' then
                    both_valid := true;
                    exit;
                end if;
            end loop;
            assert both_valid
                report message_text & " output timeout"
                severity failure;
            assert ref_hit_event = hls_hit_event
                report message_text & " V2/HLS payload mismatch"
                severity failure;
            assert ref_fault_pulse = C_GPX_HIT_DECODER_FAULTS_CLEAR and
                   hls_fault_pulse = C_GPX_HIT_DECODER_FAULTS_CLEAR
                report message_text & " unexpected fault"
                severity failure;

            held_ref := ref_hit_event;
            held_hls := hls_hit_event;
            for index in 1 to hold_clocks loop
                wait_clocks(1);
                assert ref_hit_event = held_ref and
                       hls_hit_event = held_hls
                    report message_text & " changed under backpressure"
                    severity failure;
            end loop;

            hit_ready <= '1';
            wait_clocks(1);
            hit_ready <= '0';
            assert ref_hit_event.valid = '0' and hls_hit_event.valid = '0'
                report message_text & " did not retire"
                severity failure;
        end procedure send_and_compare;

        procedure send_and_compare_fault(
            value          : gpx_raw_event_t;
            expected_fault : gpx_hit_decoder_faults_t;
            message_text   : string
        ) is
            variable ref_seen : boolean := false;
            variable hls_seen : boolean := false;
            variable ref_value : gpx_hit_decoder_faults_t :=
                C_GPX_HIT_DECODER_FAULTS_CLEAR;
            variable hls_value : gpx_hit_decoder_faults_t :=
                C_GPX_HIT_DECODER_FAULTS_CLEAR;
        begin
            hit_ready <= '0';
            offer_both(value);
            for timeout in 0 to 40 loop
                wait until falling_edge(clk);
                if ref_fault_pulse /= C_GPX_HIT_DECODER_FAULTS_CLEAR then
                    ref_seen := true;
                    ref_value := ref_fault_pulse;
                end if;
                if hls_fault_pulse /= C_GPX_HIT_DECODER_FAULTS_CLEAR then
                    hls_seen := true;
                    hls_value := hls_fault_pulse;
                end if;
                assert ref_hit_event.valid = '0' and
                       hls_hit_event.valid = '0'
                    report message_text & " unexpectedly emitted a Hit"
                    severity failure;
                exit when ref_seen and hls_seen;
            end loop;
            assert ref_seen and hls_seen
                report message_text & " fault pulse timeout"
                severity failure;
            assert ref_value = expected_fault and hls_value = expected_fault
                report message_text & " fault pulse mismatch"
                severity failure;
            wait_clocks(1);
            assert ref_fault_pulse = C_GPX_HIT_DECODER_FAULTS_CLEAR and
                   hls_fault_pulse = C_GPX_HIT_DECODER_FAULTS_CLEAR
                report message_text & " pulse longer than one clock"
                severity failure;
            assert ref_fault_sticky = hls_fault_sticky
                report message_text & " sticky mismatch"
                severity failure;
        end procedure send_and_compare_fault;

        procedure clear_and_compare_sticky is
        begin
            clear_sticky <= '1';
            wait_clocks(1);
            clear_sticky <= '0';
            assert ref_fault_sticky = C_GPX_HIT_DECODER_FAULTS_CLEAR and
                   hls_fault_sticky = C_GPX_HIT_DECODER_FAULTS_CLEAR
                report "V3-HLS-DIFF sticky clear mismatch"
                severity failure;
        end procedure clear_and_compare_sticky;

        variable expected_fault : gpx_hit_decoder_faults_t;
    begin
        wait_clocks(6);
        rst_n <= '1';
        wait_clocks(4);

        assert ref_fault_sticky = C_GPX_HIT_DECODER_FAULTS_CLEAR and
               hls_fault_sticky = C_GPX_HIT_DECODER_FAULTS_CLEAR
            report "V3-HLS-DIFF reset sticky mismatch"
            severity failure;

        case G_SCENARIO is
            when 0 =>
                send_and_compare(
                    fn_raw_data(0, 1, 3, 16#A5#, GPX_SLOPE_RISE,
                        16#1FFFF#, 1),
                    "dedicated Rise max-Hit", 5);
                send_and_compare(
                    fn_raw_data(2, 0, 1, 16#35#, GPX_SLOPE_FALL,
                        16#10001#, 2),
                    "dedicated Fall");
                send_and_compare(
                    fn_raw_control(GPX_RAW_IFIFO1_DONE, 0, 0, 3),
                    "dedicated IFIFO1 control");
                send_and_compare(
                    fn_raw_control(GPX_RAW_TIMEOUT, 3, 1, 4, '1', "101"),
                    "dedicated timeout control");
                expected_fault := fn_faults('0', '0', '1');
                send_and_compare_fault(
                    fn_raw_data(0, 0, 0, 0, GPX_SLOPE_FALL, 1, 5),
                    expected_fault, "dedicated unsupported slope");

            when 1 =>
                send_and_compare(
                    fn_raw_data(0, 0, 2, 16#21#, GPX_SLOPE_RISE,
                        16#10020#, 6),
                    "one-Chip dual-edge Rise", 3);
                send_and_compare(
                    fn_raw_data(0, 1, 2, 16#22#, GPX_SLOPE_FALL,
                        16#00020#, 7),
                    "one-Chip dual-edge Fall");
                send_and_compare(
                    fn_raw_control(GPX_RAW_DRAIN_DONE, 0, 1, 8),
                    "one-Chip terminal control");
                expected_fault := fn_faults('1', '0', '0');
                send_and_compare_fault(
                    fn_raw_data(1, 0, 0, 0, GPX_SLOPE_RISE, 1, 9),
                    expected_fault, "one-Chip absent Chip");

            when 2 =>
                send_and_compare(
                    fn_raw_data(2, 1, 1, 16#7E#, GPX_SLOPE_FALL,
                        16#1FFFF#, 10),
                    "reduced STOP valid data", 4);
                expected_fault := fn_faults('1', '0', '0');
                send_and_compare_fault(
                    fn_raw_data(3, 0, 0, 0, GPX_SLOPE_RISE, 1, 11),
                    expected_fault, "reduced absent Chip");
                expected_fault := fn_faults('0', '1', '0');
                send_and_compare_fault(
                    fn_raw_data(0, 1, 2, 0, GPX_SLOPE_RISE, 2, 12),
                    expected_fault, "reduced STOP range");
                expected_fault := fn_faults('0', '0', '1');
                send_and_compare_fault(
                    fn_raw_data(0, 0, 0, 0, GPX_SLOPE_FALL, 3, 13),
                    expected_fault, "reduced unsupported slope");
                send_and_compare(
                    fn_raw_control(GPX_RAW_TIMEOUT, 2, 1, 14, '1', "110"),
                    "reduced timeout control");

            when others =>
                send_and_compare(
                    fn_raw_data(3, 0, 3, 16#91#, GPX_SLOPE_RISE,
                        16#15555#, 15),
                    "four-Chip all-dual Rise", 3);
                send_and_compare(
                    fn_raw_data(3, 1, 3, 16#92#, GPX_SLOPE_FALL,
                        16#0AAAA#, 16),
                    "four-Chip all-dual Fall");
                send_and_compare(
                    fn_raw_data(0, 1, 0, 16#93#, GPX_SLOPE_FALL,
                        16#10000#, 17),
                    "four-Chip all-dual Chip0 Fall");
                send_and_compare(
                    fn_raw_control(GPX_RAW_DRAIN_DONE, 3, 1, 18),
                    "four-Chip all-dual terminal");
        end case;

        if ref_fault_sticky /= C_GPX_HIT_DECODER_FAULTS_CLEAR or
           hls_fault_sticky /= C_GPX_HIT_DECODER_FAULTS_CLEAR then
            clear_and_compare_sticky;
        end if;

        -- Flush an accepted event before either implementation may retire it.
        hit_ready <= '0';
        offer_both(fn_raw_data(0, 0, 0, 16#5A#, GPX_SLOPE_RISE,
            16#12345#, 20));
        abort_event <= '1';
        wait_clocks(1);
        assert ref_hit_event.valid = '0' and hls_hit_event.valid = '0' and
               ref_raw_ready = '0' and hls_raw_ready = '0'
            report "V3-HLS-DIFF abort flush mismatch"
            severity failure;
        abort_event <= '0';
        wait_clocks(3);

        send_and_compare(
            fn_raw_control(GPX_RAW_DRAIN_DONE, 0, 1, 21),
            "post-abort recovery");

        report "LIDAR_V3_GPX_HIT_DECODER_DIFF_PASS clk_mhz=" &
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

entity tb_lidar_gpx_hit_decoder_hls_diff_dedicated_150 is end entity;
architecture sim of tb_lidar_gpx_hit_decoder_hls_diff_dedicated_150 is
begin
    u_test : entity work.tb_lidar_gpx_hit_decoder_hls_diff
        generic map (G_CLK_MHZ => 150, G_SCENARIO => 0);
end architecture;

library ieee;
use ieee.std_logic_1164.all;

entity tb_lidar_gpx_hit_decoder_hls_diff_dedicated_200 is end entity;
architecture sim of tb_lidar_gpx_hit_decoder_hls_diff_dedicated_200 is
begin
    u_test : entity work.tb_lidar_gpx_hit_decoder_hls_diff
        generic map (G_CLK_MHZ => 200, G_SCENARIO => 0);
end architecture;

library ieee;
use ieee.std_logic_1164.all;

entity tb_lidar_gpx_hit_decoder_hls_diff_one_dual_150 is end entity;
architecture sim of tb_lidar_gpx_hit_decoder_hls_diff_one_dual_150 is
begin
    u_test : entity work.tb_lidar_gpx_hit_decoder_hls_diff
        generic map (G_CLK_MHZ => 150, G_SCENARIO => 1);
end architecture;

library ieee;
use ieee.std_logic_1164.all;

entity tb_lidar_gpx_hit_decoder_hls_diff_one_dual_200 is end entity;
architecture sim of tb_lidar_gpx_hit_decoder_hls_diff_one_dual_200 is
begin
    u_test : entity work.tb_lidar_gpx_hit_decoder_hls_diff
        generic map (G_CLK_MHZ => 200, G_SCENARIO => 1);
end architecture;

library ieee;
use ieee.std_logic_1164.all;

entity tb_lidar_gpx_hit_decoder_hls_diff_reduced_150 is end entity;
architecture sim of tb_lidar_gpx_hit_decoder_hls_diff_reduced_150 is
begin
    u_test : entity work.tb_lidar_gpx_hit_decoder_hls_diff
        generic map (G_CLK_MHZ => 150, G_SCENARIO => 2);
end architecture;

library ieee;
use ieee.std_logic_1164.all;

entity tb_lidar_gpx_hit_decoder_hls_diff_reduced_200 is end entity;
architecture sim of tb_lidar_gpx_hit_decoder_hls_diff_reduced_200 is
begin
    u_test : entity work.tb_lidar_gpx_hit_decoder_hls_diff
        generic map (G_CLK_MHZ => 200, G_SCENARIO => 2);
end architecture;

library ieee;
use ieee.std_logic_1164.all;

entity tb_lidar_gpx_hit_decoder_hls_diff_all_dual_150 is end entity;
architecture sim of tb_lidar_gpx_hit_decoder_hls_diff_all_dual_150 is
begin
    u_test : entity work.tb_lidar_gpx_hit_decoder_hls_diff
        generic map (G_CLK_MHZ => 150, G_SCENARIO => 3);
end architecture;

library ieee;
use ieee.std_logic_1164.all;

entity tb_lidar_gpx_hit_decoder_hls_diff_all_dual_200 is end entity;
architecture sim of tb_lidar_gpx_hit_decoder_hls_diff_all_dual_200 is
begin
    u_test : entity work.tb_lidar_gpx_hit_decoder_hls_diff
        generic map (G_CLK_MHZ => 200, G_SCENARIO => 3);
end architecture;
