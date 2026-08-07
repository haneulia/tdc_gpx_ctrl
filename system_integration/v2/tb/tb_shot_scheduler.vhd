library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library std;
use std.env.all;

use work.lidar_build_pkg.all;
use work.lidar_config_types_pkg.all;
use work.lidar_config_reference_pkg.all;
use work.lidar_event_types_pkg.all;

entity tb_shot_scheduler is
    generic (
        G_PROC_CLK_MHZ        : positive := 150;
        G_PROC_HALF_PERIOD_PS : positive := 3333
    );
end entity tb_shot_scheduler;

architecture sim of tb_shot_scheduler is

    function fn_test_build return lidar_build_config_t is
        variable result : lidar_build_config_t := C_DEFAULT_BUILD_CONFIG;
    begin
        result.proc_clk_mhz := G_PROC_CLK_MHZ;
        return result;
    end function fn_test_build;

    constant C_TEST_BUILD : lidar_build_config_t := fn_test_build;

    function fn_bool_to_sl(value : boolean) return std_logic is
    begin
        if value then
            return '1';
        end if;
        return '0';
    end function fn_bool_to_sl;

    function fn_test_config(
        version_value  : positive;
        interval_value : positive;
        columns_value  : positive;
        face_mask_value: face_mask_t := (others => '1');
        simulation_mode: std_logic := '0'
    ) return lidar_active_config_t is
        variable result   : lidar_active_config_t;
        variable source_v : lidar_runtime_config_t;
    begin
        source_v := fn_default_runtime_config(C_TEST_BUILD);
        source_v.laser.face_enable_mask := face_mask_value;
        source_v.motor.simulation_mode := simulation_mode;
        result.version := to_unsigned(version_value, result.version'length);
        result.source  := source_v;
        result.derived := fn_derive_runtime_config(C_TEST_BUILD, source_v);
        result.derived.shot_interval_states := to_unsigned(
            interval_value, result.derived.shot_interval_states'length);
        result.derived.columns_per_face := to_unsigned(
            columns_value, result.derived.columns_per_face'length);
        return result;
    end function fn_test_config;

    constant C_BASE_CONFIG : lidar_active_config_t := fn_test_config(
        201, 1, 4);

    signal clk           : std_logic := '0';
    signal rst_n         : std_logic := '0';
    signal stop_clock    : boolean := false;
    signal enable        : std_logic := '0';
    signal boundary_block: std_logic := '0';
    signal active_valid  : std_logic := '0';
    signal active_config : lidar_active_config_t := C_BASE_CONFIG;
    signal face_event    : face_event_t := C_FACE_EVENT_IDLE;
    signal executor_ready: std_logic := '1';
    signal request_accept: std_logic := '0';
    signal request_drop  : std_logic := '0';
    signal clear_diag    : std_logic := '0';

    signal shot_request   : shot_request_t;
    signal overrun_pulse  : std_logic;
    signal overrun_sticky : std_logic;
    signal overrun_count  : u32_t;
    signal scheduler_idle : std_logic;

begin

    u_dut : entity work.shot_scheduler
        generic map (
            G_BUILD_CONFIG => C_TEST_BUILD
        )
        port map (
            i_clk                     => clk,
            i_rst_n                   => rst_n,
            i_enable                  => enable,
            i_boundary_block          => boundary_block,
            i_active_valid            => active_valid,
            i_active_config           => active_config,
            i_face_event              => face_event,
            i_executor_ready          => executor_ready,
            i_request_accept          => request_accept,
            i_request_drop            => request_drop,
            i_clear_diagnostics       => clear_diag,
            o_shot_request            => shot_request,
            o_schedule_overrun_pulse  => overrun_pulse,
            o_schedule_overrun_sticky => overrun_sticky,
            o_schedule_overrun_count  => overrun_count,
            o_idle                    => scheduler_idle
        );

    p_clock : process
    begin
        while not stop_clock loop
            clk <= '0';
            wait for G_PROC_HALF_PERIOD_PS * 1 ps;
            clk <= '1';
            wait for G_PROC_HALF_PERIOD_PS * 1 ps;
        end loop;
        wait;
    end process p_clock;

    p_test : process
        procedure check(
            condition    : boolean;
            message_text : string
        ) is
        begin
            assert condition report message_text severity failure;
        end procedure check;

        procedure wait_clocks(count_value : positive) is
        begin
            for index in 1 to count_value loop
                wait until rising_edge(clk);
                wait for 1 ps;
            end loop;
        end procedure wait_clocks;

        procedure reset_dut is
        begin
            enable         <= '0';
            boundary_block <= '0';
            active_valid   <= '0';
            face_event     <= C_FACE_EVENT_IDLE;
            executor_ready <= '1';
            request_accept <= '0';
            request_drop   <= '0';
            clear_diag     <= '0';
            rst_n          <= '0';
            wait_clocks(3);
            rst_n <= '1';
            wait_clocks(2);
        end procedure reset_dut;

        procedure load_config(
            constant config_value : in lidar_active_config_t
        ) is
        begin
            enable         <= '0';
            active_config  <= config_value;
            active_valid   <= '1';
            face_event     <= C_FACE_EVENT_IDLE;
            executor_ready <= '1';
            wait_clocks(3);
            enable <= '1';
            wait_clocks(2);
        end procedure load_config;

        procedure apply_face(
            constant position_value : in natural;
            constant face_value     : in natural;
            constant direction_value: in direction_t;
            constant inside_value   : in std_logic;
            constant enter_value    : in std_logic;
            constant exit_value     : in std_logic;
            constant overlap_value  : in std_logic;
            constant ready_value    : in std_logic;
            constant expect_request : in boolean;
            constant expected_index : in natural;
            constant expected_last  : in std_logic;
            constant expect_overrun : in boolean;
            constant case_name      : in string
        ) is
            variable event_v : face_event_t;
        begin
            event_v := C_FACE_EVENT_IDLE;
            event_v.valid                := '1';
            event_v.inside               := inside_value;
            event_v.enter_event          := enter_value;
            event_v.exit_event           := exit_value;
            event_v.overlap              := overlap_value;
            event_v.face_index           := to_unsigned(
                face_value, event_v.face_index'length);
            event_v.position             := to_unsigned(
                position_value, event_v.position'length);
            event_v.direction            := direction_value;
            event_v.source_sim           :=
                active_config.source.motor.simulation_mode;
            event_v.source_latency_clks  :=
                C_POSITION_PHYSICAL_LATENCY_CLKS;
            event_v.source_latency_valid := '1';
            event_v.active_version       := active_config.version;

            wait until falling_edge(clk);
            face_event     <= event_v;
            executor_ready <= ready_value;
            wait until rising_edge(clk);
            wait for 1 ps;

            if expect_request then
                check(shot_request.valid = '1',
                    case_name & " missing request");
                check(to_integer(shot_request.face_index) = face_value and
                      to_integer(shot_request.position) = position_value,
                    case_name & " Face/position mismatch");
                check(shot_request.direction = direction_value,
                    case_name & " direction mismatch");
                check(to_integer(shot_request.shot_index) = expected_index,
                    case_name & " geometric column mismatch");
                check(shot_request.last_in_face = expected_last,
                    case_name & " last-in-Face mismatch");
                check(shot_request.source_sim =
                      active_config.source.motor.simulation_mode and
                      shot_request.source_latency_valid = '1' and
                      shot_request.source_latency_clks =
                        C_POSITION_PHYSICAL_LATENCY_CLKS and
                      shot_request.active_version = active_config.version,
                    case_name & " request context mismatch");
                check(scheduler_idle = '0',
                    case_name & " request did not own in-flight state");
            else
                check(shot_request.valid = '0',
                    case_name & " unexpected request");
            end if;

            check((overrun_pulse = '1') = expect_overrun,
                case_name & " overrun pulse mismatch");
            face_event <= C_FACE_EVENT_IDLE;
        end procedure apply_face;

        procedure resolve_request(
            constant accept_value : in boolean;
            constant case_name    : in string
        ) is
        begin
            wait until falling_edge(clk);
            face_event <= C_FACE_EVENT_IDLE;
            if accept_value then
                request_accept <= '1';
                request_drop   <= '0';
            else
                request_accept <= '0';
                request_drop   <= '1';
            end if;
            wait until rising_edge(clk);
            wait for 1 ps;
            check(shot_request.valid = '0',
                case_name & " request pulse exceeded one clock");
            check(scheduler_idle = '1',
                case_name & " request did not resolve");
            request_accept <= '0';
            request_drop   <= '0';
        end procedure resolve_request;

        procedure clear_diagnostics is
        begin
            wait until falling_edge(clk);
            clear_diag <= '1';
            wait until rising_edge(clk);
            wait for 1 ps;
            clear_diag <= '0';
            check(overrun_sticky = '0' and overrun_count = 0,
                "diagnostics did not clear");
        end procedure clear_diagnostics;

        variable config_v : lidar_active_config_t;
        variable mask_v   : face_mask_t;
    begin
        reset_dut;

        -- P20: interval one produces every geometric column in time order.
        config_v := fn_test_config(201, 1, 4);
        load_config(config_v);
        for column in 0 to 3 loop
            apply_face(10 + column, 0, DIRECTION_CW, '1',
                fn_bool_to_sl(column = 0), '0', '0', '1',
                true, column,
                fn_bool_to_sl(column = 3), false,
                "V2-SCHED-P20 interval-one");
            resolve_request(true, "V2-SCHED-P20 interval-one");
        end loop;
        apply_face(14, 0, DIRECTION_CW, '1', '0', '0', '0', '1',
            false, 0, '0', false, "V2-SCHED-P20 post-lattice");

        reset_dut;
        config_v := fn_test_config(202, 3, 3);
        load_config(config_v);
        apply_face(20, 0, DIRECTION_CW, '1', '1', '0', '0', '1',
            true, 0, '0', false, "V2-SCHED-P20 interval-N col0");
        resolve_request(true, "V2-SCHED-P20 interval-N col0");
        apply_face(21, 0, DIRECTION_CW, '1', '0', '0', '0', '1',
            false, 0, '0', false, "V2-SCHED-P20 interval-N gap1");
        apply_face(22, 0, DIRECTION_CW, '1', '0', '0', '0', '1',
            false, 0, '0', false, "V2-SCHED-P20 interval-N gap2");
        apply_face(23, 0, DIRECTION_CW, '1', '0', '0', '0', '1',
            true, 1, '0', false, "V2-SCHED-P20 interval-N col1");
        resolve_request(true, "V2-SCHED-P20 interval-N col1");
        apply_face(24, 0, DIRECTION_CW, '1', '0', '0', '0', '1',
            false, 0, '0', false, "V2-SCHED-P20 interval-N gap3");
        apply_face(25, 0, DIRECTION_CW, '1', '0', '0', '0', '1',
            false, 0, '0', false, "V2-SCHED-P20 interval-N gap4");
        apply_face(26, 0, DIRECTION_CW, '1', '0', '0', '0', '1',
            true, 2, '1', false, "V2-SCHED-P20 interval-N col2");
        resolve_request(true, "V2-SCHED-P20 interval-N col2");
        report "V2-SCHED-P20 exact interval/index sequence PASS"
            severity note;

        -- P21: inclusive Face upper bound is not a duplicate shot point.
        reset_dut;
        config_v := fn_test_config(203, 2, 3);
        load_config(config_v);
        for offset in 0 to 5 loop
            apply_face(30 + offset, 0, DIRECTION_CW, '1',
                fn_bool_to_sl(offset = 0), '0', '0', '1',
                (offset = 0 or offset = 2 or offset = 4),
                offset / 2,
                fn_bool_to_sl(offset = 4), false,
                "V2-SCHED-P21 face interior");
            if offset = 0 or offset = 2 or offset = 4 then
                resolve_request(true, "V2-SCHED-P21 face interior");
            end if;
        end loop;
        apply_face(36, 0, DIRECTION_CW, '1', '0', '0', '0', '1',
            false, 0, '0', false, "V2-SCHED-P21 inclusive endpoint");
        report "V2-SCHED-P21 inclusive endpoint exclusion PASS"
            severity note;

        -- P22: a busy due point becomes a geometric hole, never a late shot.
        reset_dut;
        config_v := fn_test_config(204, 2, 3);
        load_config(config_v);
        apply_face(40, 0, DIRECTION_CW, '1', '1', '0', '0', '0',
            false, 0, '0', true, "V2-SCHED-P22 blocked col0");
        apply_face(41, 0, DIRECTION_CW, '1', '0', '0', '0', '1',
            false, 0, '0', false, "V2-SCHED-P22 no late retry");
        check(overrun_sticky = '1' and overrun_count = 1,
            "V2-SCHED-P22 overrun diagnostics mismatch");
        apply_face(42, 0, DIRECTION_CW, '1', '0', '0', '0', '1',
            true, 1, '0', false, "V2-SCHED-P22 geometric col1");
        resolve_request(true, "V2-SCHED-P22 geometric col1");
        apply_face(43, 0, DIRECTION_CW, '1', '0', '0', '0', '1',
            false, 0, '0', false, "V2-SCHED-P22 gap");
        apply_face(44, 0, DIRECTION_CW, '1', '0', '0', '0', '1',
            true, 2, '1', false, "V2-SCHED-P22 geometric col2");
        resolve_request(true, "V2-SCHED-P22 geometric col2");
        clear_diagnostics;
        report "V2-SCHED-P22 busy skip/no-late-fire PASS"
            severity note;

        -- P23: unresolved old Face ownership cannot compact the new Face.
        reset_dut;
        config_v := fn_test_config(205, 1, 3);
        load_config(config_v);
        apply_face(50, 0, DIRECTION_CW, '1', '1', '0', '0', '1',
            true, 0, '0', false, "V2-SCHED-P23 old Face request");
        apply_face(60, 1, DIRECTION_CW, '1', '1', '1', '0', '1',
            false, 0, '0', true, "V2-SCHED-P23 zero-gap col0");
        resolve_request(true, "V2-SCHED-P23 old Face resolve");
        apply_face(61, 1, DIRECTION_CW, '1', '0', '0', '0', '1',
            true, 1, '0', false, "V2-SCHED-P23 new Face col1");
        resolve_request(true, "V2-SCHED-P23 new Face col1");
        apply_face(61, 1, DIRECTION_CCW, '1', '1', '1', '0', '1',
            true, 0, '0', false, "V2-SCHED-P23 reversal col0");
        resolve_request(false, "V2-SCHED-P23 reversal col0");
        report "V2-SCHED-P23 Face/reversal isolation PASS"
            severity note;

        -- P24: enabling in the middle of a Face waits for a new entry. Masked
        -- or overlapping defensive contexts do not become throughput overruns.
        reset_dut;
        config_v := fn_test_config(206, 1, 2);
        enable        <= '0';
        active_config <= config_v;
        active_valid  <= '1';
        wait_clocks(3);
        apply_face(70, 0, DIRECTION_CW, '1', '1', '0', '0', '1',
            false, 0, '0', false, "V2-SCHED-P24 disabled entry");
        enable <= '1';
        apply_face(71, 0, DIRECTION_CW, '1', '1', '0', '0', '1',
            false, 0, '0', false, "V2-SCHED-P24 stale entry quarantine");
        apply_face(72, 0, DIRECTION_CW, '0', '0', '1', '0', '1',
            false, 0, '0', false, "V2-SCHED-P24 outside");
        apply_face(73, 0, DIRECTION_CW, '1', '1', '0', '0', '1',
            true, 0, '0', false, "V2-SCHED-P24 next entry");
        resolve_request(true, "V2-SCHED-P24 next entry");

        reset_dut;
        mask_v := (others => '1');
        mask_v(0) := '0';
        config_v := fn_test_config(207, 1, 2, mask_v);
        load_config(config_v);
        apply_face(80, 0, DIRECTION_CW, '1', '1', '0', '0', '1',
            false, 0, '0', false, "V2-SCHED-P24 masked Face");

        reset_dut;
        config_v := fn_test_config(208, 1, 2);
        load_config(config_v);
        apply_face(90, 0, DIRECTION_CW, '1', '1', '0', '1', '1',
            false, 0, '0', false, "V2-SCHED-P24 overlap fail-close");
        check(overrun_sticky = '0' and overrun_count = 0,
            "V2-SCHED-P24 invalid context polluted overrun diagnostics");
        report "V2-SCHED-P24 operation/context fail-close PASS"
            severity note;

        -- P25: a Face-close boundary blocks the current angular session but
        -- must not restart the ARM-only two-clock stale-event quarantine.
        reset_dut;
        config_v := fn_test_config(209, 1, 1);
        load_config(config_v);
        boundary_block <= '1';
        wait_clocks(5);
        boundary_block <= '0';
        apply_face(100, 0, DIRECTION_CW, '1', '1', '0', '0', '1',
            true, 0, '1', false,
            "V2-SCHED-P25 post-boundary entry");
        resolve_request(true, "V2-SCHED-P25 post-boundary entry");
        report "V2-SCHED-P25 boundary block preserves ARM quarantine PASS"
            severity note;

        report "LIDAR_V2_SHOT_SCHEDULER_PASS proc_mhz=" &
            integer'image(G_PROC_CLK_MHZ) severity note;
        stop_clock <= true;
        wait for G_PROC_HALF_PERIOD_PS * 3 ps;
        stop;
        wait;
    end process p_test;

end architecture sim;
