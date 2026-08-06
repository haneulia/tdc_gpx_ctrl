library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library std;
use std.env.all;

use work.lidar_build_pkg.all;
use work.lidar_config_types_pkg.all;
use work.lidar_config_reference_pkg.all;
use work.lidar_event_types_pkg.all;

entity tb_laser_executor is
    generic (
        G_PROC_CLK_MHZ        : positive := 150;
        G_PROC_HALF_PERIOD_PS : positive := 3333
    );
end entity tb_laser_executor;

architecture sim of tb_laser_executor is

    constant C_FIRE_WIDTH   : positive := 4;
    constant C_FIRE_TIMEOUT : positive := 12;
    constant C_RANGE_WINDOW : positive := 6;
    constant C_START_WIDTH  : positive := 3;
    constant C_STOP_WIDTH   : positive := 2;
    constant C_SIM_DELAY    : positive := 3;

    function fn_test_build return lidar_build_config_t is
        variable result : lidar_build_config_t := C_DEFAULT_BUILD_CONFIG;
    begin
        result.proc_clk_mhz := G_PROC_CLK_MHZ;
        return result;
    end function fn_test_build;

    constant C_TEST_BUILD : lidar_build_config_t := fn_test_build;

    function fn_test_config(
        simulation_mode : std_logic;
        timeout_clks    : positive := C_FIRE_TIMEOUT
    ) return lidar_active_config_t is
        variable result   : lidar_active_config_t;
        variable source_v : lidar_runtime_config_t;
    begin
        source_v := fn_default_runtime_config(C_TEST_BUILD);
        source_v.motor.simulation_mode := simulation_mode;
        result.version := to_unsigned(401, result.version'length);
        result.source  := source_v;
        result.derived := fn_derive_runtime_config(C_TEST_BUILD, source_v);
        result.derived.fire_width_proc_clks := to_unsigned(
            C_FIRE_WIDTH, result.derived.fire_width_proc_clks'length);
        result.derived.fire_done_timeout_proc_clks := to_unsigned(
            timeout_clks,
            result.derived.fire_done_timeout_proc_clks'length);
        result.derived.target_range_proc_clks := to_unsigned(
            C_RANGE_WINDOW, result.derived.target_range_proc_clks'length);
        result.derived.start_width_proc_clks := to_unsigned(
            C_START_WIDTH, result.derived.start_width_proc_clks'length);
        result.derived.stop_width_proc_clks := to_unsigned(
            C_STOP_WIDTH, result.derived.stop_width_proc_clks'length);
        result.derived.simulation_start_delay_proc_clks := to_unsigned(
            C_SIM_DELAY,
            result.derived.simulation_start_delay_proc_clks'length);
        return result;
    end function fn_test_config;

    constant C_PHYSICAL_CONFIG : lidar_active_config_t :=
        fn_test_config('0');
    constant C_BOUNDARY_CONFIG : lidar_active_config_t :=
        fn_test_config('0', 4);
    constant C_SIM_CONFIG : lidar_active_config_t :=
        fn_test_config('1');

    signal clk           : std_logic := '0';
    signal rst_n         : std_logic := '0';
    signal stop_clock    : boolean := false;
    signal active_valid  : std_logic := '0';
    signal active_config : lidar_active_config_t := C_PHYSICAL_CONFIG;
    signal operation_state : operation_state_t := C_OPERATION_STATE_SAFE;
    signal shot_request  : shot_request_t := C_SHOT_REQUEST_IDLE;
    signal fire_done_raw : std_logic := '0';
    signal clear_diag    : std_logic := '0';

    signal executor_ready : std_logic;
    signal request_accept : std_logic;
    signal request_drop   : std_logic;
    signal fire_pulse     : std_logic;
    signal start_tdc      : std_logic;
    signal stop_tdc       : std_logic;
    signal shot_start     : shot_start_event_t;
    signal shot_result    : shot_result_t;
    signal current_request: shot_request_t;
    signal busy           : std_logic;
    signal physical_arm   : std_logic;
    signal rearm_active   : std_logic;
    signal sync_clks      : unsigned(15 downto 0);
    signal rearm_clks     : unsigned(15 downto 0);
    signal diagnostics    : laser_diagnostics_t;

begin

    u_dut : entity work.laser_executor
        generic map (
            G_BUILD_CONFIG => C_TEST_BUILD
        )
        port map (
            i_clk                 => clk,
            i_rst_n               => rst_n,
            i_active_valid        => active_valid,
            i_active_config       => active_config,
            i_operation_state     => operation_state,
            i_shot_request        => shot_request,
            i_fire_done_raw       => fire_done_raw,
            i_clear_diagnostics   => clear_diag,
            o_executor_ready      => executor_ready,
            o_request_accept      => request_accept,
            o_request_drop        => request_drop,
            o_fire_pulse          => fire_pulse,
            o_start_tdc           => start_tdc,
            o_stop_tdc            => stop_tdc,
            o_shot_start          => shot_start,
            o_shot_result         => shot_result,
            o_current_request     => current_request,
            o_busy                => busy,
            o_physical_arm        => physical_arm,
            o_rearm_active        => rearm_active,
            o_fire_done_sync_clks => sync_clks,
            o_rearm_margin_clks   => rearm_clks,
            o_diagnostics         => diagnostics
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

        procedure set_operation(simulation_mode : std_logic) is
            variable state_v : operation_state_t := C_OPERATION_STATE_SAFE;
        begin
            state_v.running           := '1';
            state_v.armed             := '1';
            state_v.config_ready      := '1';
            state_v.processing_enable := '1';
            state_v.scheduler_enable  := '1';
            if simulation_mode = '1' then
                state_v.simulation_enable := '1';
            else
                state_v.external_permit      := '1';
                state_v.physical_fire_enable := '1';
            end if;
            operation_state <= state_v;
        end procedure set_operation;

        procedure reset_dut(
            constant config_value : in lidar_active_config_t;
            constant sim_value    : in std_logic
        ) is
        begin
            rst_n          <= '0';
            active_valid   <= '0';
            active_config  <= config_value;
            operation_state <= C_OPERATION_STATE_SAFE;
            shot_request   <= C_SHOT_REQUEST_IDLE;
            fire_done_raw  <= '0';
            clear_diag     <= '0';
            wait_clocks(4);
            active_valid <= '1';
            set_operation(sim_value);
            rst_n <= '1';
            wait_clocks(6);
        end procedure reset_dut;

        procedure wait_ready(case_name : string) is
            variable wait_count : natural := 0;
        begin
            while executor_ready /= '1' loop
                wait_clocks(1);
                wait_count := wait_count + 1;
                check(wait_count < 24, case_name & " ready timeout");
            end loop;
        end procedure wait_ready;

        procedure issue_request(
            constant index_value : in natural;
            constant sim_value   : in std_logic;
            constant expect_ok   : in boolean;
            constant case_name   : in string
        ) is
            variable request_v : shot_request_t := C_SHOT_REQUEST_IDLE;
        begin
            request_v.valid                := '1';
            request_v.face_index           := to_unsigned(2, 3);
            request_v.position             := to_unsigned(123, C_POSITION_WIDTH);
            request_v.direction            := DIRECTION_CW;
            request_v.shot_index           := to_unsigned(index_value, 16);
            request_v.last_in_face         := '0';
            request_v.source_sim           := sim_value;
            request_v.source_latency_clks  := to_unsigned(4,
                request_v.source_latency_clks'length);
            request_v.source_latency_valid := '1';
            request_v.active_version       := active_config.version;

            wait until falling_edge(clk);
            shot_request <= request_v;
            wait until rising_edge(clk);
            wait for 1 ps;
            if expect_ok then
                check(request_accept = '1' and request_drop = '0',
                    case_name & " request was not accepted");
                check(current_request.shot_index = index_value and
                      current_request.face_index = 2,
                    case_name & " accepted identity mismatch");
            else
                check(request_accept = '0' and request_drop = '1',
                    case_name & " request was not dropped");
            end if;
            shot_request <= C_SHOT_REQUEST_IDLE;
        end procedure issue_request;

        procedure check_registered_width(
            signal pulse_value  : in std_logic;
            constant width_value: in positive;
            constant case_name  : in string
        ) is
            variable observed : natural := 0;
        begin
            while pulse_value = '1' loop
                observed := observed + 1;
                wait until rising_edge(clk);
                wait for 1 ps;
                check(observed <= width_value,
                    case_name & " pulse remained high too long");
            end loop;
            check(observed = width_value,
                case_name & " pulse width mismatch");
        end procedure check_registered_width;

        procedure pulse_raw_done is
        begin
            wait until falling_edge(clk);
            wait for 1 ns;
            fire_done_raw <= '1';
            wait for 1 ps;
            check(start_tdc = '1',
                "physical START did not assert from raw fire_done");
            wait for 500 ps;
            fire_done_raw <= '0';
        end procedure pulse_raw_done;

        variable start_events : natural;
        variable stop_events  : natural;
        variable wait_count   : natural;
        variable start_samples: natural;
        variable state_v      : operation_state_t;
    begin
        wait for 1 ps;
        check(to_integer(sync_clks) = 3 and to_integer(rearm_clks) = 2,
            "read-only B3 timing constants mismatch");

        -- P30: physical success, raw low-latency START and exact identity.
        reset_dut(C_PHYSICAL_CONFIG, '0');
        wait_ready("V2-LASER-P30");
        issue_request(7, '0', true, "V2-LASER-P30");
        check(fire_pulse = '1' and physical_arm = '1' and busy = '1',
            "V2-LASER-P30 fire/arm did not start at accept");
        check_registered_width(fire_pulse, C_FIRE_WIDTH,
            "V2-LASER-P30 fire");
        pulse_raw_done;

        start_events  := 0;
        start_samples := 0;
        for index in 1 to C_START_WIDTH loop
            wait until rising_edge(clk);
            wait for 1 ps;
            check(start_tdc = '1',
                "V2-LASER-P30 physical START shorter than configured minimum");
            start_samples := start_samples + 1;
            if shot_start.valid = '1' then
                start_events := start_events + 1;
                check(shot_start.request.face_index = 2 and
                      shot_start.request.shot_index = 7 and
                      shot_start.request.active_version = active_config.version,
                    "V2-LASER-P30 shot_start identity mismatch");
                check(shot_start.t0_timestamp_valid = '1' and
                      shot_start.t0_timestamp_ticks /= 0 and
                      shot_start.t0_time_sync_valid = '0',
                    "V2-LASER-P30 physical T0 timestamp mismatch");
            end if;
        end loop;
        check(start_samples = C_START_WIDTH and start_events = 1,
            "V2-LASER-P30 synchronized T0 count mismatch");

        for index in 1 to C_RANGE_WINDOW loop
            wait until rising_edge(clk);
            wait for 1 ps;
            if index < C_RANGE_WINDOW then
                check(stop_tdc = '0',
                    "V2-LASER-P30 STOP asserted before range end");
            else
                check(stop_tdc = '1' and shot_result.valid = '1' and
                      shot_result.timeout = '0' and
                      shot_result.aborted = '0',
                    "V2-LASER-P30 normal result/STOP mismatch");
                check(shot_result.request.shot_index = 7,
                    "V2-LASER-P30 result identity mismatch");
            end if;
        end loop;
        check_registered_width(stop_tdc, C_STOP_WIDTH,
            "V2-LASER-P30 STOP");
        wait_clocks(1);
        check(executor_ready = '0',
            "V2-LASER-P30 re-arm margin too short");
        wait_clocks(1);
        check(executor_ready = '1',
            "V2-LASER-P30 executor did not re-arm");
        report "V2-LASER-P30 physical success PASS" severity note;

        -- P31: stale HIGH is not ready; a missing response times out with no
        -- START or STOP and increments the dedicated lifetime counter.
        reset_dut(C_PHYSICAL_CONFIG, '0');
        fire_done_raw <= '1';
        wait_clocks(5);
        check(executor_ready = '0' and start_tdc = '0',
            "V2-LASER-P31 stale HIGH became ready or START");
        issue_request(8, '0', false, "V2-LASER-P31 stale-high");
        check(fire_pulse = '0',
            "V2-LASER-P31 dropped stale-high request fired laser");
        fire_done_raw <= '0';
        wait_ready("V2-LASER-P31 requalification");
        issue_request(9, '0', true, "V2-LASER-P31 timeout");
        start_events := 0;
        stop_events  := 0;
        wait_count   := 0;
        while shot_result.valid /= '1' loop
            wait_clocks(1);
            wait_count := wait_count + 1;
            if start_tdc = '1' then start_events := start_events + 1; end if;
            if stop_tdc = '1' then stop_events := stop_events + 1; end if;
            check(wait_count < 32, "V2-LASER-P31 timeout result missing");
        end loop;
        check(shot_result.timeout = '1' and shot_result.aborted = '0' and
              start_events = 0 and stop_events = 0,
            "V2-LASER-P31 timeout fabricated START/STOP");
        wait_clocks(2);
        check(diagnostics.request_drop_count = 1 and
              diagnostics.fire_done_timeout_count = 1 and
              diagnostics.unexpected_done_count = 1,
            "V2-LASER-P31 diagnostic counters mismatch");
        report "V2-LASER-P31 stale-high and timeout PASS" severity note;

        -- P32: a raw T0 captured immediately before timeout arm closure wins
        -- after the fixed synchronizer-resolution interval.
        reset_dut(C_BOUNDARY_CONFIG, '0');
        wait_ready("V2-LASER-P32");
        issue_request(10, '0', true, "V2-LASER-P32");
        wait_clocks(3);
        pulse_raw_done;
        wait_count := 0;
        while shot_start.valid /= '1' loop
            wait_clocks(1);
            wait_count := wait_count + 1;
            check(wait_count < 8, "V2-LASER-P32 boundary T0 was lost");
        end loop;
        check(shot_start.request.shot_index = 10,
            "V2-LASER-P32 boundary identity mismatch");
        while shot_result.valid /= '1' loop
            wait_clocks(1);
        end loop;
        check(shot_result.timeout = '0' and shot_result.aborted = '0' and
              stop_tdc = '1',
            "V2-LASER-P32 T0 did not win timeout boundary");
        report "V2-LASER-P32 timeout-boundary T0 priority PASS" severity note;

        -- P33: simulation owns START without physical fire or physical arm.
        reset_dut(C_SIM_CONFIG, '1');
        fire_done_raw <= '1';
        wait for 500 ps;
        check(start_tdc = '0',
            "V2-LASER-P33 raw fire_done entered simulation START path");
        fire_done_raw <= '0';
        wait_ready("V2-LASER-P33");
        issue_request(11, '1', true, "V2-LASER-P33");
        check(fire_pulse = '0' and physical_arm = '0',
            "V2-LASER-P33 simulation enabled physical output");
        for index in 1 to C_SIM_DELAY - 1 loop
            wait_clocks(1);
            check(start_tdc = '0' and shot_start.valid = '0',
                "V2-LASER-P33 simulation T0 was early");
        end loop;
        wait_clocks(1);
        check(start_tdc = '1' and shot_start.valid = '1' and
              shot_start.request.source_sim = '1' and
              shot_start.request.shot_index = 11 and
              shot_start.t0_timestamp_valid = '1' and
              shot_start.t0_timestamp_ticks /= 0 and
              shot_start.t0_time_sync_valid = '0',
            "V2-LASER-P33 simulation T0/identity mismatch");
        start_samples := 1;
        for index in 1 to C_RANGE_WINDOW loop
            wait_clocks(1);
            if start_tdc = '1' then
                start_samples := start_samples + 1;
            end if;
            if index < C_RANGE_WINDOW then
                check(stop_tdc = '0',
                    "V2-LASER-P33 STOP was early");
            else
                check(stop_tdc = '1' and shot_result.valid = '1' and
                      shot_result.timeout = '0',
                    "V2-LASER-P33 simulation completion mismatch");
            end if;
        end loop;
        check(start_samples = C_START_WIDTH,
            "V2-LASER-P33 simulation START width mismatch");
        report "V2-LASER-P33 simulation isolation PASS" severity note;

        -- P34: a busy duplicate is resolved as drop. Permit loss closes the
        -- fire pin immediately and finishes as abort without START/STOP.
        reset_dut(C_PHYSICAL_CONFIG, '0');
        wait_ready("V2-LASER-P34");
        issue_request(12, '0', true, "V2-LASER-P34 primary");
        issue_request(13, '0', false, "V2-LASER-P34 busy duplicate");
        wait until falling_edge(clk);
        state_v := operation_state;
        state_v.external_permit      := '0';
        state_v.physical_fire_enable := '0';
        state_v.scheduler_enable     := '0';
        operation_state <= state_v;
        wait for 1 ps;
        check(fire_pulse = '0',
            "V2-LASER-P34 permit loss did not close final fire gate");
        start_events := 0;
        stop_events  := 0;
        wait_count   := 0;
        while shot_result.valid /= '1' loop
            wait_clocks(1);
            wait_count := wait_count + 1;
            if start_tdc = '1' then start_events := start_events + 1; end if;
            if stop_tdc = '1' then stop_events := stop_events + 1; end if;
            check(wait_count < 16, "V2-LASER-P34 abort result missing");
        end loop;
        check(shot_result.aborted = '1' and shot_result.timeout = '0' and
              start_events = 0 and stop_events = 0 and
              shot_result.request.shot_index = 12,
            "V2-LASER-P34 abort lifecycle mismatch");
        wait_clocks(2);
        check(diagnostics.request_drop_count = 1 and
              diagnostics.operation_abort_count = 1,
            "V2-LASER-P34 drop/abort diagnostics mismatch");
        clear_diag <= '1';
        wait_clocks(1);
        clear_diag <= '0';
        check(diagnostics.request_drop_sticky = '0' and
              diagnostics.operation_abort_sticky = '0',
            "V2-LASER-P34 diagnostic clear failed");
        report "V2-LASER-P34 widths, busy and fail-safe abort PASS"
            severity note;

        -- P36: an armed fire_done that sticks HIGH still produces one bounded
        -- START. The executor cannot re-arm until the raw input returns LOW
        -- and passes the normal two-sample qualification again.
        reset_dut(C_PHYSICAL_CONFIG, '0');
        wait_ready("V2-LASER-P36");
        issue_request(14, '0', true, "V2-LASER-P36");
        wait until falling_edge(clk);
        wait for 1 ns;
        fire_done_raw <= '1';
        wait for 1 ps;
        check(start_tdc = '1',
            "V2-LASER-P36 stuck-high edge did not assert START");
        start_events  := 0;
        start_samples := 0;
        for index in 1 to C_START_WIDTH loop
            wait_clocks(1);
            check(start_tdc = '1',
                "V2-LASER-P36 START shorter than configured minimum");
            start_samples := start_samples + 1;
            if shot_start.valid = '1' then
                start_events := start_events + 1;
            end if;
        end loop;
        wait_clocks(1);
        check(start_tdc = '0' and start_samples = C_START_WIDTH and
              start_events = 1,
            "V2-LASER-P36 stuck-high START was not bounded");

        wait_count := 0;
        while shot_result.valid /= '1' loop
            wait_clocks(1);
            wait_count := wait_count + 1;
            check(start_tdc = '0',
                "V2-LASER-P36 START reasserted while raw input stayed HIGH");
            check(wait_count < 24,
                "V2-LASER-P36 normal result missing");
        end loop;
        check(shot_result.timeout = '0' and shot_result.aborted = '0' and
              shot_result.request.shot_index = 14,
            "V2-LASER-P36 stuck-high shot result mismatch");
        wait_clocks(C_STOP_WIDTH + 3);
        check(executor_ready = '0',
            "V2-LASER-P36 re-armed before raw LOW qualification");
        fire_done_raw <= '0';
        wait_ready("V2-LASER-P36 LOW requalification");
        check(diagnostics.unexpected_done_count = 0,
            "V2-LASER-P36 legal T0 was classified as unexpected");
        report "V2-LASER-P36 stuck-high START bound PASS" severity note;

        report "LIDAR_V2_LASER_EXECUTOR_PASS proc_mhz=" &
            integer'image(G_PROC_CLK_MHZ) severity note;
        stop_clock <= true;
        finish;
        wait;
    end process p_test;

end architecture sim;
