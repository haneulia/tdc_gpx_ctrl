-- ============================================================================
-- 테스트 자산 목적: operation, Face tracker와 scheduler의 직접 event 연결을 검증한다.
-- 핵심 검증 계약: AXIS backpressure 없이 B0~B2 위치/Face/Shot 사건 순서를 보존한다.
-- 관련 RTL: lidar_operation_manager, face_tracker, shot_scheduler.
-- 실행 회귀: scripts/run_v2_shot_scheduler.ps1
-- 유지보수 주의: 단위 TB 통과 뒤에만 이 chain의 기대 사건 순서를 변경한다.
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library std;
use std.env.all;

use work.lidar_build_pkg.all;
use work.lidar_config_types_pkg.all;
use work.lidar_config_reference_pkg.all;
use work.lidar_event_types_pkg.all;

entity tb_v2_processing_control_chain is
    generic (
        G_PROC_CLK_MHZ        : positive := 150;
        G_PROC_HALF_PERIOD_PS : positive := 3333
    );
end entity tb_v2_processing_control_chain;

architecture sim of tb_v2_processing_control_chain is

    function fn_test_build return lidar_build_config_t is
        variable result : lidar_build_config_t := C_DEFAULT_BUILD_CONFIG;
    begin
        result.proc_clk_mhz := G_PROC_CLK_MHZ;
        return result;
    end function fn_test_build;

    constant C_TEST_BUILD : lidar_build_config_t := fn_test_build;

    function fn_chain_config return lidar_active_config_t is
        variable result   : lidar_active_config_t;
        variable source_v : lidar_runtime_config_t;
    begin
        source_v := fn_default_runtime_config(C_TEST_BUILD);
        source_v.motor.simulation_mode := '0';
        source_v.laser.face_enable_mask := "00001";
        result.version := to_unsigned(301, result.version'length);
        result.source  := source_v;
        result.derived := fn_derive_runtime_config(C_TEST_BUILD, source_v);
        result.derived.total_states := to_unsigned(
            100, result.derived.total_states'length);
        result.derived.face_lower(0) := to_unsigned(
            10, result.derived.face_lower(0)'length);
        result.derived.face_upper(0) := to_unsigned(
            16, result.derived.face_upper(0)'length);
        result.derived.shot_interval_states := to_unsigned(
            2, result.derived.shot_interval_states'length);
        result.derived.columns_per_face := to_unsigned(
            3, result.derived.columns_per_face'length);
        return result;
    end function fn_chain_config;

    constant C_CHAIN_CONFIG : lidar_active_config_t := fn_chain_config;

    signal clk        : std_logic := '0';
    signal rst_n      : std_logic := '0';
    signal stop_clock : boolean := false;

    signal command_valid : std_logic := '0';
    signal command       : operation_command_t := OP_COMMAND_NONE;
    signal source_online : std_logic := '1';
    signal laser_permit  : std_logic := '0';
    signal config_enable : std_logic := '1';
    signal active_valid  : std_logic := '1';
    signal active_config : lidar_active_config_t := C_CHAIN_CONFIG;

    signal operation_state  : operation_state_t;
    signal command_accepted : std_logic;
    signal command_rejected : std_logic;
    signal permit_trip      : std_logic;
    signal safe_to_prepare  : std_logic;

    signal position_event : position_event_t := C_POSITION_EVENT_IDLE;
    signal face_event     : face_event_t;
    signal overlap_sticky : std_logic;
    signal overlap_count  : u32_t;

    signal executor_ready  : std_logic := '1';
    signal request_accept  : std_logic := '0';
    signal request_drop    : std_logic := '0';
    signal shot_request    : shot_request_t;
    signal overrun_pulse   : std_logic;
    signal overrun_sticky  : std_logic;
    signal overrun_count   : u32_t;
    signal scheduler_idle  : std_logic;

begin

    u_operation : entity work.lidar_operation_manager
        generic map (
            G_BUILD_CONFIG => C_TEST_BUILD
        )
        port map (
            i_clk                   => clk,
            i_rst_n                 => rst_n,
            i_command_valid         => command_valid,
            i_command               => command,
            i_command_source_online => source_online,
            i_external_laser_permit => laser_permit,
            i_config_enable         => config_enable,
            i_active_valid          => active_valid,
            i_active_config         => active_config,
            i_pipeline_idle         => scheduler_idle,
            o_state                 => operation_state,
            o_command_accepted      => command_accepted,
            o_command_rejected      => command_rejected,
            o_permit_trip           => permit_trip,
            o_safe_to_prepare       => safe_to_prepare
        );

    u_face : entity work.face_tracker
        generic map (
            G_BUILD_CONFIG => C_TEST_BUILD
        )
        port map (
            i_clk               => clk,
            i_rst_n             => rst_n,
            i_enable            => operation_state.processing_enable,
            i_active_valid      => active_valid,
            i_active_config     => active_config,
            i_position_event    => position_event,
            i_clear_diagnostics => '0',
            o_face_event        => face_event,
            o_overlap_sticky    => overlap_sticky,
            o_overlap_count     => overlap_count
        );

    u_scheduler : entity work.shot_scheduler
        generic map (
            G_BUILD_CONFIG => C_TEST_BUILD
        )
        port map (
            i_clk                     => clk,
            i_rst_n                   => rst_n,
            i_enable                  => operation_state.scheduler_enable,
            i_active_valid            => active_valid,
            i_active_config           => active_config,
            i_face_event              => face_event,
            i_executor_ready          => executor_ready,
            i_request_accept          => request_accept,
            i_request_drop            => request_drop,
            i_clear_diagnostics       => '0',
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

        procedure send_command(
            constant command_value : in operation_command_t;
            constant case_name     : in string
        ) is
        begin
            wait until falling_edge(clk);
            command       <= command_value;
            command_valid <= '1';
            wait until rising_edge(clk);
            wait for 1 ps;
            check(command_accepted = '1' and command_rejected = '0',
                case_name & " command was not accepted");
            command_valid <= '0';
            command       <= OP_COMMAND_NONE;
        end procedure send_command;

        procedure drive_position(
            constant position_value : in natural;
            constant expect_request : in boolean;
            constant expected_index : in natural;
            constant case_name      : in string
        ) is
            variable event_v : position_event_t;
        begin
            event_v := C_POSITION_EVENT_IDLE;
            event_v.valid                := '1';
            event_v.position             := to_unsigned(
                position_value, event_v.position'length);
            event_v.direction            := DIRECTION_CW;
            event_v.source_sim           := '0';
            event_v.source_latency_clks  :=
                C_POSITION_PHYSICAL_LATENCY_CLKS;
            event_v.source_latency_valid := '1';
            event_v.active_version       := active_config.version;

            wait until falling_edge(clk);
            position_event <= event_v;
            wait until rising_edge(clk);
            wait for 1 ps;
            position_event <= C_POSITION_EVENT_IDLE;
            check(shot_request.valid = '0',
                case_name & " bypassed B1/B2 registered stages");

            wait until rising_edge(clk);
            wait for 1 ps;
            check(shot_request.valid = '0',
                case_name & " bypassed B2 registered stage");

            wait until rising_edge(clk);
            wait for 1 ps;
            if expect_request then
                check(shot_request.valid = '1',
                    case_name & " missing chain request");
                check(to_integer(shot_request.position) = position_value and
                      to_integer(shot_request.shot_index) = expected_index and
                      shot_request.face_index = 0,
                    case_name & " chain request metadata mismatch");
            else
                check(shot_request.valid = '0',
                    case_name & " unexpected chain request");
            end if;
        end procedure drive_position;

        procedure resolve_request(
            constant case_name : in string
        ) is
        begin
            wait until falling_edge(clk);
            request_accept <= '1';
            wait until rising_edge(clk);
            wait for 1 ps;
            request_accept <= '0';
            check(scheduler_idle = '1' and shot_request.valid = '0',
                case_name & " request did not resolve");
        end procedure resolve_request;

    begin
        rst_n          <= '0';
        laser_permit   <= '0';
        command_valid  <= '0';
        position_event <= C_POSITION_EVENT_IDLE;
        wait_clocks(4);
        rst_n <= '1';
        wait_clocks(3);

        laser_permit <= '1';
        wait_clocks(4);
        send_command(OP_COMMAND_RUN, "V2-SCHED-P25 RUN");
        check(operation_state.processing_enable = '1' and
              operation_state.scheduler_enable = '0',
            "V2-SCHED-P25 RUN-only gate mismatch");

        -- Face tracking continues under RUN, but no request is admitted before
        -- ARM. ARM in the middle of that traversal must not create column 0.
        drive_position(10, false, 0, "V2-SCHED-P25 RUN-only entry");
        send_command(OP_COMMAND_ARM, "V2-SCHED-P25 ARM");
        check(operation_state.scheduler_enable = '1',
            "V2-SCHED-P25 scheduler did not enable");
        drive_position(11, false, 0, "V2-SCHED-P25 mid-Face ARM");

        drive_position(17, false, 0, "V2-SCHED-P25 Face exit");
        drive_position(10, true, 0, "V2-SCHED-P25 next Face entry");
        resolve_request("V2-SCHED-P25 first request");

        -- Raw permit loss closes scheduling and removes ARM. Re-ARM while the
        -- Face remains active still waits for a later genuine entry.
        laser_permit <= '0';
        wait_clocks(2);
        check(operation_state.scheduler_enable = '0' and
              operation_state.armed = '0',
            "V2-SCHED-P25 permit loss did not fail safe");
        drive_position(11, false, 0, "V2-SCHED-P25 disabled traversal");

        laser_permit <= '1';
        wait_clocks(4);
        send_command(OP_COMMAND_ARM, "V2-SCHED-P25 re-ARM");
        drive_position(12, false, 0, "V2-SCHED-P25 re-ARM mid-Face");
        drive_position(17, false, 0, "V2-SCHED-P25 second exit");
        drive_position(10, true, 0, "V2-SCHED-P25 second entry");
        resolve_request("V2-SCHED-P25 second request");

        check(overlap_sticky = '0' and overlap_count = 0 and
              overrun_sticky = '0' and overrun_count = 0,
            "V2-SCHED-P25 clean chain polluted diagnostics");
        report "V2-SCHED-P25 F3a/B1/B2 integration PASS" severity note;
        report "LIDAR_V2_PROCESSING_CONTROL_CHAIN_PASS proc_mhz=" &
            integer'image(G_PROC_CLK_MHZ) severity note;

        stop_clock <= true;
        wait for G_PROC_HALF_PERIOD_PS * 3 ps;
        stop;
        wait;
    end process p_test;

end architecture sim;
