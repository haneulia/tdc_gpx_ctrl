-- ============================================================================
-- 테스트 자산 목적: operation부터 laser executor까지 B0~B3 직접 제어 체인을 검증한다.
-- 핵심 검증 계약: Shot 위치, fire, fire_done, TDC start/stop과 busy 억제가 연결된다.
-- 관련 RTL: operation manager, face_tracker, shot_scheduler, laser_executor.
-- 실행 회귀: scripts/run_v2_laser_executor.ps1
-- 유지보수 주의: 블록 latency 변경 시 단위 TB와 이 end-to-end 기준을 동시에 갱신한다.
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

entity tb_v2_laser_control_chain is
    generic (
        G_PROC_CLK_MHZ        : positive := 150;
        G_PROC_HALF_PERIOD_PS : positive := 3333
    );
end entity tb_v2_laser_control_chain;

architecture sim of tb_v2_laser_control_chain is

    function fn_test_build return lidar_build_config_t is
        variable result : lidar_build_config_t := C_DEFAULT_BUILD_CONFIG;
    begin
        result.proc_clk_mhz := G_PROC_CLK_MHZ;
        return result;
    end function fn_test_build;

    constant C_TEST_BUILD : lidar_build_config_t := fn_test_build;

    function fn_chain_config(
        simulation_mode : std_logic;
        version_value   : positive
    ) return lidar_active_config_t is
        variable result   : lidar_active_config_t;
        variable source_v : lidar_runtime_config_t;
    begin
        source_v := fn_default_runtime_config(C_TEST_BUILD);
        source_v.motor.simulation_mode := simulation_mode;
        source_v.laser.face_enable_mask := "00001";
        result.version := to_unsigned(version_value, result.version'length);
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
        result.derived.fire_width_proc_clks := to_unsigned(
            3, result.derived.fire_width_proc_clks'length);
        result.derived.fire_done_timeout_proc_clks := to_unsigned(
            12, result.derived.fire_done_timeout_proc_clks'length);
        result.derived.target_range_proc_clks := to_unsigned(
            5, result.derived.target_range_proc_clks'length);
        result.derived.start_width_proc_clks := to_unsigned(
            2, result.derived.start_width_proc_clks'length);
        result.derived.stop_width_proc_clks := to_unsigned(
            2, result.derived.stop_width_proc_clks'length);
        result.derived.simulation_start_delay_proc_clks := to_unsigned(
            3, result.derived.simulation_start_delay_proc_clks'length);
        return result;
    end function fn_chain_config;

    constant C_PHYSICAL_CONFIG : lidar_active_config_t :=
        fn_chain_config('0', 501);
    constant C_SIM_CONFIG : lidar_active_config_t :=
        fn_chain_config('1', 502);

    signal clk        : std_logic := '0';
    signal rst_n      : std_logic := '0';
    signal stop_clock : boolean := false;

    signal command_valid : std_logic := '0';
    signal command       : operation_command_t := OP_COMMAND_NONE;
    signal source_online : std_logic := '1';
    signal laser_permit  : std_logic := '0';
    signal config_enable : std_logic := '1';
    signal active_valid  : std_logic := '1';
    signal active_config : lidar_active_config_t := C_PHYSICAL_CONFIG;
    signal operation_state  : operation_state_t;
    signal command_accepted : std_logic;
    signal command_rejected : std_logic;
    signal permit_trip      : std_logic;
    signal safe_to_prepare  : std_logic;

    signal position_event : position_event_t := C_POSITION_EVENT_IDLE;
    signal face_event     : face_event_t;
    signal overlap_sticky : std_logic;
    signal overlap_count  : u32_t;
    signal shot_request   : shot_request_t;
    signal overrun_pulse  : std_logic;
    signal overrun_sticky : std_logic;
    signal overrun_count  : u32_t;
    signal scheduler_idle : std_logic;

    signal executor_ready : std_logic;
    signal request_accept : std_logic;
    signal request_drop   : std_logic;
    signal fire_done_raw  : std_logic := '0';
    signal fire_pulse     : std_logic;
    signal start_tdc      : std_logic;
    signal stop_tdc       : std_logic;
    signal shot_start     : shot_start_event_t;
    signal shot_result    : shot_result_t;
    signal current_request: shot_request_t;
    signal executor_busy  : std_logic;
    signal physical_arm   : std_logic;
    signal rearm_active   : std_logic;
    signal diagnostics    : laser_diagnostics_t;
    signal pipeline_idle  : std_logic;

begin

    pipeline_idle <= scheduler_idle and not executor_busy;

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
            i_pipeline_idle         => pipeline_idle,
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

    u_executor : entity work.laser_executor
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
            i_clear_diagnostics   => '0',
            o_executor_ready      => executor_ready,
            o_request_accept      => request_accept,
            o_request_drop        => request_drop,
            o_fire_pulse          => fire_pulse,
            o_start_tdc           => start_tdc,
            o_stop_tdc            => stop_tdc,
            o_shot_start          => shot_start,
            o_shot_result         => shot_result,
            o_current_request     => current_request,
            o_busy                => executor_busy,
            o_physical_arm        => physical_arm,
            o_rearm_active        => rearm_active,
            o_fire_done_sync_clks => open,
            o_rearm_margin_clks   => open,
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
                case_name & " command rejected");
            command_valid <= '0';
            command       <= OP_COMMAND_NONE;
        end procedure send_command;

        procedure reset_chain(
            constant config_value : in lidar_active_config_t;
            constant permit_value : in std_logic
        ) is
        begin
            rst_n          <= '0';
            command_valid  <= '0';
            command        <= OP_COMMAND_NONE;
            laser_permit   <= permit_value;
            config_enable  <= '1';
            active_valid   <= '1';
            active_config  <= config_value;
            position_event <= C_POSITION_EVENT_IDLE;
            fire_done_raw  <= '0';
            wait_clocks(4);
            rst_n <= '1';
            wait_clocks(7);
        end procedure reset_chain;

        procedure pulse_position(
            constant position_value : in natural;
            constant sim_value      : in std_logic
        ) is
            variable event_v : position_event_t := C_POSITION_EVENT_IDLE;
        begin
            event_v.valid                := '1';
            event_v.position             := to_unsigned(
                position_value, event_v.position'length);
            event_v.direction            := DIRECTION_CW;
            event_v.source_sim           := sim_value;
            event_v.source_latency_clks  := to_unsigned(
                4, event_v.source_latency_clks'length);
            event_v.source_latency_valid := '1';
            event_v.active_version       := active_config.version;
            wait until falling_edge(clk);
            position_event <= event_v;
            wait until rising_edge(clk);
            wait for 1 ps;
            position_event <= C_POSITION_EVENT_IDLE;
        end procedure pulse_position;

        procedure wait_for_accept(
            constant case_name : in string
        ) is
            variable wait_count : natural := 0;
        begin
            while request_accept /= '1' loop
                wait_clocks(1);
                wait_count := wait_count + 1;
                check(wait_count < 12, case_name & " accept timeout");
            end loop;
        end procedure wait_for_accept;

        procedure pulse_raw_done is
        begin
            wait until falling_edge(clk);
            wait for 1 ns;
            fire_done_raw <= '1';
            wait for 1 ps;
            check(start_tdc = '1',
                "V2-LASER-P35 raw START latency regression");
            wait for 500 ps;
            fire_done_raw <= '0';
        end procedure pulse_raw_done;

        variable wait_count : natural;
    begin
        -- Physical F3a/B1/B2/B3 path.
        reset_chain(C_PHYSICAL_CONFIG, '1');
        send_command(OP_COMMAND_RUN, "V2-LASER-P35 physical RUN");
        send_command(OP_COMMAND_ARM, "V2-LASER-P35 physical ARM");
        check(operation_state.physical_fire_enable = '1' and
              operation_state.scheduler_enable = '1',
            "V2-LASER-P35 physical operation gate mismatch");
        wait_clocks(3);
        pulse_position(9, '0');
        wait_clocks(2);
        pulse_position(10, '0');
        wait_for_accept("V2-LASER-P35 physical");
        check(fire_pulse = '1' and current_request.face_index = 0 and
              current_request.shot_index = 0 and
              current_request.position = 10 and
              current_request.active_version = C_PHYSICAL_CONFIG.version,
            "V2-LASER-P35 physical request identity mismatch");
        check(pipeline_idle = '0',
            "V2-LASER-P35 pipeline reported idle during shot");
        pulse_raw_done;
        wait_count := 0;
        while shot_start.valid /= '1' loop
            wait_clocks(1);
            wait_count := wait_count + 1;
            check(wait_count < 8, "V2-LASER-P35 physical shot_start timeout");
        end loop;
        check(shot_start.request.face_index = 0 and
              shot_start.request.shot_index = 0 and
              shot_start.request.position = 10 and
              shot_start.request.active_version = C_PHYSICAL_CONFIG.version,
            "V2-LASER-P35 physical shot_start identity changed");
        while shot_result.valid /= '1' loop
            wait_clocks(1);
        end loop;
        check(stop_tdc = '1' and shot_result.timeout = '0' and
              shot_result.aborted = '0' and
              shot_result.request.shot_index = 0,
            "V2-LASER-P35 physical completion mismatch");
        while executor_ready /= '1' loop
            wait_clocks(1);
        end loop;
        check(scheduler_idle = '1' and overrun_count = 0 and
              diagnostics.request_drop_count = 0,
            "V2-LASER-P35 physical handshake/overrun mismatch");

        -- Simulation F3a/B1/B2/B3 path. Permit is deliberately absent.
        reset_chain(C_SIM_CONFIG, '0');
        send_command(OP_COMMAND_RUN, "V2-LASER-P35 simulation RUN");
        send_command(OP_COMMAND_ARM, "V2-LASER-P35 simulation ARM");
        check(operation_state.simulation_enable = '1' and
              operation_state.physical_fire_enable = '0',
            "V2-LASER-P35 simulation operation gate mismatch");
        wait_clocks(3);
        pulse_position(9, '1');
        wait_clocks(2);
        pulse_position(10, '1');
        wait_for_accept("V2-LASER-P35 simulation");
        check(fire_pulse = '0' and physical_arm = '0' and
              current_request.source_sim = '1' and
              current_request.shot_index = 0 and
              current_request.active_version = C_SIM_CONFIG.version,
            "V2-LASER-P35 simulation request isolation mismatch");
        wait_count := 0;
        while shot_start.valid /= '1' loop
            wait_clocks(1);
            wait_count := wait_count + 1;
            check(fire_pulse = '0',
                "V2-LASER-P35 simulation produced physical fire");
            check(wait_count < 8, "V2-LASER-P35 simulation shot_start timeout");
        end loop;
        check(start_tdc = '1' and shot_start.request.face_index = 0 and
              shot_start.request.shot_index = 0 and
              shot_start.request.active_version = C_SIM_CONFIG.version,
            "V2-LASER-P35 simulation shot_start identity changed");
        while shot_result.valid /= '1' loop
            wait_clocks(1);
        end loop;
        check(stop_tdc = '1' and shot_result.timeout = '0' and
              shot_result.aborted = '0' and fire_pulse = '0',
            "V2-LASER-P35 simulation completion mismatch");

        report "V2-LASER-P35 F3a/B1/B2/B3 physical+simulation chain PASS"
            severity note;
        report "LIDAR_V2_LASER_CONTROL_CHAIN_PASS proc_mhz=" &
            integer'image(G_PROC_CLK_MHZ) severity note;
        stop_clock <= true;
        finish;
        wait;
    end process p_test;

end architecture sim;
