-- ============================================================================
-- 테스트 자산 목적: RUN/STOP/ARM/DISARM과 외부 laser permit의 안전 상태를 검증한다.
-- 핵심 검증 계약: reset은 fail-safe이며 source mode와 fire/start enable이 동시에 열리지 않는다.
-- 관련 RTL: lidar_operation_subsystem, lidar_operation_manager.
-- 실행 회귀: scripts/run_v2_operation.ps1
-- 유지보수 주의: 새 운용 상태는 허용 전이와 금지 전이를 모두 assertion으로 추가한다.
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

entity tb_lidar_operation_subsystem is
    generic (
        G_PROC_CLK_MHZ        : positive := 150;
        G_PROC_HALF_PERIOD_PS : positive := 3333
    );
end entity tb_lidar_operation_subsystem;

architecture sim of tb_lidar_operation_subsystem is

    constant C_CSR_HALF_PERIOD  : time := 5 ns;
    constant C_PROC_HALF_PERIOD : time := G_PROC_HALF_PERIOD_PS * 1 ps;

    function fn_active_config(
        simulation_value : std_logic;
        version_value    : positive
    ) return lidar_active_config_t is
        variable source_v : lidar_runtime_config_t :=
            C_DEFAULT_RUNTIME_CONFIG;
        variable result : lidar_active_config_t;
    begin
        source_v.motor.simulation_mode := simulation_value;
        result.version := to_unsigned(version_value, result.version'length);
        result.source := source_v;
        result.derived := fn_derive_runtime_config(
            C_DEFAULT_BUILD_CONFIG, source_v);
        return result;
    end function fn_active_config;

    constant C_PHYSICAL_CONFIG : lidar_active_config_t :=
        fn_active_config('0', 41);
    constant C_SIMULATION_CONFIG : lidar_active_config_t :=
        fn_active_config('1', 42);

    signal csr_clk    : std_logic := '0';
    signal proc_clk   : std_logic := '0';
    signal csr_rst_n  : std_logic := '0';
    signal proc_rst_n : std_logic := '0';
    signal stop_clocks: boolean := false;

    signal command_valid_csr : std_logic := '0';
    signal command_csr       : operation_command_t := OP_COMMAND_NONE;
    signal command_ready_csr : std_logic;
    signal command_busy_csr  : std_logic;
    signal command_rejected_csr : std_logic;

    signal external_permit : std_logic := '0';
    signal config_enable   : std_logic := '0';
    signal active_valid    : std_logic := '0';
    signal active_config   : lidar_active_config_t := C_PHYSICAL_CONFIG;
    signal pipeline_idle   : std_logic := '1';

    signal state_proc : operation_state_t;
    signal state_csr  : operation_state_t;
    signal command_accepted_proc : std_logic;
    signal command_rejected_proc : std_logic;
    signal permit_trip_proc      : std_logic;
    signal safe_to_prepare       : std_logic;

begin

    u_dut : entity work.lidar_operation_subsystem
        generic map (
            G_BUILD_CONFIG => C_DEFAULT_BUILD_CONFIG
        )
        port map (
            i_csr_clk                 => csr_clk,
            i_csr_rst_n               => csr_rst_n,
            i_csr_command_valid       => command_valid_csr,
            i_csr_command             => command_csr,
            o_csr_command_ready       => command_ready_csr,
            o_csr_command_busy        => command_busy_csr,
            o_csr_command_rejected    => command_rejected_csr,
            i_proc_clk                => proc_clk,
            i_proc_rst_n              => proc_rst_n,
            i_external_laser_permit   => external_permit,
            i_config_enable           => config_enable,
            i_active_valid            => active_valid,
            i_active_config           => active_config,
            i_pipeline_idle           => pipeline_idle,
            o_state_proc              => state_proc,
            o_state_csr               => state_csr,
            o_command_accepted_proc   => command_accepted_proc,
            o_command_rejected_proc   => command_rejected_proc,
            o_permit_trip_proc        => permit_trip_proc,
            o_safe_to_prepare         => safe_to_prepare
        );

    p_csr_clock : process
    begin
        while not stop_clocks loop
            csr_clk <= '0';
            wait for C_CSR_HALF_PERIOD;
            csr_clk <= '1';
            wait for C_CSR_HALF_PERIOD;
        end loop;
        wait;
    end process p_csr_clock;

    p_proc_clock : process
    begin
        while not stop_clocks loop
            proc_clk <= '0';
            wait for C_PROC_HALF_PERIOD;
            proc_clk <= '1';
            wait for C_PROC_HALF_PERIOD;
        end loop;
        wait;
    end process p_proc_clock;

    p_test : process
        procedure check(
            condition_value : boolean;
            message_value   : string
        ) is
        begin
            assert condition_value report message_value severity failure;
        end procedure check;

        procedure wait_proc_clocks(count_value : positive) is
        begin
            for index in 1 to count_value loop
                wait until rising_edge(proc_clk);
                wait for 1 ps;
            end loop;
        end procedure wait_proc_clocks;

        procedure wait_csr_clocks(count_value : positive) is
        begin
            for index in 1 to count_value loop
                wait until rising_edge(csr_clk);
                wait for 1 ps;
            end loop;
        end procedure wait_csr_clocks;

        procedure wait_source_ready is
        begin
            for index in 0 to 100 loop
                if command_ready_csr = '1' then
                    return;
                end if;
                wait until rising_edge(csr_clk);
                wait for 1 ps;
            end loop;
            assert false report "V2-OP-CDC source did not become ready"
                severity failure;
        end procedure wait_source_ready;

        procedure send_command(
            command_value : operation_command_t;
            expect_accept : boolean;
            case_name     : string
        ) is
            variable observed : boolean := false;
        begin
            wait_source_ready;
            wait until falling_edge(csr_clk);
            command_csr       <= command_value;
            command_valid_csr <= '1';
            wait until rising_edge(csr_clk);
            wait for 1 ps;
            command_valid_csr <= '0';
            command_csr       <= OP_COMMAND_NONE;

            for index in 0 to 100 loop
                wait until rising_edge(proc_clk);
                wait for 1 ps;
                if command_accepted_proc = '1' or
                   command_rejected_proc = '1' then
                    if expect_accept then
                        check(command_accepted_proc = '1' and
                              command_rejected_proc = '0',
                            case_name & " expected command acceptance");
                    else
                        check(command_rejected_proc = '1' and
                              command_accepted_proc = '0',
                            case_name & " expected command rejection");
                    end if;
                    observed := true;
                    exit;
                end if;
            end loop;
            check(observed, case_name & " command result timeout");
            wait_source_ready;
        end procedure send_command;

        procedure reset_both is
        begin
            command_valid_csr <= '0';
            command_csr       <= OP_COMMAND_NONE;
            config_enable     <= '0';
            active_valid      <= '0';
            pipeline_idle     <= '1';
            external_permit   <= 'X';
            csr_rst_n         <= '0';
            proc_rst_n        <= '0';
            wait_csr_clocks(3);
            wait_proc_clocks(3);
            check(state_proc = C_OPERATION_STATE_SAFE and
                  safe_to_prepare = '1',
                "V2-OP-P40 reset is not fail-safe");
            csr_rst_n  <= '1';
            proc_rst_n <= '1';
            wait_source_ready;
        end procedure reset_both;

    begin
        reset_both;

        send_command(OP_COMMAND_RUN, false,
            "V2-OP-P40 RUN without active config");
        check(state_proc.running = '0' and state_proc.armed = '0' and
              state_proc.physical_fire_enable = '0',
            "V2-OP-P40 invalid config escaped safe state");
        report "V2-OP-P40 reset/config fail-safe PASS" severity note;

        active_config <= C_PHYSICAL_CONFIG;
        active_valid  <= '1';
        config_enable <= '1';
        external_permit <= '0';
        wait_proc_clocks(3);

        send_command(OP_COMMAND_RUN, true, "V2-OP-P41 RUN");
        check(state_proc.running = '1' and state_proc.armed = '0' and
              state_proc.processing_enable = '1' and
              state_proc.scheduler_enable = '0',
            "V2-OP-P41 RUN state mismatch");
        send_command(OP_COMMAND_ARM, false,
            "V2-OP-P41 ARM without permit");

        external_permit <= '1';
        for index in 0 to 10 loop
            exit when state_proc.external_permit = '1';
            wait_proc_clocks(1);
        end loop;
        check(state_proc.external_permit = '1',
            "V2-OP-P41 permit did not qualify");
        send_command(OP_COMMAND_ARM, true, "V2-OP-P41 physical ARM");
        check(state_proc.armed = '1' and
              state_proc.scheduler_enable = '1' and
              state_proc.physical_fire_enable = '1' and
              state_proc.simulation_enable = '0',
            "V2-OP-P41 physical enable mismatch");

        wait for C_PROC_HALF_PERIOD / 2;
        external_permit <= 'X';
        wait for 1 ps;
        check(state_proc.physical_fire_enable = '0' and
              state_proc.scheduler_enable = '0',
            "V2-OP-P41 raw permit did not close the physical gate");
        wait until rising_edge(proc_clk);
        wait for 1 ps;
        check(state_proc.armed = '0' and permit_trip_proc = '1',
            "V2-OP-P41 permit loss did not force DISARM");
        external_permit <= '1';
        wait_proc_clocks(4);
        check(state_proc.armed = '0' and
              state_proc.physical_fire_enable = '0',
            "V2-OP-P41 permit return bypassed explicit ARM");
        send_command(OP_COMMAND_ARM, true, "V2-OP-P41 re-ARM");
        for index in 0 to 10 loop
            exit when state_csr.running = '1' and
                state_csr.armed = '1' and
                state_csr.physical_fire_enable = '1';
            wait_csr_clocks(1);
        end loop;
        check(state_csr.running = '1' and state_csr.armed = '1' and
              state_csr.physical_fire_enable = '1',
            "V2-OP-P41 CSR status synchronization mismatch");
        report "V2-OP-P41 physical permit/re-arm PASS" severity note;

        pipeline_idle <= '0';
        config_enable <= '0';
        wait for 1 ps;
        check(state_proc.running = '1' and state_proc.armed = '1' and
              state_proc.scheduler_enable = '0' and
              safe_to_prepare = '0',
            "V2-OP-P42 commit gate did not close operation");
        pipeline_idle <= '1';
        wait for 1 ps;
        check(safe_to_prepare = '1',
            "V2-OP-P42 idle pipeline was not safe to prepare");
        config_enable <= '1';
        wait for 1 ps;
        check(state_proc.scheduler_enable = '1' and
              state_proc.physical_fire_enable = '1',
            "V2-OP-P42 operation did not reopen after commit gate");
        send_command(OP_COMMAND_STOP, true, "V2-OP-P42 STOP");
        check(state_proc.running = '0' and state_proc.armed = '0' and
              state_proc.scheduler_enable = '0',
            "V2-OP-P42 STOP left latent operation state");
        report "V2-OP-P42 commit gate/STOP priority PASS" severity note;

        config_enable <= '0';
        active_config <= C_SIMULATION_CONFIG;
        wait_proc_clocks(1);
        config_enable <= '1';
        external_permit <= 'Z';
        wait_proc_clocks(2);
        send_command(OP_COMMAND_RUN, true, "V2-OP-P43 simulation RUN");
        send_command(OP_COMMAND_ARM, true, "V2-OP-P43 simulation ARM");
        check(state_proc.scheduler_enable = '1' and
              state_proc.simulation_enable = '1' and
              state_proc.physical_fire_enable = '0',
            "V2-OP-P43 simulation/physical exclusion mismatch");
        for index in 0 to 10 loop
            exit when state_csr.scheduler_enable = '1' and
                state_csr.simulation_enable = '1';
            wait_csr_clocks(1);
        end loop;
        check(state_csr.scheduler_enable = '1' and
              state_csr.simulation_enable = '1' and
              state_csr.physical_fire_enable = '0',
            "V2-OP-P43 derived CSR scheduler status mismatch");
        external_permit <= '1';
        wait_proc_clocks(3);
        check(state_proc.simulation_enable = '1' and
              state_proc.physical_fire_enable = '0',
            "V2-OP-P43 permit enabled physical path in simulation");
        send_command(OP_COMMAND_DISARM, true,
            "V2-OP-P43 simulation DISARM");
        check(state_proc.running = '1' and state_proc.armed = '0' and
              state_proc.scheduler_enable = '0',
            "V2-OP-P43 DISARM changed RUN or left scheduler enabled");
        report "V2-OP-P43 simulation exclusion PASS" severity note;

        -- Submit a second source event before acknowledgement. It must be
        -- diagnosed and must not overwrite the first mailbox payload.
        send_command(OP_COMMAND_STOP, true, "V2-OP-P44 setup STOP");
        wait_source_ready;
        wait until falling_edge(csr_clk);
        command_csr       <= OP_COMMAND_RUN;
        command_valid_csr <= '1';
        wait until rising_edge(csr_clk);
        wait for 1 ps;
        command_valid_csr <= '0';
        wait until falling_edge(csr_clk);
        command_csr       <= OP_COMMAND_STOP;
        command_valid_csr <= '1';
        wait until rising_edge(csr_clk);
        wait for 1 ps;
        check(command_rejected_csr = '1',
            "V2-OP-P44 busy mailbox did not reject overwrite");
        command_valid_csr <= '0';
        command_csr       <= OP_COMMAND_NONE;
        for index in 0 to 100 loop
            exit when command_accepted_proc = '1';
            wait_proc_clocks(1);
        end loop;
        check(state_proc.running = '1',
            "V2-OP-P44 accepted RUN payload was overwritten");
        wait_source_ready;
        report "V2-OP-P44 acknowledged command CDC PASS" severity note;

        -- A Processing reset flushes an in-flight RUN/ARM and leaves the
        -- destination stopped/disarmed. Software must wait for READY and retry.
        send_command(OP_COMMAND_STOP, true, "V2-OP-P45 setup STOP");
        wait until falling_edge(csr_clk);
        command_csr       <= OP_COMMAND_RUN;
        command_valid_csr <= '1';
        wait until rising_edge(csr_clk);
        wait for 1 ps;
        command_valid_csr <= '0';
        command_csr       <= OP_COMMAND_NONE;
        proc_rst_n <= '0';
        wait for 1 ps;
        check(state_proc.running = '0' and state_proc.armed = '0' and
              state_proc.physical_fire_enable = '0',
            "V2-OP-P45 Processing reset was not immediately safe");
        wait_proc_clocks(3);
        proc_rst_n <= '1';
        wait_source_ready;
        wait_proc_clocks(5);
        check(state_proc.running = '0' and state_proc.armed = '0' and
              state_proc.scheduler_enable = '0',
            "V2-OP-P45 stale command replayed after reset");
        report "V2-OP-P45 destination reset flush PASS" severity note;

        -- A CSR-domain reset removes the command authority. The Processing
        -- owner must stop and disarm regardless of the request-toggle parity.
        active_config   <= C_PHYSICAL_CONFIG;
        config_enable   <= '1';
        active_valid    <= '1';
        external_permit <= '1';
        wait_proc_clocks(4);
        send_command(OP_COMMAND_RUN, true, "V2-OP-P46 setup RUN");
        send_command(OP_COMMAND_ARM, true, "V2-OP-P46 setup ARM");
        check(state_proc.physical_fire_enable = '1',
            "V2-OP-P46 setup did not enable physical operation");
        csr_rst_n <= '0';
        wait_proc_clocks(4);
        check(state_proc.running = '0' and state_proc.armed = '0' and
              state_proc.scheduler_enable = '0' and
              state_proc.physical_fire_enable = '0',
            "V2-OP-P46 CSR reset did not revoke command authority");
        wait_csr_clocks(3);
        csr_rst_n <= '1';
        wait_source_ready;
        wait_proc_clocks(3);
        check(state_proc.running = '0' and state_proc.armed = '0',
            "V2-OP-P46 CSR reset release restored stale operation state");
        report "V2-OP-P46 source reset fail-safe PASS" severity note;

        report "LIDAR_V2_OPERATION_PASS proc_mhz=" &
            integer'image(G_PROC_CLK_MHZ) severity note;
        stop_clocks <= true;
        stop;
        wait;
    end process p_test;

end architecture sim;
