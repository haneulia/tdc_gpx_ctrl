-- ============================================================================
-- 테스트 자산 목적: Processing/TDC 양 domain에 대한 atomic COMMIT manager를 검증한다.
-- 핵심 검증 계약: 버전 일치, safe-point, timeout, abort와 실패 시 Active 불변을 보장한다.
-- 관련 RTL: lidar_config_subsystem, lidar_config_manager, lidar_config_gateway.
-- 실행 회귀: scripts/run_v2_config_manager.ps1
-- 유지보수 주의: activation 단계가 바뀌면 두 domain의 순서 assertion을 약화하지 않는다.
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library std;
use std.env.all;

use work.lidar_build_pkg.all;
use work.lidar_config_types_pkg.all;
use work.lidar_config_reference_pkg.all;

entity tb_lidar_config_subsystem is
    generic (
        G_PROC_CLK_MHZ       : positive := 150;
        G_TDC_CLK_MHZ        : positive := 200;
        G_PROC_HALF_PERIOD_PS : positive := 3333;
        G_TDC_HALF_PERIOD_PS  : positive := 2500
    );
end entity tb_lidar_config_subsystem;

architecture sim of tb_lidar_config_subsystem is

    constant C_CSR_HALF_PERIOD  : time := 5 ns;
    constant C_PROC_HALF_PERIOD : time := G_PROC_HALF_PERIOD_PS * 1 ps;
    constant C_TDC_HALF_PERIOD  : time := G_TDC_HALF_PERIOD_PS * 1 ps;
    constant C_MAX_WAIT_CLKS    : positive := 4000;

    function fn_build_profile return lidar_build_config_t is
        variable result : lidar_build_config_t := C_DEFAULT_BUILD_CONFIG;
    begin
        result.proc_clk_mhz := G_PROC_CLK_MHZ;
        result.tdc_clk_mhz  := G_TDC_CLK_MHZ;
        return result;
    end function fn_build_profile;

    constant C_BUILD_CONFIG : lidar_build_config_t := fn_build_profile;

    signal csr_clk       : std_logic := '0';
    signal proc_clk      : std_logic := '0';
    signal tdc_clk       : std_logic := '0';
    signal csr_rst_n     : std_logic := '0';
    signal proc_rst_n    : std_logic := '0';
    signal tdc_rst_n     : std_logic := '0';
    signal stop_clocks   : boolean := false;
    signal commit        : std_logic := '0';
    signal shadow        : lidar_runtime_config_t := C_DEFAULT_RUNTIME_CONFIG;
    signal proc_safe     : std_logic := '1';
    signal tdc_safe      : std_logic := '1';
    signal busy          : std_logic;
    signal done          : std_logic;
    signal rejected      : std_logic;
    signal reject_error  : lidar_cfg_error_t;
    signal error_value   : lidar_cfg_error_t;
    signal recovery_required : std_logic;
    signal active_valid  : std_logic;
    signal active_cfg    : lidar_active_config_t;
    signal proc_enable   : std_logic;
    signal proc_active_valid : std_logic;
    signal proc_active   : lidar_active_config_t;
    signal tdc_enable    : std_logic;
    signal tdc_active_valid : std_logic;
    signal tdc_active    : lidar_active_config_t;
    signal prepare_req   : std_logic;
    signal activate_req  : std_logic;
    signal release_req   : std_logic;

begin

    u_dut : entity work.lidar_config_subsystem
        generic map (
            G_BUILD_CONFIG       => C_BUILD_CONFIG,
            G_CSR_CLK_MHZ        => 100,
            G_PHASE_TIMEOUT_US   => 1
        )
        port map (
            i_csr_clk           => csr_clk,
            i_csr_rst_n         => csr_rst_n,
            i_proc_clk          => proc_clk,
            i_proc_rst_n        => proc_rst_n,
            i_tdc_clk           => tdc_clk,
            i_tdc_rst_n         => tdc_rst_n,
            i_commit            => commit,
            i_shadow            => shadow,
            i_proc_safe         => proc_safe,
            i_tdc_safe          => tdc_safe,
            i_tdc_activate_complete => '1',
            i_tdc_activate_fault    => '0',
            o_busy              => busy,
            o_done              => done,
            o_commit_rejected   => rejected,
            o_reject_error      => reject_error,
            o_error             => error_value,
            o_recovery_required => recovery_required,
            o_active_valid      => active_valid,
            o_active            => active_cfg,
            o_candidate         => open,
            o_proc_enable       => proc_enable,
            o_proc_active_valid => proc_active_valid,
            o_proc_active       => proc_active,
            o_tdc_enable        => tdc_enable,
            o_tdc_active_valid  => tdc_active_valid,
            o_tdc_active        => tdc_active,
            o_proc_activate_start => open,
            o_tdc_activate_start => open,
            o_prepare_req       => prepare_req,
            o_activate_req      => activate_req,
            o_release_req       => release_req
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

    p_tdc_clock : process
    begin
        while not stop_clocks loop
            tdc_clk <= '0';
            wait for C_TDC_HALF_PERIOD;
            tdc_clk <= '1';
            wait for C_TDC_HALF_PERIOD;
        end loop;
        wait;
    end process p_tdc_clock;

    p_request_invariants : process (csr_clk)
        variable previous_req : std_logic_vector(2 downto 0) :=
            (others => '0');
    begin
        if rising_edge(csr_clk) then
            if csr_rst_n = '0' then
                previous_req := (others => '0');
            else
                assert activate_req = '0' or prepare_req = '1'
                    report "V2-MGR-101 ACTIVATE without PREPARE"
                    severity failure;
                assert release_req = '0'
                    or (prepare_req = '1' and activate_req = '1')
                    report "V2-MGR-102 RELEASE without PREPARE+ACTIVATE"
                    severity failure;
                if previous_req(2) = '1' and release_req = '0' then
                    assert activate_req = '1' and prepare_req = '1'
                        report "V2-MGR-103 RELEASE clear order"
                        severity failure;
                end if;
                if previous_req(1) = '1' and activate_req = '0' then
                    assert release_req = '0' and prepare_req = '1'
                        report "V2-MGR-104 ACTIVATE clear order"
                        severity failure;
                end if;
                if previous_req(0) = '1' and prepare_req = '0' then
                    assert release_req = '0' and activate_req = '0'
                        report "V2-MGR-105 PREPARE clear order"
                        severity failure;
                end if;
                previous_req := release_req & activate_req & prepare_req;
            end if;
        end if;
    end process p_request_invariants;

    p_test : process
        procedure check(condition : boolean; message_text : string) is
        begin
            assert condition report message_text severity failure;
        end procedure check;

        procedure csr_tick is
        begin
            wait until rising_edge(csr_clk);
            wait for 1 ps;
        end procedure csr_tick;

        procedure pulse_commit is
        begin
            commit <= '1';
            csr_tick;
            commit <= '0';
        end procedure pulse_commit;

        procedure wait_done(
            constant case_name : in string;
            constant expected_error : in lidar_cfg_error_t
        ) is
            variable cycles : natural := 0;
        begin
            loop
                csr_tick;
                cycles := cycles + 1;
                exit when done = '1';
                check(cycles < C_MAX_WAIT_CLKS,
                    case_name & ": completion timeout");
            end loop;
            check(error_value = expected_error,
                case_name & ": unexpected error");
            report case_name & ": PASS in "
                & natural'image(cycles) & " CSR clocks" severity note;
        end procedure wait_done;

        procedure check_active(
            constant case_name : in string;
            constant expected_source : in lidar_runtime_config_t;
            constant expected_version : in natural
        ) is
            variable expected_derived : lidar_derived_config_t;
        begin
            expected_derived := fn_derive_runtime_config(
                C_BUILD_CONFIG, expected_source);
            check(active_valid = '1', case_name & ": manager invalid");
            check(to_integer(active_cfg.version) = expected_version,
                case_name & ": manager version");
            check(active_cfg.source = expected_source
                  and active_cfg.derived = expected_derived,
                case_name & ": manager payload");
            check(proc_active_valid = '1' and tdc_active_valid = '1',
                case_name & ": domain payload invalid");
            check(proc_active = active_cfg and tdc_active = active_cfg,
                case_name & ": domain payload mismatch");
            check(proc_enable = '1' and tdc_enable = '1',
                case_name & ": domains not released");
        end procedure check_active;

        variable cfg            : lidar_runtime_config_t;
        variable committed_cfg  : lidar_runtime_config_t;
        variable preserved      : lidar_active_config_t;
        variable cycles         : natural;
    begin
        wait for 30 ns;
        csr_rst_n  <= '1';
        proc_rst_n <= '1';
        tdc_rst_n  <= '1';
        csr_tick;

        check(active_valid = '0' and proc_enable = '0' and tdc_enable = '0',
            "V2-MGR-001 startup must remain inhibited");

        shadow <= C_DEFAULT_RUNTIME_CONFIG;
        pulse_commit;
        wait_done("V2-MGR-002 first atomic commit", CFG_OK);
        check_active("V2-MGR-002", C_DEFAULT_RUNTIME_CONFIG, 1);

        cfg := C_DEFAULT_RUNTIME_CONFIG;
        cfg.laser.optical_shot_interval_udeg := to_unsigned(100_000, 30);
        committed_cfg := cfg;
        shadow <= cfg;
        pulse_commit;
        while busy /= '1' loop
            csr_tick;
        end loop;
        cfg.laser.optical_shot_interval_udeg := to_unsigned(150_000, 30);
        shadow <= cfg;
        pulse_commit;
        check(rejected = '1' and reject_error = CFG_TRANSACTION_BUSY,
            "V2-MGR-003 BUSY commit rejection");
        wait_done("V2-MGR-003 snapshot and BUSY", CFG_OK);
        check_active("V2-MGR-003", committed_cfg, 2);

        preserved := active_cfg;
        cfg := C_DEFAULT_RUNTIME_CONFIG;
        cfg.motor.cpr := (others => '0');
        shadow <= cfg;
        pulse_commit;
        wait_done("V2-MGR-004 invalid commit", CFG_RUNTIME_CPR);
        check(active_cfg = preserved and proc_active = preserved
              and tdc_active = preserved,
            "V2-MGR-004 invalid commit changed active payload");

        cfg := C_DEFAULT_RUNTIME_CONFIG;
        cfg.tdc.bus_ticks := to_unsigned(8, cfg.tdc.bus_ticks'length);
        shadow <= cfg;
        pulse_commit;
        wait_done("V2-MGR-004B bus tick physical-width guard",
            CFG_RUNTIME_BUS_TIMING);
        check(active_cfg = preserved and proc_active = preserved
              and tdc_active = preserved,
            "V2-MGR-004B invalid bus timing changed active payload");

        cfg := C_DEFAULT_RUNTIME_CONFIG;
        cfg.laser.optical_shot_interval_udeg := to_unsigned(200_000, 30);
        proc_safe <= '0';
        shadow <= cfg;
        pulse_commit;
        cycles := 0;
        while prepare_req /= '1' loop
            csr_tick;
            cycles := cycles + 1;
            check(cycles < C_MAX_WAIT_CLKS,
                "V2-MGR-005 PREPARE was not issued");
        end loop;
        while proc_enable /= '0' loop
            csr_tick;
        end loop;
        for index in 1 to 10 loop
            csr_tick;
        end loop;
        check(busy = '1' and active_cfg = preserved,
            "V2-MGR-005 activated before safe point");
        proc_safe <= '1';
        wait_done("V2-MGR-005 safe-point wait", CFG_OK);
        check_active("V2-MGR-005", cfg, 3);

        preserved := active_cfg;
        tdc_safe <= '0';
        shadow <= C_DEFAULT_RUNTIME_CONFIG;
        pulse_commit;
        wait_done("V2-MGR-006 prepare timeout",
            CFG_TRANSACTION_PREPARE_TIMEOUT);
        tdc_safe <= '1';
        for index in 1 to 10 loop
            csr_tick;
        end loop;
        check(active_cfg = preserved and proc_active = preserved
              and tdc_active = preserved,
            "V2-MGR-006 timeout changed active payload");
        check(proc_enable = '1' and tdc_enable = '1',
            "V2-MGR-006 timeout did not restore prior enable");

        cfg := C_DEFAULT_RUNTIME_CONFIG;
        cfg.laser.optical_shot_interval_udeg := to_unsigned(250_000, 30);
        shadow <= cfg;
        pulse_commit;
        cycles := 0;
        while activate_req /= '1' loop
            csr_tick;
            cycles := cycles + 1;
            check(cycles < C_MAX_WAIT_CLKS,
                "V2-MGR-007 ACTIVATE was not issued");
        end loop;
        tdc_rst_n <= '0';
        wait for 5 * C_TDC_HALF_PERIOD;
        tdc_rst_n <= '1';
        wait_done("V2-MGR-007 TDC reset replay", CFG_OK);
        check_active("V2-MGR-007", cfg, 4);

        proc_safe <= '0';
        shadow <= C_DEFAULT_RUNTIME_CONFIG;
        pulse_commit;
        while prepare_req /= '1' loop
            csr_tick;
        end loop;
        csr_rst_n  <= '0';
        proc_rst_n <= '0';
        tdc_rst_n  <= '0';
        wait for 30 ns;
        proc_safe <= '1';
        csr_rst_n  <= '1';
        proc_rst_n <= '1';
        tdc_rst_n  <= '1';
        csr_tick;
        check(active_valid = '0' and proc_active_valid = '0'
              and tdc_active_valid = '0'
              and proc_enable = '0' and tdc_enable = '0',
            "V2-MGR-008 coordinated reset state");
        shadow <= C_DEFAULT_RUNTIME_CONFIG;
        pulse_commit;
        wait_done("V2-MGR-009 restart after reset", CFG_OK);
        check_active("V2-MGR-009", C_DEFAULT_RUNTIME_CONFIG, 1);

        preserved := active_cfg;
        cfg := C_DEFAULT_RUNTIME_CONFIG;
        cfg.laser.optical_shot_interval_udeg := to_unsigned(300_000, 30);
        shadow <= cfg;
        pulse_commit;
        cycles := 0;
        while activate_req /= '1' loop
            csr_tick;
            cycles := cycles + 1;
            check(cycles < C_MAX_WAIT_CLKS,
                "V2-MGR-010 ACTIVATE timeout setup");
        end loop;
        tdc_rst_n <= '0';
        wait_done("V2-MGR-010 activate timeout",
            CFG_TRANSACTION_ACTIVATE_TIMEOUT);
        check(recovery_required = '1' and active_cfg = preserved,
            "V2-MGR-010 activation failure was not quarantined");

        csr_rst_n  <= '0';
        proc_rst_n <= '0';
        wait for 30 ns;
        csr_rst_n  <= '1';
        proc_rst_n <= '1';
        tdc_rst_n  <= '1';
        csr_tick;
        check(recovery_required = '0' and active_valid = '0'
              and proc_enable = '0' and tdc_enable = '0',
            "V2-MGR-011 activate-timeout recovery reset");
        shadow <= C_DEFAULT_RUNTIME_CONFIG;
        pulse_commit;
        wait_done("V2-MGR-012 restart after activate timeout", CFG_OK);
        check_active("V2-MGR-012", C_DEFAULT_RUNTIME_CONFIG, 1);

        preserved := active_cfg;
        cfg := C_DEFAULT_RUNTIME_CONFIG;
        cfg.laser.optical_shot_interval_udeg := to_unsigned(350_000, 30);
        shadow <= cfg;
        pulse_commit;
        cycles := 0;
        while release_req /= '1' loop
            csr_tick;
            cycles := cycles + 1;
            check(cycles < C_MAX_WAIT_CLKS,
                "V2-MGR-013 RELEASE timeout setup");
        end loop;
        tdc_rst_n <= '0';
        wait_done("V2-MGR-013 release timeout",
            CFG_TRANSACTION_RELEASE_TIMEOUT);
        check(recovery_required = '1' and active_cfg = preserved
              and tdc_enable = '0',
            "V2-MGR-013 partial release was not quarantined");

        csr_rst_n  <= '0';
        proc_rst_n <= '0';
        wait for 30 ns;
        csr_rst_n  <= '1';
        proc_rst_n <= '1';
        tdc_rst_n  <= '1';
        csr_tick;
        shadow <= C_DEFAULT_RUNTIME_CONFIG;
        pulse_commit;
        wait_done("V2-MGR-014 restart after release timeout", CFG_OK);
        check_active("V2-MGR-014", C_DEFAULT_RUNTIME_CONFIG, 1);

        check(recovery_required = '0',
            "V2-MGR-015 unexpected recovery lock");
        report "LIDAR_V2_CONFIG_SUBSYSTEM_PASS proc_mhz="
            & positive'image(G_PROC_CLK_MHZ) & " tdc_mhz="
            & positive'image(G_TDC_CLK_MHZ) severity note;
        stop_clocks <= true;
        stop;
        wait;
    end process p_test;

end architecture sim;
