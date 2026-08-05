library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library std;
use std.env.all;

use work.lidar_build_pkg.all;
use work.lidar_config_types_pkg.all;
use work.lidar_config_reference_pkg.all;

entity tb_lidar_config_gateway is
end entity tb_lidar_config_gateway;

architecture sim of tb_lidar_config_gateway is

    function fn_candidate(version_value : natural)
        return lidar_active_config_t is
        variable result : lidar_active_config_t;
    begin
        result.version := to_unsigned(version_value, 16);
        result.source := C_DEFAULT_RUNTIME_CONFIG;
        result.derived := fn_derive_runtime_config(
            C_DEFAULT_BUILD_CONFIG, C_DEFAULT_RUNTIME_CONFIG);
        return result;
    end function fn_candidate;

    signal csr_clk       : std_logic := '0';
    signal domain_clk    : std_logic := '0';
    signal csr_rst_n     : std_logic := '0';
    signal domain_rst_n  : std_logic := '0';
    signal stop_clocks   : boolean := false;
    signal prepare_req   : std_logic := '0';
    signal activate_req  : std_logic := '0';
    signal release_req   : std_logic := '0';
    signal candidate     : lidar_active_config_t := fn_candidate(1);
    signal safe_to_prepare : std_logic := '1';
    signal prepare_ack   : std_logic;
    signal activate_ack  : std_logic;
    signal release_ack   : std_logic;
    signal fault_csr     : std_logic;
    signal domain_enable : std_logic;
    signal active_valid  : std_logic;
    signal active_cfg    : lidar_active_config_t;

begin

    u_dut : entity work.lidar_config_gateway
        port map (
            i_csr_clk         => csr_clk,
            i_csr_rst_n       => csr_rst_n,
            i_domain_clk      => domain_clk,
            i_domain_rst_n    => domain_rst_n,
            i_prepare_req     => prepare_req,
            i_activate_req    => activate_req,
            i_release_req     => release_req,
            i_candidate       => candidate,
            i_safe_to_prepare => safe_to_prepare,
            o_prepare_ack     => prepare_ack,
            o_activate_ack    => activate_ack,
            o_release_ack     => release_ack,
            o_activate_start  => open,
            o_fault_csr       => fault_csr,
            o_domain_enable   => domain_enable,
            o_active_valid    => active_valid,
            o_active          => active_cfg
        );

    p_csr_clock : process
    begin
        while not stop_clocks loop
            csr_clk <= '0';
            wait for 5 ns;
            csr_clk <= '1';
            wait for 5 ns;
        end loop;
        wait;
    end process p_csr_clock;

    p_domain_clock : process
    begin
        while not stop_clocks loop
            domain_clk <= '0';
            wait for 3333 ps;
            domain_clk <= '1';
            wait for 3333 ps;
        end loop;
        wait;
    end process p_domain_clock;

    p_test : process
        procedure check(condition : boolean; message_text : string) is
        begin
            assert condition report message_text severity failure;
        end procedure check;

        procedure wait_csr_level(
            signal observed : in std_logic;
            constant expected : in std_logic;
            constant message_text : in string
        ) is
            variable cycles : natural := 0;
        begin
            while observed /= expected loop
                wait until rising_edge(csr_clk);
                wait for 1 ps;
                cycles := cycles + 1;
                check(cycles < 50, message_text);
            end loop;
        end procedure wait_csr_level;
    begin
        wait for 30 ns;
        csr_rst_n <= '1';
        domain_rst_n <= '1';

        prepare_req <= '1';
        wait_csr_level(prepare_ack, '1',
            "V2-GW-001 prepare timeout");
        check(domain_enable = '0',
            "V2-GW-001 prepare did not close gate");
        activate_req <= '1';
        wait_csr_level(activate_ack, '1',
            "V2-GW-001 activate timeout");
        release_req <= '1';
        wait_csr_level(release_ack, '1',
            "V2-GW-001 release timeout");
        check(active_valid = '1', "V2-GW-001 active-valid");
        check(active_cfg = candidate, "V2-GW-001 active payload");
        check(domain_enable = '1', "V2-GW-001 domain release");
        release_req <= '0';
        wait_csr_level(release_ack, '0',
            "V2-GW-001 release-clear timeout");
        activate_req <= '0';
        wait_csr_level(activate_ack, '0',
            "V2-GW-001 activate-clear timeout");
        prepare_req <= '0';
        wait_csr_level(prepare_ack, '0',
            "V2-GW-001 prepare-clear timeout");

        candidate <= fn_candidate(2);
        safe_to_prepare <= '0';
        prepare_req <= '1';
        wait until domain_enable = '0';
        wait for 30 ns;
        check(prepare_ack = '0' and active_cfg.version = to_unsigned(1, 16),
            "V2-GW-002 unsafe prepare changed active payload");
        prepare_req <= '0';
        wait until domain_enable = '1';
        safe_to_prepare <= '1';
        report "V2-GW-002 prepare abort: PASS" severity note;

        csr_rst_n <= '0';
        domain_rst_n <= '0';
        wait for 20 ns;
        csr_rst_n <= '1';
        domain_rst_n <= '1';
        activate_req <= '1';
        wait_csr_level(fault_csr, '1',
            "V2-GW-003 protocol fault timeout");
        check(active_valid = '0' and domain_enable = '0',
            "V2-GW-003 illegal activation escaped quarantine");
        report "LIDAR_V2_CONFIG_GATEWAY_PASS" severity note;

        stop_clocks <= true;
        stop;
        wait;
    end process p_test;

end architecture sim;
