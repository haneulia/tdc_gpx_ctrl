-- ============================================================================
-- 테스트 자산 목적: GPX 설정 image의 물리 programming과 활성화 barrier를 검증한다.
-- 핵심 검증 계약: 모든 present Chip ACK 전 enable 금지, RELEASE 후 단일 버전 적용이다.
-- 관련 RTL: lidar_gpx_config_activation.
-- 실행 회귀: scripts/run_v2_unified_csr.ps1
-- 유지보수 주의: Chip mask/ACK 정책 변경 시 조기 ACK와 fault 경로를 반드시 유지한다.
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.env.all;

use work.lidar_build_pkg.all;
use work.lidar_config_types_pkg.all;
use work.lidar_config_reference_pkg.all;
use work.lidar_gpx_pkg.all;
use work.lidar_gpx_image_pkg.all;

entity tb_lidar_gpx_config_activation is
end entity tb_lidar_gpx_config_activation;

architecture sim of tb_lidar_gpx_config_activation is

    constant C_CSR_PERIOD : time := 10 ns;
    constant C_TDC_PERIOD : time := 5 ns;

    function fn_candidate return lidar_active_config_t is
        variable result : lidar_active_config_t;
    begin
        result.version := to_unsigned(1, 16);
        result.source := C_DEFAULT_RUNTIME_CONFIG;
        result.derived := fn_derive_runtime_config(
            C_DEFAULT_BUILD_CONFIG, result.source);
        return result;
    end function fn_candidate;

    signal csr_clk : std_logic := '0';
    signal tdc_clk : std_logic := '0';
    signal rst_n : std_logic := '0';
    signal stop_clocks : boolean := false;

    signal prepare_req : std_logic := '0';
    signal activate_req : std_logic := '0';
    signal release_req : std_logic := '0';
    signal candidate : lidar_active_config_t := fn_candidate;
    signal candidate_image : gpx_register_image_t :=
        C_GPX_REGISTER_IMAGE_DEFAULT;

    signal prepare_ack : std_logic;
    signal activate_ack : std_logic;
    signal release_ack : std_logic;
    signal activate_start : std_logic;
    signal gateway_fault : std_logic;
    signal domain_enable : std_logic;
    signal active_valid : std_logic;
    signal active_config : lidar_active_config_t;

    signal apply_ready : std_logic := '0';
    signal apply_done : std_logic := '0';
    signal apply_fault : std_logic := '0';
    signal applied_image : gpx_register_image_t;
    signal apply_pulse : std_logic;
    signal activate_complete : std_logic;
    signal activate_fault : std_logic;

begin

    csr_clk <= not csr_clk after C_CSR_PERIOD / 2 when not stop_clocks;
    tdc_clk <= not tdc_clk after C_TDC_PERIOD / 2 when not stop_clocks;

    u_gateway : entity work.lidar_config_gateway
        generic map (
            G_DEFER_ACTIVATE_ACK => true
        )
        port map (
            i_csr_clk           => csr_clk,
            i_csr_rst_n         => rst_n,
            i_domain_clk        => tdc_clk,
            i_domain_rst_n      => rst_n,
            i_prepare_req       => prepare_req,
            i_activate_req      => activate_req,
            i_release_req       => release_req,
            i_candidate         => candidate,
            i_safe_to_prepare   => '1',
            i_activate_complete => activate_complete,
            i_activate_fault    => activate_fault,
            o_prepare_ack       => prepare_ack,
            o_activate_ack      => activate_ack,
            o_release_ack       => release_ack,
            o_activate_start    => activate_start,
            o_fault_csr         => gateway_fault,
            o_domain_enable     => domain_enable,
            o_active_valid      => active_valid,
            o_active            => active_config
        );

    u_activation : entity work.lidar_gpx_config_activation
        port map (
            i_clk               => tdc_clk,
            i_rst_n             => rst_n,
            i_activate_start    => activate_start,
            i_candidate_image   => candidate_image,
            i_apply_ready       => apply_ready,
            i_apply_done        => apply_done,
            i_apply_fault       => apply_fault,
            o_register_image    => applied_image,
            o_apply             => apply_pulse,
            o_activate_complete => activate_complete,
            o_activate_fault    => activate_fault,
            o_busy              => open
        );

    p_test : process
        procedure wait_csr_high(
            signal value : in std_logic;
            constant message_text : in string
        ) is
        begin
            for timeout in 0 to 100 loop
                wait until rising_edge(csr_clk);
                wait for 1 ps;
                if value = '1' then
                    return;
                end if;
            end loop;
            assert false report message_text severity failure;
        end procedure wait_csr_high;

        procedure wait_csr_low(
            signal value : in std_logic;
            constant message_text : in string
        ) is
        begin
            for timeout in 0 to 100 loop
                wait until rising_edge(csr_clk);
                wait for 1 ps;
                if value = '0' then
                    return;
                end if;
            end loop;
            assert false report message_text severity failure;
        end procedure wait_csr_low;

        procedure wait_tdc_high(
            signal value : in std_logic;
            constant message_text : in string
        ) is
        begin
            for timeout in 0 to 100 loop
                wait until rising_edge(tdc_clk);
                wait for 1 ps;
                if value = '1' then
                    return;
                end if;
            end loop;
            assert false report message_text severity failure;
        end procedure wait_tdc_high;
    begin
        candidate_image(5) <= x"00123456";
        wait for 40 ns;
        rst_n <= '1';

        prepare_req <= '1';
        wait_csr_high(prepare_ack,
            "V2-GPX-ACT-TB prepare acknowledgement timeout");
        activate_req <= '1';

        -- Backpressure at the physical programming owner must not create an
        -- early ACTIVATE acknowledgement.
        for index in 1 to 8 loop
            wait until rising_edge(tdc_clk);
            wait for 1 ps;
            assert apply_pulse = '0' and activate_ack = '0'
                report "V2-GPX-ACT-TB activation escaped before ready"
                severity failure;
        end loop;
        apply_ready <= '1';
        wait_tdc_high(apply_pulse,
            "V2-GPX-ACT-TB apply pulse timeout");
        assert applied_image(5) = x"00123456" and active_valid = '1'
            report "V2-GPX-ACT-TB candidate image/config capture mismatch"
            severity failure;

        for index in 1 to 12 loop
            wait until rising_edge(tdc_clk);
            wait for 1 ps;
            assert activate_ack = '0' and domain_enable = '0'
                report "V2-GPX-ACT-TB early physical programming ACK"
                severity failure;
        end loop;
        apply_done <= '1';
        wait until rising_edge(tdc_clk);
        apply_done <= '0';
        wait_csr_high(activate_ack,
            "V2-GPX-ACT-TB deferred activation ACK timeout");
        assert domain_enable = '0'
            report "V2-GPX-ACT-TB domain enabled before RELEASE"
            severity failure;

        release_req <= '1';
        wait_csr_high(release_ack,
            "V2-GPX-ACT-TB release acknowledgement timeout");
        assert domain_enable = '1'
            report "V2-GPX-ACT-TB domain did not enable after RELEASE"
            severity failure;

        release_req <= '0';
        wait_csr_low(release_ack,
            "V2-GPX-ACT-TB release clear timeout");
        activate_req <= '0';
        wait_csr_low(activate_ack,
            "V2-GPX-ACT-TB activate clear timeout");
        prepare_req <= '0';
        wait_csr_low(prepare_ack,
            "V2-GPX-ACT-TB prepare clear timeout");

        -- A physical programming fault blocks ACK and crosses back to CSR.
        candidate.version <= to_unsigned(2, 16);
        prepare_req <= '1';
        wait_csr_high(prepare_ack,
            "V2-GPX-ACT-TB fault-case prepare timeout");
        activate_req <= '1';
        wait_tdc_high(apply_pulse,
            "V2-GPX-ACT-TB fault-case apply timeout");
        apply_fault <= '1';
        wait until rising_edge(tdc_clk);
        apply_fault <= '0';
        wait_csr_high(gateway_fault,
            "V2-GPX-ACT-TB programming fault CDC timeout");
        assert activate_ack = '0' and domain_enable = '0'
            report "V2-GPX-ACT-TB faulted activation was acknowledged"
            severity failure;

        report "LIDAR_V2_GPX_CONFIG_ACTIVATION_PASS" severity note;
        stop_clocks <= true;
        stop;
        wait;
    end process p_test;

end architecture sim;
