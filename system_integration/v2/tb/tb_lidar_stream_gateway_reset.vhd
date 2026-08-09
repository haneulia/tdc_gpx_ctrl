-- ============================================================================
-- 테스트 자산 목적: 비동기 Stream FIFO의 Source/Destination Reset 폐기 계약을 검증한다.
-- 핵심 검증 계약: Reset 전 payload 폐기, ready/valid 차단, 복구 후 새 값 일치다.
-- 관련 RTL: system_integration/v2/rtl/common/lidar_stream_gateway.vhd
-- 실행 회귀: scripts/run_v2_stream_gateway_reset.ps1
-- 유지보수 주의: Reset 동기화 단계, 유지 clock 또는 FIFO 종류를 바꾸면
--                  두 Reset 방향과 stale payload 부재를 함께 재검증한다.
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.env.all;

use work.lidar_build_pkg.all;

entity tb_lidar_stream_gateway_reset is
end entity tb_lidar_stream_gateway_reset;

architecture sim of tb_lidar_stream_gateway_reset is

    constant C_SOURCE_PERIOD : time := 1 us / 200;
    constant C_DESTINATION_PERIOD : time := 1 us / 150;

    constant C_STALE_DESTINATION_RESET : std_logic_vector(31 downto 0) :=
        x"A55A0001";
    constant C_AFTER_DESTINATION_RESET : std_logic_vector(31 downto 0) :=
        x"A55A0002";
    constant C_STALE_SOURCE_RESET : std_logic_vector(31 downto 0) :=
        x"5AA50001";
    constant C_AFTER_SOURCE_RESET : std_logic_vector(31 downto 0) :=
        x"5AA50002";

    signal source_clk : std_logic := '0';
    signal destination_clk : std_logic := '0';
    signal source_rst_n : std_logic := '0';
    signal destination_rst_n : std_logic := '0';
    signal source_valid : std_logic := '0';
    signal source_ready : std_logic;
    signal source_data : std_logic_vector(31 downto 0) := (others => '0');
    signal destination_valid : std_logic;
    signal destination_ready : std_logic := '0';
    signal destination_data : std_logic_vector(31 downto 0);
    signal source_reset_busy : std_logic;
    signal destination_reset_busy : std_logic;
    signal reset_busy : std_logic;
    signal done : boolean := false;

begin

    source_clk <= not source_clk after C_SOURCE_PERIOD / 2 when not done;
    destination_clk <= not destination_clk after C_DESTINATION_PERIOD / 2
        when not done;

    u_dut : entity work.lidar_stream_gateway
        generic map (
            G_WIDTH      => 32,
            G_FIFO_DEPTH => 16,
            G_CLOCK_MODE => STREAM_CLOCK_ASYNC
        )
        port map (
            i_source_clk       => source_clk,
            i_source_rst_n     => source_rst_n,
            i_source_valid     => source_valid,
            o_source_ready     => source_ready,
            i_source_data      => source_data,
            i_destination_clk  => destination_clk,
            i_destination_rst_n => destination_rst_n,
            o_destination_valid => destination_valid,
            i_destination_ready => destination_ready,
            o_destination_data  => destination_data,
            o_source_reset_busy => source_reset_busy,
            o_destination_reset_busy => destination_reset_busy,
            o_reset_busy => reset_busy
        );

    p_stimulus : process
        procedure wait_for_reset_release is
            variable source_cycles : natural := 0;
        begin
            while source_reset_busy /= '0' loop
                wait until rising_edge(source_clk);
                source_cycles := source_cycles + 1;
                assert source_cycles < 128
                    report "V2-STREAM-RESET source reset did not release"
                    severity failure;
            end loop;

            source_cycles := 0;
            while destination_reset_busy /= '0' loop
                wait until rising_edge(destination_clk);
                source_cycles := source_cycles + 1;
                assert source_cycles < 128
                    report "V2-STREAM-RESET destination reset did not release"
                    severity failure;
            end loop;
        end procedure wait_for_reset_release;

        procedure push_word(
            constant value : std_logic_vector(31 downto 0)) is
            variable source_cycles : natural := 0;
        begin
            source_data <= value;
            source_valid <= '1';
            loop
                wait until rising_edge(source_clk);
                exit when source_ready = '1';
                source_cycles := source_cycles + 1;
                assert source_cycles < 128
                    report "V2-STREAM-RESET source handshake timeout"
                    severity failure;
            end loop;
            source_valid <= '0';
        end procedure push_word;

        procedure wait_for_buffered_word is
            variable destination_cycles : natural := 0;
        begin
            while destination_valid /= '1' loop
                wait until rising_edge(destination_clk);
                destination_cycles := destination_cycles + 1;
                assert destination_cycles < 128
                    report "V2-STREAM-RESET buffered word did not arrive"
                    severity failure;
            end loop;
        end procedure wait_for_buffered_word;

        procedure assert_no_stale_word is
        begin
            destination_ready <= '1';
            for cycle in 1 to 8 loop
                wait until rising_edge(destination_clk);
                assert destination_valid = '0'
                    report "V2-STREAM-RESET stale word survived FIFO reset"
                    severity failure;
            end loop;
            destination_ready <= '0';
        end procedure assert_no_stale_word;

        procedure receive_word(
            constant expected : std_logic_vector(31 downto 0)) is
            variable destination_cycles : natural := 0;
        begin
            destination_ready <= '1';
            loop
                wait until rising_edge(destination_clk);
                if destination_valid = '1' then
                    assert destination_data = expected
                        report "V2-STREAM-RESET post-reset payload mismatch"
                        severity failure;
                    exit;
                end if;
                destination_cycles := destination_cycles + 1;
                assert destination_cycles < 128
                    report "V2-STREAM-RESET destination handshake timeout"
                    severity failure;
            end loop;
            destination_ready <= '0';
        end procedure receive_word;

    begin
        source_rst_n <= '0';
        destination_rst_n <= '0';
        wait for 120 ns;
        source_rst_n <= '1';
        destination_rst_n <= '1';
        wait_for_reset_release;

        -- 목적지 리셋: FIFO에 보류된 첫 값을 폐기하고 새 값만 전달한다.
        push_word(C_STALE_DESTINATION_RESET);
        wait_for_buffered_word;
        destination_rst_n <= '0';
        wait for 1 ns;
        assert destination_valid = '0'
            report "V2-STREAM-RESET destination reset did not gate valid"
            severity failure;
        for cycle in 1 to 12 loop
            wait until rising_edge(source_clk);
        end loop;
        destination_rst_n <= '1';
        wait_for_reset_release;
        assert_no_stale_word;
        push_word(C_AFTER_DESTINATION_RESET);
        receive_word(C_AFTER_DESTINATION_RESET);

        -- 소스 리셋: 목적지를 정지시킨 상태에서 보류된 값을 폐기한다.
        push_word(C_STALE_SOURCE_RESET);
        wait_for_buffered_word;
        source_rst_n <= '0';
        for cycle in 1 to 12 loop
            wait until rising_edge(source_clk);
        end loop;
        assert source_ready = '0'
            report "V2-STREAM-RESET source ready leaked during reset"
            severity failure;
        source_rst_n <= '1';
        wait_for_reset_release;
        assert_no_stale_word;
        push_word(C_AFTER_SOURCE_RESET);
        receive_word(C_AFTER_SOURCE_RESET);

        assert reset_busy = '0'
            report "V2-STREAM-RESET aggregate reset busy remained set"
            severity failure;
        report "LIDAR_V2_STREAM_GATEWAY_RESET_200_TO_150_PASS"
            severity note;
        done <= true;
        stop;
        wait;
    end process p_stimulus;

    p_watchdog : process
    begin
        wait for 20 us;
        assert done
            report "V2-STREAM-RESET watchdog timeout"
            severity failure;
        wait;
    end process p_watchdog;

end architecture sim;
