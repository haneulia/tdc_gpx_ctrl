library ieee;
use ieee.std_logic_1164.all;

library std;
use std.env.all;

-- L0 VDMA profile bridge 검증 목적:
--   1. 150 MHz Processing -> 100 MHz CSR 비동기 전송의 49-bit 원자성
--   2. PS ACK 지연 중 profile 유지와 단 한 번의 ready 반환
--   3. 이전 ACK가 High로 남은 경우의 오승인 방지
--   4. 서로 다른 연속 profile의 순서 및 데이터 보존
entity tb_l0_vdma_profile_bridge is
end entity tb_l0_vdma_profile_bridge;

architecture sim of tb_l0_vdma_profile_bridge is

    constant C_PROC_PERIOD : time := 20 ns / 3;
    constant C_CSR_PERIOD  : time := 10 ns;

    signal proc_clk : std_logic := '0';
    signal proc_rst_n : std_logic := '0';
    signal proc_valid : std_logic := '0';
    signal proc_enable : std_logic := '0';
    signal proc_hsize : std_logic_vector(15 downto 0) := (others => '0');
    signal proc_vsize : std_logic_vector(15 downto 0) := (others => '0');
    signal proc_stride : std_logic_vector(15 downto 0) := (others => '0');
    signal proc_ready : std_logic;

    signal csr_clk : std_logic := '0';
    signal csr_rst_n : std_logic := '0';
    signal csr_valid : std_logic;
    signal csr_enable : std_logic;
    signal csr_hsize : std_logic_vector(15 downto 0);
    signal csr_vsize : std_logic_vector(15 downto 0);
    signal csr_stride : std_logic_vector(15 downto 0);
    signal csr_ack : std_logic := '0';

begin

    proc_clk <= not proc_clk after C_PROC_PERIOD / 2;
    csr_clk <= not csr_clk after C_CSR_PERIOD / 2;

    u_dut : entity work.l0_vdma_profile_bridge
        port map (
            i_proc_clk => proc_clk,
            i_proc_rst_n => proc_rst_n,
            i_proc_cfg_valid => proc_valid,
            i_proc_cfg_enable => proc_enable,
            i_proc_hsize_bytes => proc_hsize,
            i_proc_vsize_lines => proc_vsize,
            i_proc_stride_bytes => proc_stride,
            o_proc_cfg_ready => proc_ready,
            i_csr_clk => csr_clk,
            i_csr_rst_n => csr_rst_n,
            o_csr_cfg_valid => csr_valid,
            o_csr_cfg_enable => csr_enable,
            o_csr_hsize_bytes => csr_hsize,
            o_csr_vsize_lines => csr_vsize,
            o_csr_stride_bytes => csr_stride,
            i_csr_cfg_ack => csr_ack
        );

    p_stimulus : process
        procedure wait_proc_ready(constant scenario : string) is
        begin
            for cycle in 0 to 80 loop
                wait until rising_edge(proc_clk);
                if proc_ready = '1' then
                    return;
                end if;
            end loop;
            assert false
                report scenario & ": Processing ready timeout"
                severity failure;
        end procedure;

        procedure wait_csr_valid(constant scenario : string) is
        begin
            for cycle in 0 to 80 loop
                wait until rising_edge(csr_clk);
                if csr_valid = '1' then
                    return;
                end if;
            end loop;
            assert false
                report scenario & ": CSR valid timeout"
                severity failure;
        end procedure;
    begin
        wait for 80 ns;
        wait until rising_edge(proc_clk);
        proc_rst_n <= '1';
        wait until rising_edge(csr_clk);
        csr_rst_n <= '1';
        wait for 100 ns;

        -- Scenario A: ACK가 늦어져도 profile 전체가 그대로 유지된다.
        proc_enable <= '1';
        proc_hsize <= x"0120";
        proc_vsize <= x"0709";
        proc_stride <= x"0200";
        proc_valid <= '1';
        wait_csr_valid("A");
        assert csr_enable = '1' and csr_hsize = x"0120" and
               csr_vsize = x"0709" and csr_stride = x"0200"
            report "A: torn or incorrect profile"
            severity failure;
        for cycle in 0 to 4 loop
            wait until rising_edge(csr_clk);
            assert csr_valid = '1' and csr_hsize = x"0120" and
                   csr_vsize = x"0709" and csr_stride = x"0200"
                report "A: profile changed before ACK"
                severity failure;
        end loop;
        csr_ack <= '1';
        wait until rising_edge(csr_clk);
        csr_ack <= '0';
        wait_proc_ready("A");
        proc_valid <= '0';
        wait until rising_edge(proc_clk);
        assert proc_ready = '0'
            report "A: ready must be a one-clock pulse"
            severity failure;

        for cycle in 0 to 40 loop
            wait until rising_edge(csr_clk);
            exit when csr_valid = '0';
        end loop;
        assert csr_valid = '0'
            report "A: destination request did not release"
            severity failure;

        -- Scenario B: ACK가 요청 전부터 High이면 자동 승인하지 않는다.
        csr_ack <= '1';
        wait until rising_edge(csr_clk);
        proc_enable <= '0';
        proc_hsize <= x"0040";
        proc_vsize <= x"0002";
        proc_stride <= x"0080";
        proc_valid <= '1';
        wait_csr_valid("B");
        for cycle in 0 to 7 loop
            wait until rising_edge(proc_clk);
            assert proc_ready = '0'
                report "B: stale-high ACK incorrectly accepted"
                severity failure;
        end loop;
        csr_ack <= '0';
        wait until rising_edge(csr_clk);
        wait until rising_edge(csr_clk);
        csr_ack <= '1';
        wait until rising_edge(csr_clk);
        csr_ack <= '0';
        wait_proc_ready("B");
        proc_valid <= '0';

        -- Scenario C: 다음 profile은 이전 snapshot과 섞이지 않는다.
        for cycle in 0 to 40 loop
            wait until rising_edge(proc_clk);
            exit when proc_ready = '0' and csr_valid = '0';
        end loop;
        proc_enable <= '1';
        proc_hsize <= x"0300";
        proc_vsize <= x"1234";
        proc_stride <= x"0400";
        proc_valid <= '1';
        wait_csr_valid("C");
        assert csr_enable = '1' and csr_hsize = x"0300" and
               csr_vsize = x"1234" and csr_stride = x"0400"
            report "C: back-to-back profile was mixed"
            severity failure;
        csr_ack <= '1';
        wait until rising_edge(csr_clk);
        csr_ack <= '0';
        wait_proc_ready("C");
        proc_valid <= '0';

        report "L0_VDMA_PROFILE_BRIDGE_150_TO_100_PASS" severity note;
        stop;
        wait;
    end process p_stimulus;

end architecture sim;
