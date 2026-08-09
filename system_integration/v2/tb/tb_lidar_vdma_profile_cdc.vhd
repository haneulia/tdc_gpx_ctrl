-- ==========================================================================
-- Test purpose: verify the coherent VDMA profile transfer used inside the
-- integrated IP. The test covers delayed ACK, stale-high ACK rejection, a
-- one-cycle processing-domain ready pulse, and consecutive profile snapshots.
-- Related RTL: lidar_vdma_profile_cdc.
-- Maintenance rule: if the profile payload changes, update every snapshot
-- comparison here so torn-field CDC regressions remain observable.
-- ==========================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library std;
use std.env.all;

entity tb_lidar_vdma_profile_cdc is
    generic (
        G_PROC_CLK_MHZ : positive := 150;
        G_TDC_CLK_MHZ  : positive := 200
    );
end entity tb_lidar_vdma_profile_cdc;

architecture sim of tb_lidar_vdma_profile_cdc is

    -- K03의 두 극성 비동기 조합을 그대로 받아 CDC 계약을 양방향으로 검증한다.
    constant C_PROC_PERIOD : time := 1 us / G_PROC_CLK_MHZ;
    constant C_CSR_PERIOD  : time := 1 us / G_TDC_CLK_MHZ;

    signal proc_clk : std_logic := '0';
    signal proc_rst_n : std_logic := '0';
    signal proc_valid : std_logic := '0';
    signal proc_enable : std_logic := '0';
    signal proc_hsize : unsigned(15 downto 0) := (others => '0');
    signal proc_vsize : unsigned(15 downto 0) := (others => '0');
    signal proc_stride : unsigned(15 downto 0) := (others => '0');
    signal proc_ready : std_logic;

    signal csr_clk : std_logic := '0';
    signal csr_rst_n : std_logic := '0';
    signal csr_valid : std_logic;
    signal csr_enable : std_logic;
    signal csr_hsize : unsigned(15 downto 0);
    signal csr_vsize : unsigned(15 downto 0);
    signal csr_stride : unsigned(15 downto 0);
    signal csr_ack : std_logic := '0';

begin

    proc_clk <= not proc_clk after C_PROC_PERIOD / 2;
    csr_clk <= not csr_clk after C_CSR_PERIOD / 2;

    u_dut : entity work.lidar_vdma_profile_cdc
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
                report scenario & ": processing ready timeout"
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

        -- A: a delayed ACK must leave the complete snapshot stable.
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

        -- B: ACK already high before a new request must not be accepted.
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

        -- C: a later snapshot must not mix fields from its predecessor.
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
            report "C: consecutive profile was mixed"
            severity failure;
        csr_ack <= '1';
        wait until rising_edge(csr_clk);
        csr_ack <= '0';
        wait_proc_ready("C");
        proc_valid <= '0';

        report "LIDAR_V2_VDMA_PROFILE_CDC_PASS proc_mhz=" &
               integer'image(G_PROC_CLK_MHZ) & " tdc_mhz=" &
               integer'image(G_TDC_CLK_MHZ)
            severity note;
        stop;
        wait;
    end process p_stimulus;

end architecture sim;
