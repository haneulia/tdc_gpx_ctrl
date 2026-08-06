library ieee;
use ieee.std_logic_1164.all;

library std;
use std.env.all;

entity tb_lidar_system_command_cdc is
    generic (
        G_PROC_CLK_MHZ : positive := 150;
        G_TDC_CLK_MHZ  : positive := 200
    );
end entity tb_lidar_system_command_cdc;

architecture sim of tb_lidar_system_command_cdc is

    constant C_SOURCE_PERIOD : time := 10 ns;
    constant C_PROC_PERIOD : time := 1 us / G_PROC_CLK_MHZ;
    constant C_TDC_PERIOD : time := 1 us / G_TDC_CLK_MHZ;

    signal source_clk : std_logic := '0';
    signal proc_clk : std_logic := '0';
    signal tdc_clk : std_logic := '0';
    signal source_rst_n : std_logic := '0';
    signal proc_rst_n : std_logic := '0';
    signal tdc_rst_n : std_logic := '0';

    signal clear_status : std_logic := '0';
    signal soft_reset : std_logic := '0';
    signal source_ready : std_logic;
    signal source_busy : std_logic;
    signal source_rejected : std_logic;
    signal proc_clear : std_logic;
    signal proc_reset : std_logic;
    signal tdc_clear : std_logic;
    signal tdc_reset : std_logic;

    signal proc_clear_count : natural := 0;
    signal proc_reset_count : natural := 0;
    signal tdc_clear_count : natural := 0;
    signal tdc_reset_count : natural := 0;
    signal rejected_count : natural := 0;

begin

    source_clk <= not source_clk after C_SOURCE_PERIOD / 2;
    proc_clk <= not proc_clk after C_PROC_PERIOD / 2;
    tdc_clk <= not tdc_clk after C_TDC_PERIOD / 2;

    u_dut : entity work.lidar_system_command_cdc
        port map (
            i_source_clk => source_clk,
            i_source_rst_n => source_rst_n,
            i_clear_status => clear_status,
            i_soft_reset => soft_reset,
            o_source_ready => source_ready,
            o_source_busy => source_busy,
            o_source_rejected => source_rejected,
            i_proc_clk => proc_clk,
            i_proc_rst_n => proc_rst_n,
            o_proc_clear_status => proc_clear,
            o_proc_soft_reset => proc_reset,
            i_tdc_clk => tdc_clk,
            i_tdc_rst_n => tdc_rst_n,
            o_tdc_clear_status => tdc_clear,
            o_tdc_soft_reset => tdc_reset
        );

    p_proc_monitor : process (proc_clk)
        variable previous_clear : std_logic := '0';
        variable previous_reset : std_logic := '0';
    begin
        if rising_edge(proc_clk) then
            if proc_rst_n = '0' then
                proc_clear_count <= 0;
                proc_reset_count <= 0;
                previous_clear := '0';
                previous_reset := '0';
            else
                assert not (proc_clear = '1' and previous_clear = '1')
                    report "V2-K0-CMD-TB Processing CLEAR wider than one clock"
                    severity failure;
                assert not (proc_reset = '1' and previous_reset = '1')
                    report "V2-K0-CMD-TB Processing RESET wider than one clock"
                    severity failure;
                if proc_clear = '1' then
                    proc_clear_count <= proc_clear_count + 1;
                end if;
                if proc_reset = '1' then
                    proc_reset_count <= proc_reset_count + 1;
                end if;
                previous_clear := proc_clear;
                previous_reset := proc_reset;
            end if;
        end if;
    end process p_proc_monitor;

    p_tdc_monitor : process (tdc_clk)
        variable previous_clear : std_logic := '0';
        variable previous_reset : std_logic := '0';
    begin
        if rising_edge(tdc_clk) then
            if tdc_rst_n = '0' then
                tdc_clear_count <= 0;
                tdc_reset_count <= 0;
                previous_clear := '0';
                previous_reset := '0';
            else
                assert not (tdc_clear = '1' and previous_clear = '1')
                    report "V2-K0-CMD-TB TDC CLEAR wider than one clock"
                    severity failure;
                assert not (tdc_reset = '1' and previous_reset = '1')
                    report "V2-K0-CMD-TB TDC RESET wider than one clock"
                    severity failure;
                if tdc_clear = '1' then
                    tdc_clear_count <= tdc_clear_count + 1;
                end if;
                if tdc_reset = '1' then
                    tdc_reset_count <= tdc_reset_count + 1;
                end if;
                previous_clear := tdc_clear;
                previous_reset := tdc_reset;
            end if;
        end if;
    end process p_tdc_monitor;

    p_source_monitor : process (source_clk)
    begin
        if rising_edge(source_clk) then
            if source_rst_n = '0' then
                rejected_count <= 0;
            elsif source_rejected = '1' then
                rejected_count <= rejected_count + 1;
            end if;
        end if;
    end process p_source_monitor;

    p_test : process
        procedure wait_ready is
        begin
            for cycle in 0 to 100 loop
                wait until rising_edge(source_clk);
                exit when source_ready = '1';
            end loop;
            assert source_ready = '1' and source_busy = '0'
                report "V2-K0-CMD-TB source did not become ready"
                severity failure;
        end procedure wait_ready;

        procedure pulse_source(signal value : out std_logic) is
        begin
            wait until falling_edge(source_clk);
            value <= '1';
            wait until falling_edge(source_clk);
            value <= '0';
        end procedure pulse_source;
    begin
        wait for 40 ns;
        source_rst_n <= '1';
        proc_rst_n <= '1';
        tdc_rst_n <= '1';
        wait_ready;

        pulse_source(clear_status);
        assert source_busy = '1'
            report "V2-K0-CMD-TB CLEAR did not occupy mailbox"
            severity failure;

        -- The second command is deliberately issued before both destination
        -- acknowledgements return. It must be diagnosed, never overwritten.
        pulse_source(soft_reset);
        wait_ready;
        assert proc_clear_count = 1 and tdc_clear_count = 1 and
               proc_reset_count = 0 and tdc_reset_count = 0 and
               rejected_count = 1
            report "V2-K0-CMD-TB busy rejection or CLEAR delivery mismatch"
            severity failure;

        pulse_source(soft_reset);
        wait_ready;
        assert proc_clear_count = 1 and tdc_clear_count = 1 and
               proc_reset_count = 1 and tdc_reset_count = 1
            report "V2-K0-CMD-TB RESET delivery mismatch"
            severity failure;

        wait until falling_edge(source_clk);
        clear_status <= '1';
        soft_reset <= '1';
        wait until falling_edge(source_clk);
        clear_status <= '0';
        soft_reset <= '0';
        for cycle in 0 to 3 loop
            wait until rising_edge(source_clk);
        end loop;
        assert source_ready = '1' and source_busy = '0' and
               rejected_count = 2 and
               proc_clear_count = 1 and tdc_clear_count = 1 and
               proc_reset_count = 1 and tdc_reset_count = 1
            report "V2-K0-CMD-TB ambiguous command was not rejected"
            severity failure;

        report "LIDAR_V2_SYSTEM_COMMAND_CDC_PASS proc_mhz=" &
            positive'image(G_PROC_CLK_MHZ) & " tdc_mhz=" &
            positive'image(G_TDC_CLK_MHZ) severity note;
        stop;
        wait;
    end process p_test;

end architecture sim;
