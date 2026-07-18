-- =============================================================================
-- tb_tdc_gpx_atomic_snapshot_cdc.vhd
-- Asynchronous coherent-snapshot and in-flight update regression.
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;

entity tb_tdc_gpx_atomic_snapshot_cdc is
end entity tb_tdc_gpx_atomic_snapshot_cdc;

architecture sim of tb_tdc_gpx_atomic_snapshot_cdc is
    constant c_WIDTH       : positive := 16;
    constant c_SRC_PERIOD  : time := 5 ns;
    constant c_DEST_PERIOD : time := 7 ns;

    signal s_src_clk     : std_logic := '0';
    signal s_dest_clk    : std_logic := '0';
    signal s_src_resetn  : std_logic := '0';
    signal s_dest_resetn : std_logic := '0';
    signal s_src_data    : std_logic_vector(c_WIDTH - 1 downto 0) := x"0000";
    signal s_dest_data   : std_logic_vector(c_WIDTH - 1 downto 0);
    signal s_done        : boolean := false;
begin
    s_src_clk <= not s_src_clk after c_SRC_PERIOD / 2 when not s_done else '0';
    s_dest_clk <= not s_dest_clk after c_DEST_PERIOD / 2 when not s_done else '0';

    u_dut : entity work.tdc_gpx_atomic_snapshot_cdc
        generic map (
            g_WIDTH          => c_WIDTH,
            g_SYNC_FF        => 4,
            g_SIM_ASSERT_CHK => 1
        )
        port map (
            i_src_clk     => s_src_clk,
            i_src_resetn  => s_src_resetn,
            i_src_data    => s_src_data,
            i_dest_clk    => s_dest_clk,
            i_dest_resetn => s_dest_resetn,
            o_dest_data   => s_dest_data
        );

    p_no_torn_values : process(s_dest_clk)
    begin
        if rising_edge(s_dest_clk) and s_dest_resetn = '1' then
            assert s_dest_data = x"0000"
                or s_dest_data = x"1357"
                or s_dest_data = x"A5C3"
                or s_dest_data = x"5A3C"
                or s_dest_data = x"F00D"
                report "atomic snapshot CDC emitted a torn payload: 0x"
                       & to_hstring(s_dest_data)
                severity failure;
        end if;
    end process p_no_torn_values;

    p_stimulus : process
        procedure wait_for_value(
            constant i_expected : std_logic_vector(c_WIDTH - 1 downto 0);
            constant i_limit    : positive
        ) is
        begin
            for i in 1 to i_limit loop
                wait until rising_edge(s_dest_clk);
                if s_dest_data = i_expected then
                    return;
                end if;
            end loop;
            assert false
                report "atomic snapshot CDC destination timeout"
                severity failure;
        end procedure;
    begin
        wait for 80 ns;
        wait until rising_edge(s_src_clk);
        s_src_resetn <= '1';
        wait until rising_edge(s_dest_clk);
        s_dest_resetn <= '1';

        -- Change twice while the first nonzero transfer is in flight. The
        -- destination may observe the first snapshot, but must converge to
        -- the latest complete value without a torn intermediate payload.
        wait until rising_edge(s_src_clk);
        s_src_data <= x"1357";
        wait until rising_edge(s_src_clk);
        s_src_data <= x"A5C3";
        wait until rising_edge(s_src_clk);
        s_src_data <= x"5A3C";
        wait_for_value(x"5A3C", 80);

        wait until rising_edge(s_src_clk);
        s_src_data <= x"F00D";
        wait_for_value(x"F00D", 80);

        -- Both domains reset in the integration contract. The source must
        -- force a fresh post-reset snapshot even if no later change occurs.
        wait until rising_edge(s_src_clk);
        s_src_resetn <= '0';
        s_dest_resetn <= '0';
        s_src_data <= x"A5C3";
        wait for 40 ns;
        wait until rising_edge(s_src_clk);
        s_src_resetn <= '1';
        wait until rising_edge(s_dest_clk);
        s_dest_resetn <= '1';
        wait_for_value(x"A5C3", 80);

        report "ATOMIC_SNAPSHOT_CDC ASYNC/RETRIGGER/RESET PASS"
            severity note;
        s_done <= true;
        wait;
    end process p_stimulus;

    p_watchdog : process
    begin
        wait for 10 us;
        assert s_done
            report "atomic snapshot CDC testbench watchdog timeout"
            severity failure;
        wait;
    end process p_watchdog;
end architecture sim;
