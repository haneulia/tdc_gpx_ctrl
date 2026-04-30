-- =============================================================================
-- tb_tdc_gpx_stale_ready.vhd
-- Negative-style stale-ready verification for C02 skid/sync FIFO boundaries
-- =============================================================================
--
-- Purpose:
--   Verifies that registered-ready deassertion does not cause pop/drop/duplicate
--   behavior when downstream tready is held low.
--
-- Checks:
--   [1] tdc_gpx_skid_buffer accepts exactly the two legal beats while the sink
--       is blocked, then stops accepting until downstream ready returns.
--   [2] tdc_gpx_sync_fifo with input/output skid absorbs bounded stale-ready
--       pressure, then drains all beats in order after ready is released.
--
-- Standard: VHDL-2008
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity tb_tdc_gpx_stale_ready is
end entity tb_tdc_gpx_stale_ready;

architecture sim of tb_tdc_gpx_stale_ready is
    constant c_CLK_PERIOD : time := 5 ns;
    constant c_SKID_BEATS : natural := 8;
    constant c_FIFO_BEATS : natural := 12;
    constant c_FIFO_MAX_BLOCKED_ACCEPT : natural := 6; -- in skid + core + out skid

    signal s_done  : boolean := false;
    signal s_clk   : std_logic := '0';
    signal s_rst_n : std_logic := '0';

    signal s_sk_s_valid : std_logic := '0';
    signal s_sk_s_ready : std_logic;
    signal s_sk_s_data  : std_logic_vector(7 downto 0) := (others => '0');
    signal s_sk_m_valid : std_logic;
    signal s_sk_m_ready : std_logic := '0';
    signal s_sk_m_data  : std_logic_vector(7 downto 0);

    signal s_ff_s_valid : std_logic := '0';
    signal s_ff_s_ready : std_logic;
    signal s_ff_s_data  : std_logic_vector(7 downto 0) := (others => '0');
    signal s_ff_m_valid : std_logic;
    signal s_ff_m_ready : std_logic := '0';
    signal s_ff_m_data  : std_logic_vector(7 downto 0);

begin

    s_clk <= not s_clk after c_CLK_PERIOD / 2 when not s_done else '0';

    u_skid : entity work.tdc_gpx_skid_buffer
        generic map (
            g_DATA_WIDTH => 8
        )
        port map (
            i_clk     => s_clk,
            i_rst_n   => s_rst_n,
            i_flush   => '0',
            i_s_valid => s_sk_s_valid,
            o_s_ready => s_sk_s_ready,
            i_s_data  => s_sk_s_data,
            o_m_valid => s_sk_m_valid,
            i_m_ready => s_sk_m_ready,
            o_m_data  => s_sk_m_data
        );

    u_fifo : entity work.tdc_gpx_sync_fifo
        generic map (
            g_DATA_WIDTH => 8,
            g_DEPTH      => 2,
            g_LOG2_DEPTH => 1,
            g_IN_REG     => true,
            g_OUT_REG    => true
        )
        port map (
            i_clk     => s_clk,
            i_rst_n   => s_rst_n,
            i_flush   => '0',
            i_s_valid => s_ff_s_valid,
            o_s_ready => s_ff_s_ready,
            i_s_data  => s_ff_s_data,
            o_m_valid => s_ff_m_valid,
            i_m_ready => s_ff_m_ready,
            o_m_data  => s_ff_m_data
        );

    p_watchdog : process
    begin
        wait for 20 us;
        if not s_done then
            assert false report "WATCHDOG timeout in tb_tdc_gpx_stale_ready"
                severity failure;
        end if;
        wait;
    end process p_watchdog;

    p_stim : process
        variable v_sent              : natural := 0;
        variable v_recv              : natural := 0;
        variable v_accept_blocked    : natural := 0;
        variable v_ready_high_seen   : boolean := false;
        variable v_ready_low_seen    : boolean := false;
        variable v_data              : natural := 0;
    begin
        s_rst_n <= '0';
        wait for 10 * c_CLK_PERIOD;
        wait until rising_edge(s_clk);
        s_rst_n <= '1';
        wait until rising_edge(s_clk);

        -- ------------------------------------------------------------------
        -- [1] Skid buffer stale-ready test
        -- ------------------------------------------------------------------
        report "[1] skid_buffer stale-ready negative test" severity note;
        s_sk_m_ready <= '0';
        s_sk_s_valid <= '1';
        s_sk_s_data  <= std_logic_vector(to_unsigned(0, 8));
        v_sent := 0;
        v_recv := 0;
        v_accept_blocked := 0;
        v_ready_high_seen := false;
        v_ready_low_seen := false;

        for i in 0 to 7 loop
            wait until rising_edge(s_clk);

            if s_sk_s_ready = '1' then
                v_ready_high_seen := true;
            else
                v_ready_low_seen := true;
            end if;

            if s_sk_s_valid = '1' and s_sk_s_ready = '1' then
                v_sent := v_sent + 1;
                v_accept_blocked := v_accept_blocked + 1;
                if v_sent < c_SKID_BEATS then
                    s_sk_s_data <= std_logic_vector(to_unsigned(v_sent, 8));
                else
                    s_sk_s_valid <= '0';
                end if;
            end if;

            assert not (s_sk_m_valid = '1' and s_sk_m_ready = '1')
                report "[1] output handshake occurred while sink ready was low"
                severity failure;
        end loop;

        assert v_ready_high_seen and v_ready_low_seen
            report "[1] registered ready did not exercise both stale-high and backpressure-low phases"
            severity failure;
        assert v_accept_blocked = 2
            report "[1] skid accepted " & integer'image(v_accept_blocked)
                & " blocked beats; expected exactly 2"
            severity failure;

        s_sk_m_ready <= '1';
        while v_recv < c_SKID_BEATS loop
            wait until rising_edge(s_clk);

            if s_sk_s_valid = '1' and s_sk_s_ready = '1' then
                v_sent := v_sent + 1;
                if v_sent < c_SKID_BEATS then
                    s_sk_s_data <= std_logic_vector(to_unsigned(v_sent, 8));
                else
                    s_sk_s_valid <= '0';
                end if;
            end if;

            if s_sk_m_valid = '1' and s_sk_m_ready = '1' then
                v_data := to_integer(unsigned(s_sk_m_data));
                assert v_data = v_recv
                    report "[1] skid output order mismatch: got "
                        & integer'image(v_data) & ", expected "
                        & integer'image(v_recv)
                    severity failure;
                v_recv := v_recv + 1;
            end if;
        end loop;

        assert v_sent = c_SKID_BEATS
            report "[1] skid source accepted count mismatch"
            severity failure;
        s_sk_s_valid <= '0';
        wait until rising_edge(s_clk);
        wait until rising_edge(s_clk);
        assert s_sk_m_valid = '0'
            report "[1] skid produced an extra beat after expected drain"
            severity failure;
        report "[1] skid_buffer PASS: stale ready absorbed, order preserved"
            severity note;

        -- ------------------------------------------------------------------
        -- [2] Sync FIFO stale-ready test
        -- ------------------------------------------------------------------
        report "[2] sync_fifo stale-ready negative test" severity note;
        s_ff_m_ready <= '0';
        s_ff_s_valid <= '1';
        s_ff_s_data  <= std_logic_vector(to_unsigned(0, 8));
        v_sent := 0;
        v_recv := 0;
        v_accept_blocked := 0;
        v_ready_high_seen := false;
        v_ready_low_seen := false;

        for i in 0 to 15 loop
            wait until rising_edge(s_clk);

            if s_ff_s_ready = '1' then
                v_ready_high_seen := true;
            else
                v_ready_low_seen := true;
            end if;

            if s_ff_s_valid = '1' and s_ff_s_ready = '1' then
                v_sent := v_sent + 1;
                v_accept_blocked := v_accept_blocked + 1;
                if v_sent < c_FIFO_BEATS then
                    s_ff_s_data <= std_logic_vector(to_unsigned(v_sent, 8));
                else
                    s_ff_s_valid <= '0';
                end if;
            end if;

            assert not (s_ff_m_valid = '1' and s_ff_m_ready = '1')
                report "[2] FIFO output handshake occurred while sink ready was low"
                severity failure;
        end loop;

        assert v_ready_high_seen and v_ready_low_seen
            report "[2] FIFO ready did not exercise both stale-high and low phases"
            severity failure;
        assert v_accept_blocked <= c_FIFO_MAX_BLOCKED_ACCEPT
            report "[2] FIFO over-accepted while blocked: "
                & integer'image(v_accept_blocked) & " beats"
            severity failure;
        assert v_accept_blocked >= 2
            report "[2] FIFO did not absorb the registered-ready stale window"
            severity failure;

        s_ff_m_ready <= '1';
        while v_recv < c_FIFO_BEATS loop
            wait until rising_edge(s_clk);

            if s_ff_s_valid = '1' and s_ff_s_ready = '1' then
                v_sent := v_sent + 1;
                if v_sent < c_FIFO_BEATS then
                    s_ff_s_data <= std_logic_vector(to_unsigned(v_sent, 8));
                else
                    s_ff_s_valid <= '0';
                end if;
            end if;

            if s_ff_m_valid = '1' and s_ff_m_ready = '1' then
                v_data := to_integer(unsigned(s_ff_m_data));
                assert v_data = v_recv
                    report "[2] FIFO output order mismatch: got "
                        & integer'image(v_data) & ", expected "
                        & integer'image(v_recv)
                    severity failure;
                v_recv := v_recv + 1;
            end if;
        end loop;

        assert v_sent = c_FIFO_BEATS
            report "[2] FIFO source accepted count mismatch"
            severity failure;
        s_ff_s_valid <= '0';
        wait until rising_edge(s_clk);
        wait until rising_edge(s_clk);
        wait until rising_edge(s_clk);
        assert s_ff_m_valid = '0'
            report "[2] FIFO produced an extra beat after expected drain"
            severity failure;

        report "[2] sync_fifo PASS: blocked accepts="
            & integer'image(v_accept_blocked) & ", order preserved"
            severity note;
        report "tb_tdc_gpx_stale_ready: ALL TESTS PASSED" severity note;
        s_done <= true;
        wait;
    end process p_stim;

end architecture sim;
