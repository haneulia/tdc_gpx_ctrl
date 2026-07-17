-- =============================================================================
-- tb_tdc_gpx_reg_rsp_cdc.vhd
-- Worst-ratio atomic register-response CDC test (200 MHz -> 50 MHz)
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;

entity tb_tdc_gpx_reg_rsp_cdc is
end entity tb_tdc_gpx_reg_rsp_cdc;

architecture sim of tb_tdc_gpx_reg_rsp_cdc is

    constant c_DATA_WIDTH     : positive := 28;
    constant c_SRC_PERIOD     : time := 5 ns;
    constant c_DST_PERIOD     : time := 20 ns;
    constant c_RESET_HOLD     : time := 100 ns;
    constant c_WATCHDOG       : time := 20 us;

    signal s_src_clk     : std_logic := '0';
    signal s_src_rst_n   : std_logic := '0';
    signal s_src_done    : std_logic := '0';
    signal s_src_rvalid  : std_logic := '0';
    signal s_src_rdata   : std_logic_vector(c_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_src_pending : std_logic;

    signal s_dst_clk     : std_logic := '0';
    signal s_dst_rst_n   : std_logic := '0';
    signal s_dst_clear   : std_logic := '0';
    signal s_dst_done    : std_logic;
    signal s_dst_rvalid  : std_logic;
    signal s_dst_rvalid_held : std_logic;
    signal s_dst_rdata   : std_logic_vector(c_DATA_WIDTH - 1 downto 0);

    signal s_done_count  : natural := 0;
    signal s_sim_done    : boolean := false;

begin

    s_src_clk <= not s_src_clk after c_SRC_PERIOD / 2 when not s_sim_done else '0';
    s_dst_clk <= not s_dst_clk after c_DST_PERIOD / 2 when not s_sim_done else '0';

    u_dut : entity work.tdc_gpx_reg_rsp_cdc
        generic map (
            g_DATA_WIDTH => c_DATA_WIDTH,
            g_SYNC_FF    => 4
        )
        port map (
            i_src_clk     => s_src_clk,
            i_src_rst_n   => s_src_rst_n,
            i_src_done    => s_src_done,
            i_src_rvalid  => s_src_rvalid,
            i_src_rdata   => s_src_rdata,
            o_src_pending => s_src_pending,
            i_dst_clk     => s_dst_clk,
            i_dst_rst_n   => s_dst_rst_n,
            i_dst_clear   => s_dst_clear,
            o_dst_done    => s_dst_done,
            o_dst_rvalid  => s_dst_rvalid,
            o_dst_rvalid_held => s_dst_rvalid_held,
            o_dst_rdata   => s_dst_rdata
        );

    p_reset : process
    begin
        s_src_rst_n <= '0';
        s_dst_rst_n <= '0';
        wait for c_RESET_HOLD;
        wait until rising_edge(s_src_clk);
        s_src_rst_n <= '1';
        wait until rising_edge(s_dst_clk);
        s_dst_rst_n <= '1';
        wait;
    end process p_reset;

    p_count : process(s_dst_clk)
    begin
        if rising_edge(s_dst_clk) then
            if s_dst_rst_n = '0' then
                s_done_count <= 0;
            elsif s_dst_done = '1' then
                s_done_count <= s_done_count + 1;
            end if;
        end if;
    end process p_count;

    p_stimulus : process
        procedure send_response(
            constant rvalid : in std_logic;
            constant rdata  : in std_logic_vector(c_DATA_WIDTH - 1 downto 0)
        ) is
        begin
            wait until rising_edge(s_src_clk);
            s_src_rvalid <= rvalid;
            s_src_rdata  <= rdata;
            s_src_done   <= '1';
            wait until rising_edge(s_src_clk);
            s_src_done   <= '0';
            s_src_rvalid <= '0';
            s_src_rdata  <= not rdata;
            wait for 1 ps;
            assert s_src_pending = '1'
                report "reg_rsp_cdc TB: source pending did not cover completion"
                severity failure;
        end procedure send_response;

        procedure expect_response(
            constant rvalid : in std_logic;
            constant rdata  : in std_logic_vector(c_DATA_WIDTH - 1 downto 0)
        ) is
        begin
            loop
                wait until rising_edge(s_dst_clk);
                wait for 1 ps;
                exit when s_dst_done = '1';
            end loop;

            assert s_dst_rvalid = rvalid
                report "reg_rsp_cdc TB: destination rvalid mismatch"
                severity failure;
            if rvalid = '1' then
                assert s_dst_rdata = rdata
                    report "reg_rsp_cdc TB: atomic read payload mismatch"
                    severity failure;
            end if;

            wait until rising_edge(s_dst_clk);
            wait for 1 ps;
            assert s_dst_done = '0'
                report "reg_rsp_cdc TB: destination done was not one cycle"
                severity failure;
            assert s_dst_rvalid_held = rvalid
                report "reg_rsp_cdc TB: held read qualifier mismatch before retire"
                severity failure;

            s_dst_clear <= '1';
            wait until rising_edge(s_dst_clk);
            s_dst_clear <= '0';
            wait for 1 ps;
            assert s_dst_rvalid_held = '0'
                report "reg_rsp_cdc TB: held read qualifier did not clear"
                severity failure;

            if s_src_pending = '1' then
                wait until s_src_pending = '0' for 2 us;
            end if;
            assert s_src_pending = '0'
                report "reg_rsp_cdc TB: source handshake did not retire"
                severity failure;
        end procedure expect_response;

        variable v_expected_count : natural := 0;
    begin
        wait until s_src_rst_n = '1' and s_dst_rst_n = '1';

        -- Read response: mutate the live source bus immediately after capture;
        -- the destination must still receive the original bundled value.
        send_response('1', x"1234567");
        expect_response('1', x"1234567");
        v_expected_count := v_expected_count + 1;

        -- Write completion carries no read-valid event.
        wait for 7 ns;
        send_response('0', x"7654321");
        expect_response('0', x"7654321");
        v_expected_count := v_expected_count + 1;

        -- Exercise different source/destination phases at the 4:1 ratio.
        wait for 3 ns;
        send_response('1', x"0000001");
        expect_response('1', x"0000001");
        v_expected_count := v_expected_count + 1;

        wait for 11 ns;
        send_response('1', x"0ABCDEF");
        expect_response('1', x"0ABCDEF");
        v_expected_count := v_expected_count + 1;

        assert s_done_count = v_expected_count
            report "reg_rsp_cdc TB: duplicate or missing destination completion"
            severity failure;

        report "REG_RSP_CDC 200-to-50 MHz atomic transfer - PASS"
            severity note;
        s_sim_done <= true;
        wait;
    end process p_stimulus;

    p_watchdog : process
    begin
        wait for c_WATCHDOG;
        assert s_sim_done
            report "reg_rsp_cdc TB: watchdog timeout"
            severity failure;
        wait;
    end process p_watchdog;

end architecture sim;
