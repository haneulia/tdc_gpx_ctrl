-- =============================================================================
-- tb_tdc_gpx_request_loss.vhd
-- Verifies the consolidated register-request loss diagnostics.
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.tdc_gpx_pkg.all;

entity tb_tdc_gpx_request_loss is
end entity tb_tdc_gpx_request_loss;

architecture sim of tb_tdc_gpx_request_loss is
    constant c_CLK_PERIOD : time := 10 ns;

    signal s_clk   : std_logic := '0';
    signal s_rst_n : std_logic := '0';

    signal s_arb_reg_read        : std_logic := '0';
    signal s_arb_reg_write       : std_logic := '0';
    signal s_arb_reg_read_g      : std_logic_vector(c_MAX_CHIPS - 1 downto 0);
    signal s_arb_reg_write_g     : std_logic_vector(c_MAX_CHIPS - 1 downto 0);
    signal s_arb_reg_rejected    : std_logic;
    signal s_arb_reg_outstanding : std_logic;

    signal s_reg_start_read   : std_logic := '0';
    signal s_reg_start_write  : std_logic := '0';
    signal s_reg_rsp_valid    : std_logic := '0';
    signal s_reg_soft_clear   : std_logic := '0';
    signal s_reg_req_valid    : std_logic;
    signal s_reg_req_rw       : std_logic;
    signal s_reg_busy         : std_logic;
    signal s_reg_req_overflow : std_logic;
begin
    s_clk <= not s_clk after c_CLK_PERIOD / 2;

    u_cmd_arb : entity work.tdc_gpx_cmd_arb
        port map (
            i_clk                  => s_clk,
            i_rst_n                => s_rst_n,
            i_cmd_start            => '0',
            i_cmd_start_accepted   => '0',
            i_cmd_stop             => '0',
            i_cmd_soft_reset       => '0',
            i_cmd_cfg_write        => '0',
            i_cmd_reg_read         => s_arb_reg_read,
            i_cmd_reg_write        => s_arb_reg_write,
            i_cmd_reg_chip         => (others => '0'),
            i_cmd_reg_chip_address => std_logic_vector(to_unsigned(1, c_MAX_CHIPS)),
            i_cmd_reg_addr         => x"5",
            i_chip_busy            => (others => '0'),
            i_face_asm_idle        => '1',
            i_face_asm_fall_idle   => '1',
            i_hdr_idle             => '1',
            i_hdr_fall_idle        => '1',
            i_cmd_reg_done         => (others => '0'),
            o_cmd_cfg_write_g      => open,
            o_cmd_reg_read_g       => s_arb_reg_read_g,
            o_cmd_reg_write_g      => s_arb_reg_write_g,
            o_reg_outstanding      => s_arb_reg_outstanding,
            o_outstanding_chip     => open,
            o_cmd_reg_done_pulse   => open,
            o_cmd_reg_done_chip    => open,
            o_reg_loop_resume      => open,
            o_cmd_reg_addr_out     => open,
            o_reg_timeout          => open,
            o_reg_timeout_mask     => open,
            o_reg_rejected         => s_arb_reg_rejected,
            o_reg_zero_mask        => open
        );

    u_chip_reg : entity work.tdc_gpx_chip_reg
        port map (
            i_clk              => s_clk,
            i_rst_n            => s_rst_n,
            i_start_read       => s_reg_start_read,
            i_start_write      => s_reg_start_write,
            i_addr             => x"6",
            i_wdata            => std_logic_vector(to_unsigned(16#12345#, c_TDC_BUS_WIDTH)),
            o_rdata            => open,
            o_rvalid           => open,
            o_done             => open,
            o_timeout          => open,
            o_busy             => s_reg_busy,
            o_bus_req_valid    => s_reg_req_valid,
            o_bus_req_rw       => s_reg_req_rw,
            o_bus_req_addr     => open,
            o_bus_req_wdata    => open,
            i_bus_rsp_valid    => s_reg_rsp_valid,
            i_bus_rsp_rdata    => (others => '0'),
            o_err_req_overflow => s_reg_req_overflow,
            i_soft_clear       => s_reg_soft_clear
        );

    p_stimulus : process
    begin
        wait for 3 * c_CLK_PERIOD;
        wait until falling_edge(s_clk);
        s_rst_n <= '1';
        wait until rising_edge(s_clk);
        wait for 1 ns;

        -- cmd_arb: simultaneous read/write loses the read intent by contract.
        wait until falling_edge(s_clk);
        s_arb_reg_read  <= '1';
        s_arb_reg_write <= '1';
        wait until rising_edge(s_clk);
        wait for 1 ns;
        assert s_arb_reg_rejected = '1'
            report "cmd_arb did not record simultaneous R/W request loss"
            severity failure;
        assert s_arb_reg_write_g(0) = '1' and s_arb_reg_read_g(0) = '0'
            report "cmd_arb simultaneous R/W did not follow write-wins policy"
            severity failure;
        assert s_arb_reg_outstanding = '1'
            report "cmd_arb did not accept the write transaction"
            severity failure;
        wait until falling_edge(s_clk);
        s_arb_reg_read  <= '0';
        s_arb_reg_write <= '0';

        -- chip_reg: start a read, then collide read/write while it is active.
        s_reg_start_read <= '1';
        wait until rising_edge(s_clk);
        wait for 1 ns;
        assert s_reg_busy = '1' and s_reg_req_valid = '1' and s_reg_req_rw = '0'
            report "chip_reg did not start the initial read"
            severity failure;
        wait until falling_edge(s_clk);
        s_reg_start_read  <= '1';
        s_reg_start_write <= '1';
        wait until rising_edge(s_clk);
        wait for 1 ns;
        assert s_reg_req_overflow = '1'
            report "chip_reg did not record active simultaneous R/W request loss"
            severity failure;
        wait until falling_edge(s_clk);
        s_reg_start_read  <= '0';
        s_reg_start_write <= '0';
        s_reg_rsp_valid   <= '1';
        wait until rising_edge(s_clk);
        wait until falling_edge(s_clk);
        s_reg_rsp_valid <= '0';

        -- The write half of the collision must be preserved in the pending slot.
        wait until rising_edge(s_clk);
        wait for 1 ns;
        assert s_reg_req_valid = '1' and s_reg_req_rw = '1'
            report "chip_reg did not preserve the write-wins pending request"
            severity failure;

        wait until falling_edge(s_clk);
        s_reg_soft_clear <= '1';
        wait until rising_edge(s_clk);
        wait for 1 ns;
        assert s_reg_req_overflow = '0'
            report "chip_reg request-loss sticky did not clear"
            severity failure;
        s_reg_soft_clear <= '0';

        report "REQUEST_LOSS_DIAGNOSTIC_COLLAPSE PASS" severity note;
        std.env.stop;
        wait;
    end process p_stimulus;
end architecture sim;
