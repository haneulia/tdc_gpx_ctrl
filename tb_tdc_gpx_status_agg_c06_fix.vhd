-- =============================================================================
-- tb_tdc_gpx_status_agg_c06_fix.vhd
-- C06 focused testbench for tdc_gpx_status_agg register-boundary fix.
--
-- Verifies:
--   A. busy is produced through the registered status boundary.
--   B. rise/fall/pipeline overrun flags are registered and keep their meaning.
--   C. existing drain/sequence sticky soft_clear behavior is preserved.
--
-- Standard: VHDL-2008
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.env.all;

use work.tdc_gpx_pkg.all;

entity tb_tdc_gpx_status_agg_c06_fix is
end entity tb_tdc_gpx_status_agg_c06_fix;

architecture sim of tb_tdc_gpx_status_agg_c06_fix is
    constant C_CLK_PER : time := 5 ns;
    constant C_ZEROS   : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '0');

    signal s_clk       : std_logic := '0';
    signal s_rst_n     : std_logic := '0';
    signal s_soft_rst  : std_logic := '0';
    signal s_start_acc : std_logic := '0';
    signal s_soft_clr  : std_logic := '0';

    signal s_face_idle      : std_logic := '1';
    signal s_chip_busy      : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '0');
    signal s_reg_outstanding : std_logic := '0';
    signal s_face_asm_idle  : std_logic := '1';
    signal s_face_asm_fall_idle : std_logic := '1';
    signal s_hdr_idle       : std_logic := '1';
    signal s_hdr_fall_idle  : std_logic := '1';
    signal s_face_tvalid    : std_logic := '0';
    signal s_face_fall_tvalid : std_logic := '0';
    signal s_face_buf_tvalid : std_logic := '0';
    signal s_face_fall_buf_tvalid : std_logic := '0';
    signal s_axis_tvalid    : std_logic := '0';
    signal s_axis_fall_tvalid : std_logic := '0';

    signal s_stop_id_error  : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '0');
    signal s_hit_dropped    : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '0');
    signal s_hit_fall_dropped : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '0');
    signal s_err_drain_timeout : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '0');
    signal s_err_sequence   : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '0');
    signal s_chip_error     : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '0');
    signal s_face_mask      : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '1');
    signal s_shot_overrun   : std_logic := '0';
    signal s_fall_overrun   : std_logic := '0';

    signal s_status         : t_tdc_status;
    signal s_timestamp      : unsigned(63 downto 0);
    signal s_error_count    : unsigned(31 downto 0);
    signal s_drain_sticky   : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_seq_sticky     : std_logic_vector(c_N_CHIPS - 1 downto 0);

    procedure wait_clk(signal i_clk : in std_logic; constant n : natural) is
    begin
        for i in 1 to n loop
            wait until rising_edge(i_clk);
            wait for 1 ps;
        end loop;
    end procedure;

begin
    s_clk <= not s_clk after C_CLK_PER / 2;

    u_dut : entity work.tdc_gpx_status_agg
        port map (
            i_clk                  => s_clk,
            i_rst_n                => s_rst_n,
            i_cmd_soft_reset       => s_soft_rst,
            i_cmd_start_accepted   => s_start_acc,
            i_soft_clear           => s_soft_clr,
            i_face_state_idle      => s_face_idle,
            i_chip_busy            => s_chip_busy,
            i_reg_outstanding      => s_reg_outstanding,
            i_face_asm_idle        => s_face_asm_idle,
            i_face_asm_fall_idle   => s_face_asm_fall_idle,
            i_hdr_idle             => s_hdr_idle,
            i_hdr_fall_idle        => s_hdr_fall_idle,
            i_face_tvalid          => s_face_tvalid,
            i_face_fall_tvalid     => s_face_fall_tvalid,
            i_face_buf_tvalid      => s_face_buf_tvalid,
            i_face_fall_buf_tvalid => s_face_fall_buf_tvalid,
            i_m_axis_tvalid        => s_axis_tvalid,
            i_m_axis_fall_tvalid   => s_axis_fall_tvalid,
            i_stop_id_error        => s_stop_id_error,
            i_hit_dropped          => s_hit_dropped,
            i_hit_fall_dropped     => s_hit_fall_dropped,
            i_err_drain_timeout    => s_err_drain_timeout,
            i_err_sequence         => s_err_sequence,
            i_chip_error_merged    => s_chip_error,
            i_face_active_mask     => s_face_mask,
            i_shot_overrun         => s_shot_overrun,
            i_shot_fall_overrun    => s_fall_overrun,
            o_busy                 => s_status.busy,
            o_pipeline_overrun     => s_status.pipeline_overrun,
            o_rise_overrun         => s_status.rise_overrun,
            o_fall_overrun         => s_status.fall_overrun,
            o_timestamp            => s_timestamp,
            o_error_cycle_count    => s_error_count,
            o_err_drain_sticky     => s_drain_sticky,
            o_err_seq_sticky       => s_seq_sticky
        );

    p_stim : process
    begin
        s_rst_n <= '0';
        wait_clk(s_clk, 4);
        s_rst_n <= '1';
        wait_clk(s_clk, 2);

        assert s_status.busy = '0'
            report "status_agg C06: busy should be low after reset"
            severity failure;

        report "Scenario A: registered busy boundary" severity note;
        s_face_idle <= '0';
        wait_clk(s_clk, 1);
        assert s_status.busy = '1'
            report "status_agg C06: busy did not assert from face_state"
            severity failure;
        s_face_idle <= '1';
        wait_clk(s_clk, 1);
        assert s_status.busy = '0'
            report "status_agg C06: busy did not deassert after idle"
            severity failure;

        report "Scenario B: registered overrun flags" severity note;
        s_shot_overrun <= '1';
        wait_clk(s_clk, 1);
        assert s_status.pipeline_overrun = '1' and s_status.rise_overrun = '1'
               and s_status.fall_overrun = '0'
            report "status_agg C06: rise overrun mapping failed"
            severity failure;
        s_shot_overrun <= '0';
        s_fall_overrun <= '1';
        wait_clk(s_clk, 1);
        assert s_status.pipeline_overrun = '1' and s_status.rise_overrun = '0'
               and s_status.fall_overrun = '1'
            report "status_agg C06: fall overrun mapping failed"
            severity failure;
        s_fall_overrun <= '0';
        wait_clk(s_clk, 1);
        assert s_status.pipeline_overrun = '0' and s_status.rise_overrun = '0'
               and s_status.fall_overrun = '0'
            report "status_agg C06: overrun flags did not clear"
            severity failure;

        report "Scenario C: sticky soft_clear preserved" severity note;
        s_err_drain_timeout <= "0010";
        s_err_sequence      <= "0100";
        wait_clk(s_clk, 1);
        s_err_drain_timeout <= C_ZEROS;
        s_err_sequence      <= C_ZEROS;
        wait_clk(s_clk, 1);
        assert s_drain_sticky = "0010" and s_seq_sticky = "0100"
            report "status_agg C06: sticky set failed"
            severity failure;
        s_soft_clr <= '1';
        wait_clk(s_clk, 1);
        s_soft_clr <= '0';
        assert s_drain_sticky = C_ZEROS and s_seq_sticky = C_ZEROS
            report "status_agg C06: soft_clear did not clear stickies"
            severity failure;

        report "ALL STATUS_AGG C06 SCENARIOS PASSED" severity note;
        finish;
        wait;
    end process;

end architecture sim;
