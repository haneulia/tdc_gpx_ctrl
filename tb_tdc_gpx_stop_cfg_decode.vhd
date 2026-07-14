-- =============================================================================
-- tb_tdc_gpx_stop_cfg_decode.vhd
-- Unit checks for stop_cfg_decode expected-count final marker handling.
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.env.all;

use work.tdc_gpx_pkg.all;
use work.tdc_gpx_cfg_pkg.all;

entity tb_tdc_gpx_stop_cfg_decode is
end entity tb_tdc_gpx_stop_cfg_decode;

architecture sim of tb_tdc_gpx_stop_cfg_decode is
    constant c_CLK_PERIOD : time := 5 ns;

    signal s_clk   : std_logic := '0';
    signal s_rst_n : std_logic := '0';

    signal s_stop_evt_tvalid : std_logic := '0';
    signal s_stop_evt_tdata  : std_logic_vector(c_STOP_EVT_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_stop_evt_tuser  : std_logic_vector(c_STOP_EVT_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_stop_evt_tready : std_logic;

    signal s_fire_count_tvalid : std_logic := '0';
    signal s_fire_count_tdata  : std_logic_vector(31 downto 0) := (others => '0');
    signal s_fire_count_tlast  : std_logic := '0';

    signal s_shot_start_gated : std_logic := '0';
    signal s_current_fire_count : unsigned(15 downto 0) := (others => '0');
    signal s_expected_ififo1  : t_expected_array;
    signal s_expected_ififo2  : t_expected_array;
    signal s_expected_final_valid : std_logic;
    signal s_cfg              : t_tdc_cfg := c_TDC_CFG_INIT;
    signal s_cfg_image_raw    : t_cfg_image := (others => (others => '0'));
    signal s_cfg_image        : t_cfg_image;
    signal s_mono_mask        : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_orphan_sticky    : std_logic;

    procedure wait_clk(signal clk : in std_logic; n : natural) is
    begin
        for i in 1 to n loop
            wait until rising_edge(clk);
        end loop;
    end procedure;

    procedure pulse(signal sig : out std_logic) is
    begin
        sig <= '1';
        wait_clk(s_clk, 1);
        sig <= '0';
    end procedure;

begin
    s_clk <= not s_clk after c_CLK_PERIOD / 2;

    u_dut : entity work.tdc_gpx_stop_cfg_decode
        generic map (
            g_STOP_EVT_DWIDTH   => c_STOP_EVT_DATA_WIDTH,
            g_FIRE_COUNT_DWIDTH => 32,
            g_WINDOW_MARGIN_CLKS => 1
        )
        port map (
            i_clk              => s_clk,
            i_rst_n            => s_rst_n,
            i_stop_evt_tvalid  => s_stop_evt_tvalid,
            i_stop_evt_tdata   => s_stop_evt_tdata,
            i_stop_evt_tuser   => s_stop_evt_tuser,
            o_stop_evt_tready  => s_stop_evt_tready,
            i_fire_count_tvalid => s_fire_count_tvalid,
            i_fire_count_tdata  => s_fire_count_tdata,
            i_fire_count_tlast  => s_fire_count_tlast,
            i_shot_start_gated => s_shot_start_gated,
            i_current_fire_count => s_current_fire_count,
            i_max_range_axis_clks => to_unsigned(8, 16),
            o_expected_ififo1  => s_expected_ififo1,
            o_expected_ififo2  => s_expected_ififo2,
            o_expected_final_valid => s_expected_final_valid,
            i_cfg              => s_cfg,
            i_cfg_image_raw    => s_cfg_image_raw,
            o_cfg_image        => s_cfg_image,
            o_monotonic_violation_mask => s_mono_mask,
            o_orphan_stop_evt_sticky   => s_orphan_sticky
        );

    p_stim : process
    begin
        s_cfg <= c_TDC_CFG_INIT;
        s_cfg.max_range_5ns_ticks <= to_unsigned(8, 16);

        wait_clk(s_clk, 4);
        s_rst_n <= '1';
        wait_clk(s_clk, 4);

        assert s_expected_final_valid = '0'
            report "expected_final_valid should reset low"
            severity failure;

        -- Zero-stop shot: no stop_evt beat, only fire_count final summary.
        pulse(s_shot_start_gated);
        s_current_fire_count <= to_unsigned(1, 16);
        wait_clk(s_clk, 2);

        s_fire_count_tvalid <= '1';
        s_fire_count_tlast  <= '1';
        s_fire_count_tdata  <= x"00000001";
        wait_clk(s_clk, 1);
        s_fire_count_tvalid <= '0';
        s_fire_count_tlast  <= '0';
        wait_clk(s_clk, 1);

        assert s_expected_final_valid = '1'
            report "zero-stop final marker did not assert expected_final_valid"
            severity failure;
        assert s_expected_ififo1(0) = 0 and s_expected_ififo2(0) = 0
            report "zero-stop final marker changed expected counts"
            severity failure;

        -- Next shot clears the final marker.
        pulse(s_shot_start_gated);
        s_current_fire_count <= to_unsigned(2, 16);
        wait_clk(s_clk, 1);
        assert s_expected_final_valid = '0'
            report "shot_start did not clear expected_final_valid"
            severity failure;

        -- Wrong fire_count value must not update the expected counts.
        s_stop_evt_tdata <= (others => '0');
        s_stop_evt_tuser <= (others => '0');
        s_stop_evt_tdata(3 downto 0) <= x"4";
        s_stop_evt_tvalid <= '1';
        s_fire_count_tvalid <= '1';
        s_fire_count_tlast  <= '0';
        s_fire_count_tdata  <= x"00000001";
        wait_clk(s_clk, 1);
        s_stop_evt_tvalid <= '0';
        s_fire_count_tvalid <= '0';
        wait_clk(s_clk, 1);
        assert s_expected_ififo1(0) = 0 and s_expected_ififo2(0) = 0
            report "mismatched fire_count updated expected counts"
            severity failure;
        assert s_orphan_sticky = '1'
            report "mismatched fire_count did not flag orphan/ownership violation"
            severity failure;

        -- Non-zero running total plus final marker remains count-known.
        s_stop_evt_tdata <= (others => '0');
        s_stop_evt_tuser <= (others => '0');
        s_stop_evt_tdata(3 downto 0) <= x"2";
        s_stop_evt_tuser(3 downto 0) <= x"1";
        s_stop_evt_tdata(7 downto 4) <= x"1";
        s_stop_evt_tuser(7 downto 4) <= x"2";
        s_stop_evt_tvalid <= '1';
        s_fire_count_tvalid <= '1';
        s_fire_count_tlast  <= '0';
        s_fire_count_tdata  <= x"00000002";
        wait_clk(s_clk, 1);
        s_stop_evt_tvalid <= '0';
        s_fire_count_tvalid <= '0';
        wait_clk(s_clk, 1);

        assert s_expected_ififo1(0) = to_unsigned(3, 8)
            report "IFIFO1 expected count decode failed"
            severity failure;
        assert s_expected_ififo2(0) = to_unsigned(3, 8)
            report "IFIFO2 expected count decode failed"
            severity failure;

        s_fire_count_tvalid <= '1';
        s_fire_count_tlast  <= '1';
        s_fire_count_tdata  <= x"00000002";
        wait_clk(s_clk, 1);
        s_fire_count_tvalid <= '0';
        s_fire_count_tlast  <= '0';
        wait_clk(s_clk, 1);

        assert s_expected_final_valid = '1'
            report "non-zero final marker did not assert expected_final_valid"
            severity failure;
        assert s_stop_evt_tready = '1'
            report "stop_evt ready contract should be always ready"
            severity failure;

        report "tb_tdc_gpx_stop_cfg_decode: ALL TESTS PASSED"
            severity note;
        stop;
        wait;
    end process;

end architecture sim;
