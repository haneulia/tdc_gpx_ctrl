-- =============================================================================
-- tb_tdc_gpx_cfg_image_override.vhd
-- Combinational contract test for GPX register-image policy override
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.tdc_gpx_pkg.all;
use work.tdc_gpx_cfg_pkg.all;

entity tb_tdc_gpx_cfg_image_override is
end entity tb_tdc_gpx_cfg_image_override;

architecture sim of tb_tdc_gpx_cfg_image_override is
    signal s_cfg       : t_tdc_cfg := c_TDC_CFG_INIT;
    signal s_raw_image : t_cfg_image := (others => (others => '0'));
    signal s_image     : t_cfg_image;
begin
    u_dut : entity work.tdc_gpx_cfg_image_override
        port map (
            i_cfg           => s_cfg,
            i_cfg_image_raw => s_raw_image,
            o_cfg_image     => s_image
        );

    p_stimulus : process
        variable v_cfg      : t_tdc_cfg;
        variable v_raw      : t_cfg_image;
        variable v_expected : t_cfg_image;
    begin
        v_cfg := c_TDC_CFG_INIT;
        v_cfg.start_off1 := to_unsigned(16#2A155#, 18);
        v_cfg.cfg_reg7   := x"89ABCDEF";

        for i in 0 to 7 loop
            v_raw(i) := std_logic_vector(to_unsigned(
                16#10203040# + i * 16#01010101#, 32));
        end loop;

        s_cfg       <= v_cfg;
        s_raw_image <= v_raw;
        wait for 1 ns;

        v_expected := v_raw;
        v_expected(5)(c_REG5_STARTOFF1_HI downto c_REG5_STARTOFF1_LO)
            := std_logic_vector(v_cfg.start_off1);
        v_expected(5)(c_REG5_MASTER_ALU_TRIG)  := '1';
        v_expected(5)(c_REG5_PARTIAL_ALU_TRIG) := '0';
        v_expected(7) := v_cfg.cfg_reg7;

        assert s_image = v_expected
            report "tb_tdc_gpx_cfg_image_override: first override mismatch"
            severity failure;

        v_cfg.start_off1 := to_unsigned(3, 18);
        v_cfg.cfg_reg7   := x"01234567";
        s_cfg <= v_cfg;
        wait for 1 ns;

        v_expected(5)(c_REG5_STARTOFF1_HI downto c_REG5_STARTOFF1_LO)
            := std_logic_vector(v_cfg.start_off1);
        v_expected(7) := v_cfg.cfg_reg7;

        assert s_image = v_expected
            report "tb_tdc_gpx_cfg_image_override: live config update mismatch"
            severity failure;

        report "tb_tdc_gpx_cfg_image_override: ALL TESTS PASSED"
            severity note;
        std.env.stop;
        wait;
    end process p_stimulus;
end architecture sim;
