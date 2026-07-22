-- =============================================================================
-- tdc_gpx_cfg_image_override.vhd
-- TDC-GPX register-image policy override
-- =============================================================================
-- Applies the runtime fields that are owned by the merged controller
-- configuration after the CSR block has produced its raw register image.
-- This block is purely combinational; it has no shot or stream dependency.
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;

use work.tdc_gpx_pkg.all;
use work.tdc_gpx_cfg_pkg.all;

entity tdc_gpx_cfg_image_override is
    port (
        i_cfg           : in  t_tdc_cfg;
        i_cfg_image_raw : in  t_cfg_image;
        o_cfg_image     : out t_cfg_image
    );
end entity tdc_gpx_cfg_image_override;

architecture rtl of tdc_gpx_cfg_image_override is
begin
    p_override : process(all)
        variable v_img : t_cfg_image;
    begin
        v_img := i_cfg_image_raw;
        v_img(5)(c_REG5_STARTOFF1_HI downto c_REG5_STARTOFF1_LO)
            := std_logic_vector(i_cfg.start_off1);
        v_img(5)(c_REG5_MASTER_ALU_TRIG)  := '1';
        v_img(5)(c_REG5_PARTIAL_ALU_TRIG) := '0';
        v_img(7) := i_cfg.cfg_reg7;
        o_cfg_image <= v_img;
    end process p_override;
end architecture rtl;
