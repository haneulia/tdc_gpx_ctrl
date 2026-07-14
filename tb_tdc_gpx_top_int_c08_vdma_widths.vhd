-- Fixed-width C08 wrappers for the C07 4-chip dedicated-slope target.

library ieee;
use ieee.std_logic_1164.all;

entity tb_tdc_gpx_top_int_c08_vdma_w32 is
end entity;

architecture sim of tb_tdc_gpx_top_int_c08_vdma_w32 is
begin
    u_tb : entity work.tb_tdc_gpx_top_int_c07_4chip_target
        generic map (G_TDATA_WIDTH => 32);
end architecture;

library ieee;
use ieee.std_logic_1164.all;

entity tb_tdc_gpx_top_int_c08_vdma_w64 is
end entity;

architecture sim of tb_tdc_gpx_top_int_c08_vdma_w64 is
begin
    u_tb : entity work.tb_tdc_gpx_top_int_c07_4chip_target
        generic map (G_TDATA_WIDTH => 64);
end architecture;

library ieee;
use ieee.std_logic_1164.all;

entity tb_tdc_gpx_top_int_c08_vdma_w128 is
end entity;

architecture sim of tb_tdc_gpx_top_int_c08_vdma_w128 is
begin
    u_tb : entity work.tb_tdc_gpx_top_int_c07_4chip_target
        generic map (G_TDATA_WIDTH => 128);
end architecture;
