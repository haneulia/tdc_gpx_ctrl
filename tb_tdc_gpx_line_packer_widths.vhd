-- Fixed-width wrappers let xsim run the generic line-packer regression
-- without command-line generic quoting differences between host shells.

library ieee;
use ieee.std_logic_1164.all;

entity tb_tdc_gpx_line_packer_w32 is
end entity;

architecture sim of tb_tdc_gpx_line_packer_w32 is
begin
    u_tb : entity work.tb_tdc_gpx_line_packer
        generic map (G_WIDTH => 32, G_MAX_HITS => 7);
end architecture;

library ieee;
use ieee.std_logic_1164.all;

entity tb_tdc_gpx_line_packer_w64 is
end entity;

architecture sim of tb_tdc_gpx_line_packer_w64 is
begin
    u_tb : entity work.tb_tdc_gpx_line_packer
        generic map (G_WIDTH => 64, G_MAX_HITS => 7);
end architecture;

library ieee;
use ieee.std_logic_1164.all;

entity tb_tdc_gpx_line_packer_w128 is
end entity;

architecture sim of tb_tdc_gpx_line_packer_w128 is
begin
    u_tb : entity work.tb_tdc_gpx_line_packer
        generic map (G_WIDTH => 128, G_MAX_HITS => 7);
end architecture;

library ieee;
use ieee.std_logic_1164.all;

entity tb_tdc_gpx_line_packer_w32_mh3 is
end entity;

architecture sim of tb_tdc_gpx_line_packer_w32_mh3 is
begin
    u_tb : entity work.tb_tdc_gpx_line_packer
        generic map (G_WIDTH => 32, G_MAX_HITS => 3);
end architecture;

library ieee;
use ieee.std_logic_1164.all;

entity tb_tdc_gpx_line_packer_w64_mh3 is
end entity;

architecture sim of tb_tdc_gpx_line_packer_w64_mh3 is
begin
    u_tb : entity work.tb_tdc_gpx_line_packer
        generic map (G_WIDTH => 64, G_MAX_HITS => 3);
end architecture;

library ieee;
use ieee.std_logic_1164.all;

entity tb_tdc_gpx_line_packer_w128_mh3 is
end entity;

architecture sim of tb_tdc_gpx_line_packer_w128_mh3 is
begin
    u_tb : entity work.tb_tdc_gpx_line_packer
        generic map (G_WIDTH => 128, G_MAX_HITS => 3);
end architecture;
