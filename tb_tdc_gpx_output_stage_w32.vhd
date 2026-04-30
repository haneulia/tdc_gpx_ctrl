library ieee;
use ieee.std_logic_1164.all;

entity tb_tdc_gpx_output_stage_w32 is
end entity;

architecture sim of tb_tdc_gpx_output_stage_w32 is
begin
    u_tb : entity work.tb_tdc_gpx_output_stage
        generic map (
            G_OUTPUT_WIDTH => 32
        );
end architecture;
