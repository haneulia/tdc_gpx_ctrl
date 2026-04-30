library ieee;
use ieee.std_logic_1164.all;

entity tb_tdc_gpx_output_stage_maxhits_w64 is
end entity;

architecture sim of tb_tdc_gpx_output_stage_maxhits_w64 is
begin
    u_tb : entity work.tb_tdc_gpx_output_stage
        generic map (
            G_OUTPUT_WIDTH        => 64,
            G_RUN_MAX_HITS_SWEEP => true
        );
end architecture;
