-- =============================================================================
-- Dedicated-topology elaboration wrapper for the cell-pipe lane-mask TB.
-- =============================================================================
library ieee;
use ieee.std_logic_1164.all;

entity tb_tdc_gpx_cell_pipe_lane_mask_dedicated is
end entity;

architecture sim of tb_tdc_gpx_cell_pipe_lane_mask_dedicated is
begin
    u_tb : entity work.tb_tdc_gpx_cell_pipe_lane_mask
        generic map (
            G_STATIC_RISE_MASK => "0011",
            G_STATIC_FALL_MASK => "1100"
        );
end architecture;
