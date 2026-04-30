-- ============================================================================
-- Testbench wrapper: tb_tdc_gpx_distance_maxhits_matrix_100m_10m
--
-- Purpose:
--   Reuse tb_tdc_gpx_distance_maxhits_matrix with the user-requested sweep:
--   start at 100 m and increase distance by 10 m after each timing fail.
-- ============================================================================

library ieee;
use ieee.std_logic_1164.all;

entity tb_tdc_gpx_distance_maxhits_matrix_100m_10m is
end entity;

architecture sim of tb_tdc_gpx_distance_maxhits_matrix_100m_10m is
begin

    u_sweep : entity work.tb_tdc_gpx_distance_maxhits_matrix
        generic map (
            g_START_DISTANCE_M => 100,
            g_DISTANCE_STEP_M  => 10
        );

end architecture;
