-- =============================================================================
-- tb_tdc_gpx_top_int_masked_slope_stat.vhd
-- CHAIN P1: STAT7[15] masked_slope_drop_any CSR surfacing check.
--
-- The behavioral chip model emits RISING hits on all four chips while the
-- DUT topology is DEDICATED_2X2 (chips 2/3 serve the fall lane only), so
-- chips 2/3's rising hits are consumed and dropped at the cell_pipe slope
-- demux. The run must still complete with the expected beats/tlast (their
-- fall lane emits blank slices), and the final STAT7 read must return
-- exactly 0x00008000 (masked_slope_drop_any set, everything else clean).
-- =============================================================================
library ieee;
use ieee.std_logic_1164.all;

entity tb_tdc_gpx_top_int_masked_slope_stat is
end entity;

architecture sim of tb_tdc_gpx_top_int_masked_slope_stat is
begin
    u_tb : entity work.tb_tdc_gpx_top_int
        generic map (
            G_RISE_CHIP_MASK           => "0011",
            G_FALL_CHIP_MASK           => "1100",
            G_CHIP_SLOPE_MASK          => "1111",  -- all chips emit RISING hits
            G_EXPECT_MASKED_SLOPE_DROP => true
        );
end architecture;
