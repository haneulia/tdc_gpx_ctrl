-- C08 shared dual-edge geometry regression.
-- Two active chips are included in both slope lane masks. The behavioral
-- source drives chip0 rising and chip1 falling so both lane data paths carry
-- real metadata while sharing the same physical-chip group.

library ieee;
use ieee.std_logic_1164.all;

entity tb_tdc_gpx_top_int_c08_dual_edge_shared is
end entity;

architecture sim of tb_tdc_gpx_top_int_c08_dual_edge_shared is
begin
    u_tb : entity work.tb_tdc_gpx_top_int
        generic map (
            G_AXIS_CLK_MHZ          => 200.0,
            G_TDC_CLK_MHZ           => 200.0,
            G_MAX_RANGE_M           => 1000.0,
            G_TDATA_WIDTH           => 64,
            G_RISE_CHIP_MASK        => "0011",
            G_FALL_CHIP_MASK        => "0011",
            G_STOPS_PER_CHIP        => 8,
            G_COLS_PER_FACE         => 1,
            G_N_FACES               => 1,
            G_ECHOES_PER_STOP       => 7,
            G_MAX_HITS_OVERRIDE     => 7,
            G_MAX_HITS_WRITE_MODE   => 1,
            G_ACTIVE_CHIP_MASK      => "0011",
            G_CHIP_SLOPE_MASK       => "0001",
            G_FORCE_HIT16           => true,
            G_EXPECT_HIT16_META_MIN => 8,
            G_POWERUP_CLKS          => 16,
            G_RECOVERY_CLKS         => 4,
            G_ALU_PULSE_CLKS        => 3,
            G_STREAM_CLK_MODE       => "ASYNC",
            G_BP_TREADY_GAP         => 0,
            G_BP_LANE_MODE          => 0,
            G_RECOVERY_MODE         => 0
        );
end architecture;
