-- tb_tdc_gpx_top_int_c07_4chip_target.vhd
--
-- C07 target wrapper: 4 GPX chips, 8 stops/chip, 7 echoes/stop.
-- chip0/1 drive rising slope, chip2/3 drive falling slope. Raw Hit[16] is
-- forced high in the behavioral GPX model so final VDMA metadata preservation
-- is directly observable.
--
-- Standard: VHDL-2008
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;

entity tb_tdc_gpx_top_int_c07_4chip_target is
    generic (
        G_TDATA_WIDTH : natural := 64
    );
end entity tb_tdc_gpx_top_int_c07_4chip_target;

architecture sim of tb_tdc_gpx_top_int_c07_4chip_target is
begin

    u_tb : entity work.tb_tdc_gpx_top_int
        generic map (
            G_AXIS_CLK_MHZ             => 200.0,
            G_MAX_RANGE_M              => 1000.0,
            G_TDATA_WIDTH              => G_TDATA_WIDTH,
            G_STOPS_PER_CHIP           => 8,
            G_COLS_PER_FACE            => 1,
            G_N_FACES                  => 1,
            G_ECHOES_PER_STOP          => 7,
            G_MAX_HITS_OVERRIDE        => 7,
            G_MAX_HITS_WRITE_MODE      => 1,
            G_ACTIVE_CHIP_MASK         => "1111",
            G_CHIP_SLOPE_MASK          => "0011",
            G_FORCE_HIT16              => true,
            G_EXPECT_HIT16_META_MIN    => 16,
            G_POWERUP_CLKS             => 16,
            G_RECOVERY_CLKS            => 4,
            G_ALU_PULSE_CLKS           => 3,
            G_STREAM_CLK_MODE          => "ASYNC",
            G_BP_TREADY_GAP            => 0,
            G_BP_LANE_MODE             => 0,
            G_RECOVERY_MODE            => 0
        );

end architecture sim;
