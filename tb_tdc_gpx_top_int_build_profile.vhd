-- Non-default synthesis-profile integration regression.
--
-- The fixed external ABI still exposes four chip slots, while this build
-- implements sparse chips 0 and 2, at most four stops/chip, and three
-- hits/stop. Software deliberately requests all chips, eight stops, and
-- seven hits so the test proves that CSR, header/data geometry, and both
-- output lanes use the same clamped build profile.

library ieee;
use ieee.std_logic_1164.all;

entity tb_tdc_gpx_top_int_build_profile is
end entity;

architecture sim of tb_tdc_gpx_top_int_build_profile is
begin
    u_tb : entity work.tb_tdc_gpx_top_int
        generic map (
            G_AXIS_CLK_MHZ              => 200.0,
            G_TDC_CLK_MHZ               => 200.0,
            G_MAX_RANGE_M               => 1000.0,
            G_TDATA_WIDTH               => 64,
            G_RISE_CHIP_MASK            => "0101",
            G_FALL_CHIP_MASK            => "0101",
            G_PRESENT_CHIP_MASK         => "0101",
            G_BUILD_MAX_STOPS_PER_CHIP  => 4,
            G_BUILD_MAX_HITS_PER_STOP   => 3,
            G_STOPS_PER_CHIP            => 8,
            G_COLS_PER_FACE             => 1,
            G_N_FACES                   => 1,
            G_ECHOES_PER_STOP           => 3,
            G_MAX_HITS_OVERRIDE         => 7,
            G_MAX_HITS_WRITE_MODE       => 1,
            G_ACTIVE_CHIP_MASK          => "1111",
            G_CHIP_SLOPE_MASK           => "0001",
            G_POWERUP_CLKS              => 16,
            G_RECOVERY_CLKS             => 4,
            G_ALU_PULSE_CLKS            => 3,
            G_STREAM_CLK_MODE           => "ASYNC",
            G_BP_TREADY_GAP             => 0,
            G_BP_LANE_MODE              => 0,
            G_RECOVERY_MODE             => 0
        );
end architecture;

-- Empty-intersection fallback and zero-alias profile regression. The SW chip
-- request selects only absent slots, so the effective mask must become chip 0,
-- the lowest-index slot in g_PRESENT_CHIP_MASK. CTL21 max_hits=0 must alias to
-- the build maximum of three rather than the package ABI ceiling of seven.

library ieee;
use ieee.std_logic_1164.all;

entity tb_tdc_gpx_top_int_build_profile_zero_alias is
end entity;

architecture sim of tb_tdc_gpx_top_int_build_profile_zero_alias is
begin
    u_tb : entity work.tb_tdc_gpx_top_int
        generic map (
            G_AXIS_CLK_MHZ              => 200.0,
            G_TDC_CLK_MHZ               => 200.0,
            G_MAX_RANGE_M               => 1000.0,
            G_TDATA_WIDTH               => 64,
            G_RISE_CHIP_MASK            => "0101",
            G_FALL_CHIP_MASK            => "0101",
            G_PRESENT_CHIP_MASK         => "0101",
            G_BUILD_MAX_STOPS_PER_CHIP  => 4,
            G_BUILD_MAX_HITS_PER_STOP   => 3,
            G_STOPS_PER_CHIP            => 8,
            G_COLS_PER_FACE             => 1,
            G_N_FACES                   => 1,
            G_ECHOES_PER_STOP           => 3,
            G_MAX_HITS_OVERRIDE         => 7,
            G_MAX_HITS_WRITE_MODE       => 2,
            G_ACTIVE_CHIP_MASK          => "1010",
            G_CHIP_SLOPE_MASK           => "0001",
            G_POWERUP_CLKS              => 16,
            G_RECOVERY_CLKS             => 4,
            G_ALU_PULSE_CLKS            => 3,
            G_STREAM_CLK_MODE           => "ASYNC",
            G_BP_TREADY_GAP             => 0,
            G_BP_LANE_MODE              => 0,
            G_RECOVERY_MODE             => 0
        );
end architecture;
