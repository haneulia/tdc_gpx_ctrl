-- =============================================================================
-- Slope-topology integration wrappers for tb_tdc_gpx_top_int.
-- =============================================================================
library ieee;
use ieee.std_logic_1164.all;

use work.tdc_gpx_pkg.all;

entity tb_tdc_gpx_top_int_runtime_rise_only is
end entity;

architecture sim of tb_tdc_gpx_top_int_runtime_rise_only is
begin
    u_tb : entity work.tb_tdc_gpx_top_int
        generic map (
            G_TDATA_WIDTH       => 64,
            G_FALLING_ENABLE    => false,
            G_ACTIVE_CHIP_MASK  => "1111",
            G_CHIP_SLOPE_MASK   => "1111",
            G_STOPS_PER_CHIP    => 2,
            G_COLS_PER_FACE     => 1,
            G_N_FACES           => 1,
            G_ECHOES_PER_STOP   => 1,
            G_MAX_HITS_OVERRIDE => 1,
            G_MAX_HITS_WRITE_MODE => 1,
            G_POWERUP_CLKS      => 16,
            G_RECOVERY_CLKS     => 4,
            G_ALU_PULSE_CLKS    => 3
        );
end architecture;

library ieee;
use ieee.std_logic_1164.all;

use work.tdc_gpx_pkg.all;

entity tb_tdc_gpx_top_int_rise_only_build_32ch is
end entity;

architecture sim of tb_tdc_gpx_top_int_rise_only_build_32ch is
begin
    u_tb : entity work.tb_tdc_gpx_top_int
        generic map (
            G_TDATA_WIDTH       => 32,
            G_RISE_CHIP_MASK    => "1111",
            G_FALL_CHIP_MASK    => "0000",
            G_FALLING_ENABLE    => false,
            G_ACTIVE_CHIP_MASK  => "1111",
            G_CHIP_SLOPE_MASK   => "1111",
            G_STOPS_PER_CHIP    => 8,
            G_COLS_PER_FACE     => 1,
            G_N_FACES           => 1,
            G_ECHOES_PER_STOP   => 1,
            G_MAX_HITS_OVERRIDE => 1,
            G_MAX_HITS_WRITE_MODE => 1,
            G_CHECK_32CH_ORDER  => true,
            G_POWERUP_CLKS      => 16,
            G_RECOVERY_CLKS     => 4,
            G_ALU_PULSE_CLKS    => 3
        );
end architecture;

library ieee;
use ieee.std_logic_1164.all;

use work.tdc_gpx_pkg.all;

entity tb_tdc_gpx_top_int_three_chip_2r1f is
end entity;

architecture sim of tb_tdc_gpx_top_int_three_chip_2r1f is
begin
    u_tb : entity work.tb_tdc_gpx_top_int
        generic map (
            G_TDATA_WIDTH       => 64,
            G_PRESENT_CHIP_MASK => "0111",
            G_RISE_CHIP_MASK    => "0011",
            G_FALL_CHIP_MASK    => "0100",
            G_FALLING_ENABLE    => true,
            G_ACTIVE_CHIP_MASK  => "0111",
            G_CHIP_SLOPE_MASK   => "0011",
            G_STOPS_PER_CHIP    => 2,
            G_COLS_PER_FACE     => 1,
            G_N_FACES           => 1,
            G_ECHOES_PER_STOP   => 1,
            G_MAX_HITS_OVERRIDE => 1,
            G_MAX_HITS_WRITE_MODE => 1,
            G_POWERUP_CLKS      => 16,
            G_RECOVERY_CLKS     => 4,
            G_ALU_PULSE_CLKS    => 3
        );
end architecture;

library ieee;
use ieee.std_logic_1164.all;

use work.tdc_gpx_pkg.all;

entity tb_tdc_gpx_top_int_one_chip_dual_edge is
end entity;

architecture sim of tb_tdc_gpx_top_int_one_chip_dual_edge is
begin
    u_tb : entity work.tb_tdc_gpx_top_int
        generic map (
            G_TDATA_WIDTH       => 64,
            G_PRESENT_CHIP_MASK => "0001",
            G_RISE_CHIP_MASK    => "0001",
            G_FALL_CHIP_MASK    => "0001",
            G_FALLING_ENABLE    => true,
            G_ACTIVE_CHIP_MASK  => "0001",
            G_CHIP_SLOPE_MASK   => "0001",
            G_STOPS_PER_CHIP    => 2,
            G_COLS_PER_FACE     => 1,
            G_N_FACES           => 1,
            G_ECHOES_PER_STOP   => 1,
            G_MAX_HITS_OVERRIDE => 1,
            G_MAX_HITS_WRITE_MODE => 1,
            G_POWERUP_CLKS      => 16,
            G_RECOVERY_CLKS     => 4,
            G_ALU_PULSE_CLKS    => 3
        );
end architecture;
