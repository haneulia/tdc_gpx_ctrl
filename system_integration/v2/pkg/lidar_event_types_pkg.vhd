library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_build_pkg.all;

package lidar_event_types_pkg is

    constant C_POSITION_LATENCY_WIDTH : positive := 8;

    subtype position_latency_t is
        unsigned(C_POSITION_LATENCY_WIDTH - 1 downto 0);

    -- The physical value is counted from the first Processing-clock edge
    -- that samples a stable asynchronous pin value. The virtual value is
    -- counted from the registered A/B/Z transition produced in this domain.
    constant C_POSITION_PHYSICAL_LATENCY_CLKS : position_latency_t :=
        to_unsigned(4, C_POSITION_LATENCY_WIDTH);
    constant C_POSITION_VIRTUAL_LATENCY_CLKS : position_latency_t :=
        to_unsigned(1, C_POSITION_LATENCY_WIDTH);

    constant C_VIRTUAL_Z_FAULT_WIDTH     : positive := 2;
    constant C_VIRTUAL_Z_EARLY_OVER_BIT  : natural := 0;
    constant C_VIRTUAL_Z_COLLISION_BIT   : natural := 1;

    subtype virtual_z_fault_t is
        std_logic_vector(C_VIRTUAL_Z_FAULT_WIDTH - 1 downto 0);

    type position_event_t is record
        valid                : std_logic;
        position             : position_t;
        direction            : direction_t;
        source_sim           : std_logic;
        source_latency_clks  : position_latency_t;
        source_latency_valid : std_logic;
        z_event              : std_logic;
        active_version       : unsigned(15 downto 0);
    end record position_event_t;

    constant C_POSITION_EVENT_IDLE : position_event_t := (
        valid                => '0',
        position             => (others => '0'),
        direction            => DIRECTION_CW,
        source_sim           => '0',
        source_latency_clks  => (others => '0'),
        source_latency_valid => '0',
        z_event              => '0',
        active_version       => (others => '0')
    );

end package lidar_event_types_pkg;
