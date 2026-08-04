library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package lidar_gpx_pkg is

    constant C_GPX_BUS_DATA_WIDTH : positive := 28;
    constant C_GPX_BUS_ADDR_WIDTH : positive := 4;

    subtype gpx_bus_data_t is
        std_logic_vector(C_GPX_BUS_DATA_WIDTH - 1 downto 0);
    subtype gpx_bus_address_t is
        std_logic_vector(C_GPX_BUS_ADDR_WIDTH - 1 downto 0);

    type gpx_bus_timing_t is record
        clock_div : unsigned(5 downto 0);
        ticks     : unsigned(2 downto 0);
    end record gpx_bus_timing_t;

    type gpx_bus_request_t is record
        valid         : std_logic;
        write         : std_logic;
        address       : gpx_bus_address_t;
        write_data    : gpx_bus_data_t;
        oen_permanent : std_logic;
        burst         : std_logic;
    end record gpx_bus_request_t;

    type gpx_bus_response_t is record
        valid     : std_logic;
        write_ack : std_logic;
        address   : gpx_bus_address_t;
        read_data : gpx_bus_data_t;
    end record gpx_bus_response_t;

    type gpx_pin_status_t is record
        ef1     : std_logic;
        ef2     : std_logic;
        lf1     : std_logic;
        lf2     : std_logic;
        irflag  : std_logic;
        errflag : std_logic;
    end record gpx_pin_status_t;

    constant C_GPX_BUS_TIMING_DEFAULT : gpx_bus_timing_t := (
        clock_div => to_unsigned(2, 6),
        ticks     => to_unsigned(5, 3)
    );

    constant C_GPX_BUS_REQUEST_IDLE : gpx_bus_request_t := (
        valid         => '0',
        write         => '0',
        address       => (others => '0'),
        write_data    => (others => '0'),
        oen_permanent => '0',
        burst         => '0'
    );

    constant C_GPX_BUS_RESPONSE_IDLE : gpx_bus_response_t := (
        valid     => '0',
        write_ack => '0',
        address   => (others => '0'),
        read_data => (others => '0')
    );

    constant C_GPX_PIN_STATUS_RESET : gpx_pin_status_t := (
        ef1     => '1',
        ef2     => '1',
        lf1     => '0',
        lf2     => '0',
        irflag  => '0',
        errflag => '0'
    );

end package lidar_gpx_pkg;
