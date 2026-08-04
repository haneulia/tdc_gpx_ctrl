library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_gpx_pkg.all;

-- Scalar-port OOC implementation harness for the Stage-H GPX bus boundary.
entity lidar_gpx_bus_engine_impl is
    generic (
        G_TDC_CLK_MHZ : positive := 200;
        G_OEN_MODE    : string := "DYNAMIC_CONNECTED"
    );
    port (
        i_clk           : in  std_logic;
        i_rst_n         : in  std_logic;
        i_tick_en       : in  std_logic;
        i_bus_clk_div   : in  std_logic_vector(5 downto 0);
        i_bus_ticks     : in  std_logic_vector(2 downto 0);
        i_request_valid : in  std_logic;
        i_request_write : in  std_logic;
        i_request_addr  : in  std_logic_vector(3 downto 0);
        i_request_data  : in  std_logic_vector(27 downto 0);
        i_oen_permanent : in  std_logic;
        i_request_burst : in  std_logic;
        i_response_ready : in std_logic;
        i_d             : in  std_logic_vector(27 downto 0);
        i_ef1           : in  std_logic;
        i_ef2           : in  std_logic;
        i_lf1           : in  std_logic;
        i_lf2           : in  std_logic;
        i_irflag        : in  std_logic;
        i_errflag       : in  std_logic;
        o_busy          : out std_logic;
        o_response_pending : out std_logic;
        o_adr           : out std_logic_vector(3 downto 0);
        o_csn           : out std_logic;
        o_rdn           : out std_logic;
        o_wrn           : out std_logic;
        o_oen           : out std_logic;
        o_d             : out std_logic_vector(27 downto 0);
        o_d_tri         : out std_logic;
        o_response_valid : out std_logic;
        o_response_write_ack : out std_logic;
        o_response_addr : out std_logic_vector(3 downto 0);
        o_response_data : out std_logic_vector(27 downto 0);
        o_ef1           : out std_logic;
        o_ef2           : out std_logic;
        o_lf1           : out std_logic;
        o_lf2           : out std_logic;
        o_irflag        : out std_logic;
        o_errflag       : out std_logic;
        o_effective_ticks : out std_logic_vector(2 downto 0)
    );
end entity lidar_gpx_bus_engine_impl;

architecture rtl of lidar_gpx_bus_engine_impl is

    signal timing_c : gpx_bus_timing_t;
    signal request_c : gpx_bus_request_t;
    signal response_c : gpx_bus_response_t;
    signal status_c : gpx_pin_status_t;
    signal effective_ticks_c : unsigned(2 downto 0);

begin

    timing_c <= (
        clock_div => unsigned(i_bus_clk_div),
        ticks     => unsigned(i_bus_ticks)
    );
    request_c <= (
        valid         => i_request_valid,
        write         => i_request_write,
        address       => i_request_addr,
        write_data    => i_request_data,
        oen_permanent => i_oen_permanent,
        burst         => i_request_burst
    );

    o_response_valid     <= response_c.valid;
    o_response_write_ack <= response_c.write_ack;
    o_response_addr      <= response_c.address;
    o_response_data      <= response_c.read_data;
    o_ef1                <= status_c.ef1;
    o_ef2                <= status_c.ef2;
    o_lf1                <= status_c.lf1;
    o_lf2                <= status_c.lf2;
    o_irflag             <= status_c.irflag;
    o_errflag            <= status_c.errflag;
    o_effective_ticks    <= std_logic_vector(effective_ticks_c);

    u_dut : entity work.lidar_gpx_bus_engine
        generic map (
            G_TDC_CLK_MHZ => G_TDC_CLK_MHZ,
            G_OEN_MODE    => G_OEN_MODE
        )
        port map (
            i_clk              => i_clk,
            i_rst_n            => i_rst_n,
            i_tick_en          => i_tick_en,
            i_timing           => timing_c,
            i_request          => request_c,
            o_busy             => o_busy,
            o_response_pending => o_response_pending,
            o_adr              => o_adr,
            o_csn              => o_csn,
            o_rdn              => o_rdn,
            o_wrn              => o_wrn,
            o_oen              => o_oen,
            i_d                => i_d,
            o_d                => o_d,
            o_d_tri            => o_d_tri,
            i_ef1              => i_ef1,
            i_ef2              => i_ef2,
            i_lf1              => i_lf1,
            i_lf2              => i_lf2,
            i_irflag           => i_irflag,
            i_errflag          => i_errflag,
            o_response         => response_c,
            i_response_ready   => i_response_ready,
            o_status           => status_c,
            o_effective_ticks  => effective_ticks_c
        );

end architecture rtl;
