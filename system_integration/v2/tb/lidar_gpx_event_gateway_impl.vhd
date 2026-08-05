library ieee;
use ieee.std_logic_1164.all;

use work.lidar_build_pkg.all;
use work.lidar_event_types_pkg.all;
use work.lidar_gpx_event_pkg.all;

-- Scalar-port OOC harness used by the ASYNC and SYNC implementation tops.
entity lidar_gpx_event_gateway_impl is
    generic (
        G_PROC_CLK_MHZ : positive := 150;
        G_TDC_CLK_MHZ  : positive := 200;
        G_CLOCK_MODE   : stream_clock_mode_t := STREAM_CLOCK_ASYNC
    );
    port (
        i_proc_clk       : in  std_logic;
        i_proc_rst_n     : in  std_logic;
        i_tdc_clk        : in  std_logic;
        i_tdc_rst_n      : in  std_logic;

        i_proc_shot_valid : in  std_logic;
        i_proc_shot_data  : in  gpx_shot_start_payload_t;
        o_proc_shot_ready : out std_logic;
        o_tdc_shot_valid  : out std_logic;
        o_tdc_shot_data   : out gpx_shot_start_payload_t;
        i_tdc_shot_ready  : in  std_logic;

        i_tdc_result_valid : in  std_logic;
        i_tdc_result_data  : in  gpx_raw_event_payload_t;
        o_tdc_result_ready : out std_logic;
        o_proc_result_valid : out std_logic;
        o_proc_result_data  : out gpx_raw_event_payload_t;
        i_proc_result_ready : in  std_logic;

        o_reset_busy       : out std_logic
    );
end entity lidar_gpx_event_gateway_impl;

architecture rtl of lidar_gpx_event_gateway_impl is

    function fn_build_config return lidar_build_config_t is
        variable result : lidar_build_config_t := C_DEFAULT_BUILD_CONFIG;
    begin
        result.proc_clk_mhz := G_PROC_CLK_MHZ;
        result.tdc_clk_mhz := G_TDC_CLK_MHZ;
        result.stream_clock_mode := G_CLOCK_MODE;
        return result;
    end function fn_build_config;

    constant C_BUILD_CONFIG : lidar_build_config_t := fn_build_config;

    signal proc_shot_c : shot_start_event_t;
    signal tdc_shot_c : shot_start_event_t;
    signal tdc_result_c : gpx_raw_event_t;
    signal proc_result_c : gpx_raw_event_t;
    signal shot_reset_busy_c : std_logic;
    signal result_reset_busy_c : std_logic;

begin

    p_inputs : process (all)
        variable shot_value : shot_start_event_t;
        variable result_value : gpx_raw_event_t;
    begin
        shot_value := fn_unpack_shot_start(i_proc_shot_data);
        shot_value.valid := i_proc_shot_valid;
        proc_shot_c <= shot_value;

        result_value := fn_unpack_raw_event(i_tdc_result_data);
        result_value.valid := i_tdc_result_valid;
        tdc_result_c <= result_value;
    end process p_inputs;

    o_tdc_shot_valid   <= tdc_shot_c.valid;
    o_tdc_shot_data    <= fn_pack_shot_start(tdc_shot_c);
    o_proc_result_valid <= proc_result_c.valid;
    o_proc_result_data  <= fn_pack_raw_event(proc_result_c);
    o_reset_busy <= shot_reset_busy_c or result_reset_busy_c;

    u_shot_gateway : entity work.lidar_gpx_shot_gateway
        generic map (
            G_BUILD_CONFIG => C_BUILD_CONFIG,
            G_FIFO_DEPTH   => 16
        )
        port map (
            i_proc_clk   => i_proc_clk,
            i_proc_rst_n => i_proc_rst_n,
            i_proc_shot  => proc_shot_c,
            o_proc_ready => o_proc_shot_ready,
            i_tdc_clk    => i_tdc_clk,
            i_tdc_rst_n  => i_tdc_rst_n,
            o_tdc_shot   => tdc_shot_c,
            i_tdc_ready  => i_tdc_shot_ready,
            o_reset_busy => shot_reset_busy_c
        );

    u_result_gateway : entity work.lidar_gpx_result_gateway
        generic map (
            G_BUILD_CONFIG => C_BUILD_CONFIG,
            G_FIFO_DEPTH   => 16
        )
        port map (
            i_tdc_clk     => i_tdc_clk,
            i_tdc_rst_n   => i_tdc_rst_n,
            i_tdc_result  => tdc_result_c,
            o_tdc_ready   => o_tdc_result_ready,
            i_proc_clk    => i_proc_clk,
            i_proc_rst_n  => i_proc_rst_n,
            o_proc_result => proc_result_c,
            i_proc_ready  => i_proc_result_ready,
            o_reset_busy  => result_reset_busy_c
        );

end architecture rtl;

library ieee;
use ieee.std_logic_1164.all;
use work.lidar_gpx_event_pkg.all;

entity lidar_gpx_event_gateway_async_impl is
    generic (
        G_PROC_CLK_MHZ : positive := 150;
        G_TDC_CLK_MHZ  : positive := 200
    );
    port (
        i_proc_clk       : in  std_logic;
        i_proc_rst_n     : in  std_logic;
        i_tdc_clk        : in  std_logic;
        i_tdc_rst_n      : in  std_logic;
        i_proc_shot_valid : in  std_logic;
        i_proc_shot_data  : in  gpx_shot_start_payload_t;
        o_proc_shot_ready : out std_logic;
        o_tdc_shot_valid  : out std_logic;
        o_tdc_shot_data   : out gpx_shot_start_payload_t;
        i_tdc_shot_ready  : in  std_logic;
        i_tdc_result_valid : in  std_logic;
        i_tdc_result_data  : in  gpx_raw_event_payload_t;
        o_tdc_result_ready : out std_logic;
        o_proc_result_valid : out std_logic;
        o_proc_result_data  : out gpx_raw_event_payload_t;
        i_proc_result_ready : in  std_logic;
        o_reset_busy       : out std_logic
    );
end entity lidar_gpx_event_gateway_async_impl;

architecture rtl of lidar_gpx_event_gateway_async_impl is
begin
    u_impl : entity work.lidar_gpx_event_gateway_impl
        generic map (
            G_PROC_CLK_MHZ => G_PROC_CLK_MHZ,
            G_TDC_CLK_MHZ  => G_TDC_CLK_MHZ,
            G_CLOCK_MODE   => work.lidar_build_pkg.STREAM_CLOCK_ASYNC
        )
        port map (
            i_proc_clk => i_proc_clk, i_proc_rst_n => i_proc_rst_n,
            i_tdc_clk => i_tdc_clk, i_tdc_rst_n => i_tdc_rst_n,
            i_proc_shot_valid => i_proc_shot_valid,
            i_proc_shot_data => i_proc_shot_data,
            o_proc_shot_ready => o_proc_shot_ready,
            o_tdc_shot_valid => o_tdc_shot_valid,
            o_tdc_shot_data => o_tdc_shot_data,
            i_tdc_shot_ready => i_tdc_shot_ready,
            i_tdc_result_valid => i_tdc_result_valid,
            i_tdc_result_data => i_tdc_result_data,
            o_tdc_result_ready => o_tdc_result_ready,
            o_proc_result_valid => o_proc_result_valid,
            o_proc_result_data => o_proc_result_data,
            i_proc_result_ready => i_proc_result_ready,
            o_reset_busy => o_reset_busy
        );
end architecture rtl;

library ieee;
use ieee.std_logic_1164.all;
use work.lidar_gpx_event_pkg.all;

entity lidar_gpx_event_gateway_sync_impl is
    generic (
        G_CLK_MHZ : positive := 150
    );
    port (
        i_clk              : in  std_logic;
        i_rst_n            : in  std_logic;
        i_proc_shot_valid  : in  std_logic;
        i_proc_shot_data   : in  gpx_shot_start_payload_t;
        o_proc_shot_ready  : out std_logic;
        o_tdc_shot_valid   : out std_logic;
        o_tdc_shot_data    : out gpx_shot_start_payload_t;
        i_tdc_shot_ready   : in  std_logic;
        i_tdc_result_valid : in  std_logic;
        i_tdc_result_data  : in  gpx_raw_event_payload_t;
        o_tdc_result_ready : out std_logic;
        o_proc_result_valid : out std_logic;
        o_proc_result_data : out gpx_raw_event_payload_t;
        i_proc_result_ready : in  std_logic;
        o_reset_busy       : out std_logic
    );
end entity lidar_gpx_event_gateway_sync_impl;

architecture rtl of lidar_gpx_event_gateway_sync_impl is
begin
    u_impl : entity work.lidar_gpx_event_gateway_impl
        generic map (
            G_PROC_CLK_MHZ => G_CLK_MHZ,
            G_TDC_CLK_MHZ  => G_CLK_MHZ,
            G_CLOCK_MODE   => work.lidar_build_pkg.STREAM_CLOCK_SYNC
        )
        port map (
            i_proc_clk => i_clk, i_proc_rst_n => i_rst_n,
            i_tdc_clk => i_clk, i_tdc_rst_n => i_rst_n,
            i_proc_shot_valid => i_proc_shot_valid,
            i_proc_shot_data => i_proc_shot_data,
            o_proc_shot_ready => o_proc_shot_ready,
            o_tdc_shot_valid => o_tdc_shot_valid,
            o_tdc_shot_data => o_tdc_shot_data,
            i_tdc_shot_ready => i_tdc_shot_ready,
            i_tdc_result_valid => i_tdc_result_valid,
            i_tdc_result_data => i_tdc_result_data,
            o_tdc_result_ready => o_tdc_result_ready,
            o_proc_result_valid => o_proc_result_valid,
            o_proc_result_data => o_proc_result_data,
            i_proc_result_ready => i_proc_result_ready,
            o_reset_busy => o_reset_busy
        );
end architecture rtl;
