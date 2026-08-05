library ieee;
use ieee.std_logic_1164.all;

use work.lidar_build_pkg.all;
use work.lidar_gpx_event_pkg.all;

-- TDC-to-Processing raw-result gateway. Data and control beats share one
-- ordered stream so IFIFO1 completion can never overtake preceding raw words.
entity lidar_gpx_result_gateway is
    generic (
        G_BUILD_CONFIG : lidar_build_config_t := C_DEFAULT_BUILD_CONFIG;
        G_FIFO_DEPTH   : positive := 16
    );
    port (
        i_tdc_clk        : in  std_logic;
        i_tdc_rst_n      : in  std_logic;
        i_tdc_result     : in  gpx_raw_event_t;
        o_tdc_ready      : out std_logic;

        i_proc_clk       : in  std_logic;
        i_proc_rst_n     : in  std_logic;
        o_proc_result    : out gpx_raw_event_t;
        i_proc_ready     : in  std_logic;

        o_reset_busy     : out std_logic
    );
end entity lidar_gpx_result_gateway;

architecture rtl of lidar_gpx_result_gateway is

    signal source_payload_c : gpx_raw_event_payload_t;
    signal destination_payload_c : gpx_raw_event_payload_t;
    signal destination_valid_c : std_logic;

begin

    assert fn_validate_build_config(G_BUILD_CONFIG) = CFG_OK
        report "V2-GPX-CDC-002 invalid build configuration"
        severity failure;

    source_payload_c <= fn_pack_raw_event(i_tdc_result);

    p_output : process (all)
        variable result : gpx_raw_event_t;
    begin
        result := C_GPX_RAW_EVENT_IDLE;
        if destination_valid_c = '1' then
            result := fn_unpack_raw_event(destination_payload_c);
            result.valid := '1';
        end if;
        o_proc_result <= result;
    end process p_output;

    u_gateway : entity work.lidar_stream_gateway
        generic map (
            G_WIDTH      => C_GPX_RAW_EVENT_PAYLOAD_WIDTH,
            G_FIFO_DEPTH => G_FIFO_DEPTH,
            G_CLOCK_MODE => G_BUILD_CONFIG.stream_clock_mode
        )
        port map (
            i_source_clk        => i_tdc_clk,
            i_source_rst_n      => i_tdc_rst_n,
            i_source_valid      => i_tdc_result.valid,
            o_source_ready      => o_tdc_ready,
            i_source_data       => source_payload_c,
            i_destination_clk   => i_proc_clk,
            i_destination_rst_n => i_proc_rst_n,
            o_destination_valid => destination_valid_c,
            i_destination_ready => i_proc_ready,
            o_destination_data  => destination_payload_c,
            o_reset_busy        => o_reset_busy
        );

end architecture rtl;
