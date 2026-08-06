library ieee;
use ieee.std_logic_1164.all;

use work.lidar_build_pkg.all;
use work.lidar_event_types_pkg.all;
use work.lidar_gpx_event_pkg.all;

-- Processing-to-TDC Shot command gateway. The complete Shot identity is one
-- atomic ready/valid payload; no field is synchronized independently.
entity lidar_gpx_shot_gateway is
    generic (
        G_BUILD_CONFIG : lidar_build_config_t := C_DEFAULT_BUILD_CONFIG;
        G_FIFO_DEPTH   : positive := 16
    );
    port (
        i_proc_clk       : in  std_logic;
        i_proc_rst_n     : in  std_logic;
        i_proc_shot      : in  shot_start_event_t;
        o_proc_ready     : out std_logic;

        i_tdc_clk        : in  std_logic;
        i_tdc_rst_n      : in  std_logic;
        o_tdc_shot       : out shot_start_event_t;
        i_tdc_ready      : in  std_logic;

        o_proc_reset_busy : out std_logic;
        o_tdc_reset_busy  : out std_logic;
        o_reset_busy      : out std_logic
    );
end entity lidar_gpx_shot_gateway;

architecture rtl of lidar_gpx_shot_gateway is

    signal source_payload_c : gpx_shot_start_payload_t;
    signal destination_payload_c : gpx_shot_start_payload_t;
    signal destination_valid_c : std_logic;

begin

    assert fn_validate_build_config(G_BUILD_CONFIG) = CFG_OK
        report "V2-GPX-CDC-001 invalid build configuration"
        severity failure;

    source_payload_c <= fn_pack_shot_start(i_proc_shot);

    p_output : process (all)
        variable result : shot_start_event_t;
    begin
        result := C_SHOT_START_EVENT_IDLE;
        if destination_valid_c = '1' then
            result := fn_unpack_shot_start(destination_payload_c);
            result.valid := '1';
        end if;
        o_tdc_shot <= result;
    end process p_output;

    u_gateway : entity work.lidar_stream_gateway
        generic map (
            G_WIDTH      => C_GPX_SHOT_START_PAYLOAD_WIDTH,
            G_FIFO_DEPTH => G_FIFO_DEPTH,
            G_CLOCK_MODE => G_BUILD_CONFIG.stream_clock_mode
        )
        port map (
            i_source_clk        => i_proc_clk,
            i_source_rst_n      => i_proc_rst_n,
            i_source_valid      => i_proc_shot.valid,
            o_source_ready      => o_proc_ready,
            i_source_data       => source_payload_c,
            i_destination_clk   => i_tdc_clk,
            i_destination_rst_n => i_tdc_rst_n,
            o_destination_valid => destination_valid_c,
            i_destination_ready => i_tdc_ready,
            o_destination_data  => destination_payload_c,
            o_source_reset_busy => o_proc_reset_busy,
            o_destination_reset_busy => o_tdc_reset_busy,
            o_reset_busy        => o_reset_busy
        );

end architecture rtl;
