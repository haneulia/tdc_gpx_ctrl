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
    signal gateway_destination_payload_c : gpx_shot_start_payload_t;
    signal gateway_destination_valid_c : std_logic;
    signal gateway_destination_ready_c : std_logic;
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
            o_destination_valid => gateway_destination_valid_c,
            i_destination_ready => gateway_destination_ready_c,
            o_destination_data  => gateway_destination_payload_c,
            o_source_reset_busy => o_proc_reset_busy,
            o_destination_reset_busy => o_tdc_reset_busy,
            o_reset_busy        => o_reset_busy
        );

    -- TDC-domain의 Shot ready는 활성 Chip 전체의 ready 축약 결과다.
    -- 이를 XPM async FIFO의 BRAM enable에 직접 연결하면 200 MHz 경로가
    -- 길어진다. 2-entry registered-ready skid가 그 경로를 끊고, 이미
    -- CDC를 통과한 Shot을 coordinator가 받을 때까지 원자적으로 보관한다.
    -- Shot 처리량은 1 event/clock을 유지하며 지연만 1 TDC clock 늘어난다.
    u_tdc_output_skid : entity work.tdc_gpx_skid_buffer
        generic map (
            g_DATA_WIDTH => C_GPX_SHOT_START_PAYLOAD_WIDTH
        )
        port map (
            i_clk     => i_tdc_clk,
            i_rst_n   => i_tdc_rst_n,
            i_flush   => '0',
            i_s_valid => gateway_destination_valid_c,
            o_s_ready => gateway_destination_ready_c,
            i_s_data  => gateway_destination_payload_c,
            o_m_valid => destination_valid_c,
            i_m_ready => i_tdc_ready,
            o_m_data  => destination_payload_c
        );

end architecture rtl;
