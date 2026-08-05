library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_build_pkg.all;
use work.lidar_gpx_event_pkg.all;
use work.lidar_gpx_data_pkg.all;

-- OOC timing harness for the registered B6-to-B7 boundary. Record ports keep
-- this harness structural; Vivado flattens them only at the temporary top.
entity lidar_gpx_hit_cell_pipeline_impl is
    generic (
        G_PROC_CLK_MHZ : positive := 150
    );
    port (
        i_clk          : in  std_logic;
        i_rst_n        : in  std_logic;
        i_abort        : in  std_logic;
        i_clear_sticky : in  std_logic;
        i_active_version : in unsigned(15 downto 0);
        i_max_hits_per_stop : in unsigned(2 downto 0);

        i_raw_event : in  gpx_raw_event_t;
        o_raw_ready : out std_logic;

        o_cell_event : out gpx_cell_event_t;
        i_cell_ready : in  std_logic;

        o_decoder_fault_pulse  : out gpx_hit_decoder_faults_t;
        o_decoder_fault_sticky : out gpx_hit_decoder_faults_t;
        o_collector_fault_pulse  : out gpx_cell_collector_faults_t;
        o_collector_fault_sticky : out gpx_cell_collector_faults_t
    );
end entity lidar_gpx_hit_cell_pipeline_impl;

architecture rtl of lidar_gpx_hit_cell_pipeline_impl is

    constant C_BUILD_CONFIG : lidar_build_config_t := (
        proc_clk_mhz           => G_PROC_CLK_MHZ,
        tdc_clk_mhz            => 200,
        stream_clock_mode      => STREAM_CLOCK_ASYNC,
        num_chips              => 4,
        stops_per_chip         => 8,
        max_returns_per_stop   => 7,
        rise_capability_mask   => "0011",
        fall_capability_mask   => "1100",
        output_width           => 32,
        num_faces              => 4,
        enable_echo_receiver   => false,
        enable_echo_simulation => false
    );

    signal hit_event_c : gpx_hit_event_t;
    signal hit_ready_c : std_logic;

begin

    u_decoder : entity work.lidar_gpx_hit_decoder
        generic map (
            G_BUILD_CONFIG => C_BUILD_CONFIG
        )
        port map (
            i_clk          => i_clk,
            i_rst_n        => i_rst_n,
            i_abort        => i_abort,
            i_clear_sticky => i_clear_sticky,
            i_raw_event    => i_raw_event,
            o_raw_ready    => o_raw_ready,
            o_hit_event    => hit_event_c,
            i_hit_ready    => hit_ready_c,
            o_fault_pulse  => o_decoder_fault_pulse,
            o_fault_sticky => o_decoder_fault_sticky
        );

    u_collector : entity work.lidar_gpx_cell_collector
        generic map (
            G_BUILD_CONFIG => C_BUILD_CONFIG
        )
        port map (
            i_clk               => i_clk,
            i_rst_n             => i_rst_n,
            i_abort             => i_abort,
            i_clear_sticky      => i_clear_sticky,
            i_active_version    => i_active_version,
            i_max_hits_per_stop => i_max_hits_per_stop,
            i_hit_event         => hit_event_c,
            o_hit_ready         => hit_ready_c,
            o_cell_event        => o_cell_event,
            i_cell_ready        => i_cell_ready,
            o_fault_pulse       => o_collector_fault_pulse,
            o_fault_sticky      => o_collector_fault_sticky
        );

end architecture rtl;
