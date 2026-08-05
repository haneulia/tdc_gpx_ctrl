library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_build_pkg.all;
use work.lidar_config_types_pkg.all;
use work.lidar_event_types_pkg.all;
use work.lidar_gpx_event_pkg.all;
use work.lidar_gpx_data_pkg.all;

-- Production B6-B8 assembly. Every runtime-dependent field is sourced from
-- one atomic active configuration image and is never recomputed locally.
entity lidar_gpx_hit_cell_frame_pipeline is
    generic (
        G_BUILD_CONFIG : lidar_build_config_t := C_DEFAULT_BUILD_CONFIG
    );
    port (
        i_clk           : in  std_logic;
        i_rst_n         : in  std_logic;
        i_abort         : in  std_logic;
        i_clear_sticky  : in  std_logic;
        i_active_valid  : in  std_logic;
        i_active_config : in  lidar_active_config_t;

        i_raw_event : in  gpx_raw_event_t;
        o_raw_ready : out std_logic;

        i_face_close_event : in  face_close_event_t;
        o_face_close_ready : out std_logic;

        o_rise_event : out gpx_frame_cell_event_t;
        i_rise_ready : in  std_logic;
        o_fall_event : out gpx_frame_cell_event_t;
        i_fall_ready : in  std_logic;
        o_frame_close_event : out gpx_frame_close_event_t;
        i_frame_close_ready : in  std_logic;

        o_rise_line_done : out std_logic;
        o_fall_line_done : out std_logic;
        o_shot_done : out std_logic;
        o_shot_done_context : out shot_start_event_t;
        o_idle : out std_logic;

        o_hit_fault_pulse  : out gpx_hit_decoder_faults_t;
        o_hit_fault_sticky : out gpx_hit_decoder_faults_t;
        o_cell_fault_pulse  : out gpx_cell_collector_faults_t;
        o_cell_fault_sticky : out gpx_cell_collector_faults_t;
        o_frame_fault_pulse  : out gpx_frame_assembler_faults_t;
        o_frame_fault_sticky : out gpx_frame_assembler_faults_t
    );
end entity lidar_gpx_hit_cell_frame_pipeline;

architecture rtl of lidar_gpx_hit_cell_frame_pipeline is

    signal hit_event_c : gpx_hit_event_t;
    signal hit_ready_c : std_logic;
    signal cell_event_c : gpx_cell_event_t;
    signal cell_ready_c : std_logic;

begin

    assert fn_validate_build_config(G_BUILD_CONFIG) = CFG_OK
        report "V2-B6B8-001 invalid build configuration"
        severity failure;

    u_hit_decoder : entity work.lidar_gpx_hit_decoder
        generic map (
            G_BUILD_CONFIG => G_BUILD_CONFIG
        )
        port map (
            i_clk              => i_clk,
            i_rst_n            => i_rst_n,
            i_abort            => i_abort,
            i_clear_sticky     => i_clear_sticky,
            i_active_rise_mask =>
                i_active_config.derived.active_rise_mask,
            i_active_fall_mask =>
                i_active_config.derived.active_fall_mask,
            i_raw_event        => i_raw_event,
            o_raw_ready        => o_raw_ready,
            o_hit_event        => hit_event_c,
            i_hit_ready        => hit_ready_c,
            o_fault_pulse      => o_hit_fault_pulse,
            o_fault_sticky     => o_hit_fault_sticky
        );

    u_cell_collector : entity work.lidar_gpx_cell_collector
        generic map (
            G_BUILD_CONFIG => G_BUILD_CONFIG
        )
        port map (
            i_clk                  => i_clk,
            i_rst_n                => i_rst_n,
            i_abort                => i_abort,
            i_clear_sticky         => i_clear_sticky,
            i_active_version       => i_active_config.version,
            i_max_hits_per_stop     =>
                i_active_config.source.tdc.max_hits_per_stop,
            i_active_rise_mask      =>
                i_active_config.derived.active_rise_mask,
            i_active_fall_mask      =>
                i_active_config.derived.active_fall_mask,
            i_hit_event             => hit_event_c,
            o_hit_ready             => hit_ready_c,
            o_cell_event            => cell_event_c,
            i_cell_ready            => cell_ready_c,
            o_fault_pulse           => o_cell_fault_pulse,
            o_fault_sticky          => o_cell_fault_sticky
        );

    u_frame_assembler : entity work.lidar_gpx_frame_lane_assembler
        generic map (
            G_BUILD_CONFIG => G_BUILD_CONFIG
        )
        port map (
            i_clk                 => i_clk,
            i_rst_n               => i_rst_n,
            i_abort               => i_abort,
            i_clear_sticky        => i_clear_sticky,
            i_active_version      => i_active_config.version,
            i_active_rise_mask    =>
                i_active_config.derived.active_rise_mask,
            i_active_fall_mask    =>
                i_active_config.derived.active_fall_mask,
            i_columns_per_face    =>
                i_active_config.derived.columns_per_face,
            i_cell_event          => cell_event_c,
            o_cell_ready          => cell_ready_c,
            i_face_close_event    => i_face_close_event,
            o_face_close_ready    => o_face_close_ready,
            o_rise_event          => o_rise_event,
            i_rise_ready          => i_rise_ready,
            o_fall_event          => o_fall_event,
            i_fall_ready          => i_fall_ready,
            o_frame_close_event   => o_frame_close_event,
            i_frame_close_ready   => i_frame_close_ready,
            o_rise_line_done      => o_rise_line_done,
            o_fall_line_done      => o_fall_line_done,
            o_shot_done           => o_shot_done,
            o_shot_done_context   => o_shot_done_context,
            o_idle                => o_idle,
            o_fault_pulse         => o_frame_fault_pulse,
            o_fault_sticky        => o_frame_fault_sticky
        );

    p_contract : process (i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '1' and i_abort = '0' then
                if i_raw_event.valid = '1' or
                   i_face_close_event.valid = '1' then
                    assert i_active_valid = '1'
                        report "V2-B6B8-002 data event without active config"
                        severity failure;
                end if;
            end if;
        end if;
    end process p_contract;

end architecture rtl;
