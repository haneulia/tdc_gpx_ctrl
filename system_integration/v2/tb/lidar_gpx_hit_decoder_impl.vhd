library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_build_pkg.all;
use work.lidar_gpx_event_pkg.all;
use work.lidar_gpx_data_pkg.all;

entity lidar_gpx_hit_decoder_impl is
    generic (
        G_PROC_CLK_MHZ : positive := 150
    );
    port (
        i_clk          : in  std_logic;
        i_rst_n        : in  std_logic;
        i_abort        : in  std_logic;
        i_clear_sticky : in  std_logic;
        i_raw_valid    : in  std_logic;
        i_raw_payload  : in  gpx_raw_event_payload_t;
        o_raw_ready    : out std_logic;
        i_hit_ready    : in  std_logic;
        o_hit_valid    : out std_logic;
        o_hit_kind     : out std_logic_vector(1 downto 0);
        o_chip_index   : out std_logic_vector(1 downto 0);
        o_ififo_id     : out std_logic;
        o_channel_code : out std_logic_vector(1 downto 0);
        o_stop_index   : out std_logic_vector(2 downto 0);
        o_start_number : out std_logic_vector(7 downto 0);
        o_slope        : out std_logic;
        o_return_index : out std_logic_vector(2 downto 0);
        o_hit          : out std_logic_vector(16 downto 0);
        o_faulted      : out std_logic;
        o_timeout_cause : out std_logic_vector(2 downto 0);
        o_shot_context : out gpx_shot_context_t;
        o_chip_shot_seq : out std_logic_vector(15 downto 0);
        o_fault_pulse  : out std_logic_vector(3 downto 0);
        o_fault_sticky : out std_logic_vector(3 downto 0)
    );
end entity lidar_gpx_hit_decoder_impl;

architecture rtl of lidar_gpx_hit_decoder_impl is

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

    signal raw_event_c    : gpx_raw_event_t;
    signal hit_event_c    : gpx_hit_event_t;
    signal fault_pulse_c  : gpx_hit_decoder_faults_t;
    signal fault_sticky_c : gpx_hit_decoder_faults_t;

begin

    p_raw_input : process (all)
        variable result : gpx_raw_event_t;
    begin
        result := fn_unpack_raw_event(i_raw_payload);
        result.valid := i_raw_valid;
        raw_event_c <= result;
    end process p_raw_input;

    u_decoder : entity work.lidar_gpx_hit_decoder
        generic map (
            G_BUILD_CONFIG => C_BUILD_CONFIG
        )
        port map (
            i_clk          => i_clk,
            i_rst_n        => i_rst_n,
            i_abort        => i_abort,
            i_clear_sticky => i_clear_sticky,
            i_raw_event    => raw_event_c,
            o_raw_ready    => o_raw_ready,
            o_hit_event    => hit_event_c,
            i_hit_ready    => i_hit_ready,
            o_fault_pulse  => fault_pulse_c,
            o_fault_sticky => fault_sticky_c
        );

    o_hit_valid <= hit_event_c.valid;
    o_hit_kind <= std_logic_vector(to_unsigned(
        gpx_hit_event_kind_t'pos(hit_event_c.kind), o_hit_kind'length));
    o_chip_index <= std_logic_vector(hit_event_c.chip_index);
    o_ififo_id <= hit_event_c.ififo_id;
    o_channel_code <= std_logic_vector(hit_event_c.channel_code);
    o_stop_index <= std_logic_vector(hit_event_c.stop_index);
    o_start_number <= std_logic_vector(hit_event_c.start_number);
    o_slope <= fn_gpx_slope_to_bit(hit_event_c.slope);
    o_return_index <= std_logic_vector(hit_event_c.return_index);
    o_hit <= std_logic_vector(hit_event_c.hit);
    o_faulted <= hit_event_c.faulted;
    o_timeout_cause <= hit_event_c.timeout_cause;
    o_shot_context <= fn_pack_shot_context(hit_event_c.shot_context);
    o_chip_shot_seq <= std_logic_vector(hit_event_c.chip_shot_seq);

    o_fault_pulse <= fault_pulse_c.return_overflow &
                     fault_pulse_c.slope_role_error &
                     fault_pulse_c.stop_index_error &
                     fault_pulse_c.chip_index_error;
    o_fault_sticky <= fault_sticky_c.return_overflow &
                      fault_sticky_c.slope_role_error &
                      fault_sticky_c.stop_index_error &
                      fault_sticky_c.chip_index_error;

end architecture rtl;
