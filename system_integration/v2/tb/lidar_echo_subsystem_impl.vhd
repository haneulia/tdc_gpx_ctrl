library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_build_pkg.all;
use work.lidar_config_types_pkg.all;
use work.lidar_event_types_pkg.all;
use work.lidar_echo_pkg.all;

-- Scalar-port OOC implementation harness for the Stage-G Echo subsystem.
entity lidar_echo_subsystem_impl is
    generic (
        G_PROC_CLK_MHZ      : positive := 150;
        G_TDC_CLK_MHZ       : positive := 200;
        G_NUM_CHIPS         : positive range 1 to 4 := 2;
        G_ENABLE_RECEIVER   : boolean := true;
        G_ENABLE_SIMULATION : boolean := false
    );
    port (
        i_clk                : in  std_logic;
        i_rst_n              : in  std_logic;
        i_clear_diagnostics  : in  std_logic;
        i_active_valid       : in  std_logic;
        i_active_version     : in  std_logic_vector(15 downto 0);
        i_channel_0_delay_5ns : in std_logic_vector(15 downto 0);
        i_channel_step_5ns    : in std_logic_vector(15 downto 0);
        i_shot_start_valid   : in  std_logic;
        i_shot_source_sim    : in  std_logic;
        i_shot_result_valid  : in  std_logic;
        i_pd_lvds_p          : in  std_logic_vector(
            G_NUM_CHIPS * 8 - 1 downto 0);
        i_pd_lvds_n          : in  std_logic_vector(
            G_NUM_CHIPS * 8 - 1 downto 0);
        o_tdc_stop           : out std_logic_vector(
            G_NUM_CHIPS * 8 - 1 downto 0);
        o_simulation_active  : out std_logic;
        o_profile_ready      : out std_logic;
        o_profile_busy       : out std_logic;
        o_window_active      : out std_logic;
        o_idle               : out std_logic;
        o_diag_total_rise    : out std_logic_vector(15 downto 0);
        o_diag_not_ready     : out std_logic
    );
end entity lidar_echo_subsystem_impl;

architecture rtl of lidar_echo_subsystem_impl is

    constant C_BUILD_CONFIG : lidar_build_config_t := (
        proc_clk_mhz            => G_PROC_CLK_MHZ,
        tdc_clk_mhz             => G_TDC_CLK_MHZ,
        stream_clock_mode       => STREAM_CLOCK_ASYNC,
        num_chips               => G_NUM_CHIPS,
        stops_per_chip          => 8,
        max_returns_per_stop    => 7,
        rise_capability_mask    => fn_present_chip_mask(G_NUM_CHIPS),
        fall_capability_mask    => (others => '0'),
        output_width            => 32,
        num_faces               => 5,
        enable_echo_receiver    => G_ENABLE_RECEIVER,
        enable_echo_simulation  => G_ENABLE_SIMULATION
    );
    constant C_ZERO_DERIVED : lidar_derived_config_t := (
        total_states                    => (others => '0'),
        virtual_ticks_hi                => (others => '0'),
        mechanical_angle_per_state_udeg => (others => '0'),
        optical_angle_per_state_udeg    => (others => '0'),
        face_lower                      => (others => (others => '0')),
        face_upper                      => (others => (others => '0')),
        face_active_positions           => (others => '0'),
        face_angular_intervals          => (others => '0'),
        shot_interval_states            => (others => '0'),
        columns_per_face                => (others => '0'),
        present_chip_mask               => (others => '0'),
        active_rise_mask                => (others => '0'),
        active_fall_mask                => (others => '0'),
        fire_width_proc_clks            => (others => '0'),
        fire_done_timeout_proc_clks     => (others => '0'),
        target_range_proc_clks          => (others => '0'),
        start_width_proc_clks           => (others => '0'),
        stop_width_proc_clks            => (others => '0'),
        simulation_start_delay_proc_clks => (others => '0'),
        capture_window_5ns              => (others => '0'),
        capture_window_tdc_clks         => (others => '0'),
        scan_timeout_tdc_clks           => (others => '0')
    );

    signal active_config_c : lidar_active_config_t;
    signal shot_start_c : shot_start_event_t;
    signal shot_result_c : shot_result_t;
    signal diagnostics_c : echo_diagnostics_t;

begin

    p_inputs : process (all)
        variable source_v : lidar_runtime_config_t;
        variable request_v : shot_request_t;
    begin
        source_v := fn_default_runtime_config(C_BUILD_CONFIG);
        source_v.echo.channel_0_delay_5ns :=
            unsigned(i_channel_0_delay_5ns);
        source_v.echo.channel_step_5ns := unsigned(i_channel_step_5ns);
        active_config_c <= (
            version => unsigned(i_active_version),
            source  => source_v,
            derived => C_ZERO_DERIVED
        );

        request_v := C_SHOT_REQUEST_IDLE;
        request_v.valid          := i_shot_start_valid or
            i_shot_result_valid;
        request_v.source_sim     := i_shot_source_sim;
        request_v.active_version := unsigned(i_active_version);
        shot_start_c <= (
            valid            => i_shot_start_valid,
            request          => request_v,
            fire_to_t0_clks  => (others => '0')
        );
        shot_result_c <= (
            valid            => i_shot_result_valid,
            timeout          => '0',
            aborted          => '0',
            request          => request_v,
            fire_to_t0_clks  => (others => '0')
        );
    end process p_inputs;

    o_diag_total_rise <= std_logic_vector(diagnostics_c.snapshot.total_rise);
    o_diag_not_ready <= diagnostics_c.profile_not_ready_sticky;

    u_dut : entity work.lidar_echo_subsystem
        generic map (
            G_BUILD_CONFIG => C_BUILD_CONFIG
        )
        port map (
            i_clk               => i_clk,
            i_rst_n             => i_rst_n,
            i_clear_diagnostics => i_clear_diagnostics,
            i_active_valid      => i_active_valid,
            i_active_config     => active_config_c,
            i_shot_start        => shot_start_c,
            i_shot_result       => shot_result_c,
            i_pd_lvds_p         => i_pd_lvds_p,
            i_pd_lvds_n         => i_pd_lvds_n,
            o_tdc_stop          => o_tdc_stop,
            o_simulation_active => o_simulation_active,
            o_profile_ready     => o_profile_ready,
            o_profile_busy      => o_profile_busy,
            o_profile_version   => open,
            o_window_active     => o_window_active,
            o_idle              => o_idle,
            o_diagnostics       => diagnostics_c
        );

end architecture rtl;
