library ieee;
use ieee.std_logic_1164.all;

use work.lidar_build_pkg.all;
use work.lidar_config_types_pkg.all;
use work.lidar_event_types_pkg.all;
use work.lidar_echo_pkg.all;

-- Stage 4 / Checkpoint G Echo assembly.
--
-- Physical Echo is an ungated IBUFDS-to-GPX STOP path. The optional synthetic
-- source consumes the atomically activated CTL20 CH0/STEP profile. Diagnostic
-- observation is generated only when the Echo receiver is included at build.
entity lidar_echo_subsystem is
    generic (
        G_BUILD_CONFIG : lidar_build_config_t := C_DEFAULT_BUILD_CONFIG
    );
    port (
        i_clk               : in  std_logic;
        i_rst_n             : in  std_logic;
        i_clear_diagnostics : in  std_logic;
        i_active_valid      : in  std_logic;
        i_active_config     : in  lidar_active_config_t;
        i_shot_start        : in  shot_start_event_t;
        i_shot_result       : in  shot_result_t;

        i_pd_lvds_p : in std_logic_vector(
            fn_echo_channel_count(G_BUILD_CONFIG) - 1 downto 0);
        i_pd_lvds_n : in std_logic_vector(
            fn_echo_channel_count(G_BUILD_CONFIG) - 1 downto 0);
        o_tdc_stop : out std_logic_vector(
            fn_echo_channel_count(G_BUILD_CONFIG) - 1 downto 0);

        o_simulation_active : out std_logic;
        o_profile_ready     : out std_logic;
        o_profile_busy      : out std_logic;
        o_profile_version   : out u16_t;
        o_window_active     : out std_logic;
        o_idle              : out std_logic;
        o_diagnostics       : out echo_diagnostics_t
    );
end entity lidar_echo_subsystem;

architecture rtl of lidar_echo_subsystem is

    constant C_NUM_CHANNELS : positive :=
        fn_echo_channel_count(G_BUILD_CONFIG);

    signal active_clocks_c : echo_delay_clocks_array_t;
    signal profile_ready_c : std_logic;
    signal profile_busy_c : std_logic;
    signal profile_version_c : u16_t;

    signal simulation_stop_c : std_logic_vector(
        C_NUM_CHANNELS - 1 downto 0);
    signal simulation_busy_c : std_logic;
    signal simulation_active_c : std_logic;
    signal profile_not_ready_c : std_logic;
    signal diagnostic_rise_c : std_logic_vector(
        C_NUM_CHANNELS - 1 downto 0);
    signal diagnostic_fall_c : std_logic_vector(
        C_NUM_CHANNELS - 1 downto 0);
    signal window_active_c : std_logic;
    signal diagnostics_c : echo_diagnostics_t;

begin

    assert fn_validate_build_config(G_BUILD_CONFIG) = CFG_OK
        report "V2-ECHO-010 invalid subsystem build configuration"
        severity failure;

    o_simulation_active <= simulation_active_c;
    o_profile_ready     <= profile_ready_c;
    o_profile_busy      <= profile_busy_c;
    o_profile_version   <= profile_version_c;
    o_window_active     <= window_active_c;
    o_diagnostics       <= diagnostics_c;
    o_idle <= not window_active_c and not simulation_busy_c and
        not profile_busy_c;

    gen_receiver : if G_BUILD_CONFIG.enable_echo_receiver generate
        u_frontend : entity work.echo_stop_frontend
            generic map (
                G_NUM_CHANNELS      => C_NUM_CHANNELS,
                G_ENABLE_SIMULATION =>
                    G_BUILD_CONFIG.enable_echo_simulation
            )
            port map (
                i_clk               => i_clk,
                i_rst_n             => i_rst_n,
                i_mode_latch        => i_shot_start.valid,
                i_select_simulation => i_shot_start.request.source_sim,
                i_sim_stop          => simulation_stop_c,
                i_pd_lvds_p         => i_pd_lvds_p,
                i_pd_lvds_n         => i_pd_lvds_n,
                o_tdc_stop          => o_tdc_stop,
                o_diag_rise_event   => diagnostic_rise_c,
                o_diag_fall_event   => diagnostic_fall_c,
                o_simulation_active => simulation_active_c
            );

        u_diagnostics : entity work.echo_diagnostics
            generic map (
                G_BUILD_CONFIG => G_BUILD_CONFIG
            )
            port map (
                i_clk               => i_clk,
                i_rst_n             => i_rst_n,
                i_clear             => i_clear_diagnostics,
                i_simulation_active => simulation_active_c,
                i_profile_not_ready => profile_not_ready_c,
                i_shot_start        => i_shot_start,
                i_shot_result       => i_shot_result,
                i_rise_event        => diagnostic_rise_c,
                i_fall_event        => diagnostic_fall_c,
                o_window_active     => window_active_c,
                o_diagnostics       => diagnostics_c
            );
    end generate gen_receiver;

    gen_no_receiver : if not G_BUILD_CONFIG.enable_echo_receiver generate
        o_tdc_stop          <= (others => '0');
        simulation_active_c <= '0';
        window_active_c     <= '0';
        diagnostics_c       <= C_ECHO_DIAGNOSTICS_CLEAR;
    end generate gen_no_receiver;

    gen_simulation : if G_BUILD_CONFIG.enable_echo_simulation generate
        u_profile : entity work.echo_delay_profile
            generic map (
                G_BUILD_CONFIG => G_BUILD_CONFIG
            )
            port map (
                i_clk             => i_clk,
                i_rst_n           => i_rst_n,
                i_active_valid    => i_active_valid,
                i_active_config   => i_active_config,
                o_ready           => profile_ready_c,
                o_busy            => profile_busy_c,
                o_profile_version => profile_version_c,
                o_active_clocks   => active_clocks_c
            );

        u_simulation : entity work.echo_sim_source
            generic map (
                G_BUILD_CONFIG => G_BUILD_CONFIG
            )
            port map (
                i_clk               => i_clk,
                i_rst_n             => i_rst_n,
                i_shot_start        => i_shot_start,
                i_shot_result       => i_shot_result,
                i_profile_ready     => profile_ready_c,
                i_active_clocks     => active_clocks_c,
                o_stop_pulse        => simulation_stop_c,
                o_busy              => simulation_busy_c,
                o_profile_not_ready => profile_not_ready_c
            );
    end generate gen_simulation;

    gen_no_simulation : if not G_BUILD_CONFIG.enable_echo_simulation generate
        active_clocks_c       <= C_ECHO_DELAY_CLOCKS_CLEAR;
        profile_ready_c       <= '1';
        profile_busy_c        <= '0';
        profile_version_c     <= (others => '0');
        simulation_stop_c     <= (others => '0');
        simulation_busy_c     <= '0';
        profile_not_ready_c   <= '0';
    end generate gen_no_simulation;

    p_contract : process (i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '1' then
                if simulation_active_c = '1' then
                    assert G_BUILD_CONFIG.enable_echo_receiver and
                           G_BUILD_CONFIG.enable_echo_simulation
                        report "V2-ECHO-011 simulation active in production"
                        severity failure;
                end if;
                if i_shot_start.valid = '1' then
                    assert i_shot_start.request.valid = '1'
                        report "V2-ECHO-012 invalid F5 Shot context"
                        severity failure;
                end if;
            end if;
        end if;
    end process p_contract;

end architecture rtl;
