library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_build_pkg.all;
use work.lidar_config_types_pkg.all;
use work.lidar_event_types_pkg.all;

-- B3 assembly boundary. The lifecycle core, low-latency physical bridge,
-- registered pulse bank and observation-only counters remain independently
-- testable and have one owner each.
entity laser_executor is
    generic (
        G_BUILD_CONFIG : lidar_build_config_t := C_DEFAULT_BUILD_CONFIG
    );
    port (
        i_clk                    : in  std_logic;
        i_rst_n                  : in  std_logic;
        i_active_valid           : in  std_logic;
        i_active_config          : in  lidar_active_config_t;
        i_operation_state        : in  operation_state_t;
        i_shot_request           : in  shot_request_t;
        i_fire_done_raw          : in  std_logic;
        i_clear_diagnostics      : in  std_logic;

        o_executor_ready         : out std_logic;
        o_request_accept         : out std_logic;
        o_request_drop           : out std_logic;
        o_fire_pulse             : out std_logic;
        o_start_tdc              : out std_logic;
        o_stop_tdc               : out std_logic;
        o_shot_start             : out shot_start_event_t;
        o_shot_result            : out shot_result_t;
        o_current_request        : out shot_request_t;
        o_busy                   : out std_logic;
        o_physical_arm           : out std_logic;
        o_rearm_active           : out std_logic;
        o_fire_done_sync_clks    : out unsigned(15 downto 0);
        o_rearm_margin_clks      : out unsigned(15 downto 0);
        o_diagnostics            : out laser_diagnostics_t
    );
end entity laser_executor;

architecture rtl of laser_executor is

    signal bridge_ready_c        : std_logic;
    signal physical_start_c      : std_logic;
    signal physical_start_busy_c : std_logic;
    signal bridge_t0_event_c     : std_logic;
    signal unexpected_done_c     : std_logic;

    signal request_accept_c      : std_logic;
    signal request_drop_c        : std_logic;
    signal fire_trigger_c        : std_logic;
    signal simulation_t0_c       : std_logic;
    signal stop_trigger_c        : std_logic;
    signal physical_arm_c        : std_logic;
    signal shot_start_c          : shot_start_event_t;
    signal shot_result_c         : shot_result_t;
    signal current_request_c     : shot_request_t;

    signal fire_pulse_c          : std_logic;
    signal simulation_start_c    : std_logic;
    signal stop_tdc_c            : std_logic;
    signal fire_busy_c           : std_logic;
    signal simulation_start_busy_c : std_logic;
    signal stop_busy_c           : std_logic;
    signal config_ready_c        : std_logic;

begin

    config_ready_c <= '1' when i_active_valid = '1' and
        i_active_config.derived.fire_width_proc_clks /= 0 and
        i_active_config.derived.fire_done_timeout_proc_clks /= 0 and
        i_active_config.derived.target_range_proc_clks /= 0 and
        i_active_config.derived.start_width_proc_clks /= 0 and
        i_active_config.derived.stop_width_proc_clks /= 0 else '0';

    o_request_accept  <= request_accept_c;
    o_request_drop    <= request_drop_c;
    o_fire_pulse      <= fire_pulse_c;
    o_start_tdc       <= physical_start_c or simulation_start_c;
    o_stop_tdc        <= stop_tdc_c;
    o_shot_start      <= shot_start_c;
    o_shot_result     <= shot_result_c;
    o_current_request <= current_request_c;
    o_physical_arm    <= physical_arm_c;

    u_core : entity work.laser_executor_core
        generic map (
            G_BUILD_CONFIG => G_BUILD_CONFIG
        )
        port map (
            i_clk                 => i_clk,
            i_rst_n               => i_rst_n,
            i_config_ready        => config_ready_c,
            i_active_version      => i_active_config.version,
            i_simulation_mode     =>
                i_active_config.source.motor.simulation_mode,
            i_fire_done_timeout_clks =>
                i_active_config.derived.fire_done_timeout_proc_clks,
            i_target_range_clks   =>
                i_active_config.derived.target_range_proc_clks,
            i_sim_start_delay_clks =>
                i_active_config.derived.simulation_start_delay_proc_clks,
            i_physical_fire_enable =>
                i_operation_state.physical_fire_enable,
            i_simulation_enable   =>
                i_operation_state.simulation_enable,
            i_shot_request        => i_shot_request,
            i_bridge_ready        => bridge_ready_c,
            i_t0_event            => bridge_t0_event_c,
            i_physical_start_busy => physical_start_busy_c,
            i_fire_busy           => fire_busy_c,
            i_sim_start_busy      => simulation_start_busy_c,
            i_stop_busy           => stop_busy_c,
            o_executor_ready      => o_executor_ready,
            o_request_accept      => request_accept_c,
            o_request_drop        => request_drop_c,
            o_fire_trigger        => fire_trigger_c,
            o_sim_start_trigger   => simulation_t0_c,
            o_stop_trigger        => stop_trigger_c,
            o_physical_arm        => physical_arm_c,
            o_shot_start          => shot_start_c,
            o_shot_result         => shot_result_c,
            o_current_request     => current_request_c,
            o_busy                => o_busy,
            o_rearm_active        => o_rearm_active,
            o_fire_done_sync_clks => o_fire_done_sync_clks,
            o_rearm_margin_clks   => o_rearm_margin_clks
        );

    u_fire_done_bridge : entity work.laser_fire_done_bridge
        port map (
            i_clk                   => i_clk,
            i_rst_n                 => i_rst_n,
            i_fire_done_raw         => i_fire_done_raw,
            i_physical_arm          => physical_arm_c,
            i_start_width_clks      =>
                i_active_config.derived.start_width_proc_clks,
            o_fire_done_ready       => bridge_ready_c,
            o_start_tdc             => physical_start_c,
            o_start_busy            => physical_start_busy_c,
            o_t0_event              => bridge_t0_event_c,
            o_unexpected_done_pulse => unexpected_done_c
        );

    u_registered_pulses : entity work.laser_registered_pulses
        port map (
            i_clk                  => i_clk,
            i_rst_n                => i_rst_n,
            i_physical_fire_enable =>
                i_operation_state.physical_fire_enable,
            i_fire_trigger         => fire_trigger_c,
            i_sim_start_trigger    => simulation_t0_c,
            i_stop_trigger         => stop_trigger_c,
            i_fire_width_clks      =>
                i_active_config.derived.fire_width_proc_clks,
            i_start_width_clks     =>
                i_active_config.derived.start_width_proc_clks,
            i_stop_width_clks      =>
                i_active_config.derived.stop_width_proc_clks,
            o_fire_pulse           => fire_pulse_c,
            o_sim_start_pulse      => simulation_start_c,
            o_stop_pulse           => stop_tdc_c,
            o_fire_busy            => fire_busy_c,
            o_sim_start_busy       => simulation_start_busy_c,
            o_stop_busy            => stop_busy_c
        );

    u_diagnostics : entity work.laser_diagnostics_counter
        port map (
            i_clk             => i_clk,
            i_rst_n           => i_rst_n,
            i_clear           => i_clear_diagnostics,
            i_request_drop      => request_drop_c,
            i_fire_done_timeout =>
                shot_result_c.valid and shot_result_c.timeout,
            i_operation_abort   =>
                shot_result_c.valid and shot_result_c.aborted,
            i_unexpected_done   => unexpected_done_c,
            o_diagnostics       => o_diagnostics
        );

    p_contract : process (i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '1' then
                assert not (physical_start_c = '1' and
                            simulation_start_c = '1')
                    report "V2-LASER-004 physical and simulation START overlap"
                    severity failure;
                if fire_pulse_c = '1' then
                    assert current_request_c.source_sim = '0' and
                        i_operation_state.physical_fire_enable = '1'
                        report "V2-LASER-009 final fire gate contract failure"
                        severity failure;
                end if;
            end if;
        end if;
    end process p_contract;

end architecture rtl;
