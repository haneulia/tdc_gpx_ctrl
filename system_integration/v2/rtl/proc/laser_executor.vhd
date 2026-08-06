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
    signal executor_busy_c       : std_logic;
    signal config_ready_c        : std_logic;
    signal config_valid_r        : std_logic := '0';
    signal timing_valid_r        : std_logic_vector(4 downto 0) :=
        (others => '0');
    signal active_version_r      : u16_t := (others => '0');
    signal simulation_mode_r     : std_logic := '0';
    signal fire_width_r          : u32_t := (others => '0');
    signal fire_done_timeout_r   : u32_t := (others => '0');
    signal target_range_r        : u32_t := (others => '0');
    signal start_width_r         : u32_t := (others => '0');
    signal stop_width_r          : u32_t := (others => '0');
    signal simulation_delay_r    : u32_t := (others => '0');
    signal timestamp_ticks_r     : t0_timestamp_t := (others => '0');
    signal physical_t0_timestamp_c : t0_timestamp_t;
    signal physical_t0_timestamp_valid_c : std_logic;

begin

    p_timestamp : process (i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                timestamp_ticks_r <= (others => '0');
            else
                timestamp_ticks_r <= timestamp_ticks_r + 1;
            end if;
        end if;
    end process p_timestamp;

    -- Active Config is immutable while a Shot is in flight. Register only the
    -- timing fields consumed by B3 every clock, keeping the wide configuration
    -- mailbox out of the 200 MHz pulse-counter and lifecycle paths without an
    -- executor-busy feedback path on the register enables.
    p_config_snapshot : process (i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' or i_active_valid = '0' then
                config_valid_r      <= '0';
                timing_valid_r      <= (others => '0');
                active_version_r    <= (others => '0');
                simulation_mode_r   <= '0';
                fire_width_r        <= (others => '0');
                fire_done_timeout_r <= (others => '0');
                target_range_r      <= (others => '0');
                start_width_r       <= (others => '0');
                stop_width_r        <= (others => '0');
                simulation_delay_r  <= (others => '0');
            else
                config_valid_r <= '1';
                if i_active_config.derived.fire_width_proc_clks /= 0 then
                    timing_valid_r(0) <= '1';
                else
                    timing_valid_r(0) <= '0';
                end if;
                if i_active_config.derived.fire_done_timeout_proc_clks /= 0 then
                    timing_valid_r(1) <= '1';
                else
                    timing_valid_r(1) <= '0';
                end if;
                if i_active_config.derived.target_range_proc_clks /= 0 then
                    timing_valid_r(2) <= '1';
                else
                    timing_valid_r(2) <= '0';
                end if;
                if i_active_config.derived.start_width_proc_clks /= 0 then
                    timing_valid_r(3) <= '1';
                else
                    timing_valid_r(3) <= '0';
                end if;
                if i_active_config.derived.stop_width_proc_clks /= 0 then
                    timing_valid_r(4) <= '1';
                else
                    timing_valid_r(4) <= '0';
                end if;
                active_version_r <= i_active_config.version;
                simulation_mode_r <=
                    i_active_config.source.motor.simulation_mode;
                fire_width_r <=
                    i_active_config.derived.fire_width_proc_clks;
                fire_done_timeout_r <=
                    i_active_config.derived.fire_done_timeout_proc_clks;
                target_range_r <=
                    i_active_config.derived.target_range_proc_clks;
                start_width_r <=
                    i_active_config.derived.start_width_proc_clks;
                stop_width_r <=
                    i_active_config.derived.stop_width_proc_clks;
                simulation_delay_r <=
                    i_active_config.derived.simulation_start_delay_proc_clks;
            end if;
        end if;
    end process p_config_snapshot;

    config_ready_c <= '1' when config_valid_r = '1' and
        timing_valid_r = "11111" else '0';

    o_request_accept  <= request_accept_c;
    o_request_drop    <= request_drop_c;
    o_fire_pulse      <= fire_pulse_c;
    o_start_tdc       <= physical_start_c or simulation_start_c;
    o_stop_tdc        <= stop_tdc_c;
    o_shot_start      <= shot_start_c;
    o_shot_result     <= shot_result_c;
    o_current_request <= current_request_c;
    o_physical_arm    <= physical_arm_c;
    o_busy            <= executor_busy_c;

    u_core : entity work.laser_executor_core
        generic map (
            G_BUILD_CONFIG => G_BUILD_CONFIG
        )
        port map (
            i_clk                 => i_clk,
            i_rst_n               => i_rst_n,
            i_config_ready        => config_ready_c,
            i_active_version      => active_version_r,
            i_simulation_mode     => simulation_mode_r,
            i_fire_done_timeout_clks => fire_done_timeout_r,
            i_target_range_clks   => target_range_r,
            i_sim_start_delay_clks => simulation_delay_r,
            i_physical_fire_enable =>
                i_operation_state.physical_fire_enable,
            i_simulation_enable   =>
                i_operation_state.simulation_enable,
            i_shot_request        => i_shot_request,
            i_bridge_ready        => bridge_ready_c,
            i_t0_event            => bridge_t0_event_c,
            i_timestamp_ticks     => timestamp_ticks_r,
            i_physical_t0_timestamp_ticks => physical_t0_timestamp_c,
            i_physical_t0_timestamp_valid =>
                physical_t0_timestamp_valid_c,
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
            o_busy                => executor_busy_c,
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
            i_start_width_clks      => start_width_r,
            i_timestamp_ticks       => timestamp_ticks_r,
            o_fire_done_ready       => bridge_ready_c,
            o_start_tdc             => physical_start_c,
            o_start_busy            => physical_start_busy_c,
            o_t0_event              => bridge_t0_event_c,
            o_t0_timestamp_ticks    => physical_t0_timestamp_c,
            o_t0_timestamp_valid    => physical_t0_timestamp_valid_c,
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
            i_fire_width_clks      => fire_width_r,
            i_start_width_clks     => start_width_r,
            i_stop_width_clks      => stop_width_r,
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
