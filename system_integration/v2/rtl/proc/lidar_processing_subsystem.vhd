library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_build_pkg.all;
use work.lidar_config_types_pkg.all;
use work.lidar_event_types_pkg.all;
use work.lidar_processing_pkg.all;

-- Production F5 assembly of the direct registered Processing path.
--
-- The atomic configuration subsystem and operation manager remain the sole
-- owners of configuration and safety state. This boundary consumes their
-- typed outputs once, connects B0 through B3 directly, and exposes a lossy
-- observation-only AXIS copy. Monitor backpressure cannot reach shot control.
entity lidar_processing_subsystem is
    generic (
        G_BUILD_CONFIG : lidar_build_config_t := C_DEFAULT_BUILD_CONFIG
    );
    port (
        i_clk                : in  std_logic;
        i_rst_n              : in  std_logic;
        i_active_valid       : in  std_logic;
        i_active_config      : in  lidar_active_config_t;
        i_operation_state    : in  operation_state_t;
        i_enc_a              : in  std_logic;
        i_enc_b              : in  std_logic;
        i_enc_z              : in  std_logic;
        i_fire_done_raw      : in  std_logic;
        i_clear_diagnostics  : in  std_logic;

        m_mon_axis_tready    : in  std_logic;
        m_mon_axis_tvalid    : out std_logic;
        m_mon_axis_tdata     : out processing_monitor_tdata_t;
        m_mon_axis_tkeep     : out processing_monitor_tkeep_t;
        m_mon_axis_tuser     : out processing_monitor_tuser_t;
        m_mon_axis_tlast     : out std_logic;

        o_fire_pulse         : out std_logic;
        o_start_tdc          : out std_logic;
        o_stop_tdc           : out std_logic;
        o_position_event     : out position_event_t;
        o_face_event         : out face_event_t;
        o_shot_request       : out shot_request_t;
        o_shot_start         : out shot_start_event_t;
        o_shot_result        : out shot_result_t;
        o_current_request    : out shot_request_t;
        o_current_position   : out position_t;
        o_current_direction  : out direction_t;
        o_executor_ready     : out std_logic;
        o_request_accept     : out std_logic;
        o_request_drop       : out std_logic;
        o_executor_busy      : out std_logic;
        o_physical_arm       : out std_logic;
        o_rearm_active       : out std_logic;
        o_pipeline_idle      : out std_logic;
        o_virtual_a          : out std_logic;
        o_virtual_b          : out std_logic;
        o_virtual_z          : out std_logic;
        o_b0_to_accept_clks  : out processing_latency_t;
        o_physical_to_fire_clks : out processing_latency_t;
        o_virtual_to_accept_clks : out processing_latency_t;
        o_fire_done_sync_clks : out unsigned(15 downto 0);
        o_rearm_margin_clks   : out unsigned(15 downto 0);
        o_diagnostics         : out processing_diagnostics_t
    );
end entity lidar_processing_subsystem;

architecture rtl of lidar_processing_subsystem is

    signal position_event_c    : position_event_t;
    signal face_event_c        : face_event_t;
    signal shot_request_c      : shot_request_t;
    signal shot_start_c        : shot_start_event_t;
    signal shot_result_c       : shot_result_t;
    signal current_request_c   : shot_request_t;
    signal current_position_c  : position_t;
    signal current_direction_c : direction_t;

    signal invalid_transition_c        : std_logic;
    signal invalid_transition_sticky_c : std_logic;
    signal invalid_transition_count_c  : u32_t;
    signal source_switch_c             : std_logic;
    signal virtual_z_fault_c           : virtual_z_fault_t;
    signal face_overlap_sticky_c       : std_logic;
    signal face_overlap_count_c        : u32_t;
    signal schedule_overrun_pulse_c    : std_logic;
    signal schedule_overrun_sticky_c   : std_logic;
    signal schedule_overrun_count_c    : u32_t;
    signal laser_diagnostics_c         : laser_diagnostics_t;
    signal monitor_drop_pulse_c        : std_logic;
    signal monitor_drop_sticky_c       : std_logic;
    signal monitor_drop_count_c        : u32_t;

    signal executor_ready_c  : std_logic;
    signal request_accept_c  : std_logic;
    signal request_drop_c    : std_logic;
    signal executor_busy_c   : std_logic;
    signal physical_arm_c    : std_logic;
    signal rearm_active_c    : std_logic;
    signal scheduler_idle_c  : std_logic;
    signal pipeline_idle_c   : std_logic;

begin

    assert fn_validate_build_config(G_BUILD_CONFIG) = CFG_OK
        report "V2-PROC-001 invalid build configuration"
        severity failure;

    pipeline_idle_c <= scheduler_idle_c and not executor_busy_c;

    o_position_event    <= position_event_c;
    o_face_event        <= face_event_c;
    o_shot_request      <= shot_request_c;
    o_shot_start        <= shot_start_c;
    o_shot_result       <= shot_result_c;
    o_current_request   <= current_request_c;
    o_current_position  <= current_position_c;
    o_current_direction <= current_direction_c;
    o_executor_ready    <= executor_ready_c;
    o_request_accept    <= request_accept_c;
    o_request_drop      <= request_drop_c;
    o_executor_busy     <= executor_busy_c;
    o_physical_arm      <= physical_arm_c;
    o_rearm_active      <= rearm_active_c;
    o_pipeline_idle     <= pipeline_idle_c;

    o_b0_to_accept_clks <= C_B0_TO_EXECUTOR_ACCEPT_CLKS;
    o_physical_to_fire_clks <= C_PHYSICAL_SAMPLE_TO_FIRE_CLKS;
    o_virtual_to_accept_clks <= C_VIRTUAL_TRANSITION_TO_ACCEPT_CLKS;

    o_diagnostics <= (
        invalid_transition_pulse  => invalid_transition_c,
        invalid_transition_sticky => invalid_transition_sticky_c,
        invalid_transition_count  => invalid_transition_count_c,
        source_switch_pulse        => source_switch_c,
        virtual_z_fault            => virtual_z_fault_c,
        face_overlap_sticky        => face_overlap_sticky_c,
        face_overlap_count         => face_overlap_count_c,
        schedule_overrun_pulse     => schedule_overrun_pulse_c,
        schedule_overrun_sticky    => schedule_overrun_sticky_c,
        schedule_overrun_count     => schedule_overrun_count_c,
        laser                      => laser_diagnostics_c,
        monitor_drop_pulse         => monitor_drop_pulse_c,
        monitor_drop_sticky        => monitor_drop_sticky_c,
        monitor_drop_count         => monitor_drop_count_c
    );

    u_motor : entity work.motor_position_core
        port map (
            i_clk                       => i_clk,
            i_rst_n                     => i_rst_n,
            i_enable                    =>
                i_operation_state.processing_enable,
            i_active_valid              => i_active_valid,
            i_active_config             => i_active_config,
            i_enc_a                     => i_enc_a,
            i_enc_b                     => i_enc_b,
            i_enc_z                     => i_enc_z,
            i_clear_diagnostics         => i_clear_diagnostics,
            o_position_event            => position_event_c,
            o_current_position          => current_position_c,
            o_current_direction         => current_direction_c,
            o_invalid_transition        => invalid_transition_c,
            o_invalid_transition_sticky =>
                invalid_transition_sticky_c,
            o_invalid_transition_count  => invalid_transition_count_c,
            o_source_switch             => source_switch_c,
            o_virtual_a                 => o_virtual_a,
            o_virtual_b                 => o_virtual_b,
            o_virtual_z                 => o_virtual_z,
            o_virtual_z_fault           => virtual_z_fault_c
        );

    u_face : entity work.face_tracker
        generic map (
            G_BUILD_CONFIG => G_BUILD_CONFIG
        )
        port map (
            i_clk               => i_clk,
            i_rst_n             => i_rst_n,
            i_enable            =>
                i_operation_state.processing_enable,
            i_active_valid      => i_active_valid,
            i_active_config     => i_active_config,
            i_position_event    => position_event_c,
            i_clear_diagnostics => i_clear_diagnostics,
            o_face_event        => face_event_c,
            o_overlap_sticky    => face_overlap_sticky_c,
            o_overlap_count     => face_overlap_count_c
        );

    u_scheduler : entity work.shot_scheduler
        generic map (
            G_BUILD_CONFIG => G_BUILD_CONFIG
        )
        port map (
            i_clk                     => i_clk,
            i_rst_n                   => i_rst_n,
            i_enable                  =>
                i_operation_state.scheduler_enable,
            i_active_valid            => i_active_valid,
            i_active_config           => i_active_config,
            i_face_event              => face_event_c,
            i_executor_ready          => executor_ready_c,
            i_request_accept          => request_accept_c,
            i_request_drop            => request_drop_c,
            i_clear_diagnostics       => i_clear_diagnostics,
            o_shot_request            => shot_request_c,
            o_schedule_overrun_pulse  => schedule_overrun_pulse_c,
            o_schedule_overrun_sticky => schedule_overrun_sticky_c,
            o_schedule_overrun_count  => schedule_overrun_count_c,
            o_idle                    => scheduler_idle_c
        );

    u_executor : entity work.laser_executor
        generic map (
            G_BUILD_CONFIG => G_BUILD_CONFIG
        )
        port map (
            i_clk                 => i_clk,
            i_rst_n               => i_rst_n,
            i_active_valid        => i_active_valid,
            i_active_config       => i_active_config,
            i_operation_state     => i_operation_state,
            i_shot_request        => shot_request_c,
            i_fire_done_raw       => i_fire_done_raw,
            i_clear_diagnostics   => i_clear_diagnostics,
            o_executor_ready      => executor_ready_c,
            o_request_accept      => request_accept_c,
            o_request_drop        => request_drop_c,
            o_fire_pulse          => o_fire_pulse,
            o_start_tdc           => o_start_tdc,
            o_stop_tdc            => o_stop_tdc,
            o_shot_start          => shot_start_c,
            o_shot_result         => shot_result_c,
            o_current_request     => current_request_c,
            o_busy                => executor_busy_c,
            o_physical_arm        => physical_arm_c,
            o_rearm_active        => rearm_active_c,
            o_fire_done_sync_clks => o_fire_done_sync_clks,
            o_rearm_margin_clks   => o_rearm_margin_clks,
            o_diagnostics         => laser_diagnostics_c
        );

    u_monitor : entity work.lidar_processing_axis_monitor
        port map (
            i_clk               => i_clk,
            i_rst_n             => i_rst_n,
            i_clear_diagnostics => i_clear_diagnostics,
            i_face_event        => face_event_c,
            m_axis_tready       => m_mon_axis_tready,
            m_axis_tvalid       => m_mon_axis_tvalid,
            m_axis_tdata        => m_mon_axis_tdata,
            m_axis_tkeep        => m_mon_axis_tkeep,
            m_axis_tuser        => m_mon_axis_tuser,
            m_axis_tlast        => m_mon_axis_tlast,
            o_drop_pulse        => monitor_drop_pulse_c,
            o_drop_sticky       => monitor_drop_sticky_c,
            o_drop_count        => monitor_drop_count_c
        );

    p_contract : process (i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '1' then
                if i_operation_state.processing_enable = '1' then
                    assert i_active_valid = '1'
                        report "V2-PROC-002 processing enabled without config"
                        severity failure;
                end if;
                if i_operation_state.scheduler_enable = '1' then
                    assert i_operation_state.processing_enable = '1'
                        report "V2-PROC-003 scheduler enabled without B0/B1"
                        severity failure;
                end if;
                assert not (
                    i_operation_state.physical_fire_enable = '1' and
                    i_operation_state.simulation_enable = '1')
                    report "V2-PROC-004 physical/simulation overlap"
                    severity failure;
                if shot_start_c.valid = '1' then
                    assert shot_start_c.request = current_request_c
                        report "V2-PROC-005 shot-start identity mismatch"
                        severity failure;
                end if;
                if shot_result_c.valid = '1' then
                    assert shot_result_c.request = current_request_c
                        report "V2-PROC-006 shot-result identity mismatch"
                        severity failure;
                end if;
            end if;
        end if;
    end process p_contract;

end architecture rtl;
