library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_build_pkg.all;
use work.lidar_config_types_pkg.all;
use work.lidar_event_types_pkg.all;

-- Public contracts owned by the integrated Processing subsystem. The AXIS
-- stream is an observation copy of B1 and never participates in shot control.
package lidar_processing_pkg is

    constant C_PROCESSING_MONITOR_TDATA_WIDTH : positive := 64;
    constant C_PROCESSING_MONITOR_TUSER_WIDTH : positive := 8;
    constant C_PROCESSING_MONITOR_TKEEP_WIDTH : positive :=
        C_PROCESSING_MONITOR_TDATA_WIDTH / 8;

    subtype processing_monitor_tdata_t is std_logic_vector(
        C_PROCESSING_MONITOR_TDATA_WIDTH - 1 downto 0);
    subtype processing_monitor_tuser_t is std_logic_vector(
        C_PROCESSING_MONITOR_TUSER_WIDTH - 1 downto 0);
    subtype processing_monitor_tkeep_t is std_logic_vector(
        C_PROCESSING_MONITOR_TKEEP_WIDTH - 1 downto 0);

    -- TDATA[31:0] preserves the useful part of the v1 Motor monitor layout.
    -- The upper word adds atomic configuration and traversal context.
    constant C_MON_POSITION_LSB       : natural := 0;
    constant C_MON_POSITION_MSB       : natural := 14;
    constant C_MON_LATENCY_LSB        : natural := 15;
    constant C_MON_LATENCY_MSB        : natural := 31;
    constant C_MON_ACTIVE_VERSION_LSB : natural := 32;
    constant C_MON_ACTIVE_VERSION_MSB : natural := 47;
    constant C_MON_ENTER_BIT          : natural := 48;
    constant C_MON_EXIT_BIT           : natural := 49;
    constant C_MON_Z_EVENT_BIT        : natural := 50;

    constant C_MON_USER_FACE_LSB      : natural := 0;
    constant C_MON_USER_FACE_MSB      : natural := 2;
    constant C_MON_USER_SOURCE_SIM    : natural := 3;
    constant C_MON_USER_OVERLAP       : natural := 4;
    constant C_MON_USER_LATENCY_VALID : natural := 5;
    constant C_MON_USER_DIRECTION     : natural := 6;
    constant C_MON_USER_INSIDE        : natural := 7;

    subtype processing_latency_t is unsigned(7 downto 0);

    -- These are measured registered-path contracts, not configurable delay
    -- values. Physical latency starts at the first synchronizer sample edge;
    -- virtual latency starts at the virtual-source A/B/Z transition and
    -- includes the common virtual-input pipeline register.
    constant C_B0_TO_EXECUTOR_ACCEPT_CLKS : processing_latency_t :=
        to_unsigned(5, processing_latency_t'length);
    constant C_PHYSICAL_SAMPLE_TO_FIRE_CLKS : processing_latency_t :=
        C_POSITION_PHYSICAL_LATENCY_CLKS +
        C_B0_TO_EXECUTOR_ACCEPT_CLKS;
    constant C_VIRTUAL_TRANSITION_TO_ACCEPT_CLKS : processing_latency_t :=
        C_POSITION_VIRTUAL_LATENCY_CLKS +
        C_B0_TO_EXECUTOR_ACCEPT_CLKS;

    type processing_diagnostics_t is record
        invalid_transition_pulse  : std_logic;
        invalid_transition_sticky : std_logic;
        invalid_transition_count  : u32_t;
        source_switch_pulse        : std_logic;
        virtual_z_fault            : virtual_z_fault_t;
        face_overlap_sticky        : std_logic;
        face_overlap_count         : u32_t;
        schedule_overrun_pulse     : std_logic;
        schedule_overrun_sticky    : std_logic;
        schedule_overrun_count     : u32_t;
        laser                      : laser_diagnostics_t;
        monitor_drop_pulse         : std_logic;
        monitor_drop_sticky        : std_logic;
        monitor_drop_count         : u32_t;
    end record processing_diagnostics_t;

    constant C_PROCESSING_DIAGNOSTICS_CLEAR : processing_diagnostics_t := (
        invalid_transition_pulse  => '0',
        invalid_transition_sticky => '0',
        invalid_transition_count  => (others => '0'),
        source_switch_pulse        => '0',
        virtual_z_fault            => (others => '0'),
        face_overlap_sticky        => '0',
        face_overlap_count         => (others => '0'),
        schedule_overrun_pulse     => '0',
        schedule_overrun_sticky    => '0',
        schedule_overrun_count     => (others => '0'),
        laser                      => C_LASER_DIAGNOSTICS_CLEAR,
        monitor_drop_pulse         => '0',
        monitor_drop_sticky        => '0',
        monitor_drop_count         => (others => '0')
    );

    function fn_pack_processing_monitor_data(
        event_value : face_event_t
    ) return processing_monitor_tdata_t;

    function fn_pack_processing_monitor_user(
        event_value : face_event_t
    ) return processing_monitor_tuser_t;

end package lidar_processing_pkg;

package body lidar_processing_pkg is

    function fn_pack_processing_monitor_data(
        event_value : face_event_t
    ) return processing_monitor_tdata_t is
        variable result : processing_monitor_tdata_t := (others => '0');
    begin
        result(C_MON_POSITION_MSB downto C_MON_POSITION_LSB) :=
            std_logic_vector(event_value.position);
        result(C_MON_LATENCY_MSB downto C_MON_LATENCY_LSB) :=
            std_logic_vector(resize(
                event_value.source_latency_clks,
                C_MON_LATENCY_MSB - C_MON_LATENCY_LSB + 1));
        result(C_MON_ACTIVE_VERSION_MSB downto
               C_MON_ACTIVE_VERSION_LSB) :=
            std_logic_vector(event_value.active_version);
        result(C_MON_ENTER_BIT)   := event_value.enter_event;
        result(C_MON_EXIT_BIT)    := event_value.exit_event;
        result(C_MON_Z_EVENT_BIT) := event_value.z_event;
        return result;
    end function fn_pack_processing_monitor_data;

    function fn_pack_processing_monitor_user(
        event_value : face_event_t
    ) return processing_monitor_tuser_t is
        variable result : processing_monitor_tuser_t := (others => '0');
    begin
        result(C_MON_USER_FACE_MSB downto C_MON_USER_FACE_LSB) :=
            std_logic_vector(event_value.face_index);
        result(C_MON_USER_SOURCE_SIM)    := event_value.source_sim;
        result(C_MON_USER_OVERLAP)       := event_value.overlap;
        result(C_MON_USER_LATENCY_VALID) :=
            event_value.source_latency_valid;
        if event_value.direction = DIRECTION_CCW then
            result(C_MON_USER_DIRECTION) := '1';
        end if;
        result(C_MON_USER_INSIDE) := event_value.inside;
        return result;
    end function fn_pack_processing_monitor_user;

end package body lidar_processing_pkg;
