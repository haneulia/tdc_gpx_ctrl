library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_build_pkg.all;

package lidar_event_types_pkg is

    constant C_POSITION_LATENCY_WIDTH : positive := 8;

    subtype position_latency_t is
        unsigned(C_POSITION_LATENCY_WIDTH - 1 downto 0);

    -- The physical value is counted from the first Processing-clock edge
    -- that samples a stable asynchronous pin value. The virtual value is
    -- counted from the registered A/B/Z transition produced in this domain.
    constant C_POSITION_PHYSICAL_LATENCY_CLKS : position_latency_t :=
        to_unsigned(4, C_POSITION_LATENCY_WIDTH);
    constant C_POSITION_VIRTUAL_LATENCY_CLKS : position_latency_t :=
        to_unsigned(1, C_POSITION_LATENCY_WIDTH);

    constant C_VIRTUAL_Z_FAULT_WIDTH     : positive := 2;
    constant C_VIRTUAL_Z_EARLY_OVER_BIT  : natural := 0;
    constant C_VIRTUAL_Z_COLLISION_BIT   : natural := 1;

    subtype virtual_z_fault_t is
        std_logic_vector(C_VIRTUAL_Z_FAULT_WIDTH - 1 downto 0);

    type position_event_t is record
        valid                : std_logic;
        position             : position_t;
        direction            : direction_t;
        source_sim           : std_logic;
        source_latency_clks  : position_latency_t;
        source_latency_valid : std_logic;
        z_event              : std_logic;
        active_version       : unsigned(15 downto 0);
    end record position_event_t;

    constant C_POSITION_EVENT_IDLE : position_event_t := (
        valid                => '0',
        position             => (others => '0'),
        direction            => DIRECTION_CW,
        source_sim           => '0',
        source_latency_clks  => (others => '0'),
        source_latency_valid => '0',
        z_event              => '0',
        active_version       => (others => '0')
    );

    subtype face_index_t is unsigned(2 downto 0);

    -- Registered B1 result.  enter_event and exit_event may both be asserted
    -- when a position event changes Face without an intervening outside state,
    -- or when direction reverses while the position remains inside a Face.
    type face_event_t is record
        valid                : std_logic;
        inside               : std_logic;
        enter_event          : std_logic;
        exit_event           : std_logic;
        overlap              : std_logic;
        face_index           : face_index_t;
        position             : position_t;
        direction            : direction_t;
        source_sim           : std_logic;
        source_latency_clks  : position_latency_t;
        source_latency_valid : std_logic;
        z_event              : std_logic;
        active_version       : unsigned(15 downto 0);
    end record face_event_t;

    constant C_FACE_EVENT_IDLE : face_event_t := (
        valid                => '0',
        inside               => '0',
        enter_event          => '0',
        exit_event           => '0',
        overlap              => '0',
        face_index           => (others => '0'),
        position             => (others => '0'),
        direction            => DIRECTION_CW,
        source_sim           => '0',
        source_latency_clks  => (others => '0'),
        source_latency_valid => '0',
        z_event              => '0',
        active_version       => (others => '0')
    );

    -- Operation commands are one-shot events. They are deliberately separate
    -- from committed configuration so RUN/ARM state has one Processing-domain
    -- owner and can never be inferred from ACTIVE_VALID alone.
    type operation_command_t is (
        OP_COMMAND_NONE,
        OP_COMMAND_RUN,
        OP_COMMAND_STOP,
        OP_COMMAND_ARM,
        OP_COMMAND_DISARM
    );

    subtype operation_command_code_t is std_logic_vector(2 downto 0);

    constant C_OPERATION_STATUS_WIDTH : positive := 8;
    subtype operation_status_bits_t is
        std_logic_vector(C_OPERATION_STATUS_WIDTH - 1 downto 0);

    type operation_state_t is record
        running              : std_logic;
        armed                : std_logic;
        external_permit      : std_logic;
        config_ready         : std_logic;
        processing_enable    : std_logic;
        scheduler_enable     : std_logic;
        physical_fire_enable : std_logic;
        simulation_enable    : std_logic;
    end record operation_state_t;

    constant C_OPERATION_STATE_SAFE : operation_state_t := (
        running              => '0',
        armed                => '0',
        external_permit      => '0',
        config_ready         => '0',
        processing_enable    => '0',
        scheduler_enable     => '0',
        physical_fire_enable => '0',
        simulation_enable    => '0'
    );

    function fn_operation_command_code(
        command_value : operation_command_t
    ) return operation_command_code_t;

    function fn_operation_command_from_code(
        code_value : operation_command_code_t
    ) return operation_command_t;

    function fn_pack_operation_state(
        state_value : operation_state_t
    ) return operation_status_bits_t;

    function fn_unpack_operation_state(
        bits_value : operation_status_bits_t
    ) return operation_state_t;

end package lidar_event_types_pkg;

package body lidar_event_types_pkg is

    function fn_operation_command_code(
        command_value : operation_command_t
    ) return operation_command_code_t is
    begin
        case command_value is
            when OP_COMMAND_RUN    => return "001";
            when OP_COMMAND_STOP   => return "010";
            when OP_COMMAND_ARM    => return "011";
            when OP_COMMAND_DISARM => return "100";
            when others            => return "000";
        end case;
    end function fn_operation_command_code;

    function fn_operation_command_from_code(
        code_value : operation_command_code_t
    ) return operation_command_t is
    begin
        case code_value is
            when "001"  => return OP_COMMAND_RUN;
            when "010"  => return OP_COMMAND_STOP;
            when "011"  => return OP_COMMAND_ARM;
            when "100"  => return OP_COMMAND_DISARM;
            when others => return OP_COMMAND_NONE;
        end case;
    end function fn_operation_command_from_code;

    function fn_pack_operation_state(
        state_value : operation_state_t
    ) return operation_status_bits_t is
        variable result : operation_status_bits_t := (others => '0');
    begin
        result(0) := state_value.running;
        result(1) := state_value.armed;
        result(2) := state_value.external_permit;
        result(3) := state_value.config_ready;
        result(4) := state_value.processing_enable;
        result(5) := state_value.scheduler_enable;
        result(6) := state_value.physical_fire_enable;
        result(7) := state_value.simulation_enable;
        return result;
    end function fn_pack_operation_state;

    function fn_unpack_operation_state(
        bits_value : operation_status_bits_t
    ) return operation_state_t is
        variable result : operation_state_t := C_OPERATION_STATE_SAFE;
    begin
        result.running              := bits_value(0);
        result.armed                := bits_value(1);
        result.external_permit      := bits_value(2);
        result.config_ready         := bits_value(3);
        result.processing_enable    := bits_value(4);
        result.scheduler_enable     := bits_value(5);
        result.physical_fire_enable := bits_value(6);
        result.simulation_enable    := bits_value(7);
        return result;
    end function fn_unpack_operation_state;

end package body lidar_event_types_pkg;
