library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_build_pkg.all;

-- Processing pipeline을 흐르는 의미 기반 event 계약. 위치 표본, Face 경계,
-- Shot 요청, TDC 시작 기준시점(T0), operation 명령/상태를 record로 묶어
-- payload bit 순서를 각 모듈이 임의로 해석하지 않게 한다.
package lidar_event_types_pkg is

    constant C_POSITION_LATENCY_WIDTH : positive := 8;

    subtype position_latency_t is
        unsigned(C_POSITION_LATENCY_WIDTH - 1 downto 0);

    -- The physical value is counted from the first Processing-clock edge
    -- that samples a stable asynchronous pin value. The virtual value is
    -- counted from the source A/B/Z transition through the common virtual
    -- pipeline register and the registered position-event decoder.
    constant C_POSITION_PHYSICAL_LATENCY_CLKS : position_latency_t :=
        to_unsigned(4, C_POSITION_LATENCY_WIDTH);
    constant C_POSITION_VIRTUAL_LATENCY_CLKS : position_latency_t :=
        to_unsigned(2, C_POSITION_LATENCY_WIDTH);

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

    subtype shot_index_t is unsigned(15 downto 0);
    subtype face_frame_id_t is unsigned(31 downto 0);

    constant C_T0_TIMESTAMP_WIDTH : positive := 64;
    subtype t0_timestamp_t is unsigned(C_T0_TIMESTAMP_WIDTH - 1 downto 0);

    -- Processing-owned Face boundary. This event is emitted only for a Face
    -- traversal admitted by the scheduler and is held with ready/valid until
    -- the data pipeline accepts it. columns_per_face is the immutable geometry
    -- of that traversal, so B8/B9 never infer trailing or all-hole columns from
    -- the presence of TDC data.
    type face_close_event_t is record
        valid            : std_logic;
        face_frame_id    : face_frame_id_t;
        face_index       : face_index_t;
        direction        : direction_t;
        source_sim       : std_logic;
        active_version   : unsigned(15 downto 0);
        columns_per_face : shot_index_t;
    end record face_close_event_t;

    constant C_FACE_CLOSE_EVENT_IDLE : face_close_event_t := (
        valid            => '0',
        face_frame_id    => (others => '0'),
        face_index       => (others => '0'),
        direction        => DIRECTION_CW,
        source_sim       => '0',
        active_version   => (others => '0'),
        columns_per_face => (others => '0')
    );

    -- Registered B2 request. shot_index is the geometric column in the Face
    -- lattice, not the number of requests accepted by the executor. Therefore
    -- a skipped busy point remains a visible hole and cannot shift later VDMA
    -- columns toward the beginning of the line.
    type shot_request_t is record
        valid                : std_logic;
        face_index           : face_index_t;
        position             : position_t;
        direction            : direction_t;
        shot_index           : shot_index_t;
        last_in_face         : std_logic;
        source_sim           : std_logic;
        source_latency_clks  : position_latency_t;
        source_latency_valid : std_logic;
        active_version       : unsigned(15 downto 0);
    end record shot_request_t;

    constant C_SHOT_REQUEST_IDLE : shot_request_t := (
        valid                => '0',
        face_index           => (others => '0'),
        position             => (others => '0'),
        direction            => DIRECTION_CW,
        shot_index           => (others => '0'),
        last_in_face         => '0',
        source_sim           => '0',
        source_latency_clks  => (others => '0'),
        source_latency_valid => '0',
        active_version       => (others => '0')
    );

    -- B3 emits the accepted request again at the synchronized T0 boundary.
    -- Keeping the complete request in the event prevents Face, geometric
    -- column, source mode and configuration version from being reconstructed
    -- by the TDC acquisition path.
    type shot_start_event_t is record
        valid                  : std_logic;
        request                : shot_request_t;
        fire_to_t0_clks        : unsigned(31 downto 0);
        t0_timestamp_ticks     : t0_timestamp_t;
        t0_timestamp_valid     : std_logic;
        t0_time_sync_valid     : std_logic;
    end record shot_start_event_t;

    constant C_SHOT_START_EVENT_IDLE : shot_start_event_t := (
        valid              => '0',
        request            => C_SHOT_REQUEST_IDLE,
        fire_to_t0_clks    => (others => '0'),
        t0_timestamp_ticks => (others => '0'),
        t0_timestamp_valid => '0',
        t0_time_sync_valid => '0'
    );

    -- A result is emitted once per accepted request. A normal result follows
    -- STOP assertion; timeout and abort results never fabricate START/STOP.
    -- timeout and aborted are mutually exclusive when valid is asserted.
    type shot_result_t is record
        valid             : std_logic;
        timeout           : std_logic;
        aborted           : std_logic;
        request           : shot_request_t;
        fire_to_t0_clks   : unsigned(31 downto 0);
    end record shot_result_t;

    constant C_SHOT_RESULT_IDLE : shot_result_t := (
        valid           => '0',
        timeout         => '0',
        aborted         => '0',
        request         => C_SHOT_REQUEST_IDLE,
        fire_to_t0_clks => (others => '0')
    );

    -- Observation-only B3 diagnostics. Counts are modulo 2^32 by design;
    -- sticky bits remain asserted until the explicit diagnostic clear event.
    type laser_diagnostics_t is record
        request_drop_pulse        : std_logic;
        request_drop_sticky       : std_logic;
        request_drop_count        : unsigned(31 downto 0);
        fire_done_timeout_pulse   : std_logic;
        fire_done_timeout_sticky  : std_logic;
        fire_done_timeout_count   : unsigned(31 downto 0);
        operation_abort_pulse     : std_logic;
        operation_abort_sticky    : std_logic;
        operation_abort_count     : unsigned(31 downto 0);
        unexpected_done_pulse     : std_logic;
        unexpected_done_sticky    : std_logic;
        unexpected_done_count     : unsigned(31 downto 0);
    end record laser_diagnostics_t;

    constant C_LASER_DIAGNOSTICS_CLEAR : laser_diagnostics_t := (
        request_drop_pulse       => '0',
        request_drop_sticky      => '0',
        request_drop_count       => (others => '0'),
        fire_done_timeout_pulse  => '0',
        fire_done_timeout_sticky => '0',
        fire_done_timeout_count  => (others => '0'),
        operation_abort_pulse    => '0',
        operation_abort_sticky   => '0',
        operation_abort_count    => (others => '0'),
        unexpected_done_pulse    => '0',
        unexpected_done_sticky   => '0',
        unexpected_done_count    => (others => '0')
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
