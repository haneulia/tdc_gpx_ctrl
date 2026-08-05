library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_build_pkg.all;
use work.lidar_event_types_pkg.all;
use work.lidar_gpx_event_pkg.all;

package lidar_gpx_data_pkg is

    -- TDC-GPX I-Mode raw-word fields. Keep these positions in one package so
    -- later Cell/Frame stages do not duplicate bit literals.
    constant C_GPX_RAW_CHACODE_LO : natural := 26;
    constant C_GPX_RAW_CHACODE_HI : natural := 27;
    constant C_GPX_RAW_START_LO   : natural := 18;
    constant C_GPX_RAW_START_HI   : natural := 25;
    constant C_GPX_RAW_SLOPE_BIT  : natural := 17;
    constant C_GPX_RAW_HIT_LO     : natural := 0;
    constant C_GPX_RAW_HIT_HI     : natural := 16;
    constant C_GPX_HIT_WIDTH      : positive := 17;

    subtype gpx_stop_index_t is unsigned(2 downto 0);
    subtype gpx_channel_code_t is unsigned(1 downto 0);
    subtype gpx_start_number_t is unsigned(7 downto 0);
    subtype gpx_hit_value_t is unsigned(C_GPX_HIT_WIDTH - 1 downto 0);

    type gpx_slope_t is (
        GPX_SLOPE_FALL,
        GPX_SLOPE_RISE
    );

    type gpx_hit_event_kind_t is (
        GPX_HIT_DATA,
        GPX_HIT_IFIFO1_DONE,
        GPX_HIT_DRAIN_DONE,
        GPX_HIT_TIMEOUT
    );

    -- B6 typed boundary. Data and control events share the Shot/Chip identity
    -- fields, while kind explicitly states which remaining fields are valid.
    type gpx_hit_event_t is record
        valid         : std_logic;
        kind          : gpx_hit_event_kind_t;
        chip_index    : gpx_chip_index_t;
        ififo_id      : std_logic;
        channel_code  : gpx_channel_code_t;
        stop_index    : gpx_stop_index_t;
        start_number  : gpx_start_number_t;
        slope         : gpx_slope_t;
        hit           : gpx_hit_value_t;
        faulted       : std_logic;
        timeout_cause : std_logic_vector(2 downto 0);
        shot_context  : shot_start_event_t;
        chip_shot_seq : unsigned(15 downto 0);
    end record gpx_hit_event_t;

    -- Pulse and sticky records intentionally share one shape. They stay out
    -- of CSR ownership until the Stage 6 status aggregation is finalized.
    type gpx_hit_decoder_faults_t is record
        chip_index_error : std_logic;
        stop_index_error : std_logic;
        slope_role_error : std_logic;
    end record gpx_hit_decoder_faults_t;

    type gpx_hit_value_array_t is array (
        0 to C_MAX_RETURNS_PER_STOP - 1) of gpx_hit_value_t;

    type gpx_cell_event_kind_t is (
        GPX_CELL_DATA,
        GPX_CELL_IFIFO1_DONE,
        GPX_CELL_DRAIN_DONE,
        GPX_CELL_TIMEOUT
    );

    -- B7 width-independent Cell. AXIS width and byte padding do not appear in
    -- this record; the formatter later converts valid Hit slots into the
    -- canonical 32-bit-word ABI.
    type gpx_cell_event_t is record
        valid         : std_logic;
        kind          : gpx_cell_event_kind_t;
        chip_index    : gpx_chip_index_t;
        ififo_id      : std_logic;
        stop_index    : gpx_stop_index_t;
        slope         : gpx_slope_t;
        hit_count     : unsigned(2 downto 0);
        max_hits      : unsigned(2 downto 0);
        hits          : gpx_hit_value_array_t;
        hit_dropped   : std_logic;
        error_fill    : std_logic;
        faulted       : std_logic;
        timeout_cause : std_logic_vector(2 downto 0);
        shot_context  : shot_start_event_t;
        chip_shot_seq : unsigned(15 downto 0);
    end record gpx_cell_event_t;

    type gpx_cell_collector_faults_t is record
        context_mismatch     : std_logic;
        return_overflow       : std_logic;
        start_number_nonzero : std_logic;
        hit_capacity_drop    : std_logic;
    end record gpx_cell_collector_faults_t;

    -- B8 emits one typed event per canonical Cell slot. The event describes
    -- logical line ordering only: AXIS beat width, byte padding, the 48-byte
    -- prefix and VDMA HSIZE/VSIZE remain formatter-owned concerns.
    subtype gpx_frame_slot_t is unsigned(5 downto 0);

    type gpx_frame_cell_event_t is record
        valid          : std_logic;
        cell           : gpx_cell_event_t;
        slot_index     : gpx_frame_slot_t;
        slot_count     : gpx_frame_slot_t;
        line_start     : std_logic;
        line_end       : std_logic;
        first_column   : std_logic;
        last_column    : std_logic;
        gap_before     : shot_index_t;
        slot_blank     : std_logic;
        line_faulted   : std_logic;
    end record gpx_frame_cell_event_t;

    -- Ordered Face terminator for B9. gap_before on Cell events covers leading
    -- and interior holes; trailing_gap closes the remaining columns. all_hole
    -- is explicit because a Face with no accepted Shot has no Cell context
    -- from which the formatter could reconstruct its geometry.
    type gpx_frame_close_event_t is record
        valid            : std_logic;
        face_index       : face_index_t;
        direction        : direction_t;
        source_sim       : std_logic;
        active_version   : unsigned(15 downto 0);
        columns_per_face : shot_index_t;
        trailing_gap     : shot_index_t;
        all_hole         : std_logic;
        face_faulted     : std_logic;
    end record gpx_frame_close_event_t;

    type gpx_frame_assembler_faults_t is record
        context_mismatch    : std_logic;
        unexpected_cell     : std_logic;
        duplicate_cell      : std_logic;
        duplicate_terminal  : std_logic;
        missing_cell        : std_logic;
        geometry_error      : std_logic;
        column_gap          : std_logic;
        masked_payload_drop : std_logic;
    end record gpx_frame_assembler_faults_t;

    constant C_GPX_HIT_EVENT_IDLE : gpx_hit_event_t := (
        valid         => '0',
        kind          => GPX_HIT_DATA,
        chip_index    => (others => '0'),
        ififo_id      => '0',
        channel_code  => (others => '0'),
        stop_index    => (others => '0'),
        start_number  => (others => '0'),
        slope         => GPX_SLOPE_FALL,
        hit           => (others => '0'),
        faulted       => '0',
        timeout_cause => (others => '0'),
        shot_context  => C_SHOT_START_EVENT_IDLE,
        chip_shot_seq => (others => '0')
    );

    constant C_GPX_HIT_DECODER_FAULTS_CLEAR : gpx_hit_decoder_faults_t := (
        chip_index_error => '0',
        stop_index_error => '0',
        slope_role_error => '0'
    );

    constant C_GPX_CELL_EVENT_IDLE : gpx_cell_event_t := (
        valid         => '0',
        kind          => GPX_CELL_DATA,
        chip_index    => (others => '0'),
        ififo_id      => '0',
        stop_index    => (others => '0'),
        slope         => GPX_SLOPE_FALL,
        hit_count     => (others => '0'),
        max_hits      => (others => '0'),
        hits          => (others => (others => '0')),
        hit_dropped   => '0',
        error_fill    => '0',
        faulted       => '0',
        timeout_cause => (others => '0'),
        shot_context  => C_SHOT_START_EVENT_IDLE,
        chip_shot_seq => (others => '0')
    );

    constant C_GPX_CELL_COLLECTOR_FAULTS_CLEAR :
        gpx_cell_collector_faults_t := (
            context_mismatch      => '0',
            return_overflow       => '0',
            start_number_nonzero  => '0',
            hit_capacity_drop     => '0'
        );

    constant C_GPX_FRAME_CELL_EVENT_IDLE : gpx_frame_cell_event_t := (
        valid        => '0',
        cell         => C_GPX_CELL_EVENT_IDLE,
        slot_index   => (others => '0'),
        slot_count   => (others => '0'),
        line_start   => '0',
        line_end     => '0',
        first_column => '0',
        last_column  => '0',
        gap_before   => (others => '0'),
        slot_blank   => '0',
        line_faulted => '0'
    );

    constant C_GPX_FRAME_CLOSE_EVENT_IDLE : gpx_frame_close_event_t := (
        valid            => '0',
        face_index       => (others => '0'),
        direction        => DIRECTION_CW,
        source_sim       => '0',
        active_version   => (others => '0'),
        columns_per_face => (others => '0'),
        trailing_gap     => (others => '0'),
        all_hole         => '0',
        face_faulted     => '0'
    );

    constant C_GPX_FRAME_ASSEMBLER_FAULTS_CLEAR :
        gpx_frame_assembler_faults_t := (
            context_mismatch    => '0',
            unexpected_cell     => '0',
            duplicate_cell      => '0',
            duplicate_terminal  => '0',
            missing_cell        => '0',
            geometry_error      => '0',
            column_gap          => '0',
            masked_payload_drop => '0'
        );

    function fn_gpx_slope_from_bit(value : std_logic) return gpx_slope_t;
    function fn_gpx_slope_to_bit(value : gpx_slope_t) return std_logic;

end package lidar_gpx_data_pkg;

package body lidar_gpx_data_pkg is

    function fn_gpx_slope_from_bit(value : std_logic) return gpx_slope_t is
    begin
        if value = '1' then
            return GPX_SLOPE_RISE;
        end if;
        return GPX_SLOPE_FALL;
    end function fn_gpx_slope_from_bit;

    function fn_gpx_slope_to_bit(value : gpx_slope_t) return std_logic is
    begin
        if value = GPX_SLOPE_RISE then
            return '1';
        end if;
        return '0';
    end function fn_gpx_slope_to_bit;

end package body lidar_gpx_data_pkg;
