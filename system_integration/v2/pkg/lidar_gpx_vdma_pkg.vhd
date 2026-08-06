library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_build_pkg.all;
use work.lidar_event_types_pkg.all;
use work.lidar_gpx_data_pkg.all;

package lidar_gpx_vdma_pkg is

    constant C_GPX_VDMA_WORD_WIDTH       : positive := 32;
    constant C_GPX_VDMA_WORD_BYTES       : positive := 4;
    constant C_GPX_VDMA_BLOCK_WIDTH      : positive := 128;
    constant C_GPX_VDMA_BLOCK_BYTES      : positive := 16;
    constant C_GPX_VDMA_WORDS_PER_BLOCK  : positive := 4;
    constant C_GPX_VDMA_SHOT_META_BYTES  : positive := 16;
    constant C_GPX_VDMA_SHOT_META_WORDS  : positive := 4;
    constant C_GPX_VDMA_FOOTER_BYTES     : positive := 32;
    constant C_GPX_VDMA_FOOTER_WORDS     : positive := 8;

    constant C_GPX_SHOT_META_VALID                : natural := 0;
    constant C_GPX_SHOT_META_HOLE                 : natural := 1;
    constant C_GPX_SHOT_META_DIRECTION_CCW        : natural := 2;
    constant C_GPX_SHOT_META_SOURCE_SIM           : natural := 3;
    constant C_GPX_SHOT_META_TIMEOUT              : natural := 4;
    constant C_GPX_SHOT_META_ABORTED              : natural := 5;
    constant C_GPX_SHOT_META_LINE_FAULTED         : natural := 6;
    constant C_GPX_SHOT_META_T0_VALID             : natural := 7;
    constant C_GPX_SHOT_META_TIME_SYNC_VALID      : natural := 8;
    constant C_GPX_SHOT_META_LAST_IN_FACE         : natural := 9;
    constant C_GPX_SHOT_META_SOURCE_LATENCY_VALID : natural := 10;
    constant C_GPX_SHOT_META_SOURCE_LATENCY_LO    : natural := 11;
    constant C_GPX_SHOT_META_SOURCE_LATENCY_HI    : natural := 18;

    constant C_GPX_CELL_META_HIT_MSB_LO  : natural := 0;
    constant C_GPX_CELL_META_HIT_MSB_HI  : natural := 6;
    constant C_GPX_CELL_META_VALID_LO    : natural := 7;
    constant C_GPX_CELL_META_VALID_HI    : natural := 13;
    constant C_GPX_CELL_META_COUNT_LO    : natural := 14;
    constant C_GPX_CELL_META_COUNT_HI    : natural := 16;
    constant C_GPX_CELL_META_SLOPE       : natural := 17;
    constant C_GPX_CELL_META_CHIP_LO     : natural := 18;
    constant C_GPX_CELL_META_CHIP_HI     : natural := 19;
    constant C_GPX_CELL_META_STOP_LO     : natural := 20;
    constant C_GPX_CELL_META_STOP_HI     : natural := 22;
    constant C_GPX_CELL_META_BLANK       : natural := 23;
    constant C_GPX_CELL_META_ERROR_FILL  : natural := 24;
    constant C_GPX_CELL_META_HIT_DROPPED : natural := 25;
    constant C_GPX_CELL_META_RETURN_OVERFLOW : natural := 26;
    constant C_GPX_CELL_META_FAULTED     : natural := 27;
    constant C_GPX_CELL_META_TIMEOUT_LO  : natural := 28;
    constant C_GPX_CELL_META_TIMEOUT_HI  : natural := 30;
    constant C_GPX_CELL_META_VALID       : natural := 31;

    -- Legacy J2 prefix constants remain until the formatter migration in J5.
    constant C_GPX_VDMA_PREFIX_BYTES     : positive := 48;
    constant C_GPX_VDMA_PREFIX_WORDS     : positive := 12;
    constant C_GPX_VDMA_PREFIX_BLOCKS    : positive := 3;
    constant C_GPX_VDMA_MAX_CELL_WORDS   : positive := 5;
    constant C_GPX_VDMA_MAX_LINE_SLOTS   : positive :=
        C_MAX_CHIPS * C_MAX_STOPS_PER_CHIP;
    constant C_GPX_VDMA_MAX_PAYLOAD_WORDS : positive :=
        C_GPX_VDMA_MAX_LINE_SLOTS * C_GPX_VDMA_MAX_CELL_WORDS;
    constant C_GPX_VDMA_MAX_PAYLOAD_BLOCKS : positive :=
        C_GPX_VDMA_MAX_PAYLOAD_WORDS / C_GPX_VDMA_WORDS_PER_BLOCK;

    subtype gpx_vdma_word_t is
        std_logic_vector(C_GPX_VDMA_WORD_WIDTH - 1 downto 0);
    subtype gpx_vdma_block_t is
        std_logic_vector(C_GPX_VDMA_BLOCK_WIDTH - 1 downto 0);
    constant C_GPX_VDMA_FOOTER_MAGIC  : gpx_vdma_word_t := x"47504631";
    constant C_GPX_VDMA_FOOTER_COMMIT : gpx_vdma_word_t := x"434F4D54";
    subtype gpx_vdma_word_index_t is unsigned(2 downto 0);
    subtype gpx_vdma_word_count_t is unsigned(2 downto 0);
    subtype gpx_vdma_block_count_t is unsigned(5 downto 0);

    subtype gpx_vdma_line_word_index_t is unsigned(8 downto 0);
    subtype gpx_vdma_line_word_count_t is unsigned(8 downto 0);

    type gpx_vdma_shot_status_t is record
        data_valid   : std_logic;
        hole         : std_logic;
        timeout      : std_logic;
        aborted      : std_logic;
        line_faulted : std_logic;
    end record gpx_vdma_shot_status_t;

    constant C_GPX_VDMA_SHOT_STATUS_CLEAR : gpx_vdma_shot_status_t := (
        data_valid   => '0',
        hole         => '0',
        timeout      => '0',
        aborted      => '0',
        line_faulted => '0'
    );

    type gpx_vdma_line_word_kind_t is (
        GPX_VDMA_LINE_SHOT_METADATA,
        GPX_VDMA_LINE_CELL_DATA,
        GPX_VDMA_LINE_FACE_FOOTER
    );

    -- Width-independent, canonical 32-bit stream. J6 is the sole owner of
    -- combining these words into 32/64/128-bit AXIS beats and final padding.
    type gpx_vdma_line_word_event_t is record
        valid           : std_logic;
        data            : gpx_vdma_word_t;
        kind            : gpx_vdma_line_word_kind_t;
        word_index      : gpx_vdma_line_word_index_t;
        line_word_count : gpx_vdma_line_word_count_t;
        line_start      : std_logic;
        line_end        : std_logic;
        first_column    : std_logic;
        last_column     : std_logic;
        gap_before      : shot_index_t;
        slot_count      : gpx_frame_slot_t;
        cell_word_count : gpx_vdma_word_count_t;
        line_hole       : std_logic;
        line_faulted    : std_logic;
        shot_context    : shot_start_event_t;
    end record gpx_vdma_line_word_event_t;

    constant C_GPX_VDMA_LINE_WORD_EVENT_IDLE :
        gpx_vdma_line_word_event_t := (
            valid           => '0',
            data            => (others => '0'),
            kind            => GPX_VDMA_LINE_SHOT_METADATA,
            word_index      => (others => '0'),
            line_word_count => (others => '0'),
            line_start      => '0',
            line_end        => '0',
            first_column    => '0',
            last_column     => '0',
            gap_before      => (others => '0'),
            slot_count      => (others => '0'),
            cell_word_count => (others => '0'),
            line_hole       => '0',
            line_faulted    => '0',
            shot_context    => C_SHOT_START_EVENT_IDLE
        );

    type gpx_vdma_prefix_blocks_t is array (
        0 to C_GPX_VDMA_PREFIX_BLOCKS - 1
    ) of gpx_vdma_block_t;

    constant C_GPX_VDMA_PREFIX_BLOCKS_ZERO :
        gpx_vdma_prefix_blocks_t := (others => (others => '0'));

    type gpx_vdma_word_event_t is record
        valid          : std_logic;
        data           : gpx_vdma_word_t;
        word_index     : gpx_vdma_word_index_t;
        word_count     : gpx_vdma_word_count_t;
        cell_start     : std_logic;
        cell_end       : std_logic;
        line_start     : std_logic;
        line_end       : std_logic;
        first_column   : std_logic;
        last_column    : std_logic;
        slot_index     : gpx_frame_slot_t;
        slot_count     : gpx_frame_slot_t;
        gap_before     : shot_index_t;
        slot_blank     : std_logic;
        line_faulted   : std_logic;
        shot_context   : shot_start_event_t;
    end record gpx_vdma_word_event_t;

    constant C_GPX_VDMA_WORD_EVENT_IDLE : gpx_vdma_word_event_t := (
        valid        => '0',
        data         => (others => '0'),
        word_index   => (others => '0'),
        word_count   => (others => '0'),
        cell_start   => '0',
        cell_end     => '0',
        line_start   => '0',
        line_end     => '0',
        first_column => '0',
        last_column  => '0',
        slot_index   => (others => '0'),
        slot_count   => (others => '0'),
        gap_before   => (others => '0'),
        slot_blank   => '0',
        line_faulted => '0',
        shot_context => C_SHOT_START_EVENT_IDLE
    );

    function fn_gpx_vdma_effective_max_hits(
        value : unsigned(2 downto 0)
    ) return positive;

    function fn_gpx_vdma_cell_word_count(
        max_hits : positive
    ) return positive;

    function fn_gpx_vdma_cell_bytes(
        max_hits : positive
    ) return positive;

    function fn_gpx_vdma_align16(
        value : natural
    ) return natural;

    function fn_gpx_vdma_align(
        value     : natural;
        alignment : positive
    ) return natural;

    function fn_gpx_vdma_beat_bytes(
        output_width : positive
    ) return positive;

    function fn_gpx_vdma_shot_raw_hsize_bytes(
        slot_count : natural;
        max_hits   : positive
    ) return natural;

    function fn_gpx_vdma_shot_hsize_bytes(
        slot_count   : natural;
        max_hits     : positive;
        output_width : positive
    ) return natural;

    function fn_gpx_vdma_shot_line_beats(
        slot_count   : natural;
        max_hits     : positive;
        output_width : positive
    ) return natural;

    function fn_gpx_vdma_footer_lines(
        hsize_bytes : natural
    ) return natural;

    function fn_gpx_vdma_vsize_lines(
        planned_shots : natural;
        hsize_bytes   : natural
    ) return natural;

    function fn_gpx_vdma_stride_bytes(
        max_slot_count : natural;
        max_hits       : positive;
        output_width   : positive
    ) return natural;

    function fn_gpx_vdma_max_vsize_lines(
        max_planned_shots : natural;
        output_width      : positive
    ) return natural;

    function fn_gpx_vdma_frame_allocation_bytes(
        max_slot_count    : natural;
        max_hits          : positive;
        max_planned_shots : natural;
        output_width      : positive
    ) return natural;

    function fn_gpx_vdma_payload_blocks(
        slot_count : natural;
        max_hits   : positive
    ) return natural;

    function fn_gpx_vdma_hsize_bytes(
        slot_count : natural;
        max_hits   : positive
    ) return natural;

    function fn_gpx_vdma_line_beats(
        slot_count   : natural;
        max_hits     : positive;
        output_width : positive
    ) return natural;

    function fn_gpx_vdma_cell_word(
        cell_event : gpx_frame_cell_event_t;
        word_index : natural
    ) return gpx_vdma_word_t;

    function fn_gpx_vdma_make_word_event(
        cell_event : gpx_frame_cell_event_t;
        word_index : natural
    ) return gpx_vdma_word_event_t;

    function fn_gpx_vdma_shot_metadata_word(
        shot_context : shot_start_event_t;
        shot_status  : gpx_vdma_shot_status_t;
        word_index   : natural
    ) return gpx_vdma_word_t;

end package lidar_gpx_vdma_pkg;

package body lidar_gpx_vdma_pkg is

    function fn_gpx_vdma_effective_max_hits(
        value : unsigned(2 downto 0)
    ) return positive is
        variable result : natural := to_integer(value);
    begin
        if result < 1 then
            return 1;
        elsif result > C_MAX_RETURNS_PER_STOP then
            return C_MAX_RETURNS_PER_STOP;
        end if;
        return result;
    end function fn_gpx_vdma_effective_max_hits;

    function fn_gpx_vdma_cell_word_count(
        max_hits : positive
    ) return positive is
    begin
        assert max_hits <= C_MAX_RETURNS_PER_STOP
            report "V2-B9-PKG-001 max_hits exceeds physical Return capacity"
            severity failure;
        return ((max_hits + 1) / 2) + 1;
    end function fn_gpx_vdma_cell_word_count;

    function fn_gpx_vdma_cell_bytes(
        max_hits : positive
    ) return positive is
    begin
        return fn_gpx_vdma_cell_word_count(max_hits) *
            C_GPX_VDMA_WORD_BYTES;
    end function fn_gpx_vdma_cell_bytes;

    function fn_gpx_vdma_align16(
        value : natural
    ) return natural is
    begin
        return ((value + C_GPX_VDMA_BLOCK_BYTES - 1) /
            C_GPX_VDMA_BLOCK_BYTES) * C_GPX_VDMA_BLOCK_BYTES;
    end function fn_gpx_vdma_align16;

    function fn_gpx_vdma_align(
        value     : natural;
        alignment : positive
    ) return natural is
    begin
        return ((value + alignment - 1) / alignment) * alignment;
    end function fn_gpx_vdma_align;

    function fn_gpx_vdma_beat_bytes(
        output_width : positive
    ) return positive is
    begin
        assert output_width = 32 or output_width = 64 or output_width = 128
            report "V2-B9-PKG-005 output width must be 32, 64, or 128"
            severity failure;
        return output_width / 8;
    end function fn_gpx_vdma_beat_bytes;

    function fn_gpx_vdma_shot_raw_hsize_bytes(
        slot_count : natural;
        max_hits   : positive
    ) return natural is
    begin
        assert slot_count <= C_GPX_VDMA_MAX_LINE_SLOTS
            report "V2-B9-PKG-006 slot_count exceeds one slope lane"
            severity failure;
        if slot_count = 0 then
            return 0;
        end if;
        return C_GPX_VDMA_SHOT_META_BYTES +
            slot_count * fn_gpx_vdma_cell_bytes(max_hits);
    end function fn_gpx_vdma_shot_raw_hsize_bytes;

    function fn_gpx_vdma_shot_hsize_bytes(
        slot_count   : natural;
        max_hits     : positive;
        output_width : positive
    ) return natural is
        variable raw_hsize : natural;
    begin
        raw_hsize := fn_gpx_vdma_shot_raw_hsize_bytes(
            slot_count, max_hits);
        if raw_hsize = 0 then
            return 0;
        end if;
        return fn_gpx_vdma_align(
            raw_hsize, fn_gpx_vdma_beat_bytes(output_width));
    end function fn_gpx_vdma_shot_hsize_bytes;

    function fn_gpx_vdma_shot_line_beats(
        slot_count   : natural;
        max_hits     : positive;
        output_width : positive
    ) return natural is
    begin
        return fn_gpx_vdma_shot_hsize_bytes(
            slot_count, max_hits, output_width) /
            fn_gpx_vdma_beat_bytes(output_width);
    end function fn_gpx_vdma_shot_line_beats;

    function fn_gpx_vdma_footer_lines(
        hsize_bytes : natural
    ) return natural is
    begin
        if hsize_bytes = 0 then
            return 0;
        end if;
        return (C_GPX_VDMA_FOOTER_BYTES + hsize_bytes - 1) /
            hsize_bytes;
    end function fn_gpx_vdma_footer_lines;

    function fn_gpx_vdma_vsize_lines(
        planned_shots : natural;
        hsize_bytes   : natural
    ) return natural is
    begin
        if hsize_bytes = 0 then
            return 0;
        end if;
        return planned_shots + fn_gpx_vdma_footer_lines(hsize_bytes);
    end function fn_gpx_vdma_vsize_lines;

    function fn_gpx_vdma_stride_bytes(
        max_slot_count : natural;
        max_hits       : positive;
        output_width   : positive
    ) return natural is
    begin
        return fn_gpx_vdma_shot_hsize_bytes(
            max_slot_count, max_hits, output_width);
    end function fn_gpx_vdma_stride_bytes;

    function fn_gpx_vdma_max_vsize_lines(
        max_planned_shots : natural;
        output_width      : positive
    ) return natural is
        variable minimum_hsize : natural;
    begin
        minimum_hsize := fn_gpx_vdma_shot_hsize_bytes(
            1, 1, output_width);
        return max_planned_shots +
            fn_gpx_vdma_footer_lines(minimum_hsize);
    end function fn_gpx_vdma_max_vsize_lines;

    function fn_gpx_vdma_frame_allocation_bytes(
        max_slot_count    : natural;
        max_hits          : positive;
        max_planned_shots : natural;
        output_width      : positive
    ) return natural is
    begin
        return fn_gpx_vdma_stride_bytes(
            max_slot_count, max_hits, output_width) *
            fn_gpx_vdma_max_vsize_lines(
                max_planned_shots, output_width);
    end function fn_gpx_vdma_frame_allocation_bytes;

    function fn_gpx_vdma_payload_blocks(
        slot_count : natural;
        max_hits   : positive
    ) return natural is
    begin
        assert slot_count <= C_GPX_VDMA_MAX_LINE_SLOTS
            report "V2-B9-PKG-002 slot_count exceeds one slope lane"
            severity failure;
        return fn_gpx_vdma_align16(
            slot_count * fn_gpx_vdma_cell_bytes(max_hits)) /
            C_GPX_VDMA_BLOCK_BYTES;
    end function fn_gpx_vdma_payload_blocks;

    function fn_gpx_vdma_hsize_bytes(
        slot_count : natural;
        max_hits   : positive
    ) return natural is
    begin
        if slot_count = 0 then
            return 0;
        end if;
        return C_GPX_VDMA_PREFIX_BYTES +
            fn_gpx_vdma_align16(
                slot_count * fn_gpx_vdma_cell_bytes(max_hits));
    end function fn_gpx_vdma_hsize_bytes;

    function fn_gpx_vdma_line_beats(
        slot_count   : natural;
        max_hits     : positive;
        output_width : positive
    ) return natural is
        variable hsize : natural;
    begin
        assert output_width = 32 or output_width = 64 or output_width = 128
            report "V2-B9-PKG-003 output width must be 32, 64, or 128"
            severity failure;
        hsize := fn_gpx_vdma_hsize_bytes(slot_count, max_hits);
        return hsize / (output_width / 8);
    end function fn_gpx_vdma_line_beats;

    function fn_active_hit_mask(
        cell_event : gpx_frame_cell_event_t
    ) return std_logic_vector is
        variable result : std_logic_vector(
            C_MAX_RETURNS_PER_STOP - 1 downto 0) := (others => '0');
        variable hit_count : natural := to_integer(cell_event.cell.hit_count);
        variable max_hits  : positive := fn_gpx_vdma_effective_max_hits(
            cell_event.cell.max_hits);
    begin
        if cell_event.slot_blank = '0' then
            for hit_index in 0 to C_MAX_RETURNS_PER_STOP - 1 loop
                if hit_index < hit_count and hit_index < max_hits then
                    result(hit_index) := '1';
                end if;
            end loop;
        end if;
        return result;
    end function fn_active_hit_mask;

    function fn_gpx_vdma_cell_word(
        cell_event : gpx_frame_cell_event_t;
        word_index : natural
    ) return gpx_vdma_word_t is
        variable result     : gpx_vdma_word_t := (others => '0');
        variable active     : std_logic_vector(
            C_MAX_RETURNS_PER_STOP - 1 downto 0);
        variable max_hits   : positive := fn_gpx_vdma_effective_max_hits(
            cell_event.cell.max_hits);
        variable hit_words  : positive := (max_hits + 1) / 2;
        variable word_count : positive := hit_words + 1;
        variable hit_index  : natural;
    begin
        assert word_index < word_count
            report "V2-B9-PKG-004 Cell word index out of range"
            severity failure;

        active := fn_active_hit_mask(cell_event);

        if word_index < hit_words then
            hit_index := word_index * 2;
            if hit_index < C_MAX_RETURNS_PER_STOP and
               active(hit_index) = '1' then
                result(15 downto 0) := std_logic_vector(
                    cell_event.cell.hits(hit_index)(15 downto 0));
            end if;
            hit_index := hit_index + 1;
            if hit_index < C_MAX_RETURNS_PER_STOP and
               active(hit_index) = '1' then
                result(31 downto 16) := std_logic_vector(
                    cell_event.cell.hits(hit_index)(15 downto 0));
            end if;
        else
            for index in 0 to C_MAX_RETURNS_PER_STOP - 1 loop
                if active(index) = '1' then
                    result(C_GPX_CELL_META_HIT_MSB_LO + index) :=
                        cell_event.cell.hits(index)(16);
                end if;
                result(C_GPX_CELL_META_VALID_LO + index) := active(index);
            end loop;
            result(C_GPX_CELL_META_COUNT_HI downto
                   C_GPX_CELL_META_COUNT_LO) := std_logic_vector(
                cell_event.cell.hit_count);
            if cell_event.cell.slope = GPX_SLOPE_RISE then
                result(C_GPX_CELL_META_SLOPE) := '1';
            end if;
            result(C_GPX_CELL_META_CHIP_HI downto
                   C_GPX_CELL_META_CHIP_LO) := std_logic_vector(
                cell_event.cell.chip_index);
            result(C_GPX_CELL_META_STOP_HI downto
                   C_GPX_CELL_META_STOP_LO) := std_logic_vector(
                cell_event.cell.stop_index);
            result(C_GPX_CELL_META_BLANK) := cell_event.slot_blank;
            result(C_GPX_CELL_META_ERROR_FILL) :=
                cell_event.cell.error_fill;
            result(C_GPX_CELL_META_HIT_DROPPED) :=
                cell_event.cell.hit_dropped;
            result(C_GPX_CELL_META_RETURN_OVERFLOW) :=
                cell_event.cell.return_overflow;
            result(C_GPX_CELL_META_FAULTED) :=
                cell_event.cell.faulted or cell_event.line_faulted;
            result(C_GPX_CELL_META_TIMEOUT_HI downto
                   C_GPX_CELL_META_TIMEOUT_LO) :=
                cell_event.cell.timeout_cause;
            result(C_GPX_CELL_META_VALID) := '1';
        end if;

        return result;
    end function fn_gpx_vdma_cell_word;

    function fn_gpx_vdma_make_word_event(
        cell_event : gpx_frame_cell_event_t;
        word_index : natural
    ) return gpx_vdma_word_event_t is
        variable result : gpx_vdma_word_event_t :=
            C_GPX_VDMA_WORD_EVENT_IDLE;
        variable max_hits : positive := fn_gpx_vdma_effective_max_hits(
            cell_event.cell.max_hits);
        variable word_count : positive := fn_gpx_vdma_cell_word_count(max_hits);
    begin
        result.valid := '1';
        result.data := fn_gpx_vdma_cell_word(cell_event, word_index);
        result.word_index := to_unsigned(word_index, result.word_index'length);
        result.word_count := to_unsigned(word_count, result.word_count'length);
        result.cell_start := '1' when word_index = 0 else '0';
        result.cell_end := '1' when word_index + 1 = word_count else '0';
        result.line_start := cell_event.line_start and result.cell_start;
        result.line_end := cell_event.line_end and result.cell_end;
        result.first_column := cell_event.first_column;
        result.last_column := cell_event.last_column;
        result.slot_index := cell_event.slot_index;
        result.slot_count := cell_event.slot_count;
        if result.line_start = '1' then
            result.gap_before := cell_event.gap_before;
        end if;
        result.slot_blank := cell_event.slot_blank;
        result.line_faulted := cell_event.line_faulted;
        result.shot_context := cell_event.cell.shot_context;
        return result;
    end function fn_gpx_vdma_make_word_event;

    function fn_gpx_vdma_shot_metadata_word(
        shot_context : shot_start_event_t;
        shot_status  : gpx_vdma_shot_status_t;
        word_index   : natural
    ) return gpx_vdma_word_t is
        variable result : gpx_vdma_word_t := (others => '0');
    begin
        case word_index is
            when 0 =>
                result := std_logic_vector(
                    shot_context.t0_timestamp_ticks(31 downto 0));
            when 1 =>
                result := std_logic_vector(
                    shot_context.t0_timestamp_ticks(63 downto 32));
            when 2 =>
                result(15 downto 0) := std_logic_vector(
                    shot_context.request.shot_index);
                result(31 downto 16) := std_logic_vector(resize(
                    shot_context.request.position, 16));
            when 3 =>
                result(C_GPX_SHOT_META_VALID) := shot_status.data_valid;
                result(C_GPX_SHOT_META_HOLE) := shot_status.hole;
                if shot_context.request.direction = DIRECTION_CCW then
                    result(C_GPX_SHOT_META_DIRECTION_CCW) := '1';
                end if;
                result(C_GPX_SHOT_META_SOURCE_SIM) :=
                    shot_context.request.source_sim;
                result(C_GPX_SHOT_META_TIMEOUT) := shot_status.timeout;
                result(C_GPX_SHOT_META_ABORTED) := shot_status.aborted;
                result(C_GPX_SHOT_META_LINE_FAULTED) :=
                    shot_status.line_faulted;
                result(C_GPX_SHOT_META_T0_VALID) :=
                    shot_context.t0_timestamp_valid;
                result(C_GPX_SHOT_META_TIME_SYNC_VALID) :=
                    shot_context.t0_time_sync_valid;
                result(C_GPX_SHOT_META_LAST_IN_FACE) :=
                    shot_context.request.last_in_face;
                result(C_GPX_SHOT_META_SOURCE_LATENCY_VALID) :=
                    shot_context.request.source_latency_valid;
                result(C_GPX_SHOT_META_SOURCE_LATENCY_HI downto
                       C_GPX_SHOT_META_SOURCE_LATENCY_LO) :=
                    std_logic_vector(
                        shot_context.request.source_latency_clks);
            when others =>
                assert false
                    report "V2-B9-PKG-009 Shot Metadata word index out of range"
                    severity failure;
        end case;
        return result;
    end function fn_gpx_vdma_shot_metadata_word;

end package body lidar_gpx_vdma_pkg;
