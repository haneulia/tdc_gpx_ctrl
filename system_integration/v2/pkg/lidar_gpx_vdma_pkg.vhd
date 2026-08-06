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
    subtype gpx_vdma_word_index_t is unsigned(2 downto 0);
    subtype gpx_vdma_word_count_t is unsigned(2 downto 0);
    subtype gpx_vdma_block_count_t is unsigned(5 downto 0);

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
                result(25 + index) := active(index);
                if cell_event.cell.slope = GPX_SLOPE_RISE then
                    result(18 + index) := active(index);
                end if;
                if active(index) = '1' then
                    result(index) := cell_event.cell.hits(index)(16);
                end if;
            end loop;
            result(15 downto 12) := std_logic_vector(resize(
                cell_event.cell.hit_count, 4));
            result(11) := cell_event.cell.hit_dropped;
            result(10) := cell_event.cell.error_fill or
                cell_event.slot_blank;
            result(9 downto 8) := std_logic_vector(resize(
                cell_event.cell.chip_index, 2));
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

end package body lidar_gpx_vdma_pkg;
