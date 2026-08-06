library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_event_types_pkg.all;
use work.lidar_gpx_vdma_pkg.all;

-- J5 canonical Shot-Line boundary. Four Shot Metadata words are emitted before
-- the serialized Cell words. AXIS width, byte packing and final-Beat padding
-- are deliberately absent; J6 owns those transport details.
entity lidar_gpx_shot_line_builder is
    port (
        i_clk             : in  std_logic;
        i_rst_n           : in  std_logic;
        i_abort           : in  std_logic;

        i_cell_word       : in  gpx_vdma_word_event_t;
        o_cell_word_ready : out std_logic;

        o_line_word       : out gpx_vdma_line_word_event_t;
        i_line_word_ready : in  std_logic;

        o_idle            : out std_logic
    );
end entity lidar_gpx_shot_line_builder;

architecture rtl of lidar_gpx_shot_line_builder is

    type state_t is (
        ST_IDLE,
        ST_METADATA,
        ST_FIRST_CELL,
        ST_CELLS,
        ST_DRAIN_LAST
    );

    signal state_r : state_t := ST_IDLE;
    signal first_cell_word_r : gpx_vdma_word_event_t :=
        C_GPX_VDMA_WORD_EVENT_IDLE;
    signal shot_status_r : gpx_vdma_shot_status_t :=
        C_GPX_VDMA_SHOT_STATUS_CLEAR;
    signal metadata_index_r : natural range 0 to
        C_GPX_VDMA_SHOT_META_WORDS - 1 := 0;
    signal line_word_count_r : gpx_vdma_line_word_count_t :=
        (others => '0');
    signal line_word_r : gpx_vdma_line_word_event_t :=
        C_GPX_VDMA_LINE_WORD_EVENT_IDLE;
    signal output_ready_c : std_logic;

    function fn_line_word_from_cell(
        value           : gpx_vdma_word_event_t;
        line_word_count : gpx_vdma_line_word_count_t
    ) return gpx_vdma_line_word_event_t is
        variable result : gpx_vdma_line_word_event_t :=
            C_GPX_VDMA_LINE_WORD_EVENT_IDLE;
        variable global_index : natural;
    begin
        global_index := C_GPX_VDMA_SHOT_META_WORDS +
            to_integer(value.slot_index) * to_integer(value.word_count) +
            to_integer(value.word_index);
        result.valid           := '1';
        result.data            := value.data;
        result.kind            := GPX_VDMA_LINE_CELL_DATA;
        result.word_index      := to_unsigned(
            global_index, result.word_index'length);
        result.line_word_count := line_word_count;
        result.line_end        := value.line_end;
        result.first_column    := value.first_column;
        result.last_column     := value.last_column;
        result.slot_count      := value.slot_count;
        result.cell_word_count := value.word_count;
        result.line_faulted    := value.line_faulted;
        result.shot_context    := value.shot_context;
        return result;
    end function fn_line_word_from_cell;

begin

    output_ready_c <= '1' when line_word_r.valid = '0' or
        i_line_word_ready = '1' else '0';

    o_cell_word_ready <= '1' when i_abort = '0' and
        (state_r = ST_IDLE or
         (state_r = ST_CELLS and output_ready_c = '1')) else '0';
    o_line_word <= line_word_r;
    o_idle <= '1' when state_r = ST_IDLE and
        line_word_r.valid = '0' else '0';

    p_build : process (i_clk)
        variable next_word : gpx_vdma_line_word_event_t;
        variable total_words : natural;
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' or i_abort = '1' then
                state_r <= ST_IDLE;
                first_cell_word_r <= C_GPX_VDMA_WORD_EVENT_IDLE;
                shot_status_r <= C_GPX_VDMA_SHOT_STATUS_CLEAR;
                metadata_index_r <= 0;
                line_word_count_r <= (others => '0');
                line_word_r <= C_GPX_VDMA_LINE_WORD_EVENT_IDLE;
            else
                if line_word_r.valid = '1' and i_line_word_ready = '1' then
                    line_word_r.valid <= '0';
                end if;

                case state_r is
                    when ST_IDLE =>
                        if i_cell_word.valid = '1' and
                           o_cell_word_ready = '1' then
                            first_cell_word_r <= i_cell_word;
                            total_words := C_GPX_VDMA_SHOT_META_WORDS +
                                to_integer(i_cell_word.slot_count) *
                                to_integer(i_cell_word.word_count);
                            line_word_count_r <= to_unsigned(
                                total_words, line_word_count_r'length);
                            shot_status_r <= C_GPX_VDMA_SHOT_STATUS_CLEAR;
                            shot_status_r.data_valid <= '1';
                            shot_status_r.line_faulted <=
                                i_cell_word.line_faulted;
                            metadata_index_r <= 0;
                            state_r <= ST_METADATA;
                        end if;

                    when ST_METADATA =>
                        if output_ready_c = '1' then
                            next_word := C_GPX_VDMA_LINE_WORD_EVENT_IDLE;
                            next_word.valid := '1';
                            next_word.data := fn_gpx_vdma_shot_metadata_word(
                                first_cell_word_r.shot_context,
                                shot_status_r,
                                metadata_index_r);
                            next_word.kind := GPX_VDMA_LINE_SHOT_METADATA;
                            next_word.word_index := to_unsigned(
                                metadata_index_r,
                                next_word.word_index'length);
                            next_word.line_word_count := line_word_count_r;
                            if metadata_index_r = 0 then
                                next_word.line_start := '1';
                                next_word.gap_before :=
                                    first_cell_word_r.gap_before;
                            end if;
                            next_word.first_column :=
                                first_cell_word_r.first_column;
                            next_word.last_column :=
                                first_cell_word_r.last_column;
                            next_word.slot_count :=
                                first_cell_word_r.slot_count;
                            next_word.cell_word_count :=
                                first_cell_word_r.word_count;
                            next_word.line_faulted :=
                                first_cell_word_r.line_faulted;
                            next_word.shot_context :=
                                first_cell_word_r.shot_context;
                            line_word_r <= next_word;

                            if metadata_index_r + 1 =
                               C_GPX_VDMA_SHOT_META_WORDS then
                                metadata_index_r <= 0;
                                state_r <= ST_FIRST_CELL;
                            else
                                metadata_index_r <= metadata_index_r + 1;
                            end if;
                        end if;

                    when ST_FIRST_CELL =>
                        if output_ready_c = '1' then
                            line_word_r <= fn_line_word_from_cell(
                                first_cell_word_r, line_word_count_r);
                            if first_cell_word_r.line_end = '1' then
                                state_r <= ST_DRAIN_LAST;
                            else
                                state_r <= ST_CELLS;
                            end if;
                        end if;

                    when ST_CELLS =>
                        if i_cell_word.valid = '1' and
                           o_cell_word_ready = '1' then
                            line_word_r <= fn_line_word_from_cell(
                                i_cell_word, line_word_count_r);
                            if i_cell_word.line_end = '1' then
                                state_r <= ST_DRAIN_LAST;
                            end if;
                        end if;

                    when ST_DRAIN_LAST =>
                        if line_word_r.valid = '1' and
                           i_line_word_ready = '1' then
                            state_r <= ST_IDLE;
                        end if;
                end case;
            end if;
        end if;
    end process p_build;

    p_contract : process (i_clk)
        variable accepted_index : natural;
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '1' and i_abort = '0' and
               i_cell_word.valid = '1' and o_cell_word_ready = '1' then
                if state_r = ST_IDLE then
                    assert i_cell_word.line_start = '1' and
                           i_cell_word.word_index = 0 and
                           i_cell_word.slot_index = 0
                        report "V2-B9-J5-001 first Cell word is not line start"
                        severity failure;
                    assert i_cell_word.slot_count /= 0 and
                           i_cell_word.word_count /= 0
                        report "V2-B9-J5-002 zero Shot-Line geometry"
                        severity failure;
                    assert i_cell_word.shot_context.valid = '1' and
                           i_cell_word.shot_context.t0_timestamp_valid = '1'
                        report "V2-B9-J5-003 invalid Shot/T0 context"
                        severity failure;
                else
                    accepted_index := C_GPX_VDMA_SHOT_META_WORDS +
                        to_integer(i_cell_word.slot_index) *
                        to_integer(i_cell_word.word_count) +
                        to_integer(i_cell_word.word_index);
                    assert i_cell_word.line_start = '0' and
                           i_cell_word.shot_context =
                               first_cell_word_r.shot_context and
                           i_cell_word.slot_count =
                               first_cell_word_r.slot_count and
                           i_cell_word.word_count =
                               first_cell_word_r.word_count
                        report "V2-B9-J5-004 geometry/context changed in line"
                        severity failure;
                    assert (i_cell_word.line_end = '1') =
                           (accepted_index + 1 =
                            to_integer(line_word_count_r))
                        report "V2-B9-J5-005 line end/index mismatch"
                        severity failure;
                end if;
            end if;
        end if;
    end process p_contract;

end architecture rtl;
