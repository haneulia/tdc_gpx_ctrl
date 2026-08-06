library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_event_types_pkg.all;
use work.lidar_gpx_data_pkg.all;
use work.lidar_gpx_vdma_pkg.all;

-- J5B preserves the geometric Shot lattice. B8 reports leading/interior holes
-- on the next real line and trailing/all-hole columns on Face close. This block
-- expands those counts into explicit, equal-size canonical 32-bit Hole Lines.
-- It does not own AXIS width, padding, Footer contents or VDMA reconfiguration.
entity lidar_gpx_hole_line_expander is
    port (
        i_clk   : in std_logic;
        i_rst_n : in std_logic;
        i_abort : in std_logic;

        i_active_slot_count      : in gpx_frame_slot_t;
        i_active_cell_word_count : in gpx_vdma_word_count_t;

        i_real_line_word       : in  gpx_vdma_line_word_event_t;
        o_real_line_word_ready : out std_logic;

        i_frame_close_event : in  gpx_frame_close_event_t;
        o_frame_close_ready : out std_logic;

        o_line_word       : out gpx_vdma_line_word_event_t;
        i_line_word_ready : in  std_logic;

        o_frame_close_event : out gpx_frame_close_event_t;
        i_frame_close_ready : in  std_logic;

        o_idle : out std_logic
    );
end entity lidar_gpx_hole_line_expander;

architecture rtl of lidar_gpx_hole_line_expander is

    type state_t is (
        ST_IDLE,
        ST_HOLE_WORDS,
        ST_HOLE_LAST_WAIT,
        ST_REAL_FIRST,
        ST_REAL_WORDS,
        ST_REAL_LAST_WAIT,
        ST_CLOSE_WAIT
    );

    type hole_resume_t is (RESUME_REAL, RESUME_CLOSE);

    signal state_r : state_t := ST_IDLE;
    signal hole_resume_r : hole_resume_t := RESUME_REAL;

    signal first_real_word_r : gpx_vdma_line_word_event_t :=
        C_GPX_VDMA_LINE_WORD_EVENT_IDLE;
    signal pending_close_r : gpx_frame_close_event_t :=
        C_GPX_FRAME_CLOSE_EVENT_IDLE;
    signal hole_context_r : shot_start_event_t :=
        C_SHOT_START_EVENT_IDLE;
    signal hole_shot_index_r : shot_index_t := (others => '0');
    signal hole_last_index_r : shot_index_t := (others => '0');
    signal hole_remaining_r : shot_index_t := (others => '0');
    signal hole_word_index_r : gpx_vdma_line_word_index_t :=
        (others => '0');
    signal hole_line_word_count_r : gpx_vdma_line_word_count_t :=
        (others => '0');
    signal hole_slot_count_r : gpx_frame_slot_t := (others => '0');
    signal hole_cell_word_count_r : gpx_vdma_word_count_t :=
        (others => '0');
    signal hole_faulted_r : std_logic := '0';

    signal line_word_r : gpx_vdma_line_word_event_t :=
        C_GPX_VDMA_LINE_WORD_EVENT_IDLE;
    signal frame_close_r : gpx_frame_close_event_t :=
        C_GPX_FRAME_CLOSE_EVENT_IDLE;
    signal line_output_ready_c : std_logic;

    function fn_line_word_count(
        slot_count      : gpx_frame_slot_t;
        cell_word_count : gpx_vdma_word_count_t
    ) return gpx_vdma_line_word_count_t is
        variable result : natural;
    begin
        result := C_GPX_VDMA_SHOT_META_WORDS +
            to_integer(slot_count) * to_integer(cell_word_count);
        return to_unsigned(result, gpx_vdma_line_word_count_t'length);
    end function fn_line_word_count;

    function fn_hole_context_from_real(
        next_context : shot_start_event_t;
        shot_index   : shot_index_t
    ) return shot_start_event_t is
        variable result : shot_start_event_t := next_context;
    begin
        result.valid := '1';
        result.request.valid := '0';
        result.request.position := (others => '0');
        result.request.shot_index := shot_index;
        result.request.last_in_face := '0';
        result.request.source_latency_clks := (others => '0');
        result.request.source_latency_valid := '0';
        result.fire_to_t0_clks := (others => '0');
        result.t0_timestamp_ticks := (others => '0');
        result.t0_timestamp_valid := '0';
        result.t0_time_sync_valid := '0';
        return result;
    end function fn_hole_context_from_real;

    function fn_hole_context_from_close(
        close_event : gpx_frame_close_event_t;
        shot_index  : shot_index_t
    ) return shot_start_event_t is
        variable result : shot_start_event_t :=
            C_SHOT_START_EVENT_IDLE;
    begin
        result.valid := '1';
        result.request.face_index := close_event.face_index;
        result.request.direction := close_event.direction;
        result.request.shot_index := shot_index;
        result.request.source_sim := close_event.source_sim;
        result.request.active_version := close_event.active_version;
        return result;
    end function fn_hole_context_from_close;

    function fn_make_hole_word(
        context_value   : shot_start_event_t;
        shot_index      : shot_index_t;
        last_index      : shot_index_t;
        word_index      : gpx_vdma_line_word_index_t;
        line_word_count : gpx_vdma_line_word_count_t;
        slot_count      : gpx_frame_slot_t;
        cell_word_count : gpx_vdma_word_count_t;
        line_faulted    : std_logic
    ) return gpx_vdma_line_word_event_t is
        variable result : gpx_vdma_line_word_event_t :=
            C_GPX_VDMA_LINE_WORD_EVENT_IDLE;
        variable context_v : shot_start_event_t := context_value;
        variable status_v : gpx_vdma_shot_status_t :=
            C_GPX_VDMA_SHOT_STATUS_CLEAR;
        variable index_v : natural := to_integer(word_index);
    begin
        context_v.request.shot_index := shot_index;
        context_v.request.last_in_face := '0';
        if shot_index = last_index then
            context_v.request.last_in_face := '1';
        end if;

        status_v.hole := '1';
        status_v.line_faulted := line_faulted;

        result.valid := '1';
        if index_v < C_GPX_VDMA_SHOT_META_WORDS then
            result.data := fn_gpx_vdma_shot_metadata_word(
                context_v, status_v, index_v);
            result.kind := GPX_VDMA_LINE_SHOT_METADATA;
        else
            result.data := (others => '0');
            result.kind := GPX_VDMA_LINE_CELL_DATA;
        end if;
        result.word_index := word_index;
        result.line_word_count := line_word_count;
        if word_index = 0 then
            result.line_start := '1';
        end if;
        if word_index + 1 = line_word_count then
            result.line_end := '1';
        end if;
        if shot_index = 0 then
            result.first_column := '1';
        end if;
        if shot_index = last_index then
            result.last_column := '1';
        end if;
        result.slot_count := slot_count;
        result.cell_word_count := cell_word_count;
        result.line_hole := '1';
        result.line_faulted := line_faulted;
        result.shot_context := context_v;
        return result;
    end function fn_make_hole_word;

begin

    line_output_ready_c <= '1' when line_word_r.valid = '0' or
        i_line_word_ready = '1' else '0';

    o_real_line_word_ready <= '1' when i_rst_n = '1' and i_abort = '0' and
        ((state_r = ST_IDLE and i_frame_close_event.valid = '0') or
         (state_r = ST_REAL_WORDS and line_output_ready_c = '1')) else '0';

    o_frame_close_ready <= '1' when i_rst_n = '1' and i_abort = '0' and
        state_r = ST_IDLE and i_real_line_word.valid = '0' else '0';

    o_line_word <= line_word_r;
    o_frame_close_event <= frame_close_r;
    o_idle <= '1' when state_r = ST_IDLE and line_word_r.valid = '0' and
        frame_close_r.valid = '0' else '0';

    p_expand : process (i_clk)
        variable first_v : gpx_vdma_line_word_event_t;
        variable close_v : gpx_frame_close_event_t;
        variable context_v : shot_start_event_t;
        variable line_count_v : gpx_vdma_line_word_count_t;
        variable start_index_v : shot_index_t;
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' or i_abort = '1' then
                state_r <= ST_IDLE;
                hole_resume_r <= RESUME_REAL;
                first_real_word_r <= C_GPX_VDMA_LINE_WORD_EVENT_IDLE;
                pending_close_r <= C_GPX_FRAME_CLOSE_EVENT_IDLE;
                hole_context_r <= C_SHOT_START_EVENT_IDLE;
                hole_shot_index_r <= (others => '0');
                hole_last_index_r <= (others => '0');
                hole_remaining_r <= (others => '0');
                hole_word_index_r <= (others => '0');
                hole_line_word_count_r <= (others => '0');
                hole_slot_count_r <= (others => '0');
                hole_cell_word_count_r <= (others => '0');
                hole_faulted_r <= '0';
                line_word_r <= C_GPX_VDMA_LINE_WORD_EVENT_IDLE;
                frame_close_r <= C_GPX_FRAME_CLOSE_EVENT_IDLE;
            else
                if line_word_r.valid = '1' and i_line_word_ready = '1' then
                    line_word_r.valid <= '0';
                end if;
                if frame_close_r.valid = '1' and i_frame_close_ready = '1' then
                    frame_close_r.valid <= '0';
                end if;

                case state_r is
                    when ST_IDLE =>
                        if i_real_line_word.valid = '1' and
                           o_real_line_word_ready = '1' then
                            first_v := i_real_line_word;
                            first_real_word_r <= first_v;
                            if first_v.gap_before /= 0 then
                                assert first_v.shot_context.request.shot_index >=
                                       first_v.gap_before
                                    report "V2-B9-J5B-001 gap exceeds Shot index"
                                    severity failure;
                                if first_v.shot_context.request.shot_index >=
                                   first_v.gap_before then
                                    start_index_v :=
                                        first_v.shot_context.request.shot_index -
                                        first_v.gap_before;
                                else
                                    start_index_v := (others => '0');
                                end if;
                                context_v := fn_hole_context_from_real(
                                    first_v.shot_context, start_index_v);
                                hole_context_r <= context_v;
                                hole_shot_index_r <= start_index_v;
                                hole_last_index_r <= (others => '1');
                                hole_remaining_r <= first_v.gap_before;
                                hole_word_index_r <= (others => '0');
                                hole_line_word_count_r <=
                                    first_v.line_word_count;
                                hole_slot_count_r <= first_v.slot_count;
                                hole_cell_word_count_r <=
                                    first_v.cell_word_count;
                                hole_faulted_r <= '0';
                                hole_resume_r <= RESUME_REAL;
                                state_r <= ST_HOLE_WORDS;
                            else
                                state_r <= ST_REAL_FIRST;
                            end if;
                        elsif i_frame_close_event.valid = '1' and
                              o_frame_close_ready = '1' then
                            close_v := i_frame_close_event;
                            pending_close_r <= close_v;
                            if close_v.trailing_gap /= 0 then
                                assert i_active_slot_count /= 0 and
                                       i_active_cell_word_count /= 0
                                    report "V2-B9-J5B-002 zero active Hole geometry"
                                    severity failure;
                                assert close_v.trailing_gap <=
                                       close_v.columns_per_face
                                    report "V2-B9-J5B-003 trailing gap exceeds Face"
                                    severity failure;
                                start_index_v := close_v.columns_per_face -
                                    close_v.trailing_gap;
                                context_v := fn_hole_context_from_close(
                                    close_v, start_index_v);
                                line_count_v := fn_line_word_count(
                                    i_active_slot_count,
                                    i_active_cell_word_count);
                                hole_context_r <= context_v;
                                hole_shot_index_r <= start_index_v;
                                hole_last_index_r <=
                                    close_v.columns_per_face - 1;
                                hole_remaining_r <= close_v.trailing_gap;
                                hole_word_index_r <= (others => '0');
                                hole_line_word_count_r <= line_count_v;
                                hole_slot_count_r <= i_active_slot_count;
                                hole_cell_word_count_r <=
                                    i_active_cell_word_count;
                                hole_faulted_r <= close_v.face_faulted;
                                hole_resume_r <= RESUME_CLOSE;
                                state_r <= ST_HOLE_WORDS;
                            else
                                frame_close_r <= close_v;
                                state_r <= ST_CLOSE_WAIT;
                            end if;
                        end if;

                    when ST_HOLE_WORDS =>
                        if line_output_ready_c = '1' then
                            line_word_r <= fn_make_hole_word(
                                hole_context_r,
                                hole_shot_index_r,
                                hole_last_index_r,
                                hole_word_index_r,
                                hole_line_word_count_r,
                                hole_slot_count_r,
                                hole_cell_word_count_r,
                                hole_faulted_r);
                            if hole_word_index_r + 1 =
                               hole_line_word_count_r then
                                hole_word_index_r <= (others => '0');
                                if hole_remaining_r = 1 then
                                    state_r <= ST_HOLE_LAST_WAIT;
                                else
                                    hole_remaining_r <=
                                        hole_remaining_r - 1;
                                    hole_shot_index_r <=
                                        hole_shot_index_r + 1;
                                end if;
                            else
                                hole_word_index_r <= hole_word_index_r + 1;
                            end if;
                        end if;

                    when ST_HOLE_LAST_WAIT =>
                        if line_word_r.valid = '1' and
                           i_line_word_ready = '1' then
                            if hole_resume_r = RESUME_REAL then
                                state_r <= ST_REAL_FIRST;
                            else
                                frame_close_r <= pending_close_r;
                                state_r <= ST_CLOSE_WAIT;
                            end if;
                        end if;

                    when ST_REAL_FIRST =>
                        if line_output_ready_c = '1' then
                            first_v := first_real_word_r;
                            first_v.gap_before := (others => '0');
                            line_word_r <= first_v;
                            if first_v.line_end = '1' then
                                state_r <= ST_REAL_LAST_WAIT;
                            else
                                state_r <= ST_REAL_WORDS;
                            end if;
                        end if;

                    when ST_REAL_WORDS =>
                        if i_real_line_word.valid = '1' and
                           o_real_line_word_ready = '1' then
                            first_v := i_real_line_word;
                            first_v.gap_before := (others => '0');
                            line_word_r <= first_v;
                            if first_v.line_end = '1' then
                                state_r <= ST_REAL_LAST_WAIT;
                            end if;
                        end if;

                    when ST_REAL_LAST_WAIT =>
                        if line_word_r.valid = '1' and
                           i_line_word_ready = '1' then
                            state_r <= ST_IDLE;
                        end if;

                    when ST_CLOSE_WAIT =>
                        if frame_close_r.valid = '1' and
                           i_frame_close_ready = '1' then
                            state_r <= ST_IDLE;
                        end if;
                end case;
            end if;
        end if;
    end process p_expand;

    p_contract : process (i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '1' and i_abort = '0' then
                if i_real_line_word.valid = '1' and
                   o_real_line_word_ready = '1' and state_r = ST_IDLE then
                    assert i_real_line_word.line_start = '1' and
                           i_real_line_word.word_index = 0 and
                           i_real_line_word.kind =
                               GPX_VDMA_LINE_SHOT_METADATA
                        report "V2-B9-J5B-004 accepted real input mid-Line"
                        severity failure;
                    assert i_real_line_word.line_word_count =
                           fn_line_word_count(
                               i_real_line_word.slot_count,
                               i_real_line_word.cell_word_count)
                        report "V2-B9-J5B-005 real Line geometry mismatch"
                        severity failure;
                    assert i_real_line_word.slot_count =
                               i_active_slot_count and
                           i_real_line_word.cell_word_count =
                               i_active_cell_word_count
                        report "V2-B9-J5B-006 active geometry changed in Face"
                        severity failure;
                end if;
                if i_frame_close_event.valid = '1' and
                   o_frame_close_ready = '1' then
                    assert i_frame_close_event.columns_per_face /= 0
                        report "V2-B9-J5B-007 zero-column Face close"
                        severity failure;
                    assert (i_frame_close_event.all_hole = '0') or
                           (i_frame_close_event.trailing_gap =
                            i_frame_close_event.columns_per_face)
                        report "V2-B9-J5B-008 all-Hole close mismatch"
                        severity failure;
                end if;
                if line_word_r.valid = '1' and
                   line_word_r.line_hole = '1' then
                    assert line_word_r.gap_before = 0 and
                           line_word_r.shot_context.t0_timestamp_valid = '0'
                        report "V2-B9-J5B-009 Hole contains gap or valid T0"
                        severity failure;
                end if;
            end if;
        end if;
    end process p_contract;

end architecture rtl;
