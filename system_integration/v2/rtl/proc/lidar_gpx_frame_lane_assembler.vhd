library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_build_pkg.all;
use work.lidar_event_types_pkg.all;
use work.lidar_gpx_event_pkg.all;
use work.lidar_gpx_data_pkg.all;

-- B8 canonical Frame-lane assembler.
--
-- One accepted Shot becomes at most one Rise line and one Fall line. Cells
-- are reordered as ascending logical Chip then ascending STOP. The two output
-- lanes have independent registered ready/valid holding registers. No AXIS
-- width, byte packing, header prefix or VDMA alignment exists at this boundary.
entity lidar_gpx_frame_lane_assembler is
    generic (
        G_BUILD_CONFIG : lidar_build_config_t := C_DEFAULT_BUILD_CONFIG
    );
    port (
        i_clk          : in  std_logic;
        i_rst_n        : in  std_logic;
        i_abort        : in  std_logic;
        i_clear_sticky : in  std_logic;

        i_active_version   : in unsigned(15 downto 0);
        i_active_rise_mask : in chip_mask_t;
        i_active_fall_mask : in chip_mask_t;
        i_columns_per_face : in unsigned(15 downto 0);

        i_cell_event : in  gpx_cell_event_t;
        o_cell_ready : out std_logic;

        o_rise_event : out gpx_frame_cell_event_t;
        i_rise_ready : in  std_logic;
        o_fall_event : out gpx_frame_cell_event_t;
        i_fall_ready : in  std_logic;

        o_rise_line_done : out std_logic;
        o_fall_line_done : out std_logic;
        o_shot_done      : out std_logic;
        o_idle           : out std_logic;

        o_fault_pulse  : out gpx_frame_assembler_faults_t;
        o_fault_sticky : out gpx_frame_assembler_faults_t
    );
end entity lidar_gpx_frame_lane_assembler;

architecture rtl of lidar_gpx_frame_lane_assembler is

    constant C_CELLS_PER_SLOPE : positive :=
        C_MAX_CHIPS * C_MAX_STOPS_PER_CHIP;
    constant C_HITS_PACKED_WIDTH : positive :=
        C_MAX_RETURNS_PER_STOP * C_GPX_HIT_WIDTH;
    constant C_CELL_WORD_WIDTH : positive :=
        C_HITS_PACKED_WIDTH + 3 + 3 + 1 + 1 + 1 + 3 + 16;

    constant C_HIT_COUNT_LO : natural := C_HITS_PACKED_WIDTH;
    constant C_HIT_COUNT_HI : natural := C_HIT_COUNT_LO + 2;
    constant C_MAX_HITS_LO  : natural := C_HIT_COUNT_HI + 1;
    constant C_MAX_HITS_HI  : natural := C_MAX_HITS_LO + 2;
    constant C_HIT_DROPPED  : natural := C_MAX_HITS_HI + 1;
    constant C_ERROR_FILL   : natural := C_HIT_DROPPED + 1;
    constant C_FAULTED      : natural := C_ERROR_FILL + 1;
    constant C_TIMEOUT_LO   : natural := C_FAULTED + 1;
    constant C_TIMEOUT_HI   : natural := C_TIMEOUT_LO + 2;
    constant C_CHIP_SEQ_LO  : natural := C_TIMEOUT_HI + 1;
    constant C_CHIP_SEQ_HI  : natural := C_CHIP_SEQ_LO + 15;

    subtype cell_word_t is std_logic_vector(
        C_CELL_WORD_WIDTH - 1 downto 0);
    type cell_memory_t is array (0 to C_CELLS_PER_SLOPE - 1) of
        cell_word_t;
    type chip_sequence_array_t is array (0 to C_MAX_CHIPS - 1) of
        unsigned(15 downto 0);

    type assembler_state_t is (
        S_COLLECT,
        S_EVENT_CHECK,
        S_EVENT_APPLY,
        S_EMIT_INIT,
        S_EMIT
    );

    signal state_r : assembler_state_t := S_COLLECT;

    -- Payload RAM has no reset. Per-Shot presence bits are the sole validity
    -- owner, so stale RAM contents can never become visible after reset/abort.
    signal rise_memory_r : cell_memory_t;
    signal fall_memory_r : cell_memory_t;
    signal rise_present_r : std_logic_vector(
        C_CELLS_PER_SLOPE - 1 downto 0) := (others => '0');
    signal fall_present_r : std_logic_vector(
        C_CELLS_PER_SLOPE - 1 downto 0) := (others => '0');

    attribute ram_style : string;
    attribute ram_style of rise_memory_r : signal is "distributed";
    attribute ram_style of fall_memory_r : signal is "distributed";

    signal shot_active_r  : std_logic := '0';
    signal pending_event_r : gpx_cell_event_t := C_GPX_CELL_EVENT_IDLE;
    signal pending_context_match_r : std_logic := '0';
    signal shot_context_r : shot_start_event_t := C_SHOT_START_EVENT_IDLE;
    signal shot_max_hits_r : unsigned(2 downto 0) := to_unsigned(1, 3);
    signal shot_rise_mask_r : chip_mask_t := (others => '0');
    signal shot_fall_mask_r : chip_mask_t := (others => '0');
    signal shot_terminal_r  : chip_mask_t := (others => '0');
    signal shot_columns_r   : unsigned(15 downto 0) := (others => '0');
    signal shot_gap_before_r : shot_index_t := (others => '0');
    signal shot_faulted_r   : std_logic := '0';
    signal chip_sequence_r : chip_sequence_array_t :=
        (others => (others => '0'));

    signal history_valid_r   : std_logic := '0';
    signal history_face_r    : face_index_t := (others => '0');
    signal history_version_r : unsigned(15 downto 0) := (others => '0');
    signal history_column_r  : shot_index_t := (others => '0');
    signal history_last_r    : std_logic := '0';

    signal rise_emit_active_r : std_logic := '0';
    signal fall_emit_active_r : std_logic := '0';
    signal rise_emit_chip_r : natural range 0 to C_MAX_CHIPS - 1 := 0;
    signal fall_emit_chip_r : natural range 0 to C_MAX_CHIPS - 1 := 0;
    signal rise_emit_stop_r : natural range 0 to
        C_MAX_STOPS_PER_CHIP - 1 := 0;
    signal fall_emit_stop_r : natural range 0 to
        C_MAX_STOPS_PER_CHIP - 1 := 0;
    signal rise_emit_slot_r : natural range 0 to C_CELLS_PER_SLOPE := 0;
    signal fall_emit_slot_r : natural range 0 to C_CELLS_PER_SLOPE := 0;
    signal rise_slot_count_r : natural range 0 to C_CELLS_PER_SLOPE := 0;
    signal fall_slot_count_r : natural range 0 to C_CELLS_PER_SLOPE := 0;

    signal rise_event_r : gpx_frame_cell_event_t :=
        C_GPX_FRAME_CELL_EVENT_IDLE;
    signal fall_event_r : gpx_frame_cell_event_t :=
        C_GPX_FRAME_CELL_EVENT_IDLE;
    signal rise_line_done_r : std_logic := '0';
    signal fall_line_done_r : std_logic := '0';
    signal shot_done_r      : std_logic := '0';

    signal fault_pulse_r : gpx_frame_assembler_faults_t :=
        C_GPX_FRAME_ASSEMBLER_FAULTS_CLEAR;
    signal fault_sticky_r : gpx_frame_assembler_faults_t :=
        C_GPX_FRAME_ASSEMBLER_FAULTS_CLEAR;

    function fn_cell_address(
        chip_index : natural;
        stop_index : natural
    ) return natural is
    begin
        return chip_index * C_MAX_STOPS_PER_CHIP + stop_index;
    end function fn_cell_address;

    function fn_pack_cell(value : gpx_cell_event_t) return cell_word_t is
        variable result : cell_word_t := (others => '0');
        variable bit_lo : natural;
    begin
        for hit_index in 0 to C_MAX_RETURNS_PER_STOP - 1 loop
            bit_lo := hit_index * C_GPX_HIT_WIDTH;
            result(bit_lo + C_GPX_HIT_WIDTH - 1 downto bit_lo) :=
                std_logic_vector(value.hits(hit_index));
        end loop;
        result(C_HIT_COUNT_HI downto C_HIT_COUNT_LO) :=
            std_logic_vector(value.hit_count);
        result(C_MAX_HITS_HI downto C_MAX_HITS_LO) :=
            std_logic_vector(value.max_hits);
        result(C_HIT_DROPPED) := value.hit_dropped;
        result(C_ERROR_FILL)  := value.error_fill;
        result(C_FAULTED)     := value.faulted;
        result(C_TIMEOUT_HI downto C_TIMEOUT_LO) := value.timeout_cause;
        result(C_CHIP_SEQ_HI downto C_CHIP_SEQ_LO) :=
            std_logic_vector(value.chip_shot_seq);
        return result;
    end function fn_pack_cell;

    function fn_unpack_cell(
        word_value    : cell_word_t;
        present_value : std_logic;
        chip_index    : natural;
        stop_index    : natural;
        slope_value   : gpx_slope_t;
        context_value : shot_start_event_t;
        max_hits_value : unsigned(2 downto 0);
        sequence_value : unsigned(15 downto 0)
    ) return gpx_cell_event_t is
        variable result : gpx_cell_event_t := C_GPX_CELL_EVENT_IDLE;
        variable bit_lo : natural;
    begin
        result.valid        := '1';
        result.kind         := GPX_CELL_DATA;
        result.chip_index   := to_unsigned(chip_index, 2);
        result.stop_index   := to_unsigned(stop_index, 3);
        result.slope        := slope_value;
        result.shot_context := context_value;
        result.chip_shot_seq := sequence_value;
        if stop_index >= 4 then
            result.ififo_id := '1';
        end if;

        if present_value = '1' then
            for hit_index in 0 to C_MAX_RETURNS_PER_STOP - 1 loop
                bit_lo := hit_index * C_GPX_HIT_WIDTH;
                result.hits(hit_index) := unsigned(word_value(
                    bit_lo + C_GPX_HIT_WIDTH - 1 downto bit_lo));
            end loop;
            result.hit_count := unsigned(word_value(
                C_HIT_COUNT_HI downto C_HIT_COUNT_LO));
            result.max_hits := unsigned(word_value(
                C_MAX_HITS_HI downto C_MAX_HITS_LO));
            result.hit_dropped := word_value(C_HIT_DROPPED);
            result.error_fill  := word_value(C_ERROR_FILL);
            result.faulted     := word_value(C_FAULTED);
            result.timeout_cause := word_value(
                C_TIMEOUT_HI downto C_TIMEOUT_LO);
            result.chip_shot_seq := unsigned(word_value(
                C_CHIP_SEQ_HI downto C_CHIP_SEQ_LO));
        else
            result.max_hits   := max_hits_value;
            result.error_fill := '1';
            result.faulted    := '1';
        end if;
        return result;
    end function fn_unpack_cell;

    function fn_expected_cells(mask_value : chip_mask_t)
        return std_logic_vector is
        variable result : std_logic_vector(
            C_CELLS_PER_SLOPE - 1 downto 0) := (others => '0');
    begin
        for chip_index in 0 to C_MAX_CHIPS - 1 loop
            if mask_value(chip_index) = '1' then
                for stop_index in 0 to G_BUILD_CONFIG.stops_per_chip - 1 loop
                    result(fn_cell_address(chip_index, stop_index)) := '1';
                end loop;
            end if;
        end loop;
        return result;
    end function fn_expected_cells;

    function fn_first_chip(mask_value : chip_mask_t) return natural is
    begin
        for chip_index in 0 to C_MAX_CHIPS - 1 loop
            if mask_value(chip_index) = '1' then
                return chip_index;
            end if;
        end loop;
        return 0;
    end function fn_first_chip;

    function fn_next_chip(
        mask_value   : chip_mask_t;
        current_chip : natural
    ) return natural is
    begin
        for chip_index in 0 to C_MAX_CHIPS - 1 loop
            if chip_index > current_chip and mask_value(chip_index) = '1' then
                return chip_index;
            end if;
        end loop;
        return current_chip;
    end function fn_next_chip;

    function fn_make_frame_event(
        word_value     : cell_word_t;
        present_value  : std_logic;
        chip_index     : natural;
        stop_index     : natural;
        slope_value    : gpx_slope_t;
        slot_index     : natural;
        slot_count     : natural;
        context_value  : shot_start_event_t;
        max_hits_value : unsigned(2 downto 0);
        sequence_value : unsigned(15 downto 0);
        gap_value      : shot_index_t;
        line_faulted   : std_logic
    ) return gpx_frame_cell_event_t is
        variable result : gpx_frame_cell_event_t :=
            C_GPX_FRAME_CELL_EVENT_IDLE;
    begin
        result.valid := '1';
        result.cell := fn_unpack_cell(
            word_value, present_value, chip_index, stop_index,
            slope_value, context_value, max_hits_value, sequence_value);
        result.slot_index := to_unsigned(slot_index, result.slot_index'length);
        result.slot_count := to_unsigned(slot_count, result.slot_count'length);
        if slot_index = 0 then
            result.line_start := '1';
            result.gap_before := gap_value;
        end if;
        if slot_index + 1 = slot_count then
            result.line_end := '1';
        end if;
        if context_value.request.shot_index = 0 then
            result.first_column := '1';
        end if;
        result.last_column  := context_value.request.last_in_face;
        result.slot_blank   := not present_value;
        result.line_faulted := line_faulted or not present_value or
            result.cell.faulted or result.cell.error_fill;
        return result;
    end function fn_make_frame_event;

begin

    assert fn_validate_build_config(G_BUILD_CONFIG) = CFG_OK
        report "V2-B8-001 illegal build configuration"
        severity failure;

    assert C_CHIP_SEQ_HI = C_CELL_WORD_WIDTH - 1
        report "V2-B8-002 Cell storage layout mismatch"
        severity failure;

    o_cell_ready <= '1' when i_rst_n = '1' and i_abort = '0' and
                            state_r = S_COLLECT else
                    '0';
    o_rise_event <= rise_event_r;
    o_fall_event <= fall_event_r;
    o_rise_line_done <= rise_line_done_r;
    o_fall_line_done <= fall_line_done_r;
    o_shot_done      <= shot_done_r;
    o_idle <= '1' when state_r = S_COLLECT and shot_active_r = '0' else '0';
    o_fault_pulse  <= fault_pulse_r;
    o_fault_sticky <= fault_sticky_r;

    p_assemble : process (i_clk)
        variable chip_index : natural range 0 to C_MAX_CHIPS - 1;
        variable stop_index : natural range 0 to C_MAX_STOPS_PER_CHIP - 1;
        variable address_value : natural range 0 to
            C_CELLS_PER_SLOPE - 1;
        variable rise_present_v : std_logic_vector(
            C_CELLS_PER_SLOPE - 1 downto 0);
        variable fall_present_v : std_logic_vector(
            C_CELLS_PER_SLOPE - 1 downto 0);
        variable terminal_v : chip_mask_t;
        variable expected_chip_v : chip_mask_t;
        variable expected_rise_v : std_logic_vector(
            C_CELLS_PER_SLOPE - 1 downto 0);
        variable expected_fall_v : std_logic_vector(
            C_CELLS_PER_SLOPE - 1 downto 0);
        variable rise_mask_v : chip_mask_t;
        variable fall_mask_v : chip_mask_t;
        variable context_v : shot_start_event_t;
        variable columns_v : unsigned(15 downto 0);
        variable gap_v : shot_index_t;
        variable shot_fault_v : std_logic;
        variable geometry_fault_v : boolean;
        variable shot_complete_v : boolean;
        variable rise_empty_after_v : boolean;
        variable fall_empty_after_v : boolean;
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                state_r <= S_COLLECT;
                rise_present_r <= (others => '0');
                fall_present_r <= (others => '0');
                shot_active_r <= '0';
                pending_event_r <= C_GPX_CELL_EVENT_IDLE;
                pending_context_match_r <= '0';
                shot_context_r <= C_SHOT_START_EVENT_IDLE;
                shot_max_hits_r <= to_unsigned(1, 3);
                shot_rise_mask_r <= (others => '0');
                shot_fall_mask_r <= (others => '0');
                shot_terminal_r <= (others => '0');
                shot_columns_r <= (others => '0');
                shot_gap_before_r <= (others => '0');
                shot_faulted_r <= '0';
                chip_sequence_r <= (others => (others => '0'));
                history_valid_r <= '0';
                history_face_r <= (others => '0');
                history_version_r <= (others => '0');
                history_column_r <= (others => '0');
                history_last_r <= '0';
                rise_emit_active_r <= '0';
                fall_emit_active_r <= '0';
                rise_emit_chip_r <= 0;
                fall_emit_chip_r <= 0;
                rise_emit_stop_r <= 0;
                fall_emit_stop_r <= 0;
                rise_emit_slot_r <= 0;
                fall_emit_slot_r <= 0;
                rise_slot_count_r <= 0;
                fall_slot_count_r <= 0;
                rise_event_r <= C_GPX_FRAME_CELL_EVENT_IDLE;
                fall_event_r <= C_GPX_FRAME_CELL_EVENT_IDLE;
                rise_line_done_r <= '0';
                fall_line_done_r <= '0';
                shot_done_r <= '0';
                fault_pulse_r <= C_GPX_FRAME_ASSEMBLER_FAULTS_CLEAR;
                fault_sticky_r <= C_GPX_FRAME_ASSEMBLER_FAULTS_CLEAR;
            else
                rise_line_done_r <= '0';
                fall_line_done_r <= '0';
                shot_done_r <= '0';
                fault_pulse_r <= C_GPX_FRAME_ASSEMBLER_FAULTS_CLEAR;

                if i_clear_sticky = '1' then
                    fault_sticky_r <= C_GPX_FRAME_ASSEMBLER_FAULTS_CLEAR;
                end if;

                if i_abort = '1' then
                    state_r <= S_COLLECT;
                    rise_present_r <= (others => '0');
                    fall_present_r <= (others => '0');
                    shot_active_r <= '0';
                    pending_event_r <= C_GPX_CELL_EVENT_IDLE;
                    pending_context_match_r <= '0';
                    shot_terminal_r <= (others => '0');
                    shot_faulted_r <= '0';
                    history_valid_r <= '0';
                    rise_emit_active_r <= '0';
                    fall_emit_active_r <= '0';
                    rise_event_r <= C_GPX_FRAME_CELL_EVENT_IDLE;
                    fall_event_r <= C_GPX_FRAME_CELL_EVENT_IDLE;
                elsif state_r = S_COLLECT then
                    if i_cell_event.valid = '1' then
                        pending_event_r <= i_cell_event;
                        state_r <= S_EVENT_CHECK;
                    end if;

                elsif state_r = S_EVENT_CHECK then
                    if shot_active_r = '0' then
                        rise_mask_v := i_active_rise_mask;
                        fall_mask_v := i_active_fall_mask;
                        context_v := pending_event_r.shot_context;
                        columns_v := i_columns_per_face;
                        gap_v := (others => '0');
                        geometry_fault_v := false;
                        expected_chip_v := rise_mask_v or fall_mask_v;

                        if expected_chip_v = "0000" or
                           columns_v = 0 or
                           context_v.request.active_version /=
                               i_active_version or
                           context_v.request.shot_index >= columns_v then
                            geometry_fault_v := true;
                        end if;

                        if history_valid_r = '0' or
                           history_version_r /= i_active_version or
                           history_face_r /= context_v.request.face_index or
                           history_last_r = '1' then
                            gap_v := context_v.request.shot_index;
                            if history_valid_r = '1' and
                               history_last_r = '0' and
                               (history_version_r /= i_active_version or
                                history_face_r /=
                                    context_v.request.face_index) then
                                geometry_fault_v := true;
                            end if;
                        elsif context_v.request.shot_index >
                              history_column_r then
                            gap_v := context_v.request.shot_index -
                                history_column_r - 1;
                        else
                            gap_v := context_v.request.shot_index;
                            geometry_fault_v := true;
                        end if;

                        if context_v.request.shot_index + 1 >= columns_v then
                            if context_v.request.last_in_face /= '1' then
                                geometry_fault_v := true;
                            end if;
                        elsif context_v.request.last_in_face /= '0' then
                            geometry_fault_v := true;
                        end if;

                        rise_present_r <= (others => '0');
                        fall_present_r <= (others => '0');
                        shot_terminal_r <= (others => '0');
                        shot_active_r <= '1';
                        shot_context_r <= context_v;
                        shot_max_hits_r <= pending_event_r.max_hits;
                        shot_rise_mask_r <= rise_mask_v;
                        shot_fall_mask_r <= fall_mask_v;
                        shot_columns_r <= columns_v;
                        shot_gap_before_r <= gap_v;
                        shot_faulted_r <= '0';
                        pending_context_match_r <= '1';

                        history_valid_r <= '1';
                        history_face_r <= context_v.request.face_index;
                        history_version_r <= context_v.request.active_version;
                        history_column_r <= context_v.request.shot_index;
                        history_last_r <= context_v.request.last_in_face;

                        if gap_v /= 0 then
                            fault_pulse_r.column_gap <= '1';
                            fault_sticky_r.column_gap <= '1';
                        end if;
                        if geometry_fault_v then
                            shot_faulted_r <= '1';
                            fault_pulse_r.geometry_error <= '1';
                            fault_sticky_r.geometry_error <= '1';
                        end if;
                    elsif pending_event_r.shot_context = shot_context_r and
                          i_active_version =
                              shot_context_r.request.active_version and
                          i_active_rise_mask = shot_rise_mask_r and
                          i_active_fall_mask = shot_fall_mask_r and
                          i_columns_per_face = shot_columns_r then
                        pending_context_match_r <= '1';
                    else
                        pending_context_match_r <= '0';
                    end if;
                    state_r <= S_EVENT_APPLY;

                elsif state_r = S_EVENT_APPLY then
                    chip_index := to_integer(pending_event_r.chip_index);
                    stop_index := to_integer(pending_event_r.stop_index);
                    rise_present_v := rise_present_r;
                    fall_present_v := fall_present_r;
                    terminal_v := shot_terminal_r;
                    rise_mask_v := shot_rise_mask_r;
                    fall_mask_v := shot_fall_mask_r;
                    shot_fault_v := shot_faulted_r;
                    shot_complete_v := false;

                    if pending_context_match_r /= '1' then
                            shot_fault_v := '1';
                            fault_pulse_r.context_mismatch <= '1';
                            fault_sticky_r.context_mismatch <= '1';
                    elsif chip_index >= G_BUILD_CONFIG.num_chips or
                          stop_index >= G_BUILD_CONFIG.stops_per_chip then
                        shot_fault_v := '1';
                        fault_pulse_r.unexpected_cell <= '1';
                        fault_sticky_r.unexpected_cell <= '1';
                    else
                        expected_chip_v := rise_mask_v or fall_mask_v;
                        if expected_chip_v(chip_index) /= '1' then
                            shot_fault_v := '1';
                            fault_pulse_r.unexpected_cell <= '1';
                            fault_sticky_r.unexpected_cell <= '1';
                        elsif pending_event_r.kind = GPX_CELL_DATA then
                            address_value := fn_cell_address(
                                chip_index, stop_index);
                            if pending_event_r.slope = GPX_SLOPE_RISE and
                               rise_mask_v(chip_index) = '1' then
                                if rise_present_v(address_value) = '1' then
                                    shot_fault_v := '1';
                                    fault_pulse_r.duplicate_cell <= '1';
                                    fault_sticky_r.duplicate_cell <= '1';
                                else
                                    rise_memory_r(address_value) <=
                                        fn_pack_cell(pending_event_r);
                                    rise_present_v(address_value) := '1';
                                end if;
                            elsif pending_event_r.slope = GPX_SLOPE_FALL and
                                  fall_mask_v(chip_index) = '1' then
                                if fall_present_v(address_value) = '1' then
                                    shot_fault_v := '1';
                                    fault_pulse_r.duplicate_cell <= '1';
                                    fault_sticky_r.duplicate_cell <= '1';
                                else
                                    fall_memory_r(address_value) <=
                                        fn_pack_cell(pending_event_r);
                                    fall_present_v(address_value) := '1';
                                end if;
                            elsif pending_event_r.hit_count /= 0 then
                                shot_fault_v := '1';
                                fault_pulse_r.masked_payload_drop <= '1';
                                fault_sticky_r.masked_payload_drop <= '1';
                            end if;

                            if pending_event_r.faulted = '1' or
                               pending_event_r.error_fill = '1' then
                                shot_fault_v := '1';
                            end if;
                            chip_sequence_r(chip_index) <=
                                pending_event_r.chip_shot_seq;
                        elsif pending_event_r.kind = GPX_CELL_DRAIN_DONE or
                              pending_event_r.kind = GPX_CELL_TIMEOUT then
                            if terminal_v(chip_index) = '1' then
                                shot_fault_v := '1';
                                fault_pulse_r.duplicate_terminal <= '1';
                                fault_sticky_r.duplicate_terminal <= '1';
                            else
                                terminal_v(chip_index) := '1';
                            end if;
                            chip_sequence_r(chip_index) <=
                                pending_event_r.chip_shot_seq;
                            if pending_event_r.faulted = '1' or
                               pending_event_r.kind = GPX_CELL_TIMEOUT then
                                shot_fault_v := '1';
                            end if;
                        end if;
                    end if;

                    rise_present_r <= rise_present_v;
                    fall_present_r <= fall_present_v;
                    shot_terminal_r <= terminal_v;
                    shot_faulted_r <= shot_fault_v;

                    expected_chip_v := rise_mask_v or fall_mask_v;
                    if expected_chip_v /= "0000" and
                       (terminal_v and expected_chip_v) = expected_chip_v then
                        expected_rise_v := fn_expected_cells(rise_mask_v);
                        expected_fall_v := fn_expected_cells(fall_mask_v);
                        if (expected_rise_v and not rise_present_v) /=
                               (expected_rise_v'range => '0') or
                           (expected_fall_v and not fall_present_v) /=
                               (expected_fall_v'range => '0') then
                            shot_fault_v := '1';
                            shot_faulted_r <= '1';
                            fault_pulse_r.missing_cell <= '1';
                            fault_sticky_r.missing_cell <= '1';
                        end if;

                        shot_complete_v := true;
                        state_r <= S_EMIT_INIT;
                    end if;

                    if not shot_complete_v then
                        state_r <= S_COLLECT;
                    end if;

                elsif state_r = S_EMIT_INIT then
                    rise_slot_count_r <= fn_popcount(shot_rise_mask_r) *
                        G_BUILD_CONFIG.stops_per_chip;
                    fall_slot_count_r <= fn_popcount(shot_fall_mask_r) *
                        G_BUILD_CONFIG.stops_per_chip;
                    rise_emit_slot_r <= 0;
                    fall_emit_slot_r <= 0;
                    rise_emit_stop_r <= 0;
                    fall_emit_stop_r <= 0;
                    rise_emit_chip_r <= fn_first_chip(shot_rise_mask_r);
                    fall_emit_chip_r <= fn_first_chip(shot_fall_mask_r);
                    if shot_rise_mask_r /= "0000" then
                        rise_emit_active_r <= '1';
                    else
                        rise_emit_active_r <= '0';
                    end if;
                    if shot_fall_mask_r /= "0000" then
                        fall_emit_active_r <= '1';
                    else
                        fall_emit_active_r <= '0';
                    end if;
                    rise_event_r.valid <= '0';
                    fall_event_r.valid <= '0';
                    state_r <= S_EMIT;
                else
                    if rise_event_r.valid = '1' and i_rise_ready = '1' and
                       rise_event_r.line_end = '1' then
                        rise_line_done_r <= '1';
                    end if;
                    if fall_event_r.valid = '1' and i_fall_ready = '1' and
                       fall_event_r.line_end = '1' then
                        fall_line_done_r <= '1';
                    end if;

                    if rise_emit_active_r = '1' and
                       (rise_event_r.valid = '0' or i_rise_ready = '1') then
                        address_value := fn_cell_address(
                            rise_emit_chip_r, rise_emit_stop_r);
                        rise_event_r <= fn_make_frame_event(
                            rise_memory_r(address_value),
                            rise_present_r(address_value),
                            rise_emit_chip_r, rise_emit_stop_r,
                            GPX_SLOPE_RISE, rise_emit_slot_r,
                            rise_slot_count_r, shot_context_r,
                            shot_max_hits_r,
                            chip_sequence_r(rise_emit_chip_r),
                            shot_gap_before_r, shot_faulted_r);

                        if rise_emit_slot_r + 1 = rise_slot_count_r then
                            rise_emit_active_r <= '0';
                        else
                            rise_emit_slot_r <= rise_emit_slot_r + 1;
                            if rise_emit_stop_r + 1 <
                               G_BUILD_CONFIG.stops_per_chip then
                                rise_emit_stop_r <= rise_emit_stop_r + 1;
                            else
                                rise_emit_stop_r <= 0;
                                rise_emit_chip_r <= fn_next_chip(
                                    shot_rise_mask_r, rise_emit_chip_r);
                            end if;
                        end if;
                    elsif rise_event_r.valid = '1' and i_rise_ready = '1' then
                        rise_event_r.valid <= '0';
                    end if;

                    if fall_emit_active_r = '1' and
                       (fall_event_r.valid = '0' or i_fall_ready = '1') then
                        address_value := fn_cell_address(
                            fall_emit_chip_r, fall_emit_stop_r);
                        fall_event_r <= fn_make_frame_event(
                            fall_memory_r(address_value),
                            fall_present_r(address_value),
                            fall_emit_chip_r, fall_emit_stop_r,
                            GPX_SLOPE_FALL, fall_emit_slot_r,
                            fall_slot_count_r, shot_context_r,
                            shot_max_hits_r,
                            chip_sequence_r(fall_emit_chip_r),
                            shot_gap_before_r, shot_faulted_r);

                        if fall_emit_slot_r + 1 = fall_slot_count_r then
                            fall_emit_active_r <= '0';
                        else
                            fall_emit_slot_r <= fall_emit_slot_r + 1;
                            if fall_emit_stop_r + 1 <
                               G_BUILD_CONFIG.stops_per_chip then
                                fall_emit_stop_r <= fall_emit_stop_r + 1;
                            else
                                fall_emit_stop_r <= 0;
                                fall_emit_chip_r <= fn_next_chip(
                                    shot_fall_mask_r, fall_emit_chip_r);
                            end if;
                        end if;
                    elsif fall_event_r.valid = '1' and i_fall_ready = '1' then
                        fall_event_r.valid <= '0';
                    end if;

                    rise_empty_after_v := rise_emit_active_r = '0' and
                        (rise_event_r.valid = '0' or
                         (rise_event_r.valid = '1' and i_rise_ready = '1'));
                    fall_empty_after_v := fall_emit_active_r = '0' and
                        (fall_event_r.valid = '0' or
                         (fall_event_r.valid = '1' and i_fall_ready = '1'));

                    if rise_empty_after_v and fall_empty_after_v then
                        state_r <= S_COLLECT;
                        shot_active_r <= '0';
                        shot_terminal_r <= (others => '0');
                        shot_done_r <= '1';
                    end if;
                end if;
            end if;
        end if;
    end process p_assemble;

    p_contract : process (i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '1' and i_abort = '0' then
                if rise_event_r.valid = '1' then
                    assert rise_event_r.cell.slope = GPX_SLOPE_RISE
                        report "V2-B8-003 non-Rise Cell on Rise output"
                        severity failure;
                    assert rise_event_r.slot_count /= 0
                        report "V2-B8-004 zero Rise slot count"
                        severity failure;
                end if;
                if fall_event_r.valid = '1' then
                    assert fall_event_r.cell.slope = GPX_SLOPE_FALL
                        report "V2-B8-005 non-Fall Cell on Fall output"
                        severity failure;
                    assert fall_event_r.slot_count /= 0
                        report "V2-B8-006 zero Fall slot count"
                        severity failure;
                end if;
            end if;
        end if;
    end process p_contract;

end architecture rtl;
