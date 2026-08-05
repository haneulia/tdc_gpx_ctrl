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

        i_face_close_event : in face_close_event_t :=
            C_FACE_CLOSE_EVENT_IDLE;
        o_face_close_ready : out std_logic;

        o_rise_event : out gpx_frame_cell_event_t;
        i_rise_ready : in  std_logic;
        o_fall_event : out gpx_frame_cell_event_t;
        i_fall_ready : in  std_logic;

        o_frame_close_event : out gpx_frame_close_event_t;
        i_frame_close_ready : in  std_logic := '1';

        o_rise_line_done : out std_logic;
        o_fall_line_done : out std_logic;
        o_shot_done      : out std_logic;
        o_shot_done_context : out shot_start_event_t;
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
        S_SHOT_GEOMETRY,
        S_SHOT_BOUNDARY,
        S_EVENT_APPLY,
        S_CELL_WRITE,
        S_EMIT_INIT,
        S_EMIT
    );

    signal state_r : assembler_state_t := S_COLLECT;

    -- One-hot state bits keep control decode shallow at 200 MHz. The added
    -- flip-flops are negligible beside the two wide Cell payload memories.
    attribute fsm_encoding : string;
    attribute fsm_encoding of state_r : signal is "one_hot";

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

    -- Geometry arithmetic and wide Cell-memory control terminate in dedicated
    -- registers. They intentionally add latency, not throughput pressure: B7
    -- produces Cells more slowly than this assembler consumes them.
    signal pending_geometry_fault_r : std_logic := '0';
    signal pending_gap_fault_r : std_logic := '0';
    signal pending_last_index_r : shot_index_t := (others => '0');
    signal pending_rise_address_r : natural range 0 to
        C_CELLS_PER_SLOPE - 1 := 0;
    signal pending_fall_address_r : natural range 0 to
        C_CELLS_PER_SLOPE - 1 := 0;
    signal pending_write_rise_r : std_logic := '0';
    signal pending_write_fall_r : std_logic := '0';

    -- Keep separate address copies so one 5-bit source does not drive both
    -- 147-bit distributed memories after placement.
    attribute keep : string;
    attribute keep of pending_rise_address_r : signal is "true";
    attribute keep of pending_fall_address_r : signal is "true";

    signal history_valid_r   : std_logic := '0';
    signal history_face_r    : face_index_t := (others => '0');
    signal history_direction_r : direction_t := DIRECTION_CW;
    signal history_source_r  : std_logic := '0';
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

    -- The packed 147-bit LUTRAM expands to many physical RAM32 primitives.
    -- Bound address fanout so synthesis may place local register replicas
    -- beside those RAM groups instead of routing one cursor across all of them.
    attribute max_fanout : integer;
    attribute max_fanout of rise_emit_chip_r : signal is 16;
    attribute max_fanout of fall_emit_chip_r : signal is 16;
    attribute max_fanout of rise_emit_stop_r : signal is 16;
    attribute max_fanout of fall_emit_stop_r : signal is 16;
    attribute max_fanout of pending_rise_address_r : signal is 16;
    attribute max_fanout of pending_fall_address_r : signal is 16;

    -- Each lane registers the read request before reading distributed RAM into
    -- a packed local register and constructing the typed output event. The
    -- request stage isolates the Chip/STOP cursor and presence lookup from the
    -- wide blank-fill register while preserving one Cell/clock after warm-up.
    signal rise_read_valid_r : std_logic := '0';
    signal fall_read_valid_r : std_logic := '0';
    signal rise_read_address_r : natural range 0 to
        C_CELLS_PER_SLOPE - 1 := 0;
    signal fall_read_address_r : natural range 0 to
        C_CELLS_PER_SLOPE - 1 := 0;
    signal rise_read_present_r : std_logic := '0';
    signal fall_read_present_r : std_logic := '0';
    signal rise_read_chip_r : natural range 0 to C_MAX_CHIPS - 1 := 0;
    signal fall_read_chip_r : natural range 0 to C_MAX_CHIPS - 1 := 0;
    signal rise_read_stop_r : natural range 0 to
        C_MAX_STOPS_PER_CHIP - 1 := 0;
    signal fall_read_stop_r : natural range 0 to
        C_MAX_STOPS_PER_CHIP - 1 := 0;
    signal rise_read_slot_r : natural range 0 to
        C_CELLS_PER_SLOPE - 1 := 0;
    signal fall_read_slot_r : natural range 0 to
        C_CELLS_PER_SLOPE - 1 := 0;

    attribute max_fanout of rise_read_address_r : signal is 16;
    attribute max_fanout of fall_read_address_r : signal is 16;
    attribute max_fanout of rise_read_present_r : signal is 16;
    attribute max_fanout of fall_read_present_r : signal is 16;

    signal rise_prefetch_valid_r : std_logic := '0';
    signal fall_prefetch_valid_r : std_logic := '0';
    signal rise_prefetch_word_r : cell_word_t := (others => '0');
    signal fall_prefetch_word_r : cell_word_t := (others => '0');
    signal rise_prefetch_present_r : std_logic := '0';
    signal fall_prefetch_present_r : std_logic := '0';
    signal rise_prefetch_chip_r : natural range 0 to C_MAX_CHIPS - 1 := 0;
    signal fall_prefetch_chip_r : natural range 0 to C_MAX_CHIPS - 1 := 0;
    signal rise_prefetch_stop_r : natural range 0 to
        C_MAX_STOPS_PER_CHIP - 1 := 0;
    signal fall_prefetch_stop_r : natural range 0 to
        C_MAX_STOPS_PER_CHIP - 1 := 0;
    signal rise_prefetch_slot_r : natural range 0 to
        C_CELLS_PER_SLOPE - 1 := 0;
    signal fall_prefetch_slot_r : natural range 0 to
        C_CELLS_PER_SLOPE - 1 := 0;

    signal rise_event_r : gpx_frame_cell_event_t :=
        C_GPX_FRAME_CELL_EVENT_IDLE;
    signal fall_event_r : gpx_frame_cell_event_t :=
        C_GPX_FRAME_CELL_EVENT_IDLE;
    signal rise_line_done_r : std_logic := '0';
    signal fall_line_done_r : std_logic := '0';
    signal shot_done_r      : std_logic := '0';
    signal shot_done_context_r : shot_start_event_t :=
        C_SHOT_START_EVENT_IDLE;
    signal frame_close_event_r : gpx_frame_close_event_t :=
        C_GPX_FRAME_CLOSE_EVENT_IDLE;
    signal face_close_ready_c : std_logic;

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

        -- The prefetch stage has already replaced an absent LUTRAM entry with
        -- an all-zero word. Unpack every payload bit unconditionally here so
        -- present/state control cannot become a high-fanout synchronous reset
        -- on the wide output register.
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

        if present_value /= '1' then
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
                            state_r = S_COLLECT and
                            i_face_close_event.valid = '0' and
                            frame_close_event_r.valid = '0' else
                    '0';
    face_close_ready_c <= '1' when i_rst_n = '1' and i_abort = '0' and
                                  state_r = S_COLLECT and
                                  shot_active_r = '0' and
                                  frame_close_event_r.valid = '0' else
                          '0';
    o_face_close_ready <= face_close_ready_c;
    o_rise_event <= rise_event_r;
    o_fall_event <= fall_event_r;
    o_frame_close_event <= frame_close_event_r;
    o_rise_line_done <= rise_line_done_r;
    o_fall_line_done <= fall_line_done_r;
    o_shot_done      <= shot_done_r;
    o_shot_done_context <= shot_done_context_r;
    o_idle <= '1' when state_r = S_COLLECT and shot_active_r = '0' and
                         frame_close_event_r.valid = '0' else '0';
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
        variable cell_write_v : boolean;
        variable close_v : gpx_frame_close_event_t;
        variable trailing_gap_v : shot_index_t;
        variable close_fault_v : boolean;
        variable rise_output_ready_v : boolean;
        variable fall_output_ready_v : boolean;
        variable rise_prefetch_ready_v : boolean;
        variable fall_prefetch_ready_v : boolean;
        variable rise_read_ready_v : boolean;
        variable fall_read_ready_v : boolean;
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
                pending_geometry_fault_r <= '0';
                pending_gap_fault_r <= '0';
                pending_last_index_r <= (others => '0');
                pending_rise_address_r <= 0;
                pending_fall_address_r <= 0;
                pending_write_rise_r <= '0';
                pending_write_fall_r <= '0';
                history_valid_r <= '0';
                history_face_r <= (others => '0');
                history_direction_r <= DIRECTION_CW;
                history_source_r <= '0';
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
                rise_read_valid_r <= '0';
                fall_read_valid_r <= '0';
                rise_prefetch_valid_r <= '0';
                fall_prefetch_valid_r <= '0';
                rise_event_r <= C_GPX_FRAME_CELL_EVENT_IDLE;
                fall_event_r <= C_GPX_FRAME_CELL_EVENT_IDLE;
                rise_line_done_r <= '0';
                fall_line_done_r <= '0';
                shot_done_r <= '0';
                shot_done_context_r <= C_SHOT_START_EVENT_IDLE;
                frame_close_event_r <= C_GPX_FRAME_CLOSE_EVENT_IDLE;
                fault_pulse_r <= C_GPX_FRAME_ASSEMBLER_FAULTS_CLEAR;
                fault_sticky_r <= C_GPX_FRAME_ASSEMBLER_FAULTS_CLEAR;
            else
                rise_line_done_r <= '0';
                fall_line_done_r <= '0';
                shot_done_r <= '0';
                shot_done_context_r.valid <= '0';
                fault_pulse_r <= C_GPX_FRAME_ASSEMBLER_FAULTS_CLEAR;

                if frame_close_event_r.valid = '1' and
                   i_frame_close_ready = '1' then
                    frame_close_event_r.valid <= '0';
                end if;

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
                    pending_geometry_fault_r <= '0';
                    pending_gap_fault_r <= '0';
                    pending_last_index_r <= (others => '0');
                    pending_write_rise_r <= '0';
                    pending_write_fall_r <= '0';
                    history_valid_r <= '0';
                    rise_emit_active_r <= '0';
                    fall_emit_active_r <= '0';
                    rise_read_valid_r <= '0';
                    fall_read_valid_r <= '0';
                    rise_prefetch_valid_r <= '0';
                    fall_prefetch_valid_r <= '0';
                    rise_event_r <= C_GPX_FRAME_CELL_EVENT_IDLE;
                    fall_event_r <= C_GPX_FRAME_CELL_EVENT_IDLE;
                    frame_close_event_r <= C_GPX_FRAME_CLOSE_EVENT_IDLE;
                elsif state_r = S_COLLECT then
                    if i_face_close_event.valid = '1' and
                       face_close_ready_c = '1' then
                        close_v := C_GPX_FRAME_CLOSE_EVENT_IDLE;
                        close_v.valid := '1';
                        close_v.face_index := i_face_close_event.face_index;
                        close_v.direction := i_face_close_event.direction;
                        close_v.source_sim := i_face_close_event.source_sim;
                        close_v.active_version :=
                            i_face_close_event.active_version;
                        close_v.columns_per_face :=
                            i_face_close_event.columns_per_face;
                        trailing_gap_v := (others => '0');
                        close_fault_v := false;

                        if i_face_close_event.active_version /=
                               i_active_version or
                           i_face_close_event.columns_per_face /=
                               i_columns_per_face or
                           to_integer(i_face_close_event.face_index) >=
                               G_BUILD_CONFIG.num_faces or
                           i_face_close_event.columns_per_face = 0 then
                            close_fault_v := true;
                        end if;

                        if history_valid_r = '0' then
                            close_v.all_hole := '1';
                            trailing_gap_v :=
                                i_face_close_event.columns_per_face;
                        elsif history_face_r =
                                  i_face_close_event.face_index and
                              history_direction_r =
                                  i_face_close_event.direction and
                              history_source_r =
                                  i_face_close_event.source_sim and
                              history_version_r =
                                  i_face_close_event.active_version then
                            if history_last_r = '1' then
                                trailing_gap_v := (others => '0');
                            elsif history_column_r <
                                  i_face_close_event.columns_per_face then
                                trailing_gap_v :=
                                    i_face_close_event.columns_per_face -
                                    history_column_r - 1;
                            else
                                close_fault_v := true;
                            end if;
                        else
                            close_fault_v := true;
                        end if;

                        close_v.trailing_gap := trailing_gap_v;
                        if close_fault_v then
                            close_v.face_faulted := '1';
                            fault_pulse_r.geometry_error <= '1';
                            fault_sticky_r.geometry_error <= '1';
                        end if;
                        if trailing_gap_v /= 0 then
                            fault_pulse_r.column_gap <= '1';
                            fault_sticky_r.column_gap <= '1';
                        end if;

                        frame_close_event_r <= close_v;
                        history_valid_r <= '0';
                        history_last_r <= '0';
                    elsif i_cell_event.valid = '1' then
                        pending_event_r <= i_cell_event;
                        state_r <= S_EVENT_CHECK;
                    end if;

                elsif state_r = S_EVENT_CHECK then
                    pending_rise_address_r <= fn_cell_address(
                        to_integer(pending_event_r.chip_index),
                        to_integer(pending_event_r.stop_index));
                    pending_fall_address_r <= fn_cell_address(
                        to_integer(pending_event_r.chip_index),
                        to_integer(pending_event_r.stop_index));
                    pending_geometry_fault_r <= '0';
                    pending_gap_fault_r <= '0';

                    if shot_active_r = '0' then
                        rise_mask_v := i_active_rise_mask;
                        fall_mask_v := i_active_fall_mask;
                        context_v := pending_event_r.shot_context;
                        columns_v := i_columns_per_face;

                        rise_present_r <= (others => '0');
                        fall_present_r <= (others => '0');
                        shot_terminal_r <= (others => '0');
                        shot_active_r <= '1';
                        shot_context_r <= context_v;
                        shot_max_hits_r <= pending_event_r.max_hits;
                        shot_rise_mask_r <= rise_mask_v;
                        shot_fall_mask_r <= fall_mask_v;
                        shot_columns_r <= columns_v;
                        shot_gap_before_r <= (others => '0');
                        shot_faulted_r <= '0';
                        pending_context_match_r <= '1';
                        state_r <= S_SHOT_GEOMETRY;
                    elsif pending_event_r.shot_context = shot_context_r and
                          i_active_version =
                              shot_context_r.request.active_version and
                          i_active_rise_mask = shot_rise_mask_r and
                          i_active_fall_mask = shot_fall_mask_r and
                          i_columns_per_face = shot_columns_r then
                        pending_context_match_r <= '1';
                        state_r <= S_EVENT_APPLY;
                    else
                        pending_context_match_r <= '0';
                        state_r <= S_EVENT_APPLY;
                    end if;

                elsif state_r = S_SHOT_GEOMETRY then
                    context_v := shot_context_r;
                    columns_v := shot_columns_r;
                    gap_v := (others => '0');
                    geometry_fault_v := false;
                    expected_chip_v := shot_rise_mask_r or shot_fall_mask_r;

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
                            history_face_r /= context_v.request.face_index) then
                            geometry_fault_v := true;
                        end if;
                    elsif context_v.request.shot_index > history_column_r then
                        gap_v := context_v.request.shot_index -
                            history_column_r - 1;
                    else
                        gap_v := context_v.request.shot_index;
                        geometry_fault_v := true;
                    end if;

                    if columns_v = 0 then
                        pending_last_index_r <= (others => '0');
                    else
                        pending_last_index_r <= columns_v - 1;
                    end if;

                    shot_gap_before_r <= gap_v;
                    if geometry_fault_v then
                        pending_geometry_fault_r <= '1';
                    else
                        pending_geometry_fault_r <= '0';
                    end if;

                    history_valid_r <= '1';
                    history_face_r <= context_v.request.face_index;
                    history_direction_r <= context_v.request.direction;
                    history_source_r <= context_v.request.source_sim;
                    history_version_r <= context_v.request.active_version;
                    history_column_r <= context_v.request.shot_index;
                    history_last_r <= context_v.request.last_in_face;

                    state_r <= S_SHOT_BOUNDARY;

                elsif state_r = S_SHOT_BOUNDARY then
                    geometry_fault_v := pending_geometry_fault_r = '1';

                    if shot_columns_r /= 0 then
                        if shot_context_r.request.shot_index =
                           pending_last_index_r then
                            if shot_context_r.request.last_in_face /= '1' then
                                geometry_fault_v := true;
                            end if;
                        elsif shot_context_r.request.last_in_face /= '0' then
                            geometry_fault_v := true;
                        end if;
                    end if;

                    if geometry_fault_v then
                        pending_geometry_fault_r <= '1';
                    else
                        pending_geometry_fault_r <= '0';
                    end if;
                    if shot_gap_before_r /= 0 then
                        pending_gap_fault_r <= '1';
                    else
                        pending_gap_fault_r <= '0';
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
                    cell_write_v := false;
                    pending_write_rise_r <= '0';
                    pending_write_fall_r <= '0';

                    if pending_gap_fault_r = '1' then
                        fault_pulse_r.column_gap <= '1';
                        fault_sticky_r.column_gap <= '1';
                    end if;
                    pending_gap_fault_r <= '0';

                    if pending_geometry_fault_r = '1' then
                        shot_fault_v := '1';
                        fault_pulse_r.geometry_error <= '1';
                        fault_sticky_r.geometry_error <= '1';
                    end if;
                    pending_geometry_fault_r <= '0';

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
                            if pending_event_r.slope = GPX_SLOPE_RISE and
                               rise_mask_v(chip_index) = '1' then
                                address_value := pending_rise_address_r;
                                if rise_present_v(address_value) = '1' then
                                    shot_fault_v := '1';
                                    fault_pulse_r.duplicate_cell <= '1';
                                    fault_sticky_r.duplicate_cell <= '1';
                                else
                                    pending_write_rise_r <= '1';
                                    cell_write_v := true;
                                end if;
                            elsif pending_event_r.slope = GPX_SLOPE_FALL and
                                  fall_mask_v(chip_index) = '1' then
                                address_value := pending_fall_address_r;
                                if fall_present_v(address_value) = '1' then
                                    shot_fault_v := '1';
                                    fault_pulse_r.duplicate_cell <= '1';
                                    fault_sticky_r.duplicate_cell <= '1';
                                else
                                    pending_write_fall_r <= '1';
                                    cell_write_v := true;
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

                    if not shot_complete_v and cell_write_v then
                        state_r <= S_CELL_WRITE;
                    elsif not shot_complete_v then
                        state_r <= S_COLLECT;
                    end if;

                elsif state_r = S_CELL_WRITE then
                    rise_present_v := rise_present_r;
                    fall_present_v := fall_present_r;

                    if pending_write_rise_r = '1' then
                        rise_memory_r(pending_rise_address_r) <=
                            fn_pack_cell(pending_event_r);
                        rise_present_v(pending_rise_address_r) := '1';
                    end if;
                    if pending_write_fall_r = '1' then
                        fall_memory_r(pending_fall_address_r) <=
                            fn_pack_cell(pending_event_r);
                        fall_present_v(pending_fall_address_r) := '1';
                    end if;

                    rise_present_r <= rise_present_v;
                    fall_present_r <= fall_present_v;
                    pending_write_rise_r <= '0';
                    pending_write_fall_r <= '0';
                    state_r <= S_COLLECT;

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
                    rise_read_valid_r <= '0';
                    fall_read_valid_r <= '0';
                    rise_prefetch_valid_r <= '0';
                    fall_prefetch_valid_r <= '0';
                    state_r <= S_EMIT;
                else
                    rise_output_ready_v := rise_event_r.valid = '0' or
                        i_rise_ready = '1';
                    fall_output_ready_v := fall_event_r.valid = '0' or
                        i_fall_ready = '1';
                    rise_prefetch_ready_v := rise_prefetch_valid_r = '0' or
                        rise_output_ready_v;
                    fall_prefetch_ready_v := fall_prefetch_valid_r = '0' or
                        fall_output_ready_v;
                    rise_read_ready_v := rise_read_valid_r = '0' or
                        rise_prefetch_ready_v;
                    fall_read_ready_v := fall_read_valid_r = '0' or
                        fall_prefetch_ready_v;

                    if rise_event_r.valid = '1' and i_rise_ready = '1' and
                       rise_event_r.line_end = '1' then
                        rise_line_done_r <= '1';
                    end if;
                    if fall_event_r.valid = '1' and i_fall_ready = '1' and
                       fall_event_r.line_end = '1' then
                        fall_line_done_r <= '1';
                    end if;

                    if rise_output_ready_v then
                        if rise_prefetch_valid_r = '1' then
                            rise_event_r <= fn_make_frame_event(
                                rise_prefetch_word_r,
                                rise_prefetch_present_r,
                                rise_prefetch_chip_r,
                                rise_prefetch_stop_r,
                                GPX_SLOPE_RISE,
                                rise_prefetch_slot_r,
                                rise_slot_count_r,
                                shot_context_r,
                                shot_max_hits_r,
                                chip_sequence_r(rise_prefetch_chip_r),
                                shot_gap_before_r,
                                shot_faulted_r);
                        else
                            rise_event_r.valid <= '0';
                        end if;
                    end if;

                    if rise_prefetch_ready_v then
                        if rise_read_valid_r = '1' then
                            rise_prefetch_present_r <= rise_read_present_r;
                            if rise_read_present_r = '1' then
                                rise_prefetch_word_r <=
                                    rise_memory_r(rise_read_address_r);
                            else
                                rise_prefetch_word_r <= (others => '0');
                            end if;
                            rise_prefetch_chip_r <= rise_read_chip_r;
                            rise_prefetch_stop_r <= rise_read_stop_r;
                            rise_prefetch_slot_r <= rise_read_slot_r;
                            rise_prefetch_valid_r <= '1';
                        else
                            rise_prefetch_valid_r <= '0';
                        end if;
                    end if;

                    if rise_read_ready_v then
                        if rise_emit_active_r = '1' then
                            address_value := fn_cell_address(
                                rise_emit_chip_r, rise_emit_stop_r);
                            rise_read_address_r <= address_value;
                            rise_read_present_r <=
                                rise_present_r(address_value);
                            rise_read_chip_r <= rise_emit_chip_r;
                            rise_read_stop_r <= rise_emit_stop_r;
                            rise_read_slot_r <= rise_emit_slot_r;
                            rise_read_valid_r <= '1';

                            if rise_emit_slot_r + 1 = rise_slot_count_r then
                                rise_emit_active_r <= '0';
                            else
                                rise_emit_slot_r <= rise_emit_slot_r + 1;
                            end if;

                            if rise_emit_stop_r + 1 <
                               G_BUILD_CONFIG.stops_per_chip then
                                rise_emit_stop_r <= rise_emit_stop_r + 1;
                            else
                                rise_emit_stop_r <= 0;
                                rise_emit_chip_r <= fn_next_chip(
                                    shot_rise_mask_r, rise_emit_chip_r);
                            end if;
                        else
                            rise_read_valid_r <= '0';
                        end if;
                    end if;

                    if fall_output_ready_v then
                        if fall_prefetch_valid_r = '1' then
                            fall_event_r <= fn_make_frame_event(
                                fall_prefetch_word_r,
                                fall_prefetch_present_r,
                                fall_prefetch_chip_r,
                                fall_prefetch_stop_r,
                                GPX_SLOPE_FALL,
                                fall_prefetch_slot_r,
                                fall_slot_count_r,
                                shot_context_r,
                                shot_max_hits_r,
                                chip_sequence_r(fall_prefetch_chip_r),
                                shot_gap_before_r,
                                shot_faulted_r);
                        else
                            fall_event_r.valid <= '0';
                        end if;
                    end if;

                    if fall_prefetch_ready_v then
                        if fall_read_valid_r = '1' then
                            fall_prefetch_present_r <= fall_read_present_r;
                            if fall_read_present_r = '1' then
                                fall_prefetch_word_r <=
                                    fall_memory_r(fall_read_address_r);
                            else
                                fall_prefetch_word_r <= (others => '0');
                            end if;
                            fall_prefetch_chip_r <= fall_read_chip_r;
                            fall_prefetch_stop_r <= fall_read_stop_r;
                            fall_prefetch_slot_r <= fall_read_slot_r;
                            fall_prefetch_valid_r <= '1';
                        else
                            fall_prefetch_valid_r <= '0';
                        end if;
                    end if;

                    if fall_read_ready_v then
                        if fall_emit_active_r = '1' then
                            address_value := fn_cell_address(
                                fall_emit_chip_r, fall_emit_stop_r);
                            fall_read_address_r <= address_value;
                            fall_read_present_r <=
                                fall_present_r(address_value);
                            fall_read_chip_r <= fall_emit_chip_r;
                            fall_read_stop_r <= fall_emit_stop_r;
                            fall_read_slot_r <= fall_emit_slot_r;
                            fall_read_valid_r <= '1';

                            if fall_emit_slot_r + 1 = fall_slot_count_r then
                                fall_emit_active_r <= '0';
                            else
                                fall_emit_slot_r <= fall_emit_slot_r + 1;
                            end if;

                            if fall_emit_stop_r + 1 <
                               G_BUILD_CONFIG.stops_per_chip then
                                fall_emit_stop_r <= fall_emit_stop_r + 1;
                            else
                                fall_emit_stop_r <= 0;
                                fall_emit_chip_r <= fn_next_chip(
                                    shot_fall_mask_r, fall_emit_chip_r);
                            end if;
                        else
                            fall_read_valid_r <= '0';
                        end if;
                    end if;

                    if rise_emit_active_r = '0' and
                       rise_read_valid_r = '0' and
                       rise_prefetch_valid_r = '0' and
                       rise_event_r.valid = '0' and
                       fall_emit_active_r = '0' and
                       fall_read_valid_r = '0' and
                       fall_prefetch_valid_r = '0' and
                       fall_event_r.valid = '0' then
                        state_r <= S_COLLECT;
                        shot_active_r <= '0';
                        shot_terminal_r <= (others => '0');
                        shot_done_r <= '1';
                        shot_done_context_r <= shot_context_r;
                        shot_done_context_r.valid <= '1';
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
                if frame_close_event_r.valid = '1' then
                    assert frame_close_event_r.columns_per_face /= 0
                        report "V2-B8-007 zero-column Face close"
                        severity failure;
                    assert frame_close_event_r.trailing_gap <=
                           frame_close_event_r.columns_per_face
                        report "V2-B8-008 trailing gap exceeds Face geometry"
                        severity failure;
                end if;
            end if;
        end if;
    end process p_contract;

end architecture rtl;
