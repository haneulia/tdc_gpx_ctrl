library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_gpx_vdma_pkg.all;

entity lidar_gpx_vdma_lane_formatter is
    generic (
        g_OUTPUT_WIDTH : positive := 32
    );
    port (
        i_clk        : in  std_logic;
        i_rst_n      : in  std_logic;
        i_abort      : in  std_logic;

        i_word_event : in  gpx_vdma_word_event_t;
        o_word_ready : out std_logic;

        -- The formatter snapshots these blocks on the first word of a line.
        -- Only first_column emits their contents; later prefixes are zero.
        i_prefix_blocks : in gpx_vdma_prefix_blocks_t;

        o_m_axis_tdata  : out std_logic_vector(g_OUTPUT_WIDTH - 1 downto 0);
        o_m_axis_tkeep  : out std_logic_vector(g_OUTPUT_WIDTH / 8 - 1 downto 0);
        o_m_axis_tstrb  : out std_logic_vector(g_OUTPUT_WIDTH / 8 - 1 downto 0);
        o_m_axis_tvalid : out std_logic;
        o_m_axis_tlast  : out std_logic;
        o_m_axis_tuser  : out std_logic_vector(0 downto 0);
        i_m_axis_tready : in  std_logic;

        o_hsize_bytes       : out unsigned(15 downto 0);
        o_line_done         : out std_logic;
        o_line_done_faulted : out std_logic;
        o_idle              : out std_logic
    );
end entity lidar_gpx_vdma_lane_formatter;

architecture rtl of lidar_gpx_vdma_lane_formatter is

    constant C_SUBBEATS_PER_BLOCK : positive :=
        C_GPX_VDMA_BLOCK_WIDTH / g_OUTPUT_WIDTH;
    constant C_TOTAL_BLOCKS_MAX : positive :=
        C_GPX_VDMA_PREFIX_BLOCKS + C_GPX_VDMA_MAX_PAYLOAD_BLOCKS;

    type state_t is (
        ST_IDLE,
        ST_CAPTURE,
        ST_LOAD_FIRST,
        ST_SEND,
        ST_DRAIN_LAST
    );
    type payload_memory_t is array (
        0 to C_GPX_VDMA_MAX_PAYLOAD_BLOCKS - 1
    ) of gpx_vdma_block_t;

    signal state_r : state_t := ST_IDLE;
    signal payload_memory_r : payload_memory_t;
    signal prefix_blocks_r : gpx_vdma_prefix_blocks_t :=
        C_GPX_VDMA_PREFIX_BLOCKS_ZERO;
    signal block_accumulator_r : gpx_vdma_block_t := (others => '0');
    signal block_data_r : gpx_vdma_block_t := (others => '0');

    signal word_in_block_r : natural range 0 to
        C_GPX_VDMA_WORDS_PER_BLOCK - 1 := 0;
    signal write_block_r : natural range 0 to
        C_GPX_VDMA_MAX_PAYLOAD_BLOCKS - 1 := 0;
    signal captured_words_r : natural range 0 to
        C_GPX_VDMA_MAX_PAYLOAD_WORDS := 0;
    signal expected_words_r : natural range 1 to
        C_GPX_VDMA_MAX_PAYLOAD_WORDS := 1;
    signal word_count_r : natural range 2 to
        C_GPX_VDMA_MAX_CELL_WORDS := 2;
    signal slot_count_r : natural range 1 to
        C_GPX_VDMA_MAX_LINE_SLOTS := 1;

    signal total_block_count_r : natural range
        C_GPX_VDMA_PREFIX_BLOCKS + 1 to C_TOTAL_BLOCKS_MAX :=
        C_GPX_VDMA_PREFIX_BLOCKS + 1;
    signal read_block_r : natural range 0 to C_TOTAL_BLOCKS_MAX - 1 := 0;
    signal subbeat_r : natural range 0 to C_SUBBEATS_PER_BLOCK - 1 := 0;

    signal first_column_r : std_logic := '0';
    signal line_faulted_r : std_logic := '0';
    signal hsize_bytes_r : unsigned(15 downto 0) := (others => '0');

    signal axis_tdata_r : std_logic_vector(g_OUTPUT_WIDTH - 1 downto 0) :=
        (others => '0');
    signal axis_tvalid_r : std_logic := '0';
    signal axis_tlast_r : std_logic := '0';
    signal axis_tuser_r : std_logic := '0';
    signal line_done_r : std_logic := '0';
    signal line_done_faulted_r : std_logic := '0';
    signal output_ready_c : std_logic;

    attribute ram_style : string;
    attribute ram_style of payload_memory_r : signal is "distributed";
    attribute max_fanout : integer;
    attribute max_fanout of read_block_r : signal is 16;

    function fn_block_slice(
        value     : gpx_vdma_block_t;
        sub_index : natural
    ) return std_logic_vector is
        variable result : std_logic_vector(g_OUTPUT_WIDTH - 1 downto 0);
        variable low_bit : natural;
    begin
        low_bit := sub_index * g_OUTPUT_WIDTH;
        result := value(low_bit + g_OUTPUT_WIDTH - 1 downto low_bit);
        return result;
    end function fn_block_slice;

    function fn_payload_blocks_from_words(
        word_count : positive
    ) return positive is
    begin
        return (word_count + C_GPX_VDMA_WORDS_PER_BLOCK - 1) /
            C_GPX_VDMA_WORDS_PER_BLOCK;
    end function fn_payload_blocks_from_words;

begin

    assert g_OUTPUT_WIDTH = 32 or g_OUTPUT_WIDTH = 64 or
           g_OUTPUT_WIDTH = 128
        report "V2-B9-J2-001 output width must be 32, 64, or 128"
        severity failure;

    o_word_ready <= '1' when i_abort = '0' and
        (state_r = ST_IDLE or state_r = ST_CAPTURE) else '0';
    output_ready_c <= '1' when axis_tvalid_r = '0' or
        i_m_axis_tready = '1' else '0';

    o_m_axis_tdata <= axis_tdata_r;
    o_m_axis_tkeep <= (others => '1');
    o_m_axis_tstrb <= (others => '1');
    o_m_axis_tvalid <= axis_tvalid_r;
    o_m_axis_tlast <= axis_tlast_r;
    o_m_axis_tuser(0) <= axis_tuser_r;
    o_hsize_bytes <= hsize_bytes_r;
    o_line_done <= line_done_r;
    o_line_done_faulted <= line_done_faulted_r;
    o_idle <= '1' when state_r = ST_IDLE and
        axis_tvalid_r = '0' else '0';

    p_formatter : process (i_clk)
        variable next_block : gpx_vdma_block_t;
        variable total_words : natural;
        variable payload_blocks : positive;
        variable next_read_block : natural;
        variable is_last_subbeat : boolean;
        variable is_last_block : boolean;
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                state_r <= ST_IDLE;
                prefix_blocks_r <= C_GPX_VDMA_PREFIX_BLOCKS_ZERO;
                block_accumulator_r <= (others => '0');
                block_data_r <= (others => '0');
                word_in_block_r <= 0;
                write_block_r <= 0;
                captured_words_r <= 0;
                expected_words_r <= 1;
                word_count_r <= 2;
                slot_count_r <= 1;
                total_block_count_r <= C_GPX_VDMA_PREFIX_BLOCKS + 1;
                read_block_r <= 0;
                subbeat_r <= 0;
                first_column_r <= '0';
                line_faulted_r <= '0';
                hsize_bytes_r <= (others => '0');
                axis_tdata_r <= (others => '0');
                axis_tvalid_r <= '0';
                axis_tlast_r <= '0';
                axis_tuser_r <= '0';
                line_done_r <= '0';
                line_done_faulted_r <= '0';
            else
                line_done_r <= '0';
                line_done_faulted_r <= '0';

                if axis_tvalid_r = '1' and i_m_axis_tready = '1' then
                    axis_tvalid_r <= '0';
                    axis_tlast_r <= '0';
                    axis_tuser_r <= '0';
                end if;

                -- A capture has not reached AXIS yet and can be discarded
                -- locally. Once output starts, abort only marks the line;
                -- the registered AXIS line still drains through EOL.
                if i_abort = '1' and
                   (state_r = ST_IDLE or state_r = ST_CAPTURE) then
                    state_r <= ST_IDLE;
                    block_accumulator_r <= (others => '0');
                    word_in_block_r <= 0;
                    write_block_r <= 0;
                    captured_words_r <= 0;
                    line_faulted_r <= '1';
                else
                    if i_abort = '1' then
                        line_faulted_r <= '1';
                    end if;

                    case state_r is
                        when ST_IDLE =>
                            if i_word_event.valid = '1' then
                                total_words := to_integer(
                                    i_word_event.slot_count) *
                                    to_integer(i_word_event.word_count);
                                payload_blocks :=
                                    fn_payload_blocks_from_words(total_words);

                                next_block := (others => '0');
                                next_block(31 downto 0) := i_word_event.data;
                                block_accumulator_r <= next_block;
                                word_in_block_r <= 1;
                                write_block_r <= 0;
                                captured_words_r <= 1;
                                expected_words_r <= total_words;
                                word_count_r <= to_integer(
                                    i_word_event.word_count);
                                slot_count_r <= to_integer(
                                    i_word_event.slot_count);
                                total_block_count_r <=
                                    C_GPX_VDMA_PREFIX_BLOCKS + payload_blocks;
                                hsize_bytes_r <= to_unsigned(
                                    C_GPX_VDMA_PREFIX_BYTES +
                                    payload_blocks * C_GPX_VDMA_BLOCK_BYTES,
                                    hsize_bytes_r'length);
                                first_column_r <=
                                    i_word_event.first_column;
                                line_faulted_r <=
                                    i_word_event.line_faulted;
                                if i_word_event.first_column = '1' then
                                    prefix_blocks_r <= i_prefix_blocks;
                                else
                                    prefix_blocks_r <=
                                        C_GPX_VDMA_PREFIX_BLOCKS_ZERO;
                                end if;
                                state_r <= ST_CAPTURE;
                            end if;

                        when ST_CAPTURE =>
                            if i_word_event.valid = '1' then
                                next_block := block_accumulator_r;
                                next_block(
                                    word_in_block_r * 32 + 31 downto
                                    word_in_block_r * 32) :=
                                    i_word_event.data;
                                total_words := captured_words_r + 1;
                                captured_words_r <= total_words;
                                line_faulted_r <= line_faulted_r or
                                    i_word_event.line_faulted;

                                if word_in_block_r =
                                   C_GPX_VDMA_WORDS_PER_BLOCK - 1 or
                                   i_word_event.line_end = '1' then
                                    payload_memory_r(write_block_r) <=
                                        next_block;
                                    block_accumulator_r <= (others => '0');
                                    word_in_block_r <= 0;

                                    if i_word_event.line_end = '1' then
                                        read_block_r <= 0;
                                        subbeat_r <= 0;
                                        state_r <= ST_LOAD_FIRST;
                                    elsif write_block_r <
                                          C_GPX_VDMA_MAX_PAYLOAD_BLOCKS - 1 then
                                        write_block_r <= write_block_r + 1;
                                    end if;
                                else
                                    block_accumulator_r <= next_block;
                                    word_in_block_r <= word_in_block_r + 1;
                                end if;
                            end if;

                        when ST_LOAD_FIRST =>
                            block_data_r <= prefix_blocks_r(0);
                            read_block_r <= 0;
                            subbeat_r <= 0;
                            state_r <= ST_SEND;

                        when ST_SEND =>
                            if output_ready_c = '1' then
                                is_last_subbeat :=
                                    subbeat_r = C_SUBBEATS_PER_BLOCK - 1;
                                is_last_block :=
                                    read_block_r + 1 = total_block_count_r;

                                axis_tdata_r <= fn_block_slice(
                                    block_data_r, subbeat_r);
                                axis_tvalid_r <= '1';
                                if read_block_r = 0 and subbeat_r = 0 and
                                   first_column_r = '1' then
                                    axis_tuser_r <= '1';
                                else
                                    axis_tuser_r <= '0';
                                end if;

                                if is_last_subbeat and is_last_block then
                                    axis_tlast_r <= '1';
                                    state_r <= ST_DRAIN_LAST;
                                else
                                    axis_tlast_r <= '0';
                                    if is_last_subbeat then
                                        next_read_block := read_block_r + 1;
                                        read_block_r <= next_read_block;
                                        subbeat_r <= 0;
                                        if next_read_block <
                                           C_GPX_VDMA_PREFIX_BLOCKS then
                                            block_data_r <= prefix_blocks_r(
                                                next_read_block);
                                        else
                                            block_data_r <= payload_memory_r(
                                                next_read_block -
                                                C_GPX_VDMA_PREFIX_BLOCKS);
                                        end if;
                                    else
                                        subbeat_r <= subbeat_r + 1;
                                    end if;
                                end if;
                            end if;

                        when ST_DRAIN_LAST =>
                            if axis_tvalid_r = '1' and
                               i_m_axis_tready = '1' then
                                line_done_r <= '1';
                                line_done_faulted_r <= line_faulted_r or
                                    i_abort;
                                state_r <= ST_IDLE;
                                block_accumulator_r <= (others => '0');
                                word_in_block_r <= 0;
                                write_block_r <= 0;
                                captured_words_r <= 0;
                            end if;
                    end case;
                end if;
            end if;
        end if;
    end process p_formatter;

    p_contract : process (i_clk)
        variable accepted_words : natural;
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '1' and i_abort = '0' and
               i_word_event.valid = '1' and o_word_ready = '1' then
                assert to_integer(i_word_event.word_count) >= 2 and
                       to_integer(i_word_event.word_count) <=
                           C_GPX_VDMA_MAX_CELL_WORDS
                    report "V2-B9-J2-002 illegal Cell word count"
                    severity failure;
                assert to_integer(i_word_event.slot_count) >= 1 and
                       to_integer(i_word_event.slot_count) <=
                           C_GPX_VDMA_MAX_LINE_SLOTS
                    report "V2-B9-J2-003 illegal line slot count"
                    severity failure;

                if state_r = ST_IDLE then
                    assert i_word_event.line_start = '1' and
                           i_word_event.cell_start = '1' and
                           to_integer(i_word_event.slot_index) = 0 and
                           to_integer(i_word_event.word_index) = 0
                        report "V2-B9-J2-004 line did not start at Cell/word zero"
                        severity failure;
                    assert i_word_event.gap_before = 0
                        report "V2-B9-J2-005 J3 must expand gap_before before J2"
                        severity failure;
                    assert i_word_event.line_end = '0'
                        report "V2-B9-J2-009 a valid line cannot end on its first word"
                        severity failure;
                else
                    assert i_word_event.line_start = '0' and
                           to_integer(i_word_event.word_count) = word_count_r and
                           to_integer(i_word_event.slot_count) = slot_count_r
                        report "V2-B9-J2-006 geometry changed inside a line"
                        severity failure;

                    accepted_words := captured_words_r + 1;
                    assert (i_word_event.line_end = '1') =
                           (accepted_words = expected_words_r)
                        report "V2-B9-J2-007 line_end does not match geometry"
                        severity failure;
                end if;
            end if;

        end if;
    end process p_contract;

end architecture rtl;
