library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_gpx_vdma_pkg.all;

-- J6 packs the canonical 32-bit Line-word stream into a synthesis-time AXIS
-- width. Words stay contiguous; only unused upper lanes of the final Beat are
-- zero-filled. TKEEP/TSTRB remain full because those bytes are part of aligned
-- VDMA HSIZE, not omitted transfer bytes.
entity lidar_gpx_axis_word_packer is
    generic (
        G_OUTPUT_WIDTH : positive := 32
    );
    port (
        i_clk   : in std_logic;
        i_rst_n : in std_logic;
        i_abort : in std_logic;

        i_line_word       : in  gpx_vdma_line_word_event_t;
        o_line_word_ready : out std_logic;

        o_m_axis_tdata  : out std_logic_vector(G_OUTPUT_WIDTH - 1 downto 0);
        o_m_axis_tkeep  : out std_logic_vector(G_OUTPUT_WIDTH / 8 - 1 downto 0);
        o_m_axis_tstrb  : out std_logic_vector(G_OUTPUT_WIDTH / 8 - 1 downto 0);
        o_m_axis_tuser  : out std_logic_vector(0 downto 0);
        o_m_axis_tvalid : out std_logic;
        o_m_axis_tlast  : out std_logic;
        i_m_axis_tready : in  std_logic;

        o_line_done : out std_logic;
        o_frame_done : out std_logic;
        o_idle      : out std_logic
    );
end entity lidar_gpx_axis_word_packer;

architecture rtl of lidar_gpx_axis_word_packer is

    constant C_WORDS_PER_BEAT : positive := G_OUTPUT_WIDTH /
        C_GPX_VDMA_WORD_WIDTH;

    signal assembly_data_r : std_logic_vector(
        G_OUTPUT_WIDTH - 1 downto 0) := (others => '0');
    signal assembly_word_count_r : natural range 0 to
        C_WORDS_PER_BEAT - 1 := 0;
    signal assembly_sof_r : std_logic := '0';
    signal assembly_frame_end_r : std_logic := '0';

    signal axis_data_r : std_logic_vector(
        G_OUTPUT_WIDTH - 1 downto 0) := (others => '0');
    signal axis_user_r : std_logic := '0';
    signal axis_valid_r : std_logic := '0';
    signal axis_last_r : std_logic := '0';
    signal axis_frame_end_r : std_logic := '0';
    signal output_ready_c : std_logic;
    signal line_done_r : std_logic := '0';
    signal frame_done_r : std_logic := '0';

    signal line_active_r : std_logic := '0';
    signal expected_word_index_r : gpx_vdma_line_word_index_t :=
        (others => '0');
    signal active_line_word_count_r : gpx_vdma_line_word_count_t :=
        (others => '0');

begin

    assert G_OUTPUT_WIDTH = 32 or G_OUTPUT_WIDTH = 64 or
           G_OUTPUT_WIDTH = 128
        report "V2-B9-J6-001 output width must be 32, 64, or 128"
        severity failure;

    output_ready_c <= '1' when axis_valid_r = '0' or
        i_m_axis_tready = '1' else '0';

    o_line_word_ready <= '1' when i_rst_n = '1' and i_abort = '0' and
        output_ready_c = '1' else '0';

    o_m_axis_tdata <= axis_data_r;
    o_m_axis_tkeep <= (others => '1');
    o_m_axis_tstrb <= (others => '1');
    o_m_axis_tuser(0) <= axis_user_r;
    o_m_axis_tvalid <= axis_valid_r;
    o_m_axis_tlast <= axis_last_r;
    o_line_done <= line_done_r;
    o_frame_done <= frame_done_r;
    o_idle <= '1' when assembly_word_count_r = 0 and
        axis_valid_r = '0' and line_active_r = '0' else '0';

    p_pack : process (i_clk)
        variable beat_data_v : std_logic_vector(
            G_OUTPUT_WIDTH - 1 downto 0);
        variable beat_sof_v : std_logic;
        variable beat_frame_end_v : std_logic;
        variable low_bit : natural;
        variable emit_beat_v : boolean;
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' or i_abort = '1' then
                assembly_data_r <= (others => '0');
                assembly_word_count_r <= 0;
                assembly_sof_r <= '0';
                assembly_frame_end_r <= '0';
                axis_data_r <= (others => '0');
                axis_user_r <= '0';
                axis_valid_r <= '0';
                axis_last_r <= '0';
                axis_frame_end_r <= '0';
                line_done_r <= '0';
                frame_done_r <= '0';
                line_active_r <= '0';
                expected_word_index_r <= (others => '0');
                active_line_word_count_r <= (others => '0');
            else
                line_done_r <= '0';
                frame_done_r <= '0';
                if axis_valid_r = '1' and i_m_axis_tready = '1' then
                    if axis_last_r = '1' then
                        line_done_r <= '1';
                    end if;
                    if axis_frame_end_r = '1' then
                        frame_done_r <= '1';
                    end if;
                    axis_valid_r <= '0';
                    axis_last_r <= '0';
                    axis_user_r <= '0';
                    axis_frame_end_r <= '0';
                end if;

                if i_line_word.valid = '1' and
                   o_line_word_ready = '1' then
                    beat_data_v := assembly_data_r;
                    beat_sof_v := assembly_sof_r;
                    beat_frame_end_v := assembly_frame_end_r;
                    low_bit := assembly_word_count_r *
                        C_GPX_VDMA_WORD_WIDTH;
                    beat_data_v(low_bit + C_GPX_VDMA_WORD_WIDTH - 1
                        downto low_bit) := i_line_word.data;
                    if i_line_word.line_start = '1' and
                       i_line_word.first_column = '1' then
                        beat_sof_v := '1';
                    end if;
                    if i_line_word.frame_end = '1' then
                        beat_frame_end_v := '1';
                    end if;

                    emit_beat_v :=
                        assembly_word_count_r + 1 = C_WORDS_PER_BEAT or
                        i_line_word.line_end = '1';

                    if emit_beat_v then
                        axis_data_r <= beat_data_v;
                        axis_user_r <= beat_sof_v;
                        axis_valid_r <= '1';
                        axis_last_r <= i_line_word.line_end;
                        axis_frame_end_r <= beat_frame_end_v;
                        assembly_data_r <= (others => '0');
                        assembly_word_count_r <= 0;
                        assembly_sof_r <= '0';
                        assembly_frame_end_r <= '0';
                    else
                        assembly_data_r <= beat_data_v;
                        assembly_word_count_r <=
                            assembly_word_count_r + 1;
                        assembly_sof_r <= beat_sof_v;
                        assembly_frame_end_r <= beat_frame_end_v;
                    end if;

                    if line_active_r = '0' then
                        line_active_r <= not i_line_word.line_end;
                        expected_word_index_r <= to_unsigned(
                            1, expected_word_index_r'length);
                        active_line_word_count_r <=
                            i_line_word.line_word_count;
                    elsif i_line_word.line_end = '1' then
                        line_active_r <= '0';
                        expected_word_index_r <= (others => '0');
                    else
                        expected_word_index_r <=
                            expected_word_index_r + 1;
                    end if;
                end if;
            end if;
        end if;
    end process p_pack;

    p_contract : process (i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '1' and i_abort = '0' and
               i_line_word.valid = '1' and
               o_line_word_ready = '1' then
                assert i_line_word.line_word_count /= 0 and
                       (i_line_word.line_end = '1') =
                       (i_line_word.word_index + 1 =
                        i_line_word.line_word_count)
                    report "V2-B9-J6-002 Line count/end mismatch"
                    severity failure;
                assert i_line_word.frame_end = '0' or
                       (i_line_word.kind = GPX_VDMA_LINE_FACE_FOOTER and
                        i_line_word.line_end = '1')
                    report "V2-B9-J6-006 Frame end is not a Footer Line end"
                    severity failure;
                if line_active_r = '0' then
                    assert i_line_word.line_start = '1' and
                           i_line_word.word_index = 0
                        report "V2-B9-J6-003 Line did not start at Word zero"
                        severity failure;
                else
                    assert i_line_word.line_start = '0' and
                           i_line_word.word_index =
                               expected_word_index_r and
                           i_line_word.line_word_count =
                               active_line_word_count_r
                        report "V2-B9-J6-004 Word order/geometry changed"
                        severity failure;
                end if;
            end if;

            if i_rst_n = '1' and axis_valid_r = '1' and
               i_m_axis_tready = '0' then
                assert output_ready_c = '0'
                    report "V2-B9-J6-005 output-ready contract mismatch"
                    severity failure;
            end if;
        end if;
    end process p_contract;

end architecture rtl;
