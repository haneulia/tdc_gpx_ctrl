-- =============================================================================
-- tdc_gpx_line_packer.vhd
-- Canonical 32-bit cell-word packer for the VDMA data portion of one line.
--
-- The upstream cell_builder keeps its existing 32/64/128-bit beat format.
-- Only meaningful 32-bit words are retained:
--   ceil(max_hits/2) hit words + one metadata word per cell.
-- Words from adjacent cells are packed together. Padding is added only at the
-- end of a VDMA line so every line is 16-byte aligned and every output beat can
-- use full TKEEP/TSTRB. The 16-byte line alignment is independent of output
-- width and therefore keeps HSIZE/STRIDE identical at 32, 64, and 128 bits.
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.tdc_gpx_pkg.all;

entity tdc_gpx_line_packer is
    generic (
        g_TDATA_WIDTH : natural := 32
    );
    port (
        i_clk          : in  std_logic;
        i_rst_n        : in  std_logic;
        i_abort        : in  std_logic;
        i_max_hits_cfg : in  unsigned(2 downto 0);

        i_s_axis_tdata  : in  std_logic_vector(g_TDATA_WIDTH - 1 downto 0);
        i_s_axis_tvalid : in  std_logic;
        i_s_axis_tlast  : in  std_logic;
        o_s_axis_tready : out std_logic;

        o_m_axis_tdata  : out std_logic_vector(g_TDATA_WIDTH - 1 downto 0);
        o_m_axis_tvalid : out std_logic;
        o_m_axis_tlast  : out std_logic;
        i_m_axis_tready : in  std_logic;

        o_idle          : out std_logic
    );
end entity tdc_gpx_line_packer;

architecture rtl of tdc_gpx_line_packer is
    constant c_WORDS_PER_BEAT : positive := g_TDATA_WIDTH / 32;
    -- One incoming beat plus worst-case line padding can be appended while an
    -- older output beat is held under backpressure.
    constant c_QUEUE_WORDS : positive := 2 * c_WORDS_PER_BEAT + 4;

    subtype t_word is std_logic_vector(31 downto 0);
    type t_word_queue is array(0 to c_QUEUE_WORDS - 1) of t_word;

    signal s_queue_r            : t_word_queue := (others => (others => '0'));
    signal s_queue_count_r      : natural range 0 to c_QUEUE_WORDS := 0;
    signal s_cell_beat_idx_r    : natural range 0 to 7 := 0;
    signal s_line_word_mod4_r   : natural range 0 to 3 := 0;
    signal s_line_end_pending_r : std_logic := '0';

    signal s_out_tdata_r : std_logic_vector(g_TDATA_WIDTH - 1 downto 0)
                           := (others => '0');
    signal s_out_tvalid_r : std_logic := '0';
    signal s_out_tlast_r  : std_logic := '0';

    function fn_effective_max_hits(cfg : unsigned(2 downto 0)) return natural is
    begin
        case cfg is
            when "001" => return 1;
            when "010" => return 2;
            when "011" => return 3;
            when "100" => return 4;
            when "101" => return 5;
            when "110" => return 6;
            when others => return c_MAX_HITS_PER_STOP;
        end case;
    end function;

    function fn_min(a : natural; b : natural) return natural is
    begin
        if a < b then
            return a;
        else
            return b;
        end if;
    end function;

begin
    assert fn_output_width_supported(g_TDATA_WIDTH)
        report "tdc_gpx_line_packer: g_TDATA_WIDTH must be 32, 64, or 128"
        severity failure;

    assert (c_VDMA_LINE_ALIGN_BYTES / 4) mod c_WORDS_PER_BEAT = 0
        report "tdc_gpx_line_packer: line alignment must contain whole output beats"
        severity failure;

    -- Reserve room for one full input beat plus up to three 32-bit line-pad
    -- words. This conservative ready rule keeps the queue arithmetic bounded.
    o_s_axis_tready <= '1'
        when s_line_end_pending_r = '0'
         and s_queue_count_r <= c_QUEUE_WORDS - (c_WORDS_PER_BEAT + 3)
        else '0';

    o_m_axis_tdata  <= s_out_tdata_r;
    o_m_axis_tvalid <= s_out_tvalid_r;
    o_m_axis_tlast  <= s_out_tlast_r;
    o_idle <= '1' when s_queue_count_r = 0
                       and s_line_end_pending_r = '0'
                       and s_out_tvalid_r = '0'
              else '0';

    p_pack : process(i_clk)
        variable v_queue       : t_word_queue;
        variable v_count       : natural range 0 to c_QUEUE_WORDS;
        variable v_beat_idx    : natural range 0 to 7;
        variable v_mod4        : natural range 0 to 3;
        variable v_end_pending : std_logic;
        variable v_out_free    : boolean;
        variable v_out_data    : std_logic_vector(g_TDATA_WIDTH - 1 downto 0);
        variable v_max_hits    : natural range 1 to c_MAX_HITS_PER_STOP;
        variable v_hit_words   : natural range 1 to 4;
        variable v_hit_beats   : natural range 1 to 4;
        variable v_first_word  : natural range 0 to 12;
        variable v_remaining   : natural range 0 to 4;
        variable v_valid_words : natural range 1 to 4;
        variable v_new_mod4    : natural range 0 to 3;
        variable v_pad_words   : natural range 0 to 3;
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' or i_abort = '1' then
                s_queue_r            <= (others => (others => '0'));
                s_queue_count_r      <= 0;
                s_cell_beat_idx_r    <= 0;
                s_line_word_mod4_r   <= 0;
                s_line_end_pending_r <= '0';
                s_out_tdata_r        <= (others => '0');
                s_out_tvalid_r       <= '0';
                s_out_tlast_r        <= '0';
            else
                v_queue       := s_queue_r;
                v_count       := s_queue_count_r;
                v_beat_idx    := s_cell_beat_idx_r;
                v_mod4        := s_line_word_mod4_r;
                v_end_pending := s_line_end_pending_r;
                v_out_free    := s_out_tvalid_r = '0' or i_m_axis_tready = '1';

                -- Refill the registered output after its previous beat is
                -- accepted. Queue removal happens before any new input append.
                if v_out_free then
                    s_out_tvalid_r <= '0';
                    s_out_tlast_r  <= '0';
                    if v_count >= c_WORDS_PER_BEAT then
                        v_out_data := (others => '0');
                        for lane in 0 to c_WORDS_PER_BEAT - 1 loop
                            v_out_data(32 * lane + 31 downto 32 * lane)
                                := v_queue(lane);
                        end loop;
                        for idx in 0 to c_QUEUE_WORDS - c_WORDS_PER_BEAT - 1 loop
                            v_queue(idx) := v_queue(idx + c_WORDS_PER_BEAT);
                        end loop;
                        for idx in c_QUEUE_WORDS - c_WORDS_PER_BEAT
                                   to c_QUEUE_WORDS - 1 loop
                            v_queue(idx) := (others => '0');
                        end loop;
                        v_count := v_count - c_WORDS_PER_BEAT;

                        s_out_tdata_r  <= v_out_data;
                        s_out_tvalid_r <= '1';
                        if v_end_pending = '1' and v_count = 0 then
                            s_out_tlast_r  <= '1';
                            v_end_pending := '0';
                        end if;
                    end if;
                end if;

                if i_s_axis_tvalid = '1' and o_s_axis_tready = '1' then
                    v_max_hits  := fn_effective_max_hits(i_max_hits_cfg);
                    v_hit_words := fn_ceil_div(v_max_hits, 2);
                    v_hit_beats := fn_ceil_div(
                        v_max_hits, g_TDATA_WIDTH / c_HIT_SLOT_DATA_WIDTH);

                    if v_beat_idx < v_hit_beats then
                        v_first_word := v_beat_idx * c_WORDS_PER_BEAT;
                        if v_first_word < v_hit_words then
                            v_remaining := v_hit_words - v_first_word;
                        else
                            v_remaining := 0;
                        end if;
                        v_valid_words := fn_min(c_WORDS_PER_BEAT, v_remaining);
                    else
                        -- Metadata always occupies the lower 32-bit word of
                        -- the final input beat for a cell.
                        v_valid_words := 1;
                    end if;

                    for lane in 0 to c_WORDS_PER_BEAT - 1 loop
                        if lane < v_valid_words then
                            v_queue(v_count + lane) :=
                                i_s_axis_tdata(32 * lane + 31 downto 32 * lane);
                        end if;
                    end loop;
                    v_count    := v_count + v_valid_words;
                    v_new_mod4 := (v_mod4 + v_valid_words) mod 4;

                    if i_s_axis_tlast = '1' then
                        assert v_beat_idx = v_hit_beats
                            report "tdc_gpx_line_packer: line ended away from a cell metadata beat"
                            severity warning;
                        v_pad_words := (4 - v_new_mod4) mod 4;
                        for pad in 0 to 2 loop
                            if pad < v_pad_words then
                                v_queue(v_count + pad) := (others => '0');
                            end if;
                        end loop;
                        v_count       := v_count + v_pad_words;
                        v_mod4        := 0;
                        v_beat_idx    := 0;
                        v_end_pending := '1';
                    else
                        v_mod4 := v_new_mod4;
                        if v_beat_idx = v_hit_beats then
                            v_beat_idx := 0;
                        else
                            v_beat_idx := v_beat_idx + 1;
                        end if;
                    end if;
                end if;

                s_queue_r            <= v_queue;
                s_queue_count_r      <= v_count;
                s_cell_beat_idx_r    <= v_beat_idx;
                s_line_word_mod4_r   <= v_mod4;
                s_line_end_pending_r <= v_end_pending;
            end if;
        end if;
    end process p_pack;

end architecture rtl;
