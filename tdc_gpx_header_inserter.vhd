-- =============================================================================
-- tdc_gpx_header_inserter.vhd
-- TDC-GPX Controller - Header Inserter (SOF / EOL / Metadata)
-- =============================================================================
--
-- Purpose:
--   Prepends a fixed-length header prefix to every VDMA line.
--   First line of each face contains actual header metadata;
--   subsequent lines have zero-filled prefix.
--
--   Per-face flow:
--     1. face_start -> latch CSR snapshot + metadata -> ST_PREFIX
--     2. ST_PREFIX: generate c_G_HDR_PREFIX_BEATS (12) beats
--        - Line 0: header metadata (12 valid beats = 48 bytes)
--        - Line 1+: zeros
--        - Beat 0 of line 0: tuser(0)=1 (SOF for AXI-Stream Video)
--     3. ST_DATA: passthrough data beats from face_assembler
--        - Upstream tlast -> end of data -> output tlast (EOL)
--        - col_cnt increments on each tlast
--        - col_cnt = cols_per_face -> frame_done, ST_IDLE
--
--   VDMA line structure (all lines uniform):
--     [HDR prefix, 12 beats] [Cell data, 256 beats] = 268 beats = 1072 bytes
--
--     Header prefix layout (only line 0, little-endian):
--     Beat  Offset  Content (packed, LE)
--     ----  ------  -------------------------------------------------
--       0   0x00    magic "TDCG"                        [31:0]  32 full
--       1   0x04    vdma_frame_id                       [31:0]  32 full
--       2   0x08    scan_frame_id (reserved=0)          [31:0]  32 full
--       3   0x0C    face_id[7:0] | mask[11:8] |
--                   n_faces[14:12] | stops[18:15] |
--                   ndcap[22:19] | pipe_en[23] |
--                   hit_store[25:24] | dist_scale[28:26] |
--                   drain_mode[29]                              30/32
--       4   0x10    rows_per_face[15:0] | cols[31:16]   [31:0]  32 full
--       5   0x14    max_hits[7:0] | cell_size[15:8] |
--                   hit_slot_w[23:16] | n_chips[27:24] |
--                   max_stops[31:28]                            32 full
--       6   0x18    shot_seq[15:0] | bin_res_ps[31:16]  [31:0]  32 full
--       7   0x1C    start_off1[17:0] | cell_fmt[21:18] |
--                   chip_err_mask[25:22]                        26/32
--       8   0x20    k_dist_fixed                        [31:0]  32 full
--       9   0x24    timestamp_ns [31:0]                 [31:0]  32 full
--      10   0x28    timestamp_ns [63:32]                [31:0]  32 full
--      11   0x2C    error_count                         [31:0]  32 full
--
--   VDMA frame:
--     HSIZE  = (c_DATA_BEATS_MAX + c_G_HDR_PREFIX_BEATS) × TDATA_BYTES
--     VSIZE  = cols_per_face
--     STRIDE = HSIZE
--
-- Standard: VHDL-93 compatible
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.tdc_gpx_pkg.all;

entity tdc_gpx_header_inserter is
    generic (
        g_TDATA_WIDTH : natural := c_TDATA_WIDTH   -- 32 or 64
    );
    port (
        i_clk               : in  std_logic;
        i_rst_n             : in  std_logic;

        -- Face control
        i_face_start        : in  std_logic;    -- 1-clk pulse: new face
        i_face_abort        : in  std_logic;    -- 1-clk pulse: abort current face (overrun)

        -- Configuration (from CSR, stable before face_start)
        i_cfg               : in  t_tdc_cfg;

        -- Header metadata (captured at face_start)
        i_vdma_frame_id     : in  unsigned(31 downto 0);
        i_face_id           : in  unsigned(7 downto 0);
        i_shot_seq_start    : in  unsigned(c_SHOT_SEQ_WIDTH - 1 downto 0);
        i_timestamp_ns      : in  unsigned(63 downto 0);
        i_chip_error_mask   : in  std_logic_vector(c_N_CHIPS - 1 downto 0);
        i_chip_error_cnt    : in  std_logic_vector(31 downto 0);
        i_bin_resolution_ps : in  unsigned(15 downto 0);
        i_k_dist_fixed      : in  unsigned(31 downto 0);

        -- VDMA line geometry (from top, shared with VDMA config)
        i_rows_per_face     : in  unsigned(15 downto 0);   -- active_chips × stops_per_chip

        -- AXI-Stream slave (from face_assembler)
        i_s_axis_tdata      : in  std_logic_vector(g_TDATA_WIDTH - 1 downto 0);
        i_s_axis_tvalid     : in  std_logic;
        i_s_axis_tlast      : in  std_logic;
        o_s_axis_tready     : out std_logic;

        -- AXI-Stream master (to downstream / CDC FIFO)
        o_m_axis_tdata      : out std_logic_vector(g_TDATA_WIDTH - 1 downto 0);
        o_m_axis_tvalid     : out std_logic;
        o_m_axis_tlast      : out std_logic;
        o_m_axis_tuser      : out std_logic_vector(0 downto 0);   -- (0) = SOF
        i_m_axis_tready     : in  std_logic;

        -- Status
        o_frame_done        : out std_logic;    -- 1-clk pulse: face complete
        o_draining          : out std_logic;    -- '1' in ST_DRAIN_LAST (final beat pending)
        o_last_line         : out std_logic;    -- '1' when processing last line of face
        o_idle              : out std_logic     -- '1' when FSM is in ST_IDLE
    );
end entity tdc_gpx_header_inserter;

architecture rtl of tdc_gpx_header_inserter is

    -- =========================================================================
    -- FSM
    -- =========================================================================
    type t_state is (ST_IDLE, ST_PREFIX, ST_DATA, ST_DRAIN_LAST, ST_ABORT_DRAIN);
    signal s_state_r : t_state := ST_IDLE;

    -- Generic-derived constants
    constant c_G_HDR_PREFIX_BEATS : natural := fn_hdr_prefix_beats(g_TDATA_WIDTH);

    -- =========================================================================
    -- Latched header fields (captured at face_start)
    -- =========================================================================
    -- Identity
    signal s_vdma_frame_id_r     : unsigned(31 downto 0) := (others => '0');
    signal s_face_id_r           : unsigned(7 downto 0)  := (others => '0');

    -- Structure (from i_cfg)
    signal s_active_chip_mask_r  : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '0');
    signal s_n_faces_r           : unsigned(2 downto 0)  := (others => '0');
    signal s_rows_per_face_r     : unsigned(15 downto 0) := (others => '0');
    signal s_cols_per_face_r     : unsigned(15 downto 0) := (others => '0');
    signal s_stops_per_chip_r    : unsigned(3 downto 0)  := (others => '0');
    signal s_hit_store_mode_r    : unsigned(1 downto 0)  := (others => '0');

    -- Measurement
    signal s_shot_seq_start_r    : unsigned(c_SHOT_SEQ_WIDTH - 1 downto 0) := (others => '0');
    signal s_bin_resolution_ps_r : unsigned(15 downto 0) := (others => '0');
    signal s_dist_scale_r        : unsigned(2 downto 0)  := (others => '0');
    signal s_drain_mode_r        : std_logic := '0';

    -- Calibration
    signal s_start_off1_r        : unsigned(17 downto 0) := (others => '0');
    signal s_n_drain_cap_r       : unsigned(3 downto 0)  := (others => '0');
    signal s_pipeline_en_r       : std_logic := '0';

    -- Distance
    signal s_k_dist_fixed_r      : unsigned(31 downto 0) := (others => '0');

    -- Timestamp
    signal s_timestamp_ns_r      : unsigned(63 downto 0) := (others => '0');

    -- Error
    signal s_chip_error_mask_r   : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '0');
    signal s_chip_error_cnt_r    : std_logic_vector(31 downto 0) := (others => '0');

    -- =========================================================================
    -- Counters
    -- =========================================================================
    signal s_prefix_idx_r      : unsigned(3 downto 0) := (others => '0');   -- 0..11
    signal s_col_cnt_r         : unsigned(15 downto 0) := (others => '0');
    signal s_cols_per_face_m1_r : unsigned(15 downto 0) := (others => '0'); -- pre-computed

    -- =========================================================================
    -- Header word ROM: pre-computed at face_start, indexed at ST_PREFIX.
    -- Eliminates 12:1 case mux from the beat-generation critical path.
    -- =========================================================================
    type t_hdr_rom is array(0 to 11) of std_logic_vector(31 downto 0);
    signal s_hdr_rom_r : t_hdr_rom := (others => (others => '0'));

    -- =========================================================================
    -- First-line flag (header valid only on line 0)
    -- =========================================================================
    signal s_first_line_r : std_logic := '0';

    -- =========================================================================
    -- Output register (1-deep pipeline)
    -- =========================================================================
    signal s_out_tdata_r  : std_logic_vector(g_TDATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_out_tvalid_r : std_logic := '0';
    signal s_out_tlast_r  : std_logic := '0';
    signal s_out_tuser_r  : std_logic := '0';

    -- =========================================================================
    -- Status
    -- =========================================================================
    signal s_frame_done_r : std_logic := '0';

    -- =========================================================================
    -- Flow control
    -- =========================================================================
    signal s_can_accept : std_logic;

begin

    -- =========================================================================
    -- Output assignments (all registered)
    -- =========================================================================
    o_m_axis_tdata    <= s_out_tdata_r;
    o_m_axis_tvalid   <= s_out_tvalid_r;
    o_m_axis_tlast    <= s_out_tlast_r;
    o_m_axis_tuser(0) <= s_out_tuser_r;
    o_frame_done      <= s_frame_done_r;
    o_draining        <= '1' when s_state_r = ST_DRAIN_LAST else '0';
    o_idle            <= '1' when s_state_r = ST_IDLE else '0';
    -- '1' during the entire last line: from ST_PREFIX/ST_DATA on the last col
    -- through ST_DRAIN_LAST.  Closes the window where assembler output skid
    -- holds the final beat but header hasn't consumed it yet.
    o_last_line       <= '1' when s_state_r /= ST_IDLE
                                  and s_col_cnt_r = s_cols_per_face_m1_r
                              else '0';

    -- =========================================================================
    -- Flow control
    -- =========================================================================
    s_can_accept    <= (not s_out_tvalid_r) or i_m_axis_tready;
    o_s_axis_tready <= s_can_accept when s_state_r = ST_DATA else '0';

    -- =========================================================================
    -- Main process
    -- =========================================================================
    p_main : process(i_clk)
        variable v_beat    : natural range 0 to 15;
        variable v_hdr_data : std_logic_vector(g_TDATA_WIDTH - 1 downto 0);
        variable v_word     : std_logic_vector(31 downto 0);
        variable v_widx     : natural;

        -- 32-bit header word ROM (word index 0..11)
        procedure get_hdr_word(widx : natural; word : out std_logic_vector) is
        begin
            word := (others => '0');
            case widx is
                when 0  => word := c_HEADER_MAGIC;
                when 1  => word := std_logic_vector(s_vdma_frame_id_r);
                when 2  => word := (others => '0');
                when 3  =>
                    word(7 downto 0)   := std_logic_vector(s_face_id_r);
                    word(11 downto 8)  := s_active_chip_mask_r;
                    word(14 downto 12) := std_logic_vector(s_n_faces_r);
                    word(18 downto 15) := std_logic_vector(s_stops_per_chip_r);
                    word(22 downto 19) := std_logic_vector(s_n_drain_cap_r);
                    word(23)           := s_pipeline_en_r;
                    word(25 downto 24) := std_logic_vector(s_hit_store_mode_r);
                    word(28 downto 26) := std_logic_vector(s_dist_scale_r);
                    word(29)           := s_drain_mode_r;
                when 4  =>
                    word(15 downto 0)  := std_logic_vector(s_rows_per_face_r);
                    word(31 downto 16) := std_logic_vector(s_cols_per_face_r);
                when 5  =>
                    word(7 downto 0)   := std_logic_vector(to_unsigned(c_MAX_HITS_PER_STOP, 8));
                    word(15 downto 8)  := std_logic_vector(to_unsigned(c_CELL_SIZE_BYTES, 8));
                    word(23 downto 16) := std_logic_vector(to_unsigned(c_HIT_SLOT_DATA_WIDTH, 8));
                    word(27 downto 24) := std_logic_vector(to_unsigned(c_N_CHIPS, 4));
                    word(31 downto 28) := std_logic_vector(to_unsigned(c_MAX_STOPS_PER_CHIP, 4));
                when 6  =>
                    word(c_SHOT_SEQ_WIDTH - 1 downto 0) := std_logic_vector(s_shot_seq_start_r);
                    word(31 downto 16) := std_logic_vector(s_bin_resolution_ps_r);
                when 7  =>
                    word(17 downto 0)  := std_logic_vector(s_start_off1_r);
                    word(21 downto 18) := std_logic_vector(to_unsigned(c_CELL_FORMAT, 4));
                    word(25 downto 22) := s_chip_error_mask_r;
                when 8  => word := std_logic_vector(s_k_dist_fixed_r);
                when 9  => word := std_logic_vector(s_timestamp_ns_r(31 downto 0));
                when 10 => word := std_logic_vector(s_timestamp_ns_r(63 downto 32));
                when 11 => word := s_chip_error_cnt_r;
                when others => word := (others => '0');
            end case;
        end procedure;
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                s_state_r           <= ST_IDLE;
                s_prefix_idx_r      <= (others => '0');
                s_col_cnt_r         <= (others => '0');
                s_cols_per_face_m1_r <= (others => '0');
                s_first_line_r      <= '0';
                s_out_tdata_r       <= (others => '0');
                s_out_tvalid_r      <= '0';
                s_out_tlast_r       <= '0';
                s_out_tuser_r       <= '0';
                s_frame_done_r      <= '0';
                s_rows_per_face_r   <= (others => '0');
                s_cols_per_face_r   <= (others => '0');
                s_vdma_frame_id_r   <= (others => '0');
                s_face_id_r         <= (others => '0');
                s_active_chip_mask_r <= (others => '0');
                s_n_faces_r         <= (others => '0');
                s_stops_per_chip_r  <= (others => '0');
                s_hit_store_mode_r  <= (others => '0');
                s_shot_seq_start_r  <= (others => '0');
                s_bin_resolution_ps_r <= (others => '0');
                s_dist_scale_r      <= (others => '0');
                s_drain_mode_r      <= '0';
                s_start_off1_r      <= (others => '0');
                s_n_drain_cap_r     <= (others => '0');
                s_pipeline_en_r     <= '0';
                s_k_dist_fixed_r    <= (others => '0');
                s_timestamp_ns_r    <= (others => '0');
                s_chip_error_mask_r <= (others => '0');
                s_chip_error_cnt_r  <= (others => '0');
            else
                -- Default: clear single-cycle pulses
                s_frame_done_r <= '0';

                -- Default: clear output register on handshake
                if s_out_tvalid_r = '1' and i_m_axis_tready = '1' then
                    s_out_tvalid_r <= '0';
                    s_out_tlast_r  <= '0';
                    s_out_tuser_r  <= '0';
                end if;

                case s_state_r is

                -- ==============================================================
                -- ST_IDLE: wait for face_start
                -- ==============================================================
                when ST_IDLE =>
                    null;  -- face_start handled in post-case override

                -- ==============================================================
                -- ST_PREFIX: generate c_G_HDR_PREFIX_BEATS per line
                --   Line 0 (first_line=1): header metadata
                --   Line 1+ (first_line=0): zeros
                -- ==============================================================
                when ST_PREFIX =>
                    if s_can_accept = '1' then
                        v_beat    := to_integer(s_prefix_idx_r);
                        v_hdr_data := (others => '0');

                        -- ==============================================
                        -- Header ROM: 12 × 32-bit words (48 bytes)
                        -- 32-bit mode: 1 word/beat, 12 beats
                        -- 64-bit mode: 2 words/beat, 6 beats
                        --   beat N → word[2N+1](hi) | word[2N](lo)
                        -- ==============================================
                        if s_first_line_r = '1' then
                            -- Read from pre-built ROM: 1 array index per 32-bit word.
                            -- 32-bit: 1 word/beat, 64-bit: 2 words/beat.
                            for w in 0 to (g_TDATA_WIDTH / 32) - 1 loop
                                v_widx := v_beat * (g_TDATA_WIDTH / 32) + w;
                                v_hdr_data(w * 32 + 31 downto w * 32) := s_hdr_rom_r(v_widx);
                            end loop;
                        end if;
                        -- else: v_hdr_data stays all zeros (non-first lines)

                        -- Load into output register
                        s_out_tdata_r  <= v_hdr_data;
                        s_out_tvalid_r <= '1';
                        s_out_tlast_r  <= '0';

                        -- SOF on first beat of first line only
                        if s_first_line_r = '1' and s_prefix_idx_r = 0 then
                            s_out_tuser_r <= '1';
                        else
                            s_out_tuser_r <= '0';
                        end if;

                        -- Advance prefix counter
                        if s_prefix_idx_r = c_G_HDR_PREFIX_BEATS - 1 then
                            s_prefix_idx_r <= (others => '0');
                            s_state_r      <= ST_DATA;
                        else
                            s_prefix_idx_r <= s_prefix_idx_r + 1;
                        end if;
                    end if;

                -- ==============================================================
                -- ST_DATA: passthrough data beats from face_assembler
                --   Upstream tlast = end of cell data for this line -> EOL.
                --   After EOL: next line prefix (or frame done).
                -- ==============================================================
                when ST_DATA =>
                    if s_can_accept = '1' and i_s_axis_tvalid = '1' then
                        s_out_tdata_r  <= i_s_axis_tdata;
                        s_out_tvalid_r <= '1';
                        s_out_tlast_r  <= i_s_axis_tlast;
                        s_out_tuser_r  <= '0';

                        -- Line complete (upstream EOL)
                        if i_s_axis_tlast = '1' then
                            if s_col_cnt_r = s_cols_per_face_m1_r then
                                -- All lines sent → wait for last beat to
                                -- drain before reporting frame_done.
                                s_state_r      <= ST_DRAIN_LAST;
                            else
                                s_col_cnt_r    <= s_col_cnt_r + 1;
                                s_first_line_r <= '0';
                                s_state_r      <= ST_PREFIX;
                            end if;
                        end if;
                    end if;

                -- ==============================================================
                -- ST_DRAIN_LAST: final beat is in output register; wait for
                --   downstream to consume it before reporting frame_done.
                --   This prevents a race where face_start arrives before the
                --   last beat is handed off and clears the output register.
                -- ==============================================================
                when ST_DRAIN_LAST =>
                    if s_out_tvalid_r = '0' or i_m_axis_tready = '1' then
                        -- Last beat consumed (or was already consumed)
                        s_frame_done_r <= '1';
                        s_state_r      <= ST_IDLE;
                    end if;

                -- ==============================================================
                -- ST_ABORT_DRAIN: wait for pending output beat to handshake,
                --   then go to IDLE.  Prevents AXI-Stream tvalid withdrawal.
                -- ==============================================================
                when ST_ABORT_DRAIN =>
                    if i_m_axis_tready = '1' then
                        s_out_tvalid_r <= '0';
                        s_state_r      <= ST_IDLE;
                    end if;

                end case;

                -- =============================================================
                -- face_start: latch fields and start first line prefix.
                -- Works from ANY state (ST_IDLE normal, others = abort/restart).
                -- Post-case override: last assignment wins.
                -- =============================================================
                if i_face_start = '1' then
                    -- All fields latched at face_start (= s_face_start_r in top).
                    -- Error fields (chip_error_mask, chip_error_cnt) are a
                    -- SNAPSHOT at face-start time, NOT the post-drain result.
                    -- This is intentional: the header records the pre-face
                    -- error state. Post-drain errors appear in the NEXT header.

                    -- Identity
                    s_vdma_frame_id_r     <= i_vdma_frame_id;
                    s_face_id_r           <= i_face_id;

                    -- Structure
                    s_active_chip_mask_r  <= i_cfg.active_chip_mask;
                    s_n_faces_r           <= i_cfg.n_faces;
                    s_cols_per_face_r     <= i_cfg.cols_per_face;
                    s_cols_per_face_m1_r  <= i_cfg.cols_per_face - 1;
                    s_stops_per_chip_r    <= i_cfg.stops_per_chip;
                    s_hit_store_mode_r    <= i_cfg.hit_store_mode;

                    -- Measurement
                    s_shot_seq_start_r    <= i_shot_seq_start;
                    s_bin_resolution_ps_r <= i_bin_resolution_ps;
                    s_dist_scale_r        <= i_cfg.dist_scale;
                    s_drain_mode_r        <= i_cfg.drain_mode;

                    -- Calibration
                    s_start_off1_r        <= i_cfg.start_off1;
                    s_n_drain_cap_r       <= i_cfg.n_drain_cap;
                    s_pipeline_en_r       <= i_cfg.pipeline_en;

                    -- Distance
                    s_k_dist_fixed_r      <= i_k_dist_fixed;

                    -- Timestamp
                    s_timestamp_ns_r      <= i_timestamp_ns;

                    -- Error
                    s_chip_error_mask_r   <= i_chip_error_mask;
                    s_chip_error_cnt_r    <= i_chip_error_cnt;

                    -- Line geometry (from top-level)
                    s_rows_per_face_r     <= i_rows_per_face;

                    -- Pre-build header word ROM: all 12 words computed once,
                    -- eliminates 12:1 case mux from ST_PREFIX critical path.
                    for wi in 0 to 11 loop
                        get_hdr_word(wi, v_word);
                        s_hdr_rom_r(wi) <= v_word;
                    end loop;

                    s_prefix_idx_r <= (others => '0');
                    s_col_cnt_r    <= (others => '0');
                    s_first_line_r <= '1';
                    if s_out_tvalid_r = '0' or i_m_axis_tready = '1' then
                        s_out_tvalid_r <= '0';
                    end if;
                    s_out_tlast_r  <= '0';
                    s_out_tuser_r  <= '0';
                    s_state_r      <= ST_PREFIX;
                end if;

                -- =============================================================
                -- face_abort: overrun or cmd_stop/soft_reset → discard face.
                -- If no pending output beat, go directly to IDLE.
                -- If a beat is pending (tvalid=1, tready=0), enter
                -- ST_ABORT_DRAIN to let it handshake before clearing —
                -- avoids AXI-Stream tvalid withdrawal violation.
                -- NOTE: abort does NOT emit synthetic tlast. The aborted
                -- face produces a truncated line (no EOL marker). This is
                -- intentional: the corrupted data should NOT look like a
                -- valid line to VDMA.  SW detects this via shot_overrun
                -- status and discards the affected frame.
                -- Higher priority than face_start (last-assignment wins).
                -- =============================================================
                if i_face_abort = '1' then
                    if s_out_tvalid_r = '0' or i_m_axis_tready = '1' then
                        -- No pending beat (or consumed this cycle)
                        s_out_tvalid_r <= '0';
                        s_state_r      <= ST_IDLE;
                    else
                        -- Beat pending: wait for downstream to accept it
                        s_state_r      <= ST_ABORT_DRAIN;
                    end if;
                end if;

            end if;
        end if;
    end process p_main;

end architecture rtl;
