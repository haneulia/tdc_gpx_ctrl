-- =============================================================================
-- tdc_gpx_header_inserter.vhd
-- TDC-GPX Controller - Header Inserter (SOF / EOL / Metadata)
-- =============================================================================
--
-- Purpose:
--   Prepends a metadata header line at the start of each vdma_frame, then
--   passes through data lines from face_assembler.
--
--   Per-frame flow:
--     1. face_start -> latch CSR snapshot + metadata -> ST_HEADER
--     2. ST_HEADER: generate header beats (12 valid + padding zeros)
--        - Beat 0: tuser(0)=1 (SOF for AXI-Stream Video / VDMA)
--        - Last beat: tlast=1 (EOL)
--     3. ST_DATA: passthrough face_assembler rows, count cols
--        - Each row's tlast from upstream forwarded as EOL
--        - col_cnt increments on each row tlast
--        - col_cnt = cols_per_face -> frame_done, ST_IDLE
--
--   Header line structure (48 bytes valid, rest 0-padded to hsize_actual):
--     See 05_header_and_sw_parser_contract.md for full byte-level layout.
--     All multi-byte fields are little-endian.
--
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
--     12-N  0x30+   padding (zeros)
--
--   VDMA frame line count:
--     VSIZE = cols_per_face + 1 (1 header line + cols_per_face data lines)
--
--   Output register: 1-deep pipeline for registered outputs.
--   tready to upstream: combinational from registered signals + i_m_axis_tready.
--   (Upstream face_assembler has output skid buffer; downstream CDC FIFO
--    provides additional buffering.)
--
-- Standard: VHDL-93 compatible
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.tdc_gpx_pkg.all;

entity tdc_gpx_header_inserter is
    port (
        i_clk               : in  std_logic;
        i_rst_n             : in  std_logic;

        -- Face control
        i_face_start        : in  std_logic;    -- 1-clk pulse: new vdma_frame

        -- Configuration (from CSR, stable before face_start)
        i_cfg               : in  t_tdc_cfg;

        -- Header metadata (captured at face_start)
        i_vdma_frame_id     : in  unsigned(31 downto 0);
        i_face_id           : in  unsigned(7 downto 0);
        i_shot_seq_start    : in  unsigned(c_SHOT_SEQ_WIDTH - 1 downto 0);
        i_timestamp_ns      : in  unsigned(63 downto 0);
        i_chip_error_mask   : in  std_logic_vector(c_N_CHIPS - 1 downto 0);
        i_chip_error_cnt    : in  std_logic_vector(31 downto 0);  -- packed LE: cnt3 & cnt2 & cnt1 & cnt0
        i_bin_resolution_ps : in  unsigned(15 downto 0);
        i_k_dist_fixed      : in  unsigned(31 downto 0);

        -- VDMA line geometry (from top, shared with VDMA config)
        i_hsize_bytes       : in  unsigned(15 downto 0);   -- HSIZE in bytes (must be TDATA-aligned)
        i_rows_per_face     : in  unsigned(15 downto 0);   -- active_chips × stops_per_chip

        -- AXI-Stream slave (from face_assembler)
        i_s_axis_tdata      : in  std_logic_vector(c_TDATA_WIDTH - 1 downto 0);
        i_s_axis_tvalid     : in  std_logic;
        i_s_axis_tlast      : in  std_logic;
        o_s_axis_tready     : out std_logic;

        -- AXI-Stream master (to downstream / CDC FIFO)
        o_m_axis_tdata      : out std_logic_vector(c_TDATA_WIDTH - 1 downto 0);
        o_m_axis_tvalid     : out std_logic;
        o_m_axis_tlast      : out std_logic;
        o_m_axis_tuser      : out std_logic_vector(0 downto 0);   -- (0) = SOF
        i_m_axis_tready     : in  std_logic;

        -- Status
        o_frame_done        : out std_logic     -- 1-clk pulse: vdma_frame complete
    );
end entity tdc_gpx_header_inserter;

architecture rtl of tdc_gpx_header_inserter is

    -- =========================================================================
    -- FSM
    -- =========================================================================
    type t_state is (ST_IDLE, ST_HEADER, ST_DATA);
    signal s_state_r : t_state := ST_IDLE;

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
    -- Derived (latched)
    -- =========================================================================
    signal s_hsize_beats_last_r : unsigned(13 downto 0) := (others => '0');  -- HSIZE/4 - 1

    -- =========================================================================
    -- Counters
    -- =========================================================================
    signal s_beat_idx_r : unsigned(13 downto 0) := (others => '0');
    signal s_col_cnt_r  : unsigned(15 downto 0) := (others => '0');

    -- =========================================================================
    -- Output register (1-deep pipeline)
    -- =========================================================================
    signal s_out_tdata_r  : std_logic_vector(c_TDATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_out_tvalid_r : std_logic := '0';
    signal s_out_tlast_r  : std_logic := '0';
    signal s_out_tuser_r  : std_logic := '0';

    -- =========================================================================
    -- Status
    -- =========================================================================
    signal s_frame_done_r : std_logic := '0';

    -- =========================================================================
    -- Flow control: output register can accept new data
    --   s_out_tvalid_r and i_m_axis_tready are the only inputs.
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

    -- =========================================================================
    -- Flow control
    -- =========================================================================
    s_can_accept    <= (not s_out_tvalid_r) or i_m_axis_tready;
    o_s_axis_tready <= s_can_accept when s_state_r = ST_DATA else '0';

    -- =========================================================================
    -- Main process
    -- =========================================================================
    p_main : process(i_clk)
        variable v_beat       : natural range 0 to 16383;
        variable v_hdr_data   : std_logic_vector(c_TDATA_WIDTH - 1 downto 0);
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                s_state_r           <= ST_IDLE;
                s_beat_idx_r        <= (others => '0');
                s_col_cnt_r         <= (others => '0');
                s_out_tdata_r       <= (others => '0');
                s_out_tvalid_r      <= '0';
                s_out_tlast_r       <= '0';
                s_out_tuser_r       <= '0';
                s_frame_done_r      <= '0';
                s_hsize_beats_last_r <= (others => '0');
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
                -- ST_HEADER: generate header beats into output register
                --   12 beats of metadata (48 bytes), rest padding zeros.
                --   Beat 0: SOF (tuser=1). Last beat: EOL (tlast=1).
                -- ==============================================================
                when ST_HEADER =>
                    if s_can_accept = '1' then
                        v_beat    := to_integer(s_beat_idx_r);
                        v_hdr_data := (others => '0');

                        -- ==============================================
                        -- Header beat ROM (packed, little-endian)
                        -- 12 valid beats (48 bytes), rest zero-padded
                        -- ==============================================
                        case v_beat is

                            -- Beat 0 (0x00): magic_word "TDCG" (LE: T,D,C,G)
                            when 0 =>
                                v_hdr_data := c_HEADER_MAGIC;

                            -- Beat 1 (0x04): vdma_frame_id [31:0]
                            when 1 =>
                                v_hdr_data := std_logic_vector(s_vdma_frame_id_r);

                            -- Beat 2 (0x08): scan_frame_id [31:0] (Phase 1: always 0)
                            when 2 =>
                                v_hdr_data := (others => '0');

                            -- Beat 3 (0x0C): face_id[7:0] | mask[11:8] |
                            --   n_faces[14:12] | stops[18:15] | ndcap[22:19] |
                            --   pipe_en[23] | hit_store[25:24] |
                            --   dist_scale[28:26] | drain_mode[29]
                            when 3 =>
                                v_hdr_data(7 downto 0)   := std_logic_vector(s_face_id_r);
                                v_hdr_data(11 downto 8)  := s_active_chip_mask_r;
                                v_hdr_data(14 downto 12) := std_logic_vector(s_n_faces_r);
                                v_hdr_data(18 downto 15) := std_logic_vector(s_stops_per_chip_r);
                                v_hdr_data(22 downto 19) := std_logic_vector(s_n_drain_cap_r);
                                v_hdr_data(23)           := s_pipeline_en_r;
                                v_hdr_data(25 downto 24) := std_logic_vector(s_hit_store_mode_r);
                                v_hdr_data(28 downto 26) := std_logic_vector(s_dist_scale_r);
                                v_hdr_data(29)           := s_drain_mode_r;

                            -- Beat 4 (0x10): rows_per_face[15:0] | cols_per_face[31:16]
                            when 4 =>
                                v_hdr_data(15 downto 0)  := std_logic_vector(s_rows_per_face_r);
                                v_hdr_data(31 downto 16) := std_logic_vector(s_cols_per_face_r);

                            -- Beat 5 (0x14): max_hits[7:0] | cell_size[15:8] |
                            --   hit_slot_width[23:16] | n_chips[27:24] |
                            --   max_stops[31:28]
                            when 5 =>
                                v_hdr_data(7 downto 0)   := std_logic_vector(to_unsigned(c_MAX_HITS_PER_STOP, 8));
                                v_hdr_data(15 downto 8)  := std_logic_vector(to_unsigned(c_CELL_SIZE_BYTES, 8));
                                v_hdr_data(23 downto 16) := std_logic_vector(to_unsigned(c_HIT_SLOT_DATA_WIDTH, 8));
                                v_hdr_data(27 downto 24) := std_logic_vector(to_unsigned(c_N_CHIPS, 4));
                                v_hdr_data(31 downto 28) := std_logic_vector(to_unsigned(c_MAX_STOPS_PER_CHIP, 4));

                            -- Beat 6 (0x18): shot_seq[15:0] | bin_resolution_ps[31:16]
                            when 6 =>
                                v_hdr_data(c_SHOT_SEQ_WIDTH - 1 downto 0) :=
                                    std_logic_vector(s_shot_seq_start_r);
                                v_hdr_data(31 downto 16) := std_logic_vector(s_bin_resolution_ps_r);

                            -- Beat 7 (0x1C): start_off1[17:0] | cell_format[21:18] |
                            --   chip_error_mask[25:22]
                            when 7 =>
                                v_hdr_data(17 downto 0)  := std_logic_vector(s_start_off1_r);
                                v_hdr_data(21 downto 18) := std_logic_vector(to_unsigned(c_CELL_FORMAT, 4));
                                v_hdr_data(25 downto 22) := s_chip_error_mask_r;

                            -- Beat 8 (0x20): k_dist_fixed [31:0]
                            when 8 =>
                                v_hdr_data := std_logic_vector(s_k_dist_fixed_r);

                            -- Beat 9 (0x24): timestamp_ns [31:0]
                            when 9 =>
                                v_hdr_data := std_logic_vector(s_timestamp_ns_r(31 downto 0));

                            -- Beat 10 (0x28): timestamp_ns [63:32]
                            when 10 =>
                                v_hdr_data := std_logic_vector(s_timestamp_ns_r(63 downto 32));

                            -- Beat 11 (0x2C): error_count [31:0]
                            when 11 =>
                                v_hdr_data := s_chip_error_cnt_r;

                            -- Beat 12+: padding (all zeros)
                            when others =>
                                v_hdr_data := (others => '0');

                        end case;

                        -- Load into output register
                        s_out_tdata_r  <= v_hdr_data;
                        s_out_tvalid_r <= '1';

                        -- SOF on first beat only
                        if s_beat_idx_r = 0 then
                            s_out_tuser_r <= '1';
                        else
                            s_out_tuser_r <= '0';
                        end if;

                        -- EOL on last beat of header line
                        if s_beat_idx_r = s_hsize_beats_last_r then
                            s_out_tlast_r <= '1';
                            s_col_cnt_r   <= (others => '0');
                            s_state_r     <= ST_DATA;
                        else
                            s_out_tlast_r <= '0';
                            s_beat_idx_r  <= s_beat_idx_r + 1;
                        end if;
                    end if;

                -- ==============================================================
                -- ST_DATA: passthrough data lines from face_assembler
                --   Count completed rows via upstream tlast (= EOL).
                --   col_cnt = cols_per_face -> frame complete -> ST_IDLE.
                -- ==============================================================
                when ST_DATA =>
                    if s_can_accept = '1' and i_s_axis_tvalid = '1' then
                        s_out_tdata_r  <= i_s_axis_tdata;
                        s_out_tvalid_r <= '1';
                        s_out_tlast_r  <= i_s_axis_tlast;
                        s_out_tuser_r  <= '0';

                        -- Row complete (upstream EOL)
                        if i_s_axis_tlast = '1' then
                            if s_col_cnt_r = s_cols_per_face_r - 1 then
                                -- All data lines sent: frame done
                                s_frame_done_r <= '1';
                                s_state_r      <= ST_IDLE;
                            else
                                s_col_cnt_r <= s_col_cnt_r + 1;
                            end if;
                        end if;
                    end if;

                end case;

                -- =============================================================
                -- face_start: latch fields and start header generation.
                -- Works from ANY state (ST_IDLE normal, others = abort/restart).
                -- Post-case override: last assignment wins.
                -- =============================================================
                if i_face_start = '1' then
                    -- Identity
                    s_vdma_frame_id_r     <= i_vdma_frame_id;
                    s_face_id_r           <= i_face_id;

                    -- Structure
                    s_active_chip_mask_r  <= i_cfg.active_chip_mask;
                    s_n_faces_r           <= i_cfg.n_faces;
                    s_cols_per_face_r     <= i_cfg.cols_per_face;
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

                    -- Line geometry (from top-level, shared with VDMA)
                    s_rows_per_face_r     <= i_rows_per_face;
                    s_hsize_beats_last_r <= resize(shift_right(i_hsize_bytes, 2) - 1, 14);  -- HSIZE/4 - 1

                    -- Start header generation
                    s_beat_idx_r   <= (others => '0');
                    s_out_tvalid_r <= '0';
                    s_out_tlast_r  <= '0';
                    s_out_tuser_r  <= '0';
                    s_state_r      <= ST_HEADER;
                end if;

            end if;
        end if;
    end process p_main;

end architecture rtl;
