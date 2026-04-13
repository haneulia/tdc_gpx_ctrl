-- =============================================================================
-- tdc_gpx_pkg.vhd
-- TDC-GPX Controller - Record types, design constants, helper functions
-- =============================================================================
--
-- Purpose:
--   Defines record types for inter-module signal bundling:
--     t_tdc_cfg    : CSR configuration (CSR -> all submodules via TOP)
--     t_tdc_status : Status feedback   (submodules -> CSR via TOP)
--     t_raw_event  : Decoded IFIFO read result (decode -> cell_builder)
--     t_cell       : Dense hit storage for one stop/shot (cell_builder internal)
--
--   Design constants for TDC-GPX I-Mode (4-chip, 8-stop, SINGLE_SHOT).
--   Helper functions:
--     fn_ceil_div          : integer ceiling division (replaces (a+b-1)/b)
--     fn_cell_size_bytes   : cell size in bytes (ceil_pow2)
--     fn_cell_beat         : cell-to-AXI beat serialization MUX
--     fn_stop_evt_valid_bytes : stop event AXI-Stream tkeep width
--
--   Each record has a matching c_*_INIT reset constant.
--
-- Usage:
--   TOP instantiates signals of these record types and extracts individual
--   fields for submodule port maps. CSR outputs t_tdc_cfg, modules feed
--   back t_tdc_status.
--
-- Standard: VHDL-93 compatible
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package tdc_gpx_pkg is

    -- =========================================================================
    -- Utility: integer ceiling division  ceil(a / b)
    -- Declared first so that derived constants below can use it.
    -- =========================================================================
    function fn_ceil_div(a : natural; b : positive) return natural;

    -- =========================================================================
    -- Design constants (defaults — entity generics override where needed)
    -- =========================================================================
    constant c_N_CHIPS              : natural := 4;
    constant c_MAX_STOPS_PER_CHIP   : natural := 8;
    constant c_MAX_HITS_PER_STOP    : natural := 7;
    constant c_HIT_SLOT_DATA_WIDTH  : natural := 16;   -- Zynq-7000 (17 for MPSoC)
    constant c_TDATA_WIDTH          : natural := 32;
    constant c_TDATA_BYTES          : natural := c_TDATA_WIDTH / 8;   -- bytes per beat
    constant c_CELL_FORMAT          : natural := 0;     -- Phase 1: Zynq-7000

    -- Derived cell layout constants (auto-calculated from MAX_HITS / TDATA_WIDTH)
    constant c_SLOTS_PER_BEAT       : natural := c_TDATA_WIDTH / c_HIT_SLOT_DATA_WIDTH;  -- 2
    constant c_HIT_DATA_BEATS       : natural := fn_ceil_div(c_MAX_HITS_PER_STOP,
                                                              c_SLOTS_PER_BEAT);          -- 4
    constant c_META_BEAT_IDX        : natural := c_HIT_DATA_BEATS;                        -- 4

    -- Stop event AXI-Stream constants
    constant c_STOP_EVT_DATA_WIDTH  : natural := 32;    -- tdata/tuser width
    constant c_STOP_CNT_WIDTH       : natural := 4;     -- FIXED: p_stop_decode hardcodes 4-bit
    -- Layout: per-chip packed [chip3(8b)|chip2(8b)|chip1(8b)|chip0(8b)]
    --   Each 8-bit chip slice: [IFIFO2(4b) | IFIFO1(4b)]
    -- tkeep: reserved (always "1111", not consumed by p_stop_decode)

    constant c_SHOT_SEQ_WIDTH       : natural := 16;
    constant c_TDC_BUS_WIDTH        : natural := 28;
    constant c_RAW_HIT_WIDTH        : natural := 17;   -- TDC-GPX I-Mode raw hit (always 17-bit)
    constant c_MAX_ROWS_PER_FACE    : natural := c_N_CHIPS * c_MAX_STOPS_PER_CHIP;  -- 32

    -- =========================================================================
    -- TDC-GPX register addresses (ADR[3:0])
    -- =========================================================================
    constant c_TDC_REG0             : std_logic_vector(3 downto 0) := x"0";
    constant c_TDC_REG1             : std_logic_vector(3 downto 0) := x"1";
    constant c_TDC_REG2             : std_logic_vector(3 downto 0) := x"2";
    constant c_TDC_REG3             : std_logic_vector(3 downto 0) := x"3";
    constant c_TDC_REG4             : std_logic_vector(3 downto 0) := x"4";
    constant c_TDC_REG5             : std_logic_vector(3 downto 0) := x"5";
    constant c_TDC_REG6             : std_logic_vector(3 downto 0) := x"6";
    constant c_TDC_REG7             : std_logic_vector(3 downto 0) := x"7";
    constant c_TDC_REG8_IFIFO1      : std_logic_vector(3 downto 0) := x"8";
    constant c_TDC_REG9_IFIFO2      : std_logic_vector(3 downto 0) := x"9";
    constant c_TDC_REG10            : std_logic_vector(3 downto 0) := x"A";
    constant c_TDC_REG11            : std_logic_vector(3 downto 0) := x"B";
    constant c_TDC_REG12            : std_logic_vector(3 downto 0) := x"C";
    constant c_TDC_REG14            : std_logic_vector(3 downto 0) := x"E";

    -- =========================================================================
    -- TDC-GPX 28-bit raw word field positions
    -- =========================================================================
    constant c_RAW_CHACODE_HI       : natural := 27;
    constant c_RAW_CHACODE_LO       : natural := 26;
    constant c_RAW_STARTNUM_HI      : natural := 25;   -- reserved (always 0 in SINGLE_SHOT)
    constant c_RAW_STARTNUM_LO      : natural := 18;
    constant c_RAW_SLOPE_BIT        : natural := 17;
    constant c_RAW_HIT_HI           : natural := 16;
    constant c_RAW_HIT_LO           : natural := 0;

    -- =========================================================================
    -- Cell size calculation
    -- =========================================================================
    function fn_cell_payload_bits(
        hit_slot_width : natural;
        max_hits       : natural
    ) return natural;

    function fn_ceil_pow2(x : natural) return natural;

    function fn_cell_size_bytes(
        hit_slot_width : natural;
        max_hits       : natural
    ) return natural;

    function fn_count_ones(v : std_logic_vector) return natural;

    -- Stop event AXI-Stream helpers
    -- valid_bytes = ceil(n_stops * cnt_width / 8)
    function fn_stop_evt_valid_bytes(
        n_stops   : natural;
        cnt_width : natural
    ) return natural;

    -- Generate tkeep mask: lower valid_bytes bits = '1', rest = '0'
    function fn_stop_evt_tkeep(
        n_stops    : natural;
        cnt_width  : natural;
        tkeep_width : natural
    ) return std_logic_vector;

    -- Cell size and derived constants: auto-calculated from MAX_HITS
    -- (deferred constants — full definitions in package body using fn_cell_size_bytes)
    constant c_CELL_SIZE_BYTES      : natural;
    constant c_BEATS_PER_CELL       : natural;
    constant c_DATA_BEATS_MAX       : natural;
    constant c_HSIZE_BEATS_MAX      : natural;
    constant c_HSIZE_MAX            : natural;

    -- Header prefix (embedded in each VDMA line)
    constant c_HDR_PREFIX_BEATS     : natural := 12;    -- 12 beats x 4B = 48 bytes
    constant c_HDR_PREFIX_BYTES     : natural := c_HDR_PREFIX_BEATS * (c_TDATA_WIDTH / 8);  -- 48

    -- =========================================================================
    -- AXI-Stream array type (for multi-chip slice data)
    -- =========================================================================
    type t_axis_tdata_array is array(0 to c_N_CHIPS - 1)
        of std_logic_vector(c_TDATA_WIDTH - 1 downto 0);

    -- =========================================================================
    -- TDC-GPX physical bus array types (for top-level port maps)
    -- =========================================================================
    type t_tdc_bus_array is array(0 to c_N_CHIPS - 1)
        of std_logic_vector(c_TDC_BUS_WIDTH - 1 downto 0);
    type t_tdc_adr_array is array(0 to c_N_CHIPS - 1)
        of std_logic_vector(3 downto 0);

    -- =========================================================================
    -- cfg_image array type (TDC-GPX register image stored in CSR)
    -- =========================================================================
    type t_cfg_image is array(0 to 15) of std_logic_vector(31 downto 0);

    -- =========================================================================
    -- t_tdc_cfg : CSR configuration (CSR -> submodules)
    -- Fields latched at appropriate boundaries (face_start / shot_start /
    -- transaction entry) and stable during active processing.
    --
    -- Register layout (compact):
    --   CTL0  MAIN_CTRL  : packed control fields + COMMAND[31:28]
    --   CTL1  BUS_TIMING : bus_clk_div[5:0] + bus_ticks[8:6]
    --   CTL2  RANGE_COLS : max_range_clks[15:0] + cols_per_face[31:16]
    --   CTL3  START_OFF1 : [17:0]
    --   CTL4  CFG_REG7   : [31:0]
    -- =========================================================================
    type t_tdc_cfg is record
        -- CTL0: MAIN_CTRL packed fields
        active_chip_mask    : std_logic_vector(c_N_CHIPS - 1 downto 0);     -- CTL0[3:0]
        packet_scope        : std_logic;                                    -- CTL0[4]    HEADER-ONLY
        hit_store_mode      : unsigned(1 downto 0);                         -- CTL0[6:5]  HEADER-ONLY
        dist_scale          : unsigned(2 downto 0);                         -- CTL0[9:7]  HEADER-ONLY
        drain_mode          : std_logic;                                    -- CTL0[10]
        pipeline_en         : std_logic;                                    -- CTL0[11]   HEADER-ONLY
        n_faces             : unsigned(2 downto 0);                         -- CTL0[14:12]
        stops_per_chip      : unsigned(3 downto 0);                         -- CTL0[18:15]
        n_drain_cap         : unsigned(3 downto 0);                         -- CTL0[22:19]
        stopdis_override    : std_logic_vector(4 downto 0);                 -- CTL0[27:23]
        -- CTL1: BUS_TIMING
        bus_clk_div         : unsigned(5 downto 0);                         -- CTL1[5:0]
        bus_ticks           : unsigned(2 downto 0);                         -- CTL1[8:6]
        -- CTL2: RANGE_COLS
        max_range_clks      : unsigned(15 downto 0);                        -- CTL2[15:0]
        cols_per_face       : unsigned(15 downto 0);                        -- CTL2[31:16]
        -- CTL3: START_OFF1
        start_off1          : unsigned(17 downto 0);                        -- CTL3[17:0]
        -- CTL4: CFG_REG7
        cfg_reg7            : std_logic_vector(31 downto 0);                -- CTL4[31:0]
        -- CTL21: SCAN_TIMEOUT
        max_scan_clks       : unsigned(15 downto 0);                        -- CTL21[15:0]
    end record;

    constant c_TDC_CFG_INIT : t_tdc_cfg := (
        active_chip_mask    => (others => '1'),         -- all 4 chips active
        packet_scope        => '0',                     -- face scope
        hit_store_mode      => "00",                    -- RAW
        dist_scale          => "000",                   -- 1mm
        drain_mode          => '0',                     -- SourceGating
        pipeline_en         => '0',                     -- sequential
        n_faces             => to_unsigned(5, 3),
        stops_per_chip      => to_unsigned(8, 4),
        n_drain_cap         => (others => '0'),         -- unlimited
        stopdis_override    => (others => '0'),
        bus_clk_div         => to_unsigned(1, 6),
        bus_ticks           => to_unsigned(5, 3),
        max_range_clks      => to_unsigned(267, 16),    -- ~200m @200MHz
        cols_per_face       => to_unsigned(2400, 16),
        start_off1          => (others => '0'),
        cfg_reg7            => (others => '0'),
        max_scan_clks       => to_unsigned(0, 16)           -- 0 = disabled (no timeout)
    );

    -- =========================================================================
    -- t_tdc_status : Status feedback (submodules -> CSR)
    -- =========================================================================
    type t_tdc_status is record
        busy                : std_logic;
        pipeline_overrun    : std_logic;
        bin_mismatch        : std_logic;
        chip_error_mask     : std_logic_vector(c_N_CHIPS - 1 downto 0);
        drain_timeout_mask  : std_logic_vector(c_N_CHIPS - 1 downto 0);  -- per-chip drain timeout
        sequence_error_mask : std_logic_vector(c_N_CHIPS - 1 downto 0);  -- per-chip sequence error
        shot_seq_current    : unsigned(c_SHOT_SEQ_WIDTH - 1 downto 0);
        vdma_frame_count    : unsigned(31 downto 0);
        error_count         : unsigned(31 downto 0);
        shot_drop_count     : unsigned(15 downto 0);  -- deferred-shot overflow drops
        frame_abort_count   : unsigned(15 downto 0);  -- frames discarded by abort
    end record;

    constant c_TDC_STATUS_INIT : t_tdc_status := (
        busy                => '0',
        pipeline_overrun    => '0',
        bin_mismatch        => '0',
        chip_error_mask     => (others => '0'),
        drain_timeout_mask  => (others => '0'),
        sequence_error_mask => (others => '0'),
        shot_seq_current    => (others => '0'),
        vdma_frame_count    => (others => '0'),
        error_count         => (others => '0'),
        shot_drop_count     => (others => '0'),
        frame_abort_count   => (others => '0')
    );

    -- =========================================================================
    -- t_raw_event : Decoded IFIFO read (decode_i -> raw_event_builder -> cell)
    -- Key = {chip_id, shot_seq, stop_id_local}
    -- =========================================================================
    type t_raw_event is record
        valid               : std_logic;
        chip_id             : unsigned(1 downto 0);         -- 0..3
        ififo_id            : std_logic;                    -- '0'=IFIFO1, '1'=IFIFO2
        stop_id_local       : unsigned(2 downto 0);         -- 0..7
        slope               : std_logic;
        raw_hit             : unsigned(c_RAW_HIT_WIDTH - 1 downto 0);       -- 17-bit (truncation at cell_builder)
        shot_seq            : unsigned(c_SHOT_SEQ_WIDTH - 1 downto 0);
        hit_seq_local       : unsigned(3 downto 0);         -- 0..15 (>=8 means overflow)
    end record;

    constant c_RAW_EVENT_INIT : t_raw_event := (
        valid               => '0',
        chip_id             => (others => '0'),
        ififo_id            => '0',
        stop_id_local       => (others => '0'),
        slope               => '0',
        raw_hit             => (others => '0'),
        shot_seq            => (others => '0'),
        hit_seq_local       => (others => '0')
    );

    -- =========================================================================
    -- t_cell : Dense hit storage for one stop channel, one shot
    -- Cell byte layout: hit_slot[0..MAX-1] + hit_valid + slope_vec + meta
    -- =========================================================================
    type t_hit_slot_array is array (0 to c_MAX_HITS_PER_STOP - 1)
        of unsigned(c_HIT_SLOT_DATA_WIDTH - 1 downto 0);

    type t_cell is record
        hit_slot            : t_hit_slot_array;
        hit_valid           : std_logic_vector(c_MAX_HITS_PER_STOP - 1 downto 0);
        slope_vec           : std_logic_vector(c_MAX_HITS_PER_STOP - 1 downto 0);
        hit_count_actual    : unsigned(3 downto 0);         -- 0..c_MAX_HITS_PER_STOP
        hit_dropped         : std_logic;
        error_fill          : std_logic;
    end record;

    constant c_CELL_INIT : t_cell := (
        hit_slot            => (others => (others => '0')),
        hit_valid           => (others => '0'),
        slope_vec           => (others => '0'),
        hit_count_actual    => (others => '0'),
        hit_dropped         => '0',
        error_fill          => '0'
    );

    -- =========================================================================
    -- HIT_STORE_MODE constants
    -- =========================================================================
    constant c_STORE_RAW            : unsigned(1 downto 0) := "00";
    constant c_STORE_CORRECTED      : unsigned(1 downto 0) := "01";
    constant c_STORE_DISTANCE       : unsigned(1 downto 0) := "10";

    -- =========================================================================
    -- Header magic word: b"TDCG" (byte signature)
    -- =========================================================================
    constant c_HEADER_MAGIC         : std_logic_vector(31 downto 0) := x"47434454";
    -- LE interpretation: Byte0=0x54('T'), Byte1=0x44('D'), Byte2=0x43('C'), Byte3=0x47('G')

end package tdc_gpx_pkg;

-- =============================================================================
-- Package body: function implementations
-- =============================================================================

package body tdc_gpx_pkg is

    -- =========================================================================
    -- Deferred constants: cell size and derived VDMA line metrics
    -- =========================================================================
    constant c_CELL_SIZE_BYTES : natural := -- 32
        fn_cell_size_bytes(c_HIT_SLOT_DATA_WIDTH, c_MAX_HITS_PER_STOP);
    constant c_BEATS_PER_CELL  : natural := c_CELL_SIZE_BYTES / c_TDATA_BYTES;  -- 32/ 4 = 8
    constant c_DATA_BEATS_MAX  : natural := c_MAX_ROWS_PER_FACE * c_BEATS_PER_CELL;
    constant c_HSIZE_BEATS_MAX : natural := c_HDR_PREFIX_BEATS + c_DATA_BEATS_MAX;
    constant c_HSIZE_MAX       : natural := c_HSIZE_BEATS_MAX * (c_TDATA_WIDTH / 8);

    -- Cell payload bits (Format 0, Zynq-7000)
    -- hit_slot[MAX] + hit_valid + slope_vec + hit_count(4) + dropped(1) + error_fill(1)
    function fn_cell_payload_bits(
        hit_slot_width : natural;
        max_hits       : natural
    ) return natural is
    begin
        return hit_slot_width * max_hits    -- hit_slot
             + max_hits                     -- hit_valid
             + max_hits                     -- slope_vec
             + 4 + 1 + 1;                   -- hit_count_actual + hit_dropped + error_fill
    end function;

    -- Integer ceiling division: ceil(a / b)
    function fn_ceil_div(a : natural; b : positive) return natural is
    begin
        return (a + b - 1) / b;
    end function;

    -- Round up to next power of 2
    function fn_ceil_pow2(x : natural) return natural is
        variable result : natural := 1;
    begin
        while result < x loop
            result := result * 2;
        end loop;
        return result;
    end function;

    -- Cell size in bytes (aligned to power of 2)
    function fn_cell_size_bytes(
        hit_slot_width : natural;
        max_hits       : natural
    ) return natural is
        variable payload_bits : natural;
        variable raw_bytes    : natural;
    begin
        payload_bits := fn_cell_payload_bits(hit_slot_width, max_hits);
        raw_bytes    := fn_ceil_div(payload_bits, 8);
        return fn_ceil_pow2(raw_bytes);
    end function;

    -- Count '1' bits in a std_logic_vector
    function fn_count_ones(v : std_logic_vector) return natural is
        variable cnt : natural := 0;
    begin
        for i in v'range loop
            if v(i) = '1' then
                cnt := cnt + 1;
            end if;
        end loop;
        return cnt;
    end function;

    -- ceil(n_stops * cnt_width / 8)
    function fn_stop_evt_valid_bytes(
        n_stops   : natural;
        cnt_width : natural
    ) return natural is
    begin
        return fn_ceil_div(n_stops * cnt_width, 8);
    end function;

    -- Generate tkeep mask: lower valid_bytes bits set
    function fn_stop_evt_tkeep(
        n_stops    : natural;
        cnt_width  : natural;
        tkeep_width : natural
    ) return std_logic_vector is
        variable v_bytes : natural;
        variable mask    : std_logic_vector(tkeep_width - 1 downto 0)
                            := (others => '0');
    begin
        v_bytes := fn_stop_evt_valid_bytes(n_stops, cnt_width);
        for i in 0 to tkeep_width - 1 loop
            if i < v_bytes then
                mask(i) := '1';
            end if;
        end loop;
        return mask;
    end function;

end package body tdc_gpx_pkg;
