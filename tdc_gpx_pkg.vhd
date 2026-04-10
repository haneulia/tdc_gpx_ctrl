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
--   Helper functions for cell size calculation and bit manipulation.
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
    -- Design constants (defaults — entity generics override where needed)
    -- =========================================================================
    constant c_N_CHIPS              : natural := 4;
    constant c_MAX_STOPS_PER_CHIP   : natural := 8;
    constant c_MAX_HITS_PER_STOP    : natural := 8;
    constant c_HIT_SLOT_DATA_WIDTH  : natural := 16;   -- Zynq-7000 (17 for MPSoC)
    constant c_TDATA_WIDTH          : natural := 32;
    constant c_TDATA_BYTES          : natural := c_TDATA_WIDTH / 8;   -- bytes per beat
    constant c_CELL_FORMAT          : natural := 0;     -- Phase 1: Zynq-7000

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

    -- Default cell_size_bytes (for c_HIT_SLOT_DATA_WIDTH=17, c_MAX_HITS=8)
    -- 158 bits -> 20 bytes -> align_pow2 = 32 bytes
    constant c_CELL_SIZE_BYTES      : natural := 32;

    -- VDMA line constants (default generics)
    constant c_HSIZE_MAX            : natural := c_MAX_ROWS_PER_FACE * c_CELL_SIZE_BYTES;   -- 1024
    constant c_BEATS_PER_LINE_MAX   : natural := c_HSIZE_MAX / (c_TDATA_BYTES);             -- 256
    -- "Beat" = one AXI-Stream transfer (tvalid & tready handshake), TDATA_WIDTH bits wide
    constant c_BEATS_PER_CELL       : natural := c_CELL_SIZE_BYTES / (c_TDATA_BYTES);       -- 8

    -- =========================================================================
    -- AXI-Stream array type (for multi-lane slice data)
    -- =========================================================================
    type t_axis_tdata_array is array(0 to c_N_CHIPS - 1)
        of std_logic_vector(c_TDATA_WIDTH - 1 downto 0);

    -- =========================================================================
    -- cfg_image array type (TDC-GPX register image stored in CSR)
    -- =========================================================================
    type t_cfg_image is array(0 to 15) of std_logic_vector(31 downto 0);

    -- =========================================================================
    -- t_tdc_cfg : CSR configuration (CSR -> submodules)
    -- All fields latched at packet_start and stable during frame.
    -- =========================================================================
    type t_tdc_cfg is record
        -- Control registers (0x00~0x34)
        active_chip_mask    : std_logic_vector(c_N_CHIPS - 1 downto 0);     -- 0x00 [3:0]
        stops_per_chip      : unsigned(3 downto 0);                         -- 0x04 [3:0]
        cols_per_face       : unsigned(15 downto 0);                        -- 0x08 [15:0]
        packet_scope        : std_logic;                                    -- 0x0C [0]
        hit_store_mode      : unsigned(1 downto 0);                         -- 0x10 [1:0]
        dist_scale          : unsigned(2 downto 0);                         -- 0x14 [2:0]
        drain_mode          : std_logic;                                    -- 0x18 [0]
        n_drain_cap         : unsigned(7 downto 0);                         -- 0x1C [7:0]
        pipeline_en         : std_logic;                                    -- 0x20 [0]
        n_faces             : unsigned(7 downto 0);                         -- 0x24 [7:0]
        bus_clk_div         : unsigned(7 downto 0);                         -- 0x28 [7:0]
        bus_ticks           : unsigned(2 downto 0);                         -- 0x2C [2:0]
        stopdis_override    : std_logic_vector(4 downto 0);                 -- 0x30 [4:0]
        max_range_clks      : unsigned(15 downto 0);                        -- 0x34 [15:0]
        -- TDC settings (0x40~0x44)
        start_off1          : unsigned(17 downto 0);                        -- 0x40 [17:0]
        cfg_reg7            : std_logic_vector(31 downto 0);                -- 0x44
    end record;

    constant c_TDC_CFG_INIT : t_tdc_cfg := (
        active_chip_mask    => (others => '1'),         -- all 4 chips active
        stops_per_chip      => to_unsigned(8, 4),
        cols_per_face       => to_unsigned(2400, 16),
        packet_scope        => '0',                     -- face 단위
        hit_store_mode      => "00",                    -- RAW
        dist_scale          => "000",                   -- 1mm
        drain_mode          => '0',                     -- SourceGating
        n_drain_cap         => (others => '0'),         -- unlimited
        pipeline_en         => '0',                     -- sequential
        n_faces             => to_unsigned(5, 8),
        bus_clk_div         => to_unsigned(1, 8),
        bus_ticks           => to_unsigned(5, 3),
        stopdis_override    => (others => '0'),
        max_range_clks     => to_unsigned(267, 16),       -- ~200m @200MHz
        start_off1          => (others => '0'),
        cfg_reg7            => (others => '0')
    );

    -- =========================================================================
    -- t_tdc_status : Status feedback (submodules -> CSR)
    -- =========================================================================
    type t_tdc_status is record
        busy                : std_logic;
        pipeline_overrun    : std_logic;
        bin_mismatch        : std_logic;
        lane_error_mask     : std_logic_vector(c_N_CHIPS - 1 downto 0);
        shot_seq_current    : unsigned(c_SHOT_SEQ_WIDTH - 1 downto 0);
        vdma_frame_count    : unsigned(31 downto 0);
        error_count         : unsigned(31 downto 0);
    end record;

    constant c_TDC_STATUS_INIT : t_tdc_status := (
        busy                => '0',
        pipeline_overrun    => '0',
        bin_mismatch        => '0',
        lane_error_mask     => (others => '0'),
        shot_seq_current    => (others => '0'),
        vdma_frame_count    => (others => '0'),
        error_count         => (others => '0')
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
    -- Cell byte layout: hit_slot[0..7] + hit_valid + slope_vec + meta
    -- =========================================================================
    type t_hit_slot_array is array (0 to c_MAX_HITS_PER_STOP - 1)
        of unsigned(c_HIT_SLOT_DATA_WIDTH - 1 downto 0);

    type t_cell is record
        hit_slot            : t_hit_slot_array;
        hit_valid           : std_logic_vector(c_MAX_HITS_PER_STOP - 1 downto 0);
        slope_vec           : std_logic_vector(c_MAX_HITS_PER_STOP - 1 downto 0);
        hit_count_actual    : unsigned(3 downto 0);         -- 0..8
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
             + 4 + 1 + 1;                  -- hit_count_actual + hit_dropped + error_fill
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
        raw_bytes    := (payload_bits + 7) / 8;
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

end package body tdc_gpx_pkg;
