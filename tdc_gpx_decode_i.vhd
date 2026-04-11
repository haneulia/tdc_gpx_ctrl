-- =============================================================================
-- tdc_gpx_decode_i.vhd
-- TDC-GPX Controller - Raw 28-bit Word Decoder (Pure Combinational)
-- =============================================================================
--
-- Purpose:
--   Extracts structured fields from TDC-GPX I-Mode 28-bit IFIFO raw word.
--   Pure combinational logic — no clock, no state.
--
--   Field mapping (I-Mode, SINGLE_SHOT):
--     [27:26] ChaCode   (2-bit, channel within IFIFO)
--     [25:18] StartNum  (8-bit, reserved/always 0 in SINGLE_SHOT)
--     [17]    Slope     (1-bit, edge direction)
--     [16:0]  Hit       (17-bit, Stop-Start time in BIN units)
--
--   stop_id_local reconstruction:
--     stop_id_local = {ififo_id, cha_code} = ififo_id * 4 + cha_code
--     IFIFO1 (Reg8): stops 0~3,  IFIFO2 (Reg9): stops 4~7
--
-- Standard: VHDL-93 compatible
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.tdc_gpx_pkg.all;

entity tdc_gpx_decode_i is
    generic (
        g_BUS_DATA_WIDTH : natural := c_TDC_BUS_WIDTH       -- 28
    );
    port (
        -- Input (from chip_ctrl)
        i_raw_word       : in  std_logic_vector(g_BUS_DATA_WIDTH - 1 downto 0);
        i_raw_word_valid : in  std_logic;
        i_ififo_id       : in  std_logic;

        -- Output (decoded fields, purely combinational)
        o_raw_hit        : out unsigned(c_RAW_HIT_WIDTH - 1 downto 0);  -- 17-bit
        o_slope          : out std_logic;
        o_cha_code_raw   : out unsigned(1 downto 0);
        o_stop_id_local  : out unsigned(2 downto 0);                     -- 0..7
        o_decoded_valid  : out std_logic
    );
end entity tdc_gpx_decode_i;

architecture rtl of tdc_gpx_decode_i is
begin

    -- Hit data: bits [16:0], 17-bit raw time measurement
    o_raw_hit <= unsigned(i_raw_word(c_RAW_HIT_HI downto c_RAW_HIT_LO));

    -- Slope: bit [17], edge direction
    o_slope <= i_raw_word(c_RAW_SLOPE_BIT);

    -- Channel code: bits [27:26], 2-bit channel within IFIFO
    o_cha_code_raw <= unsigned(i_raw_word(c_RAW_CHACODE_HI downto c_RAW_CHACODE_LO));

    -- Stop ID reconstruction: {ififo_id, cha_code[1:0]}
    -- IFIFO1 (id=0): stop 0~3,  IFIFO2 (id=1): stop 4~7
    o_stop_id_local <= unsigned(i_ififo_id
                                & i_raw_word(c_RAW_CHACODE_HI downto c_RAW_CHACODE_LO));

    -- Valid passthrough
    o_decoded_valid <= i_raw_word_valid;

end architecture rtl;
