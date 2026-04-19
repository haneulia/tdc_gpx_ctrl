-- =============================================================================
-- tdc_gpx_decoder_i_mode.vhd
-- TDC-GPX Controller - Raw 28-bit Word Decoder (Registered, AXI-Stream)
-- =============================================================================
--
-- Purpose:
--   Extracts structured fields from TDC-GPX I-Mode 28-bit IFIFO raw word.
--   Single-stage registered pipeline with AXI-Stream input and output.
--
--   Input AXI-Stream (from chip_ctrl):
--     tdata[27:0]  = 28-bit raw IFIFO word (0 for drain_done beat)
--     tdata[31:28] = 0 (reserved)
--     tuser[0]     = ififo_id ('0'=IFIFO1, '1'=IFIFO2)
--     tuser[5]     = faulted (drain_done beat only; '0' on data beats)
--     tuser[7]     = drain_done flag ('1' = control beat, no data)
--
--   Output AXI-Stream (to raw_event_builder):
--     tdata[16:0]  = raw_hit (17-bit, 0 for drain_done beat)
--     tdata[31:17] = 0 (reserved)
--     tuser[0]     = slope (edge direction)
--     tuser[2:1]   = cha_code_raw (2-bit channel within IFIFO)
--     tuser[5:3]   = stop_id_local (0..7, reconstructed from ififo_id + cha_code)  [data beats]
--     tuser[5]     = faulted (drain_done beat only; overlaps with stop_id_local,
--                   but the drain_done flag in tuser[7] disambiguates)
--     tuser[6]     = ififo_id (pass-through)
--     tuser[7]     = drain_done (pass-through from input)
--
--   Field mapping (I-Mode, SINGLE_SHOT):
--     [27:26] ChaCode   (2-bit, channel within IFIFO)
--     [25:18] StartNum  (8-bit, reserved/always 0 in SINGLE_SHOT)
--     [17]    Slope     (1-bit, edge direction)
--     [16:0]  Hit       (17-bit, Stop-Start time in BIN units)
--
-- Standard: VHDL-2008
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.tdc_gpx_pkg.all;

entity tdc_gpx_decoder_i_mode is
    generic (
        g_BUS_DATA_WIDTH : natural := c_TDC_BUS_WIDTH       -- 28
    );
    port (
        i_clk             : in  std_logic;
        i_rst_n           : in  std_logic;

        -- AXI-Stream slave (from chip_ctrl)
        i_s_axis_tvalid   : in  std_logic;
        i_s_axis_tdata    : in  std_logic_vector(31 downto 0);
        i_s_axis_tuser    : in  std_logic_vector(7 downto 0);
        o_s_axis_tready   : out std_logic;

        -- AXI-Stream master (to raw_event_builder)
        o_m_axis_tvalid   : out std_logic;
        o_m_axis_tdata    : out std_logic_vector(31 downto 0);
        o_m_axis_tuser    : out std_logic_vector(7 downto 0);
        i_m_axis_tready   : in  std_logic
    );
end entity tdc_gpx_decoder_i_mode;

architecture rtl of tdc_gpx_decoder_i_mode is

    signal s_tvalid_r : std_logic := '0';
    signal s_tdata_r  : std_logic_vector(31 downto 0) := (others => '0');
    signal s_tuser_r  : std_logic_vector(7 downto 0)  := (others => '0');

begin

    -- Always ready to accept (single-stage pipe, downstream is always-accept)
    o_s_axis_tready <= i_m_axis_tready or (not s_tvalid_r);

    p_decode : process(i_clk)
        variable v_raw      : std_logic_vector(g_BUS_DATA_WIDTH - 1 downto 0);
        variable v_ififo_id : std_logic;
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                s_tvalid_r <= '0';
                s_tdata_r  <= (others => '0');
                s_tuser_r  <= (others => '0');
            else
                -- Handshake: advance when output consumed or empty
                if s_tvalid_r = '0' or i_m_axis_tready = '1' then
                    if i_s_axis_tvalid = '1' then
                        if i_s_axis_tuser(7) = '1' then
                            -- drain_done control beat: pass through with ififo_id
                            s_tdata_r    <= (others => '0');
                            s_tuser_r    <= (others => '0');
                            s_tuser_r(7) <= '1';                  -- drain_done flag
                            s_tuser_r(6) <= i_s_axis_tuser(0);    -- ififo_id passthrough
                            -- Round 13 follow-up (audit 4번): carry the
                            -- faulted flag from chip_ctrl's tuser(5) through
                            -- unchanged. Overlaps with stop_id_local on data
                            -- beats, but drain_done (tuser[7]) disambiguates.
                            s_tuser_r(5) <= i_s_axis_tuser(5);
                        else
                            -- Normal data beat: decode raw word
                            v_raw      := i_s_axis_tdata(g_BUS_DATA_WIDTH - 1 downto 0);
                            v_ififo_id := i_s_axis_tuser(0);

                            s_tdata_r(c_RAW_HIT_WIDTH - 1 downto 0)  <= v_raw(c_RAW_HIT_HI downto c_RAW_HIT_LO);
                            s_tdata_r(31 downto c_RAW_HIT_WIDTH)      <= (others => '0');

                            s_tuser_r(0)          <= v_raw(c_RAW_SLOPE_BIT);
                            s_tuser_r(2 downto 1) <= v_raw(c_RAW_CHACODE_HI downto c_RAW_CHACODE_LO);
                            s_tuser_r(5 downto 3) <= v_ififo_id
                                                     & v_raw(c_RAW_CHACODE_HI downto c_RAW_CHACODE_LO);
                            s_tuser_r(6)          <= v_ififo_id;
                            s_tuser_r(7)          <= '0';
                        end if;
                        s_tvalid_r <= '1';
                    else
                        s_tvalid_r <= '0';
                    end if;
                end if;
            end if;
        end if;
    end process p_decode;

    o_m_axis_tvalid <= s_tvalid_r;
    o_m_axis_tdata  <= s_tdata_r;
    o_m_axis_tuser  <= s_tuser_r;

end architecture rtl;
