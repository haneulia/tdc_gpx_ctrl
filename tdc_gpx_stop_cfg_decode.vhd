-- =============================================================================
-- tdc_gpx_stop_cfg_decode.vhd
-- TDC-GPX Controller - Stop Event Decoder
-- =============================================================================
--
-- Purpose:
--   Decodes per-chip IFIFO expected drain counts from echo_receiver.
--   Also handles cfg_image register override (StartOff1, MasterAluTrig, Reg7).
--
-- Standard: VHDL-2008
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.tdc_gpx_pkg.all;
use work.tdc_gpx_cfg_pkg.all;

entity tdc_gpx_stop_cfg_decode is
    generic (
        g_STOP_EVT_DWIDTH : natural := c_STOP_EVT_DATA_WIDTH
    );
    port (
        i_clk             : in  std_logic;
        i_rst_n           : in  std_logic;

        -- Stop event AXI-Stream slave
        i_stop_evt_tvalid : in  std_logic;
        i_stop_evt_tdata  : in  std_logic_vector(g_STOP_EVT_DWIDTH - 1 downto 0);
        i_stop_evt_tuser  : in  std_logic_vector(g_STOP_EVT_DWIDTH - 1 downto 0);
        o_stop_evt_tready : out std_logic;

        -- Shot boundary clear
        i_shot_start_gated : in  std_logic;

        -- Per-chip expected IFIFO counts
        o_expected_ififo1 : out t_expected_array;
        o_expected_ififo2 : out t_expected_array;

        -- Config image override
        i_cfg             : in  t_tdc_cfg;
        i_cfg_image_raw   : in  t_cfg_image;
        o_cfg_image       : out t_cfg_image
    );
end entity tdc_gpx_stop_cfg_decode;

architecture rtl of tdc_gpx_stop_cfg_decode is
begin

    o_stop_evt_tready <= '1';

    p_stop_decode : process(i_clk)
        variable v_lo : natural;
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                for i in 0 to c_N_CHIPS - 1 loop
                    o_expected_ififo1(i) <= (others => '0');
                    o_expected_ififo2(i) <= (others => '0');
                end loop;
            elsif i_shot_start_gated = '1' then
                for i in 0 to c_N_CHIPS - 1 loop
                    o_expected_ififo1(i) <= (others => '0');
                    o_expected_ififo2(i) <= (others => '0');
                end loop;
            elsif i_stop_evt_tvalid = '1' then
                for i in 0 to c_N_CHIPS - 1 loop
                    v_lo := i * 8;
                    o_expected_ififo1(i) <=
                        resize(unsigned(i_stop_evt_tdata(v_lo + 3 downto v_lo)), 8)
                      + resize(unsigned(i_stop_evt_tuser(v_lo + 3 downto v_lo)), 8);
                    o_expected_ififo2(i) <=
                        resize(unsigned(i_stop_evt_tdata(v_lo + 7 downto v_lo + 4)), 8)
                      + resize(unsigned(i_stop_evt_tuser(v_lo + 7 downto v_lo + 4)), 8);
                end loop;
            end if;
        end if;
    end process;

    -- cfg_image override: force specific register bits
    p_cfg_override : process(i_cfg_image_raw, i_cfg)
        variable v_img : t_cfg_image;
    begin
        v_img := i_cfg_image_raw;
        v_img(5)(c_REG5_STARTOFF1_HI downto c_REG5_STARTOFF1_LO)
            := std_logic_vector(i_cfg.start_off1);
        v_img(5)(c_REG5_MASTER_ALU_TRIG)  := '1';
        v_img(5)(c_REG5_PARTIAL_ALU_TRIG) := '0';
        v_img(7) := i_cfg.cfg_reg7;
        o_cfg_image <= v_img;
    end process;

end architecture rtl;
