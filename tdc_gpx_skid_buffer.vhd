-- =============================================================================
-- tdc_gpx_skid_buffer.vhd
-- AXI-Stream Skid Buffer (Registered Ready)
-- =============================================================================
--
-- Purpose:
--   2-deep pipeline buffer for AXI-Stream timing closure.
--   o_s_ready is FULLY REGISTERED — zero combinational depth on ready path.
--   Throughput: 1 beat/cycle (no throughput loss).
--   Latency: +1 cycle.
--
--   Implementation note:
--     This is a two-slot elastic buffer. The slave ready output is a
--     registered "space available next cycle" indication, so stale-ready
--     deassertion is absorbed by the second slot. Input data is accepted
--     only when the registered ready was high in that same handshake cycle.
--
--   Reference: matbi_skid_buffer.v (Matbi/Austin, 2022)
--              iammituraj/skid_buffer (GitHub)
--
-- Standard: VHDL-93 compatible
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;

entity tdc_gpx_skid_buffer is
    generic (
        g_DATA_WIDTH : natural := 8
    );
    port (
        i_clk       : in  std_logic;
        i_rst_n     : in  std_logic;
        i_flush     : in  std_logic;    -- synchronous flush: clear all state

        -- Slave interface (input)
        i_s_valid   : in  std_logic;
        o_s_ready   : out std_logic;    -- REGISTERED (0-depth combinational)
        i_s_data    : in  std_logic_vector(g_DATA_WIDTH - 1 downto 0);

        -- Master interface (output)
        o_m_valid   : out std_logic;
        i_m_ready   : in  std_logic;
        o_m_data    : out std_logic_vector(g_DATA_WIDTH - 1 downto 0)
    );
end entity tdc_gpx_skid_buffer;

architecture rtl of tdc_gpx_skid_buffer is

    signal s_data0_r   : std_logic_vector(g_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_data1_r   : std_logic_vector(g_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_count_r   : natural range 0 to 2 := 0;
    signal s_s_ready_r : std_logic := '1';

begin

    o_s_ready <= s_s_ready_r;
    o_m_data  <= s_data0_r;
    o_m_valid <= '1' when s_count_r /= 0 else '0';

    p_main : process(i_clk)
        variable v_data0 : std_logic_vector(g_DATA_WIDTH - 1 downto 0);
        variable v_data1 : std_logic_vector(g_DATA_WIDTH - 1 downto 0);
        variable v_count : natural range 0 to 2;
        variable v_push  : boolean;
        variable v_pop   : boolean;
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' or i_flush = '1' then
                s_data0_r   <= (others => '0');
                s_data1_r   <= (others => '0');
                s_count_r   <= 0;
                s_s_ready_r <= '1';
            else
                v_data0 := s_data0_r;
                v_data1 := s_data1_r;
                v_count := s_count_r;
                v_push  := (i_s_valid = '1' and s_s_ready_r = '1');
                v_pop   := (s_count_r /= 0 and i_m_ready = '1');

                if v_pop then
                    if v_count = 2 then
                        v_data0 := s_data1_r;
                    else
                        v_data0 := (others => '0');
                    end if;
                    v_data1 := (others => '0');
                    v_count := v_count - 1;
                end if;

                if v_push then
                    if v_count = 0 then
                        v_data0 := i_s_data;
                    else
                        v_data1 := i_s_data;
                    end if;
                    v_count := v_count + 1;
                end if;

                s_data0_r <= v_data0;
                s_data1_r <= v_data1;
                s_count_r <= v_count;
                if v_count = 2 then
                    s_s_ready_r <= '0';
                else
                    s_s_ready_r <= '1';
                end if;
            end if;
        end if;
    end process p_main;

end architecture rtl;
