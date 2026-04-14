-- =============================================================================
-- tdc_gpx_sync_fifo.vhd
-- Synchronous FIFO with optional input/output skid buffers
-- =============================================================================
--
-- Purpose:
--   Parameterizable synchronous FIFO with AXI-Stream handshake (valid/ready).
--   Circular buffer with read/write pointers and round-bit for full/empty
--   detection.  Optional skid buffers on input and/or output for timing
--   closure (registered ready paths).
--
-- Parameters:
--   g_DATA_WIDTH  : data width in bits (default 32)
--   g_DEPTH       : FIFO depth, must be a power of 2 (default 4)
--   g_LOG2_DEPTH  : log2(g_DEPTH) (default 2)
--   g_IN_REG      : true = input skid buffer (default true)
--   g_OUT_REG     : true = output skid buffer (default true)
--
-- Reference: matbi_sync_fifo.v (Matbi/Austin, 2022)
--
-- Standard: VHDL-2008
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity tdc_gpx_sync_fifo is
    generic (
        g_DATA_WIDTH : natural  := 32;
        g_DEPTH      : natural  := 4;
        g_LOG2_DEPTH : natural  := 2;
        g_IN_REG     : boolean  := true;
        g_OUT_REG    : boolean  := true
    );
    port (
        i_clk     : in  std_logic;
        i_rst_n   : in  std_logic;
        i_flush   : in  std_logic := '0';  -- synchronous flush: empty FIFO

        -- Slave interface (input)
        i_s_valid : in  std_logic;
        o_s_ready : out std_logic;
        i_s_data  : in  std_logic_vector(g_DATA_WIDTH - 1 downto 0);

        -- Master interface (output)
        o_m_valid : out std_logic;
        i_m_ready : in  std_logic;
        o_m_data  : out std_logic_vector(g_DATA_WIDTH - 1 downto 0)
    );
end entity tdc_gpx_sync_fifo;

architecture rtl of tdc_gpx_sync_fifo is

    -- =========================================================================
    -- Internal wires between skid buffers and FIFO core
    -- =========================================================================
    signal s_core_s_valid : std_logic;
    signal s_core_s_ready : std_logic;
    signal s_core_s_data  : std_logic_vector(g_DATA_WIDTH - 1 downto 0);

    signal s_core_m_valid : std_logic;
    signal s_core_m_ready : std_logic;
    signal s_core_m_data  : std_logic_vector(g_DATA_WIDTH - 1 downto 0);

    -- =========================================================================
    -- FIFO core signals
    -- =========================================================================
    type t_mem is array (0 to g_DEPTH - 1)
        of std_logic_vector(g_DATA_WIDTH - 1 downto 0);
    signal s_mem : t_mem := (others => (others => '0'));

    signal s_wptr_r       : unsigned(g_LOG2_DEPTH - 1 downto 0) := (others => '0');
    signal s_wptr_round_r : std_logic := '0';
    signal s_rptr_r       : unsigned(g_LOG2_DEPTH - 1 downto 0) := (others => '0');
    signal s_rptr_round_r : std_logic := '0';

    signal s_empty : std_logic;
    signal s_full  : std_logic;
    signal s_i_hs  : std_logic;  -- input handshake
    signal s_o_hs  : std_logic;  -- output handshake

begin

    -- =========================================================================
    -- Input skid buffer (optional)
    -- =========================================================================
    gen_in_reg : if g_IN_REG generate
        u_skid_in : entity work.tdc_gpx_skid_buffer
            generic map (g_DATA_WIDTH => g_DATA_WIDTH)
            port map (
                i_clk     => i_clk,
                i_rst_n   => i_rst_n,
                i_flush   => i_flush,
                i_s_valid => i_s_valid,
                o_s_ready => o_s_ready,
                i_s_data  => i_s_data,
                o_m_valid => s_core_s_valid,
                i_m_ready => s_core_s_ready,
                o_m_data  => s_core_s_data
            );
    end generate gen_in_reg;

    gen_no_in_reg : if not g_IN_REG generate
        s_core_s_valid <= i_s_valid;
        o_s_ready      <= s_core_s_ready;
        s_core_s_data  <= i_s_data;
    end generate gen_no_in_reg;

    -- =========================================================================
    -- Output skid buffer (optional)
    -- =========================================================================
    gen_out_reg : if g_OUT_REG generate
        u_skid_out : entity work.tdc_gpx_skid_buffer
            generic map (g_DATA_WIDTH => g_DATA_WIDTH)
            port map (
                i_clk     => i_clk,
                i_rst_n   => i_rst_n,
                i_flush   => i_flush,
                i_s_valid => s_core_m_valid,
                o_s_ready => s_core_m_ready,
                i_s_data  => s_core_m_data,
                o_m_valid => o_m_valid,
                i_m_ready => i_m_ready,
                o_m_data  => o_m_data
            );
    end generate gen_out_reg;

    gen_no_out_reg : if not g_OUT_REG generate
        o_m_valid      <= s_core_m_valid;
        s_core_m_ready <= i_m_ready;
        o_m_data       <= s_core_m_data;
    end generate gen_no_out_reg;

    -- =========================================================================
    -- FIFO core: circular buffer with round-bit full/empty detection
    -- =========================================================================

    -- Empty: pointers AND round bits match
    s_empty <= '1' when s_wptr_round_r = s_rptr_round_r
                        and s_wptr_r = s_rptr_r
               else '0';

    -- Full: pointers match but round bits differ
    s_full  <= '1' when s_wptr_round_r /= s_rptr_round_r
                        and s_wptr_r = s_rptr_r
               else '0';

    -- Handshake signals
    s_core_s_ready <= not s_full;
    s_core_m_valid <= not s_empty;
    s_i_hs         <= s_core_s_valid and s_core_s_ready;
    s_o_hs         <= s_core_m_valid and s_core_m_ready;

    -- Read port (combinational)
    s_core_m_data <= s_mem(to_integer(s_rptr_r));

    -- Write process
    p_write : process(i_clk)
        variable v_wptr_next  : unsigned(g_LOG2_DEPTH - 1 downto 0);
        variable v_round_next : std_logic;
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' or i_flush = '1' then
                s_wptr_r       <= (others => '0');
                s_wptr_round_r <= '0';
                -- Note: memory contents not cleared on flush (only pointers).
                -- This avoids the g_DEPTH-wide reset overhead.
            elsif s_i_hs = '1' then
                s_mem(to_integer(s_wptr_r)) <= s_core_s_data;

                -- Advance write pointer
                if s_wptr_r = to_unsigned(g_DEPTH - 1, g_LOG2_DEPTH) then
                    v_wptr_next  := (others => '0');
                    v_round_next := not s_wptr_round_r;
                else
                    v_wptr_next  := s_wptr_r + 1;
                    v_round_next := s_wptr_round_r;
                end if;
                s_wptr_r       <= v_wptr_next;
                s_wptr_round_r <= v_round_next;
            end if;
        end if;
    end process p_write;

    -- Read process
    p_read : process(i_clk)
        variable v_rptr_next  : unsigned(g_LOG2_DEPTH - 1 downto 0);
        variable v_round_next : std_logic;
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' or i_flush = '1' then
                s_rptr_r       <= (others => '0');
                s_rptr_round_r <= '0';
            elsif s_o_hs = '1' then
                -- Advance read pointer
                if s_rptr_r = to_unsigned(g_DEPTH - 1, g_LOG2_DEPTH) then
                    v_rptr_next  := (others => '0');
                    v_round_next := not s_rptr_round_r;
                else
                    v_rptr_next  := s_rptr_r + 1;
                    v_round_next := s_rptr_round_r;
                end if;
                s_rptr_r       <= v_rptr_next;
                s_rptr_round_r <= v_round_next;
            end if;
        end if;
    end process p_read;

end architecture rtl;
