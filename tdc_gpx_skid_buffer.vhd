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
--   States:
--     ST_PIPE: data pipes through when downstream ready.
--              On stall, input stored to temp buffer (skid).
--     ST_SKID: wait for downstream ready, then promote temp to output.
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

    type t_state is (ST_PIPE, ST_SKID);
    signal s_state_r        : t_state := ST_PIPE;

    -- Output data register
    signal s_m_data_r       : std_logic_vector(g_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_m_valid_r      : std_logic := '0';

    -- Temp (skid) buffer
    signal s_m_data_temp_r  : std_logic_vector(g_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_m_valid_temp_r : std_logic := '0';

    -- Registered ready
    signal s_s_ready_r      : std_logic := '0';

    -- Pipeline ready: downstream can accept or output is empty
    signal s_ready          : std_logic;

begin

    s_ready   <= i_m_ready or (not s_m_valid_r);
    o_s_ready <= s_s_ready_r;
    o_m_data  <= s_m_data_r;
    o_m_valid <= s_m_valid_r;

    p_main : process(i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' or i_flush = '1' then
                s_state_r        <= ST_PIPE;
                s_m_data_r       <= (others => '0');
                s_m_data_temp_r  <= (others => '0');
                s_m_valid_r      <= '0';
                s_m_valid_temp_r <= '0';
                s_s_ready_r      <= '0';
            else
                case s_state_r is

                    when ST_PIPE =>
                        if s_ready = '1' then
                            -- Pipe data through to output
                            s_m_data_r  <= i_s_data;
                            s_m_valid_r <= i_s_valid;
                            s_s_ready_r <= '1';
                            s_state_r   <= ST_PIPE;
                        else
                            -- Pipeline stall: store input to temp buffer (skid)
                            s_m_data_temp_r  <= i_s_data;
                            s_m_valid_temp_r <= i_s_valid;
                            s_s_ready_r      <= '0';
                            s_state_r        <= ST_SKID;
                        end if;

                    when ST_SKID =>
                        if s_ready = '1' then
                            -- Promote temp to output, resume pipeline
                            s_m_data_r  <= s_m_data_temp_r;
                            s_m_valid_r <= s_m_valid_temp_r;
                            s_s_ready_r <= '1';
                            s_state_r   <= ST_PIPE;
                        end if;

                end case;
            end if;
        end if;
    end process p_main;

end architecture rtl;
