-- =============================================================================
-- tdc_gpx_atomic_snapshot_cdc.vhd
-- Coherent latest-value mailbox for infrequently changing multi-bit data.
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;

library xpm;
use xpm.vcomponents.all;

entity tdc_gpx_atomic_snapshot_cdc is
    generic (
        g_WIDTH          : positive := 1;
        g_SYNC_FF        : positive range 2 to 10 := 4;
        g_SIM_ASSERT_CHK : natural range 0 to 1 := 0
    );
    port (
        i_src_clk     : in  std_logic;
        i_src_resetn  : in  std_logic;
        i_src_data    : in  std_logic_vector(g_WIDTH - 1 downto 0);
        i_dest_clk    : in  std_logic;
        i_dest_resetn : in  std_logic;
        o_dest_data   : out std_logic_vector(g_WIDTH - 1 downto 0)
    );
end entity tdc_gpx_atomic_snapshot_cdc;

architecture rtl of tdc_gpx_atomic_snapshot_cdc is
    signal s_src_payload_r : std_logic_vector(g_WIDTH - 1 downto 0) := (others => '0');
    signal s_dest_payload  : std_logic_vector(g_WIDTH - 1 downto 0);
    signal s_src_send_r    : std_logic := '0';
    signal s_src_rcv       : std_logic;
    signal s_dest_req      : std_logic;
    signal s_src_first_r   : std_logic := '1';
    signal s_dest_valid_r  : std_logic := '0';
begin
    -- XPM has no runtime reset and dest_out is undefined before the first
    -- transfer when INIT_SYNC_FF=0. Keep the consumer at zero until a complete
    -- destination request has been observed.
    o_dest_data <= s_dest_payload when i_dest_resetn = '1' and s_dest_valid_r = '1'
                   else (others => '0');

    u_snapshot : xpm_cdc_handshake
        generic map (
            DEST_EXT_HSK   => 1,
            DEST_SYNC_FF   => g_SYNC_FF,
            INIT_SYNC_FF   => 0,
            SIM_ASSERT_CHK => g_SIM_ASSERT_CHK,
            SRC_SYNC_FF    => g_SYNC_FF,
            WIDTH          => g_WIDTH
        )
        port map (
            src_clk  => i_src_clk,
            src_in   => s_src_payload_r,
            src_send => s_src_send_r,
            src_rcv  => s_src_rcv,
            dest_clk => i_dest_clk,
            dest_req => s_dest_req,
            dest_ack => s_dest_req,
            dest_out => s_dest_payload
        );

    p_dest_valid : process(i_dest_clk)
    begin
        if rising_edge(i_dest_clk) then
            if i_dest_resetn = '0' then
                s_dest_valid_r <= '0';
            elsif s_dest_req = '1' then
                s_dest_valid_r <= '1';
            end if;
        end if;
    end process p_dest_valid;

    -- Keep src_send asserted through acknowledgement, then wait for src_rcv
    -- to return low before launching another transfer. If data changes while
    -- busy, the latest live value is captured by the next handshake.
    p_send : process(i_src_clk)
    begin
        if rising_edge(i_src_clk) then
            if i_src_resetn = '0' then
                s_src_payload_r <= (others => '0');
                s_src_send_r    <= '0';
                s_src_first_r   <= '1';
            elsif s_src_send_r = '1' then
                if s_src_rcv = '1' then
                    s_src_send_r <= '0';
                end if;
            elsif s_src_rcv = '0' then
                if s_src_first_r = '1' or i_src_data /= s_src_payload_r then
                    s_src_payload_r <= i_src_data;
                    s_src_send_r    <= '1';
                    s_src_first_r   <= '0';
                end if;
            end if;
        end if;
    end process p_send;
end architecture rtl;
