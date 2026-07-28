-- =============================================================================
-- tdc_gpx_unified_cdc_snapshot.vhd
-- Coalescing multi-bit snapshot transfer for the unified TDC CSR adapter
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;

library xpm;
use xpm.vcomponents.all;

entity tdc_gpx_unified_cdc_snapshot is
    generic (
        G_WIDTH : positive := 32
    );
    port (
        src_clk     : in  std_logic;
        src_rst_n   : in  std_logic;
        i_src_data  : in  std_logic_vector(G_WIDTH - 1 downto 0);
        dest_clk    : in  std_logic;
        o_dest_data : out std_logic_vector(G_WIDTH - 1 downto 0)
    );
end entity tdc_gpx_unified_cdc_snapshot;

architecture rtl of tdc_gpx_unified_cdc_snapshot is
    constant C_ZERO : std_logic_vector(G_WIDTH - 1 downto 0) :=
        (others => '0');

    signal s_src_hold      : std_logic_vector(G_WIDTH - 1 downto 0) := C_ZERO;
    signal s_src_seen      : std_logic_vector(G_WIDTH - 1 downto 0) := not C_ZERO;
    signal s_src_send      : std_logic := '0';
    signal s_src_rcv       : std_logic;
    signal s_reset_pending : std_logic := '0';
    signal s_dest_req      : std_logic;
    signal s_dest_raw      : std_logic_vector(G_WIDTH - 1 downto 0);
    signal s_dest_data_r   : std_logic_vector(G_WIDTH - 1 downto 0) := C_ZERO;
begin
    -- XPM requires src_in to remain unchanged for the complete handshake.
    -- Changes received while busy are coalesced into one following snapshot.
    p_source : process(src_clk)
    begin
        if rising_edge(src_clk) then
            if s_src_send = '1' then
                if src_rst_n = '0' then
                    s_reset_pending <= '1';
                end if;
                if s_src_rcv = '1' then
                    s_src_send <= '0';
                    if src_rst_n = '0' or s_reset_pending = '1' then
                        s_src_seen      <= not C_ZERO;
                        s_reset_pending <= not src_rst_n;
                    end if;
                end if;
            elsif s_src_rcv = '0' then
                if src_rst_n = '0' then
                    s_src_hold      <= C_ZERO;
                    s_src_seen      <= not C_ZERO;
                    s_reset_pending <= '0';
                elsif s_reset_pending = '1' then
                    s_src_hold      <= C_ZERO;
                    s_src_seen      <= not C_ZERO;
                    s_reset_pending <= '0';
                elsif i_src_data /= s_src_seen then
                    s_src_hold <= i_src_data;
                    s_src_seen <= i_src_data;
                    s_src_send <= '1';
                end if;
            end if;
        end if;
    end process p_source;

    p_destination : process(dest_clk)
    begin
        if rising_edge(dest_clk) then
            if s_dest_req = '1' then
                s_dest_data_r <= s_dest_raw;
            end if;
        end if;
    end process p_destination;

    o_dest_data <= s_dest_data_r;

    u_handshake : xpm_cdc_handshake
        generic map (
            DEST_EXT_HSK   => 1,
            DEST_SYNC_FF   => 4,
            INIT_SYNC_FF   => 0,
            SIM_ASSERT_CHK => 1,
            SRC_SYNC_FF    => 4,
            WIDTH          => G_WIDTH
        )
        port map (
            src_clk  => src_clk,
            src_in   => s_src_hold,
            src_send => s_src_send,
            src_rcv  => s_src_rcv,
            dest_clk => dest_clk,
            dest_req => s_dest_req,
            dest_ack => s_dest_req,
            dest_out => s_dest_raw
        );
end architecture rtl;
