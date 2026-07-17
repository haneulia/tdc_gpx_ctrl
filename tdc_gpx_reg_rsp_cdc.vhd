-- =============================================================================
-- tdc_gpx_reg_rsp_cdc.vhd
-- Atomic TDC register-response crossing
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;

library xpm;
use xpm.vcomponents.all;

entity tdc_gpx_reg_rsp_cdc is
    generic (
        g_DATA_WIDTH : positive := 28;
        g_SYNC_FF    : integer range 2 to 10 := 4
    );
    port (
        i_src_clk    : in  std_logic;
        i_src_rst_n  : in  std_logic;
        i_src_done   : in  std_logic;
        i_src_rvalid : in  std_logic;
        i_src_rdata  : in  std_logic_vector(g_DATA_WIDTH - 1 downto 0);
        o_src_pending : out std_logic;

        i_dst_clk    : in  std_logic;
        i_dst_rst_n  : in  std_logic;
        i_dst_clear  : in  std_logic;
        o_dst_done   : out std_logic;
        o_dst_rvalid : out std_logic;
        o_dst_rvalid_held : out std_logic;
        o_dst_rdata  : out std_logic_vector(g_DATA_WIDTH - 1 downto 0)
    );
end entity tdc_gpx_reg_rsp_cdc;

architecture rtl of tdc_gpx_reg_rsp_cdc is

    constant c_PAYLOAD_WIDTH : positive := g_DATA_WIDTH + 1;

    signal s_src_payload_r : std_logic_vector(c_PAYLOAD_WIDTH - 1 downto 0)
        := (others => '0');
    signal s_src_send_r     : std_logic := '0';
    signal s_src_rcv        : std_logic;
    signal s_dst_req        : std_logic;
    signal s_dst_req_d_r    : std_logic := '0';
    signal s_dst_rvalid_held_r : std_logic := '0';
    signal s_dst_payload    : std_logic_vector(c_PAYLOAD_WIDTH - 1 downto 0);
    signal s_dst_pulse      : std_logic;

begin

    -- Hold both the qualifier and payload until the destination has accepted
    -- the response. i_src_done is included in o_src_pending so the caller can
    -- close the one-cycle gap before s_src_send_r is set.
    p_src_hold : process(i_src_clk)
    begin
        if rising_edge(i_src_clk) then
            if i_src_rst_n = '0' then
                s_src_payload_r <= (others => '0');
                s_src_send_r    <= '0';
            elsif s_src_send_r = '0' then
                if i_src_done = '1' and s_src_rcv = '0' then
                    s_src_payload_r(g_DATA_WIDTH) <= i_src_rvalid;
                    s_src_payload_r(g_DATA_WIDTH - 1 downto 0) <= i_src_rdata;
                    s_src_send_r <= '1';
                end if;
            elsif s_src_rcv = '1' then
                s_src_send_r <= '0';
            end if;
        end if;
    end process p_src_hold;

    -- A chip must not finish another register operation while the prior
    -- response is still crossing. config_ctrl enforces this by folding
    -- o_src_pending into the chip-busy level seen by cmd_arb.
    -- synthesis translate_off
    p_no_overlap : process(i_src_clk)
    begin
        if rising_edge(i_src_clk) then
            if i_src_rst_n = '1' then
                assert not (i_src_done = '1'
                            and (s_src_send_r = '1' or s_src_rcv = '1'))
                    report "reg_rsp_cdc: response completed while prior response was pending"
                    severity failure;
            end if;
        end if;
    end process p_no_overlap;
    -- synthesis translate_on

    u_cdc_rsp : xpm_cdc_handshake
        generic map (
            DEST_EXT_HSK   => 1,
            DEST_SYNC_FF   => g_SYNC_FF,
            INIT_SYNC_FF   => 0,
            SIM_ASSERT_CHK => 0,
            SRC_SYNC_FF    => g_SYNC_FF,
            WIDTH          => c_PAYLOAD_WIDTH
        )
        port map (
            src_clk  => i_src_clk,
            src_in   => s_src_payload_r,
            src_send => s_src_send_r,
            src_rcv  => s_src_rcv,
            dest_clk => i_dst_clk,
            dest_req => s_dst_req,
            dest_ack => s_dst_req,
            dest_out => s_dst_payload
        );

    -- Edge qualification makes the destination contract exactly one cycle,
    -- independent of the external-handshake request deassertion latency.
    p_dst_edge : process(i_dst_clk)
    begin
        if rising_edge(i_dst_clk) then
            if i_dst_rst_n = '0' then
                s_dst_req_d_r <= '0';
                s_dst_rvalid_held_r <= '0';
            else
                s_dst_req_d_r <= s_dst_req;
                if i_dst_clear = '1' then
                    s_dst_rvalid_held_r <= '0';
                elsif s_dst_pulse = '1'
                      and s_dst_payload(g_DATA_WIDTH) = '1' then
                    s_dst_rvalid_held_r <= '1';
                end if;
            end if;
        end if;
    end process p_dst_edge;

    s_dst_pulse <= s_dst_req and not s_dst_req_d_r and i_dst_rst_n;

    -- Keep pending asserted through the acknowledgement return-to-zero phase;
    -- XPM requires src_rcv low before the next src_send rising edge.
    o_src_pending <= s_src_send_r or s_src_rcv or i_src_done;
    o_dst_done     <= s_dst_pulse;
    o_dst_rvalid   <= s_dst_pulse and s_dst_payload(g_DATA_WIDTH);
    o_dst_rvalid_held <= s_dst_rvalid_held_r;
    o_dst_rdata    <= s_dst_payload(g_DATA_WIDTH - 1 downto 0);

end architecture rtl;
