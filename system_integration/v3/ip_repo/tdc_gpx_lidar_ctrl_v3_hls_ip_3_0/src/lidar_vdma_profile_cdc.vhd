library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library xpm;
use xpm.vcomponents.all;

-- Transfers one coherent VDMA lane profile from the processing clock domain
-- to the unified CSR clock domain. The processing profile remains pending
-- until software programs the external VDMA and acknowledges that exact
-- snapshot through the CSR bank.
entity lidar_vdma_profile_cdc is
    port (
        i_proc_clk          : in  std_logic;
        i_proc_rst_n        : in  std_logic;
        i_proc_cfg_valid    : in  std_logic;
        i_proc_cfg_enable   : in  std_logic;
        i_proc_hsize_bytes  : in  unsigned(15 downto 0);
        i_proc_vsize_lines  : in  unsigned(15 downto 0);
        i_proc_stride_bytes : in  unsigned(15 downto 0);
        o_proc_cfg_ready    : out std_logic;

        i_csr_clk          : in  std_logic;
        i_csr_rst_n        : in  std_logic;
        o_csr_cfg_valid    : out std_logic;
        o_csr_cfg_enable   : out std_logic;
        o_csr_hsize_bytes  : out unsigned(15 downto 0);
        o_csr_vsize_lines  : out unsigned(15 downto 0);
        o_csr_stride_bytes : out unsigned(15 downto 0);
        i_csr_cfg_ack      : in  std_logic
    );
end entity lidar_vdma_profile_cdc;

architecture rtl of lidar_vdma_profile_cdc is

    constant C_PROFILE_WIDTH : positive := 49;

    type proc_state_t is (
        PROC_IDLE,
        PROC_WAIT_ACK,
        PROC_WAIT_VALID_LOW,
        PROC_WAIT_CDC_RELEASE
    );

    signal proc_state_r : proc_state_t := PROC_IDLE;
    signal proc_payload_r : std_logic_vector(
        C_PROFILE_WIDTH - 1 downto 0) := (others => '0');
    signal proc_send_r : std_logic := '0';
    signal proc_received : std_logic;
    signal proc_ready_r : std_logic := '0';

    signal csr_payload : std_logic_vector(
        C_PROFILE_WIDTH - 1 downto 0);
    signal csr_request : std_logic;
    signal csr_ack_r : std_logic := '0';
    signal csr_ack_armed_r : std_logic := '0';

begin

    o_proc_cfg_ready <= proc_ready_r;

    o_csr_cfg_valid <= csr_request;
    o_csr_cfg_enable <= csr_payload(48);
    o_csr_hsize_bytes <= unsigned(csr_payload(47 downto 32));
    o_csr_vsize_lines <= unsigned(csr_payload(31 downto 16));
    o_csr_stride_bytes <= unsigned(csr_payload(15 downto 0));

    u_profile_cdc : xpm_cdc_handshake
        generic map (
            DEST_EXT_HSK   => 1,
            DEST_SYNC_FF   => 4,
            INIT_SYNC_FF   => 0,
            SIM_ASSERT_CHK => 1,
            SRC_SYNC_FF    => 4,
            WIDTH          => C_PROFILE_WIDTH
        )
        port map (
            src_clk  => i_proc_clk,
            src_in   => proc_payload_r,
            src_send => proc_send_r,
            src_rcv  => proc_received,
            dest_clk => i_csr_clk,
            dest_req => csr_request,
            dest_ack => csr_ack_r,
            dest_out => csr_payload
        );

    p_proc_request : process (i_proc_clk)
    begin
        if rising_edge(i_proc_clk) then
            proc_ready_r <= '0';

            if i_proc_rst_n = '0' then
                proc_state_r <= PROC_IDLE;
                proc_payload_r <= (others => '0');
                proc_send_r <= '0';
            else
                case proc_state_r is
                    when PROC_IDLE =>
                        proc_send_r <= '0';
                        if i_proc_cfg_valid = '1' and
                                proc_received = '0' then
                            proc_payload_r <= i_proc_cfg_enable &
                                std_logic_vector(i_proc_hsize_bytes) &
                                std_logic_vector(i_proc_vsize_lines) &
                                std_logic_vector(i_proc_stride_bytes);
                            proc_send_r <= '1';
                            proc_state_r <= PROC_WAIT_ACK;
                        end if;

                    when PROC_WAIT_ACK =>
                        if proc_received = '1' then
                            -- The profile manager activates the pending
                            -- profile only after this one-cycle pulse.
                            proc_ready_r <= '1';
                            proc_send_r <= '0';
                            proc_state_r <= PROC_WAIT_VALID_LOW;
                        end if;

                    when PROC_WAIT_VALID_LOW =>
                        if i_proc_cfg_valid = '0' then
                            if proc_received = '0' then
                                proc_state_r <= PROC_IDLE;
                            else
                                proc_state_r <= PROC_WAIT_CDC_RELEASE;
                            end if;
                        end if;

                    when PROC_WAIT_CDC_RELEASE =>
                        if proc_received = '0' then
                            proc_state_r <= PROC_IDLE;
                        end if;
                end case;
            end if;
        end if;
    end process p_proc_request;

    p_csr_ack : process (i_csr_clk)
    begin
        if rising_edge(i_csr_clk) then
            if i_csr_rst_n = '0' then
                csr_ack_r <= '0';
                csr_ack_armed_r <= '0';
            elsif csr_request = '0' then
                csr_ack_r <= '0';
                -- A stale-high software ACK is not accepted as the ACK for
                -- a later profile. A low level must be observed first.
                if i_csr_cfg_ack = '0' then
                    csr_ack_armed_r <= '1';
                else
                    csr_ack_armed_r <= '0';
                end if;
            elsif csr_ack_r = '0' then
                if i_csr_cfg_ack = '0' then
                    csr_ack_armed_r <= '1';
                elsif csr_ack_armed_r = '1' then
                    csr_ack_r <= '1';
                    csr_ack_armed_r <= '0';
                end if;
            end if;
        end if;
    end process p_csr_ack;

end architecture rtl;
