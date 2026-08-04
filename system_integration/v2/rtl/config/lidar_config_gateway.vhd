library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_config_types_pkg.all;

-- Coherent bundled-data mailbox between the CSR configuration domain and one
-- functional domain. The source must hold i_candidate from PREPARE assertion
-- until every request and acknowledgement returns low.
entity lidar_config_gateway is
    port (
        i_csr_clk         : in  std_logic;
        i_csr_rst_n       : in  std_logic;
        i_domain_clk      : in  std_logic;
        i_domain_rst_n    : in  std_logic;
        i_prepare_req     : in  std_logic;
        i_activate_req    : in  std_logic;
        i_release_req     : in  std_logic;
        i_candidate       : in  lidar_active_config_t;
        i_safe_to_prepare : in  std_logic;
        o_prepare_ack     : out std_logic;
        o_activate_ack    : out std_logic;
        o_release_ack     : out std_logic;
        o_fault_csr       : out std_logic;
        o_domain_enable   : out std_logic;
        o_active_valid    : out std_logic;
        o_active          : out lidar_active_config_t
    );
end entity lidar_config_gateway;

architecture rtl of lidar_config_gateway is

    signal req_meta_r : std_logic_vector(2 downto 0) := (others => '0');
    signal req_sync_r : std_logic_vector(2 downto 0) := (others => '0');

    signal prepared_r    : std_logic := '0';
    signal activated_r   : std_logic := '0';
    signal released_r    : std_logic := '0';
    signal prepare_ack_r : std_logic := '0';
    signal activate_ack_r : std_logic := '0';
    signal release_ack_r : std_logic := '0';
    signal fault_r       : std_logic := '0';
    signal domain_enable_r : std_logic := '0';
    signal active_valid_r  : std_logic := '0';
    signal prepared_cfg_r  : lidar_active_config_t;
    signal active_cfg_r    : lidar_active_config_t;

    signal ack_meta_r : std_logic_vector(3 downto 0) := (others => '0');
    signal ack_sync_r : std_logic_vector(3 downto 0) := (others => '0');

    attribute ASYNC_REG : string;
    attribute ASYNC_REG of req_meta_r : signal is "TRUE";
    attribute ASYNC_REG of req_sync_r : signal is "TRUE";
    attribute ASYNC_REG of ack_meta_r : signal is "TRUE";
    attribute ASYNC_REG of ack_sync_r : signal is "TRUE";

begin

    o_prepare_ack   <= ack_sync_r(0);
    o_activate_ack  <= ack_sync_r(1);
    o_release_ack   <= ack_sync_r(2);
    o_fault_csr     <= ack_sync_r(3);
    o_domain_enable <= domain_enable_r;
    o_active_valid  <= active_valid_r;
    o_active        <= active_cfg_r;

    p_request_sync : process (i_domain_clk, i_domain_rst_n)
    begin
        if i_domain_rst_n = '0' then
            req_meta_r <= (others => '0');
            req_sync_r <= (others => '0');
        elsif rising_edge(i_domain_clk) then
            req_meta_r <= i_release_req & i_activate_req & i_prepare_req;
            req_sync_r <= req_meta_r;
        end if;
    end process p_request_sync;

    p_domain : process (i_domain_clk, i_domain_rst_n)
    begin
        if i_domain_rst_n = '0' then
            prepared_r      <= '0';
            activated_r     <= '0';
            released_r      <= '0';
            prepare_ack_r   <= '0';
            activate_ack_r  <= '0';
            release_ack_r   <= '0';
            fault_r         <= '0';
            domain_enable_r <= '0';
            active_valid_r  <= '0';
        elsif rising_edge(i_domain_clk) then
            if req_sync_r(0) = '0' then
                prepare_ack_r <= '0';
            end if;
            if req_sync_r(1) = '0' then
                activate_ack_r <= '0';
            end if;
            if req_sync_r(2) = '0' then
                release_ack_r <= '0';
            end if;

            if req_sync_r = "000" then
                prepared_r  <= '0';
                activated_r <= '0';
                released_r  <= '0';
            elsif req_sync_r(0) = '0' then
                fault_r <= '1';
            else
                if released_r = '0' then
                    domain_enable_r <= '0';
                end if;

                if prepared_r = '0' then
                    prepared_cfg_r <= i_candidate;
                    prepared_r <= '1';
                elsif prepare_ack_r = '0' then
                    if i_safe_to_prepare = '1' then
                        prepare_ack_r <= '1';
                    end if;
                elsif req_sync_r(1) = '1' and activated_r = '0' then
                    -- PREPARE captured the coherently held mailbox payload.
                    -- Do not read the asynchronous source again here: doing so
                    -- would put a CDC comparison on every active register CE.
                    active_cfg_r   <= prepared_cfg_r;
                    active_valid_r <= '1';
                    activated_r    <= '1';
                    activate_ack_r <= '1';
                elsif req_sync_r(2) = '1' and released_r = '0' then
                    if activated_r = '1' then
                        domain_enable_r <= '1';
                        released_r      <= '1';
                        release_ack_r   <= '1';
                    else
                        fault_r <= '1';
                    end if;
                end if;
            end if;

            if req_sync_r(0) = '0' and activated_r = '0' then
                domain_enable_r <= active_valid_r;
            end if;
        end if;
    end process p_domain;

    p_ack_sync : process (i_csr_clk, i_csr_rst_n)
    begin
        if i_csr_rst_n = '0' then
            ack_meta_r <= (others => '0');
            ack_sync_r <= (others => '0');
        elsif rising_edge(i_csr_clk) then
            ack_meta_r <= fault_r & release_ack_r
                & activate_ack_r & prepare_ack_r;
            ack_sync_r <= ack_meta_r;
        end if;
    end process p_ack_sync;

end architecture rtl;
