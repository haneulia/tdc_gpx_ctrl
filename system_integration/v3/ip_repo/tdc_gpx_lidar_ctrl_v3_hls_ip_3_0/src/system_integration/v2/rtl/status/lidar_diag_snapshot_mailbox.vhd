library ieee;
use ieee.std_logic_1164.all;

use work.lidar_status_pkg.all;

-- One-outstanding bundled-data diagnostic mailbox. Request and response
-- payloads remain registered and stable while a synchronized toggle crosses
-- the boundary. This is intentionally smaller than an asynchronous FIFO:
-- software can issue only one indexed diagnostic read at a time.
entity lidar_diag_snapshot_mailbox is
    port (
        i_source_clk           : in  std_logic;
        i_source_rst_n         : in  std_logic;
        i_source_request_valid : in  std_logic;
        o_source_request_ready : out std_logic;
        i_source_request       : in  lidar_diag_index_t;
        o_source_response_valid : out std_logic;
        i_source_response_ready : in  std_logic;
        o_source_response       : out lidar_diag_response_t;

        i_domain_clk           : in  std_logic;
        i_domain_rst_n         : in  std_logic;
        o_domain_request_valid : out std_logic;
        i_domain_request_ready : in  std_logic;
        o_domain_request       : out lidar_diag_index_t;
        i_domain_response_valid : in  std_logic;
        o_domain_response_ready : out std_logic;
        i_domain_response       : in  lidar_diag_response_t
    );
end entity lidar_diag_snapshot_mailbox;

architecture rtl of lidar_diag_snapshot_mailbox is

    type source_state_t is (
        SOURCE_IDLE,
        SOURCE_WAIT_RESPONSE,
        SOURCE_CAPTURE_RESPONSE,
        SOURCE_HOLD_RESPONSE,
        SOURCE_RECOVER
    );
    type domain_state_t is (
        DOMAIN_INIT,
        DOMAIN_IDLE,
        DOMAIN_SETTLE_REQUEST,
        DOMAIN_SEND_REQUEST,
        DOMAIN_WAIT_RESPONSE,
        DOMAIN_COMMIT_RESPONSE
    );

    signal source_state_r : source_state_t := SOURCE_IDLE;
    signal request_toggle_r : std_logic := '0';
    signal request_payload_r : lidar_diag_index_t := (others => '0');
    signal source_online_r : std_logic := '0';
    signal source_response_r : lidar_diag_response_t := (others => '0');

    signal response_toggle_meta_r : std_logic := '0';
    signal response_toggle_sync_r : std_logic := '0';
    signal domain_ready_meta_r : std_logic := '0';
    signal domain_ready_sync_r : std_logic := '0';

    signal request_toggle_meta_r : std_logic := '0';
    signal request_toggle_sync_r : std_logic := '0';
    signal source_online_meta_r : std_logic := '0';
    signal source_online_sync_r : std_logic := '0';

    signal domain_state_r : domain_state_t := DOMAIN_INIT;
    signal domain_init_count_r : natural range 0 to 3 := 0;
    signal domain_ready_r : std_logic := '0';
    signal domain_request_r : lidar_diag_index_t := (others => '0');
    signal domain_response_r : lidar_diag_response_t :=
        fn_pack_diag_response((others => '0'), '1');
    signal response_toggle_r : std_logic := '0';

    signal source_request_ready_c : std_logic;

    attribute ASYNC_REG : string;
    attribute SHREG_EXTRACT : string;
    attribute ASYNC_REG of response_toggle_meta_r : signal is "TRUE";
    attribute ASYNC_REG of response_toggle_sync_r : signal is "TRUE";
    attribute ASYNC_REG of domain_ready_meta_r : signal is "TRUE";
    attribute ASYNC_REG of domain_ready_sync_r : signal is "TRUE";
    attribute ASYNC_REG of request_toggle_meta_r : signal is "TRUE";
    attribute ASYNC_REG of request_toggle_sync_r : signal is "TRUE";
    attribute ASYNC_REG of source_online_meta_r : signal is "TRUE";
    attribute ASYNC_REG of source_online_sync_r : signal is "TRUE";
    attribute SHREG_EXTRACT of response_toggle_meta_r : signal is "NO";
    attribute SHREG_EXTRACT of response_toggle_sync_r : signal is "NO";
    attribute SHREG_EXTRACT of domain_ready_meta_r : signal is "NO";
    attribute SHREG_EXTRACT of domain_ready_sync_r : signal is "NO";
    attribute SHREG_EXTRACT of request_toggle_meta_r : signal is "NO";
    attribute SHREG_EXTRACT of request_toggle_sync_r : signal is "NO";
    attribute SHREG_EXTRACT of source_online_meta_r : signal is "NO";
    attribute SHREG_EXTRACT of source_online_sync_r : signal is "NO";

begin

    source_request_ready_c <= '1' when
        source_state_r = SOURCE_IDLE and
        domain_ready_sync_r = '1' and
        response_toggle_sync_r = request_toggle_r else '0';

    o_source_request_ready <= source_request_ready_c;
    o_source_response_valid <= '1' when
        source_state_r = SOURCE_HOLD_RESPONSE else '0';
    o_source_response <= source_response_r;

    o_domain_request_valid <= '1' when
        domain_state_r = DOMAIN_SEND_REQUEST else '0';
    o_domain_request <= domain_request_r;
    o_domain_response_ready <= '1' when
        domain_state_r = DOMAIN_WAIT_RESPONSE else '0';

    p_source_return_sync : process (i_source_clk, i_source_rst_n)
    begin
        if i_source_rst_n = '0' then
            response_toggle_meta_r <= '0';
            response_toggle_sync_r <= '0';
            domain_ready_meta_r <= '0';
            domain_ready_sync_r <= '0';
        elsif rising_edge(i_source_clk) then
            response_toggle_meta_r <= response_toggle_r;
            response_toggle_sync_r <= response_toggle_meta_r;
            domain_ready_meta_r <= domain_ready_r;
            domain_ready_sync_r <= domain_ready_meta_r;
        end if;
    end process p_source_return_sync;

    p_source : process (i_source_clk, i_source_rst_n)
    begin
        if i_source_rst_n = '0' then
            source_state_r <= SOURCE_IDLE;
            request_toggle_r <= '0';
            request_payload_r <= (others => '0');
            source_online_r <= '0';
            source_response_r <= (others => '0');
        elsif rising_edge(i_source_clk) then
            source_online_r <= '1';

            case source_state_r is
                when SOURCE_IDLE =>
                    if i_source_request_valid = '1' and
                            source_request_ready_c = '1' then
                        request_payload_r <= i_source_request;
                        request_toggle_r <= not request_toggle_r;
                        source_state_r <= SOURCE_WAIT_RESPONSE;
                    end if;

                when SOURCE_WAIT_RESPONSE =>
                    if domain_ready_sync_r = '0' then
                        source_response_r <= fn_pack_diag_response(
                            (others => '0'), '1');
                        source_state_r <= SOURCE_HOLD_RESPONSE;
                    elsif response_toggle_sync_r = request_toggle_r then
                        -- The response register was committed before its
                        -- toggle. Wait one more source clock before capture.
                        source_state_r <= SOURCE_CAPTURE_RESPONSE;
                    end if;

                when SOURCE_CAPTURE_RESPONSE =>
                    source_response_r <= domain_response_r;
                    source_state_r <= SOURCE_HOLD_RESPONSE;

                when SOURCE_HOLD_RESPONSE =>
                    if i_source_response_ready = '1' then
                        if domain_ready_sync_r = '1' and
                                response_toggle_sync_r = request_toggle_r then
                            source_state_r <= SOURCE_IDLE;
                        else
                            source_state_r <= SOURCE_RECOVER;
                        end if;
                    end if;

                when SOURCE_RECOVER =>
                    if domain_ready_sync_r = '1' and
                            response_toggle_sync_r = request_toggle_r then
                        source_state_r <= SOURCE_IDLE;
                    end if;
            end case;
        end if;
    end process p_source;

    p_domain_request_sync : process (i_domain_clk, i_domain_rst_n)
    begin
        if i_domain_rst_n = '0' then
            request_toggle_meta_r <= '0';
            request_toggle_sync_r <= '0';
            source_online_meta_r <= '0';
            source_online_sync_r <= '0';
        elsif rising_edge(i_domain_clk) then
            request_toggle_meta_r <= request_toggle_r;
            request_toggle_sync_r <= request_toggle_meta_r;
            source_online_meta_r <= source_online_r;
            source_online_sync_r <= source_online_meta_r;
        end if;
    end process p_domain_request_sync;

    p_domain : process (i_domain_clk, i_domain_rst_n)
    begin
        if i_domain_rst_n = '0' then
            domain_state_r <= DOMAIN_INIT;
            domain_init_count_r <= 0;
            domain_ready_r <= '0';
            domain_request_r <= (others => '0');
            domain_response_r <= fn_pack_diag_response(
                (others => '0'), '1');
            response_toggle_r <= '0';
        elsif rising_edge(i_domain_clk) then
            if source_online_sync_r = '0' then
                domain_state_r <= DOMAIN_INIT;
                domain_init_count_r <= 0;
                domain_ready_r <= '0';
                domain_response_r <= fn_pack_diag_response(
                    (others => '0'), '1');
            else
                case domain_state_r is
                    when DOMAIN_INIT =>
                        if domain_init_count_r = 3 then
                            -- Rebase to the current source toggle. Requests
                            -- held across either reset are intentionally
                            -- flushed and never replayed.
                            response_toggle_r <= request_toggle_sync_r;
                            domain_ready_r <= '1';
                            domain_state_r <= DOMAIN_IDLE;
                        else
                            domain_init_count_r <= domain_init_count_r + 1;
                        end if;

                    when DOMAIN_IDLE =>
                        if request_toggle_sync_r /= response_toggle_r then
                            domain_state_r <= DOMAIN_SETTLE_REQUEST;
                        end if;

                    when DOMAIN_SETTLE_REQUEST =>
                        domain_request_r <= request_payload_r;
                        domain_state_r <= DOMAIN_SEND_REQUEST;

                    when DOMAIN_SEND_REQUEST =>
                        if i_domain_request_ready = '1' then
                            domain_state_r <= DOMAIN_WAIT_RESPONSE;
                        end if;

                    when DOMAIN_WAIT_RESPONSE =>
                        if i_domain_response_valid = '1' then
                            domain_response_r <= i_domain_response;
                            domain_state_r <= DOMAIN_COMMIT_RESPONSE;
                        end if;

                    when DOMAIN_COMMIT_RESPONSE =>
                        response_toggle_r <= request_toggle_sync_r;
                        domain_state_r <= DOMAIN_IDLE;
                end case;
            end if;
        end if;
    end process p_domain;

end architecture rtl;
