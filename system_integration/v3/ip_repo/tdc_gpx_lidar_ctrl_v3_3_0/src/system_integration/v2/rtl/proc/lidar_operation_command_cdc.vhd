library ieee;
use ieee.std_logic_1164.all;

use work.lidar_event_types_pkg.all;

-- One-entry acknowledged command mailbox from the CSR clock to the Processing
-- clock. The payload remains stable until acknowledgement, and the destination
-- waits one extra clock after detecting the request before sampling it.
--
-- Destination reset deliberately flushes an in-flight command. Losing a RUN or
-- ARM across reset is fail-safe; software must observe READY and reissue it.
entity lidar_operation_command_cdc is
    port (
        i_source_clk      : in  std_logic;
        i_source_rst_n    : in  std_logic;
        i_command_valid   : in  std_logic;
        i_command         : in  operation_command_t;
        o_source_ready    : out std_logic;
        o_source_busy     : out std_logic;
        o_source_rejected : out std_logic;

        i_domain_clk      : in  std_logic;
        i_domain_rst_n    : in  std_logic;
        o_domain_source_online : out std_logic;
        o_command_valid   : out std_logic;
        o_command         : out operation_command_t
    );
end entity lidar_operation_command_cdc;

architecture rtl of lidar_operation_command_cdc is

    signal request_toggle_r : std_logic := '0';
    signal payload_r        : operation_command_code_t := (others => '0');
    signal source_rejected_r : std_logic := '0';
    signal source_online_r   : std_logic := '0';

    signal ack_meta_r        : std_logic := '0';
    signal ack_sync_r        : std_logic := '0';
    signal domain_ready_meta_r : std_logic := '0';
    signal domain_ready_sync_r : std_logic := '0';

    signal request_meta_r : std_logic := '0';
    signal request_sync_r : std_logic := '0';
    signal source_online_meta_r : std_logic := '0';
    signal source_online_sync_r : std_logic := '0';
    signal payload_meta_r : operation_command_code_t := (others => '0');
    signal payload_sync_r : operation_command_code_t := (others => '0');

    signal ack_toggle_r    : std_logic := '0';
    signal domain_ready_r  : std_logic := '0';
    signal settle_pending_r: std_logic := '0';
    signal init_count_r    : natural range 0 to 3 := 0;
    signal command_valid_r : std_logic := '0';
    signal command_r       : operation_command_t := OP_COMMAND_NONE;

    signal source_busy_c  : std_logic;
    signal source_ready_c : std_logic;

    attribute ASYNC_REG : string;
    attribute SHREG_EXTRACT : string;
    attribute ASYNC_REG of ack_meta_r : signal is "TRUE";
    attribute ASYNC_REG of ack_sync_r : signal is "TRUE";
    attribute ASYNC_REG of domain_ready_meta_r : signal is "TRUE";
    attribute ASYNC_REG of domain_ready_sync_r : signal is "TRUE";
    attribute ASYNC_REG of request_meta_r : signal is "TRUE";
    attribute ASYNC_REG of request_sync_r : signal is "TRUE";
    attribute ASYNC_REG of source_online_meta_r : signal is "TRUE";
    attribute ASYNC_REG of source_online_sync_r : signal is "TRUE";
    attribute ASYNC_REG of payload_meta_r : signal is "TRUE";
    attribute ASYNC_REG of payload_sync_r : signal is "TRUE";
    attribute SHREG_EXTRACT of ack_meta_r : signal is "NO";
    attribute SHREG_EXTRACT of ack_sync_r : signal is "NO";
    attribute SHREG_EXTRACT of domain_ready_meta_r : signal is "NO";
    attribute SHREG_EXTRACT of domain_ready_sync_r : signal is "NO";
    attribute SHREG_EXTRACT of request_meta_r : signal is "NO";
    attribute SHREG_EXTRACT of request_sync_r : signal is "NO";
    attribute SHREG_EXTRACT of source_online_meta_r : signal is "NO";
    attribute SHREG_EXTRACT of source_online_sync_r : signal is "NO";
    attribute SHREG_EXTRACT of payload_meta_r : signal is "NO";
    attribute SHREG_EXTRACT of payload_sync_r : signal is "NO";

begin

    source_busy_c  <= request_toggle_r xor ack_sync_r;
    source_ready_c <= domain_ready_sync_r and not source_busy_c;

    o_source_ready    <= source_ready_c;
    o_source_busy     <= source_busy_c;
    o_source_rejected <= source_rejected_r;
    o_domain_source_online <= source_online_sync_r;
    o_command_valid   <= command_valid_r;
    o_command         <= command_r;

    p_source_return_sync : process (i_source_clk, i_source_rst_n)
    begin
        if i_source_rst_n = '0' then
            ack_meta_r           <= '0';
            ack_sync_r           <= '0';
            domain_ready_meta_r  <= '0';
            domain_ready_sync_r  <= '0';
        elsif rising_edge(i_source_clk) then
            ack_meta_r          <= ack_toggle_r;
            ack_sync_r          <= ack_meta_r;
            domain_ready_meta_r <= domain_ready_r;
            domain_ready_sync_r <= domain_ready_meta_r;
        end if;
    end process p_source_return_sync;

    p_source : process (i_source_clk, i_source_rst_n)
    begin
        if i_source_rst_n = '0' then
            request_toggle_r  <= '0';
            payload_r         <= fn_operation_command_code(
                OP_COMMAND_STOP);
            source_rejected_r <= '0';
            source_online_r   <= '0';
        elsif rising_edge(i_source_clk) then
            source_rejected_r <= '0';
            source_online_r   <= '1';
            if i_command_valid = '1' then
                if source_ready_c = '1' then
                    payload_r <= fn_operation_command_code(i_command);
                    request_toggle_r <= not request_toggle_r;
                else
                    source_rejected_r <= '1';
                end if;
            end if;
        end if;
    end process p_source;

    p_domain_request_sync : process (i_domain_clk, i_domain_rst_n)
    begin
        if i_domain_rst_n = '0' then
            request_meta_r <= '0';
            request_sync_r <= '0';
            source_online_meta_r <= '0';
            source_online_sync_r <= '0';
            payload_meta_r <= fn_operation_command_code(OP_COMMAND_STOP);
            payload_sync_r <= fn_operation_command_code(OP_COMMAND_STOP);
        elsif rising_edge(i_domain_clk) then
            request_meta_r <= request_toggle_r;
            request_sync_r <= request_meta_r;
            source_online_meta_r <= source_online_r;
            source_online_sync_r <= source_online_meta_r;
            payload_meta_r <= payload_r;
            payload_sync_r <= payload_meta_r;
        end if;
    end process p_domain_request_sync;

    p_domain : process (i_domain_clk, i_domain_rst_n)
    begin
        if i_domain_rst_n = '0' then
            ack_toggle_r     <= '0';
            domain_ready_r   <= '0';
            settle_pending_r <= '0';
            init_count_r     <= 0;
            command_valid_r  <= '0';
            command_r        <= OP_COMMAND_NONE;
        elsif rising_edge(i_domain_clk) then
            command_valid_r <= '0';

            if domain_ready_r = '0' then
                settle_pending_r <= '0';
                if init_count_r = 3 then
                    -- Flush anything held before or during destination reset.
                    ack_toggle_r   <= request_sync_r;
                    domain_ready_r <= '1';
                else
                    init_count_r <= init_count_r + 1;
                end if;
            elsif settle_pending_r = '1' then
                command_r       <= fn_operation_command_from_code(
                    payload_sync_r);
                command_valid_r <= '1';
                ack_toggle_r    <= request_sync_r;
                settle_pending_r <= '0';
            elsif request_sync_r /= ack_toggle_r then
                settle_pending_r <= '1';
            end if;
        end if;
    end process p_domain;

end architecture rtl;
