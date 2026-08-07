library ieee;
use ieee.std_logic_1164.all;

-- One-entry, acknowledged CSR command fanout to the Processing and TDC
-- domains. The four-phase request/acknowledge levels are reset-safe: resetting
-- the source returns request LOW instead of creating a false toggle event.
-- CLEAR_STATUS and SOFT_RESET are idempotent. A destination reset during an
-- in-flight command may replay that command after reset, but it cannot lose it
-- silently or convert it into the other command.
entity lidar_system_command_cdc is
    port (
        i_source_clk        : in  std_logic;
        i_source_rst_n      : in  std_logic;
        i_clear_status      : in  std_logic;
        i_soft_reset        : in  std_logic;
        o_source_ready      : out std_logic;
        o_source_busy       : out std_logic;
        o_source_rejected   : out std_logic;

        i_proc_clk          : in  std_logic;
        i_proc_rst_n        : in  std_logic;
        o_proc_clear_status : out std_logic;
        o_proc_soft_reset   : out std_logic;

        i_tdc_clk           : in  std_logic;
        i_tdc_rst_n         : in  std_logic;
        o_tdc_clear_status  : out std_logic;
        o_tdc_soft_reset    : out std_logic
    );
end entity lidar_system_command_cdc;

architecture rtl of lidar_system_command_cdc is

    subtype command_code_t is std_logic_vector(1 downto 0);
    constant C_COMMAND_CLEAR_STATUS : command_code_t := "01";
    constant C_COMMAND_SOFT_RESET   : command_code_t := "10";

    type source_state_t is (
        SOURCE_IDLE,
        SOURCE_WAIT_ACK_HIGH,
        SOURCE_WAIT_ACK_LOW
    );
    type sink_state_t is (SINK_IDLE, SINK_SETTLE, SINK_ACK);

    signal source_state_r : source_state_t := SOURCE_IDLE;
    signal request_r       : std_logic := '0';
    signal payload_r       : command_code_t := C_COMMAND_CLEAR_STATUS;
    signal rejected_r      : std_logic := '0';

    signal proc_ack_r      : std_logic := '0';
    signal proc_ready_r    : std_logic := '0';
    signal proc_ack_meta_r : std_logic := '0';
    signal proc_ack_sync_r : std_logic := '0';
    signal proc_ready_meta_r : std_logic := '0';
    signal proc_ready_sync_r : std_logic := '0';

    signal tdc_ack_r      : std_logic := '0';
    signal tdc_ready_r    : std_logic := '0';
    signal tdc_ack_meta_r : std_logic := '0';
    signal tdc_ack_sync_r : std_logic := '0';
    signal tdc_ready_meta_r : std_logic := '0';
    signal tdc_ready_sync_r : std_logic := '0';

    signal proc_request_meta_r : std_logic := '0';
    signal proc_request_sync_r : std_logic := '0';
    signal proc_payload_meta_r : command_code_t := C_COMMAND_CLEAR_STATUS;
    signal proc_payload_sync_r : command_code_t := C_COMMAND_CLEAR_STATUS;
    signal proc_state_r : sink_state_t := SINK_IDLE;
    signal proc_init_count_r : natural range 0 to 3 := 0;
    signal proc_clear_r : std_logic := '0';
    signal proc_reset_r : std_logic := '0';

    signal tdc_request_meta_r : std_logic := '0';
    signal tdc_request_sync_r : std_logic := '0';
    signal tdc_payload_meta_r : command_code_t := C_COMMAND_CLEAR_STATUS;
    signal tdc_payload_sync_r : command_code_t := C_COMMAND_CLEAR_STATUS;
    signal tdc_state_r : sink_state_t := SINK_IDLE;
    signal tdc_init_count_r : natural range 0 to 3 := 0;
    signal tdc_clear_r : std_logic := '0';
    signal tdc_reset_r : std_logic := '0';

    signal source_ready_c : std_logic;

    attribute ASYNC_REG : string;
    attribute SHREG_EXTRACT : string;
    attribute ASYNC_REG of proc_ack_meta_r : signal is "TRUE";
    attribute ASYNC_REG of proc_ack_sync_r : signal is "TRUE";
    attribute ASYNC_REG of proc_ready_meta_r : signal is "TRUE";
    attribute ASYNC_REG of proc_ready_sync_r : signal is "TRUE";
    attribute ASYNC_REG of tdc_ack_meta_r : signal is "TRUE";
    attribute ASYNC_REG of tdc_ack_sync_r : signal is "TRUE";
    attribute ASYNC_REG of tdc_ready_meta_r : signal is "TRUE";
    attribute ASYNC_REG of tdc_ready_sync_r : signal is "TRUE";
    attribute ASYNC_REG of proc_request_meta_r : signal is "TRUE";
    attribute ASYNC_REG of proc_request_sync_r : signal is "TRUE";
    attribute ASYNC_REG of proc_payload_meta_r : signal is "TRUE";
    attribute ASYNC_REG of proc_payload_sync_r : signal is "TRUE";
    attribute ASYNC_REG of tdc_request_meta_r : signal is "TRUE";
    attribute ASYNC_REG of tdc_request_sync_r : signal is "TRUE";
    attribute ASYNC_REG of tdc_payload_meta_r : signal is "TRUE";
    attribute ASYNC_REG of tdc_payload_sync_r : signal is "TRUE";
    attribute SHREG_EXTRACT of proc_ack_meta_r : signal is "NO";
    attribute SHREG_EXTRACT of proc_ack_sync_r : signal is "NO";
    attribute SHREG_EXTRACT of proc_ready_meta_r : signal is "NO";
    attribute SHREG_EXTRACT of proc_ready_sync_r : signal is "NO";
    attribute SHREG_EXTRACT of tdc_ack_meta_r : signal is "NO";
    attribute SHREG_EXTRACT of tdc_ack_sync_r : signal is "NO";
    attribute SHREG_EXTRACT of tdc_ready_meta_r : signal is "NO";
    attribute SHREG_EXTRACT of tdc_ready_sync_r : signal is "NO";
    attribute SHREG_EXTRACT of proc_request_meta_r : signal is "NO";
    attribute SHREG_EXTRACT of proc_request_sync_r : signal is "NO";
    attribute SHREG_EXTRACT of proc_payload_meta_r : signal is "NO";
    attribute SHREG_EXTRACT of proc_payload_sync_r : signal is "NO";
    attribute SHREG_EXTRACT of tdc_request_meta_r : signal is "NO";
    attribute SHREG_EXTRACT of tdc_request_sync_r : signal is "NO";
    attribute SHREG_EXTRACT of tdc_payload_meta_r : signal is "NO";
    attribute SHREG_EXTRACT of tdc_payload_sync_r : signal is "NO";

begin

    source_ready_c <= '1' when source_state_r = SOURCE_IDLE and
        proc_ready_sync_r = '1' and tdc_ready_sync_r = '1' else '0';

    o_source_ready    <= source_ready_c;
    o_source_busy     <= '0' when source_state_r = SOURCE_IDLE else '1';
    o_source_rejected <= rejected_r;
    o_proc_clear_status <= proc_clear_r;
    o_proc_soft_reset   <= proc_reset_r;
    o_tdc_clear_status  <= tdc_clear_r;
    o_tdc_soft_reset    <= tdc_reset_r;

    p_source_return_sync : process (i_source_clk, i_source_rst_n)
    begin
        if i_source_rst_n = '0' then
            proc_ack_meta_r <= '0';
            proc_ack_sync_r <= '0';
            proc_ready_meta_r <= '0';
            proc_ready_sync_r <= '0';
            tdc_ack_meta_r <= '0';
            tdc_ack_sync_r <= '0';
            tdc_ready_meta_r <= '0';
            tdc_ready_sync_r <= '0';
        elsif rising_edge(i_source_clk) then
            proc_ack_meta_r <= proc_ack_r;
            proc_ack_sync_r <= proc_ack_meta_r;
            proc_ready_meta_r <= proc_ready_r;
            proc_ready_sync_r <= proc_ready_meta_r;
            tdc_ack_meta_r <= tdc_ack_r;
            tdc_ack_sync_r <= tdc_ack_meta_r;
            tdc_ready_meta_r <= tdc_ready_r;
            tdc_ready_sync_r <= tdc_ready_meta_r;
        end if;
    end process p_source_return_sync;

    p_source : process (i_source_clk, i_source_rst_n)
    begin
        if i_source_rst_n = '0' then
            source_state_r <= SOURCE_IDLE;
            request_r <= '0';
            payload_r <= C_COMMAND_CLEAR_STATUS;
            rejected_r <= '0';
        elsif rising_edge(i_source_clk) then
            rejected_r <= '0';
            case source_state_r is
                when SOURCE_IDLE =>
                    request_r <= '0';
                    if i_clear_status = '1' or i_soft_reset = '1' then
                        if i_clear_status = '1' and i_soft_reset = '1' then
                            rejected_r <= '1';
                        elsif source_ready_c /= '1' then
                            rejected_r <= '1';
                        else
                            if i_soft_reset = '1' then
                                payload_r <= C_COMMAND_SOFT_RESET;
                            else
                                payload_r <= C_COMMAND_CLEAR_STATUS;
                            end if;
                            request_r <= '1';
                            source_state_r <= SOURCE_WAIT_ACK_HIGH;
                        end if;
                    end if;

                when SOURCE_WAIT_ACK_HIGH =>
                    if i_clear_status = '1' or i_soft_reset = '1' then
                        rejected_r <= '1';
                    end if;
                    if proc_ack_sync_r = '1' and tdc_ack_sync_r = '1' then
                        request_r <= '0';
                        source_state_r <= SOURCE_WAIT_ACK_LOW;
                    end if;

                when SOURCE_WAIT_ACK_LOW =>
                    if i_clear_status = '1' or i_soft_reset = '1' then
                        rejected_r <= '1';
                    end if;
                    if proc_ack_sync_r = '0' and tdc_ack_sync_r = '0' then
                        source_state_r <= SOURCE_IDLE;
                    end if;
            end case;
        end if;
    end process p_source;

    p_proc_request_sync : process (i_proc_clk, i_proc_rst_n)
    begin
        if i_proc_rst_n = '0' then
            proc_request_meta_r <= '0';
            proc_request_sync_r <= '0';
            proc_payload_meta_r <= C_COMMAND_CLEAR_STATUS;
            proc_payload_sync_r <= C_COMMAND_CLEAR_STATUS;
        elsif rising_edge(i_proc_clk) then
            proc_request_meta_r <= request_r;
            proc_request_sync_r <= proc_request_meta_r;
            proc_payload_meta_r <= payload_r;
            proc_payload_sync_r <= proc_payload_meta_r;
        end if;
    end process p_proc_request_sync;

    p_proc_sink : process (i_proc_clk, i_proc_rst_n)
    begin
        if i_proc_rst_n = '0' then
            proc_ack_r <= '0';
            proc_ready_r <= '0';
            proc_state_r <= SINK_IDLE;
            proc_init_count_r <= 0;
            proc_clear_r <= '0';
            proc_reset_r <= '0';
        elsif rising_edge(i_proc_clk) then
            proc_clear_r <= '0';
            proc_reset_r <= '0';
            if proc_ready_r = '0' then
                proc_ack_r <= '0';
                if proc_init_count_r = 3 then
                    proc_ready_r <= '1';
                else
                    proc_init_count_r <= proc_init_count_r + 1;
                end if;
            else
                case proc_state_r is
                    when SINK_IDLE =>
                        if proc_request_sync_r = '1' then
                            proc_state_r <= SINK_SETTLE;
                        end if;

                    when SINK_SETTLE =>
                        if proc_request_sync_r = '0' then
                            proc_state_r <= SINK_IDLE;
                        else
                            if proc_payload_sync_r = C_COMMAND_SOFT_RESET then
                                proc_reset_r <= '1';
                            else
                                proc_clear_r <= '1';
                            end if;
                            proc_ack_r <= '1';
                            proc_state_r <= SINK_ACK;
                        end if;

                    when SINK_ACK =>
                        if proc_request_sync_r = '0' then
                            proc_ack_r <= '0';
                            proc_state_r <= SINK_IDLE;
                        end if;
                end case;
            end if;
        end if;
    end process p_proc_sink;

    p_tdc_request_sync : process (i_tdc_clk, i_tdc_rst_n)
    begin
        if i_tdc_rst_n = '0' then
            tdc_request_meta_r <= '0';
            tdc_request_sync_r <= '0';
            tdc_payload_meta_r <= C_COMMAND_CLEAR_STATUS;
            tdc_payload_sync_r <= C_COMMAND_CLEAR_STATUS;
        elsif rising_edge(i_tdc_clk) then
            tdc_request_meta_r <= request_r;
            tdc_request_sync_r <= tdc_request_meta_r;
            tdc_payload_meta_r <= payload_r;
            tdc_payload_sync_r <= tdc_payload_meta_r;
        end if;
    end process p_tdc_request_sync;

    p_tdc_sink : process (i_tdc_clk, i_tdc_rst_n)
    begin
        if i_tdc_rst_n = '0' then
            tdc_ack_r <= '0';
            tdc_ready_r <= '0';
            tdc_state_r <= SINK_IDLE;
            tdc_init_count_r <= 0;
            tdc_clear_r <= '0';
            tdc_reset_r <= '0';
        elsif rising_edge(i_tdc_clk) then
            tdc_clear_r <= '0';
            tdc_reset_r <= '0';
            if tdc_ready_r = '0' then
                tdc_ack_r <= '0';
                if tdc_init_count_r = 3 then
                    tdc_ready_r <= '1';
                else
                    tdc_init_count_r <= tdc_init_count_r + 1;
                end if;
            else
                case tdc_state_r is
                    when SINK_IDLE =>
                        if tdc_request_sync_r = '1' then
                            tdc_state_r <= SINK_SETTLE;
                        end if;

                    when SINK_SETTLE =>
                        if tdc_request_sync_r = '0' then
                            tdc_state_r <= SINK_IDLE;
                        else
                            if tdc_payload_sync_r = C_COMMAND_SOFT_RESET then
                                tdc_reset_r <= '1';
                            else
                                tdc_clear_r <= '1';
                            end if;
                            tdc_ack_r <= '1';
                            tdc_state_r <= SINK_ACK;
                        end if;

                    when SINK_ACK =>
                        if tdc_request_sync_r = '0' then
                            tdc_ack_r <= '0';
                            tdc_state_r <= SINK_IDLE;
                        end if;
                end case;
            end if;
        end if;
    end process p_tdc_sink;

end architecture rtl;
