library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- Physical fire_done to TDC-GPX START boundary.
--
-- This is the deliberate low-latency exception in the Processing pipeline.
-- The raw fire_done pin asynchronously presets one capture register, so an
-- armed physical shot asserts START without waiting for a Processing clock.
-- All bookkeeping leaves this entity through synchronized events.
--
-- A new arm is legal only after raw fire_done has been observed LOW on two
-- consecutive synchronized samples. This prevents a stale HIGH input from
-- becoming a false T0. The configured width is snapshotted before arm and the
-- clocked release guarantees at least that many complete Processing periods.
-- A completion gate also bounds START when a faulty fire_done input remains
-- HIGH after its rising edge; re-arm remains blocked until raw LOW is observed.
entity laser_fire_done_bridge is
    port (
        i_clk                    : in  std_logic;
        i_rst_n                  : in  std_logic;
        i_fire_done_raw          : in  std_logic;
        i_physical_arm           : in  std_logic;
        i_start_width_clks       : in  unsigned(31 downto 0);

        o_fire_done_ready        : out std_logic;
        o_start_tdc              : out std_logic;
        o_start_busy             : out std_logic;
        o_t0_event               : out std_logic;
        o_unexpected_done_pulse  : out std_logic
    );
end entity laser_fire_done_bridge;

architecture rtl of laser_fire_done_bridge is

    signal raw_meta_r       : std_logic := '1';
    signal raw_sync_r       : std_logic := '1';
    signal raw_sync_d_r     : std_logic := '1';
    signal prearm_ready_r   : std_logic := '0';
    signal start_width_r    : unsigned(31 downto 0) := (others => '0');

    signal start_capture_r  : std_logic := '0';
    signal start_active_c   : std_logic;
    signal hold_active_r    : std_logic := '0';
    signal hold_count_r     : unsigned(31 downto 0) := (others => '0');
    signal pulse_complete_r : std_logic := '0';
    signal release_c        : std_logic;

    signal start_meta_r     : std_logic := '0';
    signal start_sync_r     : std_logic := '0';
    signal start_sync_d_r   : std_logic := '0';
    signal t0_event_c       : std_logic;
    signal unexpected_r     : std_logic := '0';

    attribute ASYNC_REG : string;
    attribute SHREG_EXTRACT : string;
    attribute ASYNC_REG of raw_meta_r   : signal is "TRUE";
    attribute ASYNC_REG of raw_sync_r   : signal is "TRUE";
    attribute ASYNC_REG of start_meta_r : signal is "TRUE";
    attribute ASYNC_REG of start_sync_r : signal is "TRUE";
    attribute SHREG_EXTRACT of raw_meta_r   : signal is "NO";
    attribute SHREG_EXTRACT of raw_sync_r   : signal is "NO";
    attribute SHREG_EXTRACT of start_meta_r : signal is "NO";
    attribute SHREG_EXTRACT of start_sync_r : signal is "NO";

begin

    -- Arm is removed synchronously by the executor. Once a START has reached a
    -- clock edge, hold_active preserves its minimum width even if permission is
    -- revoked while the pulse is active.
    start_active_c <= start_capture_r and prearm_ready_r and
        (i_physical_arm or hold_active_r) and not pulse_complete_r and
        i_rst_n;
    release_c <= '1' when hold_active_r = '1' and hold_count_r = 0 else '0';
    t0_event_c <= start_sync_r and not start_sync_d_r;

    o_fire_done_ready <= prearm_ready_r and not i_physical_arm and
        not start_capture_r and not pulse_complete_r and i_rst_n;
    o_start_tdc             <= start_active_c;
    o_start_busy            <= start_active_c;
    o_t0_event              <= t0_event_c;
    o_unexpected_done_pulse <= unexpected_r;

    p_raw_qualification : process (i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                raw_meta_r     <= '1';
                raw_sync_r     <= '1';
                raw_sync_d_r   <= '1';
                prearm_ready_r <= '0';
                start_width_r  <= (others => '0');
                unexpected_r   <= '0';
            else
                raw_meta_r   <= i_fire_done_raw;
                raw_sync_r   <= raw_meta_r;
                raw_sync_d_r <= raw_sync_r;
                unexpected_r <= '0';

                if raw_sync_r = '1' and raw_sync_d_r = '0' and
                   i_physical_arm /= '1' and hold_active_r /= '1' then
                    unexpected_r <= '1';
                end if;

                if i_physical_arm = '1' or hold_active_r = '1' then
                    null;
                elsif start_capture_r = '1' then
                    prearm_ready_r <= '0';
                else
                    start_width_r <= i_start_width_clks;
                    if raw_sync_r = '0' and raw_sync_d_r = '0' and
                       i_start_width_clks /= 0 then
                        prearm_ready_r <= '1';
                    else
                        prearm_ready_r <= '0';
                    end if;
                end if;
            end if;
        end if;
    end process p_raw_qualification;

    -- Raw fire_done is intentionally the only asynchronous preset source.
    -- Qualification and arm remain on the output side, avoiding a LUT in the
    -- capture preset path on 7-series devices.
    p_async_capture : process (i_clk, i_fire_done_raw)
    begin
        if i_fire_done_raw = '1' then
            start_capture_r <= '1';
        elsif rising_edge(i_clk) then
            if i_rst_n = '0' or release_c = '1' or
               (i_physical_arm = '0' and hold_active_r = '0') then
                start_capture_r <= '0';
            end if;
        end if;
    end process p_async_capture;

    p_minimum_width : process (i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                hold_active_r <= '0';
                hold_count_r  <= (others => '0');
                pulse_complete_r <= '0';
            elsif i_physical_arm = '0' and start_capture_r = '0' then
                hold_active_r    <= '0';
                hold_count_r     <= (others => '0');
                pulse_complete_r <= '0';
            elsif start_active_c = '0' then
                hold_active_r <= '0';
                hold_count_r  <= (others => '0');
            elsif hold_active_r = '0' then
                hold_active_r <= '1';
                hold_count_r  <= start_width_r - 1;
            elsif hold_count_r /= 0 then
                hold_count_r <= hold_count_r - 1;
            else
                hold_active_r    <= '0';
                pulse_complete_r <= '1';
            end if;
        end if;
    end process p_minimum_width;

    -- The synchronized event is the only bridge signal consumed by the
    -- Processing FSM. Edge detection is made from registered synchronizer
    -- stages; the asynchronous physical START path does not wait for them.
    p_start_synchronizer : process (i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                start_meta_r   <= '0';
                start_sync_r   <= '0';
                start_sync_d_r <= '0';
            else
                start_meta_r   <= start_active_c;
                start_sync_r   <= start_meta_r;
                start_sync_d_r <= start_sync_r;
            end if;
        end if;
    end process p_start_synchronizer;

end architecture rtl;
