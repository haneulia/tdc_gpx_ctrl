library ieee;
use ieee.std_logic_1164.all;

use work.lidar_build_pkg.all;

-- Processing-to-TDC one-shot gateway for stop_tdc. The Processing pulse may
-- be wider than one clock, but exactly one event is delivered for each rising
-- edge. ASYNC mode uses a one-bit FIFO; SYNC mode uses a registered edge pulse
-- and requires both ports to share the same physical clock/reset.
entity lidar_gpx_stop_gateway is
    generic (
        G_BUILD_CONFIG : lidar_build_config_t := C_DEFAULT_BUILD_CONFIG;
        G_FIFO_DEPTH   : positive := 16
    );
    port (
        i_proc_clk        : in  std_logic;
        i_proc_rst_n      : in  std_logic;
        i_proc_stop_tdc   : in  std_logic;
        i_clear_status    : in  std_logic := '0';
        o_overflow_sticky : out std_logic;

        i_tdc_clk         : in  std_logic;
        i_tdc_rst_n       : in  std_logic;
        o_tdc_stop_tdc    : out std_logic;

        o_proc_reset_busy : out std_logic;
        o_tdc_reset_busy  : out std_logic;
        o_reset_busy      : out std_logic
    );
end entity lidar_gpx_stop_gateway;

architecture rtl of lidar_gpx_stop_gateway is

    signal stop_tdc_c : std_logic := '0';
    signal proc_reset_busy_c : std_logic := '0';
    signal tdc_reset_busy_c : std_logic := '0';
    signal reset_busy_c : std_logic := '0';
    signal overflow_c : std_logic := '0';

begin

    assert fn_validate_build_config(G_BUILD_CONFIG) = CFG_OK
        report "V2-GPX-STOP-001 invalid build configuration"
        severity failure;

    o_tdc_stop_tdc <= stop_tdc_c;
    o_proc_reset_busy <= proc_reset_busy_c;
    o_tdc_reset_busy <= tdc_reset_busy_c;
    o_reset_busy <= reset_busy_c;
    o_overflow_sticky <= overflow_c;

    gen_sync : if G_BUILD_CONFIG.stream_clock_mode = STREAM_CLOCK_SYNC generate
        signal stop_prev_r : std_logic := '0';
        signal stop_pulse_r : std_logic := '0';
    begin
        p_sync_edge : process (i_proc_clk, i_proc_rst_n)
        begin
            if i_proc_rst_n = '0' then
                stop_prev_r <= '0';
                stop_pulse_r <= '0';
            elsif rising_edge(i_proc_clk) then
                stop_pulse_r <= i_proc_stop_tdc and not stop_prev_r;
                stop_prev_r <= i_proc_stop_tdc;
            end if;
        end process p_sync_edge;

        stop_tdc_c <= stop_pulse_r;
        proc_reset_busy_c <= '0';
        tdc_reset_busy_c <= '0';
        reset_busy_c <= '0';
        overflow_c <= '0';
    end generate gen_sync;

    gen_async : if G_BUILD_CONFIG.stream_clock_mode = STREAM_CLOCK_ASYNC generate
        signal stop_prev_r : std_logic := '0';
        signal pending_r : std_logic := '0';
        signal overflow_r : std_logic := '0';
        signal source_ready_c : std_logic;
        signal destination_valid_c : std_logic;
        signal destination_data_c : std_logic_vector(0 downto 0);
    begin
        p_source_capture : process (i_proc_clk, i_proc_rst_n)
            variable event_now : boolean;
        begin
            if i_proc_rst_n = '0' then
                stop_prev_r <= '0';
                pending_r <= '0';
                overflow_r <= '0';
            elsif rising_edge(i_proc_clk) then
                event_now := i_proc_stop_tdc = '1' and stop_prev_r = '0';
                stop_prev_r <= i_proc_stop_tdc;

                if i_clear_status = '1' then
                    overflow_r <= '0';
                end if;

                if pending_r = '1' and source_ready_c = '1' then
                    pending_r <= '0';
                end if;

                if event_now then
                    if pending_r = '1' and source_ready_c = '0' then
                        overflow_r <= '1';
                    else
                        pending_r <= '1';
                    end if;
                end if;
            end if;
        end process p_source_capture;

        u_gateway : entity work.lidar_stream_gateway
            generic map (
                G_WIDTH      => 1,
                G_FIFO_DEPTH => G_FIFO_DEPTH,
                G_CLOCK_MODE => STREAM_CLOCK_ASYNC
            )
            port map (
                i_source_clk        => i_proc_clk,
                i_source_rst_n      => i_proc_rst_n,
                i_source_valid      => pending_r,
                o_source_ready      => source_ready_c,
                i_source_data       => "1",
                i_destination_clk   => i_tdc_clk,
                i_destination_rst_n => i_tdc_rst_n,
                o_destination_valid => destination_valid_c,
                i_destination_ready => '1',
                o_destination_data  => destination_data_c,
                o_source_reset_busy => proc_reset_busy_c,
                o_destination_reset_busy => tdc_reset_busy_c,
                o_reset_busy        => reset_busy_c
            );

        stop_tdc_c <= destination_valid_c and destination_data_c(0);
        overflow_c <= overflow_r;
    end generate gen_async;

end architecture rtl;
