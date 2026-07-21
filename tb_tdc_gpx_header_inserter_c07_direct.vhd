--------------------------------------------------------------------------------
-- tb_tdc_gpx_header_inserter_c07_direct.vhd
--
-- C07 CHAIN-P1-02 direct regression for C04 header_inserter pending/stall.
--
-- Scenario:
--   - face 1 starts while output tready is low, stressing ST_PREFIX hold
--   - final data beat is loaded with output tready low, entering ST_DRAIN_LAST
--   - a second face_start pulse arrives while ST_DRAIN_LAST is holding tlast
--   - tready is released; frame 1 completes, queued face 2 starts and completes
--
-- Standard: VHDL-2008
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use std.env.all;

use work.tdc_gpx_pkg.all;

entity tb_tdc_gpx_header_inserter_c07_direct is
    generic (
        G_TDATA_WIDTH : natural := 64
    );
end entity tb_tdc_gpx_header_inserter_c07_direct;

architecture sim of tb_tdc_gpx_header_inserter_c07_direct is

    constant c_CLK_PERIOD       : time := 5 ns;
    constant c_HDR_PREFIX_BEATS : natural := fn_hdr_prefix_beats(G_TDATA_WIDTH);
    constant c_KEEP_WIDTH       : natural := fn_axis_keep_width(G_TDATA_WIDTH);
    constant c_EXPECTED_BEATS   : natural := 2 * (c_HDR_PREFIX_BEATS + 1);

    function fn_tb_cfg return t_tdc_cfg is
        variable v_cfg : t_tdc_cfg := c_TDC_CFG_INIT;
    begin
        v_cfg.active_chip_mask := "0001";
        v_cfg.stops_per_chip   := to_unsigned(1, 4);
        v_cfg.cols_per_face    := to_unsigned(1, 16);
        v_cfg.max_hits_cfg     := to_unsigned(7, 3);
        return v_cfg;
    end function;

    signal s_clk   : std_logic := '0';
    signal s_rst_n : std_logic := '0';
    signal s_done  : boolean := false;

    signal s_face_start : std_logic := '0';
    signal s_face_abort : std_logic := '0';
    signal s_cfg        : t_tdc_cfg := fn_tb_cfg;

    signal s_s_tdata  : std_logic_vector(G_TDATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_s_tvalid : std_logic := '0';
    signal s_s_tlast  : std_logic := '0';
    signal s_s_tready : std_logic;

    signal m_tdata  : std_logic_vector(G_TDATA_WIDTH - 1 downto 0);
    signal m_tkeep  : std_logic_vector(c_KEEP_WIDTH - 1 downto 0);
    signal m_tstrb  : std_logic_vector(c_KEEP_WIDTH - 1 downto 0);
    signal m_tvalid : std_logic;
    signal m_tlast  : std_logic;
    signal m_tuser  : std_logic_vector(0 downto 0);
    signal m_tready : std_logic := '1';

    signal s_frame_done      : std_logic;
    signal s_draining        : std_logic;
    signal s_last_line       : std_logic;
    signal s_idle            : std_logic;
    signal s_collapsed_count : unsigned(7 downto 0);
    signal s_drain_timeout   : std_logic;
    signal s_abort_truncated : std_logic;
    signal s_frame_faulted   : std_logic;

    signal s_out_count        : natural := 0;
    signal s_sof_count        : natural := 0;
    signal s_tlast_count      : natural := 0;
    signal s_frame_done_count : natural := 0;
    signal s_keep_ok          : boolean := true;
    signal s_magic_ok         : boolean := true;
    signal s_done_after_tlast : boolean := true;

begin

    s_clk <= not s_clk after c_CLK_PERIOD / 2;

    u_dut : entity work.tdc_gpx_header_inserter
        generic map (
            g_OUTPUT_WIDTH => G_TDATA_WIDTH
        )
        port map (
            i_clk               => s_clk,
            i_rst_n             => s_rst_n,
            i_face_start        => s_face_start,
            i_face_abort        => s_face_abort,
            i_cfg               => s_cfg,
            i_lane_chip_mask    => s_cfg.active_chip_mask,
            i_vdma_frame_id     => to_unsigned(16#1234#, 32),
            i_face_id           => to_unsigned(3, 8),
            i_shot_seq_start    => to_unsigned(16#55AA#, c_SHOT_SEQ_WIDTH),
            i_timestamp_ns      => x"0000000012345678",
            i_chip_error_mask   => (others => '0'),
            i_chip_error_cnt    => (others => '0'),
            i_bin_resolution_ps => to_unsigned(81, 16),
            i_k_dist_fixed      => x"00010000",
            i_rows_per_face     => to_unsigned(1, 16),
            i_s_axis_tdata      => s_s_tdata,
            i_s_axis_tvalid     => s_s_tvalid,
            i_s_axis_tlast      => s_s_tlast,
            o_s_axis_tready     => s_s_tready,
            o_m_axis_tdata      => m_tdata,
            o_m_axis_tkeep      => m_tkeep,
            o_m_axis_tstrb      => m_tstrb,
            o_m_axis_tvalid     => m_tvalid,
            o_m_axis_tlast      => m_tlast,
            o_m_axis_tuser      => m_tuser,
            i_m_axis_tready     => m_tready,
            o_frame_done        => s_frame_done,
            o_draining          => s_draining,
            o_last_line         => s_last_line,
            o_idle              => s_idle,
            o_face_start_collapsed_count => s_collapsed_count,
            o_drain_timeout_sticky       => s_drain_timeout,
            o_abort_truncated_sticky     => s_abort_truncated,
            o_frame_done_faulted         => s_frame_faulted
        );

    p_monitor : process(s_clk)
    begin
        if rising_edge(s_clk) then
            if s_rst_n = '0' then
                s_out_count        <= 0;
                s_sof_count        <= 0;
                s_tlast_count      <= 0;
                s_frame_done_count <= 0;
                s_keep_ok          <= true;
                s_magic_ok         <= true;
                s_done_after_tlast <= true;
            else
                if m_tvalid = '1' and m_tready = '1' then
                    if m_tkeep /= (m_tkeep'range => '1')
                       or m_tstrb /= (m_tstrb'range => '1') then
                        s_keep_ok <= false;
                    end if;
                    if m_tuser(0) = '1' then
                        s_sof_count <= s_sof_count + 1;
                        if m_tdata(31 downto 0) /= c_HEADER_MAGIC then
                            s_magic_ok <= false;
                        end if;
                    end if;
                    if m_tlast = '1' then
                        s_tlast_count <= s_tlast_count + 1;
                    end if;
                    s_out_count <= s_out_count + 1;
                end if;

                if s_frame_done = '1' then
                    if s_tlast_count = 0 and m_tlast = '0' then
                        s_done_after_tlast <= false;
                    end if;
                    s_frame_done_count <= s_frame_done_count + 1;
                end if;
            end if;
        end if;
    end process p_monitor;

    p_stim : process
        procedure wait_cycles(n : natural) is
        begin
            for i in 1 to n loop
                wait until rising_edge(s_clk);
            end loop;
            wait for 0 ns;
        end procedure;

        procedure pulse_face_start is
        begin
            s_face_start <= '1';
            wait_cycles(1);
            s_face_start <= '0';
        end procedure;

        procedure send_line(word : natural) is
            variable v_timeout : natural := 0;
            variable v_data    : std_logic_vector(G_TDATA_WIDTH - 1 downto 0);
        begin
            v_data := (others => '0');
            v_data(31 downto 0) := std_logic_vector(to_unsigned(word, 32));
            s_s_tdata  <= v_data;
            s_s_tvalid <= '1';
            s_s_tlast  <= '1';
            loop
                wait_cycles(1);
                exit when s_s_tready = '1';
                v_timeout := v_timeout + 1;
                assert v_timeout < 5000
                    report "FAIL: header_inserter slave tready timeout"
                    severity failure;
            end loop;
            s_s_tvalid <= '0';
            s_s_tlast  <= '0';
            s_s_tdata  <= (others => '0');
        end procedure;
    begin
        assert fn_output_width_supported(G_TDATA_WIDTH)
            report "FAIL: unsupported G_TDATA_WIDTH"
            severity failure;

        s_rst_n <= '0';
        m_tready <= '1';
        wait_cycles(10);
        s_rst_n <= '1';
        wait_cycles(5);

        -- Face 1: hold output before prefix can drain.
        m_tready <= '0';
        pulse_face_start;
        wait_cycles(10);
        assert s_idle = '0'
            report "FAIL: header did not leave IDLE after face_start"
            severity failure;

        -- Release prefix/header drain, then load final data beat while
        -- downstream is low so ST_DRAIN_LAST holds a real TLAST beat.
        m_tready <= '1';
        wait until s_s_tready = '1' for 20 us;
        assert s_s_tready = '1'
            report "FAIL: header did not reach ST_DATA for face 1"
            severity failure;
        wait_cycles(1);
        m_tready <= '0';
        send_line(16#25A50001#);
        wait until s_draining = '1' for 20 us;
        assert s_draining = '1'
            report "FAIL: header did not enter ST_DRAIN_LAST"
            severity failure;

        -- Pending face_start collision: legal upstream pulse while final
        -- TLAST is still pending. It must be queued, not lost.
        pulse_face_start;
        wait_cycles(5);
        assert s_frame_done_count = 0
            report "FAIL: frame_done occurred before releasing final TLAST"
            severity failure;

        m_tready <= '1';
        wait until s_frame_done_count = 1 for 20 us;
        assert s_frame_done_count = 1
            report "FAIL: first frame_done missing after releasing tready"
            severity failure;

        -- Queued face 2 should start automatically. Feed its single line.
        wait until s_s_tready = '1' for 20 us;
        assert s_s_tready = '1'
            report "FAIL: queued face_start did not reach ST_DATA for face 2"
            severity failure;
        send_line(16#25A50002#);
        wait until s_frame_done_count = 2 for 20 us;
        wait_cycles(10);

        assert s_out_count = c_EXPECTED_BEATS
            report "FAIL: header output beat count mismatch, got " &
                   integer'image(s_out_count) &
                   " expected " & integer'image(c_EXPECTED_BEATS)
            severity failure;
        assert s_sof_count = 2
            report "FAIL: header SOF count mismatch"
            severity failure;
        assert s_tlast_count = 2
            report "FAIL: header TLAST count mismatch"
            severity failure;
        assert s_frame_done_count = 2
            report "FAIL: header frame_done count mismatch"
            severity failure;
        assert s_collapsed_count = 0
            report "FAIL: one queued face_start should not increment collapsed_count"
            severity failure;
        assert s_keep_ok and s_magic_ok and s_done_after_tlast
            report "FAIL: header output protocol marker mismatch"
            severity failure;
        assert s_drain_timeout = '0' and s_abort_truncated = '0' and s_frame_faulted = '0'
            report "FAIL: unexpected header fault sticky/pulse"
            severity failure;

        report "PASS: C07 C04 header pending/stall width=" &
               integer'image(G_TDATA_WIDTH) &
               " beats=" & integer'image(c_EXPECTED_BEATS)
            severity note;

        s_done <= true;
        wait_cycles(2);
        finish;
        wait;
    end process p_stim;

    p_watchdog : process
    begin
        wait for 200 us;
        assert s_done
            report "FAIL: tb_tdc_gpx_header_inserter_c07_direct watchdog timeout"
            severity failure;
        wait;
    end process p_watchdog;

end architecture sim;
