-- =============================================================================
-- tb_tdc_gpx_header_inserter_widths.vhd
-- Phase A AXI4-Stream width matrix for tdc_gpx_header_inserter
-- =============================================================================
--
-- Purpose:
--   Verifies the output header path for the supported full-keep Phase A widths:
--   32, 64, and 128 bits. Each instance emits one 48-byte header prefix plus
--   one data beat, and every accepted output beat must carry full tkeep/tstrb.
--
-- Standard: VHDL-2008
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.tdc_gpx_pkg.all;

entity tb_tdc_gpx_header_inserter_widths is
end entity tb_tdc_gpx_header_inserter_widths;

architecture sim of tb_tdc_gpx_header_inserter_widths is
    constant C_CLK_PERIOD : time := 5 ns;

    function fn_tb_cfg return t_tdc_cfg is
        variable v_cfg : t_tdc_cfg := c_TDC_CFG_INIT;
    begin
        v_cfg.active_chip_mask := "0001";
        v_cfg.stops_per_chip   := to_unsigned(1, 4);
        v_cfg.cols_per_face    := to_unsigned(1, 16);
        v_cfg.max_hits_cfg     := to_unsigned(7, 3);
        return v_cfg;
    end function;

    signal clk        : std_logic := '0';
    signal rst_n      : std_logic := '0';
    signal face_start : std_logic := '0';
    signal cfg        : t_tdc_cfg := fn_tb_cfg;

    signal s32_tdata  : std_logic_vector(31 downto 0) := (others => '0');
    signal s32_tvalid : std_logic := '0';
    signal s32_tlast  : std_logic := '0';
    signal s32_tready : std_logic;
    signal m32_tdata  : std_logic_vector(31 downto 0);
    signal m32_tkeep  : std_logic_vector(3 downto 0);
    signal m32_tstrb  : std_logic_vector(3 downto 0);
    signal m32_tvalid : std_logic;
    signal m32_tlast  : std_logic;
    signal m32_tuser  : std_logic_vector(0 downto 0);
    signal m32_done   : std_logic;
    signal mon32_beats : integer := 0;
    signal mon32_sof   : integer := 0;
    signal mon32_last  : integer := 0;
    signal mon32_done  : integer := 0;

    signal s64_tdata  : std_logic_vector(63 downto 0) := (others => '0');
    signal s64_tvalid : std_logic := '0';
    signal s64_tlast  : std_logic := '0';
    signal s64_tready : std_logic;
    signal m64_tdata  : std_logic_vector(63 downto 0);
    signal m64_tkeep  : std_logic_vector(7 downto 0);
    signal m64_tstrb  : std_logic_vector(7 downto 0);
    signal m64_tvalid : std_logic;
    signal m64_tlast  : std_logic;
    signal m64_tuser  : std_logic_vector(0 downto 0);
    signal m64_done   : std_logic;
    signal mon64_beats : integer := 0;
    signal mon64_sof   : integer := 0;
    signal mon64_last  : integer := 0;
    signal mon64_done  : integer := 0;

    signal s128_tdata  : std_logic_vector(127 downto 0) := (others => '0');
    signal s128_tvalid : std_logic := '0';
    signal s128_tlast  : std_logic := '0';
    signal s128_tready : std_logic;
    signal m128_tdata  : std_logic_vector(127 downto 0);
    signal m128_tkeep  : std_logic_vector(15 downto 0);
    signal m128_tstrb  : std_logic_vector(15 downto 0);
    signal m128_tvalid : std_logic;
    signal m128_tlast  : std_logic;
    signal m128_tuser  : std_logic_vector(0 downto 0);
    signal m128_done   : std_logic;
    signal mon128_beats : integer := 0;
    signal mon128_sof   : integer := 0;
    signal mon128_last  : integer := 0;
    signal mon128_done  : integer := 0;

begin
    clk <= not clk after C_CLK_PERIOD / 2;

    u_hdr32 : entity work.tdc_gpx_header_inserter
        generic map (g_TDATA_WIDTH => 32)
        port map (
            i_clk => clk,
            i_rst_n => rst_n,
            i_face_start => face_start,
            i_face_abort => '0',
            i_cfg => cfg,
            i_lane_chip_mask => cfg.active_chip_mask,
            i_vdma_frame_id => to_unsigned(1, 32),
            i_face_id => to_unsigned(0, 8),
            i_shot_seq_start => (others => '0'),
            i_timestamp_ns => (others => '0'),
            i_chip_error_mask => (others => '0'),
            i_chip_error_cnt => (others => '0'),
            i_bin_resolution_ps => to_unsigned(81, 16),
            i_k_dist_fixed => (others => '0'),
            i_rows_per_face => to_unsigned(1, 16),
            i_s_axis_tdata => s32_tdata,
            i_s_axis_tvalid => s32_tvalid,
            i_s_axis_tlast => s32_tlast,
            o_s_axis_tready => s32_tready,
            o_m_axis_tdata => m32_tdata,
            o_m_axis_tkeep => m32_tkeep,
            o_m_axis_tstrb => m32_tstrb,
            o_m_axis_tvalid => m32_tvalid,
            o_m_axis_tlast => m32_tlast,
            o_m_axis_tuser => m32_tuser,
            i_m_axis_tready => '1',
            o_frame_done => m32_done,
            o_draining => open,
            o_last_line => open,
            o_idle => open,
            o_face_start_collapsed_count => open,
            o_drain_timeout_sticky => open,
            o_abort_truncated_sticky => open,
            o_frame_done_faulted => open
        );

    u_hdr64 : entity work.tdc_gpx_header_inserter
        generic map (g_TDATA_WIDTH => 64)
        port map (
            i_clk => clk,
            i_rst_n => rst_n,
            i_face_start => face_start,
            i_face_abort => '0',
            i_cfg => cfg,
            i_lane_chip_mask => cfg.active_chip_mask,
            i_vdma_frame_id => to_unsigned(1, 32),
            i_face_id => to_unsigned(0, 8),
            i_shot_seq_start => (others => '0'),
            i_timestamp_ns => (others => '0'),
            i_chip_error_mask => (others => '0'),
            i_chip_error_cnt => (others => '0'),
            i_bin_resolution_ps => to_unsigned(81, 16),
            i_k_dist_fixed => (others => '0'),
            i_rows_per_face => to_unsigned(1, 16),
            i_s_axis_tdata => s64_tdata,
            i_s_axis_tvalid => s64_tvalid,
            i_s_axis_tlast => s64_tlast,
            o_s_axis_tready => s64_tready,
            o_m_axis_tdata => m64_tdata,
            o_m_axis_tkeep => m64_tkeep,
            o_m_axis_tstrb => m64_tstrb,
            o_m_axis_tvalid => m64_tvalid,
            o_m_axis_tlast => m64_tlast,
            o_m_axis_tuser => m64_tuser,
            i_m_axis_tready => '1',
            o_frame_done => m64_done,
            o_draining => open,
            o_last_line => open,
            o_idle => open,
            o_face_start_collapsed_count => open,
            o_drain_timeout_sticky => open,
            o_abort_truncated_sticky => open,
            o_frame_done_faulted => open
        );

    u_hdr128 : entity work.tdc_gpx_header_inserter
        generic map (g_TDATA_WIDTH => 128)
        port map (
            i_clk => clk,
            i_rst_n => rst_n,
            i_face_start => face_start,
            i_face_abort => '0',
            i_cfg => cfg,
            i_lane_chip_mask => cfg.active_chip_mask,
            i_vdma_frame_id => to_unsigned(1, 32),
            i_face_id => to_unsigned(0, 8),
            i_shot_seq_start => (others => '0'),
            i_timestamp_ns => (others => '0'),
            i_chip_error_mask => (others => '0'),
            i_chip_error_cnt => (others => '0'),
            i_bin_resolution_ps => to_unsigned(81, 16),
            i_k_dist_fixed => (others => '0'),
            i_rows_per_face => to_unsigned(1, 16),
            i_s_axis_tdata => s128_tdata,
            i_s_axis_tvalid => s128_tvalid,
            i_s_axis_tlast => s128_tlast,
            o_s_axis_tready => s128_tready,
            o_m_axis_tdata => m128_tdata,
            o_m_axis_tkeep => m128_tkeep,
            o_m_axis_tstrb => m128_tstrb,
            o_m_axis_tvalid => m128_tvalid,
            o_m_axis_tlast => m128_tlast,
            o_m_axis_tuser => m128_tuser,
            i_m_axis_tready => '1',
            o_frame_done => m128_done,
            o_draining => open,
            o_last_line => open,
            o_idle => open,
            o_face_start_collapsed_count => open,
            o_drain_timeout_sticky => open,
            o_abort_truncated_sticky => open,
            o_frame_done_faulted => open
        );

    p_drive32 : process
    begin
        wait until rst_n = '1';
        wait until rising_edge(clk);
        while s32_tready = '0' loop
            wait until rising_edge(clk);
        end loop;
        s32_tdata  <= x"A5A50032";
        s32_tvalid <= '1';
        s32_tlast  <= '1';
        wait until rising_edge(clk);
        while s32_tready = '0' loop
            wait until rising_edge(clk);
        end loop;
        s32_tvalid <= '0';
        s32_tlast  <= '0';
        wait;
    end process;

    p_drive64 : process
    begin
        wait until rst_n = '1';
        wait until rising_edge(clk);
        while s64_tready = '0' loop
            wait until rising_edge(clk);
        end loop;
        s64_tdata  <= x"00000000A5A50064";
        s64_tvalid <= '1';
        s64_tlast  <= '1';
        wait until rising_edge(clk);
        while s64_tready = '0' loop
            wait until rising_edge(clk);
        end loop;
        s64_tvalid <= '0';
        s64_tlast  <= '0';
        wait;
    end process;

    p_drive128 : process
    begin
        wait until rst_n = '1';
        wait until rising_edge(clk);
        while s128_tready = '0' loop
            wait until rising_edge(clk);
        end loop;
        s128_tdata <= (others => '0');
        s128_tdata(31 downto 0) <= x"A5A51280";
        s128_tvalid <= '1';
        s128_tlast  <= '1';
        wait until rising_edge(clk);
        while s128_tready = '0' loop
            wait until rising_edge(clk);
        end loop;
        s128_tvalid <= '0';
        s128_tlast  <= '0';
        wait;
    end process;

    p_monitor : process(clk)
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                mon32_beats <= 0;
                mon32_sof   <= 0;
                mon32_last  <= 0;
                mon32_done  <= 0;
                mon64_beats <= 0;
                mon64_sof   <= 0;
                mon64_last  <= 0;
                mon64_done  <= 0;
                mon128_beats <= 0;
                mon128_sof   <= 0;
                mon128_last  <= 0;
                mon128_done  <= 0;
            else
                if m32_tvalid = '1' then
                    assert m32_tkeep = "1111" report "32-bit tkeep is not full on accepted beat" severity error;
                    assert m32_tstrb = "1111" report "32-bit tstrb is not full on accepted beat" severity error;
                    if mon32_beats = 0 then
                        assert m32_tuser(0) = '1' report "32-bit first beat must assert SOF" severity error;
                        assert m32_tdata(31 downto 0) = c_HEADER_MAGIC report "32-bit first beat must carry header magic" severity error;
                    end if;
                    if m32_tuser(0) = '1' then
                        mon32_sof <= mon32_sof + 1;
                    end if;
                    if m32_tlast = '1' then
                        mon32_last <= mon32_last + 1;
                    end if;
                    mon32_beats <= mon32_beats + 1;
                end if;

                if m64_tvalid = '1' then
                    assert m64_tkeep = x"FF" report "64-bit tkeep is not full on accepted beat" severity error;
                    assert m64_tstrb = x"FF" report "64-bit tstrb is not full on accepted beat" severity error;
                    if mon64_beats = 0 then
                        assert m64_tuser(0) = '1' report "64-bit first beat must assert SOF" severity error;
                        assert m64_tdata(31 downto 0) = c_HEADER_MAGIC report "64-bit first beat must carry header magic" severity error;
                    end if;
                    if m64_tuser(0) = '1' then
                        mon64_sof <= mon64_sof + 1;
                    end if;
                    if m64_tlast = '1' then
                        mon64_last <= mon64_last + 1;
                    end if;
                    mon64_beats <= mon64_beats + 1;
                end if;

                if m128_tvalid = '1' then
                    assert m128_tkeep = x"FFFF" report "128-bit tkeep is not full on accepted beat" severity error;
                    assert m128_tstrb = x"FFFF" report "128-bit tstrb is not full on accepted beat" severity error;
                    if mon128_beats = 0 then
                        assert m128_tuser(0) = '1' report "128-bit first beat must assert SOF" severity error;
                        assert m128_tdata(31 downto 0) = c_HEADER_MAGIC report "128-bit first beat must carry header magic" severity error;
                    end if;
                    if m128_tuser(0) = '1' then
                        mon128_sof <= mon128_sof + 1;
                    end if;
                    if m128_tlast = '1' then
                        mon128_last <= mon128_last + 1;
                    end if;
                    mon128_beats <= mon128_beats + 1;
                end if;

                if m32_done = '1' then
                    mon32_done <= mon32_done + 1;
                end if;
                if m64_done = '1' then
                    mon64_done <= mon64_done + 1;
                end if;
                if m128_done = '1' then
                    mon128_done <= mon128_done + 1;
                end if;
            end if;
        end if;
    end process;

    p_stim : process
    begin
        assert fn_output_width_supported(32) report "32-bit output width should be supported" severity failure;
        assert fn_output_width_supported(64) report "64-bit output width should be supported" severity failure;
        assert fn_output_width_supported(128) report "128-bit output width should be supported" severity failure;
        assert not fn_output_width_supported(256) report "256-bit output width is intentionally excluded" severity failure;

        wait for 10 * C_CLK_PERIOD;
        rst_n <= '1';
        wait until rising_edge(clk);
        face_start <= '1';
        wait until rising_edge(clk);
        face_start <= '0';

        wait for 2 us;

        assert mon32_beats = fn_hdr_prefix_beats(32) + 1 report "32-bit beat count mismatch" severity failure;
        assert mon64_beats = fn_hdr_prefix_beats(64) + 1 report "64-bit beat count mismatch" severity failure;
        assert mon128_beats = fn_hdr_prefix_beats(128) + 1 report "128-bit beat count mismatch" severity failure;

        assert mon32_sof = 1 and mon64_sof = 1 and mon128_sof = 1 report "SOF count mismatch" severity failure;
        assert mon32_last = 1 and mon64_last = 1 and mon128_last = 1 report "TLAST count mismatch" severity failure;
        assert mon32_done = 1 and mon64_done = 1 and mon128_done = 1 report "frame_done count mismatch" severity failure;

        report "*** tb_tdc_gpx_header_inserter_widths PASS ***" severity note;
        std.env.finish;
    end process;

end architecture sim;
