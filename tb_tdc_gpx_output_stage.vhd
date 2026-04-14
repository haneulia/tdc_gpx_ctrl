-- =============================================================================
-- tb_tdc_gpx_output_stage.vhd
-- Testbench for tdc_gpx_output_stage (Cluster 4 wrapper)
-- =============================================================================
--
-- Purpose:
--   Smoke-test: feed cell data on chip 0 rising input, verify that
--   face_assembler picks it up, header is prepended, VDMA output fires,
--   and frame_done asserts.
--
-- Configuration:
--   g_OUTPUT_WIDTH   = 64
--   g_ALU_PULSE_CLKS = 4
--   1 active chip (mask "0001"), 2 stops, 2 rows
--
-- Standard: VHDL-2008
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.tdc_gpx_pkg.all;
use work.tdc_gpx_cfg_pkg.all;

entity tb_tdc_gpx_output_stage is
end entity tb_tdc_gpx_output_stage;

architecture sim of tb_tdc_gpx_output_stage is

    -- =========================================================================
    -- Constants
    -- =========================================================================
    constant C_OUTPUT_WIDTH   : natural := 64;
    constant C_ALU_PULSE_CLKS : natural := 4;
    constant C_CLK_PERIOD     : time    := 5 ns;   -- 200 MHz
    constant C_WATCHDOG       : time    := 50 us;

    -- Beats per cell at 64-bit width: c_CELL_SIZE_BYTES / 8 = 32/8 = 4
    constant C_BEATS_PER_CELL : natural := fn_beats_per_cell(C_OUTPUT_WIDTH);
    constant C_STOPS          : natural := 2;
    constant C_TOTAL_BEATS    : natural := C_STOPS * C_BEATS_PER_CELL;  -- 2*4 = 8

    -- =========================================================================
    -- DUT signals
    -- =========================================================================
    signal clk   : std_logic := '0';
    signal rst_n : std_logic := '0';

    -- Cell rise inputs (chip 0..3)
    signal cell_rise_tdata_0 : std_logic_vector(C_OUTPUT_WIDTH - 1 downto 0) := (others => '0');
    signal cell_rise_tdata_1 : std_logic_vector(C_OUTPUT_WIDTH - 1 downto 0) := (others => '0');
    signal cell_rise_tdata_2 : std_logic_vector(C_OUTPUT_WIDTH - 1 downto 0) := (others => '0');
    signal cell_rise_tdata_3 : std_logic_vector(C_OUTPUT_WIDTH - 1 downto 0) := (others => '0');
    signal cell_rise_tvalid  : std_logic_vector(3 downto 0) := (others => '0');
    signal cell_rise_tlast   : std_logic_vector(3 downto 0) := (others => '0');
    signal cell_rise_tready  : std_logic_vector(3 downto 0);

    -- Cell fall inputs (all tied low)
    signal cell_fall_tdata_0 : std_logic_vector(C_OUTPUT_WIDTH - 1 downto 0) := (others => '0');
    signal cell_fall_tdata_1 : std_logic_vector(C_OUTPUT_WIDTH - 1 downto 0) := (others => '0');
    signal cell_fall_tdata_2 : std_logic_vector(C_OUTPUT_WIDTH - 1 downto 0) := (others => '0');
    signal cell_fall_tdata_3 : std_logic_vector(C_OUTPUT_WIDTH - 1 downto 0) := (others => '0');
    signal cell_fall_tvalid  : std_logic_vector(3 downto 0) := (others => '0');
    signal cell_fall_tlast   : std_logic_vector(3 downto 0) := (others => '0');
    signal cell_fall_tready  : std_logic_vector(3 downto 0);

    -- Control
    signal shot_start_gated  : std_logic := '0';
    signal pipeline_abort    : std_logic := '0';
    signal face_start_gated  : std_logic := '0';

    -- Configuration
    signal face_active_mask    : std_logic_vector(3 downto 0) := "0001";
    signal face_stops_per_chip : unsigned(3 downto 0)         := to_unsigned(2, 4);
    signal max_hits_cfg        : unsigned(2 downto 0)         := to_unsigned(7, 3);
    signal max_scan_clks       : unsigned(15 downto 0)        := to_unsigned(1000, 16);
    signal rows_per_face       : unsigned(15 downto 0)        := to_unsigned(2, 16);

    -- Header metadata
    signal cfg_face           : t_tdc_cfg := c_TDC_CFG_INIT;
    signal frame_id           : unsigned(31 downto 0)                       := x"00000001";
    signal face_id            : unsigned(7 downto 0)                        := x"00";
    signal global_shot_seq    : unsigned(c_SHOT_SEQ_WIDTH - 1 downto 0)     := (others => '0');
    signal timestamp          : unsigned(63 downto 0)                       := (others => '0');
    signal chip_error_merged  : std_logic_vector(3 downto 0)                := "0000";
    signal error_count        : unsigned(31 downto 0)                       := (others => '0');
    signal bin_resolution_ps  : unsigned(15 downto 0)                       := to_unsigned(81, 16);
    signal k_dist_fixed       : unsigned(31 downto 0)                       := (others => '0');

    -- VDMA rise output
    signal m_axis_tdata       : std_logic_vector(C_OUTPUT_WIDTH - 1 downto 0);
    signal m_axis_tvalid      : std_logic;
    signal m_axis_tlast       : std_logic;
    signal m_axis_tuser       : std_logic_vector(0 downto 0);
    signal m_axis_tready      : std_logic := '1';

    -- VDMA fall output
    signal m_axis_fall_tdata  : std_logic_vector(C_OUTPUT_WIDTH - 1 downto 0);
    signal m_axis_fall_tvalid : std_logic;
    signal m_axis_fall_tlast  : std_logic;
    signal m_axis_fall_tuser  : std_logic_vector(0 downto 0);
    signal m_axis_fall_tready : std_logic := '1';

    -- Status
    signal row_done             : std_logic;
    signal row_fall_done        : std_logic;
    signal chip_error_flags     : std_logic_vector(3 downto 0);
    signal chip_fall_error      : std_logic_vector(3 downto 0);
    signal shot_overrun         : std_logic;
    signal shot_fall_overrun    : std_logic;
    signal face_abort           : std_logic;
    signal face_fall_abort      : std_logic;
    signal face_asm_idle        : std_logic;
    signal face_asm_fall_idle   : std_logic;
    signal frame_done           : std_logic;
    signal frame_fall_done      : std_logic;
    signal hdr_draining         : std_logic;
    signal hdr_fall_draining    : std_logic;
    signal hdr_idle             : std_logic;
    signal hdr_fall_idle        : std_logic;
    signal face_tvalid          : std_logic;
    signal face_fall_tvalid     : std_logic;
    signal face_buf_tvalid      : std_logic;
    signal face_fall_buf_tvalid : std_logic;

    -- Monitor counters
    signal v_out_beat_cnt : natural := 0;
    signal v_sof_seen     : boolean := false;
    signal v_eol_seen     : boolean := false;

begin

    -- =========================================================================
    -- Clock generation (200 MHz)
    -- =========================================================================
    clk <= not clk after C_CLK_PERIOD / 2;

    -- =========================================================================
    -- DUT instantiation
    -- =========================================================================
    u_dut : entity work.tdc_gpx_output_stage
        generic map (
            g_OUTPUT_WIDTH   => C_OUTPUT_WIDTH,
            g_ALU_PULSE_CLKS => C_ALU_PULSE_CLKS
        )
        port map (
            i_clk                  => clk,
            i_rst_n                => rst_n,
            -- Rise cell inputs
            i_cell_rise_tdata_0    => cell_rise_tdata_0,
            i_cell_rise_tdata_1    => cell_rise_tdata_1,
            i_cell_rise_tdata_2    => cell_rise_tdata_2,
            i_cell_rise_tdata_3    => cell_rise_tdata_3,
            i_cell_rise_tvalid     => cell_rise_tvalid,
            i_cell_rise_tlast      => cell_rise_tlast,
            o_cell_rise_tready     => cell_rise_tready,
            -- Fall cell inputs
            i_cell_fall_tdata_0    => cell_fall_tdata_0,
            i_cell_fall_tdata_1    => cell_fall_tdata_1,
            i_cell_fall_tdata_2    => cell_fall_tdata_2,
            i_cell_fall_tdata_3    => cell_fall_tdata_3,
            i_cell_fall_tvalid     => cell_fall_tvalid,
            i_cell_fall_tlast      => cell_fall_tlast,
            o_cell_fall_tready     => cell_fall_tready,
            -- Control
            i_shot_start_gated     => shot_start_gated,
            i_pipeline_abort       => pipeline_abort,
            i_face_start_gated     => face_start_gated,
            -- Configuration
            i_face_active_mask     => face_active_mask,
            i_face_stops_per_chip  => face_stops_per_chip,
            i_max_hits_cfg         => max_hits_cfg,
            i_max_scan_clks        => max_scan_clks,
            i_rows_per_face        => rows_per_face,
            -- Header metadata
            i_cfg_face             => cfg_face,
            i_frame_id             => frame_id,
            i_face_id              => face_id,
            i_global_shot_seq      => global_shot_seq,
            i_timestamp            => timestamp,
            i_chip_error_merged    => chip_error_merged,
            i_error_count          => error_count,
            i_bin_resolution_ps    => bin_resolution_ps,
            i_k_dist_fixed         => k_dist_fixed,
            -- VDMA rise output
            o_m_axis_tdata         => m_axis_tdata,
            o_m_axis_tvalid        => m_axis_tvalid,
            o_m_axis_tlast         => m_axis_tlast,
            o_m_axis_tuser         => m_axis_tuser,
            i_m_axis_tready        => m_axis_tready,
            -- VDMA fall output
            o_m_axis_fall_tdata    => m_axis_fall_tdata,
            o_m_axis_fall_tvalid   => m_axis_fall_tvalid,
            o_m_axis_fall_tlast    => m_axis_fall_tlast,
            o_m_axis_fall_tuser    => m_axis_fall_tuser,
            i_m_axis_fall_tready   => m_axis_fall_tready,
            -- Status
            o_row_done             => row_done,
            o_row_fall_done        => row_fall_done,
            o_chip_error_flags     => chip_error_flags,
            o_chip_fall_error      => chip_fall_error,
            o_shot_overrun         => shot_overrun,
            o_shot_fall_overrun    => shot_fall_overrun,
            o_face_abort           => face_abort,
            o_face_fall_abort      => face_fall_abort,
            o_face_asm_idle        => face_asm_idle,
            o_face_asm_fall_idle   => face_asm_fall_idle,
            o_frame_done           => frame_done,
            o_frame_fall_done      => frame_fall_done,
            o_hdr_draining         => hdr_draining,
            o_hdr_fall_draining    => hdr_fall_draining,
            o_hdr_idle             => hdr_idle,
            o_hdr_fall_idle        => hdr_fall_idle,
            o_face_tvalid          => face_tvalid,
            o_face_fall_tvalid     => face_fall_tvalid,
            o_face_buf_tvalid      => face_buf_tvalid,
            o_face_fall_buf_tvalid => face_fall_buf_tvalid
        );

    -- =========================================================================
    -- Override cfg_face fields for this test
    -- =========================================================================
    cfg_face.active_chip_mask <= "0001";
    cfg_face.stops_per_chip   <= to_unsigned(2, 4);
    cfg_face.max_scan_clks    <= to_unsigned(1000, 16);
    cfg_face.max_hits_cfg     <= to_unsigned(7, 3);
    cfg_face.cols_per_face    <= to_unsigned(2, 16);

    -- =========================================================================
    -- VDMA output monitor
    -- =========================================================================
    p_monitor : process(clk)
    begin
        if rising_edge(clk) then
            if m_axis_tvalid = '1' and m_axis_tready = '1' then
                v_out_beat_cnt <= v_out_beat_cnt + 1;
                if m_axis_tuser(0) = '1' then
                    v_sof_seen <= true;
                end if;
                if m_axis_tlast = '1' then
                    v_eol_seen <= true;
                end if;
            end if;
        end if;
    end process p_monitor;

    -- =========================================================================
    -- Stimulus process
    -- =========================================================================
    p_stim : process
        variable v_beat_data : unsigned(C_OUTPUT_WIDTH - 1 downto 0);
    begin
        -- =====================================================================
        -- 1. Reset
        -- =====================================================================
        rst_n <= '0';
        wait for 100 ns;
        wait until rising_edge(clk);
        rst_n <= '1';
        wait until rising_edge(clk);
        wait until rising_edge(clk);

        -- =====================================================================
        -- 2. Assert face_start_gated for 1 clock
        -- =====================================================================
        face_start_gated <= '1';
        wait until rising_edge(clk);
        face_start_gated <= '0';

        -- Small gap before shot_start
        wait for 3 * C_CLK_PERIOD;
        wait until rising_edge(clk);

        -- =====================================================================
        -- 3. Assert shot_start_gated for 1 clock
        -- =====================================================================
        shot_start_gated <= '1';
        wait until rising_edge(clk);
        shot_start_gated <= '0';

        -- Wait a few clocks for face_assembler to be ready
        wait for 5 * C_CLK_PERIOD;
        wait until rising_edge(clk);

        -- =====================================================================
        -- 4. Feed cell data on chip 0 rising input
        --    2 stops x 4 beats = 8 beats total (incrementing pattern)
        -- =====================================================================
        for beat_idx in 0 to C_TOTAL_BEATS - 1 loop
            -- Wait for tready before driving
            while cell_rise_tready(0) = '0' loop
                wait until rising_edge(clk);
            end loop;

            v_beat_data := to_unsigned(beat_idx + 1, C_OUTPUT_WIDTH);
            cell_rise_tdata_0 <= std_logic_vector(v_beat_data);
            cell_rise_tvalid  <= "0001";    -- only chip 0

            -- tlast on last beat of each cell (every C_BEATS_PER_CELL beats)
            if (beat_idx + 1) mod C_BEATS_PER_CELL = 0 then
                cell_rise_tlast <= "0001";
            else
                cell_rise_tlast <= "0000";
            end if;

            wait until rising_edge(clk);
        end loop;

        -- De-assert after last beat
        cell_rise_tvalid <= "0000";
        cell_rise_tlast  <= "0000";

        -- =====================================================================
        -- 5. Wait for frame_done or watchdog
        -- =====================================================================
        wait until frame_done = '1' for C_WATCHDOG;

        -- =====================================================================
        -- 6. Report results
        -- =====================================================================
        report "=== TB Results ===" severity note;
        report "  Output beats: " & integer'image(v_out_beat_cnt) severity note;
        report "  SOF seen:     " & boolean'image(v_sof_seen)     severity note;
        report "  EOL seen:     " & boolean'image(v_eol_seen)     severity note;
        report "  frame_done:   " & std_logic'image(frame_done)   severity note;

        if frame_done = '1' and v_sof_seen and v_eol_seen and v_out_beat_cnt > 0 then
            report "*** PASS ***" severity note;
        else
            report "*** FAIL ***" severity failure;
        end if;

        wait for 10 * C_CLK_PERIOD;
        std.env.finish;
    end process p_stim;

    -- =========================================================================
    -- Watchdog timeout
    -- =========================================================================
    p_watchdog : process
    begin
        wait for C_WATCHDOG;
        report "WATCHDOG TIMEOUT (50 us) - aborting simulation" severity failure;
        std.env.finish;
    end process p_watchdog;

end architecture sim;
