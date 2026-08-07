library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.std_logic_textio.all;

library std;
use std.env.all;
use std.textio.all;

use work.lidar_build_pkg.all;
use work.lidar_event_types_pkg.all;
use work.lidar_gpx_data_pkg.all;
use work.lidar_gpx_vdma_pkg.all;

entity tb_lidar_gpx_ddr_golden is
    generic (
        G_PROC_MHZ     : positive := 150;
        G_OUTPUT_WIDTH : positive := 32
    );
end entity tb_lidar_gpx_ddr_golden;

architecture sim of tb_lidar_gpx_ddr_golden is

    function fn_build_config return lidar_build_config_t is
        variable result : lidar_build_config_t := C_DEFAULT_BUILD_CONFIG;
    begin
        result.proc_clk_mhz := G_PROC_MHZ;
        result.tdc_clk_mhz := 200;
        result.stream_clock_mode := STREAM_CLOCK_ASYNC;
        result.num_chips := 1;
        result.stops_per_chip := 1;
        result.max_returns_per_stop := 7;
        result.rise_capability_mask := "0001";
        result.fall_capability_mask := "0000";
        result.output_width := G_OUTPUT_WIDTH;
        result.num_faces := 5;
        return result;
    end function fn_build_config;

    function fn_capture_file return string is
    begin
        if G_PROC_MHZ = 150 then
            case G_OUTPUT_WIDTH is
                when 32     => return "ddr_capture_150_32.hex";
                when 64     => return "ddr_capture_150_64.hex";
                when others => return "ddr_capture_150_128.hex";
            end case;
        end if;
        case G_OUTPUT_WIDTH is
            when 32     => return "ddr_capture_200_32.hex";
            when 64     => return "ddr_capture_200_64.hex";
            when others => return "ddr_capture_200_128.hex";
        end case;
    end function fn_capture_file;

    function fn_cell_event return gpx_frame_cell_event_t is
        variable result : gpx_frame_cell_event_t :=
            C_GPX_FRAME_CELL_EVENT_IDLE;
    begin
        result.valid := '1';
        result.cell.valid := '1';
        result.cell.kind := GPX_CELL_DATA;
        result.cell.chip_index := (others => '0');
        result.cell.stop_index := (others => '0');
        result.cell.slope := GPX_SLOPE_RISE;
        result.cell.hit_count := to_unsigned(
            1, result.cell.hit_count'length);
        result.cell.max_hits := to_unsigned(
            1, result.cell.max_hits'length);
        result.cell.hits(0) := to_unsigned(
            16#10001#, result.cell.hits(0)'length);
        result.cell.shot_context.valid := '1';
        result.cell.shot_context.request.valid := '1';
        result.cell.shot_context.request.face_index := to_unsigned(
            2, result.cell.shot_context.request.face_index'length);
        result.cell.shot_context.request.position := to_unsigned(
            16#0123#, result.cell.shot_context.request.position'length);
        result.cell.shot_context.request.direction := DIRECTION_CCW;
        result.cell.shot_context.request.shot_index := (others => '0');
        result.cell.shot_context.request.source_sim := '1';
        result.cell.shot_context.request.active_version := x"1234";
        result.cell.shot_context.t0_timestamp_ticks :=
            x"0000000176543210";
        result.cell.shot_context.t0_timestamp_valid := '1';
        result.slot_index := (others => '0');
        result.slot_count := to_unsigned(1, result.slot_count'length);
        result.line_start := '1';
        result.line_end := '1';
        result.first_column := '1';
        return result;
    end function fn_cell_event;

    function fn_close_event return gpx_frame_close_event_t is
        variable result : gpx_frame_close_event_t :=
            C_GPX_FRAME_CLOSE_EVENT_IDLE;
    begin
        result.valid := '1';
        result.face_frame_id := x"11223344";
        result.face_index := to_unsigned(2, result.face_index'length);
        result.direction := DIRECTION_CCW;
        result.source_sim := '1';
        result.active_version := x"1234";
        result.columns_per_face := to_unsigned(
            2, result.columns_per_face'length);
        result.trailing_gap := to_unsigned(
            1, result.trailing_gap'length);
        return result;
    end function fn_close_event;

    constant C_BUILD_CONFIG : lidar_build_config_t := fn_build_config;
    constant C_CLK_PERIOD : time := 1 us / G_PROC_MHZ;
    constant C_WORDS_PER_BEAT : positive := G_OUTPUT_WIDTH / 32;
    constant C_HSIZE_BYTES : positive := fn_gpx_vdma_shot_hsize_bytes(
        1, 1, G_OUTPUT_WIDTH);
    constant C_HSIZE_WORDS : positive := C_HSIZE_BYTES / 4;
    constant C_STRIDE_BYTES : positive := fn_gpx_vdma_stride_bytes(
        1, 7, G_OUTPUT_WIDTH);
    constant C_STRIDE_WORDS : positive := C_STRIDE_BYTES / 4;
    constant C_FOOTER_LINES : positive := fn_gpx_vdma_footer_lines(
        C_HSIZE_BYTES);
    constant C_VSIZE_LINES : positive := 2 + C_FOOTER_LINES;
    constant C_FRAME_WORDS : positive :=
        C_STRIDE_WORDS * C_VSIZE_LINES;

    type memory_t is array (0 to C_FRAME_WORDS - 1) of
        std_logic_vector(31 downto 0);

    file capture_file : text open write_mode is fn_capture_file;

    signal clk : std_logic := '0';
    signal rst_n : std_logic := '0';
    signal abort_run : std_logic := '0';

    signal request_valid : std_logic := '0';
    signal request_ready : std_logic;
    signal request_rejected : std_logic;
    signal activate_valid : std_logic := '0';
    signal activate_ready : std_logic;
    signal datapath_idle : std_logic;
    signal vdma_cfg_valid : std_logic;
    signal vdma_cfg_ready : std_logic := '0';
    signal vdma_cfg_enable : std_logic;
    signal vdma_hsize_bytes : gpx_vdma_geometry_value_t;
    signal vdma_vsize_lines : gpx_vdma_geometry_value_t;
    signal vdma_stride_bytes : gpx_vdma_geometry_value_t;
    signal active_profile : gpx_vdma_lane_profile_t :=
        C_GPX_VDMA_LANE_PROFILE_IDLE;
    signal profile_activated : std_logic;
    signal pending_valid : std_logic;
    signal profile_busy : std_logic;

    signal cell_event : gpx_frame_cell_event_t :=
        C_GPX_FRAME_CELL_EVENT_IDLE;
    signal cell_ready : std_logic;
    signal close_event : gpx_frame_close_event_t :=
        C_GPX_FRAME_CLOSE_EVENT_IDLE;
    signal close_ready : std_logic;
    signal footer_emitted : std_logic;
    signal lane_idle : std_logic;

    signal tdata : std_logic_vector(G_OUTPUT_WIDTH - 1 downto 0);
    signal tkeep : std_logic_vector(G_OUTPUT_WIDTH / 8 - 1 downto 0);
    signal tstrb : std_logic_vector(G_OUTPUT_WIDTH / 8 - 1 downto 0);
    signal tuser : std_logic_vector(0 downto 0);
    signal tvalid : std_logic;
    signal tlast : std_logic;
    signal tready : std_logic := '0';
    signal line_done : std_logic;
    signal frame_done : std_logic;

    signal memory_r : memory_t := (others => x"A5A5A5A5");
    signal capture_line_r : natural range 0 to C_VSIZE_LINES := 0;
    signal capture_word_r : natural range 0 to C_HSIZE_WORDS := 0;
    signal sof_count_r : natural range 0 to 2 := 0;
    signal footer_emitted_seen_r : std_logic := '0';
    signal ready_cycle_r : natural := 0;

begin

    clk <= not clk after C_CLK_PERIOD / 2;
    datapath_idle <= lane_idle;

    u_profile : entity work.lidar_gpx_vdma_profile_manager
        generic map (
            G_BUILD_CONFIG => C_BUILD_CONFIG,
            G_LANE_RISE    => true
        )
        port map (
            i_clk               => clk,
            i_rst_n             => rst_n,
            i_abort             => abort_run,
            i_request_valid     => request_valid,
            o_request_ready     => request_ready,
            i_lane_chip_mask    => "0001",
            i_visible_returns   => to_unsigned(1, 3),
            i_planned_shots     => to_unsigned(2, 16),
            o_request_rejected  => request_rejected,
            i_activate_valid    => activate_valid,
            o_activate_ready    => activate_ready,
            i_datapath_idle     => datapath_idle,
            o_vdma_cfg_valid    => vdma_cfg_valid,
            i_vdma_cfg_ready    => vdma_cfg_ready,
            o_vdma_cfg_enable   => vdma_cfg_enable,
            o_vdma_hsize_bytes  => vdma_hsize_bytes,
            o_vdma_vsize_lines  => vdma_vsize_lines,
            o_vdma_stride_bytes => vdma_stride_bytes,
            o_active_profile    => active_profile,
            o_profile_activated => profile_activated,
            o_pending_valid     => pending_valid,
            o_busy              => profile_busy
        );

    u_lane : entity work.lidar_gpx_axis_lane_pipeline
        generic map (
            G_BUILD_CONFIG => C_BUILD_CONFIG,
            G_LANE_RISE    => true
        )
        port map (
            i_clk                  => clk,
            i_rst_n                => rst_n,
            i_abort                => abort_run,
            i_active_profile       => active_profile,
            i_cell_event           => cell_event,
            o_cell_ready           => cell_ready,
            i_frame_close_event    => close_event,
            o_frame_close_ready    => close_ready,
            o_m_axis_tdata    => tdata,
            o_m_axis_tkeep    => tkeep,
            o_m_axis_tstrb    => tstrb,
            o_m_axis_tuser    => tuser,
            o_m_axis_tvalid   => tvalid,
            o_m_axis_tlast    => tlast,
            i_m_axis_tready   => tready,
            o_line_done       => line_done,
            o_frame_done      => frame_done,
            o_footer_emitted  => footer_emitted,
            o_idle            => lane_idle
        );

    p_ready : process (clk)
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                ready_cycle_r <= 0;
                tready <= '0';
            else
                ready_cycle_r <= ready_cycle_r + 1;
                if ready_cycle_r mod 7 = 3 then
                    tready <= '0';
                else
                    tready <= '1';
                end if;
            end if;
        end if;
    end process p_ready;

    p_capture : process (clk)
        variable target_word_v : natural;
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                memory_r <= (others => x"A5A5A5A5");
                capture_line_r <= 0;
                capture_word_r <= 0;
                sof_count_r <= 0;
                footer_emitted_seen_r <= '0';
            else
                if footer_emitted = '1' then
                    footer_emitted_seen_r <= '1';
                end if;
                if tvalid = '1' and tready = '1' then
                assert tkeep = (tkeep'range => '1') and
                       tstrb = (tstrb'range => '1')
                    report "V2-B9-J9-TB full TKEEP/TSTRB contract failed"
                    severity failure;
                assert (tuser(0) = '1') =
                       (capture_line_r = 0 and capture_word_r = 0)
                    report "V2-B9-J9-TB SOF location mismatch"
                    severity failure;
                if tuser(0) = '1' then
                    sof_count_r <= sof_count_r + 1;
                end if;

                for lane_index in 0 to C_WORDS_PER_BEAT - 1 loop
                    target_word_v := capture_line_r * C_STRIDE_WORDS +
                        capture_word_r + lane_index;
                    assert target_word_v < C_FRAME_WORDS
                        report "V2-B9-J9-TB DDR write exceeded allocation"
                        severity failure;
                    memory_r(target_word_v) <= tdata(
                        (lane_index + 1) * 32 - 1 downto lane_index * 32);
                end loop;

                if tlast = '1' then
                    assert capture_word_r + C_WORDS_PER_BEAT =
                               C_HSIZE_WORDS
                        report "V2-B9-J9-TB TLAST does not match HSIZE"
                        severity failure;
                    capture_line_r <= capture_line_r + 1;
                    capture_word_r <= 0;
                else
                    capture_word_r <=
                        capture_word_r + C_WORDS_PER_BEAT;
                end if;
                end if;
            end if;
        end if;
    end process p_capture;

    p_test : process
        procedure send_cell is
        begin
            wait until falling_edge(clk);
            cell_event <= fn_cell_event;
            loop
                wait until rising_edge(clk);
                exit when cell_ready = '1';
            end loop;
            wait until falling_edge(clk);
            cell_event <= C_GPX_FRAME_CELL_EVENT_IDLE;
        end procedure send_cell;

        variable output_line : line;
    begin
        rst_n <= '0';
        wait for 8 * C_CLK_PERIOD;
        wait until falling_edge(clk);
        rst_n <= '1';

        wait until falling_edge(clk);
        request_valid <= '1';
        loop
            wait until rising_edge(clk);
            exit when request_ready = '1';
        end loop;
        wait until falling_edge(clk);
        request_valid <= '0';

        loop
            wait until falling_edge(clk);
            exit when pending_valid = '1';
        end loop;
        activate_valid <= '1';
        loop
            wait until rising_edge(clk);
            exit when activate_ready = '1';
        end loop;
        wait until falling_edge(clk);
        activate_valid <= '0';

        loop
            wait until falling_edge(clk);
            exit when vdma_cfg_valid = '1';
        end loop;
        assert vdma_cfg_enable = '1' and
               to_integer(vdma_hsize_bytes) = C_HSIZE_BYTES and
               to_integer(vdma_vsize_lines) = C_VSIZE_LINES and
               to_integer(vdma_stride_bytes) = C_STRIDE_BYTES
            report "V2-B9-J9-TB Pending VDMA geometry mismatch"
            severity failure;
        for delay_index in 0 to 2 loop
            wait until rising_edge(clk);
            assert vdma_cfg_valid = '1'
                report "V2-B9-J9-TB VDMA request changed under stall"
                severity failure;
        end loop;
        wait until falling_edge(clk);
        vdma_cfg_ready <= '1';
        wait until rising_edge(clk);
        wait until falling_edge(clk);
        vdma_cfg_ready <= '0';
        assert profile_activated = '1' and active_profile.valid = '1'
            report "V2-B9-J9-TB Active profile was not acknowledged"
            severity failure;

        send_cell;

        wait until falling_edge(clk);
        close_event <= fn_close_event;
        loop
            wait until rising_edge(clk);
            exit when close_ready = '1';
        end loop;
        wait until falling_edge(clk);
        close_event <= C_GPX_FRAME_CLOSE_EVENT_IDLE;

        loop
            wait until falling_edge(clk);
            exit when frame_done = '1';
        end loop;
        wait until rising_edge(clk);
        wait until falling_edge(clk);

        assert capture_line_r = C_VSIZE_LINES and capture_word_r = 0 and
               sof_count_r = 1 and footer_emitted_seen_r = '1'
            report "V2-B9-J9-TB captured Frame boundary mismatch"
            severity failure;
        assert active_profile.hsize_bytes = to_unsigned(
                   C_HSIZE_BYTES, active_profile.hsize_bytes'length) and
               active_profile.vsize_lines = to_unsigned(
                   C_VSIZE_LINES, active_profile.vsize_lines'length) and
               active_profile.stride_bytes = to_unsigned(
                   C_STRIDE_BYTES, active_profile.stride_bytes'length)
            report "V2-B9-J9-TB Active profile changed during Frame"
            severity failure;

        for word_index in memory_r'range loop
            write(output_line, string'("0x"));
            hwrite(output_line, std_logic_vector(to_unsigned(
                word_index * 4, 32)));
            write(output_line, string'(" 0x"));
            hwrite(output_line, memory_r(word_index));
            writeline(capture_file, output_line);
        end loop;

        report "LIDAR_V2_GPX_DDR_CAPTURE_PASS proc_mhz=" &
            integer'image(G_PROC_MHZ) & " width=" &
            integer'image(G_OUTPUT_WIDTH) severity note;
        finish;
        wait;
    end process p_test;

end architecture sim;

entity tb_lidar_gpx_ddr_golden_150_32 is
end entity tb_lidar_gpx_ddr_golden_150_32;
architecture sim of tb_lidar_gpx_ddr_golden_150_32 is
begin
    u : entity work.tb_lidar_gpx_ddr_golden
        generic map (G_PROC_MHZ => 150, G_OUTPUT_WIDTH => 32);
end architecture sim;

entity tb_lidar_gpx_ddr_golden_150_64 is
end entity tb_lidar_gpx_ddr_golden_150_64;
architecture sim of tb_lidar_gpx_ddr_golden_150_64 is
begin
    u : entity work.tb_lidar_gpx_ddr_golden
        generic map (G_PROC_MHZ => 150, G_OUTPUT_WIDTH => 64);
end architecture sim;

entity tb_lidar_gpx_ddr_golden_150_128 is
end entity tb_lidar_gpx_ddr_golden_150_128;
architecture sim of tb_lidar_gpx_ddr_golden_150_128 is
begin
    u : entity work.tb_lidar_gpx_ddr_golden
        generic map (G_PROC_MHZ => 150, G_OUTPUT_WIDTH => 128);
end architecture sim;

entity tb_lidar_gpx_ddr_golden_200_32 is
end entity tb_lidar_gpx_ddr_golden_200_32;
architecture sim of tb_lidar_gpx_ddr_golden_200_32 is
begin
    u : entity work.tb_lidar_gpx_ddr_golden
        generic map (G_PROC_MHZ => 200, G_OUTPUT_WIDTH => 32);
end architecture sim;

entity tb_lidar_gpx_ddr_golden_200_64 is
end entity tb_lidar_gpx_ddr_golden_200_64;
architecture sim of tb_lidar_gpx_ddr_golden_200_64 is
begin
    u : entity work.tb_lidar_gpx_ddr_golden
        generic map (G_PROC_MHZ => 200, G_OUTPUT_WIDTH => 64);
end architecture sim;

entity tb_lidar_gpx_ddr_golden_200_128 is
end entity tb_lidar_gpx_ddr_golden_200_128;
architecture sim of tb_lidar_gpx_ddr_golden_200_128 is
begin
    u : entity work.tb_lidar_gpx_ddr_golden
        generic map (G_PROC_MHZ => 200, G_OUTPUT_WIDTH => 128);
end architecture sim;
