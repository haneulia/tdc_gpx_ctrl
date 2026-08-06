library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library std;
use std.env.all;

use work.lidar_build_pkg.all;
use work.lidar_event_types_pkg.all;
use work.lidar_gpx_data_pkg.all;
use work.lidar_gpx_vdma_pkg.all;

entity tb_lidar_gpx_face_footer_builder is
    generic (
        G_CLK_MHZ      : positive := 150;
        G_OUTPUT_WIDTH : positive := 32
    );
end entity tb_lidar_gpx_face_footer_builder;

architecture sim of tb_lidar_gpx_face_footer_builder is

    function fn_build_config return lidar_build_config_t is
        variable result : lidar_build_config_t := C_DEFAULT_BUILD_CONFIG;
    begin
        result.proc_clk_mhz := G_CLK_MHZ;
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

    constant C_BUILD_CONFIG : lidar_build_config_t := fn_build_config;
    constant C_CLK_PERIOD : time := 1 us / G_CLK_MHZ;
    constant C_WORDS_PER_BEAT : positive := G_OUTPUT_WIDTH / 32;
    constant C_RAW_WORDS : positive := 6;
    constant C_HSIZE_BYTES : positive := fn_gpx_vdma_shot_hsize_bytes(
        1, 1, G_OUTPUT_WIDTH);
    constant C_HSIZE_WORDS : positive := C_HSIZE_BYTES / 4;
    constant C_FOOTER_LINES : positive := fn_gpx_vdma_footer_lines(
        C_HSIZE_BYTES);
    constant C_VSIZE_LINES : positive := 2 + C_FOOTER_LINES;
    constant C_BEATS_PER_LINE : positive :=
        C_HSIZE_BYTES / (G_OUTPUT_WIDTH / 8);
    constant C_TOTAL_BEATS : positive :=
        C_VSIZE_LINES * C_BEATS_PER_LINE;

    function fn_active_profile return gpx_vdma_lane_profile_t is
        variable result : gpx_vdma_lane_profile_t :=
            C_GPX_VDMA_LANE_PROFILE_IDLE;
    begin
        result.valid := '1';
        result.enabled := '1';
        result.slot_count := to_unsigned(1, result.slot_count'length);
        result.visible_returns := to_unsigned(
            1, result.visible_returns'length);
        result.cell_word_count := to_unsigned(
            2, result.cell_word_count'length);
        result.planned_shots := to_unsigned(
            2, result.planned_shots'length);
        result.raw_line_words := to_unsigned(
            C_RAW_WORDS, result.raw_line_words'length);
        result.hsize_bytes := to_unsigned(
            C_HSIZE_BYTES, result.hsize_bytes'length);
        result.hsize_words := to_unsigned(
            C_HSIZE_WORDS, result.hsize_words'length);
        result.footer_lines := to_unsigned(
            C_FOOTER_LINES, result.footer_lines'length);
        result.vsize_lines := to_unsigned(
            C_VSIZE_LINES, result.vsize_lines'length);
        result.stride_bytes := to_unsigned(
            fn_gpx_vdma_stride_bytes(1, 7, G_OUTPUT_WIDTH),
            result.stride_bytes'length);
        return result;
    end function fn_active_profile;

    constant C_ACTIVE_PROFILE : gpx_vdma_lane_profile_t :=
        fn_active_profile;

    signal clk : std_logic := '0';
    signal rst_n : std_logic := '0';
    signal abort_run : std_logic := '0';
    signal active_profile : gpx_vdma_lane_profile_t := C_ACTIVE_PROFILE;

    signal source_word : gpx_vdma_line_word_event_t :=
        C_GPX_VDMA_LINE_WORD_EVENT_IDLE;
    signal source_ready : std_logic;
    signal close_event : gpx_frame_close_event_t :=
        C_GPX_FRAME_CLOSE_EVENT_IDLE;
    signal close_ready : std_logic;

    signal footer_word : gpx_vdma_line_word_event_t;
    signal footer_word_ready : std_logic;
    signal hsize_bytes : gpx_vdma_geometry_value_t;
    signal vsize_lines : gpx_vdma_geometry_value_t;
    signal stride_bytes : gpx_vdma_geometry_value_t;
    signal footer_emitted : std_logic;
    signal footer_idle : std_logic;

    signal tdata : std_logic_vector(G_OUTPUT_WIDTH - 1 downto 0);
    signal tkeep : std_logic_vector(G_OUTPUT_WIDTH / 8 - 1 downto 0);
    signal tstrb : std_logic_vector(G_OUTPUT_WIDTH / 8 - 1 downto 0);
    signal tuser : std_logic_vector(0 downto 0);
    signal tvalid : std_logic;
    signal tlast : std_logic;
    signal tready : std_logic := '0';
    signal line_done : std_logic;
    signal frame_done : std_logic;
    signal packer_idle : std_logic;

    signal ready_cycle : natural := 0;
    signal frame_done_seen : std_logic := '0';

    function fn_source_word(
        line_index : natural;
        word_index : natural
    ) return gpx_vdma_line_word_event_t is
        variable result : gpx_vdma_line_word_event_t :=
            C_GPX_VDMA_LINE_WORD_EVENT_IDLE;
    begin
        result.valid := '1';
        result.data := std_logic_vector(to_unsigned(
            16#10000000# + line_index * 16#00000100# + word_index, 32));
        if word_index < C_GPX_VDMA_SHOT_META_WORDS then
            result.kind := GPX_VDMA_LINE_SHOT_METADATA;
        else
            result.kind := GPX_VDMA_LINE_CELL_DATA;
        end if;
        if line_index = 0 and word_index = 3 then
            result.data(C_GPX_SHOT_META_TIMEOUT) := '1';
        end if;
        result.word_index := to_unsigned(word_index, result.word_index'length);
        result.line_word_count := to_unsigned(
            C_RAW_WORDS, result.line_word_count'length);
        result.line_start := '1' when word_index = 0 else '0';
        result.line_end := '1' when word_index + 1 = C_RAW_WORDS else '0';
        result.first_column := '1' when line_index = 0 else '0';
        result.last_column := '1' when line_index = 1 else '0';
        result.slot_count := to_unsigned(1, result.slot_count'length);
        result.cell_word_count := to_unsigned(
            2, result.cell_word_count'length);
        result.line_hole := '1' when line_index = 1 else '0';
        result.line_faulted := '1' when line_index = 1 else '0';
        return result;
    end function fn_source_word;

    function fn_close return gpx_frame_close_event_t is
        variable result : gpx_frame_close_event_t :=
            C_GPX_FRAME_CLOSE_EVENT_IDLE;
    begin
        result.valid := '1';
        result.face_frame_id := x"11223344";
        result.face_index := to_unsigned(2, result.face_index'length);
        result.direction := DIRECTION_CCW;
        result.source_sim := '1';
        result.active_version := x"1234";
        result.columns_per_face := to_unsigned(2, 16);
        result.face_faulted := '1';
        return result;
    end function fn_close;

    function fn_expected_footer_word(
        word_index : natural
    ) return std_logic_vector is
        variable result : std_logic_vector(31 downto 0) := (others => '0');
    begin
        case word_index is
            when 0 =>
                result := C_GPX_VDMA_FOOTER_MAGIC;
            when 1 =>
                result := x"11223344";
            when 2 =>
                result(2 downto 0) := std_logic_vector(to_unsigned(2, 3));
                result(C_GPX_FOOTER_W2_SLOPE_RISE) := '1';
                result(C_GPX_FOOTER_W2_DIRECTION_CCW) := '1';
                result(C_GPX_FOOTER_W2_SOURCE_SIM) := '1';
                result(C_GPX_FOOTER_W2_WIDTH_HI downto
                       C_GPX_FOOTER_W2_WIDTH_LO) :=
                    fn_gpx_vdma_output_width_code(G_OUTPUT_WIDTH);
            when 3 =>
                result(15 downto 0) := x"1234";
            when 4 =>
                result(15 downto 0) := std_logic_vector(to_unsigned(2, 16));
                result(21 downto 16) := std_logic_vector(to_unsigned(1, 6));
                result(24 downto 22) := std_logic_vector(to_unsigned(1, 3));
            when 5 =>
                result(15 downto 0) := std_logic_vector(to_unsigned(
                    C_HSIZE_BYTES, 16));
                result(31 downto 16) := std_logic_vector(to_unsigned(
                    C_VSIZE_LINES, 16));
            when 6 =>
                result(15 downto 0) := std_logic_vector(to_unsigned(2, 16));
                result(C_GPX_FOOTER_W6_FACE_FAULT) := '1';
                result(C_GPX_FOOTER_W6_ANY_LINE_FAULT) := '1';
                result(C_GPX_FOOTER_W6_ANY_HOLE) := '1';
                result(C_GPX_FOOTER_W6_ANY_TIMEOUT) := '1';
            when 7 =>
                result := C_GPX_VDMA_FOOTER_COMMIT;
            when others =>
                null;
        end case;
        return result;
    end function fn_expected_footer_word;

    function fn_expected_word(
        line_index : natural;
        word_index : natural
    ) return std_logic_vector is
        variable footer_index : natural;
    begin
        if line_index < 2 then
            if word_index < C_RAW_WORDS then
                return fn_source_word(line_index, word_index).data;
            end if;
            return std_logic_vector(to_unsigned(0, 32));
        end if;
        footer_index := (line_index - 2) * C_HSIZE_WORDS + word_index;
        if footer_index < C_GPX_VDMA_FOOTER_WORDS then
            return fn_expected_footer_word(footer_index);
        end if;
        return std_logic_vector(to_unsigned(0, 32));
    end function fn_expected_word;

    function fn_expected_beat(
        line_index : natural;
        beat_index : natural
    ) return std_logic_vector is
        variable result : std_logic_vector(
            G_OUTPUT_WIDTH - 1 downto 0) := (others => '0');
        variable word_index : natural;
    begin
        for lane_index in 0 to C_WORDS_PER_BEAT - 1 loop
            word_index := beat_index * C_WORDS_PER_BEAT + lane_index;
            result((lane_index + 1) * 32 - 1 downto
                   lane_index * 32) := fn_expected_word(
                line_index, word_index);
        end loop;
        return result;
    end function fn_expected_beat;

begin

    clk <= not clk after C_CLK_PERIOD / 2;

    u_footer : entity work.lidar_gpx_face_footer_builder
        generic map (
            G_BUILD_CONFIG => C_BUILD_CONFIG,
            G_LANE_RISE    => true
        )
        port map (
            i_clk                => clk,
            i_rst_n              => rst_n,
            i_abort              => abort_run,
            i_active_profile     => active_profile,
            i_line_word          => source_word,
            o_line_word_ready    => source_ready,
            i_frame_close_event  => close_event,
            o_frame_close_ready  => close_ready,
            o_line_word          => footer_word,
            i_line_word_ready    => footer_word_ready,
            o_active_hsize_bytes => hsize_bytes,
            o_active_vsize_lines => vsize_lines,
            o_stride_bytes       => stride_bytes,
            o_footer_emitted     => footer_emitted,
            o_idle               => footer_idle
        );

    u_packer : entity work.lidar_gpx_axis_word_packer
        generic map (
            G_OUTPUT_WIDTH => G_OUTPUT_WIDTH
        )
        port map (
            i_clk             => clk,
            i_rst_n           => rst_n,
            i_abort           => abort_run,
            i_line_word       => footer_word,
            o_line_word_ready => footer_word_ready,
            o_m_axis_tdata    => tdata,
            o_m_axis_tkeep    => tkeep,
            o_m_axis_tstrb    => tstrb,
            o_m_axis_tuser    => tuser,
            o_m_axis_tvalid   => tvalid,
            o_m_axis_tlast    => tlast,
            i_m_axis_tready   => tready,
            o_line_done       => line_done,
            o_frame_done      => frame_done,
            o_idle            => packer_idle
        );

    p_ready : process (clk)
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                ready_cycle <= 0;
                tready <= '0';
            else
                ready_cycle <= ready_cycle + 1;
                if ready_cycle mod 7 = 2 or ready_cycle mod 7 = 3 then
                    tready <= '0';
                else
                    tready <= '1';
                end if;
            end if;
        end if;
    end process p_ready;

    p_monitor : process (clk)
        variable beat_count : natural := 0;
        variable line_index : natural;
        variable beat_index : natural;
        variable held_valid : boolean := false;
        variable held_data : std_logic_vector(
            G_OUTPUT_WIDTH - 1 downto 0);
        variable held_user : std_logic_vector(0 downto 0);
        variable held_last : std_logic;
        variable expected_user : std_logic;
        variable expected_last : std_logic;
    begin
        if rising_edge(clk) then
            if rst_n = '0' or abort_run = '1' then
                beat_count := 0;
                held_valid := false;
                frame_done_seen <= '0';
            else
                if tvalid = '1' and tready = '0' then
                    if held_valid then
                        assert tdata = held_data and tuser = held_user and
                               tlast = held_last
                            report "V2-B9-J7-TB AXIS changed under stall"
                            severity failure;
                    else
                        held_data := tdata;
                        held_user := tuser;
                        held_last := tlast;
                        held_valid := true;
                    end if;
                else
                    held_valid := false;
                end if;

                if tvalid = '1' and tready = '1' then
                    line_index := beat_count / C_BEATS_PER_LINE;
                    beat_index := beat_count mod C_BEATS_PER_LINE;
                    assert line_index < C_VSIZE_LINES and
                           tdata = fn_expected_beat(line_index, beat_index)
                        report "V2-B9-J7-TB DDR Beat image mismatch"
                        severity failure;
                    assert tkeep = (tkeep'range => '1') and
                           tstrb = (tstrb'range => '1')
                        report "V2-B9-J7-TB TKEEP/TSTRB not full"
                        severity failure;
                    expected_user := '0';
                    expected_last := '0';
                    if line_index = 0 and beat_index = 0 then
                        expected_user := '1';
                    end if;
                    if beat_index + 1 = C_BEATS_PER_LINE then
                        expected_last := '1';
                    end if;
                    assert tuser(0) = expected_user
                        report "V2-B9-J7-TB SOF mismatch"
                        severity failure;
                    assert tlast = expected_last
                        report "V2-B9-J7-TB TLAST mismatch"
                        severity failure;
                    beat_count := beat_count + 1;
                end if;

                if frame_done = '1' then
                    assert beat_count = C_TOTAL_BEATS
                        report "V2-B9-J7-TB Frame done before final Beat"
                        severity failure;
                    frame_done_seen <= '1';
                end if;
            end if;
        end if;
    end process p_monitor;

    p_test : process
        procedure send_word(
            constant value : in gpx_vdma_line_word_event_t
        ) is
        begin
            source_word <= value;
            loop
                wait until rising_edge(clk);
                exit when source_ready = '1';
            end loop;
            source_word.valid <= '0';
        end procedure send_word;

        procedure send_close(
            constant value : in gpx_frame_close_event_t
        ) is
        begin
            close_event <= value;
            loop
                wait until rising_edge(clk);
                exit when close_ready = '1';
            end loop;
            close_event.valid <= '0';
        end procedure send_close;
    begin
        rst_n <= '0';
        wait for 8 * C_CLK_PERIOD;
        wait until rising_edge(clk);
        rst_n <= '1';

        for line_index in 0 to 1 loop
            for word_index in 0 to C_RAW_WORDS - 1 loop
                send_word(fn_source_word(line_index, word_index));
            end loop;
        end loop;
        send_close(fn_close);

        for cycle in 0 to 500 loop
            wait until rising_edge(clk);
            exit when frame_done_seen = '1';
        end loop;
        assert frame_done_seen = '1'
            report "V2-B9-J7-TB Frame completion timeout"
            severity failure;
        wait until rising_edge(clk);
        assert to_integer(hsize_bytes) = C_HSIZE_BYTES and
               to_integer(vsize_lines) = C_VSIZE_LINES and
               to_integer(stride_bytes) = fn_gpx_vdma_stride_bytes(
                   1, 7, G_OUTPUT_WIDTH)
            report "V2-B9-J7-TB geometry/STRIDE mismatch"
            severity failure;
        assert footer_idle = '1' and packer_idle = '1'
            report "V2-B9-J7-TB chain did not return idle"
            severity failure;

        source_word <= fn_source_word(0, 0);
        loop
            wait until rising_edge(clk);
            exit when source_ready = '1';
        end loop;
        source_word.valid <= '0';
        abort_run <= '1';
        wait until rising_edge(clk);
        abort_run <= '0';
        wait until falling_edge(clk);
        assert footer_idle = '1' and packer_idle = '1' and tvalid = '0'
            report "V2-B9-J7-TB abort did not clear chain"
            severity failure;

        report "LIDAR_V2_GPX_FACE_FOOTER_PASS proc_mhz=" &
            integer'image(G_CLK_MHZ) & " width=" &
            integer'image(G_OUTPUT_WIDTH) & " footer_lines=" &
            integer'image(C_FOOTER_LINES) severity note;
        finish;
        wait;
    end process p_test;

end architecture sim;

entity tb_lidar_gpx_face_footer_builder_150_32 is end entity;
architecture sim of tb_lidar_gpx_face_footer_builder_150_32 is begin
    u : entity work.tb_lidar_gpx_face_footer_builder
        generic map (G_CLK_MHZ => 150, G_OUTPUT_WIDTH => 32);
end architecture;

entity tb_lidar_gpx_face_footer_builder_150_64 is end entity;
architecture sim of tb_lidar_gpx_face_footer_builder_150_64 is begin
    u : entity work.tb_lidar_gpx_face_footer_builder
        generic map (G_CLK_MHZ => 150, G_OUTPUT_WIDTH => 64);
end architecture;

entity tb_lidar_gpx_face_footer_builder_150_128 is end entity;
architecture sim of tb_lidar_gpx_face_footer_builder_150_128 is begin
    u : entity work.tb_lidar_gpx_face_footer_builder
        generic map (G_CLK_MHZ => 150, G_OUTPUT_WIDTH => 128);
end architecture;

entity tb_lidar_gpx_face_footer_builder_200_32 is end entity;
architecture sim of tb_lidar_gpx_face_footer_builder_200_32 is begin
    u : entity work.tb_lidar_gpx_face_footer_builder
        generic map (G_CLK_MHZ => 200, G_OUTPUT_WIDTH => 32);
end architecture;

entity tb_lidar_gpx_face_footer_builder_200_64 is end entity;
architecture sim of tb_lidar_gpx_face_footer_builder_200_64 is begin
    u : entity work.tb_lidar_gpx_face_footer_builder
        generic map (G_CLK_MHZ => 200, G_OUTPUT_WIDTH => 64);
end architecture;

entity tb_lidar_gpx_face_footer_builder_200_128 is end entity;
architecture sim of tb_lidar_gpx_face_footer_builder_200_128 is begin
    u : entity work.tb_lidar_gpx_face_footer_builder
        generic map (G_CLK_MHZ => 200, G_OUTPUT_WIDTH => 128);
end architecture;
