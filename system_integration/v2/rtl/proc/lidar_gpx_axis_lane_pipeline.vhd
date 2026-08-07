library ieee;
use ieee.std_logic_1164.all;

use work.lidar_build_pkg.all;
use work.lidar_gpx_data_pkg.all;
use work.lidar_gpx_vdma_pkg.all;

-- One canonical output lane. B8 Cells remain width independent until the
-- final packer; this is the only 32/64/128-bit transport conversion point.
entity lidar_gpx_axis_lane_pipeline is
    generic (
        G_BUILD_CONFIG : lidar_build_config_t := C_DEFAULT_BUILD_CONFIG;
        G_LANE_RISE    : boolean := true
    );
    port (
        i_clk   : in std_logic;
        i_rst_n : in std_logic;
        i_abort : in std_logic;

        i_active_profile : in gpx_vdma_lane_profile_t;

        i_cell_event : in  gpx_frame_cell_event_t;
        o_cell_ready : out std_logic;

        i_frame_close_event : in  gpx_frame_close_event_t;
        o_frame_close_ready : out std_logic;

        o_m_axis_tdata  : out std_logic_vector(
            G_BUILD_CONFIG.output_width - 1 downto 0);
        o_m_axis_tkeep  : out std_logic_vector(
            G_BUILD_CONFIG.output_width / 8 - 1 downto 0);
        o_m_axis_tstrb  : out std_logic_vector(
            G_BUILD_CONFIG.output_width / 8 - 1 downto 0);
        o_m_axis_tuser  : out std_logic_vector(0 downto 0);
        o_m_axis_tvalid : out std_logic;
        o_m_axis_tlast  : out std_logic;
        i_m_axis_tready : in  std_logic;

        o_line_done      : out std_logic;
        o_frame_done     : out std_logic;
        o_footer_emitted : out std_logic;
        o_idle           : out std_logic
    );
end entity lidar_gpx_axis_lane_pipeline;

architecture rtl of lidar_gpx_axis_lane_pipeline is

    signal cell_input_0_r : gpx_frame_cell_event_t :=
        C_GPX_FRAME_CELL_EVENT_IDLE;
    signal cell_input_1_r : gpx_frame_cell_event_t :=
        C_GPX_FRAME_CELL_EVENT_IDLE;
    signal cell_input_count_r : natural range 0 to 2 := 0;
    signal cell_input_read_sel_r : std_logic := '0';
    signal cell_input_write_sel_r : std_logic := '0';
    signal cell_input_ready_r : std_logic := '1';
    signal buffered_cell_event_c : gpx_frame_cell_event_t;
    signal buffered_cell_ready_c : std_logic;
    signal cell_input_empty_c : std_logic;

    signal serializer_word_c : gpx_vdma_word_event_t;
    signal serializer_word_ready_r : std_logic := '1';
    signal word_input_0_r : gpx_vdma_word_event_t :=
        C_GPX_VDMA_WORD_EVENT_IDLE;
    signal word_input_1_r : gpx_vdma_word_event_t :=
        C_GPX_VDMA_WORD_EVENT_IDLE;
    signal word_input_count_r : natural range 0 to 2 := 0;
    signal word_input_read_sel_r : std_logic := '0';
    signal word_input_write_sel_r : std_logic := '0';
    signal word_input_empty_c : std_logic;
    signal cell_word_c : gpx_vdma_word_event_t;
    signal cell_word_ready_c : std_logic;
    signal serializer_idle_c : std_logic;

    signal shot_line_word_c : gpx_vdma_line_word_event_t;
    signal shot_line_ready_c : std_logic;
    signal shot_builder_idle_c : std_logic;

    signal hole_line_word_c : gpx_vdma_line_word_event_t;
    signal hole_line_ready_c : std_logic;
    signal hole_close_event_c : gpx_frame_close_event_t;
    signal hole_close_ready_c : std_logic;
    signal hole_idle_c : std_logic;

    signal close_input_event_c : gpx_frame_close_event_t;
    signal close_input_ready_c : std_logic;
    signal close_upstream_idle_c : std_logic;

    signal footer_line_word_c : gpx_vdma_line_word_event_t;
    signal footer_line_ready_c : std_logic;
    signal footer_idle_c : std_logic;
    signal packer_idle_c : std_logic;

begin

    assert fn_validate_build_config(G_BUILD_CONFIG) = CFG_OK
        report "V2-K06-LANE-001 invalid build configuration"
        severity failure;

    o_cell_ready <= cell_input_ready_r;
    cell_input_empty_c <= '1' when cell_input_count_r = 0 else '0';
    word_input_empty_c <= '1' when word_input_count_r = 0 else '0';
    o_idle <= cell_input_empty_c and serializer_idle_c and
        word_input_empty_c and
        shot_builder_idle_c and hole_idle_c and footer_idle_c and
        packer_idle_c;

    -- The registered-ready two-entry boundary removes the complete
    -- serializer/Shot/Hole/Footer/TREADY cone from B8. Circular slot pointers
    -- avoid shifting the wide Cell record on pop, so downstream READY changes
    -- only the small count/read-pointer registers. It also absorbs the
    -- one-cycle stale-ready case without reducing a sustained one-Cell/cycle
    -- source. Abort is handled synchronously inside the buffer and therefore
    -- does not become a combinational qualifier on the B8 ready path.
    p_cell_input_skid : process (i_clk)
        variable count_v : natural range 0 to 2;
        variable push_v : boolean;
        variable pop_v : boolean;
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' or i_abort = '1' then
                cell_input_0_r <= C_GPX_FRAME_CELL_EVENT_IDLE;
                cell_input_1_r <= C_GPX_FRAME_CELL_EVENT_IDLE;
                cell_input_count_r <= 0;
                cell_input_read_sel_r <= '0';
                cell_input_write_sel_r <= '0';
                cell_input_ready_r <= '1';
            else
                count_v := cell_input_count_r;
                push_v := i_cell_event.valid = '1' and
                    cell_input_ready_r = '1';
                pop_v := cell_input_count_r /= 0 and
                    buffered_cell_ready_c = '1';

                if pop_v then
                    cell_input_read_sel_r <=
                        not cell_input_read_sel_r;
                    count_v := count_v - 1;
                end if;

                if push_v then
                    if cell_input_write_sel_r = '0' then
                        cell_input_0_r <= i_cell_event;
                    else
                        cell_input_1_r <= i_cell_event;
                    end if;
                    cell_input_write_sel_r <=
                        not cell_input_write_sel_r;
                    count_v := count_v + 1;
                end if;

                cell_input_count_r <= count_v;
                if count_v = 2 then
                    cell_input_ready_r <= '0';
                else
                    cell_input_ready_r <= '1';
                end if;
            end if;
        end if;
    end process p_cell_input_skid;

    p_buffered_cell : process (all)
        variable value_v : gpx_frame_cell_event_t;
    begin
        if cell_input_read_sel_r = '0' then
            value_v := cell_input_0_r;
        else
            value_v := cell_input_1_r;
        end if;
        if cell_input_count_r = 0 then
            value_v.valid := '0';
        else
            value_v.valid := '1';
        end if;
        buffered_cell_event_c <= value_v;
    end process p_buffered_cell;

    -- A second registered-ready boundary keeps Shot/Footer state and AXIS
    -- backpressure out of the serializer's wide Cell capture enable. Circular
    -- storage preserves one canonical Word per clock without moving records
    -- during pop.
    p_word_input_skid : process (i_clk)
        variable count_v : natural range 0 to 2;
        variable push_v : boolean;
        variable pop_v : boolean;
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' or i_abort = '1' then
                word_input_0_r <= C_GPX_VDMA_WORD_EVENT_IDLE;
                word_input_1_r <= C_GPX_VDMA_WORD_EVENT_IDLE;
                word_input_count_r <= 0;
                word_input_read_sel_r <= '0';
                word_input_write_sel_r <= '0';
                serializer_word_ready_r <= '1';
            else
                count_v := word_input_count_r;
                push_v := serializer_word_c.valid = '1' and
                    serializer_word_ready_r = '1';
                pop_v := word_input_count_r /= 0 and
                    cell_word_ready_c = '1';

                if pop_v then
                    word_input_read_sel_r <= not word_input_read_sel_r;
                    count_v := count_v - 1;
                end if;

                if push_v then
                    if word_input_write_sel_r = '0' then
                        word_input_0_r <= serializer_word_c;
                    else
                        word_input_1_r <= serializer_word_c;
                    end if;
                    word_input_write_sel_r <= not word_input_write_sel_r;
                    count_v := count_v + 1;
                end if;

                word_input_count_r <= count_v;
                if count_v = 2 then
                    serializer_word_ready_r <= '0';
                else
                    serializer_word_ready_r <= '1';
                end if;
            end if;
        end if;
    end process p_word_input_skid;

    p_buffered_word : process (all)
        variable value_v : gpx_vdma_word_event_t;
    begin
        if word_input_read_sel_r = '0' then
            value_v := word_input_0_r;
        else
            value_v := word_input_1_r;
        end if;
        if word_input_count_r = 0 then
            value_v.valid := '0';
        else
            value_v.valid := '1';
        end if;
        cell_word_c <= value_v;
    end process p_buffered_word;

    -- A B8 Cell may already be buffered or its canonical Words may still be
    -- inside the serializer/Shot-Line builder. Hold Face Close until all of
    -- them are empty so Footer can never overtake that Cell.
    close_upstream_idle_c <= cell_input_empty_c and serializer_idle_c and
        word_input_empty_c and shot_builder_idle_c and
        not i_cell_event.valid;
    o_frame_close_ready <= close_input_ready_c and close_upstream_idle_c;

    p_close_order_gate : process (all)
        variable value_v : gpx_frame_close_event_t;
    begin
        value_v := i_frame_close_event;
        if close_upstream_idle_c = '0' then
            value_v.valid := '0';
        end if;
        close_input_event_c <= value_v;
    end process p_close_order_gate;

    u_serializer : entity work.lidar_gpx_cell_word_serializer
        port map (
            i_clk => i_clk,
            i_rst_n => i_rst_n,
            i_abort => i_abort,
            i_cell_event => buffered_cell_event_c,
            o_cell_ready => buffered_cell_ready_c,
            o_word_event => serializer_word_c,
            i_word_ready => serializer_word_ready_r,
            o_idle => serializer_idle_c
        );

    u_shot_line : entity work.lidar_gpx_shot_line_builder
        port map (
            i_clk => i_clk,
            i_rst_n => i_rst_n,
            i_abort => i_abort,
            i_cell_word => cell_word_c,
            o_cell_word_ready => cell_word_ready_c,
            o_line_word => shot_line_word_c,
            i_line_word_ready => shot_line_ready_c,
            o_idle => shot_builder_idle_c
        );

    u_hole_lines : entity work.lidar_gpx_hole_line_expander
        port map (
            i_clk => i_clk,
            i_rst_n => i_rst_n,
            i_abort => i_abort,
            i_active_slot_count => i_active_profile.slot_count,
            i_active_cell_word_count => i_active_profile.cell_word_count,
            i_real_line_word => shot_line_word_c,
            o_real_line_word_ready => shot_line_ready_c,
            i_frame_close_event => close_input_event_c,
            o_frame_close_ready => close_input_ready_c,
            o_line_word => hole_line_word_c,
            i_line_word_ready => hole_line_ready_c,
            o_frame_close_event => hole_close_event_c,
            i_frame_close_ready => hole_close_ready_c,
            o_idle => hole_idle_c
        );

    u_footer : entity work.lidar_gpx_face_footer_builder
        generic map (
            G_BUILD_CONFIG => G_BUILD_CONFIG,
            G_LANE_RISE => G_LANE_RISE
        )
        port map (
            i_clk => i_clk,
            i_rst_n => i_rst_n,
            i_abort => i_abort,
            i_active_profile => i_active_profile,
            i_line_word => hole_line_word_c,
            o_line_word_ready => hole_line_ready_c,
            i_frame_close_event => hole_close_event_c,
            o_frame_close_ready => hole_close_ready_c,
            o_line_word => footer_line_word_c,
            i_line_word_ready => footer_line_ready_c,
            o_active_hsize_bytes => open,
            o_active_vsize_lines => open,
            o_stride_bytes => open,
            o_footer_emitted => o_footer_emitted,
            o_idle => footer_idle_c
        );

    u_axis_packer : entity work.lidar_gpx_axis_word_packer
        generic map (
            G_OUTPUT_WIDTH => G_BUILD_CONFIG.output_width
        )
        port map (
            i_clk => i_clk,
            i_rst_n => i_rst_n,
            i_abort => i_abort,
            i_line_word => footer_line_word_c,
            o_line_word_ready => footer_line_ready_c,
            o_m_axis_tdata => o_m_axis_tdata,
            o_m_axis_tkeep => o_m_axis_tkeep,
            o_m_axis_tstrb => o_m_axis_tstrb,
            o_m_axis_tuser => o_m_axis_tuser,
            o_m_axis_tvalid => o_m_axis_tvalid,
            o_m_axis_tlast => o_m_axis_tlast,
            i_m_axis_tready => i_m_axis_tready,
            o_line_done => o_line_done,
            o_frame_done => o_frame_done,
            o_idle => packer_idle_c
        );

    -- synthesis translate_off
    p_contract : process (i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '1' and i_abort = '0' then
                if i_cell_event.valid = '1' then
                    assert i_active_profile.valid = '1' and
                           i_active_profile.enabled = '1'
                        report "V2-K06-LANE-002 Cell arrived without active lane profile"
                        severity failure;
                end if;
                if i_frame_close_event.valid = '1' then
                    assert i_active_profile.valid = '1' and
                           i_active_profile.enabled = '1'
                        report "V2-K06-LANE-003 Close arrived without active lane profile"
                        severity failure;
                end if;
                if i_frame_close_event.valid = '1' and
                   o_frame_close_ready = '1' then
                    assert serializer_idle_c = '1' and
                           shot_builder_idle_c = '1' and
                           word_input_empty_c = '1' and
                           cell_input_empty_c = '1' and
                           i_cell_event.valid = '0'
                        report "V2-K06-LANE-004 Face Close overtook a Cell"
                        severity failure;
                end if;
            end if;
        end if;
    end process p_contract;
    -- synthesis translate_on

end architecture rtl;
