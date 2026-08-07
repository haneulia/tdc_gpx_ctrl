library ieee;
use ieee.std_logic_1164.all;

use work.lidar_build_pkg.all;
use work.lidar_gpx_data_pkg.all;
use work.lidar_gpx_vdma_pkg.all;

-- Dual-lane B8-to-AXIS owner. Face-close delivery and final AXIS completion
-- are separate: o_frame_output_done pulses only after every enabled lane's
-- Footer Beat has been accepted under TREADY.
entity lidar_gpx_axis_output_subsystem is
    generic (
        G_BUILD_CONFIG : lidar_build_config_t := C_DEFAULT_BUILD_CONFIG
    );
    port (
        i_clk   : in std_logic;
        i_rst_n : in std_logic;
        i_abort : in std_logic;

        i_rise_active_profile : in gpx_vdma_lane_profile_t;
        i_fall_active_profile : in gpx_vdma_lane_profile_t;

        i_rise_event : in  gpx_frame_cell_event_t;
        o_rise_ready : out std_logic;
        i_fall_event : in  gpx_frame_cell_event_t;
        o_fall_ready : out std_logic;

        i_frame_close_event : in  gpx_frame_close_event_t;
        o_frame_close_ready : out std_logic;
        o_frame_output_done : out std_logic;

        o_rise_tdata  : out std_logic_vector(
            G_BUILD_CONFIG.output_width - 1 downto 0);
        o_rise_tkeep  : out std_logic_vector(
            G_BUILD_CONFIG.output_width / 8 - 1 downto 0);
        o_rise_tstrb  : out std_logic_vector(
            G_BUILD_CONFIG.output_width / 8 - 1 downto 0);
        o_rise_tuser  : out std_logic_vector(0 downto 0);
        o_rise_tvalid : out std_logic;
        o_rise_tlast  : out std_logic;
        i_rise_tready : in  std_logic;

        o_fall_tdata  : out std_logic_vector(
            G_BUILD_CONFIG.output_width - 1 downto 0);
        o_fall_tkeep  : out std_logic_vector(
            G_BUILD_CONFIG.output_width / 8 - 1 downto 0);
        o_fall_tstrb  : out std_logic_vector(
            G_BUILD_CONFIG.output_width / 8 - 1 downto 0);
        o_fall_tuser  : out std_logic_vector(0 downto 0);
        o_fall_tvalid : out std_logic;
        o_fall_tlast  : out std_logic;
        i_fall_tready : in  std_logic;

        o_rise_line_done  : out std_logic;
        o_fall_line_done  : out std_logic;
        o_rise_frame_done : out std_logic;
        o_fall_frame_done : out std_logic;
        o_idle            : out std_logic
    );
end entity lidar_gpx_axis_output_subsystem;

architecture rtl of lidar_gpx_axis_output_subsystem is

    constant C_ZERO_MASK : chip_mask_t := (others => '0');
    constant C_HAS_RISE : boolean :=
        G_BUILD_CONFIG.rise_capability_mask /= C_ZERO_MASK;
    constant C_HAS_FALL : boolean :=
        G_BUILD_CONFIG.fall_capability_mask /= C_ZERO_MASK;

    type close_state_t is (CLOSE_IDLE, CLOSE_WAIT_DONE);
    signal close_state_r : close_state_t := CLOSE_IDLE;
    signal rise_expected_r : std_logic := '0';
    signal fall_expected_r : std_logic := '0';
    signal rise_done_r : std_logic := '0';
    signal fall_done_r : std_logic := '0';
    signal frame_output_done_r : std_logic := '0';

    signal fork_close_ready_c : std_logic;
    signal fork_idle_c : std_logic;
    signal fork_input_event_c : gpx_frame_close_event_t;
    signal fork_rise_event_c : gpx_frame_close_event_t;
    signal fork_fall_event_c : gpx_frame_close_event_t;
    signal fork_rise_ready_c : std_logic;
    signal fork_fall_ready_c : std_logic;

    signal rise_cell_ready_c : std_logic;
    signal fall_cell_ready_c : std_logic;
    signal rise_line_done_c : std_logic;
    signal fall_line_done_c : std_logic;
    signal rise_frame_done_c : std_logic;
    signal fall_frame_done_c : std_logic;
    signal rise_lane_idle_c : std_logic;
    signal fall_lane_idle_c : std_logic;

    signal rise_tdata_c : std_logic_vector(
        G_BUILD_CONFIG.output_width - 1 downto 0);
    signal rise_tkeep_c : std_logic_vector(
        G_BUILD_CONFIG.output_width / 8 - 1 downto 0);
    signal rise_tstrb_c : std_logic_vector(
        G_BUILD_CONFIG.output_width / 8 - 1 downto 0);
    signal rise_tuser_c : std_logic_vector(0 downto 0);
    signal rise_tvalid_c : std_logic;
    signal rise_tlast_c : std_logic;

    signal fall_tdata_c : std_logic_vector(
        G_BUILD_CONFIG.output_width - 1 downto 0);
    signal fall_tkeep_c : std_logic_vector(
        G_BUILD_CONFIG.output_width / 8 - 1 downto 0);
    signal fall_tstrb_c : std_logic_vector(
        G_BUILD_CONFIG.output_width / 8 - 1 downto 0);
    signal fall_tuser_c : std_logic_vector(0 downto 0);
    signal fall_tvalid_c : std_logic;
    signal fall_tlast_c : std_logic;

begin

    assert fn_validate_build_config(G_BUILD_CONFIG) = CFG_OK
        report "V2-K06-OUT-001 invalid build configuration"
        severity failure;

    o_frame_close_ready <= fork_close_ready_c
        when close_state_r = CLOSE_IDLE else '0';
    o_frame_output_done <= frame_output_done_r;
    o_rise_ready <= rise_cell_ready_c;
    o_fall_ready <= fall_cell_ready_c;
    o_rise_line_done <= rise_line_done_c;
    o_fall_line_done <= fall_line_done_c;
    o_rise_frame_done <= rise_frame_done_c;
    o_fall_frame_done <= fall_frame_done_c;
    o_idle <= '1' when close_state_r = CLOSE_IDLE and fork_idle_c = '1' and
        rise_lane_idle_c = '1' and fall_lane_idle_c = '1' else '0';

    o_rise_tdata <= rise_tdata_c;
    o_rise_tkeep <= rise_tkeep_c;
    o_rise_tstrb <= rise_tstrb_c;
    o_rise_tuser <= rise_tuser_c;
    o_rise_tvalid <= rise_tvalid_c;
    o_rise_tlast <= rise_tlast_c;
    o_fall_tdata <= fall_tdata_c;
    o_fall_tkeep <= fall_tkeep_c;
    o_fall_tstrb <= fall_tstrb_c;
    o_fall_tuser <= fall_tuser_c;
    o_fall_tvalid <= fall_tvalid_c;
    o_fall_tlast <= fall_tlast_c;

    -- B5-B8 keeps valid asserted until final Frame completion. The fork must
    -- see that held event only for the first delivery handshake; otherwise it
    -- would re-emit the same Footer whenever both lane slots become empty.
    p_fork_input_gate : process (all)
        variable value_v : gpx_frame_close_event_t;
    begin
        value_v := i_frame_close_event;
        if close_state_r /= CLOSE_IDLE then
            value_v.valid := '0';
        end if;
        fork_input_event_c <= value_v;
    end process p_fork_input_gate;

    u_close_fork : entity work.lidar_gpx_frame_close_fork
        port map (
            i_clk => i_clk,
            i_rst_n => i_rst_n,
            i_abort => i_abort,
            i_rise_enable => i_rise_active_profile.enabled,
            i_fall_enable => i_fall_active_profile.enabled,
            i_close_event => fork_input_event_c,
            o_close_ready => fork_close_ready_c,
            o_rise_close_event => fork_rise_event_c,
            i_rise_close_ready => fork_rise_ready_c,
            o_fall_close_event => fork_fall_event_c,
            i_fall_close_ready => fork_fall_ready_c,
            o_idle => fork_idle_c
        );

    gen_rise_lane : if C_HAS_RISE generate
        signal footer_emitted_c : std_logic;
    begin
        u_rise_lane : entity work.lidar_gpx_axis_lane_pipeline
            generic map (
                G_BUILD_CONFIG => G_BUILD_CONFIG,
                G_LANE_RISE => true
            )
            port map (
                i_clk => i_clk,
                i_rst_n => i_rst_n,
                i_abort => i_abort,
                i_active_profile => i_rise_active_profile,
                i_cell_event => i_rise_event,
                o_cell_ready => rise_cell_ready_c,
                i_frame_close_event => fork_rise_event_c,
                o_frame_close_ready => fork_rise_ready_c,
                o_m_axis_tdata => rise_tdata_c,
                o_m_axis_tkeep => rise_tkeep_c,
                o_m_axis_tstrb => rise_tstrb_c,
                o_m_axis_tuser => rise_tuser_c,
                o_m_axis_tvalid => rise_tvalid_c,
                o_m_axis_tlast => rise_tlast_c,
                i_m_axis_tready => i_rise_tready,
                o_line_done => rise_line_done_c,
                o_frame_done => rise_frame_done_c,
                o_footer_emitted => footer_emitted_c,
                o_idle => rise_lane_idle_c
            );
    end generate gen_rise_lane;

    gen_no_rise_lane : if not C_HAS_RISE generate
    begin
        rise_cell_ready_c <= '1';
        fork_rise_ready_c <= '1';
        rise_tdata_c <= (others => '0');
        rise_tkeep_c <= (others => '0');
        rise_tstrb_c <= (others => '0');
        rise_tuser_c <= (others => '0');
        rise_tvalid_c <= '0';
        rise_tlast_c <= '0';
        rise_line_done_c <= '0';
        rise_frame_done_c <= '0';
        rise_lane_idle_c <= '1';
    end generate gen_no_rise_lane;

    gen_fall_lane : if C_HAS_FALL generate
        signal footer_emitted_c : std_logic;
    begin
        u_fall_lane : entity work.lidar_gpx_axis_lane_pipeline
            generic map (
                G_BUILD_CONFIG => G_BUILD_CONFIG,
                G_LANE_RISE => false
            )
            port map (
                i_clk => i_clk,
                i_rst_n => i_rst_n,
                i_abort => i_abort,
                i_active_profile => i_fall_active_profile,
                i_cell_event => i_fall_event,
                o_cell_ready => fall_cell_ready_c,
                i_frame_close_event => fork_fall_event_c,
                o_frame_close_ready => fork_fall_ready_c,
                o_m_axis_tdata => fall_tdata_c,
                o_m_axis_tkeep => fall_tkeep_c,
                o_m_axis_tstrb => fall_tstrb_c,
                o_m_axis_tuser => fall_tuser_c,
                o_m_axis_tvalid => fall_tvalid_c,
                o_m_axis_tlast => fall_tlast_c,
                i_m_axis_tready => i_fall_tready,
                o_line_done => fall_line_done_c,
                o_frame_done => fall_frame_done_c,
                o_footer_emitted => footer_emitted_c,
                o_idle => fall_lane_idle_c
            );
    end generate gen_fall_lane;

    gen_no_fall_lane : if not C_HAS_FALL generate
    begin
        fall_cell_ready_c <= '1';
        fork_fall_ready_c <= '1';
        fall_tdata_c <= (others => '0');
        fall_tkeep_c <= (others => '0');
        fall_tstrb_c <= (others => '0');
        fall_tuser_c <= (others => '0');
        fall_tvalid_c <= '0';
        fall_tlast_c <= '0';
        fall_line_done_c <= '0';
        fall_frame_done_c <= '0';
        fall_lane_idle_c <= '1';
    end generate gen_no_fall_lane;

    p_close_completion : process (i_clk)
        variable rise_done_v : std_logic;
        variable fall_done_v : std_logic;
        variable rise_expected_v : std_logic;
        variable fall_expected_v : std_logic;
    begin
        if rising_edge(i_clk) then
            frame_output_done_r <= '0';
            if i_rst_n = '0' or i_abort = '1' then
                close_state_r <= CLOSE_IDLE;
                rise_expected_r <= '0';
                fall_expected_r <= '0';
                rise_done_r <= '0';
                fall_done_r <= '0';
            else
                case close_state_r is
                    when CLOSE_IDLE =>
                        rise_done_r <= '0';
                        fall_done_r <= '0';
                        if i_frame_close_event.valid = '1' and
                           o_frame_close_ready = '1' then
                            if C_HAS_RISE then
                                rise_expected_v :=
                                    i_rise_active_profile.enabled;
                            else
                                rise_expected_v := '0';
                            end if;
                            if C_HAS_FALL then
                                fall_expected_v :=
                                    i_fall_active_profile.enabled;
                            else
                                fall_expected_v := '0';
                            end if;
                            rise_expected_r <= rise_expected_v;
                            fall_expected_r <= fall_expected_v;
                            if rise_expected_v = '0' and
                               fall_expected_v = '0' then
                                frame_output_done_r <= '1';
                            else
                                close_state_r <= CLOSE_WAIT_DONE;
                            end if;
                        end if;

                    when CLOSE_WAIT_DONE =>
                        rise_done_v := rise_done_r or rise_frame_done_c;
                        fall_done_v := fall_done_r or fall_frame_done_c;
                        rise_done_r <= rise_done_v;
                        fall_done_r <= fall_done_v;
                        if (rise_expected_r = '0' or rise_done_v = '1') and
                           (fall_expected_r = '0' or fall_done_v = '1') then
                            frame_output_done_r <= '1';
                            close_state_r <= CLOSE_IDLE;
                            rise_expected_r <= '0';
                            fall_expected_r <= '0';
                        end if;
                end case;
            end if;
        end if;
    end process p_close_completion;

    -- synthesis translate_off
    p_contract : process (i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '1' and i_abort = '0' then
                if rise_frame_done_c = '1' then
                    assert close_state_r = CLOSE_WAIT_DONE and
                           rise_expected_r = '1'
                        report "V2-K06-OUT-004 Rise frame completion outside transaction"
                        severity failure;
                end if;
                if fall_frame_done_c = '1' then
                    assert close_state_r = CLOSE_WAIT_DONE and
                           fall_expected_r = '1'
                        report "V2-K06-OUT-005 Fall frame completion outside transaction"
                        severity failure;
                end if;
                assert not (rise_frame_done_c = '1' and
                    rise_expected_r = '0' and close_state_r = CLOSE_WAIT_DONE)
                    report "V2-K06-OUT-002 unexpected Rise frame completion"
                    severity failure;
                assert not (fall_frame_done_c = '1' and
                    fall_expected_r = '0' and close_state_r = CLOSE_WAIT_DONE)
                    report "V2-K06-OUT-003 unexpected Fall frame completion"
                    severity failure;
            end if;
        end if;
    end process p_contract;
    -- synthesis translate_on

end architecture rtl;
