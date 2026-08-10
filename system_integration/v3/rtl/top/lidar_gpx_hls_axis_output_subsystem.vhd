library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_build_pkg.all;
use work.lidar_gpx_data_pkg.all;
use work.lidar_gpx_vdma_pkg.all;
use work.lidar_v3_hls_contract_pkg.all;

-- H5 Rise/Fall 이중 lane 출력 소유 블록이다.
-- H4는 출력 폭과 무관한 canonical 32-bit Word를 만든다. 유지 RTL packer만
-- 이 Word를 합성 시 선택한 32/64-bit AXI4-Stream Beat로 결합한다. Face
-- 완료는 활성 lane 각각의 마지막 Face Footer Beat가 TREADY로 승인된 뒤에만
-- 보고한다.
entity lidar_gpx_hls_axis_output_subsystem is
    generic (
        G_BUILD_CONFIG : lidar_build_config_t := C_DEFAULT_BUILD_CONFIG
    );
    port (
        i_clk   : in std_logic;
        i_rst_n : in std_logic;
        i_abort : in std_logic;
        i_clear_sticky : in std_logic;
        i_active_version : in unsigned(15 downto 0);

        i_rise_active_profile : in gpx_vdma_lane_profile_t;
        i_fall_active_profile : in gpx_vdma_lane_profile_t;
        i_rise_event : in gpx_frame_cell_event_t;
        o_rise_ready : out std_logic;
        i_fall_event : in gpx_frame_cell_event_t;
        o_fall_ready : out std_logic;
        i_frame_close_event : in gpx_frame_close_event_t;
        o_frame_close_ready : out std_logic;
        o_frame_output_done : out std_logic;

        o_rise_tdata : out std_logic_vector(
            G_BUILD_CONFIG.output_width - 1 downto 0);
        o_rise_tkeep : out std_logic_vector(
            G_BUILD_CONFIG.output_width / 8 - 1 downto 0);
        o_rise_tstrb : out std_logic_vector(
            G_BUILD_CONFIG.output_width / 8 - 1 downto 0);
        o_rise_tuser : out std_logic_vector(0 downto 0);
        o_rise_tvalid : out std_logic;
        o_rise_tlast : out std_logic;
        i_rise_tready : in std_logic;

        o_fall_tdata : out std_logic_vector(
            G_BUILD_CONFIG.output_width - 1 downto 0);
        o_fall_tkeep : out std_logic_vector(
            G_BUILD_CONFIG.output_width / 8 - 1 downto 0);
        o_fall_tstrb : out std_logic_vector(
            G_BUILD_CONFIG.output_width / 8 - 1 downto 0);
        o_fall_tuser : out std_logic_vector(0 downto 0);
        o_fall_tvalid : out std_logic;
        o_fall_tlast : out std_logic;
        i_fall_tready : in std_logic;

        o_rise_line_done : out std_logic;
        o_fall_line_done : out std_logic;
        o_rise_frame_done : out std_logic;
        o_fall_frame_done : out std_logic;
        o_rise_emitted_lines : out unsigned(16 downto 0);
        o_fall_emitted_lines : out unsigned(16 downto 0);
        o_idle : out std_logic;
        o_rise_fault_pulse : out lidar_gpx_word_formatter_faults_t;
        o_fall_fault_pulse : out lidar_gpx_word_formatter_faults_t;
        o_rise_fault_sticky : out lidar_gpx_word_formatter_faults_t;
        o_fall_fault_sticky : out lidar_gpx_word_formatter_faults_t
    );
end entity lidar_gpx_hls_axis_output_subsystem;

architecture rtl of lidar_gpx_hls_axis_output_subsystem is
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

    signal fork_input_s : gpx_frame_close_event_t :=
        C_GPX_FRAME_CLOSE_EVENT_IDLE;
    signal fork_rise_s : gpx_frame_close_event_t :=
        C_GPX_FRAME_CLOSE_EVENT_IDLE;
    signal fork_fall_s : gpx_frame_close_event_t :=
        C_GPX_FRAME_CLOSE_EVENT_IDLE;
    signal fork_input_ready_s : std_logic;
    signal fork_rise_ready_s : std_logic;
    signal fork_fall_ready_s : std_logic;
    signal fork_idle_s : std_logic;

    signal rise_word_s : gpx_vdma_line_word_event_t :=
        C_GPX_VDMA_LINE_WORD_EVENT_IDLE;
    signal fall_word_s : gpx_vdma_line_word_event_t :=
        C_GPX_VDMA_LINE_WORD_EVENT_IDLE;
    signal rise_word_ready_s : std_logic;
    signal fall_word_ready_s : std_logic;
    signal rise_formatter_idle_s : std_logic;
    signal fall_formatter_idle_s : std_logic;
    signal rise_packer_idle_s : std_logic;
    signal fall_packer_idle_s : std_logic;
    signal rise_frame_done_s : std_logic;
    signal fall_frame_done_s : std_logic;
begin
    assert fn_validate_build_config(G_BUILD_CONFIG) = CFG_OK
        report "V3-H5-OUT-001 invalid build configuration"
        severity failure;
    assert G_BUILD_CONFIG.output_width = 32 or
           G_BUILD_CONFIG.output_width = 64
        report "V3-H5-OUT-002 V3 output width must be 32 or 64"
        severity failure;

    o_frame_close_ready <= fork_input_ready_s
        when close_state_r = CLOSE_IDLE else '0';
    o_frame_output_done <= frame_output_done_r;
    o_rise_frame_done <= rise_frame_done_s;
    o_fall_frame_done <= fall_frame_done_s;
    o_idle <= '1' when close_state_r = CLOSE_IDLE and fork_idle_s = '1' and
        rise_formatter_idle_s = '1' and fall_formatter_idle_s = '1' and
        rise_packer_idle_s = '1' and fall_packer_idle_s = '1' else '0';

    p_close_gate : process (all)
        variable value_v : gpx_frame_close_event_t;
    begin
        value_v := i_frame_close_event;
        if close_state_r /= CLOSE_IDLE then
            value_v.valid := '0';
        end if;
        fork_input_s <= value_v;
    end process p_close_gate;

    u_close_fork : entity work.lidar_gpx_frame_close_fork
        port map (
            i_clk => i_clk,
            i_rst_n => i_rst_n,
            i_abort => i_abort,
            i_rise_enable => i_rise_active_profile.enabled,
            i_fall_enable => i_fall_active_profile.enabled,
            i_close_event => fork_input_s,
            o_close_ready => fork_input_ready_s,
            o_rise_close_event => fork_rise_s,
            i_rise_close_ready => fork_rise_ready_s,
            o_fall_close_event => fork_fall_s,
            i_fall_close_ready => fork_fall_ready_s,
            o_idle => fork_idle_s
        );

    gen_rise : if C_HAS_RISE generate
        signal footer_emitted_s : std_logic;
    begin
        u_h4_rise : entity work.lidar_gpx_lane_word_formatter_hls_adapter
            generic map (
                G_BUILD_CONFIG => G_BUILD_CONFIG,
                G_LANE_RISE => true
            )
            port map (
                i_clk => i_clk,
                i_rst_n => i_rst_n,
                i_abort => i_abort,
                i_clear_sticky => i_clear_sticky,
                i_active_profile => i_rise_active_profile,
                i_active_version => i_active_version,
                i_cell_event => i_rise_event,
                o_cell_ready => o_rise_ready,
                i_frame_close_event => fork_rise_s,
                o_frame_close_ready => fork_rise_ready_s,
                o_line_word => rise_word_s,
                i_line_word_ready => rise_word_ready_s,
                o_footer_emitted => footer_emitted_s,
                o_emitted_line_count => o_rise_emitted_lines,
                o_idle => rise_formatter_idle_s,
                o_fault_pulse => o_rise_fault_pulse,
                o_fault_sticky => o_rise_fault_sticky
            );

        u_rise_packer : entity work.lidar_gpx_axis_word_packer
            generic map (G_OUTPUT_WIDTH => G_BUILD_CONFIG.output_width)
            port map (
                i_clk => i_clk,
                i_rst_n => i_rst_n,
                i_abort => i_abort,
                i_line_word => rise_word_s,
                o_line_word_ready => rise_word_ready_s,
                o_m_axis_tdata => o_rise_tdata,
                o_m_axis_tkeep => o_rise_tkeep,
                o_m_axis_tstrb => o_rise_tstrb,
                o_m_axis_tuser => o_rise_tuser,
                o_m_axis_tvalid => o_rise_tvalid,
                o_m_axis_tlast => o_rise_tlast,
                i_m_axis_tready => i_rise_tready,
                o_line_done => o_rise_line_done,
                o_frame_done => rise_frame_done_s,
                o_idle => rise_packer_idle_s
            );
    end generate gen_rise;

    gen_no_rise : if not C_HAS_RISE generate
    begin
        o_rise_ready <= '1';
        fork_rise_ready_s <= '1';
        o_rise_tdata <= (others => '0');
        o_rise_tkeep <= (others => '0');
        o_rise_tstrb <= (others => '0');
        o_rise_tuser <= (others => '0');
        o_rise_tvalid <= '0';
        o_rise_tlast <= '0';
        o_rise_line_done <= '0';
        rise_frame_done_s <= '0';
        o_rise_emitted_lines <= (others => '0');
        rise_formatter_idle_s <= '1';
        rise_packer_idle_s <= '1';
        o_rise_fault_pulse <= C_LIDAR_GPX_WORD_FORMATTER_FAULTS_CLEAR;
        o_rise_fault_sticky <= C_LIDAR_GPX_WORD_FORMATTER_FAULTS_CLEAR;
    end generate gen_no_rise;

    gen_fall : if C_HAS_FALL generate
        signal footer_emitted_s : std_logic;
    begin
        u_h4_fall : entity work.lidar_gpx_lane_word_formatter_hls_adapter
            generic map (
                G_BUILD_CONFIG => G_BUILD_CONFIG,
                G_LANE_RISE => false
            )
            port map (
                i_clk => i_clk,
                i_rst_n => i_rst_n,
                i_abort => i_abort,
                i_clear_sticky => i_clear_sticky,
                i_active_profile => i_fall_active_profile,
                i_active_version => i_active_version,
                i_cell_event => i_fall_event,
                o_cell_ready => o_fall_ready,
                i_frame_close_event => fork_fall_s,
                o_frame_close_ready => fork_fall_ready_s,
                o_line_word => fall_word_s,
                i_line_word_ready => fall_word_ready_s,
                o_footer_emitted => footer_emitted_s,
                o_emitted_line_count => o_fall_emitted_lines,
                o_idle => fall_formatter_idle_s,
                o_fault_pulse => o_fall_fault_pulse,
                o_fault_sticky => o_fall_fault_sticky
            );

        u_fall_packer : entity work.lidar_gpx_axis_word_packer
            generic map (G_OUTPUT_WIDTH => G_BUILD_CONFIG.output_width)
            port map (
                i_clk => i_clk,
                i_rst_n => i_rst_n,
                i_abort => i_abort,
                i_line_word => fall_word_s,
                o_line_word_ready => fall_word_ready_s,
                o_m_axis_tdata => o_fall_tdata,
                o_m_axis_tkeep => o_fall_tkeep,
                o_m_axis_tstrb => o_fall_tstrb,
                o_m_axis_tuser => o_fall_tuser,
                o_m_axis_tvalid => o_fall_tvalid,
                o_m_axis_tlast => o_fall_tlast,
                i_m_axis_tready => i_fall_tready,
                o_line_done => o_fall_line_done,
                o_frame_done => fall_frame_done_s,
                o_idle => fall_packer_idle_s
            );
    end generate gen_fall;

    gen_no_fall : if not C_HAS_FALL generate
    begin
        o_fall_ready <= '1';
        fork_fall_ready_s <= '1';
        o_fall_tdata <= (others => '0');
        o_fall_tkeep <= (others => '0');
        o_fall_tstrb <= (others => '0');
        o_fall_tuser <= (others => '0');
        o_fall_tvalid <= '0';
        o_fall_tlast <= '0';
        o_fall_line_done <= '0';
        fall_frame_done_s <= '0';
        o_fall_emitted_lines <= (others => '0');
        fall_formatter_idle_s <= '1';
        fall_packer_idle_s <= '1';
        o_fall_fault_pulse <= C_LIDAR_GPX_WORD_FORMATTER_FAULTS_CLEAR;
        o_fall_fault_sticky <= C_LIDAR_GPX_WORD_FORMATTER_FAULTS_CLEAR;
    end generate gen_no_fall;

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
                        rise_done_v := rise_done_r or rise_frame_done_s;
                        fall_done_v := fall_done_r or fall_frame_done_s;
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
end architecture rtl;
