library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_build_pkg.all;
use work.lidar_config_types_pkg.all;
use work.lidar_event_types_pkg.all;
use work.lidar_gpx_pkg.all;
use work.lidar_gpx_image_pkg.all;
use work.lidar_gpx_event_pkg.all;
use work.lidar_gpx_data_pkg.all;
use work.lidar_gpx_vdma_pkg.all;
use work.lidar_v3_hls_contract_pkg.all;
use work.tdc_gpx_pkg.all;

-- H6 Parent 데이터 경계.
--
-- V2에서 검증한 TDC-GPX 물리 버스와 IFIFO 전체 Drain, Shot/STOP CDC 및
-- 비동기 결과 FIFO는 그대로 유지한다. 비동기 결과 FIFO 뒤의 Processing
-- 데이터 경로만 H5 혼합 RTL/HLS Top으로 교체한다.
--
-- Face-close ACK는 입력 사건을 H5에 전달한 시점이 아니라, 활성화된
-- Rise/Fall lane의 마지막 Face Footer Beat가 실제 AXI handshake 된 뒤에만
-- 발생한다. 따라서 AXI backpressure 중 다음 Face가 먼저 시작될 수 없다.
entity lidar_gpx_hls_parent_data_subsystem is
    generic (
        G_BUILD_CONFIG : lidar_build_config_t := C_DEFAULT_BUILD_CONFIG;
        G_SHOT_FIFO_DEPTH : positive := 16;
        G_STOP_FIFO_DEPTH : positive := 16;
        G_OEN_MODE : string := "DYNAMIC_CONNECTED";
        G_POWERUP_TIME_NS         : positive := c_DEFAULT_POWERUP_TIME_NS;
        G_RECOVERY_TIME_NS        : positive := c_DEFAULT_RECOVERY_TIME_NS;
        G_ALU_PULSE_TIME_NS       : positive := c_DEFAULT_ALU_PULSE_TIME_NS;
        G_BUS_IDLE_STABLE_TIME_NS : positive :=
            c_DEFAULT_BUS_IDLE_STABLE_TIME_NS;
        G_DRAIN_MARGIN_TIME_NS    : positive := 6000
    );
    port (
        i_proc_clk           : in  std_logic;
        i_proc_rst_n         : in  std_logic;
        i_proc_abort         : in  std_logic := '0';
        i_proc_clear_status  : in  std_logic := '0';
        i_proc_active_valid  : in  std_logic;
        i_proc_active_config : in  lidar_active_config_t;
        i_proc_shot          : in  shot_start_event_t;
        o_proc_shot_ready    : out std_logic;
        i_proc_stop_tdc      : in  std_logic;
        i_face_close_event   : in  face_close_event_t;
        o_face_close_ready   : out std_logic;

        i_rise_active_profile : in gpx_vdma_lane_profile_t;
        i_fall_active_profile : in gpx_vdma_lane_profile_t;

        o_rise_tdata : out std_logic_vector(
            G_BUILD_CONFIG.output_width - 1 downto 0);
        o_rise_tkeep : out std_logic_vector(
            G_BUILD_CONFIG.output_width / 8 - 1 downto 0);
        o_rise_tstrb : out std_logic_vector(
            G_BUILD_CONFIG.output_width / 8 - 1 downto 0);
        o_rise_tuser  : out std_logic_vector(0 downto 0);
        o_rise_tvalid : out std_logic;
        o_rise_tlast  : out std_logic;
        i_rise_tready : in  std_logic;

        o_fall_tdata : out std_logic_vector(
            G_BUILD_CONFIG.output_width - 1 downto 0);
        o_fall_tkeep : out std_logic_vector(
            G_BUILD_CONFIG.output_width / 8 - 1 downto 0);
        o_fall_tstrb : out std_logic_vector(
            G_BUILD_CONFIG.output_width / 8 - 1 downto 0);
        o_fall_tuser  : out std_logic_vector(0 downto 0);
        o_fall_tvalid : out std_logic;
        o_fall_tlast  : out std_logic;
        i_fall_tready : in  std_logic;

        o_shot_done         : out std_logic;
        o_shot_done_context : out shot_start_event_t;
        o_frame_output_done : out std_logic;
        o_proc_idle         : out std_logic;
        o_outstanding_shots : out unsigned(15 downto 0);
        o_decoder_inflight  : out unsigned(7 downto 0);
        o_rise_emitted_lines : out unsigned(16 downto 0);
        o_fall_emitted_lines : out unsigned(16 downto 0);
        o_context_fault_sticky : out std_logic;

        i_tdc_clk            : in  std_logic;
        i_tdc_rst_n          : in  std_logic;
        i_tdc_active_valid   : in  std_logic;
        i_tdc_active_config  : in  lidar_active_config_t;
        i_tdc_register_image : in  gpx_register_image_t;
        i_tdc_config_apply   : in  std_logic;
        o_tdc_config_ready   : out std_logic;
        o_tdc_config_done    : out std_logic;
        i_tdc_run_enable     : in  std_logic;
        i_tdc_soft_reset     : in  std_logic := '0';
        i_tdc_force_reinit   : in  std_logic := '0';
        i_tdc_clear_status   : in  std_logic := '0';
        o_tdc_safe           : out std_logic;
        o_tdc_shot_complete  : out std_logic;
        i_tdc_register_read  : in gpx_register_read_request_t :=
            C_GPX_REGISTER_READ_REQUEST_IDLE;
        o_tdc_register_read_ready : out std_logic;
        o_tdc_register_read_response : out gpx_register_read_response_t;
        i_tdc_register_read_response_ready : in std_logic := '1';
        o_cdc_reset_busy     : out std_logic;

        o_adr        : out gpx_bus_address_array_t;
        o_csn        : out chip_mask_t;
        o_rdn        : out chip_mask_t;
        o_wrn        : out chip_mask_t;
        o_oen        : out chip_mask_t;
        i_d          : in  gpx_bus_data_array_t;
        o_d          : out gpx_bus_data_array_t;
        o_d_tri      : out gpx_bus_data_array_t;
        i_ef1        : in  chip_mask_t;
        i_ef2        : in  chip_mask_t;
        i_lf1        : in  chip_mask_t;
        i_lf2        : in  chip_mask_t;
        i_irflag     : in  chip_mask_t;
        i_errflag    : in  chip_mask_t;
        o_stopdis    : out chip_mask_t;
        o_alutrigger : out chip_mask_t;
        o_puresn     : out chip_mask_t;

        o_active_mask   : out chip_mask_t;
        o_terminal_mask : out chip_mask_t;
        o_tdc_status    : out gpx_lane_status_array_t;
        o_tdc_faults    : out gpx_lane_faults_array_t;
        o_shot_drop_sticky : out std_logic;
        o_stop_drop_sticky : out std_logic;
        o_hit_fault_sticky : out gpx_hit_decoder_faults_t;
        o_cell_fault_sticky : out gpx_cell_collector_faults_t;
        o_frame_fault_sticky : out gpx_frame_assembler_faults_t;
        o_rise_formatter_fault_sticky :
            out lidar_gpx_word_formatter_faults_t;
        o_fall_formatter_fault_sticky :
            out lidar_gpx_word_formatter_faults_t
    );
end entity lidar_gpx_hls_parent_data_subsystem;

architecture rtl of lidar_gpx_hls_parent_data_subsystem is

    type face_close_state_t is (
        FACE_CLOSE_IDLE,
        FACE_CLOSE_SEND,
        FACE_CLOSE_WAIT_OUTPUT,
        FACE_CLOSE_ACK,
        FACE_CLOSE_RELEASE
    );

    signal face_close_state_r : face_close_state_t := FACE_CLOSE_IDLE;
    signal face_close_hold_r  : face_close_event_t :=
        C_FACE_CLOSE_EVENT_IDLE;
    signal outstanding_shots_r : unsigned(15 downto 0) :=
        (others => '0');
    signal context_fault_sticky_r : std_logic := '0';

    signal acquisition_shot_s : shot_start_event_t :=
        C_SHOT_START_EVENT_IDLE;
    signal acquisition_shot_ready_s : std_logic;
    signal raw_event_s : gpx_raw_event_t := C_GPX_RAW_EVENT_IDLE;
    signal raw_ready_s : std_logic;

    signal h5_face_close_ready_s : std_logic;
    signal h5_shot_done_s : std_logic;
    signal h5_shot_done_context_s : shot_start_event_t;
    signal h5_frame_output_done_s : std_logic;
    signal h5_idle_s : std_logic;

begin

    assert fn_validate_build_config(G_BUILD_CONFIG) = CFG_OK
        report "V3-H6-DATA-001 invalid build configuration"
        severity failure;
    assert G_BUILD_CONFIG.output_width = 32 or
           G_BUILD_CONFIG.output_width = 64
        report "V3-H6-DATA-002 output width must be 32 or 64"
        severity failure;

    -- Face 종료 처리 중에는 새로운 Shot을 물리 TDC 도메인으로 넘기지 않는다.
    -- 이 조합 게이트는 valid/ready 경계의 허용 조건만 결정하며 payload는
    -- i_proc_shot의 등록된 사건 레코드를 그대로 전달한다.
    p_shot_admission : process (all)
        variable admitted_shot_v : shot_start_event_t;
    begin
        admitted_shot_v := i_proc_shot;
        if face_close_state_r /= FACE_CLOSE_IDLE or
           i_face_close_event.valid = '1' or i_proc_abort = '1' then
            admitted_shot_v.valid := '0';
        end if;
        acquisition_shot_s <= admitted_shot_v;
    end process p_shot_admission;

    o_proc_shot_ready <= acquisition_shot_ready_s when
        face_close_state_r = FACE_CLOSE_IDLE and
        i_face_close_event.valid = '0' and i_proc_abort = '0' else '0';
    o_face_close_ready <= '1' when
        face_close_state_r = FACE_CLOSE_ACK else '0';

    o_shot_done <= h5_shot_done_s;
    o_shot_done_context <= h5_shot_done_context_s;
    o_frame_output_done <= h5_frame_output_done_s;
    o_outstanding_shots <= outstanding_shots_r;
    o_context_fault_sticky <= context_fault_sticky_r;
    o_proc_idle <= '1' when outstanding_shots_r = 0 and
                            face_close_state_r = FACE_CLOSE_IDLE and
                            h5_idle_s = '1' else '0';

    -- 승인된 Shot 수와 H5에서 Cell 정렬이 끝난 Shot 수를 순차적으로 맞춘다.
    -- Face-close는 이 값이 0인 시점에만 H5로 전달되며, 마지막 Footer의 AXI
    -- handshake가 끝나기 전에는 상위 face tracker에 완료를 알리지 않는다.
    p_face_transaction_owner : process (i_proc_clk)
        variable shot_accepted_v : boolean;
        variable shot_completed_v : boolean;
    begin
        if rising_edge(i_proc_clk) then
            if i_proc_rst_n = '0' then
                face_close_state_r <= FACE_CLOSE_IDLE;
                face_close_hold_r <= C_FACE_CLOSE_EVENT_IDLE;
                outstanding_shots_r <= (others => '0');
                context_fault_sticky_r <= '0';
            else
                shot_accepted_v := i_proc_shot.valid = '1' and
                    o_proc_shot_ready = '1';
                shot_completed_v := h5_shot_done_s = '1';

                if i_proc_clear_status = '1' then
                    context_fault_sticky_r <= '0';
                end if;

                if i_proc_abort = '1' then
                    face_close_state_r <= FACE_CLOSE_IDLE;
                    face_close_hold_r <= C_FACE_CLOSE_EVENT_IDLE;
                    outstanding_shots_r <= (others => '0');
                else
                    if shot_accepted_v and not shot_completed_v then
                        if outstanding_shots_r =
                           (outstanding_shots_r'range => '1') then
                            context_fault_sticky_r <= '1';
                        else
                            outstanding_shots_r <= outstanding_shots_r + 1;
                        end if;
                    elsif shot_completed_v and not shot_accepted_v then
                        if outstanding_shots_r = 0 then
                            context_fault_sticky_r <= '1';
                        else
                            outstanding_shots_r <= outstanding_shots_r - 1;
                        end if;
                    end if;

                    case face_close_state_r is
                        when FACE_CLOSE_IDLE =>
                            if i_face_close_event.valid = '1' and
                               outstanding_shots_r = 0 and
                               not shot_accepted_v then
                                face_close_hold_r <= i_face_close_event;
                                face_close_hold_r.valid <= '1';
                                face_close_state_r <= FACE_CLOSE_SEND;
                            end if;

                        when FACE_CLOSE_SEND =>
                            if face_close_hold_r.valid = '1' and
                               h5_face_close_ready_s = '1' then
                                face_close_hold_r.valid <= '0';
                                if h5_frame_output_done_s = '1' then
                                    face_close_state_r <= FACE_CLOSE_ACK;
                                else
                                    face_close_state_r <=
                                        FACE_CLOSE_WAIT_OUTPUT;
                                end if;
                            end if;

                        when FACE_CLOSE_WAIT_OUTPUT =>
                            if h5_frame_output_done_s = '1' then
                                face_close_state_r <= FACE_CLOSE_ACK;
                            end if;

                        when FACE_CLOSE_ACK =>
                            if i_face_close_event.valid /= '1' then
                                context_fault_sticky_r <= '1';
                            end if;
                            face_close_state_r <= FACE_CLOSE_RELEASE;

                        when FACE_CLOSE_RELEASE =>
                            if i_face_close_event.valid = '0' then
                                face_close_state_r <= FACE_CLOSE_IDLE;
                            end if;
                    end case;
                end if;
            end if;
        end if;
    end process p_face_transaction_owner;

    u_acquisition : entity work.lidar_gpx_acquisition_subsystem
        generic map (
            G_BUILD_CONFIG            => G_BUILD_CONFIG,
            G_SHOT_FIFO_DEPTH         => G_SHOT_FIFO_DEPTH,
            G_STOP_FIFO_DEPTH         => G_STOP_FIFO_DEPTH,
            G_OEN_MODE                => G_OEN_MODE,
            G_POWERUP_TIME_NS         => G_POWERUP_TIME_NS,
            G_RECOVERY_TIME_NS        => G_RECOVERY_TIME_NS,
            G_ALU_PULSE_TIME_NS       => G_ALU_PULSE_TIME_NS,
            G_BUS_IDLE_STABLE_TIME_NS => G_BUS_IDLE_STABLE_TIME_NS,
            G_DRAIN_MARGIN_TIME_NS    => G_DRAIN_MARGIN_TIME_NS
        )
        port map (
            i_proc_clk          => i_proc_clk,
            i_proc_rst_n        => i_proc_rst_n,
            i_proc_shot         => acquisition_shot_s,
            o_proc_shot_ready   => acquisition_shot_ready_s,
            i_proc_stop_tdc     => i_proc_stop_tdc,
            i_proc_clear_status => i_proc_clear_status,
            o_proc_result       => raw_event_s,
            i_proc_result_ready => raw_ready_s,
            o_shot_drop_sticky  => o_shot_drop_sticky,
            o_stop_drop_sticky  => o_stop_drop_sticky,
            i_tdc_clk           => i_tdc_clk,
            i_tdc_rst_n         => i_tdc_rst_n,
            i_tdc_active_valid  => i_tdc_active_valid,
            i_tdc_active_config => i_tdc_active_config,
            i_tdc_register_image => i_tdc_register_image,
            i_tdc_config_apply  => i_tdc_config_apply,
            o_tdc_config_ready  => o_tdc_config_ready,
            o_tdc_config_done   => o_tdc_config_done,
            i_tdc_run_enable    => i_tdc_run_enable,
            i_tdc_soft_reset    => i_tdc_soft_reset,
            i_tdc_force_reinit  => i_tdc_force_reinit,
            i_tdc_clear_status  => i_tdc_clear_status,
            o_tdc_safe          => o_tdc_safe,
            o_tdc_shot_complete => o_tdc_shot_complete,
            i_tdc_register_read => i_tdc_register_read,
            o_tdc_register_read_ready => o_tdc_register_read_ready,
            o_tdc_register_read_response =>
                o_tdc_register_read_response,
            i_tdc_register_read_response_ready =>
                i_tdc_register_read_response_ready,
            o_cdc_reset_busy    => o_cdc_reset_busy,
            o_adr               => o_adr,
            o_csn               => o_csn,
            o_rdn               => o_rdn,
            o_wrn               => o_wrn,
            o_oen               => o_oen,
            i_d                 => i_d,
            o_d                 => o_d,
            o_d_tri             => o_d_tri,
            i_ef1               => i_ef1,
            i_ef2               => i_ef2,
            i_lf1               => i_lf1,
            i_lf2               => i_lf2,
            i_irflag            => i_irflag,
            i_errflag           => i_errflag,
            o_stopdis           => o_stopdis,
            o_alutrigger        => o_alutrigger,
            o_puresn            => o_puresn,
            o_active_mask       => o_active_mask,
            o_terminal_mask     => o_terminal_mask,
            o_status            => o_tdc_status,
            o_faults            => o_tdc_faults
        );

    u_h5_data_path : entity work.lidar_gpx_hls_mixed_data_top
        generic map (
            G_BUILD_CONFIG => G_BUILD_CONFIG
        )
        port map (
            i_clk => i_proc_clk,
            i_rst_n => i_proc_rst_n,
            i_abort => i_proc_abort,
            i_clear_sticky => i_proc_clear_status,
            i_active_valid => i_proc_active_valid,
            i_active_config => i_proc_active_config,
            i_rise_active_profile => i_rise_active_profile,
            i_fall_active_profile => i_fall_active_profile,
            i_raw_event => raw_event_s,
            o_raw_ready => raw_ready_s,
            i_face_close_event => face_close_hold_r,
            o_face_close_ready => h5_face_close_ready_s,
            o_rise_tdata => o_rise_tdata,
            o_rise_tkeep => o_rise_tkeep,
            o_rise_tstrb => o_rise_tstrb,
            o_rise_tuser => o_rise_tuser,
            o_rise_tvalid => o_rise_tvalid,
            o_rise_tlast => o_rise_tlast,
            i_rise_tready => i_rise_tready,
            o_fall_tdata => o_fall_tdata,
            o_fall_tkeep => o_fall_tkeep,
            o_fall_tstrb => o_fall_tstrb,
            o_fall_tuser => o_fall_tuser,
            o_fall_tvalid => o_fall_tvalid,
            o_fall_tlast => o_fall_tlast,
            i_fall_tready => i_fall_tready,
            o_shot_done => h5_shot_done_s,
            o_shot_done_context => h5_shot_done_context_s,
            o_frame_output_done => h5_frame_output_done_s,
            o_idle => h5_idle_s,
            o_decoder_inflight => o_decoder_inflight,
            o_rise_emitted_lines => o_rise_emitted_lines,
            o_fall_emitted_lines => o_fall_emitted_lines,
            o_hit_fault_sticky => o_hit_fault_sticky,
            o_cell_fault_sticky => o_cell_fault_sticky,
            o_frame_fault_sticky => o_frame_fault_sticky,
            o_rise_formatter_fault_sticky =>
                o_rise_formatter_fault_sticky,
            o_fall_formatter_fault_sticky =>
                o_fall_formatter_fault_sticky
        );

    p_contract : process (i_proc_clk)
    begin
        if rising_edge(i_proc_clk) then
            if i_proc_rst_n = '1' and i_proc_abort = '0' then
                if i_proc_shot.valid = '1' and o_proc_shot_ready = '1' then
                    assert i_proc_active_valid = '1'
                        report "V3-H6-DATA-003 Shot without active config"
                        severity failure;
                end if;
                if raw_event_s.valid = '1' then
                    assert outstanding_shots_r /= 0
                        report "V3-H6-DATA-004 GPX result without Shot"
                        severity failure;
                end if;
                if face_close_state_r /= FACE_CLOSE_IDLE and
                   face_close_state_r /= FACE_CLOSE_RELEASE then
                    assert i_face_close_event.valid = '1'
                        report "V3-H6-DATA-005 Face-close valid withdrawn"
                        severity failure;
                end if;
                if face_close_state_r = FACE_CLOSE_SEND then
                    assert outstanding_shots_r = 0
                        report "V3-H6-DATA-006 Face-close overtook a Shot"
                        severity failure;
                end if;
            end if;
        end if;
    end process p_contract;

end architecture rtl;
