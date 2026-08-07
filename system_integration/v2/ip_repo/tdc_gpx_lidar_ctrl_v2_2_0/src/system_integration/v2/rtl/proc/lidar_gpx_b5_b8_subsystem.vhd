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
use work.tdc_gpx_pkg.all;

-- Production B5-B8 integration boundary.
--
-- Accepted Processing Shots cross to the physical GPX bus in B5. Returned
-- 28-bit I-Mode words cross back, are decoded into Hits and Cells, and are
-- emitted as width-independent Rise/Fall Frame lanes. Face-close is released
-- only after every previously accepted Shot has completed B8 assembly.
entity lidar_gpx_b5_b8_subsystem is
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
        i_proc_clk          : in  std_logic;
        i_proc_rst_n        : in  std_logic;
        i_proc_abort        : in  std_logic := '0';
        i_proc_clear_status : in  std_logic := '0';
        i_proc_active_valid : in  std_logic;
        i_proc_active_config: in  lidar_active_config_t;
        i_proc_shot         : in  shot_start_event_t;
        o_proc_shot_ready   : out std_logic;
        i_proc_stop_tdc     : in  std_logic;
        i_face_close_event  : in  face_close_event_t;
        o_face_close_ready  : out std_logic;

        o_rise_event : out gpx_frame_cell_event_t;
        i_rise_ready : in  std_logic;
        o_fall_event : out gpx_frame_cell_event_t;
        i_fall_ready : in  std_logic;
        o_frame_close_event : out gpx_frame_close_event_t;
        i_frame_close_ready : in  std_logic;
        -- Completion of the final enabled-lane Footer Beat. Delivery and
        -- output completion are separate so AXIS backpressure holds the Face.
        i_frame_output_done : in std_logic := '1';
        o_rise_line_done : out std_logic;
        o_fall_line_done : out std_logic;
        o_shot_done      : out std_logic;
        o_shot_done_context : out shot_start_event_t;
        o_proc_idle      : out std_logic;
        o_outstanding_shots : out unsigned(15 downto 0);
        o_context_fault_sticky : out std_logic;

        i_tdc_clk           : in  std_logic;
        i_tdc_rst_n         : in  std_logic;
        i_tdc_active_valid  : in  std_logic;
        i_tdc_active_config : in  lidar_active_config_t;
        i_tdc_register_image: in  gpx_register_image_t;
        i_tdc_config_apply  : in  std_logic;
        o_tdc_config_ready  : out std_logic;
        o_tdc_config_done   : out std_logic;
        i_tdc_run_enable    : in  std_logic;
        i_tdc_soft_reset    : in  std_logic := '0';
        i_tdc_force_reinit  : in  std_logic := '0';
        i_tdc_clear_status  : in  std_logic := '0';
        o_tdc_safe          : out std_logic;
        o_tdc_shot_complete : out std_logic;
        i_tdc_register_read : in gpx_register_read_request_t :=
            C_GPX_REGISTER_READ_REQUEST_IDLE;
        o_tdc_register_read_ready : out std_logic;
        o_tdc_register_read_response : out gpx_register_read_response_t;
        i_tdc_register_read_response_ready : in std_logic := '1';
        o_cdc_reset_busy    : out std_logic;

        o_adr        : out gpx_bus_address_array_t;
        o_csn        : out chip_mask_t;
        o_rdn        : out chip_mask_t;
        o_wrn        : out chip_mask_t;
        o_oen        : out chip_mask_t;
        i_d          : in  gpx_bus_data_array_t;
        o_d          : out gpx_bus_data_array_t;
        o_d_tri      : out chip_mask_t;
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
        o_hit_fault_pulse  : out gpx_hit_decoder_faults_t;
        o_hit_fault_sticky : out gpx_hit_decoder_faults_t;
        o_cell_fault_pulse  : out gpx_cell_collector_faults_t;
        o_cell_fault_sticky : out gpx_cell_collector_faults_t;
        o_frame_fault_pulse  : out gpx_frame_assembler_faults_t;
        o_frame_fault_sticky : out gpx_frame_assembler_faults_t
    );
end entity lidar_gpx_b5_b8_subsystem;

architecture rtl of lidar_gpx_b5_b8_subsystem is

    type close_state_t is (
        CLOSE_IDLE,
        CLOSE_SEND,
        CLOSE_WAIT_OUTPUT,
        CLOSE_WAIT_FRAME_DONE,
        CLOSE_ACK,
        CLOSE_RELEASE
    );

    signal close_state_r : close_state_t := CLOSE_IDLE;
    signal close_hold_r  : face_close_event_t := C_FACE_CLOSE_EVENT_IDLE;
    signal outstanding_r : unsigned(15 downto 0) := (others => '0');
    signal context_fault_r : std_logic := '0';

    signal acquisition_shot_c : shot_start_event_t;
    signal acquisition_shot_ready_c : std_logic;
    signal raw_event_c : gpx_raw_event_t;
    signal raw_ready_c : std_logic;
    signal pipeline_close_ready_c : std_logic;
    signal frame_close_event_c : gpx_frame_close_event_t;
    signal pipeline_idle_c : std_logic;
    signal shot_done_c : std_logic;
    signal shot_done_context_c : shot_start_event_t;

begin

    assert fn_validate_build_config(G_BUILD_CONFIG) = CFG_OK
        report "V2-B5B8-001 invalid build configuration"
        severity failure;

    p_shot_gate : process (all)
        variable value : shot_start_event_t;
    begin
        value := i_proc_shot;
        if close_state_r /= CLOSE_IDLE or
           i_face_close_event.valid = '1' or i_proc_abort = '1' then
            value.valid := '0';
        end if;
        acquisition_shot_c <= value;
    end process p_shot_gate;

    o_proc_shot_ready <= acquisition_shot_ready_c when
        close_state_r = CLOSE_IDLE and
        i_face_close_event.valid = '0' and i_proc_abort = '0' else '0';

    o_face_close_ready <= '1' when close_state_r = CLOSE_ACK else '0';
    o_outstanding_shots <= outstanding_r;
    o_context_fault_sticky <= context_fault_r;
    o_proc_idle <= '1' when outstanding_r = 0 and
                            close_state_r = CLOSE_IDLE and
                            pipeline_idle_c = '1' else '0';
    o_shot_done <= shot_done_c;
    o_shot_done_context <= shot_done_context_c;
    o_frame_close_event <= frame_close_event_c;

    p_order_owner : process (i_proc_clk)
        variable accepted_v : boolean;
        variable completed_v : boolean;
    begin
        if rising_edge(i_proc_clk) then
            if i_proc_rst_n = '0' then
                close_state_r <= CLOSE_IDLE;
                close_hold_r <= C_FACE_CLOSE_EVENT_IDLE;
                outstanding_r <= (others => '0');
                context_fault_r <= '0';
            else
                accepted_v := i_proc_shot.valid = '1' and
                    o_proc_shot_ready = '1';
                completed_v := shot_done_c = '1';

                if i_proc_clear_status = '1' then
                    context_fault_r <= '0';
                end if;

                if i_proc_abort = '1' then
                    close_state_r <= CLOSE_IDLE;
                    close_hold_r <= C_FACE_CLOSE_EVENT_IDLE;
                    outstanding_r <= (others => '0');
                else
                    if accepted_v and not completed_v then
                        if outstanding_r =
                           (outstanding_r'range => '1') then
                            context_fault_r <= '1';
                        else
                            outstanding_r <= outstanding_r + 1;
                        end if;
                    elsif completed_v and not accepted_v then
                        if outstanding_r = 0 then
                            context_fault_r <= '1';
                        else
                            outstanding_r <= outstanding_r - 1;
                        end if;
                    end if;

                    case close_state_r is
                        when CLOSE_IDLE =>
                            if i_face_close_event.valid = '1' and
                               outstanding_r = 0 and not accepted_v then
                                close_hold_r <= i_face_close_event;
                                close_hold_r.valid <= '1';
                                close_state_r <= CLOSE_SEND;
                            end if;

                        when CLOSE_SEND =>
                            if close_hold_r.valid = '1' and
                               pipeline_close_ready_c = '1' then
                                close_hold_r.valid <= '0';
                                close_state_r <= CLOSE_WAIT_OUTPUT;
                            end if;

                        when CLOSE_WAIT_OUTPUT =>
                            if frame_close_event_c.valid = '1' and
                               i_frame_close_ready = '1' then
                                close_state_r <= CLOSE_WAIT_FRAME_DONE;
                            end if;

                        when CLOSE_WAIT_FRAME_DONE =>
                            if i_frame_output_done = '1' then
                                close_state_r <= CLOSE_ACK;
                            end if;

                        when CLOSE_ACK =>
                            if i_face_close_event.valid /= '1' then
                                context_fault_r <= '1';
                            end if;
                            close_state_r <= CLOSE_RELEASE;

                        -- The upstream owner clears valid on the CLOSE_ACK
                        -- handshake edge.  Wait for that registered release
                        -- before admitting another close so the same held
                        -- Face boundary cannot be sampled twice.
                        when CLOSE_RELEASE =>
                            if i_face_close_event.valid = '0' then
                                close_state_r <= CLOSE_IDLE;
                            end if;
                    end case;
                end if;
            end if;
        end if;
    end process p_order_owner;

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
            i_proc_shot         => acquisition_shot_c,
            o_proc_shot_ready   => acquisition_shot_ready_c,
            i_proc_stop_tdc     => i_proc_stop_tdc,
            i_proc_clear_status => i_proc_clear_status,
            o_proc_result       => raw_event_c,
            i_proc_result_ready => raw_ready_c,
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

    u_hit_cell_frame : entity work.lidar_gpx_hit_cell_frame_pipeline
        generic map (
            G_BUILD_CONFIG => G_BUILD_CONFIG
        )
        port map (
            i_clk                => i_proc_clk,
            i_rst_n              => i_proc_rst_n,
            i_abort              => i_proc_abort,
            i_clear_sticky       => i_proc_clear_status,
            i_active_valid       => i_proc_active_valid,
            i_active_config      => i_proc_active_config,
            i_raw_event          => raw_event_c,
            o_raw_ready          => raw_ready_c,
            i_face_close_event   => close_hold_r,
            o_face_close_ready   => pipeline_close_ready_c,
            o_rise_event         => o_rise_event,
            i_rise_ready         => i_rise_ready,
            o_fall_event         => o_fall_event,
            i_fall_ready         => i_fall_ready,
            o_frame_close_event  => frame_close_event_c,
            i_frame_close_ready  => i_frame_close_ready,
            o_rise_line_done     => o_rise_line_done,
            o_fall_line_done     => o_fall_line_done,
            o_shot_done          => shot_done_c,
            o_shot_done_context  => shot_done_context_c,
            o_idle               => pipeline_idle_c,
            o_hit_fault_pulse    => o_hit_fault_pulse,
            o_hit_fault_sticky   => o_hit_fault_sticky,
            o_cell_fault_pulse   => o_cell_fault_pulse,
            o_cell_fault_sticky  => o_cell_fault_sticky,
            o_frame_fault_pulse  => o_frame_fault_pulse,
            o_frame_fault_sticky => o_frame_fault_sticky
        );

    p_contract : process (i_proc_clk)
    begin
        if rising_edge(i_proc_clk) then
            if i_proc_rst_n = '1' and i_proc_abort = '0' then
                if i_proc_shot.valid = '1' and o_proc_shot_ready = '1' then
                    assert i_proc_active_valid = '1'
                        report "V2-B5B8-002 Shot without Processing config"
                        severity failure;
                end if;
                if raw_event_c.valid = '1' then
                    assert outstanding_r /= 0
                        report "V2-B5B8-003 GPX result without accepted Shot"
                        severity failure;
                end if;
                if close_state_r /= CLOSE_IDLE and
                   close_state_r /= CLOSE_RELEASE then
                    assert i_face_close_event.valid = '1'
                        report "V2-B5B8-004 Face-close source withdrew valid"
                        severity failure;
                end if;
                if close_state_r = CLOSE_SEND then
                    assert outstanding_r = 0
                        report "V2-B5B8-005 Face-close overtook a Shot"
                        severity failure;
                end if;
            end if;
        end if;
    end process p_contract;

end architecture rtl;
