library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_build_pkg.all;
use work.lidar_event_types_pkg.all;
use work.lidar_gpx_event_pkg.all;
use work.lidar_gpx_data_pkg.all;
use work.lidar_v3_hls_contract_pkg.all;

-- V3 HLS Cell-to-Frame 경계 Adapter.
-- HLS가 만든 Rise/Fall Cell을 독립 FIFO에 저장하여 한 Lane의 Backpressure가
-- 다른 Lane을 멈추지 않게 한다. shot_done은 계산 완료가 아니라 필요한 모든
-- Lane의 line_end가 실제 소비된 뒤에만 발생한다.
entity lidar_gpx_frame_lane_assembler_hls_adapter is
    generic (
        G_BUILD_CONFIG : lidar_build_config_t := C_DEFAULT_BUILD_CONFIG
    );
    port (
        i_clk          : in  std_logic;
        i_rst_n        : in  std_logic;
        i_abort        : in  std_logic;
        i_clear_sticky : in  std_logic;

        i_active_version   : in unsigned(15 downto 0);
        i_active_rise_mask : in chip_mask_t;
        i_active_fall_mask : in chip_mask_t;
        i_columns_per_face : in unsigned(15 downto 0);

        i_cell_event : in  gpx_cell_event_t;
        o_cell_ready : out std_logic;

        i_face_close_event : in face_close_event_t :=
            C_FACE_CLOSE_EVENT_IDLE;
        o_face_close_ready : out std_logic;

        o_rise_event : out gpx_frame_cell_event_t;
        i_rise_ready : in  std_logic;
        o_fall_event : out gpx_frame_cell_event_t;
        i_fall_ready : in  std_logic;

        o_frame_close_event : out gpx_frame_close_event_t;
        i_frame_close_ready : in std_logic := '1';

        o_rise_line_done : out std_logic;
        o_fall_line_done : out std_logic;
        o_shot_done      : out std_logic;
        o_shot_done_context : out shot_start_event_t;
        o_idle           : out std_logic;

        o_fault_pulse  : out gpx_frame_assembler_faults_t;
        o_fault_sticky : out gpx_frame_assembler_faults_t
    );
end entity lidar_gpx_frame_lane_assembler_hls_adapter;

architecture rtl of lidar_gpx_frame_lane_assembler_hls_adapter is

    constant C_LANE_FIFO_DEPTH : positive := 32;
    constant C_LANE_FIFO_LOG2  : positive := 5;

    component gpx_frame_assembler_hls is
        port (
            ap_clk   : in  std_logic;
            ap_rst_n : in  std_logic;
            ap_start : in  std_logic;
            ap_done  : out std_logic;
            ap_idle  : out std_logic;
            ap_ready : out std_logic;

            cell_or_face_close_event_in_TDATA  : in  std_logic_vector(
                C_V3_H3_ASSEMBLER_INPUT_AXIS_BITS - 1 downto 0);
            cell_or_face_close_event_in_TVALID : in  std_logic;
            cell_or_face_close_event_in_TREADY : out std_logic;

            ordered_rise_cell_out_TDATA  : out std_logic_vector(
                C_V3_H3_ORDERED_LANE_AXIS_BITS - 1 downto 0);
            ordered_rise_cell_out_TVALID : out std_logic;
            ordered_rise_cell_out_TREADY : in  std_logic;

            ordered_fall_cell_out_TDATA  : out std_logic_vector(
                C_V3_H3_ORDERED_LANE_AXIS_BITS - 1 downto 0);
            ordered_fall_cell_out_TVALID : out std_logic;
            ordered_fall_cell_out_TREADY : in  std_logic;

            assembler_control_out_TDATA  : out std_logic_vector(
                C_V3_H3_CONTROL_AXIS_BITS - 1 downto 0);
            assembler_control_out_TVALID : out std_logic;
            assembler_control_out_TREADY : in  std_logic;

            build_tdc_chip_count : in std_logic_vector(7 downto 0);
            build_stop_channels_per_chip : in std_logic_vector(7 downto 0);
            build_mirror_face_count : in std_logic_vector(7 downto 0);
            active_configuration_version : in std_logic_vector(15 downto 0);
            runtime_enabled_rise_chip_mask : in std_logic_vector(7 downto 0);
            runtime_enabled_fall_chip_mask : in std_logic_vector(7 downto 0);
            runtime_expected_shot_columns_per_face :
                in std_logic_vector(15 downto 0)
        );
    end component;

    signal packed_input_axis_data_c : std_logic_vector(
        C_V3_H3_ASSEMBLER_INPUT_AXIS_BITS - 1 downto 0);
    signal packed_input_axis_valid_c : std_logic;
    signal packed_input_axis_ready_c : std_logic;

    signal input_axis_data_c  : std_logic_vector(
        C_V3_H3_ASSEMBLER_INPUT_AXIS_BITS - 1 downto 0);
    signal input_axis_valid_c : std_logic;
    signal input_axis_ready_c : std_logic;
    signal input_fire_c       : std_logic;
    signal input_close_c      : std_logic;

    signal hls_rise_data_c  : std_logic_vector(
        C_V3_H3_ORDERED_LANE_AXIS_BITS - 1 downto 0);
    signal hls_rise_valid_c : std_logic;
    signal hls_rise_ready_c : std_logic;
    signal hls_fall_data_c  : std_logic_vector(
        C_V3_H3_ORDERED_LANE_AXIS_BITS - 1 downto 0);
    signal hls_fall_valid_c : std_logic;
    signal hls_fall_ready_c : std_logic;

    signal rise_fifo_data_c  : std_logic_vector(
        C_V3_H3_ORDERED_LANE_AXIS_BITS - 1 downto 0);
    signal rise_fifo_valid_c : std_logic;
    signal rise_fifo_ready_c : std_logic;
    signal fall_fifo_data_c  : std_logic_vector(
        C_V3_H3_ORDERED_LANE_AXIS_BITS - 1 downto 0);
    signal fall_fifo_valid_c : std_logic;
    signal fall_fifo_ready_c : std_logic;
    signal lane_flush_c      : std_logic;

    signal control_axis_data_c  : std_logic_vector(
        C_V3_H3_CONTROL_AXIS_BITS - 1 downto 0);
    signal control_axis_valid_c : std_logic;
    signal control_axis_ready_c : std_logic;
    signal control_fire_c       : std_logic;
    signal hls_done_c           : std_logic;

    signal rise_event_c : gpx_frame_cell_event_t :=
        C_GPX_FRAME_CELL_EVENT_IDLE;
    signal fall_event_c : gpx_frame_cell_event_t :=
        C_GPX_FRAME_CELL_EVENT_IDLE;
    signal frame_close_event_r : gpx_frame_close_event_t :=
        C_GPX_FRAME_CLOSE_EVENT_IDLE;

    signal reset_epoch_r  : unsigned(7 downto 0) := (others => '0');
    signal abort_d_r      : std_logic := '0';
    signal hls_inflight_r : std_logic := '0';
    signal flush_active_r : std_logic := '0';
    signal input_accept_enable_r : std_logic := '0';

    signal shot_open_r       : std_logic := '0';
    signal completion_armed_r : std_logic := '0';
    signal rise_required_r   : std_logic := '0';
    signal fall_required_r   : std_logic := '0';
    signal rise_line_seen_r  : std_logic := '0';
    signal fall_line_seen_r  : std_logic := '0';
    signal pending_done_context_r : shot_start_event_t :=
        C_SHOT_START_EVENT_IDLE;

    signal rise_line_fire_c : std_logic;
    signal fall_line_fire_c : std_logic;
    signal rise_line_done_r : std_logic := '0';
    signal fall_line_done_r : std_logic := '0';
    signal shot_done_r      : std_logic := '0';
    signal shot_done_context_r : shot_start_event_t :=
        C_SHOT_START_EVENT_IDLE;

    signal fault_pulse_r : gpx_frame_assembler_faults_t :=
        C_GPX_FRAME_ASSEMBLER_FAULTS_CLEAR;
    signal fault_sticky_r : gpx_frame_assembler_faults_t :=
        C_GPX_FRAME_ASSEMBLER_FAULTS_CLEAR;

    function fn_pack_cell(value : gpx_cell_event_t)
        return std_logic_vector is
        variable result : std_logic_vector(
            C_V3_H2_CELL_EVENT_BITS - 1 downto 0) :=
            (others => '0');
        variable bit_lo : natural;
    begin
        result(C_V3_H2_CELL_KIND_HI downto C_V3_H2_CELL_KIND_LO) :=
            std_logic_vector(to_unsigned(
            gpx_cell_event_kind_t'pos(value.kind), 2));
        result(C_V3_H2_CELL_CHIP_INDEX_HI downto
               C_V3_H2_CELL_CHIP_INDEX_LO) :=
            std_logic_vector(value.chip_index);
        result(C_V3_H2_CELL_IFIFO_BANK_BIT) := value.ififo_id;
        result(C_V3_H2_CELL_STOP_INDEX_HI downto
               C_V3_H2_CELL_STOP_INDEX_LO) :=
            std_logic_vector(value.stop_index);
        result(C_V3_H2_CELL_SLOPE_RISE_BIT) :=
            fn_gpx_slope_to_bit(value.slope);
        result(C_V3_H2_CELL_VISIBLE_RETURNS_HI downto
               C_V3_H2_CELL_VISIBLE_RETURNS_LO) :=
            std_logic_vector(value.hit_count);
        result(C_V3_H2_CELL_SERIALIZED_RETURN_SLOTS_HI downto
               C_V3_H2_CELL_SERIALIZED_RETURN_SLOTS_LO) :=
            std_logic_vector(value.max_hits);
        for hit_index in 0 to C_MAX_RETURNS_PER_STOP - 1 loop
            bit_lo := C_V3_H2_CELL_PACKED_HITS_LO +
                      hit_index * C_GPX_HIT_WIDTH;
            result(bit_lo + C_GPX_HIT_WIDTH - 1 downto bit_lo) :=
                std_logic_vector(value.hits(hit_index));
        end loop;
        result(C_V3_H2_CELL_HIT_DROPPED_BIT) := value.hit_dropped;
        result(C_V3_H2_CELL_RETURN_OVERFLOW_BIT) := value.return_overflow;
        result(C_V3_H2_CELL_ERROR_FILL_BIT) := value.error_fill;
        result(C_V3_H2_CELL_FAULTED_BIT) := value.faulted;
        result(C_V3_H2_CELL_TIMEOUT_CAUSE_HI downto
               C_V3_H2_CELL_TIMEOUT_CAUSE_LO) := value.timeout_cause;
        result(C_V3_H2_CELL_SHOT_CONTEXT_HI downto
               C_V3_H2_CELL_SHOT_CONTEXT_LO) :=
            fn_pack_shot_context(value.shot_context);
        result(C_V3_H2_CELL_CHIP_SHOT_SEQ_HI downto
               C_V3_H2_CELL_CHIP_SHOT_SEQ_LO) :=
            std_logic_vector(value.chip_shot_seq);
        return result;
    end function fn_pack_cell;

    function fn_unpack_cell(value : std_logic_vector(
        C_V3_H2_CELL_EVENT_BITS - 1 downto 0))
        return gpx_cell_event_t is
        variable result : gpx_cell_event_t := C_GPX_CELL_EVENT_IDLE;
        variable bit_lo : natural;
    begin
        result.valid := '1';
        result.kind := gpx_cell_event_kind_t'val(
            to_integer(unsigned(value(
                C_V3_H2_CELL_KIND_HI downto C_V3_H2_CELL_KIND_LO))));
        result.chip_index := unsigned(value(
            C_V3_H2_CELL_CHIP_INDEX_HI downto
            C_V3_H2_CELL_CHIP_INDEX_LO));
        result.ififo_id := value(C_V3_H2_CELL_IFIFO_BANK_BIT);
        result.stop_index := unsigned(value(
            C_V3_H2_CELL_STOP_INDEX_HI downto
            C_V3_H2_CELL_STOP_INDEX_LO));
        result.slope := fn_gpx_slope_from_bit(value(
            C_V3_H2_CELL_SLOPE_RISE_BIT));
        result.hit_count := unsigned(value(
            C_V3_H2_CELL_VISIBLE_RETURNS_HI downto
            C_V3_H2_CELL_VISIBLE_RETURNS_LO));
        result.max_hits := unsigned(value(
            C_V3_H2_CELL_SERIALIZED_RETURN_SLOTS_HI downto
            C_V3_H2_CELL_SERIALIZED_RETURN_SLOTS_LO));
        for hit_index in 0 to C_MAX_RETURNS_PER_STOP - 1 loop
            bit_lo := C_V3_H2_CELL_PACKED_HITS_LO +
                      hit_index * C_GPX_HIT_WIDTH;
            result.hits(hit_index) := unsigned(value(
                bit_lo + C_GPX_HIT_WIDTH - 1 downto bit_lo));
        end loop;
        result.hit_dropped := value(C_V3_H2_CELL_HIT_DROPPED_BIT);
        result.return_overflow := value(C_V3_H2_CELL_RETURN_OVERFLOW_BIT);
        result.error_fill := value(C_V3_H2_CELL_ERROR_FILL_BIT);
        result.faulted := value(C_V3_H2_CELL_FAULTED_BIT);
        result.timeout_cause := value(
            C_V3_H2_CELL_TIMEOUT_CAUSE_HI downto
            C_V3_H2_CELL_TIMEOUT_CAUSE_LO);
        result.shot_context := fn_unpack_shot_context(
            value(C_V3_H2_CELL_SHOT_CONTEXT_HI downto
                  C_V3_H2_CELL_SHOT_CONTEXT_LO));
        result.chip_shot_seq := unsigned(value(
            C_V3_H2_CELL_CHIP_SHOT_SEQ_HI downto
            C_V3_H2_CELL_CHIP_SHOT_SEQ_LO));
        return result;
    end function fn_unpack_cell;

    function fn_unpack_frame(value : std_logic_vector(
        C_V3_H3_ORDERED_LANE_AXIS_BITS - 1 downto 0))
        return gpx_frame_cell_event_t is
        variable result : gpx_frame_cell_event_t :=
            C_GPX_FRAME_CELL_EVENT_IDLE;
    begin
        result.valid := '1';
        result.cell := fn_unpack_cell(value(
            C_V3_H3_LANE_CELL_HI downto C_V3_H3_LANE_CELL_LO));
        result.slot_index := unsigned(value(
            C_V3_H3_LANE_SLOT_INDEX_HI downto
            C_V3_H3_LANE_SLOT_INDEX_LO));
        result.slot_count := unsigned(value(
            C_V3_H3_LANE_SLOT_COUNT_HI downto
            C_V3_H3_LANE_SLOT_COUNT_LO));
        result.line_start := value(C_V3_H3_LANE_LINE_START_BIT);
        result.line_end := value(C_V3_H3_LANE_LINE_END_BIT);
        result.first_column := value(C_V3_H3_LANE_FIRST_COLUMN_BIT);
        result.last_column := value(C_V3_H3_LANE_LAST_COLUMN_BIT);
        result.gap_before := unsigned(value(
            C_V3_H3_LANE_GAP_BEFORE_HI downto
            C_V3_H3_LANE_GAP_BEFORE_LO));
        result.slot_blank := value(C_V3_H3_LANE_BLANK_BIT);
        result.line_faulted := value(C_V3_H3_LANE_LINE_FAULTED_BIT);
        return result;
    end function fn_unpack_frame;

    function fn_unpack_close(value : std_logic_vector(
        C_V3_H3_FACE_CLOSE_RESULT_BITS - 1 downto 0))
        return gpx_frame_close_event_t is
        variable result : gpx_frame_close_event_t :=
            C_GPX_FRAME_CLOSE_EVENT_IDLE;
    begin
        result.valid := '1';
        result.face_frame_id := unsigned(value(
            C_V3_H3_CLOSE_FRAME_ID_HI downto
            C_V3_H3_CLOSE_FRAME_ID_LO));
        result.face_index := unsigned(value(
            C_V3_H3_CLOSE_FACE_INDEX_HI downto
            C_V3_H3_CLOSE_FACE_INDEX_LO));
        if value(C_V3_H3_CLOSE_DIRECTION_CCW_BIT) = '1' then
            result.direction := DIRECTION_CCW;
        end if;
        result.source_sim := value(C_V3_H3_CLOSE_SOURCE_SIM_BIT);
        result.active_version := unsigned(value(
            C_V3_H3_CLOSE_ACTIVE_VERSION_HI downto
            C_V3_H3_CLOSE_ACTIVE_VERSION_LO));
        result.columns_per_face := unsigned(value(
            C_V3_H3_CLOSE_EXPECTED_COLUMNS_HI downto
            C_V3_H3_CLOSE_EXPECTED_COLUMNS_LO));
        result.trailing_gap := unsigned(value(
            C_V3_H3_CLOSE_TRAILING_GAP_HI downto
            C_V3_H3_CLOSE_TRAILING_GAP_LO));
        result.all_hole := value(C_V3_H3_CLOSE_ALL_HOLE_BIT);
        result.face_faulted := value(C_V3_H3_CLOSE_FAULTED_BIT);
        return result;
    end function fn_unpack_close;

begin

    assert fn_validate_build_config(G_BUILD_CONFIG) = CFG_OK
        report "V3-HLS-H3-001 illegal build configuration"
        severity failure;
    assert C_GPX_SHOT_CONTEXT_WIDTH = 162
        report "V3-HLS-H3-002 Shot context width changed"
        severity failure;
    assert C_MAX_CHIPS * C_MAX_STOPS_PER_CHIP <= C_LANE_FIFO_DEPTH
        report "V3-HLS-H3-003 Lane FIFO cannot hold one maximum Shot"
        severity failure;

    p_pack_input : process (all)
        variable payload : std_logic_vector(
            C_V3_H3_ASSEMBLER_INPUT_AXIS_BITS - 1 downto 0);
    begin
        payload := (others => '0');
        if i_face_close_event.valid = '1' then
            payload(C_V3_H3_CLOSE_FRAME_ID_HI downto
                    C_V3_H3_CLOSE_FRAME_ID_LO) :=
                std_logic_vector(i_face_close_event.face_frame_id);
            payload(C_V3_H3_CLOSE_FACE_INDEX_HI downto
                    C_V3_H3_CLOSE_FACE_INDEX_LO) :=
                std_logic_vector(i_face_close_event.face_index);
            if i_face_close_event.direction = DIRECTION_CCW then
                payload(C_V3_H3_CLOSE_DIRECTION_CCW_BIT) := '1';
            end if;
            payload(C_V3_H3_CLOSE_SOURCE_SIM_BIT) :=
                i_face_close_event.source_sim;
            payload(C_V3_H3_CLOSE_ACTIVE_VERSION_HI downto
                    C_V3_H3_CLOSE_ACTIVE_VERSION_LO) :=
                std_logic_vector(i_face_close_event.active_version);
            payload(C_V3_H3_CLOSE_EXPECTED_COLUMNS_HI downto
                    C_V3_H3_CLOSE_EXPECTED_COLUMNS_LO) :=
                std_logic_vector(i_face_close_event.columns_per_face);
            payload(C_V3_H3_INPUT_KIND_BIT) := '1';
        else
            payload(C_V3_H3_LANE_CELL_HI downto C_V3_H3_LANE_CELL_LO) :=
                fn_pack_cell(i_cell_event);
        end if;
        payload(C_V3_H3_INPUT_RESET_EPOCH_HI downto
                C_V3_H3_INPUT_RESET_EPOCH_LO) :=
            std_logic_vector(reset_epoch_r);
        packed_input_axis_data_c <= payload;
    end process p_pack_input;

    input_close_c <= input_axis_data_c(C_V3_H3_INPUT_KIND_BIT);
    packed_input_axis_valid_c <= '1' when
        i_rst_n = '1' and i_abort = '0' and input_accept_enable_r = '1' and
        frame_close_event_r.valid = '0' and completion_armed_r = '0' and
        ((i_face_close_event.valid = '1' and shot_open_r = '0') or
         (i_face_close_event.valid = '0' and i_cell_event.valid = '1'))
        else '0';
    o_face_close_ready <= packed_input_axis_ready_c when
        i_rst_n = '1' and i_abort = '0' and input_accept_enable_r = '1' and
        frame_close_event_r.valid = '0' and completion_armed_r = '0' and
        shot_open_r = '0' else '0';
    o_cell_ready <= packed_input_axis_ready_c when
        i_rst_n = '1' and i_abort = '0' and input_accept_enable_r = '1' and
        frame_close_event_r.valid = '0' and completion_armed_r = '0' and
        i_face_close_event.valid = '0' else '0';
    input_fire_c <= input_axis_valid_c and input_axis_ready_c;

    lane_flush_c <= i_abort or flush_active_r;

    -- H2 Cell/Face-close 계약을 H3 HLS 입력까지 한 Cycle에 직접 연결하지
    -- 않는다. 처리율을 유지하는 Registered skid가 폭이 큰 Record의
    -- Placement 경계를 만들고 abort 시 대기 Event도 함께 폐기한다.
    u_input_skid : entity work.tdc_gpx_skid_buffer
        generic map (
            g_DATA_WIDTH => C_V3_H3_ASSEMBLER_INPUT_AXIS_BITS
        )
        port map (
            i_clk     => i_clk,
            i_rst_n   => i_rst_n,
            i_flush   => lane_flush_c,
            i_s_valid => packed_input_axis_valid_c,
            o_s_ready => packed_input_axis_ready_c,
            i_s_data  => packed_input_axis_data_c,
            o_m_valid => input_axis_valid_c,
            i_m_ready => input_axis_ready_c,
            o_m_data  => input_axis_data_c
        );

    u_rise_fifo : entity work.tdc_gpx_sync_fifo
        generic map (
            g_DATA_WIDTH => C_V3_H3_ORDERED_LANE_AXIS_BITS,
            g_DEPTH      => C_LANE_FIFO_DEPTH,
            g_LOG2_DEPTH => C_LANE_FIFO_LOG2,
            g_IN_REG     => true,
            g_OUT_REG    => true
        )
        port map (
            i_clk     => i_clk,
            i_rst_n   => i_rst_n,
            i_flush   => lane_flush_c,
            i_s_valid => hls_rise_valid_c,
            o_s_ready => hls_rise_ready_c,
            i_s_data  => hls_rise_data_c,
            o_m_valid => rise_fifo_valid_c,
            i_m_ready => rise_fifo_ready_c,
            o_m_data  => rise_fifo_data_c
        );

    u_fall_fifo : entity work.tdc_gpx_sync_fifo
        generic map (
            g_DATA_WIDTH => C_V3_H3_ORDERED_LANE_AXIS_BITS,
            g_DEPTH      => C_LANE_FIFO_DEPTH,
            g_LOG2_DEPTH => C_LANE_FIFO_LOG2,
            g_IN_REG     => true,
            g_OUT_REG    => true
        )
        port map (
            i_clk     => i_clk,
            i_rst_n   => i_rst_n,
            i_flush   => lane_flush_c,
            i_s_valid => hls_fall_valid_c,
            o_s_ready => hls_fall_ready_c,
            i_s_data  => hls_fall_data_c,
            o_m_valid => fall_fifo_valid_c,
            i_m_ready => fall_fifo_ready_c,
            o_m_data  => fall_fifo_data_c
        );

    rise_event_c <= fn_unpack_frame(rise_fifo_data_c);
    fall_event_c <= fn_unpack_frame(fall_fifo_data_c);
    o_rise_event <= rise_event_c when
        rise_fifo_valid_c = '1' and lane_flush_c = '0' else
        C_GPX_FRAME_CELL_EVENT_IDLE;
    o_fall_event <= fall_event_c when
        fall_fifo_valid_c = '1' and lane_flush_c = '0' else
        C_GPX_FRAME_CELL_EVENT_IDLE;
    rise_fifo_ready_c <= i_rise_ready and not lane_flush_c;
    fall_fifo_ready_c <= i_fall_ready and not lane_flush_c;
    rise_line_fire_c <= rise_fifo_valid_c and rise_fifo_ready_c and
        rise_event_c.line_end;
    fall_line_fire_c <= fall_fifo_valid_c and fall_fifo_ready_c and
        fall_event_c.line_end;

    control_axis_ready_c <= '1' when
        i_rst_n = '1' and
        (i_abort = '1' or flush_active_r = '1' or
         control_axis_valid_c = '0' or
         control_axis_data_c(C_V3_H3_CONTROL_HAS_CLOSE_BIT) = '0' or
         frame_close_event_r.valid = '0' or i_frame_close_ready = '1')
        else '0';
    control_fire_c <= control_axis_valid_c and control_axis_ready_c;

    o_frame_close_event <= frame_close_event_r;
    o_rise_line_done <= rise_line_done_r;
    o_fall_line_done <= fall_line_done_r;
    o_shot_done <= shot_done_r;
    o_shot_done_context <= shot_done_context_r;
    o_fault_pulse <= fault_pulse_r;
    o_fault_sticky <= fault_sticky_r;
    o_idle <= '1' when
        i_rst_n = '1' and i_abort = '0' and flush_active_r = '0' and
        hls_inflight_r = '0' and shot_open_r = '0' and
        completion_armed_r = '0' and
        input_axis_valid_c = '0' and i_cell_event.valid = '0' and
        i_face_close_event.valid = '0' and
        rise_fifo_valid_c = '0' and fall_fifo_valid_c = '0' and
        frame_close_event_r.valid = '0' else '0';

    -- abort stale-output Drain 상태가 H2 Ready까지 한 Cycle 조합 경로로
    -- 역전파되지 않도록 입력 허용 창을 Register한다.
    p_input_accept_enable : process (i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' or i_abort = '1' or flush_active_r = '1' then
                input_accept_enable_r <= '0';
            else
                input_accept_enable_r <= '1';
            end if;
        end if;
    end process p_input_accept_enable;

    p_control : process (i_clk)
        variable pulse_v  : gpx_frame_assembler_faults_t;
        variable sticky_v : gpx_frame_assembler_faults_t;
        variable inflight_v : std_logic;
        variable flush_v    : std_logic;
        variable rise_seen_v : std_logic;
        variable fall_seen_v : std_logic;
        variable armed_v     : std_logic;
        variable rise_required_v : std_logic;
        variable fall_required_v : std_logic;
        variable done_context_v : shot_start_event_t;
        variable close_v : gpx_frame_close_event_t;
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                reset_epoch_r <= (others => '0');
                abort_d_r <= '0';
                hls_inflight_r <= '0';
                flush_active_r <= '0';
                shot_open_r <= '0';
                completion_armed_r <= '0';
                rise_required_r <= '0';
                fall_required_r <= '0';
                rise_line_seen_r <= '0';
                fall_line_seen_r <= '0';
                pending_done_context_r <= C_SHOT_START_EVENT_IDLE;
                frame_close_event_r <= C_GPX_FRAME_CLOSE_EVENT_IDLE;
                rise_line_done_r <= '0';
                fall_line_done_r <= '0';
                shot_done_r <= '0';
                shot_done_context_r <= C_SHOT_START_EVENT_IDLE;
                fault_pulse_r <= C_GPX_FRAME_ASSEMBLER_FAULTS_CLEAR;
                fault_sticky_r <= C_GPX_FRAME_ASSEMBLER_FAULTS_CLEAR;
            else
                pulse_v := C_GPX_FRAME_ASSEMBLER_FAULTS_CLEAR;
                sticky_v := fault_sticky_r;
                inflight_v := hls_inflight_r;
                flush_v := flush_active_r;
                rise_seen_v := rise_line_seen_r;
                fall_seen_v := fall_line_seen_r;
                armed_v := completion_armed_r;
                rise_required_v := rise_required_r;
                fall_required_v := fall_required_r;
                done_context_v := pending_done_context_r;

                rise_line_done_r <= '0';
                fall_line_done_r <= '0';
                shot_done_r <= '0';
                shot_done_context_r.valid <= '0';

                if frame_close_event_r.valid = '1' and
                   i_frame_close_ready = '1' then
                    frame_close_event_r.valid <= '0';
                end if;

                if i_clear_sticky = '1' then
                    sticky_v := C_GPX_FRAME_ASSEMBLER_FAULTS_CLEAR;
                end if;

                -- ap_done and the next AXIS input may be accepted in the same
                -- cycle. Clear the completed call first so the new call wins.
                if hls_done_c = '1' then
                    inflight_v := '0';
                end if;
                if input_fire_c = '1' then
                    inflight_v := '1';
                    if input_close_c = '0' and shot_open_r = '0' then
                        shot_open_r <= '1';
                        rise_required_v := '0';
                        fall_required_v := '0';
                        if i_active_rise_mask /= "0000" then
                            rise_required_v := '1';
                        end if;
                        if i_active_fall_mask /= "0000" then
                            fall_required_v := '1';
                        end if;
                        rise_seen_v := '0';
                        fall_seen_v := '0';
                    end if;
                end if;
                if rise_line_fire_c = '1' then
                    rise_seen_v := '1';
                    rise_line_done_r <= '1';
                end if;
                if fall_line_fire_c = '1' then
                    fall_seen_v := '1';
                    fall_line_done_r <= '1';
                end if;

                if i_abort = '1' and abort_d_r = '0' then
                    reset_epoch_r <= reset_epoch_r + 1;
                    flush_v := inflight_v or control_axis_valid_c or
                        hls_rise_valid_c or hls_fall_valid_c;
                    shot_open_r <= '0';
                    armed_v := '0';
                    rise_required_v := '0';
                    fall_required_v := '0';
                    rise_seen_v := '0';
                    fall_seen_v := '0';
                    frame_close_event_r <= C_GPX_FRAME_CLOSE_EVENT_IDLE;
                end if;
                abort_d_r <= i_abort;

                if hls_done_c = '1' then
                    flush_v := '0';
                end if;

                if i_abort = '0' and flush_active_r = '0' and
                   control_fire_c = '1' then
                    assert control_axis_data_c(
                        C_V3_H3_CONTROL_RESERVED_HI downto
                        C_V3_H3_CONTROL_RESERVED_LO) = "00000"
                        report "V3-HLS-H3-004 nonzero control reserved bits"
                        severity failure;

                    pulse_v.context_mismatch := control_axis_data_c(
                        C_V3_H3_FAULT_CONTEXT_BIT);
                    pulse_v.unexpected_cell := control_axis_data_c(
                        C_V3_H3_FAULT_UNEXPECTED_BIT);
                    pulse_v.duplicate_cell := control_axis_data_c(
                        C_V3_H3_FAULT_DUPLICATE_CELL_BIT);
                    pulse_v.duplicate_terminal := control_axis_data_c(
                        C_V3_H3_FAULT_DUPLICATE_TERM_BIT);
                    pulse_v.missing_cell := control_axis_data_c(
                        C_V3_H3_FAULT_MISSING_CELL_BIT);
                    pulse_v.geometry_error := control_axis_data_c(
                        C_V3_H3_FAULT_GEOMETRY_BIT);
                    pulse_v.column_gap := control_axis_data_c(
                        C_V3_H3_FAULT_COLUMN_GAP_BIT);
                    pulse_v.masked_payload_drop := control_axis_data_c(
                        C_V3_H3_FAULT_MASKED_DROP_BIT);

                    sticky_v.context_mismatch :=
                        sticky_v.context_mismatch or control_axis_data_c(
                            C_V3_H3_FAULT_CONTEXT_BIT);
                    sticky_v.unexpected_cell :=
                        sticky_v.unexpected_cell or control_axis_data_c(
                            C_V3_H3_FAULT_UNEXPECTED_BIT);
                    sticky_v.duplicate_cell :=
                        sticky_v.duplicate_cell or control_axis_data_c(
                            C_V3_H3_FAULT_DUPLICATE_CELL_BIT);
                    sticky_v.duplicate_terminal :=
                        sticky_v.duplicate_terminal or control_axis_data_c(
                            C_V3_H3_FAULT_DUPLICATE_TERM_BIT);
                    sticky_v.missing_cell :=
                        sticky_v.missing_cell or control_axis_data_c(
                            C_V3_H3_FAULT_MISSING_CELL_BIT);
                    sticky_v.geometry_error :=
                        sticky_v.geometry_error or control_axis_data_c(
                            C_V3_H3_FAULT_GEOMETRY_BIT);
                    sticky_v.column_gap :=
                        sticky_v.column_gap or control_axis_data_c(
                            C_V3_H3_FAULT_COLUMN_GAP_BIT);
                    sticky_v.masked_payload_drop :=
                        sticky_v.masked_payload_drop or control_axis_data_c(
                            C_V3_H3_FAULT_MASKED_DROP_BIT);

                    if control_axis_data_c(
                        C_V3_H3_CONTROL_HAS_CLOSE_BIT) = '1' then
                        close_v := fn_unpack_close(
                            control_axis_data_c(
                                C_V3_H3_CONTROL_CLOSE_HI downto
                                C_V3_H3_CONTROL_CLOSE_LO));
                        frame_close_event_r <= close_v;
                    end if;
                    if control_axis_data_c(
                        C_V3_H3_CONTROL_CELLS_DONE_BIT) = '1' then
                        done_context_v := fn_unpack_shot_context(
                            control_axis_data_c(
                                C_V3_H3_CONTROL_SHOT_CONTEXT_HI downto
                                C_V3_H3_CONTROL_SHOT_CONTEXT_LO));
                        armed_v := '1';
                    end if;
                end if;

                if armed_v = '1' and
                   (rise_required_v = '0' or rise_seen_v = '1') and
                   (fall_required_v = '0' or fall_seen_v = '1') then
                    shot_done_r <= '1';
                    shot_done_context_r <= done_context_v;
                    shot_done_context_r.valid <= '1';
                    shot_open_r <= '0';
                    armed_v := '0';
                    rise_required_v := '0';
                    fall_required_v := '0';
                    rise_seen_v := '0';
                    fall_seen_v := '0';
                end if;

                hls_inflight_r <= inflight_v;
                flush_active_r <= flush_v;
                completion_armed_r <= armed_v;
                rise_required_r <= rise_required_v;
                fall_required_r <= fall_required_v;
                rise_line_seen_r <= rise_seen_v;
                fall_line_seen_r <= fall_seen_v;
                pending_done_context_r <= done_context_v;
                fault_pulse_r <= pulse_v;
                fault_sticky_r <= sticky_v;
            end if;
        end if;
    end process p_control;

    u_hls_frame_assembler : gpx_frame_assembler_hls
        port map (
            ap_clk   => i_clk,
            ap_rst_n => i_rst_n,
            ap_start => '1',
            ap_done  => hls_done_c,
            ap_idle  => open,
            ap_ready => open,

            cell_or_face_close_event_in_TDATA  => input_axis_data_c,
            cell_or_face_close_event_in_TVALID => input_axis_valid_c,
            cell_or_face_close_event_in_TREADY => input_axis_ready_c,

            ordered_rise_cell_out_TDATA  => hls_rise_data_c,
            ordered_rise_cell_out_TVALID => hls_rise_valid_c,
            ordered_rise_cell_out_TREADY => hls_rise_ready_c,

            ordered_fall_cell_out_TDATA  => hls_fall_data_c,
            ordered_fall_cell_out_TVALID => hls_fall_valid_c,
            ordered_fall_cell_out_TREADY => hls_fall_ready_c,

            assembler_control_out_TDATA  => control_axis_data_c,
            assembler_control_out_TVALID => control_axis_valid_c,
            assembler_control_out_TREADY => control_axis_ready_c,

            build_tdc_chip_count => std_logic_vector(to_unsigned(
                G_BUILD_CONFIG.num_chips, 8)),
            build_stop_channels_per_chip => std_logic_vector(to_unsigned(
                G_BUILD_CONFIG.stops_per_chip, 8)),
            build_mirror_face_count => std_logic_vector(to_unsigned(
                G_BUILD_CONFIG.num_faces, 8)),
            active_configuration_version =>
                std_logic_vector(i_active_version),
            runtime_enabled_rise_chip_mask =>
                "0000" & i_active_rise_mask,
            runtime_enabled_fall_chip_mask =>
                "0000" & i_active_fall_mask,
            runtime_expected_shot_columns_per_face =>
                std_logic_vector(i_columns_per_face)
        );

end architecture rtl;
