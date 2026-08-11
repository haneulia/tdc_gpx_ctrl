library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_build_pkg.all;
use work.lidar_event_types_pkg.all;
use work.lidar_gpx_event_pkg.all;
use work.lidar_gpx_data_pkg.all;
use work.lidar_gpx_vdma_pkg.all;
use work.lidar_v3_hls_contract_pkg.all;

-- V3 H4 PACKED17/Shot Metadata/Face Footer formatter adapter.
-- H3에서 정렬된 한 slope의 Cell을 폭 독립적인 32-bit Canonical Word로
-- 변환한다. 최종 32/64-bit AXI4-Stream Beat 조립과 Line 마지막 정렬은
-- 후단 RTL AXIS Word packer가 단독으로 소유한다.
entity lidar_gpx_lane_word_formatter_hls_adapter is
    generic (
        G_BUILD_CONFIG : lidar_build_config_t := C_DEFAULT_BUILD_CONFIG;
        G_LANE_RISE    : boolean := true
    );
    port (
        i_clk          : in  std_logic;
        i_rst_n        : in  std_logic;
        i_abort        : in  std_logic;
        i_clear_sticky : in  std_logic;

        i_active_profile : in gpx_vdma_lane_profile_t;
        i_active_version : in unsigned(15 downto 0);

        i_cell_event : in  gpx_frame_cell_event_t;
        o_cell_ready : out std_logic;

        i_frame_close_event : in  gpx_frame_close_event_t;
        o_frame_close_ready : out std_logic;

        o_line_word       : out gpx_vdma_line_word_event_t;
        i_line_word_ready : in  std_logic;

        o_footer_emitted   : out std_logic;
        o_emitted_line_count : out unsigned(16 downto 0);
        o_idle             : out std_logic;
        o_fault_pulse      : out lidar_gpx_word_formatter_faults_t;
        o_fault_sticky     : out lidar_gpx_word_formatter_faults_t
    );
end entity lidar_gpx_lane_word_formatter_hls_adapter;

architecture rtl of lidar_gpx_lane_word_formatter_hls_adapter is

    component gpx_lane_word_formatter_hls is
        port (
            ap_clk   : in  std_logic;
            ap_rst_n : in  std_logic;
            ap_start : in  std_logic;
            ap_done  : out std_logic;
            ap_idle  : out std_logic;
            ap_ready : out std_logic;

            ordered_cell_or_face_close_in_TDATA : in std_logic_vector(
                C_V3_H4_FORMATTER_INPUT_AXIS_BITS - 1 downto 0);
            ordered_cell_or_face_close_in_TVALID : in  std_logic;
            ordered_cell_or_face_close_in_TREADY : out std_logic;

            canonical_line_word_out_TDATA : out std_logic_vector(
                C_V3_H4_LINE_WORD_AXIS_BITS - 1 downto 0);
            canonical_line_word_out_TVALID : out std_logic;
            canonical_line_word_out_TREADY : in  std_logic;

            formatter_control_out_TDATA : out std_logic_vector(
                C_V3_H4_CONTROL_AXIS_BITS - 1 downto 0);
            formatter_control_out_TVALID : out std_logic;
            formatter_control_out_TREADY : in  std_logic;

            active_lane_profile : in std_logic_vector(
                C_V3_H4_LANE_PROFILE_BITS - 1 downto 0)
        );
    end component;

    signal packed_input_axis_data_c : std_logic_vector(
        C_V3_H4_FORMATTER_INPUT_AXIS_BITS - 1 downto 0);
    signal packed_input_axis_valid_c : std_logic;
    signal packed_input_axis_ready_c : std_logic;
    signal input_skid_flush_c : std_logic;

    signal input_axis_data_c : std_logic_vector(
        C_V3_H4_FORMATTER_INPUT_AXIS_BITS - 1 downto 0);
    signal input_axis_valid_c : std_logic;
    signal input_axis_ready_c : std_logic;
    signal input_fire_c       : std_logic;

    signal profile_data_c : std_logic_vector(
        C_V3_H4_LANE_PROFILE_BITS - 1 downto 0);

    signal output_axis_data_c : std_logic_vector(
        C_V3_H4_LINE_WORD_AXIS_BITS - 1 downto 0);
    signal output_axis_valid_c : std_logic;
    signal output_axis_ready_c : std_logic;
    signal hls_output_axis_data_c : std_logic_vector(
        C_V3_H4_LINE_WORD_AXIS_BITS - 1 downto 0);
    signal hls_output_axis_valid_c : std_logic;
    signal hls_output_axis_ready_c : std_logic;
    signal output_skid_flush_c     : std_logic;
    signal output_event_c      : gpx_vdma_line_word_event_t :=
        C_GPX_VDMA_LINE_WORD_EVENT_IDLE;

    signal control_axis_data_c : std_logic_vector(
        C_V3_H4_CONTROL_AXIS_BITS - 1 downto 0);
    signal control_axis_valid_c : std_logic;
    signal control_axis_ready_c : std_logic;
    signal control_fire_c       : std_logic;

    signal reset_epoch_r  : unsigned(7 downto 0) := (others => '0');
    signal abort_d_r      : std_logic := '0';
    signal hls_inflight_r : std_logic := '0';
    signal flush_active_r : std_logic := '0';
    signal input_accept_enable_r : std_logic := '0';
    signal hls_done_c     : std_logic;

    signal footer_emitted_r : std_logic := '0';
    signal emitted_line_count_r : unsigned(16 downto 0) :=
        (others => '0');
    signal fault_pulse_r : lidar_gpx_word_formatter_faults_t :=
        C_LIDAR_GPX_WORD_FORMATTER_FAULTS_CLEAR;
    signal fault_sticky_r : lidar_gpx_word_formatter_faults_t :=
        C_LIDAR_GPX_WORD_FORMATTER_FAULTS_CLEAR;

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

    function fn_pack_ordered_cell(value : gpx_frame_cell_event_t)
        return std_logic_vector is
        variable result : std_logic_vector(
            C_V3_H3_ORDERED_LANE_AXIS_BITS - 1 downto 0) :=
            (others => '0');
    begin
        result(C_V3_H3_LANE_CELL_HI downto C_V3_H3_LANE_CELL_LO) :=
            fn_pack_cell(value.cell);
        result(C_V3_H3_LANE_SLOT_INDEX_HI downto
               C_V3_H3_LANE_SLOT_INDEX_LO) :=
            std_logic_vector(value.slot_index);
        result(C_V3_H3_LANE_SLOT_COUNT_HI downto
               C_V3_H3_LANE_SLOT_COUNT_LO) :=
            std_logic_vector(value.slot_count);
        result(C_V3_H3_LANE_LINE_START_BIT) := value.line_start;
        result(C_V3_H3_LANE_LINE_END_BIT) := value.line_end;
        result(C_V3_H3_LANE_FIRST_COLUMN_BIT) := value.first_column;
        result(C_V3_H3_LANE_LAST_COLUMN_BIT) := value.last_column;
        result(C_V3_H3_LANE_GAP_BEFORE_HI downto
               C_V3_H3_LANE_GAP_BEFORE_LO) :=
            std_logic_vector(value.gap_before);
        result(C_V3_H3_LANE_BLANK_BIT) := value.slot_blank;
        result(C_V3_H3_LANE_LINE_FAULTED_BIT) := value.line_faulted;
        return result;
    end function fn_pack_ordered_cell;

    function fn_pack_face_close(value : gpx_frame_close_event_t)
        return std_logic_vector is
        variable result : std_logic_vector(
            C_V3_H3_FACE_CLOSE_RESULT_BITS - 1 downto 0) :=
            (others => '0');
    begin
        result(C_V3_H3_CLOSE_FRAME_ID_HI downto
               C_V3_H3_CLOSE_FRAME_ID_LO) :=
            std_logic_vector(value.face_frame_id);
        result(C_V3_H3_CLOSE_FACE_INDEX_HI downto
               C_V3_H3_CLOSE_FACE_INDEX_LO) :=
            std_logic_vector(value.face_index);
        if value.direction = DIRECTION_CCW then
            result(C_V3_H3_CLOSE_DIRECTION_CCW_BIT) := '1';
        end if;
        result(C_V3_H3_CLOSE_SOURCE_SIM_BIT) := value.source_sim;
        result(C_V3_H3_CLOSE_ACTIVE_VERSION_HI downto
               C_V3_H3_CLOSE_ACTIVE_VERSION_LO) :=
            std_logic_vector(value.active_version);
        result(C_V3_H3_CLOSE_EXPECTED_COLUMNS_HI downto
               C_V3_H3_CLOSE_EXPECTED_COLUMNS_LO) :=
            std_logic_vector(value.columns_per_face);
        result(C_V3_H3_CLOSE_TRAILING_GAP_HI downto
               C_V3_H3_CLOSE_TRAILING_GAP_LO) :=
            std_logic_vector(value.trailing_gap);
        result(C_V3_H3_CLOSE_ALL_HOLE_BIT) := value.all_hole;
        result(C_V3_H3_CLOSE_FAULTED_BIT) := value.face_faulted;
        return result;
    end function fn_pack_face_close;

    function fn_pack_profile(
        value          : gpx_vdma_lane_profile_t;
        active_version : unsigned(15 downto 0))
        return std_logic_vector is
        variable result : std_logic_vector(
            C_V3_H4_LANE_PROFILE_BITS - 1 downto 0) :=
            (others => '0');
    begin
        result(C_V3_H4_PROFILE_VALID_BIT) := value.valid;
        result(C_V3_H4_PROFILE_ENABLED_BIT) := value.enabled;
        if G_LANE_RISE then
            result(C_V3_H4_PROFILE_SLOPE_RISE_BIT) := '1';
        end if;
        result(C_V3_H4_PROFILE_WIDTH_CODE_HI downto
               C_V3_H4_PROFILE_WIDTH_CODE_LO) :=
            fn_gpx_vdma_output_width_code(G_BUILD_CONFIG.output_width);
        result(C_V3_H4_PROFILE_SLOT_COUNT_HI downto
               C_V3_H4_PROFILE_SLOT_COUNT_LO) :=
            std_logic_vector(value.slot_count);
        -- V2의 visible_returns 필드는 실제 수신 개수가 아니라 현재 Face에
        -- 적용된 Runtime 직렬화(전시) Return 슬롯 수를 소유한다.
        result(C_V3_H4_PROFILE_RETURN_SLOTS_HI downto
               C_V3_H4_PROFILE_RETURN_SLOTS_LO) :=
            std_logic_vector(value.visible_returns);
        result(C_V3_H4_PROFILE_CELL_WORDS_HI downto
               C_V3_H4_PROFILE_CELL_WORDS_LO) :=
            std_logic_vector(value.cell_word_count);
        result(C_V3_H4_PROFILE_PLANNED_SHOTS_HI downto
               C_V3_H4_PROFILE_PLANNED_SHOTS_LO) :=
            std_logic_vector(value.planned_shots);
        result(C_V3_H4_PROFILE_RAW_WORDS_HI downto
               C_V3_H4_PROFILE_RAW_WORDS_LO) :=
            std_logic_vector(value.raw_line_words);
        result(C_V3_H4_PROFILE_HSIZE_WORDS_HI downto
               C_V3_H4_PROFILE_HSIZE_WORDS_LO) :=
            std_logic_vector(value.hsize_words);
        result(C_V3_H4_PROFILE_HSIZE_BYTES_HI downto
               C_V3_H4_PROFILE_HSIZE_BYTES_LO) :=
            std_logic_vector(value.hsize_bytes);
        result(C_V3_H4_PROFILE_FOOTER_LINES_HI downto
               C_V3_H4_PROFILE_FOOTER_LINES_LO) :=
            std_logic_vector(value.footer_lines);
        result(C_V3_H4_PROFILE_VSIZE_LINES_HI downto
               C_V3_H4_PROFILE_VSIZE_LINES_LO) :=
            std_logic_vector(value.vsize_lines);
        result(C_V3_H4_PROFILE_ACTIVE_VERSION_HI downto
               C_V3_H4_PROFILE_ACTIVE_VERSION_LO) :=
            std_logic_vector(active_version);
        return result;
    end function fn_pack_profile;

    function fn_unpack_line_word(
        value : std_logic_vector(
            C_V3_H4_LINE_WORD_AXIS_BITS - 1 downto 0);
        slot_count : gpx_frame_slot_t;
        cell_word_count : gpx_vdma_word_count_t)
        return gpx_vdma_line_word_event_t is
        variable result : gpx_vdma_line_word_event_t :=
            C_GPX_VDMA_LINE_WORD_EVENT_IDLE;
        variable kind_value : natural;
    begin
        result.valid := '1';
        result.data := value(C_V3_H4_WORD_DATA_HI downto
                             C_V3_H4_WORD_DATA_LO);
        kind_value := to_integer(unsigned(value(
            C_V3_H4_WORD_KIND_HI downto C_V3_H4_WORD_KIND_LO)));
        case kind_value is
            when 0 => result.kind := GPX_VDMA_LINE_SHOT_METADATA;
            when 1 => result.kind := GPX_VDMA_LINE_CELL_DATA;
            when 2 => result.kind := GPX_VDMA_LINE_FACE_FOOTER;
            when others => result.kind := GPX_VDMA_LINE_SHOT_METADATA;
        end case;
        result.word_index := unsigned(value(
            C_V3_H4_WORD_INDEX_HI downto C_V3_H4_WORD_INDEX_LO));
        result.line_word_count := unsigned(value(
            C_V3_H4_WORD_COUNT_HI downto C_V3_H4_WORD_COUNT_LO));
        result.line_start := value(C_V3_H4_WORD_LINE_START_BIT);
        result.line_end := value(C_V3_H4_WORD_LINE_END_BIT);
        result.frame_end := value(C_V3_H4_WORD_FRAME_END_BIT);
        result.first_column := value(C_V3_H4_WORD_FIRST_COLUMN_BIT);
        result.last_column := value(C_V3_H4_WORD_LAST_COLUMN_BIT);
        result.line_hole := value(C_V3_H4_WORD_LINE_HOLE_BIT);
        result.line_faulted := value(C_V3_H4_WORD_LINE_FAULTED_BIT);
        -- These geometry fields are constant for the active Face and can be
        -- reconstructed locally. Shot Context is already serialized into the
        -- first four Shot Metadata Words, so the record keeps its idle value.
        result.slot_count := slot_count;
        result.cell_word_count := cell_word_count;
        return result;
    end function fn_unpack_line_word;

begin

    assert fn_validate_build_config(G_BUILD_CONFIG) = CFG_OK
        report "V3-HLS-H4-001 invalid build configuration"
        severity failure;
    assert G_BUILD_CONFIG.output_width = 32 or
           G_BUILD_CONFIG.output_width = 64
        report "V3-HLS-H4-002 H4 supports synthesis-time 32/64-bit output"
        severity failure;
    assert C_GPX_SHOT_CONTEXT_WIDTH = 162
        report "V3-HLS-H4-003 Shot context width changed"
        severity failure;
    assert C_V3_H4_LINE_WORD_AXIS_BITS = 64
        report "V3-HLS-H4-007 canonical Word boundary must remain 64 bits"
        severity failure;

    p_pack_input : process (all)
        variable payload : std_logic_vector(
            C_V3_H4_FORMATTER_INPUT_AXIS_BITS - 1 downto 0);
    begin
        payload := (others => '0');
        if i_frame_close_event.valid = '1' then
            payload(C_V3_H4_INPUT_CLOSE_HI downto
                    C_V3_H4_INPUT_CLOSE_LO) :=
                fn_pack_face_close(i_frame_close_event);
            payload(C_V3_H4_INPUT_KIND_BIT) := '1';
        else
            payload(C_V3_H4_INPUT_BODY_HI downto
                    C_V3_H4_INPUT_BODY_LO) :=
                fn_pack_ordered_cell(i_cell_event);
        end if;
        payload(C_V3_H4_INPUT_RESET_EPOCH_HI downto
                C_V3_H4_INPUT_RESET_EPOCH_LO) :=
            std_logic_vector(reset_epoch_r);
        packed_input_axis_data_c <= payload;
    end process p_pack_input;

    profile_data_c <= fn_pack_profile(i_active_profile, i_active_version);

    packed_input_axis_valid_c <= '1' when
        i_rst_n = '1' and i_abort = '0' and input_accept_enable_r = '1' and
        (i_frame_close_event.valid = '1' or i_cell_event.valid = '1')
        else '0';
    o_frame_close_ready <= packed_input_axis_ready_c when
        i_rst_n = '1' and i_abort = '0' and
        input_accept_enable_r = '1' else '0';
    o_cell_ready <= packed_input_axis_ready_c when
        i_rst_n = '1' and i_abort = '0' and
        input_accept_enable_r = '1' and
        i_frame_close_event.valid = '0' else '0';
    input_fire_c <= input_axis_valid_c and input_axis_ready_c;
    input_skid_flush_c <= i_abort or flush_active_r;

    -- H3 정렬 FIFO에서 H4 HLS 입력 Register까지 이어지던 폭이 큰
    -- 조합/Routing 경로를 Registered skid로 분리한다. Ready는 Register되며
    -- 2-slot 구조이므로 Backpressure 중에도 처리율 1 Event/clock을 보존한다.
    u_input_skid : entity work.tdc_gpx_skid_buffer
        generic map (
            g_DATA_WIDTH => C_V3_H4_FORMATTER_INPUT_AXIS_BITS
        )
        port map (
            i_clk     => i_clk,
            i_rst_n   => i_rst_n,
            i_flush   => input_skid_flush_c,
            i_s_valid => packed_input_axis_valid_c,
            o_s_ready => packed_input_axis_ready_c,
            i_s_data  => packed_input_axis_data_c,
            o_m_valid => input_axis_valid_c,
            i_m_ready => input_axis_ready_c,
            o_m_data  => input_axis_data_c
        );

    output_skid_flush_c <= i_abort or flush_active_r;

    -- HLS owns the Word generation while this explicit two-slot boundary
    -- owns backpressure timing and abort flushing in RTL.
    u_output_skid : entity work.tdc_gpx_skid_buffer
        generic map (
            g_DATA_WIDTH => C_V3_H4_LINE_WORD_AXIS_BITS
        )
        port map (
            i_clk     => i_clk,
            i_rst_n   => i_rst_n,
            i_flush   => output_skid_flush_c,
            i_s_valid => hls_output_axis_valid_c,
            o_s_ready => hls_output_axis_ready_c,
            i_s_data  => hls_output_axis_data_c,
            o_m_valid => output_axis_valid_c,
            i_m_ready => output_axis_ready_c,
            o_m_data  => output_axis_data_c
        );

    output_event_c <= fn_unpack_line_word(
        output_axis_data_c,
        i_active_profile.slot_count,
        i_active_profile.cell_word_count);
    o_line_word <= output_event_c when
        output_axis_valid_c = '1' and i_abort = '0' and
        flush_active_r = '0' else C_GPX_VDMA_LINE_WORD_EVENT_IDLE;
    output_axis_ready_c <= '1' when
        i_abort = '1' or flush_active_r = '1' else i_line_word_ready;

    control_axis_ready_c <= i_rst_n;
    control_fire_c <= control_axis_valid_c and control_axis_ready_c;

    o_footer_emitted <= footer_emitted_r;
    o_emitted_line_count <= emitted_line_count_r;
    o_fault_pulse <= fault_pulse_r;
    o_fault_sticky <= fault_sticky_r;
    o_idle <= '1' when
        i_rst_n = '1' and i_abort = '0' and flush_active_r = '0' and
        hls_inflight_r = '0' and input_axis_valid_c = '0' and
        i_cell_event.valid = '0' and i_frame_close_event.valid = '0' and
        hls_output_axis_valid_c = '0' and output_axis_valid_c = '0' and
        control_axis_valid_c = '0' else '0';

    -- abort 복구 상태를 H3 Ready에 직접 조합 연결하지 않고 Register된
    -- 입력 허용 창으로 전달한다. 정상 운용 중에는 계속 1이라 II를 바꾸지
    -- 않으며, stale-output Drain 종료 뒤 한 Cycle 후 안전하게 다시 열린다.
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
        variable pulse_v    : lidar_gpx_word_formatter_faults_t;
        variable sticky_v   : lidar_gpx_word_formatter_faults_t;
        variable inflight_v : std_logic;
        variable flush_v    : std_logic;
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                reset_epoch_r <= (others => '0');
                abort_d_r <= '0';
                hls_inflight_r <= '0';
                flush_active_r <= '0';
                footer_emitted_r <= '0';
                emitted_line_count_r <= (others => '0');
                fault_pulse_r <= C_LIDAR_GPX_WORD_FORMATTER_FAULTS_CLEAR;
                fault_sticky_r <= C_LIDAR_GPX_WORD_FORMATTER_FAULTS_CLEAR;
            else
                pulse_v := C_LIDAR_GPX_WORD_FORMATTER_FAULTS_CLEAR;
                sticky_v := fault_sticky_r;
                inflight_v := hls_inflight_r;
                flush_v := flush_active_r;
                footer_emitted_r <= '0';
                emitted_line_count_r <= (others => '0');

                if i_clear_sticky = '1' then
                    sticky_v := C_LIDAR_GPX_WORD_FORMATTER_FAULTS_CLEAR;
                end if;

                -- 완료와 다음 입력 승인이 같은 cycle이면 새 호출이 우선한다.
                if hls_done_c = '1' then
                    inflight_v := '0';
                    flush_v := '0';
                end if;
                if input_fire_c = '1' then
                    inflight_v := '1';
                end if;

                if i_abort = '1' and abort_d_r = '0' then
                    reset_epoch_r <= reset_epoch_r + 1;
                    flush_v := inflight_v or hls_output_axis_valid_c or
                               output_axis_valid_c or control_axis_valid_c;
                end if;
                abort_d_r <= i_abort;

                -- An abort can coincide with the one-cycle HLS done pulse.
                -- Release the flush from observable quiescence as well, so a
                -- buffered output at that boundary cannot leave flush active
                -- waiting for a done pulse that has already occurred.
                if i_abort = '0' and flush_active_r = '1' and
                   inflight_v = '0' and hls_output_axis_valid_c = '0' and
                   output_axis_valid_c = '0' and
                   control_axis_valid_c = '0' then
                    flush_v := '0';
                end if;

                if i_abort = '0' and flush_active_r = '0' and
                   control_fire_c = '1' then
                    assert control_axis_data_c(
                        C_V3_H4_CONTROL_RESERVED_HI downto
                        C_V3_H4_CONTROL_RESERVED_LO) = "000000"
                        report "V3-HLS-H4-004 nonzero control reserved bits"
                        severity failure;
                    pulse_v := control_axis_data_c(7 downto 0);
                    sticky_v := sticky_v or control_axis_data_c(7 downto 0);
                    footer_emitted_r <= control_axis_data_c(
                        C_V3_H4_CONTROL_FOOTER_BIT);
                    emitted_line_count_r <= unsigned(control_axis_data_c(
                        C_V3_H4_CONTROL_LINE_COUNT_HI downto
                        C_V3_H4_CONTROL_LINE_COUNT_LO));
                end if;

                if output_axis_valid_c = '1' and
                   output_axis_ready_c = '1' and
                   i_abort = '0' and flush_active_r = '0' then
                    assert output_axis_data_c(
                        C_V3_H4_WORD_RESERVED_HI downto
                        C_V3_H4_WORD_RESERVED_LO) = "00000"
                        report "V3-HLS-H4-005 nonzero Word reserved bits"
                        severity failure;
                end if;

                hls_inflight_r <= inflight_v;
                flush_active_r <= flush_v;
                fault_pulse_r <= pulse_v;
                fault_sticky_r <= sticky_v;
            end if;
        end if;
    end process p_control;

    u_hls_lane_word_formatter : gpx_lane_word_formatter_hls
        port map (
            ap_clk   => i_clk,
            ap_rst_n => i_rst_n,
            ap_start => '1',
            ap_done  => hls_done_c,
            ap_idle  => open,
            ap_ready => open,

            ordered_cell_or_face_close_in_TDATA  => input_axis_data_c,
            ordered_cell_or_face_close_in_TVALID => input_axis_valid_c,
            ordered_cell_or_face_close_in_TREADY => input_axis_ready_c,

            canonical_line_word_out_TDATA  => hls_output_axis_data_c,
            canonical_line_word_out_TVALID => hls_output_axis_valid_c,
            canonical_line_word_out_TREADY => hls_output_axis_ready_c,

            formatter_control_out_TDATA  => control_axis_data_c,
            formatter_control_out_TVALID => control_axis_valid_c,
            formatter_control_out_TREADY => control_axis_ready_c,

            active_lane_profile => profile_data_c
        );

end architecture rtl;
