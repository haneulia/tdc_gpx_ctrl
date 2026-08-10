library ieee;
use ieee.std_logic_1164.all;

-- H0-H3 HLS 경계의 VHDL 측 단일 비트 ABI 계약이다.
-- C++의 lidar_v3_h1/h2/h3_*_contract.hpp와 같은 값이어야 하며,
-- V2/HLS 차분 테스트가 두 언어 사이의 실제 Word 동일성을 검증한다.
package lidar_v3_hls_contract_pkg is

    constant C_V3_HLS_CONTRACT_ABI_MAJOR : natural := 3;
    constant C_V3_HLS_CONTRACT_ABI_MINOR : natural := 1;

    -- H1: Raw Event -> decoded Hit Event
    constant C_V3_H1_SHOT_CONTEXT_BITS       : positive := 162;
    constant C_V3_H1_RAW_EVENT_AXIS_BITS     : positive := 216;
    constant C_V3_H1_DECODED_HIT_EVENT_BITS  : positive := 218;
    constant C_V3_H1_DECODER_RESULT_AXIS_BITS : positive := 224;

    -- Shot 전체에서 변하지 않는 측정 문맥. 측정 시작 기준시점 (T0)은
    -- 동기화된 fire_done을 승인하고 start_tdc를 발생시킨 사건이다.
    constant C_V3_H1_CONTEXT_REQUEST_VALID_BIT       : natural := 0;
    constant C_V3_H1_CONTEXT_FACE_INDEX_LO           : natural := 1;
    constant C_V3_H1_CONTEXT_FACE_INDEX_HI           : natural := 3;
    constant C_V3_H1_CONTEXT_ENCODER_STATE_LO        : natural := 4;
    constant C_V3_H1_CONTEXT_ENCODER_STATE_HI        : natural := 18;
    constant C_V3_H1_CONTEXT_DIRECTION_CCW_BIT       : natural := 19;
    constant C_V3_H1_CONTEXT_SHOT_COLUMN_LO          : natural := 20;
    constant C_V3_H1_CONTEXT_SHOT_COLUMN_HI          : natural := 35;
    constant C_V3_H1_CONTEXT_LAST_COLUMN_BIT         : natural := 36;
    constant C_V3_H1_CONTEXT_SOURCE_SIM_BIT          : natural := 37;
    constant C_V3_H1_CONTEXT_ENCODER_LATENCY_LO      : natural := 38;
    constant C_V3_H1_CONTEXT_ENCODER_LATENCY_HI      : natural := 45;
    constant C_V3_H1_CONTEXT_ENCODER_LAT_VALID_BIT   : natural := 46;
    constant C_V3_H1_CONTEXT_ACTIVE_VERSION_LO       : natural := 47;
    constant C_V3_H1_CONTEXT_ACTIVE_VERSION_HI       : natural := 62;
    constant C_V3_H1_CONTEXT_FIRE_TO_T0_LATENCY_LO   : natural := 63;
    constant C_V3_H1_CONTEXT_FIRE_TO_T0_LATENCY_HI   : natural := 94;
    constant C_V3_H1_CONTEXT_T0_TIMESTAMP_LO         : natural := 95;
    constant C_V3_H1_CONTEXT_T0_TIMESTAMP_HI         : natural := 158;
    constant C_V3_H1_CONTEXT_TIMESTAMP_VALID_BIT     : natural := 159;
    constant C_V3_H1_CONTEXT_TIME_SYNC_VALID_BIT     : natural := 160;
    constant C_V3_H1_CONTEXT_VALID_BIT               : natural := 161;

    constant C_V3_H1_RAW_EVENT_KIND_LO        : natural := 0;
    constant C_V3_H1_RAW_EVENT_KIND_HI        : natural := 1;
    constant C_V3_H1_RAW_CHIP_INDEX_LO        : natural := 2;
    constant C_V3_H1_RAW_CHIP_INDEX_HI        : natural := 3;
    constant C_V3_H1_RAW_IFIFO_BANK_BIT       : natural := 4;
    constant C_V3_H1_RAW_IMODE_WORD_LO        : natural := 5;
    constant C_V3_H1_RAW_IMODE_WORD_HI        : natural := 32;
    constant C_V3_H1_RAW_FAULTED_BIT          : natural := 33;
    constant C_V3_H1_RAW_TIMEOUT_CAUSE_LO     : natural := 34;
    constant C_V3_H1_RAW_TIMEOUT_CAUSE_HI     : natural := 36;
    constant C_V3_H1_RAW_SHOT_CONTEXT_LO      : natural := 37;
    constant C_V3_H1_RAW_SHOT_CONTEXT_HI      : natural := 198;
    constant C_V3_H1_RAW_CHIP_SHOT_SEQ_LO     : natural := 199;
    constant C_V3_H1_RAW_CHIP_SHOT_SEQ_HI     : natural := 214;
    constant C_V3_H1_RAW_RESERVED_BIT         : natural := 215;

    -- Raw Event 안의 외부 TDC-GPX I-Mode 28-bit Word 내부 구조.
    constant C_V3_H1_IMODE_DISTANCE_17BIT_LO  : natural := 0;
    constant C_V3_H1_IMODE_DISTANCE_17BIT_HI  : natural := 16;
    constant C_V3_H1_IMODE_SLOPE_RISE_BIT     : natural := 17;
    constant C_V3_H1_IMODE_START_NUMBER_LO    : natural := 18;
    constant C_V3_H1_IMODE_START_NUMBER_HI    : natural := 25;
    constant C_V3_H1_IMODE_CHANNEL_INDEX_LO   : natural := 26;
    constant C_V3_H1_IMODE_CHANNEL_INDEX_HI   : natural := 27;

    constant C_V3_H1_HIT_EVENT_KIND_LO        : natural := 0;
    constant C_V3_H1_HIT_EVENT_KIND_HI        : natural := 1;
    constant C_V3_H1_HIT_CHIP_INDEX_LO        : natural := 2;
    constant C_V3_H1_HIT_CHIP_INDEX_HI        : natural := 3;
    constant C_V3_H1_HIT_IFIFO_BANK_BIT       : natural := 4;
    constant C_V3_H1_HIT_CHANNEL_LO           : natural := 5;
    constant C_V3_H1_HIT_CHANNEL_HI           : natural := 6;
    constant C_V3_H1_HIT_STOP_INDEX_LO        : natural := 7;
    constant C_V3_H1_HIT_STOP_INDEX_HI        : natural := 9;
    constant C_V3_H1_HIT_START_NUMBER_LO      : natural := 10;
    constant C_V3_H1_HIT_START_NUMBER_HI      : natural := 17;
    constant C_V3_H1_HIT_SLOPE_RISE_BIT       : natural := 18;
    constant C_V3_H1_HIT_DISTANCE_17BIT_LO    : natural := 19;
    constant C_V3_H1_HIT_DISTANCE_17BIT_HI    : natural := 35;
    constant C_V3_H1_HIT_FAULTED_BIT          : natural := 36;
    constant C_V3_H1_HIT_TIMEOUT_CAUSE_LO     : natural := 37;
    constant C_V3_H1_HIT_TIMEOUT_CAUSE_HI     : natural := 39;
    constant C_V3_H1_HIT_SHOT_CONTEXT_LO      : natural := 40;
    constant C_V3_H1_HIT_SHOT_CONTEXT_HI      : natural := 201;
    constant C_V3_H1_HIT_CHIP_SHOT_SEQ_LO     : natural := 202;
    constant C_V3_H1_HIT_CHIP_SHOT_SEQ_HI     : natural := 217;

    constant C_V3_H1_RESULT_CONTAINS_HIT_BIT  : natural := 218;
    constant C_V3_H1_RESULT_CHIP_FAULT_BIT    : natural := 219;
    constant C_V3_H1_RESULT_STOP_FAULT_BIT    : natural := 220;
    constant C_V3_H1_RESULT_SLOPE_FAULT_BIT   : natural := 221;
    constant C_V3_H1_RESULT_RESERVED_LO       : natural := 222;
    constant C_V3_H1_RESULT_RESERVED_HI       : natural := 223;

    -- H2: decoded Hit Event -> Cell Event
    constant C_V3_H2_COLLECTOR_INPUT_AXIS_BITS : positive := 232;
    constant C_V3_H2_CELL_EVENT_BITS            : positive := 319;
    constant C_V3_H2_COLLECTOR_RESULT_AXIS_BITS : positive := 328;
    constant C_V3_H2_INPUT_RESERVED_LO          : natural := 218;
    constant C_V3_H2_INPUT_RESERVED_HI          : natural := 223;
    constant C_V3_H2_INPUT_RESET_EPOCH_LO       : natural := 224;
    constant C_V3_H2_INPUT_RESET_EPOCH_HI       : natural := 231;

    constant C_V3_H2_CELL_KIND_LO               : natural := 0;
    constant C_V3_H2_CELL_KIND_HI               : natural := 1;
    constant C_V3_H2_CELL_CHIP_INDEX_LO         : natural := 2;
    constant C_V3_H2_CELL_CHIP_INDEX_HI         : natural := 3;
    constant C_V3_H2_CELL_IFIFO_BANK_BIT        : natural := 4;
    constant C_V3_H2_CELL_STOP_INDEX_LO         : natural := 5;
    constant C_V3_H2_CELL_STOP_INDEX_HI         : natural := 7;
    constant C_V3_H2_CELL_SLOPE_RISE_BIT        : natural := 8;
    constant C_V3_H2_CELL_VISIBLE_RETURNS_LO    : natural := 9;
    constant C_V3_H2_CELL_VISIBLE_RETURNS_HI    : natural := 11;
    constant C_V3_H2_CELL_RETURN_CAPACITY_LO    : natural := 12;
    constant C_V3_H2_CELL_RETURN_CAPACITY_HI    : natural := 14;
    constant C_V3_H2_CELL_PACKED_HITS_LO        : natural := 15;
    constant C_V3_H2_CELL_PACKED_HITS_HI        : natural := 133;
    constant C_V3_H2_CELL_HIT_DROPPED_BIT       : natural := 134;
    constant C_V3_H2_CELL_RETURN_OVERFLOW_BIT   : natural := 135;
    constant C_V3_H2_CELL_ERROR_FILL_BIT        : natural := 136;
    constant C_V3_H2_CELL_FAULTED_BIT           : natural := 137;
    constant C_V3_H2_CELL_TIMEOUT_CAUSE_LO      : natural := 138;
    constant C_V3_H2_CELL_TIMEOUT_CAUSE_HI      : natural := 140;
    constant C_V3_H2_CELL_SHOT_CONTEXT_LO       : natural := 141;
    constant C_V3_H2_CELL_SHOT_CONTEXT_HI       : natural := 302;
    constant C_V3_H2_CELL_CHIP_SHOT_SEQ_LO      : natural := 303;
    constant C_V3_H2_CELL_CHIP_SHOT_SEQ_HI      : natural := 318;

    constant C_V3_H2_RESULT_CONTAINS_CELL_BIT   : natural := 319;
    constant C_V3_H2_RESULT_CONTEXT_FAULT_BIT   : natural := 320;
    constant C_V3_H2_RESULT_OVERFLOW_FAULT_BIT  : natural := 321;
    constant C_V3_H2_RESULT_START_FAULT_BIT     : natural := 322;
    constant C_V3_H2_RESULT_CAPACITY_FAULT_BIT  : natural := 323;
    constant C_V3_H2_RESULT_RESERVED_LO         : natural := 324;
    constant C_V3_H2_RESULT_RESERVED_HI         : natural := 327;

    -- H3: Cell/Face-close Event -> ordered Lane Cell/Control
    constant C_V3_H3_ASSEMBLER_INPUT_AXIS_BITS  : positive := 328;
    constant C_V3_H3_ORDERED_LANE_AXIS_BITS     : positive := 360;
    constant C_V3_H3_CONTROL_AXIS_BITS          : positive := 264;
    constant C_V3_H3_FACE_CLOSE_EVENT_BITS      : positive := 69;
    constant C_V3_H3_FACE_CLOSE_RESULT_BITS     : positive := 87;

    constant C_V3_H3_INPUT_KIND_BIT             : natural := 319;
    constant C_V3_H3_INPUT_RESET_EPOCH_LO       : natural := 320;
    constant C_V3_H3_INPUT_RESET_EPOCH_HI       : natural := 327;

    constant C_V3_H3_CLOSE_FRAME_ID_LO          : natural := 0;
    constant C_V3_H3_CLOSE_FRAME_ID_HI          : natural := 31;
    constant C_V3_H3_CLOSE_FACE_INDEX_LO        : natural := 32;
    constant C_V3_H3_CLOSE_FACE_INDEX_HI        : natural := 34;
    constant C_V3_H3_CLOSE_DIRECTION_CCW_BIT    : natural := 35;
    constant C_V3_H3_CLOSE_SOURCE_SIM_BIT       : natural := 36;
    constant C_V3_H3_CLOSE_ACTIVE_VERSION_LO    : natural := 37;
    constant C_V3_H3_CLOSE_ACTIVE_VERSION_HI    : natural := 52;
    constant C_V3_H3_CLOSE_EXPECTED_COLUMNS_LO  : natural := 53;
    constant C_V3_H3_CLOSE_EXPECTED_COLUMNS_HI  : natural := 68;
    constant C_V3_H3_CLOSE_TRAILING_GAP_LO      : natural := 69;
    constant C_V3_H3_CLOSE_TRAILING_GAP_HI      : natural := 84;
    constant C_V3_H3_CLOSE_ALL_HOLE_BIT         : natural := 85;
    constant C_V3_H3_CLOSE_FAULTED_BIT          : natural := 86;

    constant C_V3_H3_LANE_CELL_LO               : natural := 0;
    constant C_V3_H3_LANE_CELL_HI               : natural := 318;
    constant C_V3_H3_LANE_SLOT_INDEX_LO         : natural := 319;
    constant C_V3_H3_LANE_SLOT_INDEX_HI         : natural := 324;
    constant C_V3_H3_LANE_SLOT_COUNT_LO         : natural := 325;
    constant C_V3_H3_LANE_SLOT_COUNT_HI         : natural := 330;
    constant C_V3_H3_LANE_LINE_START_BIT        : natural := 331;
    constant C_V3_H3_LANE_LINE_END_BIT          : natural := 332;
    constant C_V3_H3_LANE_FIRST_COLUMN_BIT      : natural := 333;
    constant C_V3_H3_LANE_LAST_COLUMN_BIT       : natural := 334;
    constant C_V3_H3_LANE_GAP_BEFORE_LO         : natural := 335;
    constant C_V3_H3_LANE_GAP_BEFORE_HI         : natural := 350;
    constant C_V3_H3_LANE_BLANK_BIT             : natural := 351;
    constant C_V3_H3_LANE_LINE_FAULTED_BIT      : natural := 352;
    constant C_V3_H3_LANE_RESERVED_LO           : natural := 353;
    constant C_V3_H3_LANE_RESERVED_HI           : natural := 359;

    constant C_V3_H3_FAULT_CONTEXT_BIT          : natural := 0;
    constant C_V3_H3_FAULT_UNEXPECTED_BIT       : natural := 1;
    constant C_V3_H3_FAULT_DUPLICATE_CELL_BIT   : natural := 2;
    constant C_V3_H3_FAULT_DUPLICATE_TERM_BIT   : natural := 3;
    constant C_V3_H3_FAULT_MISSING_CELL_BIT     : natural := 4;
    constant C_V3_H3_FAULT_GEOMETRY_BIT         : natural := 5;
    constant C_V3_H3_FAULT_COLUMN_GAP_BIT       : natural := 6;
    constant C_V3_H3_FAULT_MASKED_DROP_BIT      : natural := 7;
    constant C_V3_H3_CONTROL_HAS_CLOSE_BIT      : natural := 8;
    constant C_V3_H3_CONTROL_CLOSE_LO           : natural := 9;
    constant C_V3_H3_CONTROL_CLOSE_HI           : natural := 95;
    constant C_V3_H3_CONTROL_CELLS_DONE_BIT     : natural := 96;
    constant C_V3_H3_CONTROL_SHOT_CONTEXT_LO    : natural := 97;
    constant C_V3_H3_CONTROL_SHOT_CONTEXT_HI    : natural := 258;
    constant C_V3_H3_CONTROL_RESERVED_LO        : natural := 259;
    constant C_V3_H3_CONTROL_RESERVED_HI        : natural := 263;

end package lidar_v3_hls_contract_pkg;
