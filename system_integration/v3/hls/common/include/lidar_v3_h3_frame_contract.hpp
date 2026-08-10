#ifndef LIDAR_V3_H3_FRAME_CONTRACT_HPP
#define LIDAR_V3_H3_FRAME_CONTRACT_HPP

#include <ap_int.h>
#include <cstdint>

#include "lidar_v3_h1_raw_hit_contract.hpp"
#include "lidar_v3_h2_cell_contract.hpp"
#include "lidar_v3_hls_bit_field.hpp"
#include "lidar_v3_hls_limits.hpp"

namespace lidar_v3 {
namespace h3 {

enum class assembler_input_kind_t : std::uint8_t {
    cell_event = 0,
    face_close_event = 1
};

// One Processing-owned Face-close event. expected_shot_column_count is the
// active face geometry for this Face, not the build-time maximum capacity.
struct face_close_event_layout {
    using frame_identifier = bit_field_t<0U, 32U>;
    using mirror_face_index = bit_field_t<frame_identifier::end, 3U>;
    using direction_is_ccw = bit_field_t<mirror_face_index::end, 1U>;
    using source_is_simulation = bit_field_t<direction_is_ccw::end, 1U>;
    using active_configuration_version =
        bit_field_t<source_is_simulation::end, 16U>;
    using expected_shot_column_count =
        bit_field_t<active_configuration_version::end, 16U>;
};

constexpr unsigned kFaceCloseEventRecordBits =
    face_close_event_layout::expected_shot_column_count::end;
using face_close_event_record_t = ap_uint<kFaceCloseEventRecordBits>;

// H3 accepts either an H2 Cell Event in event_body or a Face-close event in
// the low 69 bits of event_body. reset_epoch atomically discards stale H3
// partial state when Processing requests an abort/restart generation.
struct assembler_input_layout {
    using event_body = bit_field_t<0U, h2::kCellEventRecordBits>;
    using face_close_event = bit_field_t<0U, kFaceCloseEventRecordBits>;
    using event_kind = bit_field_t<event_body::end, 1U>;
    using reset_epoch = bit_field_t<event_kind::end, 8U>;
};

constexpr unsigned kAssemblerInputAxisTdataBits =
    assembler_input_layout::reset_epoch::end;
using assembler_input_axis_t = ap_uint<kAssemblerInputAxisTdataBits>;

// Internal LUTRAM entry. Shot context and active topology are stored once per
// Shot; this entry keeps only the fields that vary per Chip/STOP/slope lane.
struct lane_cell_storage_layout {
    using packed_distance_hits_17bit =
        bit_field_t<0U, h2::kPackedReturnHitBits>;
    using visible_return_count =
        bit_field_t<packed_distance_hits_17bit::end, 3U>;
    using serialized_return_slot_count =
        bit_field_t<visible_return_count::end, 3U>;
    using hit_was_dropped =
        bit_field_t<serialized_return_slot_count::end, 1U>;
    using return_overflow = bit_field_t<hit_was_dropped::end, 1U>;
    using error_fill_inserted = bit_field_t<return_overflow::end, 1U>;
    using cell_is_faulted = bit_field_t<error_fill_inserted::end, 1U>;
    using timeout_cause_bitmap = bit_field_t<cell_is_faulted::end, 3U>;
    using tdc_chip_shot_sequence =
        bit_field_t<timeout_cause_bitmap::end, 16U>;
};

constexpr unsigned kLaneCellStorageBits =
    lane_cell_storage_layout::tdc_chip_shot_sequence::end;
using lane_cell_storage_t = ap_uint<kLaneCellStorageBits>;

// One H3 lane output is one canonical Frame Cell slot. It remains independent
// of the later 32/64-bit PACKED17 serialization and DDR Frame/Footer layout.
struct ordered_lane_cell_layout {
    using cell_event = bit_field_t<0U, h2::kCellEventRecordBits>;
    using lane_cell_slot_index = bit_field_t<cell_event::end, 6U>;
    using lane_cell_slot_count =
        bit_field_t<lane_cell_slot_index::end, 6U>;
    using is_first_cell_in_shot_line =
        bit_field_t<lane_cell_slot_count::end, 1U>;
    using is_last_cell_in_shot_line =
        bit_field_t<is_first_cell_in_shot_line::end, 1U>;
    using is_first_shot_column_in_face =
        bit_field_t<is_last_cell_in_shot_line::end, 1U>;
    using is_last_shot_column_in_face =
        bit_field_t<is_first_shot_column_in_face::end, 1U>;
    using missing_shot_columns_before =
        bit_field_t<is_last_shot_column_in_face::end, 16U>;
    using is_missing_cell_placeholder =
        bit_field_t<missing_shot_columns_before::end, 1U>;
    using shot_line_is_faulted =
        bit_field_t<is_missing_cell_placeholder::end, 1U>;
    using reserved_zero = bit_field_t<shot_line_is_faulted::end, 7U>;
};

constexpr unsigned kOrderedLaneCellSemanticBits =
    ordered_lane_cell_layout::reserved_zero::low;
constexpr unsigned kOrderedLaneCellAxisTdataBits =
    ordered_lane_cell_layout::reserved_zero::end;
using ordered_lane_cell_axis_t = ap_uint<kOrderedLaneCellAxisTdataBits>;

// Face-close result extends the 69-bit input identity with holes found after
// the last observed Shot and a summary fault. It is embedded in the H3 control
// result and is not a separate AXI stream.
struct face_close_result_layout {
    using close_event = bit_field_t<0U, kFaceCloseEventRecordBits>;
    using trailing_missing_shot_columns =
        bit_field_t<close_event::end, 16U>;
    using entire_face_is_missing =
        bit_field_t<trailing_missing_shot_columns::end, 1U>;
    using face_close_is_faulted =
        bit_field_t<entire_face_is_missing::end, 1U>;
};

constexpr unsigned kFaceCloseResultBits =
    face_close_result_layout::face_close_is_faulted::end;
using face_close_result_t = ap_uint<kFaceCloseResultBits>;

// H3 fault bitmap bit numbers. Each bit is a one-cycle event indication in
// the accepted input's control result; the RTL adapter owns sticky/counters.
enum class assembler_fault_bit_t : unsigned {
    shot_context_mismatch = 0U,
    unexpected_event = 1U,
    duplicate_cell = 2U,
    duplicate_terminal_event = 3U,
    missing_cell = 4U,
    geometry_mismatch = 5U,
    shot_column_gap = 6U,
    masked_lane_drop = 7U
};

// Exactly one H3 control result follows every accepted input. Lane Cells use
// separate rise/fall streams. cell_generation_complete says the HLS core has
// emitted all ordered Cell slots for one Shot; it is not external shot_done.
struct assembler_control_layout {
    using fault_event_bitmap = bit_field_t<0U, 8U>;
    using contains_face_close_result =
        bit_field_t<fault_event_bitmap::end, 1U>;
    using face_close_result =
        bit_field_t<contains_face_close_result::end, kFaceCloseResultBits>;
    using shot_cell_generation_complete =
        bit_field_t<face_close_result::end, 1U>;
    using completed_shot_context =
        bit_field_t<shot_cell_generation_complete::end,
                    h1::kShotContextRecordBits>;
    using reserved_zero = bit_field_t<completed_shot_context::end, 5U>;
};

constexpr unsigned kAssemblerControlSemanticBits =
    assembler_control_layout::reserved_zero::low;
constexpr unsigned kAssemblerControlAxisTdataBits =
    assembler_control_layout::reserved_zero::end;
using assembler_control_axis_t = ap_uint<kAssemblerControlAxisTdataBits>;

static_assert(kFaceCloseEventRecordBits == 69U,
              "H3 Face-close Event ABI width changed unexpectedly");
static_assert(kAssemblerInputAxisTdataBits == 328U,
              "H3 assembler input AXIS ABI width changed unexpectedly");
static_assert(kLaneCellStorageBits == 148U,
              "H3 lane storage ABI width changed unexpectedly");
static_assert(kOrderedLaneCellSemanticBits == 353U,
              "H3 ordered Lane Cell semantic width changed unexpectedly");
static_assert(kOrderedLaneCellAxisTdataBits == 360U,
              "H3 ordered Lane Cell AXIS ABI width changed unexpectedly");
static_assert(kFaceCloseResultBits == 87U,
              "H3 Face-close result ABI width changed unexpectedly");
static_assert(kAssemblerControlSemanticBits == 259U,
              "H3 assembler control semantic width changed unexpectedly");
static_assert(kAssemblerControlAxisTdataBits == 264U,
              "H3 assembler control AXIS ABI width changed unexpectedly");
static_assert((kAssemblerInputAxisTdataBits % 8U) == 0U,
              "H3 assembler input AXIS TDATA must be byte aligned");
static_assert((kOrderedLaneCellAxisTdataBits % 8U) == 0U,
              "H3 ordered Lane Cell AXIS TDATA must be byte aligned");
static_assert((kAssemblerControlAxisTdataBits % 8U) == 0U,
              "H3 assembler control AXIS TDATA must be byte aligned");

inline assembler_input_kind_t read_assembler_input_kind(
    const assembler_input_axis_t &input) {
    return read_flag<assembler_input_layout::event_kind>(input)
               ? assembler_input_kind_t::face_close_event
               : assembler_input_kind_t::cell_event;
}

inline h2::cell_event_record_t read_assembler_input_cell_event(
    const assembler_input_axis_t &input) {
    return read_field<assembler_input_layout::event_body>(input);
}

inline face_close_event_record_t read_assembler_input_face_close_event(
    const assembler_input_axis_t &input) {
    return read_field<assembler_input_layout::face_close_event>(input);
}

}  // namespace h3
}  // namespace lidar_v3

#endif
