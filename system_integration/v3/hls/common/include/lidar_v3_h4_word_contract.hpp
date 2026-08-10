#ifndef LIDAR_V3_H4_WORD_CONTRACT_HPP
#define LIDAR_V3_H4_WORD_CONTRACT_HPP

#include <ap_int.h>
#include <cstdint>

#include "lidar_v3_h1_raw_hit_contract.hpp"
#include "lidar_v3_h2_cell_contract.hpp"
#include "lidar_v3_h3_frame_contract.hpp"
#include "lidar_v3_hls_bit_field.hpp"

namespace lidar_v3 {
namespace h4 {

// H4 consumes one width-independent H3 Lane Cell or one H3 Face-close result.
// The final AXI4-Stream width is deliberately absent from this event; only the
// active Lane Profile records whether the later RTL packer uses 32 or 64 bits.
enum class formatter_input_kind_t : std::uint8_t {
    ordered_lane_cell = 0,
    face_close_result = 1
};

struct formatter_input_layout {
    using event_body = bit_field_t<0U, h3::kOrderedLaneCellAxisTdataBits>;
    using ordered_lane_cell =
        bit_field_t<0U, h3::kOrderedLaneCellAxisTdataBits>;
    using face_close_result = bit_field_t<0U, h3::kFaceCloseResultBits>;
    using event_kind = bit_field_t<event_body::end, 1U>;
    using reset_epoch = bit_field_t<event_kind::end, 8U>;
    using reserved_zero = bit_field_t<reset_epoch::end, 7U>;
};

constexpr unsigned kFormatterInputSemanticBits =
    formatter_input_layout::reserved_zero::low;
constexpr unsigned kFormatterInputAxisTdataBits =
    formatter_input_layout::reserved_zero::end;
using formatter_input_axis_t = ap_uint<kFormatterInputAxisTdataBits>;

// Registered Face-boundary profile. The RTL profile manager calculates this
// once before activation; H4 validates and consumes it without division or
// variable-width alignment arithmetic in the streaming path.
struct lane_profile_layout {
    using profile_is_valid = bit_field_t<0U, 1U>;
    using lane_is_enabled = bit_field_t<profile_is_valid::end, 1U>;
    using edge_slope_is_rise = bit_field_t<lane_is_enabled::end, 1U>;
    // 0 = 32-bit AXI4-Stream Beat, 1 = 64-bit AXI4-Stream Beat.
    using output_axis_width_code = bit_field_t<edge_slope_is_rise::end, 2U>;
    using lane_cell_slot_count =
        bit_field_t<output_axis_width_code::end, 6U>;
    // Active Runtime Return setting. H4 emits exactly this many PACKED17 Hit
    // slots per Cell, even when fewer physical Returns were received.
    using serialized_return_slot_count =
        bit_field_t<lane_cell_slot_count::end, 3U>;
    using serialized_cell_word_count =
        bit_field_t<serialized_return_slot_count::end, 3U>;
    using planned_shot_column_count =
        bit_field_t<serialized_cell_word_count::end, 16U>;
    using raw_shot_line_word_count =
        bit_field_t<planned_shot_column_count::end, 9U>;
    using aligned_hsize_word_count =
        bit_field_t<raw_shot_line_word_count::end, 9U>;
    using hsize_bytes = bit_field_t<aligned_hsize_word_count::end, 16U>;
    using face_footer_line_count = bit_field_t<hsize_bytes::end, 2U>;
    using vsize_line_count =
        bit_field_t<face_footer_line_count::end, 16U>;
    using active_configuration_version =
        bit_field_t<vsize_line_count::end, 16U>;
    using reserved_zero =
        bit_field_t<active_configuration_version::end, 3U>;
};

constexpr unsigned kLaneProfileSemanticBits =
    lane_profile_layout::reserved_zero::low;
constexpr unsigned kLaneProfileBits = lane_profile_layout::reserved_zero::end;
using lane_profile_t = ap_uint<kLaneProfileBits>;

enum class canonical_word_kind_t : std::uint8_t {
    shot_metadata = 0,
    packed17_cell = 1,
    face_footer = 2
};

// One H4 output transfer is one canonical 32-bit DDR word plus the Line/Frame
// boundary information required by the retained RTL 32/64-bit AXIS packer.
// PACKED17 means each Hit keeps 16 low bits in Hit Words and its 17th bit in
// the final Cell Metadata Word; it does not mean the AXIS Beat is 17 bits.
struct canonical_line_word_layout {
    using canonical_word_32bit = bit_field_t<0U, 32U>;
    using word_kind = bit_field_t<canonical_word_32bit::end, 2U>;
    using word_index_within_line = bit_field_t<word_kind::end, 9U>;
    using line_word_count =
        bit_field_t<word_index_within_line::end, 9U>;
    using is_line_start = bit_field_t<line_word_count::end, 1U>;
    using is_line_end = bit_field_t<is_line_start::end, 1U>;
    using is_frame_end = bit_field_t<is_line_end::end, 1U>;
    using is_first_shot_column = bit_field_t<is_frame_end::end, 1U>;
    using is_last_shot_column =
        bit_field_t<is_first_shot_column::end, 1U>;
    using line_represents_missing_shot =
        bit_field_t<is_last_shot_column::end, 1U>;
    using shot_line_is_faulted =
        bit_field_t<line_represents_missing_shot::end, 1U>;
    // H4 expands all missing columns, so this field must be zero at H4 output.
    using unexpanded_missing_shot_columns =
        bit_field_t<shot_line_is_faulted::end, 16U>;
    using lane_cell_slot_count =
        bit_field_t<unexpanded_missing_shot_columns::end, 6U>;
    using serialized_cell_word_count =
        bit_field_t<lane_cell_slot_count::end, 3U>;
    using shot_context =
        bit_field_t<serialized_cell_word_count::end,
                    h1::kShotContextRecordBits>;
    using reserved_zero = bit_field_t<shot_context::end, 2U>;
};

constexpr unsigned kCanonicalLineWordSemanticBits =
    canonical_line_word_layout::reserved_zero::low;
constexpr unsigned kCanonicalLineWordAxisTdataBits =
    canonical_line_word_layout::reserved_zero::end;
using canonical_line_word_axis_t =
    ap_uint<kCanonicalLineWordAxisTdataBits>;

enum class formatter_fault_bit_t : unsigned {
    invalid_active_lane_profile = 0U,
    edge_slope_mismatch = 1U,
    ordered_cell_geometry_mismatch = 2U,
    shot_context_or_sequence_mismatch = 3U,
    serialized_return_contract_mismatch = 4U,
    face_close_contract_mismatch = 5U,
    reserved_input_nonzero = 6U
};

// Bit 7 of fault_event_bitmap remains reserved zero. formatter_input.event_kind
// is one bit and therefore has no invalid encoding; do not create dead fault
// semantics for an impossible third input kind.

// Exactly one control result follows each accepted H4 input after all words
// from that invocation have been accepted. emitted_line_count includes Hole,
// real Shot, and Footer Lines produced by that invocation.
struct formatter_control_layout {
    using fault_event_bitmap = bit_field_t<0U, 8U>;
    using face_footer_was_emitted =
        bit_field_t<fault_event_bitmap::end, 1U>;
    using emitted_line_count =
        bit_field_t<face_footer_was_emitted::end, 17U>;
    using reserved_zero = bit_field_t<emitted_line_count::end, 6U>;
};

constexpr unsigned kFormatterControlSemanticBits =
    formatter_control_layout::reserved_zero::low;
constexpr unsigned kFormatterControlAxisTdataBits =
    formatter_control_layout::reserved_zero::end;
using formatter_control_axis_t = ap_uint<kFormatterControlAxisTdataBits>;

static_assert(kFormatterInputSemanticBits == 369U,
              "H4 formatter input semantic width changed unexpectedly");
static_assert(kFormatterInputAxisTdataBits == 376U,
              "H4 formatter input AXIS width changed unexpectedly");
static_assert(kLaneProfileSemanticBits == 101U,
              "H4 Lane Profile semantic width changed unexpectedly");
static_assert(kLaneProfileBits == 104U,
              "H4 Lane Profile width changed unexpectedly");
static_assert(kCanonicalLineWordSemanticBits == 246U,
              "H4 canonical Word semantic width changed unexpectedly");
static_assert(kCanonicalLineWordAxisTdataBits == 248U,
              "H4 canonical Word AXIS width changed unexpectedly");
static_assert(kFormatterControlSemanticBits == 26U,
              "H4 control semantic width changed unexpectedly");
static_assert(kFormatterControlAxisTdataBits == 32U,
              "H4 control AXIS width changed unexpectedly");
static_assert((kFormatterInputAxisTdataBits % 8U) == 0U,
              "H4 input AXIS TDATA must be byte aligned");
static_assert((kLaneProfileBits % 8U) == 0U,
              "H4 Lane Profile must be byte aligned");
static_assert((kCanonicalLineWordAxisTdataBits % 8U) == 0U,
              "H4 output AXIS TDATA must be byte aligned");

inline formatter_input_kind_t read_formatter_input_kind(
    const formatter_input_axis_t &input) {
    return read_flag<formatter_input_layout::event_kind>(input)
               ? formatter_input_kind_t::face_close_result
               : formatter_input_kind_t::ordered_lane_cell;
}

inline h3::ordered_lane_cell_axis_t read_ordered_lane_cell(
    const formatter_input_axis_t &input) {
    return read_field<formatter_input_layout::ordered_lane_cell>(input);
}

inline h3::face_close_result_t read_face_close_result(
    const formatter_input_axis_t &input) {
    return read_field<formatter_input_layout::face_close_result>(input);
}

}  // namespace h4
}  // namespace lidar_v3

#endif
