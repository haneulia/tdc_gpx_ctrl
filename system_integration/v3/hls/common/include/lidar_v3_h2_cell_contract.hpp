#ifndef LIDAR_V3_H2_CELL_CONTRACT_HPP
#define LIDAR_V3_H2_CELL_CONTRACT_HPP

#include <ap_int.h>
#include <cstdint>

#include "lidar_v3_h1_raw_hit_contract.hpp"
#include "lidar_v3_hls_bit_field.hpp"
#include "lidar_v3_hls_limits.hpp"

namespace lidar_v3 {
namespace h2 {

// H2 preserves the H1 event kind so DATA, IFIFO progress and bounded timeout
// events can share one deterministic input/output protocol.
enum class cell_event_kind_t : std::uint8_t {
    data = 0,
    ififo1_done = 1,
    drain_done = 2,
    timeout = 3
};

// H2 input AXI TDATA. reset_epoch is sampled with the accepted decoded Hit
// event. Changing the epoch aborts buffered partial Cells without relying on
// an asynchronous sideband pulse. Reserved bits must remain zero.
struct collector_input_layout {
    using decoded_hit_event =
        bit_field_t<0U, h1::kDecodedHitEventRecordBits>;
    using reserved_zero = bit_field_t<decoded_hit_event::end, 6U>;
    using reset_epoch = bit_field_t<reserved_zero::end, 8U>;
};

constexpr unsigned kCollectorInputAxisTdataBits =
    collector_input_layout::reset_epoch::end;
using collector_input_axis_t = ap_uint<kCollectorInputAxisTdataBits>;

// One Cell is one Shot x one TDC-GPX Chip x one logical STOP channel x one
// edge slope. All seven physical Return slots are structurally present.
// visible_return_count is the number of valid leading Return values in this
// Cell. serialized_return_slot_count is the active Runtime Return setting and
// therefore owns the number of PACKED17 Hit slots serialized by H4. The
// synthesized physical collection upper bound remains a separate H2 scalar.
constexpr unsigned kPackedReturnHitBits =
    limits::kMaximumReturnCountPerStop * 17U;

struct cell_event_layout {
    using event_kind = bit_field_t<0U, 2U>;
    using tdc_chip_index = bit_field_t<event_kind::end, 2U>;
    using ififo_bank_select = bit_field_t<tdc_chip_index::end, 1U>;
    using logical_stop_channel_index =
        bit_field_t<ififo_bank_select::end, 3U>;
    using edge_slope_is_rise =
        bit_field_t<logical_stop_channel_index::end, 1U>;
    using visible_return_count = bit_field_t<edge_slope_is_rise::end, 3U>;
    using serialized_return_slot_count =
        bit_field_t<visible_return_count::end, 3U>;
    using packed_distance_hits_17bit =
        bit_field_t<serialized_return_slot_count::end, kPackedReturnHitBits>;
    using hit_was_dropped =
        bit_field_t<packed_distance_hits_17bit::end, 1U>;
    using return_overflow = bit_field_t<hit_was_dropped::end, 1U>;
    using error_fill_inserted = bit_field_t<return_overflow::end, 1U>;
    using cell_is_faulted = bit_field_t<error_fill_inserted::end, 1U>;
    using timeout_cause_bitmap = bit_field_t<cell_is_faulted::end, 3U>;
    using shot_context =
        bit_field_t<timeout_cause_bitmap::end, h1::kShotContextRecordBits>;
    using tdc_chip_shot_sequence = bit_field_t<shot_context::end, 16U>;
};

constexpr unsigned kCellEventRecordBits =
    cell_event_layout::tdc_chip_shot_sequence::end;
using cell_event_record_t = ap_uint<kCellEventRecordBits>;

// One H2 result acknowledges every accepted input. A terminal event can emit
// several contains_cell_event=1 results and then one control-only result.
// Fault bits remain valid even when no Cell is carried.
struct collector_result_layout {
    using cell_event = bit_field_t<0U, kCellEventRecordBits>;
    using contains_cell_event = bit_field_t<cell_event::end, 1U>;
    using shot_context_fault =
        bit_field_t<contains_cell_event::end, 1U>;
    using return_overflow_fault = bit_field_t<shot_context_fault::end, 1U>;
    using nonzero_start_number_fault =
        bit_field_t<return_overflow_fault::end, 1U>;
    using return_capacity_drop_fault =
        bit_field_t<nonzero_start_number_fault::end, 1U>;
    using reserved_zero = bit_field_t<return_capacity_drop_fault::end, 4U>;
};

constexpr unsigned kCollectorResultSemanticBits =
    collector_result_layout::reserved_zero::low;
constexpr unsigned kCollectorResultAxisTdataBits =
    collector_result_layout::reserved_zero::end;
using collector_result_axis_t = ap_uint<kCollectorResultAxisTdataBits>;

static_assert(collector_input_layout::decoded_hit_event::low == 0U,
              "H2 collector input must start with the H1 decoded Hit Event");
static_assert(collector_input_layout::reserved_zero::width == 6U,
              "H2 collector input alignment reserve changed unexpectedly");
static_assert(kCollectorInputAxisTdataBits == 232U,
              "H2 collector input AXIS ABI width changed unexpectedly");
static_assert(kPackedReturnHitBits == 119U,
              "H2 must preserve seven 17-bit Return slots");
static_assert(kCellEventRecordBits == 319U,
              "H2 Cell Event ABI width changed unexpectedly");
static_assert(kCollectorResultSemanticBits == 324U,
              "H2 collector result semantic width changed unexpectedly");
static_assert(kCollectorResultAxisTdataBits == 328U,
              "H2 collector result AXIS ABI width changed unexpectedly");
static_assert((kCollectorInputAxisTdataBits % 8U) == 0U,
              "H2 collector input AXIS TDATA must be byte aligned");
static_assert((kCollectorResultAxisTdataBits % 8U) == 0U,
              "H2 collector result AXIS TDATA must be byte aligned");

inline h1::decoded_hit_event_t read_collector_input_hit_event(
    const collector_input_axis_t &input) {
    return read_field<collector_input_layout::decoded_hit_event>(input);
}

inline cell_event_record_t read_collector_result_cell_event(
    const collector_result_axis_t &result) {
    return read_field<collector_result_layout::cell_event>(result);
}

inline bool collector_result_contains_cell_event(
    const collector_result_axis_t &result) {
    return read_flag<collector_result_layout::contains_cell_event>(result);
}

}  // namespace h2
}  // namespace lidar_v3

#endif
