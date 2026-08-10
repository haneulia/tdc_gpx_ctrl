#ifndef LIDAR_V3_H1_RAW_HIT_CONTRACT_HPP
#define LIDAR_V3_H1_RAW_HIT_CONTRACT_HPP

#include <ap_int.h>
#include <cstdint>

#include "lidar_v3_hls_bit_field.hpp"
#include "lidar_v3_hls_limits.hpp"

namespace lidar_v3 {
namespace h1 {

// H1 input event kind. A DATA event carries one external TDC-GPX I-Mode word;
// the other values mark physical IFIFO progress or a bounded drain timeout.
enum class raw_event_kind_t : std::uint8_t {
    data = 0,
    ififo1_done = 1,
    drain_done = 2,
    timeout = 3
};

// Complete V2 Shot snapshot transported with every Raw/Hit/Cell event. The
// measured-data reference event is the synchronized fire_done approval that
// creates start_tdc, called the measurement start reference instant (T0).
constexpr unsigned kShotContextRecordBits = 162U;
using shot_context_t = ap_uint<kShotContextRecordBits>;

struct shot_context_layout {
    using request_valid = bit_field_t<0U, 1U>;
    using mirror_face_index = bit_field_t<request_valid::end, 3U>;
    using encoder_position_state =
        bit_field_t<mirror_face_index::end, 15U>;
    using direction_is_ccw = bit_field_t<encoder_position_state::end, 1U>;
    using shot_column_index = bit_field_t<direction_is_ccw::end, 16U>;
    using is_last_shot_column_in_face =
        bit_field_t<shot_column_index::end, 1U>;
    using source_is_simulation =
        bit_field_t<is_last_shot_column_in_face::end, 1U>;
    using encoder_to_scheduler_latency_clks =
        bit_field_t<source_is_simulation::end, 8U>;
    using encoder_latency_is_valid =
        bit_field_t<encoder_to_scheduler_latency_clks::end, 1U>;
    using active_configuration_version =
        bit_field_t<encoder_latency_is_valid::end, 16U>;
    using fire_command_to_t0_latency_clks =
        bit_field_t<active_configuration_version::end, 32U>;
    using measurement_start_t0_timestamp_ticks =
        bit_field_t<fire_command_to_t0_latency_clks::end, 64U>;
    using measurement_start_timestamp_is_valid =
        bit_field_t<measurement_start_t0_timestamp_ticks::end, 1U>;
    using measurement_start_time_sync_is_valid =
        bit_field_t<measurement_start_timestamp_is_valid::end, 1U>;
    using context_is_valid =
        bit_field_t<measurement_start_time_sync_is_valid::end, 1U>;
};

// H3 compares Shot contexts in fixed 32-bit pieces to avoid synthesizing one
// long equality carry chain. These aliases describe only that implementation
// partition; they do not add fields to the 162-bit Shot context ABI.
struct shot_context_compare_layout {
    using bits_31_to_0 = bit_field_t<0U, 32U>;
    using bits_63_to_32 = bit_field_t<bits_31_to_0::end, 32U>;
    using bits_95_to_64 = bit_field_t<bits_63_to_32::end, 32U>;
    using bits_127_to_96 = bit_field_t<bits_95_to_64::end, 32U>;
    using bits_159_to_128 = bit_field_t<bits_127_to_96::end, 32U>;
    using bits_161_to_160 = bit_field_t<bits_159_to_128::end, 2U>;
};

static_assert(shot_context_layout::context_is_valid::end ==
                  kShotContextRecordBits,
              "H1 Shot context layout must cover exactly 162 bits");
static_assert(shot_context_compare_layout::bits_161_to_160::end ==
                  kShotContextRecordBits,
              "H1 Shot context comparison chunks must cover all context bits");

// H1 input AXI TDATA. Semantic bits [214:0] are the V2 Raw Event; bit 215 is
// byte-alignment reserve and must be zero. TVALID owns transfer validity.
struct raw_event_layout {
    using event_kind = bit_field_t<0U, 2U>;
    using tdc_chip_index = bit_field_t<event_kind::end, 2U>;
    using ififo_bank_select = bit_field_t<tdc_chip_index::end, 1U>;
    using tdc_gpx_imode_word = bit_field_t<ififo_bank_select::end, 28U>;
    using upstream_event_faulted = bit_field_t<tdc_gpx_imode_word::end, 1U>;
    using timeout_cause_bitmap =
        bit_field_t<upstream_event_faulted::end, 3U>;
    using shot_context =
        bit_field_t<timeout_cause_bitmap::end, kShotContextRecordBits>;
    using tdc_chip_shot_sequence = bit_field_t<shot_context::end, 16U>;
    using reserved_zero = bit_field_t<tdc_chip_shot_sequence::end, 1U>;
};

constexpr unsigned kRawEventSemanticBits = raw_event_layout::reserved_zero::low;
constexpr unsigned kRawEventAxisTdataBits = raw_event_layout::reserved_zero::end;
using raw_event_axis_t = ap_uint<kRawEventAxisTdataBits>;

// External TDC-GPX I-Mode word contained in raw_event_layout::tdc_gpx_imode_word.
struct tdc_gpx_imode_word_layout {
    using distance_hit_17bit = bit_field_t<0U, 17U>;
    using edge_slope_is_rise = bit_field_t<distance_hit_17bit::end, 1U>;
    using start_number = bit_field_t<edge_slope_is_rise::end, 8U>;
    using channel_index_within_ififo = bit_field_t<start_number::end, 2U>;
};

constexpr unsigned kTdcGpxImodeWordBits =
    tdc_gpx_imode_word_layout::channel_index_within_ififo::end;
static_assert(kTdcGpxImodeWordBits == 28U,
              "External TDC-GPX I-Mode word must remain 28 bits");

// H1 semantic output record. distance_hit_17bit is meaningful only for DATA
// events accepted without Chip/STOP/slope faults. Other fields preserve event
// identity so downstream stages can diagnose and close a Shot deterministically.
struct decoded_hit_event_layout {
    using event_kind = bit_field_t<0U, 2U>;
    using tdc_chip_index = bit_field_t<event_kind::end, 2U>;
    using ififo_bank_select = bit_field_t<tdc_chip_index::end, 1U>;
    using tdc_gpx_channel_index = bit_field_t<ififo_bank_select::end, 2U>;
    using logical_stop_channel_index =
        bit_field_t<tdc_gpx_channel_index::end, 3U>;
    using tdc_start_number =
        bit_field_t<logical_stop_channel_index::end, 8U>;
    using edge_slope_is_rise = bit_field_t<tdc_start_number::end, 1U>;
    using distance_hit_17bit = bit_field_t<edge_slope_is_rise::end, 17U>;
    using upstream_event_faulted = bit_field_t<distance_hit_17bit::end, 1U>;
    using timeout_cause_bitmap =
        bit_field_t<upstream_event_faulted::end, 3U>;
    using shot_context =
        bit_field_t<timeout_cause_bitmap::end, kShotContextRecordBits>;
    using tdc_chip_shot_sequence = bit_field_t<shot_context::end, 16U>;
};

constexpr unsigned kDecodedHitEventRecordBits =
    decoded_hit_event_layout::tdc_chip_shot_sequence::end;
using decoded_hit_event_t = ap_uint<kDecodedHitEventRecordBits>;

// One H1 result is emitted for every accepted Raw Event. contains_hit_event
// decides whether decoded_hit_event is forwarded; fault bits remain valid for
// the acknowledgement even when no Hit is forwarded.
struct decoder_result_layout {
    using decoded_hit_event = bit_field_t<0U, kDecodedHitEventRecordBits>;
    using contains_hit_event = bit_field_t<decoded_hit_event::end, 1U>;
    using tdc_chip_index_fault = bit_field_t<contains_hit_event::end, 1U>;
    using stop_channel_index_fault =
        bit_field_t<tdc_chip_index_fault::end, 1U>;
    using edge_slope_assignment_fault =
        bit_field_t<stop_channel_index_fault::end, 1U>;
    using reserved_zero = bit_field_t<edge_slope_assignment_fault::end, 2U>;
};

constexpr unsigned kDecoderResultSemanticBits =
    decoder_result_layout::reserved_zero::low;
constexpr unsigned kDecoderResultAxisTdataBits =
    decoder_result_layout::reserved_zero::end;
using decoder_result_axis_t = ap_uint<kDecoderResultAxisTdataBits>;

static_assert(raw_event_layout::reserved_zero::end == 216U,
              "H1 Raw Event ABI width changed unexpectedly");
static_assert(kDecodedHitEventRecordBits == 218U,
              "H1 decoded Hit Event ABI width changed unexpectedly");
static_assert(decoder_result_layout::reserved_zero::end == 224U,
              "H1 decoder result ABI width changed unexpectedly");
static_assert((kRawEventAxisTdataBits % 8U) == 0U,
              "H1 Raw Event AXIS TDATA must be byte aligned");
static_assert((kDecoderResultAxisTdataBits % 8U) == 0U,
              "H1 decoder result AXIS TDATA must be byte aligned");

// Configuration sampled with an accepted H1 Raw Event. Chip/STOP counts are
// structural build limits; masks are the active Runtime topology snapshot.
struct decoder_configuration_t {
    std::uint8_t build_tdc_chip_count;
    std::uint8_t build_stop_channels_per_chip;
    std::uint8_t runtime_enabled_rise_chip_mask;
    std::uint8_t runtime_enabled_fall_chip_mask;
};

inline std::uint32_t read_tdc_gpx_imode_word(
    const raw_event_axis_t &raw_event) {
    return read_field<raw_event_layout::tdc_gpx_imode_word>(raw_event).to_uint();
}

inline decoded_hit_event_t read_decoder_result_hit_event(
    const decoder_result_axis_t &result) {
    return read_field<decoder_result_layout::decoded_hit_event>(result);
}

inline bool decoder_result_contains_hit_event(
    const decoder_result_axis_t &result) {
    return read_flag<decoder_result_layout::contains_hit_event>(result);
}

}  // namespace h1
}  // namespace lidar_v3

#endif
