#include "gpx_lane_word_formatter_hls.hpp"

#include <cstdint>

namespace {

namespace h1 = lidar_v3::h1;
namespace h2 = lidar_v3::h2;
namespace h3 = lidar_v3::h3;
namespace h4 = lidar_v3::h4;
namespace limits = lidar_v3::limits;

constexpr std::uint8_t kShotMetadataWordCount = 4U;
constexpr std::uint8_t kFaceFooterLogicalWordCount = 8U;
constexpr std::uint32_t kFaceFooterMagic = 0x47504631UL;   // "GPF1"
constexpr std::uint32_t kFaceFooterCommit = 0x434F4D54UL;  // "COMT"
constexpr std::uint16_t kMissingShotEncoderPosition = 0xFFFFU;

static std::uint8_t accepted_reset_epoch;
static bool shot_line_is_open;
static h1::shot_context_t open_shot_context;
static std::uint8_t expected_next_cell_slot;
static bool open_shot_line_is_faulted;

static std::uint16_t completed_shot_line_count;
static bool any_shot_line_fault;
static bool any_missing_shot_line;

struct decoded_profile_t {
    bool valid;
    bool enabled;
    bool slope_is_rise;
    std::uint8_t output_width_code;
    std::uint8_t cell_slot_count;
    std::uint8_t serialized_return_slots;
    std::uint8_t cell_word_count;
    std::uint16_t planned_shot_columns;
    std::uint16_t raw_line_words;
    std::uint16_t hsize_words;
    std::uint16_t hsize_bytes;
    std::uint8_t footer_lines;
    std::uint16_t vsize_lines;
    std::uint16_t active_version;
    bool reserved_is_zero;
};

void clear_face_state() {
    shot_line_is_open = false;
    open_shot_context = 0;
    expected_next_cell_slot = 0U;
    open_shot_line_is_faulted = false;
    completed_shot_line_count = 0U;
    any_shot_line_fault = false;
    any_missing_shot_line = false;
}

void set_fault(std::uint8_t &faults, h4::formatter_fault_bit_t bit) {
    faults = static_cast<std::uint8_t>(
        faults | (1U << static_cast<unsigned>(bit)));
}

decoded_profile_t decode_profile(const h4::lane_profile_t &profile) {
    decoded_profile_t result = {};
    result.valid =
        lidar_v3::read_flag<h4::lane_profile_layout::profile_is_valid>(
            profile);
    result.enabled =
        lidar_v3::read_flag<h4::lane_profile_layout::lane_is_enabled>(
            profile);
    result.slope_is_rise =
        lidar_v3::read_flag<h4::lane_profile_layout::edge_slope_is_rise>(
            profile);
    result.output_width_code =
        lidar_v3::read_field<
            h4::lane_profile_layout::output_axis_width_code>(profile)
            .to_uint();
    result.cell_slot_count =
        lidar_v3::read_field<h4::lane_profile_layout::lane_cell_slot_count>(
            profile)
            .to_uint();
    result.serialized_return_slots =
        lidar_v3::read_field<
            h4::lane_profile_layout::serialized_return_slot_count>(profile)
            .to_uint();
    result.cell_word_count =
        lidar_v3::read_field<
            h4::lane_profile_layout::serialized_cell_word_count>(profile)
            .to_uint();
    result.planned_shot_columns =
        lidar_v3::read_field<
            h4::lane_profile_layout::planned_shot_column_count>(profile)
            .to_uint();
    result.raw_line_words =
        lidar_v3::read_field<
            h4::lane_profile_layout::raw_shot_line_word_count>(profile)
            .to_uint();
    result.hsize_words =
        lidar_v3::read_field<
            h4::lane_profile_layout::aligned_hsize_word_count>(profile)
            .to_uint();
    result.hsize_bytes =
        lidar_v3::read_field<h4::lane_profile_layout::hsize_bytes>(profile)
            .to_uint();
    result.footer_lines =
        lidar_v3::read_field<
            h4::lane_profile_layout::face_footer_line_count>(profile)
            .to_uint();
    result.vsize_lines =
        lidar_v3::read_field<h4::lane_profile_layout::vsize_line_count>(
            profile)
            .to_uint();
    result.active_version =
        lidar_v3::read_field<
            h4::lane_profile_layout::active_configuration_version>(profile)
            .to_uint();
    result.reserved_is_zero =
        lidar_v3::read_field<h4::lane_profile_layout::reserved_zero>(
            profile) == 0U;
    return result;
}

std::uint16_t multiply_cell_count_by_word_count(
    std::uint8_t cell_count,
    std::uint8_t word_count) {
    const std::uint16_t count = cell_count;
    switch (word_count) {
        case 2U:
            return static_cast<std::uint16_t>(count << 1U);
        case 3U:
            return static_cast<std::uint16_t>((count << 1U) + count);
        case 4U:
            return static_cast<std::uint16_t>(count << 2U);
        case 5U:
            return static_cast<std::uint16_t>((count << 2U) + count);
        default:
            return 0U;
    }
}

bool profile_is_consistent(const decoded_profile_t &profile) {
    const std::uint8_t expected_cell_words =
        static_cast<std::uint8_t>(
            ((profile.serialized_return_slots + 1U) >> 1U) + 1U);
    const std::uint16_t expected_raw_words =
        static_cast<std::uint16_t>(
            kShotMetadataWordCount +
            multiply_cell_count_by_word_count(profile.cell_slot_count,
                                               expected_cell_words));
    const std::uint16_t expected_hsize_words =
        profile.output_width_code == 0U
            ? expected_raw_words
            : static_cast<std::uint16_t>(expected_raw_words +
                                         (expected_raw_words & 1U));
    const std::uint8_t expected_footer_lines =
        expected_hsize_words == 0U
            ? 0U
            : static_cast<std::uint8_t>(expected_hsize_words >=
                                                kFaceFooterLogicalWordCount
                                            ? 1U
                                            : 2U);
    const std::uint32_t expected_vsize =
        static_cast<std::uint32_t>(profile.planned_shot_columns) +
        expected_footer_lines;

    return profile.valid && profile.enabled && profile.reserved_is_zero &&
           profile.output_width_code <= 1U &&
           profile.cell_slot_count >= 1U &&
           profile.cell_slot_count <=
               limits::kMaximumTdcGpxChipCount *
                   limits::kMaximumStopChannelsPerChip &&
           profile.serialized_return_slots >= 1U &&
           profile.serialized_return_slots <=
               limits::kMaximumReturnCountPerStop &&
           profile.cell_word_count == expected_cell_words &&
           profile.planned_shot_columns >= 1U &&
           profile.raw_line_words == expected_raw_words &&
           profile.hsize_words == expected_hsize_words &&
           profile.hsize_bytes == (expected_hsize_words << 2U) &&
           profile.footer_lines == expected_footer_lines &&
           expected_vsize <= 0xFFFFU &&
           profile.vsize_lines == expected_vsize;
}

bool contexts_equal(const h1::shot_context_t &left,
                    const h1::shot_context_t &right) {
    std::uint32_t difference = 0U;
    difference |= lidar_v3::read_field<
                      h1::shot_context_compare_layout::bits_31_to_0>(left)
                      .to_uint() ^
                  lidar_v3::read_field<
                      h1::shot_context_compare_layout::bits_31_to_0>(right)
                      .to_uint();
    difference |= lidar_v3::read_field<
                      h1::shot_context_compare_layout::bits_63_to_32>(left)
                      .to_uint() ^
                  lidar_v3::read_field<
                      h1::shot_context_compare_layout::bits_63_to_32>(right)
                      .to_uint();
    difference |= lidar_v3::read_field<
                      h1::shot_context_compare_layout::bits_95_to_64>(left)
                      .to_uint() ^
                  lidar_v3::read_field<
                      h1::shot_context_compare_layout::bits_95_to_64>(right)
                      .to_uint();
    difference |= lidar_v3::read_field<
                      h1::shot_context_compare_layout::bits_127_to_96>(left)
                      .to_uint() ^
                  lidar_v3::read_field<
                      h1::shot_context_compare_layout::bits_127_to_96>(right)
                      .to_uint();
    difference |= lidar_v3::read_field<
                      h1::shot_context_compare_layout::bits_159_to_128>(left)
                      .to_uint() ^
                  lidar_v3::read_field<
                      h1::shot_context_compare_layout::bits_159_to_128>(right)
                      .to_uint();
    difference |= static_cast<std::uint32_t>(
        lidar_v3::read_field<
            h1::shot_context_compare_layout::bits_161_to_160>(left)
            .to_uint() ^
        lidar_v3::read_field<
            h1::shot_context_compare_layout::bits_161_to_160>(right)
            .to_uint());
    return difference == 0U;
}

h1::shot_context_t read_cell_context(
    const h3::ordered_lane_cell_axis_t &ordered_cell) {
    const h2::cell_event_record_t cell =
        lidar_v3::read_field<h3::ordered_lane_cell_layout::cell_event>(
            ordered_cell);
    return lidar_v3::read_field<h2::cell_event_layout::shot_context>(cell);
}

std::uint16_t read_context_shot_column(
    const h1::shot_context_t &context) {
    return lidar_v3::read_field<h1::shot_context_layout::shot_column_index>(
               context)
        .to_uint();
}

h1::shot_context_t make_missing_context_from_real(
    const h1::shot_context_t &real_context,
    std::uint16_t missing_shot_column,
    std::uint16_t planned_shot_columns) {
    h1::shot_context_t result = real_context;
    lidar_v3::write_flag<h1::shot_context_layout::request_valid>(result,
                                                                  false);
    lidar_v3::write_field<h1::shot_context_layout::encoder_position_state>(
        result, 0U);
    lidar_v3::write_field<h1::shot_context_layout::shot_column_index>(
        result, missing_shot_column);
    lidar_v3::write_flag<
        h1::shot_context_layout::is_last_shot_column_in_face>(
        result, missing_shot_column + 1U == planned_shot_columns);
    lidar_v3::write_field<
        h1::shot_context_layout::encoder_to_scheduler_latency_clks>(result,
                                                                    0U);
    lidar_v3::write_flag<h1::shot_context_layout::encoder_latency_is_valid>(
        result, false);
    lidar_v3::write_field<
        h1::shot_context_layout::fire_command_to_t0_latency_clks>(result, 0U);
    lidar_v3::write_field<
        h1::shot_context_layout::measurement_start_t0_timestamp_ticks>(result,
                                                                       0U);
    lidar_v3::write_flag<
        h1::shot_context_layout::measurement_start_timestamp_is_valid>(result,
                                                                        false);
    lidar_v3::write_flag<
        h1::shot_context_layout::measurement_start_time_sync_is_valid>(result,
                                                                        false);
    lidar_v3::write_flag<h1::shot_context_layout::context_is_valid>(result,
                                                                    true);
    return result;
}

h1::shot_context_t make_missing_context_from_close(
    const h3::face_close_result_t &close_result,
    std::uint16_t missing_shot_column,
    std::uint16_t planned_shot_columns) {
    h1::shot_context_t result = 0;
    const h3::face_close_event_record_t close_event =
        lidar_v3::read_field<h3::face_close_result_layout::close_event>(
            close_result);
    lidar_v3::write_field<h1::shot_context_layout::mirror_face_index>(
        result,
        lidar_v3::read_field<h3::face_close_event_layout::mirror_face_index>(
            close_event));
    lidar_v3::write_flag<h1::shot_context_layout::direction_is_ccw>(
        result,
        lidar_v3::read_flag<h3::face_close_event_layout::direction_is_ccw>(
            close_event));
    lidar_v3::write_field<h1::shot_context_layout::shot_column_index>(
        result, missing_shot_column);
    lidar_v3::write_flag<
        h1::shot_context_layout::is_last_shot_column_in_face>(
        result, missing_shot_column + 1U == planned_shot_columns);
    lidar_v3::write_flag<h1::shot_context_layout::source_is_simulation>(
        result,
        lidar_v3::read_flag<
            h3::face_close_event_layout::source_is_simulation>(close_event));
    lidar_v3::write_field<
        h1::shot_context_layout::active_configuration_version>(
        result,
        lidar_v3::read_field<
            h3::face_close_event_layout::active_configuration_version>(
            close_event));
    lidar_v3::write_flag<h1::shot_context_layout::context_is_valid>(result,
                                                                    true);
    return result;
}

h4::canonical_line_word_axis_t make_output_word(
    std::uint32_t data,
    h4::canonical_word_kind_t kind,
    std::uint16_t word_index,
    std::uint16_t line_word_count,
    bool line_start,
    bool line_end,
    bool frame_end,
    bool first_column,
    bool last_column,
    bool missing_line,
    bool line_faulted,
    std::uint8_t cell_slot_count,
    std::uint8_t cell_word_count,
    const h1::shot_context_t &shot_context) {
    h4::canonical_line_word_axis_t result = 0;
    lidar_v3::write_field<
        h4::canonical_line_word_layout::canonical_word_32bit>(result, data);
    lidar_v3::write_field<h4::canonical_line_word_layout::word_kind>(
        result, static_cast<std::uint8_t>(kind));
    lidar_v3::write_field<
        h4::canonical_line_word_layout::word_index_within_line>(result,
                                                                 word_index);
    lidar_v3::write_field<h4::canonical_line_word_layout::line_word_count>(
        result, line_word_count);
    lidar_v3::write_flag<h4::canonical_line_word_layout::is_line_start>(
        result, line_start);
    lidar_v3::write_flag<h4::canonical_line_word_layout::is_line_end>(
        result, line_end);
    lidar_v3::write_flag<h4::canonical_line_word_layout::is_frame_end>(
        result, frame_end);
    lidar_v3::write_flag<
        h4::canonical_line_word_layout::is_first_shot_column>(result,
                                                               first_column);
    lidar_v3::write_flag<
        h4::canonical_line_word_layout::is_last_shot_column>(result,
                                                              last_column);
    lidar_v3::write_flag<
        h4::canonical_line_word_layout::line_represents_missing_shot>(
        result, missing_line);
    lidar_v3::write_flag<
        h4::canonical_line_word_layout::shot_line_is_faulted>(result,
                                                               line_faulted);
    lidar_v3::write_field<
        h4::canonical_line_word_layout::unexpanded_missing_shot_columns>(
        result, 0U);
    lidar_v3::write_field<
        h4::canonical_line_word_layout::lane_cell_slot_count>(result,
                                                               cell_slot_count);
    lidar_v3::write_field<
        h4::canonical_line_word_layout::serialized_cell_word_count>(
        result, cell_word_count);
    lidar_v3::write_field<h4::canonical_line_word_layout::shot_context>(
        result, shot_context);
    return result;
}

std::uint32_t make_shot_metadata_word(const h1::shot_context_t &context,
                                      bool missing_line,
                                      bool line_faulted,
                                      std::uint8_t word_index) {
    std::uint32_t result = 0U;
    if (word_index == 0U && !missing_line) {
        result = context.range(
                            h1::shot_context_layout::
                                measurement_start_t0_timestamp_ticks::low +
                                31U,
                            h1::shot_context_layout::
                                measurement_start_t0_timestamp_ticks::low)
                     .to_uint();
    } else if (word_index == 1U && !missing_line) {
        result = context.range(
                            h1::shot_context_layout::
                                measurement_start_t0_timestamp_ticks::high,
                            h1::shot_context_layout::
                                measurement_start_t0_timestamp_ticks::low +
                                32U)
                     .to_uint();
    } else if (word_index == 2U) {
        result = read_context_shot_column(context);
        const std::uint16_t position =
            missing_line
                ? kMissingShotEncoderPosition
                : static_cast<std::uint16_t>(
                      lidar_v3::read_field<
                          h1::shot_context_layout::encoder_position_state>(
                          context)
                          .to_uint());
        result |= static_cast<std::uint32_t>(position) << 16U;
    } else if (word_index == 3U) {
        if (!missing_line) {
            result |= 1U << 0U;  // data_valid
        } else {
            result |= 1U << 1U;  // hole
        }
        if (lidar_v3::read_flag<
                h1::shot_context_layout::direction_is_ccw>(context)) {
            result |= 1U << 2U;
        }
        if (lidar_v3::read_flag<
                h1::shot_context_layout::source_is_simulation>(context)) {
            result |= 1U << 3U;
        }
        // V2 ABI reserves Bit 4/5 for Shot timeout/abort, but H3 currently
        // carries no explicit per-Shot timeout/abort status. Keep both zero;
        // TDC-GPX Drain timeout remains the Cell Metadata timeout bitmap.
        if (line_faulted) {
            result |= 1U << 6U;
        }
        if (!missing_line &&
            lidar_v3::read_flag<
                h1::shot_context_layout::
                    measurement_start_timestamp_is_valid>(context)) {
            result |= 1U << 7U;
        }
        if (!missing_line &&
            lidar_v3::read_flag<
                h1::shot_context_layout::
                    measurement_start_time_sync_is_valid>(context)) {
            result |= 1U << 8U;
        }
        if (lidar_v3::read_flag<
                h1::shot_context_layout::is_last_shot_column_in_face>(
                context)) {
            result |= 1U << 9U;
        }
        if (!missing_line &&
            lidar_v3::read_flag<
                h1::shot_context_layout::encoder_latency_is_valid>(context)) {
            result |= 1U << 10U;
            result |=
                lidar_v3::read_field<
                    h1::shot_context_layout::
                        encoder_to_scheduler_latency_clks>(context)
                    .to_uint()
                << 11U;
        }
    }
    return result;
}

std::uint32_t read_packed_distance_hit(
    const h2::cell_event_record_t &cell,
    std::uint8_t return_index) {
    constexpr unsigned kHitBase =
        h2::cell_event_layout::packed_distance_hits_17bit::low;
    switch (return_index) {
        case 0U:
            return cell.range(kHitBase + 16U, kHitBase).to_uint();
        case 1U:
            return cell.range(kHitBase + 33U, kHitBase + 17U).to_uint();
        case 2U:
            return cell.range(kHitBase + 50U, kHitBase + 34U).to_uint();
        case 3U:
            return cell.range(kHitBase + 67U, kHitBase + 51U).to_uint();
        case 4U:
            return cell.range(kHitBase + 84U, kHitBase + 68U).to_uint();
        case 5U:
            return cell.range(kHitBase + 101U, kHitBase + 85U).to_uint();
        case 6U:
            return cell.range(kHitBase + 118U, kHitBase + 102U).to_uint();
        default:
            return 0U;
    }
}

void emit_missing_shot_line(
    hls::stream<h4::canonical_line_word_axis_t> &output,
    const h1::shot_context_t &context,
    const decoded_profile_t &profile,
    bool line_faulted) {
    const std::uint16_t shot_column = read_context_shot_column(context);
    const bool first_column = shot_column == 0U;
    const bool last_column =
        shot_column + 1U == profile.planned_shot_columns;
    for (std::uint16_t word_index = 0U;
         word_index < profile.raw_line_words;
         ++word_index) {
#pragma HLS PIPELINE II=1
#pragma HLS LOOP_TRIPCOUNT min=6 max=164
        const bool line_end = word_index + 1U == profile.raw_line_words;
        const h4::canonical_word_kind_t kind =
            word_index < kShotMetadataWordCount
                ? h4::canonical_word_kind_t::shot_metadata
                : h4::canonical_word_kind_t::packed17_cell;
        const std::uint32_t data =
            word_index < kShotMetadataWordCount
                ? make_shot_metadata_word(
                      context, true, line_faulted,
                      static_cast<std::uint8_t>(word_index))
                : 0U;
        output.write(make_output_word(
            data, kind, word_index, profile.raw_line_words,
            word_index == 0U, line_end, false, first_column, last_column,
            true, line_faulted, profile.cell_slot_count,
            profile.cell_word_count, context));
    }
}

void emit_real_shot_metadata(
    hls::stream<h4::canonical_line_word_axis_t> &output,
    const h1::shot_context_t &context,
    const decoded_profile_t &profile,
    bool first_column,
    bool last_column,
    bool line_faulted) {
    for (std::uint8_t word_index = 0U;
         word_index < kShotMetadataWordCount;
         ++word_index) {
#pragma HLS PIPELINE II=1
        output.write(make_output_word(
            make_shot_metadata_word(context, false, line_faulted, word_index),
            h4::canonical_word_kind_t::shot_metadata, word_index,
            profile.raw_line_words, word_index == 0U, false, false,
            first_column, last_column, false, line_faulted,
            profile.cell_slot_count, profile.cell_word_count, context));
    }
}

std::uint32_t make_packed17_cell_word(
    const h3::ordered_lane_cell_axis_t &ordered_cell,
    const decoded_profile_t &profile,
    std::uint8_t local_word_index,
    bool line_faulted) {
    const h2::cell_event_record_t cell =
        lidar_v3::read_field<h3::ordered_lane_cell_layout::cell_event>(
            ordered_cell);
    const bool blank = lidar_v3::read_flag<
        h3::ordered_lane_cell_layout::is_missing_cell_placeholder>(
        ordered_cell);
    const std::uint8_t valid_return_count =
        lidar_v3::read_field<h2::cell_event_layout::visible_return_count>(cell)
            .to_uint();
    const std::uint8_t hit_word_count =
        static_cast<std::uint8_t>(profile.cell_word_count - 1U);

    std::uint32_t result = 0U;
    if (local_word_index < hit_word_count) {
        const std::uint8_t first_return =
            static_cast<std::uint8_t>(local_word_index * 2U);
        for (std::uint8_t half = 0U; half < 2U; ++half) {
#pragma HLS UNROLL
            const std::uint8_t return_index =
                static_cast<std::uint8_t>(first_return + half);
            if (!blank && return_index < valid_return_count &&
                return_index < profile.serialized_return_slots &&
                return_index < limits::kMaximumReturnCountPerStop) {
                const std::uint16_t low16 =
                    static_cast<std::uint16_t>(
                        read_packed_distance_hit(cell, return_index));
                result |= static_cast<std::uint32_t>(low16)
                          << (half * 16U);
            }
        }
    } else {
        for (std::uint8_t return_index = 0U;
             return_index < limits::kMaximumReturnCountPerStop;
             ++return_index) {
#pragma HLS UNROLL
            const bool active = !blank &&
                                return_index < valid_return_count &&
                                return_index <
                                    profile.serialized_return_slots;
            if (active) {
                if ((read_packed_distance_hit(cell, return_index) &
                     0x10000U) != 0U) {
                    result |= 1U << return_index;
                }
                result |= 1U << (7U + return_index);
            }
        }
        result |= static_cast<std::uint32_t>(valid_return_count & 0x7U)
                  << 14U;
        if (lidar_v3::read_flag<
                h2::cell_event_layout::edge_slope_is_rise>(cell)) {
            result |= 1U << 17U;
        }
        result |= lidar_v3::read_field<
                      h2::cell_event_layout::tdc_chip_index>(cell)
                      .to_uint()
                  << 18U;
        result |= lidar_v3::read_field<
                      h2::cell_event_layout::logical_stop_channel_index>(cell)
                      .to_uint()
                  << 20U;
        if (blank) {
            result |= 1U << 23U;
        }
        if (lidar_v3::read_flag<
                h2::cell_event_layout::error_fill_inserted>(cell)) {
            result |= 1U << 24U;
        }
        if (lidar_v3::read_flag<
                h2::cell_event_layout::hit_was_dropped>(cell)) {
            result |= 1U << 25U;
        }
        if (lidar_v3::read_flag<
                h2::cell_event_layout::return_overflow>(cell)) {
            result |= 1U << 26U;
        }
        if (line_faulted ||
            lidar_v3::read_flag<h2::cell_event_layout::cell_is_faulted>(
                cell)) {
            result |= 1U << 27U;
        }
        result |= lidar_v3::read_field<
                      h2::cell_event_layout::timeout_cause_bitmap>(cell)
                      .to_uint()
                  << 28U;
        result |= 1UL << 31U;
    }
    return result;
}

void emit_ordered_cell(
    hls::stream<h4::canonical_line_word_axis_t> &output,
    const h3::ordered_lane_cell_axis_t &ordered_cell,
    const decoded_profile_t &profile) {
    const h1::shot_context_t context = read_cell_context(ordered_cell);
    const std::uint8_t slot_index =
        lidar_v3::read_field<
            h3::ordered_lane_cell_layout::lane_cell_slot_index>(ordered_cell)
            .to_uint();
    const bool first_cell = lidar_v3::read_flag<
        h3::ordered_lane_cell_layout::is_first_cell_in_shot_line>(
        ordered_cell);
    const bool last_cell = lidar_v3::read_flag<
        h3::ordered_lane_cell_layout::is_last_cell_in_shot_line>(ordered_cell);
    const bool first_column = lidar_v3::read_flag<
        h3::ordered_lane_cell_layout::is_first_shot_column_in_face>(
        ordered_cell);
    const bool last_column = lidar_v3::read_flag<
        h3::ordered_lane_cell_layout::is_last_shot_column_in_face>(
        ordered_cell);
    const bool line_faulted = lidar_v3::read_flag<
        h3::ordered_lane_cell_layout::shot_line_is_faulted>(ordered_cell);

    if (first_cell) {
        emit_real_shot_metadata(output, context, profile, first_column,
                                last_column, line_faulted);
    }

    for (std::uint8_t local_word_index = 0U;
         local_word_index < profile.cell_word_count;
         ++local_word_index) {
#pragma HLS PIPELINE II=1
        const std::uint16_t global_word_index =
            static_cast<std::uint16_t>(
                kShotMetadataWordCount +
                multiply_cell_count_by_word_count(
                    slot_index, profile.cell_word_count) +
                local_word_index);
        const bool line_end =
            last_cell && local_word_index + 1U == profile.cell_word_count;
        output.write(make_output_word(
            make_packed17_cell_word(ordered_cell, profile, local_word_index,
                                    line_faulted),
            h4::canonical_word_kind_t::packed17_cell, global_word_index,
            profile.raw_line_words, false, line_end, false, first_column,
            last_column, false, line_faulted, profile.cell_slot_count,
            profile.cell_word_count, context));
    }
}

std::uint32_t make_footer_word(
    const h3::face_close_result_t &close_result,
    const decoded_profile_t &profile,
    std::uint16_t completed_lines,
    bool count_mismatch,
    std::uint8_t logical_word_index) {
    const h3::face_close_event_record_t close_event =
        lidar_v3::read_field<h3::face_close_result_layout::close_event>(
            close_result);
    std::uint32_t result = 0U;
    switch (logical_word_index) {
        case 0U:
            result = kFaceFooterMagic;
            break;
        case 1U:
            result = lidar_v3::read_field<
                         h3::face_close_event_layout::frame_identifier>(
                         close_event)
                         .to_uint();
            break;
        case 2U:
            result = lidar_v3::read_field<
                         h3::face_close_event_layout::mirror_face_index>(
                         close_event)
                         .to_uint();
            if (profile.slope_is_rise) {
                result |= 1U << 3U;
            }
            if (lidar_v3::read_flag<
                    h3::face_close_event_layout::direction_is_ccw>(
                    close_event)) {
                result |= 1U << 4U;
            }
            if (lidar_v3::read_flag<
                    h3::face_close_event_layout::source_is_simulation>(
                    close_event)) {
                result |= 1U << 5U;
            }
            result |= static_cast<std::uint32_t>(
                          profile.output_width_code & 0x3U)
                      << 6U;
            break;
        case 3U:
            result = lidar_v3::read_field<
                         h3::face_close_event_layout::
                             active_configuration_version>(close_event)
                         .to_uint();
            break;
        case 4U:
            result = lidar_v3::read_field<
                         h3::face_close_event_layout::
                             expected_shot_column_count>(close_event)
                         .to_uint();
            result |= static_cast<std::uint32_t>(profile.cell_slot_count)
                      << 16U;
            result |= static_cast<std::uint32_t>(
                          profile.serialized_return_slots)
                      << 22U;
            break;
        case 5U:
            result = profile.hsize_bytes;
            result |= static_cast<std::uint32_t>(profile.vsize_lines) << 16U;
            break;
        case 6U:
            result = completed_lines;
            if (lidar_v3::read_flag<
                    h3::face_close_result_layout::face_close_is_faulted>(
                    close_result)) {
                result |= 1U << 16U;
            }
            if (count_mismatch) {
                result |= 1U << 17U;
            }
            if (any_shot_line_fault) {
                result |= 1U << 18U;
            }
            if (any_missing_shot_line) {
                result |= 1U << 19U;
            }
            // Footer Bit 20/21 summarize the dormant V2 Shot timeout/abort
            // fields and therefore remain zero until an explicit upstream
            // status is added to the H3 ABI. Do not map Cell Drain timeout
            // or reset-epoch abort onto these physically different events.
            if (lidar_v3::read_flag<
                    h3::face_close_result_layout::entire_face_is_missing>(
                    close_result)) {
                result |= 1U << 22U;
            }
            break;
        case 7U:
            result = kFaceFooterCommit;
            break;
        default:
            break;
    }
    return result;
}

void emit_footer_line(
    hls::stream<h4::canonical_line_word_axis_t> &output,
    const h3::face_close_result_t &close_result,
    const decoded_profile_t &profile,
    std::uint16_t completed_lines,
    std::uint8_t logical_word_offset,
    bool final_footer_line) {
    const h1::shot_context_t empty_context = 0;
    const bool count_mismatch =
        completed_lines != profile.planned_shot_columns;
    for (std::uint16_t line_word_index = 0U;
         line_word_index < profile.hsize_words;
         ++line_word_index) {
#pragma HLS PIPELINE II=1
#pragma HLS LOOP_TRIPCOUNT min=6 max=164
        const bool line_end =
            line_word_index + 1U == profile.hsize_words;
        const std::uint16_t logical_word_index =
            static_cast<std::uint16_t>(logical_word_offset) +
            line_word_index;
        const std::uint32_t data =
            logical_word_index < kFaceFooterLogicalWordCount
                ? make_footer_word(
                      close_result, profile, completed_lines,
                      count_mismatch,
                      static_cast<std::uint8_t>(logical_word_index))
                : 0U;
        output.write(make_output_word(
            data, h4::canonical_word_kind_t::face_footer,
            line_word_index, profile.hsize_words,
            line_word_index == 0U, line_end,
            line_end && final_footer_line, false, false,
            false, false, profile.cell_slot_count,
            profile.cell_word_count, empty_context));
    }
}

void emit_footer(hls::stream<h4::canonical_line_word_axis_t> &output,
                 const h3::face_close_result_t &close_result,
                 const decoded_profile_t &profile,
                 std::uint16_t completed_lines) {
    emit_footer_line(output, close_result, profile, completed_lines, 0U,
                     profile.footer_lines == 1U);
    if (profile.footer_lines == 2U) {
        emit_footer_line(
            output, close_result, profile, completed_lines,
            static_cast<std::uint8_t>(profile.hsize_words), true);
    }
}

}  // namespace

extern "C" void gpx_lane_word_formatter_hls(
    hls::stream<lidar_v3::h4::formatter_input_axis_t>
        &ordered_cell_or_face_close_in,
    hls::stream<lidar_v3::h4::canonical_line_word_axis_t>
        &canonical_line_word_out,
    hls::stream<lidar_v3::h4::formatter_control_axis_t>
        &formatter_control_out,
    lidar_v3::h4::lane_profile_t active_lane_profile) {
#pragma HLS INTERFACE axis port=ordered_cell_or_face_close_in
#pragma HLS INTERFACE axis port=canonical_line_word_out
#pragma HLS INTERFACE axis port=formatter_control_out
#pragma HLS INTERFACE ap_none port=active_lane_profile
#pragma HLS INTERFACE ap_ctrl_hs port=return

    namespace h1 = lidar_v3::h1;
    namespace h2 = lidar_v3::h2;
    namespace h3 = lidar_v3::h3;
    namespace h4 = lidar_v3::h4;

    const h4::formatter_input_axis_t input =
        ordered_cell_or_face_close_in.read();
    const std::uint8_t reset_epoch =
        lidar_v3::read_field<h4::formatter_input_layout::reset_epoch>(input)
            .to_uint();
    if (reset_epoch != accepted_reset_epoch) {
        clear_face_state();
        accepted_reset_epoch = reset_epoch;
    }

    const decoded_profile_t profile = decode_profile(active_lane_profile);
    std::uint8_t faults = 0U;
    std::uint32_t emitted_lines = 0U;
    bool footer_emitted = false;

    if (!profile_is_consistent(profile)) {
        set_fault(faults,
                  h4::formatter_fault_bit_t::invalid_active_lane_profile);
    }
    if (!profile.reserved_is_zero ||
        lidar_v3::read_field<h4::formatter_input_layout::reserved_zero>(
            input) != 0U) {
        set_fault(faults,
                  h4::formatter_fault_bit_t::reserved_input_nonzero);
    }

    if (h4::read_formatter_input_kind(input) ==
        h4::formatter_input_kind_t::ordered_lane_cell) {
        const h3::ordered_lane_cell_axis_t ordered_cell =
            h4::read_ordered_lane_cell(input);
        const h2::cell_event_record_t cell =
            lidar_v3::read_field<h3::ordered_lane_cell_layout::cell_event>(
                ordered_cell);
        const h1::shot_context_t context = read_cell_context(ordered_cell);
        const std::uint8_t slot_index =
            lidar_v3::read_field<
                h3::ordered_lane_cell_layout::lane_cell_slot_index>(
                ordered_cell)
                .to_uint();
        const std::uint8_t slot_count =
            lidar_v3::read_field<
                h3::ordered_lane_cell_layout::lane_cell_slot_count>(
                ordered_cell)
                .to_uint();
        const bool first_cell = lidar_v3::read_flag<
            h3::ordered_lane_cell_layout::is_first_cell_in_shot_line>(
            ordered_cell);
        const bool last_cell = lidar_v3::read_flag<
            h3::ordered_lane_cell_layout::is_last_cell_in_shot_line>(
            ordered_cell);
        const bool cell_slope_is_rise = lidar_v3::read_flag<
            h2::cell_event_layout::edge_slope_is_rise>(cell);
        const std::uint8_t serialized_return_slots =
            lidar_v3::read_field<
                h2::cell_event_layout::serialized_return_slot_count>(cell)
                .to_uint();
        const std::uint16_t gap_before =
            lidar_v3::read_field<
                h3::ordered_lane_cell_layout::missing_shot_columns_before>(
                ordered_cell)
                .to_uint();

        if (cell_slope_is_rise != profile.slope_is_rise) {
            set_fault(faults,
                      h4::formatter_fault_bit_t::edge_slope_mismatch);
        }
        if (slot_count != profile.cell_slot_count ||
            slot_index >= profile.cell_slot_count ||
            first_cell != (slot_index == 0U) ||
            last_cell != (slot_index + 1U == profile.cell_slot_count)) {
            set_fault(
                faults,
                h4::formatter_fault_bit_t::ordered_cell_geometry_mismatch);
        }
        if (serialized_return_slots != profile.serialized_return_slots) {
            set_fault(
                faults,
                h4::formatter_fault_bit_t::
                    serialized_return_contract_mismatch);
        }
        if (first_cell) {
            if (shot_line_is_open) {
                set_fault(
                    faults,
                    h4::formatter_fault_bit_t::
                        shot_context_or_sequence_mismatch);
            }
            const std::uint16_t current_column =
                read_context_shot_column(context);
            if (gap_before > current_column) {
                set_fault(
                    faults,
                    h4::formatter_fault_bit_t::
                        shot_context_or_sequence_mismatch);
            }
            const std::uint16_t safe_gap =
                gap_before > current_column ? current_column : gap_before;
            const std::uint16_t first_missing_column =
                static_cast<std::uint16_t>(current_column - safe_gap);
            for (std::uint16_t gap_offset = 0U;
                 gap_offset < safe_gap;
                 ++gap_offset) {
#pragma HLS LOOP_TRIPCOUNT min=0 max=65535
                const h1::shot_context_t missing_context =
                    make_missing_context_from_real(
                        context,
                        static_cast<std::uint16_t>(first_missing_column +
                                                   gap_offset),
                        profile.planned_shot_columns);
                emit_missing_shot_line(canonical_line_word_out,
                                       missing_context, profile, false);
                completed_shot_line_count =
                    static_cast<std::uint16_t>(
                        completed_shot_line_count + 1U);
                any_missing_shot_line = true;
                ++emitted_lines;
            }
            shot_line_is_open = true;
            open_shot_context = context;
            expected_next_cell_slot = 0U;
            open_shot_line_is_faulted = false;
        } else if (!shot_line_is_open ||
                   !contexts_equal(context, open_shot_context) ||
                   slot_index != expected_next_cell_slot) {
            set_fault(
                faults,
                h4::formatter_fault_bit_t::shot_context_or_sequence_mismatch);
        }

        open_shot_line_is_faulted =
            open_shot_line_is_faulted ||
            lidar_v3::read_flag<
                h3::ordered_lane_cell_layout::shot_line_is_faulted>(
                ordered_cell);
        emit_ordered_cell(canonical_line_word_out, ordered_cell, profile);
        expected_next_cell_slot =
            static_cast<std::uint8_t>(slot_index + 1U);

        if (last_cell) {
            shot_line_is_open = false;
            expected_next_cell_slot = 0U;
            completed_shot_line_count =
                static_cast<std::uint16_t>(completed_shot_line_count + 1U);
            any_shot_line_fault =
                any_shot_line_fault || open_shot_line_is_faulted;
            open_shot_line_is_faulted = false;
            ++emitted_lines;
        }
    } else {
        const h3::face_close_result_t close_result =
            h4::read_face_close_result(input);
        const h3::face_close_event_record_t close_event =
            lidar_v3::read_field<h3::face_close_result_layout::close_event>(
                close_result);
        const std::uint16_t close_columns =
            lidar_v3::read_field<
                h3::face_close_event_layout::expected_shot_column_count>(
                close_event)
                .to_uint();
        const std::uint16_t close_version =
            lidar_v3::read_field<
                h3::face_close_event_layout::active_configuration_version>(
                close_event)
                .to_uint();
        const std::uint16_t trailing_gap =
            lidar_v3::read_field<
                h3::face_close_result_layout::
                    trailing_missing_shot_columns>(close_result)
                .to_uint();
        const bool entire_face_missing = lidar_v3::read_flag<
            h3::face_close_result_layout::entire_face_is_missing>(
            close_result);
        const bool close_faulted = lidar_v3::read_flag<
            h3::face_close_result_layout::face_close_is_faulted>(close_result);

        if (shot_line_is_open || close_columns != profile.planned_shot_columns ||
            close_version != profile.active_version ||
            trailing_gap > close_columns ||
            (entire_face_missing && trailing_gap != close_columns) ||
            completed_shot_line_count !=
                static_cast<std::uint16_t>(close_columns - trailing_gap)) {
            set_fault(
                faults,
                h4::formatter_fault_bit_t::face_close_contract_mismatch);
        }

        const std::uint16_t first_trailing_column =
            trailing_gap <= close_columns
                ? static_cast<std::uint16_t>(close_columns - trailing_gap)
                : 0U;
        const std::uint16_t safe_trailing_gap =
            trailing_gap <= close_columns ? trailing_gap : close_columns;
        for (std::uint16_t gap_offset = 0U;
             gap_offset < safe_trailing_gap;
             ++gap_offset) {
#pragma HLS LOOP_TRIPCOUNT min=0 max=65535
            const h1::shot_context_t missing_context =
                make_missing_context_from_close(
                    close_result,
                    static_cast<std::uint16_t>(first_trailing_column +
                                               gap_offset),
                    profile.planned_shot_columns);
            emit_missing_shot_line(canonical_line_word_out, missing_context,
                                   profile, close_faulted);
            completed_shot_line_count = static_cast<std::uint16_t>(
                completed_shot_line_count + 1U);
            any_missing_shot_line = true;
            any_shot_line_fault = any_shot_line_fault || close_faulted;
            ++emitted_lines;
        }

        emit_footer(canonical_line_word_out, close_result, profile,
                    completed_shot_line_count);
        emitted_lines += profile.footer_lines;
        footer_emitted = true;
        clear_face_state();
    }

    h4::formatter_control_axis_t control = 0;
    lidar_v3::write_field<h4::formatter_control_layout::fault_event_bitmap>(
        control, faults);
    lidar_v3::write_flag<
        h4::formatter_control_layout::face_footer_was_emitted>(
        control, footer_emitted);
    lidar_v3::write_field<h4::formatter_control_layout::emitted_line_count>(
        control, emitted_lines);
    formatter_control_out.write(control);
}
