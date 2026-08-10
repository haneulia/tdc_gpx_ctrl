#include "gpx_lane_word_formatter_hls.hpp"

#include <cassert>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

namespace {

namespace h1 = lidar_v3::h1;
namespace h2 = lidar_v3::h2;
namespace h3 = lidar_v3::h3;
namespace h4 = lidar_v3::h4;

constexpr std::uint16_t kVersion = 0x1234U;

struct invocation_result_t {
    std::vector<h4::canonical_line_word_axis_t> words;
    h4::formatter_control_axis_t control;
};

h4::lane_profile_t make_profile(std::uint8_t cell_slots,
                                std::uint8_t return_slots,
                                std::uint16_t shot_columns,
                                std::uint8_t width_code,
                                bool slope_is_rise,
                                std::uint16_t version = kVersion) {
    const std::uint8_t cell_words =
        static_cast<std::uint8_t>(((return_slots + 1U) / 2U) + 1U);
    const std::uint16_t raw_words =
        static_cast<std::uint16_t>(4U + cell_slots * cell_words);
    const std::uint8_t beat_words = width_code == 0U ? 1U : 2U;
    const std::uint16_t hsize_words =
        static_cast<std::uint16_t>(
            ((raw_words + beat_words - 1U) / beat_words) * beat_words);
    const std::uint8_t footer_lines =
        static_cast<std::uint8_t>((8U + hsize_words - 1U) / hsize_words);

    h4::lane_profile_t profile = 0;
    lidar_v3::write_flag<h4::lane_profile_layout::profile_is_valid>(profile,
                                                                     true);
    lidar_v3::write_flag<h4::lane_profile_layout::lane_is_enabled>(profile,
                                                                    true);
    lidar_v3::write_flag<h4::lane_profile_layout::edge_slope_is_rise>(
        profile, slope_is_rise);
    lidar_v3::write_field<h4::lane_profile_layout::output_axis_width_code>(
        profile, width_code);
    lidar_v3::write_field<h4::lane_profile_layout::lane_cell_slot_count>(
        profile, cell_slots);
    lidar_v3::write_field<
        h4::lane_profile_layout::serialized_return_slot_count>(profile,
                                                                return_slots);
    lidar_v3::write_field<
        h4::lane_profile_layout::serialized_cell_word_count>(profile,
                                                              cell_words);
    lidar_v3::write_field<
        h4::lane_profile_layout::planned_shot_column_count>(profile,
                                                             shot_columns);
    lidar_v3::write_field<
        h4::lane_profile_layout::raw_shot_line_word_count>(profile,
                                                            raw_words);
    lidar_v3::write_field<
        h4::lane_profile_layout::aligned_hsize_word_count>(profile,
                                                            hsize_words);
    lidar_v3::write_field<h4::lane_profile_layout::hsize_bytes>(
        profile, hsize_words * 4U);
    lidar_v3::write_field<
        h4::lane_profile_layout::face_footer_line_count>(profile,
                                                          footer_lines);
    lidar_v3::write_field<h4::lane_profile_layout::vsize_line_count>(
        profile, shot_columns + footer_lines);
    lidar_v3::write_field<
        h4::lane_profile_layout::active_configuration_version>(profile,
                                                                version);
    return profile;
}

h1::shot_context_t make_context(std::uint16_t shot_column,
                                std::uint16_t shot_columns,
                                std::uint16_t version = kVersion,
                                bool direction_is_ccw = false,
                                bool source_is_simulation = false) {
    h1::shot_context_t context = 0;
    lidar_v3::write_flag<h1::shot_context_layout::request_valid>(context,
                                                                  true);
    lidar_v3::write_field<h1::shot_context_layout::mirror_face_index>(context,
                                                                      2U);
    lidar_v3::write_field<h1::shot_context_layout::encoder_position_state>(
        context, 0x321U + shot_column);
    lidar_v3::write_flag<h1::shot_context_layout::direction_is_ccw>(
        context, direction_is_ccw);
    lidar_v3::write_field<h1::shot_context_layout::shot_column_index>(
        context, shot_column);
    lidar_v3::write_flag<
        h1::shot_context_layout::is_last_shot_column_in_face>(
        context, shot_column + 1U == shot_columns);
    lidar_v3::write_flag<h1::shot_context_layout::source_is_simulation>(
        context, source_is_simulation);
    lidar_v3::write_field<
        h1::shot_context_layout::encoder_to_scheduler_latency_clks>(context,
                                                                    5U);
    lidar_v3::write_flag<h1::shot_context_layout::encoder_latency_is_valid>(
        context, true);
    lidar_v3::write_field<
        h1::shot_context_layout::active_configuration_version>(context,
                                                                version);
    lidar_v3::write_field<
        h1::shot_context_layout::fire_command_to_t0_latency_clks>(context,
                                                                  19U);
    const std::uint64_t timestamp =
        0x1122334400000000ULL + shot_column;
    lidar_v3::write_field<
        h1::shot_context_layout::measurement_start_t0_timestamp_ticks>(
        context, timestamp);
    lidar_v3::write_flag<
        h1::shot_context_layout::measurement_start_timestamp_is_valid>(
        context, true);
    lidar_v3::write_flag<
        h1::shot_context_layout::measurement_start_time_sync_is_valid>(
        context, true);
    lidar_v3::write_flag<h1::shot_context_layout::context_is_valid>(context,
                                                                    true);
    return context;
}

h3::ordered_lane_cell_axis_t make_ordered_cell(
    std::uint8_t slot_index,
    std::uint8_t slot_count,
    std::uint8_t return_slots,
    std::uint8_t valid_returns,
    std::uint16_t shot_column,
    std::uint16_t shot_columns,
    bool slope_is_rise,
    std::uint16_t gap_before = 0U,
    bool blank = false,
    bool line_faulted = false) {
    h2::cell_event_record_t cell = 0;
    lidar_v3::write_field<h2::cell_event_layout::event_kind>(cell, 0U);
    lidar_v3::write_field<h2::cell_event_layout::tdc_chip_index>(
        cell, slot_index / 8U);
    lidar_v3::write_flag<h2::cell_event_layout::ififo_bank_select>(
        cell, (slot_index % 8U) >= 4U);
    lidar_v3::write_field<
        h2::cell_event_layout::logical_stop_channel_index>(cell,
                                                            slot_index % 8U);
    lidar_v3::write_flag<h2::cell_event_layout::edge_slope_is_rise>(
        cell, slope_is_rise);
    lidar_v3::write_field<h2::cell_event_layout::visible_return_count>(
        cell, valid_returns);
    lidar_v3::write_field<
        h2::cell_event_layout::serialized_return_slot_count>(cell,
                                                              return_slots);
    for (std::uint8_t hit = 0U; hit < 7U; ++hit) {
        const unsigned low =
            h2::cell_event_layout::packed_distance_hits_17bit::low +
            hit * 17U;
        const std::uint32_t value =
            0x10000U | (shot_column << 8U) | (slot_index << 4U) | hit;
        cell.range(low + 16U, low) = value;
    }
    lidar_v3::write_flag<h2::cell_event_layout::error_fill_inserted>(
        cell, blank);
    lidar_v3::write_flag<h2::cell_event_layout::cell_is_faulted>(
        cell, line_faulted || blank);
    lidar_v3::write_field<h2::cell_event_layout::timeout_cause_bitmap>(
        cell, line_faulted ? 3U : 0U);
    lidar_v3::write_field<h2::cell_event_layout::shot_context>(
        cell, make_context(shot_column, shot_columns));
    lidar_v3::write_field<h2::cell_event_layout::tdc_chip_shot_sequence>(
        cell, 0x40U + shot_column);

    h3::ordered_lane_cell_axis_t ordered = 0;
    lidar_v3::write_field<h3::ordered_lane_cell_layout::cell_event>(ordered,
                                                                    cell);
    lidar_v3::write_field<
        h3::ordered_lane_cell_layout::lane_cell_slot_index>(ordered,
                                                             slot_index);
    lidar_v3::write_field<
        h3::ordered_lane_cell_layout::lane_cell_slot_count>(ordered,
                                                             slot_count);
    lidar_v3::write_flag<
        h3::ordered_lane_cell_layout::is_first_cell_in_shot_line>(
        ordered, slot_index == 0U);
    lidar_v3::write_flag<
        h3::ordered_lane_cell_layout::is_last_cell_in_shot_line>(
        ordered, slot_index + 1U == slot_count);
    lidar_v3::write_flag<
        h3::ordered_lane_cell_layout::is_first_shot_column_in_face>(
        ordered, shot_column == 0U);
    lidar_v3::write_flag<
        h3::ordered_lane_cell_layout::is_last_shot_column_in_face>(
        ordered, shot_column + 1U == shot_columns);
    lidar_v3::write_field<
        h3::ordered_lane_cell_layout::missing_shot_columns_before>(
        ordered, slot_index == 0U ? gap_before : 0U);
    lidar_v3::write_flag<
        h3::ordered_lane_cell_layout::is_missing_cell_placeholder>(ordered,
                                                                    blank);
    lidar_v3::write_flag<
        h3::ordered_lane_cell_layout::shot_line_is_faulted>(ordered,
                                                             line_faulted);
    return ordered;
}

h3::face_close_result_t make_close(std::uint16_t shot_columns,
                                   std::uint16_t trailing_gap,
                                   bool all_hole,
                                   bool faulted = false) {
    h3::face_close_event_record_t close = 0;
    lidar_v3::write_field<h3::face_close_event_layout::frame_identifier>(
        close, 0xABC00000UL + shot_columns);
    lidar_v3::write_field<h3::face_close_event_layout::mirror_face_index>(
        close, 2U);
    lidar_v3::write_field<
        h3::face_close_event_layout::active_configuration_version>(close,
                                                                   kVersion);
    lidar_v3::write_field<
        h3::face_close_event_layout::expected_shot_column_count>(close,
                                                                  shot_columns);

    h3::face_close_result_t result = 0;
    lidar_v3::write_field<h3::face_close_result_layout::close_event>(result,
                                                                     close);
    lidar_v3::write_field<
        h3::face_close_result_layout::trailing_missing_shot_columns>(
        result, trailing_gap);
    lidar_v3::write_flag<
        h3::face_close_result_layout::entire_face_is_missing>(result,
                                                               all_hole);
    lidar_v3::write_flag<
        h3::face_close_result_layout::face_close_is_faulted>(result,
                                                              faulted);
    return result;
}

h4::formatter_input_axis_t cell_input(
    const h3::ordered_lane_cell_axis_t &cell,
    std::uint8_t reset_epoch) {
    h4::formatter_input_axis_t input = 0;
    lidar_v3::write_field<h4::formatter_input_layout::ordered_lane_cell>(
        input, cell);
    lidar_v3::write_field<h4::formatter_input_layout::reset_epoch>(input,
                                                                   reset_epoch);
    return input;
}

h4::formatter_input_axis_t close_input(
    const h3::face_close_result_t &close,
    std::uint8_t reset_epoch) {
    h4::formatter_input_axis_t input = 0;
    lidar_v3::write_field<h4::formatter_input_layout::face_close_result>(
        input, close);
    lidar_v3::write_flag<h4::formatter_input_layout::event_kind>(input, true);
    lidar_v3::write_field<h4::formatter_input_layout::reset_epoch>(input,
                                                                   reset_epoch);
    return input;
}

invocation_result_t invoke(const h4::formatter_input_axis_t &input,
                           const h4::lane_profile_t &profile) {
    hls::stream<h4::formatter_input_axis_t> input_stream;
    hls::stream<h4::canonical_line_word_axis_t> word_stream;
    hls::stream<h4::formatter_control_axis_t> control_stream;
    input_stream.write(input);
    gpx_lane_word_formatter_hls(input_stream, word_stream, control_stream,
                                profile);

    invocation_result_t result;
    while (!word_stream.empty()) {
        result.words.push_back(word_stream.read());
    }
    assert(control_stream.size() == 1U);
    result.control = control_stream.read();
    return result;
}

std::uint32_t data(const h4::canonical_line_word_axis_t &word) {
    return lidar_v3::read_field<
               h4::canonical_line_word_layout::canonical_word_32bit>(word)
        .to_uint();
}

std::uint8_t kind(const h4::canonical_line_word_axis_t &word) {
    return lidar_v3::read_field<h4::canonical_line_word_layout::word_kind>(
               word)
        .to_uint();
}

void expect_clean_control(const invocation_result_t &result,
                          std::uint32_t lines,
                          bool footer) {
    assert(lidar_v3::read_field<
               h4::formatter_control_layout::fault_event_bitmap>(
               result.control) == 0U);
    assert(lidar_v3::read_flag<
               h4::formatter_control_layout::face_footer_was_emitted>(
               result.control) == footer);
    assert(lidar_v3::read_field<
               h4::formatter_control_layout::emitted_line_count>(
               result.control) == lines);
    assert(lidar_v3::read_field<
               h4::formatter_control_layout::reserved_zero>(result.control) ==
           0U);
}

void expect_only_fault(const invocation_result_t &result,
                       h4::formatter_fault_bit_t expected_fault) {
    const std::uint8_t expected_bitmap = static_cast<std::uint8_t>(
        1U << static_cast<unsigned>(expected_fault));
    const std::uint8_t actual_bitmap =
        lidar_v3::read_field<
            h4::formatter_control_layout::fault_event_bitmap>(result.control)
            .to_uint();
    assert(actual_bitmap == expected_bitmap);
    assert(lidar_v3::read_field<
               h4::formatter_control_layout::reserved_zero>(result.control) ==
           0U);
}

void test_return_sweep() {
    std::uint8_t epoch = 1U;
    for (std::uint8_t returns = 1U; returns <= 7U; ++returns, ++epoch) {
        const h4::lane_profile_t profile =
            make_profile(1U, returns, 1U, 0U, true);
        const invocation_result_t shot = invoke(
            cell_input(make_ordered_cell(0U, 1U, returns, returns, 0U, 1U,
                                         true),
                       epoch),
            profile);
        const std::uint8_t cell_words =
            static_cast<std::uint8_t>(((returns + 1U) / 2U) + 1U);
        assert(shot.words.size() == 4U + cell_words);
        expect_clean_control(shot, 1U, false);
        assert(kind(shot.words[0]) == 0U);
        assert(data(shot.words[0]) == 0U);
        assert(data(shot.words[1]) == 0x11223344U);
        assert((data(shot.words[3]) & 0x7FFU) == 0x781U);
        const std::uint32_t metadata = shot.words.back().range(31, 0).to_uint();
        assert((metadata >> 31U) == 1U);
        assert(((metadata >> 14U) & 0x7U) == returns);
        assert(((metadata >> 7U) & 0x7FU) ==
               ((1U << returns) - 1U));
        assert((metadata & 0x7FU) == ((1U << returns) - 1U));
        assert(lidar_v3::read_flag<
            h4::canonical_line_word_layout::is_line_end>(shot.words.back()));

        const invocation_result_t footer =
            invoke(close_input(make_close(1U, 0U, false), epoch), profile);
        expect_clean_control(
            footer,
            lidar_v3::read_field<
                h4::lane_profile_layout::face_footer_line_count>(profile)
                .to_uint(),
            true);
        assert(data(footer.words.front()) == 0x47504631U);
        bool commit_seen = false;
        for (const auto &word : footer.words) {
            commit_seen = commit_seen || data(word) == 0x434F4D54U;
        }
        assert(commit_seen);
        assert(lidar_v3::read_flag<
            h4::canonical_line_word_layout::is_frame_end>(
            footer.words.back()));
    }
}

void test_multi_cell_line() {
    const std::uint8_t epoch = 20U;
    const h4::lane_profile_t profile =
        make_profile(2U, 3U, 1U, 1U, false);
    const invocation_result_t first = invoke(
        cell_input(make_ordered_cell(0U, 2U, 3U, 2U, 0U, 1U, false),
                   epoch),
        profile);
    assert(first.words.size() == 7U);
    expect_clean_control(first, 0U, false);
    assert(!lidar_v3::read_flag<
        h4::canonical_line_word_layout::is_line_end>(first.words.back()));

    const invocation_result_t second = invoke(
        cell_input(make_ordered_cell(1U, 2U, 3U, 1U, 0U, 1U, false),
                   epoch),
        profile);
    assert(second.words.size() == 3U);
    expect_clean_control(second, 1U, false);
    assert(lidar_v3::read_field<
               h4::canonical_line_word_layout::word_index_within_line>(
               second.words.front()) == 7U);
    assert(lidar_v3::read_flag<
        h4::canonical_line_word_layout::is_line_end>(second.words.back()));

    const invocation_result_t footer =
        invoke(close_input(make_close(1U, 0U, false), epoch), profile);
    expect_clean_control(footer, 1U, true);
    assert(footer.words.size() == 10U);
}

void test_holes_and_footer_32() {
    const std::uint8_t epoch = 30U;
    const h4::lane_profile_t profile =
        make_profile(1U, 1U, 4U, 0U, true);
    const invocation_result_t shot = invoke(
        cell_input(make_ordered_cell(0U, 1U, 1U, 1U, 2U, 4U, true, 2U),
                   epoch),
        profile);
    expect_clean_control(shot, 3U, false);
    assert(shot.words.size() == 18U);
    for (unsigned line = 0U; line < 2U; ++line) {
        const unsigned base = line * 6U;
        assert(lidar_v3::read_flag<
            h4::canonical_line_word_layout::line_represents_missing_shot>(
            shot.words[base]));
        assert((data(shot.words[base + 2U]) >> 16U) == 0xFFFFU);
        assert((data(shot.words[base + 3U]) & 0x3U) == 0x2U);
    }
    assert(!lidar_v3::read_flag<
        h4::canonical_line_word_layout::line_represents_missing_shot>(
        shot.words[12U]));

    const invocation_result_t close =
        invoke(close_input(make_close(4U, 1U, false), epoch), profile);
    expect_clean_control(close, 3U, true);
    assert(close.words.size() == 18U);
    assert(lidar_v3::read_flag<
        h4::canonical_line_word_layout::line_represents_missing_shot>(
        close.words.front()));
    assert(data(close.words[6U]) == 0x47504631U);
    assert(data(close.words[13U]) == 0x434F4D54U);
}

void test_all_hole_footer_64() {
    const std::uint8_t epoch = 40U;
    const h4::lane_profile_t profile =
        make_profile(2U, 1U, 3U, 1U, false);
    const invocation_result_t result =
        invoke(close_input(make_close(3U, 3U, true), epoch), profile);
    expect_clean_control(result, 4U, true);
    assert(result.words.size() == 32U);
    for (unsigned line = 0U; line < 3U; ++line) {
        assert(lidar_v3::read_flag<
            h4::canonical_line_word_layout::line_represents_missing_shot>(
            result.words[line * 8U]));
    }
    assert(data(result.words[24U]) == 0x47504631U);
    assert(((data(result.words[26U]) >> 6U) & 0x3U) == 1U);
    const std::uint32_t summary = data(result.words[30U]);
    assert((summary & 0xFFFFU) == 3U);
    assert(((summary >> 19U) & 1U) == 1U);
    assert(((summary >> 22U) & 1U) == 1U);
}

void test_reset_epoch_and_faults() {
    const h4::lane_profile_t profile =
        make_profile(2U, 2U, 1U, 0U, true);
    const invocation_result_t partial = invoke(
        cell_input(make_ordered_cell(0U, 2U, 2U, 1U, 0U, 1U, true), 50U),
        profile);
    expect_clean_control(partial, 0U, false);

    // A new Reset epoch atomically discards the open partial Shot. A complete
    // replacement Face must therefore close without a sequence mismatch.
    const invocation_result_t first = invoke(
        cell_input(make_ordered_cell(0U, 2U, 2U, 0U, 0U, 1U, true, 0U,
                                     true, true),
                   51U),
        profile);
    expect_clean_control(first, 0U, false);
    const invocation_result_t second = invoke(
        cell_input(make_ordered_cell(1U, 2U, 2U, 1U, 0U, 1U, true, 0U,
                                     false, true),
                   51U),
        profile);
    expect_clean_control(second, 1U, false);
    assert(((data(second.words.back()) >> 27U) & 1U) == 1U);
    const invocation_result_t close =
        invoke(close_input(make_close(1U, 0U, false), 51U), profile);
    expect_clean_control(close, 1U, true);
    const std::uint32_t summary = data(close.words[6U]);
    assert(((summary >> 18U) & 1U) == 1U);
    // Cell timeout_cause is a TDC-GPX Drain diagnostic. It must not be
    // re-labelled as the dormant V2 per-Shot timeout/abort Footer fields.
    assert(((summary >> 20U) & 0x3U) == 0U);

    // Each formatter contract fault is injected in an independent Reset
    // epoch. Exact bitmap comparison prevents a newly introduced fault from
    // being hidden behind the expected one.
    h4::lane_profile_t invalid_profile =
        make_profile(1U, 1U, 1U, 0U, true);
    lidar_v3::write_flag<h4::lane_profile_layout::profile_is_valid>(
        invalid_profile, false);
    expect_only_fault(
        invoke(cell_input(make_ordered_cell(0U, 1U, 1U, 1U, 0U, 1U, true),
                          60U),
               invalid_profile),
        h4::formatter_fault_bit_t::invalid_active_lane_profile);

    const h4::lane_profile_t one_cell_profile =
        make_profile(1U, 1U, 1U, 0U, true);
    expect_only_fault(
        invoke(cell_input(make_ordered_cell(0U, 1U, 1U, 1U, 0U, 1U, false),
                          61U),
               one_cell_profile),
        h4::formatter_fault_bit_t::edge_slope_mismatch);
    expect_only_fault(
        invoke(cell_input(make_ordered_cell(0U, 2U, 1U, 1U, 0U, 1U, true),
                          62U),
               one_cell_profile),
        h4::formatter_fault_bit_t::ordered_cell_geometry_mismatch);

    const h4::lane_profile_t two_cell_profile =
        make_profile(2U, 1U, 1U, 0U, true);
    expect_only_fault(
        invoke(cell_input(make_ordered_cell(1U, 2U, 1U, 1U, 0U, 1U, true),
                          63U),
               two_cell_profile),
        h4::formatter_fault_bit_t::shot_context_or_sequence_mismatch);

    const h4::lane_profile_t two_return_profile =
        make_profile(1U, 2U, 1U, 0U, true);
    expect_only_fault(
        invoke(cell_input(make_ordered_cell(0U, 1U, 3U, 1U, 0U, 1U, true),
                          64U),
               two_return_profile),
        h4::formatter_fault_bit_t::serialized_return_contract_mismatch);
    expect_only_fault(
        invoke(close_input(make_close(2U, 2U, true), 65U),
               one_cell_profile),
        h4::formatter_fault_bit_t::face_close_contract_mismatch);

    h4::formatter_input_axis_t reserved_input =
        cell_input(make_ordered_cell(0U, 1U, 1U, 1U, 0U, 1U, true), 66U);
    lidar_v3::write_field<h4::formatter_input_layout::reserved_zero>(
        reserved_input, 1U);
    expect_only_fault(
        invoke(reserved_input, one_cell_profile),
        h4::formatter_fault_bit_t::reserved_input_nonzero);

}

}  // namespace

int main() {
    const char *profile_env = std::getenv("V3_HLS_PROFILE");
    const std::string profile =
        profile_env == nullptr ? "all" : profile_env;

    if (profile == "return_sweep" || profile == "all") {
        test_return_sweep();
    }
    if (profile == "multi_cell" || profile == "all") {
        test_multi_cell_line();
    }
    if (profile == "holes_footer_32" || profile == "all") {
        test_holes_and_footer_32();
    }
    if (profile == "all_hole_footer_64" || profile == "all") {
        test_all_hole_footer_64();
    }
    if (profile == "reset_faults" || profile == "all") {
        test_reset_epoch_and_faults();
    }

    std::cout << "LIDAR_V3_HLS_GPX_LANE_WORD_FORMATTER_CSIM_PASS profile="
              << profile << '\n';
    return 0;
}
