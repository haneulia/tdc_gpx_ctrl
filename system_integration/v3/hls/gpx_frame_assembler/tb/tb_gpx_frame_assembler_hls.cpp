#include <cassert>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include "gpx_frame_assembler_hls.hpp"

// 검증 목적:
//   H2 Cell을 Shot별로 수집한 뒤 Rise/Fall Lane을 논리 Chip/STOP 오름차순으로
//   출력하는지 검증한다. 누락 Cell, 중복, Mask 위반, column gap, Face close와
//   Reset Epoch 복구를 HLS C 모델 경계에서 독립적으로 확인한다.

namespace {

namespace h1 = lidar_v3::h1;
namespace h2 = lidar_v3::h2;
namespace h3 = lidar_v3::h3;
namespace limits = lidar_v3::limits;

using slope_t = lidar_v3::tdc_edge_slope_t;
using cell_kind_t = h2::cell_event_kind_t;
using cell_payload_t = h2::cell_event_record_t;
using frame_input_payload_t = h3::assembler_input_axis_t;
using frame_cell_payload_t = h3::ordered_lane_cell_axis_t;
using frame_control_payload_t = h3::assembler_control_axis_t;

struct config_t {
    std::uint8_t num_chips;
    std::uint8_t stops_per_chip;
    std::uint8_t num_faces;
    std::uint16_t active_version;
    std::uint8_t rise_mask;
    std::uint8_t fall_mask;
    std::uint16_t columns;
    std::uint8_t reset_epoch;
};

struct invocation_t {
    std::vector<frame_cell_payload_t> rise;
    std::vector<frame_cell_payload_t> fall;
    frame_control_payload_t control;
};

constexpr std::uint8_t fault_mask(h3::assembler_fault_bit_t fault) {
    return static_cast<std::uint8_t>(
        1U << static_cast<unsigned>(fault));
}

h1::shot_context_t
make_context(std::uint16_t shot_index, std::uint16_t columns,
             std::uint16_t active_version, std::uint8_t face = 1U,
             bool direction_ccw = false, bool source_sim = false) {
    h1::shot_context_t context = 0;
    lidar_v3::write_flag<h1::shot_context_layout::request_valid>(
        context, true);
    lidar_v3::write_field<h1::shot_context_layout::mirror_face_index>(
        context, face);
    lidar_v3::write_field<h1::shot_context_layout::encoder_position_state>(
        context, 100U + shot_index);
    lidar_v3::write_flag<h1::shot_context_layout::direction_is_ccw>(
        context, direction_ccw);
    lidar_v3::write_field<h1::shot_context_layout::shot_column_index>(
        context, shot_index);
    lidar_v3::write_flag<
        h1::shot_context_layout::is_last_shot_column_in_face>(
        context, shot_index + 1U >= columns);
    lidar_v3::write_flag<h1::shot_context_layout::source_is_simulation>(
        context, source_sim);
    lidar_v3::write_field<
        h1::shot_context_layout::encoder_to_scheduler_latency_clks>(
        context, 3U);
    lidar_v3::write_flag<h1::shot_context_layout::encoder_latency_is_valid>(
        context, true);
    lidar_v3::write_field<
        h1::shot_context_layout::active_configuration_version>(
        context, active_version);
    lidar_v3::write_field<
        h1::shot_context_layout::fire_command_to_t0_latency_clks>(
        context, 7U);
    lidar_v3::write_flag<h1::shot_context_layout::context_is_valid>(
        context, true);
    return context;
}

std::uint32_t expected_hit(std::uint16_t shot, std::uint8_t chip,
                           std::uint8_t stop, std::uint8_t slope) {
    return (static_cast<std::uint32_t>(shot) * 10000U +
            static_cast<std::uint32_t>(chip) * 1000U +
            (slope == static_cast<std::uint8_t>(slope_t::fall) ? 100U : 0U) +
            stop + 1U) &
           0x1FFFFU;
}

cell_payload_t make_cell(cell_kind_t kind, std::uint8_t chip, std::uint8_t stop,
                         std::uint8_t slope, std::uint16_t shot,
                         std::uint16_t columns, std::uint16_t version,
                         std::uint8_t hit_count = 1U, bool faulted = false,
                         std::uint8_t face = 1U) {
    cell_payload_t cell = 0;
    lidar_v3::write_field<h2::cell_event_layout::event_kind>(
        cell, static_cast<std::uint8_t>(kind));
    lidar_v3::write_field<h2::cell_event_layout::tdc_chip_index>(cell, chip);
    lidar_v3::write_flag<h2::cell_event_layout::ififo_bank_select>(
        cell, stop >= 4U);
    lidar_v3::write_field<h2::cell_event_layout::logical_stop_channel_index>(
        cell, stop);
    lidar_v3::write_flag<h2::cell_event_layout::edge_slope_is_rise>(
        cell, slope == static_cast<std::uint8_t>(slope_t::rise));
    lidar_v3::write_field<h2::cell_event_layout::visible_return_count>(
        cell, hit_count);
    lidar_v3::write_field<h2::cell_event_layout::serialized_return_slot_count>(
        cell, 2U);
    if (hit_count != 0U) {
        const unsigned first_hit_low_bit =
            h2::cell_event_layout::packed_distance_hits_17bit::low;
        cell.range(first_hit_low_bit + 16U, first_hit_low_bit) =
            expected_hit(shot, chip, stop, slope);
    }
    lidar_v3::write_flag<h2::cell_event_layout::cell_is_faulted>(
        cell, faulted);
    lidar_v3::write_field<h2::cell_event_layout::shot_context>(
        cell, make_context(shot, columns, version, face));
    lidar_v3::write_field<h2::cell_event_layout::tdc_chip_shot_sequence>(
        cell, 10U + shot);
    return cell;
}

frame_input_payload_t cell_input(const cell_payload_t &cell,
                                 std::uint8_t reset_epoch) {
    frame_input_payload_t input = 0;
    lidar_v3::write_field<h3::assembler_input_layout::event_body>(
        input, cell);
    lidar_v3::write_flag<h3::assembler_input_layout::event_kind>(input, false);
    lidar_v3::write_field<h3::assembler_input_layout::reset_epoch>(
        input, reset_epoch);
    return input;
}

frame_input_payload_t face_close_input(std::uint32_t frame_id,
                                       std::uint8_t face, bool direction_ccw,
                                       bool source_sim, std::uint16_t version,
                                       std::uint16_t columns,
                                       std::uint8_t reset_epoch) {
    frame_input_payload_t input = 0;
    h3::face_close_event_record_t close_event = 0;
    lidar_v3::write_field<h3::face_close_event_layout::frame_identifier>(
        close_event, frame_id);
    lidar_v3::write_field<h3::face_close_event_layout::mirror_face_index>(
        close_event, face);
    lidar_v3::write_flag<h3::face_close_event_layout::direction_is_ccw>(
        close_event, direction_ccw);
    lidar_v3::write_flag<h3::face_close_event_layout::source_is_simulation>(
        close_event, source_sim);
    lidar_v3::write_field<
        h3::face_close_event_layout::active_configuration_version>(
        close_event, version);
    lidar_v3::write_field<
        h3::face_close_event_layout::expected_shot_column_count>(
        close_event, columns);
    lidar_v3::write_field<h3::assembler_input_layout::face_close_event>(
        input, close_event);
    lidar_v3::write_flag<h3::assembler_input_layout::event_kind>(input, true);
    lidar_v3::write_field<h3::assembler_input_layout::reset_epoch>(
        input, reset_epoch);
    return input;
}

invocation_t invoke(const frame_input_payload_t &input,
                    const config_t &config) {
    hls::stream<frame_input_payload_t> input_stream;
    hls::stream<frame_cell_payload_t> rise_stream;
    hls::stream<frame_cell_payload_t> fall_stream;
    hls::stream<frame_control_payload_t> control_stream;
    input_stream.write(input);
    gpx_frame_assembler_hls(input_stream, rise_stream, fall_stream,
                            control_stream, config.num_chips,
                            config.stops_per_chip, config.num_faces,
                            config.active_version, config.rise_mask,
                            config.fall_mask, config.columns);

    invocation_t result;
    while (!rise_stream.empty()) {
        result.rise.push_back(rise_stream.read());
    }
    while (!fall_stream.empty()) {
        result.fall.push_back(fall_stream.read());
    }
    assert(control_stream.size() == 1U);
    result.control = control_stream.read();
    return result;
}

void expect_faults(const frame_control_payload_t &control,
                   std::uint8_t expected) {
    assert(lidar_v3::read_field<
               h3::assembler_control_layout::fault_event_bitmap>(control) ==
           expected);
    assert(lidar_v3::read_field<h3::assembler_control_layout::reserved_zero>(
               control) == 0U);
}

std::uint8_t popcount4(std::uint8_t mask) {
    std::uint8_t result = 0;
    for (unsigned bit = 0; bit < 4U; ++bit) {
        result += (mask >> bit) & 1U;
    }
    return result;
}

std::uint8_t chip_for_slot(std::uint8_t mask, std::uint8_t stops,
                           std::uint8_t slot) {
    std::uint8_t rank = slot;
    for (std::uint8_t chip = 0; chip < 4U; ++chip) {
        if (((mask >> chip) & 1U) != 0U) {
            if (rank < stops) {
                return chip;
            }
            rank = static_cast<std::uint8_t>(rank - stops);
        }
    }
    return 0U;
}

void expect_lane(const std::vector<frame_cell_payload_t> &lane,
                 std::uint8_t mask, const config_t &config, std::uint16_t shot,
                 std::uint8_t slope, int missing_chip = -1,
                 int missing_stop = -1, bool line_faulted = false,
                 std::uint16_t gap = 0U) {
    const std::uint8_t count =
        static_cast<std::uint8_t>(popcount4(mask) * config.stops_per_chip);
    assert(lane.size() == count);
    for (std::uint8_t slot = 0; slot < count; ++slot) {
        const frame_cell_payload_t frame = lane[slot];
        const cell_payload_t cell =
            lidar_v3::read_field<h3::ordered_lane_cell_layout::cell_event>(
                frame);
        const std::uint8_t chip =
            chip_for_slot(mask, config.stops_per_chip, slot);
        const std::uint8_t stop = slot % config.stops_per_chip;
        const bool blank = chip == missing_chip && stop == missing_stop;

        assert(lidar_v3::read_field<
                   h3::ordered_lane_cell_layout::lane_cell_slot_index>(frame) ==
               slot);
        assert(lidar_v3::read_field<
                   h3::ordered_lane_cell_layout::lane_cell_slot_count>(frame) ==
               count);
        assert(lidar_v3::read_flag<
                   h3::ordered_lane_cell_layout::is_first_cell_in_shot_line>(
                   frame) == (slot == 0U));
        assert(lidar_v3::read_flag<
                   h3::ordered_lane_cell_layout::is_last_cell_in_shot_line>(
                   frame) == (slot + 1U == count));
        assert(lidar_v3::read_flag<
                   h3::ordered_lane_cell_layout::is_first_shot_column_in_face>(
                   frame) == (shot == 0U));
        assert(lidar_v3::read_flag<
                   h3::ordered_lane_cell_layout::is_last_shot_column_in_face>(
                   frame) == (shot + 1U >= config.columns));
        assert(lidar_v3::read_field<
                   h3::ordered_lane_cell_layout::missing_shot_columns_before>(
                   frame) == (slot == 0U ? gap : 0U));
        assert(lidar_v3::read_flag<
                   h3::ordered_lane_cell_layout::is_missing_cell_placeholder>(
                   frame) == blank);
        assert(lidar_v3::read_flag<
                   h3::ordered_lane_cell_layout::shot_line_is_faulted>(frame) ==
               line_faulted);
        assert(lidar_v3::read_field<
                   h3::ordered_lane_cell_layout::reserved_zero>(frame) == 0U);

        assert(lidar_v3::read_field<h2::cell_event_layout::event_kind>(cell) ==
               static_cast<std::uint8_t>(cell_kind_t::data));
        assert(lidar_v3::read_field<h2::cell_event_layout::tdc_chip_index>(
                   cell) == chip);
        assert(lidar_v3::read_field<
                   h2::cell_event_layout::logical_stop_channel_index>(cell) ==
               stop);
        assert(lidar_v3::read_flag<
                   h2::cell_event_layout::edge_slope_is_rise>(cell) ==
               (slope == static_cast<std::uint8_t>(slope_t::rise)));
        assert(lidar_v3::read_field<h2::cell_event_layout::shot_context>(cell) ==
               make_context(shot, config.columns, config.active_version));
        if (blank) {
            assert(lidar_v3::read_field<
                       h2::cell_event_layout::visible_return_count>(cell) ==
                   0U);
            assert(lidar_v3::read_flag<
                h2::cell_event_layout::error_fill_inserted>(cell));
            assert(lidar_v3::read_flag<
                h2::cell_event_layout::cell_is_faulted>(cell));
            assert(lidar_v3::read_field<
                       h2::cell_event_layout::tdc_chip_shot_sequence>(cell) ==
                   0U);
        } else {
            assert(lidar_v3::read_field<
                       h2::cell_event_layout::visible_return_count>(cell) ==
                   1U);
            const unsigned first_hit_low_bit =
                h2::cell_event_layout::packed_distance_hits_17bit::low;
            assert(cell.range(first_hit_low_bit + 16U, first_hit_low_bit) ==
                   expected_hit(shot, chip, stop, slope));
            assert(lidar_v3::read_field<
                       h2::cell_event_layout::tdc_chip_shot_sequence>(cell) ==
                   10U + shot);
        }
    }
}

invocation_t send_shot(config_t &config, std::uint16_t shot,
                       bool inject_faults = false,
                       std::uint8_t first_event_faults = 0U) {
    invocation_t result;
    const std::uint8_t expected_mask =
        static_cast<std::uint8_t>(config.rise_mask | config.fall_mask);
    bool first_event = true;

    for (int chip = static_cast<int>(config.num_chips) - 1; chip >= 0; --chip) {
        if (((config.rise_mask >> chip) & 1U) != 0U) {
            for (int stop = static_cast<int>(config.stops_per_chip) - 1;
                 stop >= 0; --stop) {
                if (inject_faults && chip == 1 &&
                    stop == static_cast<int>(config.stops_per_chip) - 1) {
                    continue;
                }
                result = invoke(
                    cell_input(
                        make_cell(cell_kind_t::data,
                                  static_cast<std::uint8_t>(chip),
                                  static_cast<std::uint8_t>(stop),
                                  static_cast<std::uint8_t>(slope_t::rise),
                                  shot, config.columns, config.active_version),
                        config.reset_epoch),
                    config);
                expect_faults(result.control,
                              first_event ? first_event_faults : 0U);
                first_event = false;
            }
        }
        if (((config.fall_mask >> chip) & 1U) != 0U) {
            for (int stop = static_cast<int>(config.stops_per_chip) - 1;
                 stop >= 0; --stop) {
                result = invoke(
                    cell_input(
                        make_cell(cell_kind_t::data,
                                  static_cast<std::uint8_t>(chip),
                                  static_cast<std::uint8_t>(stop),
                                  static_cast<std::uint8_t>(slope_t::fall),
                                  shot, config.columns, config.active_version),
                        config.reset_epoch),
                    config);
                expect_faults(result.control,
                              first_event ? first_event_faults : 0U);
                first_event = false;
            }
        }
    }

    std::uint8_t accumulated_faults = 0U;
    if (inject_faults) {
        result = invoke(
            cell_input(make_cell(cell_kind_t::data, 0U, 0U,
                                 static_cast<std::uint8_t>(slope_t::rise), shot,
                                 config.columns, config.active_version),
                       config.reset_epoch),
            config);
        accumulated_faults |=
            fault_mask(h3::assembler_fault_bit_t::duplicate_cell);
        expect_faults(
            result.control,
            fault_mask(h3::assembler_fault_bit_t::duplicate_cell));

        result = invoke(
            cell_input(make_cell(cell_kind_t::data, 0U, 0U,
                                 static_cast<std::uint8_t>(slope_t::fall), shot,
                                 config.columns, config.active_version),
                       config.reset_epoch),
            config);
        accumulated_faults |=
            fault_mask(h3::assembler_fault_bit_t::masked_lane_drop);
        expect_faults(
            result.control,
            fault_mask(h3::assembler_fault_bit_t::masked_lane_drop));

        cell_payload_t context_bad =
            make_cell(cell_kind_t::data, 0U, 0U,
                      static_cast<std::uint8_t>(slope_t::rise), shot,
                      config.columns, config.active_version);
        lidar_v3::write_field<h2::cell_event_layout::shot_context>(
            context_bad,
            make_context(shot, config.columns,
                         static_cast<std::uint16_t>(
                             config.active_version + 1U)));
        result = invoke(cell_input(context_bad, config.reset_epoch), config);
        accumulated_faults |=
            fault_mask(h3::assembler_fault_bit_t::shot_context_mismatch);
        expect_faults(
            result.control,
            fault_mask(h3::assembler_fault_bit_t::shot_context_mismatch));

        result = invoke(
            cell_input(make_cell(
                           cell_kind_t::data, config.num_chips, 0U,
                           static_cast<std::uint8_t>(slope_t::rise), shot,
                           config.columns, config.active_version),
                       config.reset_epoch),
            config);
        accumulated_faults |=
            fault_mask(h3::assembler_fault_bit_t::unexpected_event);
        expect_faults(
            result.control,
            fault_mask(h3::assembler_fault_bit_t::unexpected_event));
    }

    bool duplicate_sent = false;
    for (int chip = static_cast<int>(config.num_chips) - 1; chip >= 0; --chip) {
        if (((expected_mask >> chip) & 1U) != 0U) {
            const cell_payload_t terminal = make_cell(
                cell_kind_t::drain_done, static_cast<std::uint8_t>(chip), 0U,
                static_cast<std::uint8_t>(slope_t::fall), shot, config.columns,
                config.active_version, 0U);
            result = invoke(cell_input(terminal, config.reset_epoch), config);
            if (inject_faults && !duplicate_sent) {
                result =
                    invoke(cell_input(terminal, config.reset_epoch), config);
                accumulated_faults |= fault_mask(
                    h3::assembler_fault_bit_t::duplicate_terminal_event);
                expect_faults(result.control,
                              fault_mask(h3::assembler_fault_bit_t::
                                             duplicate_terminal_event));
                duplicate_sent = true;
            }
        }
    }

    assert(lidar_v3::read_flag<
        h3::assembler_control_layout::shot_cell_generation_complete>(
        result.control));
    assert(lidar_v3::read_field<
               h3::assembler_control_layout::completed_shot_context>(
               result.control) ==
           make_context(shot, config.columns, config.active_version));
    const std::uint8_t final_faults =
        inject_faults
            ? fault_mask(h3::assembler_fault_bit_t::missing_cell)
            : 0U;
    expect_faults(result.control, final_faults);
    (void)accumulated_faults;
    return result;
}

config_t profile_config(const std::string &profile) {
    config_t config = {4U, 8U, 4U, 9U, 0x3U, 0xCU, 1U, 1U};
    if (profile == "one_chip_dual") {
        config.num_chips = 1U;
        config.rise_mask = 0x1U;
        config.fall_mask = 0x1U;
    } else if (profile == "fall_off") {
        config.rise_mask = 0xFU;
        config.fall_mask = 0U;
    } else if (profile == "reduced_faults") {
        config.num_chips = 3U;
        config.stops_per_chip = 6U;
        config.rise_mask = 0x3U;
        config.fall_mask = 0x4U;
    } else if (profile == "all_dual") {
        config.rise_mask = 0xFU;
        config.fall_mask = 0xFU;
    }
    return config;
}

void run_profile(const std::string &profile) {
    config_t config = profile_config(profile);
    const bool inject_faults = profile == "reduced_faults";
    invocation_t result = send_shot(config, 0U, inject_faults);

    expect_lane(
        result.rise, config.rise_mask, config, 0U,
        static_cast<std::uint8_t>(slope_t::rise), inject_faults ? 1 : -1,
        inject_faults ? static_cast<int>(config.stops_per_chip) - 1 : -1,
        inject_faults, 0U);
    expect_lane(result.fall, config.fall_mask, config, 0U,
                static_cast<std::uint8_t>(slope_t::fall), -1, -1, inject_faults,
                0U);

    if (profile == "dedicated") {
        ++config.reset_epoch;
        config.columns = 3U;
        invocation_t first = send_shot(config, 0U);
        expect_lane(first.rise, config.rise_mask, config, 0U,
                    static_cast<std::uint8_t>(slope_t::rise));
        invocation_t gap =
            send_shot(
                config,
                2U,
                false,
                fault_mask(h3::assembler_fault_bit_t::shot_column_gap));
        expect_faults(gap.control, 0U);
        expect_lane(gap.rise, config.rise_mask, config, 2U,
                    static_cast<std::uint8_t>(slope_t::rise), -1, -1, false,
                    1U);
        assert(lidar_v3::read_flag<
            h3::assembler_control_layout::shot_cell_generation_complete>(
            gap.control));

        invocation_t close =
            invoke(face_close_input(0x12345678U, 1U, false, false,
                                    config.active_version, config.columns,
                                    config.reset_epoch),
                   config);
        assert(lidar_v3::read_flag<
            h3::assembler_control_layout::contains_face_close_result>(
            close.control));
        const h3::face_close_result_t close_payload =
            lidar_v3::read_field<
                h3::assembler_control_layout::face_close_result>(
                close.control);
        assert(lidar_v3::read_field<
                   h3::face_close_result_layout::
                       trailing_missing_shot_columns>(close_payload) == 0U);
        assert(!lidar_v3::read_flag<
            h3::face_close_result_layout::entire_face_is_missing>(
            close_payload));
        assert(!lidar_v3::read_flag<
            h3::face_close_result_layout::face_close_is_faulted>(
            close_payload));
        expect_faults(close.control, 0U);

        ++config.reset_epoch;
        invocation_t all_hole =
            invoke(face_close_input(0x12345679U, 1U, false, false,
                                    config.active_version, config.columns,
                                    config.reset_epoch),
                   config);
        const h3::face_close_result_t hole_payload =
            lidar_v3::read_field<
                h3::assembler_control_layout::face_close_result>(
                all_hole.control);
        assert(lidar_v3::read_flag<
            h3::face_close_result_layout::entire_face_is_missing>(
            hole_payload));
        assert(lidar_v3::read_field<
                   h3::face_close_result_layout::
                       trailing_missing_shot_columns>(hole_payload) ==
               config.columns);
        expect_faults(
            all_hole.control,
            fault_mask(h3::assembler_fault_bit_t::shot_column_gap));

        invocation_t bad_geometry =
            invoke(face_close_input(0x1234567AU, 1U, false, false,
                                    config.active_version + 1U,
                                    config.columns, config.reset_epoch),
                   config);
        const std::uint8_t bad_geometry_faults =
            static_cast<std::uint8_t>(
                fault_mask(h3::assembler_fault_bit_t::geometry_mismatch) |
                fault_mask(h3::assembler_fault_bit_t::shot_column_gap));
        expect_faults(bad_geometry.control, bad_geometry_faults);
        assert(lidar_v3::read_flag<
            h3::assembler_control_layout::contains_face_close_result>(
            bad_geometry.control));
    }

    if (profile == "one_chip_dual") {
        ++config.reset_epoch;
        const cell_payload_t stale = make_cell(
            cell_kind_t::data, 0U, 0U, static_cast<std::uint8_t>(slope_t::rise),
            0U, config.columns, config.active_version);
        invocation_t partial =
            invoke(cell_input(stale, config.reset_epoch), config);
        assert(partial.rise.empty() && partial.fall.empty());

        ++config.reset_epoch;
        invocation_t recovered = send_shot(config, 0U);
        expect_lane(recovered.rise, config.rise_mask, config, 0U,
                    static_cast<std::uint8_t>(slope_t::rise));
        expect_lane(recovered.fall, config.fall_mask, config, 0U,
                    static_cast<std::uint8_t>(slope_t::fall));
    }
}

} // namespace

int main() {
    const char *profile_env = std::getenv("V3_HLS_PROFILE");
    const std::string profile =
        profile_env == nullptr ? "dedicated" : profile_env;
    assert(profile == "dedicated" || profile == "one_chip_dual" ||
           profile == "fall_off" || profile == "reduced_faults" ||
           profile == "all_dual");
    run_profile(profile);
    std::cout << "LIDAR_V3_HLS_GPX_FRAME_ASSEMBLER_CSIM_PASS profile="
              << profile << '\n';
    return 0;
}
