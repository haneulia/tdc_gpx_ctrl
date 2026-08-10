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

using namespace lidar_v3;

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

ap_uint<kShotContextBits>
make_context(std::uint16_t shot_index, std::uint16_t columns,
             std::uint16_t active_version, std::uint8_t face = 1U,
             bool direction_ccw = false, bool source_sim = false) {
    ap_uint<kShotContextBits> context = 0;
    context[0] = 1;
    context.range(kContextFaceHi, kContextFaceLo) = face;
    context.range(18, 4) = 100U + shot_index;
    context[19] = direction_ccw;
    context.range(kContextShotIndexHi, kContextShotIndexLo) = shot_index;
    context[36] = shot_index + 1U >= columns;
    context[kContextSourceSim] = source_sim;
    context.range(45, 38) = 3U;
    context[46] = 1;
    context.range(kContextActiveVersionHi, kContextActiveVersionLo) =
        active_version;
    context.range(94, 63) = 7U;
    context[kShotContextBits - 1U] = 1;
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
    cell.range(kCellKindHi, kCellKindLo) = static_cast<std::uint8_t>(kind);
    cell.range(kCellChipHi, kCellChipLo) = chip;
    cell[kCellIfifo] = stop >= 4U;
    cell.range(kCellStopHi, kCellStopLo) = stop;
    cell[kCellSlope] = slope;
    cell.range(kCellHitCountHi, kCellHitCountLo) = hit_count;
    cell.range(kCellMaxHitsHi, kCellMaxHitsLo) = 2U;
    if (hit_count != 0U) {
        cell.range(kCellHitsLo + 16U, kCellHitsLo) =
            expected_hit(shot, chip, stop, slope);
    }
    cell[kCellFaulted] = faulted;
    cell.range(kCellShotContextHi, kCellShotContextLo) =
        make_context(shot, columns, version, face);
    cell.range(kCellChipShotSeqHi, kCellChipShotSeqLo) = 10U + shot;
    return cell;
}

frame_input_payload_t cell_input(const cell_payload_t &cell,
                                 std::uint8_t reset_epoch) {
    frame_input_payload_t input = 0;
    input.range(kFrameCellCellHi, kFrameCellCellLo) = cell;
    input[kFrameInputKind] = 0;
    input.range(kFrameInputResetEpochHi, kFrameInputResetEpochLo) = reset_epoch;
    return input;
}

frame_input_payload_t face_close_input(std::uint32_t frame_id,
                                       std::uint8_t face, bool direction_ccw,
                                       bool source_sim, std::uint16_t version,
                                       std::uint16_t columns,
                                       std::uint8_t reset_epoch) {
    frame_input_payload_t input = 0;
    input.range(kFaceCloseIdHi, kFaceCloseIdLo) = frame_id;
    input.range(kFaceCloseFaceHi, kFaceCloseFaceLo) = face;
    input[kFaceCloseDirection] = direction_ccw;
    input[kFaceCloseSourceSim] = source_sim;
    input.range(kFaceCloseVersionHi, kFaceCloseVersionLo) = version;
    input.range(kFaceCloseColumnsHi, kFaceCloseColumnsLo) = columns;
    input[kFrameInputKind] = 1;
    input.range(kFrameInputResetEpochHi, kFrameInputResetEpochLo) = reset_epoch;
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
    assert(control.range(7, 0).to_uint() == expected);
    assert(control.range(kFrameControlReservedHi, kFrameControlReservedLo) ==
           0U);
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
            frame.range(kFrameCellCellHi, kFrameCellCellLo);
        const std::uint8_t chip =
            chip_for_slot(mask, config.stops_per_chip, slot);
        const std::uint8_t stop = slot % config.stops_per_chip;
        const bool blank = chip == missing_chip && stop == missing_stop;

        assert(frame.range(kFrameCellSlotHi, kFrameCellSlotLo) == slot);
        assert(frame.range(kFrameCellSlotCountHi, kFrameCellSlotCountLo) ==
               count);
        assert((frame[kFrameCellLineStart] != 0) == (slot == 0U));
        assert((frame[kFrameCellLineEnd] != 0) == (slot + 1U == count));
        assert((frame[kFrameCellFirstColumn] != 0) == (shot == 0U));
        assert((frame[kFrameCellLastColumn] != 0) ==
               (shot + 1U >= config.columns));
        assert(frame.range(kFrameCellGapHi, kFrameCellGapLo) ==
               (slot == 0U ? gap : 0U));
        assert((frame[kFrameCellBlank] != 0) == blank);
        assert((frame[kFrameCellLineFaulted] != 0) == line_faulted);
        assert(frame.range(kFrameCellReservedHi, kFrameCellReservedLo) == 0U);

        assert(cell.range(kCellKindHi, kCellKindLo) ==
               static_cast<std::uint8_t>(cell_kind_t::data));
        assert(cell.range(kCellChipHi, kCellChipLo) == chip);
        assert(cell.range(kCellStopHi, kCellStopLo) == stop);
        assert((cell[kCellSlope] != 0) ==
               (slope == static_cast<std::uint8_t>(slope_t::rise)));
        assert(cell.range(kCellShotContextHi, kCellShotContextLo) ==
               make_context(shot, config.columns, config.active_version));
        if (blank) {
            assert(cell.range(kCellHitCountHi, kCellHitCountLo) == 0U);
            assert(cell[kCellErrorFill] != 0);
            assert(cell[kCellFaulted] != 0);
            assert(cell.range(kCellChipShotSeqHi, kCellChipShotSeqLo) == 0U);
        } else {
            assert(cell.range(kCellHitCountHi, kCellHitCountLo) == 1U);
            assert(cell.range(kCellHitsLo + 16U, kCellHitsLo) ==
                   expected_hit(shot, chip, stop, slope));
            assert(cell.range(kCellChipShotSeqHi, kCellChipShotSeqLo) ==
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
        accumulated_faults |= 1U << kFrameFaultDuplicateCell;
        expect_faults(result.control, 1U << kFrameFaultDuplicateCell);

        result = invoke(
            cell_input(make_cell(cell_kind_t::data, 0U, 0U,
                                 static_cast<std::uint8_t>(slope_t::fall), shot,
                                 config.columns, config.active_version),
                       config.reset_epoch),
            config);
        accumulated_faults |= 1U << kFrameFaultMaskedDrop;
        expect_faults(result.control, 1U << kFrameFaultMaskedDrop);

        cell_payload_t context_bad =
            make_cell(cell_kind_t::data, 0U, 0U,
                      static_cast<std::uint8_t>(slope_t::rise), shot,
                      config.columns, config.active_version);
        context_bad.range(kCellShotContextHi, kCellShotContextLo) =
            make_context(shot, config.columns,
                         static_cast<std::uint16_t>(
                             config.active_version + 1U));
        result = invoke(cell_input(context_bad, config.reset_epoch), config);
        accumulated_faults |= 1U << kFrameFaultContext;
        expect_faults(result.control, 1U << kFrameFaultContext);

        result = invoke(
            cell_input(make_cell(
                           cell_kind_t::data, config.num_chips, 0U,
                           static_cast<std::uint8_t>(slope_t::rise), shot,
                           config.columns, config.active_version),
                       config.reset_epoch),
            config);
        accumulated_faults |= 1U << kFrameFaultUnexpected;
        expect_faults(result.control, 1U << kFrameFaultUnexpected);
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
                accumulated_faults |= 1U << kFrameFaultDuplicateTerminal;
                expect_faults(result.control,
                              1U << kFrameFaultDuplicateTerminal);
                duplicate_sent = true;
            }
        }
    }

    assert(result.control[kFrameControlShotDone] != 0);
    assert(
        result.control.range(kFrameControlContextHi, kFrameControlContextLo) ==
        make_context(shot, config.columns, config.active_version));
    const std::uint8_t final_faults =
        inject_faults ? static_cast<std::uint8_t>(1U << kFrameFaultMissingCell)
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
            send_shot(config, 2U, false, 1U << kFrameFaultColumnGap);
        expect_faults(gap.control, 0U);
        expect_lane(gap.rise, config.rise_mask, config, 2U,
                    static_cast<std::uint8_t>(slope_t::rise), -1, -1, false,
                    1U);
        assert(gap.control[kFrameControlShotDone] != 0);

        invocation_t close =
            invoke(face_close_input(0x12345678U, 1U, false, false,
                                    config.active_version, config.columns,
                                    config.reset_epoch),
                   config);
        assert(close.control[kFrameControlEmitClose] != 0);
        const frame_close_payload_t close_payload =
            close.control.range(kFrameControlCloseHi, kFrameControlCloseLo);
        assert(close_payload.range(kFrameCloseTrailingHi,
                                   kFrameCloseTrailingLo) == 0U);
        assert(close_payload[kFrameCloseAllHole] == 0);
        assert(close_payload[kFrameCloseFaulted] == 0);
        expect_faults(close.control, 0U);

        ++config.reset_epoch;
        invocation_t all_hole =
            invoke(face_close_input(0x12345679U, 1U, false, false,
                                    config.active_version, config.columns,
                                    config.reset_epoch),
                   config);
        const frame_close_payload_t hole_payload =
            all_hole.control.range(kFrameControlCloseHi, kFrameControlCloseLo);
        assert(hole_payload[kFrameCloseAllHole] != 0);
        assert(hole_payload.range(kFrameCloseTrailingHi,
                                  kFrameCloseTrailingLo) == config.columns);
        expect_faults(all_hole.control, 1U << kFrameFaultColumnGap);

        invocation_t bad_geometry =
            invoke(face_close_input(0x1234567AU, 1U, false, false,
                                    config.active_version + 1U,
                                    config.columns, config.reset_epoch),
                   config);
        const std::uint8_t bad_geometry_faults =
            static_cast<std::uint8_t>((1U << kFrameFaultGeometry) |
                                      (1U << kFrameFaultColumnGap));
        expect_faults(bad_geometry.control, bad_geometry_faults);
        assert(bad_geometry.control[kFrameControlEmitClose] != 0);
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
