#include <cassert>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include "gpx_cell_collector_hls.hpp"

// 검증 목적:
//   물리 Return 7개 수집, Runtime 전시 Return 1~7 필터, 8번째 Return
//   overflow, timeout/error-fill과 Rise/Fall Cell 순서를 독립적으로 검증한다.
//   V2 RTL과의 Clock 단위 차동 검증은 별도 VHDL 테스트가 담당한다.

namespace {

using namespace lidar_v3;

struct config_t {
    std::uint8_t num_chips;
    std::uint8_t stops_per_chip;
    std::uint8_t max_returns;
    std::uint8_t rise_mask;
    std::uint8_t fall_mask;
    std::uint8_t visible_returns;
    std::uint16_t active_version;
    std::uint8_t reset_epoch;
};

using result_list_t = std::vector<collector_result_payload_t>;

ap_uint<kShotContextBits> make_context(
    std::uint16_t shot_index,
    std::uint16_t active_version,
    std::uint8_t face = 1U,
    bool source_sim = false) {
    ap_uint<kShotContextBits> context = 0;
    for (unsigned bit = 0; bit < kShotContextBits; ++bit) {
        context[bit] = ((bit * 7U + shot_index) % 13U) < 6U;
    }
    context.range(kContextFaceHi, kContextFaceLo) = face & 0x7U;
    context.range(kContextShotIndexHi, kContextShotIndexLo) = shot_index;
    context[kContextSourceSim] = source_sim;
    context.range(kContextActiveVersionHi, kContextActiveVersionLo) =
        active_version;
    context[kShotContextBits - 1U] = 1;
    return context;
}

cell_hit_input_t make_hit(
    raw_kind_t kind,
    std::uint8_t chip,
    std::uint8_t stop,
    slope_t slope,
    std::uint32_t hit_value,
    std::uint8_t start_number,
    std::uint16_t shot_index,
    std::uint16_t active_version,
    bool faulted = false,
    std::uint8_t timeout_cause = 0U,
    std::uint8_t face = 1U,
    bool source_sim = false) {
    cell_hit_input_t input = 0;
    hit_payload_t hit = 0;
    hit.range(kHitKindHi, kHitKindLo) = static_cast<std::uint8_t>(kind);
    hit.range(kHitChipHi, kHitChipLo) = chip;
    hit[kHitIfifo] = stop >= 4U;
    hit.range(kHitChannelHi, kHitChannelLo) = stop & 0x3U;
    hit.range(kHitStopHi, kHitStopLo) = stop;
    hit.range(kHitStartHi, kHitStartLo) = start_number;
    hit[kHitSlope] = static_cast<std::uint8_t>(slope);
    hit.range(kHitValueHi, kHitValueLo) = hit_value & 0x1FFFFU;
    hit[kHitFaulted] = faulted;
    hit.range(kHitTimeoutHi, kHitTimeoutLo) = timeout_cause & 0x7U;
    hit.range(kHitShotContextHi, kHitShotContextLo) =
        make_context(shot_index, active_version, face, source_sim);
    hit.range(kHitChipShotSeqHi, kHitChipShotSeqLo) = shot_index;
    input.range(kHitPayloadBits - 1U, 0U) = hit;
    return input;
}

result_list_t invoke(
    const cell_hit_input_t &input,
    const config_t &config) {
    hls::stream<cell_hit_input_t> hit_stream;
    hls::stream<collector_result_payload_t> result_stream;
    cell_hit_input_t tagged_input = input;
    tagged_input.range(kCellInputResetEpochHi, kCellInputResetEpochLo) =
        config.reset_epoch;
    hit_stream.write(tagged_input);
    gpx_cell_collector_hls(
        hit_stream,
        result_stream,
        config.num_chips,
        config.stops_per_chip,
        config.max_returns,
        config.rise_mask,
        config.fall_mask,
        config.visible_returns,
        config.active_version);

    result_list_t results;
    while (!result_stream.empty()) {
        results.push_back(result_stream.read());
    }
    assert(!results.empty());
    return results;
}

void expect_status(
    const collector_result_payload_t &result,
    bool context_fault = false,
    bool overflow_fault = false,
    bool start_fault = false) {
    assert(!collector_result_emits_cell(result));
    assert((result[kCollectorContextFault] != 0) == context_fault);
    assert((result[kCollectorReturnOverflowFault] != 0) == overflow_fault);
    assert((result[kCollectorStartNonzeroFault] != 0) == start_fault);
    assert(result[kCollectorCapacityDropFault] == 0);
    assert(result.range(kCollectorReservedHi, kCollectorReservedLo) == 0U);
}

cell_payload_t emitted_cell(const collector_result_payload_t &result) {
    assert(collector_result_emits_cell(result));
    assert(result.range(kCollectorReservedHi, kCollectorReservedLo) == 0U);
    assert(result.range(
               kCollectorCapacityDropFault,
               kCollectorContextFault) == 0U);
    return collector_result_cell(result);
}

std::uint32_t cell_hit(const cell_payload_t &cell, unsigned index) {
    const unsigned lo = kCellHitsLo + index * 17U;
    return cell.range(lo + 16U, lo).to_uint();
}

void expect_data_cell(
    const cell_payload_t &cell,
    std::uint8_t chip,
    std::uint8_t stop,
    slope_t slope,
    std::uint8_t hit_count,
    std::uint8_t max_hits,
    const std::vector<std::uint32_t> &expected_hits,
    bool overflow = false,
    bool error_fill = false,
    bool faulted = false,
    std::uint8_t timeout_cause = 0U) {
    assert(cell.range(kCellKindHi, kCellKindLo).to_uint() ==
           static_cast<std::uint8_t>(cell_kind_t::data));
    assert(cell.range(kCellChipHi, kCellChipLo).to_uint() == chip);
    assert((cell[kCellIfifo] != 0) == (stop >= 4U));
    assert(cell.range(kCellStopHi, kCellStopLo).to_uint() == stop);
    assert((cell[kCellSlope] ? 1U : 0U) ==
           static_cast<std::uint8_t>(slope));
    assert(cell.range(kCellHitCountHi, kCellHitCountLo).to_uint() == hit_count);
    assert(cell.range(kCellMaxHitsHi, kCellMaxHitsLo).to_uint() == max_hits);
    assert(cell[kCellHitDropped] == 0);
    assert((cell[kCellReturnOverflow] != 0) == overflow);
    assert((cell[kCellErrorFill] != 0) == error_fill);
    assert((cell[kCellFaulted] != 0) == faulted);
    assert(cell.range(kCellTimeoutHi, kCellTimeoutLo).to_uint() ==
           timeout_cause);

    for (unsigned index = 0; index < kMaxReturnsPerStop; ++index) {
        const std::uint32_t expected =
            index < expected_hits.size() ? expected_hits[index] : 0U;
        assert(cell_hit(cell, index) == expected);
    }
}

void expect_control_cell(
    const cell_payload_t &cell,
    cell_kind_t kind,
    std::uint8_t chip,
    std::uint8_t max_hits,
    bool error_fill = false,
    bool faulted = false,
    std::uint8_t timeout_cause = 0U) {
    assert(cell.range(kCellKindHi, kCellKindLo).to_uint() ==
           static_cast<std::uint8_t>(kind));
    assert(cell.range(kCellChipHi, kCellChipLo).to_uint() == chip);
    assert(cell.range(kCellHitCountHi, kCellHitCountLo).to_uint() == 0U);
    assert(cell.range(kCellMaxHitsHi, kCellMaxHitsLo).to_uint() == max_hits);
    assert((cell[kCellErrorFill] != 0) == error_fill);
    assert((cell[kCellFaulted] != 0) == faulted);
    assert(cell.range(kCellTimeoutHi, kCellTimeoutLo).to_uint() ==
           timeout_cause);
}

void expect_data_status_ok(
    const cell_hit_input_t &input,
    const config_t &config) {
    const result_list_t results = invoke(input, config);
    assert(results.size() == 1U);
    expect_status(results[0]);
}

void run_dedicated_profile() {
    config_t config{4, 8, 7, 0x3, 0xC, 1, 5, 0};

    for (std::uint8_t visible = 1; visible <= 7; ++visible) {
        config.visible_returns = visible;
        const std::uint16_t shot = 10U + visible;
        std::vector<std::uint32_t> hits;
        for (std::uint8_t index = 0; index < 7; ++index) {
            const std::uint32_t value =
                ((visible * 0x1001U) + index * 0x111U) & 0x1FFFFU;
            if (index < visible) {
                hits.push_back(value);
            }
            expect_data_status_ok(
                make_hit(
                    raw_kind_t::data,
                    0,
                    0,
                    slope_t::rise,
                    value,
                    0,
                    shot,
                    config.active_version),
                config);
        }

        const result_list_t results = invoke(
            make_hit(
                raw_kind_t::drain_done,
                0,
                7,
                slope_t::rise,
                0,
                0,
                shot,
                config.active_version),
            config);
        assert(results.size() == 10U);
        expect_status(results[0]);
        for (std::uint8_t stop = 0; stop < 8; ++stop) {
            expect_data_cell(
                emitted_cell(results[1U + stop]),
                0,
                stop,
                slope_t::rise,
                stop == 0U ? visible : 0U,
                visible,
                stop == 0U ? hits : std::vector<std::uint32_t>{});
        }
        expect_control_cell(
            emitted_cell(results.back()),
            cell_kind_t::drain_done,
            0,
            visible);
    }

    config.visible_returns = 2;
    const std::uint16_t shot = 30;
    expect_data_status_ok(
        make_hit(
            raw_kind_t::data,
            2,
            3,
            slope_t::fall,
            0x1FFFF,
            0,
            shot,
            config.active_version),
        config);
    const result_list_t fall_results = invoke(
        make_hit(
            raw_kind_t::drain_done,
            2,
            7,
            slope_t::fall,
            0,
            0,
            shot,
            config.active_version),
        config);
    assert(fall_results.size() == 10U);
    for (std::uint8_t stop = 0; stop < 8; ++stop) {
        expect_data_cell(
            emitted_cell(fall_results[1U + stop]),
            2,
            stop,
            slope_t::fall,
            stop == 3U ? 1U : 0U,
            2,
            stop == 3U ? std::vector<std::uint32_t>{0x1FFFFU}
                       : std::vector<std::uint32_t>{});
    }
}

void run_one_chip_dual_profile() {
    config_t config{1, 8, 7, 0x1, 0x1, 7, 5, 0};
    const std::uint16_t shot = 40;

    expect_data_status_ok(
        make_hit(raw_kind_t::data, 0, 2, slope_t::rise,
                 0x10011, 0, shot, 5),
        config);
    expect_data_status_ok(
        make_hit(raw_kind_t::data, 0, 2, slope_t::fall,
                 0x00021, 0, shot, 5),
        config);

    result_list_t lower = invoke(
        make_hit(raw_kind_t::ififo1_done, 0, 0, slope_t::rise,
                 0, 0, shot, 5),
        config);
    assert(lower.size() == 10U);
    expect_status(lower[0]);
    for (unsigned index = 0; index < 8; ++index) {
        const slope_t slope = index < 4 ? slope_t::rise : slope_t::fall;
        const std::uint8_t stop = index & 0x3U;
        const bool populated = stop == 2U;
        expect_data_cell(
            emitted_cell(lower[1U + index]),
            0,
            stop,
            slope,
            populated ? 1U : 0U,
            7,
            populated
                ? std::vector<std::uint32_t>{
                      slope == slope_t::rise ? 0x10011U : 0x00021U}
                : std::vector<std::uint32_t>{});
    }
    expect_control_cell(
        emitted_cell(lower.back()), cell_kind_t::ififo1_done, 0, 7);

    expect_data_status_ok(
        make_hit(raw_kind_t::data, 0, 6, slope_t::rise,
                 0x1ABCD, 0, shot, 5),
        config);
    result_list_t upper = invoke(
        make_hit(raw_kind_t::drain_done, 0, 7, slope_t::rise,
                 0, 0, shot, 5),
        config);
    assert(upper.size() == 10U);
    for (unsigned index = 0; index < 8; ++index) {
        const slope_t slope = index < 4 ? slope_t::rise : slope_t::fall;
        const std::uint8_t stop = 4U + (index & 0x3U);
        const bool populated = slope == slope_t::rise && stop == 6U;
        expect_data_cell(
            emitted_cell(upper[1U + index]),
            0,
            stop,
            slope,
            populated ? 1U : 0U,
            7,
            populated ? std::vector<std::uint32_t>{0x1ABCDU}
                      : std::vector<std::uint32_t>{});
    }

    const std::uint16_t timeout_shot = 41;
    result_list_t timeout = invoke(
        make_hit(raw_kind_t::timeout, 0, 7, slope_t::rise,
                 0, 0, timeout_shot, 5, true, 5),
        config);
    assert(timeout.size() == 18U);
    expect_status(timeout[0]);
    for (unsigned index = 0; index < 16; ++index) {
        const slope_t slope = index < 8 ? slope_t::rise : slope_t::fall;
        const std::uint8_t stop = index & 0x7U;
        expect_data_cell(
            emitted_cell(timeout[1U + index]),
            0,
            stop,
            slope,
            0,
            7,
            {},
            false,
            true,
            true,
            5);
    }
    expect_control_cell(
        emitted_cell(timeout.back()),
        cell_kind_t::timeout,
        0,
        7,
        true,
        true,
        5);
}

void run_reduced_fault_profile() {
    config_t config{3, 6, 7, 0x3, 0x4, 3, 5, 0};
    const std::uint16_t shot = 50;
    std::vector<std::uint32_t> first_three;

    for (std::uint8_t index = 0; index < 8; ++index) {
        const std::uint32_t value = 0x10020U + index;
        if (index < 3U) {
            first_three.push_back(value);
        }
        const result_list_t status = invoke(
            make_hit(
                raw_kind_t::data,
                0,
                1,
                slope_t::rise,
                value,
                index == 0U ? 1U : 0U,
                shot,
                5),
            config);
        assert(status.size() == 1U);
        expect_status(
            status[0], false, index == 7U, index == 0U);
    }

    result_list_t overflow_cells = invoke(
        make_hit(raw_kind_t::drain_done, 0, 5, slope_t::rise,
                 0, 0, shot, 5),
        config);
    assert(overflow_cells.size() == 8U);
    expect_status(overflow_cells[0]);
    for (std::uint8_t stop = 0; stop < 6; ++stop) {
        expect_data_cell(
            emitted_cell(overflow_cells[1U + stop]),
            0,
            stop,
            slope_t::rise,
            stop == 1U ? 3U : 0U,
            3,
            stop == 1U ? first_three : std::vector<std::uint32_t>{},
            stop == 1U,
            false,
            true);
    }
    expect_control_cell(
        emitted_cell(overflow_cells.back()),
        cell_kind_t::drain_done,
        0,
        3,
        false,
        true);

    // A different Shot identity within the same Chip Shot is retained in the
    // owner Cell but raises context_mismatch and faults the complete Shot.
    const std::uint16_t owner_shot = 60;
    expect_data_status_ok(
        make_hit(raw_kind_t::data, 1, 2, slope_t::rise,
                 0x12340, 0, owner_shot, 5),
        config);
    result_list_t mismatch_status = invoke(
        make_hit(raw_kind_t::data, 1, 2, slope_t::rise,
                 0x12341, 0, owner_shot + 1U, 5),
        config);
    assert(mismatch_status.size() == 1U);
    expect_status(mismatch_status[0], true);
    result_list_t mismatch_cells = invoke(
        make_hit(raw_kind_t::drain_done, 1, 5, slope_t::rise,
                 0, 0, owner_shot, 5),
        config);
    assert(mismatch_cells.size() == 8U);
    expect_data_cell(
        emitted_cell(mismatch_cells[3U]),
        1,
        2,
        slope_t::rise,
        2,
        3,
        {0x12340U, 0x12341U},
        false,
        false,
        true);

    // Abort is represented by a changed reset_epoch on the first Hit after
    // recovery. The stale Shot payload must disappear before that Hit is
    // accepted, even though the physical Hit RAM itself is not reset.
    const std::uint16_t stale_shot = 62;
    expect_data_status_ok(
        make_hit(raw_kind_t::data, 1, 3, slope_t::rise,
                 0x0AAAA, 0, stale_shot, 5),
        config);
    config.reset_epoch = 1;
    const std::uint16_t recovered_shot = 63;
    expect_data_status_ok(
        make_hit(raw_kind_t::data, 1, 3, slope_t::rise,
                 0x15555, 0, recovered_shot, 5),
        config);
    result_list_t recovered_cells = invoke(
        make_hit(raw_kind_t::drain_done, 1, 5, slope_t::rise,
                 0, 0, recovered_shot, 5),
        config);
    assert(recovered_cells.size() == 8U);
    expect_data_cell(
        emitted_cell(recovered_cells[4U]),
        1,
        3,
        slope_t::rise,
        1,
        3,
        {0x15555U});
}

void run_all_dual_profile() {
    config_t config{4, 8, 7, 0xF, 0xF, 4, 5, 0};
    for (std::uint8_t chip = 0; chip < 4; ++chip) {
        const std::uint16_t shot = 70U + chip;
        expect_data_status_ok(
            make_hit(raw_kind_t::data, chip, chip, slope_t::rise,
                     0x10000U + chip, 0, shot, 5),
            config);
        expect_data_status_ok(
            make_hit(raw_kind_t::data, chip, chip, slope_t::fall,
                     0x01000U + chip, 0, shot, 5),
            config);
        const result_list_t results = invoke(
            make_hit(raw_kind_t::drain_done, chip, 7, slope_t::rise,
                     0, 0, shot, 5),
            config);
        assert(results.size() == 18U);
        expect_status(results[0]);
        expect_data_cell(
            emitted_cell(results[1U + chip]),
            chip,
            chip,
            slope_t::rise,
            1,
            4,
            {0x10000U + chip});
        expect_data_cell(
            emitted_cell(results[9U + chip]),
            chip,
            chip,
            slope_t::fall,
            1,
            4,
            {0x01000U + chip});
    }
}

}  // namespace

int main() {
    const char *profile_env = std::getenv("V3_HLS_PROFILE");
    const std::string profile =
        profile_env == nullptr ? "dedicated" : profile_env;

    if (profile == "dedicated") {
        run_dedicated_profile();
    } else if (profile == "one_chip_dual") {
        run_one_chip_dual_profile();
    } else if (profile == "reduced") {
        run_reduced_fault_profile();
    } else if (profile == "all_dual") {
        run_all_dual_profile();
    } else {
        std::cerr << "Unknown V3_HLS_PROFILE: " << profile << '\n';
        return 2;
    }

    std::cout << "LIDAR_V3_HLS_GPX_CELL_COLLECTOR_CSIM_PASS profile="
              << profile << '\n';
    return 0;
}
