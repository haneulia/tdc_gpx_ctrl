#include "gpx_cell_collector_hls.hpp"

#include <cstdint>

namespace {

using namespace lidar_v3;

constexpr unsigned kCellsPerChip = 2U * kMaxStopsPerChip;
constexpr unsigned kCellAddressCount = kMaxChips * kCellsPerChip;

// ap_uint is intentionally limited to physical packed storage and RTL/HLS
// boundaries. Control arithmetic uses standard fixed-width C++ integer types.
using hit_storage_t = ap_uint<17>;
using context_storage_t = ap_uint<kShotContextBits>;

static hit_storage_t hit_banks[kMaxReturnsPerStop][kCellAddressCount];
static std::uint8_t seen_count[kCellAddressCount];
static bool return_overflow[kCellAddressCount];
static bool cell_error[kCellAddressCount];

static bool shot_active[kMaxChips];
static context_storage_t owner_context[kMaxChips];
static std::uint16_t owner_sequence[kMaxChips];
static std::uint8_t owner_max_hits[kMaxChips];
static bool owner_faulted[kMaxChips];
static bool lower_emitted[kMaxChips];
static bool owner_rise_enabled[kMaxChips];
static bool owner_fall_enabled[kMaxChips];
static std::uint8_t accepted_reset_epoch;

unsigned cell_address(
    std::uint8_t chip,
    std::uint8_t slope,
    std::uint8_t stop) {
    const unsigned slope_offset =
        slope == static_cast<std::uint8_t>(slope_t::fall)
            ? kMaxStopsPerChip
            : 0U;
    return static_cast<unsigned>(chip) * kCellsPerChip + slope_offset + stop;
}

std::uint8_t effective_visible_returns(
    std::uint8_t requested,
    std::uint8_t physical_max) {
    const std::uint8_t bounded_physical =
        physical_max < 1U
            ? 1U
            : (physical_max > kMaxReturnsPerStop
                   ? static_cast<std::uint8_t>(kMaxReturnsPerStop)
                   : physical_max);
    if (requested == 0U) {
        return 1U;
    }
    return requested > bounded_physical ? bounded_physical : requested;
}

context_storage_t hit_context(const hit_payload_t &hit) {
    return hit.range(kHitShotContextHi, kHitShotContextLo);
}

bool context_identity_matches(
    const context_storage_t &candidate,
    std::uint16_t candidate_sequence,
    const context_storage_t &owner,
    std::uint16_t expected_sequence) {
    return candidate_sequence == expected_sequence &&
           candidate.range(kContextActiveVersionHi, kContextActiveVersionLo) ==
               owner.range(kContextActiveVersionHi, kContextActiveVersionLo) &&
           candidate.range(kContextShotIndexHi, kContextShotIndexLo) ==
               owner.range(kContextShotIndexHi, kContextShotIndexLo) &&
           candidate.range(kContextFaceHi, kContextFaceLo) ==
               owner.range(kContextFaceHi, kContextFaceLo) &&
           candidate[kContextSourceSim] == owner[kContextSourceSim];
}

bool slope_supported(
    std::uint8_t slope,
    bool rise_enabled,
    bool fall_enabled) {
    return slope == static_cast<std::uint8_t>(slope_t::rise)
               ? rise_enabled
               : fall_enabled;
}

collector_result_payload_t status_result(
    bool context_fault,
    bool overflow_fault,
    bool start_nonzero_fault) {
    collector_result_payload_t result = 0;
    result[kCollectorContextFault] = context_fault;
    result[kCollectorReturnOverflowFault] = overflow_fault;
    result[kCollectorStartNonzeroFault] = start_nonzero_fault;
    // hit_capacity_drop is reserved for an unintended internal capacity loss.
    // Runtime visible Return filtering never sets this fault.
    result[kCollectorCapacityDropFault] = 0;
    return result;
}

cell_payload_t build_data_cell(
    std::uint8_t chip,
    std::uint8_t stop,
    std::uint8_t slope,
    std::uint8_t max_hits,
    bool error_fill,
    bool shot_fault,
    std::uint8_t timeout_cause,
    const context_storage_t &context,
    std::uint16_t sequence) {
    cell_payload_t cell = 0;
    const unsigned address = cell_address(chip, slope, stop);
    const std::uint8_t visible_count =
        seen_count[address] > max_hits ? max_hits : seen_count[address];

    cell.range(kCellKindHi, kCellKindLo) =
        static_cast<std::uint8_t>(cell_kind_t::data);
    cell.range(kCellChipHi, kCellChipLo) = chip;
    cell[kCellIfifo] = stop >= 4U;
    cell.range(kCellStopHi, kCellStopLo) = stop;
    cell[kCellSlope] = slope;
    cell.range(kCellHitCountHi, kCellHitCountLo) = visible_count;
    cell.range(kCellMaxHitsHi, kCellMaxHitsLo) = max_hits;

    for (unsigned index = 0; index < kMaxReturnsPerStop; ++index) {
#pragma HLS UNROLL
        const unsigned lo = kCellHitsLo + index * 17U;
        cell.range(lo + 16U, lo) =
            index < visible_count
                ? hit_banks[index][address]
                : hit_storage_t(0);
    }

    cell[kCellHitDropped] = 0;
    cell[kCellReturnOverflow] = return_overflow[address];
    cell[kCellErrorFill] = error_fill;
    cell[kCellFaulted] = cell_error[address] || shot_fault || error_fill;
    cell.range(kCellTimeoutHi, kCellTimeoutLo) = timeout_cause & 0x7U;
    cell.range(kCellShotContextHi, kCellShotContextLo) = context;
    cell.range(kCellChipShotSeqHi, kCellChipShotSeqLo) = sequence;
    return cell;
}

cell_payload_t build_control_cell(
    cell_kind_t kind,
    std::uint8_t chip,
    std::uint8_t ififo,
    std::uint8_t first_slope,
    std::uint8_t max_hits,
    bool error_fill,
    bool shot_fault,
    std::uint8_t timeout_cause,
    const context_storage_t &context,
    std::uint16_t sequence) {
    cell_payload_t cell = 0;
    cell.range(kCellKindHi, kCellKindLo) = static_cast<std::uint8_t>(kind);
    cell.range(kCellChipHi, kCellChipLo) = chip;
    cell[kCellIfifo] = ififo != 0U;
    cell[kCellSlope] = first_slope;
    cell.range(kCellMaxHitsHi, kCellMaxHitsLo) = max_hits;
    cell[kCellErrorFill] = error_fill;
    cell[kCellFaulted] = shot_fault || error_fill;
    cell.range(kCellTimeoutHi, kCellTimeoutLo) = timeout_cause & 0x7U;
    cell.range(kCellShotContextHi, kCellShotContextLo) = context;
    cell.range(kCellChipShotSeqHi, kCellChipShotSeqLo) = sequence;
    return cell;
}

void emit_cell(
    hls::stream<collector_result_payload_t> &result_out,
    const cell_payload_t &cell) {
    collector_result_payload_t result = 0;
    result.range(kCollectorCellHi, kCollectorCellLo) = cell;
    result[kCollectorEmit] = 1;
    result_out.write(result);
}

void clear_owner_state() {
    for (unsigned chip = 0; chip < kMaxChips; ++chip) {
#pragma HLS PIPELINE II=1
        shot_active[chip] = false;
        owner_faulted[chip] = false;
        lower_emitted[chip] = false;
        owner_rise_enabled[chip] = false;
        owner_fall_enabled[chip] = false;
    }
}

void scrub_chip(std::uint8_t chip) {
    const unsigned base = static_cast<unsigned>(chip) * kCellsPerChip;
    for (unsigned offset = 0; offset < kCellsPerChip; ++offset) {
#pragma HLS PIPELINE II=1
        const unsigned address = base + offset;
        seen_count[address] = 0;
        return_overflow[address] = false;
        cell_error[address] = false;
    }
}

}  // namespace

extern "C" void gpx_cell_collector_hls(
    hls::stream<lidar_v3::cell_hit_input_t> &hit_in,
    hls::stream<lidar_v3::collector_result_payload_t> &result_out,
    std::uint8_t num_chips,
    std::uint8_t stops_per_chip,
    std::uint8_t max_returns_per_stop,
    std::uint8_t rise_mask,
    std::uint8_t fall_mask,
    std::uint8_t visible_returns,
    std::uint16_t active_version) {
#pragma HLS INTERFACE axis port=hit_in
#pragma HLS INTERFACE axis port=result_out
#pragma HLS INTERFACE ap_none port=num_chips
#pragma HLS INTERFACE ap_none port=stops_per_chip
#pragma HLS INTERFACE ap_none port=max_returns_per_stop
#pragma HLS INTERFACE ap_none port=rise_mask
#pragma HLS INTERFACE ap_none port=fall_mask
#pragma HLS INTERFACE ap_none port=visible_returns
#pragma HLS INTERFACE ap_none port=active_version
#pragma HLS INTERFACE ap_ctrl_hs port=return
#pragma HLS ARRAY_PARTITION variable=hit_banks complete dim=1
#pragma HLS BIND_STORAGE variable=hit_banks type=ram_2p impl=lutram
#pragma HLS BIND_STORAGE variable=seen_count type=ram_2p impl=lutram
#pragma HLS BIND_STORAGE variable=return_overflow type=ram_2p impl=lutram
#pragma HLS BIND_STORAGE variable=cell_error type=ram_2p impl=lutram
#pragma HLS BIND_STORAGE variable=shot_active type=ram_2p impl=lutram
#pragma HLS BIND_STORAGE variable=owner_context type=ram_2p impl=lutram
#pragma HLS BIND_STORAGE variable=owner_sequence type=ram_2p impl=lutram
#pragma HLS BIND_STORAGE variable=owner_max_hits type=ram_2p impl=lutram
#pragma HLS BIND_STORAGE variable=owner_faulted type=ram_2p impl=lutram
#pragma HLS BIND_STORAGE variable=lower_emitted type=ram_2p impl=lutram
#pragma HLS BIND_STORAGE variable=owner_rise_enabled type=ram_2p impl=lutram
#pragma HLS BIND_STORAGE variable=owner_fall_enabled type=ram_2p impl=lutram

    using namespace lidar_v3;

    // The RTL adapter keeps ap_start asserted. ap_ctrl_hs therefore starts a
    // new invocation as soon as the previous input and all of its Cell output
    // have retired. Runtime configuration is sampled on that boundary.
    const cell_hit_input_t input = hit_in.read();
    const std::uint8_t reset_epoch = input.range(
        kCellInputResetEpochHi, kCellInputResetEpochLo).to_uint();

    // reset_epoch travels with the accepted Hit. A changed epoch therefore
    // clears all per-Chip ownership before this same Hit opens a new Shot,
    // even when the core was blocked during the original abort pulse.
    if (reset_epoch != accepted_reset_epoch) {
        clear_owner_state();
        accepted_reset_epoch = reset_epoch;
    }

    const hit_payload_t hit = cell_input_hit(input);
    const std::uint8_t kind = hit.range(kHitKindHi, kHitKindLo).to_uint();
    const std::uint8_t chip = hit.range(kHitChipHi, kHitChipLo).to_uint();
    const std::uint8_t ififo = hit[kHitIfifo] ? 1U : 0U;
    const std::uint8_t stop = hit.range(kHitStopHi, kHitStopLo).to_uint();
    const std::uint8_t slope = hit[kHitSlope] ? 1U : 0U;
    const std::uint16_t sequence =
        hit.range(kHitChipShotSeqHi, kHitChipShotSeqLo).to_uint();
    const context_storage_t context = hit_context(hit);
    const std::uint8_t requested_max =
        effective_visible_returns(visible_returns, max_returns_per_stop);
    const bool build_valid = num_chips >= 1U && num_chips <= kMaxChips &&
                             stops_per_chip >= 1U &&
                             stops_per_chip <= kMaxStopsPerChip &&
                             max_returns_per_stop >= 1U &&
                             max_returns_per_stop <= kMaxReturnsPerStop;
    const bool config_valid =
        build_valid && visible_returns >= 1U &&
        visible_returns <= max_returns_per_stop &&
        context.range(kContextActiveVersionHi, kContextActiveVersionLo) ==
            active_version;

    bool context_fault = false;
    bool overflow_fault = false;
    bool start_nonzero_fault = false;

    if (chip >= num_chips || chip >= kMaxChips) {
        context_fault = true;
        result_out.write(status_result(context_fault, false, false));
        return;
    }

    const bool opens_new_shot = !shot_active[chip];
    if (opens_new_shot) {
        scrub_chip(chip);
        shot_active[chip] = true;
        owner_context[chip] = context;
        owner_sequence[chip] = sequence;
        owner_max_hits[chip] = requested_max;
        owner_faulted[chip] = (hit[kHitFaulted] != 0) || !config_valid;
        lower_emitted[chip] = false;
        owner_rise_enabled[chip] = ((rise_mask >> chip) & 0x1U) != 0U;
        owner_fall_enabled[chip] = ((fall_mask >> chip) & 0x1U) != 0U;
    }

    if (opens_new_shot) {
        // The accepted event itself defines a new owner, so comparing that
        // freshly written owner back to the same event is redundant. Keeping
        // only configuration validation here removes a wide write-bypass
        // path without changing the V2 ownership contract.
        if (!config_valid) {
            context_fault = true;
            owner_faulted[chip] = true;
        }
    } else {
        if (!context_identity_matches(
                context,
                sequence,
                owner_context[chip],
                owner_sequence[chip]) ||
            owner_max_hits[chip] != requested_max || !config_valid) {
            context_fault = true;
            owner_faulted[chip] = true;
        } else if (hit[kHitFaulted] != 0) {
            owner_faulted[chip] = true;
        }
    }

    if (kind == static_cast<std::uint8_t>(raw_kind_t::data)) {
        if (stop >= stops_per_chip || stop >= kMaxStopsPerChip ||
            !slope_supported(
                slope,
                owner_rise_enabled[chip],
                owner_fall_enabled[chip])) {
            context_fault = true;
            owner_faulted[chip] = true;
        } else {
            const unsigned address = cell_address(chip, slope, stop);
            const std::uint8_t seen = seen_count[address];

            if (hit.range(kHitStartHi, kHitStartLo) != 0U) {
                start_nonzero_fault = true;
                cell_error[address] = true;
                owner_faulted[chip] = true;
            }

            if (seen < max_returns_per_stop &&
                seen < kMaxReturnsPerStop) {
                if (seen < owner_max_hits[chip]) {
                    hit_banks[seen][address] =
                        hit.range(kHitValueHi, kHitValueLo);
                }
                // Returns above the Runtime visible count are intentionally
                // filtered after physical IFIFO Drain and are not a fault.
                seen_count[address] = seen + 1U;
            } else {
                return_overflow[address] = true;
                cell_error[address] = true;
                owner_faulted[chip] = true;
                overflow_fault = true;
            }
        }

        result_out.write(status_result(
            context_fault, overflow_fault, start_nonzero_fault));
        return;
    }

    result_out.write(status_result(context_fault, false, false));

    cell_kind_t cell_kind = cell_kind_t::drain_done;
    std::uint8_t first_stop = 0U;
    std::uint8_t last_stop = stops_per_chip - 1U;
    bool error_fill = hit[kHitFaulted] != 0;
    bool terminal = true;

    if (kind == static_cast<std::uint8_t>(raw_kind_t::ififo1_done)) {
        cell_kind = cell_kind_t::ififo1_done;
        last_stop = stops_per_chip > 4U ? 3U : stops_per_chip - 1U;
        terminal = false;
    } else if (kind == static_cast<std::uint8_t>(raw_kind_t::drain_done)) {
        cell_kind = cell_kind_t::drain_done;
        first_stop = lower_emitted[chip] ? 4U : 0U;
    } else {
        cell_kind = cell_kind_t::timeout;
        first_stop = lower_emitted[chip] ? 4U : 0U;
        error_fill = true;
        owner_faulted[chip] = true;
    }

    const std::uint8_t first_slope =
        owner_rise_enabled[chip]
            ? static_cast<std::uint8_t>(slope_t::rise)
            : static_cast<std::uint8_t>(slope_t::fall);
    const std::uint8_t timeout_cause =
        hit.range(kHitTimeoutHi, kHitTimeoutLo).to_uint();

    for (unsigned slope_order = 0; slope_order < 2U; ++slope_order) {
        const std::uint8_t emit_slope =
            slope_order == 0U
                ? static_cast<std::uint8_t>(slope_t::rise)
                : static_cast<std::uint8_t>(slope_t::fall);
        const bool enabled =
            emit_slope == static_cast<std::uint8_t>(slope_t::rise)
                ? owner_rise_enabled[chip]
                : owner_fall_enabled[chip];
        if (enabled && first_stop < stops_per_chip) {
            for (unsigned emit_stop = 0; emit_stop < kMaxStopsPerChip;
                 ++emit_stop) {
#pragma HLS PIPELINE II=1
                if (emit_stop >= first_stop && emit_stop <= last_stop &&
                    emit_stop < stops_per_chip) {
                    emit_cell(
                        result_out,
                        build_data_cell(
                            chip,
                            static_cast<std::uint8_t>(emit_stop),
                            emit_slope,
                            owner_max_hits[chip],
                            error_fill,
                            owner_faulted[chip],
                            timeout_cause,
                            owner_context[chip],
                            owner_sequence[chip]));
                }
            }
        }
    }

    emit_cell(
        result_out,
        build_control_cell(
            cell_kind,
            chip,
            ififo,
            first_slope,
            owner_max_hits[chip],
            error_fill,
            owner_faulted[chip],
            timeout_cause,
            owner_context[chip],
            owner_sequence[chip]));

    if (terminal) {
        shot_active[chip] = false;
        owner_faulted[chip] = false;
        lower_emitted[chip] = false;
        owner_rise_enabled[chip] = false;
        owner_fall_enabled[chip] = false;
    } else {
        lower_emitted[chip] = true;
    }
}
