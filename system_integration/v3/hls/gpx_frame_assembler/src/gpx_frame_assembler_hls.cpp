#include "gpx_frame_assembler_hls.hpp"

#include <cstdint>

namespace {

using namespace lidar_v3;

constexpr unsigned kCellsPerSlope = kMaxChips * kMaxStopsPerChip;
using context_storage_t = ap_uint<kShotContextBits>;

static lane_cell_storage_t rise_cells[kCellsPerSlope];
static lane_cell_storage_t fall_cells[kCellsPerSlope];
static std::uint32_t rise_present;
static std::uint32_t fall_present;

static bool shot_active;
static context_storage_t shot_context;
static std::uint8_t shot_max_hits;
static std::uint8_t shot_rise_mask;
static std::uint8_t shot_fall_mask;
static std::uint8_t shot_terminal_mask;
static std::uint16_t shot_columns;
static std::uint16_t shot_gap_before;
static bool shot_faulted;
static std::uint16_t chip_sequence[kMaxChips];

static bool history_valid;
static std::uint8_t history_face;
static bool history_direction;
static bool history_source_sim;
static std::uint16_t history_version;
static std::uint16_t history_column;
static bool history_last;

static std::uint8_t accepted_reset_epoch;

unsigned cell_address(std::uint8_t chip, std::uint8_t stop) {
    return static_cast<unsigned>(chip) * kMaxStopsPerChip + stop;
}

std::uint8_t popcount4(std::uint8_t mask) {
    std::uint8_t count = 0;
    for (unsigned bit = 0; bit < kMaxChips; ++bit) {
#pragma HLS UNROLL
        count += (mask >> bit) & 0x1U;
    }
    return count;
}

struct slot_address_t {
    std::uint8_t chip;
    std::uint8_t stop;
};

slot_address_t slot_address(std::uint8_t mask, std::uint8_t stops_per_chip,
                            std::uint8_t slot) {
    std::uint8_t remaining = slot;
    slot_address_t result = {0U, 0U};
    bool found = false;
    for (unsigned chip = 0; chip < kMaxChips; ++chip) {
#pragma HLS UNROLL
        if (!found && ((mask >> chip) & 0x1U) != 0U) {
            if (remaining < stops_per_chip) {
                result.chip = static_cast<std::uint8_t>(chip);
                result.stop = remaining;
                found = true;
            } else {
                remaining =
                    static_cast<std::uint8_t>(remaining - stops_per_chip);
            }
        }
    }
    return result;
}

context_storage_t cell_context(const cell_payload_t &cell) {
    return cell.range(kCellShotContextHi, kCellShotContextLo);
}

std::uint16_t context_shot_index(const context_storage_t &context) {
    return context.range(kContextShotIndexHi, kContextShotIndexLo).to_uint();
}

std::uint8_t context_face(const context_storage_t &context) {
    return context.range(kContextFaceHi, kContextFaceLo).to_uint();
}

std::uint16_t context_version(const context_storage_t &context) {
    return context.range(kContextActiveVersionHi, kContextActiveVersionLo)
        .to_uint();
}

bool context_direction(const context_storage_t &context) {
    // fn_pack_shot_context places request.direction at bit 19.
    return context[19] != 0;
}

bool context_last_column(const context_storage_t &context) {
    // fn_pack_shot_context places request.last_in_face at bit 36.
    return context[36] != 0;
}

bool contexts_equal(const context_storage_t &left,
                    const context_storage_t &right) {
    // A direct 162-bit operator!= becomes a long subtract/carry comparator on
    // 7-series devices. Parallel 32-bit XOR groups preserve exact equality of
    // every packed bit while keeping the result on a shallow reduction tree.
    std::uint32_t difference = 0U;
    difference |= left.range(31, 0).to_uint() ^ right.range(31, 0).to_uint();
    difference |= left.range(63, 32).to_uint() ^ right.range(63, 32).to_uint();
    difference |= left.range(95, 64).to_uint() ^ right.range(95, 64).to_uint();
    difference |=
        left.range(127, 96).to_uint() ^ right.range(127, 96).to_uint();
    difference |=
        left.range(159, 128).to_uint() ^ right.range(159, 128).to_uint();
    difference |= static_cast<std::uint32_t>(left.range(161, 160).to_uint() ^
                                             right.range(161, 160).to_uint());
    return difference == 0U;
}

lane_cell_storage_t pack_lane_cell(const cell_payload_t &cell) {
    lane_cell_storage_t result = 0;
    result.range(kLaneHitsHi, kLaneHitsLo) =
        cell.range(kCellHitsHi, kCellHitsLo);
    result.range(kLaneHitCountHi, kLaneHitCountLo) =
        cell.range(kCellHitCountHi, kCellHitCountLo);
    result.range(kLaneMaxHitsHi, kLaneMaxHitsLo) =
        cell.range(kCellMaxHitsHi, kCellMaxHitsLo);
    result[kLaneHitDropped] = cell[kCellHitDropped];
    result[kLaneReturnOverflow] = cell[kCellReturnOverflow];
    result[kLaneErrorFill] = cell[kCellErrorFill];
    result[kLaneFaulted] = cell[kCellFaulted];
    result.range(kLaneTimeoutHi, kLaneTimeoutLo) =
        cell.range(kCellTimeoutHi, kCellTimeoutLo);
    result.range(kLaneSequenceHi, kLaneSequenceLo) =
        cell.range(kCellChipShotSeqHi, kCellChipShotSeqLo);
    return result;
}

frame_cell_payload_t
make_frame_cell(const lane_cell_storage_t &stored, bool present,
                std::uint8_t chip, std::uint8_t stop, std::uint8_t slope,
                std::uint8_t slot, std::uint8_t slot_count,
                const context_storage_t &context, std::uint8_t missing_max_hits,
                std::uint16_t gap_before, bool line_faulted) {
    cell_payload_t cell = 0;
    cell.range(kCellKindHi, kCellKindLo) =
        static_cast<std::uint8_t>(cell_kind_t::data);
    cell.range(kCellChipHi, kCellChipLo) = chip;
    cell[kCellIfifo] = stop >= 4U;
    cell.range(kCellStopHi, kCellStopLo) = stop;
    cell[kCellSlope] = slope;
    cell.range(kCellShotContextHi, kCellShotContextLo) = context;

    if (present) {
        cell.range(kCellHitsHi, kCellHitsLo) =
            stored.range(kLaneHitsHi, kLaneHitsLo);
        cell.range(kCellHitCountHi, kCellHitCountLo) =
            stored.range(kLaneHitCountHi, kLaneHitCountLo);
        cell.range(kCellMaxHitsHi, kCellMaxHitsLo) =
            stored.range(kLaneMaxHitsHi, kLaneMaxHitsLo);
        cell[kCellHitDropped] = stored[kLaneHitDropped];
        cell[kCellReturnOverflow] = stored[kLaneReturnOverflow];
        cell[kCellErrorFill] = stored[kLaneErrorFill];
        cell[kCellFaulted] = stored[kLaneFaulted];
        cell.range(kCellTimeoutHi, kCellTimeoutLo) =
            stored.range(kLaneTimeoutHi, kLaneTimeoutLo);
        cell.range(kCellChipShotSeqHi, kCellChipShotSeqLo) =
            stored.range(kLaneSequenceHi, kLaneSequenceLo);
    } else {
        cell.range(kCellMaxHitsHi, kCellMaxHitsLo) = missing_max_hits;
        cell[kCellErrorFill] = 1;
        cell[kCellFaulted] = 1;
    }

    frame_cell_payload_t result = 0;
    result.range(kFrameCellCellHi, kFrameCellCellLo) = cell;
    result.range(kFrameCellSlotHi, kFrameCellSlotLo) = slot;
    result.range(kFrameCellSlotCountHi, kFrameCellSlotCountLo) = slot_count;
    result[kFrameCellLineStart] = slot == 0U;
    result[kFrameCellLineEnd] = slot + 1U == slot_count;
    result[kFrameCellFirstColumn] = context_shot_index(context) == 0U;
    result[kFrameCellLastColumn] = context_last_column(context);
    result.range(kFrameCellGapHi, kFrameCellGapLo) =
        slot == 0U ? gap_before : 0U;
    result[kFrameCellBlank] = !present;
    result[kFrameCellLineFaulted] =
        line_faulted || !present || cell[kCellFaulted] || cell[kCellErrorFill];
    return result;
}

void clear_shot_state() {
    shot_active = false;
    rise_present = 0U;
    fall_present = 0U;
    shot_terminal_mask = 0U;
    shot_faulted = false;
}

void clear_all_state() {
    clear_shot_state();
    history_valid = false;
    history_last = false;
}

frame_control_payload_t make_control(std::uint8_t faults) {
    frame_control_payload_t control = 0;
    control.range(7, 0) = faults;
    return control;
}

void process_face_close(const frame_input_payload_t &input,
                        frame_control_payload_t &control,
                        std::uint8_t num_faces, std::uint16_t active_version,
                        std::uint16_t columns_per_face) {
    frame_close_payload_t close = 0;
    close.range(kFaceCloseIdHi, kFaceCloseIdLo) =
        input.range(kFaceCloseIdHi, kFaceCloseIdLo);
    close.range(kFaceCloseFaceHi, kFaceCloseFaceLo) =
        input.range(kFaceCloseFaceHi, kFaceCloseFaceLo);
    close[kFaceCloseDirection] = input[kFaceCloseDirection];
    close[kFaceCloseSourceSim] = input[kFaceCloseSourceSim];
    close.range(kFaceCloseVersionHi, kFaceCloseVersionLo) =
        input.range(kFaceCloseVersionHi, kFaceCloseVersionLo);
    close.range(kFaceCloseColumnsHi, kFaceCloseColumnsLo) =
        input.range(kFaceCloseColumnsHi, kFaceCloseColumnsLo);

    const std::uint8_t face =
        input.range(kFaceCloseFaceHi, kFaceCloseFaceLo).to_uint();
    const bool direction = input[kFaceCloseDirection] != 0;
    const bool source_sim = input[kFaceCloseSourceSim] != 0;
    const std::uint16_t version =
        input.range(kFaceCloseVersionHi, kFaceCloseVersionLo).to_uint();
    const std::uint16_t columns =
        input.range(kFaceCloseColumnsHi, kFaceCloseColumnsLo).to_uint();

    bool geometry_fault = version != active_version ||
                          columns != columns_per_face || face >= num_faces ||
                          columns == 0U;
    bool all_hole = false;
    std::uint16_t trailing_gap = 0U;

    if (!history_valid) {
        all_hole = true;
        trailing_gap = columns;
    } else if (history_face == face && history_direction == direction &&
               history_source_sim == source_sim && history_version == version) {
        if (history_last) {
            trailing_gap = 0U;
        } else if (history_column < columns) {
            trailing_gap =
                static_cast<std::uint16_t>(columns - history_column - 1U);
        } else {
            geometry_fault = true;
        }
    } else {
        geometry_fault = true;
    }

    close.range(kFrameCloseTrailingHi, kFrameCloseTrailingLo) = trailing_gap;
    close[kFrameCloseAllHole] = all_hole;
    close[kFrameCloseFaulted] = geometry_fault;

    std::uint8_t faults = 0U;
    if (geometry_fault) {
        faults |= 1U << kFrameFaultGeometry;
    }
    if (trailing_gap != 0U) {
        faults |= 1U << kFrameFaultColumnGap;
    }
    control.range(7, 0) = faults;
    control[kFrameControlEmitClose] = 1;
    control.range(kFrameControlCloseHi, kFrameControlCloseLo) = close;

    history_valid = false;
    history_last = false;
}

} // namespace

extern "C" void gpx_frame_assembler_hls(
    hls::stream<lidar_v3::frame_input_payload_t> &event_in,
    hls::stream<lidar_v3::frame_cell_payload_t> &rise_out,
    hls::stream<lidar_v3::frame_cell_payload_t> &fall_out,
    hls::stream<lidar_v3::frame_control_payload_t> &control_out,
    std::uint8_t num_chips, std::uint8_t stops_per_chip, std::uint8_t num_faces,
    std::uint16_t active_version, std::uint8_t rise_mask,
    std::uint8_t fall_mask, std::uint16_t columns_per_face) {
#pragma HLS INTERFACE axis port=event_in
#pragma HLS INTERFACE axis port=rise_out
#pragma HLS INTERFACE axis port=fall_out
#pragma HLS INTERFACE axis port=control_out
#pragma HLS INTERFACE ap_none port=num_chips
#pragma HLS INTERFACE ap_none port=stops_per_chip
#pragma HLS INTERFACE ap_none port=num_faces
#pragma HLS INTERFACE ap_none port=active_version
#pragma HLS INTERFACE ap_none port=rise_mask
#pragma HLS INTERFACE ap_none port=fall_mask
#pragma HLS INTERFACE ap_none port=columns_per_face
#pragma HLS INTERFACE ap_ctrl_hs port=return
#pragma HLS BIND_STORAGE variable=rise_cells type=ram_2p impl=lutram
#pragma HLS BIND_STORAGE variable=fall_cells type=ram_2p impl=lutram
#pragma HLS ARRAY_PARTITION variable=chip_sequence complete dim=1

    using namespace lidar_v3;

    const frame_input_payload_t input = event_in.read();
    const std::uint8_t reset_epoch =
        input.range(kFrameInputResetEpochHi, kFrameInputResetEpochLo).to_uint();
    if (reset_epoch != accepted_reset_epoch) {
        clear_all_state();
        accepted_reset_epoch = reset_epoch;
    }

    frame_control_payload_t control = make_control(0U);
    if (input[kFrameInputKind] != 0) {
        process_face_close(input, control, num_faces, active_version,
                           columns_per_face);
    } else {
        const cell_payload_t cell =
            input.range(kFrameCellCellHi, kFrameCellCellLo);
        const std::uint8_t chip =
            cell.range(kCellChipHi, kCellChipLo).to_uint();
        const std::uint8_t stop =
            cell.range(kCellStopHi, kCellStopLo).to_uint();
        const std::uint8_t slope = cell[kCellSlope] ? 1U : 0U;
        const std::uint8_t kind =
            cell.range(kCellKindHi, kCellKindLo).to_uint();
        const context_storage_t context = cell_context(cell);
        const bool build_valid = num_chips >= 1U && num_chips <= kMaxChips &&
                                 stops_per_chip >= 1U &&
                                 stops_per_chip <= kMaxStopsPerChip &&
                                 num_faces >= 1U && num_faces <= kMaxFaces;

        std::uint8_t faults = 0U;
        bool accept_cell = true;
        if (!shot_active) {
            rise_present = 0U;
            fall_present = 0U;
            shot_terminal_mask = 0U;
            shot_active = true;
            shot_context = context;
            shot_max_hits =
                cell.range(kCellMaxHitsHi, kCellMaxHitsLo).to_uint();
            shot_rise_mask = rise_mask;
            shot_fall_mask = fall_mask;
            shot_columns = columns_per_face;
            shot_gap_before = 0U;
            shot_faulted = false;

            bool geometry_fault =
                !build_valid || (rise_mask | fall_mask) == 0U ||
                columns_per_face == 0U ||
                context_version(context) != active_version ||
                context_shot_index(context) >= columns_per_face;
            std::uint16_t gap = 0U;
            const std::uint16_t shot_index = context_shot_index(context);
            const std::uint8_t face = context_face(context);

            if (!history_valid || history_version != active_version ||
                history_face != face || history_last) {
                gap = shot_index;
                if (history_valid && !history_last &&
                    (history_version != active_version ||
                     history_face != face)) {
                    geometry_fault = true;
                }
            } else if (shot_index > history_column) {
                gap = static_cast<std::uint16_t>(shot_index - history_column -
                                                 1U);
            } else {
                gap = shot_index;
                geometry_fault = true;
            }

            if (columns_per_face != 0U) {
                const bool expected_last =
                    shot_index ==
                    static_cast<std::uint16_t>(columns_per_face - 1U);
                if (context_last_column(context) != expected_last) {
                    geometry_fault = true;
                }
            }

            shot_gap_before = gap;
            if (gap != 0U) {
                faults |= 1U << kFrameFaultColumnGap;
            }
            if (geometry_fault) {
                faults |= 1U << kFrameFaultGeometry;
                shot_faulted = true;
            }

            history_valid = true;
            history_face = face;
            history_direction = context_direction(context);
            history_source_sim = context[kContextSourceSim] != 0;
            history_version = context_version(context);
            history_column = shot_index;
            history_last = context_last_column(context);
        } else if (!contexts_equal(context, shot_context) ||
                   active_version != context_version(shot_context) ||
                   rise_mask != shot_rise_mask || fall_mask != shot_fall_mask ||
                   columns_per_face != shot_columns) {
            faults |= 1U << kFrameFaultContext;
            shot_faulted = true;
            accept_cell = false;
        }

        if (accept_cell &&
            (chip >= num_chips || chip >= kMaxChips || stop >= stops_per_chip ||
             stop >= kMaxStopsPerChip)) {
            faults |= 1U << kFrameFaultUnexpected;
            shot_faulted = true;
        } else if (accept_cell) {
            const std::uint8_t expected_mask =
                static_cast<std::uint8_t>(shot_rise_mask | shot_fall_mask);
            if (((expected_mask >> chip) & 0x1U) == 0U) {
                faults |= 1U << kFrameFaultUnexpected;
                shot_faulted = true;
            } else if (kind == static_cast<std::uint8_t>(cell_kind_t::data)) {
                const unsigned address = cell_address(chip, stop);
                const std::uint32_t bit = 1UL << address;
                if (slope == static_cast<std::uint8_t>(slope_t::rise) &&
                    ((shot_rise_mask >> chip) & 0x1U) != 0U) {
                    if ((rise_present & bit) != 0U) {
                        faults |= 1U << kFrameFaultDuplicateCell;
                        shot_faulted = true;
                    } else {
                        rise_cells[address] = pack_lane_cell(cell);
                        rise_present |= bit;
                    }
                } else if (slope == static_cast<std::uint8_t>(slope_t::fall) &&
                           ((shot_fall_mask >> chip) & 0x1U) != 0U) {
                    if ((fall_present & bit) != 0U) {
                        faults |= 1U << kFrameFaultDuplicateCell;
                        shot_faulted = true;
                    } else {
                        fall_cells[address] = pack_lane_cell(cell);
                        fall_present |= bit;
                    }
                } else if (cell.range(kCellHitCountHi, kCellHitCountLo) != 0U) {
                    faults |= 1U << kFrameFaultMaskedDrop;
                    shot_faulted = true;
                }

                if (cell[kCellFaulted] != 0 || cell[kCellErrorFill] != 0) {
                    shot_faulted = true;
                }
                chip_sequence[chip] =
                    cell.range(kCellChipShotSeqHi, kCellChipShotSeqLo)
                        .to_uint();
            } else if (kind ==
                           static_cast<std::uint8_t>(cell_kind_t::drain_done) ||
                       kind ==
                           static_cast<std::uint8_t>(cell_kind_t::timeout)) {
                const std::uint8_t bit = static_cast<std::uint8_t>(1U << chip);
                if ((shot_terminal_mask & bit) != 0U) {
                    faults |= 1U << kFrameFaultDuplicateTerminal;
                    shot_faulted = true;
                } else {
                    shot_terminal_mask =
                        static_cast<std::uint8_t>(shot_terminal_mask | bit);
                }
                chip_sequence[chip] =
                    cell.range(kCellChipShotSeqHi, kCellChipShotSeqLo)
                        .to_uint();
                if (cell[kCellFaulted] != 0 ||
                    kind == static_cast<std::uint8_t>(cell_kind_t::timeout)) {
                    shot_faulted = true;
                }
            }
        }

        const std::uint8_t expected_mask =
            static_cast<std::uint8_t>(shot_rise_mask | shot_fall_mask);
        const bool shot_complete =
            accept_cell && expected_mask != 0U &&
            (shot_terminal_mask & expected_mask) == expected_mask;

        if (shot_complete) {
            bool missing = false;
            for (unsigned active_chip = 0; active_chip < kMaxChips;
                 ++active_chip) {
#pragma HLS UNROLL
                for (unsigned active_stop = 0; active_stop < kMaxStopsPerChip;
                     ++active_stop) {
#pragma HLS UNROLL
                    if (active_stop < stops_per_chip) {
                        const unsigned address =
                            active_chip * kMaxStopsPerChip + active_stop;
                        const std::uint32_t bit = 1UL << address;
                        if (((shot_rise_mask >> active_chip) & 0x1U) != 0U &&
                            (rise_present & bit) == 0U) {
                            missing = true;
                        }
                        if (((shot_fall_mask >> active_chip) & 0x1U) != 0U &&
                            (fall_present & bit) == 0U) {
                            missing = true;
                        }
                    }
                }
            }
            if (missing) {
                faults |= 1U << kFrameFaultMissingCell;
                shot_faulted = true;
            }

            const std::uint8_t rise_count = static_cast<std::uint8_t>(
                popcount4(shot_rise_mask) * stops_per_chip);
            const std::uint8_t fall_count = static_cast<std::uint8_t>(
                popcount4(shot_fall_mask) * stops_per_chip);
            const std::uint8_t max_count =
                rise_count > fall_count ? rise_count : fall_count;

            for (unsigned slot = 0; slot < kCellsPerSlope; ++slot) {
#pragma HLS PIPELINE II=1
                if (slot < max_count) {
                    if (slot < rise_count) {
                        const slot_address_t address =
                            slot_address(shot_rise_mask, stops_per_chip,
                                         static_cast<std::uint8_t>(slot));
                        const unsigned linear =
                            cell_address(address.chip, address.stop);
                        const bool present =
                            (rise_present & (1UL << linear)) != 0U;
                        rise_out.write(make_frame_cell(
                            rise_cells[linear], present, address.chip,
                            address.stop,
                            static_cast<std::uint8_t>(slope_t::rise),
                            static_cast<std::uint8_t>(slot), rise_count,
                            shot_context, shot_max_hits, shot_gap_before,
                            shot_faulted));
                    }
                    if (slot < fall_count) {
                        const slot_address_t address =
                            slot_address(shot_fall_mask, stops_per_chip,
                                         static_cast<std::uint8_t>(slot));
                        const unsigned linear =
                            cell_address(address.chip, address.stop);
                        const bool present =
                            (fall_present & (1UL << linear)) != 0U;
                        fall_out.write(make_frame_cell(
                            fall_cells[linear], present, address.chip,
                            address.stop,
                            static_cast<std::uint8_t>(slope_t::fall),
                            static_cast<std::uint8_t>(slot), fall_count,
                            shot_context, shot_max_hits, shot_gap_before,
                            shot_faulted));
                    }
                }
            }

            control[kFrameControlShotDone] = 1;
            control.range(kFrameControlContextHi, kFrameControlContextLo) =
                shot_context;
            clear_shot_state();
        }

        control.range(7, 0) = faults;
    }

    // Every accepted input event has exactly one control result. Keeping this
    // as one unconditional write avoids a wide, condition-selected AXIS output
    // enable path in the generated RTL.
    control_out.write(control);
}
