#include "gpx_frame_assembler_hls.hpp"

#include <cstdint>

namespace {

namespace h1 = lidar_v3::h1;
namespace h2 = lidar_v3::h2;
namespace h3 = lidar_v3::h3;
namespace limits = lidar_v3::limits;

constexpr unsigned kMaximumCellSlotsPerSlope =
    limits::kMaximumTdcGpxChipCount *
    limits::kMaximumStopChannelsPerChip;
using context_storage_t = h1::shot_context_t;

static h3::lane_cell_storage_t rise_cells[kMaximumCellSlotsPerSlope];
static h3::lane_cell_storage_t fall_cells[kMaximumCellSlotsPerSlope];
static std::uint32_t rise_present;
static std::uint32_t fall_present;

static bool shot_active;
static context_storage_t shot_context;
static std::uint8_t shot_serialized_return_slots;
static std::uint8_t shot_rise_mask;
static std::uint8_t shot_fall_mask;
static std::uint8_t shot_terminal_mask;
static std::uint16_t shot_columns;
static std::uint16_t shot_gap_before;
static bool shot_faulted;
static std::uint16_t chip_sequence[limits::kMaximumTdcGpxChipCount];

static bool history_valid;
static std::uint8_t history_face;
static bool history_direction;
static bool history_source_sim;
static std::uint16_t history_version;
static std::uint16_t history_column;
static bool history_last;

static std::uint8_t accepted_reset_epoch;

unsigned cell_address(std::uint8_t chip, std::uint8_t stop) {
    return static_cast<unsigned>(chip) *
               limits::kMaximumStopChannelsPerChip +
           stop;
}

std::uint8_t popcount4(std::uint8_t mask) {
    std::uint8_t count = 0;
    for (unsigned bit = 0; bit < limits::kMaximumTdcGpxChipCount; ++bit) {
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
    for (unsigned chip = 0;
         chip < limits::kMaximumTdcGpxChipCount;
         ++chip) {
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

context_storage_t cell_context(const h2::cell_event_record_t &cell) {
    return lidar_v3::read_field<h2::cell_event_layout::shot_context>(cell);
}

std::uint16_t context_shot_index(const context_storage_t &context) {
    return lidar_v3::read_field<h1::shot_context_layout::shot_column_index>(
               context)
        .to_uint();
}

std::uint8_t context_face(const context_storage_t &context) {
    return lidar_v3::read_field<h1::shot_context_layout::mirror_face_index>(
               context)
        .to_uint();
}

std::uint16_t context_version(const context_storage_t &context) {
    return lidar_v3::read_field<
               h1::shot_context_layout::active_configuration_version>(context)
        .to_uint();
}

bool context_direction(const context_storage_t &context) {
    return lidar_v3::read_flag<h1::shot_context_layout::direction_is_ccw>(
        context);
}

bool context_last_column(const context_storage_t &context) {
    return lidar_v3::read_flag<
        h1::shot_context_layout::is_last_shot_column_in_face>(context);
}

bool contexts_equal(const context_storage_t &left,
                    const context_storage_t &right) {
    // A direct 162-bit operator!= becomes a long subtract/carry comparator on
    // 7-series devices. Parallel 32-bit XOR groups preserve exact equality of
    // every packed bit while keeping the result on a shallow reduction tree.
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

h3::lane_cell_storage_t pack_lane_cell(
    const h2::cell_event_record_t &cell) {
    h3::lane_cell_storage_t result = 0;
    lidar_v3::write_field<
        h3::lane_cell_storage_layout::packed_distance_hits_17bit>(
        result,
        lidar_v3::read_field<
            h2::cell_event_layout::packed_distance_hits_17bit>(cell));
    lidar_v3::write_field<h3::lane_cell_storage_layout::visible_return_count>(
        result,
        lidar_v3::read_field<h2::cell_event_layout::visible_return_count>(
            cell));
    lidar_v3::write_field<
        h3::lane_cell_storage_layout::serialized_return_slot_count>(
        result,
        lidar_v3::read_field<
            h2::cell_event_layout::serialized_return_slot_count>(cell));
    lidar_v3::write_flag<h3::lane_cell_storage_layout::hit_was_dropped>(
        result,
        lidar_v3::read_flag<h2::cell_event_layout::hit_was_dropped>(cell));
    lidar_v3::write_flag<h3::lane_cell_storage_layout::return_overflow>(
        result,
        lidar_v3::read_flag<h2::cell_event_layout::return_overflow>(cell));
    lidar_v3::write_flag<h3::lane_cell_storage_layout::error_fill_inserted>(
        result,
        lidar_v3::read_flag<h2::cell_event_layout::error_fill_inserted>(cell));
    lidar_v3::write_flag<h3::lane_cell_storage_layout::cell_is_faulted>(
        result,
        lidar_v3::read_flag<h2::cell_event_layout::cell_is_faulted>(cell));
    lidar_v3::write_field<h3::lane_cell_storage_layout::timeout_cause_bitmap>(
        result,
        lidar_v3::read_field<h2::cell_event_layout::timeout_cause_bitmap>(
            cell));
    lidar_v3::write_field<
        h3::lane_cell_storage_layout::tdc_chip_shot_sequence>(
        result,
        lidar_v3::read_field<
            h2::cell_event_layout::tdc_chip_shot_sequence>(cell));
    return result;
}

h3::ordered_lane_cell_axis_t
make_frame_cell(const h3::lane_cell_storage_t &stored, bool present,
                std::uint8_t chip, std::uint8_t stop, std::uint8_t slope,
                std::uint8_t slot, std::uint8_t slot_count,
                const context_storage_t &context,
                std::uint8_t missing_serialized_return_slots,
                std::uint16_t gap_before, bool line_faulted) {
    h2::cell_event_record_t cell = 0;
    lidar_v3::write_field<h2::cell_event_layout::event_kind>(
        cell, static_cast<std::uint8_t>(h2::cell_event_kind_t::data));
    lidar_v3::write_field<h2::cell_event_layout::tdc_chip_index>(cell, chip);
    lidar_v3::write_flag<h2::cell_event_layout::ififo_bank_select>(
        cell, stop >= 4U);
    lidar_v3::write_field<h2::cell_event_layout::logical_stop_channel_index>(
        cell, stop);
    lidar_v3::write_flag<h2::cell_event_layout::edge_slope_is_rise>(
        cell, slope != 0U);
    lidar_v3::write_field<h2::cell_event_layout::shot_context>(cell, context);

    if (present) {
        lidar_v3::write_field<
            h2::cell_event_layout::packed_distance_hits_17bit>(
            cell,
            lidar_v3::read_field<
                h3::lane_cell_storage_layout::packed_distance_hits_17bit>(
                stored));
        lidar_v3::write_field<h2::cell_event_layout::visible_return_count>(
            cell,
            lidar_v3::read_field<
                h3::lane_cell_storage_layout::visible_return_count>(stored));
        lidar_v3::write_field<
            h2::cell_event_layout::serialized_return_slot_count>(
            cell,
            lidar_v3::read_field<
                h3::lane_cell_storage_layout::serialized_return_slot_count>(
                stored));
        lidar_v3::write_flag<h2::cell_event_layout::hit_was_dropped>(
            cell,
            lidar_v3::read_flag<
                h3::lane_cell_storage_layout::hit_was_dropped>(stored));
        lidar_v3::write_flag<h2::cell_event_layout::return_overflow>(
            cell,
            lidar_v3::read_flag<
                h3::lane_cell_storage_layout::return_overflow>(stored));
        lidar_v3::write_flag<h2::cell_event_layout::error_fill_inserted>(
            cell,
            lidar_v3::read_flag<
                h3::lane_cell_storage_layout::error_fill_inserted>(stored));
        lidar_v3::write_flag<h2::cell_event_layout::cell_is_faulted>(
            cell,
            lidar_v3::read_flag<h3::lane_cell_storage_layout::cell_is_faulted>(
                stored));
        lidar_v3::write_field<h2::cell_event_layout::timeout_cause_bitmap>(
            cell,
            lidar_v3::read_field<
                h3::lane_cell_storage_layout::timeout_cause_bitmap>(stored));
        lidar_v3::write_field<
            h2::cell_event_layout::tdc_chip_shot_sequence>(
            cell,
            lidar_v3::read_field<
                h3::lane_cell_storage_layout::tdc_chip_shot_sequence>(stored));
    } else {
        lidar_v3::write_field<
            h2::cell_event_layout::serialized_return_slot_count>(
            cell, missing_serialized_return_slots);
        lidar_v3::write_flag<h2::cell_event_layout::error_fill_inserted>(
            cell, true);
        lidar_v3::write_flag<h2::cell_event_layout::cell_is_faulted>(
            cell, true);
    }

    h3::ordered_lane_cell_axis_t result = 0;
    lidar_v3::write_field<h3::ordered_lane_cell_layout::cell_event>(
        result, cell);
    lidar_v3::write_field<h3::ordered_lane_cell_layout::lane_cell_slot_index>(
        result, slot);
    lidar_v3::write_field<h3::ordered_lane_cell_layout::lane_cell_slot_count>(
        result, slot_count);
    lidar_v3::write_flag<
        h3::ordered_lane_cell_layout::is_first_cell_in_shot_line>(
        result, slot == 0U);
    lidar_v3::write_flag<
        h3::ordered_lane_cell_layout::is_last_cell_in_shot_line>(
        result, slot + 1U == slot_count);
    lidar_v3::write_flag<
        h3::ordered_lane_cell_layout::is_first_shot_column_in_face>(
        result, context_shot_index(context) == 0U);
    lidar_v3::write_flag<
        h3::ordered_lane_cell_layout::is_last_shot_column_in_face>(
        result, context_last_column(context));
    lidar_v3::write_field<
        h3::ordered_lane_cell_layout::missing_shot_columns_before>(
        result, slot == 0U ? gap_before : 0U);
    lidar_v3::write_flag<
        h3::ordered_lane_cell_layout::is_missing_cell_placeholder>(
        result, !present);
    lidar_v3::write_flag<h3::ordered_lane_cell_layout::shot_line_is_faulted>(
        result,
        line_faulted || !present ||
            lidar_v3::read_flag<h2::cell_event_layout::cell_is_faulted>(cell) ||
            lidar_v3::read_flag<
                h2::cell_event_layout::error_fill_inserted>(cell));
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

h3::assembler_control_axis_t make_control(std::uint8_t faults) {
    h3::assembler_control_axis_t control = 0;
    lidar_v3::write_field<h3::assembler_control_layout::fault_event_bitmap>(
        control, faults);
    return control;
}

void process_face_close(const h3::assembler_input_axis_t &input,
                        h3::assembler_control_axis_t &control,
                        std::uint8_t num_faces, std::uint16_t active_version,
                        std::uint16_t columns_per_face) {
    const h3::face_close_event_record_t close_event =
        h3::read_assembler_input_face_close_event(input);
    h3::face_close_result_t close_result = 0;
    lidar_v3::write_field<h3::face_close_result_layout::close_event>(
        close_result, close_event);

    const std::uint8_t face =
        lidar_v3::read_field<h3::face_close_event_layout::mirror_face_index>(
            close_event)
            .to_uint();
    const bool direction =
        lidar_v3::read_flag<h3::face_close_event_layout::direction_is_ccw>(
            close_event);
    const bool source_sim =
        lidar_v3::read_flag<h3::face_close_event_layout::source_is_simulation>(
            close_event);
    const std::uint16_t version =
        lidar_v3::read_field<
            h3::face_close_event_layout::active_configuration_version>(
            close_event)
            .to_uint();
    const std::uint16_t columns =
        lidar_v3::read_field<
            h3::face_close_event_layout::expected_shot_column_count>(
            close_event)
            .to_uint();

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

    lidar_v3::write_field<
        h3::face_close_result_layout::trailing_missing_shot_columns>(
        close_result, trailing_gap);
    lidar_v3::write_flag<
        h3::face_close_result_layout::entire_face_is_missing>(
        close_result, all_hole);
    lidar_v3::write_flag<h3::face_close_result_layout::face_close_is_faulted>(
        close_result, geometry_fault);

    std::uint8_t faults = 0U;
    if (geometry_fault) {
        faults |= 1U << static_cast<unsigned>(
            h3::assembler_fault_bit_t::geometry_mismatch);
    }
    if (trailing_gap != 0U) {
        faults |= 1U << static_cast<unsigned>(
            h3::assembler_fault_bit_t::shot_column_gap);
    }
    lidar_v3::write_field<h3::assembler_control_layout::fault_event_bitmap>(
        control, faults);
    lidar_v3::write_flag<
        h3::assembler_control_layout::contains_face_close_result>(
        control, true);
    lidar_v3::write_field<h3::assembler_control_layout::face_close_result>(
        control, close_result);

    history_valid = false;
    history_last = false;
}

} // namespace

extern "C" void gpx_frame_assembler_hls(
    hls::stream<lidar_v3::h3::assembler_input_axis_t>
        &cell_or_face_close_event_in,
    hls::stream<lidar_v3::h3::ordered_lane_cell_axis_t>
        &ordered_rise_cell_out,
    hls::stream<lidar_v3::h3::ordered_lane_cell_axis_t>
        &ordered_fall_cell_out,
    hls::stream<lidar_v3::h3::assembler_control_axis_t>
        &assembler_control_out,
    std::uint8_t build_tdc_chip_count,
    std::uint8_t build_stop_channels_per_chip,
    std::uint8_t build_mirror_face_count,
    std::uint16_t active_configuration_version,
    std::uint8_t runtime_enabled_rise_chip_mask,
    std::uint8_t runtime_enabled_fall_chip_mask,
    std::uint16_t runtime_expected_shot_columns_per_face) {
#pragma HLS INTERFACE axis port=cell_or_face_close_event_in
#pragma HLS INTERFACE axis port=ordered_rise_cell_out
#pragma HLS INTERFACE axis port=ordered_fall_cell_out
#pragma HLS INTERFACE axis port=assembler_control_out
#pragma HLS INTERFACE ap_none port=build_tdc_chip_count
#pragma HLS INTERFACE ap_none port=build_stop_channels_per_chip
#pragma HLS INTERFACE ap_none port=build_mirror_face_count
#pragma HLS INTERFACE ap_none port=active_configuration_version
#pragma HLS INTERFACE ap_none port=runtime_enabled_rise_chip_mask
#pragma HLS INTERFACE ap_none port=runtime_enabled_fall_chip_mask
#pragma HLS INTERFACE ap_none port=runtime_expected_shot_columns_per_face
#pragma HLS INTERFACE ap_ctrl_hs port=return
#pragma HLS BIND_STORAGE variable=rise_cells type=ram_2p impl=lutram
#pragma HLS BIND_STORAGE variable=fall_cells type=ram_2p impl=lutram
#pragma HLS ARRAY_PARTITION variable=chip_sequence complete dim=1

    namespace h1 = lidar_v3::h1;
    namespace h2 = lidar_v3::h2;
    namespace h3 = lidar_v3::h3;
    namespace limits = lidar_v3::limits;

    const h3::assembler_input_axis_t input =
        cell_or_face_close_event_in.read();
    const std::uint8_t reset_epoch =
        lidar_v3::read_field<h3::assembler_input_layout::reset_epoch>(input)
            .to_uint();
    if (reset_epoch != accepted_reset_epoch) {
        clear_all_state();
        accepted_reset_epoch = reset_epoch;
    }

    h3::assembler_control_axis_t control = make_control(0U);
    if (h3::read_assembler_input_kind(input) ==
        h3::assembler_input_kind_t::face_close_event) {
        process_face_close(
            input,
            control,
            build_mirror_face_count,
            active_configuration_version,
            runtime_expected_shot_columns_per_face);
    } else {
        const h2::cell_event_record_t cell =
            h3::read_assembler_input_cell_event(input);
        const std::uint8_t chip =
            lidar_v3::read_field<h2::cell_event_layout::tdc_chip_index>(cell)
                .to_uint();
        const std::uint8_t stop =
            lidar_v3::read_field<
                h2::cell_event_layout::logical_stop_channel_index>(cell)
                .to_uint();
        const bool slope_is_rise =
            lidar_v3::read_flag<h2::cell_event_layout::edge_slope_is_rise>(
                cell);
        const std::uint8_t kind =
            lidar_v3::read_field<h2::cell_event_layout::event_kind>(cell)
                .to_uint();
        const context_storage_t context = cell_context(cell);
        const bool build_valid =
            build_tdc_chip_count >= 1U &&
            build_tdc_chip_count <= limits::kMaximumTdcGpxChipCount &&
            build_stop_channels_per_chip >= 1U &&
            build_stop_channels_per_chip <=
                limits::kMaximumStopChannelsPerChip &&
            build_mirror_face_count >= 1U &&
            build_mirror_face_count <= limits::kMaximumMirrorFaceCount;

        std::uint8_t faults = 0U;
        bool accept_cell = true;
        if (!shot_active) {
            rise_present = 0U;
            fall_present = 0U;
            shot_terminal_mask = 0U;
            shot_active = true;
            shot_context = context;
            shot_serialized_return_slots =
                lidar_v3::read_field<
                    h2::cell_event_layout::serialized_return_slot_count>(cell)
                    .to_uint();
            shot_rise_mask = runtime_enabled_rise_chip_mask;
            shot_fall_mask = runtime_enabled_fall_chip_mask;
            shot_columns = runtime_expected_shot_columns_per_face;
            shot_gap_before = 0U;
            shot_faulted = false;

            bool geometry_fault =
                !build_valid ||
                (runtime_enabled_rise_chip_mask |
                 runtime_enabled_fall_chip_mask) == 0U ||
                runtime_expected_shot_columns_per_face == 0U ||
                context_version(context) != active_configuration_version ||
                context_shot_index(context) >=
                    runtime_expected_shot_columns_per_face;
            std::uint16_t gap = 0U;
            const std::uint16_t shot_index = context_shot_index(context);
            const std::uint8_t face = context_face(context);

            if (!history_valid ||
                history_version != active_configuration_version ||
                history_face != face || history_last) {
                gap = shot_index;
                if (history_valid && !history_last &&
                    (history_version != active_configuration_version ||
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

            if (runtime_expected_shot_columns_per_face != 0U) {
                const bool expected_last =
                    shot_index ==
                    static_cast<std::uint16_t>(
                        runtime_expected_shot_columns_per_face - 1U);
                if (context_last_column(context) != expected_last) {
                    geometry_fault = true;
                }
            }

            shot_gap_before = gap;
            if (gap != 0U) {
                faults |= 1U << static_cast<unsigned>(
                    h3::assembler_fault_bit_t::shot_column_gap);
            }
            if (geometry_fault) {
                faults |= 1U << static_cast<unsigned>(
                    h3::assembler_fault_bit_t::geometry_mismatch);
                shot_faulted = true;
            }

            history_valid = true;
            history_face = face;
            history_direction = context_direction(context);
            history_source_sim =
                lidar_v3::read_flag<
                    h1::shot_context_layout::source_is_simulation>(context);
            history_version = context_version(context);
            history_column = shot_index;
            history_last = context_last_column(context);
        } else if (!contexts_equal(context, shot_context) ||
                   active_configuration_version !=
                       context_version(shot_context) ||
                   runtime_enabled_rise_chip_mask != shot_rise_mask ||
                   runtime_enabled_fall_chip_mask != shot_fall_mask ||
                   runtime_expected_shot_columns_per_face != shot_columns) {
            faults |= 1U << static_cast<unsigned>(
                h3::assembler_fault_bit_t::shot_context_mismatch);
            shot_faulted = true;
            accept_cell = false;
        }

        if (accept_cell &&
            (chip >= build_tdc_chip_count ||
             chip >= limits::kMaximumTdcGpxChipCount ||
             stop >= build_stop_channels_per_chip ||
             stop >= limits::kMaximumStopChannelsPerChip)) {
            faults |= 1U << static_cast<unsigned>(
                h3::assembler_fault_bit_t::unexpected_event);
            shot_faulted = true;
        } else if (accept_cell) {
            const std::uint8_t expected_mask =
                static_cast<std::uint8_t>(shot_rise_mask | shot_fall_mask);
            if (((expected_mask >> chip) & 0x1U) == 0U) {
                faults |= 1U << static_cast<unsigned>(
                    h3::assembler_fault_bit_t::unexpected_event);
                shot_faulted = true;
            } else if (kind == static_cast<std::uint8_t>(
                                   h2::cell_event_kind_t::data)) {
                const unsigned address = cell_address(chip, stop);
                const std::uint32_t bit = 1UL << address;
                if (slope_is_rise &&
                    ((shot_rise_mask >> chip) & 0x1U) != 0U) {
                    if ((rise_present & bit) != 0U) {
                        faults |= 1U << static_cast<unsigned>(
                            h3::assembler_fault_bit_t::duplicate_cell);
                        shot_faulted = true;
                    } else {
                        rise_cells[address] = pack_lane_cell(cell);
                        rise_present |= bit;
                    }
                } else if (!slope_is_rise &&
                           ((shot_fall_mask >> chip) & 0x1U) != 0U) {
                    if ((fall_present & bit) != 0U) {
                        faults |= 1U << static_cast<unsigned>(
                            h3::assembler_fault_bit_t::duplicate_cell);
                        shot_faulted = true;
                    } else {
                        fall_cells[address] = pack_lane_cell(cell);
                        fall_present |= bit;
                    }
                } else if (lidar_v3::read_field<
                               h2::cell_event_layout::visible_return_count>(
                               cell) != 0U) {
                    faults |= 1U << static_cast<unsigned>(
                        h3::assembler_fault_bit_t::masked_lane_drop);
                    shot_faulted = true;
                }

                if (lidar_v3::read_flag<
                        h2::cell_event_layout::cell_is_faulted>(cell) ||
                    lidar_v3::read_flag<
                        h2::cell_event_layout::error_fill_inserted>(cell)) {
                    shot_faulted = true;
                }
                chip_sequence[chip] =
                    lidar_v3::read_field<
                        h2::cell_event_layout::tdc_chip_shot_sequence>(cell)
                        .to_uint();
            } else if (kind ==
                           static_cast<std::uint8_t>(
                               h2::cell_event_kind_t::drain_done) ||
                       kind ==
                           static_cast<std::uint8_t>(
                               h2::cell_event_kind_t::timeout)) {
                const std::uint8_t bit = static_cast<std::uint8_t>(1U << chip);
                if ((shot_terminal_mask & bit) != 0U) {
                    faults |= 1U << static_cast<unsigned>(
                        h3::assembler_fault_bit_t::duplicate_terminal_event);
                    shot_faulted = true;
                } else {
                    shot_terminal_mask =
                        static_cast<std::uint8_t>(shot_terminal_mask | bit);
                }
                chip_sequence[chip] =
                    lidar_v3::read_field<
                        h2::cell_event_layout::tdc_chip_shot_sequence>(cell)
                        .to_uint();
                if (lidar_v3::read_flag<
                        h2::cell_event_layout::cell_is_faulted>(cell) ||
                    kind == static_cast<std::uint8_t>(
                                h2::cell_event_kind_t::timeout)) {
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
            for (unsigned active_chip = 0;
                 active_chip < limits::kMaximumTdcGpxChipCount;
                 ++active_chip) {
#pragma HLS UNROLL
                for (unsigned active_stop = 0;
                     active_stop < limits::kMaximumStopChannelsPerChip;
                     ++active_stop) {
#pragma HLS UNROLL
                    if (active_stop < build_stop_channels_per_chip) {
                        const unsigned address =
                            active_chip *
                                limits::kMaximumStopChannelsPerChip +
                            active_stop;
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
                faults |= 1U << static_cast<unsigned>(
                    h3::assembler_fault_bit_t::missing_cell);
                shot_faulted = true;
            }

            const std::uint8_t rise_count = static_cast<std::uint8_t>(
                popcount4(shot_rise_mask) * build_stop_channels_per_chip);
            const std::uint8_t fall_count = static_cast<std::uint8_t>(
                popcount4(shot_fall_mask) * build_stop_channels_per_chip);
            const std::uint8_t max_count =
                rise_count > fall_count ? rise_count : fall_count;

            for (unsigned slot = 0;
                 slot < kMaximumCellSlotsPerSlope;
                 ++slot) {
#pragma HLS PIPELINE II=1
                if (slot < max_count) {
                    if (slot < rise_count) {
                        const slot_address_t address =
                            slot_address(shot_rise_mask,
                                         build_stop_channels_per_chip,
                                         static_cast<std::uint8_t>(slot));
                        const unsigned linear =
                            cell_address(address.chip, address.stop);
                        const bool present =
                            (rise_present & (1UL << linear)) != 0U;
                        ordered_rise_cell_out.write(make_frame_cell(
                            rise_cells[linear], present, address.chip,
                            address.stop,
                            static_cast<std::uint8_t>(
                                lidar_v3::tdc_edge_slope_t::rise),
                            static_cast<std::uint8_t>(slot), rise_count,
                            shot_context, shot_serialized_return_slots,
                            shot_gap_before,
                            shot_faulted));
                    }
                    if (slot < fall_count) {
                        const slot_address_t address =
                            slot_address(shot_fall_mask,
                                         build_stop_channels_per_chip,
                                         static_cast<std::uint8_t>(slot));
                        const unsigned linear =
                            cell_address(address.chip, address.stop);
                        const bool present =
                            (fall_present & (1UL << linear)) != 0U;
                        ordered_fall_cell_out.write(make_frame_cell(
                            fall_cells[linear], present, address.chip,
                            address.stop,
                            static_cast<std::uint8_t>(
                                lidar_v3::tdc_edge_slope_t::fall),
                            static_cast<std::uint8_t>(slot), fall_count,
                            shot_context, shot_serialized_return_slots,
                            shot_gap_before,
                            shot_faulted));
                    }
                }
            }

            lidar_v3::write_flag<
                h3::assembler_control_layout::shot_cell_generation_complete>(
                control, true);
            lidar_v3::write_field<
                h3::assembler_control_layout::completed_shot_context>(
                control, shot_context);
            clear_shot_state();
        }

        lidar_v3::write_field<
            h3::assembler_control_layout::fault_event_bitmap>(
            control, faults);
    }

    // Every accepted input event has exactly one control result. Keeping this
    // as one unconditional write avoids a wide, condition-selected AXIS output
    // enable path in the generated RTL.
    assembler_control_out.write(control);
}
