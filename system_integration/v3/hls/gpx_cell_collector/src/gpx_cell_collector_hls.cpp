#include "gpx_cell_collector_hls.hpp"

#include <cstdint>

namespace {

namespace h1 = lidar_v3::h1;
namespace h2 = lidar_v3::h2;
namespace limits = lidar_v3::limits;

constexpr unsigned kSlopeLanesPerChip = 2U;
constexpr unsigned kCellSlotsPerChip =
    kSlopeLanesPerChip * limits::kMaximumStopChannelsPerChip;
constexpr unsigned kTotalCellSlotCapacity =
    limits::kMaximumTdcGpxChipCount * kCellSlotsPerChip;

// ap_uint is limited to physical packed storage and RTL/HLS boundaries.
// Control arithmetic uses standard fixed-width C++ integer types.
using distance_hit_storage_t = ap_uint<17>;
using shot_context_storage_t = h1::shot_context_t;

static distance_hit_storage_t
    distance_hit_banks[limits::kMaximumReturnCountPerStop]
                      [kTotalCellSlotCapacity];
static std::uint8_t received_return_count[kTotalCellSlotCapacity];
static bool return_overflow[kTotalCellSlotCapacity];
static bool cell_error[kTotalCellSlotCapacity];

static bool shot_is_active[limits::kMaximumTdcGpxChipCount];
static shot_context_storage_t
    owner_shot_context[limits::kMaximumTdcGpxChipCount];
static std::uint16_t owner_chip_shot_sequence[
    limits::kMaximumTdcGpxChipCount];
static std::uint8_t owner_visible_return_count[
    limits::kMaximumTdcGpxChipCount];
static bool owner_shot_is_faulted[limits::kMaximumTdcGpxChipCount];
static bool lower_ififo_cells_were_emitted[
    limits::kMaximumTdcGpxChipCount];
static bool owner_rise_slope_is_enabled[
    limits::kMaximumTdcGpxChipCount];
static bool owner_fall_slope_is_enabled[
    limits::kMaximumTdcGpxChipCount];
static std::uint8_t accepted_reset_epoch;

unsigned calculate_cell_slot_address(
    std::uint8_t tdc_chip_index,
    bool edge_slope_is_rise,
    std::uint8_t logical_stop_channel_index) {
    const unsigned slope_slot_offset =
        edge_slope_is_rise ? 0U : limits::kMaximumStopChannelsPerChip;
    return static_cast<unsigned>(tdc_chip_index) * kCellSlotsPerChip +
           slope_slot_offset + logical_stop_channel_index;
}

std::uint8_t calculate_effective_visible_return_count(
    std::uint8_t runtime_requested_count,
    std::uint8_t build_capacity) {
    const std::uint8_t bounded_build_capacity =
        build_capacity < 1U
            ? 1U
            : (build_capacity > limits::kMaximumReturnCountPerStop
                   ? static_cast<std::uint8_t>(
                         limits::kMaximumReturnCountPerStop)
                   : build_capacity);
    if (runtime_requested_count == 0U) {
        return 1U;
    }
    return runtime_requested_count > bounded_build_capacity
               ? bounded_build_capacity
               : runtime_requested_count;
}

shot_context_storage_t read_hit_shot_context(
    const h1::decoded_hit_event_t &decoded_hit_event) {
    return lidar_v3::read_field<h1::decoded_hit_event_layout::shot_context>(
        decoded_hit_event);
}

bool shot_identity_matches(
    const shot_context_storage_t &candidate_context,
    std::uint16_t candidate_chip_shot_sequence,
    const shot_context_storage_t &owner_context,
    std::uint16_t owner_sequence) {
    return candidate_chip_shot_sequence == owner_sequence &&
           lidar_v3::read_field<
               h1::shot_context_layout::active_configuration_version>(
               candidate_context) ==
               lidar_v3::read_field<
                   h1::shot_context_layout::active_configuration_version>(
                   owner_context) &&
           lidar_v3::read_field<h1::shot_context_layout::shot_column_index>(
               candidate_context) ==
               lidar_v3::read_field<
                   h1::shot_context_layout::shot_column_index>(owner_context) &&
           lidar_v3::read_field<h1::shot_context_layout::mirror_face_index>(
               candidate_context) ==
               lidar_v3::read_field<
                   h1::shot_context_layout::mirror_face_index>(owner_context) &&
           lidar_v3::read_flag<h1::shot_context_layout::source_is_simulation>(
               candidate_context) ==
               lidar_v3::read_flag<
                   h1::shot_context_layout::source_is_simulation>(owner_context);
}

bool edge_slope_is_supported(
    bool edge_slope_is_rise,
    bool rise_slope_is_enabled,
    bool fall_slope_is_enabled) {
    return edge_slope_is_rise
               ? rise_slope_is_enabled
               : fall_slope_is_enabled;
}

h2::collector_result_axis_t build_status_result(
    bool shot_context_fault,
    bool return_overflow_fault,
    bool nonzero_start_number_fault) {
    h2::collector_result_axis_t result = 0;
    lidar_v3::write_flag<h2::collector_result_layout::shot_context_fault>(
        result, shot_context_fault);
    lidar_v3::write_flag<h2::collector_result_layout::return_overflow_fault>(
        result, return_overflow_fault);
    lidar_v3::write_flag<
        h2::collector_result_layout::nonzero_start_number_fault>(
        result, nonzero_start_number_fault);
    // Runtime visible Return filtering is intentional and never asserts the
    // unintended internal-capacity-loss diagnostic.
    lidar_v3::write_flag<
        h2::collector_result_layout::return_capacity_drop_fault>(
        result, false);
    return result;
}

h2::cell_event_record_t build_data_cell_event(
    std::uint8_t tdc_chip_index,
    std::uint8_t logical_stop_channel_index,
    bool edge_slope_is_rise,
    std::uint8_t visible_return_count,
    bool insert_error_fill,
    bool shot_is_faulted,
    std::uint8_t timeout_cause_bitmap,
    const shot_context_storage_t &shot_context,
    std::uint16_t tdc_chip_shot_sequence) {
    h2::cell_event_record_t cell_event = 0;
    const unsigned cell_slot_address = calculate_cell_slot_address(
        tdc_chip_index,
        edge_slope_is_rise,
        logical_stop_channel_index);
    const std::uint8_t emitted_return_count =
        received_return_count[cell_slot_address] > visible_return_count
            ? visible_return_count
            : received_return_count[cell_slot_address];

    lidar_v3::write_field<h2::cell_event_layout::event_kind>(
        cell_event, static_cast<std::uint8_t>(h2::cell_event_kind_t::data));
    lidar_v3::write_field<h2::cell_event_layout::tdc_chip_index>(
        cell_event, tdc_chip_index);
    lidar_v3::write_flag<h2::cell_event_layout::ififo_bank_select>(
        cell_event, logical_stop_channel_index >= 4U);
    lidar_v3::write_field<h2::cell_event_layout::logical_stop_channel_index>(
        cell_event, logical_stop_channel_index);
    lidar_v3::write_flag<h2::cell_event_layout::edge_slope_is_rise>(
        cell_event, edge_slope_is_rise);
    lidar_v3::write_field<h2::cell_event_layout::visible_return_count>(
        cell_event, emitted_return_count);
    lidar_v3::write_field<h2::cell_event_layout::serialized_return_slot_count>(
        cell_event, visible_return_count);

    for (unsigned return_index = 0U;
         return_index < limits::kMaximumReturnCountPerStop;
         ++return_index) {
#pragma HLS UNROLL
        const unsigned distance_hit_low_bit =
            h2::cell_event_layout::packed_distance_hits_17bit::low +
            return_index * 17U;
        cell_event.range(distance_hit_low_bit + 16U, distance_hit_low_bit) =
            return_index < emitted_return_count
                ? distance_hit_banks[return_index][cell_slot_address]
                : distance_hit_storage_t(0);
    }

    lidar_v3::write_flag<h2::cell_event_layout::hit_was_dropped>(
        cell_event, false);
    lidar_v3::write_flag<h2::cell_event_layout::return_overflow>(
        cell_event, return_overflow[cell_slot_address]);
    lidar_v3::write_flag<h2::cell_event_layout::error_fill_inserted>(
        cell_event, insert_error_fill);
    lidar_v3::write_flag<h2::cell_event_layout::cell_is_faulted>(
        cell_event,
        cell_error[cell_slot_address] || shot_is_faulted || insert_error_fill);
    lidar_v3::write_field<h2::cell_event_layout::timeout_cause_bitmap>(
        cell_event, timeout_cause_bitmap & 0x7U);
    lidar_v3::write_field<h2::cell_event_layout::shot_context>(
        cell_event, shot_context);
    lidar_v3::write_field<h2::cell_event_layout::tdc_chip_shot_sequence>(
        cell_event, tdc_chip_shot_sequence);
    return cell_event;
}

h2::cell_event_record_t build_control_cell_event(
    h2::cell_event_kind_t event_kind,
    std::uint8_t tdc_chip_index,
    bool ififo_bank_select,
    bool first_enabled_slope_is_rise,
    std::uint8_t visible_return_count,
    bool insert_error_fill,
    bool shot_is_faulted,
    std::uint8_t timeout_cause_bitmap,
    const shot_context_storage_t &shot_context,
    std::uint16_t tdc_chip_shot_sequence) {
    h2::cell_event_record_t cell_event = 0;
    lidar_v3::write_field<h2::cell_event_layout::event_kind>(
        cell_event, static_cast<std::uint8_t>(event_kind));
    lidar_v3::write_field<h2::cell_event_layout::tdc_chip_index>(
        cell_event, tdc_chip_index);
    lidar_v3::write_flag<h2::cell_event_layout::ififo_bank_select>(
        cell_event, ififo_bank_select);
    lidar_v3::write_flag<h2::cell_event_layout::edge_slope_is_rise>(
        cell_event, first_enabled_slope_is_rise);
    lidar_v3::write_field<h2::cell_event_layout::serialized_return_slot_count>(
        cell_event, visible_return_count);
    lidar_v3::write_flag<h2::cell_event_layout::error_fill_inserted>(
        cell_event, insert_error_fill);
    lidar_v3::write_flag<h2::cell_event_layout::cell_is_faulted>(
        cell_event, shot_is_faulted || insert_error_fill);
    lidar_v3::write_field<h2::cell_event_layout::timeout_cause_bitmap>(
        cell_event, timeout_cause_bitmap & 0x7U);
    lidar_v3::write_field<h2::cell_event_layout::shot_context>(
        cell_event, shot_context);
    lidar_v3::write_field<h2::cell_event_layout::tdc_chip_shot_sequence>(
        cell_event, tdc_chip_shot_sequence);
    return cell_event;
}

void emit_cell_event(
    hls::stream<h2::collector_result_axis_t> &collector_result_out,
    const h2::cell_event_record_t &cell_event) {
    h2::collector_result_axis_t result = 0;
    lidar_v3::write_field<h2::collector_result_layout::cell_event>(
        result, cell_event);
    lidar_v3::write_flag<h2::collector_result_layout::contains_cell_event>(
        result, true);
    collector_result_out.write(result);
}

void clear_all_shot_owner_state() {
    for (unsigned chip_index = 0U;
         chip_index < limits::kMaximumTdcGpxChipCount;
         ++chip_index) {
#pragma HLS PIPELINE II=1
        shot_is_active[chip_index] = false;
        owner_shot_is_faulted[chip_index] = false;
        lower_ififo_cells_were_emitted[chip_index] = false;
        owner_rise_slope_is_enabled[chip_index] = false;
        owner_fall_slope_is_enabled[chip_index] = false;
    }
}

void scrub_chip_cell_state(std::uint8_t tdc_chip_index) {
    const unsigned chip_base_address =
        static_cast<unsigned>(tdc_chip_index) * kCellSlotsPerChip;
    for (unsigned cell_slot_offset = 0U;
         cell_slot_offset < kCellSlotsPerChip;
         ++cell_slot_offset) {
#pragma HLS PIPELINE II=1
        const unsigned cell_slot_address =
            chip_base_address + cell_slot_offset;
        received_return_count[cell_slot_address] = 0U;
        return_overflow[cell_slot_address] = false;
        cell_error[cell_slot_address] = false;
    }
}

}  // namespace

extern "C" void gpx_cell_collector_hls(
    hls::stream<lidar_v3::h2::collector_input_axis_t> &decoded_hit_event_in,
    hls::stream<lidar_v3::h2::collector_result_axis_t> &collector_result_out,
    std::uint8_t build_tdc_chip_count,
    std::uint8_t build_stop_channels_per_chip,
    std::uint8_t build_max_return_count_per_stop,
    std::uint8_t runtime_enabled_rise_chip_mask,
    std::uint8_t runtime_enabled_fall_chip_mask,
    std::uint8_t runtime_visible_return_count,
    std::uint16_t active_configuration_version) {
#pragma HLS INTERFACE axis port=decoded_hit_event_in
#pragma HLS INTERFACE axis port=collector_result_out
#pragma HLS INTERFACE ap_none port=build_tdc_chip_count
#pragma HLS INTERFACE ap_none port=build_stop_channels_per_chip
#pragma HLS INTERFACE ap_none port=build_max_return_count_per_stop
#pragma HLS INTERFACE ap_none port=runtime_enabled_rise_chip_mask
#pragma HLS INTERFACE ap_none port=runtime_enabled_fall_chip_mask
#pragma HLS INTERFACE ap_none port=runtime_visible_return_count
#pragma HLS INTERFACE ap_none port=active_configuration_version
#pragma HLS INTERFACE ap_ctrl_hs port=return
#pragma HLS ARRAY_PARTITION variable=distance_hit_banks complete dim=1
#pragma HLS BIND_STORAGE variable=distance_hit_banks type=ram_2p impl=lutram
#pragma HLS BIND_STORAGE variable=received_return_count type=ram_2p impl=lutram
#pragma HLS BIND_STORAGE variable=return_overflow type=ram_2p impl=lutram
#pragma HLS BIND_STORAGE variable=cell_error type=ram_2p impl=lutram
#pragma HLS BIND_STORAGE variable=shot_is_active type=ram_2p impl=lutram
#pragma HLS BIND_STORAGE variable=owner_shot_context type=ram_2p impl=lutram
#pragma HLS BIND_STORAGE variable=owner_chip_shot_sequence type=ram_2p impl=lutram
#pragma HLS BIND_STORAGE variable=owner_visible_return_count type=ram_2p impl=lutram
#pragma HLS BIND_STORAGE variable=owner_shot_is_faulted type=ram_2p impl=lutram
#pragma HLS BIND_STORAGE variable=lower_ififo_cells_were_emitted type=ram_2p impl=lutram
#pragma HLS BIND_STORAGE variable=owner_rise_slope_is_enabled type=ram_2p impl=lutram
#pragma HLS BIND_STORAGE variable=owner_fall_slope_is_enabled type=ram_2p impl=lutram

    namespace h1 = lidar_v3::h1;
    namespace h2 = lidar_v3::h2;
    namespace limits = lidar_v3::limits;

    // ap_ctrl_hs samples all active configuration values with this accepted
    // input boundary. The RTL adapter keeps ap_start asserted continuously.
    const h2::collector_input_axis_t collector_input =
        decoded_hit_event_in.read();
    const std::uint8_t reset_epoch =
        lidar_v3::read_field<h2::collector_input_layout::reset_epoch>(
            collector_input)
            .to_uint();

    if (reset_epoch != accepted_reset_epoch) {
        clear_all_shot_owner_state();
        accepted_reset_epoch = reset_epoch;
    }

    const h1::decoded_hit_event_t decoded_hit_event =
        h2::read_collector_input_hit_event(collector_input);
    const std::uint8_t event_kind =
        lidar_v3::read_field<h1::decoded_hit_event_layout::event_kind>(
            decoded_hit_event)
            .to_uint();
    const std::uint8_t tdc_chip_index =
        lidar_v3::read_field<h1::decoded_hit_event_layout::tdc_chip_index>(
            decoded_hit_event)
            .to_uint();
    const bool ififo_bank_select =
        lidar_v3::read_flag<h1::decoded_hit_event_layout::ififo_bank_select>(
            decoded_hit_event);
    const std::uint8_t logical_stop_channel_index =
        lidar_v3::read_field<
            h1::decoded_hit_event_layout::logical_stop_channel_index>(
            decoded_hit_event)
            .to_uint();
    const bool edge_slope_is_rise =
        lidar_v3::read_flag<h1::decoded_hit_event_layout::edge_slope_is_rise>(
            decoded_hit_event);
    const std::uint16_t tdc_chip_shot_sequence =
        lidar_v3::read_field<
            h1::decoded_hit_event_layout::tdc_chip_shot_sequence>(
            decoded_hit_event)
            .to_uint();
    const shot_context_storage_t shot_context =
        read_hit_shot_context(decoded_hit_event);
    const std::uint8_t effective_visible_return_count =
        calculate_effective_visible_return_count(
            runtime_visible_return_count,
            build_max_return_count_per_stop);
    const bool build_configuration_is_valid =
        build_tdc_chip_count >= 1U &&
        build_tdc_chip_count <= limits::kMaximumTdcGpxChipCount &&
        build_stop_channels_per_chip >= 1U &&
        build_stop_channels_per_chip <=
            limits::kMaximumStopChannelsPerChip &&
        build_max_return_count_per_stop >= 1U &&
        build_max_return_count_per_stop <=
            limits::kMaximumReturnCountPerStop;
    const bool active_configuration_is_valid =
        build_configuration_is_valid &&
        runtime_visible_return_count >= 1U &&
        runtime_visible_return_count <= build_max_return_count_per_stop &&
        lidar_v3::read_field<
            h1::shot_context_layout::active_configuration_version>(
            shot_context) == active_configuration_version;

    bool shot_context_fault = false;
    bool return_overflow_fault = false;
    bool nonzero_start_number_fault = false;

    if (tdc_chip_index >= build_tdc_chip_count ||
        tdc_chip_index >= limits::kMaximumTdcGpxChipCount) {
        collector_result_out.write(
            build_status_result(true, false, false));
        return;
    }

    const bool opens_new_shot = !shot_is_active[tdc_chip_index];
    if (opens_new_shot) {
        scrub_chip_cell_state(tdc_chip_index);
        shot_is_active[tdc_chip_index] = true;
        owner_shot_context[tdc_chip_index] = shot_context;
        owner_chip_shot_sequence[tdc_chip_index] = tdc_chip_shot_sequence;
        owner_visible_return_count[tdc_chip_index] =
            effective_visible_return_count;
        owner_shot_is_faulted[tdc_chip_index] =
            lidar_v3::read_flag<
                h1::decoded_hit_event_layout::upstream_event_faulted>(
                decoded_hit_event) ||
            !active_configuration_is_valid;
        lower_ififo_cells_were_emitted[tdc_chip_index] = false;
        owner_rise_slope_is_enabled[tdc_chip_index] =
            ((runtime_enabled_rise_chip_mask >> tdc_chip_index) & 0x1U) != 0U;
        owner_fall_slope_is_enabled[tdc_chip_index] =
            ((runtime_enabled_fall_chip_mask >> tdc_chip_index) & 0x1U) != 0U;
    }

    if (opens_new_shot) {
        // The accepted event defines the new owner. Comparing a just-written
        // owner back to itself would add a wide write-bypass path.
        if (!active_configuration_is_valid) {
            shot_context_fault = true;
            owner_shot_is_faulted[tdc_chip_index] = true;
        }
    } else if (!shot_identity_matches(
                   shot_context,
                   tdc_chip_shot_sequence,
                   owner_shot_context[tdc_chip_index],
                   owner_chip_shot_sequence[tdc_chip_index]) ||
               owner_visible_return_count[tdc_chip_index] !=
                   effective_visible_return_count ||
               !active_configuration_is_valid) {
        shot_context_fault = true;
        owner_shot_is_faulted[tdc_chip_index] = true;
    } else if (lidar_v3::read_flag<
                   h1::decoded_hit_event_layout::upstream_event_faulted>(
                   decoded_hit_event)) {
        owner_shot_is_faulted[tdc_chip_index] = true;
    }

    if (event_kind ==
        static_cast<std::uint8_t>(h2::cell_event_kind_t::data)) {
        if (logical_stop_channel_index >= build_stop_channels_per_chip ||
            logical_stop_channel_index >=
                limits::kMaximumStopChannelsPerChip ||
            !edge_slope_is_supported(
                edge_slope_is_rise,
                owner_rise_slope_is_enabled[tdc_chip_index],
                owner_fall_slope_is_enabled[tdc_chip_index])) {
            shot_context_fault = true;
            owner_shot_is_faulted[tdc_chip_index] = true;
        } else {
            const unsigned cell_slot_address = calculate_cell_slot_address(
                tdc_chip_index,
                edge_slope_is_rise,
                logical_stop_channel_index);
            const std::uint8_t received_count =
                received_return_count[cell_slot_address];

            if (lidar_v3::read_field<
                    h1::decoded_hit_event_layout::tdc_start_number>(
                    decoded_hit_event) != 0U) {
                nonzero_start_number_fault = true;
                cell_error[cell_slot_address] = true;
                owner_shot_is_faulted[tdc_chip_index] = true;
            }

            if (received_count < build_max_return_count_per_stop &&
                received_count < limits::kMaximumReturnCountPerStop) {
                if (received_count <
                    owner_visible_return_count[tdc_chip_index]) {
                    distance_hit_banks[received_count][cell_slot_address] =
                        lidar_v3::read_field<
                            h1::decoded_hit_event_layout::distance_hit_17bit>(
                            decoded_hit_event);
                }
                // Returns beyond the Runtime visible count are intentionally
                // filtered after complete physical IFIFO Drain.
                received_return_count[cell_slot_address] = received_count + 1U;
            } else {
                return_overflow[cell_slot_address] = true;
                cell_error[cell_slot_address] = true;
                owner_shot_is_faulted[tdc_chip_index] = true;
                return_overflow_fault = true;
            }
        }

        collector_result_out.write(build_status_result(
            shot_context_fault,
            return_overflow_fault,
            nonzero_start_number_fault));
        return;
    }

    collector_result_out.write(
        build_status_result(shot_context_fault, false, false));

    h2::cell_event_kind_t terminal_event_kind =
        h2::cell_event_kind_t::drain_done;
    std::uint8_t first_stop_channel = 0U;
    std::uint8_t last_stop_channel = build_stop_channels_per_chip - 1U;
    bool insert_error_fill =
        lidar_v3::read_flag<
            h1::decoded_hit_event_layout::upstream_event_faulted>(
            decoded_hit_event);
    bool closes_shot = true;

    if (event_kind ==
        static_cast<std::uint8_t>(h2::cell_event_kind_t::ififo1_done)) {
        terminal_event_kind = h2::cell_event_kind_t::ififo1_done;
        last_stop_channel = build_stop_channels_per_chip > 4U
                                ? 3U
                                : build_stop_channels_per_chip - 1U;
        closes_shot = false;
    } else if (event_kind ==
               static_cast<std::uint8_t>(h2::cell_event_kind_t::drain_done)) {
        terminal_event_kind = h2::cell_event_kind_t::drain_done;
        first_stop_channel =
            lower_ififo_cells_were_emitted[tdc_chip_index] ? 4U : 0U;
    } else {
        terminal_event_kind = h2::cell_event_kind_t::timeout;
        first_stop_channel =
            lower_ififo_cells_were_emitted[tdc_chip_index] ? 4U : 0U;
        insert_error_fill = true;
        owner_shot_is_faulted[tdc_chip_index] = true;
    }

    const bool first_enabled_slope_is_rise =
        owner_rise_slope_is_enabled[tdc_chip_index];
    const std::uint8_t timeout_cause_bitmap =
        lidar_v3::read_field<
            h1::decoded_hit_event_layout::timeout_cause_bitmap>(
            decoded_hit_event)
            .to_uint();

    for (unsigned slope_order = 0U; slope_order < 2U; ++slope_order) {
        const bool emitted_slope_is_rise = slope_order == 0U;
        const bool emitted_slope_is_enabled =
            emitted_slope_is_rise
                ? owner_rise_slope_is_enabled[tdc_chip_index]
                : owner_fall_slope_is_enabled[tdc_chip_index];
        if (emitted_slope_is_enabled &&
            first_stop_channel < build_stop_channels_per_chip) {
            for (unsigned emitted_stop_channel = 0U;
                 emitted_stop_channel < limits::kMaximumStopChannelsPerChip;
                 ++emitted_stop_channel) {
#pragma HLS PIPELINE II=1
                if (emitted_stop_channel >= first_stop_channel &&
                    emitted_stop_channel <= last_stop_channel &&
                    emitted_stop_channel < build_stop_channels_per_chip) {
                    emit_cell_event(
                        collector_result_out,
                        build_data_cell_event(
                            tdc_chip_index,
                            static_cast<std::uint8_t>(emitted_stop_channel),
                            emitted_slope_is_rise,
                            owner_visible_return_count[tdc_chip_index],
                            insert_error_fill,
                            owner_shot_is_faulted[tdc_chip_index],
                            timeout_cause_bitmap,
                            owner_shot_context[tdc_chip_index],
                            owner_chip_shot_sequence[tdc_chip_index]));
                }
            }
        }
    }

    emit_cell_event(
        collector_result_out,
        build_control_cell_event(
            terminal_event_kind,
            tdc_chip_index,
            ififo_bank_select,
            first_enabled_slope_is_rise,
            owner_visible_return_count[tdc_chip_index],
            insert_error_fill,
            owner_shot_is_faulted[tdc_chip_index],
            timeout_cause_bitmap,
            owner_shot_context[tdc_chip_index],
            owner_chip_shot_sequence[tdc_chip_index]));

    if (closes_shot) {
        shot_is_active[tdc_chip_index] = false;
        owner_shot_is_faulted[tdc_chip_index] = false;
        lower_ififo_cells_were_emitted[tdc_chip_index] = false;
        owner_rise_slope_is_enabled[tdc_chip_index] = false;
        owner_fall_slope_is_enabled[tdc_chip_index] = false;
    } else {
        lower_ififo_cells_were_emitted[tdc_chip_index] = true;
    }
}
