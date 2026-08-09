#include "lidar_packed17_ps_decoder.h"

#include <string.h>

#define LIDAR_GPX_FOOTER_MAGIC 0x47504631U
#define LIDAR_GPX_FOOTER_COMMIT 0x434F4D54U
#define LIDAR_PS_FACE_HEADER_MAGIC 0x3148464CU
#define LIDAR_PS_HLINE_MAGIC 0x314C484CU

#define LIDAR_SHOT_VALID_BIT 0U
#define LIDAR_SHOT_HOLE_BIT 1U
#define LIDAR_SHOT_CCW_BIT 2U
#define LIDAR_SHOT_SIM_BIT 3U
#define LIDAR_SHOT_TIMEOUT_BIT 4U
#define LIDAR_SHOT_ABORT_BIT 5U
#define LIDAR_SHOT_FAULT_BIT 6U
#define LIDAR_SHOT_T0_VALID_BIT 7U

#define LIDAR_SAMPLE_VALID_BIT 17U
#define LIDAR_SAMPLE_HOLE_BIT 18U
#define LIDAR_SAMPLE_FAULT_BIT 19U
#define LIDAR_SAMPLE_TIMEOUT_BIT 20U
#define LIDAR_SAMPLE_ABORT_BIT 21U

typedef struct {
    uint32_t words[8];
    uint32_t frame_id;
    uint16_t active_config_version;
    uint16_t planned_shots;
    uint16_t completed_shots;
    uint16_t hsize_bytes;
    uint16_t vsize_lines;
    uint8_t face_index;
    uint8_t slope_rise;
    uint8_t direction_ccw;
    uint8_t source_simulation;
    uint8_t cell_slots;
    uint8_t visible_returns;
    uint8_t cell_words;
    uint8_t footer_lines;
    uint16_t status_flags;
} lidar_ps_footer_t;

static uint32_t read_u32_le(const uint8_t *data)
{
    return (uint32_t)data[0] |
        ((uint32_t)data[1] << 8U) |
        ((uint32_t)data[2] << 16U) |
        ((uint32_t)data[3] << 24U);
}

static void write_u16_le(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)((value >> 8U) & 0xFFU);
}

static void write_u24_le(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)((value >> 8U) & 0xFFU);
    data[2] = (uint8_t)((value >> 16U) & 0xFFU);
}

static void write_u32_le(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)((value >> 8U) & 0xFFU);
    data[2] = (uint8_t)((value >> 16U) & 0xFFU);
    data[3] = (uint8_t)((value >> 24U) & 0xFFU);
}

static uint8_t popcount_u8(uint8_t value)
{
    uint8_t count = 0U;

    while (value != 0U) {
        count = (uint8_t)(count + (value & 1U));
        value = (uint8_t)(value >> 1U);
    }
    return count;
}

static uint8_t width_code(uint16_t width_bits)
{
    if (width_bits == 32U) {
        return 0U;
    }
    if (width_bits == 64U) {
        return 1U;
    }
    if (width_bits == 128U) {
        return 2U;
    }
    return 0xFFU;
}

static bool frame_range_valid(
    const lidar_ps_ddr_frame_t *frame,
    size_t offset,
    size_t length)
{
    return offset <= frame->allocation_bytes &&
        length <= frame->allocation_bytes - offset;
}

static lidar_ps_status_t read_footer(
    const lidar_ps_ddr_frame_t *frame,
    lidar_ps_footer_t *footer)
{
    uint8_t bytes[32];
    uint16_t byte_index;
    uint8_t expected_width_code;
    uint32_t raw_hsize;

    if (frame->vsize_lines <= frame->planned_shots ||
        frame->hsize_bytes == 0U || frame->stride_bytes < frame->hsize_bytes) {
        return LIDAR_PS_ERR_GEOMETRY;
    }

    footer->footer_lines = (uint8_t)(frame->vsize_lines - frame->planned_shots);
    for (byte_index = 0U; byte_index < sizeof(bytes); ++byte_index) {
        size_t footer_line = (size_t)frame->planned_shots +
            ((size_t)byte_index / frame->hsize_bytes);
        size_t line_byte = (size_t)byte_index % frame->hsize_bytes;
        size_t address = footer_line * frame->stride_bytes + line_byte;

        if (footer_line >= frame->vsize_lines ||
            !frame_range_valid(frame, address, 1U)) {
            return LIDAR_PS_ERR_FRAME_BOUNDS;
        }
        bytes[byte_index] = frame->base[address];
    }

    for (byte_index = 0U; byte_index < 8U; ++byte_index) {
        footer->words[byte_index] = read_u32_le(&bytes[(size_t)byte_index * 4U]);
    }
    if (footer->words[0] != LIDAR_GPX_FOOTER_MAGIC ||
        footer->words[7] != LIDAR_GPX_FOOTER_COMMIT) {
        return LIDAR_PS_ERR_FOOTER;
    }

    footer->frame_id = footer->words[1];
    footer->face_index = (uint8_t)(footer->words[2] & 0x7U);
    footer->slope_rise = (uint8_t)((footer->words[2] >> 3U) & 0x1U);
    footer->direction_ccw = (uint8_t)((footer->words[2] >> 4U) & 0x1U);
    footer->source_simulation = (uint8_t)((footer->words[2] >> 5U) & 0x1U);
    footer->active_config_version = (uint16_t)(footer->words[3] & 0xFFFFU);
    footer->planned_shots = (uint16_t)(footer->words[4] & 0xFFFFU);
    footer->cell_slots = (uint8_t)((footer->words[4] >> 16U) & 0x3FU);
    footer->visible_returns = (uint8_t)((footer->words[4] >> 22U) & 0x7U);
    footer->hsize_bytes = (uint16_t)(footer->words[5] & 0xFFFFU);
    footer->vsize_lines = (uint16_t)((footer->words[5] >> 16U) & 0xFFFFU);
    footer->completed_shots = (uint16_t)(footer->words[6] & 0xFFFFU);
    footer->status_flags = (uint16_t)((footer->words[6] >> 16U) & 0xFFFFU);

    expected_width_code = width_code(frame->output_width_bits);
    if (expected_width_code == 0xFFU ||
        ((footer->words[2] >> 6U) & 0x3U) != expected_width_code ||
        footer->planned_shots != frame->planned_shots ||
        footer->hsize_bytes != frame->hsize_bytes ||
        footer->vsize_lines != frame->vsize_lines ||
        footer->visible_returns == 0U || footer->visible_returns > 7U ||
        footer->cell_slots == 0U) {
        return LIDAR_PS_ERR_GEOMETRY;
    }

    footer->cell_words = (uint8_t)(((footer->visible_returns + 1U) / 2U) + 1U);
    raw_hsize = 16U + (uint32_t)footer->cell_slots *
        (uint32_t)footer->cell_words * 4U;
    if (raw_hsize > frame->hsize_bytes ||
        ((uint32_t)footer->footer_lines * frame->hsize_bytes) < 32U) {
        return LIDAR_PS_ERR_GEOMETRY;
    }
    return LIDAR_PS_OK;
}

static lidar_ps_status_t validate_view_config(
    const lidar_ps_view_config_t *view_config,
    const lidar_ps_footer_t *footer)
{
    uint16_t expected_slots;

    if (view_config->t0_tick_hz == 0U ||
        view_config->bin_resolution_ps == 0U ||
        view_config->configured_face_count == 0U ||
        footer->face_index >= view_config->configured_face_count ||
        view_config->stops_per_chip == 0U) {
        return LIDAR_PS_ERR_ARGUMENT;
    }

    expected_slots = (uint16_t)popcount_u8(view_config->lane_chip_mask) *
        view_config->stops_per_chip;
    if (expected_slots != footer->cell_slots) {
        return LIDAR_PS_ERR_GEOMETRY;
    }
    return LIDAR_PS_OK;
}

static lidar_ps_status_t find_first_valid_t0(
    const lidar_ps_ddr_frame_t *frame,
    uint32_t *t0_low,
    uint32_t *t0_high,
    bool *valid)
{
    uint16_t shot;

    *valid = false;
    *t0_low = 0U;
    *t0_high = 0U;
    for (shot = 0U; shot < frame->planned_shots; ++shot) {
        size_t address = (size_t)shot * frame->stride_bytes;
        uint32_t status;

        if (!frame_range_valid(frame, address, 16U)) {
            return LIDAR_PS_ERR_FRAME_BOUNDS;
        }
        status = read_u32_le(&frame->base[address + 12U]);
        if (((status >> LIDAR_SHOT_T0_VALID_BIT) & 1U) != 0U &&
            ((status >> LIDAR_SHOT_HOLE_BIT) & 1U) == 0U) {
            *t0_low = read_u32_le(&frame->base[address]);
            *t0_high = read_u32_le(&frame->base[address + 4U]);
            *valid = true;
            return LIDAR_PS_OK;
        }
    }
    return LIDAR_PS_OK;
}

static lidar_ps_status_t emit_face_header(
    const lidar_ps_ddr_frame_t *frame,
    const lidar_ps_view_config_t *view_config,
    const lidar_ps_footer_t *footer,
    uint16_t hline_packet_count,
    lidar_ps_packet_sink_t packet_sink,
    void *packet_sink_context)
{
    uint8_t packet[LIDAR_PS_FACE_HEADER_BYTES];
    uint32_t t0_low;
    uint32_t t0_high;
    bool t0_valid;
    uint16_t header_flags;
    lidar_ps_status_t status;

    memset(packet, 0, sizeof(packet));
    status = find_first_valid_t0(frame, &t0_low, &t0_high, &t0_valid);
    if (status != LIDAR_PS_OK) {
        return status;
    }

    header_flags = 0U;
    if (t0_valid) {
        header_flags |= 0x0001U;
    }
    if ((footer->status_flags & 0x0008U) != 0U) {
        header_flags |= 0x0004U;
    }
    if ((footer->status_flags & 0x0077U) != 0U) {
        header_flags |= 0x0008U;
    }

    write_u32_le(&packet[0], LIDAR_PS_FACE_HEADER_MAGIC);
    write_u16_le(&packet[4], 1U);
    write_u16_le(&packet[6], LIDAR_PS_FACE_HEADER_USED_BYTES);
    write_u32_le(&packet[8], footer->frame_id);
    write_u16_le(&packet[12], footer->active_config_version);
    packet[14] = footer->face_index;
    packet[15] = view_config->configured_face_count;
    packet[16] = footer->slope_rise;
    packet[17] = footer->direction_ccw;
    packet[18] = footer->source_simulation;
    packet[19] = footer->visible_returns;
    packet[20] = view_config->lane_chip_mask;
    packet[21] = view_config->stops_per_chip;
    packet[22] = footer->cell_words;
    packet[23] = LIDAR_PS_SAMPLE_BYTES;
    write_u16_le(&packet[24], footer->cell_slots);
    write_u16_le(&packet[26], footer->planned_shots);
    write_u16_le(&packet[28], footer->completed_shots);
    write_u16_le(&packet[30],
        (uint16_t)footer->cell_slots * footer->visible_returns);
    write_u32_le(&packet[32], view_config->t0_tick_hz);
    write_u32_le(&packet[36], view_config->bin_resolution_ps);
    write_u32_le(&packet[40], view_config->total_states_per_revolution);
    write_u16_le(&packet[44], view_config->face_lower_state);
    write_u16_le(&packet[46], view_config->face_center_state);
    write_u16_le(&packet[48], view_config->face_upper_state_exclusive);
    write_u16_le(&packet[50], footer->status_flags);
    write_u32_le(&packet[52], t0_low);
    write_u32_le(&packet[56], t0_high);
    write_u16_le(&packet[60], hline_packet_count);
    write_u16_le(&packet[62], header_flags);

    if (packet_sink(packet, sizeof(packet), packet_sink_context) != 0) {
        return LIDAR_PS_ERR_PACKET_SINK;
    }
    return LIDAR_PS_OK;
}

static lidar_ps_status_t validate_hole_cell(
    const lidar_ps_ddr_frame_t *frame,
    size_t cell_address,
    const lidar_ps_footer_t *footer)
{
    size_t byte_index;
    size_t cell_bytes = (size_t)footer->cell_words * 4U;

    if (!frame_range_valid(frame, cell_address, cell_bytes)) {
        return LIDAR_PS_ERR_FRAME_BOUNDS;
    }
    for (byte_index = 0U; byte_index < cell_bytes; ++byte_index) {
        if (frame->base[cell_address + byte_index] != 0U) {
            return LIDAR_PS_ERR_SHOT_LINE;
        }
    }
    return LIDAR_PS_OK;
}

static lidar_ps_status_t build_sample(
    const lidar_ps_ddr_frame_t *frame,
    const lidar_ps_footer_t *footer,
    uint16_t shot,
    uint8_t cell_slot,
    uint8_t return_index,
    uint32_t *sample,
    bool *is_valid,
    bool *is_hole,
    bool *is_fault)
{
    size_t line_address = (size_t)shot * frame->stride_bytes;
    size_t cell_address = line_address + 16U +
        (size_t)cell_slot * footer->cell_words * 4U;
    size_t metadata_address = cell_address +
        ((size_t)footer->cell_words - 1U) * 4U;
    size_t hit_word_address = cell_address +
        ((size_t)return_index / 2U) * 4U;
    uint32_t shot_index_position;
    uint32_t shot_status;
    uint32_t metadata;
    uint32_t hit_word;
    uint32_t hit;
    bool timeout;
    bool aborted;
    lidar_ps_status_t status;

    if (!frame_range_valid(frame, line_address, frame->hsize_bytes) ||
        !frame_range_valid(frame, metadata_address, 4U) ||
        !frame_range_valid(frame, hit_word_address, 4U)) {
        return LIDAR_PS_ERR_FRAME_BOUNDS;
    }

    shot_index_position = read_u32_le(&frame->base[line_address + 8U]);
    shot_status = read_u32_le(&frame->base[line_address + 12U]);
    if ((shot_index_position & 0xFFFFU) != shot) {
        return LIDAR_PS_ERR_SHOT_LINE;
    }

    *is_hole = ((shot_status >> LIDAR_SHOT_HOLE_BIT) & 1U) != 0U;
    if ((*is_hole && ((shot_status >> LIDAR_SHOT_VALID_BIT) & 1U) != 0U) ||
        (!*is_hole && ((shot_status >> LIDAR_SHOT_VALID_BIT) & 1U) == 0U)) {
        return LIDAR_PS_ERR_SHOT_LINE;
    }
    if (*is_hole && return_index == 0U) {
        status = validate_hole_cell(frame, cell_address, footer);
        if (status != LIDAR_PS_OK) {
            return status;
        }
    }

    metadata = read_u32_le(&frame->base[metadata_address]);
    hit_word = read_u32_le(&frame->base[hit_word_address]);
    if (!*is_hole && ((metadata >> 31U) & 1U) == 0U) {
        return LIDAR_PS_ERR_SHOT_LINE;
    }

    *is_valid = !*is_hole &&
        (((metadata >> (7U + return_index)) & 1U) != 0U);
    hit = ((return_index & 1U) == 0U) ?
        (hit_word & 0xFFFFU) : ((hit_word >> 16U) & 0xFFFFU);
    hit |= ((metadata >> return_index) & 1U) << 16U;
    *is_fault = ((shot_status >> LIDAR_SHOT_FAULT_BIT) & 1U) != 0U ||
        ((metadata >> 27U) & 1U) != 0U;
    timeout = ((shot_status >> LIDAR_SHOT_TIMEOUT_BIT) & 1U) != 0U;
    aborted = ((shot_status >> LIDAR_SHOT_ABORT_BIT) & 1U) != 0U;

    *sample = hit;
    if (*is_valid) {
        *sample |= 1U << LIDAR_SAMPLE_VALID_BIT;
    }
    if (*is_hole) {
        *sample |= 1U << LIDAR_SAMPLE_HOLE_BIT;
    }
    if (*is_fault) {
        *sample |= 1U << LIDAR_SAMPLE_FAULT_BIT;
    }
    if (timeout) {
        *sample |= 1U << LIDAR_SAMPLE_TIMEOUT_BIT;
    }
    if (aborted) {
        *sample |= 1U << LIDAR_SAMPLE_ABORT_BIT;
    }
    return LIDAR_PS_OK;
}

static lidar_ps_status_t emit_hline_packets(
    const lidar_ps_ddr_frame_t *frame,
    const lidar_ps_footer_t *footer,
    lidar_ps_packet_sink_t packet_sink,
    void *packet_sink_context,
    lidar_ps_decode_summary_t *summary)
{
    uint8_t packet[LIDAR_PS_ETH_PAYLOAD_MAX_BYTES];
    uint16_t packets_per_hline = (uint16_t)(
        ((uint32_t)footer->planned_shots +
            LIDAR_PS_HLINE_SAMPLES_PER_PACKET - 1U) /
        LIDAR_PS_HLINE_SAMPLES_PER_PACKET);
    uint8_t cell_slot;
    uint8_t return_index;

    for (cell_slot = 0U; cell_slot < footer->cell_slots; ++cell_slot) {
        for (return_index = 0U;
            return_index < footer->visible_returns;
            ++return_index) {
            uint16_t packet_index;

            for (packet_index = 0U;
                packet_index < packets_per_hline;
                ++packet_index) {
                uint16_t first_shot = (uint16_t)(
                    packet_index * LIDAR_PS_HLINE_SAMPLES_PER_PACKET);
                uint16_t remaining = (uint16_t)(
                    footer->planned_shots - first_shot);
                uint16_t sample_count = remaining >
                    LIDAR_PS_HLINE_SAMPLES_PER_PACKET ?
                    LIDAR_PS_HLINE_SAMPLES_PER_PACKET : remaining;
                uint16_t sample_index;
                uint16_t valid_count = 0U;
                uint8_t flags = 0U;
                size_t packet_bytes = LIDAR_PS_HLINE_HEADER_BYTES +
                    (size_t)sample_count * LIDAR_PS_SAMPLE_BYTES;

                memset(packet, 0, packet_bytes);
                if (footer->slope_rise != 0U) {
                    flags |= 0x01U;
                }
                if (footer->direction_ccw != 0U) {
                    flags |= 0x02U;
                }
                if (footer->source_simulation != 0U) {
                    flags |= 0x04U;
                }
                if (packet_index + 1U == packets_per_hline) {
                    flags |= 0x20U;
                }

                for (sample_index = 0U;
                    sample_index < sample_count;
                    ++sample_index) {
                    uint32_t sample;
                    bool is_valid;
                    bool is_hole;
                    bool is_fault;
                    lidar_ps_status_t status = build_sample(
                        frame, footer,
                        (uint16_t)(first_shot + sample_index),
                        cell_slot, return_index, &sample,
                        &is_valid, &is_hole, &is_fault);

                    if (status != LIDAR_PS_OK) {
                        return status;
                    }
                    write_u24_le(&packet[LIDAR_PS_HLINE_HEADER_BYTES +
                        (size_t)sample_index * LIDAR_PS_SAMPLE_BYTES], sample);
                    if (is_valid) {
                        ++valid_count;
                        ++summary->valid_sample_count;
                    }
                    if (is_hole) {
                        flags |= 0x08U;
                        ++summary->hole_sample_count;
                    }
                    if (is_fault) {
                        flags |= 0x10U;
                    }
                }

                write_u32_le(&packet[0], LIDAR_PS_HLINE_MAGIC);
                write_u32_le(&packet[4], footer->frame_id);
                write_u16_le(&packet[8], footer->active_config_version);
                packet[10] = footer->face_index;
                packet[11] = flags;
                write_u16_le(&packet[12], cell_slot);
                packet[14] = cell_slot;
                packet[15] = return_index;
                write_u16_le(&packet[16], first_shot);
                write_u16_le(&packet[18], sample_count);
                write_u16_le(&packet[20], packet_index);
                write_u16_le(&packet[22], packets_per_hline);
                write_u16_le(&packet[24], valid_count);
                write_u16_le(&packet[26],
                    (uint16_t)(sample_count * LIDAR_PS_SAMPLE_BYTES));
                packet[28] = LIDAR_PS_SAMPLE_BYTES;
                packet[29] = footer->visible_returns;
                write_u16_le(&packet[30], footer->planned_shots);

                if (packet_sink(packet, packet_bytes,
                        packet_sink_context) != 0) {
                    return LIDAR_PS_ERR_PACKET_SINK;
                }
                ++summary->hline_packet_count;
            }
        }
    }
    return LIDAR_PS_OK;
}

lidar_ps_status_t lidar_ps_publish_cpu_owned(
    lidar_ps_ddr_frame_t *frame,
    bool platform_cache_sync_completed)
{
    if (frame == NULL) {
        return LIDAR_PS_ERR_ARGUMENT;
    }
    if (!platform_cache_sync_completed) {
        return LIDAR_PS_ERR_CACHE_SYNC_REQUIRED;
    }
    if (frame->owner != LIDAR_PS_BUFFER_DMA_OWNED) {
        return LIDAR_PS_ERR_BUFFER_OWNER;
    }
    frame->owner = LIDAR_PS_BUFFER_CPU_OWNED;
    return LIDAR_PS_OK;
}

void lidar_ps_release_to_dma(lidar_ps_ddr_frame_t *frame)
{
    if (frame != NULL) {
        frame->owner = LIDAR_PS_BUFFER_DMA_OWNED;
    }
}

lidar_ps_status_t lidar_ps_inspect_frame(
    const lidar_ps_ddr_frame_t *frame,
    lidar_ps_frame_info_t *frame_info)
{
    lidar_ps_footer_t footer;
    lidar_ps_status_t status;
    uint32_t required_bytes;

    if (frame == NULL || frame_info == NULL || frame->base == NULL) {
        return LIDAR_PS_ERR_ARGUMENT;
    }
    if (frame->owner != LIDAR_PS_BUFFER_CPU_OWNED) {
        return LIDAR_PS_ERR_BUFFER_OWNER;
    }
    required_bytes = (uint32_t)frame->stride_bytes * frame->vsize_lines;
    if (required_bytes > frame->allocation_bytes) {
        return LIDAR_PS_ERR_FRAME_BOUNDS;
    }

    memset(&footer, 0, sizeof(footer));
    status = read_footer(frame, &footer);
    if (status != LIDAR_PS_OK) {
        return status;
    }

    memset(frame_info, 0, sizeof(*frame_info));
    frame_info->face_frame_id = footer.frame_id;
    frame_info->active_config_version = footer.active_config_version;
    frame_info->planned_shots = footer.planned_shots;
    frame_info->completed_shots = footer.completed_shots;
    frame_info->hsize_bytes = footer.hsize_bytes;
    frame_info->vsize_lines = footer.vsize_lines;
    frame_info->footer_status_flags = footer.status_flags;
    frame_info->face_index = footer.face_index;
    frame_info->slope_rise = footer.slope_rise;
    frame_info->direction_ccw = footer.direction_ccw;
    frame_info->source_simulation = footer.source_simulation;
    frame_info->cell_slots = footer.cell_slots;
    frame_info->visible_returns = footer.visible_returns;
    return LIDAR_PS_OK;
}

lidar_ps_status_t lidar_ps_decode_and_packetize(
    const lidar_ps_ddr_frame_t *frame,
    const lidar_ps_view_config_t *view_config,
    lidar_ps_packet_sink_t packet_sink,
    void *packet_sink_context,
    lidar_ps_decode_summary_t *summary)
{
    lidar_ps_footer_t footer;
    lidar_ps_status_t status;
    uint32_t required_bytes;
    uint16_t packets_per_hline;
    uint16_t hline_packet_count;

    if (frame == NULL || view_config == NULL || packet_sink == NULL ||
        summary == NULL || frame->base == NULL) {
        return LIDAR_PS_ERR_ARGUMENT;
    }
    if (frame->owner != LIDAR_PS_BUFFER_CPU_OWNED) {
        return LIDAR_PS_ERR_BUFFER_OWNER;
    }

    required_bytes = (uint32_t)frame->stride_bytes * frame->vsize_lines;
    if (required_bytes > frame->allocation_bytes) {
        return LIDAR_PS_ERR_FRAME_BOUNDS;
    }

    memset(&footer, 0, sizeof(footer));
    status = read_footer(frame, &footer);
    if (status != LIDAR_PS_OK) {
        return status;
    }
    status = validate_view_config(view_config, &footer);
    if (status != LIDAR_PS_OK) {
        return status;
    }

    packets_per_hline = (uint16_t)(
        ((uint32_t)footer.planned_shots +
            LIDAR_PS_HLINE_SAMPLES_PER_PACKET - 1U) /
        LIDAR_PS_HLINE_SAMPLES_PER_PACKET);
    hline_packet_count = (uint16_t)((uint16_t)footer.cell_slots *
        footer.visible_returns * packets_per_hline);

    memset(summary, 0, sizeof(*summary));
    summary->face_frame_id = footer.frame_id;
    summary->active_config_version = footer.active_config_version;
    summary->hline_stream_count = (uint16_t)footer.cell_slots *
        footer.visible_returns;

    status = emit_face_header(frame, view_config, &footer,
        hline_packet_count, packet_sink, packet_sink_context);
    if (status != LIDAR_PS_OK) {
        return status;
    }
    status = emit_hline_packets(frame, &footer, packet_sink,
        packet_sink_context, summary);
    if (status != LIDAR_PS_OK) {
        return status;
    }
    if (summary->hline_packet_count != hline_packet_count) {
        return LIDAR_PS_ERR_GEOMETRY;
    }
    return LIDAR_PS_OK;
}

const char *lidar_ps_status_name(lidar_ps_status_t status)
{
    static const char *const names[] = {
        "OK",
        "ARGUMENT",
        "CACHE_SYNC_REQUIRED",
        "BUFFER_OWNER",
        "FRAME_BOUNDS",
        "FOOTER",
        "GEOMETRY",
        "SHOT_LINE",
        "PACKET_SINK"
    };

    if ((unsigned int)status >= (sizeof(names) / sizeof(names[0]))) {
        return "UNKNOWN";
    }
    return names[status];
}
