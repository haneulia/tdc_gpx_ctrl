#include "lidar_packed17_ps_decoder.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>

typedef struct {
    FILE *file;
    unsigned int packet_count;
} capture_sink_t;

static int write_u32_le(FILE *file, uint32_t value)
{
    uint8_t bytes[4];

    bytes[0] = (uint8_t)(value & 0xFFU);
    bytes[1] = (uint8_t)((value >> 8U) & 0xFFU);
    bytes[2] = (uint8_t)((value >> 16U) & 0xFFU);
    bytes[3] = (uint8_t)((value >> 24U) & 0xFFU);
    return fwrite(bytes, 1U, sizeof(bytes), file) == sizeof(bytes) ? 0 : -1;
}

static int capture_packet(
    const uint8_t *payload,
    size_t payload_bytes,
    void *context)
{
    capture_sink_t *sink = (capture_sink_t *)context;

    if (payload_bytes > UINT32_MAX ||
        write_u32_le(sink->file, (uint32_t)payload_bytes) != 0 ||
        fwrite(payload, 1U, payload_bytes, sink->file) != payload_bytes) {
        return -1;
    }
    ++sink->packet_count;
    return 0;
}

static int parse_u32(const char *text, uint32_t maximum, uint32_t *value)
{
    char *end = NULL;
    unsigned long parsed;

    errno = 0;
    parsed = strtoul(text, &end, 0);
    if (errno != 0 || end == text || *end != '\0' || parsed > maximum) {
        return -1;
    }
    *value = (uint32_t)parsed;
    return 0;
}

static uint8_t *read_binary(const char *path, size_t *bytes)
{
    FILE *file;
    long length;
    uint8_t *data;

    file = fopen(path, "rb");
    if (file == NULL || fseek(file, 0L, SEEK_END) != 0) {
        if (file != NULL) {
            fclose(file);
        }
        return NULL;
    }
    length = ftell(file);
    if (length <= 0L || fseek(file, 0L, SEEK_SET) != 0) {
        fclose(file);
        return NULL;
    }
    data = (uint8_t *)malloc((size_t)length);
    if (data == NULL || fread(data, 1U, (size_t)length, file) != (size_t)length) {
        free(data);
        fclose(file);
        return NULL;
    }
    fclose(file);
    *bytes = (size_t)length;
    return data;
}

static void usage(const char *program)
{
    fprintf(stderr,
        "usage: %s ddr.bin capture.pkt width hsize stride vsize planned "
        "clock_hz bin_ps face_count total_states lower center upper "
        "lane_mask stops_per_chip\n",
        program);
}

int main(int argc, char **argv)
{
    uint32_t values[14];
    const uint32_t limits[14] = {
        128U, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX,
        UINT32_MAX, UINT32_MAX, UINT8_MAX, UINT32_MAX,
        UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT8_MAX, UINT8_MAX
    };
    uint8_t *ddr_data;
    size_t ddr_bytes;
    FILE *capture_file;
    capture_sink_t sink;
    lidar_ps_ddr_frame_t frame;
    lidar_ps_view_config_t config;
    lidar_ps_decode_summary_t summary;
    lidar_ps_status_t status;
    int index;

    if (argc != 17) {
        usage(argv[0]);
        return 2;
    }
    for (index = 0; index < 14; ++index) {
        if (parse_u32(argv[index + 3], limits[index], &values[index]) != 0) {
            fprintf(stderr, "invalid numeric argument %d: %s\n",
                index + 3, argv[index + 3]);
            return 2;
        }
    }

    ddr_data = read_binary(argv[1], &ddr_bytes);
    if (ddr_data == NULL) {
        fprintf(stderr, "cannot read DDR fixture: %s\n", argv[1]);
        return 3;
    }
    capture_file = fopen(argv[2], "wb");
    if (capture_file == NULL) {
        fprintf(stderr, "cannot create capture: %s\n", argv[2]);
        free(ddr_data);
        return 3;
    }

    frame.base = ddr_data;
    frame.allocation_bytes = ddr_bytes;
    frame.output_width_bits = (uint16_t)values[0];
    frame.hsize_bytes = (uint16_t)values[1];
    frame.stride_bytes = (uint16_t)values[2];
    frame.vsize_lines = (uint16_t)values[3];
    frame.planned_shots = (uint16_t)values[4];
    frame.owner = LIDAR_PS_BUFFER_DMA_OWNED;

    config.t0_tick_hz = values[5];
    config.bin_resolution_ps = values[6];
    config.configured_face_count = (uint8_t)values[7];
    config.total_states_per_revolution = values[8];
    config.face_lower_state = (uint16_t)values[9];
    config.face_center_state = (uint16_t)values[10];
    config.face_upper_state_exclusive = (uint16_t)values[11];
    config.lane_chip_mask = (uint8_t)values[12];
    config.stops_per_chip = (uint8_t)values[13];

    sink.file = capture_file;
    sink.packet_count = 0U;

    status = lidar_ps_decode_and_packetize(
        &frame, &config, capture_packet, &sink, &summary);
    if (status != LIDAR_PS_ERR_BUFFER_OWNER) {
        fprintf(stderr, "DMA-owned decode was not rejected: %s\n",
            lidar_ps_status_name(status));
        fclose(capture_file);
        free(ddr_data);
        return 4;
    }
    status = lidar_ps_publish_cpu_owned(&frame, false);
    if (status != LIDAR_PS_ERR_CACHE_SYNC_REQUIRED) {
        fprintf(stderr, "missing cache sync was not rejected: %s\n",
            lidar_ps_status_name(status));
        fclose(capture_file);
        free(ddr_data);
        return 4;
    }
    status = lidar_ps_publish_cpu_owned(&frame, true);
    if (status != LIDAR_PS_OK) {
        fprintf(stderr, "CPU ownership publish failed: %s\n",
            lidar_ps_status_name(status));
        fclose(capture_file);
        free(ddr_data);
        return 4;
    }

    {
        size_t footer_commit_address;
        uint8_t saved_commit_byte;

        if (frame.hsize_bytes == 0U) {
            fprintf(stderr, "invalid zero HSIZE\n");
            fclose(capture_file);
            free(ddr_data);
            return 4;
        }
        footer_commit_address =
            ((size_t)frame.planned_shots + (28U / frame.hsize_bytes)) *
                frame.stride_bytes +
            (28U % frame.hsize_bytes);
        if (footer_commit_address >= ddr_bytes) {
            fprintf(stderr, "Footer commit address is outside the fixture\n");
            fclose(capture_file);
            free(ddr_data);
            return 4;
        }
        saved_commit_byte = ddr_data[footer_commit_address];

        ddr_data[footer_commit_address] ^= 0x01U;
        status = lidar_ps_decode_and_packetize(
            &frame, &config, capture_packet, &sink, &summary);
        ddr_data[footer_commit_address] = saved_commit_byte;
        if (status != LIDAR_PS_ERR_FOOTER) {
            fprintf(stderr, "corrupt Footer was not rejected: %s\n",
                lidar_ps_status_name(status));
            fclose(capture_file);
            free(ddr_data);
            return 4;
        }
    }

    {
        uint16_t saved_width = frame.output_width_bits;

        frame.output_width_bits = 16U;
        status = lidar_ps_decode_and_packetize(
            &frame, &config, capture_packet, &sink, &summary);
        frame.output_width_bits = saved_width;
        if (status != LIDAR_PS_ERR_GEOMETRY) {
            fprintf(stderr, "illegal transport width was not rejected: %s\n",
                lidar_ps_status_name(status));
            fclose(capture_file);
            free(ddr_data);
            return 4;
        }
    }

    status = lidar_ps_decode_and_packetize(
        &frame, &config, capture_packet, &sink, &summary);
    if (status != LIDAR_PS_OK) {
        fprintf(stderr, "decode failed: %s\n", lidar_ps_status_name(status));
        fclose(capture_file);
        free(ddr_data);
        return 5;
    }
    lidar_ps_release_to_dma(&frame);
    status = lidar_ps_decode_and_packetize(
        &frame, &config, capture_packet, &sink, &summary);
    if (status != LIDAR_PS_ERR_BUFFER_OWNER) {
        fprintf(stderr, "released buffer decode was not rejected: %s\n",
            lidar_ps_status_name(status));
        fclose(capture_file);
        free(ddr_data);
        return 4;
    }

    if (fclose(capture_file) != 0) {
        free(ddr_data);
        return 6;
    }
    free(ddr_data);

    printf("LIDAR_V2_PS_DECODE_PASS frame=0x%08lX streams=%u packets=%u "
        "valid=%lu holes=%lu\n",
        (unsigned long)summary.face_frame_id,
        (unsigned int)summary.hline_stream_count,
        sink.packet_count,
        (unsigned long)summary.valid_sample_count,
        (unsigned long)summary.hole_sample_count);
    return 0;
}
