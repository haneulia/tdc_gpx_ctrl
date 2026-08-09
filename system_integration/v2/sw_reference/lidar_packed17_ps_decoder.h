#ifndef LIDAR_PACKED17_PS_DECODER_H
#define LIDAR_PACKED17_PS_DECODER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LIDAR_PS_ETH_PAYLOAD_MAX_BYTES 1440U
#define LIDAR_PS_FACE_HEADER_BYTES 1440U
#define LIDAR_PS_FACE_HEADER_USED_BYTES 64U
#define LIDAR_PS_HLINE_HEADER_BYTES 32U
#define LIDAR_PS_SAMPLE_BYTES 3U
#define LIDAR_PS_HLINE_SAMPLES_PER_PACKET \
    ((LIDAR_PS_ETH_PAYLOAD_MAX_BYTES - LIDAR_PS_HLINE_HEADER_BYTES) / \
        LIDAR_PS_SAMPLE_BYTES)

typedef enum {
    LIDAR_PS_OK = 0,
    LIDAR_PS_ERR_ARGUMENT,
    LIDAR_PS_ERR_CACHE_SYNC_REQUIRED,
    LIDAR_PS_ERR_BUFFER_OWNER,
    LIDAR_PS_ERR_FRAME_BOUNDS,
    LIDAR_PS_ERR_FOOTER,
    LIDAR_PS_ERR_GEOMETRY,
    LIDAR_PS_ERR_SHOT_LINE,
    LIDAR_PS_ERR_PACKET_SINK
} lidar_ps_status_t;

typedef enum {
    LIDAR_PS_BUFFER_DMA_OWNED = 0,
    LIDAR_PS_BUFFER_CPU_OWNED = 1
} lidar_ps_buffer_owner_t;

typedef struct {
    const uint8_t *base;
    size_t allocation_bytes;
    uint16_t hsize_bytes;
    uint16_t stride_bytes;
    uint16_t vsize_lines;
    uint16_t planned_shots;
    uint16_t output_width_bits;
    lidar_ps_buffer_owner_t owner;
} lidar_ps_ddr_frame_t;

typedef struct {
    uint32_t t0_tick_hz;
    uint32_t bin_resolution_ps;
    uint32_t total_states_per_revolution;
    uint16_t face_lower_state;
    uint16_t face_center_state;
    uint16_t face_upper_state_exclusive;
    uint8_t configured_face_count;
    uint8_t lane_chip_mask;
    uint8_t stops_per_chip;
} lidar_ps_view_config_t;

typedef int (*lidar_ps_packet_sink_t)(
    const uint8_t *payload,
    size_t payload_bytes,
    void *context);

typedef struct {
    uint32_t face_frame_id;
    uint16_t active_config_version;
    uint16_t hline_stream_count;
    uint16_t hline_packet_count;
    uint32_t valid_sample_count;
    uint32_t hole_sample_count;
} lidar_ps_decode_summary_t;

typedef struct {
    uint32_t face_frame_id;
    uint16_t active_config_version;
    uint16_t planned_shots;
    uint16_t completed_shots;
    uint16_t hsize_bytes;
    uint16_t vsize_lines;
    uint16_t footer_status_flags;
    uint8_t face_index;
    uint8_t slope_rise;
    uint8_t direction_ccw;
    uint8_t source_simulation;
    uint8_t cell_slots;
    uint8_t visible_returns;
} lidar_ps_frame_info_t;

/*
 * The platform cache API must run before this function. Passing false is an
 * explicit contract failure; this library never pretends to invalidate a DMA
 * buffer itself.
 */
lidar_ps_status_t lidar_ps_publish_cpu_owned(
    lidar_ps_ddr_frame_t *frame,
    bool platform_cache_sync_completed);

void lidar_ps_release_to_dma(lidar_ps_ddr_frame_t *frame);

/* Read only the committed Face Footer before selecting Face-specific bounds. */
lidar_ps_status_t lidar_ps_inspect_frame(
    const lidar_ps_ddr_frame_t *frame,
    lidar_ps_frame_info_t *frame_info);

lidar_ps_status_t lidar_ps_decode_and_packetize(
    const lidar_ps_ddr_frame_t *frame,
    const lidar_ps_view_config_t *view_config,
    lidar_ps_packet_sink_t packet_sink,
    void *packet_sink_context,
    lidar_ps_decode_summary_t *summary);

const char *lidar_ps_status_name(lidar_ps_status_t status);

#ifdef __cplusplus
}
#endif

#endif
