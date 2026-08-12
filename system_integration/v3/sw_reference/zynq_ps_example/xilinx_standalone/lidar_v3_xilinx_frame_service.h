#ifndef LIDAR_V3_XILINX_FRAME_SERVICE_H
#define LIDAR_V3_XILINX_FRAME_SERVICE_H

#include "lidar_v3_xilinx_vdma_adapter.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * CTL23/CTL24 indexed diagnostic contract used before a VDMA geometry
 * transaction. Index 0x10 is sampled through the RTL mailbox; these masks
 * must stay aligned with lidar_processing_status_source.vhd.
 */
#define LIDAR_V3_DIAG_PROCESSING_FLAGS_INDEX 0x10U
#define LIDAR_V3_DIAG_CAPTURE (1U << 8)
#define LIDAR_V3_DIAG_BUSY (1U << 8)
#define LIDAR_V3_DIAG_VALID (1U << 9)
#define LIDAR_V3_DIAG_ERROR (1U << 10)
#define LIDAR_V3_DIAG_SEQUENCE_SHIFT 16U
#define LIDAR_V3_DIAG_PIPELINE_IDLE (1U << 12)
#define LIDAR_V3_DIAG_ECHO_IDLE (1U << 13)
#define LIDAR_V3_DIAG_GPX_PROCESSING_IDLE (1U << 14)
#define LIDAR_V3_DIAG_GPX_AXIS_IDLE (1U << 15)
#define LIDAR_V3_DIAG_GPX_CDC_RESET_BUSY (1U << 26)
#define LIDAR_V3_DIAG_ALL_PROCESSING_IDLE \
    (LIDAR_V3_DIAG_PIPELINE_IDLE | LIDAR_V3_DIAG_ECHO_IDLE | \
     LIDAR_V3_DIAG_GPX_PROCESSING_IDLE | LIDAR_V3_DIAG_GPX_AXIS_IDLE)

typedef int (*lidar_v3_xilinx_frame_consumer_t)(
    void *context,
    lidar_v3_vdma_lane_id_t lane_id,
    uint8_t frame_index,
    const lidar_ps_ddr_frame_t *frame);

typedef struct {
    const lidar_v2_device_t *device;
    lidar_v3_xilinx_vdma_adapter_t *adapter;
    lidar_v3_xilinx_frame_consumer_t consumer;
    void *consumer_context;
    bool held_frame_valid;
    bool held_consumer_completed;
    lidar_v3_vdma_lane_id_t held_lane_id;
    uint8_t held_frame_index;
    lidar_ps_ddr_frame_t held_frame;
    uint32_t delivered_count[2];
    uint32_t consumer_retry_count;
    uint32_t consumer_failure_count;
    uint32_t release_failure_count;
    bool idle_capture_pending;
    uint16_t idle_sequence_before;
} lidar_v3_xilinx_frame_service_t;

void lidar_v3_xilinx_frame_service_init(
    lidar_v3_xilinx_frame_service_t *service,
    const lidar_v2_device_t *device,
    lidar_v3_xilinx_vdma_adapter_t *adapter,
    lidar_v3_xilinx_frame_consumer_t consumer,
    void *consumer_context);

/*
 * Register this function with lidar_v3_vdma_transaction_set_progress_service().
 * XST_NO_DATA means temporary backpressure and is retried on the next poll.
 * Other consumer failures keep that Frame CPU-owned so a new VDMA profile
 * cannot overwrite it. The application may call this function after recovery.
 */
int lidar_v3_xilinx_service_completed_frames(void *context);

/*
 * Execute after lidar_v2_stop_and_disarm() and before writing CSR COMMIT.
 * The function first captures diagnostic index 0x10 and verifies that the
 * processing, echo, GPX-processing, and GPX-AXIS paths are all idle and that
 * GPX CDC reset is inactive. Only then may it stop VDMA. XST_SUCCESS means
 * every lane is stopped and all old-geometry Frames have been consumed.
 * XST_NO_DATA means the mailbox, datapath, or Frame consumer is temporarily
 * busy; yield the PS task and call again. XST_DEVICE_BUSY means the LiDAR
 * operation is still RUNNING/ARMED or a command is in progress.
 */
int lidar_v3_xilinx_prepare_commit_step(void *context);

/* Map Xilinx status values to the portable V3 pre-COMMIT callback contract. */
int lidar_v3_xilinx_prepare_commit_service(void *context);

/* Bind both pre-COMMIT drain and in-COMMIT race recovery to one transaction. */
int lidar_v3_xilinx_frame_service_bind_transaction(
    lidar_v3_xilinx_frame_service_t *service,
    lidar_v3_vdma_transaction_t *transaction);

#ifdef __cplusplus
}
#endif

#endif
