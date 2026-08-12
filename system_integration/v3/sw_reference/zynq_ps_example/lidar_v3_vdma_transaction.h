#ifndef LIDAR_V3_VDMA_TRANSACTION_H
#define LIDAR_V3_VDMA_TRANSACTION_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "lidar_v2_ps_control.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LIDAR_V3_VDMA_MIN_RETRY_LIMIT 1U
#define LIDAR_V3_VDMA_PRECOMMIT_READY 0
#define LIDAR_V3_VDMA_PRECOMMIT_RETRY 1
#define LIDAR_V3_VDMA_PRECOMMIT_FATAL (-1)

typedef enum {
    LIDAR_V3_VDMA_LANE_RISE = 0,
    LIDAR_V3_VDMA_LANE_FALL = 1
} lidar_v3_vdma_lane_id_t;

typedef enum {
    LIDAR_V3_VDMA_APPLY_OK = 0,
    LIDAR_V3_VDMA_APPLY_RETRY,
    LIDAR_V3_VDMA_APPLY_FATAL
} lidar_v3_vdma_apply_result_t;

typedef enum {
    LIDAR_V3_VDMA_OK = 0,
    LIDAR_V3_VDMA_ARGUMENT,
    LIDAR_V3_VDMA_UNSUPPORTED_WIDTH,
    LIDAR_V3_VDMA_INVALID_PROFILE,
    LIDAR_V3_VDMA_INVALID_REGION,
    LIDAR_V3_VDMA_REGION_OVERLAP,
    LIDAR_V3_VDMA_PRECOMMIT_FAILED,
    LIDAR_V3_VDMA_PRECOMMIT_TIMEOUT,
    LIDAR_V3_VDMA_PROFILE_CHANGED,
    LIDAR_V3_VDMA_RETRY_EXHAUSTED,
    LIDAR_V3_VDMA_PROGRESS_FAILED,
    LIDAR_V3_VDMA_APPLY_FAILED,
    LIDAR_V3_VDMA_ACK_FAILED
} lidar_v3_vdma_status_t;

typedef lidar_v3_vdma_apply_result_t (*lidar_v3_vdma_apply_lane_t)(
    void *context,
    lidar_v3_vdma_lane_id_t lane_id,
    const lidar_v2_vdma_lane_profile_t *profile);

typedef int (*lidar_v3_vdma_progress_service_t)(void *context);
typedef int (*lidar_v3_vdma_precommit_service_t)(void *context);
typedef void (*lidar_v3_vdma_poll_wait_t)(void *context);

typedef struct {
    bool ack_submitted;
    bool attempt_profile_valid;
    bool apply_completed;
    uint32_t apply_attempt_count;
    uint32_t consecutive_retry_count;
    uint32_t apply_success_count;
    uint32_t ack_count;
    lidar_v3_vdma_apply_result_t last_apply_result;
    lidar_v2_vdma_lane_profile_t attempt_profile;
    lidar_v2_vdma_lane_profile_t applied_profile;
} lidar_v3_vdma_lane_state_t;

typedef struct {
    const lidar_v2_device_t *device;
    lidar_v3_vdma_apply_lane_t apply_lane;
    void *apply_context;
    lidar_v3_vdma_progress_service_t progress_service;
    void *progress_context;
    lidar_v3_vdma_precommit_service_t precommit_service;
    void *precommit_context;
    lidar_v3_vdma_poll_wait_t poll_wait;
    void *poll_wait_context;
    uint32_t retry_limit;
    uint32_t profile_read_retry_count;
    bool transaction_active;
    lidar_v3_vdma_status_t last_status;
    lidar_v3_vdma_lane_state_t rise;
    lidar_v3_vdma_lane_state_t fall;
} lidar_v3_vdma_transaction_t;

typedef struct {
    uintptr_t base_address;
    size_t capacity_bytes;
} lidar_v3_dma_region_t;

void lidar_v3_vdma_transaction_init(
    lidar_v3_vdma_transaction_t *transaction,
    const lidar_v2_device_t *device,
    lidar_v3_vdma_apply_lane_t apply_lane,
    void *apply_context,
    uint32_t retry_limit);

/* Call once immediately before issuing one CSR COMMIT command. */
void lidar_v3_vdma_transaction_begin(
    lidar_v3_vdma_transaction_t *transaction);

void lidar_v3_vdma_transaction_set_progress_service(
    lidar_v3_vdma_transaction_t *transaction,
    lidar_v3_vdma_progress_service_t progress_service,
    void *progress_context);

/*
 * The pre-COMMIT service must return READY only after STOP has cleared both
 * RUNNING and ARMED, a fresh CTL23/24 snapshot proves the processing paths
 * idle, VDMA is stopped, and every completed old-geometry Frame is consumed.
 * RETRY is polled again before the CSR COMMIT command is written; FATAL
 * aborts without writing COMMIT.
 */
void lidar_v3_vdma_transaction_set_precommit_service(
    lidar_v3_vdma_transaction_t *transaction,
    lidar_v3_vdma_precommit_service_t precommit_service,
    void *precommit_context);

/*
 * Optional scheduler hook called after a pre-COMMIT RETRY and after an active
 * in-COMMIT poll that still needs work. FreeRTOS may use taskYIELD/vTaskDelay;
 * bare-metal may leave it NULL or use a bounded delay/watchdog service.
 */
void lidar_v3_vdma_transaction_set_poll_wait(
    lidar_v3_vdma_transaction_t *transaction,
    lidar_v3_vdma_poll_wait_t poll_wait,
    void *poll_wait_context);

int lidar_v3_vdma_transaction_prepare(
    lidar_v3_vdma_transaction_t *transaction,
    uint32_t poll_limit);

/* Compatible with lidar_v2_commit_shadow()'s poll-service callback. */
int lidar_v3_vdma_transaction_poll(void *context);

lidar_v3_vdma_status_t lidar_v3_vdma_validate_profile(
    const lidar_v2_vdma_lane_profile_t *profile,
    uint16_t output_width_bits,
    size_t frame_capacity_bytes);

lidar_v3_vdma_status_t lidar_v3_vdma_validate_regions(
    const lidar_v3_dma_region_t *regions,
    size_t region_count,
    size_t alignment_bytes);

const char *lidar_v3_vdma_status_name(lidar_v3_vdma_status_t status);

#ifdef __cplusplus
}
#endif

#endif
