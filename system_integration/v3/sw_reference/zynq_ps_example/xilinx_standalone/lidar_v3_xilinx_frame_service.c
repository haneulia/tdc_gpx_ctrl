#include "lidar_v3_xilinx_frame_service.h"

#include <string.h>

#include "xstatus.h"

static bool device_is_valid(const lidar_v2_device_t *device)
{
    return device != NULL && device->read32 != NULL &&
        device->write32 != NULL;
}

static uint32_t mask_load(const volatile uint32_t *mask)
{
    return __atomic_load_n(mask, __ATOMIC_ACQUIRE);
}

static bool lane_has_fatal_status(
    const lidar_v2_xilinx_vdma_lane_t *lane)
{
    return lane != NULL &&
        (mask_load(&lane->overwrite_mask) != 0U ||
         mask_load(&lane->error_mask) != 0U);
}

static bool all_lanes_are_quiesced(
    const lidar_v3_xilinx_vdma_adapter_t *adapter)
{
    return adapter != NULL &&
        (adapter->rise == NULL || adapter->rise_quiesced) &&
        (adapter->fall == NULL || adapter->fall_quiesced);
}

/*
 * A fresh sequence value is mandatory. Reading only CTL24 could accept a
 * stale idle snapshot captured before STOP/DISARM reached the processing
 * clock domain.
 */
static int poll_processing_idle_snapshot(
    lidar_v3_xilinx_frame_service_t *service)
{
    uint32_t control;
    uint32_t flags;
    uint16_t sequence;

    control = lidar_v2_read_ctl(
        service->device, LIDAR_V2_CTL_DIAG_INDEX);
    if (!service->idle_capture_pending) {
        if ((control & LIDAR_V3_DIAG_BUSY) != 0U) {
            return XST_NO_DATA;
        }
        service->idle_sequence_before = (uint16_t)(
            control >> LIDAR_V3_DIAG_SEQUENCE_SHIFT);
        lidar_v2_write_ctl(service->device, LIDAR_V2_CTL_DIAG_INDEX,
            LIDAR_V3_DIAG_CAPTURE |
                LIDAR_V3_DIAG_PROCESSING_FLAGS_INDEX);
        service->idle_capture_pending = true;
        return XST_NO_DATA;
    }

    sequence = (uint16_t)(control >> LIDAR_V3_DIAG_SEQUENCE_SHIFT);
    if ((control & LIDAR_V3_DIAG_BUSY) != 0U ||
        sequence == service->idle_sequence_before) {
        return XST_NO_DATA;
    }
    service->idle_capture_pending = false;
    if ((control & 0xFFU) != LIDAR_V3_DIAG_PROCESSING_FLAGS_INDEX ||
        (control & LIDAR_V3_DIAG_VALID) == 0U ||
        (control & LIDAR_V3_DIAG_ERROR) != 0U) {
        return XST_FAILURE;
    }

    flags = lidar_v2_read_ctl(
        service->device, LIDAR_V2_CTL_DIAG_DATA);
    if ((flags & LIDAR_V3_DIAG_ALL_PROCESSING_IDLE) !=
            LIDAR_V3_DIAG_ALL_PROCESSING_IDLE ||
        (flags & LIDAR_V3_DIAG_GPX_CDC_RESET_BUSY) != 0U) {
        return XST_NO_DATA;
    }
    return XST_SUCCESS;
}

static int deliver_held_frame(lidar_v3_xilinx_frame_service_t *service)
{
    size_t lane_index = (size_t)service->held_lane_id;
    int status;

    if (!service->held_consumer_completed) {
        status = service->consumer(service->consumer_context,
            service->held_lane_id, service->held_frame_index,
            &service->held_frame);
        if (status == XST_NO_DATA) {
            ++service->consumer_retry_count;
            return XST_NO_DATA;
        }
        if (status != XST_SUCCESS) {
            ++service->consumer_failure_count;
            return XST_FAILURE;
        }
        service->held_consumer_completed = true;
    }
    if (lidar_v3_xilinx_vdma_release_frame(service->adapter,
            service->held_lane_id, service->held_frame_index,
            &service->held_frame) != XST_SUCCESS) {
        ++service->release_failure_count;
        return XST_FAILURE;
    }
    ++service->delivered_count[lane_index];
    service->held_frame_valid = false;
    service->held_consumer_completed = false;
    return XST_SUCCESS;
}

static int service_lane(
    lidar_v3_xilinx_frame_service_t *service,
    lidar_v3_vdma_lane_id_t lane_id,
    uint16_t planned_shots)
{
    lidar_v2_xilinx_vdma_lane_t *lane =
        lane_id == LIDAR_V3_VDMA_LANE_RISE ?
            service->adapter->rise : service->adapter->fall;
    uint8_t attempt;
    int status;

    if (lane == NULL) {
        return XST_SUCCESS;
    }
    for (attempt = 0U; attempt < lane->frame_store_count; ++attempt) {
        status = lidar_v3_xilinx_vdma_take_completed(
            service->adapter, lane_id, planned_shots,
            &service->held_frame_index, &service->held_frame);
        if (status == XST_NO_DATA) {
            return XST_SUCCESS;
        }
        if (status != XST_SUCCESS) {
            return status;
        }
        service->held_lane_id = lane_id;
        service->held_frame_valid = true;
        service->held_consumer_completed = false;
        status = deliver_held_frame(service);
        if (status != XST_SUCCESS) {
            return status;
        }
    }
    return XST_SUCCESS;
}

void lidar_v3_xilinx_frame_service_init(
    lidar_v3_xilinx_frame_service_t *service,
    const lidar_v2_device_t *device,
    lidar_v3_xilinx_vdma_adapter_t *adapter,
    lidar_v3_xilinx_frame_consumer_t consumer,
    void *consumer_context)
{
    if (service == NULL) {
        return;
    }
    memset(service, 0, sizeof(*service));
    service->device = device;
    service->adapter = adapter;
    service->consumer = consumer;
    service->consumer_context = consumer_context;
}

int lidar_v3_xilinx_service_completed_frames(void *context)
{
    lidar_v3_xilinx_frame_service_t *service =
        (lidar_v3_xilinx_frame_service_t *)context;
    uint32_t derived_face;
    uint16_t planned_shots;
    int status;

    if (service == NULL || !device_is_valid(service->device) ||
        service->adapter == NULL || service->consumer == NULL) {
        return XST_INVALID_PARAM;
    }
    if (service->held_frame_valid) {
        status = deliver_held_frame(service);
        return status == XST_NO_DATA ? XST_SUCCESS : status;
    }
    if ((service->adapter->rise == NULL ||
         mask_load(&service->adapter->rise->ready_mask) == 0U) &&
        (service->adapter->fall == NULL ||
         mask_load(&service->adapter->fall->ready_mask) == 0U)) {
        return XST_SUCCESS;
    }

    derived_face = lidar_v2_read_stat(
        service->device, LIDAR_V2_STAT_DERIVED_FACE);
    planned_shots = (uint16_t)(derived_face >> 16U);
    if (planned_shots == 0U) {
        return XST_FAILURE;
    }
    status = service_lane(
        service, LIDAR_V3_VDMA_LANE_RISE, planned_shots);
    if (status == XST_NO_DATA) {
        return XST_SUCCESS;
    }
    if (status != XST_SUCCESS) {
        return XST_FAILURE;
    }
    status = service_lane(
        service, LIDAR_V3_VDMA_LANE_FALL, planned_shots);
    if (status == XST_NO_DATA) {
        return XST_SUCCESS;
    }
    if (status != XST_SUCCESS) {
        return XST_FAILURE;
    }
    return XST_SUCCESS;
}

int lidar_v3_xilinx_prepare_commit_step(void *context)
{
    lidar_v3_xilinx_frame_service_t *service =
        (lidar_v3_xilinx_frame_service_t *)context;
    uint32_t operation;
    int status;

    if (service == NULL || !device_is_valid(service->device) ||
        service->adapter == NULL || service->consumer == NULL) {
        return XST_INVALID_PARAM;
    }
    operation = lidar_v2_read_stat(
        service->device, LIDAR_V2_STAT_ACTIVE_VERSION);
    if ((operation & LIDAR_V2_OP_COMMAND_READY) == 0U ||
        (operation & (LIDAR_V2_OP_RUNNING | LIDAR_V2_OP_ARMED |
            LIDAR_V2_OP_COMMAND_BUSY)) != 0U) {
        /* A snapshot captured before a later RUN is not proof for next STOP. */
        service->idle_capture_pending = false;
        return XST_DEVICE_BUSY;
    }
    /* Every step requires a fresh snapshot, including recovery after a prior
     * pre-COMMIT timeout that already left VDMA quiesced. */
    status = poll_processing_idle_snapshot(service);
    if (status != XST_SUCCESS) {
        return status;
    }
    if (!all_lanes_are_quiesced(service->adapter)) {
        status = lidar_v3_xilinx_vdma_quiesce_for_commit(service->adapter);
        if (status != XST_SUCCESS) {
            return status;
        }
    }
    status = lidar_v3_xilinx_service_completed_frames(service);
    if (status != XST_SUCCESS) {
        return status;
    }
    if (lane_has_fatal_status(service->adapter->rise) ||
        lane_has_fatal_status(service->adapter->fall)) {
        return XST_FAILURE;
    }
    if (service->held_frame_valid ||
        !lidar_v3_xilinx_vdma_buffers_idle(service->adapter)) {
        return XST_NO_DATA;
    }
    return XST_SUCCESS;
}

int lidar_v3_xilinx_prepare_commit_service(void *context)
{
    int status = lidar_v3_xilinx_prepare_commit_step(context);

    if (status == XST_SUCCESS) {
        return LIDAR_V3_VDMA_PRECOMMIT_READY;
    }
    if (status == XST_NO_DATA || status == XST_DEVICE_BUSY) {
        return LIDAR_V3_VDMA_PRECOMMIT_RETRY;
    }
    return LIDAR_V3_VDMA_PRECOMMIT_FATAL;
}

int lidar_v3_xilinx_frame_service_bind_transaction(
    lidar_v3_xilinx_frame_service_t *service,
    lidar_v3_vdma_transaction_t *transaction)
{
    if (service == NULL || transaction == NULL ||
        service->device == NULL || transaction->device != service->device ||
        transaction->apply_context != service->adapter) {
        return XST_INVALID_PARAM;
    }
    lidar_v3_vdma_transaction_set_precommit_service(transaction,
        lidar_v3_xilinx_prepare_commit_service, service);
    lidar_v3_vdma_transaction_set_progress_service(transaction,
        lidar_v3_xilinx_service_completed_frames, service);
    return XST_SUCCESS;
}
