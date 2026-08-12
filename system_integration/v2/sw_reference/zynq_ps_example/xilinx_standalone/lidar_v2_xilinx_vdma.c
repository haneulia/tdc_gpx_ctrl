#include "lidar_v2_xilinx_vdma.h"

#include <string.h>

#include "xil_cache.h"
#include "xstatus.h"

#define LIDAR_V2_VDMA_STOP_POLL_LIMIT 1000000U

static size_t profile_frame_bytes(
    const lidar_v2_vdma_lane_profile_t *profile)
{
    return (size_t)profile->stride_bytes * profile->vsize_lines;
}

static int validate_profile(
    const lidar_v2_xilinx_vdma_lane_t *lane,
    const lidar_v2_vdma_lane_profile_t *profile)
{
    uint16_t beat_bytes;
    size_t frame_bytes;
    uint8_t index;

    if (lane == NULL || profile == NULL || lane->instance == NULL) {
        return XST_INVALID_PARAM;
    }
    if (!profile->enabled) {
        return XST_SUCCESS;
    }
    beat_bytes = (uint16_t)(lane->output_width_bits / 8U);
    frame_bytes = profile_frame_bytes(profile);
    if ((lane->output_width_bits != 32U &&
         lane->output_width_bits != 64U &&
         lane->output_width_bits != 128U) ||
        profile->hsize_bytes == 0U || profile->vsize_lines == 0U ||
        profile->stride_bytes < profile->hsize_bytes ||
        (profile->hsize_bytes % beat_bytes) != 0U ||
        (profile->stride_bytes % beat_bytes) != 0U ||
        frame_bytes > lane->frame_capacity_bytes) {
        return XST_INVALID_PARAM;
    }
    for (index = 0U; index < lane->frame_store_count; ++index) {
        if ((lane->frame_addresses[index] % beat_bytes) != 0U) {
            return XST_INVALID_PARAM;
        }
    }
    return XST_SUCCESS;
}

static int stop_channel(
    lidar_v2_xilinx_vdma_lane_t *lane,
    uint32_t poll_limit)
{
    uint32_t poll;

    XAxiVdma_IntrDisable(lane->instance, XAXIVDMA_IXR_ALL_MASK,
        XAXIVDMA_WRITE);
    XAxiVdma_DmaStop(lane->instance, XAXIVDMA_WRITE);
    for (poll = 0U; poll < poll_limit; ++poll) {
        if (!XAxiVdma_IsBusy(lane->instance, XAXIVDMA_WRITE)) {
            lane->running = false;
            return XST_SUCCESS;
        }
    }
    return XST_FAILURE;
}

int lidar_v2_xilinx_vdma_lane_init(
    lidar_v2_xilinx_vdma_lane_t *lane,
    XAxiVdma *instance,
    const UINTPTR *frame_addresses,
    uint8_t frame_store_count,
    size_t frame_capacity_bytes,
    uint16_t output_width_bits)
{
    uint8_t index;

    if (lane == NULL || instance == NULL || frame_addresses == NULL ||
        frame_store_count == 0U ||
        frame_store_count > XAXIVDMA_MAX_FRAMESTORE ||
        frame_store_count != (uint8_t)instance->WriteChannel.NumFrames ||
        frame_capacity_bytes == 0U) {
        return XST_INVALID_PARAM;
    }
    memset(lane, 0, sizeof(*lane));
    lane->instance = instance;
    lane->frame_store_count = frame_store_count;
    lane->frame_capacity_bytes = frame_capacity_bytes;
    lane->output_width_bits = output_width_bits;
    for (index = 0U; index < frame_store_count; ++index) {
        lane->frame_addresses[index] = frame_addresses[index];
    }

    if (XAxiVdma_SetCallBack(instance, XAXIVDMA_HANDLER_GENERAL,
            (void *)lidar_v2_xilinx_vdma_completion_callback, lane,
            XAXIVDMA_WRITE) != XST_SUCCESS ||
        XAxiVdma_SetCallBack(instance, XAXIVDMA_HANDLER_ERROR,
            (void *)lidar_v2_xilinx_vdma_error_callback, lane,
            XAXIVDMA_WRITE) != XST_SUCCESS) {
        return XST_FAILURE;
    }
    return XST_SUCCESS;
}

int lidar_v2_xilinx_vdma_program(
    lidar_v2_xilinx_vdma_lane_t *lane,
    const lidar_v2_vdma_lane_profile_t *profile,
    uint32_t stop_poll_limit)
{
    XAxiVdma_DmaSetup setup;
    UINTPTR addresses[XAXIVDMA_MAX_FRAMESTORE];
    size_t frame_bytes;
    uint8_t index;
    int status;

    status = validate_profile(lane, profile);
    if (status != XST_SUCCESS) {
        return status;
    }
    status = stop_channel(lane, stop_poll_limit);
    if (status != XST_SUCCESS) {
        return status;
    }
    lane->ready_mask = 0U;
    lane->cpu_owned_mask = 0U;
    lane->overwrite_mask = 0U;
    lane->error_mask = 0U;
    lane->active_profile = *profile;
    if (!profile->enabled) {
        return XST_SUCCESS;
    }

    frame_bytes = profile_frame_bytes(profile);
    memset(&setup, 0, sizeof(setup));
    memset(addresses, 0, sizeof(addresses));
    setup.VertSizeInput = profile->vsize_lines;
    setup.HoriSizeInput = profile->hsize_bytes;
    setup.Stride = profile->stride_bytes;
    setup.FrameDelay = 0;
    setup.EnableCircularBuf = 1;
    setup.EnableSync = 0;
    setup.PointNum = 0;
    /*
     * Keep circular S2MM running across Face Frames.  This bit controls the
     * transfer-stop frame counter, not the completion-interrupt coalescing
     * threshold.  V3 separately sets that threshold to one Face Frame with
     * XAxiVdma_SetFrameCounter(); XAxiVdma_DmaConfig() preserves its [23:16]
     * register field while this zero keeps the DMA in endless-transfer mode.
     */
    setup.EnableFrameCounter = 0;
    setup.FixedFrameStoreAddr = 0;
    setup.EnableVFlip = 0U;

    status = XAxiVdma_DmaConfig(lane->instance, XAXIVDMA_WRITE, &setup);
    if (status != XST_SUCCESS) {
        return status;
    }
    for (index = 0U; index < lane->frame_store_count; ++index) {
        addresses[index] = lane->frame_addresses[index];
        /* Remove dirty CPU lines before S2MM owns this range. */
        Xil_DCacheFlushRange(addresses[index], (u32)frame_bytes);
    }
    status = XAxiVdma_DmaSetBufferAddr(
        lane->instance, XAXIVDMA_WRITE, addresses);
    if (status != XST_SUCCESS) {
        return status;
    }
    status = XAxiVdma_DmaStart(lane->instance, XAXIVDMA_WRITE);
    if (status != XST_SUCCESS) {
        return status;
    }
    XAxiVdma_IntrEnable(lane->instance,
        XAXIVDMA_IXR_FRMCNT_MASK | XAXIVDMA_IXR_ERROR_MASK,
        XAXIVDMA_WRITE);
    lane->running = true;
    return XST_SUCCESS;
}

int lidar_v2_xilinx_service_vdma_profiles(void *context)
{
    lidar_v2_xilinx_vdma_service_t *service =
        (lidar_v2_xilinx_vdma_service_t *)context;
    lidar_v2_vdma_profiles_t profiles;
    bool rise_ack = false;
    bool fall_ack = false;

    if (service == NULL || service->device == NULL) {
        return -1;
    }
    if (lidar_v2_read_vdma_profiles(service->device, &profiles) !=
        LIDAR_V2_CONTROL_OK) {
        return -1;
    }
    if (profiles.rise.pending) {
        if (service->rise == NULL ||
            lidar_v2_xilinx_vdma_program(service->rise, &profiles.rise,
                LIDAR_V2_VDMA_STOP_POLL_LIMIT) != XST_SUCCESS) {
            return -1;
        }
        rise_ack = true;
    }
    if (profiles.fall.pending) {
        if (profiles.fall.enabled && service->fall == NULL) {
            return -1;
        }
        if (service->fall != NULL &&
            lidar_v2_xilinx_vdma_program(service->fall, &profiles.fall,
                LIDAR_V2_VDMA_STOP_POLL_LIMIT) != XST_SUCCESS) {
            return -1;
        }
        fall_ack = true;
    }
    if (lidar_v2_ack_vdma_profiles(
            service->device, rise_ack, fall_ack) != LIDAR_V2_CONTROL_OK) {
        return -1;
    }
    return 0;
}

void lidar_v2_xilinx_vdma_completion_callback(
    void *callback_reference,
    uint32_t interrupt_types)
{
    lidar_v2_xilinx_vdma_lane_t *lane =
        (lidar_v2_xilinx_vdma_lane_t *)callback_reference;
    uint32_t current;
    uint32_t completed;
    uint32_t mask;

    if (lane == NULL ||
        (interrupt_types & XAXIVDMA_IXR_FRMCNT_MASK) == 0U) {
        return;
    }
    current = XAxiVdma_CurrFrameStore(lane->instance, XAXIVDMA_WRITE);
    completed = (current + lane->frame_store_count - 1U) %
        lane->frame_store_count;
    mask = 1U << completed;
    if ((lane->ready_mask & mask) != 0U ||
        (lane->cpu_owned_mask & mask) != 0U) {
        lane->overwrite_mask |= mask;
    }
    lane->ready_mask |= mask;
}

void lidar_v2_xilinx_vdma_error_callback(
    void *callback_reference,
    uint32_t error_mask)
{
    lidar_v2_xilinx_vdma_lane_t *lane =
        (lidar_v2_xilinx_vdma_lane_t *)callback_reference;

    if (lane != NULL) {
        lane->error_mask |= error_mask;
    }
}

int lidar_v2_xilinx_vdma_take_completed(
    lidar_v2_xilinx_vdma_lane_t *lane,
    uint16_t planned_shots,
    uint8_t *frame_index,
    lidar_ps_ddr_frame_t *frame)
{
    uint8_t index;
    uint32_t mask = 0U;
    size_t frame_bytes;

    if (lane == NULL || frame_index == NULL || frame == NULL ||
        !lane->running || !lane->active_profile.enabled ||
        planned_shots == 0U) {
        return XST_INVALID_PARAM;
    }
    XAxiVdma_IntrDisable(lane->instance, XAXIVDMA_IXR_FRMCNT_MASK,
        XAXIVDMA_WRITE);
    for (index = 0U; index < lane->frame_store_count; ++index) {
        mask = 1U << index;
        if ((lane->ready_mask & mask) != 0U) {
            break;
        }
    }
    if (index == lane->frame_store_count) {
        XAxiVdma_IntrEnable(lane->instance, XAXIVDMA_IXR_FRMCNT_MASK,
            XAXIVDMA_WRITE);
        return XST_NO_DATA;
    }
    lane->ready_mask &= ~mask;
    lane->cpu_owned_mask |= mask;
    XAxiVdma_IntrEnable(lane->instance, XAXIVDMA_IXR_FRMCNT_MASK,
        XAXIVDMA_WRITE);

    frame_bytes = profile_frame_bytes(&lane->active_profile);
    Xil_DCacheInvalidateRange(lane->frame_addresses[index], (u32)frame_bytes);
    memset(frame, 0, sizeof(*frame));
    frame->base = (const uint8_t *)lane->frame_addresses[index];
    frame->allocation_bytes = lane->frame_capacity_bytes;
    frame->hsize_bytes = lane->active_profile.hsize_bytes;
    frame->stride_bytes = lane->active_profile.stride_bytes;
    frame->vsize_lines = lane->active_profile.vsize_lines;
    frame->planned_shots = planned_shots;
    frame->output_width_bits = lane->output_width_bits;
    frame->owner = LIDAR_PS_BUFFER_DMA_OWNED;
    if (lidar_ps_publish_cpu_owned(frame, true) != LIDAR_PS_OK) {
        lane->cpu_owned_mask &= ~mask;
        return XST_FAILURE;
    }
    *frame_index = index;
    return XST_SUCCESS;
}

void lidar_v2_xilinx_vdma_release_frame(
    lidar_v2_xilinx_vdma_lane_t *lane,
    uint8_t frame_index,
    lidar_ps_ddr_frame_t *frame)
{
    if (lane == NULL || frame == NULL ||
        frame_index >= lane->frame_store_count) {
        return;
    }
    lidar_ps_release_to_dma(frame);
    XAxiVdma_IntrDisable(lane->instance, XAXIVDMA_IXR_FRMCNT_MASK,
        XAXIVDMA_WRITE);
    lane->cpu_owned_mask &= ~(1U << frame_index);
    XAxiVdma_IntrEnable(lane->instance, XAXIVDMA_IXR_FRMCNT_MASK,
        XAXIVDMA_WRITE);
}
