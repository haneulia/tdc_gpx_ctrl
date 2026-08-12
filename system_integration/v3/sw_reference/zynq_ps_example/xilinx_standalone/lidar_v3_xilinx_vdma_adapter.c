#include "lidar_v3_xilinx_vdma_adapter.h"

#include <string.h>

#include "xil_cache.h"
#include "xstatus.h"

static bool lane_id_is_valid(lidar_v3_vdma_lane_id_t lane_id)
{
    return lane_id == LIDAR_V3_VDMA_LANE_RISE ||
        lane_id == LIDAR_V3_VDMA_LANE_FALL;
}

static bool *select_quiesced_state(
    lidar_v3_xilinx_vdma_adapter_t *adapter,
    lidar_v3_vdma_lane_id_t lane_id)
{
    return lane_id == LIDAR_V3_VDMA_LANE_RISE ?
        &adapter->rise_quiesced : &adapter->fall_quiesced;
}

static lidar_v2_xilinx_vdma_lane_t *select_lane(
    lidar_v3_xilinx_vdma_adapter_t *adapter,
    lidar_v3_vdma_lane_id_t lane_id)
{
    return lane_id == LIDAR_V3_VDMA_LANE_RISE ?
        adapter->rise : adapter->fall;
}

static size_t profile_frame_bytes(
    const lidar_v2_vdma_lane_profile_t *profile)
{
    return (size_t)profile->stride_bytes * profile->vsize_lines;
}

static uint32_t mask_load(const volatile uint32_t *mask)
{
    return __atomic_load_n(mask, __ATOMIC_ACQUIRE);
}

static void mask_set(volatile uint32_t *mask, uint32_t bits)
{
    (void)__atomic_fetch_or(mask, bits, __ATOMIC_ACQ_REL);
}

static void mask_clear(volatile uint32_t *mask, uint32_t bits)
{
    (void)__atomic_fetch_and(mask, ~bits, __ATOMIC_ACQ_REL);
}

static bool lane_buffers_idle(const lidar_v2_xilinx_vdma_lane_t *lane)
{
    return lane == NULL ||
        (mask_load(&lane->ready_mask) == 0U &&
         mask_load(&lane->cpu_owned_mask) == 0U);
}

static int configure_s2mm_frame_interrupt(
    lidar_v2_xilinx_vdma_lane_t *lane)
{
    XAxiVdma_FrameCounter counter;

    if (lane == NULL || lane->instance == NULL ||
        lane->instance->HasMm2S != 0 || lane->instance->HasS2Mm == 0 ||
        lane->instance->WriteChannel.IsValid == 0 ||
        lane->frame_store_count < LIDAR_V3_XILINX_MIN_FRAME_STORES ||
        lane->frame_store_count !=
            (uint8_t)lane->instance->WriteChannel.NumFrames) {
        return XST_INVALID_PARAM;
    }

    /*
     * XAxiVdma_SetFrameCounter() requires both count fields to be nonzero,
     * even though this design deliberately instantiates no MM2S channel.
     * The invalid MM2S channel is skipped by the AMD driver; only S2MM is
     * programmed. A threshold of one preserves one completion event per Face.
     * XAxiVdma_DmaConfig() later clears only the transfer-stop enable bit and
     * deliberately preserves this interrupt-coalescing threshold field.
     */
    memset(&counter, 0, sizeof(counter));
    counter.ReadFrameCount = LIDAR_V3_XILINX_FRAME_IRQ_THRESHOLD;
    counter.WriteFrameCount = LIDAR_V3_XILINX_FRAME_IRQ_THRESHOLD;
    return XAxiVdma_SetFrameCounter(lane->instance, &counter);
}

void lidar_v3_xilinx_vdma_completion_callback(
    void *callback_reference,
    uint32_t interrupt_types)
{
    lidar_v2_xilinx_vdma_lane_t *lane =
        (lidar_v2_xilinx_vdma_lane_t *)callback_reference;
    uint32_t current;
    uint32_t completed;
    uint32_t mask;

    if (lane == NULL || lane->instance == NULL ||
        lane->frame_store_count == 0U ||
        lane->frame_store_count > XAXIVDMA_MAX_FRAMESTORE ||
        (interrupt_types & XAXIVDMA_IXR_FRMCNT_MASK) == 0U) {
        return;
    }
    current = XAxiVdma_CurrFrameStore(lane->instance, XAXIVDMA_WRITE);
    if (current >= lane->frame_store_count) {
        mask_set(&lane->error_mask, XAXIVDMA_IXR_ERROR_MASK);
        return;
    }
    completed = (current + lane->frame_store_count - 1U) %
        lane->frame_store_count;
    mask = 1U << completed;
    if ((mask_load(&lane->ready_mask) & mask) != 0U ||
        (mask_load(&lane->cpu_owned_mask) & mask) != 0U) {
        mask_set(&lane->overwrite_mask, mask);
    }
    mask_set(&lane->ready_mask, mask);
}

void lidar_v3_xilinx_vdma_error_callback(
    void *callback_reference,
    uint32_t error_mask)
{
    lidar_v2_xilinx_vdma_lane_t *lane =
        (lidar_v2_xilinx_vdma_lane_t *)callback_reference;

    if (lane != NULL) {
        mask_set(&lane->error_mask, error_mask);
    }
}

static int quiesce_lane(
    lidar_v2_xilinx_vdma_lane_t *lane,
    uint32_t poll_limit)
{
    uint32_t poll;
    uint32_t pending;

    XAxiVdma_IntrDisable(lane->instance, XAXIVDMA_IXR_ALL_MASK,
        XAXIVDMA_WRITE);
    XAxiVdma_DmaStop(lane->instance, XAXIVDMA_WRITE);
    for (poll = 0U; poll < poll_limit; ++poll) {
        if (!XAxiVdma_IsBusy(lane->instance, XAXIVDMA_WRITE)) {
            break;
        }
    }
    if (poll == poll_limit) {
        return XST_FAILURE;
    }
    lane->running = false;

    /* Recover a final frame/error that completed after the software mask test. */
    pending = XAxiVdma_IntrGetPending(lane->instance, XAXIVDMA_WRITE);
    XAxiVdma_IntrClear(lane->instance, pending, XAXIVDMA_WRITE);
    if ((pending & XAXIVDMA_IXR_ERROR_MASK) != 0U) {
        lidar_v3_xilinx_vdma_error_callback(
            lane, pending & XAXIVDMA_IXR_ERROR_MASK);
    } else if ((pending & XAXIVDMA_IXR_FRMCNT_MASK) != 0U) {
        lidar_v3_xilinx_vdma_completion_callback(lane, pending);
    }
    return XST_SUCCESS;
}

int lidar_v3_xilinx_vdma_adapter_init(
    lidar_v3_xilinx_vdma_adapter_t *adapter,
    lidar_v2_xilinx_vdma_lane_t *rise,
    lidar_v2_xilinx_vdma_lane_t *fall,
    uint32_t stop_poll_limit)
{
    lidar_v3_dma_region_t regions[2U * XAXIVDMA_MAX_FRAMESTORE];
    lidar_v2_xilinx_vdma_lane_t *lanes[2];
    size_t region_count = 0U;
    size_t lane_index;
    uint8_t frame_index;

    if (adapter == NULL) {
        return XST_INVALID_PARAM;
    }
    memset(adapter, 0, sizeof(*adapter));
    adapter->rise = rise;
    adapter->fall = fall;
    adapter->stop_poll_limit = stop_poll_limit;
    if (stop_poll_limit == 0U || (rise == NULL && fall == NULL)) {
        return XST_INVALID_PARAM;
    }
    if (rise != NULL && fall != NULL &&
        (rise->instance == fall->instance ||
         rise->output_width_bits != fall->output_width_bits)) {
        return XST_INVALID_PARAM;
    }

    lanes[0] = rise;
    lanes[1] = fall;
    for (lane_index = 0U; lane_index < 2U; ++lane_index) {
        lidar_v2_xilinx_vdma_lane_t *lane = lanes[lane_index];

        if (lane == NULL) {
            continue;
        }
        if (lane->instance == NULL ||
            lane->frame_store_count < LIDAR_V3_XILINX_MIN_FRAME_STORES ||
            lane->frame_store_count > XAXIVDMA_MAX_FRAMESTORE ||
            lane->frame_capacity_bytes == 0U ||
            (lane->output_width_bits != 32U &&
             lane->output_width_bits != 64U)) {
            return XST_INVALID_PARAM;
        }
        for (frame_index = 0U;
             frame_index < lane->frame_store_count;
             ++frame_index) {
            regions[region_count].base_address =
                lane->frame_addresses[frame_index];
            regions[region_count].capacity_bytes =
                lane->frame_capacity_bytes;
            ++region_count;
        }
    }
    if (lidar_v3_vdma_validate_regions(regions, region_count,
            LIDAR_V3_XILINX_DMA_ALIGNMENT_BYTES) != LIDAR_V3_VDMA_OK) {
        return XST_INVALID_PARAM;
    }
    for (lane_index = 0U; lane_index < 2U; ++lane_index) {
        lidar_v2_xilinx_vdma_lane_t *lane = lanes[lane_index];
        int status;

        if (lane == NULL) {
            continue;
        }
        status = configure_s2mm_frame_interrupt(lane);
        if (status != XST_SUCCESS) {
            return status;
        }
        if (XAxiVdma_SetCallBack(lane->instance,
                XAXIVDMA_HANDLER_GENERAL,
                (void *)lidar_v3_xilinx_vdma_completion_callback,
                lane, XAXIVDMA_WRITE) != XST_SUCCESS ||
            XAxiVdma_SetCallBack(lane->instance,
                XAXIVDMA_HANDLER_ERROR,
                (void *)lidar_v3_xilinx_vdma_error_callback,
                lane, XAXIVDMA_WRITE) != XST_SUCCESS) {
            return XST_FAILURE;
        }
    }
    adapter->buffers_validated = true;
    return XST_SUCCESS;
}

int lidar_v3_xilinx_vdma_quiesce_for_commit(
    lidar_v3_xilinx_vdma_adapter_t *adapter)
{
    lidar_v2_xilinx_vdma_lane_t *lanes[2];
    bool *quiesced[2];
    size_t lane_index;

    if (adapter == NULL || !adapter->buffers_validated ||
        adapter->stop_poll_limit == 0U) {
        return XST_INVALID_PARAM;
    }
    lanes[0] = adapter->rise;
    lanes[1] = adapter->fall;
    quiesced[0] = &adapter->rise_quiesced;
    quiesced[1] = &adapter->fall_quiesced;
    for (lane_index = 0U; lane_index < 2U; ++lane_index) {
        lidar_v2_xilinx_vdma_lane_t *lane = lanes[lane_index];

        if (lane == NULL || *quiesced[lane_index]) {
            continue;
        }
        if (mask_load(&lane->overwrite_mask) != 0U ||
            mask_load(&lane->error_mask) != 0U ||
            quiesce_lane(lane, adapter->stop_poll_limit) != XST_SUCCESS) {
            return XST_FAILURE;
        }
        *quiesced[lane_index] = true;
        if (mask_load(&lane->overwrite_mask) != 0U ||
            mask_load(&lane->error_mask) != 0U) {
            return XST_FAILURE;
        }
    }
    return XST_SUCCESS;
}

bool lidar_v3_xilinx_vdma_buffers_idle(
    const lidar_v3_xilinx_vdma_adapter_t *adapter)
{
    if (adapter == NULL || !adapter->buffers_validated) {
        return false;
    }
    return lane_buffers_idle(adapter->rise) &&
        lane_buffers_idle(adapter->fall);
}

lidar_v3_vdma_apply_result_t lidar_v3_xilinx_vdma_apply_lane(
    void *context,
    lidar_v3_vdma_lane_id_t lane_id,
    const lidar_v2_vdma_lane_profile_t *profile)
{
    lidar_v3_xilinx_vdma_adapter_t *adapter =
        (lidar_v3_xilinx_vdma_adapter_t *)context;
    lidar_v2_xilinx_vdma_lane_t *lane;
    bool *quiesced;
    int status;

    if (adapter == NULL || profile == NULL ||
        !adapter->buffers_validated ||
        !lane_id_is_valid(lane_id) ||
        adapter->stop_poll_limit == 0U) {
        return LIDAR_V3_VDMA_APPLY_FATAL;
    }
    lane = select_lane(adapter, lane_id);
    quiesced = select_quiesced_state(adapter, lane_id);
    if (lane == NULL) {
        return profile->enabled ? LIDAR_V3_VDMA_APPLY_FATAL :
            LIDAR_V3_VDMA_APPLY_OK;
    }
    if (lane->instance == NULL || lane->frame_store_count == 0U ||
        lane->frame_store_count > XAXIVDMA_MAX_FRAMESTORE) {
        return LIDAR_V3_VDMA_APPLY_FATAL;
    }
    if (lidar_v3_vdma_validate_profile(profile, lane->output_width_bits,
            lane->frame_capacity_bytes) != LIDAR_V3_VDMA_OK) {
        return LIDAR_V3_VDMA_APPLY_FATAL;
    }

    if (mask_load(&lane->overwrite_mask) != 0U ||
        mask_load(&lane->error_mask) != 0U) {
        return LIDAR_V3_VDMA_APPLY_FATAL;
    }
    /* Let the progress service consume already-known frames before stopping. */
    if (mask_load(&lane->cpu_owned_mask) != 0U ||
        mask_load(&lane->ready_mask) != 0U) {
        return LIDAR_V3_VDMA_APPLY_RETRY;
    }
    if (!*quiesced) {
        if (quiesce_lane(lane, adapter->stop_poll_limit) != XST_SUCCESS) {
            return LIDAR_V3_VDMA_APPLY_FATAL;
        }
        *quiesced = true;
    }
    if (mask_load(&lane->overwrite_mask) != 0U ||
        mask_load(&lane->error_mask) != 0U) {
        return LIDAR_V3_VDMA_APPLY_FATAL;
    }
    /* Geometry cannot change while CPU still owns or has not consumed a frame. */
    if (mask_load(&lane->cpu_owned_mask) != 0U ||
        mask_load(&lane->ready_mask) != 0U) {
        return LIDAR_V3_VDMA_APPLY_RETRY;
    }
    status = lidar_v2_xilinx_vdma_program(
        lane, profile, adapter->stop_poll_limit);
    if (status != XST_SUCCESS) {
        return LIDAR_V3_VDMA_APPLY_FATAL;
    }
    *quiesced = !profile->enabled;
    return LIDAR_V3_VDMA_APPLY_OK;
}

int lidar_v3_xilinx_vdma_take_completed(
    lidar_v3_xilinx_vdma_adapter_t *adapter,
    lidar_v3_vdma_lane_id_t lane_id,
    uint16_t planned_shots,
    uint8_t *frame_index,
    lidar_ps_ddr_frame_t *frame)
{
    lidar_v2_xilinx_vdma_lane_t *lane;
    bool *quiesced;
    uint32_t current_frame_store;
    uint8_t index;
    uint8_t offset;
    uint32_t mask = 0U;
    size_t frame_bytes;

    if (adapter == NULL || !adapter->buffers_validated ||
        frame_index == NULL || frame == NULL ||
        !lane_id_is_valid(lane_id) ||
        planned_shots == 0U) {
        return XST_INVALID_PARAM;
    }
    lane = select_lane(adapter, lane_id);
    quiesced = select_quiesced_state(adapter, lane_id);
    if (lane == NULL || lane->instance == NULL ||
        lane->frame_store_count == 0U ||
        lane->frame_store_count > XAXIVDMA_MAX_FRAMESTORE ||
        (!lane->running && !*quiesced) ||
        !lane->active_profile.enabled) {
        return XST_INVALID_PARAM;
    }
    if (mask_load(&lane->overwrite_mask) != 0U ||
        mask_load(&lane->error_mask) != 0U) {
        return XST_FAILURE;
    }

    XAxiVdma_IntrDisable(lane->instance, XAXIVDMA_IXR_FRMCNT_MASK,
        XAXIVDMA_WRITE);
    current_frame_store = XAxiVdma_CurrFrameStore(
        lane->instance, XAXIVDMA_WRITE);
    if (current_frame_store >= lane->frame_store_count) {
        if (!*quiesced) {
            XAxiVdma_IntrEnable(lane->instance, XAXIVDMA_IXR_FRMCNT_MASK,
                XAXIVDMA_WRITE);
        }
        return XST_FAILURE;
    }
    if ((mask_load(&lane->ready_mask) &
            (1U << current_frame_store)) != 0U) {
        mask_set(&lane->overwrite_mask, 1U << current_frame_store);
        if (!*quiesced) {
            XAxiVdma_IntrEnable(lane->instance, XAXIVDMA_IXR_FRMCNT_MASK,
                XAXIVDMA_WRITE);
        }
        return XST_FAILURE;
    }
    /* Search oldest-to-newest behind the Frame Store hardware is using. */
    for (offset = 1U; offset < lane->frame_store_count; ++offset) {
        index = (uint8_t)((current_frame_store + offset) %
            lane->frame_store_count);
        mask = 1U << index;
        if ((mask_load(&lane->ready_mask) & mask) != 0U) {
            break;
        }
    }
    if (offset >= lane->frame_store_count) {
        if (!*quiesced) {
            XAxiVdma_IntrEnable(lane->instance, XAXIVDMA_IXR_FRMCNT_MASK,
                XAXIVDMA_WRITE);
        }
        return XST_NO_DATA;
    }
    mask_clear(&lane->ready_mask, mask);
    mask_set(&lane->cpu_owned_mask, mask);
    if (!*quiesced) {
        XAxiVdma_IntrEnable(lane->instance, XAXIVDMA_IXR_FRMCNT_MASK,
            XAXIVDMA_WRITE);
    }

    frame_bytes = profile_frame_bytes(&lane->active_profile);
    Xil_DCacheInvalidateRange(
        lane->frame_addresses[index], (uint32_t)frame_bytes);
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
        mask_clear(&lane->cpu_owned_mask, mask);
        mask_set(&lane->ready_mask, mask);
        return XST_FAILURE;
    }
    *frame_index = index;
    return XST_SUCCESS;
}

int lidar_v3_xilinx_vdma_release_frame(
    lidar_v3_xilinx_vdma_adapter_t *adapter,
    lidar_v3_vdma_lane_id_t lane_id,
    uint8_t frame_index,
    lidar_ps_ddr_frame_t *frame)
{
    lidar_v2_xilinx_vdma_lane_t *lane;
    bool *quiesced;

    if (adapter == NULL || !adapter->buffers_validated || frame == NULL ||
        !lane_id_is_valid(lane_id)) {
        return XST_INVALID_PARAM;
    }
    lane = select_lane(adapter, lane_id);
    quiesced = select_quiesced_state(adapter, lane_id);
    if (lane == NULL || lane->instance == NULL ||
        frame_index >= lane->frame_store_count ||
        frame->owner != LIDAR_PS_BUFFER_CPU_OWNED ||
        frame->base != (const uint8_t *)lane->frame_addresses[frame_index] ||
        (mask_load(&lane->cpu_owned_mask) &
            (1U << frame_index)) == 0U) {
        return XST_INVALID_PARAM;
    }
    lidar_ps_release_to_dma(frame);
    XAxiVdma_IntrDisable(lane->instance, XAXIVDMA_IXR_FRMCNT_MASK,
        XAXIVDMA_WRITE);
    mask_clear(&lane->cpu_owned_mask, 1U << frame_index);
    if (!*quiesced) {
        XAxiVdma_IntrEnable(lane->instance, XAXIVDMA_IXR_FRMCNT_MASK,
            XAXIVDMA_WRITE);
    }
    return XST_SUCCESS;
}
