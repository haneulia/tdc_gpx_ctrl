#include "lidar_v3_xilinx_vdma_adapter.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "xil_cache.h"
#include "xstatus.h"
#include "lidar_v3_xilinx_frame_service.h"

#define TEST_CSR_BASE 0x40000000U
#define TEST_WORD_COUNT 68U
#define TEST_MAX_VDMA_INSTANCES 64U

static uint32_t program_calls;
static int program_result;
static uint32_t pending_interrupts;
static uint32_t current_frame_store;
static uint32_t interrupt_disable_calls;
static uint32_t interrupt_enable_calls;
static uint32_t stop_calls;
static uint32_t start_calls;
static uint32_t interrupt_clear_calls;
static uintptr_t cache_invalidate_address;
static uint32_t cache_invalidate_length;
static uint32_t fake_csr_words[TEST_WORD_COUNT];
static uint32_t acknowledge_count;
static uint32_t consumer_calls[2];
static uint32_t consumer_retries_remaining;
static uint32_t consumer_failures_remaining;
static lidar_v2_xilinx_vdma_lane_t *consumer_release_tamper_lane;
static uint32_t callback_registration_count;
static int callback_registration_result;
static uint32_t fake_diag_data;
static uint32_t fake_diag_capture_count;
static XAxiVdma fake_vdma_instances[TEST_MAX_VDMA_INSTANCES];
static size_t fake_vdma_instance_count;
static uint32_t frame_counter_calls;
static int frame_counter_result;
static XAxiVdma_FrameCounter last_frame_counter;

int XAxiVdma_SetFrameCounter(
    XAxiVdma *instance,
    XAxiVdma_FrameCounter *frame_counter)
{
    assert(instance != NULL && frame_counter != NULL);
    ++frame_counter_calls;
    last_frame_counter = *frame_counter;
    if (frame_counter_result != XST_SUCCESS) {
        return frame_counter_result;
    }
    if (instance->WriteChannel.IsValid == 0) {
        return XST_FAILURE;
    }
    if (frame_counter->ReadFrameCount == 0U ||
        frame_counter->WriteFrameCount == 0U) {
        return XST_INVALID_PARAM;
    }
    if ((instance->WriteChannel.DbgFeatureFlags &
            (XAXIVDMA_ENABLE_DBG_FRM_CNTR |
             XAXIVDMA_ENABLE_DBG_DLY_CNTR)) !=
        (XAXIVDMA_ENABLE_DBG_FRM_CNTR |
         XAXIVDMA_ENABLE_DBG_DLY_CNTR)) {
        return XST_NO_FEATURE;
    }
    return XST_SUCCESS;
}

int XAxiVdma_SetCallBack(
    XAxiVdma *instance,
    uint32_t handler_type,
    void *callback,
    void *callback_reference,
    uint16_t direction)
{
    assert(instance != NULL && callback != NULL &&
        callback_reference != NULL && direction == XAXIVDMA_WRITE &&
        (handler_type == XAXIVDMA_HANDLER_GENERAL ||
         handler_type == XAXIVDMA_HANDLER_ERROR));
    ++callback_registration_count;
    return callback_registration_result;
}

static size_t csr_word(uintptr_t address)
{
    size_t word = (size_t)((address - TEST_CSR_BASE) / 4U);

    assert(address >= TEST_CSR_BASE && word < TEST_WORD_COUNT);
    return word;
}

static uint32_t fake_read32(void *context, uintptr_t address)
{
    (void)context;
    return fake_csr_words[csr_word(address)];
}

static void fake_write32(void *context, uintptr_t address, uint32_t value)
{
    size_t word;

    (void)context;
    word = csr_word(address);
    if (word == LIDAR_V2_CTL_OFFSET(
            LIDAR_V2_CTL_VDMA_PROFILE_CONTROL) / 4U) {
        if ((value & LIDAR_V2_VDMA_RISE_ACK) != 0U) {
            fake_csr_words[word] &= ~LIDAR_V2_VDMA_RISE_PENDING;
        }
        if ((value & LIDAR_V2_VDMA_FALL_ACK) != 0U) {
            fake_csr_words[word] &= ~LIDAR_V2_VDMA_FALL_PENDING;
        }
        ++acknowledge_count;
        return;
    }
    if (word == LIDAR_V2_CTL_OFFSET(LIDAR_V2_CTL_DIAG_INDEX) / 4U &&
        (value & LIDAR_V3_DIAG_CAPTURE) != 0U) {
        uint32_t sequence = (fake_csr_words[word] >>
            LIDAR_V3_DIAG_SEQUENCE_SHIFT) + 1U;

        ++fake_diag_capture_count;
        fake_csr_words[LIDAR_V2_CTL_OFFSET(
            LIDAR_V2_CTL_DIAG_DATA) / 4U] = fake_diag_data;
        fake_csr_words[word] =
            ((sequence & 0xFFFFU) << LIDAR_V3_DIAG_SEQUENCE_SHIFT) |
            LIDAR_V3_DIAG_VALID | (value & 0xFFU);
        return;
    }
    fake_csr_words[word] = value;
}

static int consume_frame(
    void *context,
    lidar_v3_vdma_lane_id_t lane_id,
    uint8_t frame_index,
    const lidar_ps_ddr_frame_t *frame)
{
    (void)context;
    assert((size_t)lane_id < 2U && frame_index < 3U && frame != NULL);
    assert(frame->owner == LIDAR_PS_BUFFER_CPU_OWNED);
    ++consumer_calls[(size_t)lane_id];
    if (consumer_retries_remaining != 0U) {
        --consumer_retries_remaining;
        return XST_NO_DATA;
    }
    if (consumer_failures_remaining != 0U) {
        --consumer_failures_remaining;
        return XST_FAILURE;
    }
    if (consumer_release_tamper_lane != NULL) {
        consumer_release_tamper_lane->cpu_owned_mask &=
            ~(1U << frame_index);
        consumer_release_tamper_lane = NULL;
    }
    return XST_SUCCESS;
}

void XAxiVdma_IntrDisable(
    XAxiVdma *instance,
    uint32_t interrupt_types,
    uint16_t direction)
{
    assert(instance != NULL &&
        (interrupt_types == XAXIVDMA_IXR_ALL_MASK ||
         interrupt_types == XAXIVDMA_IXR_FRMCNT_MASK) &&
        direction == XAXIVDMA_WRITE);
    ++interrupt_disable_calls;
}

void XAxiVdma_IntrEnable(
    XAxiVdma *instance,
    uint32_t interrupt_types,
    uint16_t direction)
{
    assert(instance != NULL &&
        interrupt_types == XAXIVDMA_IXR_FRMCNT_MASK &&
        direction == XAXIVDMA_WRITE);
    ++interrupt_enable_calls;
}

void XAxiVdma_DmaStop(XAxiVdma *instance, uint16_t direction)
{
    assert(instance != NULL && direction == XAXIVDMA_WRITE);
    ++stop_calls;
}

int XAxiVdma_DmaStart(XAxiVdma *instance, uint16_t direction)
{
    assert(instance != NULL && direction == XAXIVDMA_WRITE);
    ++start_calls;
    return XST_SUCCESS;
}

void Xil_DCacheInvalidateRange(uintptr_t address, uint32_t length)
{
    cache_invalidate_address = address;
    cache_invalidate_length = length;
}

int XAxiVdma_IsBusy(XAxiVdma *instance, uint16_t direction)
{
    assert(instance != NULL && direction == XAXIVDMA_WRITE);
    return 0;
}

uint32_t XAxiVdma_CurrFrameStore(
    XAxiVdma *instance,
    uint16_t direction)
{
    assert(instance != NULL && direction == XAXIVDMA_WRITE);
    return current_frame_store;
}

uint32_t XAxiVdma_IntrGetPending(XAxiVdma *instance, uint16_t direction)
{
    assert(instance != NULL && direction == XAXIVDMA_WRITE);
    return pending_interrupts;
}

void XAxiVdma_IntrClear(
    XAxiVdma *instance,
    uint32_t interrupt_types,
    uint16_t direction)
{
    assert(instance != NULL && interrupt_types == pending_interrupts &&
        direction == XAXIVDMA_WRITE);
    ++interrupt_clear_calls;
    pending_interrupts = 0U;
}

int lidar_v2_xilinx_vdma_program(
    lidar_v2_xilinx_vdma_lane_t *lane,
    const lidar_v2_vdma_lane_profile_t *profile,
    uint32_t stop_poll_limit)
{
    assert(lane != NULL && profile != NULL && stop_poll_limit != 0U);
    ++program_calls;
    if (program_result == XST_SUCCESS) {
        lane->active_profile = *profile;
        lane->running = profile->enabled;
    }
    return program_result;
}

static void clear_lane(lidar_v2_xilinx_vdma_lane_t *lane)
{
    XAxiVdma *instance;

    assert(fake_vdma_instance_count < TEST_MAX_VDMA_INSTANCES);
    instance = &fake_vdma_instances[fake_vdma_instance_count++];
    memset(instance, 0, sizeof(*instance));
    instance->HasMm2S = 0;
    instance->HasS2Mm = 1;
    instance->WriteChannel.IsValid = 1;
    instance->WriteChannel.NumFrames = 3U;
    instance->WriteChannel.DbgFeatureFlags =
        XAXIVDMA_ENABLE_DBG_FRM_CNTR |
        XAXIVDMA_ENABLE_DBG_DLY_CNTR;

    memset(lane, 0, sizeof(*lane));
    lane->instance = instance;
    lane->output_width_bits = 64U;
    lane->frame_capacity_bytes = 0x200000U;
    lane->frame_store_count = 3U;
    lane->frame_addresses[0] = 0x10000000U;
    lane->frame_addresses[1] = 0x10200000U;
    lane->frame_addresses[2] = 0x10400000U;
    lane->active_profile.enabled = true;
}

static void set_lane_base(
    lidar_v2_xilinx_vdma_lane_t *lane,
    UINTPTR base_address)
{
    uint8_t index;

    for (index = 0U; index < lane->frame_store_count; ++index) {
        lane->frame_addresses[index] = base_address +
            (UINTPTR)index * lane->frame_capacity_bytes;
    }
}

static void test_adapter_init_rejects_ddr_overlap(void)
{
    /* Rise/Fall 또는 Frame Store DDR 영역이 겹치면 시작 전에 거부한다. */
    lidar_v2_xilinx_vdma_lane_t rise;
    lidar_v2_xilinx_vdma_lane_t fall;
    lidar_v3_xilinx_vdma_adapter_t adapter;
    XAxiVdma *fall_instance;

    clear_lane(&rise);
    clear_lane(&fall);
    fall_instance = fall.instance;
    callback_registration_count = 0U;
    callback_registration_result = XST_SUCCESS;
    assert(lidar_v3_xilinx_vdma_adapter_init(
        &adapter, &rise, &fall, 100U) == XST_INVALID_PARAM);
    assert(!adapter.buffers_validated);

    set_lane_base(&fall, 0x11000000U);
    assert(lidar_v3_xilinx_vdma_adapter_init(
        &adapter, &rise, &fall, 100U) == XST_SUCCESS);
    assert(adapter.buffers_validated);
    assert(callback_registration_count == 4U);

    fall.instance = rise.instance;
    assert(lidar_v3_xilinx_vdma_adapter_init(
        &adapter, &rise, &fall, 100U) == XST_INVALID_PARAM);
    fall.instance = fall_instance;
    fall.output_width_bits = 32U;
    assert(lidar_v3_xilinx_vdma_adapter_init(
        &adapter, &rise, &fall, 100U) == XST_INVALID_PARAM);
    fall.output_width_bits = rise.output_width_bits;
    fall.frame_addresses[0] += 4U;
    assert(lidar_v3_xilinx_vdma_adapter_init(
        &adapter, &rise, &fall, 100U) == XST_INVALID_PARAM);

    set_lane_base(&fall, 0x11000000U);
    callback_registration_result = XST_FAILURE;
    assert(lidar_v3_xilinx_vdma_adapter_init(
        &adapter, &rise, &fall, 100U) == XST_FAILURE);
    assert(!adapter.buffers_validated);
    callback_registration_result = XST_SUCCESS;
}

static void test_adapter_init_requires_per_frame_s2mm_irq(void)
{
    /* Face마다 완료 IRQ를 만들 수 없는 VDMA 합성/드라이버 설정은 거부한다. */
    lidar_v2_xilinx_vdma_lane_t rise;
    lidar_v3_xilinx_vdma_adapter_t adapter;

    clear_lane(&rise);
    frame_counter_calls = 0U;
    frame_counter_result = XST_SUCCESS;
    callback_registration_result = XST_SUCCESS;
    assert(lidar_v3_xilinx_vdma_adapter_init(
        &adapter, &rise, NULL, 100U) == XST_SUCCESS);
    assert(frame_counter_calls == 1U);
    assert(last_frame_counter.ReadFrameCount ==
        LIDAR_V3_XILINX_FRAME_IRQ_THRESHOLD);
    assert(last_frame_counter.WriteFrameCount ==
        LIDAR_V3_XILINX_FRAME_IRQ_THRESHOLD);
    assert(last_frame_counter.ReadDelayTimerCount == 0U &&
        last_frame_counter.WriteDelayTimerCount == 0U);

    rise.frame_store_count = 2U;
    rise.instance->WriteChannel.NumFrames = 2U;
    assert(lidar_v3_xilinx_vdma_adapter_init(
        &adapter, &rise, NULL, 100U) == XST_INVALID_PARAM);
    rise.frame_store_count = 3U;
    rise.instance->WriteChannel.NumFrames = 3U;

    rise.instance->HasMm2S = 1;
    assert(lidar_v3_xilinx_vdma_adapter_init(
        &adapter, &rise, NULL, 100U) == XST_INVALID_PARAM);
    rise.instance->HasMm2S = 0;
    rise.instance->HasS2Mm = 0;
    assert(lidar_v3_xilinx_vdma_adapter_init(
        &adapter, &rise, NULL, 100U) == XST_INVALID_PARAM);
    rise.instance->HasS2Mm = 1;

    rise.instance->WriteChannel.NumFrames = 4U;
    assert(lidar_v3_xilinx_vdma_adapter_init(
        &adapter, &rise, NULL, 100U) == XST_INVALID_PARAM);
    rise.instance->WriteChannel.NumFrames = 3U;

    rise.instance->WriteChannel.DbgFeatureFlags =
        XAXIVDMA_ENABLE_DBG_FRM_CNTR;
    assert(lidar_v3_xilinx_vdma_adapter_init(
        &adapter, &rise, NULL, 100U) == XST_NO_FEATURE);
    rise.instance->WriteChannel.DbgFeatureFlags =
        XAXIVDMA_ENABLE_DBG_FRM_CNTR |
        XAXIVDMA_ENABLE_DBG_DLY_CNTR;

    frame_counter_result = XST_FAILURE;
    assert(lidar_v3_xilinx_vdma_adapter_init(
        &adapter, &rise, NULL, 100U) == XST_FAILURE);
    frame_counter_result = XST_SUCCESS;
}

static void test_completed_frame_progress_service(void)
{
    /* COMMIT 대기 중 정지된 마지막 Frame까지 소비하고 소유권을 반환한다. */
    lidar_v2_xilinx_vdma_lane_t rise;
    lidar_v2_xilinx_vdma_lane_t fall;
    lidar_v3_xilinx_vdma_adapter_t adapter;
    lidar_v3_xilinx_frame_service_t service;
    lidar_v2_device_t device;

    clear_lane(&rise);
    clear_lane(&fall);
    set_lane_base(&fall, 0x11000000U);
    rise.running = true;
    fall.running = true;
    rise.active_profile.hsize_bytes = 400U;
    rise.active_profile.vsize_lines = 1801U;
    rise.active_profile.stride_bytes = 656U;
    fall.active_profile = rise.active_profile;
    rise.ready_mask = 1U;
    fall.ready_mask = 2U;
    current_frame_store = 2U;
    memset(fake_csr_words, 0, sizeof(fake_csr_words));
    fake_csr_words[(LIDAR_V2_STAT_OFFSET(
        LIDAR_V2_STAT_DERIVED_FACE)) / 4U] = 1800U << 16U;
    memset(consumer_calls, 0, sizeof(consumer_calls));
    consumer_retries_remaining = 0U;
    consumer_failures_remaining = 0U;
    consumer_release_tamper_lane = NULL;

    lidar_v2_device_init(&device, TEST_CSR_BASE, NULL,
        fake_read32, fake_write32, NULL);
    assert(lidar_v3_xilinx_vdma_adapter_init(
        &adapter, &rise, &fall, 100U) == XST_SUCCESS);
    lidar_v3_xilinx_frame_service_init(&service, &device, &adapter,
        consume_frame, NULL);

    assert(lidar_v3_xilinx_service_completed_frames(&service) ==
        XST_SUCCESS);
    assert(consumer_calls[0] == 1U && consumer_calls[1] == 1U);
    assert(service.delivered_count[0] == 1U &&
        service.delivered_count[1] == 1U);
    assert(rise.ready_mask == 0U && rise.cpu_owned_mask == 0U &&
        fall.ready_mask == 0U && fall.cpu_owned_mask == 0U);
    assert(!service.held_frame_valid);

    /* 실패한 Consumer Frame은 CPU 소유로 보존하고 명시적 재시도한다. */
    rise.ready_mask = 4U;
    current_frame_store = 0U;
    consumer_failures_remaining = 1U;
    assert(lidar_v3_xilinx_service_completed_frames(&service) ==
        XST_FAILURE);
    assert(service.held_frame_valid && service.held_frame_index == 2U &&
        service.held_lane_id == LIDAR_V3_VDMA_LANE_RISE);
    assert(rise.ready_mask == 0U && rise.cpu_owned_mask == 4U &&
        service.consumer_failure_count == 1U);
    assert(lidar_v3_xilinx_service_completed_frames(&service) ==
        XST_SUCCESS);
    assert(!service.held_frame_valid && rise.cpu_owned_mask == 0U &&
        service.delivered_count[0] == 2U);

    /* 일시 backpressure는 치명 오류가 아니며 다음 Poll에서 재시도한다. */
    rise.ready_mask = 2U;
    current_frame_store = 2U;
    consumer_retries_remaining = 1U;
    assert(lidar_v3_xilinx_service_completed_frames(&service) ==
        XST_SUCCESS);
    assert(service.held_frame_valid && rise.cpu_owned_mask == 2U &&
        service.consumer_retry_count == 1U);
    assert(lidar_v3_xilinx_service_completed_frames(&service) ==
        XST_SUCCESS);
    assert(!service.held_frame_valid && rise.cpu_owned_mask == 0U &&
        service.delivered_count[0] == 3U);

    /* Consumer 성공 뒤 반환만 실패하면 Consumer를 중복 호출하지 않는다. */
    rise.ready_mask = 1U;
    current_frame_store = 2U;
    consumer_release_tamper_lane = &rise;
    assert(lidar_v3_xilinx_service_completed_frames(&service) ==
        XST_FAILURE);
    assert(service.held_frame_valid && service.held_consumer_completed &&
        service.release_failure_count == 1U && consumer_calls[0] == 6U);
    rise.cpu_owned_mask = 1U;
    assert(lidar_v3_xilinx_service_completed_frames(&service) ==
        XST_SUCCESS);
    assert(!service.held_frame_valid && rise.cpu_owned_mask == 0U &&
        consumer_calls[0] == 6U && service.delivered_count[0] == 4U);

    /* Active 계획 Shot 수가 없으면 ready Frame을 건드리지 않는다. */
    rise.ready_mask = 1U;
    current_frame_store = 2U;
    fake_csr_words[(LIDAR_V2_STAT_OFFSET(
        LIDAR_V2_STAT_DERIVED_FACE)) / 4U] = 0U;
    assert(lidar_v3_xilinx_service_completed_frames(&service) ==
        XST_FAILURE);
    assert(rise.ready_mask == 1U && rise.cpu_owned_mask == 0U);
}

static void test_completed_frames_follow_circular_time_order(void)
{
    /* Frame 번호 wrap 뒤에도 가장 오래된 완료 Frame부터 넘겨야 한다. */
    lidar_v2_xilinx_vdma_lane_t rise;
    lidar_v3_xilinx_vdma_adapter_t adapter;
    lidar_ps_ddr_frame_t frame;
    uint8_t frame_index;

    clear_lane(&rise);
    rise.running = true;
    rise.active_profile.hsize_bytes = 400U;
    rise.active_profile.vsize_lines = 1801U;
    rise.active_profile.stride_bytes = 656U;
    rise.ready_mask = (1U << 1U) | (1U << 2U);
    current_frame_store = 0U;
    assert(lidar_v3_xilinx_vdma_adapter_init(
        &adapter, &rise, NULL, 100U) == XST_SUCCESS);

    assert(lidar_v3_xilinx_vdma_take_completed(&adapter,
        LIDAR_V3_VDMA_LANE_RISE, 1800U, &frame_index, &frame) ==
        XST_SUCCESS);
    assert(frame_index == 1U);
    assert(lidar_v3_xilinx_vdma_release_frame(&adapter,
        LIDAR_V3_VDMA_LANE_RISE, frame_index, &frame) == XST_SUCCESS);
    assert(lidar_v3_xilinx_vdma_take_completed(&adapter,
        LIDAR_V3_VDMA_LANE_RISE, 1800U, &frame_index, &frame) ==
        XST_SUCCESS);
    assert(frame_index == 2U);
    assert(lidar_v3_xilinx_vdma_release_frame(&adapter,
        LIDAR_V3_VDMA_LANE_RISE, frame_index, &frame) == XST_SUCCESS);
    assert(rise.ready_mask == 0U && rise.cpu_owned_mask == 0U);

    rise.ready_mask = 1U;
    rise.error_mask = 1U;
    assert(lidar_v3_xilinx_vdma_take_completed(&adapter,
        LIDAR_V3_VDMA_LANE_RISE, 1800U, &frame_index, &frame) ==
        XST_FAILURE);
    assert(rise.ready_mask == 1U && rise.cpu_owned_mask == 0U);

    /* 하드웨어가 현재 쓰는 Frame이 ready이면 overwrite로 격상한다. */
    rise.error_mask = 0U;
    rise.ready_mask = 1U;
    current_frame_store = 0U;
    assert(lidar_v3_xilinx_vdma_take_completed(&adapter,
        LIDAR_V3_VDMA_LANE_RISE, 1800U, &frame_index, &frame) ==
        XST_FAILURE);
    assert(rise.ready_mask == 1U && rise.cpu_owned_mask == 0U &&
        rise.overwrite_mask == 1U);
}

static void test_transaction_drains_final_frame_before_ack(void)
{
    /* 정지 직전 완료 Frame을 처리한 다음 Poll에서만 새 Geometry를 ACK한다. */
    lidar_v2_xilinx_vdma_lane_t rise;
    lidar_v3_xilinx_vdma_adapter_t adapter;
    lidar_v3_xilinx_frame_service_t frame_service;
    lidar_v3_vdma_transaction_t transaction;
    lidar_v2_device_t device;
    size_t control_word = LIDAR_V2_CTL_OFFSET(
        LIDAR_V2_CTL_VDMA_PROFILE_CONTROL) / 4U;

    clear_lane(&rise);
    rise.running = true;
    rise.active_profile.hsize_bytes = 656U;
    rise.active_profile.vsize_lines = 1801U;
    rise.active_profile.stride_bytes = 656U;
    memset(fake_csr_words, 0, sizeof(fake_csr_words));
    fake_csr_words[control_word] =
        LIDAR_V2_VDMA_RISE_PENDING | LIDAR_V2_VDMA_RISE_ENABLE;
    fake_csr_words[LIDAR_V2_CTL_OFFSET(
        LIDAR_V2_CTL_VDMA_RISE_GEOMETRY) / 4U] =
        (1801U << 16U) | 400U;
    fake_csr_words[LIDAR_V2_CTL_OFFSET(
        LIDAR_V2_CTL_VDMA_RISE_STRIDE) / 4U] = 656U;
    fake_csr_words[LIDAR_V2_STAT_OFFSET(
        LIDAR_V2_STAT_DERIVED_FACE) / 4U] = 1800U << 16U;
    acknowledge_count = 0U;
    program_calls = 0U;
    program_result = XST_SUCCESS;
    pending_interrupts = XAXIVDMA_IXR_FRMCNT_MASK;
    current_frame_store = 2U;
    memset(consumer_calls, 0, sizeof(consumer_calls));
    consumer_retries_remaining = 0U;
    consumer_failures_remaining = 0U;
    consumer_release_tamper_lane = NULL;

    lidar_v2_device_init(&device, TEST_CSR_BASE, NULL,
        fake_read32, fake_write32, NULL);
    assert(lidar_v3_xilinx_vdma_adapter_init(
        &adapter, &rise, NULL, 100U) == XST_SUCCESS);
    lidar_v3_xilinx_frame_service_init(&frame_service,
        &device, &adapter, consume_frame, NULL);
    lidar_v3_vdma_transaction_init(&transaction, &device,
        lidar_v3_xilinx_vdma_apply_lane, &adapter, 4U);
    lidar_v3_vdma_transaction_set_progress_service(&transaction,
        lidar_v3_xilinx_service_completed_frames, &frame_service);
    lidar_v3_vdma_transaction_begin(&transaction);

    assert(lidar_v3_vdma_transaction_poll(&transaction) == 0);
    assert(adapter.rise_quiesced && !rise.running &&
        rise.ready_mask == 2U);
    assert(program_calls == 0U && acknowledge_count == 0U &&
        consumer_calls[0] == 0U);

    assert(lidar_v3_vdma_transaction_poll(&transaction) == 0);
    assert(consumer_calls[0] == 1U && rise.ready_mask == 0U &&
        rise.cpu_owned_mask == 0U);
    assert(program_calls == 1U && acknowledge_count == 1U && rise.running);
    assert((fake_csr_words[control_word] &
        LIDAR_V2_VDMA_RISE_PENDING) == 0U);

    assert(lidar_v3_vdma_transaction_poll(&transaction) == 0);
    assert(!transaction.transaction_active && program_calls == 1U &&
        acknowledge_count == 1U);
}

static void test_precommit_quiesces_and_drains_before_commit(void)
{
    /* STOP/DISARM 확인 뒤 마지막 Frame을 모두 소비해야 COMMIT 준비가 끝난다. */
    lidar_v2_xilinx_vdma_lane_t rise;
    lidar_v3_xilinx_vdma_adapter_t adapter;
    lidar_v3_xilinx_frame_service_t frame_service;
    lidar_v3_vdma_transaction_t transaction;
    lidar_v2_device_t device;
    uint32_t stop_calls_before;

    clear_lane(&rise);
    rise.running = true;
    rise.active_profile.hsize_bytes = 400U;
    rise.active_profile.vsize_lines = 1801U;
    rise.active_profile.stride_bytes = 656U;
    memset(fake_csr_words, 0, sizeof(fake_csr_words));
    fake_csr_words[LIDAR_V2_STAT_OFFSET(
        LIDAR_V2_STAT_DERIVED_FACE) / 4U] = 1800U << 16U;
    fake_csr_words[LIDAR_V2_STAT_OFFSET(
        LIDAR_V2_STAT_ACTIVE_VERSION) / 4U] =
        LIDAR_V2_OP_COMMAND_READY | LIDAR_V2_OP_RUNNING;
    pending_interrupts = XAXIVDMA_IXR_FRMCNT_MASK;
    current_frame_store = 2U;
    memset(consumer_calls, 0, sizeof(consumer_calls));
    consumer_retries_remaining = 1U;
    consumer_failures_remaining = 0U;
    consumer_release_tamper_lane = NULL;
    fake_diag_data = 0U;
    fake_diag_capture_count = 0U;

    lidar_v2_device_init(&device, TEST_CSR_BASE, NULL,
        fake_read32, fake_write32, NULL);
    assert(lidar_v3_xilinx_vdma_adapter_init(
        &adapter, &rise, NULL, 100U) == XST_SUCCESS);
    lidar_v3_xilinx_frame_service_init(&frame_service,
        &device, &adapter, consume_frame, NULL);
    lidar_v3_vdma_transaction_init(&transaction, &device,
        lidar_v3_xilinx_vdma_apply_lane, &adapter, 4U);
    assert(lidar_v3_xilinx_frame_service_bind_transaction(
        &frame_service, &transaction) == XST_SUCCESS);
    assert(transaction.precommit_service != NULL &&
        transaction.progress_service != NULL);

    stop_calls_before = stop_calls;
    assert(lidar_v3_xilinx_prepare_commit_step(&frame_service) ==
        XST_DEVICE_BUSY);
    assert(!adapter.rise_quiesced && rise.running &&
        stop_calls == stop_calls_before);

    fake_csr_words[LIDAR_V2_STAT_OFFSET(
        LIDAR_V2_STAT_ACTIVE_VERSION) / 4U] =
        LIDAR_V2_OP_COMMAND_READY;
    assert(lidar_v3_xilinx_prepare_commit_step(&frame_service) ==
        XST_NO_DATA);
    assert(!adapter.rise_quiesced && rise.running &&
        stop_calls == stop_calls_before);
    assert(lidar_v3_xilinx_prepare_commit_step(&frame_service) ==
        XST_NO_DATA);
    assert(!adapter.rise_quiesced && rise.running &&
        stop_calls == stop_calls_before);

    fake_diag_data = LIDAR_V3_DIAG_ALL_PROCESSING_IDLE;
    assert(lidar_v3_xilinx_prepare_commit_step(&frame_service) ==
        XST_NO_DATA);
    assert(!adapter.rise_quiesced && rise.running &&
        stop_calls == stop_calls_before);
    assert(lidar_v3_xilinx_prepare_commit_step(&frame_service) ==
        XST_NO_DATA);
    assert(adapter.rise_quiesced && !rise.running &&
        frame_service.held_frame_valid && rise.ready_mask == 0U &&
        rise.cpu_owned_mask == 2U && consumer_calls[0] == 1U);
    assert(lidar_v3_xilinx_prepare_commit_step(&frame_service) ==
        XST_NO_DATA);
    assert(lidar_v3_xilinx_prepare_commit_step(&frame_service) ==
        XST_SUCCESS);
    assert(!frame_service.held_frame_valid &&
        lidar_v3_xilinx_vdma_buffers_idle(&adapter) &&
        rise.cpu_owned_mask == 0U && consumer_calls[0] == 2U);
    assert(lidar_v3_vdma_transaction_prepare(&transaction, 2U) == 0);
}

static void test_precommit_rejects_unusable_idle_snapshots(void)
{
    /* 오래되었거나 오류인 CTL23/24 응답으로 VDMA를 정지하면 안 된다. */
    lidar_v2_xilinx_vdma_lane_t rise;
    lidar_v3_xilinx_vdma_adapter_t adapter;
    lidar_v3_xilinx_frame_service_t frame_service;
    lidar_v2_device_t device;
    size_t diag_control_word = LIDAR_V2_CTL_OFFSET(
        LIDAR_V2_CTL_DIAG_INDEX) / 4U;
    uint32_t stop_calls_before;

    clear_lane(&rise);
    rise.running = true;
    rise.active_profile.hsize_bytes = 400U;
    rise.active_profile.vsize_lines = 1801U;
    rise.active_profile.stride_bytes = 656U;
    memset(fake_csr_words, 0, sizeof(fake_csr_words));
    fake_csr_words[LIDAR_V2_STAT_OFFSET(
        LIDAR_V2_STAT_DERIVED_FACE) / 4U] = 1800U << 16U;
    fake_csr_words[LIDAR_V2_STAT_OFFSET(
        LIDAR_V2_STAT_ACTIVE_VERSION) / 4U] =
        LIDAR_V2_OP_COMMAND_READY;
    fake_diag_data = LIDAR_V3_DIAG_ALL_PROCESSING_IDLE |
        LIDAR_V3_DIAG_GPX_CDC_RESET_BUSY;
    fake_diag_capture_count = 0U;
    pending_interrupts = 0U;
    current_frame_store = 0U;

    lidar_v2_device_init(&device, TEST_CSR_BASE, NULL,
        fake_read32, fake_write32, NULL);
    assert(lidar_v3_xilinx_vdma_adapter_init(
        &adapter, &rise, NULL, 100U) == XST_SUCCESS);
    lidar_v3_xilinx_frame_service_init(&frame_service,
        &device, &adapter, consume_frame, NULL);
    stop_calls_before = stop_calls;

    /* 다른 진단 사용자가 BUSY이면 요청을 덮어쓰지 않는다. */
    fake_csr_words[diag_control_word] = LIDAR_V3_DIAG_BUSY;
    assert(lidar_v3_xilinx_prepare_commit_step(&frame_service) ==
        XST_NO_DATA);
    assert(fake_diag_capture_count == 0U && stop_calls == stop_calls_before);

    /* RUN 재진입은 미완료 캡처를 버리고 다음 STOP에서 새로 요청한다. */
    fake_csr_words[diag_control_word] = 0U;
    assert(lidar_v3_xilinx_prepare_commit_step(&frame_service) ==
        XST_NO_DATA);
    assert(fake_diag_capture_count == 1U);
    fake_csr_words[LIDAR_V2_STAT_OFFSET(
        LIDAR_V2_STAT_ACTIVE_VERSION) / 4U] =
        LIDAR_V2_OP_COMMAND_READY | LIDAR_V2_OP_RUNNING;
    assert(lidar_v3_xilinx_prepare_commit_step(&frame_service) ==
        XST_DEVICE_BUSY);
    assert(!frame_service.idle_capture_pending);
    fake_csr_words[LIDAR_V2_STAT_OFFSET(
        LIDAR_V2_STAT_ACTIVE_VERSION) / 4U] =
        LIDAR_V2_OP_COMMAND_READY;
    assert(lidar_v3_xilinx_prepare_commit_step(&frame_service) ==
        XST_NO_DATA);
    assert(fake_diag_capture_count == 2U);

    /* 같은 sequence의 오래된 VALID 응답은 idle 증거가 아니다. */
    fake_csr_words[diag_control_word] =
        (1U << LIDAR_V3_DIAG_SEQUENCE_SHIFT) |
        LIDAR_V3_DIAG_VALID |
        LIDAR_V3_DIAG_PROCESSING_FLAGS_INDEX;
    assert(lidar_v3_xilinx_prepare_commit_step(&frame_service) ==
        XST_NO_DATA);
    assert(stop_calls == stop_calls_before);

    /* 새 snapshot이어도 GPX CDC reset 중이면 VDMA를 유지한다. */
    fake_csr_words[diag_control_word] =
        (2U << LIDAR_V3_DIAG_SEQUENCE_SHIFT) |
        LIDAR_V3_DIAG_VALID | LIDAR_V3_DIAG_PROCESSING_FLAGS_INDEX;
    assert(lidar_v3_xilinx_prepare_commit_step(&frame_service) ==
        XST_NO_DATA);
    assert(stop_calls == stop_calls_before);

    /* Mailbox ERROR는 재시도가 아닌 치명 오류이며 역시 VDMA를 보존한다. */
    assert(lidar_v3_xilinx_prepare_commit_step(&frame_service) ==
        XST_NO_DATA);
    fake_csr_words[diag_control_word] |= LIDAR_V3_DIAG_ERROR;
    assert(lidar_v3_xilinx_prepare_commit_step(&frame_service) ==
        XST_FAILURE);
    assert(stop_calls == stop_calls_before);

    /* 새 정상 idle snapshot만 VDMA 정지를 승인한다. */
    fake_diag_data = LIDAR_V3_DIAG_ALL_PROCESSING_IDLE;
    assert(lidar_v3_xilinx_prepare_commit_step(&frame_service) ==
        XST_NO_DATA);
    assert(lidar_v3_xilinx_prepare_commit_step(&frame_service) ==
        XST_SUCCESS);
    assert(adapter.rise_quiesced && !rise.running &&
        stop_calls == stop_calls_before + 1U);
}

int main(void)
{
    /* 실제 XAxiVdma 호출 전 Frame 소유권과 치명 상태 차단을 검증한다. */
    lidar_v2_xilinx_vdma_lane_t rise;
    lidar_v2_xilinx_vdma_lane_t fall;
    lidar_v2_vdma_lane_profile_t profile;
    lidar_v3_xilinx_vdma_adapter_t adapter;
    lidar_ps_ddr_frame_t frame;
    uint8_t frame_index = 0xFFU;

    clear_lane(&rise);
    clear_lane(&fall);
    set_lane_base(&fall, 0x11000000U);
    memset(&profile, 0, sizeof(profile));
    profile.enabled = true;
    profile.hsize_bytes = 400U;
    profile.vsize_lines = 1801U;
    profile.stride_bytes = 656U;
    assert(lidar_v3_xilinx_vdma_adapter_init(
        &adapter, &rise, &fall, 100U) == XST_SUCCESS);

    program_result = XST_SUCCESS;
    assert(lidar_v3_xilinx_vdma_apply_lane(&adapter,
        LIDAR_V3_VDMA_LANE_RISE, &profile) == LIDAR_V3_VDMA_APPLY_OK);
    assert(program_calls == 1U && rise.running);
    assert(interrupt_disable_calls == 1U && stop_calls == 1U &&
        interrupt_clear_calls == 1U);

    assert(lidar_v3_xilinx_vdma_apply_lane(&adapter,
        (lidar_v3_vdma_lane_id_t)2, &profile) ==
        LIDAR_V3_VDMA_APPLY_FATAL);

    rise.output_width_bits = 128U;
    assert(lidar_v3_xilinx_vdma_apply_lane(&adapter,
        LIDAR_V3_VDMA_LANE_RISE, &profile) ==
        LIDAR_V3_VDMA_APPLY_FATAL);
    assert(program_calls == 1U);
    rise.output_width_bits = 64U;

    rise.cpu_owned_mask = 1U;
    assert(lidar_v3_xilinx_vdma_apply_lane(&adapter,
        LIDAR_V3_VDMA_LANE_RISE, &profile) ==
        LIDAR_V3_VDMA_APPLY_RETRY);
    assert(program_calls == 1U);
    rise.cpu_owned_mask = 0U;
    rise.ready_mask = 2U;
    assert(lidar_v3_xilinx_vdma_apply_lane(&adapter,
        LIDAR_V3_VDMA_LANE_RISE, &profile) ==
        LIDAR_V3_VDMA_APPLY_RETRY);
    assert(program_calls == 1U);

    /* A final completion after quiesce must become a ready frame, not vanish. */
    rise.ready_mask = 0U;
    adapter.rise_quiesced = false;
    pending_interrupts = XAXIVDMA_IXR_FRMCNT_MASK;
    current_frame_store = 2U;
    assert(lidar_v3_xilinx_vdma_apply_lane(&adapter,
        LIDAR_V3_VDMA_LANE_RISE, &profile) ==
        LIDAR_V3_VDMA_APPLY_RETRY);
    assert(rise.ready_mask == 2U && program_calls == 1U);
    assert(start_calls == 0U && interrupt_enable_calls == 0U &&
        adapter.rise_quiesced && !rise.running);

    assert(lidar_v3_xilinx_vdma_take_completed(&adapter,
        LIDAR_V3_VDMA_LANE_RISE, 1800U, &frame_index, &frame) ==
        XST_SUCCESS);
    assert(frame_index == 1U && rise.ready_mask == 0U &&
        rise.cpu_owned_mask == 2U);
    assert(frame.owner == LIDAR_PS_BUFFER_CPU_OWNED &&
        frame.base == (const uint8_t *)rise.frame_addresses[1]);
    assert(cache_invalidate_address == rise.frame_addresses[1] &&
        cache_invalidate_length == 656U * 1801U);
    assert(interrupt_enable_calls == 0U);

    assert(lidar_v3_xilinx_vdma_release_frame(&adapter,
        LIDAR_V3_VDMA_LANE_RISE, 0U, &frame) == XST_INVALID_PARAM);
    assert(rise.cpu_owned_mask == 2U &&
        frame.owner == LIDAR_PS_BUFFER_CPU_OWNED);
    assert(lidar_v3_xilinx_vdma_release_frame(&adapter,
        LIDAR_V3_VDMA_LANE_RISE, frame_index, &frame) == XST_SUCCESS);
    assert(rise.cpu_owned_mask == 0U &&
        frame.owner == LIDAR_PS_BUFFER_DMA_OWNED);
    assert(interrupt_enable_calls == 0U);
    assert(lidar_v3_xilinx_vdma_release_frame(&adapter,
        LIDAR_V3_VDMA_LANE_RISE, frame_index, &frame) ==
        XST_INVALID_PARAM);

    assert(lidar_v3_xilinx_vdma_apply_lane(&adapter,
        LIDAR_V3_VDMA_LANE_RISE, &profile) == LIDAR_V3_VDMA_APPLY_OK);
    assert(program_calls == 2U && rise.running &&
        !adapter.rise_quiesced);

    pending_interrupts = XAXIVDMA_IXR_ERROR_MASK;
    adapter.rise_quiesced = false;
    assert(lidar_v3_xilinx_vdma_apply_lane(&adapter,
        LIDAR_V3_VDMA_LANE_RISE, &profile) ==
        LIDAR_V3_VDMA_APPLY_FATAL);
    assert(rise.error_mask == XAXIVDMA_IXR_ERROR_MASK);
    rise.error_mask = 0U;

    rise.ready_mask = 0U;
    rise.overwrite_mask = 4U;
    assert(lidar_v3_xilinx_vdma_apply_lane(&adapter,
        LIDAR_V3_VDMA_LANE_RISE, &profile) ==
        LIDAR_V3_VDMA_APPLY_FATAL);
    rise.ready_mask = 1U;
    assert(lidar_v3_xilinx_vdma_apply_lane(&adapter,
        LIDAR_V3_VDMA_LANE_RISE, &profile) ==
        LIDAR_V3_VDMA_APPLY_FATAL);
    rise.ready_mask = 0U;
    rise.overwrite_mask = 0U;
    rise.error_mask = 8U;
    assert(lidar_v3_xilinx_vdma_apply_lane(&adapter,
        LIDAR_V3_VDMA_LANE_RISE, &profile) ==
        LIDAR_V3_VDMA_APPLY_FATAL);

    rise.error_mask = 0U;
    program_result = XST_FAILURE;
    assert(lidar_v3_xilinx_vdma_apply_lane(&adapter,
        LIDAR_V3_VDMA_LANE_RISE, &profile) ==
        LIDAR_V3_VDMA_APPLY_FATAL);
    assert(program_calls == 3U);

    adapter.fall = NULL;
    assert(lidar_v3_xilinx_vdma_apply_lane(&adapter,
        LIDAR_V3_VDMA_LANE_FALL, &profile) ==
        LIDAR_V3_VDMA_APPLY_FATAL);
    profile.enabled = false;
    assert(lidar_v3_xilinx_vdma_apply_lane(&adapter,
        LIDAR_V3_VDMA_LANE_FALL, &profile) == LIDAR_V3_VDMA_APPLY_OK);

    test_completed_frame_progress_service();
    test_completed_frames_follow_circular_time_order();
    test_transaction_drains_final_frame_before_ack();
    test_precommit_quiesces_and_drains_before_commit();
    test_precommit_rejects_unusable_idle_snapshots();
    test_adapter_init_requires_per_frame_s2mm_irq();
    test_adapter_init_rejects_ddr_overlap();

    puts("LIDAR_V3_H6B3A_XILINX_ADAPTER_PASS");
    return 0;
}
