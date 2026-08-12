#include "lidar_v3_vdma_transaction.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

#define TEST_CSR_BASE 0x40000000U
#define TEST_WORD_COUNT 68U

typedef struct {
    uint32_t words[TEST_WORD_COUNT];
    uint32_t barrier_count;
    uint32_t ack_history[16];
    size_t ack_count;
    bool hold_ack_pending;
    bool replace_rise_after_apply;
    uint32_t unstable_control_reads;
} fake_csr_t;

typedef struct {
    fake_csr_t *csr;
    uint32_t calls[2];
    uint32_t successes[2];
    uint32_t retry_before_success[2];
    bool fatal[2];
} fake_apply_t;

typedef struct {
    uint32_t call_count;
    uint32_t failure_on_call;
} fake_progress_t;

typedef struct {
    uint32_t call_count;
    uint32_t retry_count;
    bool fatal;
} fake_precommit_t;

static size_t ctl_word(uint8_t index)
{
    return LIDAR_V2_CTL_OFFSET(index) / 4U;
}

static uint32_t fake_read32(void *context, uintptr_t address)
{
    fake_csr_t *csr = (fake_csr_t *)context;
    size_t word = (size_t)((address - TEST_CSR_BASE) / 4U);
    uint32_t value;

    assert(word < TEST_WORD_COUNT);
    value = csr->words[word];
    if (word == ctl_word(LIDAR_V2_CTL_VDMA_PROFILE_CONTROL) &&
        csr->unstable_control_reads != 0U) {
        if ((csr->unstable_control_reads & 1U) != 0U) {
            value ^= LIDAR_V2_VDMA_RISE_ENABLE;
        }
        --csr->unstable_control_reads;
    }
    return value;
}

static void fake_write32(void *context, uintptr_t address, uint32_t value)
{
    fake_csr_t *csr = (fake_csr_t *)context;
    size_t word = (size_t)((address - TEST_CSR_BASE) / 4U);

    assert(word < TEST_WORD_COUNT);
    if (word == ctl_word(LIDAR_V2_CTL_VDMA_PROFILE_CONTROL)) {
        assert(csr->ack_count < sizeof(csr->ack_history) /
            sizeof(csr->ack_history[0]));
        csr->ack_history[csr->ack_count++] = value;
        if (!csr->hold_ack_pending) {
            if ((value & LIDAR_V2_VDMA_RISE_ACK) != 0U) {
                csr->words[word] &= ~LIDAR_V2_VDMA_RISE_PENDING;
            }
            if ((value & LIDAR_V2_VDMA_FALL_ACK) != 0U) {
                csr->words[word] &= ~LIDAR_V2_VDMA_FALL_PENDING;
            }
        }
        return;
    }
    csr->words[word] = value;
}

static void fake_barrier(void *context)
{
    fake_csr_t *csr = (fake_csr_t *)context;
    ++csr->barrier_count;
}

static void set_profiles(
    fake_csr_t *csr,
    uint16_t rise_hsize,
    uint16_t fall_hsize)
{
    csr->words[ctl_word(LIDAR_V2_CTL_VDMA_PROFILE_CONTROL)] =
        LIDAR_V2_VDMA_RISE_PENDING | LIDAR_V2_VDMA_RISE_ENABLE |
        LIDAR_V2_VDMA_FALL_PENDING | LIDAR_V2_VDMA_FALL_ENABLE;
    csr->words[ctl_word(LIDAR_V2_CTL_VDMA_RISE_GEOMETRY)] =
        (1801U << 16U) | rise_hsize;
    csr->words[ctl_word(LIDAR_V2_CTL_VDMA_RISE_STRIDE)] = 656U;
    csr->words[ctl_word(LIDAR_V2_CTL_VDMA_FALL_GEOMETRY)] =
        (1801U << 16U) | fall_hsize;
    csr->words[ctl_word(LIDAR_V2_CTL_VDMA_FALL_STRIDE)] = 656U;
}

static lidar_v3_vdma_apply_result_t fake_apply_lane(
    void *context,
    lidar_v3_vdma_lane_id_t lane_id,
    const lidar_v2_vdma_lane_profile_t *profile)
{
    fake_apply_t *apply = (fake_apply_t *)context;
    size_t index = (size_t)lane_id;

    assert(profile != NULL && index < 2U);
    ++apply->calls[index];
    if (apply->fatal[index]) {
        return LIDAR_V3_VDMA_APPLY_FATAL;
    }
    if (apply->calls[index] <= apply->retry_before_success[index]) {
        return LIDAR_V3_VDMA_APPLY_RETRY;
    }
    ++apply->successes[index];
    if (lane_id == LIDAR_V3_VDMA_LANE_RISE &&
        apply->csr->replace_rise_after_apply) {
        apply->csr->replace_rise_after_apply = false;
        apply->csr->words[ctl_word(LIDAR_V2_CTL_VDMA_RISE_GEOMETRY)] =
            (1801U << 16U) | 400U;
    }
    return LIDAR_V3_VDMA_APPLY_OK;
}

static lidar_v3_vdma_apply_result_t fake_apply_with_read_instability(
    void *context,
    lidar_v3_vdma_lane_id_t lane_id,
    const lidar_v2_vdma_lane_profile_t *profile)
{
    fake_apply_t *apply = (fake_apply_t *)context;
    lidar_v3_vdma_apply_result_t result =
        fake_apply_lane(context, lane_id, profile);

    if (lane_id == LIDAR_V3_VDMA_LANE_RISE &&
        apply->calls[LIDAR_V3_VDMA_LANE_RISE] == 1U) {
        /* Four coherent-read attempts, two CTL25 reads per attempt. */
        apply->csr->unstable_control_reads = 8U;
    }
    return result;
}

static int fake_progress(void *context)
{
    fake_progress_t *progress = (fake_progress_t *)context;

    ++progress->call_count;
    return progress->failure_on_call == progress->call_count ? -1 : 0;
}

static int fake_precommit(void *context)
{
    fake_precommit_t *precommit = (fake_precommit_t *)context;

    ++precommit->call_count;
    if (precommit->fatal) {
        return LIDAR_V3_VDMA_PRECOMMIT_FATAL;
    }
    if (precommit->call_count <= precommit->retry_count) {
        return LIDAR_V3_VDMA_PRECOMMIT_RETRY;
    }
    return LIDAR_V3_VDMA_PRECOMMIT_READY;
}

static void fake_poll_wait(void *context)
{
    uint32_t *count = (uint32_t *)context;

    assert(count != NULL);
    ++*count;
}

static void initialize_fixture(
    fake_csr_t *csr,
    fake_apply_t *apply,
    lidar_v2_device_t *device,
    lidar_v3_vdma_transaction_t *transaction,
    uint32_t retry_limit)
{
    memset(csr, 0, sizeof(*csr));
    memset(apply, 0, sizeof(*apply));
    apply->csr = csr;
    lidar_v2_device_init(device, TEST_CSR_BASE, csr,
        fake_read32, fake_write32, fake_barrier);
    lidar_v3_vdma_transaction_init(transaction, device,
        fake_apply_lane, apply, retry_limit);
    lidar_v3_vdma_transaction_begin(transaction);
}

static void test_lane_ack_is_independent(void)
{
    /* 한 Lane의 일시 실패가 이미 적용된 반대 Lane을 재설정하지 않는지 검증한다. */
    fake_csr_t csr;
    fake_apply_t apply;
    lidar_v2_device_t device;
    lidar_v3_vdma_transaction_t transaction;

    initialize_fixture(&csr, &apply, &device, &transaction, 4U);
    set_profiles(&csr, 656U, 656U);
    apply.retry_before_success[LIDAR_V3_VDMA_LANE_FALL] = 1U;

    assert(lidar_v3_vdma_transaction_poll(&transaction) == 0);
    assert(csr.ack_count == 1U);
    assert(csr.ack_history[0] == LIDAR_V2_VDMA_RISE_ACK);
    assert(apply.calls[LIDAR_V3_VDMA_LANE_RISE] == 1U);
    assert(apply.calls[LIDAR_V3_VDMA_LANE_FALL] == 1U);

    assert(lidar_v3_vdma_transaction_poll(&transaction) == 0);
    assert(csr.ack_count == 2U);
    assert(csr.ack_history[1] == LIDAR_V2_VDMA_FALL_ACK);
    assert(apply.calls[LIDAR_V3_VDMA_LANE_RISE] == 1U);
    assert(apply.calls[LIDAR_V3_VDMA_LANE_FALL] == 2U);
}

static void test_ack_release_latency_does_not_reprogram(void)
{
    /* ACK 뒤 CDC Pending 해제 지연 동안 같은 VDMA 설정을 반복하지 않아야 한다. */
    fake_csr_t csr;
    fake_apply_t apply;
    lidar_v2_device_t device;
    lidar_v3_vdma_transaction_t transaction;

    initialize_fixture(&csr, &apply, &device, &transaction, 4U);
    set_profiles(&csr, 656U, 656U);
    csr.hold_ack_pending = true;

    assert(lidar_v3_vdma_transaction_poll(&transaction) == 0);
    assert(csr.ack_count == 2U);
    assert(lidar_v3_vdma_transaction_poll(&transaction) == 0);
    assert(csr.ack_count == 2U);
    assert(apply.calls[LIDAR_V3_VDMA_LANE_RISE] == 1U);
    assert(apply.calls[LIDAR_V3_VDMA_LANE_FALL] == 1U);

    csr.hold_ack_pending = false;
    csr.words[ctl_word(LIDAR_V2_CTL_VDMA_PROFILE_CONTROL)] = 0U;
    assert(lidar_v3_vdma_transaction_poll(&transaction) == 0);
    assert(!transaction.transaction_active);
}

static void test_changed_snapshot_is_not_acknowledged(void)
{
    /* VDMA 설정 중 바뀐 CTL25~29 스냅샷을 이전 값으로 ACK하지 않아야 한다. */
    fake_csr_t csr;
    fake_apply_t apply;
    lidar_v2_device_t device;
    lidar_v3_vdma_transaction_t transaction;

    initialize_fixture(&csr, &apply, &device, &transaction, 4U);
    set_profiles(&csr, 656U, 656U);
    apply.csr->replace_rise_after_apply = true;

    assert(lidar_v3_vdma_transaction_poll(&transaction) == 0);
    assert(csr.ack_count == 1U);
    assert(csr.ack_history[0] == LIDAR_V2_VDMA_FALL_ACK);
    assert(transaction.last_status == LIDAR_V3_VDMA_PROFILE_CHANGED);

    assert(lidar_v3_vdma_transaction_poll(&transaction) == 0);
    assert(csr.ack_count == 2U);
    assert(csr.ack_history[1] == LIDAR_V2_VDMA_RISE_ACK);
    assert(apply.calls[LIDAR_V3_VDMA_LANE_RISE] == 2U);
}

static void test_confirmation_read_retry_does_not_reprogram(void)
{
    /* 적용 뒤 MMIO 재확인만 흔들리면 VDMA가 아니라 읽기만 재시도한다. */
    fake_csr_t csr;
    fake_apply_t apply;
    lidar_v2_device_t device;
    lidar_v3_vdma_transaction_t transaction;

    initialize_fixture(&csr, &apply, &device, &transaction, 4U);
    transaction.apply_lane = fake_apply_with_read_instability;
    set_profiles(&csr, 656U, 656U);

    assert(lidar_v3_vdma_transaction_poll(&transaction) == 0);
    assert(transaction.last_status == LIDAR_V3_VDMA_PROFILE_CHANGED);
    assert(apply.calls[LIDAR_V3_VDMA_LANE_RISE] == 1U);
    assert(csr.ack_count == 1U);
    assert(csr.ack_history[0] == LIDAR_V2_VDMA_FALL_ACK);

    assert(lidar_v3_vdma_transaction_poll(&transaction) == 0);
    assert(apply.calls[LIDAR_V3_VDMA_LANE_RISE] == 1U);
    assert(csr.ack_count == 2U);
    assert(csr.ack_history[1] == LIDAR_V2_VDMA_RISE_ACK);
}

static void test_retry_limit_and_fatal_failure(void)
{
    /* 제한된 보류와 즉시 중단해야 하는 치명 실패를 구분하는지 검증한다. */
    fake_csr_t csr;
    fake_apply_t apply;
    lidar_v2_device_t device;
    lidar_v3_vdma_transaction_t transaction;

    initialize_fixture(&csr, &apply, &device, &transaction, 2U);
    set_profiles(&csr, 656U, 656U);
    apply.retry_before_success[LIDAR_V3_VDMA_LANE_RISE] = 3U;
    assert(lidar_v3_vdma_transaction_poll(&transaction) == 0);
    assert(lidar_v3_vdma_transaction_poll(&transaction) == -1);
    assert(transaction.last_status == LIDAR_V3_VDMA_RETRY_EXHAUSTED);
    assert(csr.ack_count == 1U);
    assert(csr.ack_history[0] == LIDAR_V2_VDMA_FALL_ACK);

    initialize_fixture(&csr, &apply, &device, &transaction, 2U);
    set_profiles(&csr, 656U, 656U);
    apply.fatal[LIDAR_V3_VDMA_LANE_RISE] = true;
    assert(lidar_v3_vdma_transaction_poll(&transaction) == -1);
    assert(transaction.last_status == LIDAR_V3_VDMA_APPLY_FAILED);
    assert(csr.ack_count == 0U);
}

static void test_progress_service_failure_is_not_hidden(void)
{
    /* COMMIT Poll과 함께 수행한 완료 Frame 처리 실패를 즉시 전파해야 한다. */
    fake_csr_t csr;
    fake_apply_t apply;
    fake_progress_t progress;
    lidar_v2_device_t device;
    lidar_v3_vdma_transaction_t transaction;

    initialize_fixture(&csr, &apply, &device, &transaction, 4U);
    memset(&progress, 0, sizeof(progress));
    progress.failure_on_call = 1U;
    lidar_v3_vdma_transaction_set_progress_service(
        &transaction, fake_progress, &progress);
    set_profiles(&csr, 656U, 656U);
    assert(lidar_v3_vdma_transaction_poll(&transaction) == -1);
    assert(transaction.last_status == LIDAR_V3_VDMA_PROGRESS_FAILED);
    assert(apply.calls[LIDAR_V3_VDMA_LANE_RISE] == 0U);
    assert(csr.ack_count == 0U);
}

static void test_precommit_drain_is_bounded(void)
{
    /* COMMIT 이전 Frame drain의 성공, 치명 실패와 timeout을 분리한다. */
    fake_csr_t csr;
    fake_apply_t apply;
    fake_precommit_t precommit;
    lidar_v2_device_t device;
    lidar_v3_vdma_transaction_t transaction;
    uint32_t poll_wait_count = 0U;

    initialize_fixture(&csr, &apply, &device, &transaction, 4U);
    memset(&precommit, 0, sizeof(precommit));
    precommit.retry_count = 2U;
    lidar_v3_vdma_transaction_set_precommit_service(
        &transaction, fake_precommit, &precommit);
    lidar_v3_vdma_transaction_set_poll_wait(
        &transaction, fake_poll_wait, &poll_wait_count);
    assert(lidar_v3_vdma_transaction_prepare(&transaction, 3U) == 0);
    assert(precommit.call_count == 3U &&
        transaction.last_status == LIDAR_V3_VDMA_OK &&
        poll_wait_count == 2U);

    memset(&precommit, 0, sizeof(precommit));
    precommit.retry_count = 3U;
    assert(lidar_v3_vdma_transaction_prepare(&transaction, 2U) == -1);
    assert(precommit.call_count == 2U &&
        transaction.last_status == LIDAR_V3_VDMA_PRECOMMIT_TIMEOUT &&
        poll_wait_count == 4U);

    memset(&precommit, 0, sizeof(precommit));
    precommit.fatal = true;
    assert(lidar_v3_vdma_transaction_prepare(&transaction, 2U) == -1);
    assert(precommit.call_count == 1U &&
        transaction.last_status == LIDAR_V3_VDMA_PRECOMMIT_FAILED);
}

static void test_geometry_and_ddr_regions(void)
{
    /* 32/64-bit Geometry, Frame 용량, 정렬과 DDR 영역 비중첩 계약을 검증한다. */
    lidar_v2_vdma_lane_profile_t profile;
    lidar_v3_dma_region_t regions[3];

    memset(&profile, 0, sizeof(profile));
    profile.enabled = true;
    profile.hsize_bytes = 400U;
    profile.vsize_lines = 1801U;
    profile.stride_bytes = 656U;
    assert(lidar_v3_vdma_validate_profile(&profile, 32U, 1181456U) ==
        LIDAR_V3_VDMA_OK);
    assert(lidar_v3_vdma_validate_profile(&profile, 64U, 1181456U) ==
        LIDAR_V3_VDMA_OK);
    assert(lidar_v3_vdma_validate_profile(&profile, 128U, 1181456U) ==
        LIDAR_V3_VDMA_UNSUPPORTED_WIDTH);
    profile.hsize_bytes = 402U;
    assert(lidar_v3_vdma_validate_profile(&profile, 64U, 1181456U) ==
        LIDAR_V3_VDMA_INVALID_PROFILE);
    profile.hsize_bytes = 400U;
    assert(lidar_v3_vdma_validate_profile(&profile, 64U, 1000U) ==
        LIDAR_V3_VDMA_INVALID_PROFILE);

    regions[0].base_address = 0x10000000U;
    regions[0].capacity_bytes = 0x00100000U;
    regions[1].base_address = 0x10100000U;
    regions[1].capacity_bytes = 0x00100000U;
    regions[2].base_address = 0x11000000U;
    regions[2].capacity_bytes = 0x00100000U;
    assert(lidar_v3_vdma_validate_regions(regions, 3U, 64U) ==
        LIDAR_V3_VDMA_OK);
    regions[1].base_address = 0x10080000U;
    assert(lidar_v3_vdma_validate_regions(regions, 3U, 64U) ==
        LIDAR_V3_VDMA_REGION_OVERLAP);
    regions[1].base_address = 0x10100004U;
    assert(lidar_v3_vdma_validate_regions(regions, 3U, 64U) ==
        LIDAR_V3_VDMA_INVALID_REGION);
}

int main(void)
{
    test_lane_ack_is_independent();
    test_ack_release_latency_does_not_reprogram();
    test_changed_snapshot_is_not_acknowledged();
    test_confirmation_read_retry_does_not_reprogram();
    test_retry_limit_and_fatal_failure();
    test_progress_service_failure_is_not_hidden();
    test_precommit_drain_is_bounded();
    test_geometry_and_ddr_regions();
    puts("LIDAR_V3_H6B3A_VDMA_TRANSACTION_PASS");
    return 0;
}
