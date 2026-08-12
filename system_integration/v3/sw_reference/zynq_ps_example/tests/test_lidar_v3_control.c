#include "lidar_v3_control.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

#define TEST_CSR_BASE 0x40000000U
#define TEST_WORD_COUNT 68U

typedef struct {
    uint32_t words[TEST_WORD_COUNT];
    uint32_t transaction_reads;
    uint32_t acknowledge_count;
    bool force_apply_failure;
    uint32_t commit_write_count;
} fake_csr_t;

typedef struct {
    uint32_t call_count;
    uint32_t retry_count;
    bool fatal;
} fake_precommit_t;

static size_t ctl_word(uint8_t index)
{
    return LIDAR_V2_CTL_OFFSET(index) / 4U;
}

static size_t stat_word(uint8_t index)
{
    return LIDAR_V2_STAT_OFFSET(index) / 4U;
}

static uint32_t fake_read32(void *context, uintptr_t address)
{
    fake_csr_t *csr = (fake_csr_t *)context;
    size_t word = (size_t)((address - TEST_CSR_BASE) / 4U);

    assert(word < TEST_WORD_COUNT);
    if (word == stat_word(LIDAR_V2_STAT_TRANSACTION)) {
        ++csr->transaction_reads;
        if (csr->force_apply_failure && csr->transaction_reads >= 4U) {
            csr->words[word] = LIDAR_V2_TXN_DONE | LIDAR_V2_TXN_ERROR |
                LIDAR_V2_TXN_RECOVERY_REQUIRED | (0x24U << 16U);
        }
    }
    return csr->words[word];
}

static void fake_write32(void *context, uintptr_t address, uint32_t value)
{
    fake_csr_t *csr = (fake_csr_t *)context;
    size_t word = (size_t)((address - TEST_CSR_BASE) / 4U);

    assert(word < TEST_WORD_COUNT);
    if (word == ctl_word(LIDAR_V2_CTL_COMMAND) &&
        (value & LIDAR_V2_CMD_COMMIT) != 0U) {
        ++csr->commit_write_count;
        csr->words[stat_word(LIDAR_V2_STAT_TRANSACTION)] = LIDAR_V2_TXN_BUSY;
        csr->words[ctl_word(LIDAR_V2_CTL_VDMA_PROFILE_CONTROL)] =
            LIDAR_V2_VDMA_RISE_PENDING | LIDAR_V2_VDMA_RISE_ENABLE;
        csr->words[ctl_word(LIDAR_V2_CTL_VDMA_RISE_GEOMETRY)] =
            (1801U << 16U) | 656U;
        csr->words[ctl_word(LIDAR_V2_CTL_VDMA_RISE_STRIDE)] = 656U;
        return;
    }
    if (word == ctl_word(LIDAR_V2_CTL_VDMA_PROFILE_CONTROL) &&
        (value & LIDAR_V2_VDMA_RISE_ACK) != 0U) {
        ++csr->acknowledge_count;
        csr->words[word] &= ~LIDAR_V2_VDMA_RISE_PENDING;
        csr->words[stat_word(LIDAR_V2_STAT_TRANSACTION)] =
            LIDAR_V2_TXN_DONE | LIDAR_V2_TXN_SUCCESS |
            LIDAR_V2_TXN_ACTIVE_VALID;
        csr->words[stat_word(LIDAR_V2_STAT_ACTIVE_VERSION)] = 3U;
        return;
    }
    csr->words[word] = value;
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

static lidar_v3_vdma_apply_result_t fake_apply(
    void *context,
    lidar_v3_vdma_lane_id_t lane_id,
    const lidar_v2_vdma_lane_profile_t *profile)
{
    fake_csr_t *csr = (fake_csr_t *)context;

    assert(lane_id == LIDAR_V3_VDMA_LANE_RISE);
    assert(profile != NULL && profile->enabled);
    return csr->force_apply_failure ? LIDAR_V3_VDMA_APPLY_FATAL :
        LIDAR_V3_VDMA_APPLY_OK;
}

static void run_case(bool force_failure)
{
    /* VDMA 실패 뒤에도 RTL 설정 트랜잭션의 최종 오류를 회수해야 한다. */
    fake_csr_t csr;
    lidar_v2_device_t device;
    lidar_v3_vdma_transaction_t transaction;
    fake_precommit_t precommit;
    lidar_v2_commit_result_t result;
    lidar_v2_control_status_t status;

    memset(&csr, 0, sizeof(csr));
    csr.force_apply_failure = force_failure;
    lidar_v2_device_init(&device, TEST_CSR_BASE, &csr,
        fake_read32, fake_write32, NULL);
    lidar_v3_vdma_transaction_init(&transaction, &device,
        fake_apply, &csr, 2U);
    memset(&precommit, 0, sizeof(precommit));
    precommit.retry_count = 2U;
    lidar_v3_vdma_transaction_set_precommit_service(
        &transaction, fake_precommit, &precommit);

    status = lidar_v3_commit_shadow(&device, &transaction, 16U, &result);
    assert(precommit.call_count == 3U && csr.commit_write_count == 1U);
    if (!force_failure) {
        assert(status == LIDAR_V2_CONTROL_OK);
        assert(csr.acknowledge_count == 1U);
        assert(result.active_version == 3U);
        assert((result.transaction_word & LIDAR_V2_TXN_SUCCESS) != 0U);
    } else {
        assert(status == LIDAR_V2_CONTROL_RECOVERY_REQUIRED);
        assert(csr.acknowledge_count == 0U);
        assert(csr.transaction_reads >= 4U);
        assert(result.error_code == 0x24U);
    }
}

static void test_precommit_failure_never_writes_commit(void)
{
    /* VDMA 사전 정리 실패 시 RTL COMMIT을 시작하지 않아야 한다. */
    fake_csr_t csr;
    fake_precommit_t precommit;
    lidar_v2_device_t device;
    lidar_v3_vdma_transaction_t transaction;
    lidar_v2_commit_result_t result;

    memset(&csr, 0, sizeof(csr));
    memset(&precommit, 0, sizeof(precommit));
    precommit.fatal = true;
    lidar_v2_device_init(&device, TEST_CSR_BASE, &csr,
        fake_read32, fake_write32, NULL);
    lidar_v3_vdma_transaction_init(&transaction, &device,
        fake_apply, &csr, 2U);
    lidar_v3_vdma_transaction_set_precommit_service(
        &transaction, fake_precommit, &precommit);
    assert(lidar_v3_commit_shadow(&device, &transaction, 4U, &result) ==
        LIDAR_V2_CONTROL_VDMA_SERVICE_ERROR);
    assert(csr.commit_write_count == 0U && csr.transaction_reads == 0U &&
        transaction.last_status == LIDAR_V3_VDMA_PRECOMMIT_FAILED);

    lidar_v3_vdma_transaction_set_precommit_service(
        &transaction, NULL, NULL);
    assert(lidar_v3_commit_shadow(&device, &transaction, 4U, &result) ==
        LIDAR_V2_CONTROL_VDMA_SERVICE_ERROR);
    assert(csr.commit_write_count == 0U &&
        transaction.last_status == LIDAR_V3_VDMA_PRECOMMIT_FAILED);
}

int main(void)
{
    run_case(false);
    run_case(true);
    test_precommit_failure_never_writes_commit();
    puts("LIDAR_V3_H6B3A_CONTROL_RECOVERY_PASS");
    return 0;
}
