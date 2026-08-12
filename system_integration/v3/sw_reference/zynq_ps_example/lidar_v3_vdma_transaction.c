#include "lidar_v3_vdma_transaction.h"

#include <limits.h>
#include <string.h>

static bool profile_equal(
    const lidar_v2_vdma_lane_profile_t *left,
    const lidar_v2_vdma_lane_profile_t *right)
{
    return left->pending == right->pending &&
        left->enabled == right->enabled &&
        left->hsize_bytes == right->hsize_bytes &&
        left->vsize_lines == right->vsize_lines &&
        left->stride_bytes == right->stride_bytes;
}

static const lidar_v2_vdma_lane_profile_t *select_profile(
    const lidar_v2_vdma_profiles_t *profiles,
    lidar_v3_vdma_lane_id_t lane_id)
{
    return lane_id == LIDAR_V3_VDMA_LANE_RISE ?
        &profiles->rise : &profiles->fall;
}

static lidar_v3_vdma_lane_state_t *select_state(
    lidar_v3_vdma_transaction_t *transaction,
    lidar_v3_vdma_lane_id_t lane_id)
{
    return lane_id == LIDAR_V3_VDMA_LANE_RISE ?
        &transaction->rise : &transaction->fall;
}

static int service_lane(
    lidar_v3_vdma_transaction_t *transaction,
    lidar_v3_vdma_lane_id_t lane_id,
    const lidar_v2_vdma_lane_profile_t *profile)
{
    lidar_v3_vdma_lane_state_t *state =
        select_state(transaction, lane_id);
    lidar_v2_vdma_profiles_t confirmed_profiles;
    const lidar_v2_vdma_lane_profile_t *confirmed;
    lidar_v3_vdma_apply_result_t apply_result;
    lidar_v2_control_status_t control_status;

    if (!profile->pending) {
        state->ack_submitted = false;
        state->attempt_profile_valid = false;
        state->apply_completed = false;
        state->consecutive_retry_count = 0U;
        return 0;
    }
    if (state->ack_submitted &&
        profile_equal(profile, &state->applied_profile)) {
        /* The CDC request can remain high briefly after the W1S ACK. */
        return 0;
    }
    if (state->ack_submitted) {
        /* A later profile appeared without an observed low interval. */
        state->ack_submitted = false;
        state->attempt_profile_valid = false;
        state->apply_completed = false;
        state->consecutive_retry_count = 0U;
    }
    if (!state->attempt_profile_valid ||
        !profile_equal(profile, &state->attempt_profile)) {
        state->attempt_profile = *profile;
        state->attempt_profile_valid = true;
        state->apply_completed = false;
        state->consecutive_retry_count = 0U;
    }

    if (!state->apply_completed) {
        ++state->apply_attempt_count;
        apply_result = transaction->apply_lane(
            transaction->apply_context, lane_id, profile);
        state->last_apply_result = apply_result;
        if (apply_result == LIDAR_V3_VDMA_APPLY_RETRY) {
            ++state->consecutive_retry_count;
            if (state->consecutive_retry_count >= transaction->retry_limit) {
                transaction->last_status = LIDAR_V3_VDMA_RETRY_EXHAUSTED;
                return -1;
            }
            return 0;
        }
        if (apply_result != LIDAR_V3_VDMA_APPLY_OK) {
            transaction->last_status = LIDAR_V3_VDMA_APPLY_FAILED;
            return -1;
        }
        state->consecutive_retry_count = 0U;
        state->apply_completed = true;
        ++state->apply_success_count;
    }

    /* A slow VDMA stop/program/start must not ACK a replaced snapshot. */
    control_status = lidar_v2_read_vdma_profiles(
        transaction->device, &confirmed_profiles);
    if (control_status != LIDAR_V2_CONTROL_OK) {
        transaction->last_status = LIDAR_V3_VDMA_PROFILE_CHANGED;
        ++state->consecutive_retry_count;
        if (state->consecutive_retry_count >= transaction->retry_limit) {
            return -1;
        }
        return 0;
    }
    confirmed = select_profile(&confirmed_profiles, lane_id);
    if (!confirmed->pending || !profile_equal(profile, confirmed)) {
        transaction->last_status = LIDAR_V3_VDMA_PROFILE_CHANGED;
        state->attempt_profile_valid = false;
        state->apply_completed = false;
        state->consecutive_retry_count = 0U;
        return 0;
    }

    control_status = lidar_v2_ack_vdma_profiles(
        transaction->device,
        lane_id == LIDAR_V3_VDMA_LANE_RISE,
        lane_id == LIDAR_V3_VDMA_LANE_FALL);
    if (control_status != LIDAR_V2_CONTROL_OK) {
        transaction->last_status = LIDAR_V3_VDMA_ACK_FAILED;
        return -1;
    }
    state->ack_submitted = true;
    state->applied_profile = *profile;
    ++state->ack_count;
    return 0;
}

void lidar_v3_vdma_transaction_init(
    lidar_v3_vdma_transaction_t *transaction,
    const lidar_v2_device_t *device,
    lidar_v3_vdma_apply_lane_t apply_lane,
    void *apply_context,
    uint32_t retry_limit)
{
    if (transaction == NULL) {
        return;
    }
    memset(transaction, 0, sizeof(*transaction));
    transaction->device = device;
    transaction->apply_lane = apply_lane;
    transaction->apply_context = apply_context;
    transaction->retry_limit = retry_limit;
    transaction->last_status = LIDAR_V3_VDMA_OK;
}

void lidar_v3_vdma_transaction_begin(
    lidar_v3_vdma_transaction_t *transaction)
{
    if (transaction == NULL) {
        return;
    }
    memset(&transaction->rise, 0, sizeof(transaction->rise));
    memset(&transaction->fall, 0, sizeof(transaction->fall));
    transaction->profile_read_retry_count = 0U;
    transaction->transaction_active = true;
    transaction->last_status = LIDAR_V3_VDMA_OK;
}

void lidar_v3_vdma_transaction_set_progress_service(
    lidar_v3_vdma_transaction_t *transaction,
    lidar_v3_vdma_progress_service_t progress_service,
    void *progress_context)
{
    if (transaction == NULL) {
        return;
    }
    transaction->progress_service = progress_service;
    transaction->progress_context = progress_context;
}

void lidar_v3_vdma_transaction_set_precommit_service(
    lidar_v3_vdma_transaction_t *transaction,
    lidar_v3_vdma_precommit_service_t precommit_service,
    void *precommit_context)
{
    if (transaction == NULL) {
        return;
    }
    transaction->precommit_service = precommit_service;
    transaction->precommit_context = precommit_context;
}

void lidar_v3_vdma_transaction_set_poll_wait(
    lidar_v3_vdma_transaction_t *transaction,
    lidar_v3_vdma_poll_wait_t poll_wait,
    void *poll_wait_context)
{
    if (transaction == NULL) {
        return;
    }
    transaction->poll_wait = poll_wait;
    transaction->poll_wait_context = poll_wait_context;
}

int lidar_v3_vdma_transaction_prepare(
    lidar_v3_vdma_transaction_t *transaction,
    uint32_t poll_limit)
{
    uint32_t poll;

    if (transaction == NULL || transaction->device == NULL ||
        transaction->apply_lane == NULL || poll_limit == 0U) {
        if (transaction != NULL) {
            transaction->last_status = LIDAR_V3_VDMA_ARGUMENT;
        }
        return -1;
    }
    if (transaction->precommit_service == NULL) {
        transaction->last_status = LIDAR_V3_VDMA_PRECOMMIT_FAILED;
        return -1;
    }
    for (poll = 0U; poll < poll_limit; ++poll) {
        int service_status = transaction->precommit_service(
            transaction->precommit_context);

        if (service_status == LIDAR_V3_VDMA_PRECOMMIT_READY) {
            transaction->last_status = LIDAR_V3_VDMA_OK;
            return 0;
        }
        if (service_status != LIDAR_V3_VDMA_PRECOMMIT_RETRY) {
            transaction->last_status = LIDAR_V3_VDMA_PRECOMMIT_FAILED;
            return -1;
        }
        if (transaction->poll_wait != NULL) {
            transaction->poll_wait(transaction->poll_wait_context);
        }
    }
    transaction->last_status = LIDAR_V3_VDMA_PRECOMMIT_TIMEOUT;
    return -1;
}

int lidar_v3_vdma_transaction_poll(void *context)
{
    lidar_v3_vdma_transaction_t *transaction =
        (lidar_v3_vdma_transaction_t *)context;
    lidar_v2_vdma_profiles_t profiles;

    if (transaction == NULL || transaction->device == NULL ||
        transaction->apply_lane == NULL ||
        transaction->retry_limit < LIDAR_V3_VDMA_MIN_RETRY_LIMIT) {
        if (transaction != NULL) {
            transaction->last_status = LIDAR_V3_VDMA_ARGUMENT;
        }
        return -1;
    }
    if (!transaction->transaction_active) {
        lidar_v3_vdma_transaction_begin(transaction);
    }
    transaction->last_status = LIDAR_V3_VDMA_OK;
    if (transaction->progress_service != NULL &&
        transaction->progress_service(transaction->progress_context) != 0) {
        transaction->last_status = LIDAR_V3_VDMA_PROGRESS_FAILED;
        return -1;
    }
    if (lidar_v2_read_vdma_profiles(transaction->device, &profiles) !=
        LIDAR_V2_CONTROL_OK) {
        transaction->last_status = LIDAR_V3_VDMA_PROFILE_CHANGED;
        ++transaction->profile_read_retry_count;
        return transaction->profile_read_retry_count >=
            transaction->retry_limit ? -1 : 0;
    }
    transaction->profile_read_retry_count = 0U;
    if (service_lane(transaction, LIDAR_V3_VDMA_LANE_RISE,
            &profiles.rise) != 0) {
        return -1;
    }
    if (service_lane(transaction, LIDAR_V3_VDMA_LANE_FALL,
            &profiles.fall) != 0) {
        return -1;
    }
    if (!profiles.rise.pending && !profiles.fall.pending) {
        transaction->transaction_active = false;
    }
    if (transaction->transaction_active && transaction->poll_wait != NULL) {
        transaction->poll_wait(transaction->poll_wait_context);
    }
    return 0;
}

lidar_v3_vdma_status_t lidar_v3_vdma_validate_profile(
    const lidar_v2_vdma_lane_profile_t *profile,
    uint16_t output_width_bits,
    size_t frame_capacity_bytes)
{
    size_t beat_bytes;
    size_t frame_bytes;

    if (profile == NULL || frame_capacity_bytes == 0U) {
        return LIDAR_V3_VDMA_ARGUMENT;
    }
    if (output_width_bits != 32U && output_width_bits != 64U) {
        return LIDAR_V3_VDMA_UNSUPPORTED_WIDTH;
    }
    if (!profile->enabled) {
        return LIDAR_V3_VDMA_OK;
    }
    beat_bytes = (size_t)output_width_bits / 8U;
    frame_bytes = (size_t)profile->stride_bytes * profile->vsize_lines;
    if (profile->hsize_bytes == 0U || profile->vsize_lines == 0U ||
        profile->stride_bytes < profile->hsize_bytes ||
        ((size_t)profile->hsize_bytes % beat_bytes) != 0U ||
        ((size_t)profile->stride_bytes % beat_bytes) != 0U ||
        frame_bytes > frame_capacity_bytes) {
        return LIDAR_V3_VDMA_INVALID_PROFILE;
    }
    return LIDAR_V3_VDMA_OK;
}

lidar_v3_vdma_status_t lidar_v3_vdma_validate_regions(
    const lidar_v3_dma_region_t *regions,
    size_t region_count,
    size_t alignment_bytes)
{
    size_t left;
    size_t right;

    if (regions == NULL || region_count == 0U || alignment_bytes == 0U) {
        return LIDAR_V3_VDMA_ARGUMENT;
    }
    for (left = 0U; left < region_count; ++left) {
        uintptr_t left_end;

        if (regions[left].capacity_bytes == 0U ||
            (regions[left].base_address % alignment_bytes) != 0U ||
            regions[left].base_address >
                UINTPTR_MAX - regions[left].capacity_bytes) {
            return LIDAR_V3_VDMA_INVALID_REGION;
        }
        left_end = regions[left].base_address +
            regions[left].capacity_bytes;
        for (right = left + 1U; right < region_count; ++right) {
            uintptr_t right_end;

            if (regions[right].capacity_bytes == 0U ||
                (regions[right].base_address % alignment_bytes) != 0U ||
                regions[right].base_address >
                    UINTPTR_MAX - regions[right].capacity_bytes) {
                return LIDAR_V3_VDMA_INVALID_REGION;
            }
            right_end = regions[right].base_address +
                regions[right].capacity_bytes;
            if (regions[left].base_address < right_end &&
                regions[right].base_address < left_end) {
                return LIDAR_V3_VDMA_REGION_OVERLAP;
            }
        }
    }
    return LIDAR_V3_VDMA_OK;
}

const char *lidar_v3_vdma_status_name(lidar_v3_vdma_status_t status)
{
    static const char *const names[] = {
        "OK",
        "ARGUMENT",
        "UNSUPPORTED_WIDTH",
        "INVALID_PROFILE",
        "INVALID_REGION",
        "REGION_OVERLAP",
        "PRECOMMIT_FAILED",
        "PRECOMMIT_TIMEOUT",
        "PROFILE_CHANGED",
        "RETRY_EXHAUSTED",
        "PROGRESS_FAILED",
        "APPLY_FAILED",
        "ACK_FAILED"
    };

    if ((unsigned int)status >= sizeof(names) / sizeof(names[0])) {
        return "UNKNOWN";
    }
    return names[status];
}
