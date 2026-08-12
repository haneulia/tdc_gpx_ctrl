#ifndef LIDAR_V3_CONTROL_H
#define LIDAR_V3_CONTROL_H

#include "lidar_v3_vdma_transaction.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * V3 keeps the verified V2 CSR ABI 2.7. The V3 name represents the HLS data
 * pipeline generation, not an incompatible CSR register map.
 */
#define LIDAR_V3_CSR_ABI_MAJOR LIDAR_V2_CSR_ABI_MAJOR
#define LIDAR_V3_CSR_ABI_MINOR LIDAR_V2_CSR_ABI_MINOR

/*
 * The caller must first complete lidar_v2_stop_and_disarm(). The bound VDMA
 * pre-COMMIT service then proves datapath idle, drains completed Frames, and
 * stops both lanes before this function writes the CSR COMMIT command.
 */
lidar_v2_control_status_t lidar_v3_commit_shadow(
    const lidar_v2_device_t *device,
    lidar_v3_vdma_transaction_t *vdma_transaction,
    uint32_t poll_limit,
    lidar_v2_commit_result_t *result);

#ifdef __cplusplus
}
#endif

#endif
