#ifndef LIDAR_V3_XILINX_VDMA_ADAPTER_H
#define LIDAR_V3_XILINX_VDMA_ADAPTER_H

#include "lidar_v2_xilinx_vdma.h"
#include "../lidar_v3_vdma_transaction.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LIDAR_V3_XILINX_DMA_ALIGNMENT_BYTES 64U
#define LIDAR_V3_XILINX_MIN_FRAME_STORES 3U
#define LIDAR_V3_XILINX_FRAME_IRQ_THRESHOLD 1U

typedef struct {
    lidar_v2_xilinx_vdma_lane_t *rise;
    lidar_v2_xilinx_vdma_lane_t *fall;
    uint32_t stop_poll_limit;
    bool buffers_validated;
    bool rise_quiesced;
    bool fall_quiesced;
} lidar_v3_xilinx_vdma_adapter_t;

int lidar_v3_xilinx_vdma_adapter_init(
    lidar_v3_xilinx_vdma_adapter_t *adapter,
    lidar_v2_xilinx_vdma_lane_t *rise,
    lidar_v2_xilinx_vdma_lane_t *fall,
    uint32_t stop_poll_limit);

/*
 * adapter_init() must run after XAxiVdma_CfgInitialize() and before the first
 * S2MM start. Each lane must be an S2MM-only VDMA with at least three Frame
 * Stores. The adapter programs a one-Face-Frame completion interrupt threshold
 * and fails closed when that AXI VDMA counter feature is absent.
 */

/*
 * V3 owns the Write-channel callbacks after adapter_init(). The callbacks use
 * atomic 32-bit mask updates because the VDMA ISR and the PS processing task
 * can update Frame ownership at the same time.
 */
void lidar_v3_xilinx_vdma_completion_callback(
    void *callback_reference,
    uint32_t interrupt_types);
void lidar_v3_xilinx_vdma_error_callback(
    void *callback_reference,
    uint32_t error_mask);

/*
 * Call only after the LiDAR operation is STOPPED and DISARMED and diagnostic
 * index 0x10 proves every processing path idle. Both VDMA lanes are stopped
 * and a completion that raced with the stop is recovered into ready_mask.
 * No completed Frame is discarded by this function.
 */
int lidar_v3_xilinx_vdma_quiesce_for_commit(
    lidar_v3_xilinx_vdma_adapter_t *adapter);

/* True only when no completed or CPU-owned Frame remains on either lane. */
bool lidar_v3_xilinx_vdma_buffers_idle(
    const lidar_v3_xilinx_vdma_adapter_t *adapter);

lidar_v3_vdma_apply_result_t lidar_v3_xilinx_vdma_apply_lane(
    void *context,
    lidar_v3_vdma_lane_id_t lane_id,
    const lidar_v2_vdma_lane_profile_t *profile);

int lidar_v3_xilinx_vdma_take_completed(
    lidar_v3_xilinx_vdma_adapter_t *adapter,
    lidar_v3_vdma_lane_id_t lane_id,
    uint16_t planned_shots,
    uint8_t *frame_index,
    lidar_ps_ddr_frame_t *frame);

int lidar_v3_xilinx_vdma_release_frame(
    lidar_v3_xilinx_vdma_adapter_t *adapter,
    lidar_v3_vdma_lane_id_t lane_id,
    uint8_t frame_index,
    lidar_ps_ddr_frame_t *frame);

#ifdef __cplusplus
}
#endif

#endif
