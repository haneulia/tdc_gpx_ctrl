#ifndef LIDAR_V2_XILINX_VDMA_H
#define LIDAR_V2_XILINX_VDMA_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "xaxivdma.h"

#include "../lidar_v2_ps_control.h"
#include "../../lidar_packed17_ps_decoder.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    XAxiVdma *instance;
    UINTPTR frame_addresses[XAXIVDMA_MAX_FRAMESTORE];
    size_t frame_capacity_bytes;
    uint16_t output_width_bits;
    uint8_t frame_store_count;
    bool running;
    lidar_v2_vdma_lane_profile_t active_profile;
    volatile uint32_t ready_mask;
    volatile uint32_t cpu_owned_mask;
    volatile uint32_t overwrite_mask;
    volatile uint32_t error_mask;
} lidar_v2_xilinx_vdma_lane_t;

typedef struct {
    const lidar_v2_device_t *device;
    lidar_v2_xilinx_vdma_lane_t *rise;
    lidar_v2_xilinx_vdma_lane_t *fall;
} lidar_v2_xilinx_vdma_service_t;

int lidar_v2_xilinx_vdma_lane_init(
    lidar_v2_xilinx_vdma_lane_t *lane,
    XAxiVdma *instance,
    const UINTPTR *frame_addresses,
    uint8_t frame_store_count,
    size_t frame_capacity_bytes,
    uint16_t output_width_bits);

int lidar_v2_xilinx_vdma_program(
    lidar_v2_xilinx_vdma_lane_t *lane,
    const lidar_v2_vdma_lane_profile_t *profile,
    uint32_t stop_poll_limit);

/* Pass this as lidar_v2_commit_shadow()'s poll service. */
int lidar_v2_xilinx_service_vdma_profiles(void *context);

/* Connect XAxiVdma_WriteIntrHandler() to the GIC; these are driver callbacks. */
void lidar_v2_xilinx_vdma_completion_callback(
    void *callback_reference,
    uint32_t interrupt_types);
void lidar_v2_xilinx_vdma_error_callback(
    void *callback_reference,
    uint32_t error_mask);

int lidar_v2_xilinx_vdma_take_completed(
    lidar_v2_xilinx_vdma_lane_t *lane,
    uint16_t planned_shots,
    uint8_t *frame_index,
    lidar_ps_ddr_frame_t *frame);
void lidar_v2_xilinx_vdma_release_frame(
    lidar_v2_xilinx_vdma_lane_t *lane,
    uint8_t frame_index,
    lidar_ps_ddr_frame_t *frame);

#ifdef __cplusplus
}
#endif

#endif
