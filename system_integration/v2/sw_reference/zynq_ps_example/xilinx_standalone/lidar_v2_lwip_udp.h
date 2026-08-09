#ifndef LIDAR_V2_LWIP_UDP_H
#define LIDAR_V2_LWIP_UDP_H

#include <stddef.h>
#include <stdint.h>

#include "lwip/ip_addr.h"
#include "lwip/udp.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    struct udp_pcb *pcb;
    ip_addr_t remote_address;
    uint16_t remote_port;
    uint32_t packet_count;
    uint32_t error_count;
} lidar_v2_lwip_udp_t;

int lidar_v2_lwip_udp_open(
    lidar_v2_lwip_udp_t *transport,
    const ip_addr_t *remote_address,
    uint16_t remote_port);
void lidar_v2_lwip_udp_close(lidar_v2_lwip_udp_t *transport);

/* lidar_ps_packet_sink_t-compatible callback. */
int lidar_v2_lwip_udp_packet_sink(
    const uint8_t *payload,
    size_t payload_bytes,
    void *context);

#ifdef __cplusplus
}
#endif

#endif
