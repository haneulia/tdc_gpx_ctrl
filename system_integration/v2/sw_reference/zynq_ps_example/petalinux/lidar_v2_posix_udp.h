#ifndef LIDAR_V2_POSIX_UDP_H
#define LIDAR_V2_POSIX_UDP_H

#include <stddef.h>
#include <stdint.h>

#include <netinet/in.h>

typedef struct {
    int socket_fd;
    struct sockaddr_in remote;
    uint32_t packet_count;
    uint32_t error_count;
} lidar_v2_posix_udp_t;

int lidar_v2_posix_udp_open(
    lidar_v2_posix_udp_t *transport,
    const char *remote_ipv4,
    uint16_t remote_port);
void lidar_v2_posix_udp_close(lidar_v2_posix_udp_t *transport);
int lidar_v2_posix_udp_packet_sink(
    const uint8_t *payload,
    size_t payload_bytes,
    void *context);

#endif
