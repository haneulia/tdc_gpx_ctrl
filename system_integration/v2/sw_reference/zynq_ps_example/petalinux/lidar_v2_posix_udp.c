#include "lidar_v2_posix_udp.h"

#include <arpa/inet.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

int lidar_v2_posix_udp_open(
    lidar_v2_posix_udp_t *transport,
    const char *remote_ipv4,
    uint16_t remote_port)
{
    if (transport == NULL || remote_ipv4 == NULL || remote_port == 0U) {
        return -1;
    }
    memset(transport, 0, sizeof(*transport));
    transport->socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (transport->socket_fd < 0) {
        return -1;
    }
    transport->remote.sin_family = AF_INET;
    transport->remote.sin_port = htons(remote_port);
    if (inet_pton(AF_INET, remote_ipv4,
            &transport->remote.sin_addr) != 1) {
        close(transport->socket_fd);
        transport->socket_fd = -1;
        return -1;
    }
    return 0;
}

void lidar_v2_posix_udp_close(lidar_v2_posix_udp_t *transport)
{
    if (transport != NULL && transport->socket_fd >= 0) {
        close(transport->socket_fd);
        transport->socket_fd = -1;
    }
}

int lidar_v2_posix_udp_packet_sink(
    const uint8_t *payload,
    size_t payload_bytes,
    void *context)
{
    lidar_v2_posix_udp_t *transport = (lidar_v2_posix_udp_t *)context;
    ssize_t sent;

    if (transport == NULL || transport->socket_fd < 0 || payload == NULL ||
        payload_bytes == 0U || payload_bytes > 1440U) {
        return -1;
    }
    sent = sendto(transport->socket_fd, payload, payload_bytes, 0,
        (const struct sockaddr *)&transport->remote,
        sizeof(transport->remote));
    if (sent < 0 || (size_t)sent != payload_bytes) {
        ++transport->error_count;
        return -1;
    }
    ++transport->packet_count;
    return 0;
}
