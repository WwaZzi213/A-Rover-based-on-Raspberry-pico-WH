#pragma once

// Pico SDK lwIP configuration for Pico W
// Minimal + stable options for TCP/UDP sockets and AP mode.

#define LWIP_TIMEVAL_PRIVATE 0

#ifndef NO_SYS
#define NO_SYS 0
#endif

// Memory / buffers
#define MEM_ALIGNMENT 4
#define MEM_SIZE (16 * 1024)
#define MEMP_NUM_PBUF 16
#define PBUF_POOL_SIZE 16
#define PBUF_POOL_BUFSIZE 1536

// TCP
#define LWIP_TCP 1
#define TCP_MSS 1460
#define TCP_SND_BUF (4 * TCP_MSS)
#define TCP_WND (4 * TCP_MSS)
#define TCP_SND_QUEUELEN 8

// UDP
#define LWIP_UDP 1

// Sockets API
#define LWIP_SOCKET 1
#define LWIP_NETCONN 1
#define LWIP_COMPAT_SOCKETS 1

// DHCP (client) + AutoIP can be on; AP-side DHCP server is your own code
#define LWIP_DHCP 1
#define LWIP_AUTOIP 0

// DNS (optional)
#define LWIP_DNS 1

// ICMP (ping)
#define LWIP_ICMP 1

// Debug (keep off)
#define LWIP_DEBUG 0

// Disable stats to save space
#define LWIP_STATS 0

// Optional: allow loopback
#define LWIP_NETIF_LOOPBACK 0

// Checksums: use lwIP defaults (software)
// If you later care about perf, you can revisit.
