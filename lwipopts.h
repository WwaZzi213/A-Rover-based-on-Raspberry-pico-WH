#pragma once

// Pico W lwIP 配置 - Raw API 模式
// 适配 pico_cyw43_arch_lwip_threadsafe_background

// 核心配置：NO_SYS 模式
#ifndef NO_SYS
#define NO_SYS 1
#endif

// 明确禁用 Sequential API（这是关键！）
#define LWIP_NETCONN 0
#define LWIP_SOCKET 0
#define LWIP_COMPAT_SOCKETS 0

// 禁用 netconn/socket 相关的信号量
#define LWIP_NETCONN_SEM_PER_THREAD 0
#define LWIP_NETCONN_FULLDUPLEX 0

// 线程相关（NO_SYS=1 时不需要）
#define LWIP_TCPIP_CORE_LOCKING 0
#define LWIP_TCPIP_CORE_LOCKING_INPUT 0

// 内存配置
#define MEM_ALIGNMENT 4
#define MEM_SIZE (32 * 1024)        // 增加到 32KB
#define MEMP_NUM_PBUF 32            // 增加 pbuf
#define MEMP_NUM_TCP_PCB 8
#define MEMP_NUM_TCP_SEG 32         // 增加 TCP 段
#define MEMP_NUM_NETCONN 0
#define PBUF_POOL_SIZE 32           // 增加 pool
#define PBUF_POOL_BUFSIZE 1536

// TCP
#define LWIP_TCP 1
#define TCP_MSS 1460
#define TCP_SND_BUF (4 * TCP_MSS)
#define TCP_WND (4 * TCP_MSS)
#define TCP_SND_QUEUELEN 8
#define TCP_LISTEN_BACKLOG 1

// UDP
#define LWIP_UDP 1
#define MEMP_NUM_UDP_PCB 8

// DHCP
#define LWIP_DHCP 1
#define LWIP_AUTOIP 0

// DNS
#define LWIP_DNS 1

// ICMP
#define LWIP_ICMP 1

// ARP（确保启用）
#define LWIP_ARP 1
#define ARP_QUEUEING 1
#define ETHARP_SUPPORT_STATIC_ENTRIES 1

// 以太网
#define LWIP_ETHERNET 1

// 调试和统计
#define LWIP_STATS 0
#define LWIP_DEBUG 0

// 其他
#define LWIP_NETIF_LOOPBACK 0
#define LWIP_HAVE_LOOPIF 0

// 校验和
#define CHECKSUM_GEN_IP 1
#define CHECKSUM_GEN_UDP 1
#define CHECKSUM_GEN_TCP 1
#define CHECKSUM_CHECK_IP 1
#define CHECKSUM_CHECK_UDP 1
#define CHECKSUM_CHECK_TCP 1

// IPv6 (禁用以节省空间)
#define LWIP_IPV6 0