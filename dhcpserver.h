#pragma once
#include "lwip/netif.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct dhcp_server dhcp_server_t;

// 在指定 netif 上启动 DHCP server（给手机分配IP）
dhcp_server_t* dhcp_server_start(struct netif* netif, const ip4_addr_t* ip, const ip4_addr_t* mask, const ip4_addr_t* gw);

// 停止 DHCP server
void dhcp_server_stop(dhcp_server_t* server);

#ifdef __cplusplus
}
#endif
