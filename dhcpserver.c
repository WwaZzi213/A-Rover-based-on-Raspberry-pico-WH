#include "dhcpserver.h"
#include <string.h>
#include <stdlib.h>
#include "lwip/udp.h"
#include "lwip/ip4_addr.h"
#include "lwip/prot/dhcp.h"

#ifndef DHCP_SERVER_LEASE_TIME
#define DHCP_SERVER_LEASE_TIME (24 * 60 * 60) // 24h
#endif

#define DHCP_SERVER_PORT 67
#define DHCP_CLIENT_PORT 68

// 简单地址池：192.168.4.2 ~ 192.168.4.20
#define POOL_START 2
#define POOL_END   20

struct dhcp_server {
    struct udp_pcb* pcb;
    struct netif* netif;
    ip4_addr_t ip, mask, gw;
    uint8_t next_host;
};

static void write_u32(uint8_t* p, uint32_t v) {
    p[0] = (uint8_t)((v >> 24) & 0xFF);
    p[1] = (uint8_t)((v >> 16) & 0xFF);
    p[2] = (uint8_t)((v >> 8) & 0xFF);
    p[3] = (uint8_t)(v & 0xFF);
}

static uint8_t* add_opt(uint8_t* opt, uint8_t code, uint8_t len, const void* data) {
    *opt++ = code;
    *opt++ = len;
    memcpy(opt, data, len);
    return opt + len;
}

static uint8_t* add_opt_u32(uint8_t* opt, uint8_t code, uint32_t val) {
    uint8_t tmp[4];
    write_u32(tmp, val);
    return add_opt(opt, code, 4, tmp);
}

static uint8_t* add_opt_u8(uint8_t* opt, uint8_t code, uint8_t val) {
    return add_opt(opt, code, 1, &val);
}

static const uint8_t* find_opt(const uint8_t* opts, uint16_t len, uint8_t code, uint8_t* out_len) {
    uint16_t i = 0;
    while (i < len) {
        uint8_t c = opts[i++];
        if (c == DHCP_OPTION_END) break;
        if (c == DHCP_OPTION_PAD) continue;
        if (i >= len) break;
        uint8_t l = opts[i++];
        if (i + l > len) break;
        if (c == code) {
            if (out_len) *out_len = l;
            return &opts[i];
        }
        i += l;
    }
    return NULL;
}

static ip4_addr_t make_pool_ip(const ip4_addr_t* base, uint8_t host) {
    ip4_addr_t out = *base;
    // base assumed 192.168.4.1; just replace last octet
    uint32_t a = lwip_htonl(ip4_addr_get_u32(&out));
    a = (a & 0xFFFFFF00u) | (uint32_t)host;
    ip4_addr_set_u32(&out, lwip_ntohl(a));
    return out;
}

static void dhcp_recv(void* arg, struct udp_pcb* pcb, struct pbuf* p,
                      const ip_addr_t* addr, u16_t port) {
    (void)pcb; (void)addr; (void)port;
    dhcp_server_t* s = (dhcp_server_t*)arg;
    if (!p) return;

    // Expect DHCP message inside BOOTP format
    if (p->tot_len < sizeof(struct dhcp_msg)) {
        pbuf_free(p);
        return;
    }

    struct dhcp_msg msg;
    pbuf_copy_partial(p, &msg, sizeof(msg), 0);

    // Validate magic cookie
    if (msg.cookie != PP_HTONL(DHCP_MAGIC_COOKIE)) {
        pbuf_free(p);
        return;
    }

    // Options start after fixed dhcp_msg
    uint16_t opt_offset = sizeof(struct dhcp_msg);
    uint16_t opt_len = p->tot_len - opt_offset;
    uint8_t opts_buf[312]; // common size; enough for our parsing
    if (opt_len > sizeof(opts_buf)) opt_len = sizeof(opts_buf);
    pbuf_copy_partial(p, opts_buf, opt_len, opt_offset);

    uint8_t mt_len = 0;
    const uint8_t* mt = find_opt(opts_buf, opt_len, DHCP_OPTION_MESSAGE_TYPE, &mt_len);
    if (!mt || mt_len != 1) {
        pbuf_free(p);
        return;
    }
    uint8_t msg_type = mt[0];

    // Choose an IP from pool
    uint8_t host = s->next_host;
    if (host < POOL_START || host > POOL_END) host = POOL_START;
    s->next_host = (host == POOL_END) ? POOL_START : (uint8_t)(host + 1);

    ip4_addr_t yiaddr = make_pool_ip(&s->ip, host);

    // Build reply
    struct pbuf* q = pbuf_alloc(PBUF_TRANSPORT, sizeof(struct dhcp_msg) + 128, PBUF_RAM);
    if (!q) { pbuf_free(p); return; }

    struct dhcp_msg* r = (struct dhcp_msg*)q->payload;
    memset(r, 0, q->len);

    r->op = DHCP_BOOTREPLY;
    r->htype = msg.htype;
    r->hlen = msg.hlen;
    r->hops = 0;
    r->xid = msg.xid;
    r->secs = 0;
    r->flags = msg.flags;
    memcpy(r->chaddr, msg.chaddr, 16);

    r->cookie = PP_HTONL(DHCP_MAGIC_COOKIE);

    // yiaddr
    ip4_addr_set_u32((ip4_addr_t*)&r->yiaddr, ip4_addr_get_u32(&yiaddr));
    // siaddr = server ip
    ip4_addr_set_u32((ip4_addr_t*)&r->siaddr, ip4_addr_get_u32(&s->ip));

    uint8_t* opt = (uint8_t*)((uint8_t*)r + sizeof(struct dhcp_msg));

    // Message type: OFFER or ACK
    uint8_t reply_type = (msg_type == DHCP_DISCOVER) ? DHCP_OFFER :
                         (msg_type == DHCP_REQUEST)  ? DHCP_ACK   : 0;
    if (!reply_type) { pbuf_free(q); pbuf_free(p); return; }

    opt = add_opt_u8(opt, DHCP_OPTION_MESSAGE_TYPE, reply_type);
    opt = add_opt_u32(opt, DHCP_OPTION_SERVER_ID, ip4_addr_get_u32(&s->ip));
    opt = add_opt_u32(opt, DHCP_OPTION_LEASE_TIME, PP_HTONL(DHCP_SERVER_LEASE_TIME));
    opt = add_opt_u32(opt, DHCP_OPTION_SUBNET_MASK, ip4_addr_get_u32(&s->mask));
    opt = add_opt_u32(opt, DHCP_OPTION_ROUTER, ip4_addr_get_u32(&s->gw));

    // DNS: set to gateway (not required if user uses http://192.168.4.1)
    opt = add_opt_u32(opt, DHCP_OPTION_DNS_SERVER, ip4_addr_get_u32(&s->gw));

    *opt++ = DHCP_OPTION_END;

    // Send to broadcast on client port
    ip_addr_t dst;
    ip4_addr_t bcast;
    ip4_addr_set_u32(&bcast, IPADDR_BROADCAST);
    ip_addr_copy_from_ip4(dst, bcast);

    udp_sendto(s->pcb, q, &dst, DHCP_CLIENT_PORT);

    pbuf_free(q);
    pbuf_free(p);
}

dhcp_server_t* dhcp_server_start(struct netif* netif, const ip4_addr_t* ip, const ip4_addr_t* mask, const ip4_addr_t* gw) {
    dhcp_server_t* s = (dhcp_server_t*)calloc(1, sizeof(dhcp_server_t));
    if (!s) return NULL;

    s->netif = netif;
    s->ip = *ip;
    s->mask = *mask;
    s->gw = *gw;
    s->next_host = POOL_START;

    s->pcb = udp_new_ip_type(IPADDR_TYPE_V4);
    if (!s->pcb) { free(s); return NULL; }

    udp_bind(s->pcb, IP4_ADDR_ANY, DHCP_SERVER_PORT);
    udp_recv(s->pcb, dhcp_recv, s);
    return s;
}

void dhcp_server_stop(dhcp_server_t* server) {
    if (!server) return;
    if (server->pcb) udp_remove(server->pcb);
    free(server);
}
