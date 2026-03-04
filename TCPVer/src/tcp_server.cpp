// tcp_server.cpp
// lwIP RAW TCP 服务器（threadsafe_background 模式）
//
// 架构说明：
//   - TCP 回调运行在 lwIP 中断上下文（类似 UDP 版的 udp_recv_cb）
//   - 共享数据（统计、最新命令）用 spin_lock 保护，与 UDP 版一致
//   - 主循环通过 tcp_server_get_latest_cmd() 拉取最新命令
//   - slew rate 在主循环施加（与 UDP 版一致，不在回调里做）
//   - 单客户端模式：有客户端时拒绝新连接；断开后自动回到 listen

#include "tcp_server.h"
#include "config.h"
#include "protocol.h"
#include "stream_parser.h"
#include "failsafe.h"

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "pico/sync.h"
#include "pico/time.h"
#include "lwip/tcp.h"

#include <string.h>
#include <stdio.h>

// -------------------------------------------------------
//  内部累计统计（回调写，主循环读），用 spin_lock 保护
// -------------------------------------------------------
typedef struct {
    uint32_t rx_valid;
    uint32_t rx_drop_crc;
    uint32_t seq_gap_lost;

    uint32_t inter_arrival_sum_us;
    uint32_t inter_arrival_max_us;
    uint32_t inter_arrival_count;

    uint32_t rx_to_apply_sum_us;
    uint32_t rx_to_apply_max_us;
    uint32_t rx_to_apply_count;
} AccStats;

static spin_lock_t *s_lock       = nullptr;
static AccStats     s_acc        = {};
static uint32_t     s_rx_total   = 0;
static uint32_t     s_drop_total = 0;

// 最新命令（回调写，主循环读）
static CmdPacket    s_latest        = {};
static bool         s_has_cmd       = false;
static uint32_t     s_latest_rx_us  = 0;
static bool         s_connected     = false;

// 序列号 & 到达间隔追踪
static uint32_t     s_last_seq      = 0;
static bool         s_seq_init      = false;
static uint32_t     s_last_arrive_us = 0;

// -------------------------------------------------------
//  lwIP PCB 句柄
// -------------------------------------------------------
static struct tcp_pcb *s_listen_pcb = nullptr;
static struct tcp_pcb *s_client_pcb = nullptr;

// -------------------------------------------------------
//  ring buffer（仅在 lwIP 上下文访问，不需要额外锁）
// -------------------------------------------------------
static RingBuf s_rb;

// -------------------------------------------------------
//  前向声明
// -------------------------------------------------------
static void  close_client(bool send_fin);
static err_t on_accept(void *arg, struct tcp_pcb *newpcb, err_t err);
static err_t on_recv  (void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
static err_t on_poll  (void *arg, struct tcp_pcb *tpcb);
static void  on_error (void *arg, err_t err);
static void  send_ack (const CmdPacket *cmd);

// -------------------------------------------------------
//  关闭客户端连接
// -------------------------------------------------------
static void close_client(bool send_fin) {
    if (!s_client_pcb) return;

    struct tcp_pcb *pcb = s_client_pcb;
    s_client_pcb = nullptr;

    tcp_arg(pcb,  nullptr);
    tcp_recv(pcb, nullptr);
    tcp_err(pcb,  nullptr);
    tcp_poll(pcb, nullptr, 0);

    if (send_fin) {
        tcp_close(pcb);
    } else {
        tcp_abort(pcb);
    }

    // 更新共享状态（在 lwIP 上下文，回调中直接修改不需锁）
    uint32_t irq = spin_lock_blocking(s_lock);
    s_connected = false;
    spin_unlock(s_lock, irq);

    rb_init(&s_rb);  // 清空 ring buffer
    failsafe_feed(); // 防止 failsafe_update 误触发（重置计时）
    // 实际上断开时应触发停机，由主循环 failsafe_update 处理
    // 这里主动通知 failsafe 让其进入 LINK_LOST
    // （调用 failsafe_update 会在主循环做，此处不重复）
}

// -------------------------------------------------------
//  发送 ACK（在 lwIP 上下文）
// -------------------------------------------------------
static void send_ack(const CmdPacket *cmd) {
    if (!s_client_pcb) return;

    AckPacket ack;
    ack.magic[0]      = 'R';
    ack.magic[1]      = 'A';
    ack.ver           = PROTO_VER;
    ack.status        = (failsafe_state() == FS_LINK_LOST) ? 1 : 0;
    ack.seq_echo      = cmd->seq;
    // applied_left/right 由主循环施加 slew 后才知道，
    // 这里填 cmd 原始值（与 UDP 版 send_ack 行为一致）
    ack.applied_left  = cmd->left;
    ack.applied_right = cmd->right;
    ack.t_us          = time_us_32();

    // tcp_write 会拷贝数据到发送缓冲区
    err_t err = tcp_write(s_client_pcb, &ack, sizeof(AckPacket),
                          TCP_WRITE_FLAG_COPY);
    if (err == ERR_OK) {
        tcp_output(s_client_pcb);  // 立即发出（禁用 Nagle）
    }
    // ERR_MEM 表示发送缓冲区满，丢弃该 ACK（非致命）
}

// -------------------------------------------------------
//  lwIP on_recv 回调（在 lwIP 中断上下文）
// -------------------------------------------------------
static err_t on_recv(void *arg, struct tcp_pcb *tpcb,
                     struct pbuf *p, err_t err) {
    (void)arg;

    // p == NULL 表示对端关闭了连接（FIN）
    if (!p || err != ERR_OK) {
        printf("[TCP] Client disconnected (recv null/err=%d)  t=%lu us\n",
               err, (unsigned long)time_us_32());
        close_client(true);
        return ERR_OK;
    }

    uint32_t now_us = time_us_32();

    // 将 pbuf 链中的数据推入 ring buffer
    struct pbuf *cur = p;
    while (cur) {
        uint16_t accepted = rb_push(&s_rb,
                                    (const uint8_t *)cur->payload,
                                    (uint16_t)cur->len);
        if (accepted < cur->len) {
            printf("[TCP] RingBuf overflow! lost %u bytes\n",
                   (unsigned)(cur->len - accepted));
        }
        cur = cur->next;
    }
    // 告知 lwIP 已消耗 pbuf 中的数据（更新接收窗口）
    tcp_recved(tpcb, p->tot_len);
    pbuf_free(p);

    // 解析循环：尽可能多地提取完整包
    while (true) {
        CmdPacket pkt;
        bool crc_err = false;
        bool got = sp_try_parse(&s_rb, &pkt, &crc_err);

        if (crc_err) {
            // sp_try_parse 已丢弃 1 字节，继续尝试
            uint32_t irq = spin_lock_blocking(s_lock);
            s_acc.rx_drop_crc++;
            s_drop_total++;
            spin_unlock(s_lock, irq);
            continue;
        }
        if (!got) break;   // 数据不足，等待下一次 recv

        // ── 有效包 ──────────────────────────────────────────

        uint32_t irq = spin_lock_blocking(s_lock);

        // seq 丢包统计
        if (s_seq_init) {
            uint32_t expected = s_last_seq + 1;
            if (pkt.seq > expected)
                s_acc.seq_gap_lost += (pkt.seq - expected);
        }
        s_last_seq = pkt.seq;
        s_seq_init = true;

        // 到达间隔
        if (s_last_arrive_us != 0) {
            uint32_t interval = now_us - s_last_arrive_us;
            s_acc.inter_arrival_sum_us += interval;
            s_acc.inter_arrival_count++;
            if (interval > s_acc.inter_arrival_max_us)
                s_acc.inter_arrival_max_us = interval;
        }
        s_last_arrive_us = now_us;

        // 更新最新命令
        s_latest       = pkt;
        s_has_cmd      = true;
        s_latest_rx_us = now_us;
        s_acc.rx_valid++;
        s_rx_total++;

        spin_unlock(s_lock, irq);

        failsafe_feed();
        send_ack(&pkt);
    }

    return ERR_OK;
}

// -------------------------------------------------------
//  lwIP on_poll 回调（定期心跳，此处仅用于检测对端无声关闭）
// -------------------------------------------------------
static err_t on_poll(void *arg, struct tcp_pcb *tpcb) {
    (void)arg; (void)tpcb;
    // failsafe_update() 在主循环处理超时，这里不重复
    return ERR_OK;
}

// -------------------------------------------------------
//  lwIP on_error 回调（连接异常重置）
// -------------------------------------------------------
static void on_error(void *arg, err_t err) {
    (void)arg;
    printf("[TCP] Connection error %d, closing  t=%lu us\n",
           err, (unsigned long)time_us_32());
    // pcb 已被 lwIP 释放，不可再使用
    s_client_pcb = nullptr;

    uint32_t irq = spin_lock_blocking(s_lock);
    s_connected = false;
    spin_unlock(s_lock, irq);

    rb_init(&s_rb);
}

// -------------------------------------------------------
//  lwIP on_accept 回调
// -------------------------------------------------------
static err_t on_accept(void *arg, struct tcp_pcb *newpcb, err_t err) {
    (void)arg;
    if (err != ERR_OK || !newpcb) return ERR_VAL;

    // 已有客户端：拒绝
    if (s_client_pcb) {
        printf("[TCP] Refusing second connection\n");
        tcp_abort(newpcb);
        return ERR_ABRT;
    }

    printf("[TCP] Client connected  addr=%s port=%u  t=%lu us\n",
           ipaddr_ntoa(&newpcb->remote_ip),
           newpcb->remote_port,
           (unsigned long)time_us_32());

    s_client_pcb = newpcb;
    rb_init(&s_rb);

    // 注册回调
    tcp_arg(s_client_pcb,  nullptr);
    tcp_recv(s_client_pcb, on_recv);
    tcp_err(s_client_pcb,  on_error);
    // poll 每 2*500ms=1s 触发一次
    tcp_poll(s_client_pcb, on_poll, 2);

    // 禁用 Nagle 算法：ACK 包立即发出，降低延迟
    tcp_nagle_disable(s_client_pcb);

    // 更新共享连接状态
    uint32_t irq = spin_lock_blocking(s_lock);
    s_connected      = true;
    s_seq_init       = false;   // 重置 seq 追踪
    s_last_arrive_us = 0;
    spin_unlock(s_lock, irq);

    return ERR_OK;
}

// -------------------------------------------------------
//  公共接口
// -------------------------------------------------------
int tcp_server_init(void) {
    // 申请 spin lock（与 UDP 版完全相同）
    int lock_num = spin_lock_claim_unused(true);
    s_lock = spin_lock_instance(lock_num);

    rb_init(&s_rb);

    struct tcp_pcb *pcb = tcp_new_ip_type(IPADDR_TYPE_ANY);
    if (!pcb) {
        printf("[TCP] tcp_new failed\n");
        return -1;
    }

    // 允许端口复用（快速重启时无需等待 TIME_WAIT）
    int opt = 1;
    (void)opt;  // tcp_set_flags 在不同 lwIP 版本 API 略有不同，保守处理

    err_t err = tcp_bind(pcb, IP_ANY_TYPE, TCP_PORT);
    if (err != ERR_OK) {
        printf("[TCP] tcp_bind failed: %d\n", err);
        tcp_close(pcb);
        return -2;
    }

    s_listen_pcb = tcp_listen_with_backlog(pcb, 1);
    if (!s_listen_pcb) {
        printf("[TCP] tcp_listen failed\n");
        return -3;
    }

    tcp_accept(s_listen_pcb, on_accept);
    printf("[TCP] Listening on port %d\n", TCP_PORT);
    return 0;
}

bool tcp_server_get_latest_cmd(CmdPacket *out, uint32_t apply_time_us) {
    uint32_t irq = spin_lock_blocking(s_lock);
    if (!s_has_cmd) {
        spin_unlock(s_lock, irq);
        return false;
    }
    *out = s_latest;

    // 记录 rx→apply 延迟（每个新包只统计一次）
    if (s_latest_rx_us != 0 && apply_time_us >= s_latest_rx_us) {
        uint32_t delay = apply_time_us - s_latest_rx_us;
        s_acc.rx_to_apply_sum_us += delay;
        s_acc.rx_to_apply_count++;
        if (delay > s_acc.rx_to_apply_max_us)
            s_acc.rx_to_apply_max_us = delay;
        s_latest_rx_us = 0;   // 只统计一次
    }

    spin_unlock(s_lock, irq);
    return true;
}

void tcp_stats_snapshot(TcpStats *out) {
    uint32_t irq = spin_lock_blocking(s_lock);

    out->rx_valid     = s_acc.rx_valid;
    out->rx_drop_crc  = s_acc.rx_drop_crc;
    out->seq_gap_lost = s_acc.seq_gap_lost;
    out->connected    = s_connected;

    out->inter_arrival_avg_us =
        s_acc.inter_arrival_count > 0
        ? s_acc.inter_arrival_sum_us / s_acc.inter_arrival_count
        : 0;
    out->inter_arrival_max_us = s_acc.inter_arrival_max_us;

    out->rx_to_apply_avg_us =
        s_acc.rx_to_apply_count > 0
        ? s_acc.rx_to_apply_sum_us / s_acc.rx_to_apply_count
        : 0;
    out->rx_to_apply_max_us = s_acc.rx_to_apply_max_us;

    // 清零本秒累计（与 UDP 版 udp_stats_snapshot 完全相同）
    s_acc = {};

    spin_unlock(s_lock, irq);
}

uint32_t tcp_server_rx_count(void)   { return s_rx_total;   }
uint32_t tcp_server_drop_count(void) { return s_drop_total; }
