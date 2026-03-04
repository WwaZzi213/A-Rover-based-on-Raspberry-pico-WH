#pragma once
#include "protocol.h"
#include <stdint.h>
#include <stdbool.h>

// 初始化 TCP 服务器（在 cyw43_arch_lwip_begin/end 中调用）
int tcp_server_init(void);

// 主循环调用：获取最新命令，同时记录 rx_to_apply 时间戳
// apply_time_us 传入当前 time_us_32()，用于计算 rx→apply 延迟
// 与 UDP 版 udp_server_get_latest_cmd 接口完全相同
bool tcp_server_get_latest_cmd(CmdPacket *out, uint32_t apply_time_us);

// -------------------------------------------------------
//  每秒统计快照（与 UDP 版 UdpStats 字段完全一致，便于对照）
// -------------------------------------------------------
typedef struct {
    // 包计数
    uint32_t rx_valid;           // 本秒有效包
    uint32_t rx_drop_crc;        // 本秒 CRC/格式丢弃
    uint32_t seq_gap_lost;       // 根据 seq 推算的丢包数

    // 到达间隔（us）
    uint32_t inter_arrival_avg_us;
    uint32_t inter_arrival_max_us;

    // rx → apply 延迟（us）
    uint32_t rx_to_apply_avg_us;
    uint32_t rx_to_apply_max_us;

    // 连接状态
    bool     connected;          // 当前是否有 TCP 客户端
} TcpStats;

// 获取本秒快照并原子清零累计值（主循环 1Hz 调用）
void tcp_stats_snapshot(TcpStats *out);

// 全局累计（不清零）
uint32_t tcp_server_rx_count(void);
uint32_t tcp_server_drop_count(void);
