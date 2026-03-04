#pragma once
#include "protocol.h"
#include <stdint.h>
#include <stdbool.h>

// -------------------------------------------------------
//  初始化 BLE 服务器（BTstack + GATT）
//  必须在 cyw43_arch_init 之后调用
// -------------------------------------------------------
void ble_server_init(void);

// -------------------------------------------------------
//  主循环调用：驱动 BTstack 事件循环
//  使用 cyw43_arch_none 模式时必须周期调用
// -------------------------------------------------------
void ble_server_poll(void);

// -------------------------------------------------------
//  获取最新命令（主循环调用）
//  apply_time_us：当前 time_us_32()，用于计算 rx→apply 延迟
//  返回 true 表示有新命令
// -------------------------------------------------------
bool ble_server_get_latest_cmd(CmdPacket *out, uint32_t apply_time_us);

// -------------------------------------------------------
//  发送 ACK Notify（主循环在 motor_set 后调用）
// -------------------------------------------------------
void ble_server_send_ack(uint32_t seq_echo,
                         int16_t  applied_left,
                         int16_t  applied_right,
                         uint8_t  status);

// -------------------------------------------------------
//  每秒统计快照（主循环 1Hz 调用，读取并清零）
// -------------------------------------------------------
typedef struct {
    uint32_t rx_valid;              // 本秒有效包
    uint32_t rx_drop_crc;           // 本秒 CRC/格式丢弃
    uint32_t seq_gap_lost;          // 根据 seq 推算的丢包数

    uint32_t inter_arrival_avg_us;  // 到达间隔均值
    uint32_t inter_arrival_max_us;  // 到达间隔峰值

    uint32_t rx_to_apply_avg_us;    // rx→apply 延迟均值
    uint32_t rx_to_apply_max_us;    // rx→apply 延迟峰值

    bool     connected;             // 当前是否有客户端连接
} BleStats;

void ble_stats_snapshot(BleStats *out);

// -------------------------------------------------------
//  全局累计（不清零）
// -------------------------------------------------------
uint32_t ble_server_rx_count(void);
uint32_t ble_server_drop_count(void);
