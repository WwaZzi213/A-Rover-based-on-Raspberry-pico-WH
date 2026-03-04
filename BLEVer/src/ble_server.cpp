#include "ble_server.h"
#include "config.h"
#include "protocol.h"
#include "failsafe.h"

#include "btstack.h"
#include "pico/cyw43_arch.h"
#include "pico/btstack_cyw43.h"
#include "pico/time.h"
#include "pico/sync.h"

#include <string.h>
#include <stdio.h>

#include "rover_gatt.h"

// ATT handles（与 rover_gatt.h 中宏名完全一致）
#define HANDLE_CMD     ATT_CHARACTERISTIC_12345678_1234_1234_1234_1234567890ac_01_VALUE_HANDLE
#define HANDLE_ACK     ATT_CHARACTERISTIC_12345678_1234_1234_1234_1234567890ad_01_VALUE_HANDLE
#define HANDLE_ACK_CCC ATT_CHARACTERISTIC_12345678_1234_1234_1234_1234567890ad_01_CLIENT_CONFIGURATION_HANDLE

// -------------------------------------------------------
//  统计：内部累计
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

// 最新命令
static CmdPacket s_latest        = {};
static bool      s_has_cmd       = false;
static uint32_t  s_latest_rx_us  = 0;
static uint32_t  s_last_seq      = 0;
static bool      s_seq_init      = false;
static uint32_t  s_last_arrive_us = 0;

// 待发 ACK
typedef struct {
    bool     pending;
    uint32_t seq_echo;
    int16_t  applied_left;
    int16_t  applied_right;
    uint8_t  status;
} PendingAck;
static PendingAck s_pending_ack = {};

// 连接状态
static hci_con_handle_t s_conn_handle    = HCI_CON_HANDLE_INVALID;
static bool             s_notify_enabled = false;
static bool             s_connected      = false;

// BTstack 注册容器
static btstack_packet_callback_registration_t s_hci_event_cb_reg;

// 广播数据（静态构建，避免在中断上下文操作栈）
static uint8_t s_adv_data[31];
static uint8_t s_adv_len = 0;

// -------------------------------------------------------
//  构建广播数据
// -------------------------------------------------------
static void build_adv_data(void) {
    const uint8_t name_len = (uint8_t)(sizeof(BLE_DEVICE_NAME) - 1);
    uint8_t pos = 0;
    s_adv_data[pos++] = 0x02;
    s_adv_data[pos++] = 0x01;  // Flags
    s_adv_data[pos++] = 0x06;  // LE General Discoverable | BR/EDR Not Supported
    s_adv_data[pos++] = (uint8_t)(1 + name_len);
    s_adv_data[pos++] = 0x09;  // Complete Local Name
    memcpy(&s_adv_data[pos], BLE_DEVICE_NAME, name_len);
    pos += name_len;
    s_adv_len = pos;
}

// -------------------------------------------------------
//  开始广播
// -------------------------------------------------------
static void start_advertising(void) {
    // 广播间隔 100ms，可连接非定向
    uint16_t adv_int_min = 0x00A0;
    uint16_t adv_int_max = 0x00C8;
    uint8_t  adv_type    = 0;       // ADV_IND
    bd_addr_t null_addr  = {};

    gap_advertisements_set_params(adv_int_min, adv_int_max, adv_type,
                                  0, null_addr, 0x07, 0x00);
    gap_advertisements_set_data(s_adv_len, s_adv_data);
    gap_advertisements_enable(1);
    printf("[BLE] Advertising as '%s'\n", BLE_DEVICE_NAME);
}

// -------------------------------------------------------
//  处理 CMD 写入
// -------------------------------------------------------
static void handle_cmd_write(const uint8_t *buf, uint16_t len) {
    uint32_t now_us = time_us_32();
    CmdPacket cmd;
    if (!proto_parse_cmd(buf, len, &cmd)) {
        uint32_t irq = spin_lock_blocking(s_lock);
        s_acc.rx_drop_crc++;
        s_drop_total++;
        spin_unlock(s_lock, irq);
        return;
    }

    uint32_t irq = spin_lock_blocking(s_lock);
    if (s_seq_init) {
        uint32_t expected = s_last_seq + 1;
        if (cmd.seq > expected)
            s_acc.seq_gap_lost += (cmd.seq - expected);
    }
    s_last_seq  = cmd.seq;
    s_seq_init  = true;

    if (s_last_arrive_us != 0) {
        uint32_t interval = now_us - s_last_arrive_us;
        s_acc.inter_arrival_sum_us += interval;
        s_acc.inter_arrival_count++;
        if (interval > s_acc.inter_arrival_max_us)
            s_acc.inter_arrival_max_us = interval;
    }
    s_last_arrive_us = now_us;

    s_latest       = cmd;
    s_has_cmd      = true;
    s_latest_rx_us = now_us;
    s_acc.rx_valid++;
    s_rx_total++;
    spin_unlock(s_lock, irq);

    failsafe_feed();
}

// -------------------------------------------------------
//  ATT 回调
// -------------------------------------------------------
static uint16_t att_read_cb(hci_con_handle_t connection_handle,
                             uint16_t attribute_handle,
                             uint16_t offset,
                             uint8_t *buffer,
                             uint16_t buffer_size)
{
    (void)connection_handle; (void)attribute_handle;
    (void)offset; (void)buffer; (void)buffer_size;
    return 0;
}

static int att_write_cb(hci_con_handle_t connection_handle,
                        uint16_t attribute_handle,
                        uint16_t transaction_mode,
                        uint16_t offset,
                        uint8_t *buffer,
                        uint16_t buffer_size)
{
    (void)connection_handle; (void)transaction_mode; (void)offset;
    if (attribute_handle == HANDLE_CMD) {
        handle_cmd_write(buffer, buffer_size);
    } else if (attribute_handle == HANDLE_ACK_CCC) {
        if (buffer_size >= 2) {
            uint16_t val = (uint16_t)(buffer[0] | (buffer[1] << 8));
            s_notify_enabled = (val == GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION);
        }
    }
    return 0;
}

// -------------------------------------------------------
//  HCI 事件回调
// -------------------------------------------------------
static void hci_event_handler(uint8_t packet_type,
                               uint16_t channel,
                               uint8_t *packet,
                               uint16_t size)
{
    (void)channel; (void)size;
    if (packet_type != HCI_EVENT_PACKET) return;

    switch (hci_event_packet_get_type(packet)) {
    case BTSTACK_EVENT_STATE:
        if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING) {
            bd_addr_t local_addr;
            gap_local_bd_addr(local_addr);
            printf("[BLE] Stack ready. addr=%s\n", bd_addr_to_str(local_addr));
            start_advertising();
        }
        break;

    case HCI_EVENT_LE_META:
        if (hci_event_le_meta_get_subevent_code(packet) ==
            HCI_SUBEVENT_LE_CONNECTION_COMPLETE) {
            s_conn_handle    = hci_subevent_le_connection_complete_get_connection_handle(packet);
            s_connected      = true;
            s_notify_enabled = false;
            uint32_t irq = spin_lock_blocking(s_lock);
            s_seq_init       = false;
            s_last_arrive_us = 0;
            spin_unlock(s_lock, irq);
            printf("[BLE] Connected  handle=0x%04x  t=%lu us\n",
                   s_conn_handle, (unsigned long)time_us_32());
        }
        break;

    case HCI_EVENT_DISCONNECTION_COMPLETE:
        printf("[BLE] Disconnected  reason=0x%02x  t=%lu us\n",
               hci_event_disconnection_complete_get_reason(packet),
               (unsigned long)time_us_32());
        s_conn_handle    = HCI_CON_HANDLE_INVALID;
        s_connected      = false;
        s_notify_enabled = false;
        start_advertising();
        break;

    default:
        break;
    }
}

// -------------------------------------------------------
//  Notify 回调
// -------------------------------------------------------
static void att_server_notify_ready(uint8_t packet_type, uint16_t channel,
                                    uint8_t *packet, uint16_t size)
{
    (void)channel; (void)size;
    if (packet_type != ATT_EVENT_CAN_SEND_NOW) return;

    hci_con_handle_t handle = s_conn_handle;
    if (!s_notify_enabled) return;

    uint32_t irq = spin_lock_blocking(s_lock);
    if (!s_pending_ack.pending) {
        spin_unlock(s_lock, irq);
        return;
    }
    PendingAck pa = s_pending_ack;
    s_pending_ack.pending = false;
    spin_unlock(s_lock, irq);

    AckPacket ack;
    ack.magic[0]      = 'R';
    ack.magic[1]      = 'A';
    ack.ver           = 1;
    ack.status        = pa.status;
    ack.seq_echo      = pa.seq_echo;
    ack.applied_left  = pa.applied_left;
    ack.applied_right = pa.applied_right;
    ack.t_us          = time_us_32();

    att_server_notify(handle, HANDLE_ACK,
                      (const uint8_t *)&ack, sizeof(AckPacket));
}

// -------------------------------------------------------
//  公共接口
// -------------------------------------------------------
void ble_server_init(void) {
    int lock_num = spin_lock_claim_unused(true);
    s_lock = spin_lock_instance(lock_num);

    build_adv_data();

    // 使用 pico-sdk 提供的 btstack_cyw43 run loop（与 pico-examples 完全一致）
    btstack_cyw43_init(cyw43_arch_async_context());

    l2cap_init();
    sm_init();
    sm_set_io_capabilities(IO_CAPABILITY_NO_INPUT_NO_OUTPUT);

    att_server_init(profile_data, att_read_cb, att_write_cb);
    att_server_register_packet_handler(att_server_notify_ready);

    s_hci_event_cb_reg.callback = &hci_event_handler;
    hci_add_event_handler(&s_hci_event_cb_reg);

    hci_power_control(HCI_POWER_ON);

    printf("[BLE] Initialized\n");
}

void ble_server_poll(void) {
    // btstack_cyw43 通过 async_context 自动驱动，无需手动 poll
}

bool ble_server_get_latest_cmd(CmdPacket *out, uint32_t apply_time_us) {
    uint32_t irq = spin_lock_blocking(s_lock);
    if (!s_has_cmd) {
        spin_unlock(s_lock, irq);
        return false;
    }
    *out = s_latest;
    if (s_latest_rx_us != 0 && apply_time_us >= s_latest_rx_us) {
        uint32_t delay = apply_time_us - s_latest_rx_us;
        s_acc.rx_to_apply_sum_us += delay;
        s_acc.rx_to_apply_count++;
        if (delay > s_acc.rx_to_apply_max_us)
            s_acc.rx_to_apply_max_us = delay;
        s_latest_rx_us = 0;
    }
    spin_unlock(s_lock, irq);
    return true;
}

void ble_server_send_ack(uint32_t seq_echo,
                         int16_t  applied_left,
                         int16_t  applied_right,
                         uint8_t  status)
{
    if (!s_connected || !s_notify_enabled) return;
    uint32_t irq = spin_lock_blocking(s_lock);
    s_pending_ack.pending       = true;
    s_pending_ack.seq_echo      = seq_echo;
    s_pending_ack.applied_left  = applied_left;
    s_pending_ack.applied_right = applied_right;
    s_pending_ack.status        = status;
    spin_unlock(s_lock, irq);
    att_server_request_can_send_now_event(s_conn_handle);
}

void ble_stats_snapshot(BleStats *out) {
    uint32_t irq = spin_lock_blocking(s_lock);
    out->rx_valid     = s_acc.rx_valid;
    out->rx_drop_crc  = s_acc.rx_drop_crc;
    out->seq_gap_lost = s_acc.seq_gap_lost;
    out->connected    = s_connected;
    out->inter_arrival_avg_us = s_acc.inter_arrival_count > 0
        ? s_acc.inter_arrival_sum_us / s_acc.inter_arrival_count : 0;
    out->inter_arrival_max_us = s_acc.inter_arrival_max_us;
    out->rx_to_apply_avg_us = s_acc.rx_to_apply_count > 0
        ? s_acc.rx_to_apply_sum_us / s_acc.rx_to_apply_count : 0;
    out->rx_to_apply_max_us = s_acc.rx_to_apply_max_us;
    s_acc = {};
    spin_unlock(s_lock, irq);
}

uint32_t ble_server_rx_count(void)   { return s_rx_total;   }
uint32_t ble_server_drop_count(void) { return s_drop_total; }