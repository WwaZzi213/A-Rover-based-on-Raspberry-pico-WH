#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "pico/time.h"
#include "btstack.h"

#include "config.h"
#include "protocol.h"
#include "motor_control.h"
#include "failsafe.h"
#include "ble_server.h"

// -------------------------------------------------------
//  Slew rate 限幅辅助
// -------------------------------------------------------
static inline int16_t slew(int16_t current, int16_t target, int16_t step) {
    int16_t diff = target - current;
    if (diff >  step) diff =  step;
    if (diff < -step) diff = -step;
    return (int16_t)(current + diff);
}

// -------------------------------------------------------
//  main
// -------------------------------------------------------
int main() {
    stdio_init_all();
    sleep_ms(1500);
    printf("\n=== Pico BLE Car ===\n");

    // ---- CYW43 初始化（none 模式：无 WiFi，纯 BLE）----
    if (cyw43_arch_init()) {
        printf("[INIT] cyw43_arch_init failed!\n");
        while (true) tight_loop_contents();
    }

    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);

    // ---- 硬件初始化 ----
    motor_init();
    failsafe_init();

    // ---- BLE 服务器初始化（内部启动 BTstack run loop）----
    ble_server_init();

    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);

    // ---- 主循环状态 ----
    int16_t  applied_left  = 0;
    int16_t  applied_right = 0;
    uint32_t last_log_ms   = to_ms_since_boot(get_absolute_time());
    uint32_t last_loop_us  = time_us_32();

    printf("[MAIN] Control loop running at %d Hz\n", CONTROL_LOOP_HZ);

    while (true) {
        // ---- 驱动 BTstack 事件循环（cyw43_arch_none 必须手动 poll）----
        cyw43_arch_poll();

        // ---- 精确 100 Hz 定时 ----
        uint32_t now_us = time_us_32();
        if ((now_us - last_loop_us) < CONTROL_LOOP_US) {
            continue;   // 未到 10ms，继续 poll BTstack
        }
        last_loop_us = now_us;

        // ---- Failsafe 检查 ----
        FailsafeState fs = failsafe_update();

        if (fs == FS_LINK_LOST) {
            applied_left  = slew(applied_left,  0, SLEW_RATE_PER_TICK);
            applied_right = slew(applied_right, 0, SLEW_RATE_PER_TICK);
            motor_set(applied_left, applied_right, false);
        } else {
            CmdPacket cmd;
            uint32_t apply_us = time_us_32();
            if (ble_server_get_latest_cmd(&cmd, apply_us)) {
                bool enable = (cmd.flags & CMD_FLAG_ENABLE) != 0;
                bool brake  = (cmd.flags & CMD_FLAG_BRAKE)  != 0;

                int16_t target_left  = enable ? cmd.left  : 0;
                int16_t target_right = enable ? cmd.right : 0;

                applied_left  = slew(applied_left,  target_left,  SLEW_RATE_PER_TICK);
                applied_right = slew(applied_right, target_right, SLEW_RATE_PER_TICK);

                motor_set(applied_left, applied_right, brake);

                uint8_t status = (fs == FS_LINK_LOST) ? 1 : 0;
                ble_server_send_ack(cmd.seq, applied_left, applied_right, status);
            }
        }

        // ---- 1Hz 统计日志 ----
        uint32_t now_ms = to_ms_since_boot(get_absolute_time());
        if (now_ms - last_log_ms >= 1000) {
            last_log_ms = now_ms;

            BleStats st;
            ble_stats_snapshot(&st);

            const char *conn_str;
            if (!st.connected)       conn_str = "ADVERTISING";
            else if (fs == FS_LINK_LOST) conn_str = "LINK_LOST";
            else                     conn_str = "CONNECTED";

            printf("[STAT t=%lus] state=%-11s "
                   "rx=%3lu drop_crc=%2lu seq_lost=%2lu | "
                   "arr_avg=%5luus arr_max=%5luus | "
                   "apply_avg=%4luus apply_max=%4luus | "
                   "fs_total=%lu\n",
                   (unsigned long)(now_ms / 1000),
                   conn_str,
                   (unsigned long)st.rx_valid,
                   (unsigned long)st.rx_drop_crc,
                   (unsigned long)st.seq_gap_lost,
                   (unsigned long)st.inter_arrival_avg_us,
                   (unsigned long)st.inter_arrival_max_us,
                   (unsigned long)st.rx_to_apply_avg_us,
                   (unsigned long)st.rx_to_apply_max_us,
                   (unsigned long)failsafe_trigger_count());
        }
    }

    cyw43_arch_deinit();
    return 0;
}