// main.cpp — Pico W TCP RC 小车控制器
// 主循环结构与 UDP 版完全一致，便于对照实验

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "pico/time.h"
#include "lwip/netif.h"

#include "config.h"
#include "protocol.h"
#include "motor_control.h"
#include "failsafe.h"
#include "tcp_server.h"

// -------------------------------------------------------
//  Slew rate 限幅辅助（与 UDP 版完全相同）
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

    // 短暂等待 USB CDC 枚举
    sleep_ms(1500);
    printf("\n=== Pico W TCP Car ===\n");
    printf("Build: %s %s\n", __DATE__, __TIME__);
    printf("PWM: %uHz  max=%d  slew=%d/tick\n",
           PWM_FREQ_HZ, PWM_CMD_MAX, SLEW_RATE_PER_TICK);
    printf("Failsafe: %dms  TCP port: %d\n\n",
           FAILSAFE_TIMEOUT_MS, TCP_PORT);

    // ---- CYW43 初始化（与 UDP 版相同）----
    if (cyw43_arch_init()) {
        printf("[INIT] cyw43_arch_init failed!\n");
        while (true) tight_loop_contents();
    }
    cyw43_arch_enable_sta_mode();

    // LED on = 正在连接
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);

    printf("[WIFI] Connecting to '%s'...\n", WIFI_SSID);
    int wifi_ret = cyw43_arch_wifi_connect_timeout_ms(
        WIFI_SSID, WIFI_PASSWORD,
        CYW43_AUTH_WPA2_AES_PSK,
        WIFI_TIMEOUT_MS);

    if (wifi_ret != 0) {
        printf("[WIFI] Connect failed! err=%d\n", wifi_ret);
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        while (true) tight_loop_contents();
    }

    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
    const ip4_addr_t *ip = netif_ip4_addr(netif_default);
    printf("[WIFI] Connected! IP: %s\n", ip4addr_ntoa(ip));

    // ---- 硬件初始化（与 UDP 版完全相同）----
    motor_init();
    failsafe_init();

    // ---- TCP 服务器（对应 UDP 版的 udp_server_init）----
    cyw43_arch_lwip_begin();
    int tcp_ret = tcp_server_init();
    cyw43_arch_lwip_end();

    if (tcp_ret != 0) {
        printf("[TCP] Init failed! err=%d\n", tcp_ret);
        while (true) tight_loop_contents();
    }

    // ---- 主循环状态（与 UDP 版完全相同）----
    int16_t applied_left  = 0;
    int16_t applied_right = 0;
    uint32_t last_log_ms  = to_ms_since_boot(get_absolute_time());
    uint32_t last_loop_us = time_us_32();

    printf("[MAIN] Control loop running at %d Hz\n", CONTROL_LOOP_HZ);
    // 串口统计头（CSV 格式，与 UDP 版相同字段顺序）
    printf("[STAT] fmt: t(s), state, rx, drop_crc, seq_lost, "
           "arr_avg(us), arr_max(us), apply_avg(us), apply_max(us), fs_total\n");

    while (true) {
        // ---- 精确 100 Hz 定时（busy-wait，与 UDP 版相同）----
        while ((time_us_32() - last_loop_us) < CONTROL_LOOP_US)
            tight_loop_contents();
        last_loop_us = time_us_32();

        // ---- Failsafe 检查（与 UDP 版完全相同）----
        FailsafeState fs = failsafe_update();

        if (fs == FS_LINK_LOST) {
            // 失联：slew 渐减到 0，避免突停
            applied_left  = slew(applied_left,  0, SLEW_RATE_PER_TICK);
            applied_right = slew(applied_right, 0, SLEW_RATE_PER_TICK);
            motor_set(applied_left, applied_right, false);
        } else {
            // 获取最新命令（对应 UDP 版 udp_server_get_latest_cmd）
            CmdPacket cmd;
            uint32_t apply_us = time_us_32();
            if (tcp_server_get_latest_cmd(&cmd, apply_us)) {
                bool enable = (cmd.flags & CMD_FLAG_ENABLE) != 0;
                bool brake  = (cmd.flags & CMD_FLAG_BRAKE)  != 0;

                int16_t target_left  = enable ? cmd.left  : 0;
                int16_t target_right = enable ? cmd.right : 0;

                // Slew rate 限幅（与 UDP 版相同）
                applied_left  = slew(applied_left,  target_left,  SLEW_RATE_PER_TICK);
                applied_right = slew(applied_right, target_right, SLEW_RATE_PER_TICK);

                motor_set(applied_left, applied_right, brake);
            }
            // get_latest_cmd 返回 false（尚未收到命令）：保持当前输出
        }

        // ---- 1Hz 统计日志（格式与 UDP 版完全对齐）----
        uint32_t now_ms = to_ms_since_boot(get_absolute_time());
        if (now_ms - last_log_ms >= 1000) {
            last_log_ms = now_ms;

            TcpStats st;
            tcp_stats_snapshot(&st);

            // 连接状态字符串（与 UDP 版 conn_str 逻辑相同）
            const char *conn_str;
            if (!st.connected)
                conn_str = "LISTEN";
            else if (fs == FS_LINK_LOST)
                conn_str = "LINK_LOST";
            else
                conn_str = "CONNECTED";

            // 输出格式与 UDP 版 [STAT] 行完全一致
            printf("[STAT t=%lus] state=%-10s "
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
