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
#include "ultrasonic.h"
#include "servo_control.h"
#include "auto_drive.h"

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
    ultrasonic_init();
    servo_init();
    auto_drive_init();

    // ---- BLE 服务器初始化（内部启动 BTstack run loop）----
    ble_server_init();

    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);

    // ---- 主循环状态 ----
    int16_t   applied_left    = 0;
    int16_t   applied_right   = 0;
    bool      obstacle_active = false;
    DriveMode prev_mode       = DRIVE_MODE_MANUAL;
    uint32_t  last_log_ms     = to_ms_since_boot(get_absolute_time());
    uint32_t  last_loop_us    = time_us_32();

    printf("[MAIN] Control loop running at %d Hz\n", CONTROL_LOOP_HZ);

    while (true) {
        // ---- 驱动 BTstack 事件循环（cyw43_arch_none 必须手动 poll）----
        cyw43_arch_poll();

        // ---- 精确 100 Hz 定时 ----
        uint32_t now_us = time_us_32();
        if ((now_us - last_loop_us) < CONTROL_LOOP_US) {
            continue;
        }
        last_loop_us = now_us;

        // ---- 超声波测量（三路交错，每路 60ms 一次）----
        ultrasonic_update();

        // ---- Failsafe 检查 ----
        FailsafeState fs = failsafe_update();

        if (fs == FS_LINK_LOST) {
            applied_left  = slew(applied_left,  0, SLEW_RATE_PER_TICK);
            applied_right = slew(applied_right, 0, SLEW_RATE_PER_TICK);
            motor_set(applied_left, applied_right, false);
        } else {
            // ---- 取最新 BLE 命令 ----
            CmdPacket cmd;
            uint32_t apply_us = time_us_32();
            bool has_cmd = ble_server_get_latest_cmd(&cmd, apply_us);

            DriveMode mode = ble_server_get_drive_mode();

            // ---- 模式切换时调整舵机角度 ----
            if (mode != prev_mode) {
                if (mode == DRIVE_MODE_AUTO) {
                    servo_set_angle(SERVO_LEFT,  SERVO_AUTO_LEFT_DEG);
                    servo_set_angle(SERVO_RIGHT, SERVO_AUTO_RIGHT_DEG);
                } else {
                    servo_set_angle(SERVO_LEFT,  SERVO_MANUAL_LEFT_DEG);
                    servo_set_angle(SERVO_RIGHT, SERVO_MANUAL_RIGHT_DEG);
                }
                prev_mode = mode;
            }

            if (mode == DRIVE_MODE_AUTO) {
                // ---- 全自动模式：算法提供目标值 ----
                int16_t auto_left = 0, auto_right = 0;
                auto_drive_update(
                    ultrasonic_get_cm(US_FRONT),
                    ultrasonic_get_cm(US_LEFT),
                    ultrasonic_get_cm(US_RIGHT),
                    &auto_left, &auto_right);
                applied_left  = slew(applied_left,  auto_left,  SLEW_RATE_PER_TICK);
                applied_right = slew(applied_right, auto_right, SLEW_RATE_PER_TICK);
                motor_set(applied_left, applied_right, false);
                obstacle_active = false;
            } else {
                // ---- 手动模式 ----
                bool enable = has_cmd && (cmd.flags & CMD_FLAG_ENABLE);
                bool brake  = has_cmd && (cmd.flags & CMD_FLAG_BRAKE);
                int16_t target_left  = enable ? cmd.left  : 0;
                int16_t target_right = enable ? cmd.right : 0;

                // ---- 三传感器避障状态机 ----
                uint8_t threshold = ble_server_get_obstacle_cm();
                bool obstacle_now = ultrasonic_any_in_range((int32_t)threshold);

                if (obstacle_now && !obstacle_active) {
                    printf("[US] OBSTACLE! F=%ldcm L=%ldcm R=%ldcm (thr=%ucm)\n",
                           (long)ultrasonic_get_cm(US_FRONT),
                           (long)ultrasonic_get_cm(US_LEFT),
                           (long)ultrasonic_get_cm(US_RIGHT),
                           threshold);
                    obstacle_active = true;
                }

                // 退出避障：障碍消失 且 不是纯前进
                if (obstacle_active && !obstacle_now) {
                    bool pure_forward = (target_left < 0 && target_right < 0);
                    if (!pure_forward) {
                        printf("[US] Obstacle cleared\n");
                        obstacle_active = false;
                    }
                }

                // 避障中：屏蔽两轮同时前进，保留后退和坦克转
                if (obstacle_active) {
                    if (target_left < 0 && target_right < 0) {
                        target_left  = 0;
                        target_right = 0;
                    }
                    brake = false;
                }

                if (has_cmd) {
                    applied_left  = slew(applied_left,  target_left,  SLEW_RATE_PER_TICK);
                    applied_right = slew(applied_right, target_right, SLEW_RATE_PER_TICK);
                    motor_set(applied_left, applied_right, brake);
                    ble_server_send_ack(cmd.seq, applied_left, applied_right, 0);
                }
            }
        }

        // ---- 1Hz 统计日志 ----
        uint32_t now_ms = to_ms_since_boot(get_absolute_time());
        if (now_ms - last_log_ms >= 1000) {
            last_log_ms = now_ms;

            BleStats st;
            ble_stats_snapshot(&st);

            const char *conn_str;
            if (!st.connected)            conn_str = "ADVERTISING";
            else if (fs == FS_LINK_LOST)  conn_str = "LINK_LOST";
            else                          conn_str = "CONNECTED";

            printf("[STAT t=%lus] state=%-11s F=%ldcm L=%ldcm R=%ldcm "
                   "rx=%3lu drop=%2lu lost=%2lu | "
                   "arr_avg=%5luus arr_max=%5luus | "
                   "apply_avg=%4luus apply_max=%4luus | "
                   "fs=%lu\n",
                   (unsigned long)(now_ms / 1000),
                   conn_str,
                   (long)ultrasonic_get_cm(US_FRONT),
                   (long)ultrasonic_get_cm(US_LEFT),
                   (long)ultrasonic_get_cm(US_RIGHT),
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
