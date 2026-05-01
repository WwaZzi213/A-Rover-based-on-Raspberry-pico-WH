#include "ultrasonic.h"
#include "config.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/gpio.h"

// 测量间隔：@100Hz 控制循环，6 tick = 60ms（HC-SR04 建议最小间隔 60ms）
#define MEASURE_INTERVAL_TICKS  6u

// 量程上限及对应回波超时
// 距离(cm) = 回波时长(µs) / 58；50cm → 2900µs
#define ULTRASONIC_MAX_RANGE_CM  50u
#define ECHO_TIMEOUT_US          (ULTRASONIC_MAX_RANGE_CM * 58u)  // 2900 µs

static int32_t s_cached_cm  = -1;
static uint8_t s_tick_count = 0;

// 返回值：
//   -1                       传感器无响应（ECHO 未在 1ms 内拉高）
//   1 .. ULTRASONIC_MAX_RANGE_CM  实测距离（cm）
//   ULTRASONIC_MAX_RANGE_CM+1    超出量程（视为无障碍）
static int32_t measure_once(void) {
    // 触发脉冲：先 LOW 2µs 清空，再 HIGH 10µs
    gpio_put(ULTRASONIC_TRIG_PIN, 0);
    sleep_us(2);
    gpio_put(ULTRASONIC_TRIG_PIN, 1);
    sleep_us(10);
    gpio_put(ULTRASONIC_TRIG_PIN, 0);

    // 等待 ECHO 拉高（HC-SR04 发出超声波后拉高，典型 ~500µs，最长等 5ms）
    uint32_t t0 = time_us_32();
    while (!gpio_get(ULTRASONIC_ECHO_PIN)) {
        if ((time_us_32() - t0) > 5000u) return -1;
    }

    // 测量 ECHO 高电平持续时间
    uint32_t echo_start = time_us_32();
    while (gpio_get(ULTRASONIC_ECHO_PIN)) {
        if ((time_us_32() - echo_start) > ECHO_TIMEOUT_US) {
            return (int32_t)(ULTRASONIC_MAX_RANGE_CM + 1u);
        }
    }
    uint32_t duration_us = time_us_32() - echo_start;

    return (int32_t)(duration_us / 58u);
}

void ultrasonic_init(void) {
    gpio_init(ULTRASONIC_TRIG_PIN);
    gpio_set_dir(ULTRASONIC_TRIG_PIN, GPIO_OUT);
    gpio_put(ULTRASONIC_TRIG_PIN, 0);

    gpio_init(ULTRASONIC_ECHO_PIN);
    gpio_set_dir(ULTRASONIC_ECHO_PIN, GPIO_IN);
    gpio_pull_down(ULTRASONIC_ECHO_PIN);  // 防止浮高导致误判
}

void ultrasonic_update(void) {
    if (++s_tick_count < MEASURE_INTERVAL_TICKS) return;
    s_tick_count = 0;
    s_cached_cm = measure_once();
}

int32_t ultrasonic_get_distance_cm(void) {
    return s_cached_cm;
}
