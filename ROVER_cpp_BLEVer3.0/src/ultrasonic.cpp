#include "ultrasonic.h"
#include "config.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/gpio.h"

// 量程约 50cm → 回波时长上限 2900µs；超出视为无障碍
#define ECHO_TIMEOUT_US   2900u
#define START_TIMEOUT_US  5000u   // 等待 ECHO 拉高的超时（HC-SR04 典型 ~500µs）

static const uint8_t TRIG_PINS[US_COUNT] = {
    ULTRASONIC_TRIG_PIN,
    ULTRASONIC_L_TRIG_PIN,
    ULTRASONIC_R_TRIG_PIN
};
static const uint8_t ECHO_PINS[US_COUNT] = {
    ULTRASONIC_ECHO_PIN,
    ULTRASONIC_L_ECHO_PIN,
    ULTRASONIC_R_ECHO_PIN
};

static int32_t s_dist[US_COUNT] = {-1, -1, -1};
static uint8_t s_tick = 0;

static int32_t measure_once(uint8_t trig, uint8_t echo) {
    gpio_put(trig, 0);
    sleep_us(2);
    gpio_put(trig, 1);
    sleep_us(10);
    gpio_put(trig, 0);

    uint32_t t0 = time_us_32();
    while (!gpio_get(echo)) {
        if ((time_us_32() - t0) > START_TIMEOUT_US) return -1;
    }
    uint32_t start = time_us_32();
    while (gpio_get(echo)) {
        if ((time_us_32() - start) > ECHO_TIMEOUT_US) return -1;
    }
    return (int32_t)((time_us_32() - start) / 58u);
}

void ultrasonic_init(void) {
    for (int i = 0; i < US_COUNT; i++) {
        gpio_init(TRIG_PINS[i]);
        gpio_set_dir(TRIG_PINS[i], GPIO_OUT);
        gpio_put(TRIG_PINS[i], 0);

        gpio_init(ECHO_PINS[i]);
        gpio_set_dir(ECHO_PINS[i], GPIO_IN);
        gpio_pull_down(ECHO_PINS[i]);
    }
}

// 6-tick 周期：tick 0 测前方，tick 2 测左，tick 4 测右
void ultrasonic_update(void) {
    if (s_tick == 0) s_dist[US_FRONT] = measure_once(TRIG_PINS[US_FRONT], ECHO_PINS[US_FRONT]);
    if (s_tick == 2) s_dist[US_LEFT]  = measure_once(TRIG_PINS[US_LEFT],  ECHO_PINS[US_LEFT]);
    if (s_tick == 4) s_dist[US_RIGHT] = measure_once(TRIG_PINS[US_RIGHT], ECHO_PINS[US_RIGHT]);
    if (++s_tick >= 6) s_tick = 0;
}

int32_t ultrasonic_get_cm(uint8_t sensor_id) {
    if (sensor_id >= US_COUNT) return -1;
    return s_dist[sensor_id];
}

bool ultrasonic_any_in_range(int32_t threshold_cm) {
    for (int i = 0; i < US_COUNT; i++) {
        if (s_dist[i] > 0 && s_dist[i] < threshold_cm) return true;
    }
    return false;
}
