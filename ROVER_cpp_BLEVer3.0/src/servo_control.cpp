#include "servo_control.h"
#include "config.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"

// 50Hz PWM：目标 1MHz 有效时钟，wrap=19999 → 周期 20ms
#define SERVO_WRAP  19999u

void servo_init(void) {
    gpio_set_function(LEFT_SERVO_PIN,  GPIO_FUNC_PWM);
    gpio_set_function(RIGHT_SERVO_PIN, GPIO_FUNC_PWM);

    uint slice = pwm_gpio_to_slice_num(LEFT_SERVO_PIN);  // GPIO14/15 均在 slice 7

    // clkdiv 使每计数 = 1µs（有效时钟 = 1MHz）
    float div = (float)clock_get_hz(clk_sys) / 1000000.0f;
    pwm_set_clkdiv(slice, div);
    pwm_set_wrap(slice, SERVO_WRAP);
    pwm_set_enabled(slice, true);

    servo_set_angle(SERVO_LEFT,  SERVO_MANUAL_LEFT_DEG);
    servo_set_angle(SERVO_RIGHT, SERVO_MANUAL_RIGHT_DEG);
}

void servo_set_angle(uint8_t servo_id, uint8_t degrees) {
    if (degrees > 180) degrees = 180;
    // 线性插值：0° → SERVO_MIN_US，180° → SERVO_MAX_US
    uint16_t us = (uint16_t)(SERVO_MIN_US +
                  (uint32_t)degrees * (SERVO_MAX_US - SERVO_MIN_US) / 180u);
    uint slice = pwm_gpio_to_slice_num(LEFT_SERVO_PIN);
    if (servo_id == SERVO_LEFT) {
        pwm_set_chan_level(slice, PWM_CHAN_A, us);
    } else {
        pwm_set_chan_level(slice, PWM_CHAN_B, us);
    }
}
