#include "motor_control.h"
#include "config.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include <stdlib.h>  // abs

// PWM wrap 值（在 motor_init 中计算）
static uint32_t pwm_wrap = 0;

static inline void setup_pwm_pin(uint pin, uint32_t wrap, float div) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(pin);
    pwm_set_clkdiv(slice, div);
    pwm_set_wrap(slice, (uint16_t)wrap);
    pwm_set_enabled(slice, true);
}

static inline void set_channel_pwm(uint pwm_pin, uint dir_pin,
                                    int16_t value, bool brake, bool invert) {
    // value: -1000..1000
    bool forward = (value >= 0);
    if (invert) forward = !forward;
    int16_t magnitude = (int16_t)abs((int)value);
    if (magnitude > PWM_CMD_MAX) magnitude = PWM_CMD_MAX;

    // DIR 信号
    // Cytron MDDS20 DIR+PWM 模式：DIR=1 正转，DIR=0 反转
    gpio_put(dir_pin, forward ? 1 : 0);

    if (brake) {
        // 刹车：PWM=0（驱动器自动短路两端）+ 保持 DIR
        pwm_set_gpio_level(pwm_pin, 0);
    } else {
        // 将 magnitude(0..1000) 映射到 PWM 占空比
        uint32_t duty = (uint32_t)magnitude * pwm_wrap / PWM_CMD_MAX;
        pwm_set_gpio_level(pwm_pin, (uint16_t)duty);
    }
}

void motor_init(void) {
    // 计算 wrap：使 PWM 频率 = PWM_FREQ_HZ
    // f_pwm = f_sys / (div * (wrap+1))
    // 取 div=1，wrap = f_sys/f_pwm - 1
    uint32_t sys_clk = clock_get_hz(clk_sys);
    pwm_wrap = sys_clk / PWM_FREQ_HZ - 1;
    if (pwm_wrap > 65535) pwm_wrap = 65535;

    float div = 1.0f;

    // DIR 引脚：普通 GPIO 输出
    gpio_init(LEFT_DIR_PIN);   gpio_set_dir(LEFT_DIR_PIN, GPIO_OUT);
    gpio_init(RIGHT_DIR_PIN);  gpio_set_dir(RIGHT_DIR_PIN, GPIO_OUT);

    // PWM 引脚
    setup_pwm_pin(LEFT_PWM_PIN,  pwm_wrap, div);
    setup_pwm_pin(RIGHT_PWM_PIN, pwm_wrap, div);

    motor_stop_all();
}

void motor_set(int16_t left, int16_t right, bool brake) {
    set_channel_pwm(LEFT_PWM_PIN,  LEFT_DIR_PIN,  left,  brake, false);
    set_channel_pwm(RIGHT_PWM_PIN, RIGHT_DIR_PIN, right, brake, RIGHT_MOTOR_INVERT);
}

void motor_stop_all(void) {
    gpio_put(LEFT_DIR_PIN,  0);
    gpio_put(RIGHT_DIR_PIN, 0);
    pwm_set_gpio_level(LEFT_PWM_PIN,  0);
    pwm_set_gpio_level(RIGHT_PWM_PIN, 0);
}