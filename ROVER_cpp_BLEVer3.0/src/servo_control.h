#pragma once
#include <stdint.h>

#define SERVO_LEFT   0   // GPIO14 (PWM7A)
#define SERVO_RIGHT  1   // GPIO15 (PWM7B)

// 初始化 PWM slice 7，并将两舵机置于手动模式角度（朝向正前方）
void servo_init(void);

// 设置舵机角度，degrees 范围 0..180
void servo_set_angle(uint8_t servo_id, uint8_t degrees);
