#pragma once
#include <stdint.h>
#include <stdbool.h>

// 初始化 PWM + GPIO
void motor_init(void);

// 设置两路电机
// value: -1000..1000，正向/反向
// brake: true 时刹车（PWM=0，保持当前 DIR 信号 → 短路刹车）
void motor_set(int16_t left, int16_t right, bool brake);

// 强制停机（PWM=0，DIR=0）
void motor_stop_all(void);