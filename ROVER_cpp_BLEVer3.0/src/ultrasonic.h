#pragma once
#include <stdint.h>
#include <stdbool.h>

#define US_FRONT  0   // 前方（固定，GPIO9/10）
#define US_LEFT   1   // 左侧（左舵机，GPIO11/12）
#define US_RIGHT  2   // 右侧（右舵机，GPIO13/16）
#define US_COUNT  3

// 初始化所有 TRIG/ECHO 引脚
void ultrasonic_init(void);

// 每个控制周期调用；内部以 6-tick（60ms）为周期交错触发三路传感器
// 每次调用最多阻塞 ~8ms（5ms 启动超时 + 2.9ms 回波超时）
void ultrasonic_update(void);

// 返回指定传感器的缓存距离（cm）；-1 = 无回波（超出量程或未响应）
int32_t ultrasonic_get_cm(uint8_t sensor_id);

// 任意传感器距离 > 0 且 < threshold_cm 时返回 true
bool ultrasonic_any_in_range(int32_t threshold_cm);
