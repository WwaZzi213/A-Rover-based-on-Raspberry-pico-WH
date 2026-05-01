#pragma once
#include <stdint.h>

// 初始化 TRIG/ECHO 引脚
void ultrasonic_init(void);

// 每个控制周期调用；内部每 60ms 触发一次测量（阻塞上限约 3ms）
void ultrasonic_update(void);

// 返回最近一次测量缓存距离（cm）；-1 表示传感器无响应或尚未首次测量
int32_t ultrasonic_get_distance_cm(void);
