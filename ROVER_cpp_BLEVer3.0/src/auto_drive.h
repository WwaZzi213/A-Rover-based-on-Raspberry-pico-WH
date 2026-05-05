#pragma once
#include <stdint.h>

// -------------------------------------------------------
//  全自动驾驶接口
//
//  auto_drive_init()  — 固件启动时调用一次，初始化内部状态
//
//  auto_drive_update() — AUTO 模式每个控制周期（10ms）调用
//    输入：
//      front_cm  前方传感器距离 cm（-1 = 无读数）
//      left_cm   左侧传感器距离 cm（舵机 90°，-1 = 无读数/超出量程）
//      right_cm  右侧传感器距离 cm（舵机 90°，-1 = 无读数/超出量程）
//    输出：
//      *out_left / *out_right  目标电机值 -1000..+1000
//      【符号约定】负值 = 向前，正值 = 向后（与手动模式一致）
//
//  注意：
//    - slew rate 限幅（20/tick）仍在 main.cpp 中生效，输出会被平滑
//    - 失联保护在 AUTO 模式同样有效，断连时自动停车
// -------------------------------------------------------

void auto_drive_init(void);

void auto_drive_update(int32_t front_cm,
                       int32_t left_cm,
                       int32_t right_cm,
                       int16_t *out_left,
                       int16_t *out_right);
