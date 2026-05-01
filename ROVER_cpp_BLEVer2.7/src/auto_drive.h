#pragma once
#include <stdint.h>

// -------------------------------------------------------
//  全自动驾驶接口
//
//  【给另一位开发者的说明】
//  在 auto_drive.cpp 里实现下面两个函数：
//
//  1. auto_drive_init()
//     - 在固件启动时被调用一次
//     - 初始化你的算法所需的状态、变量等
//
//  2. auto_drive_update()
//     - 在 AUTO 模式下每个控制周期（10ms）被调用
//     - 输入：front_dist_cm  前方超声波距离（cm），-1 表示无读数
//     - 输出：*out_left / *out_right  目标电机值，范围 -1000..+1000
//             【符号约定】负值 = 向前，正值 = 向后（与手动模式一致）
//
//  注意：
//  - 固件的 slew rate 限幅仍然生效（最大 20 单位/tick），输出会被平滑
//  - 如需访问其他传感器，在此头文件增加参数即可
//  - 失联保护（failsafe）在 AUTO 模式下同样有效，断连时自动停车
// -------------------------------------------------------

void auto_drive_init(void);

void auto_drive_update(int32_t front_dist_cm,
                       int16_t *out_left,
                       int16_t *out_right);
