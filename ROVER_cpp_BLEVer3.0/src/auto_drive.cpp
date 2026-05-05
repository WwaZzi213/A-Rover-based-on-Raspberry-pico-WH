#include "auto_drive.h"
#include "config.h"
#include <stdbool.h>

typedef enum { AUTO_FORWARD, AUTO_TURNING } AutoState;

static AutoState s_state;
static int32_t   s_turn_ticks_left;
static bool      s_turn_right;   // true = 右转，false = 左转

void auto_drive_init(void) {
    s_state           = AUTO_FORWARD;
    s_turn_ticks_left = 0;
    s_turn_right      = false;
}

// 自动驾驶状态机
//
//  AUTO_FORWARD：以 AUTO_DRIVE_SPEED 直行，监测前方障碍
//    → 前方 < AUTO_OBSTACLE_CM：比较两侧距离，决定转向方向，切换 AUTO_TURNING
//
//  AUTO_TURNING：原地坦克转 AUTO_TURN_TICKS 个 tick（每 tick=10ms）
//    → 计时结束后回到 AUTO_FORWARD 继续直行
//
//  转向逻辑：
//    右侧距离比左侧多 > AUTO_TURN_SIDE_TOLERANCE_CM → 右转（右侧更空旷）
//    否则 → 左转（默认，包括两侧相等或都无读数的情况）

void auto_drive_update(int32_t front_cm,
                       int32_t left_cm,
                       int32_t right_cm,
                       int16_t *out_left,
                       int16_t *out_right) {
    if (s_state == AUTO_FORWARD) {
        if (front_cm > 0 && front_cm < AUTO_OBSTACLE_CM) {
            // -1（无读数）视为距离极远（200cm），不阻挡通行
            int32_t left_eff  = (left_cm  > 0) ? left_cm  : 200;
            int32_t right_eff = (right_cm > 0) ? right_cm : 200;
            s_turn_right      = (right_eff > left_eff + AUTO_TURN_SIDE_TOLERANCE_CM);
            s_turn_ticks_left = AUTO_TURN_TICKS;
            s_state           = AUTO_TURNING;
        }
    }

    if (s_state == AUTO_TURNING) {
        if (s_turn_ticks_left > 0) {
            s_turn_ticks_left--;
            if (s_turn_right) {
                // 右转：左轮前进，右轮后退
                *out_left  = -(int16_t)AUTO_TURN_SPEED;
                *out_right = +(int16_t)AUTO_TURN_SPEED;
            } else {
                // 左转：左轮后退，右轮前进
                *out_left  = +(int16_t)AUTO_TURN_SPEED;
                *out_right = -(int16_t)AUTO_TURN_SPEED;
            }
            return;
        }
        s_state = AUTO_FORWARD;
    }

    // AUTO_FORWARD：直行
    *out_left  = -(int16_t)AUTO_DRIVE_SPEED;
    *out_right = -(int16_t)AUTO_DRIVE_SPEED;
}
