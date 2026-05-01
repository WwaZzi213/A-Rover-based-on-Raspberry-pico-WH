#include "auto_drive.h"

// -------------------------------------------------------
//  TODO: 在此实现自动驾驶算法
//
//  当前为占位存根（原地停止），替换 auto_drive_update 内容即可。
//  auto_drive_init 中初始化你需要的状态变量。
// -------------------------------------------------------

void auto_drive_init(void) {
    // TODO: 初始化算法状态
}

void auto_drive_update(int32_t front_dist_cm,
                       int16_t *out_left,
                       int16_t *out_right) {
    // TODO: 实现自动驾驶逻辑
    // 示例：原地停止
    (void)front_dist_cm;
    *out_left  = 0;
    *out_right = 0;
}
