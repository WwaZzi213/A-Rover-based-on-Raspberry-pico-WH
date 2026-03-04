#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef enum {
    FS_CONTROL_ACTIVE = 0,
    FS_LINK_LOST      = 1
} FailsafeState;

void     failsafe_init(void);
void     failsafe_feed(void);          // 收到有效命令时调用
FailsafeState failsafe_update(void);   // 主循环调用，返回当前状态
FailsafeState failsafe_state(void);
uint32_t failsafe_trigger_count(void);
