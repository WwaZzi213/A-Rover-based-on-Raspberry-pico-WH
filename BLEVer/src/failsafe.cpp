#include "failsafe.h"
#include "config.h"
#include "pico/time.h"
#include <stdio.h>

static volatile absolute_time_t s_last_feed;
static volatile FailsafeState   s_state        = FS_LINK_LOST;
static volatile uint32_t        s_trigger_count = 0;

void failsafe_init(void) {
    s_last_feed     = get_absolute_time();
    s_state         = FS_LINK_LOST;
    s_trigger_count = 0;
}

void failsafe_feed(void) {
    s_last_feed = get_absolute_time();
    if (s_state == FS_LINK_LOST) {
        s_state = FS_CONTROL_ACTIVE;
        printf("[FS] Link restored\n");
    }
}

FailsafeState failsafe_update(void) {
    if (s_state == FS_CONTROL_ACTIVE) {
        int64_t elapsed_us = absolute_time_diff_us(s_last_feed, get_absolute_time());
        if (elapsed_us > (int64_t)FAILSAFE_TIMEOUT_MS * 1000) {
            s_state = FS_LINK_LOST;
            s_trigger_count++;
            printf("[FS] LINK LOST! triggers=%lu\n", (unsigned long)s_trigger_count);
        }
    }
    return s_state;
}

FailsafeState failsafe_state(void)        { return s_state; }
uint32_t      failsafe_trigger_count(void){ return s_trigger_count; }