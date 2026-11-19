#pragma once

#define TD_BT_TASK_STACK       768u
#define TD_BT_TASK_PRIORITY    1u

void bluetooth_link_init(void);
void bluetooth_link_task(void *argument);
