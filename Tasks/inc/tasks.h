#ifndef TASKS_H
#define TASKS_H

#include "stm32f1xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 全局 1 ms 节拍计数。
 * 名字必须叫 tick、必须全局、必须 volatile，这样 Ozone 在 Debug 下
 * 才能按符号找到它并连续监视（第 2、3 题截图都靠它）。
 */
extern volatile uint32_t tick;

/*
 * 初始化函数：由 main.c 的 USER CODE BEGIN 2 调用。
 * 放在这里而不是 main.c，是为了满足"业务代码只写在 Tasks 里"。
 */
void Tasks_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* TASKS_H */
