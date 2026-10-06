#include "tasks.h"
#include "main.h"          /* LED_Pin / LED_GPIO_Port 由 CubeMX 在 main.h 里定义 */

/* CubeMX 在 main.c 里定义的外设句柄，本文件要用，先声明 */
extern TIM_HandleTypeDef htim2;
extern IWDG_HandleTypeDef hiwdg;

/* 全局 tick：volatile 保证每次中断都真正读写内存，Ozone 才能看到它一直涨 */
volatile uint32_t tick = 0;

void Tasks_Init(void)
{
    /* ---- 第 1 题 GPIO ----
     * PC13 输出低电平，F103 最小系统板板载灯点亮。
     * 只在这里写一次，不在 while(1) 里翻转（作业要求）。
     */
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

    /* ---- 第 2 题 定时器 ----
     * 启动 TIM2 并打开更新中断。
     * CubeMX 生成的代码默认只初始化、不启动，不加这句回调永远不会执行。
     * 第 1 题阶段可以先不加这行（此时反正也没人喂狗，现象符合作业描述）。
     */
    HAL_TIM_Base_Start_IT(&htim2);
}

/*
 * TIM2 更新中断回调：全工程只能有这一份，必须写在 Tasks 里。
 * 如果 main.c 或中断文件里也有同名函数，删掉那一份，否则链接报重复定义。
 * 用 C++ 写时函数要加 extern "C"（本文件是 .c 则不用管）。
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM2)
    {
        return;                 /* 不是 TIM2 的中断，直接返回 */
    }

    tick++;                     /* 每 1 ms 加 1（第 2、3 题共用） */

    /* ---- 第 2 题：喂狗 ----
     * 有这句：芯片不复位，Ozone 里 tick 每秒约 +1000（第 2 题截图用这版代码）。
     */
    HAL_IWDG_Refresh(&hiwdg);

    /* ---- 第 3 题：把上面那句 HAL_IWDG_Refresh 注释掉 ----
     * 没有这句：约 2 秒后看门狗复位，tick 涨到约 2000 归零、循环往复（第 3 题截图）。
     * 改完必须重新 Build 并单独下载一次，和第 2 题分开。
     */
}
