/**
 *******************************************************************************
 * @file      :main_task.cpp
 * @brief     : GM6020 正弦速度 / 位置跟随（电流模式，使用发放的 Gm6020/Pid 类）
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team，Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "main_task.hpp"
#include "system_user.hpp"

#include "DT7.hpp"
#include "HW_can.hpp"
#include "Gm6020.h"   // ← 发放的电机类
#include "Pid.h"      // ← 发放的 PID 类
#include "iwdg.h"
#include "math.h"
/* Private macro -------------------------------------------------------------*/
/* ======================== 用户可调参数（按硬件修改） ======================== */
#define MOTOR_ID 1              // GM6020 电机 ID = 1（拨码 Bit[2:0]=001）
#define USE_CURRENT_MODE 1      // 1=电流模式(0x1FE), 0=电压模式(0x1FF)
#define TEST_MODE 0             // 0=位置正弦 1=速度正弦 2=速度阶跃 3=位置阶跃 4=开环测试

#define SINE_FREQ_HZ 0.2f       // 正弦频率，周期 5s
#define POS_AMPLITUDE 1.5f      // 位置幅值 rad（约 ±86°）
#define SPD_AMPLITUDE 6.0f      // 速度幅值 rad/s（约 ±57rpm）

/* 电流模式控制量范围：±16384 对应 ±3A
 * ！！首次调试请先用 4000（约 0.73A）确认方向/行为正常，再放开到 16384 */
#define CURR_LIMIT 4000.0f

/* 开环测试(TEST_MODE=4)用的恒定电流，绕过 PID */
#define OPENLOOP_CURR 1500.0f

/* 速度反馈一阶低通滤波系数：越小越平滑、但相位滞后越大。
 * @1kHz 时截止频率 ≈ alpha * fs / (2π)，0.15 ≈ 24Hz。
 * 噪声大就调小(0.08)，跟得太慢就调大(0.3)。 */
#define SPD_FILT_ALPHA 0.15f

/* 速度环 PID：输入 rad/s，输出 = 电流给定(-CURR_LIMIT ~ +CURR_LIMIT) */
#define SPD_PID_KP 150.0f
#define SPD_PID_KI 400.0f
#define SPD_PID_KD 0.0f

/* 位置环 PID：输入 rad，输出 = 速度给定(rad/s) */
#define POS_PID_KP 6.0f
#define POS_PID_KI 0.0f
#define POS_PID_KD 0.02f
#define POS_SPD_LIMIT 20.0f     // 位置环输出限幅(rad/s)
/* =========================================================================== */
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint32_t tick = 0;

/* 电机与控制器对象 */
Gm6020 *g_gm6020_ptr = nullptr;  // 全局指针，供 CAN 回调里 decode() 用
static Pid *g_spd_pid = nullptr;
static Pid *g_pos_pid = nullptr;

/* 供 Ozone 观察/在线调参的变量 */
volatile float g_ref_pos = 0.0f;   // 位置给定 rad
volatile float g_act_pos = 0.0f;   // 位置反馈 rad
volatile float g_ref_spd = 0.0f;   // 速度给定 rad/s
volatile float g_act_spd = 0.0f;   // 速度反馈 rad/s
volatile float g_curr_out = 0.0f;  // 电流给定输出
volatile float g_setpoint = 0.0f;  // 阶跃调参用给定（Ozone 在线改）

volatile float g_spd_kp = SPD_PID_KP;
volatile float g_spd_ki = SPD_PID_KI;
volatile float g_spd_kd = SPD_PID_KD;
volatile float g_pos_kp = POS_PID_KP;
volatile float g_pos_ki = POS_PID_KI;
volatile float g_pos_kd = POS_PID_KD;
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

namespace remote_control = hello_world::devices::remote_control;
static const uint8_t kRxBufLen = remote_control::kRcRxDataLen;
static uint8_t rx_buf[kRxBufLen];
remote_control::DT7 *rc_ptr;

void RobotInit(void) { rc_ptr = new remote_control::DT7(); }

void MainInit(void) {
  RobotInit();

  // 开启CAN1和CAN2
  CanFilter_Init(&hcan1);
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

  CanFilter_Init(&hcan2);
  HAL_CAN_Start(&hcan2);
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);

  // 开启遥控器接收
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rx_buf, kRxBufLen);

  // 新增：创建电机对象（电流模式）与两个 PID
  g_gm6020_ptr = new Gm6020(MOTOR_ID, USE_CURRENT_MODE);
  g_spd_pid = new Pid(SPD_PID_KP, SPD_PID_KI, SPD_PID_KD, CURR_LIMIT, -CURR_LIMIT);
  g_pos_pid = new Pid(POS_PID_KP, POS_PID_KI, POS_PID_KD, POS_SPD_LIMIT,
                      -POS_SPD_LIMIT);

  // 开启定时器
  HAL_TIM_Base_Start_IT(&htim6);
}

void MainTask(void) {
  tick++;
  if (g_gm6020_ptr == nullptr) return;

  // 读取反馈（发放 Gm6020 类已换算好单位）
  g_act_pos = g_gm6020_ptr->angle();  // rad, 多圈连续
  // 速度反馈一阶低通滤波：抑制高频噪声，便于把 KP 调高
  static float spd_filt = 0.0f;
  spd_filt += SPD_FILT_ALPHA * (g_gm6020_ptr->vel() - spd_filt);
  g_act_spd = spd_filt;               // rad/s（滤波后）

  // 在线调参：把 Ozone 里改的增益写进 PID
  g_spd_pid->setParams(g_spd_kp, g_spd_ki, g_spd_kd);
  g_pos_pid->setParams(g_pos_kp, g_pos_ki, g_pos_kd);

  const float t = (float)tick * kCtrlPeriod;  // 秒
  const float w = 2.0f * 3.14159265358979f * SINE_FREQ_HZ;

  float curr = 0.0f;
#if (TEST_MODE == 0)
  /* ---------- 位置正弦跟随（位置环 -> 速度环 串级） ---------- */
  g_ref_pos = POS_AMPLITUDE * sinf(w * t);
  float spd_ff = POS_AMPLITUDE * w * cosf(w * t);  // 速度前馈 = d(ref)/dt
  float spd_ref = g_pos_pid->calc(g_ref_pos, g_act_pos) + spd_ff;
  g_ref_spd = spd_ref;
  curr = g_spd_pid->calc(spd_ref, g_act_spd);
#elif (TEST_MODE == 1)
  /* ---------- 速度正弦跟随（单速度环） ---------- */
  g_ref_spd = SPD_AMPLITUDE * sinf(w * t);
  curr = g_spd_pid->calc(g_ref_spd, g_act_spd);
#elif (TEST_MODE == 2)
  /* ---------- 速度阶跃（调参用，Ozone 里改 g_setpoint） ---------- */
  g_ref_spd = g_setpoint;
  curr = g_spd_pid->calc(g_ref_spd, g_act_spd);
#elif (TEST_MODE == 4)
  /* ---------- 开环测试：恒定电流，绕过 PID（判断符号/环路/硬件） ---------- */
  curr = OPENLOOP_CURR;
#else
  /* ---------- 位置阶跃（调参用，Ozone 里改 g_setpoint） ---------- */
  g_ref_pos = g_setpoint;
  float spd_ref = g_pos_pid->calc(g_ref_pos, g_act_pos);
  g_ref_spd = spd_ref;
  curr = g_spd_pid->calc(spd_ref, g_act_spd);
#endif
  g_curr_out = curr;

  // 输出：设电流并组帧发送（电流帧标识符 0x1FE）
  g_gm6020_ptr->setCurrent((int16_t)curr);
  uint8_t frame[8] = {0};
  g_gm6020_ptr->encode(frame);
  CAN_Send_Msg(&hcan1, frame, g_gm6020_ptr->txId(), 8);

  // 看门狗：1kHz 控制循环正常运行就喂狗
  HAL_IWDG_Refresh(&hiwdg);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

  if (htim == &htim6) {
    MainTask();
  }
}
uint8_t rx_data = 0;
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  if (huart == &huart3) {
    if (Size == remote_control::kRcRxDataLen) {
      // TODO:在这里刷新看门狗

      rc_ptr->decode(rx_buf);
    }

    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rx_buf, kRxBufLen);
  }
}
