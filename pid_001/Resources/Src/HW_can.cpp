/**
 *******************************************************************************
 * @file      :HW_can.cpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2023 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "HW_can.hpp"
#include "stdint.h"

#include "dm4310_drv.hpp"
#include "Gm6020.h"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static CAN_RxHeaderTypeDef rx_header1, rx_header2;
static uint8_t can1_rx_data[8], can2_rx_data[8];
uint32_t pTxMailbox;

extern Gm6020 *g_gm6020_ptr;

/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/**
 * @brief
 * @param        *hcan:
 * @retval       None
 * @note        None
 */
void CanFilter_Init(CAN_HandleTypeDef *hcan) {
  CAN_FilterTypeDef canfilter;

  canfilter.FilterMode = CAN_FILTERMODE_IDLIST;
  canfilter.FilterScale = CAN_FILTERSCALE_16BIT;

  canfilter.FilterActivation = ENABLE;
  canfilter.SlaveStartFilterBank = 14;
  if (hcan == &hcan1) {
    canfilter.FilterFIFOAssignment = CAN_FilterFIFO0;

    canfilter.FilterMode = CAN_FILTERMODE_IDMASK;
    canfilter.FilterScale = CAN_FILTERSCALE_32BIT;
    canfilter.FilterIdHigh = 0x0000;
    canfilter.FilterIdLow = 0x0000;
    canfilter.FilterMaskIdHigh = 0x0000;
    canfilter.FilterMaskIdLow = 0x0000;
    canfilter.FilterBank = 0;
    canfilter.FilterActivation = ENABLE;
    if (HAL_CAN_ConfigFilter(hcan, &canfilter) != HAL_OK) {
      Error_Handler();
    }
  } else if (hcan == &hcan2) {
    canfilter.FilterFIFOAssignment = CAN_FilterFIFO1;
    canfilter.FilterMode = CAN_FILTERMODE_IDMASK;
    canfilter.FilterScale = CAN_FILTERSCALE_32BIT;
    canfilter.FilterIdHigh = 0x0000;
    canfilter.FilterIdLow = 0x0000;
    canfilter.FilterMaskIdHigh = 0x0000;
    canfilter.FilterMaskIdLow = 0x0000;
    canfilter.FilterActivation = ENABLE;
    canfilter.FilterBank = 14;

    if (HAL_CAN_ConfigFilter(hcan, &canfilter) != HAL_OK) {
      Error_Handler();
    }
  }
}

uint32_t can_rec_times = 0;
uint32_t can_success_times = 0;
uint32_t can_receive_data = 0;

/**
 * @brief   CAN中断的回调函数，全部数据解析都在该函数中
 * @param   hcan为CAN句柄
 * @retval  none
 * @note
 **/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  if (hcan == &hcan1) {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header1, can1_rx_data) ==
        HAL_OK) // 获得接收到的数据头和数据
    {
      if (rx_header1.StdId == 0x200) { // 帧头校验
                                       // 校验通过进行具体数据处理
      }
      //把匹配到的反馈帧交给 Gm6020 类解析 ↓
      if (g_gm6020_ptr != nullptr && rx_header1.StdId == g_gm6020_ptr->rxId()) {
        g_gm6020_ptr->decode(can1_rx_data);
      }
    }
  }
  HAL_CAN_ActivateNotification(
      hcan, CAN_IT_RX_FIFO0_MSG_PENDING); // 再次使能FIFO0接收中断
}

/**
 * @brief   CAN中断的回调函数，全部数据解析都在该函数中
 * @param   hcan为CAN句柄
 * @retval  none
 * @note
 **/
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {

  if (hcan == &hcan2) {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header2, can2_rx_data) ==
        HAL_OK) // 获得接收到的数据头和数据
    {
      if (rx_header2.StdId == 0x200) { // 帧头校验
        // 校验通过进行具体数据处理
      }
    }
  }
  HAL_CAN_ActivateNotification(
      hcan, CAN_IT_RX_FIFO1_MSG_PENDING); // 再次使能FIFO0接收中断
}

/**
 * @brief   向can总线发送数据，抄官方的
 * @param   hcan为CAN句柄
 * @param	msg为发送数组首地址
 * @param	id为发送报文
 * @param	len为发送数据长度（字节数）
 * @retval  none
 * @note    主控发送都是len=8字节，再加上帧间隔3位，理论上can总线1ms最多传输9帧
 **/
void CAN_Send_Msg(CAN_HandleTypeDef *hcan, uint8_t *msg, uint32_t id,
                  uint8_t len) {
  CAN_TxHeaderTypeDef TxMessageHeader = {0};
  TxMessageHeader.StdId = id;
  TxMessageHeader.IDE = CAN_ID_STD;
  TxMessageHeader.RTR = CAN_RTR_DATA;
  TxMessageHeader.DLC = len;
  if (HAL_CAN_AddTxMessage(hcan, &TxMessageHeader, msg, &pTxMailbox) !=
      HAL_OK) {
  }
}
