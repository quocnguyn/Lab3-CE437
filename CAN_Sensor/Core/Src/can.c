/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */
CAN_FilterTypeDef 		can_filter;
CAN_RxHeaderTypeDef		can_rx_header;
CAN_TxHeaderTypeDef		can_tx_header;
uint32_t				can_mailbox = CAN_TX_MAILBOX0;
volatile uint8_t		data_received_flag = 0;
uint8_t 				can_rx_buffer[CAN_DATA_LENGTH];
uint8_t					can_tx_buffer[CAN_DATA_LENGTH] = {
							0x00, 0x00, 0x00, 0x00,
							0x00, 0x00, 0x00, 0x00
						};

void CAN_Tx_stdHeaderConfig(
	uint32_t std_id,
	uint32_t rtr,
	uint32_t dlc
){
	can_tx_header.StdId 				= std_id;
	can_tx_header.RTR					= rtr;
	can_tx_header.DLC					= dlc;
	can_tx_header.IDE					= CAN_ID_STD;
	can_tx_header.ExtId					= 0x00;
	can_tx_header.TransmitGlobalTime	= DISABLE;
}

void CAN_Rx_stdHeaderConfig(
	uint32_t std_id,
	uint32_t rtr,
	uint32_t dlc
){
	can_rx_header.StdId 				= std_id;
	can_rx_header.RTR					= rtr;
	can_rx_header.DLC					= dlc;
	can_rx_header.IDE					= CAN_ID_STD;
	can_rx_header.ExtId					= 0x00;
	can_rx_header.Timestamp				= DISABLE;
}
/* USER CODE END 0 */

CAN_HandleTypeDef hcan;

/* CAN init function */
void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 4;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_2TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
  can_filter.FilterActivation = ENABLE;
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
  can_filter.FilterBank = 0;
  can_filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  can_filter.FilterIdHigh = (CAN_RX_STD_ID << 5) & 0xFFFF;
  can_filter.FilterIdLow = 0;
  can_filter.FilterMaskIdHigh = (0x7FF << 5) & 0xFFFF;
  can_filter.FilterMaskIdLow = 0;
  can_filter.SlaveStartFilterBank = 14;

  CAN_Tx_stdHeaderConfig(CAN_TX_STD_ID, CAN_RTR_DATA, CAN_DATA_LENGTH);
  CAN_Rx_stdHeaderConfig(CAN_RX_STD_ID, CAN_RTR_DATA, CAN_DATA_LENGTH);
  HAL_CAN_ConfigFilter(&hcan, &can_filter);
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
  /* USER CODE END CAN_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN GPIO Configuration
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN GPIO Configuration
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &can_rx_header, can_rx_buffer) != HAL_OK)
	{
		Error_Handler();
	}

	if (can_rx_header.StdId == CAN_RX_STD_ID)
	{
		HAL_GPIO_TogglePin(LED_B_GPIO_Port, LED_B_Pin);
		data_received_flag = 1;
	}
}
/* USER CODE END 1 */
