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
uint8_t 				can_rx_buffer[CAN_DATA_LENGTH];
volatile uint8_t		data_received_flag = 0;
uint32_t				can_mailbox = CAN_TX_MAILBOX0;
uint8_t					can_tx_buffer[CAN_DATA_LENGTH] = {
							0x01, 0x02, 0x00, 0x00,
							0x00, 0x00, 0x00, 0x00
						};

void CAN_Tx_stdHeaderConfig(
	uint32_t std_id,
	uint32_t rtr,
	uint32_t dlc
){
	can_tx_header.StdId 				= std_id;
	can_tx_header.RTR					= rtr;
	can_tx_header.DLC					= CAN_DATA_LENGTH;
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
	can_rx_header.DLC					= CAN_DATA_LENGTH;
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
  can_filter.FilterBank = 0;
  can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
  can_filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  can_filter.FilterIdHigh = CAN_RX_STD_ID << 5;
  can_filter.FilterIdLow = 0;
  can_filter.FilterMaskIdHigh = 0xFFF << 5;
  can_filter.FilterMaskIdLow = 0;
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter.FilterActivation = ENABLE;
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

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN GPIO Configuration
    PB8     ------> CAN_RX
    PB9     ------> CAN_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    __HAL_AFIO_REMAP_CAN1_2();

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
    PB8     ------> CAN_RX
    PB9     ------> CAN_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8|GPIO_PIN_9);

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
	uint8_t calculated_crc = calc_CRC8_SAEJ1850(can_rx_buffer, CAN_DATA_LENGTH - 1);
	uint8_t received_crc   = can_rx_buffer[CAN_DATA_LENGTH - 1];

	if (can_rx_header.StdId == CAN_RX_STD_ID && !(calculated_crc ^ received_crc))
	{
		HAL_GPIO_TogglePin(LED_B_GPIO_Port, LED_B_Pin);
		data_received_flag = 1;
	}
}

uint8_t calc_CRC8_SAEJ1850(uint8_t *data, uint8_t length) {
    uint8_t crc = INITIAL_VAL;

    for (size_t byte = 0; byte < length; byte++) {
        crc ^= data[byte];
        for (size_t bit = 0; bit < 8; bit++) {
			crc = (crc << 1) ^ (crc & 0x80 ? CRC8_SAEJ1850_POLY : 0);
        }
    }
    return (crc ^ FINAL_XOR_VAL) & 0xFF;
}
/* USER CODE END 1 */
