/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan;

/* USER CODE BEGIN Private defines */
#define CAN_TX_STD_ID	0x012
#define CAN_RX_STD_ID	0x0A2
#define CAN_DATA_LENGTH	0x08

extern CAN_RxHeaderTypeDef		can_rx_header;
extern CAN_TxHeaderTypeDef		can_tx_header;
extern uint8_t 					can_rx_buffer[];
extern uint8_t					can_tx_buffer[];
extern volatile uint8_t			data_received_flag;
extern CAN_FilterTypeDef 		can_filter;
extern uint32_t					can_mailbox;
/* USER CODE END Private defines */

void MX_CAN_Init(void);

/* USER CODE BEGIN Prototypes */
uint8_t calc_CRC8_SAEJ1850(uint8_t *data, uint8_t length);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

