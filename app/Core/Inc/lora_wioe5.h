/*
 * lora_wioe5.h
 *
 *  Created on: Jan 1, 2024
 *      Author: 33768
 */

#ifndef INC_LORA_WIOE5_H_
#define INC_LORA_WIOE5_H_

#include "stm32g4xx_hal.h"


uint8_t WIOE5_Init(UART_HandleTypeDef *huart);
uint8_t WIOE5_ChannelSwitch(uint8_t channel,UART_HandleTypeDef *huart);
void WIOE5_ReadFirmwareVersion(uint8_t version_output[],UART_HandleTypeDef *huart);
uint8_t WIOE5_SetDataRate(uint8_t dr,UART_HandleTypeDef *huart);
uint8_t WIOE5_SetTxPower(uint8_t tx_power,UART_HandleTypeDef *huart);
uint8_t WIOE5_SendData(uint32_t data,UART_HandleTypeDef *huart);
/*define */
#define LORA_CHANNEL_INIT 3

#define LORA_868Mhz_SF12_250BPS 0
#define LORA_868Mhz_SF11_440BPS 1
#define LORA_868Mhz_SF10_980BPS 2
#define LORA_868Mhz_SF9_1760BPS 3
#define LORA_868Mhz_SF8_3125BPS 4
#define LORA_868Mhz_SF7_5470BPS 5
#define LORA_868Mhz_SF7_11000BPS 6
#define LORA_868Mhz_FSK_50000BPS 7

#define LORA_868Mhz_16dBm 16
#define LORA_868Mhz_14dBm 14
#define LORA_868Mhz_12dBm 12
#define LORA_868Mhz_10dBm 10
#define LORA_868Mhz_8dBm 8
#define LORA_868Mhz_6dBm 6
#define LORA_868Mhz_4dBm 4
#define LORA_868Mhz_2dBm 2

#endif /* INC_LORA_WIOE5_H_ */
