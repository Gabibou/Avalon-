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

/*define */
#define LORA_CHANNEL_INIT 3


#endif /* INC_LORA_WIOE5_H_ */
