/*
 * lora_wioe5.c
 *
 *  Created on: Jan 1, 2024
 *      Author: 33768
 */

#include "lora_wioe5.h"

/*Initialisation function for the Lora module
 * INPUT:
 *    @huart is a pointer on uart handdle
 *
 * OUTPUT:
 * 	  @res is an integer use to count the number of error reported during initialisation
 * */
uint8_t WIOE5_Init(UART_HandleTypeDef *huart){

	uint8_t res = 0;
	uint8_t string[100] = {0};
	uint8_t firmware_version[10] = {0};

	/*Check for correct wiring*/
	HAL_UART_Transmit(huart, "AT\r\n", 8, 100);
	HAL_UART_Receive(huart, string, 100,1000);
	if(strcmp(string,"+AT: OK\r\n") != 0x0){
		res++;
	}

	WIOE5_ReadFirmwareVersion(firmware_version, huart);


	WIOE5_ChannelSwitch(LORA_CHANNEL_INIT,huart);

	return res;
}


/*Channel switch function use to modifiy carring frequency
 * INPUT:
 *    @channel is an integer that define the channel number (Should be between 0 and 2 for EU868 Mhz version
 *	  @huart is a pointer on uart handdle
 * OUTPUT:
 * 	  @res is an integer use to check error
 * */
uint8_t WIOE5_ChannelSwitch(uint8_t channel,UART_HandleTypeDef *huart){

	uint8_t querry[25] = "AT+CH=";
	querry[6] =  (channel+'0');
	uint8_t string[100];
	uint8_t res = 0;

	strcat(querry, "\r\n");
	HAL_UART_Transmit(huart, querry, sizeof(querry), 100);
	HAL_UART_Receive(huart, string, 100,1000);
	if(string[0]!='+'){
		res++;
	}
	return res;
}

/* Function use to read WIOE5 firmware version
 * INPUT:
 *    @version_output is an array of
 *	  @huart is a pointer on uart handdle
 * OUTPUT:
 * 	  @res is an integer use to check error
 * */
void WIOE5_ReadFirmwareVersion(uint8_t version_output[],UART_HandleTypeDef *huart){

	uint8_t string[100];
	uint8_t index = 6;

	HAL_UART_Transmit(huart, "AT+VER\r\n", sizeof("AT+VER\r\n"), 100);
	HAL_UART_Receive(huart, string, 100,1000);

	while((string[index] != '\r') || (index > sizeof(string))){
		version_output[index-5] = string[index];
		index++;
	}
}


