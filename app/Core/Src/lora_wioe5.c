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

	/* Reset WIOE5 configuration */
	WIOE5_FactoryReset(huart);

	WIOE5_SendString(huart, "ABCDEF", 6);


	/*Check for correct wiring*/
	HAL_UART_Transmit(huart, "AT\r\n", 8, 100);
	HAL_UART_Receive(huart, string, 100,1000);
	if(strcmp(string,"+AT: OK\r\n") != 0x0){
		res++;
	}

	WIOE5_ReadFirmwareVersion(firmware_version, huart);

	/*Set lora channel*/
	WIOE5_ChannelSwitch(LORA_CHANNEL_INIT,huart);
	/*Set data rate*/
	WIOE5_SetDataRate(LORA_868Mhz_FSK_50000BPS,huart);
	/*Set transmit power*/
	WIOE5_SetTxPower(LORA_868Mhz_16dBm, huart);

	/*Send 0x1234 5678*/
	WIOE5_SendData(305419896, huart);

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
	uint8_t string[100] = {0};
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
 * */
void WIOE5_ReadFirmwareVersion(uint8_t version_output[],UART_HandleTypeDef *huart){

	uint8_t string[100] = {0};
	uint8_t index = 6;

	HAL_UART_Transmit(huart, "AT+VER\r\n", sizeof("AT+VER\r\n"), 100);
	HAL_UART_Receive(huart, string, 100,1000);

	while((string[index] != '\r') || (index > sizeof(string))){
		version_output[index-5] = string[index];
		index++;
	}
}

/* Function use to set data rate
 * INPUT:
 *    @dr is an integer use to set data rate
 *	  @huart is a pointer on uart handdle
 *OUTPUT:
 * 	  @res is an integer use to check error
 * */
uint8_t WIOE5_SetDataRate(uint8_t dr,UART_HandleTypeDef *huart){

	uint8_t querry[25] = "AT+DR=";
	querry[6] =  (dr+'0');
	uint8_t string[100] = {0};
	uint8_t res = 0;

	strcat(querry, "\r\n");
	HAL_UART_Transmit(huart, querry, sizeof(querry), 100);
	HAL_UART_Receive(huart, string, 100,1000);
	if(string[0]!='+'){
		res++;
	}
	return res;
}


/* Function use to set transmit power
 * INPUT:
 *    @tx_power is an integer use to set data rate
 *	  @huart is a pointer on uart handdle
 *OUTPUT:
 * 	  @res is an integer use to check error
 * */
uint8_t WIOE5_SetTxPower(uint8_t tx_power,UART_HandleTypeDef *huart){

	uint8_t querry[25] = "AT+POWER=";
	uint8_t string[100] = {0};
	uint8_t res = 0;

	if(tx_power < 10){	/*Two digit at least*/
		querry[9] =  (tx_power+'0');
	}
	else{
		querry[9] =  ((tx_power/10)+'0');
		querry[10] =  ((tx_power%10)+'0');
	}
	strcat(querry, "\r\n");
	HAL_UART_Transmit(huart, querry, sizeof(querry), 100);
	HAL_UART_Receive(huart, string, 100,1000);
	if(string[0]!='+'){
		res++;
	}
	return res;
}
/* Function use to transmit 4 byte
 * INPUT:
 *    @data is an integer - this is the 4 byte to send
 *	  @huart is a pointer on uart handdle
 *OUTPUT:
 * 	  @res is an integer use to check error
 * */
uint8_t WIOE5_SendData(uint32_t data,UART_HandleTypeDef *huart){

	uint8_t querry[30] = "AT+MSGHEX=";
	querry[10] = '"';
	uint8_t hex_char = 0;
	uint8_t res;
	uint8_t string[100] = {0};

	/*Split 32 bits into 4 byte*/
	for(int i=0;i<8;i++){
		hex_char = (data&(0xf<<(i*4)))>>(i*4);
		if(hex_char > 10){
			hex_char = hex_char + 55;
		}
		else{
			hex_char = hex_char + 48;
		}
		querry[11+i] = hex_char;
	}
	querry[19] = '"';
	strcat(querry,"\r\n");
	HAL_UART_Transmit(huart, querry, sizeof(querry), 100);
	HAL_UART_Receive(huart, string, 100,1000);

	/*Check if receive a DONE*/
	if(strcmp("+MSGHEX: Start\r\n",string)!=0x00){
		res = 1;
	}
	return res;
}

/* Function use to reset the WIOE5 settings
 * INPUT:
 *	  @huart is a pointer on uart handdle
 *OUTPUT:
 * 	  @res is an integer use to check error
 * */
uint8_t WIOE5_FactoryReset(UART_HandleTypeDef *huart){

	uint8_t res = 0;
	uint8_t string[100] = {0};
	uint8_t querry[30] = "AT+FDEFAULT\r\n";

	HAL_UART_Transmit(huart, querry, sizeof(querry), 100);
	HAL_UART_Receive(huart, string, 100,1000);

	if(strcmp("+FDEFAULT: OK\r\n",string)!=0x00){
		res = 1;
	}

	return res;
}


/* Function use to send message through Lora
 * INPUT:
 *	  @huart is a pointer on uart handdle
 *	  @string is the actual message to be send
 *	  @string_size is an integer
 *OUTPUT:
 * 	  @res is an integer use to check error
 * */
uint8_t WIOE5_SendString(UART_HandleTypeDef *huart,uint8_t string[],uint32_t string_size){

	uint8_t res = 0;
	uint8_t received_str[100] = {0};
	uint8_t querry[10] = "AT+MSG=";
	/*Init a pointer on char as NULL*/
	char * string_to_send = NULL;

	/*Allocate some space for the string to send*/
	string_to_send = (char *) malloc( (string_size+30) * sizeof(char));

	/*Clear the string*/
	for(int i=0;i<(string_size+30);i++){
		string_to_send[i] = 0x0;
	}

	strcat(string_to_send,querry);
	strncat(string_to_send,0x34,1);

	/*Add the string to*/
	for(int i=0;i<string_size;i++){
		string_to_send[7+i] = string[i];
	}

	strncat(string_to_send,0x34,1);
	strcat(string_to_send,"\r\n");

	HAL_UART_Transmit(huart, string_to_send, (string_size+30), 100);
	HAL_UART_Receive(huart, received_str, 100,1000);

	free(string_to_send);
	return res;
}
