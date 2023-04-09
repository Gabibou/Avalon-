/*
 * at25x041b.c
 *
 *  Created on: Feb 19, 2023
 *      Author: Bertet Gabriel
 */
#include "at25x041b.h"
#include "main.h"



uint8_t AT25X041B_Init(SPI_HandleTypeDef *SPI,AT25X041B_t *AT25X041B,uint32_t CS_PORT,uint32_t CS_PIN){
	/*@Function use to init the memory and check if the read and write is working
	 * by looking into the device id an manufacturer
	 */

	uint8_t txBuffer[1] = {0};
	uint8_t rxBuffer[2] = {0};
	uint8_t result = 1;

	//store the port and pin number into the struct
	AT25X041B->CS_PIN = CS_PIN;
	AT25X041B->CS_PORT = CS_PORT;


	//set the cs to a high level to make sure the communication didn't start before
	HAL_GPIO_WritePin(AT25X041B->CS_PORT, AT25X041B->CS_PIN, GPIO_PIN_SET);
	HAL_Delay(5);
	//Pull low the GPIO to start communication
	HAL_GPIO_WritePin(AT25X041B->CS_PORT, AT25X041B->CS_PIN, GPIO_PIN_RESET);

	txBuffer[0] = RESUME_DEEP_POWER_DOWN_CMD;
	HAL_SPI_Transmit(SPI,txBuffer, 1, 100);

	HAL_GPIO_WritePin(AT25X041B->CS_PORT, AT25X041B->CS_PIN, GPIO_PIN_SET);
	HAL_Delay(5);
	HAL_GPIO_WritePin(AT25X041B->CS_PORT, AT25X041B->CS_PIN, GPIO_PIN_RESET);

	txBuffer[0] = READ_ID_CMD;
	HAL_SPI_Transmit(SPI, txBuffer, 1, 100);
	HAL_SPI_Receive(SPI, rxBuffer, 2, 100);

	HAL_GPIO_WritePin(AT25X041B->CS_PORT, AT25X041B->CS_PIN, GPIO_PIN_SET);

	AT25X041B->LastWrittenAddr = 0x07FFFF;
	AT25X041B->ManufacturerId = rxBuffer[0];
	if(AT25X041B->ManufacturerId !=0x1F){
		result = 0;
	}
	return result;
}

void AT25X041B_Sleep(SPI_HandleTypeDef *SPI,AT25X041B_t *AT25X041B){
	/*
	 * Function use to put the memory chip into deep power down mode
	 */
	uint8_t data;

	HAL_GPIO_WritePin(AT25X041B->CS_PORT, AT25X041B->CS_PIN, GPIO_PIN_RESET);

	data = DEEP_POWER_DOWN_CMD;
	HAL_SPI_Transmit(SPI, &data, 1, 100);

	AT25X041B->SleepMode = 0x01;

	HAL_Delay(5);
	HAL_GPIO_WritePin(AT25X041B->CS_PORT, AT25X041B->CS_PIN, GPIO_PIN_SET);
}

void AT25X041B_WakeUp(SPI_HandleTypeDef *SPI,AT25X041B_t *AT25X041B){
	/*
	 * Function use to put the memory chip into wake up mode
	 */
	uint8_t data;

	HAL_GPIO_WritePin(AT25X041B->CS_PORT, AT25X041B->CS_PIN, GPIO_PIN_RESET);

	data = RESUME_DEEP_POWER_DOWN_CMD;
	HAL_SPI_Transmit(SPI, &data, 1, 100);

	AT25X041B->SleepMode = 0x00;

	HAL_Delay(5);
	HAL_GPIO_WritePin(AT25X041B->CS_PORT, AT25X041B->CS_PIN, GPIO_PIN_SET);
}

uint8_t AT25X041B_ChipErase(SPI_HandleTypeDef *SPI,AT25X041B_t *AT25X041B){
	/*
	 * A simple function that erase the full 4Mb of the chip return 1 if the chip erase worked 0 if not
	 */
	uint8_t data[1];
	uint8_t status_reg_val;
	uint8_t result = 1;
	uint32_t SECTOR_ADDR_LIST[11] = {0x07FFFF,0x07BFFF,0x079FFF,0x076FFF,0x060FFF,0x050FFF,0x040FFF,0x030FFF,0x020FFF,0x010FFF,0x000FFF};

	//All sector should be unprotect first to ensure a full erase
	for(int i=0;i<sizeof(SECTOR_ADDR_LIST)/4;i++){
		AT25X041B_UnprotectSector(SPI, AT25X041B, SECTOR_ADDR_LIST[i]);
	}
	//We also need to disable write protection (don't know why but the datasheet say to do it)
	AT25X041B_WriteProtectionDisable(SPI, AT25X041B);

	HAL_GPIO_WritePin(AT25X041B->CS_PORT, AT25X041B->CS_PIN, GPIO_PIN_RESET);

	//Send the full chip erase command
	data[0] = FULL_CHIP_ERASE_CMD;
	HAL_SPI_Transmit(SPI, data, 1, 100);

	HAL_GPIO_WritePin(AT25X041B->CS_PORT, AT25X041B->CS_PIN, GPIO_PIN_SET);

	status_reg_val = AT25X041B_ReadStatusReg_Byte1(SPI,AT25X041B);

	//wait until the chip erase is done (check RDY/ bits)
	while(status_reg_val & 0x01 != 0x00){
		status_reg_val = AT25X041B_ReadStatusReg_Byte1(SPI,AT25X041B);
	}


	//All sector should be protected again
	for(int i=0;i<sizeof(SECTOR_ADDR_LIST)/4;i++){
		AT25X041B_ProtectSector(SPI, AT25X041B, SECTOR_ADDR_LIST[i]);
	}

	//chek the EPE bit from the status register to ensure no error
	if(((status_reg_val & 0x20)>>5) == 0x01){
		//If an error occure --> return 0
		result = 0;
	}
	else{
		//if the chip erase worked then the memory is clear and the las written adress is 0 (0x7FFFF as i read the flash from max addr downto 0
		AT25X041B->LastWrittenAddr = 0x07FFFF;
	}
	return result;

}

uint8_t AT25X041B_ReadStatusReg_Byte1(SPI_HandleTypeDef *SPI,AT25X041B_t *AT25X041B){
	/*
	 * Function use to read only the first byte of Status register
	 * The second byte of the status reg isn't usefull at all for us so we don't read it
	 */

	uint8_t data[1];
	uint8_t status_reg_value[1];


	HAL_GPIO_WritePin(AT25X041B->CS_PORT, AT25X041B->CS_PIN, GPIO_PIN_RESET);

	data[0] = READ_STATUS_CMD;
	HAL_SPI_Transmit(SPI, data, 1, 100);
	HAL_SPI_Receive(SPI, status_reg_value, 1, 100);

	HAL_GPIO_WritePin(AT25X041B->CS_PORT, AT25X041B->CS_PIN, GPIO_PIN_SET);

	return status_reg_value[0];
}

uint8_t AT25X041B_ReadMemory8(SPI_HandleTypeDef *SPI,AT25X041B_t *AT25X041B,uint32_t addr){
	uint8_t data[1];
	uint8_t memory_value[1];
	uint8_t addr8bit[3];

	//Split the 23 bits addr into 3*8bits
	addr8bit[2] = (addr & 0xFF);
	addr8bit[1] = (uint8_t)((addr & 0xFF00)>>8);
	addr8bit[0] = (uint8_t)((addr & 0xFF0000)>>16);

	//Before reading the sector should be first unprotected
	AT25X041B_UnprotectSector(SPI, AT25X041B, addr);


	HAL_GPIO_WritePin(AT25X041B->CS_PORT, AT25X041B->CS_PIN, GPIO_PIN_RESET);

	//Write command
	data[0] = READ_BYTE_CMD;

	HAL_SPI_Transmit(SPI, data, 1, 100);
	//write the 23bit addr
	HAL_SPI_Transmit(SPI, addr8bit, 3, 100);
	//dummy byte
	data[0] = 0xFF;
	HAL_SPI_Transmit(SPI, data, 1, 100);

	//Read the value store in memory
	HAL_SPI_Receive(SPI, memory_value, 1, 100);

	HAL_GPIO_WritePin(AT25X041B->CS_PORT, AT25X041B->CS_PIN, GPIO_PIN_SET);

	//When everything is done the sector should be protected again
	AT25X041B_ProtectSector(SPI, AT25X041B, addr);

	return memory_value[0];
}

void AT25X041B_WriteMemory8(SPI_HandleTypeDef *SPI,AT25X041B_t *AT25X041B,uint32_t addr,uint8_t data_in){

	uint8_t addr8bit[3];
	uint8_t data[1];
	uint8_t data_in_array[1];

	//Split the 23 bits addr into 3*8bits
	addr8bit[2] = (addr & 0xFF);
	addr8bit[1] = (uint8_t)((addr & 0xFF00)>>8);
	addr8bit[0] = (uint8_t)((addr & 0xFF0000)>>16);
	data_in_array[0] = data_in;

	//Before reading the sector should be first unprotected
	AT25X041B_UnprotectSector(SPI, AT25X041B, addr);

	//We should send a write protection disable
	AT25X041B_WriteProtectionDisable(SPI, AT25X041B);


	HAL_GPIO_WritePin(AT25X041B->CS_PORT, AT25X041B->CS_PIN, GPIO_PIN_RESET);

	data[0] = WRITE_BYTE_CMD;
	HAL_SPI_Transmit(SPI, data, 1, 100);
	//write the 23bit addr
	HAL_SPI_Transmit(SPI, addr8bit, 3, 100);
	//send the data to write
	HAL_SPI_Transmit(SPI,data_in_array,1,100);

	HAL_GPIO_WritePin(AT25X041B->CS_PORT, AT25X041B->CS_PIN, GPIO_PIN_SET);

	AT25X041B->LastWrittenAddr--;

	//When everything is done the sector should be protected again
	AT25X041B_ProtectSector(SPI, AT25X041B, addr);


}

uint8_t AT25X041B_ReadProtectionReg(SPI_HandleTypeDef *SPI,AT25X041B_t *AT25X041B,uint32_t addr){

	uint8_t addr8bit[3];
	uint8_t data[1];
	uint8_t sector_status[1];

	//Convert the 4bytes into 3 splited bytes
	addr8bit[2] = (addr & 0xFF);
	addr8bit[1] = (uint8_t)((addr & 0xFF00)>>8);
	addr8bit[0] = (uint8_t)((addr & 0xFF0000)>>16);

	data[0] = READ_SECTOR_PROTECT_CMD;
	HAL_GPIO_WritePin(AT25X041B->CS_PORT, AT25X041B->CS_PIN, GPIO_PIN_RESET);
	//send the command
	HAL_SPI_Transmit(SPI, data, 1, 100);
	//send the addr
	HAL_SPI_Transmit(SPI, addr8bit, 3, 100);
	//receive the value
	HAL_SPI_Receive(SPI, sector_status, 1, 100);
	HAL_GPIO_WritePin(AT25X041B->CS_PORT, AT25X041B->CS_PIN, GPIO_PIN_SET);

	return sector_status[0];
}

void AT25X041B_UnprotectSector(SPI_HandleTypeDef *SPI,AT25X041B_t *AT25X041B,uint32_t addr){
	uint8_t addr8bit[3];
	uint8_t data[1];
	uint8_t write_protect_safety;

	//Convert the 4bytes into 3 splited bytes
	addr8bit[2] = (addr & 0xFF);
	addr8bit[1] = (uint8_t)((addr & 0xFF00)>>8);
	addr8bit[0] = (uint8_t)((addr & 0xFF0000)>>16);

	//This should be send before each command --> if it didn't work then the command is send again
	write_protect_safety = AT25X041B_WriteProtectionDisable(SPI,AT25X041B);
	if(write_protect_safety != 0x01){
		AT25X041B_WriteProtectionDisable(SPI,AT25X041B);
	}


	data[0] = UNPROTECT_SECTOR_CMD;
	HAL_GPIO_WritePin(AT25X041B->CS_PORT, AT25X041B->CS_PIN, GPIO_PIN_RESET);
	//send the command
	HAL_SPI_Transmit(SPI, data, 1, 100);
	//send the addr
	HAL_SPI_Transmit(SPI, addr8bit, 3, 100);

	HAL_GPIO_WritePin(AT25X041B->CS_PORT, AT25X041B->CS_PIN, GPIO_PIN_SET);

}

void AT25X041B_ProtectSector(SPI_HandleTypeDef *SPI,AT25X041B_t *AT25X041B,uint32_t addr){
	uint8_t addr8bit[3];
	uint8_t data[1];
	uint8_t write_protect_safety;

	//Convert the 4bytes into 3 splited bytes
	addr8bit[2] = (addr & 0xFF);
	addr8bit[1] = (uint8_t)((addr & 0xFF00)>>8);
	addr8bit[0] = (uint8_t)((addr & 0xFF0000)>>16);

	//This should be send before each command --> if it didn't work then the command is send again
	write_protect_safety = AT25X041B_WriteProtectionDisable(SPI,AT25X041B);
	if(write_protect_safety != 0x01){
		AT25X041B_WriteProtectionDisable(SPI,AT25X041B);
	}


	data[0] = PROTECT_SECTOR_CMD;
	HAL_GPIO_WritePin(AT25X041B->CS_PORT, AT25X041B->CS_PIN, GPIO_PIN_RESET);
	//send the command
	HAL_SPI_Transmit(SPI, data, 1, 100);
	//send the addr
	HAL_SPI_Transmit(SPI, addr8bit, 3, 100);

	HAL_GPIO_WritePin(AT25X041B->CS_PORT, AT25X041B->CS_PIN, GPIO_PIN_SET);
}

uint8_t AT25X041B_WriteProtectionEnable(SPI_HandleTypeDef *SPI,AT25X041B_t *AT25X041B){
	uint8_t data[1];
	uint8_t status_reg_val;
	uint8_t res = 1;

	HAL_GPIO_WritePin(AT25X041B->CS_PORT, AT25X041B->CS_PIN, GPIO_PIN_RESET);

	data[0] = WRITE_ENABLE_CMD;
	HAL_SPI_Transmit(SPI, data, 1, 100);

	HAL_GPIO_WritePin(AT25X041B->CS_PORT, AT25X041B->CS_PIN, GPIO_PIN_SET);

	status_reg_val = AT25X041B_ReadStatusReg_Byte1(SPI, AT25X041B);

	//if the command didn't worked then return 0
	if(((status_reg_val&0x02)>>1) == 0x01){
		res = 0;
	}
	return res;
}

uint8_t AT25X041B_WriteProtectionDisable(SPI_HandleTypeDef *SPI,AT25X041B_t *AT25X041B){
	uint8_t data[1];
	uint8_t status_reg_val;
	uint8_t res = 1;

	HAL_GPIO_WritePin(AT25X041B->CS_PORT, AT25X041B->CS_PIN, GPIO_PIN_RESET);

	data[0] = WRITE_DISABLE_CMD;
	HAL_SPI_Transmit(SPI, data, 1, 100);

	HAL_GPIO_WritePin(AT25X041B->CS_PORT, AT25X041B->CS_PIN, GPIO_PIN_SET);

	status_reg_val = AT25X041B_ReadStatusReg_Byte1(SPI, AT25X041B);

	//if the command didn't worked then return 0
	if(((status_reg_val&0x02)>>1) == 0x00){
		res = 0;
	}
	return res;
}

void WriteCoordinateFlash(SPI_HandleTypeDef *SPI,AT25X041B_t *AT25X041B,coord_t coordinate){

	//Create union in order to convert float into a 4bytes array
	//As union only have one value at the same time , you can convert like this
	union {
		float altitude;
		uint8_t altitude_byte[4];
	} altitude_u;

	union {
		float latitude;
		uint8_t latitude_byte[4];
	} latitude_u;

	union {
		float longitude;
		uint8_t longitude_byte[4];
	} longitude_u;

	//Affect the coordinate to the union
	longitude_u.longitude = coordinate.longitude;
	latitude_u.latitude = coordinate.latitude;
	altitude_u.altitude = coordinate.altitude;

	//Store all data's into the memory
	for(int i=0;i<sizeof(altitude_u.altitude_byte);i++){
		AT25X041B_WriteMemory8(SPI, AT25X041B, AT25X041B->LastWrittenAddr, altitude_u.altitude_byte[i]);
	}

	for(int i=0;i<sizeof(latitude_u.latitude_byte);i++){
		AT25X041B_WriteMemory8(SPI, AT25X041B, AT25X041B->LastWrittenAddr,latitude_u.latitude_byte[i]);
	}

	for(int i=0;i<sizeof(longitude_u.longitude_byte);i++){
		AT25X041B_WriteMemory8(SPI, AT25X041B, AT25X041B->LastWrittenAddr, longitude_u.longitude_byte[i]);
	}

}

coord_t ReadLastCoordinateFlash(SPI_HandleTypeDef *SPI,AT25X041B_t *AT25X041B){

	coord_t output;
	uint8_t altitude_bytes[4];
	uint8_t latitude_bytes[4];
	uint8_t longitude_bytes[4];

	union {
		float altitude;
		uint8_t altitude_byte[4];
	} altitude_u;

	union {
		float latitude;
		uint8_t latitude_byte[4];
	} latitude_u;

	union {
		float longitude;
		uint8_t longitude_byte[4];
	} longitude_u;


	//Read the memory using SPI
	for(int i=0;i<4;i++){
		longitude_bytes[i] = AT25X041B_ReadMemory8(SPI, AT25X041B, (AT25X041B->LastWrittenAddr+AT25X041B->AddrOffset+1));
		AT25X041B->AddrOffset++;
	}
	for(int i=0;i<4;i++){
		latitude_bytes[i] = AT25X041B_ReadMemory8(SPI, AT25X041B, (AT25X041B->LastWrittenAddr+AT25X041B->AddrOffset+1));
		AT25X041B->AddrOffset++;
	}
	for(int i=0;i<4;i++){
		altitude_bytes[i] = AT25X041B_ReadMemory8(SPI, AT25X041B, (AT25X041B->LastWrittenAddr+AT25X041B->AddrOffset+1));
		AT25X041B->AddrOffset++;
	}

	//As a union can't have pointer in it we have to share data using for method
	for(int i=0;i<4;i++){
		longitude_u.longitude_byte[i] = longitude_bytes[3-i];
	}
	for(int i=0;i<4;i++){
		latitude_u.latitude_byte[i] = latitude_bytes[3-i];
	}
	for(int i=0;i<4;i++){
		altitude_u.altitude_byte[i] = altitude_bytes[3-i];
	}

	//storing data into a coord_t type
	output.altitude = altitude_u.altitude;
	output.latitude = latitude_u.latitude;
	output.longitude = longitude_u.longitude;

	return output;

}


