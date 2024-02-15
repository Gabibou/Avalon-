/*
 * AT25X041B.h
 *
 *  Created on: Feb 19, 2023
 *      Author: Bertet Gabriel
 */

#ifndef INC_AT25X041B_H_
#define INC_AT25X041B_H_

#include "stm32g4xx_hal.h"

#define READ_ID_CMD 0x9F
#define READ_STATUS_CMD 0x05
#define RESET_CMD 0xF0
#define WRITE_STATUS_REG1_CMD 0x01
#define WRITE_STATUS_REG2_CMD 0x31
#define WRITE_ENABLE_CMD 0x04
#define WRITE_DISABLE_CMD 0x06
#define READ_BYTE_CMD 0x0B 		//Can also be 0x03 for lower frequency
#define WRITE_BYTE_CMD 0x02
#define PAGE_ERASE_CMD 0x81
#define BLOCK_ERASE_4K_CMD 0x20
#define BLOCK_ERASE_32K_CMD 0x52
#define BLOCK_ERASE_64K_CMD 0xD8
#define FULL_CHIP_ERASE_CMD 0xC7
#define RESUME_DEEP_POWER_DOWN_CMD 0xAB
#define DEEP_POWER_DOWN_CMD 0xB9
#define READ_SECTOR_PROTECT_CMD 0x3C
#define PROTECT_SECTOR_CMD 0x36
#define UNPROTECT_SECTOR_CMD 0x39




typedef struct{
	float latitude;
	float longitude;
	float altitude;

}coord_t;


typedef struct{

	//@Adress goes from 0x 00 00 00 to 0x 07 FF FF

	//Chip information
	uint8_t ManufacturerId;

	/*A sequence of coordinate is coded into 10byte
	 * @4byte for Longitude
	 * @4byte for latitude
	 * @2byte for Altitude
	 */
	uint8_t Rawcoordinate[10];

	//Store the last coordinate readed from the memory
	float latitude;
	float logitude;
	float altitude;

	//If the memory is not full,we should read only from this @ to the
	uint32_t LastWrittenAddr;
	//Usefull when we need to read the max_addr - some offset
	uint32_t AddrOffset;

	//Sleep mode is update every time we enter a sleep or a wake up function
	//2 mean ultra deep power down , 1 mean sleep mode and 0 mean wake up
	uint8_t SleepMode;

	//Store the CS pin into the struct (make the use much easier)
	uint32_t CS_PORT;
	uint32_t CS_PIN;

}AT25X041B_t;




uint8_t AT25X041B_Init(SPI_HandleTypeDef *SPI,AT25X041B_t *AT25X041B,uint32_t CS_PORT,uint32_t CS_PIN);
void AT25X041B_Sleep(SPI_HandleTypeDef *SPI,AT25X041B_t *AT25X041B);
void AT25X041B_WakeUp(SPI_HandleTypeDef *SPI,AT25X041B_t *AT25X041B);
uint8_t AT25X041B_ChipErase(SPI_HandleTypeDef *SPI,AT25X041B_t *AT25X041B);
uint8_t AT25X041B_ReadStatusReg_Byte1(SPI_HandleTypeDef *SPI,AT25X041B_t *AT25X041B);
void AT25X041B_WriteMemory8(SPI_HandleTypeDef *SPI,AT25X041B_t *AT25X041B,uint32_t addr,uint8_t data_in);
uint8_t AT25X041B_WriteProtectionEnable(SPI_HandleTypeDef *SPI,AT25X041B_t *AT25X041B);
uint8_t AT25X041B_WriteProtectionDisable(SPI_HandleTypeDef *SPI,AT25X041B_t *AT25X041B);
uint8_t AT25X041B_ReadProtectionReg(SPI_HandleTypeDef *SPI,AT25X041B_t *AT25X041B,uint32_t addr);
void AT25X041B_UnprotectSector(SPI_HandleTypeDef *SPI,AT25X041B_t *AT25X041B,uint32_t addr);
void AT25X041B_ProtectSector(SPI_HandleTypeDef *SPI,AT25X041B_t *AT25X041B,uint32_t addr);
void WriteCoordinateFlash(SPI_HandleTypeDef *SPI,AT25X041B_t *AT25X041B,coord_t coordinate);
coord_t ReadLastCoordinateFlash(SPI_HandleTypeDef *SPI,AT25X041B_t *AT25X041B);
#endif /* INC_AT25X041B_H_ */
