/*
 * bmp390.h
 *
 *  Created on: Apr 9, 2023
 *      Author: 33768
 */

#ifndef INC_BMP390_H_
#define INC_BMP390_H_
//---------------------------------------------------------------- INCLUDE  ----------------------------------------------------------------
#include "stm32g4xx_hal.h"
#include "cmsis_os.h"
//---------------------------------------------------------------- DEFINE ----------------------------------------------------------------

#define CHIP_ID_REG 0x00		//0x60
#define REV_ID_REG 0x01			//0x01
#define ERR_REG 0x02
#define STATUS_REG 0x03
#define EVENT_REG 0x10
#define INT_STATUS_REG 0x11
#define INT_CTRL_REG 0x19
#define IF_CONFIG_REG 0x1A
#define PWR_CTRL_REG 0x1B
#define OSR_REG 0x1C
#define ODR_REG 0x1D
#define CONFIG_REG 0x1F

#define TIME_XLSB_REG 0x0C
#define TIME_LSB_REG 0x0D
#define TIME_MSB_REG 0x0E

#define PRESS_XLSB_REG 0x04
#define PRESS_LSB_REG 0x05
#define PRESS_MSB_REG 0x06

#define TEMP_XLSB_REG 0x07
#define TEMP_LSB_REG 0x08
#define TEMP_MSB_REG 0x09

/*The addr should be 0x76 but as the HAL have a int8 bit and it took bit 7:1 then we have to do 0x76<<1*/
#define BMP390_I2C_ADDR 0xEC
#define PRESS_STEP 0.00005662441253662109375
/*only use at startup, the pressure sensor will do X measurement in order to get ground pressure (P0)*/
#define STARTUP_PRESSURE_AVERAGE_COUNT 10

//---------------------------------------------------------------- TYPE DEF ----------------------------------------------------------------

typedef struct{
	//pressure measured by the sensor
	float pressure_hpa;

	float temp_data;

	float time_data;

	float calibration_pressure;

	float differential_altitude;

}ALTIMETER_t;


//---------------------------------------------------------------- PROTOTYPE ----------------------------------------------------------------
uint8_t BMP390_CheckFatalError(I2C_HandleTypeDef *I2C);
uint8_t BMP390_CheckCmdError(I2C_HandleTypeDef *I2C);
uint8_t BMP390_CheckConfError(I2C_HandleTypeDef *I2C);
uint8_t BMP390_CheckDrdyPress(I2C_HandleTypeDef *I2C);
uint8_t BMP390_CheckDrdyTemp(I2C_HandleTypeDef *I2C);
void BMP390_ClearDrdyFlag(I2C_HandleTypeDef *I2C);
void BMP390_WakeUp(I2C_HandleTypeDef *I2C);
void BMP390_SleepMode(I2C_HandleTypeDef *I2C);
void BMP390_EnableTempSensor(I2C_HandleTypeDef *I2C);
void BMP390_DisableTempSensor(I2C_HandleTypeDef *I2C);
void BMP390_DisablePressureSensor(I2C_HandleTypeDef *I2C);
void BMP390_EnablePressureSensor(I2C_HandleTypeDef *I2C);
void BMP390_SetOverSamplingPressure(I2C_HandleTypeDef *I2C,uint8_t value);
void BMP390_SetOverSamplingTemp(I2C_HandleTypeDef *I2C,uint8_t value);
void BMP390_SetFilter(I2C_HandleTypeDef *I2C,uint8_t value);
void BMP390_ReadTemp(I2C_HandleTypeDef *I2C,ALTIMETER_t *altimeter,osMutexId I2CControllerProtect);
void BMP390_ReadPress(I2C_HandleTypeDef *I2C,ALTIMETER_t *altimeter,osMutexId I2CControllerProtect);
void BMP390_ReadTime(I2C_HandleTypeDef *I2C,ALTIMETER_t *altimeter,osMutexId I2CControllerProtect);
uint8_t BMP390_Init(I2C_HandleTypeDef *I2C);
void BMP390_GetP0Pressure(I2C_HandleTypeDef *I2C,ALTIMETER_t *altimeter,osMutexId I2CControllerProtect);
void BMP390_GetRelativeAltitude(ALTIMETER_t *altimeter);
#endif /* INC_BMP390_H_ */
