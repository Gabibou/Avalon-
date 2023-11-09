/*
 * bmp390.c
 *
 *  Created on: Apr 9, 2023
 *      Author: 33768
 */

#include "bmp390.h"

uint8_t BMP390_CheckFatalError(I2C_HandleTypeDef *I2C){

	uint8_t data;
	uint8_t res = 0;
	HAL_I2C_Mem_Read(I2C, BMP390_I2C_ADDR, ERR_REG, 1, &data, 1, 10);
	if((data&0x01) == 0x01){
		res = 1;
	}
	return res;

}

uint8_t BMP390_CheckCmdError(I2C_HandleTypeDef *I2C){

	uint8_t data;
	uint8_t res = 0;
	HAL_I2C_Mem_Read(I2C, BMP390_I2C_ADDR, ERR_REG, 1, &data, 1, 10);
	if((data&0x02)>>1 == 0x01){
		res = 1;
	}
	return res;

}

uint8_t BMP390_CheckConfError(I2C_HandleTypeDef *I2C){

	uint8_t data;
	uint8_t res = 0;
	HAL_I2C_Mem_Read(I2C, BMP390_I2C_ADDR, ERR_REG, 1, &data, 1, 10);
	if((data&0x04)>>2 == 0x01){
		res = 1;
	}
	return res;

}

uint8_t BMP390_CheckDrdyPress(I2C_HandleTypeDef *I2C){
	uint8_t data;
	uint8_t res = 0;
	HAL_I2C_Mem_Read(I2C, BMP390_I2C_ADDR, STATUS_REG, 1, &data, 1, 10);
	if((data&0x20)>>5 == 0x01){
		res = 1;
	}
	return res;
}

uint8_t BMP390_CheckDrdyTemp(I2C_HandleTypeDef *I2C){

	uint8_t data;
	uint8_t res = 0;
	HAL_I2C_Mem_Read(I2C, BMP390_I2C_ADDR, STATUS_REG, 1, &data, 1, 10);
	if((data&0x40)>>6 == 0x01){
		res = 1;
	}
	return res;
}

void BMP390_ClearDrdyFlag(I2C_HandleTypeDef *I2C){
	//Clearing the register is done by reading it
	uint8_t data;
	HAL_I2C_Mem_Read(I2C, BMP390_I2C_ADDR, STATUS_REG, 1, &data, 1, 10);
}

void BMP390_EnablePressureSensor(I2C_HandleTypeDef *I2C){
	uint8_t reg_value;
	uint8_t data;
	HAL_I2C_Mem_Read(I2C, BMP390_I2C_ADDR, PWR_CTRL_REG, 1, &reg_value, 1, 10);
	data = (reg_value&0x32) + 0x01;
	HAL_I2C_Mem_Write(I2C, BMP390_I2C_ADDR, PWR_CTRL_REG, 1, &data, 1, 10);
}

void BMP390_DisablePressureSensor(I2C_HandleTypeDef *I2C){
	uint8_t reg_value;
	uint8_t data;
	HAL_I2C_Mem_Read(I2C, BMP390_I2C_ADDR, PWR_CTRL_REG, 1, &reg_value, 1, 10);
	data = (reg_value&0x32);
	HAL_I2C_Mem_Write(I2C, BMP390_I2C_ADDR, PWR_CTRL_REG, 1, &data, 1, 10);
}

void BMP390_DisableTempSensor(I2C_HandleTypeDef *I2C){
	uint8_t reg_value;
	uint8_t data;
	HAL_I2C_Mem_Read(I2C, BMP390_I2C_ADDR, PWR_CTRL_REG, 1, &reg_value, 1, 10);
	data = (reg_value&0x31);
	HAL_I2C_Mem_Write(I2C, BMP390_I2C_ADDR, PWR_CTRL_REG, 1, &data, 1, 10);
}

void BMP390_EnableTempSensor(I2C_HandleTypeDef *I2C){
	uint8_t reg_value;
	uint8_t data;
	HAL_I2C_Mem_Read(I2C, BMP390_I2C_ADDR, PWR_CTRL_REG, 1, &reg_value, 1, 10);
	data = (reg_value&0x31) + 0x01;
	HAL_I2C_Mem_Write(I2C, BMP390_I2C_ADDR, PWR_CTRL_REG, 1, &data, 1, 10);
}

void BMP390_SleepMode(I2C_HandleTypeDef *I2C){
	uint8_t reg_value;
	uint8_t data;
	HAL_I2C_Mem_Read(I2C, BMP390_I2C_ADDR, PWR_CTRL_REG, 1, &reg_value, 1, 10);
	data = (reg_value&0x03);
	HAL_I2C_Mem_Write(I2C, BMP390_I2C_ADDR, PWR_CTRL_REG, 1, &data, 1, 10);
}

void BMP390_WakeUp(I2C_HandleTypeDef *I2C){
	uint8_t reg_value;
	uint8_t data;
	HAL_I2C_Mem_Read(I2C, BMP390_I2C_ADDR, PWR_CTRL_REG, 1, &reg_value, 1, 10);
	data = (reg_value&0x03) + 0x30;
	HAL_I2C_Mem_Write(I2C, BMP390_I2C_ADDR, PWR_CTRL_REG, 1, &data, 1, 10);
}

void BMP390_SetOverSamplingPressure(I2C_HandleTypeDef *I2C,uint8_t value){
	//You have to make sure that value is on 3bit
	uint8_t reg_value;
	uint8_t data;
	HAL_I2C_Mem_Read(I2C, BMP390_I2C_ADDR, OSR_REG, 1, &reg_value, 1, 10);
	data = (reg_value&0x38) + value;
	HAL_I2C_Mem_Write(I2C, BMP390_I2C_ADDR, OSR_REG, 1, &data, 1, 10);
}

void BMP390_SetOverSamplingTemp(I2C_HandleTypeDef *I2C,uint8_t value){
	//You have to make sure that value is on 3bit
	uint8_t reg_value;
	uint8_t data;
	HAL_I2C_Mem_Read(I2C, BMP390_I2C_ADDR, OSR_REG, 1, &reg_value, 1, 10);
	data = (reg_value&0x07) + (value<<3);
	HAL_I2C_Mem_Write(I2C, BMP390_I2C_ADDR, OSR_REG, 1, &data, 1, 10);
}

void BMP390_SetFilter(I2C_HandleTypeDef *I2C,uint8_t value){
	uint8_t reg_value;
	uint8_t data;
	HAL_I2C_Mem_Read(I2C, BMP390_I2C_ADDR, CONFIG_REG, 1, &reg_value, 1, 10);
	data = (reg_value&0x01) + (value<<1);
	HAL_I2C_Mem_Write(I2C, BMP390_I2C_ADDR, OSR_REG, 1, &data, 1, 10);
}

void BMP390_ReadTemp(I2C_HandleTypeDef *I2C,ALTIMETER_t *altimeter,osMutexId I2CControllerProtect){
	uint8_t XLSB;
	uint8_t LSB;
	uint8_t MSB;
	uint32_t temp;

	xSemaphoreTake(I2CControllerProtect, 35);
	HAL_I2C_Mem_Read(I2C, BMP390_I2C_ADDR, TEMP_XLSB_REG, 1, &XLSB, 1, 10);
	HAL_I2C_Mem_Read(I2C, BMP390_I2C_ADDR, TEMP_LSB_REG, 1, &LSB, 1, 10);
	HAL_I2C_Mem_Read(I2C, BMP390_I2C_ADDR, TEMP_MSB_REG, 1, &MSB, 1, 10);
	xSemaphoreGive(I2CControllerProtect);
	temp = XLSB + (LSB<<8) + (MSB<<16);
	altimeter->temp_data = (temp/258111);
}

void BMP390_ReadPress(I2C_HandleTypeDef *I2C,ALTIMETER_t *altimeter,osMutexId I2CControllerProtect){
	uint8_t XLSB;
	uint8_t LSB;
	uint8_t MSB;
	uint32_t press;

	xSemaphoreTake(I2CControllerProtect, 35);
	HAL_I2C_Mem_Read(I2C, BMP390_I2C_ADDR, PRESS_XLSB_REG, 1, &XLSB, 1, 10);
	HAL_I2C_Mem_Read(I2C, BMP390_I2C_ADDR, PRESS_LSB_REG, 1, &LSB, 1, 10);
	HAL_I2C_Mem_Read(I2C, BMP390_I2C_ADDR, PRESS_MSB_REG, 1, &MSB, 1, 10);
	xSemaphoreGive(I2CControllerProtect);
	press = XLSB + (LSB<<8) + (MSB<<16);
	altimeter->pressure_hpa = (press*PRESS_STEP)+300;

}


void BMP390_ReadTime(I2C_HandleTypeDef *I2C,ALTIMETER_t *altimeter,osMutexId I2CControllerProtect){
	uint8_t XLSB;
	uint8_t LSB;
	uint8_t MSB;
	uint32_t time;

	xSemaphoreTake(I2CControllerProtect, 35);
	HAL_I2C_Mem_Read(I2C, BMP390_I2C_ADDR, TIME_XLSB_REG, 1, &XLSB, 1, 10);
	HAL_I2C_Mem_Read(I2C, BMP390_I2C_ADDR, TIME_LSB_REG, 1, &LSB, 1, 10);
	HAL_I2C_Mem_Read(I2C, BMP390_I2C_ADDR, TIME_MSB_REG, 1, &MSB, 1, 10);
	xSemaphoreGive(I2CControllerProtect);
	time = XLSB + (LSB<<8) + (MSB<<16);
	altimeter->time_data = time;
}

uint8_t BMP390_Init(I2C_HandleTypeDef *I2C){

	uint8_t data;
	uint8_t res = 0;

	//Start by checking dev ID and REV iD
	HAL_I2C_Mem_Read(I2C, BMP390_I2C_ADDR, CHIP_ID_REG, 1, &data, 1, 10);
	if(data!=0x60){
		res = 1;
	}
	HAL_I2C_Mem_Read(I2C, BMP390_I2C_ADDR, REV_ID_REG, 1, &data, 1, 10);
	if(data!=0x01){
		res = 1;
	}

	//Enable all sensors
	BMP390_EnableTempSensor(I2C);
	BMP390_EnablePressureSensor(I2C);

	BMP390_WakeUp(I2C);
	/*Wait until sensor wake up */
	HAL_Delay(5);

	//Set the IT pin on high level and enable with pressure/temp sensor
	data = 0x42;
	HAL_I2C_Mem_Write(I2C, BMP390_I2C_ADDR, INT_CTRL_REG, 1, &data, 1, 10);

	//Set oversampling value
	//BMP390_SetOverSamplingPressure(I2C, 0x3);	//x8
	//BMP390_SetOverSamplingTemp(I2C, 0x00);	//x1

	//Set Filter value
	//BMP390_SetFilter(I2C, 0x02);


	return res;
}

void BMP390_GetP0Pressure(I2C_HandleTypeDef *I2C,ALTIMETER_t *altimeter,osMutexId I2CControllerProtect){
	float P0;
	for(int i=0;i<STARTUP_PRESSURE_AVERAGE_COUNT;i++){
		BMP390_ReadPress(I2C, altimeter, I2CControllerProtect);
		P0 += altimeter->pressure_hpa;
	}
	altimeter->calibration_pressure = (P0/STARTUP_PRESSURE_AVERAGE_COUNT);
}

/*Function use to calculate relative altitude. In order to work properly it should'nt be call before a calibration*/
void BMP390_GetRelativeAltitude(ALTIMETER_t *altimeter){
	altimeter->differential_altitude = 44330.0 * (1.0 - pow(altimeter->pressure_hpa / altimeter->calibration_pressure, 0.1903));
}
