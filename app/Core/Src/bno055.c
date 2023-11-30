/*(*******************************************************
PROGRAM NAME - Bno055 driver

PROGRAMMER - Bertet Gabriel

SYSTEM - Currently working on a STM32G474RET6 with the Mpu6050 accelerometer

DATE - Started 15/04/2023

BUGS - No major bug ever reported. If you find a bug please report it on github

DESCRIPTION - Basic driver for the bno055 accelerometer.
*******************************************************)*/
#include "bno055.h"

uint16_t accelScale = 100;
uint16_t tempScale = 1;
uint16_t angularRateScale = 16;
uint16_t eulerScale = 16;
uint16_t magScale = 16;
uint16_t quaScale = (1<<14);    // 2^14


/*
 * Method use to switch page in IMU memory
 * param: I2C --> pointer on I2C handle struct
 * param: page --> integer value range 0 to 1 include
 * note: If a page value is bigger than 1 then no error will be raise so be carefull
 */
void BNO055_SetPage(I2C_HandleTypeDef *I2C,uint8_t page){
	HAL_I2C_Mem_Write(I2C, BNO055_I2C_ADDR, BNO055_PAGE_ID, 1, &page, 1, 10);
}

/*
 * Method use to switch operation mode of IMU
 * param: I2C --> pointer on I2C handle struct
 * param: BNO055 --> pointer on IMU struct
 * param: mode --> operation mode already define in <bno055.h>
 */
void BNO055_SetOperationMode(I2C_HandleTypeDef *I2C,bno055_opmode_t mode,BNO055_t *BNO055){
	BNO055->operational_mode = mode;
	HAL_I2C_Mem_Write(I2C, BNO055_I2C_ADDR, BNO055_OPR_MODE, 1, &mode, 1, 10);
}

/*
 * Method use to get operation mode of IMU
 * param: I2C --> pointer on I2C handle struct
 */
bno055_opmode_t BNO055_getOperationMode(I2C_HandleTypeDef *I2C){
  bno055_opmode_t mode;
  HAL_I2C_Mem_Read(I2C, BNO055_I2C_ADDR, BNO055_OPR_MODE, 1, &mode, 1, 10);
  return mode;
}

/*
 * Function use read the system register
 * param: I2C --> pointer on I2C handle struct
 */
uint8_t BNO055_ReadSystemReg(I2C_HandleTypeDef *I2C){
	uint8_t system_reg;
	HAL_I2C_Mem_Read(I2C, BNO055_I2C_ADDR, BNO055_SYS_TRIGGER, 1, &system_reg, 1, 100);
	return system_reg;
}

/*
 * Function use to read interrupt register
 * param: I2C --> pointer on I2C handle struct
 */
uint8_t BNO055_ReadITStatus(I2C_HandleTypeDef * I2C){
	uint8_t system_reg;
	HAL_I2C_Mem_Read(I2C, BNO055_I2C_ADDR, BNO055_INT_STATUS, 1, &system_reg, 1, 100);
	return system_reg;
}
/*
 * Method use to set high G acceleration interrupt
 * param: I2C --> pointer on I2C handle struct
 * note: This method should only be call after a SetPage(1)
 */
void BNO055_EnableAccHighG(I2C_HandleTypeDef *I2C){
	uint8_t system_reg;
	HAL_I2C_Mem_Read(I2C, BNO055_I2C_ADDR, BNO055_INT_EN, 1, &system_reg, 1, 100);
	system_reg|=1<<5;
	HAL_I2C_Mem_Write(I2C, BNO055_I2C_ADDR, BNO055_INT_EN, 1, &system_reg, 1, 10);
}

/*
 * Method use to set high G acceleration interrupt for each axis
 * param: I2C --> pointer on I2C handle struct
 * axis: --> char use to define an axis Eg: 'X' or 'Y' or 'Z'
 */
void BNO055_EnableHighGAcc(I2C_HandleTypeDef *I2C,char axis){
	uint8_t system_reg;
	HAL_I2C_Mem_Read(I2C, BNO055_I2C_ADDR, BNO055_ACC_INT_SETTINGS, 1, &system_reg, 1, 100);
	switch (axis) {
		case 'X':
			system_reg|=1<<5;
			break;
		case 'Y':
			system_reg|=1<<6;
			break;
		case 'Z':
			system_reg|=1<<7;
			break;
	}
	HAL_I2C_Mem_Write(I2C, BNO055_I2C_ADDR, BNO055_ACC_INT_SETTINGS, 1, &system_reg, 1, 10);
	HAL_I2C_Mem_Read(I2C, BNO055_I2C_ADDR, BNO055_ACC_INT_SETTINGS, 1, &system_reg, 1, 100);
}

/*
 * Method use to clear interrupt flag set by hardware
 * param: I2C --> pointer on I2C handle struct
 * note: if call in interrupt, you can find SPI pointer in IMU struct
 */
void BNO055_ClearIntFlag(I2C_HandleTypeDef *I2C){
	uint8_t system_reg;
	HAL_I2C_Mem_Read(I2C, BNO055_I2C_ADDR, BNO055_SYS_TRIGGER, 1, &system_reg, 1, 100);
	system_reg|=1<<6;
	HAL_I2C_Mem_Write(I2C, BNO055_I2C_ADDR, BNO055_SYS_TRIGGER, 1, &system_reg, 1, 10);
}


/*
 * Method use to enable external clock for IMU
 * param: I2C --> pointer on I2C handle struct
 * note: If not clock selected then work with LSI else LSE
 */
void BNO055_EnableExtClock(I2C_HandleTypeDef *I2C){
	uint8_t actual_reg_value = BNO055_ReadSystemReg(I2C);
	//Change the 7th bit of this reg --> activate external clock
	uint8_t sys_reg_value = (actual_reg_value&0x7F) + (0x01<<7);
	HAL_I2C_Mem_Write(I2C, BNO055_I2C_ADDR, BNO055_SYS_TRIGGER, 1, &sys_reg_value, 1, 10);
}

/*
 * Method use to disable external clock for IMU
 * param: I2C --> pointer on I2C handle struct
 * note: If not clock selected then work with LSI else LSE
 */
void BNO055_DisableExtClock(I2C_HandleTypeDef *I2C){
	uint8_t actual_reg_value = BNO055_ReadSystemReg(I2C);
	//Change the 7th bit of this reg --> activate external clock
	uint8_t sys_reg_value = (actual_reg_value&0x7F);
	HAL_I2C_Mem_Write(I2C, BNO055_I2C_ADDR, BNO055_SYS_TRIGGER, 1, &sys_reg_value, 1, 10);
}


/*
 * Method use to start a self test
 * param: I2C --> pointer on I2C handle struct
 */
void BNO055_TriggerSelfTest(I2C_HandleTypeDef *I2C){
	uint8_t actual_reg_value = BNO055_ReadSystemReg(I2C);
	//Change the 7th bit of this reg --> activate external clock
	uint8_t sys_reg_value = (actual_reg_value&0xFE) + 0x01;
	HAL_I2C_Mem_Write(I2C, BNO055_I2C_ADDR, BNO055_SYS_TRIGGER, 1, &sys_reg_value, 1, 10);
}

/*
 * Method use to check self test result
 * param: I2C --> pointer on I2C handle struct
 * param: BNO055 --> pointer on IMU struct
 */
uint8_t BNO055_CheckSelfTestResult(I2C_HandleTypeDef *I2C,BNO055_t *BNO055){
	uint8_t res = 0;
	uint8_t reg_value;
	//Checking all self power on test result
	HAL_I2C_Mem_Read(I2C, BNO055_I2C_ADDR, BNO055_ST_RESULT, 1, &reg_value, 1, 10);
	if((reg_value&0x0F)!=0x0F){
		res = 1;
	}
	//Trigger a test to ensure sensor is working properly
	BNO055_TriggerSelfTest(I2C);
	//Wait some time to ensure test is done (made with hal delay because task hasn't started yet)
	osDelay(500);
	HAL_I2C_Mem_Read(I2C, BNO055_I2C_ADDR, BNO055_SYS_ERR, 1, &reg_value, 1, 10);
	if(reg_value!=0x00){
		res = 1;
	}
	HAL_I2C_Mem_Read(I2C, BNO055_I2C_ADDR, BNO055_SYS_STATUS, 1, &reg_value, 1, 10);

	return res;
}
/*
 * Method use to set the accelerometer range
 * param: I2C --> pointer on I2C handle struct
 * param: range --> type range already defined in <bno055.h>
 * note: need to be call after page(1)
 */
void BNO055_SetAccelerometerRange(I2C_HandleTypeDef *I2C,bno055_accel_range_t range){
	uint8_t system_reg;
	HAL_I2C_Mem_Read(I2C, BNO055_I2C_ADDR, BNO055_ACC_CONFIG, 1, &system_reg, 1, 100);
	system_reg|= range;
	HAL_I2C_Mem_Write(I2C, BNO055_I2C_ADDR, BNO055_ACC_CONFIG, 1, &system_reg, 1, 10);
}

/*
 * Method use to set the accelerometer range
 * param: I2C --> pointer on I2C handle struct
 * param: unit --> type unit already defined in <bno055.h>
 */
void BNO055_SetAccelerometerUnit(I2C_HandleTypeDef *I2C,bno055_accel_unit_t unit){
	uint8_t system_reg;
	HAL_I2C_Mem_Read(I2C, BNO055_I2C_ADDR, BNO055_UNIT_SEL, 1, &system_reg, 1, 100);
	system_reg|= unit;
	HAL_I2C_Mem_Write(I2C, BNO055_I2C_ADDR, BNO055_UNIT_SEL, 1, &system_reg, 1, 10);
}


/*
 * Init function for IMU, it enable it run self test
 * param: I2C --> pointer on I2C handle struct
 * param: BNO055 --> pointer on IMU struct
 */
uint8_t BNO055_Init(I2C_HandleTypeDef *I2C,BNO055_t *BNO055){

	uint8_t res = 0;
	uint8_t reg;

	//Reset the IMU error counter
	BNO055->self_test_error = 0;

	//Add pointer into IMU struct
	BNO055->I2C_pt = I2C;

	//We need to select the page 1
	BNO055_SetPage(I2C,1);

	//Set range at 4G
	BNO055_SetAccelerometerRange(I2C, BNO055_ACCEL_RANGE_4G);

	//Enable High G accelerometer interrupt
	BNO055_EnableAccHighG(I2C);

	//Enable HIGH G for axis X/Y/Z
	BNO055_EnableHighGAcc(I2C, 'X');
	BNO055_EnableHighGAcc(I2C, 'Y');
	BNO055_EnableHighGAcc(I2C, 'Z');

	//We need to select the page 0
	BNO055_SetPage(I2C,0);

	//Set unit for accelerometer in G
	BNO055_SetAccelerometerUnit(I2C, BNO055_ACCEL_UNIT_G);

	//As the board have external 32.756 khz clock we use it
	BNO055_EnableExtClock(I2C);
	/*Make BIST and power up test*/
	osDelay(200);
	reg = BNO055_CheckSelfTestResult(I2C, BNO055);
	if(reg != 0x00){
		res = 1;
	}

	//Select the operation mode (the NDOF use all 3 sensor and will calculate data using absolute orientation (USE BNO055_OPERATION_MODE_IMU if you want to have relative orientation)
	BNO055_SetOperationMode(I2C, BNO055_OPERATION_MODE_NDOF, BNO055);

	/*Check for writing problem of OPR_MODE_REG*/
	HAL_I2C_Mem_Read(I2C, BNO055_I2C_ADDR, BNO055_OPR_MODE, 1, &reg, 1, 10);
	if( reg != BNO055_OPERATION_MODE_NDOF){
		res = 1;
	}

	return res;
}

/*
 * Function use to read gravity data from IMU
 * param: I2C --> pointer on I2C handle struct
 * param: BNO055 --> pointer on IMU struct
 * param: I2CControllerProtect --> Semaphore use to protect I2C hardware from being modified by more than one task at the same time
 */
void BNO055_ReadGrav(I2C_HandleTypeDef *I2C,BNO055_t *BNO055,osMutexId I2CControllerProtect){
	//Read gyroscope value
	uint8_t reg_value_lsb;
	uint8_t reg_value_msb;

	xSemaphoreTake(I2CControllerProtect,15);
	HAL_I2C_Mem_Read(I2C, BNO055_I2C_ADDR, BNO055_GRV_DATA_X_LSB, 1, &reg_value_lsb, 1, 10);
	HAL_I2C_Mem_Read(I2C, BNO055_I2C_ADDR, BNO055_GRV_DATA_X_MSB, 1, &reg_value_msb, 1, 10);
	xSemaphoreGive(I2CControllerProtect);
	BNO055->processed_data.gravity_x = reg_value_lsb + (reg_value_msb<<8);


	xSemaphoreTake(I2CControllerProtect,15);
	HAL_I2C_Mem_Read(I2C, BNO055_I2C_ADDR, BNO055_GRV_DATA_Y_LSB, 1, &reg_value_lsb, 1, 10);
	HAL_I2C_Mem_Read(I2C, BNO055_I2C_ADDR, BNO055_GRV_DATA_Y_MSB, 1, &reg_value_msb, 1, 10);
	xSemaphoreGive(I2CControllerProtect);
	BNO055->processed_data.gravity_y = reg_value_lsb + (reg_value_msb<<8);


	xSemaphoreTake(I2CControllerProtect,15);
	HAL_I2C_Mem_Read(I2C, BNO055_I2C_ADDR, BNO055_GRV_DATA_Z_LSB, 1, &reg_value_lsb, 1, 10);
	HAL_I2C_Mem_Read(I2C, BNO055_I2C_ADDR, BNO055_GRV_DATA_Z_MSB, 1, &reg_value_msb, 1, 10);
	xSemaphoreGive(I2CControllerProtect);
	BNO055->processed_data.gravity_z = reg_value_lsb + (reg_value_msb<<8);
}

/*
 * Function use to read temperature inside the IMU
 * param: I2C --> pointer on I2C handle struct
 * param: BNO055 --> pointer on IMU struct
 * param: I2CControllerProtect --> Semaphore use to protect I2C hardware from being modified by more than one task at the same time
 */
void BNO055_ReadTemp(I2C_HandleTypeDef *I2C,BNO055_t *BNO055,osMutexId I2CControllerProtect){

	//Read temp value
	uint8_t reg_value;

	xSemaphoreTake(I2CControllerProtect,15);
	HAL_I2C_Mem_Read(I2C, BNO055_I2C_ADDR, BNO055_TEMP, 1, &reg_value, 1, 10);
	xSemaphoreGive(I2CControllerProtect);
	BNO055->raw_data.temp = reg_value;

}
/*
 * Function use to read gyrosope value from IMU
 * param: I2C --> pointer on I2C handle struct
 * param: BNO055 --> pointer on IMU struct
 * param: I2CControllerProtect --> Semaphore use to protect I2C hardware from being modified by more than one task at the same time
 */
void BNO055_ReadGyro(I2C_HandleTypeDef *I2C,BNO055_t *BNO055,osMutexId I2CControllerProtect){
	//Read gyroscope value
	uint8_t reg_value_lsb;
	uint8_t reg_value_msb;

	xSemaphoreTake(I2CControllerProtect,25);
	HAL_I2C_Mem_Read(I2C, BNO055_I2C_ADDR, BNO055_GYR_DATA_X_LSB, 1, &reg_value_lsb, 1, 10);
	HAL_I2C_Mem_Read(I2C, BNO055_I2C_ADDR, BNO055_GYR_DATA_X_MSB, 1, &reg_value_msb, 1, 10);
	xSemaphoreGive(I2CControllerProtect);
	BNO055->raw_data.gyro_x = reg_value_lsb + (reg_value_msb<<8);


	xSemaphoreTake(I2CControllerProtect,25);
	HAL_I2C_Mem_Read(I2C, BNO055_I2C_ADDR, BNO055_GYR_DATA_Y_LSB, 1, &reg_value_lsb, 1, 10);
	HAL_I2C_Mem_Read(I2C, BNO055_I2C_ADDR, BNO055_GYR_DATA_Y_MSB, 1, &reg_value_msb, 1, 10);
	xSemaphoreGive(I2CControllerProtect);
	BNO055->raw_data.gyro_y = reg_value_lsb + (reg_value_msb<<8);


	xSemaphoreTake(I2CControllerProtect,25);
	HAL_I2C_Mem_Read(I2C, BNO055_I2C_ADDR, BNO055_GYR_DATA_Z_LSB, 1, &reg_value_lsb, 1, 10);
	HAL_I2C_Mem_Read(I2C, BNO055_I2C_ADDR, BNO055_GYR_DATA_Z_MSB, 1, &reg_value_msb, 1, 10);
	xSemaphoreGive(I2CControllerProtect);
	BNO055->raw_data.gyro_z = reg_value_lsb + (reg_value_msb<<8);
}

/*
 * Function use to read accelerometer data from IMU
 * param: I2C --> pointer on I2C handle struct
 * param: BNO055 --> pointer on IMU struct
 * param: I2CControllerProtect --> Semaphore use to protect I2C hardware from being modified by more than one task at the same time
 */
void BNO055_ReadAccel(I2C_HandleTypeDef *I2C,BNO055_t *BNO055,osMutexId I2CControllerProtect){
	//Read gyroscope value
	uint8_t reg_value_lsb;
	uint8_t reg_value_msb;


	xSemaphoreTake(I2CControllerProtect,25);
	HAL_I2C_Mem_Read(I2C, BNO055_I2C_ADDR, BNO055_ACC_DATA_X_LSB, 1, &reg_value_lsb, 1, 10);
	HAL_I2C_Mem_Read(I2C, BNO055_I2C_ADDR, BNO055_ACC_DATA_X_MSB, 1, &reg_value_msb, 1, 10);
	xSemaphoreGive(I2CControllerProtect);
	BNO055->raw_data.accelerometer_x = reg_value_lsb + (reg_value_msb<<8);

	xSemaphoreTake(I2CControllerProtect,25);
	HAL_I2C_Mem_Read(I2C, BNO055_I2C_ADDR, BNO055_ACC_DATA_Y_LSB, 1, &reg_value_lsb, 1, 10);
	HAL_I2C_Mem_Read(I2C, BNO055_I2C_ADDR, BNO055_ACC_DATA_Y_MSB, 1, &reg_value_msb, 1, 10);
	xSemaphoreGive(I2CControllerProtect);
	BNO055->raw_data.accelerometer_y = reg_value_lsb + (reg_value_msb<<8);

	xSemaphoreTake(I2CControllerProtect,25);
	HAL_I2C_Mem_Read(I2C, BNO055_I2C_ADDR, BNO055_ACC_DATA_Z_LSB, 1, &reg_value_lsb, 1, 10);
	HAL_I2C_Mem_Read(I2C, BNO055_I2C_ADDR, BNO055_ACC_DATA_Z_MSB, 1, &reg_value_msb, 1, 10);
	xSemaphoreGive(I2CControllerProtect);
	BNO055->raw_data.accelerometer_z = reg_value_lsb + (reg_value_msb<<8);
}

/*
 * Function use to read magnetometer data from IMU
 * param: I2C --> pointer on I2C handle struct
 * param: BNO055 --> pointer on IMU struct
 * param: I2CControllerProtect --> Semaphore use to protect I2C hardware from being modified by more than one task at the same time
 */
void BNO055_ReadMagneter(I2C_HandleTypeDef *I2C,BNO055_t *BNO055,osMutexId I2CControllerProtect){
	//Read gyroscope value
	uint8_t reg_value_lsb;
	uint8_t reg_value_msb;

	xSemaphoreTake(I2CControllerProtect,25);
	HAL_I2C_Mem_Read(I2C, BNO055_I2C_ADDR, BNO055_MAG_DATA_X_LSB, 1, &reg_value_lsb, 1, 10);
	HAL_I2C_Mem_Read(I2C, BNO055_I2C_ADDR, BNO055_MAG_DATA_X_MSB, 1, &reg_value_msb, 1, 10);
	xSemaphoreGive(I2CControllerProtect);
	BNO055->raw_data.magnetometer_x = (float) ((reg_value_lsb + (reg_value_msb<<8))/ (float) magScale);


	xSemaphoreTake(I2CControllerProtect,25);
	HAL_I2C_Mem_Read(I2C, BNO055_I2C_ADDR, BNO055_MAG_DATA_Y_LSB, 1, &reg_value_lsb, 1, 10);
	HAL_I2C_Mem_Read(I2C, BNO055_I2C_ADDR, BNO055_MAG_DATA_Y_MSB, 1, &reg_value_msb, 1, 10);
	xSemaphoreGive(I2CControllerProtect);
	BNO055->raw_data.magnetometer_y = (float) ((reg_value_lsb + (reg_value_msb<<8))/ (float) magScale);


	xSemaphoreTake(I2CControllerProtect,25);
	HAL_I2C_Mem_Read(I2C, BNO055_I2C_ADDR, BNO055_MAG_DATA_Z_LSB, 1, &reg_value_lsb, 1, 10);
	HAL_I2C_Mem_Read(I2C, BNO055_I2C_ADDR, BNO055_MAG_DATA_Z_MSB, 1, &reg_value_msb, 1, 10);
	xSemaphoreGive(I2CControllerProtect);
	BNO055->raw_data.magnetometer_z = (float) ((reg_value_lsb + (reg_value_msb<<8))/ (float) magScale);
}
/*
 * Function use to read quaternion data from IMU (from fusion sensor)
 * param: I2C --> pointer on I2C handle struct
 * param: BNO055 --> pointer on IMU struct
 * param: I2CControllerProtect --> Semaphore use to protect I2C hardware from being modified by more than one task at the same time
 */
void BNO055_ReadQua(I2C_HandleTypeDef *I2C,BNO055_t *BNO055,osMutexId I2CControllerProtect){
	//Read gyroscope value
	uint8_t reg_value_lsb;
	uint8_t reg_value_msb;

	xSemaphoreTake(I2CControllerProtect,25);
	HAL_I2C_Mem_Read(I2C, BNO055_I2C_ADDR, BNO055_QUA_DATA_W_LSB, 1, &reg_value_lsb, 1, 10);
	HAL_I2C_Mem_Read(I2C, BNO055_I2C_ADDR, BNO055_QUA_DATA_W_MSB, 1, &reg_value_msb, 1, 10);
	xSemaphoreGive(I2CControllerProtect);
	BNO055->processed_data.quaternion_w = (float) ((reg_value_lsb + (reg_value_msb<<8))/ (float) quaScale);

	xSemaphoreTake(I2CControllerProtect,25);
	HAL_I2C_Mem_Read(I2C, BNO055_I2C_ADDR, BNO055_QUA_DATA_X_LSB, 1, &reg_value_lsb, 1, 10);
	HAL_I2C_Mem_Read(I2C, BNO055_I2C_ADDR, BNO055_QUA_DATA_X_MSB, 1, &reg_value_msb, 1, 10);
	xSemaphoreGive(I2CControllerProtect);
	BNO055->processed_data.quaternion_x = (float) ((reg_value_lsb + (reg_value_msb<<8))/ (float) quaScale);

	xSemaphoreTake(I2CControllerProtect,25);
	HAL_I2C_Mem_Read(I2C, BNO055_I2C_ADDR, BNO055_QUA_DATA_Y_LSB, 1, &reg_value_lsb, 1, 10);
	HAL_I2C_Mem_Read(I2C, BNO055_I2C_ADDR, BNO055_QUA_DATA_Y_MSB, 1, &reg_value_msb, 1, 10);
	xSemaphoreGive(I2CControllerProtect);
	BNO055->processed_data.quaternion_y = (float) ((reg_value_lsb + (reg_value_msb<<8))/ (float) quaScale);

	xSemaphoreTake(I2CControllerProtect,25);
	HAL_I2C_Mem_Read(I2C, BNO055_I2C_ADDR, BNO055_QUA_DATA_Z_LSB, 1, &reg_value_lsb, 1, 10);
	HAL_I2C_Mem_Read(I2C, BNO055_I2C_ADDR, BNO055_QUA_DATA_Z_MSB, 1, &reg_value_msb, 1, 10);
	xSemaphoreGive(I2CControllerProtect);
	BNO055->processed_data.quaternion_z = (float) ((reg_value_lsb + (reg_value_msb<<8))/ (float) quaScale);
}


/*
 * Function use to read linear acceleration data from IMU (from fusion sensor)
 * param: I2C --> pointer on I2C handle struct
 * param: BNO055 --> pointer on IMU struct
 * param: I2CControllerProtect --> Semaphore use to protect I2C hardware from being modified by more than one task at the same time
 */
void BNO055_ReadLina(I2C_HandleTypeDef *I2C,BNO055_t *BNO055,osMutexId I2CControllerProtect){
	//Read gyroscope value
	uint8_t reg_value_lsb;
	uint8_t reg_value_msb;

	xSemaphoreTake(I2CControllerProtect,25);
	HAL_I2C_Mem_Read(I2C, BNO055_I2C_ADDR, BNO055_LIA_DATA_X_LSB, 1, &reg_value_lsb, 1, 10);
	HAL_I2C_Mem_Read(I2C, BNO055_I2C_ADDR, BNO055_LIA_DATA_X_MSB, 1, &reg_value_msb, 1, 10);
	xSemaphoreGive(I2CControllerProtect);
	BNO055->processed_data.linear_accel_x = (float) ((reg_value_lsb + (reg_value_msb<<8))/ (float) accelScale);


	xSemaphoreTake(I2CControllerProtect,25);
	HAL_I2C_Mem_Read(I2C, BNO055_I2C_ADDR, BNO055_LIA_DATA_Y_LSB, 1, &reg_value_lsb, 1, 10);
	HAL_I2C_Mem_Read(I2C, BNO055_I2C_ADDR, BNO055_LIA_DATA_Y_MSB, 1, &reg_value_msb, 1, 10);
	xSemaphoreGive(I2CControllerProtect);
	BNO055->processed_data.linear_accel_y = (float) ((reg_value_lsb + (reg_value_msb<<8))/ (float) accelScale);


	xSemaphoreTake(I2CControllerProtect,25);
	HAL_I2C_Mem_Read(I2C, BNO055_I2C_ADDR, BNO055_LIA_DATA_Z_LSB, 1, &reg_value_lsb, 1, 10);
	HAL_I2C_Mem_Read(I2C, BNO055_I2C_ADDR, BNO055_LIA_DATA_Z_MSB, 1, &reg_value_msb, 1, 10);
	xSemaphoreGive(I2CControllerProtect);
	BNO055->processed_data.linear_accel_z = (float) ((reg_value_lsb + (reg_value_msb<<8)) / (float) accelScale);
	//In order to match with physical data as axis is map in strange way

}

/*Function use to read the euler roll value from an IMU
 * @INPUT - I2C interface struct
 * @INPUT - IMU struc
 * @OUTPUT - None
 * @INFORMATIONS - If use with freertos or other reeltime os please use a semaphore/mutex to protect I2C interface
 */
void BNO055_ReadEuler_Roll(I2C_HandleTypeDef *I2C,BNO055_t *BNO055){

	uint8_t reg_value_lsb;
	uint8_t reg_value_msb;

	HAL_I2C_Mem_Read(I2C, BNO055_I2C_ADDR, BNO055_EUL_ROLL_LSB, 1, &reg_value_lsb, 1, 10);
	HAL_I2C_Mem_Read(I2C, BNO055_I2C_ADDR, BNO055_EUL_ROLL_MSB, 1, &reg_value_msb, 1, 10);
	BNO055->processed_data.euler_roll = (float) ((reg_value_lsb + (reg_value_msb<<8))/ (float) eulerScale);
	/*Make sure the data are rotating clockwise*/
	if(BNO055->processed_data.euler_roll > 2000){
		BNO055->processed_data.euler_roll = BNO055->processed_data.euler_roll - 4096;
	}
}
/*Function use to read the euler pitch value from an IMU
 * @INPUT - I2C interface struct
 * @INPUT - IMU struc
 * @OUTPUT - None
 * @INFORMATIONS - If use with freertos or other reeltime os please use a semaphore/mutex to protect I2C interface
 */
void BNO055_ReadEuler_Pitch(I2C_HandleTypeDef *I2C,BNO055_t *BNO055){

	uint8_t reg_value_lsb;
	uint8_t reg_value_msb;

	HAL_I2C_Mem_Read(I2C, BNO055_I2C_ADDR, BNO055_EUL_PITCH_LSB, 1, &reg_value_lsb, 1, 10);
	HAL_I2C_Mem_Read(I2C, BNO055_I2C_ADDR, BNO055_EUL_PITCH_MSB, 1, &reg_value_msb, 1, 10);
	BNO055->processed_data.euler_pitch = (float) ((reg_value_lsb + (reg_value_msb<<8))/(float) eulerScale);
	/*Make sure the data are rotating clockwise*/
	if(BNO055->processed_data.euler_pitch > 2000){
		BNO055->processed_data.euler_pitch = BNO055->processed_data.euler_pitch - 4096;
	}
}
/*Function use to read the euler yaw value from an IMU
 * @INPUT - I2C interface struct
 * @INPUT - IMU struc
 * @OUTPUT - None
 * @INFORMATIONS - If use with freertos or other reeltime os please use a semaphore/mutex to protect I2C interface
 */
void BNO055_ReadEuler_Yaw(I2C_HandleTypeDef *I2C,BNO055_t *BNO055){

	uint8_t reg_value_lsb;
	uint8_t reg_value_msb;

	HAL_I2C_Mem_Read(I2C, BNO055_I2C_ADDR, BNO055_EUL_HEADING_LSB, 1, &reg_value_lsb, 1, 10);
	HAL_I2C_Mem_Read(I2C, BNO055_I2C_ADDR, BNO055_EUL_HEADING_MSB, 1, &reg_value_msb, 1, 10);
	BNO055->processed_data.euler_heading = (float) ((reg_value_lsb + (reg_value_msb<<8))/ (float) eulerScale);
}
