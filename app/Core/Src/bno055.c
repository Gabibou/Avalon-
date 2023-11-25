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


void BNO055_SetPage(I2C_HandleTypeDef *I2C,uint8_t page){
	HAL_I2C_Mem_Write(I2C, BNO055_I2C_ADDR, BNO055_PAGE_ID, 1, &page, 1, 10);
}

void BNO055_SetOperationMode(I2C_HandleTypeDef *I2C,bno055_opmode_t mode,BNO055_t *BNO055){
	BNO055->operational_mode = mode;
	HAL_I2C_Mem_Write(I2C, BNO055_I2C_ADDR, BNO055_OPR_MODE, 1, &mode, 1, 10);
}

bno055_opmode_t BNO055_getOperationMode(I2C_HandleTypeDef *I2C){
  bno055_opmode_t mode;
  HAL_I2C_Mem_Read(I2C, BNO055_I2C_ADDR, BNO055_OPR_MODE, 1, &mode, 1, 10);
  return mode;
}

uint8_t BNO055_ReadSystemReg(I2C_HandleTypeDef *I2C){
	uint8_t system_reg;
	HAL_I2C_Mem_Read(I2C, BNO055_I2C_ADDR, BNO055_SYS_TRIGGER, 1, &system_reg, 1, 100);
	return system_reg;
}

//Enable or disable the external oscillator --> If the lse is disable then we use lsi
void BNO055_EnableExtClock(I2C_HandleTypeDef *I2C){
	uint8_t actual_reg_value = BNO055_ReadSystemReg(I2C);
	//Change the 7th bit of this reg --> activate external clock
	uint8_t sys_reg_value = (actual_reg_value&0x7F) + (0x01<<7);
	HAL_I2C_Mem_Write(I2C, BNO055_I2C_ADDR, BNO055_SYS_TRIGGER, 1, &sys_reg_value, 1, 10);
}

void BNO055_DisableExtClock(I2C_HandleTypeDef *I2C){
	uint8_t actual_reg_value = BNO055_ReadSystemReg(I2C);
	//Change the 7th bit of this reg --> activate external clock
	uint8_t sys_reg_value = (actual_reg_value&0x7F);
	HAL_I2C_Mem_Write(I2C, BNO055_I2C_ADDR, BNO055_SYS_TRIGGER, 1, &sys_reg_value, 1, 10);
}

void BNO055_TriggerSelfTest(I2C_HandleTypeDef *I2C){
	uint8_t actual_reg_value = BNO055_ReadSystemReg(I2C);
	//Change the 7th bit of this reg --> activate external clock
	uint8_t sys_reg_value = (actual_reg_value&0xFE) + 0x01;
	HAL_I2C_Mem_Write(I2C, BNO055_I2C_ADDR, BNO055_SYS_TRIGGER, 1, &sys_reg_value, 1, 10);
}

//Check the self test result and check some basic register of IMU --> If an error occure then return 1
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

uint8_t BNO055_Init(I2C_HandleTypeDef *I2C,BNO055_t *BNO055){

	uint8_t res = 0;
	uint8_t reg;

	//Reset the IMU error counter
	BNO055->self_test_error = 0;

	//We need to select the page 0
	BNO055_SetPage(I2C,0);
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
/*(NEED TO BE CHECK PRIOR TO LAUNCH)*/
void BNO055_ResetIntFlag(I2C_HandleTypeDef *I2C){
	uint8_t reg_value;
	uint8_t data;
	HAL_I2C_Mem_Read(I2C, BNO055_I2C_ADDR, BNO055_SYS_TRIGGER, 1, &reg_value, 1, 10);
	data = reg_value&0xBF;
	HAL_I2C_Mem_Write(I2C, BNO055_I2C_ADDR, BNO055_SYS_TRIGGER, 1, &data, 1, 10);
}

uint8_t BNO055_ReadInterruptStatus(I2C_HandleTypeDef *I2C){
	uint8_t reg_value;
	HAL_I2C_Mem_Read(I2C, BNO055_I2C_ADDR, BNO055_INT_STATUS, 1, &reg_value, 1, 10);
	return reg_value;
}



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

void BNO055_ReadTemp(I2C_HandleTypeDef *I2C,BNO055_t *BNO055,osMutexId I2CControllerProtect){

	//Read temp value
	uint8_t reg_value;

	xSemaphoreTake(I2CControllerProtect,15);
	HAL_I2C_Mem_Read(I2C, BNO055_I2C_ADDR, BNO055_TEMP, 1, &reg_value, 1, 10);
	xSemaphoreGive(I2CControllerProtect);
	BNO055->raw_data.temp = reg_value;

}

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
	BNO055->processed_data.linear_accel_z = (float) ((reg_value_lsb + (reg_value_msb<<8))/ (float) accelScale);
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
