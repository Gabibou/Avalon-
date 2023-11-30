/*
 * bno055.h
 *
 *  Created on: 22 avr. 2023
 *      Author: 33768
 */

#ifndef INC_BNO055_H_
#define INC_BNO055_H_
//---------------------------------------------------------------- INCLUDE  ----------------------------------------------------------------
#include "stm32g4xx_hal.h"
#include "cmsis_os.h"
//---------------------------------------------------------------- DEFINE ----------------------------------------------------------------
//Define register
// Page 0
#define BNO055_ID (0xA0)
#define BNO055_CHIP_ID 0x00        // value: 0xA0
#define BNO055_ACC_ID 0x01         // value: 0xFB
#define BNO055_MAG_ID 0x02         // value: 0x32
#define BNO055_GYRO_ID 0x03        // value: 0x0F
#define BNO055_SW_REV_ID_LSB 0x04  // value: 0x08
#define BNO055_SW_REV_ID_MSB 0x05  // value: 0x03
#define BNO055_BL_REV_ID 0x06      // N/A
#define BNO055_PAGE_ID 0x07
#define BNO055_ACC_DATA_X_LSB 0x08
#define BNO055_ACC_DATA_X_MSB 0x09
#define BNO055_ACC_DATA_Y_LSB 0x0A
#define BNO055_ACC_DATA_Y_MSB 0x0B
#define BNO055_ACC_DATA_Z_LSB 0x0C
#define BNO055_ACC_DATA_Z_MSB 0x0D
#define BNO055_MAG_DATA_X_LSB 0x0E
#define BNO055_MAG_DATA_X_MSB 0x0F
#define BNO055_MAG_DATA_Y_LSB 0x10
#define BNO055_MAG_DATA_Y_MSB 0x11
#define BNO055_MAG_DATA_Z_LSB 0x12
#define BNO055_MAG_DATA_Z_MSB 0x13
#define BNO055_GYR_DATA_X_LSB 0x14
#define BNO055_GYR_DATA_X_MSB 0x15
#define BNO055_GYR_DATA_Y_LSB 0x16
#define BNO055_GYR_DATA_Y_MSB 0x17
#define BNO055_GYR_DATA_Z_LSB 0x18
#define BNO055_GYR_DATA_Z_MSB 0x19
#define BNO055_EUL_HEADING_LSB 0x1A
#define BNO055_EUL_HEADING_MSB 0x1B
#define BNO055_EUL_ROLL_LSB 0x1C
#define BNO055_EUL_ROLL_MSB 0x1D
#define BNO055_EUL_PITCH_LSB 0x1E
#define BNO055_EUL_PITCH_MSB 0x1F
#define BNO055_QUA_DATA_W_LSB 0x20
#define BNO055_QUA_DATA_W_MSB 0x21
#define BNO055_QUA_DATA_X_LSB 0x22
#define BNO055_QUA_DATA_X_MSB 0x23
#define BNO055_QUA_DATA_Y_LSB 0x24
#define BNO055_QUA_DATA_Y_MSB 0x25
#define BNO055_QUA_DATA_Z_LSB 0x26
#define BNO055_QUA_DATA_Z_MSB 0x27
#define BNO055_LIA_DATA_X_LSB 0x28
#define BNO055_LIA_DATA_X_MSB 0x29
#define BNO055_LIA_DATA_Y_LSB 0x2A
#define BNO055_LIA_DATA_Y_MSB 0x2B
#define BNO055_LIA_DATA_Z_LSB 0x2C
#define BNO055_LIA_DATA_Z_MSB 0x2D
#define BNO055_GRV_DATA_X_LSB 0x2E
#define BNO055_GRV_DATA_X_MSB 0x2F
#define BNO055_GRV_DATA_Y_LSB 0x30
#define BNO055_GRV_DATA_Y_MSB 0x31
#define BNO055_GRV_DATA_Z_LSB 0x32
#define BNO055_GRV_DATA_Z_MSB 0x33
#define BNO055_TEMP 0x34
#define BNO055_CALIB_STAT 0x35
#define BNO055_ST_RESULT 0x36
#define BNO055_INT_STATUS 0x37
#define BNO055_SYS_CLK_STATUS 0x38
#define BNO055_SYS_STATUS 0x39
#define BNO055_SYS_ERR 0x3A
#define BNO055_UNIT_SEL 0x3B
#define BNO055_OPR_MODE 0x3D
#define BNO055_PWR_MODE 0x3E
#define BNO055_SYS_TRIGGER 0x3F
#define BNO055_TEMP_SOURCE 0x40
#define BNO055_AXIS_MAP_CONFIG 0x41
#define BNO055_AXIS_MAP_SIGN 0x42
#define BNO055_ACC_OFFSET_X_LSB 0x55
#define BNO055_ACC_OFFSET_X_MSB 0x56
#define BNO055_ACC_OFFSET_Y_LSB 0x57
#define BNO055_ACC_OFFSET_Y_MSB 0x58
#define BNO055_ACC_OFFSET_Z_LSB 0x59
#define BNO055_ACC_OFFSET_Z_MSB 0x5A
#define BNO055_MAG_OFFSET_X_LSB 0x5B
#define BNO055_MAG_OFFSET_X_MSB 0x5C
#define BNO055_MAG_OFFSET_Y_LSB 0x5D
#define BNO055_MAG_OFFSET_Y_MSB 0x5E
#define BNO055_MAG_OFFSET_Z_LSB 0x5F
#define BNO055_MAG_OFFSET_Z_MSB 0x60
#define BNO055_GYR_OFFSET_X_LSB 0x61
#define BNO055_GYR_OFFSET_X_MSB 0x62
#define BNO055_GYR_OFFSET_Y_LSB 0x63
#define BNO055_GYR_OFFSET_Y_MSB 0x64
#define BNO055_GYR_OFFSET_Z_LSB 0x65
#define BNO055_GYR_OFFSET_Z_MSB 0x66
#define BNO055_ACC_RADIUS_LSB 0x67
#define BNO055_ACC_RADIUS_MSB 0x68
#define BNO055_MAG_RADIUS_LSB 0x69
#define BNO055_MAG_RADIUS_MSB 0x6A
//
// BNO055 Page 1
#define BNO055_PAGE_ID 0x07
#define BNO055_ACC_CONFIG 0x08
#define BNO055_MAG_CONFIG 0x09
#define BNO055_GYRO_CONFIG_0 0x0A
#define BNO055_GYRO_CONFIG_1 0x0B
#define BNO055_ACC_SLEEP_CONFIG 0x0C
#define BNO055_GYR_SLEEP_CONFIG 0x0D
#define BNO055_INT_MSK 0x0F
#define BNO055_INT_EN 0x10
#define BNO055_ACC_AM_THRES 0x11
#define BNO055_ACC_INT_SETTINGS 0x12
#define BNO055_ACC_HG_DURATION 0x13
#define BNO055_ACC_HG_THRESH 0x14
#define BNO055_ACC_NM_THRESH 0x15
#define BNO055_ACC_NM_SET 0x16
#define BNO055_GYR_INT_SETTINGS 0x17
#define BNO055_GYR_HR_X_SET 0x18
#define BNO055_GYR_DUR_X 0x19
#define BNO055_GYR_HR_Y_SET 0x1A
#define BNO055_GYR_DUR_Y 0x1B
#define BNO055_GYR_HR_Z_SET 0x1C
#define BNO055_GYR_DUR_Z 0x1D
#define BNO055_GYR_AM_THRESH 0x1E
#define BNO055_GYR_AM_SET 0x1F

/*As the hal take addr bit number 7 to 1 and the addr is on bit 6 to 0 we have to do a 0x28<<1*/
#define BNO055_I2C_ADDR 0x50	//Can also be 0x29 if the COM3 pin is connected to VCC




//---------------------------------------------------------------- TYPE DEF ----------------------------------------------------------------
typedef enum {  // BNO-55 operation modes
  BNO055_OPERATION_MODE_CONFIG = 0x00,
  // Sensor Mode
  BNO055_OPERATION_MODE_ACCONLY,
  BNO055_OPERATION_MODE_MAGONLY,
  BNO055_OPERATION_MODE_GYRONLY,
  BNO055_OPERATION_MODE_ACCMAG,
  BNO055_OPERATION_MODE_ACCGYRO,
  BNO055_OPERATION_MODE_MAGGYRO,
  BNO055_OPERATION_MODE_AMG,  // 0x07
                              // Fusion Mode
  BNO055_OPERATION_MODE_IMU,
  BNO055_OPERATION_MODE_COMPASS,
  BNO055_OPERATION_MODE_M4G,
  BNO055_OPERATION_MODE_NDOF_FMC_OFF,
  BNO055_OPERATION_MODE_NDOF  // 0x0C
} bno055_opmode_t;

typedef enum {	//Accelerometer range
	BNO055_ACCEL_RANGE_2G = 0x00,
	BNO055_ACCEL_RANGE_4G = 0x01,
	BNO055_ACCEL_RANGE_8G = 0x02,
	BNO055_ACCEL_RANGE_16G = 0x03,
}bno055_accel_range_t;

typedef enum {	//Accelerometer unit
	BNO055_ACCEL_UNIT_MS = 0x00,
	BNO055_ACCEL_UNIT_G = 0x01,
}bno055_accel_unit_t;

typedef struct{
	//magnetometer raw value
	float magnetometer_x;
	float magnetometer_y;
	float magnetometer_z;

	//accelerometer raw value
	float accelerometer_x;
	float accelerometer_y;
	float accelerometer_z;

	//gyroscope raw value
	float gyro_x;
	float gyro_y;
	float gyro_z;

	uint8_t temp;

}BNO055_raw_t;

typedef struct{

	//Come from euler algorythm
	float euler_roll;
	float euler_pitch;
	float euler_heading;

	//Come from the quaternion algorythm
	float quaternion_w;
	float quaternion_x;
	float quaternion_y;
	float quaternion_z;

	//Calculated by the sensor (idk the algorythm this time)
	float linear_accel_x;
	float linear_accel_y;
	float linear_accel_z;

	//Calculated by the sensor (probably use some basic formula like p=mg)
	float gravity_x;
	float gravity_y;
	float gravity_z;

}BNO055_calculated_t;

typedef struct{

	//struct that store all important data incomming from the sensor
	BNO055_raw_t raw_data;

	//struct use to stora all processed data calculated from the sensor
	BNO055_calculated_t processed_data;

	//The IMU can work using many mode , the selected one should be writen here
	bno055_opmode_t operational_mode;

	//Check status --> Should stay at 0 else this mean error happend many time during self test
	uint8_t self_test_error;

	I2C_HandleTypeDef *I2C_pt;

}BNO055_t;



//---------------------------------------------------------------- PROTOTYPE ----------------------------------------------------------------
uint8_t BNO055_Init(I2C_HandleTypeDef *I2C,BNO055_t *BNO055);
void BNO055_SetPage(I2C_HandleTypeDef *I2C,uint8_t page);
void BNO055_SetOperationMode(I2C_HandleTypeDef *I2C,bno055_opmode_t mode,BNO055_t *BNO055);
bno055_opmode_t BNO055_getOperationMode(I2C_HandleTypeDef *I2C);
uint8_t BNO055_ReadSystemReg(I2C_HandleTypeDef *I2C);
void BNO055_DisableExtClock(I2C_HandleTypeDef *I2C);
void BNO055_EnableExtClock(I2C_HandleTypeDef *I2C);
uint8_t BNO055_CheckSelfTestResult(I2C_HandleTypeDef *I2C,BNO055_t *BNO055);
void BNO055_TriggerSelfTest(I2C_HandleTypeDef *I2C);
void BNO055_ReadTemp(I2C_HandleTypeDef *I2C,BNO055_t *BNO055,osMutexId I2CControllerProtect);
void BNO055_ReadGyro(I2C_HandleTypeDef *I2C,BNO055_t *BNO055,osMutexId I2CControllerProtect);
void BNO055_ReadAccel(I2C_HandleTypeDef *I2C,BNO055_t *BNO055,osMutexId I2CControllerProtect);
void BNO055_ReadMagneter(I2C_HandleTypeDef *I2C,BNO055_t *BNO055,osMutexId I2CControllerProtect);
void BNO055_ReadQua(I2C_HandleTypeDef *I2C,BNO055_t *BNO055,osMutexId I2CControllerProtect);
void BNO055_ReadLina(I2C_HandleTypeDef *I2C,BNO055_t *BNO055,osMutexId I2CControllerProtect);
void BNO055_ReadGrav(I2C_HandleTypeDef *I2C,BNO055_t *BNO055,osMutexId I2CControllerProtect);
uint8_t BNO055_ReadITStatus(I2C_HandleTypeDef * I2C);
void BNO055_EnableAccHighG(I2C_HandleTypeDef *I2C);
void BNO055_EnableHighGAcc(I2C_HandleTypeDef *I2C,char axis);
void BNO055_ClearIntFlag(I2C_HandleTypeDef *I2C);
void BNO055_SetAccelerometerRange(I2C_HandleTypeDef *I2C,bno055_accel_range_t range);
void BNO055_SetAccelerometerUnit(I2C_HandleTypeDef *I2C,bno055_accel_unit_t unit);

void BNO055_ReadEuler_Yaw(I2C_HandleTypeDef *I2C,BNO055_t *BNO055);
void BNO055_ReadEuler_Pitch(I2C_HandleTypeDef *I2C,BNO055_t *BNO055);
void BNO055_ReadEuler_Roll(I2C_HandleTypeDef *I2C,BNO055_t *BNO055);

#endif /* INC_BNO055_H_ */
