/*
 * mpu6050.h
 *
 *  Created on: Feb 16, 2023
 *      Author: 33768
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_
//---------------------------------------------------------------- INCLUDE  ----------------------------------------------------------------
#include "stm32g4xx_hal.h"

//---------------------------------------------------------------- DEFINE ----------------------------------------------------------------

//REG @ of the MPU6050
#define WHO_AM_I_REG 0x75
#define PWR_MGMT_1_REG 0x6B
#define PWR_MGMT_2_REG 0x6C
#define SMPLRT_DIV_REG 0x19
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_CONFIG_REG 0x1B
#define GYRO_XOUT_H_REG 0x43
#define INT_ENABLE_REG 0x38
#define INT_PIN_CFG_REG 0x37

//MPU6050 I2C @
#define MPU6050_ADDR 0xD0

//Cycle time frequency
#define CYCLE_FREQ_1HZ25 0x00
#define CYCLE_FREQ_5HZ 0x01
#define CYCLE_FREQ_20HZ 0x02
#define CYCLE_FREQ_40HZ 0x03



//---------------------------------------------------------------- TYPE DEF ----------------------------------------------------------------
typedef struct{

	//calculated acceleration value on different axis (unit g)
	float Accel_x;
	float Accel_y;
	float Accel_z;

	//calculated gyroscope value on different axis (unit °/s)
	float gyro_x;
	float gyro_y;
	float gyro_z;

	/*
	 * @define the sleep mode of the sensors
	 * If 0 then wake up mode
	 * If 1 then sleep mode
	 * If 2 then cycle mode (wake up at regular interval to gather data then goes down into sleep mode)
	 */
	uint8_t sleep_mode;


}MPU6050_t;


//---------------------------------------------------------------- PROTOTYPE ----------------------------------------------------------------

uint8_t MPU6050_Init(I2C_HandleTypeDef *I2C,MPU6050_t *MPU6050);
void MPU6050_Sleep(I2C_HandleTypeDef *I2C,MPU6050_t *MPU6050);
void MPU6050_Wakeup(I2C_HandleTypeDef *I2C,MPU6050_t *MPU6050);
void MPU6050_CycleMode(I2C_HandleTypeDef *I2C,MPU6050_t *MPU6050,uint8_t cycle_mode);
void MPU6050_Get_Accel(I2C_HandleTypeDef *I2C,MPU6050_t *MPU6050);
void MPU6050_Get_Gyro(I2C_HandleTypeDef *I2C,MPU6050_t *MPU6050);
void MPU6050_It_Enable(I2C_HandleTypeDef *I2C,MPU6050_t *MPU6050);


#endif /* INC_MPU6050_H_ */
