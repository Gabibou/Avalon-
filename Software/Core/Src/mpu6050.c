/*
 * mpu6050.c
 *
 *  Created on: Feb 16, 2023
 *      Author: Bertet Gabriel
 */

#include <mpu6050.h>


/*Function use to Read only gyroscopic measurement stored into the MPU6050 chip and store it into the MPU6050 entity
*/
uint8_t MPU6050_Init(I2C_HandleTypeDef *I2C,MPU6050_t *MPU6050){
    uint8_t check;
    uint8_t Data;

    // check device ID WHO_AM_I
    HAL_I2C_Mem_Read(I2C, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 10);

    if (check == 104) // 0x68 will be returned by the sensor if everything goes well
    {
        // power management register 0X6B we should write all 0's to wake the sensor up
        Data = 0x00;
        HAL_I2C_Mem_Write(I2C, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, 10);

        // Set DATA RATE of 1KHz by writing SMPLRT_DIV register
        Data = 0x07;
        HAL_I2C_Mem_Write(I2C, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 10);

        // Set accelerometer configuration in ACCEL_CONFIG Register
        // All self test are disabled and the range of accelerator is 2g --> 0x00 or 4g -->0x08
        Data = 0x08;
        HAL_I2C_Mem_Write(I2C, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 10);

        // Set Gyroscopic configuration in GYRO_CONFIG Register
        // self test diabled and gyroscope range-> 250 °/s 0x00 or 	± 500 °/s -->0x08
        Data = 0x08;
        HAL_I2C_Mem_Write(I2C, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 10);
        return 0;
    }
    return 1;
}

void MPU6050_Get_Accel(I2C_HandleTypeDef *I2C,MPU6050_t *MPU6050){
	/*
	 * @Function that read all acceleration value
	 */
	uint8_t rec_data[6];
	int16_t raw_data[3];

	//Read 6 reg starting by the 0x3B
	HAL_I2C_Mem_Read(I2C, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, rec_data, 6, 20);

	//X axis
	raw_data[0] = (int16_t)(rec_data[0]<<8 | rec_data[1]);
	//Y axis
	raw_data[1] = (int16_t)(rec_data[2]<<8 | rec_data[3]);
	//Z axis
	raw_data[2] = (int16_t)(rec_data[4]<<8 | rec_data[5]);

	//Calculating the acceleration real value --> As the range of the sensors goes from -4g to 4g and the value is a 16 bits value
	//Then 65536/8 = 8192
	MPU6050->Accel_x = (raw_data[0]/8192.0);
	MPU6050->Accel_y = (raw_data[1]/8192.0);
	MPU6050->Accel_z = (raw_data[2]/8192.0);
}

void MPU6050_Get_Gyro(I2C_HandleTypeDef *I2C,MPU6050_t *MPU6050){
	uint8_t rec_data[6];
	int16_t raw_data[3];

	HAL_I2C_Mem_Read(I2C, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, rec_data, 6, 20);

	raw_data[0] = (int16_t)(rec_data[0]<<8 | rec_data[1]);
	//Y axis
	raw_data[1] = (int16_t)(rec_data[2]<<8 | rec_data[3]);
	//Z axis
	raw_data[2] = (int16_t)(rec_data[4]<<8 | rec_data[5]);


	//Same as the accel valu --> but +- 500°/s
	MPU6050->gyro_x = (raw_data[0]/65.536);
	MPU6050->gyro_y = (raw_data[1]/65.536);
	MPU6050->gyro_z = (raw_data[2]/65.536);
}


void MPU6050_Sleep(I2C_HandleTypeDef *I2C,MPU6050_t *MPU6050){
	/*
	 * Function use to put the MPU6050 into sleep mode without using temp sensors
	 *
	 */
    uint8_t Data;
	Data = 0x40;
	HAL_I2C_Mem_Write(I2C, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, 10);
	MPU6050->sleep_mode = 1;
}

void MPU6050_Wakeup(I2C_HandleTypeDef *I2C,MPU6050_t *MPU6050){
	/*
	 * @Function use to wake up the sensor
	 */
    uint8_t Data;
	Data = 0x00;
	HAL_I2C_Mem_Write(I2C, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, 10);
	HAL_I2C_Mem_Write(I2C, MPU6050_ADDR, PWR_MGMT_2_REG, 1, &Data, 1, 10);
	MPU6050->sleep_mode = 0;
}

void MPU6050_CycleMode(I2C_HandleTypeDef *I2C,MPU6050_t *MPU6050,uint8_t cycle_mode){
	/*
	 * @Function use to wake up the sensor
	 * @Cylcle mode is defined into the .h file
	 * @More the value is high more the power consumption will be high
	 * cycle_mode could be :
	 * 					@CYCLE_FREQ_1HZ25
	 * 					@CYCLE_FREQ_5HZ
	 * 					@CYCLE_FREQ_20HZ
	 * 					@CYCLE_FREQ_40HZ
	 */
	uint8_t data;

	data = 0x20;
	//Enable the cycle mode
	HAL_I2C_Mem_Write(I2C, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &data, 1, 10);

	//Choose the cycle_mode_frequency
	data = cycle_mode<<6;
	HAL_I2C_Mem_Write(I2C, MPU6050_ADDR, PWR_MGMT_2_REG, 1, &data, 1, 10);

	MPU6050->sleep_mode = 2;
}

void MPU6050_It_Enable(I2C_HandleTypeDef *I2C,MPU6050_t *MPU6050){
	uint8_t data;
	data = 0x90;
	//Change the it pin to push pull + IT pulse = 50us + it active low
	HAL_I2C_Mem_Write(I2C, MPU6050_ADDR, INT_PIN_CFG_REG, 1, &data, 1, 10);

	//Activate the EOC interupt
	data = 0x01;
	HAL_I2C_Mem_Write(I2C, MPU6050_ADDR, INT_ENABLE_REG, 1, &data, 1, 10);

}



