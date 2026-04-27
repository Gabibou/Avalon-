/** 
* 
* @file bno055_hal.c
* @date 25/04/2026
* @version  1.0.0
* @note  File use as junction between project and bosh library
*/

#include "components\bno055_hal.h"
#include "main.h"

extern I2C_HandleTypeDef hi2c2;

// Define function pointer location
s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt){
/**
 * @brief Function used to perform I2C bus read from STM32G474RET6 to BNO055 
 * @param dev_addr : The device address of the sensor
 * @param reg_addr : Address of the first register, will data is going to be read
 * @param reg_data : Pointer on data to write
 * @param cnt : The number of byte of data to be read
 */
    return (s8) HAL_I2C_Mem_Read(&hi2c2, dev_addr, reg_addr, 1, reg_data, cnt, 10);
}

s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt){
 /**
 * @brief Function used to perform I2C bus write from STM32G474RET6 to BNO055 
 * @param dev_addr : The device address of the sensor
 * @param reg_addr : Address of the first register, will data is going to be read
 * @param reg_data : Pointer on data to write
 * @param cnt : The number of byte of data to be read
 */
    return (s8) HAL_I2C_Mem_Read(&hi2c2, dev_addr, reg_addr, 1, reg_data, cnt, 10);
}
