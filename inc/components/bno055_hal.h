/** 
* 
* @file bno055_hal.h
* @date 25/04/2026
* @version  1.0.0
* @note  File use as junction between project and bosh library
*/
#ifndef INC_BNO055_HAL_
#define INC_BNO055_HAL_

//---------------------------------------------------------------- CONFIGURATION ----------------------------------------------------------------

#define BNO055_I2C_ADDR 0x50	//Can also be 0x29 if the COM3 pin is connected to VCC

//---------------------------------------------------------------- INCLUDE ----------------------------------------------------------------

#include "..\..\lib\components_driver\BNO055_SensorAPI\bno055.h"

//---------------------------------------------------------------- PROTOTYPE ----------------------------------------------------------------

s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);

#endif /* INC_BNO055_HAL_ */