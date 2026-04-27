/** 
* 
* @file components.h
* @date 25/04/2026
* @version  0.0.1
* @note  File use as junction between every components hardware abstraction layer and main firmware
*/

#include "components/bno055_hal.h"

// Define all on board components initialisation status available
typedef enum{
    COMPONENTS_INIT_OK,
    COMPONENTS_INIT_SPI_COMMUNICATION_FAILED,
    COMPONENTS_INIT_I2C_COMMUNICATION_FAILED,
    COMPONENTS_INIT_IMU_SELF_TEST_FAILED,
}components_init_status;

/**************/ 
/* PROTOTYPES */
/**************/ 

components_init_status init_embedded_components(void);
components_init_status boot_imu(void);