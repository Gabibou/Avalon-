/** 
* 
* @file components.c
* @date 25/04/2026
* @version  0.0.1
* @note  File use as junction between every components hardware abstraction layer and main firmware
*/

#include "components/components.h"

/******************************/
/*  GLOBAL COMPONENTS STRUCT  */
/******************************/
struct bno055_t bno055;

components_init_status_t init_embedded_components(void){
    /**
    * @brief Initialise all embedded hardware components available
    */
    components_init_status_t init_status = COMPONENTS_INIT_OK;

    // ---------- IMU ----------
    init_status = boot_imu();

    return init_status;
}

components_init_status_t boot_imu(void){
    /**
     * @brief Initialise the inertial measurement unit 
     */

    components_init_status_t imu_init_status = COMPONENTS_INIT_OK;
    u8 mcu_self_test_p = 0x0;
    u8 accel_self_test_p = 0x0;
    u8 gyro_self_test_p = 0x0;
    u8 mag_self_test_p = 0x0;

    // Define function pointer for I2C bus access 
    bno055.bus_read = BNO055_I2C_bus_read;
    bno055.bus_write = BNO055_I2C_bus_write;
    bno055.dev_addr = BNO055_I2C_ADDR;
    
    // Check I2C communication and retrieve chipID, revisionID, firmwareID etc
    if(bno055_init(&bno055) != 0x0){
        imu_init_status = COMPONENTS_INIT_I2C_COMMUNICATION_FAILED;    
    }
    // Trigger self
    bno055_set_selftest(0x01);
    // Retrieve self test result
    if(bno055_get_selftest_mcu(&mcu_self_test_p) != 0x0 || bno055_get_selftest_accel(&accel_self_test_p) != 0x0 ||
       bno055_get_selftest_gyro(&gyro_self_test_p) != 0x0 || bno055_get_selftest_mag(&mag_self_test_p) != 0x0){
        imu_init_status = COMPONENTS_INIT_I2C_COMMUNICATION_FAILED;        
    }
    if(mcu_self_test_p != 0x1 || accel_self_test_p != 0x1 || gyro_self_test_p != 0x1 || mag_self_test_p != 0x1){
        imu_init_status = COMPONENTS_INIT_IMU_SELF_TEST_FAILED;
    }
    return imu_init_status;
}

