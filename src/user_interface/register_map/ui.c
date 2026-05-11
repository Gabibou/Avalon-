/** 
* 
* @file ui.c
* @date 27/04/2026
* @version  1.0.0
* @note  File use to define the user interface used in the project
*/

#include "user_interface\register_map\ui.h"

user_interface_t ui;

void user_interface_init(void){
    /**
     * @brief Methods use to initialise the user interface.
     */

    /*----- REVISION_STATUS registers -----*/
    ui.REVISION_STATUS.base_address = UI_REVISION_STATUS_BASE_ADDR;
    ui.REVISION_STATUS.access_control = UI_REVISION_STATUS_ACCESS_CONTROL;
    /* FW revision with major.minor.fix patern*/
    ui.REVISION_STATUS.FW_REVISION[0].value[0] = UI_REVISION_STATUS_FW_REVISION_MAJOR;
    ui.REVISION_STATUS.FW_REVISION[1].value[0] = UI_REVISION_STATUS_FW_REVISION_MINOR;
    ui.REVISION_STATUS.FW_REVISION[2].value[0] = UI_REVISION_STATUS_FW_REVISION_FIX;
    /* UI revision with major.minor.fix patern*/
    ui.REVISION_STATUS.UI_REVISION[0].value[0] = UI_REVISION_STATUS_UI_REVISION_MAJOR;
    ui.REVISION_STATUS.UI_REVISION[1].value[0] = UI_REVISION_STATUS_UI_REVISION_MINOR;
    ui.REVISION_STATUS.UI_REVISION[2].value[0] = UI_REVISION_STATUS_UI_REVISION_FIX;

    /* ----- SYSTEM_STATUS registers -----*/
    ui.SYSTEM_STATUS.base_address = UI_SYSTEM_STATUS_BASE_ADDR;
    ui.SYSTEM_STATUS.access_control = UI_SYSTEM_STATUS_ACCESS_CONTROL;
    ui.SYSTEM_STATUS.SYSTEM_FSM.value[0] = UI_SYSTEM_STATUS_SYSTEM_FSM_DEFAULT_VALUE;

    /* ----- PID_CONTROL registers -----*/
    ui.PID_CONTROL.access_control = UI_PID_CONTROL_BASE_ADDR;
    ui.PID_CONTROL.base_address = UI_PID_CONTROL_ACCESS_CONTROL;

    /* ----- COMMAND registers -----*/
    ui.COMMAND.access_control = UI_COMMAND_ACCESS_CONTROL;
    ui.COMMAND.base_address = UI_COMMAND_BASE_ADDR;

    /* ----- SENSOR_CONFIG registers -----*/
    ui.SENSOR_CONFIG.access_control = UI_SENSOR_CONFIG_ACCESS_CONTROL;
    ui.SENSOR_CONFIG.base_address = UI_SENSOR_CONFIG_BASE_ADDR;

    /* ----- DEBUG registers -----*/
    ui.DEBUG.access_control = UI_DEBUG_ACCESS_CONTROL;
    ui.DEBUG.base_address = UI_DEBUG_BASE_ADDR;

    /* ----- ACTUATOR_CONFIG registers -----*/
    ui.ACTUATOR_CONFIG.access_control = UI_ACTUATOR_CFG_ACCESS_CONTROL;
    ui.ACTUATOR_CONFIG.base_address = UI_ACTUATOR_CFG_BASE_ADDR;

    /* ----- FLIGHT_CONTROL registers -----*/
    ui.FLIGHT_CONTROL.access_control = UI_FLIGHT_CTRL_ACCESS_CONTROL;
    ui.FLIGHT_CONTROL.base_address = UI_FLIGHT_CTRL_BASE_ADDR;

    /* ----- ALERTS registers -----*/
    ui.ALERTS.access_control = UI_ALERTS_ACCESS_CONTROL;
    ui.ALERTS.base_address = UI_ALERTS_BASE_ADDR;

    /* ----- MEMORY_CONTROL registers -----*/
    ui.MEMORY_CONTROL.access_control = UI_MEMORY_CTRL_ACCESS_CONTROL;
    ui.MEMORY_CONTROL.base_address = UI_MEMORY_CTRL_BASE_ADDR;

}
    