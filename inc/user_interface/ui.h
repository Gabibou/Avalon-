/** 
* 
* @file ui.h
* @date 27/04/2026
* @version  1.0.0
* @note  File use to define the user interface used in the project
*/
#ifndef INC_UI_H_
#define INC_UI_H_

// --------------------------------------------- INCLUDE ---------------------------------------------

#include "ui_register.h"
#include "stdio.h"

// --------------------------------------------- CONFIG ---------------------------------------------

#define UI_REVISION_MAJOR 0x01
#define UI_REVISION_MINOR 0x00
#define UI_REVISION_PATCH 0x00

// --------------------------------------------- TYPEDEF ---------------------------------------------

/* Define all types of register access available*/
typedef enum{
    READ_ONLY,
    WRITE_ONLY,
    READ_WRITE,
}ui_register_access_control_t;

/*Define all types of register available*/
typedef struct{
    uint8_t value[1];
    uint16_t address;
}ui_8bits_register_t;

typedef struct{
    uint8_t value[2];
    uint16_t address;
}ui_16bits_register_t;

typedef struct{
    uint8_t value[4];
    uint16_t address;
}ui_32bits_register_t;

/*Define REVISION_STATUS_UI*/
typedef struct{
    uint32_t base_address;
    ui_register_access_control_t access_control;
    ui_32bits_register_t DEVICE_ID;
    ui_32bits_register_t HW_REVISION;
    ui_8bits_register_t FW_REVISION[3];
    ui_8bits_register_t UI_REVISION[3];
}revision_status_ui_t;

/*Define SYSTEM_STATUS*/
typedef struct{
    uint32_t base_address;
    ui_register_access_control_t access_control;
    ui_8bits_register_t SYSTEM_FSM;
    ui_8bits_register_t FLIGHT_MODE;
    ui_16bits_register_t SYSTEM_WARNINGS;
    ui_16bits_register_t SYSTEM_ERRORS;
}system_status_ui_t;

/*Define PID_CONTROL*/
typedef struct{
    uint32_t base_address;
    ui_register_access_control_t access_control;
    ui_32bits_register_t ROLL_KP;
    ui_32bits_register_t ROLL_KI;
    ui_32bits_register_t ROLL_KD;
    ui_32bits_register_t PITCH_KP;
    ui_32bits_register_t PITCH_KI;
    ui_32bits_register_t PITCH_KD;
    ui_32bits_register_t YAW_KP;
    ui_32bits_register_t YAW_KI;
    ui_32bits_register_t YAW_KD;
}pid_control_ui_t;

/*Define COMMAND*/
typedef struct{
    uint32_t base_address;
    ui_register_access_control_t access_control;
    ui_8bits_register_t COMMAND;
    ui_8bits_register_t COMMAND_ASSERTION;
}command_ui_t;

/*Define SENSOR_CONFIG*/
// TBD --> TOUT ce qui est lié a la config des sensors
typedef struct{
    uint32_t base_address;
    ui_register_access_control_t access_control;
    ui_8bits_register_t TBD;
}sensor_cfg_ui_t;

/*Define DEBUG*/
// TBD --> TOUTes les options de debug possible genre PWM OVERWRITE
typedef struct{
    uint32_t base_address;
    ui_register_access_control_t access_control;
    ui_8bits_register_t TBD;
}debug_ui_t;

/*Define ACTUATOR_CONFIG*/
// TBD --> TOUT ce qui est lié a la config des moteurs et flaps
typedef struct{
    uint32_t base_address;
    ui_register_access_control_t access_control;
    ui_8bits_register_t TBD;
}actuator_config_ui_t;

/*Define FLIGHT_CONTROL*/
// TBD --> TOUT ce qui est lié au vol automatique ou manuel --> ICI c'est vraiment les commandes de vols
typedef struct{
    uint32_t base_address;
    ui_register_access_control_t access_control;
    ui_8bits_register_t TBD;
}flight_control_ui_t;

/*Define ALERTS*/
// TBD --> tout les flags possiblent et imaginable
typedef struct{
    uint32_t base_address;
    ui_register_access_control_t access_control;
    ui_8bits_register_t TBD;
}alerts_ui_t;

/*Define MEMORY*/
// TBD --> tout ce qui est lié a la mémoire
typedef struct{
    uint32_t base_address;
    ui_register_access_control_t access_control;
    ui_8bits_register_t TBD;
}memory_ui_t;

// Main user interface
typedef struct{
    revision_status_ui_t REVISION_STATUS;
    system_status_ui_t SYSTEM_STATUS;
    pid_control_ui_t PID_CONTROL;
    command_ui_t COMMAND;
    sensor_cfg_ui_t SENSOR_CONFIG;
    debug_ui_t DEBUG;
    actuator_config_ui_t ACTUATOR_CONFIG;
    flight_control_ui_t FLIGHT_CONTROL;
    alerts_ui_t ALERTS;
    memory_ui_t MEMORY_CONTROL;
}user_interface_t;

#endif /*INC_UI_H_*/