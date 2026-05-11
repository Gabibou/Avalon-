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
    READ_ONLY,      /*Register section read only*/
    WRITE_ONLY,     /*Register section write only*/
    READ_WRITE,     /*Register section read write*/
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
    uint32_t base_address;                          /*Register section base address*/
    ui_register_access_control_t access_control;    /*Register section access control*/
    ui_32bits_register_t DEVICE_ID;                 /*STM32G7474RET6 IC iD*/
    ui_32bits_register_t HW_REVISION;               /*Board revision ID store in embedded memory*/
    ui_8bits_register_t FW_REVISION[3];             /*Firmware revision Major.Minor.Fix*/
    ui_8bits_register_t UI_REVISION[3];             /*UI revision Major.Minor.Fix*/
}revision_status_ui_t;

/*Define SYSTEM_STATUS*/
typedef struct{
    uint32_t base_address;                          /*Register section base address*/
    ui_register_access_control_t access_control;    /*Register section access control*/
    ui_8bits_register_t SYSTEM_FSM;                 /*Firmware state machine COLD_BOOT-WARM_BOOT-IDLE-ARMED-FLIGHT-FAILSAFE*/
    ui_8bits_register_t FLIGHT_MODE;                /*Flight state machine MANUAL-STABILIZED-AUTO*/
    ui_16bits_register_t SYSTEM_WARNINGS;           /*Warning raised*/
    ui_16bits_register_t SYSTEM_ERRORS;             /*Error raised*/
}system_status_ui_t;

/*Define PID_CONTROL*/
typedef struct{
    uint32_t base_address;                          /*Register section base address*/
    ui_register_access_control_t access_control;    /*Register section access control*/            
    ui_32bits_register_t ROLL_KP;                   /*KP factor used for PIP on roll axis*/
    ui_32bits_register_t ROLL_KI;                   /*KI factor used for PIP on roll axis*/
    ui_32bits_register_t ROLL_KD;                   /*KD factor used for PIP on roll axis*/
    ui_32bits_register_t PITCH_KP;                  /*KP factor used for PIP on pitch axis*/
    ui_32bits_register_t PITCH_KI;                  /*KI factor used for PIP on pitch axis*/
    ui_32bits_register_t PITCH_KD;                  /*KD factor used for PIP on pitch axis*/
    ui_32bits_register_t YAW_KP;                    /*KP factor used for PIP on yaw axis*/
    ui_32bits_register_t YAW_KI;                    /*KI factor used for PIP on yaw axis*/
    ui_32bits_register_t YAW_KD;                    /*KD factor used for PIP on yaw axis*/
}pid_control_ui_t;

/*Define COMMAND*/
typedef struct{ 
    uint32_t base_address;                          /*Register section base address*/
    ui_register_access_control_t access_control;    /*Register section access control*/  
    ui_8bits_register_t COMMAND;                    /*Command sent to the board controller*/
    ui_8bits_register_t COMMAND_ASSERTION;          /*Command assertion flag raised when command has been processed*/
}command_ui_t;

/*Define SENSOR_CONFIG*/
/* IMU configuration*/
typedef struct{
    ui_8bits_register_t ACC_FREQUENCY;                  /*Define BNO_055 acceleration reading frequency*/
    ui_8bits_register_t GYR_FREQUENCY;                  /*Define BNO_055 gyroscope reading frequency*/
    ui_8bits_register_t MAG_FREQUENCY;                  /*Define BNO_055 magnetometer reading frequency*/
    ui_8bits_register_t ACC_POWER_ON;                   /* Accelerometer power ON/OFF */
    ui_8bits_register_t GYR_POWER_ON;                   /* Gyroscope power ON/OFF */
    ui_8bits_register_t MAG_POWER_ON;                   /* Magnetometer power ON/OFF */     
    ui_8bits_register_t AXIS_REMAP;                     /* Remap axis X-Y-Z */ 
    ui_8bits_register_t ACC_RANGE;                      /* Define acceleration reading range */ 
    ui_8bits_register_t GYR_RANGE;                      /* Define magnetometer reading range */ 
    ui_8bits_register_t MAG_RANGE;                      /* Define gyroscope reading range */
    ui_8bits_register_t ACC_LOW_PASS_FILTER;            /* Define acceleration reading low pass filter */  
    ui_8bits_register_t GYR_LOW_PASS_FILTER;            /* Define gyroscope reading low pass filter */  
    ui_8bits_register_t ACC_HIGH_G_DETECTION_ENABLE;    /* Define if HIGH G interrupt is enable */
    ui_16bits_register_t ACC_HIGH_G_DETECTION_TIME_MS;  /* Define gyroscope reading low pass filter */
    ui_8bits_register_t ACC_HIGH_G_DETECTION_THRESHOLD; /* Define the amount of G needed to trigger high G */
}sensor_config_bno055_t;

/* GPS configuration*/
typedef struct{
    ui_8bits_register_t MIN_SAT_IN_RANGE;                /* Define the minimum amount of sattelite in range before flight */
}sensor_config_l80_t;

// TBD --> TOUT ce qui est lié a la config des sensors
typedef struct{
    uint32_t base_address;
    ui_register_access_control_t access_control;
    sensor_config_bno055_t BNO055;
    sensor_config_l80_t L80;
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

/* ---------- PROTOTYPE ----------*/
void user_interface_init(void);

#endif /*INC_UI_H_*/