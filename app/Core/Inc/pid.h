/*
 * pid.h
 *
 *  Created on: 5 avr. 2023
 *      Author: 33768
 */

#ifndef SRC_PID_H_
#define SRC_PID_H_

#include "stm32g4xx_hal.h"
#include "arm_math.h"
#include "bno055.h"
#include <propulsion_and_control.h>

//------------------------------------------- PID SETTINGS -------------------------------------------
#define PID_KP_SPEED 0.3
#define PID_KI_SPEED 10
#define PID_KD_SPEED 0

#define PID_KP_ALTITUDE 100
#define PID_KI_ALTITUDE 0.025
#define PID_KD_ALTITUDE 20

#define PID_KP_PITCH 100
#define PID_KI_PITCH 0.025
#define PID_KD_PITCH 20

#define PID_KP_YAW 100
#define PID_KI_YAW 0.025
#define PID_KD_YAW 20

#define PID_KP_ROLL -0.13256588114745735
#define PID_KI_ROLL -0.02802730408540254
#define PID_KD_ROLL -0.0865905458952401



//------------------------------------------- COMMAND STRUCT -------------------------------------------

//Struct use to send command to the drone. If a value is set here then the drone will automatically try to do it
typedef struct{

	float speed;

	float altitude;

	float pitch_angle;

	float yaw_angle;

	float roll_angle;

}COMMAND_t;

//------------------------------------------- FUNTION PROTOTYPE -------------------------------------------
void Pid_Init(arm_pid_instance_f32 *PID,float32_t KP,float32_t KI,float32_t KD);
float32_t Pid_CalculateRollError(COMMAND_t *command,BNO055_t *Accelerometer);
float32_t Pid_CalculateYawError(COMMAND_t *command,BNO055_t *Accelerometer);
float32_t Pid_CalculatePitchError(COMMAND_t *command,BNO055_t *Accelerometer);
float32_t Pid_CalculateYaw(arm_pid_instance_f32 *PID,float32_t error);
void Pid_CompensateYaw(arm_pid_instance_f32 *PID,COMMAND_t *command,BNO055_t *Accelerometer,PROPULSION_t *propulsion);
void Pid_CompensateRoll(arm_pid_instance_f32 *PID,COMMAND_t *command,BNO055_t *Accelerometer,PROPULSION_t *propulsion);
void Pid_CompensatePitch(arm_pid_instance_f32 *PID,COMMAND_t *command,BNO055_t *Accelerometer,PROPULSION_t *propulsion);
#endif /* SRC_PID_H_ */
