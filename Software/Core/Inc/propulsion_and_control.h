/*
 * propulsion_and_control.h
 *
 *  Created on: 19 mars 2023
 *      Author: 33768
 */

#ifndef INC_PROPULSION_AND_CONTROL_H_
#define INC_PROPULSION_AND_CONTROL_H_
//---------------------------------------------------------------- INCLUDE  ----------------------------------------------------------------
#include "stm32g4xx_hal.h"
//---------------------------------------------------------------- TYPE DEF ----------------------------------------------------------------

//Set all maximum propulsion and extension value until which the aicraft will crash
//this values will be use to calculate next trajectory or travel time
#define MAX_PROPULSION_LEVEL 6000
#define MAX_LEFT_EXTENSION 6000
#define MAX_RIGHT_EXTENSION 5000
#define MIN_PROPULSION_LEVEL 3000
#define MIN_LEFT_EXTENSION 3000
#define MIN_RIGHT_EXTENSION 4000

typedef struct{

	//Pin definition
	uint32_t servo_left_pin;
	uint32_t servo_left_port;
	uint32_t servo_right_pin;
	uint32_t servo_right_port;
	uint32_t esc_pin;
	uint32_t esc_port;

	//Usefull to know which timer is in use
	TIM_HandleTypeDef *timer;

	//Needed to know which channel is currently connected to every pin
	uint8_t esc_timer_channel;
	uint8_t servo_left_timer_channel;
	uint8_t servo_right_timer_channel;

	//Timer value for each actuator
	uint32_t esc_timer_val;
	uint32_t servo_left_timer_val;
	uint32_t servo_right_timer_val;

	//if this value is set to '1' this mean that the timer is actually generating this list of value
	//Else you will need to update using the update fuction
	uint8_t update_timer;

}PROPULSION_t;

//---------------------------------------------------------------- PROTOTYPE ----------------------------------------------------------------
void PropulsionAndControl_Init(PROPULSION_t *propulsion,uint32_t esc_pin,uint32_t esc_port,uint32_t servo_left_pin,uint32_t servo_left_port,uint32_t servo_right_pin,uint32_t servo_right_port,uint32_t esc_channel,uint32_t servo_left_channel,uint32_t servo_right_channel,TIM_HandleTypeDef *timer_entity);
void PropulsionAndControl_Update(PROPULSION_t *propulsion);
void PropulsionAndControl_EscSetValue(PROPULSION_t *propulsion,float esc_power);
void PropulsionAndControl_RightServoSetValue(PROPULSION_t *propulsion,float esc_power);
void PropulsionAndControl_LeftServoSetValue(PROPULSION_t *propulsion,float esc_power);
#endif /* INC_PROPULSION_AND_CONTROL_H_ */
