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
#define MAX_PROPULSION_LEVEL 7200
#define MIN_PROPULSION_LEVEL 3000

#define MAX_LEFT_EXTENSION 3500
#define MIN_LEFT_EXTENSION 1300

/*The servo motor is mounted reverse so i have to swithc max/min*/
#define MAX_RIGHT_EXTENSION 1300	//3500
#define MIN_RIGHT_EXTENSION 3500	//1300

/*Uncomment if the flaps are mounted in reverse
 * @You will also have to flip MAX and MIN for the reversed servo motor
 * */
#define SERVO_RIGHT_REVERSED
//#define SERVO_LEFT_REVERSED

/*GPIO definition for PWM generation*/
#define ESC_GPIO_PORT GPIOB
#define ESC_GPIO_PIN GPIO_PIN_9
#define ESC_TIMER_CHANNEL_NBR 4
#define SERVO_LEFT_GPIO_PIN GPIO_PIN_6
#define SERVO_LEFT_GPIO_PORT GPIOB
#define SERVO_LEFT_TIMER_CHANNEL_NBR 1
#define SERVO_RIGHT_GPIO_PIN GPIO_PIN_7
#define SERVO_RIGHT_GPIO_PORT GPIOB
#define SERVO_RIGHT_TIMER_CHANNEL_NBR 2

typedef struct{

	//Pin definition
	uint32_t servo_left_pin;
	GPIO_TypeDef *servo_left_port;
	uint32_t servo_right_pin;
	GPIO_TypeDef *servo_right_port;
	uint32_t esc_pin;
	GPIO_TypeDef *esc_port;

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
void PropulsionAndControl_Init(PROPULSION_t *propulsion,uint32_t esc_pin,GPIO_TypeDef *esc_port,uint32_t servo_left_pin,GPIO_TypeDef *servo_left_port,uint32_t servo_right_pin,GPIO_TypeDef *servo_right_port,uint32_t esc_channel,uint32_t servo_left_channel,uint32_t servo_right_channel,TIM_HandleTypeDef *timer_entity);
void PropulsionAndControl_UpdateESC(PROPULSION_t *propulsion);
void PropulsionAndControl_UpdateRightFlaps(PROPULSION_t *propulsion);
void PropulsionAndControl_UpdateLeftFlaps(PROPULSION_t *propulsion);
void PropulsionAndControl_EscSetValue(PROPULSION_t *propulsion,float esc_power);
void PropulsionAndControl_RightServoSetValue(PROPULSION_t *propulsion,float esc_power);
void PropulsionAndControl_LeftServoSetValue(PROPULSION_t *propulsion,float esc_power);
#endif /* INC_PROPULSION_AND_CONTROL_H_ */
