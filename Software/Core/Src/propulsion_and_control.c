/*
 * propulsion_and_control.c
 *
 *  Created on: 19 mars 2023
 *      Author: 33768
 */

#include <propulsion_and_control.h>


//To be tested
void PropulsionAndControl_Update(PROPULSION_t *propulsion){

	uint32_t * timer_base_addr_ptr = &(propulsion->timer->Instance->CCR1);

	*(timer_base_addr_ptr + ((propulsion->servo_left_timer_channel-1)*4)) = propulsion->servo_left_timer_val;

	*(timer_base_addr_ptr + ((propulsion->servo_right_timer_channel-1))) = propulsion->servo_right_timer_val;

	*(timer_base_addr_ptr + ((propulsion->esc_timer_channel-1))) = propulsion->esc_timer_val;

	propulsion->update_timer = 0x01;
}

void PropulsionAndControl_Init(PROPULSION_t *propulsion,uint32_t esc_pin,uint32_t esc_port,uint32_t servo_left_pin,uint32_t servo_left_port,uint32_t servo_right_pin,uint32_t servo_right_port,uint32_t esc_channel,uint32_t servo_left_channel,uint32_t servo_right_channel,TIM_HandleTypeDef *timer_entity){

	//Store all pins into the struct
	propulsion->esc_pin = esc_pin;
	propulsion->esc_port = esc_port;
	propulsion->esc_timer_channel = esc_channel;

	propulsion->servo_left_pin = servo_left_pin;
	propulsion->servo_left_port = servo_left_port;
	propulsion->servo_left_timer_channel = servo_left_channel;

	propulsion->servo_right_pin = servo_right_pin;
	propulsion->servo_right_port = servo_right_port;
	propulsion->servo_right_timer_channel = servo_right_channel;

	propulsion->timer = timer_entity;

	//Throttle up at maximum value --> needed to scale the throttle (here the value are btw 3000 and 6000 with 3000 = 0% throttle and 6000 = 100%)
	//HAL_Delay(5000);
	//propulsion->esc_timer_val = MAX_PROPULSION_LEVEL;
	//PropulsionAndControl_Update(propulsion);

	//Throttle down at minimum value --> needed to scale the throttle (here the value are btw 3000 and 6000 with 3000 = 0% throttle and 6000 = 100%)
	//HAL_Delay(5000);
	//propulsion->esc_timer_val = MIN_PROPULSION_LEVEL;
	//PropulsionAndControl_Update(propulsion);

	//put the flaps at minimum
	HAL_Delay(20);
	propulsion->servo_left_timer_val = MIN_LEFT_EXTENSION;
	PropulsionAndControl_Update(propulsion);

	//put the flaps at minimum
	propulsion->servo_right_timer_val = MIN_RIGHT_EXTENSION;
	PropulsionAndControl_Update(propulsion);

	//put the flaps at minimum
	HAL_Delay(1500);
	propulsion->servo_left_timer_val = MAX_LEFT_EXTENSION;
	PropulsionAndControl_Update(propulsion);

	//put the flaps at minimum
	propulsion->servo_right_timer_val = MAX_RIGHT_EXTENSION;
	PropulsionAndControl_Update(propulsion);

	propulsion->esc_timer_val = MAX_RIGHT_EXTENSION;
	PropulsionAndControl_Update(propulsion);

	propulsion->esc_timer_val = MIN_RIGHT_EXTENSION;
	PropulsionAndControl_Update(propulsion);
}



void PropulsionAndControl_EscSetValue(PROPULSION_t *propulsion,float esc_power){
	//esc power represent the esc power in %

	//Use a quick and easy "triangle rule" --> in french its call "regle de trois"
	uint32_t timer_value = ((MAX_PROPULSION_LEVEL*esc_power)/100) + MIN_PROPULSION_LEVEL;

	//Add into the struct the timer value
	propulsion->esc_timer_val = timer_value;

	//Add the update flag to 1
	propulsion->update_timer = 0x00;

	PropulsionAndControl_Update(propulsion);

}

void PropulsionAndControl_LeftServoSetValue(PROPULSION_t *propulsion,float esc_power){
	//esc power represent the esc power in %

	//Use a quick and easy "triangle rule" --> in french its call "regle de trois"
	uint32_t timer_value = (MAX_LEFT_EXTENSION*esc_power) + MIN_LEFT_EXTENSION;

	//Add into the struct the timer value
	propulsion->servo_left_timer_val = timer_value;

	//Add the update flag to 1
	propulsion->update_timer = 0x00;

	PropulsionAndControl_Update(propulsion);

}

void PropulsionAndControl_RightServoSetValue(PROPULSION_t *propulsion,float esc_power){
	//esc power represent the esc power in %

	//Use a quick and easy "triangle rule" --> in french its call "regle de trois"
	uint32_t timer_value = (MAX_RIGHT_EXTENSION*esc_power) + MIN_RIGHT_EXTENSION;

	//Add into the struct the timer value
	propulsion->servo_right_timer_val = timer_value;

	//Add the update flag to 1
	propulsion->update_timer = 0x00;

	PropulsionAndControl_Update(propulsion);

}
