/*(*******************************************************
PROGRAM NAME - Propulsion driver

PROGRAMMER - Bertet Gabriel

SYSTEM - Currently working on a STM32G474RET6 with any servo motor and esc

DATE - Started 07/04/2023

BUGS - No major bug ever reported. If you find a bug please report it on github

DESCRIPTION - Driver for 2 servo motor and 1 ESC. It generate a pwm at 50hz on 3 different channel
*******************************************************)*/
#include <propulsion_and_control.h>


void PropulsionAndControl_UpdateESC(PROPULSION_t *propulsion){
	uint32_t * timer_base_addr_ptr = &(propulsion->timer->Instance->CCR1);
	*(timer_base_addr_ptr + ((propulsion->esc_timer_channel-1))) = propulsion->esc_timer_val;
	propulsion->update_timer = 0x01;
}
void PropulsionAndControl_UpdateLeftFlaps(PROPULSION_t *propulsion){
	uint32_t * timer_base_addr_ptr = &(propulsion->timer->Instance->CCR1);
	*(timer_base_addr_ptr + ((propulsion->servo_left_timer_channel-1))) = propulsion->servo_left_timer_val;
	propulsion->update_timer = 0x01;
}
void PropulsionAndControl_UpdateRightFlaps(PROPULSION_t *propulsion){
	uint32_t * timer_base_addr_ptr = &(propulsion->timer->Instance->CCR1);
	*(timer_base_addr_ptr + ((propulsion->servo_right_timer_channel-1))) = propulsion->servo_right_timer_val;
	propulsion->update_timer = 0x01;
}

void PropulsionAndControl_Init(PROPULSION_t *propulsion,uint32_t esc_pin,GPIO_TypeDef *esc_port,uint32_t servo_left_pin,GPIO_TypeDef *servo_left_port,uint32_t servo_right_pin,GPIO_TypeDef *servo_right_port,uint32_t esc_channel,uint32_t servo_left_channel,uint32_t servo_right_channel,TIM_HandleTypeDef *timer_entity){

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

	//put the flaps at minimum
	osDelay(20);
	propulsion->servo_left_timer_val = MIN_LEFT_EXTENSION;
	PropulsionAndControl_UpdateLeftFlaps(propulsion);

	//put the flaps at minimum
	propulsion->servo_right_timer_val = MIN_RIGHT_EXTENSION;
	PropulsionAndControl_UpdateRightFlaps(propulsion);

	//put the flaps at maximum
	osDelay(1500);
	propulsion->servo_left_timer_val = MAX_LEFT_EXTENSION;
	PropulsionAndControl_UpdateLeftFlaps(propulsion);

	//put the flaps at maximum
	propulsion->servo_right_timer_val = MAX_RIGHT_EXTENSION;
	PropulsionAndControl_UpdateRightFlaps(propulsion);

	osDelay(1500);
	propulsion->servo_left_timer_val = (MAX_LEFT_EXTENSION+MIN_LEFT_EXTENSION)/2;
	PropulsionAndControl_UpdateLeftFlaps(propulsion);

	//put the flaps at minimum
	propulsion->servo_right_timer_val = (MAX_RIGHT_EXTENSION+MIN_RIGHT_EXTENSION)/2;
	PropulsionAndControl_UpdateRightFlaps(propulsion);
}



void PropulsionAndControl_EscSetValue(PROPULSION_t *propulsion,float esc_power){
	//esc power represent the esc power in %

	//Use a quick and easy "triangle rule" --> in french its call "regle de trois" or y = ax+b coeef equal --> Min/100
	uint32_t timer_value = ((MIN_PROPULSION_LEVEL/100)*esc_power) + MIN_PROPULSION_LEVEL;

	//Add into the struct the timer value
	propulsion->esc_timer_val = timer_value;

	//Add the update flag to 1
	propulsion->update_timer = 0x00;

	PropulsionAndControl_Update(propulsion);

}

void PropulsionAndControl_LeftServoSetValue(PROPULSION_t *propulsion,float servo_extension){
	//esc power represent the esc power in %

	//Use a quick and easy "triangle rule" --> in french its call "regle de trois"
	uint32_t timer_value = ((MIN_LEFT_EXTENSION/100)*servo_extension) + MIN_LEFT_EXTENSION;

	//Add into the struct the timer value
	propulsion->servo_left_timer_val = timer_value;

	//Add the update flag to 1
	propulsion->update_timer = 0x00;

	PropulsionAndControl_Update(propulsion);

}

void PropulsionAndControl_RightServoSetValue(PROPULSION_t *propulsion,float servo_extension){
	//esc power represent the esc power in %

	//Use a quick and easy "triangle rule" --> in french its call "regle de trois"
	uint32_t timer_value = ((MIN_RIGHT_EXTENSION/100)*servo_extension) + MIN_RIGHT_EXTENSION;

	//Add into the struct the timer value
	propulsion->servo_right_timer_val = timer_value;

	//Add the update flag to 1
	propulsion->update_timer = 0x00;

	PropulsionAndControl_Update(propulsion);

}
