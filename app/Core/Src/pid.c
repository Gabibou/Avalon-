/*(*******************************************************
PROGRAM NAME - PID compensation driver

PROGRAMMER - Bertet Gabriel

SYSTEM - Currently working on a STM32G474RET6

DATE - Started 17/04/2023

BUGS - Small problem (As the mpu6050 doesen't have embedded algorithm  we can't easily find the euler angle).
		For now PID compesate acceleration and not angle (wich doesn't make any sense but done for test only)

DESCRIPTION - Basic driver for the mpu6050 accelerometer.
*******************************************************)*/
#include "main.h"

//As the gyro got the front at x axis then rotation on X mean roll / rotation on Y mean pitch and rotation on Z mean yaw
float32_t Pid_CalculatePitchError(COMMAND_t *command,BNO055_t *Accelerometer){
	float32_t res = (Accelerometer->processed_data.euler_pitch)-(command->pitch_angle);
	return res;
}

float32_t Pid_CalculateYawError(COMMAND_t *command,BNO055_t *Accelerometer){
	float32_t res = (Accelerometer->processed_data.euler_heading)-(command->yaw_angle);
	return res;
}

float32_t Pid_CalculateRollError(COMMAND_t *command,BNO055_t *Accelerometer){
	float32_t res = (Accelerometer->processed_data.euler_roll)-(command->roll_angle);
	return res;
}

void Pid_Init(arm_pid_instance_f32 *PID,float32_t KP,float32_t KI,float32_t KD){

	//Set PID gain
	PID->Kp = KP;
	PID->Ki = KI;
	PID->Kd = KD;

	//Set the PID
	arm_pid_init_f32(PID, 1);

}

//Function use to compensate a roatation on yaw axis --> make sure to update accelerometer before reading and compensate
void Pid_CompensateYaw(arm_pid_instance_f32 *PID,COMMAND_t *command,BNO055_t *Accelerometer,PROPULSION_t *propulsion){
	float32_t error;
	float32_t pid_value;
	//Calculate the erro in °/s around yaw/z axis
	error = Pid_CalculateYawError(command, Accelerometer);
	//calculate the amount of mouve needed to force back the plane
	pid_value = arm_pid_f32(PID, error);

	//Ase the plane is a fixed wing we can't compensate move on yaw axis ...
	//If you have different plane make sure to create a thing to do here
}

//Function use to compensate a roatation on Roll axis --> make sure to update accelerometer before reading and compensate
void Pid_CompensateRoll(arm_pid_instance_f32 *PID,COMMAND_t *command,BNO055_t *Accelerometer,PROPULSION_t *propulsion){

	float32_t error;
	float32_t pid_value;
	//Calculate the erro in °/s around yaw/z axis
	error = Pid_CalculateRollError(command, Accelerometer);
	//calculate the amount of mouve needed to force back the plane
	pid_value = arm_pid_f32(PID, error);
	//If pid value is >0 then we are rolling to the right
	//We need to turn up left and down right

	/*Left flaps up and right flaps down*/


#ifndef SERVO_LEFT_REVERSED
	if((propulsion->servo_left_timer_val + pid_value)>MAX_LEFT_EXTENSION){
		propulsion->servo_left_timer_val = MAX_LEFT_EXTENSION;
	}
	else{
		if((propulsion->servo_left_timer_val + pid_value)<MIN_LEFT_EXTENSION){
			propulsion->servo_left_timer_val = MIN_LEFT_EXTENSION;
		}
		else{
			propulsion->servo_left_timer_val += pid_value;
		}
	}
#endif
#ifndef SERVO_RIGHT_REVERSED
	if((propulsion->servo_right_timer_val - pid_value)>MAX_RIGHT_EXTENSION){
		propulsion->servo_right_timer_val = MAX_RIGHT_EXTENSION;
	}
	else{
		if((propulsion->servo_right_timer_val - pid_value)<MIN_RIGHT_EXTENSION){
			propulsion->servo_right_timer_val = MIN_RIGHT_EXTENSION;
		}
		else{
			propulsion->servo_right_timer_val -= pid_value;
		}
	}
#endif
#ifdef SERVO_RIGHT_REVERSED
	if((propulsion->servo_right_timer_val + pid_value)>MIN_RIGHT_EXTENSION){
		propulsion->servo_right_timer_val = MIN_RIGHT_EXTENSION;
	}
	else{
		if((propulsion->servo_right_timer_val + pid_value)<MAX_RIGHT_EXTENSION){
			propulsion->servo_right_timer_val = MAX_RIGHT_EXTENSION;
		}
		else{
			propulsion->servo_right_timer_val += pid_value;
		}
	}
#endif
#ifdef SERVO_LEFT_REVERSED
	if((propulsion->servo_left_timer_val - pid_value)>MIN_LEFT_EXTENSION){
		propulsion->servo_left_timer_val = MIN_LEFT_EXTENSION;
	}
	else{
		if((propulsion->servo_left_timer_val - pid_value)<MAX_LEFT_EXTENSION){
			propulsion->servo_left_timer_val = MAX_LEFT_EXTENSION;
		}
		else{
			propulsion->servo_left_timer_val -= pid_value;
		}
	}
#endif

	PropulsionAndControl_UpdateLeftFlaps(propulsion);
	PropulsionAndControl_UpdateRightFlaps(propulsion);

}
//Function use to compensate a roatation on Pitch axis --> make sure to update accelerometer before reading and compensate
void Pid_CompensatePitch(arm_pid_instance_f32 *PID,COMMAND_t *command,BNO055_t *Accelerometer,PROPULSION_t *propulsion){

	float32_t error;
	float32_t pid_value;
	//Calculate the erro in °/s around yaw/z axis
	error = Pid_CalculatePitchError(command, Accelerometer);
	//calculate the amount of mouve needed to force back the plane
	pid_value = arm_pid_f32(PID, error);


#ifndef SERVO_LEFT_REVERSED
	if((propulsion->servo_left_timer_val + pid_value)>MAX_LEFT_EXTENSION){
		propulsion->servo_left_timer_val = MAX_LEFT_EXTENSION;
	}
	else{
		if((propulsion->servo_left_timer_val + pid_value)<MIN_LEFT_EXTENSION){
			propulsion->servo_left_timer_val = MIN_LEFT_EXTENSION;
		}
		else{
			propulsion->servo_left_timer_val += pid_value;
		}
	}
#endif
#ifndef SERVO_RIGHT_REVERSED
	if((propulsion->servo_right_timer_val + pid_value)>MAX_RIGHT_EXTENSION){
		propulsion->servo_right_timer_val = MAX_RIGHT_EXTENSION;
	}
	else{
		if((propulsion->servo_right_timer_val + pid_value)<MIN_RIGHT_EXTENSION){
			propulsion->servo_right_timer_val = MIN_RIGHT_EXTENSION;
		}
		else{
			propulsion->servo_right_timer_val += pid_value;
		}
	}
#endif
#ifdef SERVO_RIGHT_REVERSED
	if((propulsion->servo_right_timer_val - pid_value)>MIN_RIGHT_EXTENSION){
		propulsion->servo_right_timer_val = MIN_RIGHT_EXTENSION;
	}
	else{
		if((propulsion->servo_right_timer_val - pid_value)<MAX_RIGHT_EXTENSION){
			propulsion->servo_right_timer_val = MAX_RIGHT_EXTENSION;
		}
		else{
			propulsion->servo_right_timer_val -= pid_value;
		}
	}
#endif
#ifdef SERVO_LEFT_REVERSED
	if((propulsion->servo_left_timer_val - pid_value)>MIN_LEFT_EXTENSION){
		propulsion->servo_left_timer_val = MIN_LEFT_EXTENSION;
	}
	else{
		if((propulsion->servo_left_timer_val - pid_value)<MAX_LEFT_EXTENSION){
			propulsion->servo_left_timer_val = MAX_LEFT_EXTENSION;
		}
		else{
			propulsion->servo_left_timer_val -= pid_value;
		}
	}
#endif


	PropulsionAndControl_UpdateLeftFlaps(propulsion);
	PropulsionAndControl_UpdateRightFlaps(propulsion);

}
