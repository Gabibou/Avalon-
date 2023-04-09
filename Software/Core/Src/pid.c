/*
 * pid.c
 *
 *  Created on: 5 avr. 2023
 *      Author: 33768
 */

#include "pid.h"
#include "main.h"
#include "arm_math.h"


//As the gyro got the front at x axis then rotation on X mean roll / rotation on Y mean pitch and rotation on Z mean yaw
float Pid_CalculatePitchError(COMMAND_t *command,MPU6050_t *Accelerometer){
	float32_t res = (Accelerometer->gyro_y)-(command->pitch_angle);
	return res;
}

float Pid_CalculateYawError(COMMAND_t *command,MPU6050_t *Accelerometer){
	float32_t res = (Accelerometer->gyro_z)-(command->yaw_angle);
	return res;
}

float Pid_CalculateRolError(COMMAND_t *command,MPU6050_t *Accelerometer){
	float32_t res = (Accelerometer->gyro_x)-(command->roll_angle);
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
void Pid_CompensateYaw(arm_pid_instance_f32 *PID,COMMAND_t *command,MPU6050_t *Accelerometer,PROPULSION_t *propulsion){
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
void Pid_CompensateRoll(arm_pid_instance_f32 *PID,COMMAND_t *command,MPU6050_t *Accelerometer,PROPULSION_t *propulsion){

	float32_t error;
	float32_t pid_value;
	//Calculate the erro in °/s around yaw/z axis
	error = Pid_CalculateRolError(command, Accelerometer);
	//calculate the amount of mouve needed to force back the plane
	pid_value = arm_pid_f32(PID, error);

	//If pid value is >0 then we are rolling to the right
	//We need to turn up left and down right

	if(pid_value>0){

		//Set a maximum value
		if(propulsion->servo_left_timer_val+=pid_value > MAX_LEFT_EXTENSION){
			propulsion->servo_left_timer_val = MAX_LEFT_EXTENSION;
		}
		if(propulsion->servo_right_timer_val-=pid_value < MIN_RIGHT_EXTENSION){
			propulsion->servo_right_timer_val = MIN_RIGHT_EXTENSION;
		}

		propulsion->servo_left_timer_val+=pid_value;			//Left elevon up
		propulsion->servo_right_timer_val-=pid_value;			//Right elevon down
	}
	else{

		//Set a maximum value
		if(propulsion->servo_left_timer_val-=pid_value < MIN_LEFT_EXTENSION){
			propulsion->servo_left_timer_val = MIN_LEFT_EXTENSION;
		}
		if(propulsion->servo_right_timer_val +=pid_value > MAX_RIGHT_EXTENSION){
			propulsion->servo_right_timer_val = MAX_RIGHT_EXTENSION;
		}

		propulsion->servo_left_timer_val-=pid_value;			//Left elevon down
		propulsion->servo_right_timer_val+=pid_value;			//Right elevon up
	}

	PropulsionAndControl_Update(propulsion);

}
//Function use to compensate a roatation on Pitch axis --> make sure to update accelerometer before reading and compensate
void Pid_CompensatePitch(arm_pid_instance_f32 *PID,COMMAND_t *command,MPU6050_t *Accelerometer,PROPULSION_t *propulsion){

	float32_t error;
	float32_t pid_value;
	//Calculate the erro in °/s around yaw/z axis
	error = Pid_CalculatePitchError(command, Accelerometer);
	//calculate the amount of mouve needed to force back the plane
	pid_value = arm_pid_f32(PID, error);

	//If pid value is >0 then we are rolling to the right
	//We need to turn up left and down right

	if(pid_value>0){

		//Set a maximum value
		if(propulsion->servo_left_timer_val-=pid_value < MIN_LEFT_EXTENSION){
			propulsion->servo_left_timer_val = MIN_LEFT_EXTENSION;
		}
		if(propulsion->servo_right_timer_val-=pid_value < MIN_RIGHT_EXTENSION){
			propulsion->servo_right_timer_val = MIN_RIGHT_EXTENSION;
		}

		propulsion->servo_left_timer_val-=pid_value;			//Left elevon down
		propulsion->servo_right_timer_val-=pid_value;			//Right elevon down
	}
	else{

		//Set a maximum value
		if(propulsion->servo_left_timer_val+=pid_value > MAX_LEFT_EXTENSION){
			propulsion->servo_left_timer_val = MAX_LEFT_EXTENSION;
		}
		if(propulsion->servo_right_timer_val +=pid_value > MAX_RIGHT_EXTENSION){
			propulsion->servo_right_timer_val = MAX_RIGHT_EXTENSION;
		}

		propulsion->servo_left_timer_val+=pid_value;			//Left elevon up
		propulsion->servo_right_timer_val+=pid_value;			//Right elevon up
	}
	PropulsionAndControl_Update(propulsion);
}
