/*
 * pid.c
 *
 *  Created on: Mar 28, 2018
 *      Author: atmelfan
 */

#include "pid.h"

float pid_update(pid* pid, float set, float actual, float dt){
	/*Calc error and update running integral*/
	float error = set - actual;
	pid->integral_val += error*dt;

	/*Limit integral value*/
	if(pid->integral_val > PID_MAX_INTEGRAL){
		pid->integral_val = PID_MAX_INTEGRAL;
	}else if(pid->integral_val < -PID_MAX_INTEGRAL){
		pid->integral_val = -PID_MAX_INTEGRAL;
	}

	/*PID*/
	float out = error*pid->Kp + pid->Ki*pid->integral_val + pid->Kd*(error - pid->error_val)/dt;

	/*Save error value to next time*/
	pid->error_val = error;

	return out;
}
