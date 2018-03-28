/*
 * pid.h
 *
 *  Created on: Mar 28, 2018
 *      Author: atmelfan
 */

#ifndef PID_H_
#define PID_H_


/*PID settings and state struct*/
typedef struct {
	float Kp, Ki, Kd;
	float integral_val, error_val;
} pid;

/*Used to initialize pid_t struct*/
#define PID_INIT .integral_val = 0, .error_val = 0

/*Maximum integral value*/
#define PID_MAX_INTEGRAL 100

/**
 * Updates the PID controller
 * @param pid, pid struct
 * @param set, wanted value
 * @param actual, measured value
 * @param dt, delta time
 * @return correction value
 */
float pid_update(pid* pid, float set, float actual, float dt);

#endif /* PID_H_ */
