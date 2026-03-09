/******************************************************************************
* lib_pid
* A robust but simple PID Controller Library with fast Intager maths, and
* adaptive clamping etc.
*
* ADBeta (c)
******************************************************************************/
#include "lib_pid.h"


#include <stdint.h>
#include <stddef.h>



/*** Function Definitions ****************************************************/
void pid_init(pid_ctrl_t *pid)
{
	if(pid == NULL) return;

	// Reset the tracking values
	pid_reset(pid);

	// Make sure Scale cannot be 0
	if(pid->scale == 0)   pid->scale = 1;
	
	// Set the Integral Clamping value based on the maximum output value, and
	// the Integral Gain value - or set to 0 if Ki is 0 to disable clamping
	pid->_iclamp = (pid->Ki != 0) ? ((pid->out_max * pid->scale) / pid->Ki) : 0;

}


void pid_reset(pid_ctrl_t *pid)
{
	if(pid != NULL)
	{
		pid->_integral    = 0;
		pid->_prev_error  = 0;
	}
}


int32_t pid_calculate(pid_ctrl_t *pid, const int32_t pv, const int32_t sp)
{
	if(pid == NULL) return 0;

	// Calculate the current Error and Derivative
	int32_t error = sp - pv;
	int32_t derivative = error - pid->_prev_error;

	// Prevent Integral Windup by clamping
	pid->_integral = CLAMP_MIN_MAX(pid->_integral, -pid->_iclamp, pid->_iclamp);
	//if(pid->_integral >  pid->_iclamp)   pid->_integral =  pid->_iclamp;
	//if(pid->_integral < -pid->_iclamp)   pid->_integral = -pid->_iclamp;


	int64_t P = (int32_t)pid->Kp * error;
	int64_t I = (int32_t)pid->Ki * pid->_integral;
	int64_t D = (int32_t)pid->Kd * derivative;
	// Add the PID values, then scale to preserve resolution of intagers
	int32_t cv = (int64_t)(P + I + D) / pid->scale;

	// Only accumulate the integration value if the output is not in saturation
	if(cv <= pid->out_max && cv >= pid->out_min)    pid->_integral += error;
	
	// Clamp the CV Output to the min/max
	cv = CLAMP_MIN_MAX(cv, pid->out_min, pid->out_max);

	// Update the previous error for derivative
	pid->_prev_error = error;

	return cv;
}
