/******************************************************************************
* lib_pid
* A robust but simple PID Controller Library with fast Intager maths, and
* adaptive clamping etc.
*
* ADBeta (c)    09 Mar 2026    Ver 1.0
******************************************************************************/
#ifndef LIB_PID_H
#define LIB_PID_H

#include <stdint.h>
#include <stddef.h>

/*** Macro Functions *********************************************************/
/// @brief Clamps a value to the minimum or maximum value given
#define CLAMP_MIN_MAX(x,min,max) \
	(((x) < (min)) ? (min) : (((x) > (max)) ? (max) : (x)))



/*** Types and Structures ****************************************************/
typedef struct
{
	int32_t Kp, Ki, Kd;                 // PID Gain Values (Scaled)
	int32_t scale;                      // Scaling value for the Gain Values
	int32_t out_min;                    // Output Minimum & Maximum for clamping
	int32_t out_max;

	// Hidden tracking variables
	int32_t _integral;                  // Accumulated Integral Value
	int32_t _iclamp;                    // Integral value Clamp to prevent windup
	int32_t _prev_error;                // Previous Error for Derivative calcs
} pid_ctrl_t;



/*** Function Declarations ***************************************************/

/// @brief Initialises the PID Structures integral clamp value, protects againt
/// certain invalid donfigurations and resets the tracking variables
/// @param pid, Pointer to PID Control Struct
/// @return None
void pid_init(pid_ctrl_t *pid);


/// @brief Resets the Tracking variables in a given PID Control Struct
/// @param pid, Pointer to PID Control Structure
/// @return none
void pid_reset(pid_ctrl_t *pid);


/// @brief Calculates an CV Output value from a given set and target value,
/// using PID Control
/// @param pid, pointer to pid_ctrl_t struct
/// @param  pv, Process Variable      - Current measured value 
/// @param  sp, Setpoint              - Target Value
/// @return cv, Control Variable      - Output Actuator Value 
int32_t pid_calculate(pid_ctrl_t *pid, const int32_t pv, const int32_t sp);


#endif
