/******************************************************************************
* Basic but feature-rich Cooperativew Scheduler Library for low power MCUs
*
* Ver 1.0    30 May 2026
* (c) ADBeta 2026
******************************************************************************/
#ifndef LIB_SCHEDULER_H
#define LIB_SCHEDULER_H

#include <stdint.h>
#include <stdbool.h>

#define MAX_SCHEDULER_TASKS    8

/*** API Functions ***********************************************************/
/// @brief Adds a task to the scheduler, with a specified executon period 
/// Returns a unique identifier of the task
/// @param task, function pointer to the task to be scheduled
/// @param period_ms, Execution period of the task in milliseconds
/// @return UID of the task or 0 on failure
uint8_t scheduler_add(void (*task)(void), uint32_t period_ms);


/// @brief Removes a task from the scheduler by its unique identifier
/// @param uid, Uinque identifier of the task to remove
/// @return true on success, false on failure
bool scheduler_remove(const uint8_t uid);


/// @brief Runs the scheduler loop and executes tasks whos timers have elapsed
/// @param now, Current time in milliseconds, (e.g. from millis())
/// @return None
void scheduler_run(const uint32_t now);


/// @brief Forces execution of a task immediately, regardless of its timer 
/// This updates the tasks last execution timestamp
/// @param uid, Unique identifier of the task to execute
/// @param now, Current time in milliseconds, (e.g. from millis())
/// @return None
void scheduler_force_task(const uint8_t uid, const uint32_t now);










#endif
