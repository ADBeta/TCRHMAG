/******************************************************************************
* Basic but feature-rich Cooperativew Scheduler Library for low power MCUs
*
* (c) ADBeta 2026
******************************************************************************/
#include "lib_scheduler.h"

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>


/*** Hidden ******************************************************************/
typedef struct
{
	uint8_t uid;
	void (*task)(void);
	uint32_t period_ms;
	uint32_t last_ms;
} scheduler_task_t;

static scheduler_task_t tasks[MAX_SCHEDULER_TASKS];
static uint8_t task_count = 0;
static uint8_t current_uid = 1;



/*** Public API Functions ****************************************************/
uint8_t scheduler_add(void (*task)(void), uint32_t period_ms)
{
	// If the task list if full, return error
	if(task_count >= MAX_SCHEDULER_TASKS) return -1;
	uint8_t uid = current_uid++;

	tasks[task_count].uid         = uid;
	tasks[task_count].task        = task;
	tasks[task_count].period_ms   = period_ms;
	tasks[task_count].last_ms     = 0;
	
	task_count++;
	return uid;
}


bool scheduler_remove(const uint8_t uid)
{
	// Search for the given UID in the task list
	for(size_t idx = 0; idx < task_count; idx++)
	{
		if(tasks[idx].uid == uid)
		{
			task_count--;

			// If this isn't the last task in the list, swap it
			if(idx != task_count)
				tasks[idx] = tasks[task_count];

			// Clear the last slot for safety
			tasks[task_count] = (scheduler_task_t){0};

			// Success
			return true;
		}
	}

	// UID Was not found
	return false;
}


void scheduler_run(const uint32_t now)
{
	for(size_t idx = 0; idx < task_count; idx++)
	{
	 	// If the timer is up, execute the task and update its last time
		if((now - tasks[idx].last_ms) >= tasks[idx].period_ms)
		{
			tasks[idx].task();
			tasks[idx].last_ms = now;
		}
	}
}


void scheduler_force_task(const uint8_t uid, const uint32_t now)
{
	for(size_t idx = 0; idx < task_count; idx++)
	{
		if(tasks[idx].uid == uid)
		{
			tasks[idx].task();
			tasks[idx].last_ms = now;
		}
	}
}
