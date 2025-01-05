#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "Shared/common.h"
#include "Shared/messages.h"

#include "Sync/modNameSync.h"


//Local Parameters:


//Local Function Prototypes:


//external control access functions

/// @brief Function to lock access to local parameters/functions. No wait time.
/// @param  None
/// @return true if access is granted. Otherwise returns false.
bool modNameTryGrab(void)
{
  if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    return (xSemaphoreTake( modNameSemaphoreHandle, 0));
  else
    return false;
}

/// @brief Function to lock access to local parameters/functions. Will wait a specific amount of time for access.
/// @param delay - How log to wait (ms)
/// @return true if access is granted. Otherwise returns false.
bool modNameWaitAndGrab(TickType_t delay)
{
  if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    return (xSemaphoreTake( modNameSemaphoreHandle, delay));
  else
    return false;
}

/// @brief Unlock access to local parameters/functions.
/// @param  None
/// @return true if access has been unlocked. Otherwise returns false.
bool modNameRelease(void)
{
  if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    return (xSemaphoreGive( modNameSemaphoreHandle ));
  else
    return false;
}

//external functions to access local data

/// @brief modName sync module
/// @param  None
void execmodNameControl(void)
{
	
	//Increment counts
	
	//Check counter
	
	//Process Function when count is reached
	
	//Check for new commands/data
	
	//Process function associated with the new command/data
	
}

/* [] END OF FILE */