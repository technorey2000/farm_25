#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_timer.h"
#include <esp_log.h>
#include "string.h"


//Common includes
#include "Shared/common.h"

//Task includes
#include "Task/serialTask.h"
#include "Task/sysControlTask.h"
#include "Task/dispatcherTask.h"
#include "Task/strgTask.h"
#include "Task/bleTask.h"
#include "Task/sensorsTask.h"
#include "Task/wifiTask.h"
#include "Sync/syncTask.h"


//Task Prototypes
void serialTask(void *param);
void sysControlTask(void *param);
void dispatcherTask(void *param);
void strgTask(void *param);
void bleTask(void *param);
void snrTask(void *param);
void wifiTask(void *param);

static uint32_t systemTimeStamp_ms = 0;
static void sync_timer_callback(void* arg);

//Queue Handles:
QueueHandle_t serialQueueHandle;
QueueHandle_t sysControlQueueHandle;
QueueHandle_t dispatcherQueueHandle;
QueueHandle_t strgQueueHandle;
QueueHandle_t bleQueueHandle;
QueueHandle_t snrQueueHandle;
QueueHandle_t wifiQueueHandle;

//Event Queue Handler
QueueHandle_t eventQueueHandle;

//Ble logging Queue Handler
QueueHandle_t logQueueHandle;

//Task Handles:
BaseType_t serialTaskHandle;
BaseType_t sysControlTaskHandle;
BaseType_t dispatcherTaskHandle;
BaseType_t strgTaskHandle;
BaseType_t bleTaskHandle;
BaseType_t snrTaskHandle;
BaseType_t wifiTaskHandle;

int app_main() {
  //Create sync timer
  const esp_timer_create_args_t sync_timer_args = {
          .callback = &sync_timer_callback,
          /* name is optional, but may help identify the timer when debugging */
          .name = "syncTmr"
  };

  esp_timer_handle_t sync_timer;
  ESP_ERROR_CHECK(esp_timer_create(&sync_timer_args, &sync_timer));

  /* Start the sync timer */
  ESP_ERROR_CHECK(esp_timer_start_periodic(sync_timer, SYNC_TIME));  //interrupt once every ms

    //Create Queues:
    serialQueueHandle       = xQueueCreate( RTOS_QUEUE_SIZE, sizeof( RTOS_message_t ) );
    sysControlQueueHandle   = xQueueCreate( RTOS_QUEUE_SIZE, sizeof( RTOS_message_t ) );
    dispatcherQueueHandle   = xQueueCreate( RTOS_QUEUE_SIZE, sizeof( RTOS_message_t ) );
    strgQueueHandle         = xQueueCreate( RTOS_QUEUE_SIZE, sizeof( RTOS_message_t ) );
    bleQueueHandle          = xQueueCreate( RTOS_QUEUE_SIZE, sizeof( RTOS_message_t ) ); 
    snrQueueHandle          = xQueueCreate( RTOS_QUEUE_SIZE, sizeof( RTOS_message_t ) );
    wifiQueueHandle         = xQueueCreate( RTOS_QUEUE_SIZE, sizeof( RTOS_message_t ) );

    eventQueueHandle        = xQueueCreate( EVENT_QUEUE_SIZE, sizeof( EVENT_message_t ) );
    logQueueHandle          = xQueueCreate( LOG_QUEUE_SIZE, STRING_MAX_LOG_CHARACTERS );

    //Create Tasks:
    serialTaskHandle        = xTaskCreate(serialTask,       "serial",       RTOS_TASK_STACK_SIZE_4K,  NULL, RTOS_TASK_PRIORITY, NULL);
    sysControlTaskHandle    = xTaskCreate(sysControlTask,   "sysControl",   RTOS_TASK_STACK_SIZE_4K,  NULL, RTOS_TASK_PRIORITY, NULL);
    dispatcherTaskHandle    = xTaskCreate(dispatcherTask,   "dispatcher",   RTOS_TASK_STACK_SIZE_2K,  NULL, RTOS_TASK_PRIORITY, NULL);
    strgTaskHandle          = xTaskCreate(strgTask,         "storage",      RTOS_TASK_STACK_SIZE_4K,  NULL, RTOS_TASK_PRIORITY, NULL);
    bleTaskHandle           = xTaskCreate(bleTask,          "ble",          RTOS_TASK_STACK_SIZE_4K,  NULL, RTOS_TASK_PRIORITY, NULL);
    snrTaskHandle           = xTaskCreate(snrTask,          "snr",          RTOS_TASK_STACK_SIZE_4K,  NULL, RTOS_TASK_PRIORITY, NULL);
    wifiTaskHandle          = xTaskCreate(wifiTask,         "wifi",         RTOS_TASK_STACK_SIZE_4K,  NULL, RTOS_TASK_PRIORITY, NULL);

    sysStartSystem();

    return 0;
}


static void sync_timer_callback(void* arg)
{
  systemTimeStamp_ms++;
  syncTaskApp();
}

uint32_t sys_getMsgTimeStamp(void)
{
    return systemTimeStamp_ms;
}


//Task Functions:
void serialTask(void *param)
{
  while(1)
  {
	  serialTaskApp();
  }
}

void sysControlTask(void *param)
{
  while(1)
  {
	  sysControlTaskApp();
   }
}

void dispatcherTask(void *param)
{
  while(1)
  {
	  dispatcherTaskApp();
  }
}


void strgTask(void *param)
{
  while(1)
  {
    strgTaskApp();
    }
}

void bleTask(void *param)
{
  while(1)
  {
	  bleTaskApp();
  }
}

void snrTask(void *param)
{
  while(1)
  {
    snrTaskApp();
  }
}

void wifiTask(void *param) 
{
  while(1) 
  {
    wifiTaskApp();
  }
}

