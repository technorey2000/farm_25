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

static char tag[]="arc_cpp_main";

//Task Prototypes
void serialTask(void *param);
void sysControlTask(void *param);
void dispatcherTask(void *param);
void strgTask(void *param);
void bleTask(void *param);

static uint32_t systemTimeStamp_ms = 0;
static void sync_timer_callback(void* arg);

//Queue Handles:
QueueHandle_t serialQueueHandle;
QueueHandle_t sysControlQueueHandle;
QueueHandle_t dispatcherQueueHandle;
QueueHandle_t strgQueueHandle;
QueueHandle_t bleQueueHandle;

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

RTC_NOINIT_ATTR uint32_t otaSignature;
RTC_NOINIT_ATTR char wifiSwUrl[SOFTWARE_URL_MAX_CHARACTERS];

RTC_NOINIT_ATTR uint8_t otaSsid[GATTS_CHAR_SSID_LEN_MAX];
RTC_NOINIT_ATTR uint8_t otaPassword[GATTS_CHAR_PWD_LEN_MAX];

RTC_NOINIT_ATTR uint8_t otaCount = 0;
RTC_NOINIT_ATTR char otaLastVersion[SOFTWARE_VERSION_MAX_CHARACTERS];
RTC_NOINIT_ATTR uint32_t otaCountSig;

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

    eventQueueHandle        = xQueueCreate( EVENT_QUEUE_SIZE, sizeof( EVENT_message_t ) );
    logQueueHandle          = xQueueCreate( LOG_QUEUE_SIZE, STRING_MAX_LOG_CHARACTERS );

    //Create Tasks:
    serialTaskHandle        = xTaskCreate(serialTask,       "serial",       RTOS_TASK_STACK_SIZE_4K,  NULL, RTOS_TASK_PRIORITY, NULL);
    sysControlTaskHandle    = xTaskCreate(sysControlTask,   "sysControl",   RTOS_TASK_STACK_SIZE_4K,  NULL, RTOS_TASK_PRIORITY, NULL);
    dispatcherTaskHandle    = xTaskCreate(dispatcherTask,   "dispatcher",   RTOS_TASK_STACK_SIZE_2K,  NULL, RTOS_TASK_PRIORITY, NULL);
    strgTaskHandle          = xTaskCreate(strgTask,         "storage",      RTOS_TASK_STACK_SIZE_4K,  NULL, RTOS_TASK_PRIORITY, NULL);
    bleTaskHandle           = xTaskCreate(bleTask,          "ble",          RTOS_TASK_STACK_SIZE_4K,  NULL, RTOS_TASK_PRIORITY, NULL);

    sysStartSystem();

    return 0;
}


static void sync_timer_callback(void* arg)
{
  systemTimeStamp_ms++;
}

uint32_t sys_getMsgTimeStamp(void)
{
    return systemTimeStamp_ms;
}

// uint32_t sys_getMsgTimeStamp(void)
// {
//     return esp_timer_get_time();
// }


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