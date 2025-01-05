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
#include "esp_adc_cal.h"
#include "driver/adc.h"


//Common includes
#include "Shared/common.h"

//Task includes
#include "Task/serialTask.h"
#include "Task/sysControlTask.h"
#include "Task/dispatcherTask.h"
#include "Task/strgTask.h"
#include "Task/bleTask.h"
#include "Task/sensorsTask.h"

#include "../components/dht/include/dht11.h"

static char tag[]="arc_cpp_main";

//Move these function prototypes to the sensor module
void calculate_running_average(float new_value, float *average, float *window, int *index);

//Task Prototypes
void serialTask(void *param);
void sysControlTask(void *param);
void dispatcherTask(void *param);
void strgTask(void *param);
void bleTask(void *param);
void snrTask(void *param);

static uint32_t systemTimeStamp_ms = 0;
static void sync_timer_callback(void* arg);

//Queue Handles:
QueueHandle_t serialQueueHandle;
QueueHandle_t sysControlQueueHandle;
QueueHandle_t dispatcherQueueHandle;
QueueHandle_t strgQueueHandle;
QueueHandle_t bleQueueHandle;
QueueHandle_t snrQueueHandle;

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

RTC_NOINIT_ATTR uint32_t otaSignature;
RTC_NOINIT_ATTR char wifiSwUrl[SOFTWARE_URL_MAX_CHARACTERS];

RTC_NOINIT_ATTR uint8_t otaSsid[GATTS_CHAR_SSID_LEN_MAX];
RTC_NOINIT_ATTR uint8_t otaPassword[GATTS_CHAR_PWD_LEN_MAX];

RTC_NOINIT_ATTR uint8_t otaCount = 0;
RTC_NOINIT_ATTR char otaLastVersion[SOFTWARE_VERSION_MAX_CHARACTERS];
RTC_NOINIT_ATTR uint32_t otaCountSig;

static const char *TAG = "Moisture Sensor Calibration";

int app_main() {
  float window[WINDOW_SIZE] = {0};
  float average_reading = 0;
  int index = 0;
  float new_value;

  // Configure ADC width and attenuation
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(SENSOR_PIN, ADC_ATTEN_DB_11);

  // Check if ADC calibration values are burned into eFuse
  esp_adc_cal_value_t cal_type = ESP_ADC_CAL_VAL_EFUSE_VREF;
  if (esp_adc_cal_check_efuse(cal_type) == ESP_ERR_NOT_SUPPORTED) {
      ESP_LOGW(TAG, "eFuse Vref not supported, using default Vref");
      cal_type = ESP_ADC_CAL_VAL_DEFAULT_VREF;
  }

  // Characterize ADC
  esp_adc_cal_characteristics_t *adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);


  DHT11_init(GPIO_NUM_4);

  while(1) {
      printf("Temperature is %d \n", DHT11_read().temperature);
      printf("Humidity is %d\n", DHT11_read().humidity);
      printf("Status code is %d\n", DHT11_read().status);
      vTaskDelay(pdMS_TO_TICKS(1000));
  }

  while (1) {
      // Read raw ADC value
      uint32_t moisture_value = adc1_get_raw(SENSOR_PIN);

      // Log moisture value
      ESP_LOGI(TAG, "Moisture value: %ld", moisture_value);
      
      // Convert raw ADC value to calibrated voltage
      uint32_t moisture_voltage = esp_adc_cal_raw_to_voltage(moisture_value, adc_chars);
      
      // Log calibrated voltage
      ESP_LOGI(TAG, "Calibrated voltage: %ld mV \r\n", moisture_voltage);
      new_value = (float)moisture_voltage;

      // Calculate the running average
      calculate_running_average(new_value, &average_reading, window, &index);

      // Print the current running average
      ESP_LOGI(TAG,"Running average: %.2f\n", average_reading);

      // Calculate moisture percentage
      float moisture_percent = ((float)(average_reading - V_DRY) / (V_WET - V_DRY)) * 100.0;

      // Log calibrated voltage and moisture percentage
      ESP_LOGI(TAG, "Voltage: %.2f mV, Moisture: %.2f%%", average_reading, moisture_percent);

      
      // Delay for readability
      vTaskDelay(pdMS_TO_TICKS(1000));
  }

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

    eventQueueHandle        = xQueueCreate( EVENT_QUEUE_SIZE, sizeof( EVENT_message_t ) );
    logQueueHandle          = xQueueCreate( LOG_QUEUE_SIZE, STRING_MAX_LOG_CHARACTERS );

    //Create Tasks:
    serialTaskHandle        = xTaskCreate(serialTask,       "serial",       RTOS_TASK_STACK_SIZE_4K,  NULL, RTOS_TASK_PRIORITY, NULL);
    sysControlTaskHandle    = xTaskCreate(sysControlTask,   "sysControl",   RTOS_TASK_STACK_SIZE_4K,  NULL, RTOS_TASK_PRIORITY, NULL);
    dispatcherTaskHandle    = xTaskCreate(dispatcherTask,   "dispatcher",   RTOS_TASK_STACK_SIZE_2K,  NULL, RTOS_TASK_PRIORITY, NULL);
    strgTaskHandle          = xTaskCreate(strgTask,         "storage",      RTOS_TASK_STACK_SIZE_4K,  NULL, RTOS_TASK_PRIORITY, NULL);
    bleTaskHandle           = xTaskCreate(bleTask,          "ble",          RTOS_TASK_STACK_SIZE_4K,  NULL, RTOS_TASK_PRIORITY, NULL);
    snrTaskHandle           = xTaskCreate(snrTask,          "snr",          RTOS_TASK_STACK_SIZE_4K,  NULL, RTOS_TASK_PRIORITY, NULL);

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

void snrTask(void *param)
{
  while(1)
  {
    snrTaskApp();
    }
}

// Function to calculate the running average
void calculate_running_average(float new_value, float *average, float *window, int *index) {
    // Subtract the oldest value from the running sum
    *average -= window[*index] / WINDOW_SIZE;

    // Add the new value to the running sum
    window[*index] = new_value;
    *average += new_value / WINDOW_SIZE;

    // Update the index to the next position in the window
    *index = (*index + 1) % WINDOW_SIZE;
}

