#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "string.h"
#include "esp_adc_cal.h"
#include "driver/adc.h"

#include "Shared/common.h"
#include "Task/sensorsTask.h"

#include "Shared/cJSON.h"
#include "Shared/messages.h"

#include "../components/dht/include/dht11.h"


//Local Parameters:
static char SNR_TAG[]="SNR";
RTOS_message_t snrRxMessage;
RTOS_message_t snrTxMessage;
UBaseType_t snrNewHighWaterMark = 0;
UBaseType_t snrCurrHighWaterMark = 0;
UBaseType_t snrInitHighWaterMark = 0;
uint8_t snrStatus = SNR_INIT_UNKNOWN;
char snrTmpStr[SNR_STRING_MAX_ARRAY_CHARACTERS];

esp_adc_cal_characteristics_t *adc_chars;

//For Temperature/humidity sensor
uint32_t tmpTemperature = 0;
uint32_t tmpHumidity = 0;
uint32_t tmpCode = 0;

//For moisture sensor
uint32_t moisture_value = 0;
uint32_t moisture_voltage = 0;
float moisture_percent = 0.0;
float new_value = 0.0;
float window[WINDOW_SIZE] = {0};
float average_reading = 0;
uint32_t snrAvgIndex = 0;
uint32_t currentTime = 0;
uint32_t snrMositureTimeout = 0;

//Local Function Prototypes:
void snrLog(char * strPtr, bool forced, bool printTag);
void snrLogR(char * strPtr);
void snrLogIF(char * strPtr);
void snrLogEF(char * strPtr);
void snrLogI(char * strPtr);
void snrLogE(char * strPtr);
void snrSendMessage(uint8_t dstAddr, uint8_t msgType, uint16_t msgCmd, uint8_t * msgDataPtr, uint32_t msgData, uint32_t msgDataLen);
void calculate_running_average(float new_value, float *average, float *window, int *index);
void snrTakeMostureReading(void);
void snrTakeTemperatureReading(void);

//External Functions:

/// @brief snr task
/// @param  None
void snrTaskApp(void)
{
	//DHT11_init(GPIO_NUM_4);
    while(1)
    {
        if (xQueueReceive(snrQueueHandle, &snrRxMessage, 10))
        {	
			switch(snrRxMessage.msgCmd)
			{
				case SNR_CMD_INIT:
					ESP_LOGI(SNR_TAG, "SNR_CMD_INIT received");

					// Configure ADC width and attenuation
					adc1_config_width(ADC_WIDTH_BIT_12);
					adc1_config_channel_atten(SENSOR_PIN, ADC_ATTEN_DB_11);

					// Check if ADC calibration values are burned into eFuse
					esp_adc_cal_value_t cal_type = ESP_ADC_CAL_VAL_EFUSE_VREF;
					if (esp_adc_cal_check_efuse(cal_type) == ESP_ERR_NOT_SUPPORTED) {
						ESP_LOGW(SNR_TAG, "eFuse Vref not supported, using default Vref");
						cal_type = ESP_ADC_CAL_VAL_DEFAULT_VREF;
					}

					// Characterize ADC
					adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
					esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);

					DHT11_init(GPIO_NUM_4);

					snrMositureTimeout = sys_getMsgTimeStamp();

                    snrStatus = SNR_INIT_COMPLETE;
                    snrSendMessage(snrRxMessage.srcAddr, MSG_DATA_8, SNR_CMD_INIT, NULL, snrStatus, MSG_DATA_8_LEN);
                    break;
				case SNR_CMD_PING:
                    snrSendMessage(snrRxMessage.srcAddr, MSG_DATA_8, SNR_CMD_PING, NULL, SNR_PING_RECEIVED, MSG_DATA_8_LEN);
					break;
				case SNR_CMD_STATUS:
					//TBD
					break;

				case SNR_CMD_RD_TEMPERATURE:
					tmpTemperature = DHT11_read().temperature;
					printf("Temperature is %ld Deg C \n", tmpTemperature);
					snrSendMessage(snrRxMessage.srcAddr, MSG_DATA_32, SNR_CMD_RD_TEMPERATURE, NULL, tmpTemperature, MSG_DATA_32_LEN);
					break;

				case SNR_CMD_RD_HUMIDITY:
					tmpHumidity = DHT11_read().humidity;
					printf("Humidity is %ld%% \n", tmpHumidity);
					snrSendMessage(snrRxMessage.srcAddr, MSG_DATA_32, SNR_CMD_RD_HUMIDITY, NULL, tmpHumidity, MSG_DATA_32_LEN);
					break;

				case SNR_CMD_RD_TH_CODE:
					tmpCode = DHT11_read().status;
					printf("Status code is %ld \n", tmpCode);
					snrSendMessage(snrRxMessage.srcAddr, MSG_DATA_32, SNR_CMD_RD_TH_CODE, NULL, tmpCode, MSG_DATA_32_LEN);
					break;

				case SNR_CMD_RD_MOISTURE_VOLTAGE:
					printf("Voltage: %.2f", average_reading);
					snrSendMessage(snrRxMessage.srcAddr, MSG_DATA_FLOAT, SNR_CMD_RD_MOISTURE_VOLTAGE, NULL, average_reading, MSG_DATA_32_LEN);
					break;

				case SNR_CMD_RD_MOISTURE_PERCENT:
					printf("Moisture: %.2f%%", moisture_percent);
					snrSendMessage(snrRxMessage.srcAddr, MSG_DATA_FLOAT, SNR_CMD_RD_MOISTURE_PERCENT, NULL, moisture_percent, MSG_DATA_32_LEN);
					break;

                case SNR_CMD_LOG1_TEST:
                    snrLogI("This is an information message from the snr module!");
                    break;    	

                case SNR_CMD_LOG2_TEST:
                    snrLogE("This is an error message from the snr module!");
                    break;    

				default:;

			}

        }

		currentTime = sys_getMsgTimeStamp();
		if ((currentTime - snrMositureTimeout) > SNR_READ_MOISTURE_TIMEOUT) {
			snrTakeMostureReading();
			snrTakeTemperatureReading();
			snrMositureTimeout = sys_getMsgTimeStamp();
		}

    }
 
}

//Local Functions:
/// @brief  This function sends the info log message to the BLE queue reguardless of 
///    the mobile device connected and/or the Log notification enabled. Iw also send
///    the log message to the terminal.
/// @param strPtr 
void snrLogR(char * strPtr)
{
    ESP_LOGI(SNR_TAG, "%s", strPtr);
    snrLog(strPtr, true, false);
}

/// @brief  This function sends the info log message to the BLE queue reguardless of 
///    the mobile device connected and/or the Log notification enabled. Iw also send
///    the log message to the terminal.
/// @param strPtr 
void snrLogIF(char * strPtr)
{
    ESP_LOGI(SNR_TAG, "%s", strPtr);
    snrLog(strPtr, true, true);
}

/// @brief  This function sends the error log message to the BLE queue reguardless of 
///    the mobile device connected and/or the Log notification enabled. Iw also send
///    the log message to the terminal.
void snrLogEF(char * strPtr)
{
    ESP_LOGE(SNR_TAG, "%s", strPtr);
    snrLog(strPtr, true, true);
}

/// @brief  This function send the info log message to the Mobile device and to the 
///    terminal.
/// @param strPtr 
void snrLogI(char * strPtr)
{
    ESP_LOGI(SNR_TAG, "%s", strPtr);
    snrLog(strPtr, false, true);
}

/// @brief This function send the error log message to the Mobile device and to the 
///    terminal. 
/// @param strPtr 
void snrLogE(char * strPtr)
{
    ESP_LOGE(SNR_TAG, "%s", strPtr);
    snrLog(strPtr, false, true);
}

/// @brief This function sends a message to the BLE module to send out a log message
///   to the mobile device via BLE. Mobile device has to be connected and the
///   log notifications (RX Characteristics) enabled.
/// @param strPtr - message to be sent to phone.
/// @param forced - If true, message is forced in the log queue.
/// @param printTag - If true the tag is printed with the message. 
void snrLog(char * strPtr, bool forced, bool printTag)
{
    static uint8_t printIndex = 0;
    RTOS_message_t modPrintMsg;
    char tmpStr[STRING_MAX_LOG_CHARACTERS];
    static char modStr[SNR_STRING_MAX_ARRAY_CHARACTERS][STRING_MAX_LOG_CHARACTERS];

    //copy string into next available string array
    //memcpy(modStr[printIndex], strPtr, strlen(strPtr)+1);
    memcpy(tmpStr, strPtr, strlen(strPtr)+1);
	if (printTag){
        sprintf(modStr[printIndex], "%s:%s", SNR_TAG, tmpStr);
    }else{
        sprintf(modStr[printIndex], "%s", tmpStr);
    }

    //form message to print out string
    modPrintMsg.srcAddr         =  MSG_ADDR_SNR; 
    modPrintMsg.dstAddr         =  MSG_ADDR_BLE;
    modPrintMsg.msgType         =  MSG_DATA_STR;

    if (forced){
        modPrintMsg.msgCmd          =  BLE_CMD_SEND_FORCED_LOG;
    }else{
        modPrintMsg.msgCmd          =  BLE_CMD_SEND_LOG;
    }
    modPrintMsg.msgRef          = msg_getMsgRef();
    modPrintMsg.msgTimeStamp    = sys_getMsgTimeStamp();
    modPrintMsg.msgDataPtr      = (uint8_t *)modStr[printIndex];
    modPrintMsg.msgData         = MSG_DATA_POINTER_ONLY;
    modPrintMsg.msgDataLen      = strlen(modStr[printIndex]); 

    //send message
    xQueueSend(dispatcherQueueHandle,&modPrintMsg,0);

    //Set next array index
    printIndex++;
    if (printIndex >= SNR_STRING_MAX_ARRAY_LOG_ELEMENTS)
    {
        printIndex = 0;
    }
}


/// @brief Function to create message and sent it to the designated destination.
/// @param dstAddr - Task to send message to. (see message address enum)
/// @param msgType - Type of message. (see message type enum)
/// @param msgCmd  - Message associated with the message. (see specific task for command enum)
/// @param msgDataPtr - A pointer to data greater than 32 bits if used. Null otherwise.
/// @param msgData    - Data that is 32 bits or less. Set to 0 if not used.
/// @param msgDataLen - Data length of either msgData or data pointed to msgDataPtr (in bytes).
void snrSendMessage(uint8_t dstAddr, uint8_t msgType, uint16_t msgCmd, uint8_t * msgDataPtr, uint32_t msgData, uint32_t msgDataLen)
{  
    static uint8_t msgIndex = 0;
    RTOS_message_t sendRtosMsg;
    static char sendMsgArray[SNR_MESSAGE_MAX_ARRAY_ELEMENTS][SNR_STRING_MAX_ARRAY_CHARACTERS];

    if ((msgType == MSG_DATA_0) || (msgType == MSG_DATA_8) || (msgType == MSG_DATA_16) || (msgType == MSG_DATA_32))
    {
       sendRtosMsg.msgDataPtr = NULL;
       switch(msgType)
       {
            case MSG_DATA_0:    sendRtosMsg.msgDataLen =  MSG_DATA_0_LEN;    break;
            case MSG_DATA_8:    sendRtosMsg.msgDataLen =  MSG_DATA_8_LEN;    break;
            case MSG_DATA_16:   sendRtosMsg.msgDataLen =  MSG_DATA_16_LEN;   break;
            case MSG_DATA_32:   sendRtosMsg.msgDataLen =  MSG_DATA_32_LEN;   break;
            default:            sendRtosMsg.msgDataLen =  MSG_DATA_0_LEN;    break;
       }
    } 
    else
    {
        memcpy(sendMsgArray[msgIndex], msgDataPtr, msgDataLen);
        sendRtosMsg.msgDataPtr = (uint8_t *)sendMsgArray[msgIndex];
        sendRtosMsg.msgDataLen = msgDataLen;
    }
    sendRtosMsg.srcAddr = MSG_ADDR_SNR;
    sendRtosMsg.dstAddr = dstAddr;
    sendRtosMsg.msgRef = msg_getMsgRef();
    sendRtosMsg.msgTimeStamp = sys_getMsgTimeStamp();
    sendRtosMsg.msgType = msgType;
    sendRtosMsg.msgCmd = msgCmd;
    sendRtosMsg.msgData = msgData;
 
    //send message
    xQueueSend(dispatcherQueueHandle,&sendRtosMsg,0);

    //Set next array index
    msgIndex++;
    if (msgIndex >= SNR_MESSAGE_MAX_ARRAY_ELEMENTS)
    {
        msgIndex = 0;
    }

}

// Function to calculate the running average
void calculate_running_average(float new_value, float *average, float *window, int *avgIndex) {
    // Subtract the oldest value from the running sum
    *average -= window[*avgIndex] / WINDOW_SIZE;

    // Add the new value to the running sum
    window[*avgIndex] = new_value;
    *average += new_value / WINDOW_SIZE;

    // Update the index to the next position in the window
    *avgIndex = (*avgIndex + 1) % WINDOW_SIZE;
}

void snrTakeMostureReading(void) {
	// Read raw ADC value
	moisture_value = adc1_get_raw(SENSOR_PIN);

	// Log moisture value
	ESP_LOGI(SNR_TAG, "Moisture value: %ld", moisture_value);
	
	// Convert raw ADC value to calibrated voltage
	moisture_voltage = esp_adc_cal_raw_to_voltage(moisture_value, adc_chars);
	
	// Log calibrated voltage
	ESP_LOGI(SNR_TAG, "Calibrated voltage: %ld mV", moisture_voltage);
	new_value = (float)moisture_voltage;

	// Calculate the running average
	calculate_running_average(new_value, &average_reading, window, &snrAvgIndex);

	// Print the current running average
	ESP_LOGI(SNR_TAG,"Running average: %.2f", average_reading);

	// Calculate moisture percentage
	moisture_percent = ((float)(average_reading - V_DRY) / (V_WET - V_DRY)) * 100.0;

	// Log calibrated voltage and moisture percentage
	ESP_LOGI(SNR_TAG, "Voltage: %.2f mV, Moisture: %.2f%% \n", average_reading, moisture_percent);
}

void snrTakeTemperatureReading(void) {
	// tmpTemperature = DHT11_read().temperature;
	// printf("Temperature is %ld Deg C \n", tmpTemperature);

	// tmpHumidity = DHT11_read().humidity;
	// printf("Humidity is %ld%% \n", tmpHumidity);
	
	// tmpCode = DHT11_read().status;
	// printf("Status code is %ld \n", tmpCode);
	printf("Temperature is %d \n", DHT11_read().temperature);
    printf("Humidity is %d\n", DHT11_read().humidity);
    printf("Status code is %d\n", DHT11_read().status);
}
/* [] END OF FILE */

