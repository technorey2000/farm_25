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

#include "driver/i2c.h"

#include "Shared/common.h"
#include "Task/sensorsTask.h"

#include "Shared/cJSON.h"
#include "Shared/messages.h"

#include "../components/dht/include/dht11.h"
#include "./components/bme280/include/bme280.h"

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
float tmpFloatMP = 0.0;
float tmpFloatMV = 0.0;
uint32_t snrAvgIndex = 0;
uint32_t currentTime = 0;
uint32_t snrMositureTimeout = 0;

//For BME280
s32 com_rslt;
s32 v_uncomp_pressure_s32;
s32 v_uncomp_temperature_s32;
s32 v_uncomp_humidity_s32;
double press = 0.0;
double temp = 0.0;
double hum = 0.0;
float tmpFloatBT = 0.0;
float tmpFloatBP = 0.0;
float tmpFloatBH = 0.0;
char temperature[12];
char pressure[20];
char humidity[10];

//Local Function Prototypes:
void snrLog(char * strPtr, bool forced, bool printTag);
void snrLogR(char * strPtr);
void snrLogIF(char * strPtr);
void snrLogEF(char * strPtr);
void snrLogI(char * strPtr);
void snrLogE(char * strPtr);
void snrSendMessage(uint8_t dstAddr, uint8_t msgType, uint16_t msgCmd, uint8_t * msgDataPtr, uint32_t msgData, uint32_t msgDataLen);
void calculate_running_average(float new_value, float *average, float *window, uint32_t *index);

//For DTH11
void snrTakeMostureReading(void);
void snrTakeDht11TemperatureReading(void);

//For BME280
void i2c_master_init();
s8 BME280_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
s8 BME280_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
void BME280_delay_msek(u32 msek);

// BME280 I2C communication structure
struct bme280_t bme280 = {
    .bus_write = BME280_I2C_bus_write,
    .bus_read = BME280_I2C_bus_read,
    .dev_addr = BME280_I2C_ADDRESS1,
    .delay_msec = BME280_delay_msek};

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
#ifdef SNR_BME280
                    // Initialize BME280 sensor and set internal parameters
                    i2c_master_init();
                    com_rslt = bme280_init(&bme280);
                    printf("com_rslt %d\n", com_rslt);

                    com_rslt += bme280_set_oversamp_pressure(BME280_OVERSAMP_16X);
                    com_rslt += bme280_set_oversamp_temperature(BME280_OVERSAMP_2X);
                    com_rslt += bme280_set_oversamp_humidity(BME280_OVERSAMP_1X);
                    com_rslt += bme280_set_standby_durn(BME280_STANDBY_TIME_1_MS);
                    com_rslt += bme280_set_filter(BME280_FILTER_COEFF_16);
                    com_rslt += bme280_set_power_mode(BME280_NORMAL_MODE);
                    if (com_rslt != SUCCESS)
                    {
                        ESP_LOGE(SNR_TAG,"Error Initalizig the BME280 sensor");
                    }
#endif //SNR_BME280
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

				case SNR_CMD_RD_DHT11_TEMP:
					tmpTemperature = DHT11_read().temperature;
					printf("Temperature is %ld Deg C \n", tmpTemperature);
					snrSendMessage(snrRxMessage.srcAddr, MSG_DATA_32, snrRxMessage.msgCmd, NULL, tmpTemperature, MSG_DATA_32_LEN);
					break;

				case SNR_CMD_RD_DHT11_HUM:
					tmpHumidity = DHT11_read().humidity;
					printf("Humidity is %ld%% \n", tmpHumidity);
					snrSendMessage(snrRxMessage.srcAddr, MSG_DATA_32, snrRxMessage.msgCmd, NULL, tmpHumidity, MSG_DATA_32_LEN);
					break;

				case SNR_CMD_RD_DHT11_CODE:
					tmpCode = DHT11_read().status;
					printf("Status code is %ld \n", tmpCode);
					snrSendMessage(snrRxMessage.srcAddr, MSG_DATA_32, snrRxMessage.msgCmd, NULL, tmpCode, MSG_DATA_32_LEN);
					break;

				case SNR_CMD_RD_MOISTURE_VOLTAGE:
					printf("Voltage: %.2f", average_reading);
                    tmpFloatMV = (float)average_reading;
					snrSendMessage(snrRxMessage.srcAddr, MSG_DATA_FLOAT, snrRxMessage.msgCmd, (uint8_t *)&tmpFloatMV, MSG_DATA_0 ,sizeof(tmpFloatMV));
					break;

				case SNR_CMD_RD_MOISTURE_PERCENT:
					printf("Moisture: %.2f%%", moisture_percent);
                    tmpFloatMP = (float)moisture_percent;
					snrSendMessage(snrRxMessage.srcAddr, MSG_DATA_FLOAT, snrRxMessage.msgCmd, (uint8_t *)&tmpFloatMP, MSG_DATA_0 ,sizeof(tmpFloatMP));
					break;

#ifdef SNR_BME280
				case SNR_CMD_RD_BME280_TEMP:
                    // Read BME280 data
                    com_rslt = bme280_read_uncomp_pressure_temperature_humidity(
                        &v_uncomp_pressure_s32, &v_uncomp_temperature_s32, &v_uncomp_humidity_s32);

                    temp = bme280_compensate_temperature_double(v_uncomp_temperature_s32);
                    sprintf(temperature, "%.2f degC", temp);
                    printf("Temperature %s\n",temperature);
                    tmpFloatBT = (float)temp;
					snrSendMessage(snrRxMessage.srcAddr, MSG_DATA_FLOAT, snrRxMessage.msgCmd, (uint8_t *)&tmpFloatBT, MSG_DATA_0 ,sizeof(tmpFloatBT));
					break;

				case SNR_CMD_RD_BME280_HUM:
                    // Read BME280 data
                    com_rslt = bme280_read_uncomp_pressure_temperature_humidity(
                        &v_uncomp_pressure_s32, &v_uncomp_temperature_s32, &v_uncomp_humidity_s32);

                    hum = bme280_compensate_humidity_double(v_uncomp_humidity_s32);
                    sprintf(humidity, "%.2f %%", hum);
                    printf("Humidity %s\n",humidity);
                    tmpFloatBH = (float)hum;
					snrSendMessage(snrRxMessage.srcAddr, MSG_DATA_FLOAT, snrRxMessage.msgCmd, (uint8_t *)&tmpFloatBH, MSG_DATA_0 ,sizeof(tmpFloatBH));
					break;

				case SNR_CMD_RD_BME280_P_HPA:
                    // Read BME280 data
                    com_rslt = bme280_read_uncomp_pressure_temperature_humidity(
                        &v_uncomp_pressure_s32, &v_uncomp_temperature_s32, &v_uncomp_humidity_s32);

                    press = bme280_compensate_pressure_double(v_uncomp_pressure_s32) / 100; // Pa -> hPa
                    sprintf(pressure, "%.2f hPa", press);
                    printf("Pressure %s\n",pressure);
                    tmpFloatBP = (float)press;
					snrSendMessage(snrRxMessage.srcAddr, MSG_DATA_FLOAT, snrRxMessage.msgCmd, (uint8_t *)&tmpFloatBP, MSG_DATA_0 ,sizeof(tmpFloatBP));
					break;

				case SNR_CMD_RD_BME280_P_PSI:
                    // Read BME280 data
                    com_rslt = bme280_read_uncomp_pressure_temperature_humidity(
                        &v_uncomp_pressure_s32, &v_uncomp_temperature_s32, &v_uncomp_humidity_s32);

                    press = (bme280_compensate_pressure_double(v_uncomp_pressure_s32) / 100) * 0.01450377; // Pa -> psi
                    sprintf(pressure, "%.2f psi", press);
                    printf("Pressure %s\n",pressure);
                    tmpFloatBP = (float)press;
					snrSendMessage(snrRxMessage.srcAddr, MSG_DATA_FLOAT, snrRxMessage.msgCmd, (uint8_t *)&tmpFloatBP, MSG_DATA_0 ,sizeof(tmpFloatBP));
					break;

				case SNR_CMD_RD_BME280_P_INHG:
                    // Read BME280 data
                    com_rslt = bme280_read_uncomp_pressure_temperature_humidity(
                        &v_uncomp_pressure_s32, &v_uncomp_temperature_s32, &v_uncomp_humidity_s32);

                    press = (bme280_compensate_pressure_double(v_uncomp_pressure_s32) / 100) * 0.029529983071445; // Pa -> inhg
                    sprintf(pressure, "%.2f inhg", press);
                    printf("Pressure %s\n",pressure);
                    tmpFloatBP = (float)press;
					snrSendMessage(snrRxMessage.srcAddr, MSG_DATA_FLOAT, snrRxMessage.msgCmd, (uint8_t *)&tmpFloatBP, MSG_DATA_0 ,sizeof(tmpFloatBP));
					break;
#endif //SNR_BME280
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
			snrTakeDht11TemperatureReading();
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
void calculate_running_average(float new_value, float *average, float *window, uint32_t *avgIndex) {
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

void snrTakeDht11TemperatureReading(void) {
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

// Initialize I2C communication parameters
void i2c_master_init()
{
	i2c_config_t i2c_config = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = SDA_PIN,
		.scl_io_num = SCL_PIN,
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master.clk_speed = 1000000};
	i2c_param_config(I2C_NUM_0, &i2c_config);
	i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
}

// BME280 I2C write function
s8 BME280_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 iError = BME280_INIT_VALUE;

	esp_err_t espRc;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);

	i2c_master_write_byte(cmd, reg_addr, true);
	i2c_master_write(cmd, reg_data, cnt, true);
	i2c_master_stop(cmd);

	espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
	if (espRc == ESP_OK)
	{
		iError = SUCCESS;
	}
	else
	{
		iError = FAIL;
	}
	i2c_cmd_link_delete(cmd);

	return (s8)iError;
}

// BME280 I2C read function
s8 BME280_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 iError = BME280_INIT_VALUE;
	esp_err_t espRc;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(cmd, reg_addr, true);

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);

	if (cnt > 1)
	{
		i2c_master_read(cmd, reg_data, cnt - 1, I2C_MASTER_ACK);
	}
	i2c_master_read_byte(cmd, reg_data + cnt - 1, I2C_MASTER_NACK);
	i2c_master_stop(cmd);

	espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
	if (espRc == ESP_OK)
	{
		iError = SUCCESS;
	}
	else
	{
		iError = FAIL;
	}

	i2c_cmd_link_delete(cmd);

	return (s8)iError;
}

// BME280 I2C delay function
void BME280_delay_msek(u32 msek)
{
	vTaskDelay(msek / portTICK_PERIOD_MS);
}
/* [] END OF FILE */

