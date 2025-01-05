#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"

#include "Shared/common.h"
#include "Task/sensorsTask.h"

#include "Shared/cJSON.h"
#include "Shared/messages.h"


//Local Parameters:
RTOS_message_t snrRxMessage;
RTOS_message_t snrTxMessage;
UBaseType_t snrNewHighWaterMark = 0;
UBaseType_t snrCurrHighWaterMark = 0;
UBaseType_t snrInitHighWaterMark = 0;
uint8_t snrStatus = snr_INIT_UNKNOWN;
char snrTmpStr[STRING_MAX_ARRAY_CHARACTERS];

//Local Function Prototypes:
void snrLog(char * strPtr, bool forced, bool printTag);
void snrLogR(char * strPtr);
void snrLogIF(char * strPtr);
void snrLogEF(char * strPtr);
void snrLogI(char * strPtr);
void snrLogE(char * strPtr);
void snrSendMessage(uint8_t dstAddr, uint8_t msgType, uint16_t msgCmd, uint8_t * msgDataPtr, uint32_t msgData, uint32_t msgDataLen);

//External Functions:

/// @brief snr task
/// @param  None
void snrTaskApp(void)
{
    while(1)
    {
        if (xQueueReceive(snrQueueHandle, &snrRxMessage, portMAX_DELAY))
        {
			switch(snrRxMessage.msgCmd)
			{
				case SNR_CMD_INIT:
                    snrStatus = snr_INIT_COMPLETE;
                    snrSendMessage(snrRxMessage.srcAddr, MSG_DATA_8, snr_CMD_INIT, NULL, snrStatus, MSG_DATA_8_LEN);
                    break;
				case SNR_CMD_PING:
                    snrSendMessage(snrRxMessage.srcAddr, MSG_DATA_8, snr_CMD_PING, NULL, snr_PING_RECEIVED, MSG_DATA_8_LEN);
					break;
				case snr_CMD_STATUS:
					//TBD
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

    }
 
}

//Local Functions:
/// @brief  This function sends the info log message to the BLE queue reguardless of 
///    the mobile device connected and/or the Log notification enabled. Iw also send
///    the log message to the terminal.
/// @param strPtr 
void snrLogR(char * strPtr)
{
    ESP_LOGI(snr_TAG, "%s", strPtr);
    snrLog(strPtr, true, false);
}

/// @brief  This function sends the info log message to the BLE queue reguardless of 
///    the mobile device connected and/or the Log notification enabled. Iw also send
///    the log message to the terminal.
/// @param strPtr 
void snrLogIF(char * strPtr)
{
    ESP_LOGI(snr_TAG, "%s", strPtr);
    snrLog(strPtr, true, true);
}

/// @brief  This function sends the error log message to the BLE queue reguardless of 
///    the mobile device connected and/or the Log notification enabled. Iw also send
///    the log message to the terminal.
void snrLogEF(char * strPtr)
{
    ESP_LOGE(snr_TAG, "%s", strPtr);
    snrLog(strPtr, true, true);
}

/// @brief  This function send the info log message to the Mobile device and to the 
///    terminal.
/// @param strPtr 
void snrLogI(char * strPtr)
{
    ESP_LOGI(snr_TAG, "%s", strPtr);
    snrLog(strPtr, false, true);
}

/// @brief This function send the error log message to the Mobile device and to the 
///    terminal. 
/// @param strPtr 
void snrLogE(char * strPtr)
{
    ESP_LOGE(snr_TAG, "%s", strPtr);
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
    static char modStr[<snr>_STRING_MAX_ARRAY_LOG_ELEMENTS][STRING_MAX_LOG_CHARACTERS];

    //copy string into next available string array
    //memcpy(modStr[printIndex], strPtr, strlen(strPtr)+1);
    memcpy(tmpStr, strPtr, strlen(strPtr)+1);
	if (printTag){
        sprintf(modStr[printIndex], "%s:%s", snr_TAG, tmpStr);
    }else{
        sprintf(modStr[printIndex], "%s", tmpStr);
    }

    //form message to print out string
    modPrintMsg.srcAddr         =  MSG_ADDR_snr; 
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
    if (printIndex >= <snr>_STRING_MAX_ARRAY_LOG_ELEMENTS)
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
    static char sendMsgArray[<snr>_MESSAGE_MAX_ARRAY_ELEMENTS][snr_STRING_MAX_ARRAY_CHARACTERS];

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
    sendRtosMsg.srcAddr = MSG_ADDR_snr;
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
    if (msgIndex >= <snr>_MESSAGE_MAX_ARRAY_ELEMENTS)
    {
        msgIndex = 0;
    }

}

    /**************************************************************************************************************
    Notes:
	For any function created, add an information comment about that function. Type "///" ,above the function, and 
	  the IDE will automaticaly create the infomation block for you to fill in.
	  
	All defines, externs, enums, JSON tag definitions and debug macros need to go into the common.h file in the 
	  corresponding locations as described below.  
    
    Put the following in the common.h file:
	
	//snr Task Defines:
	<any defines for the specific module>
	
	//External Queue Handles:	
	extern QueueHandle_t snrQueueHandle;
	
	//External Semaphore Handles:
	extern SemaphoreHandle_t snrSemaphoreHandle;
	
	Put the address of the module into the "enum message_address"
		MSG_ADDR_snr 		= XX,    //snr module

	Recommended addresses:
	MSG_ADDR_SCTL 		= 0,    //System Control
	MSG_ADDR_DSPR 		= 1,    //Dispatcher
	MSG_ADDR_SER 		= 2,    //Serial module
	MSG_ADDR_SUPR 		= 3,   	//Supervisor module
	MSG_ADDR_BLE 		= 4,	//BLE module
	MSG_ADDR_snr 		= 5,    //snr module
	MSG_ADDR_STRG 		= 6,    //Data storage module
	MSG_ADDR_ALARMS 	= 7,    //Alarms module
	MSG_ADDR_LED 		= 8,    //LED module
	MSG_ADDR_KEY 		= 9,   	//KeyPad module
	MSG_ADDR_SCALE 		= 10,   //Scale module
	MSG_ADDR_BATTERY 	= 11 	//Battery control
		
	//snr Structures: Used for message data
	typedef struct <structure name> {
		uint16_t snrDataStructID;		//To identify the structure. This name needs to be entered into the "enum structure_ids"
		<Structure Parameters>;
		} <structure name>_t;
		
	//snr Structures: Not used for message data	
	typedef struct <structure name> {
		<Structure Parameters>;
		} <structure name>_t;


	Note: the snr needs to be all capitals for the following:
	
	////////////////////////////////////////////////////////////////////////////////////////////////////////
	//snr Parameters:
	////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	All commands need to be unique. Here is the range for the corresponding tasks:
	
	SysControl 	- 	0     to 999
	dispatcher	-	1000  to 1999
	serial		-	2000  to 2999
	supervisor	-	3000  to 3999
	ble			- 	4000  to 4999
	snr			-	5000  to 5999
	storage		- 	6000  to 6999
	alarms		- 	7000  to 7999
	led			-	8000  to 8999
	keypad		-	9000  to 9999
	scale		-	10000 to 10999
	battery		-	11000 to 11999
	http		-   12000 to 12999
	
    enum snr_cmd_type {
        snr_CMD_UNKNOWN         = -1,
        snr_CMD_INIT            = xxxx,    //intialize task
        snr_CMD_PING            = xxxx,    //echo message received back to sender
        snr_CMD_STATUS          = xxxx,    //Send back status to sender

		snr_CMD_LOG1_TEST		= xxxx,		//Send information log to mobile device via BLE
		snr_CMD_LOG2_TEST		= xxxx,		//Send error log to mobile device via BLE
		<Add new commands here>
    };

    enum snr_status {
        snr_INIT_UNKNOWN  		= -1,
        snr_INIT_COMPLETE 		= 0,
        snr_INIT_ERROR    		= 1,
        snr_STATUS_GOOD   		= 2,
        snr_STATUS_FAILURE		= 3
    };  

    enum snr_ping_resp {
        snr_PING_RECEIVED 		= 0,
        snr_PING_ERROR    		= 1
    };
  
  
    ------------------------------------------------------------------------------------------------------
    put the following in the main.c file:


	//Task includes	  
    #include "Task/snrTask.h"

	//Task Prototypes
	void snrTask(void *param);
	
	//Queue Handles:
	QueueHandle_t snrQueueHandle;
	
	//Semaphore Handles
	SemaphoreHandle_t snrSemaphoreHandle = NULL;
	
	//Task Handles:
	BaseType_t snrTaskHandle;
	
	//In the app_main function:

		//Define I/O: - If I/O is used, define it here.
		//Example:
		  gpio_set_direction(GPIO_NUM_33, GPIO_MODE_OUTPUT_OD);

    };
	
		//Create semaphores:
		vSemaphoreCreateBinary( ssnrSemaphoreHandle );
	
		//Create Queues:
		modNaleQueueHandle       = xQueueCreate( RTOS_QUEUE_SIZE, sizeof( RTOS_message_t ) );
	
		//Create Tasks:
		snrTaskHandle        = xTaskCreate(snrTask,       "snr",       RTOS_TASK_STACK_SIZE, NULL, RTOS_TASK_PRIORITY, NULL);	
	
	
	
	
	//Task Functions:
	void snrTask(void *param)
	{
	  while(1)
	  {
		  snrTaskApp();
	   }
	}

	
    ------------------------------------------------------------------------------------------------------
	Put the following in the sysControlTask.c :
	Note: snr needs to be all capitals.

	//Task Includes:	  
    #include "Task/snrTask.h"
	
	//Initialization flags
	static bool syscsnrInit = false;        //snr port has been intialized
	
	In "void sysControlTaskApp(void)" at the end of the while(1) loop
		if (syscSerInit && syscDsprInit && syscsnrInit && !syscSysInit)
	
    For task initalization:
	- Add a flag that will be used to tell the state machine that the task has been initalized and if it was successful:
	  in common.h, add a flag to typedef struct sys_init_complete and typedef struct sys_init_result. Decrement the number 
	    on the unused flags. Each section should add up to 8 bits. If there is no more room, add another section of 8 bits.
	
	- Add a call to the task's snr_CMD_INIT in the sysProcessSmSysInit function. Go to the last case statement and add
	  a new state SCTL_SM1_STXX:
	  
	    case SCTL_SM1_STXX:
            if (sysInitComplete.is_<your flag name>)
            {
                ESP_LOGI(SYS_TAG,"Initialize ...");
                sysSendMessage(MSG_ADDR_XXXX, MSG_DATA_0, XXXX_CMD_INIT,  NULL, MSG_DATA_COMMAND_ONLY, MSG_DATA_0_LEN);
                sysSeqState = SCTL_SM1_STXX;
            }
            break;

	- Add a check of the intialization results in the sysProcessSmSysInit function.	Go to the last case statement and 
	  look for code similar to the following. Add the new flags to the corresponding if statements:
	  
                if (sysInitResult.dsprInit      && 
                    sysInitResult.strgInit      &&
                    sysInitResult.serInit       &&
                    sysInitResult.LedInit       &&
                    sysInitResult.bleInit       &&
                    sysInitResult.wifiInit      &&
                    sysInitResult.scaleInit     &&
                    sysInitResult.batteryInit   &&
                    sysInitResult.keypadInit    &&
                    sysInitResult.alarmsInit    &&
                    sysInitResult.mountInit    &&
                    sysInitResult.dataQInit    &&
                    sysInitResult.logQInit		&&
					sysInitResult.snrInit)          <==================== add a new line 
                {
                    sysInitResult.systemInit = true;
                    ESP_LOGI(SYS_TAG,"System Initialization Passed");
                }
                else
                {
                    ESP_LOGI(SYS_TAG,"System Initialization Failed");
                    if (!sysInitResult.dsprInit)    {ESP_LOGE(SYS_TAG,"Dispatcher init Failed");}
                    if (!sysInitResult.strgInit)    {ESP_LOGE(SYS_TAG,"Storage init Failed");}
                    if (!sysInitResult.serInit)     {ESP_LOGE(SYS_TAG,"Serial init Failed");}
                    if (!sysInitResult.LedInit)     {ESP_LOGE(SYS_TAG,"Led init Failed");}
                    if (!sysInitResult.bleInit)     {ESP_LOGE(SYS_TAG,"Ble init Failed");}
                    if (!sysInitResult.wifiInit)    {ESP_LOGE(SYS_TAG,"Wifi init Failed");}
                    if (!sysInitResult.scaleInit)   {ESP_LOGE(SYS_TAG,"Scale init Failed");}
                    if (!sysInitResult.batteryInit) {ESP_LOGE(SYS_TAG,"battery init Failed");}
                    if (!sysInitResult.keypadInit)  {ESP_LOGE(SYS_TAG,"Keypad init Failed");}
                    if (!sysInitResult.alarmsInit)  {ESP_LOGE(SYS_TAG,"Alarms init Failed");}
                    if (!sysInitResult.mountInit)   {ESP_LOGE(SYS_TAG,"SPIFFS mount Failed");}
                    if (!sysInitResult.dataQInit)   {ESP_LOGE(SYS_TAG,"Data Queue init Failed");}
                    if (!sysInitResult.logQInit)    {ESP_LOGE(SYS_TAG,"Log Queue init Failed");}
                    if (!sysInitResult.snrInit)    {ESP_LOGE(SYS_TAG,"snr init Failed");}   <===== add a new line
                }	
			
			
	- Add a response case when the intialization response is received. Go to fucntion sctl_processSctlResponseMsg and add  
      a new case for snr_CMD_INIT:

	  	case snr_CMD_INIT:
            sysInitComplete.is_snrDone = true;
			if (sctl_Pm_sysCtrlRxMsg.msgData == snr_INIT_COMPLETE){sysInitResult.snrInit = true;}
			break;		
				

    ------------------------------------------------------------------------------------------------------
	Put the following in the dispatcherTask.c :
	
	In function "void dispatcherTaskApp(void)" under the "switch (dispatcherRxMessage.dstAddr)" statement
		case MSG_ADDR_snr:
		dispatcherTxMessage = dispatcherRxMessage;
		xQueueSend(snrQueueHandle,&dispatcherTxMessage,0);
		break;

    ------------------------------------------------------------------------------------------------------
	Put the following in the syncTask.c :	

	#include "Sync/snrSync.h"

	//execute sync functions:
	execsnrControl();	

    ------------------------------------------------------------------------------------------------------
	To send a stucture to another task:
    ------------------------------------------------------------------------------------------------------

	in common.c:
	----------------------

	Define the structure ID:
	------------------------
	Example:

	/// @brief Structure IDs
	enum structure_ids {
		STRUCT_UNKNOWN = -1,
		STRUCT_UNDEFINED = 0,
		SCALE_MEAS_RESULTS_STRUCT_ID = 1,
		BATTERY_MEAS_RESULTS_STRUCT_ID = 2,
	};

	Define the structure:
	---------------------
	Example:

	//battery Structures: Used for message data
	typedef struct Battery_Meas_Results {
		uint16_t batteryDataStructID;		//To identify the structure. This name needs to be entered into the "enum structure_ids"
			uint16_t adc_value;
			double   battery_voltage_raw;		
		double   battery_voltage_calibrated;
		} Battery_Meas_Results_t;


	in snrTask.c:
	------------------

	Define the structure variable:
	-------------------------------
	Example:

	Battery_Meas_Results_t MeasResults;

	Load the stucture with the data:
	---------------------------------
	Example:

	MeasResults.batteryDataStructID = BATTERY_MEAS_RESULTS_STRUCT_ID;
	MeasResults.adc_value = batteryAdcValue;
	MeasResults.battery_voltage_raw = batteryVoltageRaw;		
	MeasResults.battery_voltage_calibrated = batteryVoltageCalibrated;

	Send the structure to a task:
	-----------------------------
	Example:

	batterySendMessage(batteryRxMessage.srcAddr, MSG_DATA_STRUC, BATTERY_CMD_MEAS, (uint8_t *) &MeasResults.batteryDataStructID, 0, sizeof(MeasResults));

    ------------------------------------------------------------------------------------------------------
	To send the structure out of the serial port:
    ------------------------------------------------------------------------------------------------------

	The serial port needs to know how to convert the structure to a JSON string. You will need to define tags for each parmaeter you want to print out
	of the serial port.

	in common.h:
	------------

	define the tags:
	-----------------
	Example:

	/// @brief battery JSON tags
	#define JSON_BATTERY_ADC_VALUE          "bat_adc"
	#define JSON_BATTERY_VOLTAGE_RAW        "bat_bvr"
	#define JSON_BATTERY_VOLTAGE_CAL        "bat_bvc"

	in messages.c:
	-------------
	in function "print_convertMsgToJSON", case "MSG_DATA_STRUC", need to add the case when your function needs to be printed. There is a switch statement
	where you use the structure id as the case. Under that case, read the data from the structure and combine it into the JSON string being printed out.

	Define a temporary stucture, in the "print_convertMsgToJSON" function, to read the data from the RTOS message into.
	Example:
	--------
	void print_convertMsgToJSON(RTOS_message_t msg)
{
...

    Battery_Meas_Results_t batt_measurement;
    Scale_Meas_Results_t scale_measurement;


	Under the case "MSG_DATA_STRUC" add the code to read the data and insert itinto the JSON string with the corresponding tag.
	Example:
            switch (print_strucID)
            {
                case BATTERY_MEAS_RESULTS_STRUCT_ID:
                    memcpy(&batt_measurement,msg.msgDataPtr,msg.msgDataLen);
                    sprintf(
                       printMessage3,
                       "\"%s\":%d,\"%s\":%lf,\"%s\":%lf,",
                       JSON_BATTERY_ADC_VALUE, batt_measurement.adcValue,
                       JSON_BATTERY_VOLTAGE_RAW, batt_measurement.batteryVoltageRaw,
                       JSON_BATTERY_VOLTAGE_CAL, batt_measurement.batteryVoltageCalibrated                       
                    );
                    break;
                default:;


    ------------------------------------------------------------------------------------------------------	
	Creating a state machine - simple 
    ------------------------------------------------------------------------------------------------------
	1) create defines in common.h:
	
	enum system_control_sys_state {
		//sm idle
		SCTL_SEQ_STATE_IDLE        = 0,        //Idle state

		...
	 
		//<description of what these states are for>
		// XX is the next available number.
		SCTL_SQXX_ST10          = XX10,      
		SCTL_SQXX_ST20          = XX20,      
		SCTL_SQXX_ST30          = XX30,  
	};
	
	2) create function in system controller:
	-------------------
	void sysProcessSeq<sm name>(void)
	{
		switch(sysSeqState)
		{
			case SCTL_SEQ_STATE_IDLE: 
				ESP_LOGI(SYS_TAG,"<what does this state machine do?>");				
				sysSeqState = SCTL_SQXX_ST10;
				break;

			case SCTL_SQXX_ST10:
				ESP_LOGI(SYS_TAG,"Scale Data for MQTT Message (SGS testing only)");
				//sprintf(sys_TmpWifiStr, "{\"m\":\"%.01f,%.01f,%d\"}\r\n", 
				sprintf(sys_TmpWifiStr, "{\"sn\":\"%s\",\"m\":\"%.01f,%.01f,%d\"}\r\n", 
						advSerialNumber,
						sysScaleData.scaleWeight,
						sysScaleData.scaleBattVoltage,
						sysScaleData.scaleAlarms); 

				//sysLogI(sys_TmpWifiStr);
				sysSendMessage(MSG_ADDR_WIFI, MSG_DATA_STR, WIFI_CMD_MQTT_SEND_DATA, (uint8_t *)&sys_TmpWifiStr, 0, sizeof(sys_TmpWifiStr));
				sysGetDataComp.is_sendScaleDataDone = true;
				sysSetSeqSystemIdle();
				break;

			default:   
				sysSetSeqSystemIdle(); 
				ESP_LOGE(SYS_TAG,"sysProcessSeqGetScaleData - default");
				break;
		}
		if (sysCurrentSmComplete){
			ESP_LOGI(SYS_TAG,"sysProcessSeqGetScaleData - current Seq complete");
			}

	}

	
	
	
	
    ------------------------------------------------------------------------------------------------------	
	Creating a state machine - complex with flags 
    ------------------------------------------------------------------------------------------------------

    ------------------------------------------------------------------------------------------------------	
	Testing a state machine
    ------------------------------------------------------------------------------------------------------

	------------------------------------------------------------------------------------------------------	
	Creating a state machine sequence
    ------------------------------------------------------------------------------------------------------


    ------------------------------------------------------------------------------------------------------	
	REMOVE THESE NOTES BEFORE BUILDING CODE!!!
    ------------------------------------------------------------------------------------------------------

 /* [] END OF FILE */

