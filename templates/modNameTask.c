#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"

#include "Shared/common.h"
#include "Shared/messages.h"

#include "Task/modNameTask.h"
#include "Sync/modNameSync.h"


//Local Parameters:
static char modName_TAG[]="modName";
RTOS_message_t modNameRxMessage;
RTOS_message_t modNameTxMessage;
UBaseType_t modNameNewHighWaterMark = 0;
UBaseType_t modNameCurrHighWaterMark = 0;
UBaseType_t modNameInitHighWaterMark = 0;
uint8_t modNameStatus = modName_INIT_UNKNOWN;
char modNameTmpStr[STRING_MAX_ARRAY_CHARACTERS];

//Local Function Prototypes:
void modNameLog(char * strPtr, bool forced, bool printTag);
void modNameLogR(char * strPtr);
void modNameLogIF(char * strPtr);
void modNameLogEF(char * strPtr);
void modNameLogI(char * strPtr);
void modNameLogE(char * strPtr);
void modNameSendMessage(uint8_t dstAddr, uint8_t msgType, uint16_t msgCmd, uint8_t * msgDataPtr, uint32_t msgData, uint32_t msgDataLen);

//External Functions:

/// @brief modName task
/// @param  None
void modNameTaskApp(void)
{
    while(1)
    {
        if (xQueueReceive(modNameQueueHandle, &modNameRxMessage, portMAX_DELAY))
        {
			switch(modNameRxMessage.msgCmd)
			{
				case modName_CMD_INIT:
                    modNameStatus = modName_INIT_COMPLETE;
                    modNameSendMessage(modNameRxMessage.srcAddr, MSG_DATA_8, modName_CMD_INIT, NULL, modNameStatus, MSG_DATA_8_LEN);
                    break;
				case modName_CMD_PING:
                    modNameSendMessage(modNameRxMessage.srcAddr, MSG_DATA_8, modName_CMD_PING, NULL, modName_PING_RECEIVED, MSG_DATA_8_LEN);
					break;
				case modName_CMD_STATUS:
					//TBD
					break;

                case modName_CMD_LOG1_TEST:
                    modNameLogI("This is an information message from the modName module!");
                    break;    	

                case modName_CMD_LOG2_TEST:
                    modNameLogE("This is an error message from the modName module!");
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
void modNameLogR(char * strPtr)
{
    ESP_LOGI(modName_TAG, "%s", strPtr);
    modNameLog(strPtr, true, false);
}

/// @brief  This function sends the info log message to the BLE queue reguardless of 
///    the mobile device connected and/or the Log notification enabled. Iw also send
///    the log message to the terminal.
/// @param strPtr 
void modNameLogIF(char * strPtr)
{
    ESP_LOGI(modName_TAG, "%s", strPtr);
    modNameLog(strPtr, true, true);
}

/// @brief  This function sends the error log message to the BLE queue reguardless of 
///    the mobile device connected and/or the Log notification enabled. Iw also send
///    the log message to the terminal.
void modNameLogEF(char * strPtr)
{
    ESP_LOGE(modName_TAG, "%s", strPtr);
    modNameLog(strPtr, true, true);
}

/// @brief  This function send the info log message to the Mobile device and to the 
///    terminal.
/// @param strPtr 
void modNameLogI(char * strPtr)
{
    ESP_LOGI(modName_TAG, "%s", strPtr);
    modNameLog(strPtr, false, true);
}

/// @brief This function send the error log message to the Mobile device and to the 
///    terminal. 
/// @param strPtr 
void modNameLogE(char * strPtr)
{
    ESP_LOGE(modName_TAG, "%s", strPtr);
    modNameLog(strPtr, false, true);
}

/// @brief This function sends a message to the BLE module to send out a log message
///   to the mobile device via BLE. Mobile device has to be connected and the
///   log notifications (RX Characteristics) enabled.
/// @param strPtr - message to be sent to phone.
/// @param forced - If true, message is forced in the log queue.
/// @param printTag - If true the tag is printed with the message. 
void modNameLog(char * strPtr, bool forced, bool printTag)
{
    static uint8_t printIndex = 0;
    RTOS_message_t modPrintMsg;
    char tmpStr[STRING_MAX_LOG_CHARACTERS];
    static char modStr[<modName>_STRING_MAX_ARRAY_LOG_ELEMENTS][STRING_MAX_LOG_CHARACTERS];

    //copy string into next available string array
    //memcpy(modStr[printIndex], strPtr, strlen(strPtr)+1);
    memcpy(tmpStr, strPtr, strlen(strPtr)+1);
	if (printTag){
        sprintf(modStr[printIndex], "%s:%s", modName_TAG, tmpStr);
    }else{
        sprintf(modStr[printIndex], "%s", tmpStr);
    }

    //form message to print out string
    modPrintMsg.srcAddr         =  MSG_ADDR_modName; 
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
    if (printIndex >= <modName>_STRING_MAX_ARRAY_LOG_ELEMENTS)
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
void modNameSendMessage(uint8_t dstAddr, uint8_t msgType, uint16_t msgCmd, uint8_t * msgDataPtr, uint32_t msgData, uint32_t msgDataLen)
{  
    static uint8_t msgIndex = 0;
    RTOS_message_t sendRtosMsg;
    static char sendMsgArray[<modName>_MESSAGE_MAX_ARRAY_ELEMENTS][modName_STRING_MAX_ARRAY_CHARACTERS];

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
    sendRtosMsg.srcAddr = MSG_ADDR_modName;
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
    if (msgIndex >= <modName>_MESSAGE_MAX_ARRAY_ELEMENTS)
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
	
	//modName Task Defines:
	<any defines for the specific module>
	
	//External Queue Handles:	
	extern QueueHandle_t modNameQueueHandle;
	
	//External Semaphore Handles:
	extern SemaphoreHandle_t modNameSemaphoreHandle;
	
	Put the address of the module into the "enum message_address"
		MSG_ADDR_modName 		= XX,    //modName module

	Recommended addresses:
	MSG_ADDR_SCTL 		= 0,    //System Control
	MSG_ADDR_DSPR 		= 1,    //Dispatcher
	MSG_ADDR_SER 		= 2,    //Serial module
	MSG_ADDR_SUPR 		= 3,   	//Supervisor module
	MSG_ADDR_BLE 		= 4,	//BLE module
	MSG_ADDR_modName 		= 5,    //modName module
	MSG_ADDR_STRG 		= 6,    //Data storage module
	MSG_ADDR_ALARMS 	= 7,    //Alarms module
	MSG_ADDR_LED 		= 8,    //LED module
	MSG_ADDR_KEY 		= 9,   	//KeyPad module
	MSG_ADDR_SCALE 		= 10,   //Scale module
	MSG_ADDR_BATTERY 	= 11 	//Battery control
		
	//modName Structures: Used for message data
	typedef struct <structure name> {
		uint16_t modNameDataStructID;		//To identify the structure. This name needs to be entered into the "enum structure_ids"
		<Structure Parameters>;
		} <structure name>_t;
		
	//modName Structures: Not used for message data	
	typedef struct <structure name> {
		<Structure Parameters>;
		} <structure name>_t;

    //For state machines
	//---------------------------------------------------------------------------------
	//System state machines
	//---------------------------------------------------------------------------------

	//Add Sequences here:
	enum system_state_machines {
		//System test state machines
		SM_TST_SYS_INIT                 = 1,    //test sysProcessSmSysInit
		SM_TST_SYS_BEGIN                = 2,   //test sysProcessSmSysBegin 
		SM_TST_BLE_ADV_ON               = 3,    //test BLE on
		MAX_STATE_MACHINES              = 4,   //This is always the last one in the list 
	};		

	//---------------------------------------------------------------------------------
	// modName on State Machine
	//---------------------------------------------------------------------------------

	typedef struct sys_modName_complete
	{
	uint8_t start_SM			: 1;    //Start the state machine? 1 = yes, 0 = no
	uint8_t modName_unused		: 7;

	uint8_t modName_unused1		: 7;    
	uint8_t is_modNameDone		: 1;    //modName seqence complete
	} sys_modName_complete_t;

	typedef struct sys_modName_result
	{
	uint8_t modName_unused		: 8;

	uint8_t modName_unused1		: 7;
	uint8_t modNameResult		: 1;    //modName sequence passed
	} sys_modName_result_t;

	Note: the modName needs to be all capitals for the following:
	
	////////////////////////////////////////////////////////////////////////////////////////////////////////
	//modName Parameters:
	////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	All commands need to be unique. Here is the range for the corresponding tasks:
	
	SysControl 	- 	0     to 999
	dispatcher	-	1000  to 1999
	serial		-	2000  to 2999
	supervisor	-	3000  to 3999
	ble			- 	4000  to 4999
	modName		-	5000  to 5999
	storage		- 	6000  to 6999
	alarms		- 	7000  to 7999
	led			-	8000  to 8999
	keypad		-	9000  to 9999
	scale		-	10000 to 10999
	battery		-	11000 to 11999
	http		-   12000 to 12999
	
    enum modName_cmd_type {
        modName_CMD_UNKNOWN         = -1,
        modName_CMD_INIT            = xxxx,    //intialize task
        modName_CMD_PING            = xxxx,    //echo message received back to sender
        modName_CMD_STATUS          = xxxx,    //Send back status to sender

		modName_CMD_LOG1_TEST		= xxxx,		//Send information log to mobile device via BLE
		modName_CMD_LOG2_TEST		= xxxx,		//Send error log to mobile device via BLE
		<Add new commands here>
    };

    enum modName_status {
        modName_INIT_UNKNOWN  		= -1,
        modName_INIT_COMPLETE 		= 0,
        modName_INIT_ERROR    		= 1,
        modName_STATUS_GOOD   		= 2,
        modName_STATUS_FAILURE		= 3
    };  

    enum modName_ping_resp {
        modName_PING_RECEIVED 		= 0,
        modName_PING_ERROR    		= 1
    };

	#define SM_modName_STRING_MAX_ARRAY_CHARACTERS 150
	#define modName_STRING_MAX_ARRAY_CHARACTERS 150
	#define modName_MESSAGE_MAX_ARRAY_ELEMENTS  10
	#define modName_STRING_MAX_ARRAY_LOG_ELEMENTS 10
   
    ------------------------------------------------------------------------------------------------------
    put the following in the main.c file:


	//Task includes	  
    #include "Task/modNameTask.h"

	//Task Prototypes
	void modNameTask(void *param);
	
	//Queue Handles:
	QueueHandle_t modNameQueueHandle;
	
	//Semaphore Handles
	SemaphoreHandle_t modNameSemaphoreHandle = NULL;
	
	//Task Handles:
	BaseType_t modNameTaskHandle;
	
	//In the app_main function:

		//Define I/O: - If I/O is used, define it here.
		//Example:
		  gpio_set_direction(GPIO_NUM_33, GPIO_MODE_OUTPUT_OD);

    };
	
		//Create semaphores:
		vSemaphoreCreateBinary( smodNameSemaphoreHandle );
	
		//Create Queues:
		modNaleQueueHandle       = xQueueCreate( RTOS_QUEUE_SIZE, sizeof( RTOS_message_t ) );
	
		//Create Tasks:
		modNameTaskHandle        = xTaskCreate(modNameTask,       "modName",       RTOS_TASK_STACK_SIZE, NULL, RTOS_TASK_PRIORITY, NULL);	
	
	
	
	
	//Task Functions:
	void modNameTask(void *param)
	{
	  while(1)
	  {
		  modNameTaskApp();
	   }
	}

	
    ------------------------------------------------------------------------------------------------------
	Put the following in the sysControlTask.c :
	Note: modName needs to be all capitals.

	//Task Includes:	  
    #include "Task/modNameTask.h"

	//Command response flags
	static sys_modName_complete_t       sysModNameComplete;
	static sys_modName_result_t         sysModNameResult;
	
	//Initialization flags
	static bool syscmodNameInit = false;        //modName port has been intialized
	
	In "void sysControlTaskApp(void)" at the end of the while(1) loop
		if (syscSerInit && syscDsprInit && syscmodNameInit && !syscSysInit)
	
    For task initalization:
	- Add a flag that will be used to tell the state machine that the task has been initalized and if it was successful:
	  in common.h, add a flag to typedef struct sys_init_complete and typedef struct sys_init_result. Decrement the number 
	    on the unused flags. Each section should add up to 8 bits. If there is no more room, add another section of 8 bits.
	
	- Add a call to the task's modName_CMD_INIT in the sysProcessSmSysInit function. Go to the last case statement and add
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
					sysInitResult.modNameInit)          <==================== add a new line 
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
                    if (!sysInitResult.modNameInit)    {ESP_LOGE(SYS_TAG,"modName init Failed");}   <===== add a new line
                }	
			
			
	- Add a response case when the intialization response is received. Go to fucntion sctl_processSctlResponseMsg and add  
      a new case for modName_CMD_INIT:

	  	case modName_CMD_INIT:
            sysInitComplete.is_modNameDone = true;
			if (sctl_Pm_sysCtrlRxMsg.msgData == modName_INIT_COMPLETE){sysInitResult.modNameInit = true;}
			break;		


	//Add this to function sctl_processSctlResponseMsg
	        case modName_CMD_INIT:
            if (sctl_Pm_sysCtrlRxMsg.msgData == modName_INIT_COMPLETE){sysInitResult.modNameInit = true;}
            sysInitComplete.is_ModNameDone = true;
            break;	

	//Add modName init to the system control state machine:
	//---------------------------------------------------------------------------------
	//System initalizaion State Machine
	//---------------------------------------------------------------------------------

	typedef struct sys_init_complete
	{
	uint8_t start_SM			: 1;    //Start the state machine? 1 = yes, 0 = no
	uint8_t is_DsprDone			: 1;    //Dispatcher initalization complete
	uint8_t is_SerDone			: 1;    //Serial initalization complete
	uint8_t is_StrgDone			: 1;    //Storage initalization complete
	uint8_t is_BleDone			: 1;    //BLE initalization complete
	uint8_t is_WifiDone			: 1;    //Wifi initalization complete
	uint8_t is_ModNameDone		: 1;    //modName initalization complete   <===================== Add here
	uint8_t unused				: 1;

	uint8_t unused1				: 7;
	uint8_t is_SystemDone		: 1;    //System initalization complete
	} sys_init_complete_t;

	typedef struct sys_init_result
	{
	uint8_t dsprInit			: 1;    //Dispatcher initalization complete
	uint8_t serInit				: 1;    //Serial port has been intialized
	uint8_t strgInit			: 1;    //Storage module has been intialized
	uint8_t bleInit				: 1;    //BLE module has been intialized
	uint8_t wifiInit			: 1;    //Wifi module has been intialized
	uint8_t modNameInit          : 1;    //ModName module has been intialized <===================== Add here
	uint8_t unused				: 2;

	uint8_t unused1				: 7;
	uint8_t systemInit			: 1;    //System has been initalized
	} sys_init_result_t;

	typedef struct sys_begin_complete
	{                                   //true = yes
	uint8_t start_SM          : 1;    //Start the state machine?
	uint8_t is_SnReadInNvs    : 1;    //Was the currently stored Serial Number loaded into the NVS data structure?
	uint8_t is_SnReadInBLE    : 1;    //Was the currently stored Serial Number loaded into the BLE module?
	uint8_t is_BleStarted     : 1;    //Is the BLE module started?
	uint8_t is_ModNameStarted     : 1;    //Is the ModName module started?  <====================  Add here
	uint8_t unused1    		: 4;

	uint8_t unused2    		: 7;
	uint8_t is_SysBeginDone   : 1;    //System Begin complete?
	} sys_begin_complete_t;

	typedef struct sys_begin_result
	{
	uint8_t snInNvs           : 1;    //Was loading the Serial Number into the NVS data structure successful?
	uint8_t snInBLE           : 1;    //Was loading the Serial Number into the BLE module successful?
	uint8_t bleStartDone      : 1;    //Is the BLE module started successfully?
	uint8_t modNameStartDone      : 1;    //Is the modName module started successfully?  <================== Add here
	uint8_t uUnused1   		: 5;

	uint8_t uUnused2   		: 7;  
	uint8_t systemBegin       : 1;    //System has been initalized
	} sys_begin_result_t;					

    ------------------------------------------------------------------------------------------------------
	Put the following in the dispatcherTask.c :
	
	In function "void dispatcherTaskApp(void)" under the "switch (dispatcherRxMessage.dstAddr)" statement
		case MSG_ADDR_modName:
		dispatcherTxMessage = dispatcherRxMessage;
		xQueueSend(modNameQueueHandle,&dispatcherTxMessage,0);
		break;

    ------------------------------------------------------------------------------------------------------
	Put the following in the syncTask.c :	

	#include "Sync/modNameSync.h"

	//execute sync functions:
	execmodNameControl();	

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


	in modnameTask.c:
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


	//In stateMachines.c:
	//Add this to the

	        case SCTL_SM1_STXX:
                if (sysInitComplete->is_ModNameDone)
                {
                    if(sysInitResult->modNameInit)
                    {
                        smSysInitState = SCTL_SM1_ST98;
                    }
                    else
                    {
                        ESP_LOGE(SM_TAG,"ModName init Failed");
                        smSysInitState = SCTL_SM1_ST99;
                    }
                }
                break;

	//In stateMachines.h:



 /* [] END OF FILE */

