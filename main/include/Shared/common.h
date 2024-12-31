#ifndef SHARED_FILES_HEADER_FILES_COMMON_H_
#define SHARED_FILES_HEADER_FILES_COMMON_H_

#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"

#define FIRMWARE_VERSION_RELEASE 	0
#define FIRMWARE_VERSION_MAJOR 		0
#define FIRMWARE_VERSION_MINOR 		1

uint32_t sys_getMsgTimeStamp(void);
#define SYNC_TIME	1000	//once every 1ms
#define STRING_MAX_BLE_CON_CMD_CHAR 50
#define BLE_CON_DELAY	10

//////////////////////////////////////////////////////////////////////////////////
// Compiler Directives
//////////////////////////////////////////////////////////////////////////////////

//#define SLEEP_TIMER_OFF                 //Disable the sleep timer
//#define TEST_MODE                       //Disable normal startup and go into a test mode
//#define BLE_DATA_GATEWAY                //Enable BLE gateway test

#define WAKE_TIME_IN_MIN 5  //5 is default
#define BLE_INACTIVITY_TIME_MS (5 * ONE_MINUTE_IN_SEC * ONE_SECOND_IN_MS) 
#define SYS_INACTIVITY_TIME_MS (5 * ONE_MINUTE_IN_SEC * ONE_SECOND_IN_MS) 

//#define BROWNOUT_DETECT_OFF             //Disable brownout detection
//#define BROWNOUT_OVERRIDE true          //Used to override the brownout test

//#define TIME_DURATION_OVERRIDE          //Used to override time duration between data uploads
//#define OVERRIDE_UPLOAD_DURATION_MIN  360 //30  //upload when time exceed x min, default is 360 min (every 6 hours)

//#define SLEEP_DURATION_OVERRIDE         //Used to override the time the scale stays asleep
//#define SLEEP_OVERRIDE_MIN  60 //5           //sleep time in minutes, default is 60 min

//#define OVERRIDE_CONFIGURATION          //If defined, will override the scale configuration with a test string
//#define OVERRIDE_OTA_URL               //If defined, will override the url used to get the latest binary file with a test string

#define SQ_GET_DATE_TIME_REPEAT_COUNT 10   //Set to 0 to test rtc failure alarm

#define NEW_SCALE                   //If this is defined, the code is for the new scale with changed IO pin for the hidden button.

#define ENABLE_OTA                    //If defined, will do an OTA test

#define BLE_SECURITY          //When defined the BLE security will be enabled by default


//////////////////////////////////////////////////////////////////////////////////


//External Queue Handles:
extern QueueHandle_t serialQueueHandle;
extern QueueHandle_t sysControlQueueHandle;
extern QueueHandle_t dispatcherQueueHandle;
extern QueueHandle_t strgQueueHandle;
extern QueueHandle_t bleQueueHandle;

extern QueueHandle_t eventQueueHandle;
extern QueueHandle_t logQueueHandle;

typedef struct RTOS_message {
	uint8_t srcAddr;
	uint8_t dstAddr;
	uint32_t msgRef;
	uint32_t msgTimeStamp;
	uint8_t msgType;
	uint16_t msgCmd;
	uint8_t *msgDataPtr;
	uint32_t msgData;
	uint32_t msgDataLen;
} RTOS_message_t;

enum message_address {
	MSG_ADDR_UNKNOWN 	= -1, 	//Unknown address
	MSG_ADDR_SCTL 		= 0,    //System Control
	MSG_ADDR_DSPR 		= 1,    //Dispatcher
	MSG_ADDR_SER 		= 2,    //Serial module
	MSG_ADDR_SUPR 		= 3,   	//Supervisor module
    MSG_ADDR_STRG       = 4,    //Storage module
    MSG_ADDR_BLE        = 5,    //BLE module
    MSG_ADDR_WIFI       = 6,    //Wifi module
};

enum message_data_type {
	MSG_DATA_UNKNOWN 	= -1,
	MSG_DATA_0 			= 1,    //No Data
	MSG_DATA_8 			= 2,    //8 bit data
	MSG_DATA_16 		= 3,    //16 bit data
	MSG_DATA_32 		= 4,    //32 bit data
	MSG_DATA_STR 		= 5,    //pointer to Data in memory representing a String
	MSG_DATA_STRUC 		= 6,   	//pointer to Data in memory representing a structure 1
	MSG_DATA_FLOAT 		= 7,    //float
    MSG_DATA_EVENT      = 8,    //Event 
    MSG_DATA_OBJECT      = 9,    //Object 
	MSG_DATA_UNDEFINED 	= 99 	//pointer to Data in memory representation undefined
};

enum message_data_len {
	MSG_DATA_0_LEN 		= 0,    //No Data
	MSG_DATA_8_LEN 		= 1,    //8 bit data
	MSG_DATA_16_LEN 	= 2,    //16 bit data
	MSG_DATA_32_LEN 	= 3,    //32 bit data
};

/// @brief Structure IDs
enum structure_ids {
	STRUCT_UNKNOWN = -1,
	STRUCT_UNDEFINED = 0,
	SCALE_MEAS_RESULTS_STRUCT_ID = 1,
	BATTERY_MEAS_RESULTS_STRUCT_ID = 2,
	DEVICE_DATA_STRUCT_ID = 3,
    SCALE_CONFIG_STRUCT_ID = 4,
};

enum system_wakeup_reasons {
    WKUP_UNKNOWN        = 0,
    WKUP_TIMER          = 1,
    WKUP_POWER_ON       = 2,
    WKUP_EXTERN0        = 3,
    WKUP_EXTERN1        = 4,
};

//---------------------------------------------------------------------------------
//Module command range
//---------------------------------------------------------------------------------

// All commands need to be unique. Here is the range for the corresponding tasks:

// SysControl 	- 	0     to 999
// dispatcher	-	1000  to 1999
// serial		-	2000  to 2999
// supervisor	-	3000  to 3999
// storage		- 	4000  to 4999
// ble			- 	5000  to 5999
// Wifi		    -	6000  to 6999
// mod1		    - 	7000  to 7999
// mod2			-	8000  to 8999
// mod3		    -	9000  to 9999
// mod4		    -	10000 to 10999
// mod5		    -	11000 to 11999
// mod6		    -   12000 to 12999

//RTOS defines:
#define RTOS_TASK_STACK_SIZE_2K	1024*2	//bytes
#define RTOS_TASK_STACK_SIZE_3K	1024*3	//bytes
#define RTOS_TASK_STACK_SIZE_4K	1024*4	//bytes
#define RTOS_TASK_STACK_SIZE_5K	1024*5	//bytes

#define RTOS_QUEUE_SIZE			16		//messages
#define EVENT_QUEUE_SIZE		16		//messages
#define LOG_QUEUE_SIZE			25		//messages

#define RTOS_TASK_PRIORITY		3

//System defines
#define DISP_STRING_MAX_ARRAY_CHARACTERS 	150 
#define SER_STRING_MAX_ARRAY_CHARACTERS 	150 
#define SYS_STRING_MAX_ARRAY_CHARACTERS 	150

#define MAX_ADV_SERIAL_NUMBER_LEN           30
#define MAX_HW_REV_LEN						10

#define MESSAGE_MAX_ARRAY_ELEMENTS			10
#define SER_MESSAGE_MAX_ARRAY_ELEMENTS	    10
#define SYS_MESSAGE_MAX_ARRAY_ELEMENTS	    10

#define MESSAGE_MAX_ARRAY_BYTES		sizeof(RTOS_message_t)
#define MSG_DATA_POINTER_ONLY	0
#define MSG_DATA_COMMAND_ONLY	0

#define EVT_MIN     60000   //Event command min value
#define EVT_MAX     64899   //Event command max value

#define QUARTER_SECOND	250
#define HALF_SECOND  	500
#define ONE_SECOND  	1000
#define ONE_SECOND_IN_MS  	1000
#define FIVE_SECONDS	(5 * ONE_SECOND)
#define ONE_MINUTE  	(60 * ONE_SECOND)
#define FIVE_MINUTE  	(5 * ONE_SECOND)
#define ONE_HOUR  		(60 * ONE_MINUTE)

#define  ONE_MINUTE_IN_SEC  	60
#define  ONE_HOUR_IN_SEC  		(60 * ONE_MINUTE_IN_SEC)
#define uS_TO_S_FACTOR 1000000 // Conversion factor for microseconds to seconds

#define MAX_QUEUE_DATA_COUNT  70U

#define SOFTWARE_VERSION_MAX_CHARACTERS 10
#define SOFTWARE_URL_MAX_CHARACTERS 1000

#define MAX_DATA_STRING_LENGTH 250 //For data and log messages

//---------------------------------------------------------------------------------
//State machine timeouts
//---------------------------------------------------------------------------------

#define SM_SYS_INIT_TIMEOUT             5 * ONE_MINUTE_IN_SEC * ONE_SECOND_IN_MS
#define SM_SYS_BEGIN_TIMEOUT            5 * ONE_MINUTE_IN_SEC * ONE_SECOND_IN_MS
#define SM_SYS_BLE_CON_TIMEOUT          5 * ONE_MINUTE_IN_SEC * ONE_SECOND_IN_MS



//---------------------------------------------------------------------------------
//System status
//---------------------------------------------------------------------------------

typedef struct sys_status
{
  uint8_t bleConnected          : 1;
  uint8_t wakeupReason          : 3;    //Wakeup reason: 0-unknown, 1-power on, 2-external wakeup 
  uint8_t unused1       	    : 4;
  
  uint8_t unused2       	    : 8;
} sys_status_t;


//---------------------------------------------------------------------------------
//System events
//---------------------------------------------------------------------------------

enum system_events {
    //SysControl 	- 	60000  to 60399 
    EVT_SYS_IDLE                = 60000,    //No new sequence
    EVT_SYS_INIT                = 60001,    //Initialize system
    EVT_SYS_TEST_SM             = 60002,    //Test state machine
    EVT_SYS_STARTUP             = 60003,    //Start startup workflow after initialization
    EVT_SYS_POWER_ON            = 60004,    //Power on workflow
    EVT_SYS_TIMER_WAKE_UP       = 60005,    //Timer wakeup event

    //dispatcher	-	60400  to 60899
    
    //serial		-	60900  to 61299
   
	//supervisor	-	61300  to 61699
	
    //storage		- 	61700  to 62099
    EVT_STRG_CFG_CNG_BURL       = 61700,    //Config change - Base URL
    EVT_STRG_CFG_CNG_LID        = 61701,    //Config change - location Id
    EVT_STRG_CFG_CNG_PID        = 61702,    //Config change - provider Id
    EVT_STRG_CFG_CNG_WRET       = 61703,    //Config change - wifi_retries
    EVT_STRG_CFG_CNG_SDUR       = 61704,    //Config change - sleep_duration
    EVT_STRG_CFG_CNG_LBT        = 61705,    //Config change - low_batt_threshold
    EVT_STRG_CFG_CNG_BRT        = 61706,    //Config change - brownout_threshold
    EVT_STRG_CFG_CNG_UDUR       = 61707,    //Config change - upload_duration


    //ble			- 	62100  to 62499
    EVT_BLE_SSID_RD             = 62100,    //Send current SSID stored to phone
    EVT_BLE_PWD_RD              = 62101,    //Send current Password stored to phone
    EVT_BLE_SCALE_RD            = 62102,    //Send current scale weight to phone

    EVT_BLE_ADV_ON              = 62103,    //Advertisement has been turned on
    EVT_BLE_ADV_OFF             = 62104,    //Advertisement has been turned off
    EVT_BLE_CONNECT             = 62105,    //BLE has conneted to a device
    EVT_BLE_DISCONNECT          = 62106,    //BLE has disconneted from a device
    EVT_BLE_KEEP_ALIVE          = 62107,    //BLE module has received a keep alive message
    EVT_APP_BLE_PWD_RD          = 62108,    //Mobile app requesting encrypted PWD
    EVT_APP_BLE_SSID_RD         = 62109,    //Mobile app requesting encrypted SSID

    //wifi		    -	62500  to 62899
    EVT_CMD_WIFI_CONNECT        = 62500, //Wifi Get SSID and PWD, Init Wifi, Connect wifi, init sntp and wifi get date and time.
    EVT_CMD_WIFI_DISCONNECT     = 62501, //Wifi disconnect command.
    EVT_CMD_WIFI_SCAN           = 62502, //Scan for available Wifi access points.
    EVT_CMD_BLE_WIFI_CONNECT    = 62503, //Process request to connect from the phone

    EVT_WIFI_CONNECTED          = 62504,    //Wifi is connected

    EVT_ENTER_TANK_CAL          = 62505,    //Enter tank calibration mode
    EVT_EXIT_TANK_CAL           = 62506,    //Exit tank calibration moded

	//mod1			- 	62900  to 63299
	//mod2			-	63300  to 63699
	//mod3        	-	63700  to 64099  
	//mod4		    -	64100  to 64499
	//mod5			-	64500  to 64899

    //Exceptions - treat as event under certain circumstances
    //           - If the circumstance is an event then treat as event otherwise treat as incomming message
    //           - Range: 64900 to 65535
   
};


//---------------------------------------------------------------------------------
//System state machines
//---------------------------------------------------------------------------------

enum system_state_machines {
    //System test state machines
    SM_TST_SYS_INIT                 = 1,    //test sysProcessSmSysInit
	SM_TST_SYS_BEGIN                = 2,   //test sysProcessSmSysBegin 
    SM_TST_BLE_ADV_ON               = 3,    //test BLE on
    MAX_STATE_MACHINES              = 4,   //This is always the last one in the list 
};


//---------------------------------------------------------------------------------
//System initalizaion State Machine
//---------------------------------------------------------------------------------

typedef struct sys_init_complete
{
  uint8_t start_SM          : 1;    //Start the state machine? 1 = yes, 0 = no
  uint8_t is_DsprDone       : 1;    //Dispatcher initalization complete
  uint8_t is_SerDone        : 1;    //Serial initalization complete
  uint8_t is_StrgDone       : 1;    //Storage initalization complete
  uint8_t is_BleDone        : 1;    //BLE initalization complete
  uint8_t is_WifiDone       : 1;    //Wifi initalization complete
  uint8_t unused1			: 2;

  uint8_t is_mountDone      : 1;    //SPIFFS Mount complete
  uint8_t is_dataQDone      : 1;    //Data Queue initialization complete
  uint8_t is_logQDone       : 1;    //Log Queue initialization complete
  uint8_t unused2       	: 4;
  uint8_t is_SystemDone     : 1;    //System initalization complete
} sys_init_complete_t;

typedef struct sys_init_result
{
  uint8_t dsprInit          : 1;    //Dispatcher initalization complete
  uint8_t serInit           : 1;    //Serial port has been intialized
  uint8_t strgInit          : 1;    //Storage module has been intialized
  uint8_t bleInit           : 1;    //BLE module has been intialized
  uint8_t wifiInit          : 1;    //Wifi module has been intialized
  uint8_t unused1       	: 3;

  uint8_t mountInit         : 1;    //SPIFFS has been mounted
  uint8_t dataQInit         : 1;    //Data queue has been initialized
  uint8_t logQInit          : 1;    //Log queue has been initialized
  uint8_t unused2       	: 4;
  uint8_t systemInit        : 1;    //System has been initalized
} sys_init_result_t;

//---------------------------------------------------------------------------------
//System begin State Machine
//---------------------------------------------------------------------------------

typedef struct sys_begin_complete
{                                   //true = yes
  uint8_t start_SM          : 1;    //Start the state machine?
  uint8_t is_SnReadInNvs    : 1;    //Was the currently stored Serial Number loaded into the NVS data structure?
  uint8_t is_SnReadInBLE    : 1;    //Was the currently stored Serial Number loaded into the BLE module?
  uint8_t is_BleStarted     : 1;    //Is the BLE module started?
  uint8_t unused1    		: 4;

  uint8_t unused2    		: 7;
  uint8_t is_SysBeginDone   : 1;    //System Begin complete?
} sys_begin_complete_t;

typedef struct sys_begin_result
{
  uint8_t snInNvs           : 1;    //Was loading the Serial Number into the NVS data structure successful?
  uint8_t snInBLE           : 1;    //Was loading the Serial Number into the BLE module successful?
  uint8_t bleStartDone      : 1;    //Is the BLE module started successfully?
  uint8_t uUnused1   		: 5;

  uint8_t uUnused2   		: 7;  
  uint8_t systemBegin       : 1;    //System has been initalized
} sys_begin_result_t;


typedef struct EVENT_message {
	uint16_t evtCmd;
    uint32_t evtData;	
    uint32_t *evtDataPtr;
	uint16_t evtDataLen;	
} EVENT_message_t;

//---------------------------------------------------------------------------------
//BLE on State Machine
//---------------------------------------------------------------------------------

typedef struct sys_ble_complete
{
  uint8_t start_SM              : 1;    //Start the state machine? 1 = yes, 0 = no
  uint8_t is_SN_Set             : 1;    //Wifi to get latest Serial Number
  uint8_t is_Adv_On_Done        : 1;    //Turn BLE Advertising On
  uint8_t is_Avd_Off_Done       : 1;    //Turn BLE Advertising Off
  uint8_t ble_unused1           : 4;

  uint8_t ble_unused2           : 7;    
  uint8_t is_BleDone            : 1;    //Ble seqence complete
} sys_ble_complete_t;

typedef struct sys_ble_result
{
  uint8_t Adv_On                : 1;    //BLE Advertising is on
  uint8_t Adv_Off               : 1;    //BLE Advertising is off
  uint8_t SN_Loaded             : 1;    //Serial Number loaded into wifi module
  uint8_t ble_unused3           : 5;

  uint8_t ble_unused4           : 7;
  uint8_t bleResult             : 1;    //Ble sequence passed
} sys_ble_result_t;

//---------------------------------------------------------------------------------
//System Controller Parameters
//---------------------------------------------------------------------------------

#define SCTL_CMD_MIN_VALUE      0
#define SCTL_CMD_MAX_VALUE      999

#define SYS_INIT_TEST_COMPLETE  0x00FF
#define SYS_INIT_TEST_MASK      0xFF00

#define SCTL_CMD_MIN_VALUE      0
#define SCTL_CMD_MAX_VALUE      999

#define EVT_MIN     60000   //Event command min value
#define EVT_MAX     64899   //Event command max value

enum system_control_cmd {
    //SysControl 	- 	0 to 999
	SCTL_CMD_UNKNOWN 	        = -1,   //Unknown Command
	SCTL_CMD_INIT 		        = 1,    //request initialization
	SCTL_CMD_PING 		        = 2,    //send PING request
	SCTL_CMD_STATUS 	        = 3,    //Request status

    // Command sequences
	SCTL_CMD_TEST_SM            = 20,   //Used to test a specific state machine. Send the state machine index with this 
    SCTL_CMD_TEST_EVT           = 21,   //Used to insert a given event into the event queue     

};

typedef struct sys_seq
{  
  //1 = active sequence, 0 = inactive sequence  
  uint8_t init   		        : 1;    //init
  uint8_t sm_test               : 1;    //test state machine
  uint8_t startup         	    : 1;    //startup 
  uint8_t power_on              : 1;    //power on
  uint8_t unused1          	    : 4;

  uint8_t unused2          	    : 8;
} sys_seq_t;

enum system_control_sys_state {
    //seq idle
    SCTL_SEQ_STATE_IDLE        = 0,        //Idle state

    //sm sys init
    SCTL_SM1_ST00          = 100,       
    SCTL_SM1_ST10          = 110,       
    SCTL_SM1_ST14          = 114,       
    SCTL_SM1_ST15          = 115,       
    SCTL_SM1_ST16          = 116,       
    SCTL_SM1_ST17          = 117,       
    SCTL_SM1_ST20          = 120,       
    SCTL_SM1_ST30          = 130,  
    SCTL_SM1_ST40          = 140,  
    SCTL_SM1_ST50          = 150,  
    SCTL_SM1_ST60          = 160,  
    SCTL_SM1_ST70          = 170,  
    SCTL_SM1_ST80          = 180,   
    SCTL_SM1_ST90          = 190,  
    SCTL_SM1_ST98          = 198,  
    SCTL_SM1_ST99          = 199,

	//sm begin
    SCTL_SM2_ST00          = 200,       
    SCTL_SM2_ST10          = 210,       
    SCTL_SM2_ST20          = 220,       
    SCTL_SM2_ST30          = 230,      
    SCTL_SM2_ST40          = 240,  
    SCTL_SM2_ST50          = 250,  
    SCTL_SM2_ST60          = 260,  
    SCTL_SM2_ST70          = 270,  
    SCTL_SM2_ST80          = 280,  
    SCTL_SM2_ST90          = 290,  
    SCTL_SM2_ST98          = 298,  
    SCTL_SM2_ST99          = 299, 

	//BLE on
    SCTL_SM3_ST00          = 300,       
    SCTL_SM3_ST05          = 305,       
    SCTL_SM3_ST10          = 310,       
    SCTL_SM3_ST20          = 320,       
    SCTL_SM3_ST30          = 330,      
    SCTL_SM3_ST40          = 340,  
    SCTL_SM3_ST50          = 350,  
    SCTL_SM3_ST60          = 360,  
    SCTL_SM3_ST70          = 370,  
    SCTL_SM3_ST80          = 380,  
    SCTL_SM3_ST90          = 390,  
    SCTL_SM3_ST98          = 398,  
    SCTL_SM3_ST99          = 399,                 
};

enum system_control_sequence_states {
    //event - idle
    SCTL_EVT_STATE_IDLE        = 0,        //Idle state

    //event - sys init (EVT_SYS_INIT)
    SCTL_SEQ1_ST10          = 110,       
    SCTL_SEQ1_ST20          = 120,       
    SCTL_SEQ1_ST30          = 130,  
    SCTL_SEQ1_ST40          = 140,   
    SCTL_SEQ1_ST50          = 150,   
    SCTL_SEQ1_ST60          = 160,   
    SCTL_SEQ1_ST70          = 170,   
    SCTL_SEQ1_ST80          = 180,   
    SCTL_SEQ1_ST90          = 190,   
    SCTL_SEQ1_ST98          = 198,   
    SCTL_SEQ1_ST99          = 199, 

    //event - sys startup (EVT_SYS_STARTUP)
    SCTL_SEQ2_ST10          = 210,       
    SCTL_SEQ2_ST20          = 220,       
    SCTL_SEQ2_ST30          = 230,  
    SCTL_SEQ2_ST40          = 240,   
    SCTL_SEQ2_ST50          = 250,   
    SCTL_SEQ2_ST60          = 260,   
    SCTL_SEQ2_ST70          = 270,   
    SCTL_SEQ2_ST80          = 280,   
    SCTL_SEQ2_ST90          = 290,   
    SCTL_SEQ2_ST98          = 298,   
    SCTL_SEQ2_ST99          = 299, 	

	//event - test state machines (EVT_SYS_TEST_SM)
    SCTL_SEQ3_ST00          = 300,       
    SCTL_SEQ3_ST10          = 310,       
    SCTL_SEQ3_ST20          = 320,       
    SCTL_SEQ3_ST30          = 330,      
    SCTL_SEQ3_ST40          = 340,  
    SCTL_SEQ3_ST50          = 350,  
    SCTL_SEQ3_ST60          = 360,  
    SCTL_SEQ3_ST70          = 370,  
    SCTL_SEQ3_ST80          = 380,  
    SCTL_SEQ3_ST90          = 390,  
    SCTL_SEQ3_ST98          = 398,  
    SCTL_SEQ3_ST99          = 399, 

    //event - test state machines (EVT_SYS_POWER_ON)
    SCTL_SEQ4_ST00          = 400,       
    SCTL_SEQ4_ST10          = 410,       
    SCTL_SEQ4_ST20          = 420,       
    SCTL_SEQ4_ST30          = 430,      
    SCTL_SEQ4_ST40          = 440,  
    SCTL_SEQ4_ST50          = 450,  
    SCTL_SEQ4_ST60          = 460,  
    SCTL_SEQ4_ST70          = 470,  
    SCTL_SEQ4_ST80          = 480,  
    SCTL_SEQ4_ST90          = 490,  
    SCTL_SEQ4_ST98          = 498,  
    SCTL_SEQ4_ST99          = 499,      
};

enum system_control_status {
	SCTL__INIT_UNKNOWN  = -1,
	SCTL_INIT_COMPLETE 	= 0, 
	SCTL_INIT_ERROR 	= 1,
	SCTL_STATUS_GOOD 	= 3, 
	SCTL_STATUS_FAILURE = 4,
	SCTL_PING_RECEIVED 	= 5, 
	SCTL_PING_ERROR 	= 6
};



//---------------------------------------------------------------------------------
//Dispatcher Parameters
//---------------------------------------------------------------------------------

enum dispatcher_cmd_type {
	DSPR_CMD_UNKNOWN 	= -1, 
	DSPR_CMD_INIT 		= 1001,    //intialize task
	DSPR_CMD_PING 		= 1002,    //echo message received back to sender
	DSPR_CMD_STATUS 	= 1003,    //Send back status to sender
	DSPR_CMD_CONF 		= 1004     //Configure task
};

enum dispatcher_status {
	DSPR_INIT_COMPLETE = 0, 
	DSPR_INIT_ERROR = 1
};

enum dispatcher_ping_resp {
	DSPR_PING_RECEIVED = 0, 
	DSPR_PING_ERROR = 1
};

//---------------------------------------------------------------------------------
//Serial Parameters
//---------------------------------------------------------------------------------

//Serial task defines:
#define JSON_RX_BUFFER_SIZE 	1024
#define JSON_ERR_BUFFER_SIZE 	200
#define SERIAL_RX_BUFFER_SIZE 	10

#define EMI_TEST_START  true
#define EMI_TEST_STOP   false


enum serial_cmd_type {
	SER_CMD_UNKNOWN 			= -1,
	SER_CMD_INIT 				= 2001,	//Initialize task
	SER_CMD_PING 				= 2002,	//echo message received back to sender
	SER_CMD_STATUS 				= 2003,	//Send back status to sender
	SER_CMD_PRINT 				= 2004,	//Print message out of serial port

	SER_CMD_PRINT_FW_VER 	    = 2010,	//Send FW version out serial port in a JSON string.
	SER_CMD_PRINT_HW_VER 	    = 2011,	//Send HW version out serial port in a JSON string.

    SER_CMD_CON_CMD             = 2100 //Process a console command received in another task.
};

enum serial_response {
	SER_INIT_COMPLETE   = 0,
	SER_INIT_ERROR      = 1,
	SER_PING_RECEIVED   = 2,
	SER_PING_ERROR      = 3,
	SER_PRINT_COMPLETE  = 4,
    SER_PRINT_ERROR     = 5,    
};



//---------------------------------------------------------------------------------
//BLE Parameters
//---------------------------------------------------------------------------------

#define MAX_ADV_SERIAL_NUMBER_LEN               30
#define NOTIFICATION_TIMER (15 * ONE_SECOND)
#define RECEIVE_BLE_DATA_TIMEOUT (1 * ONE_SECOND) //Used when the phone is sending data to the scale such as SSID and PWD
#define ADV_TIMER (2 * ONE_MINUTE)
#define BLE_NAME_FMT_EMPTY "XX:"

//  The max length of characteristic value. When the GATT client performs 
//    a write or prepare write operation,the data length must be less than GATTS_CHAR_VAL_LEN_MAX.
#define GATTS_CHAR_VAL_LEN_MAX                  100U                   
#define GATTS_CHAR_SSID_LEN_MAX                 51U                   
#define GATTS_CHAR_PWD_LEN_MAX                  51U	
#define GATTS_CON_VAL_LEN_MAX                   51U     //Max amount of characters received from the BLE console write command	
#define GATTS_INDEX_VAL_LEN_MAX                 10U

#define PREPARE_BUF_MAX_SIZE                    1024U
#define CHAR_DECLARATION_SIZE                   (sizeof(uint8_t))
#define INT_DECLARATION_SIZE                    (sizeof(uint32_t))
#define PROFILE_NUM                             1U
#define PROFILE_APP_IDX                         0U
#define PROFILE_A_APP_ID 						0U
#define ESP_LOX_APP_ID                              0x55
#define SAMPLE_DEVICE_NAME                      "ESP_GATTS_DEMO"
#define SVC_INST_ID                             0U
#define JSON_PAYLOAD_LEN                        180U
#define MAX_MTU_SIZE                            180U
#define SERIAL_NO_SIZE                          10U
#define SERIAL_NO_OFFSET                        10U

#define MAX_SCALE_SERIAL_NUMBER_LEN             15 //50
#define DEFALT_BLE_PASSKEY                      501134; //"hellos"

#define ADV_CONFIG_FLAG                         (1UL << 0U)
#define SCAN_RSP_CONFIG_FLAG                    (1UL << 1U)

#define BLE_MESSAGE_WAIT_DELAY_MS   10     //Wait time (in ms) for a BLE task message

#define BLE_MAX_CON_JSON_STRING         100     //Max characters allowed in the console command.
#define BLE_MAX_CON_PARM_LEN            20      //Max parameter length.
#define BLE_MAX_CON_PARMS               5       //Max parameters allowed for each console command including the command.

//Wifi Connect functions
#define CHAR_WIFI_CONNECT_FUNC      1
#define CHAR_WIFI_ENTER_TANK_CAL    2
#define CHAR_WIFI_EXIT_TANK_CAL     3

//BLE Encryption
#define GATTS_TABLE_TAG "SEC_GATTS_DEMO"

#define SM_BLE_STRING_MAX_ARRAY_CHARACTERS 150
#define BLE_STRING_MAX_ARRAY_CHARACTERS 150
#define BLE_MESSAGE_MAX_ARRAY_ELEMENTS  10

#define BLE_DEVICE_PREFIX   "OP"

//ble			- 	5000  to 5999
enum ble_cmd_type {
	BLE_CMD_UNKNOWN         = -1,
	BLE_CMD_INIT            = 5001,     //intialize task
	BLE_CMD_PING            = 5002,     //echo message received back to sender
	BLE_CMD_STATUS          = 5003,     //Send back status to sender
	BLE_CMD_ADV_START		= 5004,	    //Turn advertising on
	BLE_CMD_ADV_STOP		= 5005,		//Turn off 
	BLE_CMD_SET_SN		    = 5006,		//Read S/N from storage and set advSerialNumber 
	BLE_CMD_INIT_ADV	    = 5007,		//Initialize BLE advertisement 
	BLE_CMD_GET_IDLE_TM	    = 5008,		//Get the idle time of ble activity 

	BLE_CMD_ADV_TEST1	    = 5050,		//Set advertisement to a new string

    BLE_CMD_SEND_LOG        = 5109,     //Send received message to the Mobile device via BLE RX notifications
    BLE_CMD_SEND_FORCED_LOG = 5110,     //Send received start message to the ble notification queue even if the
                                        // BLE is not connected and/or log notifications is not enabled.

    //BLE Notify commands:                                    

    //BLE Console commands                             
    BLE_VER_CON_CMD         = 5300,     //Command received from the BLE console (phone) to get Firmware version                                   
    BLE_CMD_RD_SSID         = 5303,     //Command received from the BLE console (phone) to get the stored SSID                               
    BLE_CMD_RD_PWD          = 5304,     //Command received from the BLE console (phone) to get the stored password  
    BLE_CMD_RD_SN           = 5310,     //Command received from the BLE console (phone) to get the Serial Number stored 
    BLE_CMD_RD_BASE_URL     = 5312,     //Command received from the BLE console (phone) to read base URL 

    BLE_CMD_RD_LOC_ID       = 5313,     //Command received from the BLE console (phone) to read location Id 
    BLE_CMD_RD_PROV_ID      = 5314,     //Command received from the BLE console (phone) to read provider Id 
    BLE_CMD_RD_WIFI_RET     = 5315,     //Command received from the BLE console (phone) to read wifi_retries 
    BLE_CMD_RD_SLP_DUR      = 5317,     //Command received from the BLE console (phone) to read sleep_duration 
    BLE_CMD_RD_UPL_DUR      = 5318,     //Command received from the BLE console (phone) to read upload_duration 
    BLE_CMD_RD_LBAT_THR     = 5319,     //Command received from the BLE console (phone) to read low_batt_threshold 
    BLE_CMD_RD_BRNO_THR     = 5320,     //Command received from the BLE console (phone) to read brownout_threshold 
};

enum ble_cmd_response {
	BLE_INIT_UNKNOWN  		= -1,
	BLE_INIT_COMPLETE 		= 0,
	BLE_INIT_ERROR    		= 1,
	BLE_STATUS_GOOD   		= 2,
	BLE_STATUS_FAILURE		= 3,
	BLE_RETURN_GOOD   		= 4,
	BLE_RETURN_FAILURE		= 5
};  

enum ble_ping_resp {
	BLE_PING_RECEIVED 		= 0,
	BLE_PING_ERROR    		= 1
};

typedef struct prepare_type_env
{
    uint8_t *prepare_buf;
    int prepare_len;
} prepare_type_env_t;

typedef enum BLEAdvId
{
	START_ADV   =   1U,
    STOP_ADV    =   2U,
   
}__attribute__((__packed__))
BLEAdvId_e;

typedef int esp_err_t;

typedef enum BLECharcId
{
    RESULTS_HANDLE = 42U,
    RESULTS_NOTIFY_HANDLE = 44U,
    SSID_HANDLE = 46U,
    PSWD_HANDLE = 49U,
    SCALE_WEIGHT_HANDLE = 52U,
    FULL_LEVEL_HANDLE = 55U, //58U,
    CALIB_HANDLE = 58U, //61U,
    WIFI_NOTIFY_HANDLE = 63U, //66U,
	WIFI_CONNECT_HANDLE = 62U,
    LOG_HANDLE = 65U, //68U,
    LOG_NOTIFY_HANDLE = 66U, //69U,
    CON_HANDLE = 68U, //71U,
    ALIVE_CONNECT_HANDLE = 71U, //74U,   
    //BLE_READ_INDEX_HANDLE = 76U,   
    BLE_READ_INDEX_HANDLE = 73U,   
} __attribute__((__packed__))
BLECharcId_e;

/// BLE Characteristics identifier enumeration
enum
{
    IDX_SVC,
    IDX_CHAR_RESULTS,
    IDX_CHAR_VAL_RESULTS,
    IDX_CHAR_CFG_DES_RESULTS,
    IDX_CHAR_CFG_RESULTS,

    IDX_CHAR_SSID,
    IDX_CHAR_VAL_SSID,
    IDX_CHAR_VAL_SSID_DES,

    IDX_CHAR_PSWD,
    IDX_CHAR_VAL_PSWD,
    IDX_CHAR_VAL_PSWD_DES,

    IDX_CHAR_SCALE_WEGH,
    IDX_CHAR_VAL_SCALE_WEGH,
    IDX_CHAR_VAL_SCALE_WEGH_DES,

    IDX_CHAR_ZERO_LEV,
    IDX_CHAR_VAL_ZERO_LEV,
    IDX_CHAR_VAL_ZERO_LEV_DES,

    IDX_CHAR_FULL_LEV,
    IDX_CHAR_VAL_FULL_LEV,
    IDX_CHAR_VAL_FULL_LEV_DES,

    IDX_CHAR_CALIB,
    IDX_CHAR_VAL_CALIB,
    IDX_CHAR_VAL_CALIB_DES,
    IDX_CHAR_CFG_CALIB,

    IDX_CHAR_WIFI,
    IDX_CHAR_VAL_WIFI,
    IDX_CHAR_VAL_WIFI_DES,

    IDX_CHAR_LOG,
    IDX_CHAR_VAL_LOG,
    IDX_CHAR_VAL_LOG_DES,
    IDX_CHAR_CFG_LOG,

    IDX_CHAR_CON,
    IDX_CHAR_VAL_CON,
    IDX_CHAR_VAL_CON_DES,

    IDX_CHAR_ALIVE,
    IDX_CHAR_VAL_ALIVE,
    //IDX_CHAR_VAL_ALIVE_DES,

    IDX_CHAR_INDEX,
    IDX_CHAR_VAL_INDEX,
    IDX_CHAR_VAL_INDEX_DES,

    LOX_IDX_NB,
};

enum ble_console_commnds {
	BLE_CON_VER_CMD         = 0,
	BLE_CON_MEAS_CMD        = 1,
};

/// BLE Characteristics structure
typedef struct ble_charc_uuid
{             
    uint8_t uuid[ESP_UUID_LEN_128] ;  
    uint8_t descriptor[GATTS_CHAR_VAL_LEN_MAX];      /*!< results charc data */
} __attribute__((__packed__))  ble_charc_uuid_t;


///	BLE Characteristics structure
typedef struct ble_charc_data
{             
    uint8_t  char_value[GATTS_CHAR_VAL_LEN_MAX] ;	/*!< results charc data */
    uint8_t  ssid[GATTS_CHAR_SSID_LEN_MAX];         /*!< ssid charc data */
    uint8_t  password[GATTS_CHAR_PWD_LEN_MAX];      /*!< pswd charc data */
    uint8_t  calibration[GATTS_CHAR_VAL_LEN_MAX];   /*!< calib charc data */
    uint8_t  scale_weight[GATTS_CHAR_VAL_LEN_MAX];  /*!< scale weight charc data */
    uint32_t zero_level_val;                        /*!< zero level charc data */
    uint32_t full_level_val;                        /*!< full level charc data */
    uint8_t  console_value[GATTS_CON_VAL_LEN_MAX] ; /*!< results console data */
    uint8_t  index[GATTS_INDEX_VAL_LEN_MAX] ;       /*!< lox index value to select security key */

} __attribute__((__packed__))  ble_charc_data_t;

/// BLE connection status Structure
typedef struct ble_conn
{             
    bool is_ble_connected;  
} __attribute__((__packed__)) ble_conn_t ;

typedef struct ble_status
{
  uint8_t is_initialized            : 1;
  uint8_t is_connected              : 1;
  uint8_t is_scan_done              : 1;
  uint8_t enable_data_ntf           : 1;
  uint8_t is_scale_notify_enabled   : 1;
  uint8_t is_log_notify_enabled     : 1;
  uint8_t is_wifi_notify_enabled    : 1;

  uint8_t send_scale_data           : 1;    //This is set if a request to send scale data has been received
  uint8_t scale_weight_received     : 1;    //Set when the requested scale data has been received
  uint8_t battery_volts_received    : 1;    //Set when the requested battery volts full has been received
  uint8_t alarm_status_received     : 1;    //Set when the requested alarm status has been received

  uint8_t unused_2                  : 5;
  uint16_t unused_3                 : 16;
} __attribute__((__packed__))  ble_status_t;


typedef struct gattc_profile_inst {
    esp_gattc_cb_t gattc_cb;
    uint16_t gattc_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_start_handle;
    uint16_t service_end_handle;
    uint16_t char_handle;
    esp_bd_addr_t remote_bda;
} __attribute__((__packed__)) gattc_profile_inst_t;


/// Results Characteristics Notification Structure
typedef struct results_notify_data
{             
    //uint32_t weight;  
    float weight;  
    float battery_volts; 
    uint16_t alarms;      
} __attribute__((__packed__)) results_notify_data_t;

/// Calib Characteristics Notification Structure
typedef struct calib_notify_data
{             
    uint32_t tare_weight;  
    uint32_t zero_level;  
    uint32_t full_level;   
} __attribute__((__packed__)) calib_notify_data_t;

/// BLE lat/long Structure
typedef struct ble_lat_long_send 
{
    uint8_t lat_value[10];
    uint8_t long_value[10];
} __attribute__((__packed__)) ble_lat_long_send_t;

#define ESP_MAX_NOTIFY_SIZE       128 //  30 //20
#define ESP_MAX_SEND_SIZE         130 //100 

//---------------------------------------------------------------------------------
//Wifi Parameters
//---------------------------------------------------------------------------------

#define WIFI_GET_TIME_MIN   1675268525000   //Wed 1 Feb 2023 11:22:05

#define WIFI_SCAN_LIST_SIZE 10
//#define STRING_MAX_TOPIC_CHARACTERS 100
//#define DEFAULT_TOPIC "topic/lox/testData"

#define WIFI_LOX_ID_MAX_CHAR  70

//#define WIFI_MIN_HEAP_SIZE 40000
#define WIFI_MIN_HEAP_SIZE 40000

#define BASE_URL_MAX_LEN 50
#define FULL_URL_MAX_LEN 150
//#define OTA_URL_MAX_LEN 1000
#define POST_DATA_MAX_LEN 150
#define MED_REC_MAX_LEN 10
#define LOX_ID_MAX_LEN 50
#define DEVICE_TOKEN_MAX_LEN 1000

#define OTA_URL_SIZE 256
#define OTA_ENABLE_TEST_STRING_MAX_LEN 10

//#define CONFIG_FIRMWARE_UPGRADE_URL "https://dev-portal.caireinc.com/assets/simple_ota-pico_v0.1.bin"
#define CONFIG_FIRMWARE_UPGRADE_URL "https://storage.googleapis.com/caire-dev-reports/lox-gateway_v0.1.37.bin"

#define CONFIG_EXAMPLE_CONNECT_WIFI 1
#define CONFIG_EXAMPLE_WIFI_CONN_MAX_RETRY 6
#define CONFIG_EXAMPLE_WIFI_SCAN_RSSI_THRESHOLD -127

#define OTA_SIG_START   0xAA551234
#define OTA_SIG_FAIL    0x55AA4321
#define OTA_SIG_PASS    0xA5A51342
#define OTA_SIG_DONE    0x5A5A3124

#define OTA_CONFIG_GOOD 0x1342A5A5

#define OTA_COUNT_GOOD  0xABBA1221

#define MAX_OTA_ATTEMPTS 3

#define DEVICE_ACCESS_TOKEN_MAX_CHARACTERS   1000   //600
#define RECEIVED_TOKEN_MAX_CHARACTERS   1600 //1400

extern RTC_NOINIT_ATTR uint32_t otaSignature;

extern RTC_NOINIT_ATTR char wifiSwUrl[SOFTWARE_URL_MAX_CHARACTERS];
extern RTC_NOINIT_ATTR uint8_t otaSsid[GATTS_CHAR_SSID_LEN_MAX];
extern RTC_NOINIT_ATTR uint8_t otaPassword[GATTS_CHAR_PWD_LEN_MAX];
extern RTC_NOINIT_ATTR uint8_t otaCount;
extern RTC_NOINIT_ATTR uint32_t otaCountSig;
extern RTC_NOINIT_ATTR char otaLastVersion[SOFTWARE_VERSION_MAX_CHARACTERS];


//static char wifiSwUrl[SOFTWARE_URL_MAX_CHARACTERS] = {0};

/// Device Data structure used when creating a lox device in MyCaire
typedef struct device_data
{   
    uint16_t    deviceDataStructId;  //To identify this structure. Use "DEVICE_DATA_STRUCT_ID" from the enum structure_ids.
    uint8_t     providerId;
    uint8_t     locationId;
    char        deviceToken[DEVICE_ACCESS_TOKEN_MAX_CHARACTERS];
    char        loxId[LOX_ID_MAX_LEN];
    char        url[BASE_URL_MAX_LEN];
    uint32_t    sig;       
} __attribute__((__packed__))  device_data_t;


/// scale config Data structure used when moving config data to or from storage
typedef struct scale_config_data
{   
    uint16_t    scaleConfigDataStructId;  //To identify this structure. Use "SCALE_CONFIG_STRUCT_ID" from the enum structure_ids.
    uint8_t     locationId;
    uint8_t     providerId;
    uint8_t     wifi_retries;
    uint8_t     tank_retries;
    uint16_t    sleep_duration;
    uint16_t    upload_duration;
    uint16_t    low_batt_threshold;
    uint16_t    brownout_threshold;
    uint32_t    tank_threshold;
    uint32_t    tank_timer;
    float       ver;
    char        url[BASE_URL_MAX_LEN];
} __attribute__((__packed__))  scale_config_data_t;



//#define WIFI_MIN_HEAP_SIZE 40000
#define WIFI_MIN_HEAP_SIZE 40000

#define BASE_URL_MAX_LEN 50
#define FULL_URL_MAX_LEN 150
#define POST_DATA_MAX_LEN 150
#define MED_REC_MAX_LEN 10
#define LOX_ID_MAX_LEN 50
#define DEVICE_TOKEN_MAX_LEN 1000

#define OTA_URL_SIZE 256

//#define CONFIG_FIRMWARE_UPGRADE_URL "https://dev-portal.caireinc.com/assets/simple_ota-pico_v0.1.bin"
#define CONFIG_FIRMWARE_UPGRADE_URL "https://storage.googleapis.com/caire-dev-reports/lox-gateway_v0.1.37.bin"

#define CONFIG_EXAMPLE_CONNECT_WIFI 1
#define CONFIG_EXAMPLE_WIFI_CONN_MAX_RETRY 6
#define CONFIG_EXAMPLE_WIFI_SCAN_RSSI_THRESHOLD -127

#define OTA_SIG_START   0xAA551234
#define OTA_SIG_FAIL    0x55AA4321
#define OTA_SIG_PASS    0xA5A51342
#define OTA_SIG_DONE    0x5A5A3124

#define OTA_CONFIG_GOOD 0x1342A5A5

#define CORRUPT_DATA_ERROR 400

//extern RTC_NOINIT_ATTR uint32_t otaSignature;

///	WIFI Characteristics structure
typedef struct wifi_charc_data
{             
    uint8_t  ssid[GATTS_CHAR_SSID_LEN_MAX];         
    uint8_t  password[GATTS_CHAR_PWD_LEN_MAX];      
    uint8_t  baseUrl[BASE_URL_MAX_LEN];
    uint8_t  serialNum[MAX_ADV_SERIAL_NUMBER_LEN];
    uint8_t  deviceToken[DEVICE_ACCESS_TOKEN_MAX_CHARACTERS];
    uint8_t  loxId[LOX_ID_MAX_LEN];
    uint8_t providerId;
    uint8_t locationId;
} __attribute__((__packed__))  wifi_charc_data_t;

typedef enum
{
  DEVICE_ACCESS_TOKEN = 0,
  DEVICE_TOKEN = 1,
  LOX_ID = 2,
  CONFIGURATION = 3,
  TEST = 4,
}
sys_wifi_token_type_t;

typedef enum
{
  WIFI_CONNECT = 0,
  WIFI_DISCONNECT = 1,
  WIFI_SCAN = 2,
  WIFI_API_AVAILABLE = 3,
  WIFI_API_NOT_AVAILABLE = 4
}
sys_wifi_connect_mode_t;

typedef struct wifi_status
{
  uint8_t is_initialized    : 1;
  uint8_t is_connected      : 1;
  uint8_t is_scan_done      : 1;
  uint8_t is_api_accessable : 1;
  uint8_t unused_1          : 4;
  uint16_t unused_2         : 16;
} wifi_status_t;

//Wifi		    -	6000  to 6999
enum wifi_cmd_type {
    WIFI_CMD_UNKNOWN         	= -1,
    WIFI_CMD_INIT            	= 6001,    	//intialize task
    WIFI_CMD_PING            	= 6002,    	//echo message received back to sender
    WIFI_CMD_STATUS          	= 6003,    	//Send back status to sender

    WIFI_CMD_RD_SSID            = 6010,     //Get SSID stored
    WIFI_CMD_RD_PWD             = 6011,     //Get Password stored
    WIFI_CMD_RD_SSID_PWD        = 6012,     //Get Password and SSID stored
    WIFI_CMD_RD_BASE_URL        = 6013,     //Get BASE URL stored, Should be "lox", "dev", or "stage"
    WIFI_CMD_RD_SN              = 6014,     //Get Serial Number stored
    WIFI_CMD_READ_DEVICE_DATA   = 6015,     //Get Device Data stored
    WIFI_CMD_PRINT_DEVICE_DATA  = 6016,     //Print out device data contained in the wifi module

    WIFI_CMD_STA_INIT		 	= 6030,	   	//Init wifi Station
    WIFI_CMD_STA_CONNECT    	= 6031,    	//Connect wifi Station
    WIFI_CMD_STA_DISCONNECT 	= 6032,    	//Disconnect wifi Station
    WIFI_CMD_STA_SSID_RECON 	= 6033,    	//Reconnect wifi Station with new SSID
    WIFI_CMD_STA_PWD_RECON 	    = 6034,    	//Reconnect wifi Station with new PWD
    WIFI_CMD_GET_TSF_TIME       = 6035,     //Get current TSF time from router
    WIFI_CMD_GET_DATE_TIME      = 6036,     //Get Date/Time from router
    WIFI_CMD_INIT_SNTP          = 6037,     //Initalize sntp
    WIFI_CMD_STA_SCAN           = 6038,     //Scan for access points (AP)
    WIFI_CMD_GET_RTC_TIME       = 6039,     //Get the time reported by the real time clock

    WIFI_CMD_AWS_INIT			= 6040,		//AWS Init
    WIFI_CMD_AWS_CONNECT        = 6041,     //AWS Connect

    WIFI_CMD_PROVISION			= 6060,		//Provision unit  

    WIFI_CMD_LOG1_TEST          = 6070,     //Send information log to mobile device via BLE
    WIFI_CMD_LOG2_TEST          = 6071,     //Send error log to mobile device via BLE         
    WIFI_CMD_LOG3_TEST          = 6072,     //Send forced info log to BLE log notification queue         
    WIFI_CMD_LOG4_TEST          = 6073,     //Send forced error log to BLE log notification queue 
    WIFI_CMD_WR_TOKEN_TEST      = 6074,     //Test to write token data to storage
    WIFI_CMD_WR_LOX_ID_TEST     = 6075,     //Test to write lox id data to storage

    WIFI_CMD_HTTPS_GET_APP_VER  = 6200,     //Get configuration from the cloud 
    WIFI_CMD_GET_DEV_ACC_TOKEN  = 6201,     //Get lox id from the cloud 
    WIFI_CREATE_LOX_DEVICE      = 6202,     //Create a lox device in the cloud 
    WIFI_GET_FREE_HEAP_SPACE    = 6203,     //Return the amount of free heap space
    //WIFI_GET_DEVICE_TOKEN       = 6204,     //Get device token from the cloud using the device serial number
    WIFI_POST_DEVICE_DATA       = 6205,     //Post device data to the cloud
    //WIFI_GET_LOX_ID             = 6206,     //Get lox id from the cloud using the device serial number
    WIFI_GET_CONFIG_DATA        = 6207,     //Get scale configuration data from the cloud.
    WIFI_POST_LOG_DATA          = 6208,     //Post log data to the cloud
    WIFI_CMD_OPEN_DATA_CONN     = 6209,     //Open https connection to post scale data
    WIFI_CMD_CLOSE_DATA_CONN    = 6210,     //Close https connection to post scale data
    WIFI_CMD_OPEN_LOG_CONN      = 6211,     //Open https connection to post log data
    WIFI_CMD_CLOSE_LOG_CONN     = 6212,     //Close https connection to post log data
    WIFI_CMD_GET_ID_TOKEN       = 6213,     //Get and store the lox id and device token

    WIFI_CMD_SET_CON_LOST_ALM   = 6300,     //Turn on connection lost alarm
    WIFI_CMD_CLR_CON_LOST_ALM   = 6301,     //Turn off connection lost alarm
    WIFI_CMD_SET_NET_ERR_ALM    = 6302,     //Turn on network error alarm
    WIFI_CMD_CLR_NET_ERR_ALM    = 6303,     //Turn off network error alarm

    WIFI_CMD_START_OTA          = 6350,     //Start the OTA process
    WIFI_GET_OTA_URL            = 6351,     //Get OTA URL from the cloud.
    WIFI_RD_OTA_URL             = 6352,     //Read the latest OTA url
    WIFI_RD_OTA_SIG             = 6353,     //Read the latest OTA signature
};

enum wifi_init_status {
    WIFI_INIT_UNKNOWN  		= -1,
    WIFI_INIT_COMPLETE 		= 0,
    WIFI_INIT_ERROR    		= 1,
    WIFI_STATUS_GOOD   		= 2,
    WIFI_STATUS_FAILURE		= 3,
    WIFI_RETURN_GOOD        = 4,
    WIFI_RETURN_FAILURE     = 5
};  

enum wifi_ping_resp {
    WIFI_PING_RECEIVED 		= 0,
    WIFI_PING_ERROR    		= 1
};

enum wifi_connect_resp {
    WIFI_CONNECTING 		    = 0,
    WIFI_CONNECTION_FAILED      = 1,
    WIFI_CONNECT_API_FAILED     = 3,
    WIFI_CONNECTION_PASSED      = 5,
    ENTER_TANK_CALIBRATION_MODE = 6,
    TANK_WEIGHT_STABILIZING     = 7,
    TANK_WEIGHT_READY           = 8,
    TANK_WEIGHT_ERROR           = 9,
    TANK_WEIGHT_REMOVED         = 10,
    EXIT_TANK_CALIBRATION_MODE  = 11,
    SCALE_GOING_TO_SLEEP        = 12,
};

typedef struct enqueueData
{
    unsigned    long    weight;
    unsigned    long    temp;
    unsigned    long    battery;
    unsigned    long    humidity;
    unsigned    long    volume;
    unsigned    long    alarm_code;
    int   timestamp;
} enqueueData_t;

//// For AWS
#define AWS_HOST_ADDRESS "aflobc52fa8dv-ats.iot.us-east-1.amazonaws.com"

#if (_CONFIG_ENVIRONMENT_DEV)
#define AWS_TEMPLATE_NAME "esp32-dev-provisioning-template"
#define AWS_WORKING_ENVIRONMENT "dev"
#endif

#if (_CONFIG_ENVIRONMENT_PRODUCTION)
#define AWS_TEMPLATE_NAME "esp32-prod-provisioning-template"
#define AWS_WORKING_ENVIRONMENT "production"
#endif

#define AWS_OFFICIAL_CERTIFICATE_PATH "/spiffs/certificate.pem.crt"
#define AWS_OFFICIAL_PRIVATE_KEY_PATH "/spiffs/private.pem.key"
#define AWS_PORT (8883)

#define FLAG_QRCODE_NOT_SET (0x00)
#define FLAG_QRCODE_SET (0x11)
#define FLAG_QRCODE_SET_SUCCESS (0x22)

/**
 * @brief AWS packet type enum
 */
typedef enum
{
    AWS_PKT_RESP = 0x00 // Response
    ,
    AWS_PKT_NOTI = 0x01 // Notification
    ,
    AWS_PKT_ACK = 0x02 // Acknowledgement

    ,
    AWS_PKT_UNKNOWN
} aws_packet_type_t;

typedef enum
{
    QUES_YES = 0,
    QUES_NO,
    QUES_SKIP,
    QUES_TIMEOUT
} sys_ques_status_t;

/**
 * @brief AWS result type
 */
typedef enum
{
    AWS_RES_OK,
    AWS_RES_BUSY,
    AWS_RES_INVALID_PARAM,
    AWS_RES_INVALID_OPERATION

    ,
    AWS_RES_UNKNOWN
} aws_res_type_t;

/**
 * @brief AWS request type
 */
typedef enum
{
    AWS_REQ_GET_DEVICE_INFO

    ,
    AWS_REQ_UNKNOWN
} aws_req_type_t;

/**
 * @brief AWS notification type
 */
typedef enum
{
    AWS_NOTI_ALARM,
    AWS_NOTI_DEVICE_DATA

    ,
    AWS_NOTI_UNKNOWN
} aws_noti_type_t;

/**
 * @brief AWS notification info
 */
typedef struct
{
    bool ack;
    char *const name;
} aws_noti_info_t;

/**
 * @brief AWS request info
 */
typedef struct
{
    char *const name;
} aws_req_info_t;

/**
 * @brief AWS result info
 */
typedef struct
{
    char *const name;
} aws_res_info_t;

/**
 * @brief AWS response param info
 */
typedef struct
{
    struct
    {
        char hw[10];
        char fw[10];
    } dev_info;
} aws_resp_param_info_t;

/**
 * @brief AWS notification device data struct
 */
/*
typedef struct
{
    char serial_number[50];
    float longitude;
    float lattitude;
    enqueueData_t lox_noti_data[MAX_LOX_DATA_COUNT];
}  __attribute__((__packed__))aws_noti_dev_data_t;
*/

/**
 * @brief AWS notification param info
 */
/*
typedef struct
{
    // uint64_t time;
    // uint32_t alarm_code;

    aws_noti_dev_data_t device_data;
} aws_noti_param_info_t;
*/

/**
 * @brief AWS response param
 */
typedef struct
{
    aws_req_type_t req_type;
    aws_res_type_t res_type;

    aws_resp_param_info_t info;
} aws_resp_param_t;


/**
 * @brief AWS notification param
 */
/*
typedef struct
{
    aws_noti_type_t noti_type;
    uint32_t noti_id;

    aws_noti_param_info_t info;
} aws_noti_param_t;
*/


/**
 * AWS publish topics
 */


/**
 * AWS subcribe topics
 */

/**
 * @brief AWS Provision Success Callback
 */
typedef void (*aws_provision_success_callback)(const char *thing_name);

/**
 * @brief AWS Provision Initialization Parameters
 */
typedef struct
{
    char aws_client_id[32];
    uint16_t aws_client_id_len;
    char qr_code[50];
    uint16_t qr_code_len;
    aws_provision_success_callback provision_success_cb;
} sys_aws_provision_init_params_t;


///// aws shadow
/**
 * @brief Shadow name enum
 */
typedef enum
{
    SYS_SHADOW_FIRMWARE_ID = 0,
    SYS_SHADOW_SCALE_TARE,
    SYS_SHADOW_MAX
} sys_aws_shadow_name_t;

/**
 * @brief AWS shadown command enum
 */
typedef enum
{
    SYS_AWS_SHADOW_CMD_GET = 0,
    SYS_AWS_SHADOW_CMD_SET
} sys_aws_shadow_cmd_t;


////// for aws logs
/* This is a stub replacement for the aws_iot_log.h header in the AWS IoT SDK,
   which redirects their logging framework into the esp-idf logging framework.

   The current (2.1.1) upstream AWS IoT SDK doesn't allow this as some of its
   headers include aws_iot_log.h, but our modified fork does.
*/

// redefine the AWS IoT log functions to call into the IDF log layer
#define IOT_DEBUG(format, ...) ESP_LOGD("aws_iot", format, ##__VA_ARGS__)
#define IOT_INFO(format, ...) ESP_LOGI("aws_iot", format, ##__VA_ARGS__)
#define IOT_WARN(format, ...) ESP_LOGW("aws_iot", format, ##__VA_ARGS__)
#define IOT_ERROR(format, ...) ESP_LOGE("aws_iot", format, ##__VA_ARGS__)

/* Function tracing macros used in AWS IoT SDK,
   mapped to "verbose" level output
*/
#define FUNC_ENTRY ESP_LOGV("aws_iot", "FUNC_ENTRY:   %s L#%d \n", __func__, __LINE__)
#define FUNC_EXIT_RC(x) \
    do {                                                                \
        ESP_LOGV("aws_iot", "FUNC_EXIT:   %s L#%d Return Code : %d \n", __func__, __LINE__, x); \
        return x; \
    } while(0)

////// aws - ota

#define OTA_STATE_NONE (0)
#define OTA_STATE_FAILED (1)
#define OTA_STATE_SUCCEEDED (2)

#define APP_STATE_NORMAL (0)
#define APP_STATE_OTA (1)



//---------------------------------------------------------------------------------
//Storage Parameters
//---------------------------------------------------------------------------------

#define STRING_MAX_LOG_ELEMENTS	20      //For logging
#define STRING_MAX_LOG_CHARACTERS 100

//For log function
#define SER_STRING_MAX_ARRAY_LOG_ELEMENTS   10
#define STRG_STRING_MAX_ARRAY_LOG_ELEMENTS  10
#define SYS_STRING_MAX_ARRAY_LOG_ELEMENTS   10

#define STRG_MESSAGE_MAX_ARRAY_ELEMENTS	    10
#define STRG_STRING_MAX_ARRAY_CHARACTERS 150

#define STRG_MAX_FILES_OPENED_AT_ONE_TIME   5 
#define DATA_READINGS_PER_DAY 24
#define MAX_DATA_RETENTION_DAYS 21
#define DATA_QUEUE_MAX_COUNT ((DATA_READINGS_PER_DAY * MAX_DATA_RETENTION_DAYS) + 2)    //2 extra spots

#define SPIFFS_DIR "/spiffs"

#define DATA_QUEUE_DIR "/spiffs/data"
#define NEW_DATA_QUEUE_DIR "/spiffs/new_data"
#define DATA_QUEUE_MAX_SIZE  (MAX_DATA_STRING_LENGTH * DATA_QUEUE_MAX_COUNT)     //2048
#define MIN_DATA_QUEUE_AVAILABLE (MAX_DATA_STRING_LENGTH * 2)

#define LOG_QUEUE_DIR "/spiffs/log"
#define MAX_LOG_STRING_LENGTH 50
#define LOG_QUEUE_MAX_COUNT 2000
#define LOG_QUEUE_MAX_SIZE (MAX_LOG_STRING_LENGTH * LOG_QUEUE_MAX_COUNT)
#define MIN_LOG_QUEUE_AVAILABLE 100

#define NVS_STORAGE_SPACENAME "Storage"

#define NVS_OTA_STORAGE_SPACENAME "OTA_Storage"
#define OTA_PARTITION_LABEL "ota_nvs"

#define DEFAULT_SLEEP_DURATION_MIN 60   //minutes
#define DEFAULT_SLEEP_DURATION_MIN_MAX 7200    //minutes = 5 Days
#define UPLOAD_DURATION 360 //minutes (6 hours)
#define LOW_BATTERY_THRESHOLD 489 //4.89 volts
#define BROWNOUT_THRESHOLD 425    //4.25 volts
#define WIFI_RETRIES 5
#define TANK_RETRIES 20 
#define TANK_THRESHOLD 50   //5.0 kg
#define TANK_TIMER 50   //msec

#define UPLOAD_TIME_SIG_SET     0xABCD1234  //Last upload time has been initialized

#define CONFIG_SIG_DEFAULT      0x12345678  //Config signature to verify configuration data is good
#define CONFIG_SIG_SET          0xABCDDCBA  //Configuration received from the cloud

#define SCALE_SIG_DEFAULT       0x12345678  //Scale signature to verify scale cal data is good
#define SCALE_SIG_INIT_SET      0x0000BEEF  //Siganture for scale initalized
#define SCALE_SIG_WGHT_SET      0x00AD0000  //Siganture for scale weight set (gain)
#define SCALE_SIG_TARE_SET      0xDE000000  //Siganture for scale tare set (offset)
#define SCALE_SIG_SET           0xDEADBEEF  //Siganture for all three set


#define WIFI_SIG_DEFAULT        0x12345678  //Wifi signature to verify wifi data is good
#define WIFI_SIG_SSID_SET       0x0000FAED  //Siganture for SSID set
#define WIFI_SIG_PWD_SET        0xDEAF0000  //Signature for pwd set
#define WIFI_SIG_SET            0xDEAFFAED  //Signature for both set

#define HTTPS_SIG_DEFAULT       0x12345678  //default https signature to verify https data is good
#define HTTPS_SIG_LI_SET        0x0000DEBA  //Signature for lox id set
#define HTTPS_SIG_DT_SET        0xABED0000  //signature for device token set
#define HTTPS_SIG_SET           0xABEDDEBA  //signature if both lox id and device token set

#define DEVICE_SIG_DEFAULT      0x12345678  //device signature to verify device data is good
#define DEVICE_SIG_SET          0xBEADDAEB

#define DEFAULT_SSID		"ssid1234"
#define DEFAULT_PWD			"pwd1234"

#define DEFAULT_SN			"CBLSA12345"
#define DEFAULT_HW_REV		"A"
#define BLE_DEFAULT_SERIAL_NUMBER "CBLSA123456789"
#define CONFIG_SN_SIG_DEFAULT   0x12345678  //config Serial Number signature to verify sn is good
#define CONFIG_SN_SIG_SET       0xAABBCCDD

#define DEFAULT_BASE_URL	"https://portal.caireinc.com"
#define DEFAULT_PROV_ID 10
#define DEFAULT_LOC_ID  9
//#define DEFAULT_PAT_ID  0
//#define DEFAULT_MED_REC_ID  "A0001"
#define DEFAULT_LOX_ID  "LOX1234"
#define DEFAULT_DEVICE_TOKEN  "DEVICE1234"

#define LOG_STRING_MAX_ARRAY_CHARACTERS   500
#define FILE_NAME_MAX_CHARACTERS   25
#define DIR_AND_FILE_NAME_MAX_CHARACTERS   50

#define NVS_VERSION_KEY_NAME "VERS"
#define NVS_OTA_VERSION_KEY_NAME "OTA_VERS"
#define NVS_OTA_CONFIG_KEY_NAME  "OTA_CONFIG"


#define NVS_DATA_PAIR(key_id, name)                       \
    {                                                     \
        .key = key_id,                                    \
        .offset = offsetof(struct nvs_data_struct, name), \
        .size = sizeof(g_nvs_setting_data.name)           \
    }


#define NVS_OTA_DATA_PAIR(key_id, name)                   \
    {                                                     \
        .key = key_id,                                    \
        .offset = offsetof(struct nvs_ota_data_struct, name), \
        .size = sizeof(g_nvs_ota_setting_data.name)           \
    }    

#define BSP_ERR_NVS_INIT 20400
#define BSP_ERR_NVS_COMMUNICATION 20401

//storage		- 	4000  to 4999
enum STRG_cmd_type {
	STRG_CMD_UNKNOWN            = -1,
	STRG_CMD_INIT               = 4001,	//intialize task
	STRG_CMD_PING               = 4002,	//echo message received back to sender
	STRG_CMD_STATUS			    = 4003,	//Send back status to sender
	STRG_CMD_RD_NVS_DATA        = 4010,	//Read all data stored in nvs_data


    STRG_CMD_RST_NVS            = 4020, //Reset NVS data
    STRG_CMD_DATA_QUEUE_INIT    = 4021, //Initalize data queue
    STRG_CMD_NVS_STORE_ALL      = 4022, //Store all data in RAM into NVS
    STRG_CMD_READ_DEVICE_DATA   = 4301, //Get all device related data from nvs_data, store it in a structure and send the struc to the calling task.


    //data queue
    STRG_CMD_DATA_QUEUE_SIZE    = 4024, //Get size of data queue
    STRG_CMD_DATA_QUEUE_SPACE   = 4025, //Get available space for data queue
    STRG_CMD_DATA_QUEUE_COUNT   = 4026, //Get count of elements in data queue
    STRG_CMD_DATA_QUEUE_FSIZE   = 4027, //Get file size of data queue
    STRG_CMD_DATA_QUEUE_INFO    = 4028, //Get all information on data queue
    STRG_CMD_DATA_QUEUE_PERCENT = 4036, //Percent of data available in data queue
    STRG_CMD_DQ_WRITE_TEST      = 4170, //Data Queue Write test
    STRG_CMD_DQ_READ_TEST       = 4171, //Data Queue Read test
    STRG_CMD_DQ_WRITE_DATA      = 4174, //Write data to data queue
    STRG_CMD_DQ_READ_DATA       = 4175, //Read data from data queue
    STRG_CMD_DQ_CLR_QUEUE       = 4178, //Erases all data in Data queue
    STRG_CMD_DQ_REWRITE_DATA    = 4181, //ReWrite scale data into queue that came from log queue
    STRG_CMD_DATA_DIR_LIST      = 4182, //List out files in the Data directory
    STRG_CMD2_DATA_QUEUE_INIT   = 4151, //Initalize data queue
    BLE_CMD_DQ_READ_DATA        = 4060, //Read next string out of the data queue and send it to the BLE console
    BLE_CMD_DQ_CLR_QUEUE        = 4062, //Clear Data queue from ble console


    //log queue
    STRG_CMD_LOG_QUEUE_INIT     = 4023, //Initalize log queue   
    STRG_CMD_LOG_QUEUE_SIZE     = 4030, //Get size of log queue
    STRG_CMD_LOG_QUEUE_SPACE    = 4031, //Get available space for log queue
    STRG_CMD_LOG_QUEUE_COUNT    = 4032, //Get count of elements in log queue
    STRG_CMD_LOG_QUEUE_FSIZE    = 4033, //Get file size of log queue
    STRG_CMD_LOG_QUEUE_INFO     = 4034, //Get all information on log queue
    STRG_CMD_LOG_QUEUE_PERCENT  = 4037, //Percent of data available in log queue
    STRG_CMD_LQ_WRITE_TEST      = 4172, //Log Queue Write test
    STRG_CMD_LQ_READ_TEST       = 4173, //Log Queue Read test
    STRG_CMD_LQ_WRITE_DATA      = 4176, //Write data to log queue
    STRG_CMD_LQ_READ_DATA       = 4177, //Read data from log queue
    STRG_CMD_LQ_CLR_QUEUE       = 4179, //Erases all data in Log queue
    STRG_CMD_LQ_REWRITE_DATA    = 4180, //ReWrite log data into queue that came from log queue
    STRG_CMD_LOG_DIR_LIST       = 4183, //List out files in the Log directory
    STRG_CMD2_LOG_QUEUE_INIT    = 4152, //Initalize log queue
    STRG_CMD_LOG1_TEST          = 4410, //Send information log to mobile device via BLE
    STRG_CMD_LOG2_TEST          = 4411, //Send error log to mobile device via BLE
    BLE_CMD_LQ_READ_DATA        = 4061, //Read next string out of the log queue and send it to the BLE console
    BLE_CMD_LQ_CLR_QUEUE        = 4063, //Clear Log queue from ble console

    //wifi
	STRG_CMD_RD_SSID		    = 4004,	//Get ssid from storage
	STRG_CMD_WR_SSID		    = 4005,	//Store ssid to storage
	STRG_CMD_RD_PWD			    = 4006,	//Get password from storage
	STRG_CMD_WR_PWD			    = 4007,	//Store password to storage
    STRG_CMD_RD_WIFI_SIG        = 4082, //Get Wifi signature to verify wifi data is good
    STRG_CMD_WR_WIFI_SIG        = 6083, //Set wifi signature to indicate that the wifi cal data is good
    STRG_CMD_WR_WIFI_SSID_SIG   = 4122, //Wifi signature - SSID set
    STRG_CMD_WR_WIFI_PWD_SIG    = 4123, //Wifi signature - Password set
    STRG_CMD_RD_BASE_URL        = 4084, //Get Base URL stored
    STRG_CMD_WR_BASE_URL        = 4085, //Set Base URL
    STRG_CMD_SET_WIFI_DEFAULTS  = 4298, //Set all wifi config values to their defaults

    //https
    STRG_CMD_RD_HTTPS_SIG       = 4086, //Get https signature to verify https data is good
    STRG_CMD_WR_HTTPS_SIG       = 4087, //Set https signature to indicate that the https cal data is good
    STRG_CMD_RD_SN_SIG          = 4088, //Get Serial Number signature
    STRG_CMD_WR_SN_SIG          = 4090, //Set Serial Number signature to indicate that the serail number is good
    STRG_CMD_WR_HTTPS_DEFAULT_SIG   = 4132, //Set https signature to default
    STRG_CMD_WR_HTTPS_SET_SIG       = 4133, //Set https signature to good
    STRG_CMD_SET_HTTPS_DEFAULTS = 4300, //Set all HTTPS value to their defaults

    //device data
    STRG_CMD_RD_DEVICE_SIG      = 4092, //Get device signature to verify device data is good
    STRG_CMD_WR_DEVICE_SIG      = 4093, //Set device signature to indicate that the device cal data is good
	STRG_CMD_RD_SN			    = 4008,	//Get serial number from storage
	STRG_CMD_WR_SN			    = 4009,	//Store serial number to storage
	STRG_CMD_RD_SN_INIT         = 4011,	//Get serial number from storage during itialization
	STRG_CMD_RD_HW_REV			= 4012,	//Get hardware rev from storage
	STRG_CMD_WR_HW_REV			= 4013,	//Store hardware rev to storage
    STRG_CMD_RD_LAST_UL_TIME    = 4128, //Read time last update to cloud occured
    STRG_CMD_WR_LAST_UL_TIME    = 4129, //Write time last update to cloud occured
    STRG_CMD_SET_DEVICE_DEFAULTS= 4299, //Set all device values to their defaults

    //device configs
    STRG_CMD_RD_CONFIG_SIG      = 4080, //Get Config signature to verify configuration data is good
    STRG_CMD_WR_CONFIG_SIG      = 4081, //Set Config signature to indicate that the configuration cal data is good
    STRG_CMD_WR_DEVICE_CONFIG   = 4103, //Set Device configuration from the data received in structure called SCALE_CONFIG_STRUCT_ID
    STRG_CMD_SET_CONFIG_SIG     = 4127, //Set configuration signature to it's default value
    STRG_CMD_RD_LOW_BATT_THRES  = 4290, //Read low battery threshold in 1/10 volts (16 bits)
    STRG_CMD_RD_BROWNOUT_THRES  = 4291, //Read brownout threshold in 1/10 volts (16 bits)
    STRG_CMD_SET_CFG_DEFAULTS   = 4295, //Set all system config values to their defaults
    STRG_CMD_RD_WIFI_RET        = 4104, //Get wifi_retries from storage
    STRG_CMD_WR_WIFI_RET        = 4105, //Set wifi_retries in storage
    STRG_CMD_RD_SLP_DUR         = 4108, //Get sleep_duration from storage
    STRG_CMD_WR_SLP_DUR         = 4109, //Set sleep_duration in storage
    STRG_CMD_RD_UPL_DUR         = 4110, //Get upload_duration from storage
    STRG_CMD_WR_UPL_DUR         = 4111, //Set upload_duration in storage
    STRG_CMD_RD_LBAT_THR        = 4112, //Get low_batt_threshold from storage
    STRG_CMD_WR_LBAT_THR        = 4113, //Set low_batt_threshold in storage
    //STRG_CMD_RD_BRNO_THR        = 4114, //Get brownout_threshold from storage
    STRG_CMD_WR_BRNO_THR        = 4115, //Set brownout_threshold in storage

    //OTA
    STRG_CMD_RST_OTA_NVS            = 4140, //Reset OTA config flash
    STRG_CMD_COPY_NVS_TO_OTA        = 4141, //Copy NVS config to OTA config in flash
    STRG_CMD_COPY_OTA_TO_NVS        = 4142, //Copy OTA config to NVS config in flash
    STRG_CMD_RD_OTA_DATA            = 4143,	//Read all data stored in nvs_ota_data
    STRG_CMD_CLR_OTA_DATA           = 4144, //Set all numbers to 0 and strings to null.

    //spiffs
    STRG_CMD_MOUNT_SPIFFS       = 4038, //Mount SPIFFS before intializing them
    STRG_CMD2_MOUNT_SPIFFS      = 4150, //Mount SPIFFS before intializing them
    STRG_CMD2_DISMOUNT_SPIFFS   = 4153, //Dismount SPIFFS 
    STRG_CMD_SPIFFS_USAGE       = 4184, //report SPIFFS Usage

    STRG_CMD2_WR_TEST           = 4190, //Write test    
    STRG_CMD2_RD_TEST           = 4191, //Read test
    STRG_CMD2_DIR_LIST          = 4192, //directory list test    
    STRG_CMD2_CREATE_DIR        = 4193, //Create the new data queue directory
    STRG_CMD2_DIR2_LIST         = 4194, //new directory list test
    STRG_CMD2_WR2_TEST          = 4195, //Write 2 test     
    STRG_CMD2_WR3_TEST          = 4196, //Write 3 test    
    STRG_CMD2_NEXT_FILE         = 4198, //Get next file name
    STRG_CMD2_FILE_COUNT        = 4199, //Get number of files in directory
    STRG_CMD2_DELETE_FILE       = 4200, //Delete file from directory
    STRG_CMD2_READ_FILE         = 4201, //Read test - scale data

};


enum STRG_status {
	STRG_INIT_UNKNOWN  		= -1,
	STRG_INIT_COMPLETE 		= 0,
	STRG_INIT_ERROR    		= 1,
	STRG_STATUS_GOOD   		= 2,
	STRG_STATUS_FAILURE		= 3,	
    STRG_RETURN_GOOD   		= 4,
	STRG_RETURN_FAILURE		= 5
};  

enum STRG_ping_resp {
	STRG_PING_RECEIVED 		= 0,
	STRG_PING_ERROR    		= 1
};

enum strg_queue_type {
	STRG_DATA_QUEUE 		= 0,
	STRG_LOG_QUEUE    		= 1,
};


enum STRG_key_names {
	STRG_KEY_DEV			= 0,
	STRG_KEY_THING_NAME		= 1,	//not used
	STRG_KEY_MAC_ADDR		= 2,	
	STRG_KEY_PROV_STAT		= 3,	//not used
	STRG_KEY_OTA			= 4,	//not used	
	STRG_KEY_WIFI			= 5,	
	STRG_KEY_SOFT_OP		= 6,	//not used
	STRG_KEY_MEASUREMENTS	= 7,	
	STRG_KEY_LOCATION		= 8
};

// IMPORTANT: The revision of nvs_data_t. Everytime nvs_data_t is changed,
// the NVS_DATA_VERSION value must be updated too.
#define NVS_DATA_VERSION (uint32_t)(0x00000015)

/// @brief NVS data structure definition
typedef struct nvs_data_struct
{
    uint32_t data_version; // Version of NVS data

    struct
    {
        char        ssid[GATTS_CHAR_SSID_LEN_MAX];
        char        pwd[GATTS_CHAR_PWD_LEN_MAX];
        char        url[BASE_URL_MAX_LEN];    //Base URL. Ex https://dev-portal.caireinc.com or https://lox-portal.caireinc.com 
        uint32_t    sig;
    } wifi;

    // struct
    // {
    //     uint32_t    gain;
    //     uint32_t    calOffset;  //The initial offset after calibration
    //     uint32_t    offset;     //The current offset after a scale tare
    //     uint32_t    sig;
    // } scale;

    struct
    {
        uint16_t    sleep_duration;
        uint16_t    upload_duration;
        uint32_t    duration_start_time;
        uint16_t    low_batt_threshold;
        uint16_t    brownout_threshold;
		uint8_t     wifi_retries;
		// uint8_t     tank_retries;
        // uint32_t    tank_threshold;
        // uint32_t    tank_timer;
        char        sn[MAX_ADV_SERIAL_NUMBER_LEN];
        uint32_t    sig;
        uint32_t    sn_sig;
        uint32_t    passkey;    //Used for BLE security
        char        hw_rev[MAX_HW_REV_LEN];
    } config;

    // struct
    // {
    //     char        loxId[LOX_ID_MAX_LEN];
    //     char        deviceToken[DEVICE_TOKEN_MAX_LEN];
    //     uint32_t    sig;    
    // } https;  

    // struct
    // {   
    //     uint8_t     providerId;
    //     uint8_t     locationId;
    //     uint32_t    sig;    
    // } device;  

} nvs_data_t;

/// @brief  Store one specific data belongs to nvs_data_struct into NVS storage
/// @param[in]  member  the name of data in the structure @ref nvs_data_struct
///                     Example: SYS_NVS_STORE(sample1);
///                          or SYS_NVS_STORE(sample2);
#define SYS_NVS_STORE(member)                             \
    do                                                    \
    {                                                     \
        sys_nvs_store(SYS_NVS_LOOKUP_KEY(member),         \
                      &g_nvs_setting_data.member,         \
                      sizeof(g_nvs_setting_data.member)); \
    } while (0)


/// @brief  load one specific data from NVS storage to nvs_data_struct.
/// @param[in]  member  the name of data in the structure @ref nvs_data_struct
///                    Example: SYS_NVS_LOAD(sample1);
///                          or SYS_NVS_LOAD(sample2);
#define SYS_NVS_LOAD(member)                             \
    do                                                   \
    {                                                    \
        sys_nvs_load(SYS_NVS_LOOKUP_KEY(member),         \
                     &g_nvs_setting_data.member,         \
                     sizeof(g_nvs_setting_data.member)); \
    } while (0)



// IMPORTANT: The revision of nvs_data_t. Everytime nvs_data_t is changed,
// the NVS_DATA_VERSION value must be updated too.
#define NVS_OTA_DATA_VERSION (uint32_t)(0x00000004)

/// @brief NVS OTA data structure definition
typedef struct nvs_ota_data_struct
{
    uint32_t ota_data_version; // Version of NVS OTA data
    uint32_t ota_data_sig;      //set if the ota config has been set.

    struct
    {
        char        ssid[GATTS_CHAR_SSID_LEN_MAX];
        char        pwd[GATTS_CHAR_PWD_LEN_MAX];
        char        url[BASE_URL_MAX_LEN];    //Base URL. Ex https://dev-portal.caireinc.com or https://lox-portal.caireinc.com 
        uint32_t    sig;
    } wifi_ota;

    // struct
    // {
    //     uint32_t    gain;
    //     uint32_t    calOffset;  //The initial offset after calibration
    //     uint32_t    offset;     //The current offset after a scale tare
    //     uint32_t    sig;
    // } scale_ota;

    struct
    {
        uint16_t    sleep_duration;
        uint16_t    upload_duration;
        uint32_t    duration_start_time;
        uint16_t    low_batt_threshold;
        uint16_t    brownout_threshold;
		uint8_t     wifi_retries;
		// uint8_t     tank_retries;
        // uint32_t    tank_threshold;
        // uint32_t    tank_timer;
        char        sn[MAX_ADV_SERIAL_NUMBER_LEN];
        uint32_t    sig;
        uint32_t    sn_sig;
    } config_ota;

    // struct
    // {
    //     char        loxId[LOX_ID_MAX_LEN];
    //     char        deviceToken[DEVICE_TOKEN_MAX_LEN];
    //     uint32_t    sig;    
    // } https_ota;   

} nvs_ota_data_t;


typedef struct
{
    char key[4];     // This is the key-pair of data stored in NVS, we limit it in 4 ASCII number, start from "0000" -> "9999"
    uint32_t offset; // The offset of variable in @ref nvs_data_struct
    uint32_t size;   // The size of variable in bytes
} nvs_key_data_t;

typedef struct
{
    char key[4];     // This is the key-pair of data stored in NVS, we limit it in 4 ASCII number, start from "0000" -> "9999"
    uint32_t offset; // The offset of variable in @ref nvs_data_struct
    uint32_t size;   // The size of variable in bytes
} nvs_ota_key_data_t;

typedef struct lox_queue_data
{
    uint16_t no_of_msg;
    uint16_t active_index;
    enqueueData_t lox_data[MAX_QUEUE_DATA_COUNT];
} __attribute__((__packed__)) lox_queue_data_t;



#endif /* SHARED_FILES_HEADER_FILES_COMMON_H_ */