#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_sleep.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

#include "Shared/common.h"
#include "Shared/messages.h"

//Task Includes:
#include "Task/sysControlTask.h"
#include "Task/stateMachines.h"
#include "Task/strgTask.h"
#include "Task/bleTask.h"
#include "Task/sensorsTask.h"


//Local Functions:
void sctl_processSctlResponseMsg(void);
void sctl_processSystemControlMsg(void); 
void sctl_processEventMsg(uint16_t msgCmd, uint32_t msgData, uint8_t * msgDataPtr, uint32_t msgDataLen);          
void sctl_Initialize_system(void);
void sysResetSequenceMachine(void);
void sys_ProcessStateSequences(void);
void sys_ProcessEvent(void);
void sysSetSeqSystemIdle(void);
void sctl_processEventMsgFromSys(EVENT_message_t * evtMsg);
void sysProcessTestStateMachineStart(uint16_t stateMachineId);
bool sysProcessTestStateMachine(uint16_t stateMachineId);
void sysTestStateMachine(uint16_t stateMachineId);

bool bleProcessResult(char * msgString);
bool bleProcessSnResult(char * msgString);
bool bleStartProcessResult(char * msgString);

void sysLog(char * strPtr, bool forced, bool printTag);
void sysLogIF(char * strPtr);
void sysLogEF(char * strPtr);
void sysLogE(char * strPtr);
void sysLogR(char * strPtr);
void sysLogI(char * strPtr);

//Local Parameters
#define SYS_TAG "SYS"

static bool systemReadyToStart = false;

RTOS_message_t sctl_Pm_sysCtrlRxMsg;
RTOS_message_t sctl_Pm_sysCtrlTxMsg;
RTOS_message_t sysControlRxMessage;
RTOS_message_t sysControlTxMessage;

EVENT_message_t sysControlEvtTxMessage;
EVENT_message_t sysControlEvtRxMessage;

static uint16_t sysCurrentEvt = EVT_SYS_IDLE;
static uint32_t sysCurrentEvtData = 0;
static uint16_t sysCurrentEvtSeq = SCTL_EVT_STATE_IDLE;
static uint16_t sysCurrentSmId = 0;     //Current ID of state machine being tested 

static uint16_t sysSeqState = SCTL_SEQ_STATE_IDLE;
static bool sysCurrentSmComplete = true;
static bool sysCurrentEvtComplete = true;

uint16_t sysTmp16 = 0;
bool sysResultBit = false;
static sys_status_t sysStatus;

char sysHwRev[MAX_HW_REV_LEN] = {0};
static char advSerialNumber[MAX_ADV_SERIAL_NUMBER_LEN] = {0};

static sys_seq_t sysSeq;

char sctl_TmpStr[SYS_STRING_MAX_ARRAY_CHARACTERS];
char sys_TmpStr[SYS_STRING_MAX_ARRAY_CHARACTERS];

//Command response flags
static sys_init_complete_t      sysInitComplete;
static sys_init_result_t        sysInitResult;

static sys_begin_complete_t     sysBeginComplete;
static sys_begin_result_t       sysBeginResult;

static sys_ble_complete_t       sysBleComplete;
static sys_ble_result_t         sysBleResult;

static sys_snr_complete_t       sysSnrComplete;
static sys_snr_result_t         sysSnrResult;

//External Functions:

void sysStartSystem(void)
{
    systemReadyToStart = true;
    ESP_LOGI(SYS_TAG, "From main - System is ready to start");
}


/// @brief system controller task
/// @param  None
void sysControlTaskApp(void)
{
    ESP_LOGI(SYS_TAG, "Serial Task running");
    //gpio_config_t io_conf;
    while(!systemReadyToStart)
    {
        vTaskDelay(10);
    }
    ESP_LOGI(SYS_TAG, "System is ready to start");

    sctl_Initialize_system();

    while(1)
    {
        // wait for control message in queue
        if (xQueueReceive(sysControlQueueHandle,&sctl_Pm_sysCtrlRxMsg,10))
        {
            ESP_LOGI(SYS_TAG, "System control message %d received from: %d, ID: %lu, TS: %lu\r\n", 
                        sctl_Pm_sysCtrlRxMsg.msgCmd, sctl_Pm_sysCtrlRxMsg.srcAddr, sctl_Pm_sysCtrlRxMsg.msgRef, sctl_Pm_sysCtrlRxMsg.msgTimeStamp);
 
            if ((sctl_Pm_sysCtrlRxMsg.msgCmd >= EVT_MIN) && (sctl_Pm_sysCtrlRxMsg.msgCmd <= EVT_MAX))
            {
                ESP_LOGI(SYS_TAG,"Processing event command message %d from %d", sctl_Pm_sysCtrlRxMsg.msgCmd, sctl_Pm_sysCtrlRxMsg.srcAddr);
                //sctl_processEventMsg(sctl_Pm_sysCtrlRxMsg.msgCmd, sctl_Pm_sysCtrlRxMsg.msgData, sctl_Pm_sysCtrlRxMsg.msgDataPtr, sctl_Pm_sysCtrlRxMsg.msgDataLen);        
            }
            else if (sctl_Pm_sysCtrlRxMsg.msgCmd <= SCTL_CMD_MAX_VALUE)
            {
                ESP_LOGI(SYS_TAG,"Processing system control message %d", sctl_Pm_sysCtrlRxMsg.msgCmd);
                sctl_processSystemControlMsg();        
            }
            else if (sctl_Pm_sysCtrlRxMsg.dstAddr == MSG_ADDR_SCTL)
            {
                ESP_LOGI(SYS_TAG, "Processing system control response message %d from: %d",sctl_Pm_sysCtrlRxMsg.msgCmd, sctl_Pm_sysCtrlRxMsg.srcAddr);
                sctl_processSctlResponseMsg();
            }
            else
            {
                ESP_LOGE(SYS_TAG, "System control task was not able to process received message %d from %d", sctl_Pm_sysCtrlRxMsg.msgCmd, sctl_Pm_sysCtrlRxMsg.srcAddr);
            }
        }
        sys_ProcessEvent();
        sys_ProcessStateSequences();               
    }
}

//Local Functions:



/// @brief Function to initalize system
/// @param  None
void sctl_Initialize_system(void)
{
    EVENT_message_t initEvtMsg;

    //Display firmware version
    //sysPrint("\r\n----------------------------------------------------\r\n");
    ESP_LOGI(SYS_TAG,"\r\n----------------------------------------------------\r\n");
    sprintf(sctl_TmpStr, "Firmware version: %d.%d.%d\r\n", FIRMWARE_VERSION_RELEASE, FIRMWARE_VERSION_MAJOR, FIRMWARE_VERSION_MINOR);
    //sysPrint(sctl_TmpStr);
    ESP_LOGI(SYS_TAG, "%s", sctl_TmpStr);

    //Initialize tasks
    memset(&sysInitComplete,0,sizeof(sysInitComplete));
    memset(&sysInitResult,0,sizeof(sysInitResult));
  
    sysSetSeqSystemIdle(); 

    initEvtMsg.evtCmd = EVT_SYS_INIT;
    initEvtMsg.evtData = 0;
    initEvtMsg.evtDataPtr = NULL;
    initEvtMsg.evtDataLen = 0;

    sctl_processEventMsgFromSys(&initEvtMsg);
}

/// @brief Function to create message and sent it to the designated destination.
/// @param dstAddr - Task to send message to. (see message address enum)
/// @param msgType - Type of message. (see message type enum)
/// @param msgCmd  - Message associated with the message. (see specific task for command enum)
/// @param msgDataPtr - A pointer to data greater than 32 bits if used. Null otherwise.
/// @param msgData    - Data that is 32 bits or less. Set to 0 if not used.
/// @param msgDataLen - Data length of either msgData or data pointed to msgDataPtr (in bytes).
void sysSendMessage(uint8_t dstAddr, uint8_t msgType, uint16_t msgCmd, uint8_t * msgDataPtr, uint32_t msgData, uint32_t msgDataLen)
{  
    static uint8_t msgIndex = 0;
    RTOS_message_t sysSendMsg;
    static char sctl_Msg[SYS_MESSAGE_MAX_ARRAY_ELEMENTS][SYS_STRING_MAX_ARRAY_CHARACTERS];

    if ((msgType == MSG_DATA_0) || (msgType == MSG_DATA_8) || (msgType == MSG_DATA_16) || (msgType == MSG_DATA_32))
    {
       sysSendMsg.msgDataPtr = NULL;
       switch(msgType)
       {
            case MSG_DATA_0:    sysSendMsg.msgDataLen =  MSG_DATA_0_LEN;    break;
            case MSG_DATA_8:    sysSendMsg.msgDataLen =  MSG_DATA_8_LEN;    break;
            case MSG_DATA_16:   sysSendMsg.msgDataLen =  MSG_DATA_16_LEN;   break;
            case MSG_DATA_32:   sysSendMsg.msgDataLen =  MSG_DATA_32_LEN;   break;
            default:            sysSendMsg.msgDataLen =  MSG_DATA_0_LEN;    break;
       }
    } 
    else
    {
        memcpy(sctl_Msg[msgIndex], msgDataPtr, msgDataLen);
        sysSendMsg.msgDataPtr = (uint8_t *)sctl_Msg[msgIndex];
        sysSendMsg.msgDataLen = msgDataLen;
    }
    sysSendMsg.srcAddr = MSG_ADDR_SCTL;
    sysSendMsg.dstAddr = dstAddr;
    sysSendMsg.msgRef = msg_getMsgRef();
    sysSendMsg.msgTimeStamp = sys_getMsgTimeStamp();
    sysSendMsg.msgType = msgType;
    sysSendMsg.msgCmd = msgCmd;
    sysSendMsg.msgData = msgData;
 
    //send message
    xQueueSend(dispatcherQueueHandle,&sysSendMsg,0);
    ESP_LOGI(SYS_TAG, "Message %d sent from syscontroller to %d", msgCmd, dstAddr);

    //ESP_LOGW(SYS_TAG, "msgCmd %d dstAddr %d", sysSendMsg.msgCmd, sysSendMsg.dstAddr);
    //ESP_LOGW(SYS_TAG, "srcAddr %d msgRef %ld", sysSendMsg.srcAddr, sysSendMsg.msgRef);
    //ESP_LOGW(SYS_TAG, "msgType %d msgData %ld", sysSendMsg.msgType, sysSendMsg.msgData);
    //ESP_LOGW(SYS_TAG, "msgTimeStamp %ld", sysSendMsg.msgTimeStamp);

    //Set next array index
    //ESP_LOGW(SYS_TAG, "before msgIndex %d", msgIndex);
    msgIndex++;
    if (msgIndex >= SYS_MESSAGE_MAX_ARRAY_ELEMENTS)
    {
        msgIndex = 0;
    }
    //ESP_LOGW(SYS_TAG, "after msgIndex %d", msgIndex);

}

void sysSendDirMessage(uint8_t dstAddr, uint8_t msgType, uint16_t msgCmd, uint8_t * msgDataPtr, uint32_t msgData, uint32_t msgDataLen)
{  
    RTOS_message_t sysSendMsg;

    sysSendMsg.srcAddr = MSG_ADDR_SCTL;
    sysSendMsg.dstAddr = dstAddr;
    sysSendMsg.msgRef = msg_getMsgRef();
    sysSendMsg.msgTimeStamp = sys_getMsgTimeStamp();
    sysSendMsg.msgType = msgType;
    sysSendMsg.msgCmd = msgCmd;
    sysSendMsg.msgData = msgData;
    sysSendMsg.msgDataPtr = msgDataPtr;
    sysSendMsg.msgDataLen = msgDataLen;
 
    //send message
    xQueueSend(dispatcherQueueHandle,&sysSendMsg,0);
    ESP_LOGI(SYS_TAG, "Direct Message %d sent from syscontroller to %d", msgCmd, dstAddr);
}



/// @brief Function to process incoming messages
/// @param  None
void sctl_processSctlResponseMsg(void)
{
    EVENT_message_t procMsgEvtMsg;

    ESP_LOGI(SYS_TAG, "sctl_processSctlResponseMsg - msgCmd:%d", sctl_Pm_sysCtrlRxMsg.msgCmd);
    switch(sctl_Pm_sysCtrlRxMsg.msgCmd)
    {
        case DSPR_CMD_INIT:
            if (sctl_Pm_sysCtrlRxMsg.msgData == DSPR_INIT_COMPLETE){sysInitResult.dsprInit = true;} 
            sysInitComplete.is_DsprDone = true;
            break;

        case SER_CMD_INIT:
            if (sctl_Pm_sysCtrlRxMsg.msgData == SER_INIT_COMPLETE){sysInitResult.serInit = true;}
            sysInitComplete.is_SerDone = true;
            break;

        case STRG_CMD_INIT:
            if (sctl_Pm_sysCtrlRxMsg.msgData == STRG_INIT_COMPLETE){sysInitResult.strgInit = true;}
            sysInitComplete.is_StrgDone = true;
            break;	  

		case STRG_CMD_MOUNT_SPIFFS:
			if (sctl_Pm_sysCtrlRxMsg.msgData == STRG_RETURN_GOOD){sysInitResult.mountInit = true;}
            sysInitComplete.is_mountDone = true;
			break;		

		case STRG_CMD_DATA_QUEUE_INIT:
			if (sctl_Pm_sysCtrlRxMsg.msgData == STRG_RETURN_GOOD){sysInitResult.dataQInit = true;}
            sysInitComplete.is_dataQDone = true;
			break;		

		case STRG_CMD_LOG_QUEUE_INIT:
			if (sctl_Pm_sysCtrlRxMsg.msgData == STRG_RETURN_GOOD){sysInitResult.logQInit = true;}
            sysInitComplete.is_logQDone = true;
			break;		

        case STRG_CMD_RD_SN:
            memset(advSerialNumber, 0x00, sizeof(advSerialNumber));
            memcpy(&advSerialNumber, (char *)sctl_Pm_sysCtrlRxMsg.msgDataPtr, sctl_Pm_sysCtrlRxMsg.msgDataLen);

            sysBeginComplete.is_SnReadInNvs = true;
            sysBeginResult.snInNvs = true;

            ESP_LOGI(SYS_TAG, "Serial Number read by SYS:%s", advSerialNumber);
            break;

        case BLE_CMD_INIT:
            if (sctl_Pm_sysCtrlRxMsg.msgData == BLE_INIT_COMPLETE){sysInitResult.bleInit = true;}
            sysInitComplete.is_BleDone = true;
            break;	

        case BLE_CMD_INIT_ADV:
            sysBeginResult.bleStartDone  = bleStartProcessResult("BLE Module Started");
            sysBeginComplete.is_BleStarted = true;
            break;

		case BLE_CMD_ADV_START:
            sysBleResult.Adv_On         = bleProcessResult("Turn BLE Advertising On");
            sysBleComplete.is_Adv_On_Done   = true;
            break;

		case BLE_CMD_SET_SN:
            sysBleResult.SN_Loaded          = bleProcessSnResult("Load Serial Number");
            sysBleComplete.is_SN_Set            = true;

            sysBeginResult.snInBLE = sysBleResult.SN_Loaded;
            sysBeginComplete.is_SnReadInBLE = true;
            
            break;

        case WIFI_CMD_INIT:
            if (sctl_Pm_sysCtrlRxMsg.msgData == WIFI_INIT_COMPLETE){sysInitResult.wifiInit = true;}
            sysInitComplete.is_WifiDone = true;
            break;	

        case SNR_CMD_INIT:
            ESP_LOGI(SYS_TAG, "Process response msg - SNR_CMD_INIT");
            if (sctl_Pm_sysCtrlRxMsg.msgData == SNR_INIT_COMPLETE){sysInitResult.snrInit = true;}
            sysInitComplete.is_SnrDone = true;
            break;	

        default:;
    }
}

/// @brief Function to process messages from the supervisor port
/// @param  None
void sctl_processSystemControlMsg(void)        
{
    char tmpMsgStr[100] = {0};

    ESP_LOGI(SYS_TAG, "Process Supervisor message to sys control, msg ID:%lu", (ulong)sctl_Pm_sysCtrlRxMsg.msgRef);
 
    switch (sctl_Pm_sysCtrlRxMsg.msgCmd)
    {
        case SCTL_CMD_PING:
            ESP_LOGI(SYS_TAG, "System control - processing SCTL_CMD_PING");
            sysSendMessage(MSG_ADDR_SUPR,MSG_DATA_8,SCTL_CMD_PING,NULL,SCTL_PING_RECEIVED,MSG_DATA_8_LEN);  
            break;

        case SCTL_CMD_TEST_SM:
            ESP_LOGI(SYS_TAG, "Test state machine id:%d", (uint16_t)sctl_Pm_sysCtrlRxMsg.msgData);
            sysTestStateMachine((uint16_t)sctl_Pm_sysCtrlRxMsg.msgData);
            break;      
 
        case SCTL_CMD_TEST_EVT:
            EVENT_message_t evtTestMsg;

            evtTestMsg.evtCmd = (uint16_t)sctl_Pm_sysCtrlRxMsg.msgData;
            evtTestMsg.evtData = 0;
            evtTestMsg.evtDataPtr = NULL;
            evtTestMsg.evtDataLen = 0;

            ESP_LOGI(SYS_TAG, "Test event id:%d", evtTestMsg.evtCmd);
            sctl_processEventMsgFromSys(&evtTestMsg);
            break;      
 
        default:;
    }
}


/// @brief Used to test state machines
/// @param stateMachineId 
void sysProcessTestStateMachineStart(uint16_t stateMachineId)
{
    switch(stateMachineId)
    {
        case SM_TST_SYS_INIT:               sysProcessSmSysInitStart(&sysInitComplete);                     break;  
        case SM_TST_SYS_BEGIN:              sysProcessSmSysBeginStart(&sysBeginComplete);                   break;  
        case SM_TST_BLE_ADV_ON:             sysProcessSmBleOnStart(&sysBleComplete);                            break;  
        default:    
            ESP_LOGE(SYS_TAG, "Call to a non-exsiting statemachine in sysProcessTestStateMachine");    
    }
}

bool sysProcessTestStateMachine(uint16_t stateMachineId)
{
    bool retVal = false;

    switch(stateMachineId)
    {
        case SM_TST_SYS_INIT:           
            retVal = sysProcessSmSysInit(&sysInitComplete, &sysInitResult);
            break;

        case SM_TST_SYS_BEGIN:        
            retVal = sysProcessSmSysBegin(&sysBeginComplete, &sysBeginResult);
            break;

        case SM_TST_BLE_ADV_ON:         
            retVal = sysProcessSmSysBegin(&sysBleComplete, &sysBleResult);
            break;

        default:    
            ESP_LOGE(SYS_TAG, "Call to a non-exsiting statemachine in sysProcessTestStateMachine");
            retVal = true;
    }
    return retVal;    
}

//System commands



//----------------------------------------------------------------------
//Sequence Machine
//----------------------------------------------------------------------

/// @brief 
/// @param  
void sys_ProcessStateSequences(void)
{
    EVENT_message_t initEvtMsg;

    switch(sysCurrentEvt)
    {
        case EVT_SYS_IDLE:
            break;

        //------------------------------------------------------------------
        // System Events
        //------------------------------------------------------------------
        case EVT_SYS_INIT:
            switch(sysCurrentEvtSeq)
            {
                case SCTL_EVT_STATE_IDLE:
                    sysCurrentEvtSeq = SCTL_SEQ1_ST10;
                    sysCurrentSmComplete = false;
                    sysCurrentEvtComplete = false;
	                sysSeq.init = true;
                    break;

                case SCTL_SEQ1_ST10:    //Start sequence
                        sysSeqState = SCTL_SEQ_STATE_IDLE;                      
                        sysCurrentEvtSeq = SCTL_SEQ1_ST20;
                        sysCurrentSmComplete = false;
                        sysCurrentEvtComplete = false;
                        sysProcessSmSysInitStart(&sysInitComplete);
                        ESP_LOGI(SYS_TAG,"\n\nEVT_SYS_INIT - Started\n\n");
                    break;

                case SCTL_SEQ1_ST20:    //Init System sequence
                    sysCurrentSmComplete = sysProcessSmSysInit(&sysInitComplete, &sysInitResult);
                    if (sysCurrentSmComplete)
                    {
                        sysSeqState = SCTL_SEQ_STATE_IDLE;
                        sysCurrentEvtSeq = SCTL_SEQ1_ST30; 
                        sysLogR("\r\n----------------------------------------------------");
                        sysLogR("FARM 25");
                        sprintf(sctl_TmpStr, "Firmware version: %d.%d.%d", FIRMWARE_VERSION_RELEASE, FIRMWARE_VERSION_MAJOR, FIRMWARE_VERSION_MINOR);
                        sysLogR(sctl_TmpStr);                      
                        sysLogR("----------------------------------------------------");
                        ESP_LOGI(SYS_TAG,"\n\nsysProcessSmSysInit - complete\n\n"); 
					}
					break;

                case SCTL_SEQ1_ST30:    //Begin System sequence
                        sysSeqState = SCTL_SEQ_STATE_IDLE;
                        sysCurrentEvtSeq = SCTL_SEQ1_ST40;                       
                        sysCurrentSmComplete = false;
                        ESP_LOGI(SYS_TAG,"\n\nsysProcessSmSysBeginStart\n\n");
                        sysProcessSmSysBeginStart(&sysBeginComplete);
                    break;

                case SCTL_SEQ1_ST40:
                    sysCurrentSmComplete = sysProcessSmSysBegin(&sysBeginComplete, &sysBeginResult);
					
                    if (sysCurrentSmComplete)
					{                       
                        sysStatus.wakeupReason = sysControlCheckWakeupReason();
                        sysCurrentEvtSeq = SCTL_SEQ1_ST98;                       
                    }
                    break;

                case SCTL_SEQ1_ST98:
                        sysSeqState = SCTL_SEQ_STATE_IDLE;
                        sysCurrentEvt = EVT_SYS_IDLE;
                        initEvtMsg.evtData = 0;
                        initEvtMsg.evtDataPtr = NULL;
                        initEvtMsg.evtDataLen = 0;
                        sysCurrentEvtSeq = SCTL_EVT_STATE_IDLE; 
                        sysCurrentEvtComplete = true;

                        ESP_LOGI(SYS_TAG,"VT_SYS_INIT - complete");

                        initEvtMsg.evtCmd = EVT_SYS_STARTUP;
                        sctl_processEventMsgFromSys(&initEvtMsg);                      
	                    sysSeq.init = false;
                        break;


                case SCTL_SEQ1_ST99:
                    sysSeqState = SCTL_SEQ_STATE_IDLE;
                    sysCurrentEvtSeq = SCTL_EVT_STATE_IDLE;
                    sysCurrentEvt = EVT_SYS_IDLE;
                    ESP_LOGE(SYS_TAG, "EVT_SYS_INIT - failed");
                    sysCurrentEvtComplete = true;
	                sysSeq.init = false;
                    break;


                default:
                    sysSeqState = SCTL_SEQ_STATE_IDLE;
                    sysCurrentEvtSeq = SCTL_EVT_STATE_IDLE;
                    sysCurrentEvt = EVT_SYS_IDLE;
                    ESP_LOGE(SYS_TAG, "EVT_SYS_INIT - default");
                    sysCurrentEvtComplete = true;
	                sysSeq.init = false;                    
                    break;
            }
            break;

        case EVT_SYS_STARTUP:
            switch(sysCurrentEvtSeq)
            {
                case SCTL_EVT_STATE_IDLE:
                    sysCurrentEvtComplete = false;
                    sysSeq.startup = true;
                    sysCurrentEvtSeq = SCTL_SEQ2_ST20;
                    break;

                case SCTL_SEQ2_ST20:
                    {
                        switch(sysStatus.wakeupReason)
                        {
                            case WKUP_TIMER:
                                ESP_LOGI(SYS_TAG,"wakeup timer");
                                initEvtMsg.evtCmd = EVT_SYS_TIMER_WAKE_UP; 
                                //initEvtMsg.evtCmd = EVT_SYS_POWER_ON; 
                                break;
                                
                            case WKUP_POWER_ON:
                                ESP_LOGI(SYS_TAG,"wakeup power on");
                                initEvtMsg.evtCmd = EVT_SYS_POWER_ON;
                                break;
                                
                            case WKUP_EXTERN0:
                                ESP_LOGI(SYS_TAG,"wakeup from extern 0 input");
                                initEvtMsg.evtCmd = EVT_SYS_POWER_ON;
                                break;
                                
                            case WKUP_EXTERN1:
                                ESP_LOGI(SYS_TAG,"wakeup from extern 1 input");
                                initEvtMsg.evtCmd = EVT_SYS_POWER_ON;
                                break;
                                
                            case WKUP_UNKNOWN:
                                ESP_LOGE(SYS_TAG,"wakeup reason - unknown!");
                                initEvtMsg.evtCmd = EVT_SYS_POWER_ON; //For now until a wakeup from timer routine has been created
                                break;

                            default:
                                ESP_LOGE(SYS_TAG,"wakeup reason - default!"); 
                                initEvtMsg.evtCmd = EVT_SYS_POWER_ON; //For now until a wakeup from timer routine has been created   
                                
                        }                        
                        sctl_processEventMsgFromSys(&initEvtMsg);                                               
                        sysCurrentEvtSeq = SCTL_SEQ2_ST98;
                    }
                    break;


                case SCTL_SEQ2_ST98:    //Sequence ended sucessfully
                    sysSeqState = SCTL_SEQ_STATE_IDLE;
                    sysCurrentEvt = EVT_SYS_IDLE;
                    sysCurrentEvtSeq = SCTL_EVT_STATE_IDLE;
                    sysCurrentEvtComplete = true;
                    sysSeq.startup = false;
                    ESP_LOGI(SYS_TAG,"\n\nEVT_SYS_STARTUP - pass and complete\n\n");
                    break;

                case SCTL_SEQ2_ST99:    //Sequence ended unsucessfully
                    sysSeqState = SCTL_SEQ_STATE_IDLE;
                    sysCurrentEvtSeq = SCTL_EVT_STATE_IDLE;
                    sysCurrentEvt = EVT_SYS_IDLE;
                    ESP_LOGE(SYS_TAG, "EVT_SYS_STARTUP - fail and complete");
                    sysCurrentEvtComplete = true;
                    sysSeq.startup = false;
                    break;

                default:
                    sysSeqState = SCTL_SEQ_STATE_IDLE;
                    sysCurrentEvtSeq = SCTL_EVT_STATE_IDLE;
                    sysCurrentEvt = EVT_SYS_IDLE;
                    ESP_LOGE(SYS_TAG, "EVT_SYS_STARTUP - default");
                    sysCurrentEvtComplete = true;
                    sysSeq.startup = false;
                    break;

            }
            break;


        case EVT_SYS_TEST_SM:
            switch(sysCurrentEvtSeq)
            {
                case SCTL_EVT_STATE_IDLE:
                    sysCurrentEvtSeq = SCTL_SEQ3_ST10;
                    sysCurrentSmId = (uint16_t)sysCurrentEvtData;
                    sysCurrentSmComplete = false;
                    sysCurrentEvtComplete = false;
                    sysSeq.sm_test = true;
                    sysProcessTestStateMachineStart(sysCurrentSmId);
                    ESP_LOGI(SYS_TAG, "Process state machine test, ID:%d", sysCurrentSmId);
                    break;

                case SCTL_SEQ3_ST10:
                    sysCurrentSmComplete = sysProcessTestStateMachine(sysCurrentSmId);
                    if (sysCurrentSmComplete)
                    {
                        sysSeqState = SCTL_SEQ_STATE_IDLE;
                        sysCurrentEvtSeq = SCTL_EVT_STATE_IDLE;
                        sysCurrentEvt = EVT_SYS_IDLE;
                        sysCurrentSmComplete = true;
                        sysCurrentEvtComplete = true;
                        sysSeq.sm_test = false;
                        ESP_LOGI(SYS_TAG, "EVT_SYS_TEST_SM - complete");
                    }
                    break;

                default:
                    sysSeqState = SCTL_SEQ_STATE_IDLE;
                    sysCurrentEvtSeq = SCTL_EVT_STATE_IDLE;
                    sysCurrentEvt = EVT_SYS_IDLE;
                    ESP_LOGE(SYS_TAG, "EVT_SYS_TEST_SM - default");
                    sysCurrentEvtComplete = true;
                    sysSeq.sm_test = false;
                    break;
            }
            break;

        case EVT_SYS_POWER_ON:
            switch(sysCurrentEvtSeq)
            {
                case SCTL_EVT_STATE_IDLE:
                    sysCurrentEvtComplete = false;
                    sysCurrentEvtSeq = SCTL_SEQ4_ST10;
                    sysSeq.power_on = true;
                    break;

                case SCTL_SEQ4_ST10: // Start BLE On sequence
                    if(sysStatus.bleConnected)
                    {
                        sysCurrentEvtSeq = SCTL_SEQ4_ST98;
                    }
                    else
                    {
                        sysProcessSmBleOnStart(&sysBleComplete);
                        sysCurrentEvtSeq = SCTL_SEQ4_ST80;
                    }
                    break;

                case SCTL_SEQ4_ST80: // BLE On sequence
                    sysCurrentSmComplete = sysProcessSmBleOn(&sysBleComplete, &sysBleResult);

                    if (sysCurrentSmComplete)
                    {
                        sysCurrentEvtSeq = SCTL_SEQ4_ST98;
                    }
                    break;

                case SCTL_SEQ4_ST98:    //Sequence ended sucessfully
                    sysSeqState = SCTL_SEQ_STATE_IDLE;
                    sysCurrentEvtSeq = SCTL_EVT_STATE_IDLE;
                    sysCurrentEvt = EVT_SYS_IDLE;
                    sysCurrentEvtComplete = true;
                    sysSeq.power_on = false;
// #ifndef SLEEP_TIMER_OFF                    
//                     sysSleepTimerStart();
// #endif                    
                    ESP_LOGI(SYS_TAG,"\n\nEVT_SYS_POWER_ON - complete\n\n");
                    printf("{\"system\":\"startup\"}\r\n");

//                     if (otaSignature == OTA_SIG_START)
//                     {
//                         ESP_LOGI(SYS_TAG,"\n\nReset esp for OTA\n\n");
// #ifdef ENABLE_OTA                        
//                         esp_restart();
// #endif                        
//                     }
                  
                    break;

                case SCTL_SEQ4_ST99:    //Sequence ended unsucessfully
                    sysSeqState = SCTL_SEQ_STATE_IDLE;
                    sysCurrentEvtSeq = SCTL_EVT_STATE_IDLE;
                    sysCurrentEvt = EVT_SYS_IDLE;                   
                    sysCurrentEvtComplete = true;
                    sysSeq.power_on = false;
                    ESP_LOGE(SYS_TAG, "EVT_SYS_POWER_ON - failed");
                    break;

                default:
                    sysSeqState = SCTL_SEQ_STATE_IDLE;
                    sysCurrentEvtSeq = SCTL_EVT_STATE_IDLE;
                    sysCurrentEvt = EVT_SYS_IDLE;
                    sysCurrentEvtComplete = true;
                    sysSeq.power_on = false;
                    ESP_LOGE(SYS_TAG, "EVT_SYS_POWER_ON - default");                   
                    break;

            }
            break;


        //------------------------------------------------------------------
        // Dispatcher Events
        //------------------------------------------------------------------

        //------------------------------------------------------------------
        // Serial Events
        //------------------------------------------------------------------

        //------------------------------------------------------------------
        // Supervisor Events
        //------------------------------------------------------------------

        //------------------------------------------------------------------
        // Default Event
        //------------------------------------------------------------------
    }
}


//----------------------------------------------------------------------
//Local Functions
//----------------------------------------------------------------------

void sysTestStateMachine(uint16_t stateMachineId)
{
    EVENT_message_t evtMsg;

    evtMsg.evtCmd = EVT_SYS_TEST_SM;
    evtMsg.evtData = stateMachineId;
    evtMsg.evtDataPtr = NULL;
    evtMsg.evtDataLen = 0;

    ESP_LOGI(SYS_TAG, "Put test Statemachine request into event queue");

    sctl_processEventMsgFromSys(&evtMsg);    
}

void sysSetSeqSystemIdle(void)
{
    ESP_LOGI(SYS_TAG,"sysSetSeqSystemIdle called");
    sysSeqState = SCTL_SEQ_STATE_IDLE;
    sysCurrentSmComplete = true;
}

void sys_ProcessEvent(void)
{
    if (sysCurrentEvtComplete)
    {
        if (xQueueReceive(eventQueueHandle,&sysControlEvtRxMessage,10))
        {
            sysCurrentEvt = sysControlEvtRxMessage.evtCmd;
            sysCurrentEvtData = sysControlEvtRxMessage.evtData;
            sysCurrentEvtSeq = SCTL_EVT_STATE_IDLE;
            ESP_LOGI(SYS_TAG, " Next Event from Event Queue: %d", sysCurrentEvt);
        }
    }

}

void sctl_processEventMsgFromSys(EVENT_message_t * evtMsg)
{
    EVENT_message_t evtTxMessage;

    ESP_LOGI(SYS_TAG, "Event %d from system controller detected and processing", evtMsg->evtCmd);

    evtTxMessage.evtCmd       = evtMsg->evtCmd;
    evtTxMessage.evtData      = evtMsg->evtData;
    evtTxMessage.evtDataPtr   = evtMsg->evtDataPtr;
    evtTxMessage.evtDataLen   = evtMsg->evtDataLen;
    
    ESP_LOGI(SYS_TAG, "EMS Queue space available before: %d", uxQueueSpacesAvailable(eventQueueHandle));

    if (xQueueSend(eventQueueHandle,&evtTxMessage,0) != pdPASS){
        ESP_LOGE(SYS_TAG, "Event Queue is full! Event %d from system controller has been lost", evtTxMessage.evtCmd);
    }

    ESP_LOGI(SYS_TAG, "EMS Queue space available after: %d", uxQueueSpacesAvailable(eventQueueHandle));
}


bool bleProcessSnResult(char * msgString)
{
    if (sctl_Pm_sysCtrlRxMsg.msgData == BLE_RETURN_GOOD)
    {

        sprintf(sctl_TmpStr, "%s Passed, msg ID:%lu, TS:%lu\r\n", msgString, (ulong)sctl_Pm_sysCtrlRxMsg.msgRef, (ulong)sctl_Pm_sysCtrlRxMsg.msgTimeStamp);
        ESP_LOGI(SYS_TAG,"%s",sctl_TmpStr);        
        return (true);
    }
    else
    {
        sprintf(sctl_TmpStr, "%s Error!, msg ID:%lu, TS:%lu\r\n", msgString, (ulong)sctl_Pm_sysCtrlRxMsg.msgRef, (ulong)sctl_Pm_sysCtrlRxMsg.msgTimeStamp);
        ESP_LOGE(SYS_TAG,"%s",sctl_TmpStr);
    }

    return (false);
}

bool bleProcessResult(char * msgString)
{
    if (sctl_Pm_sysCtrlRxMsg.msgData == BLE_RETURN_GOOD)
    {

        sprintf(sctl_TmpStr, "%s Passed, msg ID:%lu, TS:%lu\r\n", msgString, (ulong)sctl_Pm_sysCtrlRxMsg.msgRef, (ulong)sctl_Pm_sysCtrlRxMsg.msgTimeStamp);
        ESP_LOGI(SYS_TAG,"%s",sctl_TmpStr);        
        return (true);
    }
    else
    {
        sprintf(sctl_TmpStr, "%s Error!, msg ID:%lu, TS:%lu\r\n", msgString, (ulong)sctl_Pm_sysCtrlRxMsg.msgRef, (ulong)sctl_Pm_sysCtrlRxMsg.msgTimeStamp);
        ESP_LOGE(SYS_TAG,"%s",sctl_TmpStr);
    }

    //sysSendMessage(MSG_ADDR_LED, MSG_DATA_8, LED_CMD_OFF, NULL, BLE_LED_REF, MSG_DATA_8_LEN);

    return (false);
}

bool bleStartProcessResult(char * msgString)
{
    if (sctl_Pm_sysCtrlRxMsg.msgData == BLE_RETURN_GOOD)
    {

        sprintf(sctl_TmpStr, "%s Passed, msg ID:%lu, TS:%lu\r\n", msgString, (ulong)sctl_Pm_sysCtrlRxMsg.msgRef, (ulong)sctl_Pm_sysCtrlRxMsg.msgTimeStamp);
        ESP_LOGI(SYS_TAG,"%s",sctl_TmpStr);        
        return (true);
    }
    else
    {
        sprintf(sctl_TmpStr, "%s Error!, msg ID:%lu, TS:%lu\r\n", msgString, (ulong)sctl_Pm_sysCtrlRxMsg.msgRef, (ulong)sctl_Pm_sysCtrlRxMsg.msgTimeStamp);
        ESP_LOGE(SYS_TAG,"%s",sctl_TmpStr);
    }
    return (false);
}

uint8_t sysControlCheckWakeupReason(void)
{
    esp_sleep_wakeup_cause_t wakeup_reason;

    gpio_config_t io_conf;
    esp_err_t     ret;

    uint8_t WakeupSource = WKUP_UNKNOWN;
    
    wakeup_reason = esp_sleep_get_wakeup_cause();

    // io_conf.intr_type = GPIO_INTR_DISABLE;
    // io_conf.mode = GPIO_MODE_INPUT;
    // io_conf.pin_bit_mask = (1ULL << AUTO_WAKE_TARE) | (1ULL << WAKEUP_BTN);
    // io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    // io_conf.pull_up_en = GPIO_PULLUP_DISABLE;

    // ret = gpio_config(&io_conf);

    // if (ret != ESP_OK)
    // {
    //     ESP_LOGE(SYS_TAG, "keypadInit failed: ret = 0x%.04X", ret);
    // }

    printf("\r\n\n@@@@@\r\n");
    printf("wakeup_reason: 0x%X\r\n", wakeup_reason);
    printf("@@@@@\r\n\n");

    switch (wakeup_reason)
    {
        case ESP_SLEEP_WAKEUP_TIMER:
            printf("-----\r\n");
            printf("Wakeup caused by timer\r\n");
            WakeupSource = WKUP_TIMER;
            break;

        case ESP_SLEEP_WAKEUP_EXT0: 
            printf("-----\r\n");
            printf("Wakeup caused by Extern 0 input\r\n");
            WakeupSource = WKUP_EXTERN0;
            break;

        case ESP_SLEEP_WAKEUP_EXT1: 
            printf("-----\r\n");
            printf("Wakeup caused by Extern 1 input\r\n");
            WakeupSource = WKUP_EXTERN1;
            break;

        default:
            printf("-----\r\n");
            printf("Initial scale power-on\r\n");
            WakeupSource = WKUP_POWER_ON;
            break;
    }
    return WakeupSource;
}


/// @brief  This function sends the info log message to the BLE queue reguardless of 
///    the mobile device connected and/or the Log notification enabled. It will also send
///    the log message to the terminal.
/// @param strPtr 
void sysLogR(char * strPtr)
{
    ESP_LOGI(SYS_TAG, "%s", strPtr);
    sysLog(strPtr, true, false);
}

/// @brief  This function sends the info log message to the BLE queue reguardless of 
///    the mobile device connected and/or the Log notification enabled. It will also send
///    the log message to the terminal.
/// @param strPtr 
void sysLogIF(char * strPtr)
{
    ESP_LOGI(SYS_TAG, "%s", strPtr);
    sysLog(strPtr, true, true);
}

/// @brief  This function sends the error log message to the BLE queue reguardless of 
///    the mobile device connected and/or the Log notification enabled. It will also send
///    the log message to the terminal.
void sysLogEF(char * strPtr)
{
    ESP_LOGE(SYS_TAG, "%s", strPtr);
    sysLog(strPtr, true, true);
}

/// @brief  This function send the info log message to the Mobile device and to the 
///    terminal.
/// @param strPtr 
void sysLogI(char * strPtr)
{
    ESP_LOGI(SYS_TAG, "%s", strPtr);
    sysLog(strPtr, false, true);
}

/// @brief This function send the error log message to the Mobile device and to the 
///    terminal. 
/// @param strPtr 
void sysLogE(char * strPtr)
{
    ESP_LOGE(SYS_TAG, "%s", strPtr);
    sysLog(strPtr, false, true);
}

/// @brief This function sends a message to the BLE module to send out a log message
///   to the mobile device via BLE. Mobile device has to be connected and the
///   log notifications (RX Characteristics) enabled.
/// @param strPtr - message to be sent to phone.
/// @param forced - If true, message is forced in the log queue.
/// @param printTag - If true the tag is printed with the message. 
void sysLog(char * strPtr, bool forced, bool printTag)
{
    static uint8_t printIndex = 0;
    RTOS_message_t modPrintMsg;
    char tmpStr[STRING_MAX_LOG_CHARACTERS];
    static char modStr[SYS_STRING_MAX_ARRAY_LOG_ELEMENTS][STRING_MAX_LOG_CHARACTERS];

    memcpy(tmpStr, strPtr, strlen(strPtr)+1);
	if (printTag){
        sprintf(modStr[printIndex], "%s:%s", SYS_TAG, tmpStr);
    }else{
        sprintf(modStr[printIndex], "%s", tmpStr);
    }

    //form message to print out string
    modPrintMsg.srcAddr         =  MSG_ADDR_SCTL; 
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
    if (printIndex >= SYS_STRING_MAX_ARRAY_LOG_ELEMENTS)
    {
        printIndex = 0;
    }
}
