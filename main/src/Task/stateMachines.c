#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/task.h"
#include <time.h>
#include <sys/time.h>
#include "esp_system.h"
#include "esp_log.h"
#include "esp_sleep.h"

#include "Shared/common.h"
#include "Shared/messages.h"
#include "Task/sysControlTask.h"
#include "Task/stateMachines.h"



//Local Parameters
#define SM_TAG "SM"

//External Functions
bool sysProcessSmIdle(uint16_t sysCurrentEvt, uint16_t sysNextEvt)
{
    static bool retVal = true;

    if (sysCurrentEvt != sysNextEvt)
    {
        ESP_LOGI(SM_TAG, "sysProcessSmIdle called");
        ESP_LOGI(SM_TAG, "sysCurrentEvt = %d, sysNextEvt = %d", sysCurrentEvt, sysNextEvt);
        sysCurrentEvt = sysNextEvt;
        sysNextEvt = EVT_SYS_IDLE;
        retVal = false;
    }

    return retVal;
}

bool smTimeout(uint32_t * smTime, const uint32_t smTimeCheck, const char * sm)
{  
    uint32_t currentTime = sys_getMsgTimeStamp();
    //ESP_LOGI(SM_TAG,"Current time: %ld", currentTime);

    if (sys_getMsgTimeStamp() < *smTime)    //In case the timer rolls over
    {
        *smTime = currentTime; 
    }

    if ((currentTime - *smTime) > smTimeCheck)
    {
        ESP_LOGE(SM_TAG,"%s SM Timeout!", sm);
        ESP_LOGI(SM_TAG,"Current time: %ld, sm time: %ld, check time: %ld", currentTime, *smTime, smTimeCheck);
        return true;
    }

    return false;
}

//----------------------------------------------------------------------
//System initalizaion State Machine
//----------------------------------------------------------------------

/// @brief 
/// @param sysInitComplete 
void sysProcessSmSysInitStart(sys_init_complete_t * sysInitComplete)
{
    sysInitComplete->start_SM = true;    //start the state machine
} 

/// @brief 
/// @param sysInitComplete 
/// @param sysInitResult 
/// @return 
bool sysProcessSmSysInit(sys_init_complete_t * sysInitComplete, sys_init_result_t * sysInitResult)
{
    static uint16_t smSysInitState = SCTL_SM1_ST00;
    static uint32_t smSysInitTime = 0;                           

    if (sysInitComplete->start_SM)
    {
        switch(smSysInitState)
        {
            case SCTL_SM1_ST00: 
                memset((void*)sysInitComplete,0,sizeof(sysInitComplete));
                sysInitComplete->start_SM = true;
                memset((void*)sysInitResult,0,sizeof(sysInitResult));
                ESP_LOGI(SM_TAG,"\n\nsysProcessSmSysInit - start\n\n");
                ESP_LOGI(SM_TAG,"Initialize Dispatcher");
                sysSendMessage(MSG_ADDR_DSPR, MSG_DATA_0, DSPR_CMD_INIT,  NULL, MSG_DATA_COMMAND_ONLY, MSG_DATA_0_LEN);
                smSysInitState = SCTL_SM1_ST10;
                smSysInitTime = sys_getMsgTimeStamp();
                ESP_LOGI(SM_TAG, "sysProcessSmSysInit start time:%ld", smSysInitTime);
                break;

            //Intialization Sequence
            case SCTL_SM1_ST10:
                if (sysInitComplete->is_DsprDone)
                {
                    if(sysInitResult->dsprInit)
                    {
                        ESP_LOGI(SM_TAG,"Initialize Storage");
                        sysSendMessage(MSG_ADDR_STRG, MSG_DATA_0, STRG_CMD_INIT,  NULL, MSG_DATA_COMMAND_ONLY, MSG_DATA_0_LEN);
                        smSysInitState = SCTL_SM1_ST14;
                    }
                    else
                    {
                        ESP_LOGE(SM_TAG,"Dispatcher init Failed");
                        smSysInitState = SCTL_SM1_ST99;
                    }
                }
                break;

            case SCTL_SM1_ST14:
                if (sysInitComplete->is_StrgDone)
                {
                    if(sysInitResult->strgInit)                    {
                        ESP_LOGI(SM_TAG,"Mount SPIFFS");
                        sysSendMessage(MSG_ADDR_STRG, MSG_DATA_0, STRG_CMD_MOUNT_SPIFFS,  NULL, MSG_DATA_COMMAND_ONLY, MSG_DATA_0_LEN);
                        smSysInitState = SCTL_SM1_ST15;
                    }
                    else
                    {
                        ESP_LOGE(SM_TAG,"Wifi init Failed");
                        smSysInitState = SCTL_SM1_ST99;
                    }
                }
                break;

            case SCTL_SM1_ST15:
                if (sysInitComplete->is_mountDone)
                {
                    if (sysInitResult->mountInit)
                    {
                        ESP_LOGI(SM_TAG,"Initialize Data Queue");
                        sysSendMessage(MSG_ADDR_STRG, MSG_DATA_0, STRG_CMD_DATA_QUEUE_INIT,  NULL, MSG_DATA_COMMAND_ONLY, MSG_DATA_0_LEN);
                        smSysInitState = SCTL_SM1_ST16;
                    }
                    else
                    {
                        ESP_LOGE(SM_TAG,"SPIFFS MOUNT init Failed");
                        smSysInitState = SCTL_SM1_ST99;
                    }
                }
                break;

            case SCTL_SM1_ST16:
                if (sysInitComplete->is_dataQDone)
                {
                    if (sysInitResult->dataQInit)
                    {
                        ESP_LOGI(SM_TAG,"Initialize Log Queue");
                        sysSendMessage(MSG_ADDR_STRG, MSG_DATA_0, STRG_CMD_LOG_QUEUE_INIT,  NULL, MSG_DATA_COMMAND_ONLY, MSG_DATA_0_LEN);
                        smSysInitState = SCTL_SM1_ST17;
                    }
                    else
                    {
                        ESP_LOGE(SM_TAG,"Data Queue init Failed");
                        smSysInitState = SCTL_SM1_ST99;
                    }
                }
                break;

            case SCTL_SM1_ST17:
                if (sysInitComplete->is_logQDone)
                {
                    if (sysInitResult->logQInit)
                    {
                        ESP_LOGI(SM_TAG,"Initialize Serial");
                        sysSendMessage(MSG_ADDR_SER, MSG_DATA_0, SER_CMD_INIT,  NULL, MSG_DATA_COMMAND_ONLY, MSG_DATA_0_LEN);
                        smSysInitState = SCTL_SM1_ST20;
                    }
                    else
                    {
                        ESP_LOGE(SM_TAG,"Log Queue init Failed");
                        smSysInitState = SCTL_SM1_ST99;
                    }
                }
                break;


            case SCTL_SM1_ST20:
                if (sysInitComplete->is_SerDone)
                {
                    if(sysInitResult->serInit)
                    {
                        ESP_LOGI(SM_TAG,"Initialize BLE");
                        sysSendMessage(MSG_ADDR_BLE, MSG_DATA_0, BLE_CMD_INIT,  NULL, MSG_DATA_COMMAND_ONLY, MSG_DATA_0_LEN);
                        smSysInitState = SCTL_SM1_ST30;
                    }
                    else
                    {
                        ESP_LOGE(SM_TAG,"Serial init Failed");
                        smSysInitState = SCTL_SM1_ST99;
                    }
                }
                break;

            case SCTL_SM1_ST30:
                if (sysInitComplete->is_BleDone)
                {
                    if(sysInitResult->bleInit)
                    {
                        ESP_LOGI(SM_TAG,"Initialize WIFI");
                        //sysSendMessage(MSG_ADDR_WIFI, MSG_DATA_0, WIFI_CMD_INIT,  NULL, MSG_DATA_COMMAND_ONLY, MSG_DATA_0_LEN);
                        smSysInitState = SCTL_SM1_ST40;
                    }
                    else
                    {
                        ESP_LOGE(SM_TAG,"BLE init Failed");
                        smSysInitState = SCTL_SM1_ST99;
                    }
                }
                break;

            case SCTL_SM1_ST40:
                sysInitComplete->is_WifiDone = true;    //temporary - remove
                sysInitResult->wifiInit = true;         //temporary - remove

                if (sysInitComplete->is_WifiDone)
                {
                    if(sysInitResult->wifiInit)
                    {
                        smSysInitState = SCTL_SM1_ST98;
                    }
                    else
                    {
                        ESP_LOGE(SM_TAG,"Wifi init Failed");
                        smSysInitState = SCTL_SM1_ST99;
                    }
                }
                break;


            case  SCTL_SM1_ST98:
                sysInitComplete->start_SM = false;
                sysInitComplete->is_SystemDone = true;
                smSysInitState = SCTL_SM1_ST00;
                ESP_LOGI(SM_TAG,"sysProcessSmSysInit - Pass and Complete");                 
                break;   

            case  SCTL_SM1_ST99:
                sysInitComplete->start_SM = false;
                sysInitComplete->is_SystemDone = true;
                smSysInitState = SCTL_SM1_ST00;                    
                ESP_LOGE(SM_TAG,"sysProcessSmSysInit - Fail and Complete");
                esp_restart();
                break;   

            default:   
                sysInitComplete->start_SM = false;
                smSysInitState = SCTL_SM1_ST00;
                ESP_LOGE(SM_TAG,"sysProcessSmSysInit - default");
                break;
        }
        if (smTimeout(&smSysInitTime, SM_SYS_INIT_TIMEOUT, "sysProcessSmSysInit")) {smSysInitState = SCTL_SM1_ST99;}
    }
    return sysInitComplete->is_SystemDone;
}


//----------------------------------------------------------------------
//System start State Machine
//----------------------------------------------------------------------

/// @brief 
/// @param sysInitComplete 
void sysProcessSmSysBeginStart(sys_begin_complete_t * beginComplete)
{
    beginComplete->start_SM = true;    //start the state machine
} 

/// @brief 
/// @param sysBeginComplete 
/// @param sysBeginResult 
/// @return 
bool sysProcessSmSysBegin(sys_begin_complete_t * beginComplete, sys_begin_result_t * beginResult)
{
    static uint16_t smSysBeginState = SCTL_SM2_ST00;
    static uint32_t smSysBeginTime = 0; 

    if (beginComplete->start_SM)
    {
        switch(smSysBeginState)
        {
            case SCTL_SM2_ST00: 
                memset((void*)beginComplete,0,sizeof(beginComplete));
                beginComplete->start_SM = true;
                memset((void*)beginResult,0,sizeof(beginResult));

                //Read current unit serial number and hardware revision
                sysSendMessage(MSG_ADDR_STRG, MSG_DATA_0, STRG_CMD_RD_SN, NULL, MSG_DATA_COMMAND_ONLY, MSG_DATA_0_LEN);
                sysSendMessage(MSG_ADDR_STRG, MSG_DATA_0, STRG_CMD_RD_HW_REV, NULL, MSG_DATA_COMMAND_ONLY, MSG_DATA_0_LEN); 

                smSysBeginState = SCTL_SM2_ST10;
                smSysBeginTime = sys_getMsgTimeStamp();
                break;

            case SCTL_SM2_ST10:
                if (beginComplete->is_SnReadInNvs)
                {
                    if(beginResult->snInNvs){
                        //BLE load current SN
                        sysSendMessage(MSG_ADDR_BLE,MSG_DATA_8, BLE_CMD_SET_SN,NULL,MSG_DATA_COMMAND_ONLY,MSG_DATA_0_LEN);
                        smSysBeginState = SCTL_SM2_ST20;
                    }else{
                        ESP_LOGE(SM_TAG,"SN to NVS Failed");
                        smSysBeginState = SCTL_SM2_ST99;   //Failure - End state machine
                    }
                }
                break;

            case SCTL_SM2_ST20:
                if (beginComplete->is_SnReadInBLE)
                {
                    if(beginResult->snInBLE){
                        //Initialize BLE advertisement
                        sysSendMessage(MSG_ADDR_BLE,MSG_DATA_8, BLE_CMD_INIT_ADV,NULL,MSG_DATA_COMMAND_ONLY,MSG_DATA_0_LEN);
                        smSysBeginState = SCTL_SM2_ST30;
                    }else{
                        ESP_LOGE(SM_TAG,"SN to BLE Failed");
                        smSysBeginState = SCTL_SM2_ST99;   
                    }
                }
                break;

            case SCTL_SM2_ST30:
                if (beginComplete->is_BleStarted)
                {
                    if (beginResult->bleStartDone){
                        smSysBeginState = SCTL_SM2_ST98;   
                    }else{
                        ESP_LOGE(SM_TAG,"BLE Adv Init Failed!");
                        smSysBeginState = SCTL_SM2_ST99;   
                    }
                }
                break;    

            case SCTL_SM2_ST98:
                {
                    beginComplete->is_SysBeginDone = true;
                    beginResult->systemBegin = true;

                    beginComplete->start_SM = false;
                    smSysBeginState = SCTL_SM2_ST00;
                    ESP_LOGI(SM_TAG,"sysProcessSmSysBegin - Pass and Complete");

                }
                break;

            case SCTL_SM2_ST99:
                {
                    beginComplete->is_SysBeginDone = true;
                    beginResult->systemBegin = false;
                    beginComplete->start_SM = false;
                    smSysBeginState = SCTL_SM2_ST00;
                    ESP_LOGI(SM_TAG,"sysProcessSmSysBegin - Fail and Complete");
                }
                break;

            default:   
                beginComplete->start_SM = false;
                smSysBeginState = SCTL_SM2_ST00;
                ESP_LOGE(SM_TAG,"sysProcessSmSysBegin - default");
                break;
        }
        if (smTimeout(&smSysBeginTime, SM_SYS_BEGIN_TIMEOUT, "sysProcessSmSysBegin")) {smSysBeginState = SCTL_SM2_ST99;}
    }
    return beginComplete->is_SysBeginDone;
}

//----------------------------------------------------------------------
//BLE Advertisement On state Machine
//----------------------------------------------------------------------

/// @brief 
/// @param sys_ble_complete 
void sysProcessSmBleOnStart(sys_ble_complete_t * sys_ble_complete)
{
    sys_ble_complete->start_SM = true;    //start the state machine
}

/// @brief 
/// @param sys_ble_complete 
/// @param sys_ble_result 
/// @return 
bool sysProcessSmBleOn(sys_ble_complete_t * sys_ble_complete, sys_ble_result_t * sys_ble_result)
{
    static uint16_t smBleOnState = SCTL_SM3_ST00;
    static uint32_t smSysBleOnTime = 0;

    if (sys_ble_complete->start_SM)
    {
        switch(smBleOnState)
        {
            case SCTL_SM3_ST00: 
                memset((void*)sys_ble_complete,0,sizeof(sys_ble_complete));
                sys_ble_complete->start_SM = true;
                memset((void*)sys_ble_result,0,sizeof(sys_ble_result));
                ESP_LOGI(SM_TAG,"sysProcessSmBleOn - started");
                sysSendMessage(MSG_ADDR_BLE, MSG_DATA_0, BLE_CMD_SET_SN,  NULL, MSG_DATA_COMMAND_ONLY, MSG_DATA_0_LEN);
                smBleOnState = SCTL_SM3_ST05;
                smSysBleOnTime = sys_getMsgTimeStamp();
                break;

            //SN loaded On?
            case SCTL_SM3_ST05:
                if (sys_ble_complete->is_SN_Set)
                {
                    if (sys_ble_result->SN_Loaded)
                    {
                        ESP_LOGI(SM_TAG,"SN loaded");
                        sysSendMessage(MSG_ADDR_BLE, MSG_DATA_0, BLE_CMD_ADV_START,  NULL, MSG_DATA_COMMAND_ONLY, MSG_DATA_0_LEN);
                        smBleOnState = SCTL_SM3_ST10;
                    }
                    else
                    {
                        ESP_LOGE(SM_TAG,"SN load - failure!");
                        smBleOnState = SCTL_SM3_ST99;
                    }
                }
                break;

            //BLE Advertisement on?
            case SCTL_SM3_ST10:
                if (sys_ble_complete->is_Adv_On_Done)
                {
                    if (sys_ble_result->Adv_On)
                    {
                        ESP_LOGI(SM_TAG,"BLE advertisement on");
                        smBleOnState = SCTL_SM3_ST98;
                    }
                    else
                    {
                        ESP_LOGE(SM_TAG,"BLE advertisement Failed");
                        smBleOnState = SCTL_SM3_ST99;
                    }
                }
                break;

            //End of SM           
            case SCTL_SM3_ST98:
                {
                    sys_ble_complete->is_BleDone = true;

                    if (sys_ble_result->Adv_On  &&
                        sys_ble_result->SN_Loaded)
                    {
                        sys_ble_result->bleResult = true;
                        ESP_LOGI(SM_TAG, "sysProcessSmBleOn - Passed");
                    }
                    else
                    {
                        ESP_LOGE(SM_TAG, "sysProcessSmBleOn - Failed!");
                        if (!sys_ble_result->Adv_On){
                            ESP_LOGE(SM_TAG, "Turn Advertising on Failed");
                        }
                        if (!sys_ble_result->SN_Loaded){
                            ESP_LOGE(SM_TAG, "Serial number not loaded");
                        }
                    }
                    sys_ble_complete->start_SM = false;
                    smBleOnState = SCTL_SM3_ST00;
                    ESP_LOGI(SM_TAG, "sysProcessSmBleOn - complete");
                }       
                break;   

            //Terminate this state machine due to a failure
            case SCTL_SM3_ST99:
                sys_ble_complete->is_BleDone = true;
                sys_ble_result->bleResult = false;
                sys_ble_complete->start_SM = false;
                ESP_LOGE(SM_TAG,"sysProcessSmBleOn - Failed!");
                smBleOnState = SCTL_SM3_ST00;
                ESP_LOGI(SM_TAG,"sysProcessSmBleOn - complete");
                break;    

            default:   
                sys_ble_complete->start_SM = false;
                smBleOnState = SCTL_SM3_ST00;
                ESP_LOGE(SM_TAG,"sysProcessSmBleOn - default");
                break;
        }
        if (smTimeout(&smSysBleOnTime, SM_SYS_BLE_CON_TIMEOUT, "sysProcessSmBleOn")) {smBleOnState = SCTL_SM3_ST99;}
    }
    return sys_ble_complete->is_BleDone;    
}

