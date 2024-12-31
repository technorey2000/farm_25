#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"

#include "Shared/common.h"
#include "Task/serialTask.h"
//#include "Shared/parson.h"           //Need to remove this
#include "Shared/cJSON.h"
#include "Shared/messages.h"

//Local parameters
#define SER_TAG "SERIAL"
static char JSON_rxBuffer[JSON_RX_BUFFER_SIZE] = {0};
char SER_rxBuffer[1024] = { 0 };
char SER_txBuffer[1024] = { 0 };
bool rxJSON_MsgReceived = false;

RTOS_message_t suprTxMessage;
RTOS_message_t serialRxMessage;
RTOS_message_t serialTxMessage;

uint8_t tmp8Val = 0;
uint8_t serStatus = 0;

char serTmpStr[SER_STRING_MAX_ARRAY_CHARACTERS] = {0};
char serTmpBleStr[SER_STRING_MAX_ARRAY_CHARACTERS] = {0};
char serTmpConStr[STRING_MAX_BLE_CON_CMD_CHAR] = {0};

char serHwRev[MAX_HW_REV_LEN] = {0};

uint8_t serTmp8 = 0;

//Local functions
bool serJsonStringReceived(char * buffer);
void rxJSON_ProcessMessage(char * inBuffer);
void serSendMessage(uint8_t dstAddr, uint8_t msgType, uint16_t msgCmd, uint8_t * msgDataPtr, uint32_t msgData, uint32_t msgDataLen);
void serLog(char * strPtr, bool forced, bool printTag);
void serLogR(char * strPtr);
void serLogIF(char * strPtr);
void serLogEF(char * strPtr);
void serLogI(char * strPtr);
void serLogE(char * strPtr);
void serProcessConsoleCommand(uint8_t srcAddr, char * command);
void serProcessVerCmd(uint8_t addr);
void serProcessMenuCmd(void);
//void serProcessBatteryVoltagCmd(void);
void serProcessWifiCmd(uint8_t connect);
void serProcessResetCmd(void);
void serProcessSleepCmd(void);

void serialTaskApp(void)
{
    ESP_LOGI(SER_TAG, "Serial Task running");
    while (1) 
    {
        
		if (xQueueReceive(serialQueueHandle, &serialRxMessage, 1))
		{
            switch(serialRxMessage.dstAddr)
            {
                case MSG_ADDR_SUPR:
                    ESP_LOGI(SER_TAG, "supervisor message received");
                    print_convertMsgToJSON(serialRxMessage);
                    break;
                case MSG_ADDR_SER:
                    ESP_LOGI(SER_TAG, "serial message received");
                    switch (serialRxMessage.msgCmd) 
                    {
                        case SER_CMD_INIT:
                            ESP_LOGI(SER_TAG, "SER_CMD_INIT received");
                            serStatus = SER_INIT_COMPLETE;
                            serSendMessage( MSG_ADDR_SCTL, MSG_DATA_8, SER_CMD_INIT, NULL, serStatus, sizeof(serStatus)); 
                            break;

                        case SER_CMD_PING:  //Send ping response back to sender
                            serSendMessage(serialRxMessage.srcAddr, MSG_DATA_8, SER_CMD_PING, NULL, SER_PING_RECEIVED, MSG_DATA_8_LEN);
                            break;

                        case SER_CMD_STATUS:
                            ESP_LOGI(SER_TAG, "SER_CMD_STATUS received");
                            break;

                        case SER_CMD_PRINT:
                            ESP_LOGI(SER_TAG, "SER_CMD_PRINT received from: %d, msgID: %lu, msgTS: %lu", serialRxMessage.srcAddr, serialRxMessage.msgRef, serialRxMessage.msgTimeStamp);			
                            break;

                        case SER_CMD_CON_CMD:
                            ESP_LOGI(SER_TAG, "Console Command received from Task: %d, Command: %s", serialRxMessage.srcAddr, serialRxMessage.msgDataPtr);
                            memset(&serTmpConStr, 0, sizeof(serTmpConStr));
                            memcpy(&serTmpConStr, (char *)serialRxMessage.msgDataPtr, serialRxMessage.msgDataLen);
                            serTmp8 = serialRxMessage.srcAddr;

                            serProcessConsoleCommand(serTmp8, serTmpConStr);
                            break; 

                        case SER_CMD_PRINT_FW_VER:
                            memset(serTmpStr,0,sizeof(serTmpStr));
                            sprintf(serTmpStr, "%d.%d.%d", FIRMWARE_VERSION_RELEASE, FIRMWARE_VERSION_MAJOR, FIRMWARE_VERSION_MINOR);
                            //ESP_LOGI(SER_TAG, "{\"fw\":\"%d,%d,%d\"}", FIRMWARE_VERSION_RELEASE, FIRMWARE_VERSION_MAJOR, FIRMWARE_VERSION_MINOR);
                            serSendMessage(serialRxMessage.srcAddr, MSG_DATA_STR, serialRxMessage.msgCmd, (uint8_t *) serTmpStr, 0, strlen(serTmpStr));
                            break;    

                        default: ESP_LOGE(SER_TAG, "Unknown serial command:%d", serialRxMessage.msgCmd);   break;
                    }  
                    break;

                default: ESP_LOGE(SER_TAG,"Unknown destination address received by serial module");                         
            } 
        }
        
        rxJSON_MsgReceived = serJsonStringReceived(JSON_rxBuffer);
        //Process JSON message
        if (rxJSON_MsgReceived)
        {
            rxJSON_ProcessMessage(JSON_rxBuffer);
        }
    }
}

bool serJsonStringReceived(char * buffer)
{
    static bool rxJSON_Start = false;
    static bool rxJSON_MsgComplete = false;
    static uint16_t rxJSON_count = 0;
    uint8_t ch;
    static bool rxJSON_StringReady = false;

    if (rxJSON_StringReady)
    {
        rxJSON_StringReady = false;
    }
    
    ch = fgetc(stdin);

    if (ch!=0xFF)
    {
        if (ch == '{')
        {
            rxJSON_Start = true;
            rxJSON_count = 0;
            rxJSON_StringReady = false;
        }
        else if ((ch == '}') && rxJSON_Start)
        {
            rxJSON_Start = false;
            buffer[rxJSON_count] = ch;
            buffer[rxJSON_count+1] = 0; //Terminate string
            rxJSON_MsgComplete = true;
        }

        if (rxJSON_Start)
        {
            if (rxJSON_count >= JSON_RX_BUFFER_SIZE)
            {
                //printf("count:%d", rxJSON_count);
                rxJSON_Start = false;
                rxJSON_MsgComplete = false;
                rxJSON_count = 0;
                ESP_LOGE(SER_TAG, "***Receive Buffer Overload. Message deleted***\r\n");
                
            }
            if ((ch != '\r') && (ch != '\n') && (ch != ' '))
            {
                buffer[rxJSON_count] = ch;
                rxJSON_count++;
            }
        }

        if (rxJSON_MsgComplete)
        {
            rxJSON_MsgComplete = false;            
            buffer[rxJSON_count+1] = 0;
            rxJSON_count = 0;
            //ESP_LOGI(SER_TAG,"%s\r\n", buffer);
             
            rxJSON_StringReady = true;
        }
    }
    return (rxJSON_StringReady);

}

void rxJSON_ProcessMessage(char * inBuffer)
{
    static uint8_t msgJsonIndex = 0;
    RTOS_message_t serSendJsonMsg; 

    char tmpBuffer[200] = { 0 };   
    static char tmpStr[MESSAGE_MAX_ARRAY_BYTES] = {0};
    static char tmpVarsStr[SYS_STRING_MAX_ARRAY_CHARACTERS] = {0};

    uint8_t json_rx_src = 0;
    uint8_t json_rx_dst = 0;
    uint16_t json_rx_cmd = 0;
    uint8_t json_rx_type = 0;
    uint32_t json_rx_data = 0;
    uint16_t bufSize = 0;
    char * tmpPtr = NULL;

    bufSize = strlen(inBuffer);

    memcpy(tmpBuffer, inBuffer, bufSize);

    cJSON *root = cJSON_Parse((char *) tmpBuffer);

    ESP_LOGI(SER_TAG,"tmpBuffer:%s", tmpBuffer);
    

    if (cJSON_GetObjectItem(root, "src")) {
        json_rx_src = cJSON_GetObjectItem(root,"src")->valueint;
    }else{
        ESP_LOGE(SER_TAG,"json_rx_src not received!");
    }

    if (cJSON_GetObjectItem(root, "dst")) {
        json_rx_dst = cJSON_GetObjectItem(root,"dst")->valueint;
    }else{
        ESP_LOGE(SER_TAG,"json_rx_dst not received!");
    }


    if (cJSON_GetObjectItem(root, "cmd")) {
        json_rx_cmd = cJSON_GetObjectItem(root,"cmd")->valueint;
    }else{
        ESP_LOGE(SER_TAG,"json_rx_cmd not received!");
    }

    if (cJSON_GetObjectItem(root, "type")) {
        json_rx_type = cJSON_GetObjectItem(root,"type")->valueint;
    }else{
        ESP_LOGE(SER_TAG,"json_rx_type not received!");
    }

    if (cJSON_GetObjectItem(root, "data")) {
        json_rx_data = cJSON_GetObjectItem(root,"data")->valueint;
    }else{
        ESP_LOGE(SER_TAG,"json_rx_data not received!");
    }

    switch(json_rx_type)
    {
        case MSG_DATA_STR:
            memset(tmpStr,0,sizeof(tmpStr));
            tmpPtr = cJSON_GetObjectItem(root,"vars")->valuestring;
            memcpy(tmpVarsStr,tmpPtr, strlen(tmpPtr));
            serSendJsonMsg.msgDataLen = strlen(tmpPtr);
            serSendJsonMsg.msgDataPtr = (uint8_t *)tmpVarsStr;

            //ESP_LOGI(SER_TAG,"String received by serial module:%s", tmpStr);
            break;

        default:
            serSendJsonMsg.msgDataPtr = NULL;
            switch(json_rx_type)
            {
                case MSG_DATA_0:    serSendJsonMsg.msgDataLen =  MSG_DATA_0_LEN;    break;
                case MSG_DATA_8:    serSendJsonMsg.msgDataLen =  MSG_DATA_8_LEN;    break;
                case MSG_DATA_16:   serSendJsonMsg.msgDataLen =  MSG_DATA_16_LEN;   break;
                case MSG_DATA_32:   serSendJsonMsg.msgDataLen =  MSG_DATA_32_LEN;   break;
                default:            serSendJsonMsg.msgDataLen =  MSG_DATA_0_LEN;    break;
            }
            break;
    }

    serSendJsonMsg.srcAddr = json_rx_src;
    serSendJsonMsg.dstAddr = json_rx_dst;
    serSendJsonMsg.msgRef = msg_getMsgRef();
    serSendJsonMsg.msgTimeStamp = sys_getMsgTimeStamp();
    serSendJsonMsg.msgType = json_rx_type;
    serSendJsonMsg.msgCmd = json_rx_cmd;
    serSendJsonMsg.msgData = json_rx_data;

     cJSON_Delete(root);
    //send message
    xQueueSend(dispatcherQueueHandle,&serSendJsonMsg,0);

    //Set next array index
    msgJsonIndex++;
    if (msgJsonIndex > SER_MESSAGE_MAX_ARRAY_ELEMENTS)
    {
        msgJsonIndex = 0;
    }
}


void serSendMessage(uint8_t dstAddr, uint8_t msgType, uint16_t msgCmd, uint8_t * msgDataPtr, uint32_t msgData, uint32_t msgDataLen)
{  
    static uint8_t msgIndex = 0;
    RTOS_message_t serSendMsg;
    static char ser_Msg[SER_MESSAGE_MAX_ARRAY_ELEMENTS][SER_STRING_MAX_ARRAY_CHARACTERS];

    if ((msgType == MSG_DATA_0) || (msgType == MSG_DATA_8) || (msgType == MSG_DATA_16) || (msgType == MSG_DATA_32))
    {
       serSendMsg.msgDataPtr = NULL;
       switch(msgType)
       {
            case MSG_DATA_0:    serSendMsg.msgDataLen =  MSG_DATA_0_LEN;    break;
            case MSG_DATA_8:    serSendMsg.msgDataLen =  MSG_DATA_8_LEN;    break;
            case MSG_DATA_16:   serSendMsg.msgDataLen =  MSG_DATA_16_LEN;   break;
            case MSG_DATA_32:   serSendMsg.msgDataLen =  MSG_DATA_32_LEN;   break;
            default:            serSendMsg.msgDataLen =  MSG_DATA_0_LEN;    break;
       }
    } 
    else
    {
        memcpy(ser_Msg[msgIndex], msgDataPtr, msgDataLen);
        serSendMsg.msgDataPtr = (uint8_t *)ser_Msg[msgIndex];
        serSendMsg.msgDataLen = msgDataLen;
    }
    serSendMsg.srcAddr = MSG_ADDR_SER;
    serSendMsg.dstAddr = dstAddr;
    serSendMsg.msgRef = msg_getMsgRef();
    serSendMsg.msgTimeStamp = sys_getMsgTimeStamp();
    serSendMsg.msgType = msgType;
    serSendMsg.msgCmd = msgCmd;
    serSendMsg.msgData = msgData;
 
    //send message
    xQueueSend(dispatcherQueueHandle,&serSendMsg,0);

    //Set next array index
    msgIndex++;
    if (msgIndex >= SER_MESSAGE_MAX_ARRAY_ELEMENTS)
    {
        msgIndex = 0;
    }

}

void serProcessConsoleCommand(uint8_t srcAddr, char * command)
{
    char * tmpPtr = NULL;  
    uint32_t tmpInt32 = 0;
    uint16_t tmpInt16 = 0;
    uint8_t tmpInt8 = 0;
    float tmpFloat = 0.0;
    char consoleCmd[BLE_MAX_CON_PARM_LEN] = {0};
    char consoleParam[BLE_MAX_CON_PARMS - 1][BLE_MAX_CON_PARM_LEN] = {0};

    ESP_LOGI(SER_TAG, "serProcessConsoleCommand - Command: %s", command);

    cJSON *root = cJSON_Parse((char *) command);

    if (cJSON_GetObjectItem(root, "conc"))
    {
        memset(consoleCmd,0,sizeof(consoleCmd));
        tmpPtr = cJSON_GetObjectItem(root,"conc")->valuestring;
        memcpy(consoleCmd,tmpPtr, strlen(tmpPtr));
    }

    if (cJSON_GetObjectItem(root, "conp1"))
    {
        memset(consoleParam[0],0,sizeof(consoleParam[0]));
        tmpPtr = cJSON_GetObjectItem(root,"conp1")->valuestring;
        memcpy(consoleParam[0],tmpPtr, strlen(tmpPtr));
    }

    if (cJSON_GetObjectItem(root, "conp2"))
    {
        memset(consoleParam[1],0,sizeof(consoleParam[1]));
        tmpPtr = cJSON_GetObjectItem(root,"conp2")->valuestring;
        memcpy(consoleParam[1],tmpPtr, strlen(tmpPtr));
    }

    if (cJSON_GetObjectItem(root, "conp3"))
    {
        memset(consoleParam[2],0,sizeof(consoleParam[2]));
        tmpPtr = cJSON_GetObjectItem(root,"conp3")->valuestring;
        memcpy(consoleParam[2],tmpPtr, strlen(tmpPtr));
    }

    if (cJSON_GetObjectItem(root, "conp4"))
    {
        memset(consoleParam[3],0,sizeof(consoleParam[3]));
        tmpPtr = cJSON_GetObjectItem(root,"conp4")->valuestring;
        memcpy(consoleParam[3],tmpPtr, strlen(tmpPtr));
    }

    ESP_LOGI(SER_TAG, "Command:%s(%s,%s,%s,%s)", consoleCmd, consoleParam[0], consoleParam[1], consoleParam[2], consoleParam[3]);

    cJSON_Delete(root);
    
    if (strcmp("ver",consoleCmd) == 0){ serProcessVerCmd(srcAddr);                              return;}

    //if (strcmp("bv",consoleCmd) == 0){ serProcessBatteryVoltagCmd();            return;}

    if (strcmp("sn",consoleCmd) == 0)
    { 
        if (strlen(consoleParam[0]) > 0){
            ESP_LOGI(SER_TAG, "process set serial number");
            sprintf(serTmpStr,"%s",consoleParam[0]);
            serSendMessage(MSG_ADDR_STRG, MSG_DATA_STR, STRG_CMD_WR_SN, (uint8_t *)serTmpStr, 0, strlen(serTmpStr)+1);
        }else{
            ESP_LOGI(SER_TAG, "process get serial number"); 
            serSendMessage(MSG_ADDR_STRG, MSG_DATA_0, BLE_CMD_RD_SN, NULL, 0, MSG_DATA_0_LEN);
        }                                  
        return;
    }

    // if (strcmp("wc",consoleCmd) == 0){
    //     serProcessWifiCmd(WIFI_CONNECT);
    //     return;

    // }  

    // if (strcmp("wd",consoleCmd) == 0){
    //     serProcessWifiCmd(WIFI_DISCONNECT);
    //     return;
    // }    
    
    if (strcmp("rst",consoleCmd) == 0){
        serProcessResetCmd();
        return;
    }

    // if (strcmp("slp",consoleCmd) == 0){
    //     serProcessSleepCmd();
    //     return;
    // }

    if (strcmp("dc",consoleCmd) == 0)
    { 
        ESP_LOGI(SER_TAG, "process clear data queue");
        serSendMessage(MSG_ADDR_STRG, MSG_DATA_0, BLE_CMD_DQ_CLR_QUEUE, NULL, 0, MSG_DATA_0_LEN);
        return;
    }

    if (strcmp("lc",consoleCmd) == 0)
    { 
        ESP_LOGI(SER_TAG, "process clear log queue");
        serSendMessage(MSG_ADDR_STRG, MSG_DATA_0, BLE_CMD_LQ_CLR_QUEUE, NULL, 0, MSG_DATA_0_LEN);
        return;
    }

    if (strcmp("wrt",consoleCmd) == 0)
    { 
        if (strlen(consoleParam[0]) > 0){
            tmpInt8 = atoi(consoleParam[0]);
            serSendMessage(MSG_ADDR_STRG, MSG_DATA_8, STRG_CMD_WR_WIFI_RET, NULL, tmpInt8, MSG_DATA_8_LEN);
        }else{
            serSendMessage(MSG_ADDR_STRG, MSG_DATA_0, BLE_CMD_RD_WIFI_RET, NULL, MSG_DATA_COMMAND_ONLY, MSG_DATA_0_LEN);
        }
        return;
    }

    // if (strcmp("sdu",consoleCmd) == 0)
    // { 
    //     if (strlen(consoleParam[0]) > 0){
    //         tmpInt16 = atoi(consoleParam[0]);
    //         serSendMessage(MSG_ADDR_STRG, MSG_DATA_16, STRG_CMD_WR_SLP_DUR, NULL, tmpInt16, MSG_DATA_16_LEN);
    //     }else{
    //         serSendMessage(MSG_ADDR_STRG, MSG_DATA_0, BLE_CMD_RD_SLP_DUR, NULL, MSG_DATA_COMMAND_ONLY, MSG_DATA_0_LEN);
    //     }
    //     return;
    // }

    // if (strcmp("lbt",consoleCmd) == 0)
    // { 
    //     if (strlen(consoleParam[0]) > 0){
    //         tmpInt16 = atoi(consoleParam[0]);
    //         serSendMessage(MSG_ADDR_STRG, MSG_DATA_16, STRG_CMD_WR_LBAT_THR, NULL, tmpInt16, MSG_DATA_16_LEN);
    //     }else{
    //         serSendMessage(MSG_ADDR_STRG, MSG_DATA_0, BLE_CMD_RD_LBAT_THR, NULL, MSG_DATA_COMMAND_ONLY, MSG_DATA_0_LEN);
    //     }
    //     return;
    // }

    // if (strcmp("brt",consoleCmd) == 0)
    // { 
    //     if (strlen(consoleParam[0]) > 0){
    //         tmpInt16 = atoi(consoleParam[0]);
    //         serSendMessage(MSG_ADDR_STRG, MSG_DATA_16, STRG_CMD_WR_BRNO_THR, NULL, tmpInt16, MSG_DATA_16_LEN);
    //     }else{
    //         serSendMessage(MSG_ADDR_STRG, MSG_DATA_0, BLE_CMD_RD_BRNO_THR, NULL, MSG_DATA_COMMAND_ONLY, MSG_DATA_0_LEN);
    //     }
    //     return;
    // }

    // if (strcmp("dig",consoleCmd) == 0)
    // { 
    //     ESP_LOGI(SER_TAG, "process display system diagnostics");
    //     serSendMessage(MSG_ADDR_SCTL, MSG_DATA_0, BLE_CMD_DIAGNOSTICS, NULL, 0, MSG_DATA_0_LEN);
    //     return;
    // }
    serProcessMenuCmd();   
}

void serProcessMenuCmd(void)
{

    ESP_LOGI(SER_TAG, "process menu Command"); 
    serLogR("------------------- MENU -----------------------");
    serLogR("ver - Firmware version");
    serLogR("sn <number> - get/set serial number");
    serLogR("ssid <ssid> - get/set ssid");
    serLogR("pwd <pwd> - get/set pwd");
    serLogR("wc - connect wifi");
    serLogR("wd - disconnect wifi");
    serLogR("bv - battery voltage");
    serLogR("rst - Reset scale");
    serLogR("slp - Put scale to sleep");
    serLogR("dc <val> - Clear Data Queue");
    serLogR("lc <val> - Clear Log Queue");
    serLogR("wrt <val> - wifi retries");
    serLogR("dig - system diagnostics");   
    serLogR("----------------------------------------------------");

}


// void serProcessSleepCmd(void)
// {
//     ESP_LOGI(SER_TAG, "process sleep Command"); 
//     //serSendMessage(MSG_ADDR_SCTL, MSG_DATA_0, SCTL_CMD_BLE_SLEEP, NULL, 0, MSG_DATA_0_LEN);
//     serSendMessage(MSG_ADDR_SCTL, MSG_DATA_8, SCTL_CMD_TEST_SM, NULL, SM_TST_CONFIG_SLEEP, MSG_DATA_8_LEN);  
// }

void serProcessResetCmd(void)
{
    ESP_LOGI(SER_TAG, "process reset Command"); 
    esp_restart();  
}

void serProcessWifiCmd(uint8_t connect)
{
    switch(connect){
        case WIFI_CONNECT: 
            serSendMessage(MSG_ADDR_SCTL, MSG_DATA_0, EVT_CMD_WIFI_CONNECT, NULL, 0, MSG_DATA_0_LEN);
            break;
        case WIFI_DISCONNECT:    
            serSendMessage(MSG_ADDR_SCTL, MSG_DATA_0, EVT_CMD_WIFI_DISCONNECT, NULL, 0, MSG_DATA_0_LEN);
            break;
        case WIFI_SCAN:
            serSendMessage(MSG_ADDR_SCTL, MSG_DATA_0, EVT_CMD_WIFI_SCAN, NULL, 0, MSG_DATA_0_LEN);
        default:;    
    }
}


// void serProcessBatteryVoltagCmd(void)
// {

//     ESP_LOGI(SER_TAG, "process battery voltage Command"); 
//     serSendMessage(MSG_ADDR_BATTERY, MSG_DATA_0, BLE_BATTV_CON_CMD, NULL, 0, MSG_DATA_0_LEN);
// }


void serProcessVerCmd(uint8_t addr)
{
    char tmpStr[10] = {0};

    ESP_LOGI(SER_TAG, "process firmware ver Command"); 
    sprintf(tmpStr, "%d.%d.%d\r\n", FIRMWARE_VERSION_RELEASE, FIRMWARE_VERSION_MAJOR, FIRMWARE_VERSION_MINOR);
    serLogR(tmpStr);
}


/// @brief  This function sends the info log message to the BLE queue reguardless of 
///    the mobile device connected and/or the Log notification enabled. It will also send
///    the log message to the terminal.
/// @param strPtr 
void serLogR(char * strPtr)
{
    vTaskDelay(BLE_CON_DELAY / portTICK_PERIOD_MS);
    ESP_LOGI(SER_TAG, "%s", strPtr);
    serLog(strPtr, true, false);
}

/// @brief  This function sends the info log message to the BLE queue reguardless of 
///    the mobile device connected and/or the Log notification enabled. It will also send
///    the log message to the terminal.
/// @param strPtr 
void serLogIF(char * strPtr)
{
    ESP_LOGI(SER_TAG, "%s", strPtr);
    serLog(strPtr, true, true);
}

/// @brief  This function sends the error log message to the BLE queue reguardless of 
///    the mobile device connected and/or the Log notification enabled. It will also send
///    the log message to the terminal.
void serLogEF(char * strPtr)
{
    ESP_LOGE(SER_TAG, "%s", strPtr);
    serLog(strPtr, true, true);
}

/// @brief  This function send the info log message to the Mobile device and to the 
///    terminal.
/// @param strPtr 
void serLogI(char * strPtr)
{
    ESP_LOGI(SER_TAG, "%s", strPtr);
    serLog(strPtr, false, true);
}

/// @brief This function send the error log message to the Mobile device and to the 
///    terminal. 
/// @param strPtr 
void serLogE(char * strPtr)
{
    ESP_LOGE(SER_TAG, "%s", strPtr);
    serLog(strPtr, false, true);
}

/// @brief This function sends a message to the BLE module to send out a log message
///   to the mobile device via BLE. Mobile device has to be connected and the
///   log notifications (RX Characteristics) enabled.
/// @param strPtr - message to be sent to phone.
/// @param forced - If true, message is forced in the log queue.
/// @param printTag - If true the tag is printed with the message. 
void serLog(char * strPtr, bool forced, bool printTag)
{
    static uint8_t printIndex = 0;
    RTOS_message_t modPrintMsg;
    char tmpStr[STRING_MAX_LOG_CHARACTERS];
    static char modStr[SER_STRING_MAX_ARRAY_LOG_ELEMENTS][STRING_MAX_LOG_CHARACTERS];

    memcpy(tmpStr, strPtr, strlen(strPtr)+1);
	if (printTag){
        sprintf(modStr[printIndex], "%s:%s", SER_TAG, tmpStr);
    }else{
        sprintf(modStr[printIndex], "%s", tmpStr);
    }

    //form message to print out string
    modPrintMsg.srcAddr         =  MSG_ADDR_SER; 
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
    if (printIndex >= SER_STRING_MAX_ARRAY_LOG_ELEMENTS)
    {
        printIndex = 0;
    }
}

