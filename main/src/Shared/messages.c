#include <stdio.h>
#include <string.h>
#include "Shared/messages.h"
#include "freertos/task.h"
#include "esp_log.h"

static char msg_tag[]="MSG";

static uint32_t messageReference = 0;
//uint32_t timeStamp;

uint32_t msg_getMsgRef(void)
{
    return messageReference++;
}

/*
uint32_t msg_getMsgTimeStamp(void)
{
    return (uint32_t)xTaskGetTickCount;
}
*/

void msg_form_message(uint8_t srcAddr, uint8_t dstAddr, uint8_t msgType, uint16_t msgCmd, uint8_t * msgDataPtr, uint32_t msgData, uint32_t msgDataLen, RTOS_message_t * msg)
{  
    if ((msgType == MSG_DATA_0) || (msgType == MSG_DATA_8) || (msgType == MSG_DATA_16) || (msgType == MSG_DATA_32))
    {
       msg->msgDataPtr = NULL; 
    } 
    else
    {
        msg->msgDataPtr = msgDataPtr;  
    }
    msg->srcAddr = srcAddr;
    msg->dstAddr = dstAddr;
    msg->msgRef = messageReference++;
    msg->msgTimeStamp = (uint32_t)xTaskGetTickCount;
    msg->msgType = msgType;
    msg->msgCmd = msgCmd;
    msg->msgData = msgData;
    msg->msgDataLen = msgDataLen;
}

void msg_assignMessageId(RTOS_message_t * msg)
{
    msg->msgRef = messageReference++;
}

void print_convertMsgToJSON(RTOS_message_t msg)
{
    uint16_t print_strucID;
    char printMessage1[150] = {0};
    char printMessage2[50] = {0};
    char printMessage3[200] = {0};
    char printMessage4[20] = {0};
    char printMessage5[20] = {0};
    char printMessage6[50] = {0};
    char printMessage7[100] = {0};


    sprintf(printMessage1,"\r\nReceived Message: {\"src\":%d,\"dst\":%d,\"type\":%d,\"cmd\":%d,\"len\":%ld,\"id\":%ld,",
        msg.srcAddr,
        msg.dstAddr,
        msg.msgType,
        msg.msgCmd,
        msg.msgDataLen,
        msg.msgRef
        );

    switch (msg.msgType)
    {
        case MSG_DATA_8:
        case MSG_DATA_16:
        case MSG_DATA_32:
             sprintf(printMessage2,"\"data\":%ld,", msg.msgData);
            break;
        case MSG_DATA_FLOAT:
            sprintf(printMessage2,"\"data\":%.2f,", (float)msg.msgData);
            break;
        case MSG_DATA_STR:      
            memset(printMessage6, 0, sizeof(printMessage6));
            memcpy(printMessage6,(char *)msg.msgDataPtr, msg.msgDataLen);
            memset(printMessage7, 0, sizeof(printMessage7));
            sprintf(printMessage7,"\"vars\":\"%s\",", printMessage6);
            //sprintf(printMessage7,"\"%s\",", printMessage6);
            break;

        case MSG_DATA_STRUC:
            memcpy(&print_strucID,msg.msgDataPtr,2);
            sprintf(printMessage2,"\"struc_id\":%d,", print_strucID);
            break;
        default:
        	sprintf(printMessage4, "\"data\":0,");
            ESP_LOGI(msg_tag,"printMessage4:%s", printMessage4);
            break;
    }
    sprintf(printMessage5, "\"ts\":%lu}", sys_getMsgTimeStamp());
    printf("%s%s%s%s%s%s\r\n", printMessage1, printMessage2, printMessage3, printMessage4, printMessage7, printMessage5);
}
