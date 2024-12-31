#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"

#include "Shared/common.h"
#include "Shared/messages.h"

//Task Includes:
#include "Task/dispatcherTask.h"

//Local functions
//void dsprPrint(char * msgPtr);

//Local parameters
RTOS_message_t dispatcherRxMessage;
RTOS_message_t dispatcherTxMessage;

uint8_t dsprStatus = 0;

static char disp_tag[]="DISP";

//char dsprTmpStr[DISP_STRING_MAX_ARRAY_CHARACTERS];

//External Functions:

/// @brief dispatecher task
/// @param  None
void dispatcherTaskApp(void)
{
	ESP_LOGI(disp_tag, "dispatcher is active");
    while(1)
    {
        if (xQueueReceive(dispatcherQueueHandle, &dispatcherRxMessage, portMAX_DELAY))
        {
			//ESP_LOGI(disp_tag, "dispatcher message received - command dstAddr:%d, msgCmd:%d", dispatcherRxMessage.dstAddr, dispatcherRxMessage.msgCmd);
			switch (dispatcherRxMessage.dstAddr)
			{
				case MSG_ADDR_SCTL:
					dispatcherTxMessage = dispatcherRxMessage;
					xQueueSend(sysControlQueueHandle,&dispatcherTxMessage,0);
					break;

				case MSG_ADDR_SUPR:
				case MSG_ADDR_SER:
					dispatcherTxMessage = dispatcherRxMessage;
					xQueueSend(serialQueueHandle,&dispatcherTxMessage,0);
					break;

				case MSG_ADDR_STRG:
					dispatcherTxMessage = dispatcherRxMessage;
					xQueueSend(strgQueueHandle,&dispatcherTxMessage,0);
					break;

				case MSG_ADDR_BLE:
					dispatcherTxMessage = dispatcherRxMessage;
					xQueueSend(bleQueueHandle,&dispatcherTxMessage,0);
					break;
			
				case MSG_ADDR_DSPR:
					switch(dispatcherRxMessage.msgCmd)
					{
						case DSPR_CMD_INIT:
							ESP_LOGI(disp_tag, "DSPR_CMD_INIT received");
							dsprStatus = DSPR_INIT_COMPLETE;
							msg_form_message(
                                MSG_ADDR_DSPR, 
                                MSG_ADDR_SCTL, 
                                MSG_DATA_8, 
                                DSPR_CMD_INIT, 
                                NULL, 
                                dsprStatus, 
                                sizeof(dsprStatus), 
                                &dispatcherTxMessage
								);
							xQueueSend(dispatcherQueueHandle,&dispatcherTxMessage,0);
							break;
						case DSPR_CMD_PING:
							msg_form_message(
								MSG_ADDR_DSPR,
								dispatcherRxMessage.srcAddr,
								MSG_DATA_8,
								DSPR_CMD_PING,
                                NULL,
                                DSPR_PING_RECEIVED,
								MSG_DATA_8_LEN,
								&dispatcherTxMessage
								);
							xQueueSend(dispatcherQueueHandle,&dispatcherTxMessage,0);
							break;
						case DSPR_CMD_STATUS:
							break;
						default:
							ESP_LOGE(disp_tag, "dispatcherTaskApp - default command received");
					}
					break;
				default: ;
			}

        }

    }
 
}

//Local Functions:
