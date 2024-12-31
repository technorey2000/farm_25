#ifndef SHARED_FILES_HEADER_FILES_MESSAGES_H_
#define SHARED_FILES_HEADER_FILES_MESSAGES_H_
#include "common.h"

void msg_form_message(uint8_t srcAddr, uint8_t dstAddr, uint8_t msgType, uint16_t msgCmd, uint8_t * msgDataPtr, uint32_t msgData, uint32_t msgDataLen, RTOS_message_t * msg);
void msg_assignMessageId(RTOS_message_t * msg);
void print_convertMsgToJSON(RTOS_message_t msg);
uint32_t msg_getMsgRef(void);
//uint32_t msg_getMsgTimeStamp(void);

#endif /* SHARED_FILES_HEADER_FILES_MESSAGES_H_ */