#ifndef HEADER_FILES_TASK_SYSCONTROLTASK_H_
#define HEADER_FILES_TASK_SYSCONTROLTASK_H_

void sysSendMessage(uint8_t dstAddr, uint8_t msgType, uint16_t msgCmd, uint8_t * msgDataPtr, uint32_t msgData, uint32_t msgDataLen);
void sysSendDirMessage(uint8_t dstAddr, uint8_t msgType, uint16_t msgCmd, uint8_t * msgDataPtr, uint32_t msgData, uint32_t msgDataLen);

void sysControlTaskApp(void);
void sysStartSystem(void);
void sysSetSeqSystemIdle(void);

uint8_t sysControlCheckWakeupReason(void);

#endif /* HEADER_FILES_TASK_SYSCONTROLTASK_H_ */