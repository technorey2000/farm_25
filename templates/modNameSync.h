#ifndef HEADER_FILES_SYNC_modName_SYNC_H_
#define HEADER_FILES_SYNC_modName_SYNC_H_

#include "freertos/FreeRTOS.h"

void execmodNameControl(void);

bool modNameTryGrab(void);
bool modNameWaitAndGrab(TickType_t delay);
bool modNameRelease(void);

#endif /* HEADER_FILES_SYNC_modName_SYNC_H_ */
