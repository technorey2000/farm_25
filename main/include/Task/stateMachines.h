#ifndef HEADER_FILES_STATE_MACHINES_H_
#define HEADER_FILES_STATE_MACHINES_H_


bool sysProcessSmIdle(uint16_t sysCurrentEvt, uint16_t sysNextEvt);

void sysProcessSmSysInitStart(sys_init_complete_t * sysInitComplete);
bool sysProcessSmSysInit(sys_init_complete_t * sysInitComplete, sys_init_result_t * sysInitResult);

void sysProcessSmSysBeginStart(sys_begin_complete_t * beginComplete);
bool sysProcessSmSysBegin(sys_begin_complete_t * beginComplete, sys_begin_result_t * beginResult);

void sysProcessSmBleOnStart(sys_ble_complete_t * sys_ble_complete);
bool sysProcessSmBleOn(sys_ble_complete_t * sys_ble_complete, sys_ble_result_t * sys_ble_result);

#endif /* HEADER_FILES_STATE_MACHINES_H_ */