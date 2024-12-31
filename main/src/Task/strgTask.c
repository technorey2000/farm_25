#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"

#include "nvs_flash.h"
#include "esp_spiffs.h"

#include "esp_vfs.h"
#include <errno.h>
#include <sys/fcntl.h>
#include <unistd.h>
#include <ctype.h>
#include "sys/stat.h"

#include "Shared/common.h"
#include "Shared/messages.h"
#include "Shared/spiffs_circular_queue.h"

#include "Task/strgTask.h"

#include "esp_log.h"

// fnmatch defines
#define	FNM_NOMATCH     1       // Match failed.
#define	FNM_NOESCAPE	0x01	// Disable backslash escaping.
#define	FNM_PATHNAME	0x02	// Slash must be matched by slash.
#define	FNM_PERIOD		0x04	// Period must be matched by period.
#define	FNM_LEADING_DIR	0x08	// Ignore /<tail> after Imatch.
#define	FNM_CASEFOLD	0x10	// Case insensitive search.
#define FNM_PREFIX_DIRS	0x20	// Directory prefixes of pattern match too.
#define	EOS	            '\0'


//Local Parameters:
static RTC_NOINIT_ATTR uint32_t strgLastUploadTime;     //Storage in the RTC memory. This can't be initalized here.
static RTC_NOINIT_ATTR uint32_t strgLastUploadTimeSig;  //signature used to indicate if Storage in the RTC memory has been initalized.

RTOS_message_t strgRxMessage;
RTOS_message_t strgTxMessage;
UBaseType_t strgNewHighWaterMark = 0;
UBaseType_t strgCurrHighWaterMark = 0;
UBaseType_t strgInitHighWaterMark = 0;
uint8_t strgStatus = STRG_INIT_UNKNOWN;
uint8_t strgReturnStatus = STRG_RETURN_FAILURE;
char strgTmpStr[BLE_STRING_MAX_ARRAY_CHARACTERS];
char strgTmpLog[LOG_STRING_MAX_ARRAY_CHARACTERS];
uint8_t strgTmpStrLen = 0;
char strgTmpBleStr[BLE_STRING_MAX_ARRAY_CHARACTERS];
char strgTmpFileName[FILE_NAME_MAX_CHARACTERS];
//char strgKeyName[5];

nvs_data_t          g_nvs_setting_data;
nvs_ota_data_t      g_nvs_ota_setting_data;

static char *STRG_TAG = "STRG";


const nvs_key_data_t nvs_data_list[] =
{
    NVS_DATA_PAIR("0001", wifi)
  //, NVS_DATA_PAIR("0002", scale)
  //, NVS_DATA_PAIR("0003", battery)
  , NVS_DATA_PAIR("0002", config)
  //, NVS_DATA_PAIR("0004", https)
  //, NVS_DATA_PAIR("0005", device)
};

const nvs_ota_key_data_t nvs_ota_data_list[] =
{
    NVS_OTA_DATA_PAIR("0001", wifi_ota)
  //, NVS_OTA_DATA_PAIR("0002", scale_ota)
  , NVS_OTA_DATA_PAIR("0002", config_ota)
  //, NVS_OTA_DATA_PAIR("0004", https_ota)
};


nvs_handle m_nvs_handle;
nvs_handle m_nvs_ota_handle;

device_data_t strgDeviceData;

//Circular Buffer
circular_queue_t cq_data;   // data queue with variable elem size
circular_queue_t cq_log;    // log queue with variable elem size

//New SPIFFS code
esp_vfs_spiffs_conf_t conf = {
      .base_path = "/spiffs",
      .partition_label = NULL,
      .max_files = 5,
      .format_if_mount_failed = true
    };


//Local Function Prototypes:
void strgLog(char * strPtr, bool forced, bool printTag);
void strgLogR(char * strPtr);
void strgLogIF(char * strPtr);
void strgLogEF(char * strPtr);
void strgLogI(char * strPtr);
void strgLogE(char * strPtr);
//void strgPrint(char * msgPtr);
void strgSendMessage(uint8_t dstAddr, uint8_t msgType, uint16_t msgCmd, uint8_t * msgDataPtr, uint32_t msgData, uint32_t msgDataLen);


void sys_nvs_reset_data(void);
void sys_nvs_init(void);
void sys_nvs_deinit(void);
void sys_nvs_store_all(void);
void sys_nvs_load_all(void);
void sys_nvs_factory_reset(void);

//void sys_nvs_ota_reset_data(void);
void sys_nvs_ota_init(void);
void sys_nvs_ota_deinit(void);
void sys_nvs_ota_store_all(void);
void sys_nvs_ota_load_all(void);
void sys_nvs_ota_factory_reset(void);
void strgClearOtaData(void);


//void strgSetConfigDefaults(void);
//void strgSetScaleDefaults(void);
void strgSetConfigDefaults(void);
//void strgSetDeviceDefaults(void);
//void strgSetHttpsDefaults(void);
void strgSetWifiDefaults(void);

uint8_t strgQueueInit(uint8_t queueId);
uint8_t strgQueueClr(uint8_t queueId);
//void strgMountSpiffs(void);

uint32_t strgCirQueueSize(uint8_t queueId);

uint32_t strgQueueAvailSpace(uint8_t queueId);
uint16_t strgQueueCount(uint8_t queueId);
uint32_t strgQueueFsize(uint8_t queueId);
uint8_t strgQueueEmpty(uint8_t queueId);
bool strgWriteQueue(uint8_t queueId, char * queueFileName, char * queueData);
uint8_t strgReadQueue(uint8_t queueId, char * queueData, uint16_t * queueDataLen);
uint8_t strgQueueFree(uint8_t queueId);
uint8_t strgQueuePercent(uint8_t queueId);
void strgQueueInfo(uint8_t queueId);

//New SPIFFS code
bool strg2MountSpiffs(void);
bool strg2DismountSpiffs(void);
static void list(char *path, char *match);
static bool strgNextFile(char *path, char *fileName, uint8_t fileNameLen); 
uint8_t strgFileCount(char *path);

//OTA commands
void strgDisplayOtaData(void);
void strgOtaToNvs(void);
void strgNvsToOta(void);

//Encryption
const char *key = "vJKfkSLShUDJfyBoExTO6PzeeWryODkX";
const char *testSsid = "NETGEAR70";

const char *testPwd = "freeearth497";

void encryptString(const char *toEncrypt, const char *key, unsigned char *result);
void decryptString(const unsigned char *toDecrypt, const char *key, unsigned char *result);

//void strgNvsLoad(nvs_data_t setting_data, uint8_t indx);
//void strgNvsStore(nvs_data_t setting_data, uint8_t indx);

//External Functions:

/// @brief strg task
/// @param  None
void strgTaskApp(void)
{
    //uint32_t addr;
    //void *p_data;
    //size_t var_len;
    uint8_t tmpVal8 = 0;
    uint32_t tmpVal32 = 0;
    //uint16_t strgQueueInx = 1;
    struct timeval tv_now;
    int64_t time_us = 0;
    //uint32_t tot=0, used=0;
    size_t tot=0, used=0;
    scale_config_data_t scaleConfigData;

    while(1)
    {
        if (xQueueReceive(strgQueueHandle, &strgRxMessage, portMAX_DELAY))
        {
			switch(strgRxMessage.msgCmd)
			{
				case STRG_CMD_INIT:
                    ESP_LOGI(STRG_TAG, "STRG_CMD_INIT received");
					//bsp_spiffs_init();
					sys_nvs_init();

                  
                    // sys_nvs_ota_init();

                    // if(otaSignature == OTA_SIG_PASS)
                    // {
                    //     ESP_LOGI(STRG_TAG,"STRG_CMD_INIT- copy OTA config to NVS config");
                    //     strgOtaToNvs();
                    //     sys_nvs_store_all();
                    //     otaSignature = OTA_SIG_DONE;
                    // }


                    strgStatus = STRG_INIT_COMPLETE;
                    strgSendMessage(strgRxMessage.srcAddr, MSG_DATA_8, strgRxMessage.msgCmd, NULL, strgStatus, MSG_DATA_8_LEN);
                    break;
				case STRG_CMD_PING:
                    strgSendMessage(strgRxMessage.srcAddr, MSG_DATA_8, strgRxMessage.msgCmd, NULL, STRG_PING_RECEIVED, MSG_DATA_8_LEN);
					break;
				case STRG_CMD_STATUS:
                    strgStatus = STRG_INIT_COMPLETE;
                    strgSendMessage(strgRxMessage.srcAddr, MSG_DATA_8, strgRxMessage.msgCmd, NULL, strgStatus, MSG_DATA_8_LEN);
					//TBD
					break;
                case STRG_CMD_RST_NVS:
                    sys_nvs_init();
                    strgReturnStatus = STRG_RETURN_GOOD;
                    strgSendMessage(strgRxMessage.srcAddr, MSG_DATA_8, strgRxMessage.msgCmd, NULL, strgReturnStatus, MSG_DATA_8_LEN);                    
                    break;    

                case STRG_CMD_RD_LAST_UL_TIME:
                    if (strgLastUploadTimeSig != UPLOAD_TIME_SIG_SET)
                    {
                        strgLastUploadTimeSig = UPLOAD_TIME_SIG_SET;
                        strgLastUploadTime = 0;
                    }
                    strgSendMessage(strgRxMessage.srcAddr, MSG_DATA_32, strgRxMessage.msgCmd, NULL, strgLastUploadTime, MSG_DATA_32_LEN);
                    ESP_LOGI(STRG_TAG, "Last upload time to cloud read:%ld", strgLastUploadTime);
					break;

                case STRG_CMD_WR_LAST_UL_TIME:
                    strgLastUploadTime = strgRxMessage.msgData;
                    ESP_LOGI(STRG_TAG, "Last upload time to cloud written:%ld", strgLastUploadTime);
                    strgReturnStatus = STRG_RETURN_GOOD;
                    strgSendMessage(strgRxMessage.srcAddr, MSG_DATA_8, strgRxMessage.msgCmd, NULL, strgReturnStatus, MSG_DATA_8_LEN);                    
					break;

                case STRG_CMD2_MOUNT_SPIFFS:
                    if (strg2MountSpiffs()){
                        strgReturnStatus = STRG_RETURN_GOOD;
                    }else{
                        strgReturnStatus = STRG_RETURN_FAILURE;
                    }                   
                    strgSendMessage(strgRxMessage.srcAddr, MSG_DATA_8, strgRxMessage.msgCmd, NULL, strgReturnStatus, MSG_DATA_8_LEN);                    
                    break;  

                case STRG_CMD2_DISMOUNT_SPIFFS:
                    if (strg2DismountSpiffs()){
                        strgReturnStatus = STRG_RETURN_GOOD;
                    }else{
                        strgReturnStatus = STRG_RETURN_FAILURE;
                    }                   
                    strgSendMessage(strgRxMessage.srcAddr, MSG_DATA_8, strgRxMessage.msgCmd, NULL, strgReturnStatus, MSG_DATA_8_LEN);                    
                    break;  


                case STRG_CMD2_WR_TEST:
                    ESP_LOGI(STRG_TAG, "Opening file");
                    FILE* f = fopen("/spiffs/hello.txt", "w");
                    if (f == NULL) {
                        ESP_LOGE(STRG_TAG, "Failed to open file for writing");
                        return;
                    }
                    fprintf(f, "Hello World!\n");
                    fclose(f);
                    ESP_LOGI(STRG_TAG, "File written");
                    break;  

                case STRG_CMD2_WR2_TEST:
                    ESP_LOGI(STRG_TAG, "Opening file");
                    FILE* f2 = fopen("/spiffs/new_data/hello2.txt", "w");
                    if (f2 == NULL) {
                        ESP_LOGE(STRG_TAG, "Failed to open file for writing");
                        return;
                    }
                    fprintf(f2, "Hello World 2!\n");
                    fclose(f2);
                    ESP_LOGI(STRG_TAG, "File 2 written");
                    break;  

                case STRG_CMD2_WR3_TEST:
                    ESP_LOGI(STRG_TAG, "Opening file");
                    FILE* f3 = fopen("/spiffs/hello3.txt", "w");
                    if (f3 == NULL) {
                        ESP_LOGE(STRG_TAG, "Failed to open file for writing");
                        return;
                    }
                    fprintf(f3, "Hello World 3!\n");
                    fclose(f3);
                    ESP_LOGI(STRG_TAG, "File written");
                    break;  

                case STRG_CMD2_RD_TEST:
                    ESP_LOGI(STRG_TAG, "Reading file");
                    f = fopen("/spiffs/hello.txt", "r");
                    if (f == NULL) {
                        ESP_LOGE(STRG_TAG, "Failed to open file for reading");
                        return;
                    }
                    char line[64];
                    fgets(line, sizeof(line), f);
                    fclose(f);
                    // strip newline
                    char* pos = strchr(line, '\n');
                    if (pos) {
                        *pos = '\0';
                    }
                    ESP_LOGI(STRG_TAG, "Read from file: '%s'", line);
                    break;  

                case STRG_CMD2_DIR_LIST:
                    list("/spiffs", NULL);
                    break;



                case STRG_CMD2_DELETE_FILE:
                    if (unlink("/spiffs/hello.txt")){
                        ESP_LOGI(STRG_TAG,"File removed - Failed");
                    }else{
                        ESP_LOGE(STRG_TAG,"File removed");
                    }
                    break;

                case STRG_CMD2_DIR2_LIST:
                    list(NEW_DATA_QUEUE_DIR, NULL);
                    break;

                case STRG_CMD2_NEXT_FILE:
                    bool strgFilesInDir = false;
                    char strgNextFileName[FILE_NAME_MAX_CHARACTERS];

                     memset(strgNextFileName,0,sizeof(strgNextFileName));

                    strgFilesInDir = strgNextFile("/spiffs", &strgNextFileName[0], sizeof(strgNextFileName));
                    if( strgFilesInDir)
                    {
                        ESP_LOGI(STRG_TAG,"Next file name:%s",strgNextFileName);
                    }
                    else{
                        ESP_LOGI(STRG_TAG,"No files found!");
                    }
                    break;

                case STRG_CMD2_FILE_COUNT:
                    ESP_LOGI(STRG_TAG,"File count:%d",strgFileCount("/spiffs"));
                    break;

                case STRG_CMD2_CREATE_DIR:
                    {
                        struct stat sb;
                        FILE *fd = NULL;

                        // stat returns 0 upon succes (file exists) and -1 on failure (does not)
                        if (stat("/spiffs/new_data", &sb) < 0) {
                            if ((fd = fopen("/spiffs/new_data", "w"))) {
                                ESP_LOGI(STRG_TAG, "Directory /spiffs/new_data was created");
                            }else{
                                ESP_LOGE(STRG_TAG, "Directory /spiffs/new_data was not created!");
                            }  
                        } 
                        else
                        {
	                        if ((fd = fopen("/spiffs/new_data", "r+b"))) {
                                ESP_LOGI(STRG_TAG, "Directory /spiffs/new_data was opened");
                            }else{
                                ESP_LOGE(STRG_TAG, "Directory /spiffs/new_data was not opened!");
                            }  
                        }    
                        fclose(fd);
                    }
                    break;

////////////////////////////////////////////////////////////////////////////
// Fixed SPIFFS commands
////////////////////////////////////////////////////////////////////////////

                case STRG_CMD_MOUNT_SPIFFS:
                    if (strg2MountSpiffs()){
                        strgReturnStatus = STRG_RETURN_GOOD;
                    }else{
                        strgReturnStatus = STRG_RETURN_FAILURE;
                    }                   
                    strgSendMessage(strgRxMessage.srcAddr, MSG_DATA_8, strgRxMessage.msgCmd, NULL, strgReturnStatus, MSG_DATA_8_LEN);                    
                    break;  

                //Data Queues
                case STRG_CMD_DATA_QUEUE_INIT:
                    strgReturnStatus = STRG_RETURN_FAILURE;
                    if (strgQueueInit(STRG_DATA_QUEUE) == 1){
                        strgReturnStatus = STRG_RETURN_GOOD;
                    }
                    strgSendMessage(strgRxMessage.srcAddr, MSG_DATA_8, strgRxMessage.msgCmd, NULL, strgReturnStatus, MSG_DATA_8_LEN);                    
                    break;  

                case STRG_CMD_LOG_QUEUE_INIT:
                    strgReturnStatus = STRG_RETURN_FAILURE;
                    if (strgQueueInit(STRG_LOG_QUEUE) == 1){
                        strgReturnStatus = STRG_RETURN_GOOD;
                    }
                    strgSendMessage(strgRxMessage.srcAddr, MSG_DATA_8, strgRxMessage.msgCmd, NULL, strgReturnStatus, MSG_DATA_8_LEN);                    
                    break;  


                case STRG_CMD_DATA_QUEUE_COUNT:
                    ESP_LOGI(STRG_TAG,"File count:%d",strgFileCount(DATA_QUEUE_DIR));
                    strgSendMessage(strgRxMessage.srcAddr, MSG_DATA_16, strgRxMessage.msgCmd, NULL, strgFileCount(DATA_QUEUE_DIR), MSG_DATA_16_LEN);
                    break; 

                case STRG_CMD_LOG_QUEUE_COUNT:
                    ESP_LOGI(STRG_TAG,"File count:%d",strgFileCount(LOG_QUEUE_DIR));
                    strgSendMessage(strgRxMessage.srcAddr, MSG_DATA_16, strgRxMessage.msgCmd, NULL, strgFileCount(LOG_QUEUE_DIR), MSG_DATA_16_LEN);
                    break; 

                case STRG_CMD_DQ_REWRITE_DATA:
                case STRG_CMD_DQ_WRITE_DATA:
                    gettimeofday(&tv_now, NULL);
                    time_us = ((int64_t)tv_now.tv_sec * 1000000L)/1000;
                    memset(strgTmpFileName,0,sizeof(strgTmpFileName));
                    sprintf(strgTmpFileName,"Data-%lld",time_us);

                    //Test code:
                    ESP_LOGI(STRG_TAG,"Data file name:%s", strgTmpFileName);
                    //memset(strgTmpStr,0,sizeof(strgTmpFileName));
                    //sprintf(strgTmpStr,"{lox data1234567890}");
                    //if (strgWriteQueue(STRG_DATA_QUEUE, strgTmpFileName, strgTmpStr))
                    //----------------------------------------------------------

                    if (strgWriteQueue(STRG_DATA_QUEUE, strgTmpFileName, (char *)strgRxMessage.msgDataPtr))
                    {
                        strgReturnStatus = STRG_RETURN_GOOD;
                    }else{
                        strgReturnStatus = STRG_RETURN_FAILURE;
                    }

                    if( strgRxMessage.msgCmd == STRG_CMD_DQ_WRITE_DATA){
                        strgSendMessage(strgRxMessage.srcAddr, MSG_DATA_8, STRG_CMD_DQ_WRITE_DATA, NULL, strgReturnStatus, MSG_DATA_8_LEN); 
                    }else{
                        strgSendMessage(strgRxMessage.srcAddr, MSG_DATA_8, STRG_CMD_DQ_REWRITE_DATA, NULL, strgReturnStatus, MSG_DATA_8_LEN);
                    }                   
                    break;  


                case STRG_CMD_LQ_REWRITE_DATA:
                case STRG_CMD_LQ_WRITE_DATA:
                    gettimeofday(&tv_now, NULL);
                    time_us = ((int64_t)tv_now.tv_sec * 1000000L)/1000;
                    memset(strgTmpFileName,0,sizeof(strgTmpFileName));
                    sprintf(strgTmpFileName,"Log-%lld",time_us);

                    memset(strgTmpLog,0,sizeof(strgTmpLog));

                    if (strgRxMessage.msgCmd == STRG_CMD_LQ_WRITE_DATA)
                    {
                        sprintf(strgTmpLog,"\"%s,ts:%lld\"",(char *)strgRxMessage.msgDataPtr, time_us);
                    }
                    else
                    {
                        memcpy(strgTmpLog,(char *)strgRxMessage.msgDataPtr, strgRxMessage.msgDataLen);
                    }

                    if (strgWriteQueue(STRG_LOG_QUEUE, strgTmpFileName, strgTmpLog))
                    {
                        strgReturnStatus = STRG_RETURN_GOOD;
                    }else{
                        strgReturnStatus = STRG_RETURN_FAILURE;
                    }

                    if( strgRxMessage.msgCmd == STRG_CMD_LQ_WRITE_DATA){
                        strgSendMessage(strgRxMessage.srcAddr, MSG_DATA_8, STRG_CMD_LQ_WRITE_DATA, NULL, strgReturnStatus, MSG_DATA_8_LEN); 
                    }else{
                        strgSendMessage(strgRxMessage.srcAddr, MSG_DATA_8, STRG_CMD_LQ_REWRITE_DATA, NULL, strgReturnStatus, MSG_DATA_8_LEN);
                    }                   
                    break;  

                case BLE_CMD_DQ_READ_DATA:
                case STRG_CMD_DQ_READ_DATA:
                    memset(&strgTmpStr,0,sizeof(strgTmpStr));
                    tmpVal8 = strgReadQueue(STRG_DATA_QUEUE, (char *)&strgTmpStr, (uint16_t *)&strgTmpStrLen);                                 
                    if ( strgRxMessage.msgCmd == BLE_CMD_DQ_READ_DATA){
                        if (strlen(strgTmpStr) == 0){
                            strgLogR("Data Queue Empty!");
                        }else{
                            strgLogR(strgTmpStr);                            
                        }
                    }else{
                       if(tmpVal8 == 1)
                        {
                            strgSendMessage(strgRxMessage.srcAddr, MSG_DATA_STR, strgRxMessage.msgCmd, (uint8_t *)&strgTmpStr, STRG_RETURN_GOOD, strgTmpStrLen + 1);
                        }
                        else
                        {
                            strgSendMessage(strgRxMessage.srcAddr, MSG_DATA_STR, strgRxMessage.msgCmd, (uint8_t *)&strgTmpStr, STRG_RETURN_FAILURE, strgTmpStrLen + 1); 
                        }
                    }
                    break;   

                case BLE_CMD_LQ_READ_DATA:
                case STRG_CMD_LQ_READ_DATA:
                    memset(&strgTmpStr,0,sizeof(strgTmpStr));
                    tmpVal8 = strgReadQueue(STRG_LOG_QUEUE, (char *)&strgTmpStr, (uint16_t *)&strgTmpStrLen);                                      
                    if ( strgRxMessage.msgCmd == BLE_CMD_LQ_READ_DATA){
                        if (strlen(strgTmpStr) == 0){
                            strgLogR("Log Queue Empty!");
                        }else{
                            strgLogR(strgTmpStr);                            
                        }
                    }else{
                        if(tmpVal8 == 1)
                        {
                            strgSendMessage(strgRxMessage.srcAddr, MSG_DATA_STR, strgRxMessage.msgCmd, (uint8_t *)&strgTmpStr, STRG_RETURN_GOOD, strgTmpStrLen + 1);
                        }
                        else
                        {
                            strgSendMessage(strgRxMessage.srcAddr, MSG_DATA_STR, strgRxMessage.msgCmd, (uint8_t *)&strgTmpStr, STRG_RETURN_FAILURE, strgTmpStrLen + 1); 
                        }
                    }
                    break;  

                case BLE_CMD_DQ_CLR_QUEUE:
                case STRG_CMD_DQ_CLR_QUEUE:
                    strgQueueClr(STRG_DATA_QUEUE);
                    if ( strgRxMessage.msgCmd == BLE_CMD_DQ_CLR_QUEUE){
                        strgLogR("Data Queue Cleared");
                    }
                    break; 

                case BLE_CMD_LQ_CLR_QUEUE:
                case STRG_CMD_LQ_CLR_QUEUE:
                    strgQueueClr(STRG_LOG_QUEUE);
                    if ( strgRxMessage.msgCmd == BLE_CMD_LQ_CLR_QUEUE){
                        strgLogR("Log Queue Cleared");
                    }
                    break; 

// New commands:
                case STRG_CMD_DATA_DIR_LIST:
                    list(DATA_QUEUE_DIR, NULL);
                    break;

                case STRG_CMD_LOG_DIR_LIST:
                    list(LOG_QUEUE_DIR, NULL);
                    break;

                case STRG_CMD_SPIFFS_USAGE:
                	//uint32_t tot=0, used=0;
                    esp_spiffs_info(NULL, &tot, &used);
                    ESP_LOGI(STRG_TAG,"SPIFFS: free %d Bytes of %d Bytes", (tot-used), tot);
                    break;


////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////




                case STRG_CMD_DATA_QUEUE_SIZE:
                    ESP_LOGI(STRG_TAG, "Data Queue Size: %ld", strgCirQueueSize(STRG_DATA_QUEUE));
                    break;    
                case STRG_CMD_LOG_QUEUE_SIZE:
                    ESP_LOGI(STRG_TAG, "Log Queue Size: %ld", strgCirQueueSize(STRG_LOG_QUEUE));
                    break;    

                case STRG_CMD_DATA_QUEUE_SPACE:
                    ESP_LOGI(STRG_TAG, "Data Queue Available space: %ld", strgQueueAvailSpace(STRG_DATA_QUEUE));
                    break;  
                case STRG_CMD_LOG_QUEUE_SPACE:
                    ESP_LOGI(STRG_TAG, "Log Queue Available space: %ld", strgQueueAvailSpace(STRG_LOG_QUEUE));
                    break;  



                case STRG_CMD_DATA_QUEUE_FSIZE:
                    ESP_LOGI(STRG_TAG, "Data Queue File Size: %ld", strgQueueFsize(STRG_DATA_QUEUE));
                    break; 
                case STRG_CMD_LOG_QUEUE_FSIZE:
                    ESP_LOGI(STRG_TAG, "Log Queue File Size: %ld", strgQueueFsize(STRG_LOG_QUEUE));
                    break; 


                case STRG_CMD_DATA_QUEUE_PERCENT:
                    ESP_LOGI(STRG_TAG, "Data Queue percent available: %d", strgQueuePercent(STRG_DATA_QUEUE));
                    break; 
                case STRG_CMD_LOG_QUEUE_PERCENT:
                    ESP_LOGI(STRG_TAG, "Log Queue percent available: %d", strgQueuePercent(STRG_LOG_QUEUE));
                    break; 

                case STRG_CMD_DATA_QUEUE_INFO:
                    strgQueueInfo(STRG_DATA_QUEUE);
                    break;  

                case STRG_CMD_LOG_QUEUE_INFO:
                    strgQueueInfo(STRG_LOG_QUEUE);
                    break;       

                case STRG_CMD_DQ_WRITE_TEST:
                    gettimeofday(&tv_now, NULL);
                    time_us = ((int64_t)tv_now.tv_sec * 1000000L)/1000;
                    memset(strgTmpFileName,0,sizeof(strgTmpFileName));
                    sprintf(strgTmpFileName,"Data-%lld",time_us);

                    ESP_LOGI(STRG_TAG,"Data file name:%s", strgTmpFileName);

                    memset(strgTmpStr,0,sizeof(strgTmpStr));
                    sprintf(strgTmpStr,"This is a test of bad scale data");

                    if (strgWriteQueue(STRG_DATA_QUEUE, strgTmpFileName, strgTmpStr))
                    {
                        strgReturnStatus = STRG_RETURN_GOOD;
                    }else{
                        strgReturnStatus = STRG_RETURN_FAILURE;
                    }
                    break;

                case STRG_CMD_DQ_READ_TEST:
                    while (strgQueueEmpty(STRG_DATA_QUEUE) == 0)
                    {
                        strgReadQueue(STRG_DATA_QUEUE, (char *)&strgTmpStr, (uint16_t *)&strgTmpStrLen);
                    }
                    break; 

                case STRG_CMD_LQ_WRITE_TEST:
                    gettimeofday(&tv_now, NULL);
                    time_us = ((int64_t)tv_now.tv_sec * 1000000L)/1000;
                    memset(strgTmpFileName,0,sizeof(strgTmpFileName));
                    sprintf(strgTmpFileName,"Data-%lld",time_us);

                    ESP_LOGI(STRG_TAG,"Data file name:%s", strgTmpFileName);

                    memset(strgTmpStr,0,sizeof(strgTmpStr));
                    sprintf(strgTmpStr,"This is a test of a bad log");

                    if (strgWriteQueue(STRG_LOG_QUEUE, strgTmpFileName, strgTmpStr))
                    {
                        strgReturnStatus = STRG_RETURN_GOOD;
                    }else{
                        strgReturnStatus = STRG_RETURN_FAILURE;
                    }                        
                    break;

                case STRG_CMD_LQ_READ_TEST:
                    while (strgQueueEmpty(STRG_LOG_QUEUE) == 0)
                    {
                        strgReadQueue(STRG_LOG_QUEUE, (char *)&strgTmpStr, (uint16_t *)&strgTmpStrLen);
                    }
                    break;  

                //Configurations
                case STRG_CMD_NVS_STORE_ALL:
                    sys_nvs_store_all();
                    strgReturnStatus = STRG_RETURN_GOOD;
                    strgSendMessage(strgRxMessage.srcAddr, MSG_DATA_8, strgRxMessage.msgCmd, NULL, strgReturnStatus, MSG_DATA_8_LEN);                    
                    break;  

                case STRG_CMD_SET_CFG_DEFAULTS:
                    sys_nvs_load_all();	
                    strgSetConfigDefaults();
                    sys_nvs_store_all();
                    ESP_LOGI(STRG_TAG, "Config defaults set");
                    break;

                // case STRG_CMD_SET_SCALE_DEFAULTS:
                //     strgSetScaleDefaults();
                //     sys_nvs_store_all();
                //     ESP_LOGI(STRG_TAG, "Scale defaults set");
                //     break;

                case STRG_CMD_SET_WIFI_DEFAULTS:
                    strgSetWifiDefaults();
                    sys_nvs_store_all();
                    ESP_LOGI(STRG_TAG, "Wifi defaults set");
                    break;

                // case STRG_CMD_SET_HTTPS_DEFAULTS:
                //     strgSetHttpsDefaults();
                //     sys_nvs_store_all();
                //     ESP_LOGI(STRG_TAG, "HTTPS defaults set");
                //     break;
/*
                case STRG_CMD_SET_DEVICE_DEFAULTS:
                    strgSetDeviceDefaults();
                    sys_nvs_store_all();
                    ESP_LOGI(STRG_TAG, "DEVICE defaults set");
                    break;
*/

                //HTTPS data
				// case STRG_CMD_RD_HTTPS_SIG:    //https Signature
                //     sys_nvs_load_all();			
                //     strgSendMessage(strgRxMessage.srcAddr, MSG_DATA_32, strgRxMessage.msgCmd, NULL, g_nvs_setting_data.https.sig, MSG_DATA_32_LEN);
                //     ESP_LOGI(STRG_TAG, "Https sig read:%lX", g_nvs_setting_data.https.sig);
				// 	break;

				// case STRG_CMD_WR_HTTPS_SIG:    //https signature
                //     sys_nvs_load_all();
                //     g_nvs_setting_data.https.sig = strgRxMessage.msgData;
                //     sys_nvs_store_all();	
                //     ESP_LOGI(STRG_TAG, "Https sig written:%lX", g_nvs_setting_data.https.sig);
				// 	break;	

				// case STRG_CMD_WR_HTTPS_DEFAULT_SIG:    //https default signature
                //     sys_nvs_load_all();
                //     g_nvs_setting_data.https.sig = WIFI_SIG_DEFAULT;
                //     sys_nvs_store_all();	
                //     ESP_LOGI(STRG_TAG, "Https default sig written:%lX", g_nvs_setting_data.https.sig);
				// 	break;	

				// case STRG_CMD_WR_HTTPS_SET_SIG:    //https set signature
                //     sys_nvs_load_all();
                //     g_nvs_setting_data.https.sig = HTTPS_SIG_SET;
                //     sys_nvs_store_all();	
                //     ESP_LOGI(STRG_TAG, "Https set sig written:%lX", g_nvs_setting_data.https.sig);
				// 	break;	

				// case STRG_CMD_WR_HTTPS_LI_SIG:    //https signature - Lox ID set
                //     sys_nvs_load_all();
                //     tmpVal32 = g_nvs_setting_data.https.sig & 0xFFFF0000;
                //     tmpVal32 = tmpVal32 | HTTPS_SIG_LI_SET;
                //     g_nvs_setting_data.https.sig = tmpVal32;
                //     sys_nvs_store_all();	
                //     ESP_LOGI(STRG_TAG, "Https sig - Lox Id set written:%lX", g_nvs_setting_data.https.sig);
				// 	break;	

				// case STRG_CMD_WR_HTTPS_DT_SIG:    //https signature - Device Token set
                //     sys_nvs_load_all();
                //     tmpVal32 = g_nvs_setting_data.https.sig & 0x0000FFFF;
                //     tmpVal32 = tmpVal32 | HTTPS_SIG_DT_SET;
                //     g_nvs_setting_data.https.sig = tmpVal32;
                //     sys_nvs_store_all();	
                //     ESP_LOGI(STRG_TAG, "Https sig - device token written:%lX", g_nvs_setting_data.https.sig);
				// 	break;	

				// case STRG_CMD_WR_DEVICE_TOKEN:  
                //     ESP_LOGI(STRG_TAG, "Device token being stored:%s\nlength:%ld", (char *)strgRxMessage.msgDataPtr, strgRxMessage.msgDataLen);
                //     sys_nvs_load_all();
                //     memset(g_nvs_setting_data.https.deviceToken,0,sizeof(g_nvs_setting_data.https.deviceToken));
				// 	memcpy(g_nvs_setting_data.https.deviceToken, strgRxMessage.msgDataPtr, strgRxMessage.msgDataLen);	
                //     sys_nvs_store_all();	
				// 	break;

				// case STRG_CMD_WR_LOX_ID:  
                //     ESP_LOGI(STRG_TAG, "Lox Id being stored:%s\nlength:%ld", (char *)strgRxMessage.msgDataPtr, strgRxMessage.msgDataLen);
                //     sys_nvs_load_all();
                //     memset(g_nvs_setting_data.https.loxId,0,sizeof(g_nvs_setting_data.https.loxId));
				// 	memcpy(g_nvs_setting_data.https.loxId, strgRxMessage.msgDataPtr, strgRxMessage.msgDataLen);	
                //     sys_nvs_store_all();	
				// 	break;

				// case STRG_CMD_WR_LOX_ID_TEST: 
                //     memset(strgTmpStr,0,sizeof(strgTmpStr));
                //     sprintf(strgTmpStr,"%s","84023ffa-98ef-4fd1-b081-ed1bbe155640"); //LoxId for JIM061623 
                //     ESP_LOGI(STRG_TAG, "Lox Id being stored:%s", strgTmpStr);
                //     sys_nvs_load_all();
                //     memset(g_nvs_setting_data.https.loxId,0,sizeof(g_nvs_setting_data.https.loxId));
				// 	memcpy(g_nvs_setting_data.https.loxId, strgTmpStr, strlen(strgTmpStr));	
                //     sys_nvs_store_all();	
				// 	break;

                //Device
				case STRG_CMD_RD_SN_SIG:    //SN Signature
                    sys_nvs_load_all();			
                    strgSendMessage(strgRxMessage.srcAddr, MSG_DATA_32, strgRxMessage.msgCmd, NULL, g_nvs_setting_data.config.sn_sig, MSG_DATA_32_LEN);
                    ESP_LOGI(STRG_TAG, "SN sig read:%lX", g_nvs_setting_data.config.sn_sig);
					break;

				case STRG_CMD_WR_SN_SIG:    //SN signature
                    sys_nvs_load_all();
                    g_nvs_setting_data.config.sn_sig = strgRxMessage.msgData;
                    sys_nvs_store_all();	
                    ESP_LOGI(STRG_TAG, "SN sig written:%lX", g_nvs_setting_data.config.sn_sig);
					break;	





                case STRG_CMD_WR_DEVICE_CONFIG:
                    memcpy(&scaleConfigData.scaleConfigDataStructId, (scale_config_data_t *)strgRxMessage.msgDataPtr, strgRxMessage.msgDataLen);
                    sys_nvs_load_all();

                    memset(g_nvs_setting_data.wifi.url,0,sizeof(g_nvs_setting_data.wifi.url));
                    memcpy(g_nvs_setting_data.wifi.url, (char *)scaleConfigData.url, sizeof(scaleConfigData.url));

                    // g_nvs_setting_data.device.providerId =          scaleConfigData.providerId;
                    // g_nvs_setting_data.device.locationId =          scaleConfigData.locationId;
                    // g_nvs_setting_data.device.sig =                 DEVICE_SIG_SET;

                    g_nvs_setting_data.config.sleep_duration =      scaleConfigData.sleep_duration;
                    g_nvs_setting_data.config.upload_duration =     scaleConfigData.upload_duration;
                    g_nvs_setting_data.config.low_batt_threshold =  scaleConfigData.low_batt_threshold;
                    g_nvs_setting_data.config.brownout_threshold =  scaleConfigData.brownout_threshold;
                    g_nvs_setting_data.config.wifi_retries =        scaleConfigData.wifi_retries;
                    // g_nvs_setting_data.config.tank_retries =        scaleConfigData.tank_retries;
                    // g_nvs_setting_data.config.tank_threshold =      scaleConfigData.tank_threshold;
                    // g_nvs_setting_data.config.tank_timer =          scaleConfigData.tank_timer;
                    g_nvs_setting_data.config.sig =                 CONFIG_SIG_SET;
                    sys_nvs_store_all();	
                    ESP_LOGI(STRG_TAG, "STRG_CMD_WR_DEVICE_CONFIG - data stored");
                    break;

				// case STRG_CMD_RD_DEVICE_SIG:    //Device Signature
                //     sys_nvs_load_all();			
                //     strgSendMessage(strgRxMessage.srcAddr, MSG_DATA_32, strgRxMessage.msgCmd, NULL, g_nvs_setting_data.device.sig, MSG_DATA_32_LEN);
                //     ESP_LOGI(STRG_TAG, "Device sig read:%lX", g_nvs_setting_data.device.sig);
				// 	break;

				// case STRG_CMD_WR_DEVICE_SIG:    //Device signature
                //     sys_nvs_load_all();
                //     g_nvs_setting_data.device.sig = strgRxMessage.msgData;
                //     sys_nvs_store_all();	
                //     ESP_LOGI(STRG_TAG, "Device sig written:%lX", g_nvs_setting_data.device.sig);
				// 	break;	

				// case STRG_CMD_RD_PROV_ID:    //Provider ID
                //     sys_nvs_load_all();			
                //     strgSendMessage(strgRxMessage.srcAddr, MSG_DATA_8, strgRxMessage.msgCmd, NULL, g_nvs_setting_data.device.providerId, MSG_DATA_8_LEN);
                //     ESP_LOGI(STRG_TAG, "Provider ID read:%d", g_nvs_setting_data.device.providerId);
				// 	break;

				// case STRG_CMD_WR_PROV_ID:    //Provider ID
                //     //ESP_LOGI(STRG_TAG, "Provider ID received:%ld", strgRxMessage.msgData);
                //     sys_nvs_load_all();
                //     if (g_nvs_setting_data.device.providerId != strgRxMessage.msgData)
                //     {
                //         g_nvs_setting_data.device.providerId = strgRxMessage.msgData;
                //         sys_nvs_store_all();	
                //         ESP_LOGI(STRG_TAG, "Provider ID written:%d", g_nvs_setting_data.device.providerId);
                //         strgSendMessage(MSG_ADDR_SCTL, MSG_DATA_0, EVT_STRG_CFG_CNG_PID, NULL, MSG_DATA_COMMAND_ONLY, MSG_DATA_0_LEN);
                //     }
                //     if (strgRxMessage.srcAddr == MSG_ADDR_SUPR)	
                //     {
                //         memset(strgTmpStr,0,sizeof(strgTmpStr));
                //         sprintf(strgTmpStr,"pass");
                //         strgSendMessage(strgRxMessage.srcAddr, MSG_DATA_STR, strgRxMessage.msgCmd, (uint8_t *)strgTmpStr, 0, strlen(strgTmpStr)+1);	
                //     }					
                //     break;	

				// case STRG_CMD_RD_LOC_ID:    //Location ID
                //     sys_nvs_load_all();			
                //     strgSendMessage(strgRxMessage.srcAddr, MSG_DATA_8, strgRxMessage.msgCmd, NULL, g_nvs_setting_data.device.locationId, MSG_DATA_8_LEN);
                //     ESP_LOGI(STRG_TAG, "Location ID read:%d", g_nvs_setting_data.device.locationId);
				// 	break;

				// case STRG_CMD_WR_LOC_ID:    //Location ID
                //     sys_nvs_load_all();
                //     if (g_nvs_setting_data.device.locationId != strgRxMessage.msgData)
                //     {
                //         g_nvs_setting_data.device.locationId = strgRxMessage.msgData;
                //         sys_nvs_store_all();	
                //         ESP_LOGI(STRG_TAG, "Location ID written:%d", g_nvs_setting_data.device.locationId);
                //         strgSendMessage(MSG_ADDR_SCTL, MSG_DATA_0, EVT_STRG_CFG_CNG_LID, NULL, MSG_DATA_COMMAND_ONLY, MSG_DATA_0_LEN);
                //     }
                //     if (strgRxMessage.srcAddr == MSG_ADDR_SUPR)	
                //     {
                //         memset(strgTmpStr,0,sizeof(strgTmpStr));
                //         sprintf(strgTmpStr,"pass");
                //         strgSendMessage(strgRxMessage.srcAddr, MSG_DATA_STR, strgRxMessage.msgCmd, (uint8_t *)strgTmpStr, 0, strlen(strgTmpStr)+1);	
                //     }
				// 	break;	

				case STRG_CMD_RD_WIFI_RET:    //wifi_retries
                    sys_nvs_load_all();			
                    strgSendMessage(strgRxMessage.srcAddr, MSG_DATA_8, strgRxMessage.msgCmd, NULL, g_nvs_setting_data.config.wifi_retries, MSG_DATA_8_LEN);
                    ESP_LOGI(STRG_TAG, "wifi_retries read:%d", g_nvs_setting_data.config.wifi_retries);
					break;

				case STRG_CMD_WR_WIFI_RET:    //wifi_retries
                    sys_nvs_load_all();
                    if (g_nvs_setting_data.config.wifi_retries != strgRxMessage.msgData)
                    {
                        g_nvs_setting_data.config.wifi_retries = strgRxMessage.msgData;
                        sys_nvs_store_all();	
                        ESP_LOGI(STRG_TAG, "wifi_retries written:%d", g_nvs_setting_data.config.wifi_retries);
                        strgSendMessage(MSG_ADDR_SCTL, MSG_DATA_0, EVT_STRG_CFG_CNG_WRET, NULL, MSG_DATA_COMMAND_ONLY, MSG_DATA_0_LEN);
                    }
					break;	

				// case STRG_CMD_RD_TANK_RET:    //tank_retries
                //     sys_nvs_load_all();			
                //     strgSendMessage(strgRxMessage.srcAddr, MSG_DATA_8, strgRxMessage.msgCmd, NULL, g_nvs_setting_data.config.tank_retries, MSG_DATA_8_LEN);
                //     ESP_LOGI(STRG_TAG, "tank_retries read:%d", g_nvs_setting_data.config.tank_retries);
				// 	break;

				// case STRG_CMD_WR_TANK_RET:    //tank_retries
                //     sys_nvs_load_all();
                //     if (g_nvs_setting_data.config.tank_retries != strgRxMessage.msgData)
                //     {
                //         g_nvs_setting_data.config.tank_retries = strgRxMessage.msgData;
                //         sys_nvs_store_all();	
                //         ESP_LOGI(STRG_TAG, "tank_retries written:%d", g_nvs_setting_data.config.tank_retries);
                //         strgSendMessage(MSG_ADDR_SCTL, MSG_DATA_0, EVT_STRG_CFG_CNG_TRET, NULL, MSG_DATA_COMMAND_ONLY, MSG_DATA_0_LEN);
                //     }
				// 	break;	

				case STRG_CMD_RD_SLP_DUR:    //sleep_duration
                    sys_nvs_load_all();	
#ifndef SLEEP_DURATION_OVERRIDE                    		
                    strgSendMessage(strgRxMessage.srcAddr, MSG_DATA_16, strgRxMessage.msgCmd, NULL, g_nvs_setting_data.config.sleep_duration, MSG_DATA_16_LEN);
#else
                    strgSendMessage(strgRxMessage.srcAddr, MSG_DATA_16, strgRxMessage.msgCmd, NULL, SLEEP_OVERRIDE_MIN, MSG_DATA_16_LEN);
#endif                    
                    ESP_LOGI(STRG_TAG, "sleep_duration read:%d", g_nvs_setting_data.config.sleep_duration);
					break;

				case STRG_CMD_WR_SLP_DUR:    //sleep_duration
                    sys_nvs_load_all();
                    if (g_nvs_setting_data.config.sleep_duration != strgRxMessage.msgData)
                    {
                        g_nvs_setting_data.config.sleep_duration = strgRxMessage.msgData;
                        sys_nvs_store_all();	
                        ESP_LOGI(STRG_TAG, "sleep_duration written:%d", g_nvs_setting_data.config.sleep_duration);
                        strgSendMessage(MSG_ADDR_SCTL, MSG_DATA_0, EVT_STRG_CFG_CNG_SDUR, NULL, MSG_DATA_COMMAND_ONLY, MSG_DATA_0_LEN);
                    }
					break;	

				case STRG_CMD_RD_UPL_DUR:    //upload_duration
                    sys_nvs_load_all();			
                    strgSendMessage(strgRxMessage.srcAddr, MSG_DATA_16, strgRxMessage.msgCmd, NULL, g_nvs_setting_data.config.upload_duration, MSG_DATA_16_LEN);
                    ESP_LOGI(STRG_TAG, "upload_duration read:%d", g_nvs_setting_data.config.upload_duration);
					break;

				case STRG_CMD_WR_UPL_DUR:    //upload_duration
                    sys_nvs_load_all();
                    if (g_nvs_setting_data.config.upload_duration != strgRxMessage.msgData)
                    {
                        g_nvs_setting_data.config.upload_duration = strgRxMessage.msgData;
                        sys_nvs_store_all();	
                        ESP_LOGI(STRG_TAG, "upload_duration written:%d", g_nvs_setting_data.config.upload_duration);
                        strgSendMessage(MSG_ADDR_SCTL, MSG_DATA_0, EVT_STRG_CFG_CNG_UDUR, NULL, MSG_DATA_COMMAND_ONLY, MSG_DATA_0_LEN);
                    }
					break;	

				case STRG_CMD_RD_LBAT_THR:    //low_batt_threshold
                    sys_nvs_load_all();			
                    strgSendMessage(strgRxMessage.srcAddr, MSG_DATA_16, strgRxMessage.msgCmd, NULL, g_nvs_setting_data.config.low_batt_threshold, MSG_DATA_16_LEN);
                    ESP_LOGI(STRG_TAG, "low_batt_threshold read:%d", g_nvs_setting_data.config.low_batt_threshold);
					break;

				case STRG_CMD_WR_LBAT_THR:    //low_batt_threshold
                    sys_nvs_load_all();
                    if (g_nvs_setting_data.config.low_batt_threshold != strgRxMessage.msgData)
                    {
                        g_nvs_setting_data.config.low_batt_threshold = strgRxMessage.msgData;
                        sys_nvs_store_all();	
                        ESP_LOGI(STRG_TAG, "low_batt_threshold written:%d", g_nvs_setting_data.config.low_batt_threshold);
                        strgSendMessage(MSG_ADDR_SCTL, MSG_DATA_0, EVT_STRG_CFG_CNG_LBT, NULL, MSG_DATA_COMMAND_ONLY, MSG_DATA_0_LEN);
                    }
					break;	

				case STRG_CMD_WR_BRNO_THR:    //brownout_threshold
                    sys_nvs_load_all();
                    if (g_nvs_setting_data.config.brownout_threshold != strgRxMessage.msgData)
                    {
                        g_nvs_setting_data.config.brownout_threshold = strgRxMessage.msgData;
                        sys_nvs_store_all();	
                        ESP_LOGI(STRG_TAG, "brownout_threshold written:%d", g_nvs_setting_data.config.brownout_threshold);
                        strgSendMessage(MSG_ADDR_SCTL, MSG_DATA_0, EVT_STRG_CFG_CNG_BRT, NULL, MSG_DATA_COMMAND_ONLY, MSG_DATA_0_LEN);
                    }
					break;	

				// case STRG_CMD_RD_TANK_THR:    //tank_threshold
                //     sys_nvs_load_all();			
                //     strgSendMessage(strgRxMessage.srcAddr, MSG_DATA_32, strgRxMessage.msgCmd, NULL, g_nvs_setting_data.config.tank_threshold, MSG_DATA_32_LEN);
                //     ESP_LOGI(STRG_TAG, "tank_threshold read:%ld", g_nvs_setting_data.config.tank_threshold);
				// 	break;

				// case STRG_CMD_WR_TANK_THR:    //tank_threshold
                //     sys_nvs_load_all();
                //     if (g_nvs_setting_data.config.tank_threshold != strgRxMessage.msgData)
                //     {
                //         g_nvs_setting_data.config.tank_threshold = strgRxMessage.msgData;
                //         sys_nvs_store_all();	
                //         ESP_LOGI(STRG_TAG, "tank_threshold written:%ld", g_nvs_setting_data.config.tank_threshold);
                //         strgSendMessage(MSG_ADDR_SCTL, MSG_DATA_0, EVT_STRG_CFG_CNG_TTH, NULL, MSG_DATA_COMMAND_ONLY, MSG_DATA_0_LEN);
                //     }
				// 	break;	

				// case STRG_CMD_RD_TANK_TMR:    //tank_timer
                //     sys_nvs_load_all();			
                //     strgSendMessage(strgRxMessage.srcAddr, MSG_DATA_32, strgRxMessage.msgCmd, NULL, g_nvs_setting_data.config.tank_timer, MSG_DATA_32_LEN);
                //     ESP_LOGI(STRG_TAG, "tank_timer read:%ld", g_nvs_setting_data.config.tank_timer);
				// 	break;

				// case STRG_CMD_WR_TANK_TMR:    //tank_timer
                //     sys_nvs_load_all();
                //     if (g_nvs_setting_data.config.tank_timer != strgRxMessage.msgData)
                //     {
                //         g_nvs_setting_data.config.tank_timer = strgRxMessage.msgData;
                //         sys_nvs_store_all();	
                //         ESP_LOGI(STRG_TAG, "tank_timer written:%ld", g_nvs_setting_data.config.tank_timer);
                //         strgSendMessage(MSG_ADDR_SCTL, MSG_DATA_0, EVT_STRG_CFG_CNG_TTM, NULL, MSG_DATA_COMMAND_ONLY, MSG_DATA_0_LEN);
                //     }
				// 	break;	


                // case STRG_CMD_READ_DEVICE_DATA: //Get all device related data from nvs_data
                //     strgDeviceData.deviceDataStructId   = DEVICE_DATA_STRUCT_ID;
                //     strgDeviceData.providerId           = g_nvs_setting_data.device.providerId;
                //     strgDeviceData.locationId           = g_nvs_setting_data.device.locationId;
                //     memcpy(strgDeviceData.deviceToken,  g_nvs_setting_data.https.deviceToken, strlen(g_nvs_setting_data.https.deviceToken));
                //     memcpy(strgDeviceData.loxId,        g_nvs_setting_data.https.loxId, strlen(g_nvs_setting_data.https.loxId));
                //     memcpy(strgDeviceData.url,          g_nvs_setting_data.wifi.url, strlen(g_nvs_setting_data.wifi.url));
                //     strgDeviceData.sig                  = g_nvs_setting_data.device.sig;

                //     //strgSendMessage(strgRxMessage.srcAddr, MSG_DATA_STRUC, STRG_CMD_READ_DEVICE_DATA, (uint8_t *) &strgDeviceData.deviceDataStructId, 0, sizeof(strgDeviceData));
                //     strgTxMessage.srcAddr         =  MSG_ADDR_STRG; 
                //     strgTxMessage.dstAddr         =  strgRxMessage.srcAddr;
                //     strgTxMessage.msgType         =  MSG_DATA_STRUC;
                //     strgTxMessage.msgCmd          =  STRG_CMD_READ_DEVICE_DATA;
                //     strgTxMessage.msgRef          = msg_getMsgRef();
                //     strgTxMessage.msgTimeStamp    = sys_getMsgTimeStamp();
                //     strgTxMessage.msgDataPtr      = (uint8_t *) &strgDeviceData.deviceDataStructId;
                //     strgTxMessage.msgData         = MSG_DATA_POINTER_ONLY;
                //     strgTxMessage.msgDataLen      = sizeof(strgDeviceData); 
                //     xQueueSend(dispatcherQueueHandle,&strgTxMessage,0);
                //     break;

                //Configuration
				case STRG_CMD_RD_SN:  //serial number
                    ESP_LOGI(STRG_TAG, "STRG_CMD_RD_SN command received");
                    sys_nvs_load_all();
                    memset(strgTmpStr,0,sizeof(strgTmpStr));
                    if (0 >= strlen(g_nvs_setting_data.config.sn)) { //Set data to default
                        memcpy(strgTmpStr,DEFAULT_SN, strlen(DEFAULT_SN));
                    }
                    else {
                        memcpy(strgTmpStr,g_nvs_setting_data.config.sn, strlen(g_nvs_setting_data.config.sn));
                    }
					strgSendMessage(strgRxMessage.srcAddr, MSG_DATA_STR, STRG_CMD_RD_SN, (uint8_t *)strgTmpStr, 0, strlen(strgTmpStr)+1);
					break;

				case STRG_CMD_WR_SN:  //serial number
                    ESP_LOGI(STRG_TAG, "Serial number being stored:%s", (char *)strgRxMessage.msgDataPtr);
                    sys_nvs_load_all();
                    memset(g_nvs_setting_data.config.sn,0,sizeof(g_nvs_setting_data.config.sn));
					memcpy(g_nvs_setting_data.config.sn, strgRxMessage.msgDataPtr, strgRxMessage.msgDataLen);
                    g_nvs_setting_data.config.sn_sig	= CONFIG_SN_SIG_SET;
                    sys_nvs_store_all();	
                    strgSendMessage(MSG_ADDR_BLE, MSG_DATA_STR, STRG_CMD_RD_SN, (uint8_t *)strgRxMessage.msgDataPtr, 0, strgRxMessage.msgDataLen);
                    strgSendMessage(MSG_ADDR_WIFI, MSG_DATA_STR, STRG_CMD_RD_SN, (uint8_t *)strgRxMessage.msgDataPtr, 0, strgRxMessage.msgDataLen);
                    strgSendMessage(MSG_ADDR_SCTL, MSG_DATA_STR, STRG_CMD_RD_SN, (uint8_t *)strgRxMessage.msgDataPtr, 0, strgRxMessage.msgDataLen);
                    //strgSendMessage(MSG_ADDR_BLE, MSG_DATA_0, BLE_CMD_GENERATE_KEY, NULL, MSG_DATA_COMMAND_ONLY, MSG_DATA_0_LEN);

                    if (strgRxMessage.srcAddr == MSG_ADDR_SUPR)	
                    {
                        memset(strgTmpStr,0,sizeof(strgTmpStr));
                        sprintf(strgTmpStr,"pass");
                        strgSendMessage(strgRxMessage.srcAddr, MSG_DATA_STR, strgRxMessage.msgCmd, (uint8_t *)strgTmpStr, 0, strlen(strgTmpStr)+1);	
                    }
					break;

				case STRG_CMD_RD_HW_REV:  //hardware rev
                    ESP_LOGI(STRG_TAG, "STRG_CMD_RD_HW_REV command received");
                    sys_nvs_load_all();
                    memset(strgTmpStr,0,sizeof(strgTmpStr));
                    if (0 >= strlen(g_nvs_setting_data.config.hw_rev)) { //Set data to default
                        memcpy(strgTmpStr,DEFAULT_HW_REV, strlen(DEFAULT_HW_REV));
                    }
                    else {
                        memcpy(strgTmpStr,g_nvs_setting_data.config.hw_rev, strlen(g_nvs_setting_data.config.hw_rev));
                    }
					strgSendMessage(strgRxMessage.srcAddr, MSG_DATA_STR, strgRxMessage.msgCmd, (uint8_t *)strgTmpStr, 0, strlen(strgTmpStr)+1);
					break;

				case STRG_CMD_WR_HW_REV:  //hardware rev
                    ESP_LOGI(STRG_TAG, "Hardware rev being stored:%s", (char *)strgRxMessage.msgDataPtr);
                    sys_nvs_load_all();
                    memset(g_nvs_setting_data.config.hw_rev,0,sizeof(g_nvs_setting_data.config.hw_rev));
					memcpy(g_nvs_setting_data.config.hw_rev, strgRxMessage.msgDataPtr, strgRxMessage.msgDataLen);
                    //g_nvs_setting_data.config.sn_sig	= CONFIG_SN_SIG_SET;
                    sys_nvs_store_all();	
                    //strgSendMessage(MSG_ADDR_BLE, MSG_DATA_STR, STRG_CMD_RD_SN, (uint8_t *)strgRxMessage.msgDataPtr, 0, strgRxMessage.msgDataLen);
                    //strgSendMessage(MSG_ADDR_WIFI, MSG_DATA_STR, STRG_CMD_RD_SN, (uint8_t *)strgRxMessage.msgDataPtr, 0, strgRxMessage.msgDataLen);
                    strgSendMessage(MSG_ADDR_SCTL, MSG_DATA_STR, STRG_CMD_RD_HW_REV, (uint8_t *)strgRxMessage.msgDataPtr, 0, strgRxMessage.msgDataLen);
                    strgSendMessage(MSG_ADDR_SER, MSG_DATA_STR, STRG_CMD_RD_HW_REV, (uint8_t *)strgRxMessage.msgDataPtr, 0, strgRxMessage.msgDataLen);

                    if (strgRxMessage.srcAddr == MSG_ADDR_SUPR)	
                    {
                        memset(strgTmpStr,0,sizeof(strgTmpStr));
                        sprintf(strgTmpStr,"pass");
                        strgSendMessage(strgRxMessage.srcAddr, MSG_DATA_STR, strgRxMessage.msgCmd, (uint8_t *)strgTmpStr, 0, strlen(strgTmpStr)+1);	
                    }
					break;


				// case STRG_CMD_RD_PASSKEY:    //BLE security passkey
                //     sys_nvs_load_all();			
                //     strgSendMessage(strgRxMessage.srcAddr, MSG_DATA_32, strgRxMessage.msgCmd, NULL, g_nvs_setting_data.config.passkey, MSG_DATA_32_LEN);
                //     ESP_LOGI(STRG_TAG, "passkey read from storage:%ld", g_nvs_setting_data.config.passkey);
				// 	break;

				// case STRG_CMD_WR_PASSKEY:    //BLE security passkey
                //     sys_nvs_load_all();
                //     if (g_nvs_setting_data.config.passkey != strgRxMessage.msgData)
                //     {
                //         g_nvs_setting_data.config.passkey = strgRxMessage.msgData;
                //         sys_nvs_store_all();	
                //         ESP_LOGI(STRG_TAG, "passkey written to storage:%ld", g_nvs_setting_data.config.passkey);
                //         //strgSendMessage(MSG_ADDR_SCTL, MSG_DATA_0, EVT_STRG_CFG_CNG_TTM, NULL, MSG_DATA_COMMAND_ONLY, MSG_DATA_0_LEN);
                //     }
				// 	break;	

				case STRG_CMD_RD_NVS_DATA:  //serial number
                    sys_nvs_load_all();

                    ESP_LOGI(STRG_TAG,"NVS_DATA-version:%ld",g_nvs_setting_data.data_version);

                    ESP_LOGI(STRG_TAG,"NVS_DATA-wifi-ssid:%s",g_nvs_setting_data.wifi.ssid);
                    ESP_LOGI(STRG_TAG,"NVS_DATA-wifi-pwd:%s",g_nvs_setting_data.wifi.pwd);
                    ESP_LOGI(STRG_TAG,"NVS_DATA-wifi-url:%s",g_nvs_setting_data.wifi.url);
                    ESP_LOGI(STRG_TAG,"NVS_DATA-wifi-sig:%lX",g_nvs_setting_data.wifi.sig);

                    // ESP_LOGI(STRG_TAG,"NVS_DATA-scale-gain:%ld",g_nvs_setting_data.scale.gain);
                    // ESP_LOGI(STRG_TAG,"NVS_DATA-scale-offset:%ld",g_nvs_setting_data.scale.offset);
                    // ESP_LOGI(STRG_TAG,"NVS_DATA-scale-offset:%ld",g_nvs_setting_data.scale.calOffset);
                    // ESP_LOGI(STRG_TAG,"NVS_DATA-scale-sig:%lX",g_nvs_setting_data.scale.sig);

                    ESP_LOGI(STRG_TAG,"NVS_DATA-config-sleep_duration:%d",g_nvs_setting_data.config.sleep_duration);
                    ESP_LOGI(STRG_TAG,"NVS_DATA-config-upload_duration:%d",g_nvs_setting_data.config.upload_duration);
                    ESP_LOGI(STRG_TAG,"NVS_DATA-config-duration_start_time:%ld",g_nvs_setting_data.config.duration_start_time);
                    ESP_LOGI(STRG_TAG,"NVS_DATA-config-low_batt_threshold:%d",g_nvs_setting_data.config.low_batt_threshold);
                    ESP_LOGI(STRG_TAG,"NVS_DATA-config-brownout_threshold:%d",g_nvs_setting_data.config.brownout_threshold);
                    ESP_LOGI(STRG_TAG,"NVS_DATA-config-wifi_retries:%d",g_nvs_setting_data.config.wifi_retries);
                    // ESP_LOGI(STRG_TAG,"NVS_DATA-config-tank_retries:%d",g_nvs_setting_data.config.tank_retries);
                    // ESP_LOGI(STRG_TAG,"NVS_DATA-config-tank_threshold:%ld",g_nvs_setting_data.config.tank_threshold);
                    // ESP_LOGI(STRG_TAG,"NVS_DATA-config-tank_timer:%ld",g_nvs_setting_data.config.tank_timer);
                    ESP_LOGI(STRG_TAG,"NVS_DATA-config-sn:%s",g_nvs_setting_data.config.sn);
                    ESP_LOGI(STRG_TAG,"NVS_DATA-config-sig:%lX",g_nvs_setting_data.config.sig);
                    ESP_LOGI(STRG_TAG,"NVS_DATA-config-sn_sig:%lX",g_nvs_setting_data.config.sn_sig);

                    // ESP_LOGI(STRG_TAG,"NVS_DATA-https-loxId:%s",g_nvs_setting_data.https.loxId);
                    // ESP_LOGI(STRG_TAG,"NVS_DATA-https-deviceToken:%s",g_nvs_setting_data.https.deviceToken);
                    // ESP_LOGI(STRG_TAG,"NVS_DATA-https-sig:%lX",g_nvs_setting_data.https.sig);

                    // ESP_LOGI(STRG_TAG,"NVS_DATA-device-providerId:%d",g_nvs_setting_data.device.providerId);
                    // ESP_LOGI(STRG_TAG,"NVS_DATA-device-locationId:%d",g_nvs_setting_data.device.locationId);
                    // ESP_LOGI(STRG_TAG,"NVS_DATA-device-sig:%lX",g_nvs_setting_data.device.sig);
					break;

				case STRG_CMD_RD_CONFIG_SIG:    //Config Signature
                    sys_nvs_load_all();			
                    strgSendMessage(strgRxMessage.srcAddr, MSG_DATA_32, strgRxMessage.msgCmd, NULL, g_nvs_setting_data.config.sig, MSG_DATA_32_LEN);
                    ESP_LOGI(STRG_TAG, "config sig read:%lX", g_nvs_setting_data.config.sig);
					break;

				// case STRG_CMD_WR_CONFIG_SIG:    //Config signature
                //     sys_nvs_load_all();
                //     g_nvs_setting_data.config.sig = strgRxMessage.msgData;
                //     sys_nvs_store_all();	
                //     ESP_LOGI(STRG_TAG, "config sig written:%lX", g_nvs_setting_data.scale.sig);
				// 	break;	

				// case STRG_CMD_SET_CONFIG_SIG:    //Config signature = set value
                //     sys_nvs_load_all();
                //     g_nvs_setting_data.config.sig = CONFIG_SIG_SET;
                //     sys_nvs_store_all();	
                //     ESP_LOGI(STRG_TAG, "config sig written:%lX", g_nvs_setting_data.scale.sig);
				// 	break;	

                //Scale data storage
				// case STRG_CMD_RD_SCALE_SIG:    //Scale Signature
                //     sys_nvs_load_all();			
                //     strgSendMessage(strgRxMessage.srcAddr, MSG_DATA_32, STRG_CMD_RD_SCALE_SIG, NULL, g_nvs_setting_data.scale.sig, MSG_DATA_32_LEN);
                //     ESP_LOGI(STRG_TAG, "scale sig read:%lX", g_nvs_setting_data.scale.sig);
				// 	break;

				// case STRG_CMD_WR_SCALE_INIT_SIG:    //Scale signature - Scale Initalized
                //     sys_nvs_load_all();
                //     //tmpVal32 = g_nvs_setting_data.scale.sig & 0xFFFF0000;
                //     tmpVal32 = SCALE_SIG_INIT_SET;
                //     g_nvs_setting_data.scale.sig = tmpVal32;
                //     sys_nvs_store_all();	
                //     ESP_LOGI(STRG_TAG, "Scale sig written:%lX", g_nvs_setting_data.scale.sig);
				// 	break;	

				// case STRG_CMD_WR_SCALE_WGHT_SIG:    //Scale signature - Scale Gain set
                //     sys_nvs_load_all();
                //     tmpVal32 = g_nvs_setting_data.scale.sig & 0xFF00FFFF;
                //     tmpVal32 = tmpVal32 | SCALE_SIG_WGHT_SET;
                //     g_nvs_setting_data.scale.sig = tmpVal32;
                //     sys_nvs_store_all();	
                //     ESP_LOGI(STRG_TAG, "Scale sig written:%lX", g_nvs_setting_data.scale.sig);
				// 	break;	

				// case STRG_CMD_WR_SCALE_TARE_SIG:    //Scale signature - Scale Offset set
                //     sys_nvs_load_all();
                //     tmpVal32 = g_nvs_setting_data.scale.sig & 0x00FFFFFF;
                //     tmpVal32 = tmpVal32 | SCALE_SIG_TARE_SET;
                //     g_nvs_setting_data.scale.sig = tmpVal32;
                //     sys_nvs_store_all();	
                //     ESP_LOGI(STRG_TAG, "Scale sig written:%lX", g_nvs_setting_data.scale.sig);
				// 	break;	

				// case STRG_CMD_WR_SCALE_SIG:    //scale signature
                //     sys_nvs_load_all();
                //     g_nvs_setting_data.scale.sig = strgRxMessage.msgData;
                //     sys_nvs_store_all();	
                //     ESP_LOGI(STRG_TAG, "scale sig written:%lX", g_nvs_setting_data.scale.sig);
				// 	break;	

				// case STRG_CMD_RD_SCALE_GAIN:    //Scale Gain
                //     sys_nvs_load_all();			
                //     strgSendMessage(strgRxMessage.srcAddr, MSG_DATA_32, STRG_CMD_RD_SCALE_GAIN, NULL, g_nvs_setting_data.scale.gain, MSG_DATA_32_LEN);
                //     ESP_LOGI(STRG_TAG, "scale gain read:%ld", g_nvs_setting_data.scale.gain);
				// 	break;

				// case STRG_CMD_WR_SCALE_GAIN:    //scale gain
                //     sys_nvs_load_all();
                //     g_nvs_setting_data.scale.gain = strgRxMessage.msgData;
                //     sys_nvs_store_all();
                //     ESP_LOGI(STRG_TAG, "scale gain written:%ld", g_nvs_setting_data.scale.gain);
				// 	break;	

				// case STRG_CMD_RD_SCALE_OFFSET:    //Scale offset
                //     sys_nvs_load_all();			
                //     strgSendMessage(strgRxMessage.srcAddr, MSG_DATA_32, STRG_CMD_RD_SCALE_OFFSET, NULL, g_nvs_setting_data.scale.offset, MSG_DATA_32_LEN);
                //     ESP_LOGI(STRG_TAG, "scale offset read:%ld", g_nvs_setting_data.scale.offset);
				// 	break;

				// case STRG_CMD_WR_SCALE_OFFSET:    //scale offset
                //     sys_nvs_load_all();
                //     g_nvs_setting_data.scale.offset = strgRxMessage.msgData;
                //     sys_nvs_store_all();	
                //     ESP_LOGI(STRG_TAG, "scale offset written:%ld", g_nvs_setting_data.scale.offset);
				// 	break;	

				// case STRG_CMD_RD_SCALE_CAL_OFFSET:    //Scale Calibration offset
                //     sys_nvs_load_all();			
                //     strgSendMessage(strgRxMessage.srcAddr, MSG_DATA_32, STRG_CMD_RD_SCALE_CAL_OFFSET, NULL, g_nvs_setting_data.scale.calOffset, MSG_DATA_32_LEN);
                //     ESP_LOGI(STRG_TAG, "scale Calibration offset read:%ld", g_nvs_setting_data.scale.calOffset);
				// 	break;

				// case STRG_CMD_WR_SCALE_CAL_OFFSET:    //scale Calibration offset
                //     sys_nvs_load_all();
                //     g_nvs_setting_data.scale.calOffset = strgRxMessage.msgData;
                //     sys_nvs_store_all();	
                //     ESP_LOGI(STRG_TAG, "scale Calibration offset written:%ld", g_nvs_setting_data.scale.calOffset);
				// 	break;	

                // case STRG_CMD_RD_TANK_RETRIES:
                //     sys_nvs_load_all();			
                //     strgSendMessage(strgRxMessage.srcAddr, MSG_DATA_8, STRG_CMD_RD_TANK_RETRIES, NULL, g_nvs_setting_data.config.tank_retries, MSG_DATA_8_LEN);
                //     ESP_LOGI(STRG_TAG, "tank retries read:%d", g_nvs_setting_data.config.tank_retries);
                //     break;

                // case STRG_CMD_RD_TANK_THRESHOLD:
                //     sys_nvs_load_all();			
                //     strgSendMessage(strgRxMessage.srcAddr, MSG_DATA_32, STRG_CMD_RD_TANK_THRESHOLD, NULL, g_nvs_setting_data.config.tank_threshold, MSG_DATA_32_LEN);
                //     ESP_LOGI(STRG_TAG, "tank threshold read:%ld", g_nvs_setting_data.config.tank_threshold);
                //     break;

                // case STRG_CMD_RD_TANK_TIMER:
                //     sys_nvs_load_all();			
                //     strgSendMessage(strgRxMessage.srcAddr, MSG_DATA_32, STRG_CMD_RD_TANK_TIMER, NULL, g_nvs_setting_data.config.tank_timer, MSG_DATA_32_LEN);
                //     ESP_LOGI(STRG_TAG, "tank timer read:%ld", g_nvs_setting_data.config.tank_timer);
                //     break;

                //Battery data storage
                case STRG_CMD_RD_LOW_BATT_THRES:
                    sys_nvs_load_all();			
                    strgSendMessage(strgRxMessage.srcAddr, MSG_DATA_16, STRG_CMD_RD_LOW_BATT_THRES, NULL, g_nvs_setting_data.config.low_batt_threshold, MSG_DATA_16_LEN);
                    ESP_LOGI(STRG_TAG, "low battery threshold read:%X", g_nvs_setting_data.config.low_batt_threshold);
                    break;

                case STRG_CMD_RD_BROWNOUT_THRES:
                    sys_nvs_load_all();			
                    strgSendMessage(strgRxMessage.srcAddr, MSG_DATA_16, STRG_CMD_RD_BROWNOUT_THRES, NULL, g_nvs_setting_data.config.brownout_threshold, MSG_DATA_16_LEN);
                    ESP_LOGI(STRG_TAG, "brownout threshold read:%X", g_nvs_setting_data.config.brownout_threshold);
                    break;

                //Wifi Data storage
				case STRG_CMD_RD_SSID:  //ssid
                    sys_nvs_load_all();
                    memset(strgTmpStr,0,sizeof(strgTmpStr));
                    if (0 >= strlen(g_nvs_setting_data.wifi.ssid)) { //Set data to default
                        memcpy(strgTmpStr,DEFAULT_SSID, strlen(DEFAULT_SSID));
                    }
                    else {
                        memcpy(strgTmpStr,g_nvs_setting_data.wifi.ssid, strlen(g_nvs_setting_data.wifi.ssid));
                    }
					strgSendMessage(strgRxMessage.srcAddr, MSG_DATA_STR, STRG_CMD_RD_SSID, (uint8_t *)strgTmpStr, 0, strlen(strgTmpStr)+1);
					break;

				case STRG_CMD_WR_SSID:  //ssid
                    ESP_LOGI(STRG_TAG, "Wifi ssid being stored:%s", (char *)strgRxMessage.msgDataPtr);
                    sys_nvs_load_all();
                    memset(g_nvs_setting_data.wifi.ssid,0,sizeof(g_nvs_setting_data.wifi.ssid));
					memcpy(g_nvs_setting_data.wifi.ssid, strgRxMessage.msgDataPtr, strgRxMessage.msgDataLen);

                    tmpVal32 = g_nvs_setting_data.wifi.sig;
                    ESP_LOGI(STRG_TAG,"Wifi Signature read from NVS:%lX", tmpVal32);
                    tmpVal32 = tmpVal32 & 0xFFFF0000;
                    tmpVal32 = tmpVal32 | WIFI_SIG_SSID_SET;
                    g_nvs_setting_data.wifi.sig = tmpVal32;
                    ESP_LOGI(STRG_TAG,"Wifi Signature stored in NVS:%lX", g_nvs_setting_data.wifi.sig);

                    sys_nvs_store_all();

                    if (strgRxMessage.srcAddr == MSG_ADDR_SUPR)	
                    {
                        memset(strgTmpStr,0,sizeof(strgTmpStr));
                        sprintf(strgTmpStr,"pass");
                        strgSendMessage(strgRxMessage.srcAddr, MSG_DATA_STR, strgRxMessage.msgCmd, (uint8_t *)strgTmpStr, 0, strlen(strgTmpStr)+1);	
                    }
					break;
                    
				case STRG_CMD_RD_PWD:   //password
                    ESP_LOGI(STRG_TAG, "STRG_CMD_RD_PWD command received");
                    sys_nvs_load_all();
                    memset(strgTmpStr,0,sizeof(strgTmpStr));
                    if (0 >= strlen(g_nvs_setting_data.wifi.pwd)) { //Set data to default
                        memcpy(strgTmpStr,DEFAULT_PWD, strlen(DEFAULT_PWD));
                    }
                    else {
                        memcpy(strgTmpStr,g_nvs_setting_data.wifi.pwd, strlen(g_nvs_setting_data.wifi.pwd));
                    }					
                    strgSendMessage(strgRxMessage.srcAddr, MSG_DATA_STR, STRG_CMD_RD_PWD, (uint8_t *)strgTmpStr, 0, strlen(strgTmpStr)+1);
					break;

				case STRG_CMD_WR_PWD:   //password
                    ESP_LOGI(STRG_TAG, "Wifi pwd being stored:%s", (char *)strgRxMessage.msgDataPtr);
                    sys_nvs_load_all();
                    memset(g_nvs_setting_data.wifi.pwd,0,sizeof(g_nvs_setting_data.wifi.pwd));
					memcpy(g_nvs_setting_data.wifi.pwd, strgRxMessage.msgDataPtr, strgRxMessage.msgDataLen);	

                    tmpVal32 = g_nvs_setting_data.wifi.sig;
                    ESP_LOGI(STRG_TAG,"Wifi Signature read from NVS:%lX", tmpVal32);
                    tmpVal32 = tmpVal32 & 0x0000FFFF;
                    tmpVal32 = tmpVal32 | WIFI_SIG_PWD_SET;
                    g_nvs_setting_data.wifi.sig = tmpVal32;
                    ESP_LOGI(STRG_TAG,"Wifi Signature stored in NVS:%lX", g_nvs_setting_data.wifi.sig);
                    sys_nvs_store_all();

                    if (strgRxMessage.srcAddr == MSG_ADDR_SUPR)	
                    {
                        memset(strgTmpStr,0,sizeof(strgTmpStr));
                        sprintf(strgTmpStr,"pass");
                        strgSendMessage(strgRxMessage.srcAddr, MSG_DATA_STR, strgRxMessage.msgCmd, (uint8_t *)strgTmpStr, 0, strlen(strgTmpStr)+1);	
                    }
					break;

                //Base URL
				case STRG_CMD_RD_BASE_URL:  //base url
                    ESP_LOGI(STRG_TAG, "STRG_CMD_RD_BASE_URL command received");
                    sys_nvs_load_all();
                    memset(strgTmpStr,0,sizeof(strgTmpStr));
                    if (0 >= strlen(g_nvs_setting_data.wifi.url)) { //Set data to default
                        memcpy(strgTmpStr,DEFAULT_BASE_URL, strlen(DEFAULT_BASE_URL));
                    }
                    else {
                        memcpy(strgTmpStr,g_nvs_setting_data.wifi.url, strlen(g_nvs_setting_data.wifi.url));
                    }
					strgSendMessage(strgRxMessage.srcAddr, MSG_DATA_STR, STRG_CMD_RD_BASE_URL, (uint8_t *)strgTmpStr, 0, strlen(strgTmpStr)+1);
					break;

				case STRG_CMD_WR_BASE_URL:  //base url
                    sys_nvs_load_all();
                    if (strgRxMessage.srcAddr == MSG_ADDR_SUPR)
                    {
                        memset(strgTmpStr,0,sizeof(strgTmpStr));
                        if (strcmp((char *)strgRxMessage.msgDataPtr,(char *)g_nvs_setting_data.wifi.url) != 0)
                        {
                            memset(g_nvs_setting_data.wifi.url,0,sizeof(g_nvs_setting_data.wifi.url));
                            memcpy(g_nvs_setting_data.wifi.url, strgRxMessage.msgDataPtr, strgRxMessage.msgDataLen);	
                            sys_nvs_store_all();	
                        }
                        sprintf(strgTmpStr,"pass");
                        strgSendMessage(strgRxMessage.srcAddr, MSG_DATA_STR, strgRxMessage.msgCmd, (uint8_t *)strgTmpStr, 0, strlen(strgTmpStr)+1);	
                    }
                    else
                    {
                        ESP_LOGI(STRG_TAG, "Base URL received:%s", (char *)strgRxMessage.msgDataPtr);
                        if (strcmp((char *)strgRxMessage.msgDataPtr,(char *)g_nvs_setting_data.wifi.url) != 0)
                        {
                            memset(g_nvs_setting_data.wifi.url,0,sizeof(g_nvs_setting_data.wifi.url));
                            memcpy(g_nvs_setting_data.wifi.url, strgRxMessage.msgDataPtr, strgRxMessage.msgDataLen);	
                            sys_nvs_store_all();	
                            ESP_LOGI(STRG_TAG, "New Base URL being stored:%s", (char *)strgRxMessage.msgDataPtr);
                            strgSendMessage(MSG_ADDR_SCTL, MSG_DATA_0, EVT_STRG_CFG_CNG_BURL, NULL, MSG_DATA_COMMAND_ONLY, MSG_DATA_0_LEN); 
                        }
                    }
					break;


                //WIFI 
				case STRG_CMD_RD_WIFI_SIG:    //Wifi Signature
                    sys_nvs_load_all();			
                    strgSendMessage(strgRxMessage.srcAddr, MSG_DATA_32, strgRxMessage.msgCmd, NULL, g_nvs_setting_data.wifi.sig, MSG_DATA_32_LEN);
                    ESP_LOGI(STRG_TAG, "wifi sig read:%lX", g_nvs_setting_data.wifi.sig);
					break;

				case STRG_CMD_WR_WIFI_SSID_SIG:    //Wifi signature - SSID
                    sys_nvs_load_all();
                    tmpVal32 = g_nvs_setting_data.wifi.sig & 0xFFFF0000;
                    tmpVal32 = tmpVal32 | WIFI_SIG_SSID_SET;
                    g_nvs_setting_data.wifi.sig = tmpVal32;
                    sys_nvs_store_all();	
                    ESP_LOGI(STRG_TAG, "Wifi SSID sig written:%lX", g_nvs_setting_data.wifi.sig);
					break;	

				case STRG_CMD_WR_WIFI_PWD_SIG:    //Wifi signature - Password
                    sys_nvs_load_all();
                    tmpVal32 = g_nvs_setting_data.wifi.sig & 0x0000FFFF;
                    tmpVal32 = tmpVal32 | WIFI_SIG_PWD_SET;
                    g_nvs_setting_data.wifi.sig = tmpVal32;
                    sys_nvs_store_all();	
                    ESP_LOGI(STRG_TAG, "Wifi Password sig written:%lX", g_nvs_setting_data.wifi.sig);
					break;	

				case STRG_CMD_WR_WIFI_SIG:    //Wifi signature
                    sys_nvs_load_all();
                    g_nvs_setting_data.wifi.sig = strgRxMessage.msgData;
                    sys_nvs_store_all();	
                    ESP_LOGI(STRG_TAG, "config sig written:%lX", g_nvs_setting_data.wifi.sig);
					break;	

                //miscellaneous tests
                case STRG_CMD_LOG1_TEST:
                    strgLogI("This is an information message from the STRG module!");
                    break;    	

                case STRG_CMD_LOG2_TEST:
                    strgLogE("This is an error message from the STRG module!");
                    break;       

                //Commands to send data to the ble console
				// case BLE_CMD_RD_SO:    //Scale offset
                //     sys_nvs_load_all();	
                //     memset(strgTmpStr,0,sizeof(strgTmpStr));
                //     sprintf(strgTmpStr, "Scale Offset Stored:%ld", g_nvs_setting_data.scale.offset);
                //     strgLogR(strgTmpStr);		
				// 	break;

				// case BLE_CMD_RD_SCO:    //Scale calibration offset
                //     sys_nvs_load_all();	
                //     memset(strgTmpStr,0,sizeof(strgTmpStr));
                //     sprintf(strgTmpStr, "Scale Calibration Offset Stored:%ld", g_nvs_setting_data.scale.calOffset);
                //     strgLogR(strgTmpStr);		
				// 	break;

				// case BLE_CMD_RD_SG:    //Scale gain
                //     sys_nvs_load_all();
                //     memset(strgTmpStr,0,sizeof(strgTmpStr));
                //     sprintf(strgTmpStr, "Scale Gain Stored:%0.2f", g_nvs_setting_data.scale.gain / 100.0);
                //     strgLogR(strgTmpStr);		
				// 	break;

				// case BLE_CMD_RD_SS:    //Scale signature
                //     sys_nvs_load_all();	
                //     memset(strgTmpStr,0,sizeof(strgTmpStr));
                //     sprintf(strgTmpStr, "Scale Signature Stored:%lX", g_nvs_setting_data.scale.sig);
                //     strgLogR(strgTmpStr);		
				// 	break;

				case BLE_CMD_RD_SN:    //Serial Number
                    sys_nvs_load_all();	
                    memset(strgTmpStr,0,sizeof(strgTmpStr));
                    sprintf(strgTmpStr, "SN Stored:%s", g_nvs_setting_data.config.sn);
                    strgLogR(strgTmpStr);		
					break;

                // case SCTL_CMD_RD_APP_SSID:
                // case SCTL_CMD_RD_SSID:
				// case BLE_CMD_RD_SSID:  //ssid
                //     ESP_LOGI(STRG_TAG, "BLE_CMD_RD_SSID command received");
                //     sys_nvs_load_all();
                //     memset(strgTmpStr,0,sizeof(strgTmpStr));
                //     if (0 >= strlen(g_nvs_setting_data.wifi.ssid)) { //Set data to default
                //         memcpy(strgTmpStr,DEFAULT_SSID, strlen(DEFAULT_SSID));
                //     }
                //     else {
                //         memcpy(strgTmpStr,g_nvs_setting_data.wifi.ssid, strlen(g_nvs_setting_data.wifi.ssid));
                //     }
                //     if ( strgRxMessage.msgCmd == BLE_CMD_RD_SSID){
                //         strgLogR(strgTmpStr);
                //     }else if (strgRxMessage.msgCmd == SCTL_CMD_RD_SSID){
                //         strgSendMessage(MSG_ADDR_BLE, MSG_DATA_STR, STRG_CMD_RD_SSID, (uint8_t *)strgTmpStr, 0, strlen(strgTmpStr)+1);
                //     }else{
                //         strgSendMessage(MSG_ADDR_BLE, MSG_DATA_STR, STRG_CMD_RD_APP_SSID, (uint8_t *)strgTmpStr, 0, strlen(strgTmpStr)+1);
                //     }
				// 	break;
                    
                // case SCTL_CMD_RD_APP_PWD:
                // case SCTL_CMD_RD_PWD:    
				// case BLE_CMD_RD_PWD:   //password
                //     ESP_LOGI(STRG_TAG, "BLE_CMD_RD_PWD command received");
                //     sys_nvs_load_all();
                //     memset(strgTmpStr,0,sizeof(strgTmpStr));
                //     if (0 >= strlen(g_nvs_setting_data.wifi.pwd)) { //Set data to default
                //         memcpy(strgTmpStr,DEFAULT_PWD, strlen(DEFAULT_PWD));
                //     }
                //     else {
                //         memcpy(strgTmpStr,g_nvs_setting_data.wifi.pwd, strlen(g_nvs_setting_data.wifi.pwd));
                //     }					
                //     if ( strgRxMessage.msgCmd == BLE_CMD_RD_PWD){
                //         strgLogR(strgTmpStr);
                //     }else if (strgRxMessage.msgCmd == SCTL_CMD_RD_PWD){
                //         strgSendMessage(MSG_ADDR_BLE, MSG_DATA_STR, STRG_CMD_RD_PWD, (uint8_t *)strgTmpStr, 0, strlen(strgTmpStr)+1);
                //     }else{
                //         strgSendMessage(MSG_ADDR_BLE, MSG_DATA_STR, STRG_CMD_RD_APP_PWD, (uint8_t *)strgTmpStr, 0, strlen(strgTmpStr)+1);
                //     }
				// 	break;

                // case SCTL_CMD_RD_BASE_URL:
				// case BLE_CMD_RD_BASE_URL:  //base URL
                //     ESP_LOGI(STRG_TAG, "BLE_CMD_RD_BASE_URL command received");
                //     sys_nvs_load_all();
                //     memset(strgTmpStr,0,sizeof(strgTmpStr));
                //     if (0 >= strlen(g_nvs_setting_data.wifi.url)) { //Set data to default
                //         memcpy(strgTmpStr,DEFAULT_BASE_URL, strlen(DEFAULT_BASE_URL));
                //     }
                //     else {
                //         memcpy(strgTmpStr,g_nvs_setting_data.wifi.url, strlen(g_nvs_setting_data.wifi.url));
                //     }
                //     if ( strgRxMessage.msgCmd == BLE_CMD_RD_BASE_URL){
                //         strgLogR(strgTmpStr);
                //     }else{
                //         strgSendMessage(MSG_ADDR_BLE, MSG_DATA_STR, STRG_CMD_RD_BASE_URL, (uint8_t *)strgTmpStr, 0, strlen(strgTmpStr)+1);
                //     }
				// 	break;

                //Configurations
                
				// case BLE_CMD_RD_LOC_ID:    //location Id
                //     sys_nvs_load_all();	
                //     memset(strgTmpStr,0,sizeof(strgTmpStr));
                //     sprintf(strgTmpStr, "Location Id Stored:%d", g_nvs_setting_data.device.locationId);
                //     strgLogR(strgTmpStr);		
				// 	break;

				// case BLE_CMD_RD_PROV_ID:    //provider Id
                //     sys_nvs_load_all();	
                //     memset(strgTmpStr,0,sizeof(strgTmpStr));
                //     sprintf(strgTmpStr, "Provider Id Stored:%d", g_nvs_setting_data.device.providerId);
                //     strgLogR(strgTmpStr);		
				// 	break;

				case BLE_CMD_RD_WIFI_RET:    //wifi retries
                    sys_nvs_load_all();	
                    memset(strgTmpStr,0,sizeof(strgTmpStr));
                    sprintf(strgTmpStr, "wifi retries Stored:%d", g_nvs_setting_data.config.wifi_retries);
                    strgLogR(strgTmpStr);		
					break;

				// case BLE_CMD_RD_TANK_RET:    //tank retries
                //     sys_nvs_load_all();	
                //     memset(strgTmpStr,0,sizeof(strgTmpStr));
                //     sprintf(strgTmpStr, "tank retries Stored:%d", g_nvs_setting_data.config.tank_retries);
                //     strgLogR(strgTmpStr);		
				// 	break;

				case BLE_CMD_RD_SLP_DUR:    //sleep duration
                    sys_nvs_load_all();	
                    memset(strgTmpStr,0,sizeof(strgTmpStr));
                    sprintf(strgTmpStr, "sleep duration Stored:%d", g_nvs_setting_data.config.sleep_duration);
                    strgLogR(strgTmpStr);		
					break;

				case BLE_CMD_RD_UPL_DUR:    //upload duration
                    sys_nvs_load_all();	
                    memset(strgTmpStr,0,sizeof(strgTmpStr));
                    sprintf(strgTmpStr, "upload duration Stored:%d", g_nvs_setting_data.config.upload_duration);
                    strgLogR(strgTmpStr);		
					break;

				case BLE_CMD_RD_LBAT_THR:    //low battery threshold
                    sys_nvs_load_all();	
                    memset(strgTmpStr,0,sizeof(strgTmpStr));
                    sprintf(strgTmpStr, "low battery threshold Stored:%d", g_nvs_setting_data.config.low_batt_threshold);
                    strgLogR(strgTmpStr);		
					break;

				case BLE_CMD_RD_BRNO_THR:    //brownout threshold
                    sys_nvs_load_all();	
                    memset(strgTmpStr,0,sizeof(strgTmpStr));
                    sprintf(strgTmpStr, "brownout threshold Stored:%d", g_nvs_setting_data.config.brownout_threshold);
                    strgLogR(strgTmpStr);		
					break;

				// case BLE_CMD_RD_TANK_THR:    //tank threshold
                //     sys_nvs_load_all();	
                //     memset(strgTmpStr,0,sizeof(strgTmpStr));
                //     sprintf(strgTmpStr, "tank threshold Stored:%ld", g_nvs_setting_data.config.tank_threshold);
                //     strgLogR(strgTmpStr);		
				// 	break;

				// case BLE_CMD_RD_TANK_TMR:    //tank timer
                //     sys_nvs_load_all();	
                //     memset(strgTmpStr,0,sizeof(strgTmpStr));
                //     sprintf(strgTmpStr, "tank timer Stored:%ld", g_nvs_setting_data.config.tank_timer);
                //     strgLogR(strgTmpStr);		
				// 	break;
                    
                //OTA Configurations
                case STRG_CMD_RST_OTA_NVS:
                    sys_nvs_ota_init();
                    strgReturnStatus = STRG_RETURN_GOOD;
                    strgSendMessage(strgRxMessage.srcAddr, MSG_DATA_8, strgRxMessage.msgCmd, NULL, strgReturnStatus, MSG_DATA_8_LEN);                    
                    break;   

                case STRG_CMD_COPY_NVS_TO_OTA:
                    sys_nvs_load_all();
                    strgNvsToOta();
                    sys_nvs_ota_store_all();
                    strgReturnStatus = STRG_RETURN_GOOD;
                    strgSendMessage(strgRxMessage.srcAddr, MSG_DATA_8, strgRxMessage.msgCmd, NULL, strgReturnStatus, MSG_DATA_8_LEN);                    
                    break;   

                case STRG_CMD_COPY_OTA_TO_NVS:
                    sys_nvs_ota_load_all();
                    strgOtaToNvs();
                    sys_nvs_store_all();
                    strgReturnStatus = STRG_RETURN_GOOD;
                    strgSendMessage(strgRxMessage.srcAddr, MSG_DATA_8, strgRxMessage.msgCmd, NULL, strgReturnStatus, MSG_DATA_8_LEN);                    
                    break;   

                case STRG_CMD_RD_OTA_DATA:
                    sys_nvs_ota_load_all();
                    strgDisplayOtaData();
                    strgReturnStatus = STRG_RETURN_GOOD;
                    strgSendMessage(strgRxMessage.srcAddr, MSG_DATA_8, strgRxMessage.msgCmd, NULL, strgReturnStatus, MSG_DATA_8_LEN);                    
                    break;   

                case STRG_CMD_CLR_OTA_DATA:
                    strgClearOtaData();
                    strgReturnStatus = STRG_RETURN_GOOD;
                    strgSendMessage(strgRxMessage.srcAddr, MSG_DATA_8, strgRxMessage.msgCmd, NULL, strgReturnStatus, MSG_DATA_8_LEN);                    
                    break;  

                default:;

			}

        }

    }
 
}

//////////////////////////////////////////////////////////////////////////////////////////
//Local Functions:
//////////////////////////////////////////////////////////////////////////////////////////

//encryption
void encryptString(const char *toEncrypt, const char *key, unsigned char *result) {
    size_t toEncryptLen = strlen(toEncrypt);
    size_t keyLen = strlen(key);
    unsigned char iv = 0;

    //printf("toEncryptLen:%d, keyLen:%d\n",toEncryptLen, keyLen);

    if (toEncryptLen > keyLen) {
        for (size_t i = 0; i < toEncryptLen; i++) {
            result[i] = (unsigned char)(key[i % keyLen] ^ toEncrypt[i]);
            result[i] ^= iv;
        }
    } else {
        for (size_t i = 0; i < keyLen; i++) {
            if (i < toEncryptLen) {
                result[i] = (unsigned char)(key[i] ^ toEncrypt[i]);
            } else {
                result[i] = (unsigned char)(key[i] ^ 0);
            }
            result[i] ^= iv;
        }
    }
}

//void decryptString(const unsigned char *toDecrypt, const char *key, char *result) {
void decryptString(const unsigned char *toDecrypt, const char *key, unsigned char *result) {
    size_t keyLen = strlen(key);
    unsigned char iv = 0;

    //printf("decryptString - keyLen:%d, strlen(result):%d\n", keyLen, strlen((char *)result));

    for (size_t i = 0; i < keyLen; i++) {
        //if (i < strlen((char *)result)) {
        if (i < strlen((char *)toDecrypt)) {
            result[i] = (char)(key[i] ^ toDecrypt[i]);
        } else {
            result[i] = (char)(key[i] ^ 0);
        }
        result[i] ^= iv;
        //printf("%d, S:%02X, K:%02X D:%02X\n", i, (uint8_t)toDecrypt[i], key[i], (uint8_t)result[i]);
    }
}

//nvs flash:

void strgSetConfigDefaults(void)
{
    g_nvs_setting_data.config.sleep_duration  = DEFAULT_SLEEP_DURATION_MIN;
    g_nvs_setting_data.config.upload_duration = UPLOAD_DURATION;
    g_nvs_setting_data.config.low_batt_threshold = LOW_BATTERY_THRESHOLD;
    g_nvs_setting_data.config.brownout_threshold = BROWNOUT_THRESHOLD;
    g_nvs_setting_data.config.wifi_retries = WIFI_RETRIES;
    // g_nvs_setting_data.config.tank_retries = TANK_RETRIES;
    // g_nvs_setting_data.config.tank_threshold = TANK_THRESHOLD;
    // g_nvs_setting_data.config.tank_timer = TANK_TIMER;
    memset(g_nvs_setting_data.config.sn,0,sizeof(g_nvs_setting_data.config.sn));
    memcpy(g_nvs_setting_data.config.sn, DEFAULT_SN , sizeof(DEFAULT_SN));
    memset(g_nvs_setting_data.wifi.url,0,sizeof(g_nvs_setting_data.wifi.url));
    memcpy(g_nvs_setting_data.wifi.url, DEFAULT_BASE_URL , sizeof(DEFAULT_BASE_URL));
    g_nvs_setting_data.config.sig = CONFIG_SIG_DEFAULT;
    g_nvs_setting_data.config.sn_sig = CONFIG_SN_SIG_DEFAULT;
    ESP_LOGI(STRG_TAG,"Default SN:%s",g_nvs_setting_data.config.sn);
}

// void strgSetScaleDefaults(void)
// {
//     g_nvs_setting_data.scale.gain = 0;     
//     g_nvs_setting_data.scale.offset = 0.0;    
//     g_nvs_setting_data.scale.calOffset = 0.0;    
//     g_nvs_setting_data.scale.sig = SCALE_SIG_DEFAULT;
// }

void strgSetWifiDefaults(void)
{
    memset(g_nvs_setting_data.wifi.ssid,0,sizeof(g_nvs_setting_data.wifi.ssid));
    memcpy(g_nvs_setting_data.wifi.ssid, DEFAULT_SSID , sizeof(DEFAULT_SSID));
    memset(g_nvs_setting_data.wifi.pwd,0,sizeof(g_nvs_setting_data.wifi.pwd));	
    memcpy(g_nvs_setting_data.wifi.pwd, DEFAULT_PWD , sizeof(DEFAULT_PWD));	
    g_nvs_setting_data.wifi.sig = WIFI_SIG_DEFAULT;
}


// void strgSetDeviceDefaults(void)
// {
//     g_nvs_setting_data.device.providerId = DEFAULT_PROV_ID;     
//     g_nvs_setting_data.device.locationId = DEFAULT_LOC_ID;    
//     g_nvs_setting_data.device.sig = DEVICE_SIG_DEFAULT;
// }


// void strgSetHttpsDefaults(void)
// {
//     memset(g_nvs_setting_data.https.loxId,0,sizeof(g_nvs_setting_data.https.loxId));
//     memcpy(g_nvs_setting_data.https.loxId, DEFAULT_LOX_ID , sizeof(DEFAULT_LOX_ID));  
//     memset(g_nvs_setting_data.https.deviceToken,0,sizeof(g_nvs_setting_data.https.deviceToken));
//     memcpy(g_nvs_setting_data.https.deviceToken, DEFAULT_DEVICE_TOKEN , sizeof(DEFAULT_DEVICE_TOKEN));  
//     g_nvs_setting_data.https.sig = HTTPS_SIG_DEFAULT;
// }

void sys_nvs_reset_data(void)
{
    g_nvs_setting_data.data_version = NVS_DATA_VERSION;
    strgSetConfigDefaults();
    //strgSetScaleDefaults();
    strgSetWifiDefaults();
    //strgSetDeviceDefaults();
    //strgSetHttpsDefaults();
}

void sys_nvs_init(void)
{
    uint32_t nvs_ver;
    esp_err_t err;

    // Initialize NVS
    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_LOGI(STRG_TAG, "NVS partition was truncated and needs to be erased, retry nvs_flash_init");
        // NVS partition was truncated and needs to be erased, retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }

    if (err != ESP_OK)
        goto _LBL_END_;

    // Open NVS
    if (nvs_open(NVS_STORAGE_SPACENAME, NVS_READWRITE, &m_nvs_handle) != ESP_OK)
        goto _LBL_END_;

    // Get NVS data version
    nvs_get_u32(m_nvs_handle, NVS_VERSION_KEY_NAME, &nvs_ver);

    ESP_LOGI(STRG_TAG, "Data version in NVS storage: %ld ", nvs_ver);

    // Check NVS data version
    if (nvs_ver != NVS_DATA_VERSION)
    {
        ESP_LOGI(STRG_TAG, "NVS data version is different, all current data in NVS will be erased");
        // Erase NVS storage
        sys_nvs_factory_reset();

        // Set default value to g_nvs_setting_data struct
        sys_nvs_reset_data();

        // Store new data into NVS
        sys_nvs_store_all();

        // Update new NVS data version
        if (nvs_set_u32(m_nvs_handle, NVS_VERSION_KEY_NAME, g_nvs_setting_data.data_version) != ESP_OK)
            goto _LBL_END_;

        if (nvs_commit(m_nvs_handle) != ESP_OK)
            goto _LBL_END_;
    }
    else
    {
        // Update NVS data version into RAM
        ESP_LOGI(STRG_TAG, "Update NVS data version into RAM");
        g_nvs_setting_data.data_version = nvs_ver;

        // Load all data from NVS to RAM structure data
        sys_nvs_load_all();
    }

    return;

_LBL_END_:

    ESP_LOGE(STRG_TAG, "NVS storage init failed");
    //bsp_error_handler(BSP_ERR_NVS_INIT);
	ESP_LOGE(STRG_TAG, "BS_ERROR: %d", BSP_ERR_NVS_INIT);
}


void sys_nvs_deinit(void)
{
    nvs_close(m_nvs_handle);
}


void sys_nvs_store_all(void)
{
    uint16_t sizeof_nvs_data_list;
    uint32_t addr;
    void *p_data;
    size_t var_len;

    // Automatically looking into the nvs data list in order to get data information and store to NVS
    sizeof_nvs_data_list = (uint16_t)(sizeof(nvs_data_list) / sizeof(nvs_data_list[0]));
    addr = (uint32_t)&g_nvs_setting_data;

    for (uint_fast16_t i = 0; i < sizeof_nvs_data_list; i++)
    {
        p_data = (void *)(addr + nvs_data_list[i].offset);
        var_len = (size_t)nvs_data_list[i].size;

        if (nvs_set_blob(m_nvs_handle, nvs_data_list[i].key, p_data, var_len) != ESP_OK)
            goto _LBL_END_;

        // Commit written value
        if (nvs_commit(m_nvs_handle) != ESP_OK)
            goto _LBL_END_;
    }

    return;

_LBL_END_:

    ESP_LOGE(STRG_TAG, "NVS store all data error");
    //bsp_error_handler(BSP_ERR_NVS_COMMUNICATION);
	ESP_LOGE(STRG_TAG, "BS_ERROR: %d", BSP_ERR_NVS_COMMUNICATION);
	
}


void sys_nvs_load_all(void)
{
    uint16_t sizeof_nvs_data_list;
    uint32_t addr;
    void *p_data;
    size_t var_len;

    if (nvs_get_u32(m_nvs_handle, NVS_VERSION_KEY_NAME, &g_nvs_setting_data.data_version) != ESP_OK)
        ESP_LOGE(STRG_TAG,"Error in getting nvs data version"); 

    // Load variable data from ID List Table
    addr = (uint32_t)&g_nvs_setting_data;
    sizeof_nvs_data_list = (uint16_t)(sizeof(nvs_data_list) / sizeof(nvs_data_list[0]));

    //ESP_LOGI(STRG_TAG,"size of data list:%d, address:%ld", sizeof_nvs_data_list, addr);

    //ESP_LOGI(STRG_TAG,"size of data list:%d", (uint16_t)(sizeof(nvs_data_list)));
    //ESP_LOGI(STRG_TAG,"size of data list[0]:%d", (uint16_t)(sizeof(nvs_data_list[0])));
    //ESP_LOGI(STRG_TAG,"size of data list[1]:%d", (uint16_t)(sizeof(nvs_data_list[1])));
    //ESP_LOGI(STRG_TAG,"size of data list[2]:%d", (uint16_t)(sizeof(nvs_data_list[2])));
    //ESP_LOGI(STRG_TAG,"size of data list[3]:%d", (uint16_t)(sizeof(nvs_data_list[3])));
    //ESP_LOGI(STRG_TAG,"size of data list[4]:%d", (uint16_t)(sizeof(nvs_data_list[4])));

    for (uint_fast16_t i = 0; i < sizeof_nvs_data_list; i++)
    {
        p_data = (void *)(addr + nvs_data_list[i].offset);
        var_len = (size_t)nvs_data_list[i].size;

        //ESP_LOGI(STRG_TAG,"var_len:%d, p_data:%ld", var_len, (uint32_t)p_data);

        //ESP_LOGE(STRG_TAG, "Key: %s", nvs_data_list[i].key);

        if (nvs_get_blob(m_nvs_handle, nvs_data_list[i].key, p_data, &var_len) != ESP_OK)
        {
            ESP_LOGE(STRG_TAG, "NVS load all data error");
            //bsp_error_handler(BSP_ERR_NVS_COMMUNICATION);
			ESP_LOGE(STRG_TAG, "BS_ERROR: %d", BSP_ERR_NVS_COMMUNICATION);
        }
    }
}


void sys_nvs_factory_reset(void)
{
    if (nvs_erase_all(m_nvs_handle) != ESP_OK)
        goto _LBL_END_;

    if (nvs_commit(m_nvs_handle) != ESP_OK)
        goto _LBL_END_;

    return;

_LBL_END_:

    ESP_LOGE(STRG_TAG, "NVS factory reset failed");
    //bsp_error_handler(BSP_ERR_NVS_COMMUNICATION);
	ESP_LOGE(STRG_TAG, "BS_ERROR: %d", BSP_ERR_NVS_COMMUNICATION);
}

//otadata flash:

void sys_nvs_ota_factory_reset(void)
{
    if (nvs_erase_all(m_nvs_ota_handle) != ESP_OK)
        goto _LBL_END_;

    if (nvs_commit(m_nvs_ota_handle) != ESP_OK)
        goto _LBL_END_;

    return;

_LBL_END_:

    ESP_LOGE(STRG_TAG, "NVS OTA factory reset failed");
    //bsp_error_handler(BSP_ERR_NVS_COMMUNICATION);
	ESP_LOGE(STRG_TAG, "BS_ERROR: %d", BSP_ERR_NVS_COMMUNICATION);
}

/*
void sys_nvs_ota_reset_data(void)
{
    g_nvs_setting_data.data_version = NVS_DATA_VERSION;
    strgSetConfigDefaults();
    strgSetScaleDefaults();
    strgSetWifiDefaults();
    strgSetDeviceDefaults();
    strgSetHttpsDefaults();
}
*/

void sys_nvs_ota_init(void)
{
    uint32_t nvs_ota_ver;
    esp_err_t err;

    // Initialize NVS
    err = nvs_flash_init_partition(OTA_PARTITION_LABEL);

    ESP_LOGI(STRG_TAG, "nvs_flash_init_partition err:%d", err);

    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_LOGI(STRG_TAG, "NVS OTA partition was truncated and needs to be erased, retry nvs_flash_init");
        // NVS partition was truncated and needs to be erased, retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase_partition(OTA_PARTITION_LABEL));
        err = nvs_flash_init_partition(OTA_PARTITION_LABEL);
    }

    if (err != ESP_OK)
        goto _LBL_END_;

    // Open NVS
    if (nvs_open_from_partition(OTA_PARTITION_LABEL, NVS_OTA_STORAGE_SPACENAME, NVS_READWRITE, &m_nvs_ota_handle) != ESP_OK)
        goto _LBL_END_;

    // Get NVS data version
    nvs_get_u32(m_nvs_ota_handle, NVS_OTA_VERSION_KEY_NAME, &nvs_ota_ver);

    ESP_LOGI(STRG_TAG, "OTA Data version in NVS OTA storage: %ld ", nvs_ota_ver);

    // Check NVS data version
    if (nvs_ota_ver != NVS_OTA_DATA_VERSION)
    {
        ESP_LOGI(STRG_TAG, "NVS OTA data version is different, all current data in NVS OTA will be erased");

        // Erase NVS storage
        sys_nvs_ota_factory_reset();

        // Set default value to g_nvs_ota_setting_data struct
        //sys_nvs_reset_data();
        strgClearOtaData();

        // Store new data into NVS
        sys_nvs_ota_store_all();

        g_nvs_ota_setting_data.ota_data_version = NVS_OTA_DATA_VERSION;

        // Update new NVS data version
        if (nvs_set_u32(m_nvs_ota_handle, NVS_OTA_VERSION_KEY_NAME, g_nvs_ota_setting_data.ota_data_version) != ESP_OK)
            goto _LBL_END_;

        if (nvs_commit(m_nvs_ota_handle) != ESP_OK)
            goto _LBL_END_;
    }
    else
    {
        // Update NVS data version into RAM
        ESP_LOGI(STRG_TAG, "Update NVS OTA data version into RAM");
        g_nvs_ota_setting_data.ota_data_version = nvs_ota_ver;

        // Load all data from NVS to RAM structure data
        sys_nvs_ota_load_all();
    }

    return;

_LBL_END_:

    ESP_LOGE(STRG_TAG, "NVS OTA storage init failed");
	ESP_LOGE(STRG_TAG, "BS_ERROR: %d", BSP_ERR_NVS_INIT);
}


void sys_nvs_ota_deinit(void)
{
    nvs_close(m_nvs_ota_handle);
}


void sys_nvs_ota_store_all(void)
{
    uint16_t sizeof_nvs_ota_data_list;
    uint32_t addr;
    void *p_data;
    size_t var_len;

    // Automatically looking into the nvs data list in order to get data information and store to NVS
    sizeof_nvs_ota_data_list = (uint16_t)(sizeof(nvs_ota_data_list) / sizeof(nvs_ota_data_list[0]));
    addr = (uint32_t)&g_nvs_ota_setting_data;

    //ESP_LOGI(STRG_TAG,"size of data list:%d, address:%ld", sizeof_nvs_ota_data_list, addr);

    for (uint_fast16_t i = 0; i < sizeof_nvs_ota_data_list; i++)
    {
        p_data = (void *)(addr + nvs_ota_data_list[i].offset);
        var_len = (size_t)nvs_ota_data_list[i].size;

        //ESP_LOGI(STRG_TAG,"data list:%s", (char *)p_data);

        if (nvs_set_blob(m_nvs_ota_handle, nvs_ota_data_list[i].key, p_data, var_len) != ESP_OK)
            goto _LBL_END_;

        // Commit written value
        if (nvs_commit(m_nvs_ota_handle) != ESP_OK)
            goto _LBL_END_;
    }

    return;

_LBL_END_:

    ESP_LOGE(STRG_TAG, "NVS OTA store all data error");
	ESP_LOGE(STRG_TAG, "BS_ERROR: %d", BSP_ERR_NVS_COMMUNICATION);
	
}


void sys_nvs_ota_load_all(void)
{
    uint16_t sizeof_nvs_ota_data_list;
    uint32_t addr;
    void *p_data;
    size_t var_len;

    esp_err_t err;

    //load version
    nvs_get_u32(m_nvs_ota_handle, NVS_OTA_VERSION_KEY_NAME, &g_nvs_ota_setting_data.ota_data_version);
    nvs_get_u32(m_nvs_ota_handle, NVS_OTA_CONFIG_KEY_NAME, &g_nvs_ota_setting_data.ota_data_sig);
    

    // Load variable data from ID List Table
    addr = (uint32_t)&g_nvs_ota_setting_data;
    sizeof_nvs_ota_data_list = (uint16_t)(sizeof(nvs_ota_data_list) / sizeof(nvs_ota_data_list[0]));
    
    for (uint_fast16_t i = 0; i < sizeof_nvs_ota_data_list; i++)
    {
        p_data = (void *)(addr + nvs_ota_data_list[i].offset);
        var_len = (size_t)nvs_ota_data_list[i].size;

        //ESP_LOGI(STRG_TAG,"var_len:%d, p_data:%ld", var_len, (uint32_t)p_data);

        err = nvs_get_blob(m_nvs_ota_handle, nvs_ota_data_list[i].key, p_data, &var_len);
        //if (nvs_get_blob(m_nvs_ota_handle, nvs_ota_data_list[i].key, p_data, &var_len) != ESP_OK)
        if (err != ESP_OK)
        {
            ESP_LOGE(STRG_TAG, "NVS OTA load all data error:%d", err);
			ESP_LOGE(STRG_TAG, "BS_ERROR: %d", BSP_ERR_NVS_COMMUNICATION);
            //ESP_LOGE(STRG_TAG, "Key: %s", nvs_ota_data_list[i].key);
            g_nvs_ota_setting_data.ota_data_sig = 0;
        }
        else
        {
            g_nvs_ota_setting_data.ota_data_sig = OTA_CONFIG_GOOD;
        }
        nvs_set_u32(m_nvs_ota_handle, NVS_OTA_CONFIG_KEY_NAME, g_nvs_ota_setting_data.ota_data_sig);

    }
}


//////////////////////////////////////////////////////////////////////////////////////////
//SPIFFS
//////////////////////////////////////////////////////////////////////////////////////////

uint32_t strgCirQueueSize(uint8_t queueId)
{   
    uint32_t retVal = 0;

    switch (queueId)
    {
        case STRG_DATA_QUEUE:
            retVal = spiffs_circular_queue_size(&cq_data);
            break;
        case STRG_LOG_QUEUE:
            retVal = spiffs_circular_queue_size(&cq_log);
            break;
        default:
            ESP_LOGE(STRG_TAG, "strgCirQueueSize - Undefined SPIFFS queue:%d",queueId);    
            
    }
    return retVal;
}

uint32_t strgQueueAvailSpace(uint8_t queueId)
{
    uint32_t retVal = 0;

    switch (queueId)
    {
        case STRG_DATA_QUEUE:
            retVal = spiffs_circular_queue_available_space(&cq_data);
            break;
        case STRG_LOG_QUEUE:
            retVal = spiffs_circular_queue_available_space(&cq_log);
            break;
        default:
            ESP_LOGE(STRG_TAG, "strgQueueAvailSpace - Undefined SPIFFS queue:%d",queueId);    
            
    }
    return retVal;
}

uint16_t strgQueueCount(uint8_t queueId)
{
    uint16_t retVal = 0;

    switch (queueId)
    {
        case STRG_DATA_QUEUE:
            retVal = spiffs_circular_queue_get_count(&cq_data);
            break;
        case STRG_LOG_QUEUE:
            retVal = spiffs_circular_queue_get_count(&cq_log);
            break;
        default:
            ESP_LOGE(STRG_TAG, "strgQueueCount - Undefined SPIFFS queue:%d",queueId);    
            
    }
    return retVal;
}

uint32_t strgQueueFsize(uint8_t queueId)
{
    uint32_t retVal = 0;

    switch (queueId)
    {
        case STRG_DATA_QUEUE:
            retVal = spiffs_circular_queue_get_file_size(&cq_data);
            break;
        case STRG_LOG_QUEUE:
            retVal = spiffs_circular_queue_get_file_size(&cq_log);
            break;
        default:
            ESP_LOGE(STRG_TAG, "strgQueueFsize - Undefined SPIFFS queue:%d",queueId);    
            
    }
    return retVal;
}

uint8_t strgQueuePercent(uint8_t queueId)
{
    uint8_t retVal = 0;
    float tmpAvail = 0.0;
    float tmpFloat = 0.0;

    switch (queueId)
    {
        case STRG_DATA_QUEUE:
            tmpAvail = (float)strgQueueAvailSpace(STRG_DATA_QUEUE);
            tmpFloat = (1 - ((DATA_QUEUE_MAX_SIZE - tmpAvail) / DATA_QUEUE_MAX_SIZE)) * 100.0;
            retVal = (uint8_t)tmpFloat;
            //ESP_LOGI(STRG_TAG,"max size:%d, Avail:%f, percent:%f, ret vale:%d", DATA_QUEUE_MAX_SIZE, tmpAvail, tmpFloat, retVal);
            break;
        case STRG_LOG_QUEUE:
            tmpFloat = (1 - ((LOG_QUEUE_MAX_SIZE - strgQueueAvailSpace(STRG_LOG_QUEUE)) / LOG_QUEUE_MAX_SIZE)) * 100.0;
            retVal = (uint8_t)tmpFloat;
            break;
        default:
            ESP_LOGE(STRG_TAG, "strgQueueFsize - Undefined SPIFFS queue:%d",queueId);    
            
    }
    return retVal;
}


uint8_t strgQueueEmpty(uint8_t queueId)
{
    uint8_t retVal = 0;

    switch (queueId)
    {
        case STRG_DATA_QUEUE:
            retVal = spiffs_circular_queue_is_empty(&cq_data);
            break;
        case STRG_LOG_QUEUE:
            retVal = spiffs_circular_queue_is_empty(&cq_log);
            break;
        default:
            ESP_LOGE(STRG_TAG, "strgQueueEmpty - Undefined SPIFFS queue:%d",queueId);              
    }
    return retVal;
}

uint8_t strgQueueFree(uint8_t queueId)
{
    uint8_t retVal = 0;

    switch (queueId)
    {
        case STRG_DATA_QUEUE:
            retVal = spiffs_circular_queue_free(&cq_data, 1);
            break;
        case STRG_LOG_QUEUE:
            retVal = spiffs_circular_queue_free(&cq_log, 1);
            break;
        default:
            ESP_LOGE(STRG_TAG, "strgQueueEmpty - Undefined SPIFFS queue:%d",queueId);              
    }
    return retVal;
}


void strgQueueInfo(uint8_t queueId)
{
    switch(queueId)
    {
        case STRG_DATA_QUEUE:
            ESP_LOGI(STRG_TAG, "Data Queue Available space: %ld", strgQueueAvailSpace(STRG_DATA_QUEUE));
            ESP_LOGI(STRG_TAG, "Data Queue Size: %ld", strgCirQueueSize(STRG_DATA_QUEUE));
            ESP_LOGI(STRG_TAG, "Data Queue count: %d", strgQueueCount(STRG_DATA_QUEUE));
            ESP_LOGI(STRG_TAG, "Data Queue File Size: %ld", strgQueueFsize(STRG_DATA_QUEUE));
            break;
        case  STRG_LOG_QUEUE:
            ESP_LOGI(STRG_TAG, "Log Queue Available space: %ld", strgQueueAvailSpace(STRG_LOG_QUEUE));
            ESP_LOGI(STRG_TAG, "Log Queue Size: %ld", strgCirQueueSize(STRG_LOG_QUEUE));
            ESP_LOGI(STRG_TAG, "Log Queue count: %d", strgQueueCount(STRG_LOG_QUEUE));
            ESP_LOGI(STRG_TAG, "Log Queue File Size: %ld", strgQueueFsize(STRG_LOG_QUEUE));
            break;
        default:
            ESP_LOGE(STRG_TAG, "strgQueueInfo - Undefined SPIFFS queue:%d",queueId);              
    }
}


/// @brief  This function sends the info log message to the BLE queue reguardless of 
///    the mobile device connected and/or the Log notification enabled. It will also send
///    the log message to the terminal.
/// @param strPtr 
void strgLogR(char * strPtr)
{
    ESP_LOGI(STRG_TAG, "%s", strPtr);
    strgLog(strPtr, true, false);
}

/// @brief  This function sends the info log message to the BLE queue reguardless of 
///    the mobile device connected and/or the Log notification enabled. It will also send
///    the log message to the terminal.
/// @param strPtr 
void strgLogIF(char * strPtr)
{
    ESP_LOGI(STRG_TAG, "%s", strPtr);
    strgLog(strPtr, true, true);
}

/// @brief  This function sends the error log message to the BLE queue reguardless of 
///    the mobile device connected and/or the Log notification enabled. It will also send
///    the log message to the terminal.
void strgLogEF(char * strPtr)
{
    ESP_LOGE(STRG_TAG, "%s", strPtr);
    strgLog(strPtr, true, true);
}

/// @brief  This function send the info log message to the Mobile device and to the 
///    terminal.
/// @param strPtr 
void strgLogI(char * strPtr)
{
    ESP_LOGI(STRG_TAG, "%s", strPtr);
    strgLog(strPtr, false, true);
}

/// @brief This function send the error log message to the Mobile device and to the 
///    terminal. 
/// @param strPtr 
void strgLogE(char * strPtr)
{
    ESP_LOGE(STRG_TAG, "%s", strPtr);
    strgLog(strPtr, false, true);
}

/// @brief This function sends a message to the BLE module to send out a log message
///   to the mobile device via BLE. Mobile device has to be connected and the
///   log notifications (RX Characteristics) enabled.
/// @param strPtr - message to be sent to phone.
/// @param forced - If true, message is forced in the log queue.
/// @param printTag - If true the tag is printed with the message. 
void strgLog(char * strPtr, bool forced, bool printTag)
{
    static uint8_t printIndex = 0;
    RTOS_message_t modPrintMsg;
    char tmpStr[STRING_MAX_LOG_CHARACTERS];
    static char modStr[STRG_STRING_MAX_ARRAY_LOG_ELEMENTS][STRING_MAX_LOG_CHARACTERS];

    //copy string into next available string array
    //memcpy(modStr[printIndex], strPtr, strlen(strPtr)+1);
    memcpy(tmpStr, strPtr, strlen(strPtr)+1);
	if (printTag){
        sprintf(modStr[printIndex], "%s:%s", STRG_TAG, tmpStr);
    }else{
        sprintf(modStr[printIndex], "%s", tmpStr);
    }

    //form message to print out string
    modPrintMsg.srcAddr         =  MSG_ADDR_STRG; 
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
    if (printIndex >= STRG_STRING_MAX_ARRAY_LOG_ELEMENTS)
    {
        printIndex = 0;
    }
}

/// @brief Function to create message and sent it to the designated destination.
/// @param dstAddr - Task to send message to. (see message address enum)
/// @param msgType - Type of message. (see message type enum)
/// @param msgCmd  - Message associated with the message. (see specific task for command enum)
/// @param msgDataPtr - A pointer to data greater than 32 bits if used. Null otherwise.
/// @param msgData    - Data that is 32 bits or less. Set to 0 if not used.
/// @param msgDataLen - Data length of either msgData or data pointed to msgDataPtr (in bytes).
void strgSendMessage(uint8_t dstAddr, uint8_t msgType, uint16_t msgCmd, uint8_t * msgDataPtr, uint32_t msgData, uint32_t msgDataLen)
{  
    static uint8_t msgIndex = 0;
    RTOS_message_t sendRtosMsg;
    static char sendMsgArray[STRG_MESSAGE_MAX_ARRAY_ELEMENTS][STRG_STRING_MAX_ARRAY_CHARACTERS];

    if ((msgType == MSG_DATA_0) || (msgType == MSG_DATA_8) || (msgType == MSG_DATA_16) || (msgType == MSG_DATA_32))
    {
       sendRtosMsg.msgDataPtr = NULL;
       switch(msgType)
       {
            case MSG_DATA_0:    sendRtosMsg.msgDataLen =  MSG_DATA_0_LEN;    break;
            case MSG_DATA_8:    sendRtosMsg.msgDataLen =  MSG_DATA_8_LEN;    break;
            case MSG_DATA_16:   sendRtosMsg.msgDataLen =  MSG_DATA_16_LEN;   break;
            case MSG_DATA_32:   sendRtosMsg.msgDataLen =  MSG_DATA_32_LEN;   break;
            default:            sendRtosMsg.msgDataLen =  MSG_DATA_0_LEN;    break;
       }
    } 
    else
    {
        memcpy(sendMsgArray[msgIndex], msgDataPtr, msgDataLen);
        sendRtosMsg.msgDataPtr = (uint8_t *)sendMsgArray[msgIndex];
        sendRtosMsg.msgDataLen = msgDataLen;
    }
    sendRtosMsg.srcAddr = MSG_ADDR_STRG;
    sendRtosMsg.dstAddr = dstAddr;
    sendRtosMsg.msgRef = msg_getMsgRef();
    sendRtosMsg.msgTimeStamp = sys_getMsgTimeStamp();
    sendRtosMsg.msgType = msgType;
    sendRtosMsg.msgCmd = msgCmd;
    sendRtosMsg.msgData = msgData;
 
    //send message
    xQueueSend(dispatcherQueueHandle,&sendRtosMsg,0);

    //Set next array index
    msgIndex++;
    if (msgIndex >= STRG_MESSAGE_MAX_ARRAY_ELEMENTS)
    {
        msgIndex = 0;
    }

}


//////////////////////////////////////////////////////////////////////////////////////////
//SPIFFS - new code
//////////////////////////////////////////////////////////////////////////////////////////

bool strg2MountSpiffs(void)
{
    ESP_LOGI(STRG_TAG, "Initializing SPIFFS - New code");

    // Use settings defined above to initialize and mount SPIFFS filesystem.
    // Note: esp_vfs_spiffs_register is an all-in-one convenience function.
    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(STRG_TAG, "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(STRG_TAG, "Failed to find SPIFFS partition");
        } else {
            ESP_LOGE(STRG_TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return false;
    }
    return true;
}

bool strg2DismountSpiffs(void)
{
    esp_err_t ret = esp_vfs_spiffs_unregister(conf.partition_label);
    ESP_LOGI(STRG_TAG, "SPIFFS unmounted");
    if (ret == ESP_OK){
        return true;
    }else{
        return false;
    }
}

//-----------------------------------------
static void list(char *path, char *match) {

    DIR *dir = NULL;
    struct dirent *ent;
    char type;
    char size[20];
    char tpath[255];
    char tbuffer[80];
    struct stat sb;
    struct tm *tm_info;
    char *lpath = NULL;
    int statok;

    printf("\nList of Directory [%s]\n", path);
    printf("-----------------------------------\n");
    // Open directory
    dir = opendir(path);
    if (!dir) {
        printf("Error opening directory\n");
        return;
    }

    // Read directory entries
    uint64_t total = 0;
    int nfiles = 0;
    printf("T  Size      Date/Time         Name\n");
    printf("-----------------------------------\n");
    while ((ent = readdir(dir)) != NULL) {
        sprintf(tpath, path);
        if (path[strlen(path)-1] != '/') strcat(tpath,"/");
        strcat(tpath,ent->d_name);
        tbuffer[0] = '\0';

        //if ((match == NULL) || (fnmatch(match, tpath, (FNM_PERIOD)) == 0)) 
        if (match == NULL)  
        {
            // Get file stat
            statok = stat(tpath, &sb);

            if (statok == 0) {
                tm_info = localtime(&sb.st_mtime);
                strftime(tbuffer, 80, "%d/%m/%Y %R", tm_info);
            }
            else sprintf(tbuffer, "                ");

            if (ent->d_type == DT_REG) {
                type = 'f';
                nfiles++;
                if (statok) strcpy(size, "       ?");
                else {
                    total += sb.st_size;
                    if (sb.st_size < (1024*1024)) sprintf(size,"%8d", (int)sb.st_size);
                    else if ((sb.st_size/1024) < (1024*1024)) sprintf(size,"%6dKB", (int)(sb.st_size / 1024));
                    else sprintf(size,"%6dMB", (int)(sb.st_size / (1024 * 1024)));
                }
            }
            else {
                type = 'd';
                strcpy(size, "       -");
            }

            printf("%c  %s  %s  %s\r\n",
                type,
                size,
                tbuffer,
                ent->d_name
            );
        }
    }
    if (total) {
        printf("-----------------------------------\n");
    	if (total < (1024*1024)) printf("   %8d", (int)total);
    	else if ((total/1024) < (1024*1024)) printf("   %6dKB", (int)(total / 1024));
    	else printf("   %6dMB", (int)(total / (1024 * 1024)));
    	printf(" in %d file(s)\n", nfiles);
    }
    printf("-----------------------------------\n");
    closedir(dir);
    free(lpath);

	//uint32_t tot=0, used=0;
	size_t tot=0, used=0;
    esp_spiffs_info(NULL, &tot, &used);
    printf("SPIFFS: free %d KB of %d KB\n", (tot-used) / 1024, tot / 1024);
    printf("-----------------------------------\n\n");
}

/// @brief returns the number of files in a dirtectory
/// @param path - spiffs path
/// @return number of files in a dirtectory
uint8_t strgFileCount(char *path)
{
    DIR *dir = NULL;
    struct dirent *ent;

    // Open directory
    dir = opendir(path);
    if (!dir) {
        printf("Error opening directory\n");
        return 0;
    }

    int nfiles = 0;
    while ((ent = readdir(dir)) != NULL) 
    {
        if (ent->d_type == DT_REG)
        {
            nfiles++;
        }       
    }
    closedir(dir);
    return nfiles;
}

/// @brief Returns the Next file name in the directory
/// @param path Directory were the files are stored
/// @param fileName The Next file name in the directory
/// @return Number of files in the directory
static bool strgNextFile(char *path, char *fileName, uint8_t fileNameLen) 
{
    DIR *dir = NULL;
    struct dirent *ent;
    char tpath[255];

    // Open directory
    dir = opendir(path);
    if (!dir) {
        ESP_LOGE(STRG_TAG,"Error opening directory:%s", path);
        return false;
    }

    // Read directory entries
    if ((ent = readdir(dir)) != NULL) {
        if (path[strlen(path)-1] != '/') strcat(tpath,"/");
        strcat(tpath,ent->d_name);
        memset(fileName,0,fileNameLen);
        memcpy(fileName,ent->d_name,strlen(ent->d_name));
        //printf("Next File Name: %s\r\n", ent->d_name);
    }else{
        return false;
    }
    
    closedir(dir);
    return true;
}


uint8_t strgQueueInit(uint8_t queueId)
{
    uint8_t retVal = 0;
    struct stat sb;
    FILE *fd = NULL;

    switch (queueId)
    {
        case STRG_DATA_QUEUE:
            // stat returns 0 upon succes (file exists) and -1 on failure (does not)
            if (stat("/spiffs/data", &sb) < 0) {
                if ((fd = fopen("/spiffs/data", "w"))) {
                    ESP_LOGI(STRG_TAG, "Directory /spiffs/data was created");
                    retVal = 1;
                }else{
                    ESP_LOGE(STRG_TAG, "Directory /spiffs/data was not created!");
                }  
            } 
            else
            {
                if ((fd = fopen("/spiffs/data", "r+b"))) {
                    ESP_LOGI(STRG_TAG, "Directory /spiffs/data was opened");
                    retVal = 1;
                }else{
                    ESP_LOGE(STRG_TAG, "Directory /spiffs/data was not opened!");
                }  
            }    
            fclose(fd);           
            break;
        case STRG_LOG_QUEUE:
            // stat returns 0 upon succes (file exists) and -1 on failure (does not)
            if (stat("/spiffs/log", &sb) < 0) {
                if ((fd = fopen("/spiffs/log", "w"))) {
                    ESP_LOGI(STRG_TAG, "Directory /spiffs/log was created");
                    retVal = 1;
                }else{
                    ESP_LOGE(STRG_TAG, "Directory /spiffs/log was not created!");
                }  
            } 
            else
            {
                if ((fd = fopen("/spiffs/log", "r+b"))) {
                    ESP_LOGI(STRG_TAG, "Directory /spiffs/log was opened");
                    retVal = 1;
                }else{
                    ESP_LOGE(STRG_TAG, "Directory /spiffs/log was not opened!");
                }  
            }    
            fclose(fd);           
            break;
        default:
            ESP_LOGE(STRG_TAG, "strgQueueInit - Undefined SPIFFS queue:%d",queueId);               
    }
    return retVal;
}

bool strgWriteQueue(uint8_t queueId, char * queueFileName, char * queueData)
{
    uint16_t strLenValue = 0;
    bool retVal = false;
    char tmpStr[BLE_STRING_MAX_ARRAY_CHARACTERS];
    //uint8_t tmpStrLen = 0;
    //uint32_t bytesTotal = 0, bytesUsed = 0, bytesFree = 0;
    size_t bytesTotal = 0, bytesUsed = 0, bytesFree = 0;

    FILE* writeFile;

    strLenValue = strlen(queueData) + 1;
    esp_spiffs_info(NULL, &bytesTotal, &bytesUsed);
    bytesFree = bytesTotal-bytesUsed;
    ESP_LOGI(STRG_TAG,"SPIFFS: free %d Bytes of %d Bytes", bytesFree, bytesTotal);

    switch (queueId)
    {
        case STRG_DATA_QUEUE:       
            if ((strLenValue + MIN_DATA_QUEUE_AVAILABLE)  >= bytesFree)
            {
                //strgReadQueue(STRG_DATA_QUEUE, &tmpStr, &tmpStrLen);
                //ESP_LOGI(STRG_TAG, "Data Queue full, digarding last data:%s", tmpStr) ;
                ESP_LOGI(STRG_TAG, "Data Queue full, digarding last data entry");
            }

            sprintf(tmpStr,"%s/%s",DATA_QUEUE_DIR,queueFileName);
            writeFile = fopen(tmpStr, "w");
            if (writeFile == NULL) {
                ESP_LOGE(STRG_TAG, "Failed to open data file for writing");
            }
            else
            {
                fprintf(writeFile, queueData);
                fclose(writeFile);
                ESP_LOGI(STRG_TAG, "Data file written");
                retVal = true;
            }
            break;

        case STRG_LOG_QUEUE:
            if ((strLenValue + MIN_LOG_QUEUE_AVAILABLE)  >= bytesFree)
            {
            //    strgReadQueue(STRG_LOG_QUEUE, &tmpStr, &tmpStrLen);
            //    ESP_LOGI(STRG_TAG, "Log Queue full, digarding last data:%s", tmpStr) ;
                ESP_LOGI(STRG_TAG, "Log Queue full, digarding last data entry");
            }

            sprintf(tmpStr,"%s/%s",LOG_QUEUE_DIR,queueFileName);
            writeFile = fopen(tmpStr, "w");
            if (writeFile == NULL) {
                ESP_LOGE(STRG_TAG, "Failed to open log file for writing");
            }
            else{
                fprintf(writeFile, queueData);
                fclose(writeFile);
                ESP_LOGI(STRG_TAG, "Log file written");
                retVal = true;
            }

            break;
        default:
            ESP_LOGE(STRG_TAG, "strgWriteQueue - Undefined SPIFFS queue:%d",queueId);              
    }
    return retVal;
}

uint8_t strgReadQueue(uint8_t queueId, char * queueData, uint16_t * queueDataLen)
{
    uint8_t retVal = 0;
    FILE* readFile;
    bool strgFilesInDir = false;
    char strgNextFileName[FILE_NAME_MAX_CHARACTERS];
    char strgReadDirFileName[DIR_AND_FILE_NAME_MAX_CHARACTERS];

    switch (queueId)
    {
        case STRG_DATA_QUEUE:
            if (strgFileCount(DATA_QUEUE_DIR) > 0)
            {
                memset(strgNextFileName,0,sizeof(strgNextFileName));

                strgFilesInDir = strgNextFile(DATA_QUEUE_DIR, &strgNextFileName[0], sizeof(strgNextFileName));
                if( strgFilesInDir)
                {
                    sprintf(strgReadDirFileName,"%s/%s",DATA_QUEUE_DIR,strgNextFileName);
                    ESP_LOGI(STRG_TAG,"Next file name:%s",strgReadDirFileName);
                }
                else{
                    ESP_LOGI(STRG_TAG,"No files found!");
                    break;
                }

                ESP_LOGI(STRG_TAG, "Reading file");
                readFile = fopen(strgReadDirFileName, "r");
                if (readFile == NULL) {
                    ESP_LOGE(STRG_TAG, "Failed to open file for reading");
                    break;
                }
                fgets(queueData, BLE_STRING_MAX_ARRAY_CHARACTERS, readFile);
                fclose(readFile);

                if (unlink(strgReadDirFileName)){
                    ESP_LOGE(STRG_TAG,"File removed - Failed");
                }else{
                    ESP_LOGI(STRG_TAG,"File removed");
                }
                // strip newline
                char* pos = strchr(queueData, '\n');
                if (pos) {
                    *pos = '\0';
                }
                *queueDataLen = strlen(queueData);
                ESP_LOGI(STRG_TAG, "Read from file: '%s' with length: %d", queueData, *queueDataLen);
                retVal = 1;
            }
            else
                ESP_LOGE(STRG_TAG, "Trying to read from an empty Data Queue!");
            break;


        case STRG_LOG_QUEUE:
            if (strgFileCount(LOG_QUEUE_DIR) > 0)
            {
                memset(strgNextFileName,0,sizeof(strgNextFileName));

                strgFilesInDir = strgNextFile(LOG_QUEUE_DIR, &strgNextFileName[0], sizeof(strgNextFileName));
                if( strgFilesInDir)
                {
                    sprintf(strgReadDirFileName,"%s/%s",LOG_QUEUE_DIR,strgNextFileName);
                    ESP_LOGI(STRG_TAG,"Next file name:%s",strgReadDirFileName);
                }
                else{
                    ESP_LOGI(STRG_TAG,"No files found!");
                    break;
                }

                ESP_LOGI(STRG_TAG, "Reading file");
                readFile = fopen(strgReadDirFileName, "r");
                if (readFile == NULL) {
                    ESP_LOGE(STRG_TAG, "Failed to open file for reading");
                    break;
                }
                fgets(queueData, BLE_STRING_MAX_ARRAY_CHARACTERS, readFile);
                fclose(readFile);

                if (unlink(strgReadDirFileName)){
                    ESP_LOGE(STRG_TAG,"File removed - Failed");
                }else{
                    ESP_LOGI(STRG_TAG,"File removed");
                }
                // strip newline
                char* pos = strchr(queueData, '\n');
                if (pos) {
                    *pos = '\0';
                }
                *queueDataLen = strlen(queueData);
                ESP_LOGI(STRG_TAG, "Read from file: '%s' with length: %d", queueData, *queueDataLen);
                retVal = 1;
            }
            else
                ESP_LOGE(STRG_TAG, "Trying to read from an empty Log Queue!"); 
            break;

        default:
            ESP_LOGE(STRG_TAG, "strgReadQueue - Undefined SPIFFS queue:%d",queueId);              
    }
    return retVal;
}

uint8_t strgQueueClr(uint8_t queueId)
{
    uint8_t retVal = 0;
    //FILE* readFile;
    //bool strgFilesInDir = false;
    char strgNextFileName[FILE_NAME_MAX_CHARACTERS];
    char strgDirAndFileName[DIR_AND_FILE_NAME_MAX_CHARACTERS];

    switch (queueId)
    {
        case STRG_DATA_QUEUE:
            if (strgFileCount(DATA_QUEUE_DIR) > 0)
            {
                while (strgNextFile(DATA_QUEUE_DIR, &strgNextFileName[0], sizeof(strgNextFileName)) > 0 )
                {
                    memset(strgDirAndFileName,0,sizeof(strgDirAndFileName));
                    sprintf(strgDirAndFileName,"%s/%s",DATA_QUEUE_DIR,strgNextFileName);
                    ESP_LOGI(STRG_TAG,"Data file to remove:%s",strgDirAndFileName);
                    if (unlink(strgDirAndFileName)){
                        ESP_LOGE(STRG_TAG,"Data File removed - Failed");
                        break;
                    }
                    //else{
                    //    ESP_LOGI(STRG_TAG,"File removed");
                    //}
                }
                ESP_LOGI(STRG_TAG, "Data Queue clear: success");
                retVal = 1;
            }                     
            break;

        case STRG_LOG_QUEUE:
            if (strgFileCount(LOG_QUEUE_DIR) > 0)
            {
                while (strgNextFile(LOG_QUEUE_DIR, &strgNextFileName[0], sizeof(strgNextFileName)) > 0)
                {
                    memset(strgDirAndFileName,0,sizeof(strgDirAndFileName));
                    sprintf(strgDirAndFileName,"%s/%s",LOG_QUEUE_DIR,strgNextFileName);
                    ESP_LOGI(STRG_TAG,"Log file to remove:%s",strgDirAndFileName);
                    if (unlink(strgDirAndFileName)){
                        ESP_LOGE(STRG_TAG,"Log File removed - Failed");
                        break;
                    }
                    //else{
                    //    ESP_LOGI(STRG_TAG,"Log File removed");
                    // }
                }
                ESP_LOGI(STRG_TAG, "Log Queue clear: success");
                retVal = 1;
            }                     
            break;
        default:
            ESP_LOGE(STRG_TAG, "strgQueueClr - Undefined SPIFFS queue:%d",queueId);    
            
    }
    return retVal;
}



void strgDisplayOtaData(void)
{
    ESP_LOGI(STRG_TAG,"OTA_DATA-version:%ld",g_nvs_ota_setting_data.ota_data_version);
    ESP_LOGI(STRG_TAG,"OTA_DATA-config signature:%ld",g_nvs_ota_setting_data.ota_data_sig);

    ESP_LOGI(STRG_TAG,"OTA_DATA-wifi-ssid:%s",g_nvs_ota_setting_data.wifi_ota.ssid);
    ESP_LOGI(STRG_TAG,"OTA_DATA-wifi-pwd:%s",g_nvs_ota_setting_data.wifi_ota.pwd);
    ESP_LOGI(STRG_TAG,"OTA_DATA-wifi-url:%s",g_nvs_ota_setting_data.wifi_ota.url);
    ESP_LOGI(STRG_TAG,"OTA_DATA-wifi-sig:%lX",g_nvs_ota_setting_data.wifi_ota.sig);

    // ESP_LOGI(STRG_TAG,"OTA_DATA-scale-gain:%ld",g_nvs_ota_setting_data.scale_ota.gain);
    // ESP_LOGI(STRG_TAG,"OTA_DATA-scale-offset:%ld",g_nvs_ota_setting_data.scale_ota.offset);
    // ESP_LOGI(STRG_TAG,"OTA_DATA-scale-offset:%ld",g_nvs_ota_setting_data.scale_ota.calOffset);
    // ESP_LOGI(STRG_TAG,"OTA_DATA-scale-sig:%lX",g_nvs_ota_setting_data.scale_ota.sig);

    ESP_LOGI(STRG_TAG,"OTA_DATA-config-sleep_duration:%d",g_nvs_ota_setting_data.config_ota.sleep_duration);
    ESP_LOGI(STRG_TAG,"OTA_DATA-config-upload_duration:%d",g_nvs_ota_setting_data.config_ota.upload_duration);
    ESP_LOGI(STRG_TAG,"OTA_DATA-config-duration_start_time:%ld",g_nvs_ota_setting_data.config_ota.duration_start_time);
    ESP_LOGI(STRG_TAG,"OTA_DATA-config-low_batt_threshold:%d",g_nvs_ota_setting_data.config_ota.low_batt_threshold);
    ESP_LOGI(STRG_TAG,"OTA_DATA-config-brownout_threshold:%d",g_nvs_ota_setting_data.config_ota.brownout_threshold);
    ESP_LOGI(STRG_TAG,"OTA_DATA-config-wifi_retries:%d",g_nvs_ota_setting_data.config_ota.wifi_retries);
    // ESP_LOGI(STRG_TAG,"OTA_DATA-config-tank_retries:%d",g_nvs_ota_setting_data.config_ota.tank_retries);
    // ESP_LOGI(STRG_TAG,"OTA_DATA-config-tank_threshold:%ld",g_nvs_ota_setting_data.config_ota.tank_threshold);
    // ESP_LOGI(STRG_TAG,"OTA_DATA-config-tank_timer:%ld",g_nvs_ota_setting_data.config_ota.tank_timer);
    ESP_LOGI(STRG_TAG,"OTA_DATA-config-sn:%s",g_nvs_ota_setting_data.config_ota.sn);
    ESP_LOGI(STRG_TAG,"OTA_DATA-config-sig:%lX",g_nvs_ota_setting_data.config_ota.sig);

    // ESP_LOGI(STRG_TAG,"OTA_DATA-https-loxId:%s",g_nvs_ota_setting_data.https_ota.loxId);
    // ESP_LOGI(STRG_TAG,"OTA_DATA-https-deviceToken:%s",g_nvs_ota_setting_data.https_ota.deviceToken);
    // ESP_LOGI(STRG_TAG,"OTA_DATA-https-sig:%lX",g_nvs_ota_setting_data.https_ota.sig);    
}

void strgOtaToNvs(void)
{
    memcpy(g_nvs_setting_data.wifi.ssid, g_nvs_ota_setting_data.wifi_ota.ssid, sizeof(g_nvs_ota_setting_data.wifi_ota.ssid));
    memcpy(g_nvs_setting_data.wifi.pwd, g_nvs_ota_setting_data.wifi_ota.pwd, sizeof(g_nvs_ota_setting_data.wifi_ota.pwd));
    memcpy(g_nvs_setting_data.wifi.url, g_nvs_ota_setting_data.wifi_ota.url, sizeof(g_nvs_ota_setting_data.wifi_ota.url));
    g_nvs_setting_data.wifi.sig = g_nvs_ota_setting_data.wifi_ota.sig;

    // g_nvs_setting_data.scale.gain = g_nvs_ota_setting_data.scale_ota.gain;
    // g_nvs_setting_data.scale.offset = g_nvs_ota_setting_data.scale_ota.offset;
    // g_nvs_setting_data.scale.calOffset = g_nvs_ota_setting_data.scale_ota.calOffset;
    // g_nvs_setting_data.scale.sig = g_nvs_ota_setting_data.scale_ota.sig;

    g_nvs_setting_data.config.sleep_duration = g_nvs_ota_setting_data.config_ota.sleep_duration;
    g_nvs_setting_data.config.upload_duration = g_nvs_ota_setting_data.config_ota.upload_duration;
    g_nvs_setting_data.config.duration_start_time = g_nvs_ota_setting_data.config_ota.duration_start_time;
    g_nvs_setting_data.config.low_batt_threshold = g_nvs_ota_setting_data.config_ota.low_batt_threshold;
    g_nvs_setting_data.config.brownout_threshold = g_nvs_ota_setting_data.config_ota.brownout_threshold;
    g_nvs_setting_data.config.wifi_retries = g_nvs_ota_setting_data.config_ota.wifi_retries;
    // g_nvs_setting_data.config.tank_retries = g_nvs_ota_setting_data.config_ota.tank_retries;
    // g_nvs_setting_data.config.tank_threshold = g_nvs_ota_setting_data.config_ota.tank_threshold;
    // g_nvs_setting_data.config.tank_timer = g_nvs_ota_setting_data.config_ota.tank_timer;

    memcpy(g_nvs_setting_data.config.sn, g_nvs_ota_setting_data.config_ota.sn, sizeof(g_nvs_ota_setting_data.config_ota.sn));
    g_nvs_setting_data.config.sig = g_nvs_ota_setting_data.config_ota.sig;
    g_nvs_setting_data.config.sn_sig = g_nvs_ota_setting_data.config_ota.sn_sig;

    // memcpy(g_nvs_setting_data.https.loxId, g_nvs_ota_setting_data.https_ota.loxId, sizeof(g_nvs_ota_setting_data.https_ota.loxId));
    // memcpy(g_nvs_setting_data.https.deviceToken, g_nvs_ota_setting_data.https_ota.deviceToken, sizeof(g_nvs_ota_setting_data.https_ota.deviceToken));
    // g_nvs_setting_data.https.sig = g_nvs_ota_setting_data.https_ota.sig; 
}

void strgNvsToOta(void)
{
    memcpy(g_nvs_ota_setting_data.wifi_ota.ssid, g_nvs_setting_data.wifi.ssid, sizeof(g_nvs_setting_data.wifi.ssid));
    memcpy(g_nvs_ota_setting_data.wifi_ota.pwd, g_nvs_setting_data.wifi.pwd, sizeof(g_nvs_setting_data.wifi.pwd));
    memcpy(g_nvs_ota_setting_data.wifi_ota.url, g_nvs_setting_data.wifi.url, sizeof(g_nvs_setting_data.wifi.url));
    g_nvs_ota_setting_data.wifi_ota.sig = g_nvs_setting_data.wifi.sig;

    // g_nvs_ota_setting_data.scale_ota.gain = g_nvs_setting_data.scale.gain;
    // g_nvs_ota_setting_data.scale_ota.offset = g_nvs_setting_data.scale.offset;
    // g_nvs_ota_setting_data.scale_ota.calOffset = g_nvs_setting_data.scale.calOffset;
    // g_nvs_ota_setting_data.scale_ota.sig = g_nvs_setting_data.scale.sig;

    g_nvs_ota_setting_data.config_ota.sleep_duration = g_nvs_setting_data.config.sleep_duration;
    g_nvs_ota_setting_data.config_ota.upload_duration = g_nvs_setting_data.config.upload_duration;
    g_nvs_ota_setting_data.config_ota.duration_start_time = g_nvs_setting_data.config.duration_start_time;
    g_nvs_ota_setting_data.config_ota.low_batt_threshold = g_nvs_setting_data.config.low_batt_threshold;
    g_nvs_ota_setting_data.config_ota.brownout_threshold = g_nvs_setting_data.config.brownout_threshold;
    g_nvs_ota_setting_data.config_ota.wifi_retries = g_nvs_setting_data.config.wifi_retries;
    // g_nvs_ota_setting_data.config_ota.tank_retries = g_nvs_setting_data.config.tank_retries;
    // g_nvs_ota_setting_data.config_ota.tank_threshold = g_nvs_setting_data.config.tank_threshold;
    // g_nvs_ota_setting_data.config_ota.tank_timer = g_nvs_setting_data.config.tank_timer;
    memcpy(g_nvs_ota_setting_data.config_ota.sn, g_nvs_setting_data.config.sn, sizeof(g_nvs_setting_data.config.sn));
    g_nvs_ota_setting_data.config_ota.sig = g_nvs_setting_data.config.sig;
    g_nvs_ota_setting_data.config_ota.sn_sig = g_nvs_setting_data.config.sn_sig;

    // memcpy(g_nvs_ota_setting_data.https_ota.loxId, g_nvs_setting_data.https.loxId, sizeof(g_nvs_setting_data.https.loxId));
    // memcpy(g_nvs_ota_setting_data.https_ota.deviceToken, g_nvs_setting_data.https.deviceToken, sizeof(g_nvs_setting_data.https.deviceToken));
    // g_nvs_ota_setting_data.https_ota.sig = g_nvs_setting_data.https.sig; 
}

void strgClearOtaData(void)
{
    memset(g_nvs_ota_setting_data.wifi_ota.ssid, 0, sizeof(g_nvs_ota_setting_data.wifi_ota.ssid));
    memset(g_nvs_ota_setting_data.wifi_ota.pwd, 0, sizeof(g_nvs_ota_setting_data.wifi_ota.pwd));
    memset(g_nvs_ota_setting_data.wifi_ota.url, 0, sizeof(g_nvs_ota_setting_data.wifi_ota.url));
    g_nvs_ota_setting_data.wifi_ota.sig = 0;

    // g_nvs_ota_setting_data.scale_ota.gain = 0;
    // g_nvs_ota_setting_data.scale_ota.offset = 0;
    // g_nvs_ota_setting_data.scale_ota.calOffset = 0;
    // g_nvs_ota_setting_data.scale_ota.sig = 0;

    g_nvs_ota_setting_data.config_ota.sleep_duration = 0;
    g_nvs_ota_setting_data.config_ota.upload_duration = 0;
    g_nvs_ota_setting_data.config_ota.duration_start_time = 0;
    g_nvs_ota_setting_data.config_ota.low_batt_threshold = 0;
    g_nvs_ota_setting_data.config_ota.brownout_threshold = 0;
    g_nvs_ota_setting_data.config_ota.wifi_retries = 0;
    // g_nvs_ota_setting_data.config_ota.tank_retries = 0;
    // g_nvs_ota_setting_data.config_ota.tank_threshold = 0;
    // g_nvs_ota_setting_data.config_ota.tank_timer = 0;
    memset(g_nvs_ota_setting_data.config_ota.sn, 0, sizeof(g_nvs_ota_setting_data.config_ota.sn));
    g_nvs_ota_setting_data.config_ota.sig = 0;

    // memset(g_nvs_ota_setting_data.https_ota.loxId, 0, sizeof(g_nvs_ota_setting_data.https_ota.loxId));
    // memset(g_nvs_ota_setting_data.https_ota.deviceToken, 0, sizeof(g_nvs_ota_setting_data.https_ota.deviceToken));
    // g_nvs_ota_setting_data.https_ota.sig = 0; 
}