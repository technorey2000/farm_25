//-------------------------------------------------------------------------------------------------
//  @file       wifiTask.c
//  @brief      Wifi module to perfom Wifi functions
//-------------------------------------------------------------------------------------------------

/* Includes ----------------------------------------------------------------- */
#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "lwip/err.h"
#include "lwip/sys.h"

#include "esp_sntp.h"
#include <time.h>
#include <sys/time.h>
#include <netdb.h>
#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "esp_http_client.h"
#include "esp_tls.h"
#include "esp_transport_tcp.h"

#include "Shared/common.h"
#include "Shared/messages.h"
#include "Shared/cJSON.h"

#include "Task/wifiTask.h"


/* Private defines ---------------------------------------------------------- */
#define WIFI_TAG "WIFI"

/* Private variables -------------------------------------------------------- */

static esp_http_client_handle_t wifiDataClient;

RTOS_message_t wifiRxMessage;
RTOS_message_t wifiTxMessage;
UBaseType_t wifiNewHighWaterMark = 0;
UBaseType_t wifiCurrHighWaterMark = 0;
UBaseType_t wifiInitHighWaterMark = 0;

static uint8_t wifiRdSsidSrcAddr = 0;
static uint16_t wifiRdSsidMsgCmd = 0;

static uint8_t wifiRdPwdSrcAddr = 0;
static uint16_t wifiRdPwdMsgCmd = 0;

static uint8_t wifiRdRetrySrcAddr = 0;
static uint16_t wifiRdRetryMsgCmd = 0;

static uint8_t wifiRdSnSrcAddr = 0;
static uint16_t wifiRdSnMsgCmd = 0;

static uint8_t wifiRdDeviceDataSrcAddr = 0;
static uint16_t wifiRdDeviceDataMsgCmd = 0;

static uint8_t wifiCmdReturn = WIFI_RETURN_FAILURE;

static char * deviceAccessToken[DEVICE_ACCESS_TOKEN_MAX_CHARACTERS] = {0};

static char * receivedToken[RECEIVED_TOKEN_MAX_CHARACTERS] = {0};

static char wifiSwVer[SOFTWARE_VERSION_MAX_CHARACTERS] = {0};
char wifiCurrentSwVer[SOFTWARE_VERSION_MAX_CHARACTERS] = {0};

uint32_t free_heap_size=0;

static bool wifiSimAlmConLost  = false;
static bool wifiSimAlmNetErr   = false;

time_t now;
struct tm timeinfo;
char strftime_buf[64];
int64_t time_us = 0;
uint32_t time_sec = 0;

wifi_status_t wifiStatus;
uint32_t wifiSendStatus = 0;
char wifiTmpStr[WIFI_STRING_MAX_ARRAY_CHARACTERS];
char wifiTmpBleStr[WIFI_STRING_MAX_ARRAY_CHARACTERS];
char wifiTmpLoxId[WIFI_LOX_ID_MAX_CHAR];

static bool wifiStationInitalized = false;
static bool wifiSntpInitalized = false;
static bool wifiStationConnected = false;
static bool wifiScanInitialized = false;

wifi_config_t wifi_config;
device_data_t wifiDeviceData;

static uint64_t wifiCurrentTime = 0;

//static const char *WIFI_TAG = "WIFI";

static int s_retry_num = 0;

static wifi_charc_data_t wifi_charc_data_current;

// FreeRTOS event group to signal when we are connected
static EventGroupHandle_t s_wifi_event_group;

static uint8_t wifiTokenType = DEVICE_ACCESS_TOKEN;
static bool tokenFound = false;
static scale_config_data_t config;

EventGroupHandle_t g_sys_evt_group;


/* Private function prototypes ---------------------------------------------- */
uint8_t wifiInitStation(void);
void wifiLog(char * strPtr, bool forced, bool printTag);
void wifiLogR(char * strPtr);
void wifiLogI(char * strPtr);
void wifiLogE(char * strPtr);
void wifiLogIF(char * strPtr);
void wifiLogEF(char * strPtr);
void wifiSendMessage(uint8_t dstAddr, uint8_t msgType, uint16_t msgCmd, uint8_t * msgDataPtr, uint32_t msgData, uint32_t msgDataLen);
bool wifiProvision(char * serialNum);
void wifiConnect(void);
void wifiScan(void);
void wifiUpdateSSID(void);
void wifiUpdatePWD(void);
void wifiSetRTC(uint64_t timeInUsec);
uint32_t wifiGetRTC(void);

//void wifiStartOtaProcess(void);

esp_err_t _http_event_handler(esp_http_client_event_t *evt);

///////////////////////////////////

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    ESP_LOGI(WIFI_TAG,"event_id:%ld", event_id);

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        //if (s_retry_num < WIFI_MAXIMUM_RETRY) {
        if (s_retry_num < wifi_charc_data_current.wifi_retries) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(WIFI_TAG, "retry to connect to the AP");
            wifiLogR("retry to connect to the AP");

        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGE(WIFI_TAG,"connect to the AP fail");
        wifiLogR("Wifi Disconnected");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(WIFI_TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}


/* Global variables --------------------------------------------------------- */


/* Function definitions ----------------------------------------------------- */
/// @brief wifi task
/// @param  None
void wifiTaskApp(void)
{   
    ESP_LOGI(WIFI_TAG, "wifi task is running");
    while(1)
    {
        if (xQueueReceive(wifiQueueHandle, &wifiRxMessage, portMAX_DELAY))
        {
            ESP_LOGI(WIFI_TAG, "wifi command received: %d",  wifiRxMessage.msgCmd);
            switch(wifiRxMessage.msgCmd)
            {
                case WIFI_CMD_INIT:
                
                    ESP_LOGI(WIFI_TAG, "WIFI_CMD_INIT received");
                    memset(&wifiStatus, 0, sizeof(wifiStatus));
                    
                    wifiSendStatus = WIFI_INIT_COMPLETE;                   
                    wifiStatus.is_initialized = true;
                    wifiSendMessage(wifiRxMessage.srcAddr, MSG_DATA_8, wifiRxMessage.msgCmd, NULL, wifiSendStatus, MSG_DATA_8_LEN);                    
                    break;
                case WIFI_CMD_PING:
                    wifiSendMessage(wifiRxMessage.srcAddr, MSG_DATA_8, wifiRxMessage.msgCmd, NULL, WIFI_PING_RECEIVED, MSG_DATA_8_LEN);
                    break;
                case WIFI_CMD_STATUS:
                    memcpy(&wifiSendStatus, &wifiStatus, sizeof(wifiStatus));
                    wifiSendMessage(wifiRxMessage.srcAddr, MSG_DATA_32, wifiRxMessage.msgCmd, NULL, wifiSendStatus, MSG_DATA_32_LEN);
                    break;    
                case WIFI_CMD_STA_INIT:
                    if (wifiStationInitalized){
                        ESP_LOGI(WIFI_TAG, "Wifi already initialized!");
                        wifiCmdReturn = WIFI_RETURN_GOOD;
                    }else{
                        ESP_LOGI(WIFI_TAG, "Initialized wifi station");
                        wifiCmdReturn = wifiInitStation();
                        wifiStationInitalized = true;
                    }
                    wifiSendMessage(wifiRxMessage.srcAddr, MSG_DATA_8, wifiRxMessage.msgCmd, NULL, wifiCmdReturn, MSG_DATA_8_LEN);
                    break;

                case WIFI_CMD_START:
                    wifiSendStatus = WIFI_RETURN_GOOD;
                    //Read stored SSID and Password
                    ESP_LOGI(WIFI_TAG, "get ssid for wifi");
                    wifiSendMessage(MSG_ADDR_STRG, MSG_DATA_0, STRG_CMD_RD_SSID, NULL, 0, MSG_DATA_0_LEN);
                    wifiSendMessage(MSG_ADDR_STRG, MSG_DATA_0, STRG_CMD_RD_PWD, NULL, 0, MSG_DATA_0_LEN);
                    wifiSendMessage(wifiRxMessage.srcAddr, MSG_DATA_8, wifiRxMessage.msgCmd, NULL, wifiSendStatus, MSG_DATA_8_LEN);
                    s_wifi_event_group = xEventGroupCreate();
                    break;    

                case WIFI_CMD_STA_CONNECT:
                    wifiConnect();    
                    if (wifiStatus.is_connected){
                        wifiCmdReturn = WIFI_RETURN_GOOD;
                        wifiStationConnected = true;
                    }
                    else{
                        wifiCmdReturn = WIFI_RETURN_FAILURE;
                        wifiStationConnected = false;
                    }
                    
                    if (wifiRxMessage.srcAddr == MSG_ADDR_SUPR)
                    {
                        memset(wifiTmpStr,0,sizeof(wifiTmpStr));
                        sprintf(wifiTmpStr,"\",\"wifi\":\"con");
                        wifiSendMessage(wifiRxMessage.srcAddr, MSG_DATA_STR, wifiRxMessage.msgCmd, (uint8_t *)wifiTmpStr, 0, strlen(wifiTmpStr)+1);
                        //wifiSendMessage(MSG_ADDR_LED, MSG_DATA_8, LED_CMD_ON, NULL, WIFI_LED_REF, MSG_DATA_8_LEN);
                    }
                    else
                    {
                        wifiSendMessage(wifiRxMessage.srcAddr, MSG_DATA_8, wifiRxMessage.msgCmd, NULL, wifiCmdReturn, MSG_DATA_8_LEN);
                    }
                    break;    
                case WIFI_CMD_GET_TSF_TIME:
                    wifiCurrentTime = esp_wifi_get_tsf_time(WIFI_IF_STA);
                    ESP_LOGI(WIFI_TAG, "Time from router %lld", wifiCurrentTime);
                    break;

                case WIFI_CMD_INIT_SNTP: 
                    if (!wifiSntpInitalized){
                        sntp_setoperatingmode(SNTP_OPMODE_POLL);
                        sntp_setservername(0, "north-america.pool.ntp.org");
                        ESP_LOGI(WIFI_TAG, "Initializing SNTP");
                        sntp_init();                            // init and set time   
                        wifiSntpInitalized = true; 
                    }
                    ESP_LOGI(WIFI_TAG, "Initializing SNTP complete"); 
                    wifiCmdReturn = WIFI_RETURN_GOOD;                    
                    wifiSendMessage(wifiRxMessage.srcAddr, MSG_DATA_8, wifiRxMessage.msgCmd, NULL, wifiCmdReturn, MSG_DATA_8_LEN);               
                    break;

                case WIFI_CMD_GET_DATE_TIME:    
                    time(&now);
                    localtime_r(&now, &timeinfo);
                    setenv("TZ", "GTM", 1);
                    tzset();
                    localtime_r(&now, &timeinfo);
                    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
                    ESP_LOGI(WIFI_TAG, "20%d-%d-%d+%d:%d:%d", timeinfo.tm_year - 100, timeinfo.tm_mon + 1, timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);

                    struct timeval tv_now;
                    gettimeofday(&tv_now, NULL);
                    time_us = ((int64_t)tv_now.tv_sec * 1000000L)/1000;
                    ESP_LOGI(WIFI_TAG, "Time from router in ms: %lld", time_us);
                    if (WIFI_GET_TIME_MIN < time_us){
                        wifiSetRTC((uint64_t) (time_us * 1000)); // Set RTC with time obtained from the network
                        wifiCmdReturn = WIFI_RETURN_GOOD;
                    }else{
                        wifiCmdReturn = WIFI_RETURN_FAILURE;
                    }                  
                    wifiSendMessage(wifiRxMessage.srcAddr, MSG_DATA_8, wifiRxMessage.msgCmd, NULL, wifiCmdReturn, MSG_DATA_8_LEN);               
                    break;   

                case WIFI_CMD_GET_RTC_TIME:
                    time_sec = wifiGetRTC();
                    ESP_LOGI(WIFI_TAG, "Current RTC in sec: %ld", time_sec);
                    wifiSendMessage(wifiRxMessage.srcAddr, MSG_DATA_32, wifiRxMessage.msgCmd, NULL, time_sec, MSG_DATA_32_LEN); 
                    break;    

                case WIFI_CMD_SET_RTC_TIME:
                    uint64_t tmpTimeUsec = 0;

                    ESP_LOGI(WIFI_TAG, "Current received time in sec: %s", (char *)wifiRxMessage.msgDataPtr);
                    tmpTimeUsec = strtoul((char *)wifiRxMessage.msgDataPtr, NULL, 10);

                    if ((tmpTimeUsec >= BLE_GET_TIME_MIN) && (tmpTimeUsec <= BLE_GET_TIME_MAX))
                    {
                        tmpTimeUsec = tmpTimeUsec * SEC_TO_MSEC_MULTIPLYER * MSEC_TO_USEC_MULTIPLYER;
                        ESP_LOGI(WIFI_TAG, " rtc_write_handler microseconds received  %lld ", tmpTimeUsec);

                        struct timeval tv = { .tv_sec = tmpTimeUsec / 1000000L, .tv_usec = tmpTimeUsec % 1000000L };
                        settimeofday(&tv, NULL);
                    }
                    else
                    {
                        ESP_LOGE(WIFI_TAG, "Time received is out of bounds!");
                    }
                    break;    

                case WIFI_CMD_STA_DISCONNECT:
                    esp_wifi_disconnect();
                    esp_wifi_stop();
                    wifiStationConnected = false;
                    if (wifiRxMessage.srcAddr == MSG_ADDR_SUPR)
                    {
                        memset(wifiTmpStr,0,sizeof(wifiTmpStr));
                        sprintf(wifiTmpStr,"\",\"wifi\":\"dis");
                        wifiSendMessage(wifiRxMessage.srcAddr, MSG_DATA_STR, wifiRxMessage.msgCmd, (uint8_t *)wifiTmpStr, 0, strlen(wifiTmpStr)+1);
                        //wifiSendMessage(MSG_ADDR_LED, MSG_DATA_8, LED_CMD_OFF, NULL, WIFI_LED_REF, MSG_DATA_8_LEN);
                    }
                    break;   

                case WIFI_CMD_STA_SCAN:
                    if(wifiStationConnected){
                        esp_wifi_disconnect();
                        esp_wifi_stop();
                        wifiStationConnected = false;
                    }
                    wifiScan();
                    break;  

                case WIFI_CMD_STA_SSID_RECON:
                    wifiUpdateSSID();
                    break; 

                case WIFI_CMD_STA_PWD_RECON:
                    wifiUpdatePWD();
                    break; 

                case WIFI_CMD_RD_SSID:
                    wifiRdSsidSrcAddr = wifiRxMessage.srcAddr;  //Address to send response to
                    wifiRdSsidMsgCmd = wifiRxMessage.msgCmd;    //Command to send back
                    wifiSendMessage(MSG_ADDR_STRG, MSG_DATA_0, STRG_CMD_RD_SSID, NULL, 0, MSG_DATA_0_LEN);
                    break; 

                case WIFI_CMD_RD_PWD:
                    wifiRdPwdSrcAddr = wifiRxMessage.srcAddr;  //Address to send response to
                    wifiRdPwdMsgCmd = wifiRxMessage.msgCmd;    //Command to send back
                    wifiSendMessage(MSG_ADDR_STRG, MSG_DATA_0, STRG_CMD_RD_PWD, NULL, 0, MSG_DATA_0_LEN);                
                    break; 

                case WIFI_CMD_RD_SSID_PWD:
                    wifiSendMessage(MSG_ADDR_STRG, MSG_DATA_0, STRG_CMD_RD_SSID, NULL, 0, MSG_DATA_0_LEN);
                    wifiSendMessage(MSG_ADDR_STRG, MSG_DATA_0, STRG_CMD_RD_PWD, NULL, 0, MSG_DATA_0_LEN);                
                    break; 

                case WIFI_CMD_READ_DEVICE_DATA:
                    wifiRdDeviceDataSrcAddr = wifiRxMessage.srcAddr;  //Address to send response to
                    wifiRdDeviceDataMsgCmd = wifiRxMessage.msgCmd;    //Command to send back
                    wifiSendMessage(MSG_ADDR_STRG, MSG_DATA_0, STRG_CMD_READ_DEVICE_DATA, NULL, 0, MSG_DATA_0_LEN);
                    break;    

                case WIFI_CMD_PRINT_DEVICE_DATA:
                    ESP_LOGI(WIFI_TAG,"wifi_charc_data_current-ssid:%s"         ,wifi_charc_data_current.ssid);
                    ESP_LOGI(WIFI_TAG,"wifi_charc_data_current-password:%s"     ,wifi_charc_data_current.password);
                    break;    

                //Responses to commands
                case STRG_CMD_RD_SSID:
                    ESP_LOGI(WIFI_TAG, "wifi-SSID received from storage: %s\r\n", (char *)wifiRxMessage.msgDataPtr);
                    memset(wifi_charc_data_current.ssid, 0x00, sizeof(wifi_charc_data_current.ssid));
                    memcpy(wifi_charc_data_current.ssid, (char *)wifiRxMessage.msgDataPtr, wifiRxMessage.msgDataLen);                     
                    //memcpy(otaSsid, (char *)wifiRxMessage.msgDataPtr, wifiRxMessage.msgDataLen);
                    wifiSendStatus = WIFI_RETURN_GOOD;
                    wifiSendMessage(wifiRdSsidSrcAddr, MSG_DATA_8, wifiRdSsidMsgCmd, NULL, wifiSendStatus, MSG_DATA_8_LEN); 
                    break;   

                case STRG_CMD_RD_PWD:
                    ESP_LOGI(WIFI_TAG, "wifi-Password received from storage: %s\r\n", (char *)wifiRxMessage.msgDataPtr);
                    memset(wifi_charc_data_current.password, 0x00, sizeof(wifi_charc_data_current.password));
                    memcpy(wifi_charc_data_current.password, (char *)wifiRxMessage.msgDataPtr, wifiRxMessage.msgDataLen);                   
                    //memcpy(otaPassword, (char *)wifiRxMessage.msgDataPtr, wifiRxMessage.msgDataLen);
                    wifiSendStatus = WIFI_RETURN_GOOD;
                    wifiSendMessage(wifiRdPwdSrcAddr, MSG_DATA_8, wifiRdPwdMsgCmd, NULL, wifiSendStatus, MSG_DATA_8_LEN); 
                    break;       		

                case STRG_CMD_READ_DEVICE_DATA:
                    // memcpy(&wifiDeviceData.deviceDataStructId, (device_data_t *)wifiRxMessage.msgDataPtr, wifiRxMessage.msgDataLen);

                    // memset(wifi_charc_data_current.deviceToken, 0x00, sizeof(wifi_charc_data_current.deviceToken));
                    // memcpy(wifi_charc_data_current.deviceToken, (char *)wifiDeviceData.deviceToken, sizeof(wifiDeviceData.deviceToken));

                    // memset(wifi_charc_data_current.loxId, 0x00, sizeof(wifi_charc_data_current.loxId));
                    // memcpy(wifi_charc_data_current.loxId, (char *)wifiDeviceData.loxId, sizeof(wifiDeviceData.loxId));

                    // memset(wifi_charc_data_current.sslCertUrl, 0x00, sizeof(wifi_charc_data_current.sslCertUrl));
                    // memcpy(wifi_charc_data_current.sslCertUrl, (char *)wifiDeviceData.sslCertUrl, sizeof(wifiDeviceData.sslCertUrl));

                    // memset(wifi_charc_data_current.baseUrl, 0x00, sizeof(wifi_charc_data_current.baseUrl));
                    // memcpy(wifi_charc_data_current.baseUrl, (char *)wifiDeviceData.url, sizeof(wifiDeviceData.url));

                    // wifi_charc_data_current.providerId = wifiDeviceData.providerId;
                    // wifi_charc_data_current.locationId = wifiDeviceData.locationId;
                    // wifi_charc_data_current.wifi_retries = wifiDeviceData.wifi_retries;
                    // ESP_LOGI(WIFI_TAG, "STRG_CMD_READ_DEVICE_DATA - Wifi retries:%d",wifi_charc_data_current.wifi_retries);

                    // wifiSendStatus = WIFI_RETURN_GOOD;
                    // wifiSendMessage(wifiRdDeviceDataSrcAddr, MSG_DATA_8, wifiRdDeviceDataMsgCmd, NULL, wifiSendStatus, MSG_DATA_8_LEN); 
                    break;       		
                default:;

            }

        }
    }
 
}

/* Private function --------------------------------------------------------- */

void wifiScan(void)
{
    if (!wifiScanInitialized)
    {
        ESP_ERROR_CHECK(esp_netif_init());
        ESP_ERROR_CHECK(esp_event_loop_create_default());
        esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
        assert(sta_netif);
        wifiScanInitialized = true;
    }

    if(!wifiStationInitalized){
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_wifi_init(&cfg));
        wifiStationInitalized = true;
    }

    uint16_t number = WIFI_SCAN_LIST_SIZE;
    wifi_ap_record_t ap_info[WIFI_SCAN_LIST_SIZE];
    uint16_t ap_count = 0;
    memset(ap_info, 0, sizeof(ap_info));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    esp_wifi_scan_start(NULL, true);
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&number, ap_info));
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&ap_count));
    ESP_LOGI(WIFI_TAG, "Total APs scanned = %u", ap_count);
    for (int i = 0; (i < WIFI_SCAN_LIST_SIZE) && (i < ap_count); i++) {
        sprintf(wifiTmpBleStr, "%s,%i", ap_info[i].ssid, ap_info[i].rssi);
        wifiLogR(wifiTmpBleStr);        
    }
    esp_wifi_scan_stop();
    esp_wifi_stop();
}


/// @brief 
/// @param ssid 
/// @param len 
void wifiUpdateSSID(void)
{
    ESP_LOGI(WIFI_TAG, "ssid_handle");
    esp_wifi_disconnect();
    memcpy(wifi_config.sta.ssid, (char *)wifi_charc_data_current.ssid, sizeof(wifi_charc_data_current.ssid));
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    ESP_LOGI(WIFI_TAG, "Recv STA SSID: %s\n", wifi_config.sta.ssid);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    esp_wifi_connect();
    ESP_LOGI(WIFI_TAG, "Waiting for wifi");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(WIFI_TAG, "connected to ap SSID:%s password:%s",
                 wifi_charc_data_current.ssid, wifi_charc_data_current.password);
        wifiStatus.is_connected = true;         
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(WIFI_TAG, "Failed to connect to SSID:%s, password:%s",
                 wifi_charc_data_current.ssid, wifi_charc_data_current.password);
        wifiStatus.is_connected = false;         
    } else {
        ESP_LOGE(WIFI_TAG, "UNEXPECTED EVENT");
        wifiStatus.is_connected = false;
    }
}

/// @brief 
/// @param pswd 
/// @param len 
void wifiUpdatePWD(void)
{
    ESP_LOGI(WIFI_TAG, "pswd_handle");
    esp_wifi_disconnect();
    memcpy(wifi_config.sta.password, (char *)wifi_charc_data_current.password, sizeof(wifi_charc_data_current.password));
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    ESP_LOGI(WIFI_TAG, "Recv STA password: %s\n", wifi_config.sta.password);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    esp_wifi_connect();
    ESP_LOGI(WIFI_TAG, "Waiting for wifi");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(WIFI_TAG, "connected to ap SSID:%s password:%s",
                 wifi_charc_data_current.ssid, wifi_charc_data_current.password);
        wifiStatus.is_connected = true;         
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(WIFI_TAG, "Failed to connect to SSID:%s, password:%s",
                 wifi_charc_data_current.ssid, wifi_charc_data_current.password);
        wifiStatus.is_connected = false;         
    } else {
        ESP_LOGE(WIFI_TAG, "UNEXPECTED EVENT");
        wifiStatus.is_connected = false;
    }
}

/// @brief 
/// @param timeInUsec 
void wifiSetRTC(uint64_t timeInUsec)
{
    struct timeval tv = { .tv_sec = timeInUsec / 1000000L, .tv_usec = timeInUsec % 1000000L };
    settimeofday(&tv, NULL);
}

/// @brief 
/// @param  
/// @return 
uint32_t wifiGetRTC(void)
{
    struct timeval tv;

    gettimeofday(&tv, NULL);

    return ((uint32_t) (tv.tv_sec + tv.tv_usec/1000000L));
}


/// @brief 
/// @param  
uint8_t wifiInitStation(void)
{
    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    return(WIFI_RETURN_GOOD);                                                
}

/// @brief 
/// @param  
void wifiConnect(void)
{
    ESP_LOGI(WIFI_TAG, "wifiConnect - start - free heap size = %ld",esp_get_free_heap_size());

    if (!wifiStationInitalized){
        ESP_LOGI(WIFI_TAG,"wifiConnect - intialize wifi");
        wifiInitStation();
        wifiStationInitalized = true; 
    }else{
        esp_wifi_stop();
        ESP_LOGI(WIFI_TAG,"wifiConnect - wifi reintialized");
    }

    ESP_LOGI(WIFI_TAG, "wifiConnect intialized - free heap size = %ld",esp_get_free_heap_size());
    
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    memcpy(wifi_config.sta.ssid, (char *)wifi_charc_data_current.ssid, sizeof(wifi_charc_data_current.ssid));
    memcpy(wifi_config.sta.password, (char *)wifi_charc_data_current.password, sizeof(wifi_charc_data_current.password));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );

    ESP_LOGI(WIFI_TAG, "wifiConnect esp_wifi_set_mode - free heap size = %ld",esp_get_free_heap_size());

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );

    ESP_LOGI(WIFI_TAG, "wifiConnect esp_wifi_set_config - free heap size = %ld",esp_get_free_heap_size());

    xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(WIFI_TAG, "wifiConnect esp_wifi_start - free heap size = %ld",esp_get_free_heap_size());

    ESP_LOGI(WIFI_TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(WIFI_TAG, "connected to ap SSID:%s password:%s",
                 wifi_charc_data_current.ssid, wifi_charc_data_current.password);
        wifiStatus.is_connected = true;         
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(WIFI_TAG, "Failed to connect to SSID:%s, password:%s",
                 wifi_charc_data_current.ssid, wifi_charc_data_current.password);
        wifiStatus.is_connected = false;         
    } else {
        ESP_LOGE(WIFI_TAG, "UNEXPECTED EVENT");
        wifiStatus.is_connected = false;
    }
}

//------------------------------------------------------------------------------------------------------------
//MQTT code

//End MQTT code
//------------------------------------------------------------------------------------------------------------



/// @brief  This function sends the info log message to the BLE queue reguardless of 
///    the mobile device connected and/or the Log notification enabled. It will also send
///    the log message to the terminal.
/// @param strPtr 
void wifiLogR(char * strPtr)
{
    ESP_LOGI(WIFI_TAG, "%s", strPtr);
    wifiLog(strPtr, true, false);
}

/// @brief  This function sends the info log message to the BLE queue reguardless of 
///    the mobile device connected and/or the Log notification enabled. It will also send
///    the log message to the terminal.
/// @param strPtr 
void wifiLogIF(char * strPtr)
{
    ESP_LOGI(WIFI_TAG, "%s", strPtr);
    wifiLog(strPtr, true, true);
}

/// @brief  This function sends the error log message to the BLE queue reguardless of 
///    the mobile device connected and/or the Log notification enabled. It will also send
///    the log message to the terminal.
void wifiLogEF(char * strPtr)
{
    ESP_LOGE(WIFI_TAG, "%s", strPtr);
    wifiLog(strPtr, true, true);
}

/// @brief  This function send the info log message to the Mobile device and to the 
///    terminal.
/// @param strPtr 
void wifiLogI(char * strPtr)
{
    ESP_LOGI(WIFI_TAG, "%s", strPtr);
    wifiLog(strPtr, false, true);
}

/// @brief This function send the error log message to the Mobile device and to the 
///    terminal. 
/// @param strPtr 
void wifiLogE(char * strPtr)
{
    ESP_LOGE(WIFI_TAG, "%s", strPtr);
    wifiLog(strPtr, false, true);
}

/// @brief This function sends a message to the BLE module to send out a log message
///   to the mobile device via BLE. Mobile device has to be connected and the
///   log notifications (RX Characteristics) enabled.
/// @param strPtr - message to be sent to phone.
/// @param forced - If true, message is forced in the log queue.
/// @param printTag - If true the tag is printed with the message. 
void wifiLog(char * strPtr, bool forced, bool printTag)
{
    static uint8_t printIndex = 0;
    RTOS_message_t modPrintMsg;
    char tmpStr[STRING_MAX_LOG_CHARACTERS];
    static char modStr[WIFI_STRING_MAX_ARRAY_LOG_ELEMENTS][STRING_MAX_LOG_CHARACTERS];

    //copy string into next available string array
    //memcpy(modStr[printIndex], strPtr, strlen(strPtr)+1);
    memcpy(tmpStr, strPtr, strlen(strPtr)+1);
	if (printTag){
        sprintf(modStr[printIndex], "%s:%s", WIFI_TAG, tmpStr);
    }else{
        sprintf(modStr[printIndex], "%s", tmpStr);
    }

    //form message to print out string
    modPrintMsg.srcAddr         =  MSG_ADDR_WIFI; 
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
    if (printIndex >= WIFI_STRING_MAX_ARRAY_LOG_ELEMENTS)
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
void wifiSendMessage(uint8_t dstAddr, uint8_t msgType, uint16_t msgCmd, uint8_t * msgDataPtr, uint32_t msgData, uint32_t msgDataLen)
{  
    static uint8_t msgIndex = 0;
    RTOS_message_t sendRtosMsg;
    static char sendMsgArray[WIFI_MESSAGE_MAX_ARRAY_ELEMENTS][WIFI_STRING_MAX_ARRAY_CHARACTERS];

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
    sendRtosMsg.srcAddr = MSG_ADDR_WIFI;
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
    if (msgIndex >= WIFI_MESSAGE_MAX_ARRAY_ELEMENTS)
    {
        msgIndex = 0;
    }

}
 /* [] END OF FILE */

