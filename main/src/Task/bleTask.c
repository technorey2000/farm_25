#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_system.h"
#include "esp_log.h"


#include "esp_bt.h"             // implements BT controller and VHCI configuration procedures from the host side.
#include "esp_gap_ble_api.h"    // implements GAP configuration, such as advertising and connection parameters.
#include "esp_gatts_api.h"      // implements GATT configuration, such as creating services and characteristics.
#include "esp_bt_defs.h"
#include "esp_bt_main.h"        // implements initialization and enabling of the Bluedroid stack.
#include "esp_bt_device.h"
#include "esp_blufi_api.h"
#include "esp_gatt_common_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"


#include "Shared/common.h"
#include "Shared/messages.h"

#include "Task/bleTask.h"


//Parameters that need to move to the storage module
static char advSerialNumber[MAX_ADV_SERIAL_NUMBER_LEN] = {'T', 'E', 'S', 'T', 'A', 'D', 'V'};
static char scaleSerialNumber[MAX_SCALE_SERIAL_NUMBER_LEN] = {'T', 'E', 'S', 'T', 'S', 'C', 'L'};

#define BLE_TAG "BLE"
#define REMOTE_SERVICE_UUID         0x00FF
#define REMOTE_NOTIFY_CHAR_UUID     0xFF01 
#define INVALID_HANDLE              0

#define LOX_SVC_INST_ID	0

#define EXAMPLE_DEVICE_NAME                       "LOX:CBLSA223300005:0.0.1" 

//Local Parameters
RTOS_message_t bleRxMessage;
RTOS_message_t bleTxMessage;
UBaseType_t bleNewHighWaterMark = 0;
UBaseType_t bleCurrHighWaterMark = 0;
UBaseType_t bleInitHighWaterMark = 0;

ble_status_t bleStatus;
uint32_t bleSendStatus = BLE_INIT_ERROR;

char bleTmpStr[BLE_STRING_MAX_ARRAY_CHARACTERS];
char bleTmpCharStr[GATTS_CHAR_VAL_LEN_MAX];
char blufi_name[60];

ble_conn_t ble_conn = {
    .is_ble_connected = false};


uint8_t bleReturnStatus = BLE_RETURN_FAILURE;

static uint32_t bleLastActivityTime = 0;


uint16_t sleep_duration_tmp;
uint16_t transmit_delay_tmp;

esp_gatt_if_t bleSsidGattsIf;
uint16_t bleSsidReadHandle;
uint16_t bleSsidReadConnId;
uint32_t bleSsidReadTransId;

esp_gatt_if_t blePwdGattsIf;
uint16_t blePwdReadHandle;
uint16_t blePwdReadConnId;
uint32_t blePwdReadTransId;

esp_gatt_if_t bleRwGattsIf;
uint16_t bleRwReadHandle;
uint16_t bleRwReadConnId;
uint32_t bleRwReadTransId;

static uint16_t lox_conn_id = 0xffff;
static esp_gatt_if_t lox_gatts_if = 0xff;

char bleLoxName[60] = {0};
uint8_t logging_buffer[MAX_MTU_SIZE] = {0};

// #ifdef BLE_SECURITY

    static uint8_t sec_service_uuid[16] = {
        /* LSB <--------------------------------------------------------------------------------> MSB */
        //first uuid, 16bit, [12],[13] is the value
        0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x18, 0x0D, 0x00, 0x00,
    };


    static uint8_t adv_config_done = 0;
    static uint8_t test_manufacturer[3]={'E', 'S', 'P'}; 

//Encryption
const char *encryptionKey = "vJKfkSLShUDJfyBoExTO6PzeeWryODkX";

typedef struct lox_receive_data_node{
    int32_t len;
    uint8_t * node_buff;
    struct lox_receive_data_node * next_node;
}lox_receive_data_node_t;

typedef struct lox_receive_data_buff{
    int32_t node_num;
    int32_t buff_size;
    lox_receive_data_node_t * first_node;
}lox_receive_data_buff_t;

static ble_charc_data_t ble_charc_data = {
    .char_value = "LOX_SCALE_DEFAULT_VALUE\0",
    .ssid = "CaireInc",
    .password = "12345",
    .calibration = "abcd",
    .scale_weight = "123.4",
    .zero_level_val = 0xabcd,
    .full_level_val = 0xfedcba,
    .console_value = "LOX_CONSOLE_DEFAULT_VALUE\0"
};

ble_lat_long_send_t ble_lat_long_send = {0};
uint16_t lox_handle_table[LOX_IDX_NB];

ble_charc_data_t ble_charc_data_update;

/****************************************************************************************
 *  LOX PROFILE ATTRIBUTES
 ****************************************************************************************/
static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint16_t character_client_desc_uuid = ESP_GATT_UUID_CHAR_DESCRIPTION;

static const uint8_t char_prop_read_notify  = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_write_notify = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_read_write   = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE;

static esp_bd_addr_t lox_remote_bda = {0x0,};


ble_charc_uuid_t ble_charc_uuid[12] = {
    {{0x03, 0x00, 0x13, 0xac, 0x42, 0x02, 0xb3, 0xa8, 0xeb, 0x11, 0x07, 0x97, 0x26, 0xc3, 0x86, 0x6d}, {"service_charc"}},
    {{0x03, 0x00, 0x13, 0xac, 0x42, 0x02, 0xb3, 0xa8, 0xeb, 0x11, 0x07, 0x97, 0x56, 0xc5, 0x86, 0x6d}, {"scale_charc"}},  
    {{0x02, 0x00, 0x13, 0xac, 0x42, 0x02, 0xbc, 0xbc, 0xeb, 0x11, 0x1c, 0xa9, 0x60, 0x79, 0x1b, 0x6c}, {"ssid_charc"}},
    {{0x02, 0x00, 0x13, 0xac, 0x42, 0x02, 0xbc, 0xbc, 0xeb, 0x11, 0x1c, 0xa9, 0x68, 0x7b, 0x1b, 0x6c}, {"password_charc"}},
    {{0x03, 0x00, 0x13, 0xac, 0x42, 0x02, 0xbc, 0xb8, 0xeb, 0x11, 0xff, 0xd8, 0x80, 0x80, 0x65, 0xc4}, {"scale_weight_charc"}},
    //{{0x03, 0x00, 0x13, 0xac, 0x42, 0x02, 0x03, 0x9a, 0xeb, 0x11, 0x19, 0xef, 0x02, 0xa5, 0xd7, 0x3a}, {"zero_level_charc"}},
    //{{0xC7, 0x69, 0x35, 0xEB, 0xBC, 0xFC, 0xE8, 0xA9, 0xB5, 0x4C, 0x28, 0xA4, 0x69, 0x32, 0xDA, 0xF9}, {"ble_keep_alive_charc"}}, //F9DA3269-A428-4CB5-A9E8-FCBCEB3569C7
    {{0x7E, 0x29, 0x9D, 0xB8, 0x7C, 0xC8, 0xC1, 0x93, 0xCB, 0x49, 0xE9, 0xDC, 0x40, 0x9A, 0xB5, 0x3A}, {"lox_index_read_charc"}},
    {{0x03, 0x00, 0x13, 0xac, 0x42, 0x02, 0x03, 0x9a, 0xeb, 0x11, 0x19, 0xef, 0x22, 0xb8, 0x5d, 0x4a}, {"full_level_charc"}},
    {{0x03, 0x00, 0x13, 0xac, 0x42, 0x02, 0x03, 0x9a, 0xec, 0x11, 0xf2, 0x04, 0x28, 0x1e, 0xe8, 0xa5}, {"calibration_charc"}},
    {{0x02, 0x00, 0x12, 0xac, 0x42, 0x02, 0x1d, 0x86, 0xed, 0x11, 0x88, 0x0e, 0xfe, 0x9f, 0x9f, 0x5a}, {"wifi_conn_charc"}},
    {{0x9e, 0xca, 0Xdc, 0X24, 0x0e, 0xe5, 0xa9, 0xe0, 0x93, 0xf3, 0xa3, 0xb5, 0x02, 0x00, 0x40, 0x6e}, {"logging_charc"}},
    {{0x9e, 0xca, 0Xdc, 0X24, 0x0e, 0xe5, 0xa9, 0xe0, 0x93, 0xf3, 0xa3, 0xb5, 0x03, 0x00, 0x40, 0x6e}, {"console_charc"}},
    {{0xf9, 0xda, 0X32, 0X69, 0xa4, 0x28, 0x4c, 0xb5, 0xa9, 0xe8, 0xfc, 0xbc, 0xeb, 0x35, 0x69, 0xc7}, {"keep_alive_charc"}},
};

results_notify_data_t results_notify_data = {
    .weight = 34.7,
    .battery_volts = 6.0,
    .alarms = 0x0001};

calib_notify_data_t calib_notify_data = {
    .tare_weight = 25,
    .zero_level = 87,
    .full_level = 564,
};   


// Full Database Description - Used to add attributes into the database 

static const esp_gatts_attr_db_t lox_gatt_db[LOX_IDX_NB] =
    {
        // Service Declaration
        [IDX_SVC] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ, ESP_UUID_LEN_128, ESP_UUID_LEN_128, (uint8_t *)ble_charc_uuid[0].uuid}},

        // Characteristic Declaration 
        [IDX_CHAR_RESULTS] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},

        // Characteristic Value 
        [IDX_CHAR_VAL_RESULTS] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)ble_charc_uuid[1].uuid, ESP_GATT_PERM_READ, GATTS_CHAR_VAL_LEN_MAX, sizeof(ble_charc_data.char_value), (uint8_t *)ble_charc_data.char_value}},

        [IDX_CHAR_CFG_DES_RESULTS] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_desc_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, GATTS_CHAR_VAL_LEN_MAX, sizeof(ble_charc_uuid[1].descriptor), (uint8_t *)ble_charc_uuid[1].descriptor}},
        [IDX_CHAR_CFG_RESULTS] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, JSON_PAYLOAD_LEN, JSON_PAYLOAD_LEN, (uint8_t *)&results_notify_data}},

        // Characteristic Declaration 
        [IDX_CHAR_SSID] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

        // Characteristic Value 
        [IDX_CHAR_VAL_SSID] =
            {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_128, (uint8_t *)ble_charc_uuid[2].uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, GATTS_CHAR_VAL_LEN_MAX, sizeof(ble_charc_data.ssid), (uint8_t *)ble_charc_data.ssid}},

        [IDX_CHAR_VAL_SSID_DES] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_desc_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, GATTS_CHAR_VAL_LEN_MAX, sizeof(ble_charc_uuid[2].descriptor), (uint8_t *)ble_charc_uuid[2].descriptor}},

        // Characteristic Declaration 
        [IDX_CHAR_PSWD] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

        // Characteristic Value 
        [IDX_CHAR_VAL_PSWD] =
            {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_128, (uint8_t *)ble_charc_uuid[3].uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, GATTS_CHAR_VAL_LEN_MAX, sizeof(ble_charc_data.password), (uint8_t *)ble_charc_data.password}},

        [IDX_CHAR_VAL_PSWD_DES] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_desc_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, GATTS_CHAR_VAL_LEN_MAX, sizeof(ble_charc_uuid[3].descriptor), (uint8_t *)ble_charc_uuid[3].descriptor}},

        // Characteristic Declaration 
        [IDX_CHAR_SCALE_WEGH] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

        // Characteristic Value 
        [IDX_CHAR_VAL_SCALE_WEGH] =
            {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_128, (uint8_t *)ble_charc_uuid[4].uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, GATTS_CHAR_VAL_LEN_MAX, sizeof(ble_charc_data.ssid), (uint8_t *)ble_charc_data.scale_weight}},

        [IDX_CHAR_VAL_SCALE_WEGH_DES] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_desc_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, GATTS_CHAR_VAL_LEN_MAX, sizeof(ble_charc_uuid[4].descriptor), (uint8_t *)ble_charc_uuid[4].descriptor}},

        // Characteristic Declaration - Read Index
        [IDX_CHAR_INDEX] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

        // Characteristic Value - Read Index
        [IDX_CHAR_VAL_INDEX] =
            {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_128, (uint8_t *)ble_charc_uuid[5].uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, GATTS_CHAR_VAL_LEN_MAX, sizeof(ble_charc_data.index), (uint8_t *)ble_charc_data.index}},

        [IDX_CHAR_VAL_INDEX_DES] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_desc_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, GATTS_CHAR_VAL_LEN_MAX, sizeof(ble_charc_uuid[5].descriptor), (uint8_t *)ble_charc_uuid[5].descriptor}},

        // Characteristic Declaration 
        [IDX_CHAR_FULL_LEV] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, INT_DECLARATION_SIZE, sizeof(uint32_t), (uint8_t *)&char_prop_read_write}},

        // Characteristic Value 
        [IDX_CHAR_VAL_FULL_LEV] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)ble_charc_uuid[6].uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, INT_DECLARATION_SIZE, INT_DECLARATION_SIZE, (uint8_t *)&ble_charc_data.full_level_val}},
        [IDX_CHAR_VAL_FULL_LEV_DES] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_desc_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, GATTS_CHAR_VAL_LEN_MAX, sizeof(ble_charc_uuid[6].descriptor), (uint8_t *)ble_charc_uuid[6].descriptor}},
    
        // Characteristic Declaration
        [IDX_CHAR_CALIB] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},

        // Characteristic Value 
        [IDX_CHAR_VAL_CALIB] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)ble_charc_uuid[7].uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, GATTS_CHAR_VAL_LEN_MAX, sizeof(ble_charc_data.calibration), (uint8_t *)ble_charc_data.calibration}},
        [IDX_CHAR_VAL_CALIB_DES] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_desc_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, GATTS_CHAR_VAL_LEN_MAX, sizeof(ble_charc_uuid[7].descriptor), (uint8_t *)ble_charc_uuid[7].descriptor}},
        // Client Characteristic Configuration Descriptor 
        [IDX_CHAR_CFG_CALIB] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, JSON_PAYLOAD_LEN, JSON_PAYLOAD_LEN, (uint8_t *)&calib_notify_data}},

        // Characteristic Declaration
        [IDX_CHAR_WIFI] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, INT_DECLARATION_SIZE, INT_DECLARATION_SIZE, (uint8_t *)&char_prop_write_notify}},

        // Characteristic Value 
        [IDX_CHAR_VAL_WIFI] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)ble_charc_uuid[8].uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, GATTS_CHAR_VAL_LEN_MAX, sizeof(ble_charc_data.char_value), (uint8_t *)ble_charc_data.char_value}},

        [IDX_CHAR_VAL_WIFI_DES] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, JSON_PAYLOAD_LEN, JSON_PAYLOAD_LEN, (uint8_t *)&results_notify_data}},

        // Characteristic Declaration - logging
        [IDX_CHAR_LOG] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},

        // Characteristic Value  logging
        [IDX_CHAR_VAL_LOG] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)ble_charc_uuid[9].uuid, ESP_GATT_PERM_READ, GATTS_CHAR_VAL_LEN_MAX, sizeof(ble_charc_data.char_value), (uint8_t *)ble_charc_data.char_value}},

        [IDX_CHAR_CFG_LOG] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, JSON_PAYLOAD_LEN, JSON_PAYLOAD_LEN, (uint8_t *)&results_notify_data}},

        // Characteristic Declaration - console
        [IDX_CHAR_CON] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

        // Characteristic Value  console
        [IDX_CHAR_VAL_CON] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)ble_charc_uuid[10].uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, GATTS_CHAR_VAL_LEN_MAX, sizeof(ble_charc_data.console_value), (uint8_t *)ble_charc_data.console_value}},

        [IDX_CHAR_VAL_CON_DES] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_desc_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, GATTS_CHAR_VAL_LEN_MAX, sizeof(ble_charc_uuid[10].descriptor), (uint8_t *)ble_charc_uuid[10].descriptor}},

        [IDX_CHAR_ALIVE] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, INT_DECLARATION_SIZE, INT_DECLARATION_SIZE, (uint8_t *)&char_prop_write_notify}},

        [IDX_CHAR_VAL_ALIVE] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)ble_charc_uuid[11].uuid, ESP_GATT_PERM_WRITE, GATTS_CHAR_VAL_LEN_MAX, sizeof(ble_charc_data.char_value), (uint8_t *)ble_charc_data.char_value}},

 };
  
TimerHandle_t results_timer;
TimerHandle_t calib_timer;
TimerHandle_t ble_SSID_data_timer;
TimerHandle_t ble_PWD_data_timer;

static uint8_t bleRdSnSrcAddr = 0;
static uint16_t bleRdSnMsgCmd = 0;

unsigned char encryptedData[256];
unsigned char decryptedData[256];

//Local Functions
void bleSendMessage(uint8_t dstAddr, uint8_t msgType, uint16_t msgCmd, uint8_t * msgDataPtr, uint32_t msgData, uint32_t msgDataLen);
static void bleSendScaleData(void);

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
void ble_timer_init(void);
void ble_timers_stop(void);

void bleClearNotificationFlags(void);
void ble_adv_start_stop(BLEAdvId_e state);
static void notification_results_data_handler(TimerHandle_t xTimer);
esp_err_t RF_SendDataToMobile(void *arg, BLECharcId_e BLECharcId);
esp_err_t RF_SendDataToMobileLog(void *arg, BLECharcId_e BLECharcId);

static uint8_t find_char_and_desr_index(uint16_t handle);
void ssid_write_handler(uint8_t *data , uint16_t len, esp_gatt_if_t gattsIf,uint16_t handle, uint16_t conn_id, uint32_t trans_id );
void pswd_write_handler(uint8_t *data , uint16_t len, esp_gatt_if_t gattsIf,uint16_t handle, uint16_t conn_id, uint32_t trans_id);
void console_write_handler(uint8_t *data , uint16_t len);
void ssid_read_handler(void);
void pwd_read_handler(void);
void scale_read_handler(void);
void read_index_handler(uint16_t readHandle,uint16_t connId, uint32_t transId, esp_gatt_if_t gattsIf);

void exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gattc_cb_param_t *param);

void bleStoreLog(char * strPtr, bool forced);
void bleSendLog(void);
void bleLogI(char * strPtr);
void bleLogE(char * strPtr);
void bleLogIF(char * strPtr);
void bleLogEF(char * strPtr);

void encryptDataString(const char *toEncrypt, const char *key, uint16_t *strLen, unsigned char *result);
void decryptDataString(const unsigned char *toDecrypt, const char *key, const uint16_t strLen, unsigned char *result);

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

static struct gatts_profile_inst lox_profile_tab[PROFILE_NUM] = {
    [PROFILE_APP_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};


    static esp_ble_adv_data_t lox_adv_config =
        {
            .set_scan_rsp = false,
            .include_txpower = true,
            .min_interval = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
            .max_interval = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
            .appearance = 0x00,
            .manufacturer_len = 0, //TEST_MANUFACTURER_DATA_LEN,
            .p_manufacturer_data =  NULL, //&test_manufacturer[0],
            .service_data_len = 0,
            .p_service_data = NULL,
            .service_uuid_len = sizeof(sec_service_uuid),
            .p_service_uuid = sec_service_uuid,
            .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),            
        };

    // config scan response data
    static esp_ble_adv_data_t lox_scan_rsp_config = {
        .set_scan_rsp = true,
        .include_name = true,
        .manufacturer_len = sizeof(test_manufacturer),
        .p_manufacturer_data = test_manufacturer,
    };   
     

    static esp_ble_adv_params_t lox_adv_params =
        {
            .adv_int_min = 0x100,
            .adv_int_max = 0x100,
            .adv_type = ADV_TYPE_IND,
            .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
            .channel_map = ADV_CHNL_ALL,
            .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
    };  



void bleTaskApp(void)
{

    esp_bt_controller_config_t bt_cfg;
    esp_gatt_rsp_t rsp;
    esp_err_t rspErr;
    char wifiConnNotifyBuff[MAX_MTU_SIZE] = {0};
    uint16_t tmpLength;

    while(1)
    {
        if (xQueueReceive(bleQueueHandle, &bleRxMessage, BLE_MESSAGE_WAIT_DELAY_MS))
        {
			switch(bleRxMessage.msgCmd)
			{
				case BLE_CMD_INIT:
                    bt_cfg.controller_task_stack_size = ESP_TASK_BT_CONTROLLER_STACK;
                    bt_cfg.controller_task_prio = ESP_TASK_BT_CONTROLLER_PRIO;
                    bt_cfg.hci_uart_no = BT_HCI_UART_NO_DEFAULT;
                    bt_cfg.hci_uart_baudrate = BT_HCI_UART_BAUDRATE_DEFAULT;
                    bt_cfg.scan_duplicate_mode = SCAN_DUPLICATE_MODE;
                    bt_cfg.scan_duplicate_type = SCAN_DUPLICATE_TYPE_VALUE;
                    bt_cfg.normal_adv_size = NORMAL_SCAN_DUPLICATE_CACHE_SIZE;
                    bt_cfg.mesh_adv_size = MESH_DUPLICATE_SCAN_CACHE_SIZE;
                    bt_cfg.send_adv_reserved_size = SCAN_SEND_ADV_RESERVED_SIZE;
                    bt_cfg.controller_debug_flag = CONTROLLER_ADV_LOST_DEBUG_BIT;
                    bt_cfg.mode = BTDM_CONTROLLER_MODE_EFF;
                    bt_cfg.ble_max_conn = CONFIG_BTDM_CTRL_BLE_MAX_CONN_EFF;
                    bt_cfg.bt_max_acl_conn = CONFIG_BTDM_CTRL_BR_EDR_MAX_ACL_CONN_EFF;
                    bt_cfg.bt_sco_datapath = CONFIG_BTDM_CTRL_BR_EDR_SCO_DATA_PATH_EFF;
                    bt_cfg.auto_latency = BTDM_CTRL_AUTO_LATENCY_EFF;
                    bt_cfg.bt_legacy_auth_vs_evt = BTDM_CTRL_LEGACY_AUTH_VENDOR_EVT_EFF;
                    bt_cfg.bt_max_sync_conn = CONFIG_BTDM_CTRL_BR_EDR_MAX_SYNC_CONN_EFF;
                    bt_cfg.ble_sca = CONFIG_BTDM_BLE_SLEEP_CLOCK_ACCURACY_INDEX_EFF;
                    bt_cfg.pcm_role = CONFIG_BTDM_CTRL_PCM_ROLE_EFF;
                    bt_cfg.pcm_polar = CONFIG_BTDM_CTRL_PCM_POLAR_EFF;
                    bt_cfg.hli = BTDM_CTRL_HLI;
                    bt_cfg.magic = ESP_BT_CONTROLLER_CONFIG_MAGIC_VAL;

                    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
                    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
                    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
                    ESP_ERROR_CHECK(esp_bluedroid_init());
                    ESP_ERROR_CHECK(esp_bluedroid_enable());

                    bleSendMessage(MSG_ADDR_STRG, MSG_DATA_0, STRG_CMD_RD_SN_INIT, NULL, 0, MSG_DATA_0_LEN);
                    
                    bleSendStatus = BLE_INIT_COMPLETE;
                    bleSendMessage(bleRxMessage.srcAddr, MSG_DATA_8, BLE_CMD_INIT, NULL, bleSendStatus, MSG_DATA_8_LEN);
                    break;

                case BLE_CMD_INIT_ADV:
                    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gatts_event_handler));                    
                    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));
                   
                    ESP_ERROR_CHECK(esp_ble_gatts_app_register(ESP_LOX_APP_ID));
#ifdef   BLE_DATA_GATEWAY                  
                    ble_timer_init();
                    ESP_LOGI(BLE_TAG,"ble timer initilized");
#endif
                    vTaskDelay(pdMS_TO_TICKS(10));
                    esp_ble_gap_stop_advertising();

                    //Load ble parameters from storage
                    bleSendMessage(MSG_ADDR_STRG, MSG_DATA_0, STRG_CMD_RD_SSID, NULL, 0, MSG_DATA_0_LEN);
                    bleSendMessage(MSG_ADDR_STRG, MSG_DATA_0, STRG_CMD_RD_PWD, NULL, 0, MSG_DATA_0_LEN);                   

                    bleStatus.is_connected = false;
                    bleStatus.is_log_notify_enabled = false;
                    bleStatus.is_scale_notify_enabled = false;                    
                    bleStatus.is_initialized = true;

                    bleSendStatus = BLE_RETURN_GOOD;

                    bleSendMessage(bleRxMessage.srcAddr, MSG_DATA_8, BLE_CMD_INIT_ADV, NULL, bleSendStatus, MSG_DATA_8_LEN);

                    break;
				case BLE_CMD_PING:
                    bleSendMessage(bleRxMessage.srcAddr, MSG_DATA_8, BLE_CMD_PING, NULL, BLE_PING_RECEIVED, MSG_DATA_8_LEN);
					break;
				case BLE_CMD_STATUS:
					//TBD
					break;


                case BLE_CMD_ADV_START:
                    ble_adv_start_stop(START_ADV);
                    bleReturnStatus = BLE_RETURN_GOOD;
                    bleSendMessage(bleRxMessage.srcAddr, MSG_DATA_8, bleRxMessage.msgCmd, NULL, bleReturnStatus, MSG_DATA_8_LEN);                    
                    break;

                case BLE_CMD_GET_IDLE_TM:
                    ESP_LOGI(BLE_TAG,"BLE inactivity time: %ld ms", sys_getMsgTimeStamp() - bleLastActivityTime);
                    bleSendMessage(bleRxMessage.srcAddr, MSG_DATA_32, bleRxMessage.msgCmd, NULL, sys_getMsgTimeStamp() - bleLastActivityTime, MSG_DATA_32_LEN);
                    break;

                case BLE_CMD_ADV_STOP:
                    ble_adv_start_stop(STOP_ADV);
                    bleReturnStatus = BLE_RETURN_GOOD;
                    bleSendMessage(bleRxMessage.srcAddr, MSG_DATA_8, bleRxMessage.msgCmd, NULL, bleReturnStatus, MSG_DATA_8_LEN);                    
                    break;   

                case BLE_CMD_SET_SN:
                    bleRdSnSrcAddr      = bleRxMessage.srcAddr;     //Address to send response to
                    bleRdSnMsgCmd       = bleRxMessage.msgCmd;      //Command to send back
                    bleSendMessage(MSG_ADDR_STRG, MSG_DATA_0, STRG_CMD_RD_SN, NULL, 0, MSG_DATA_0_LEN);
                    break;    

                //log testing commands:

                case BLE_CMD_SEND_LOG:
                    bleStoreLog((char *)bleRxMessage.msgDataPtr, false);
                    break;        

                case BLE_CMD_SEND_FORCED_LOG:
                    bleStoreLog((char *)bleRxMessage.msgDataPtr, true);
                    break;        

                case BLE_CMD_ADV_TEST1:
                    memset(advSerialNumber, 0x00, sizeof(advSerialNumber));
                    sprintf(advSerialNumber,"%s:1234:%d.%d.%d", BLE_DEVICE_PREFIX, FIRMWARE_VERSION_RELEASE, FIRMWARE_VERSION_MAJOR, FIRMWARE_VERSION_MINOR);

                    ESP_ERROR_CHECK(esp_ble_gap_stop_advertising());
                    ESP_ERROR_CHECK(esp_ble_gap_set_device_name(advSerialNumber));
                    ESP_ERROR_CHECK(esp_ble_gap_config_adv_data(&lox_scan_rsp_config));                 
                    ESP_ERROR_CHECK(esp_ble_gap_start_advertising(&lox_adv_params));
                    break;

                //Responses to commands   
                case STRG_CMD_RD_SN_INIT:
                case STRG_CMD_RD_SN:
                    memset(advSerialNumber, 0x00, sizeof(advSerialNumber));
                    memset(scaleSerialNumber, 0x00, sizeof(scaleSerialNumber));
                    memcpy(&scaleSerialNumber, (char *)bleRxMessage.msgDataPtr, bleRxMessage.msgDataLen);
                    sprintf(advSerialNumber,"%s:%s:%d.%d.%d", BLE_DEVICE_PREFIX, scaleSerialNumber, FIRMWARE_VERSION_RELEASE, FIRMWARE_VERSION_MAJOR, FIRMWARE_VERSION_MINOR);
                    ESP_LOGI(BLE_TAG, "Serial Number read by BLE:%s", advSerialNumber);
                    if (bleRxMessage.msgCmd == STRG_CMD_RD_SN)
                    {
                        bleSendStatus = BLE_RETURN_GOOD;
                        bleSendMessage(bleRdSnSrcAddr, MSG_DATA_8, bleRdSnMsgCmd, NULL, bleSendStatus, MSG_DATA_8_LEN); 
                    }
                    break;

                // case BLE_MEAS_NOTIFY_CMD:
                //     if (bleStatus.send_scale_data){
                //         bleStatus.scale_weight_received = true;
                //         results_notify_data.weight = bleRxMessage.msgData / 10.0;
                //         ESP_LOGI(BLE_TAG, "Scale Data received:%.02f kg", results_notify_data.weight);
                //     }
                //     break;

                // case BLE_BATT_NOTIFY_CMD:
                //     if (bleStatus.send_scale_data){
                //         bleStatus.battery_volts_received = true;
                //         results_notify_data.battery_volts = bleRxMessage.msgData / 10.0;
                //         ESP_LOGI(BLE_TAG, "Battery Data received:%.02f Volts", results_notify_data.battery_volts);
                //     }
                //     break;

                // case BLE_ALM_NOTIFY_CMD:
                //     if (bleStatus.send_scale_data){
                //         bleStatus.alarm_status_received = true;
                //         results_notify_data.alarms = bleRxMessage.msgData;
                //     }
                //     break;

                // case STRG_CMD_RD_APP_SSID:
                //     ESP_LOGI(BLE_TAG, "SSID received from storage: %s", (char *)bleRxMessage.msgDataPtr);
                //     ESP_LOGI(BLE_TAG, "conn_id %d, trans_id %" PRIu32 ", gatts_if %d", 
                //             bleSsidReadConnId, bleSsidReadTransId, bleSsidGattsIf);

                //     memcpy(bleTmpStr, (char *)bleRxMessage.msgDataPtr, bleRxMessage.msgDataLen);
                //     memset(encryptedData,0,sizeof(encryptedData));
                   
                //     encryptDataString(bleTmpStr, encryptionKey, &tmpLength, encryptedData);

                //     memset(&rsp, 0, sizeof(esp_gatt_rsp_t));  
                //     rsp.attr_value.handle = bleSsidReadHandle;  
                //     rsp.attr_value.len = bleRxMessage.msgDataLen;  
                //     memcpy(rsp.attr_value.value,encryptedData, tmpLength);

                //     esp_ble_gatts_send_response(bleSsidGattsIf,//gatts_if,  
                //                                 bleSsidReadConnId,//param->read.conn_id,  
                //                                 bleSsidReadTransId,//param->read.trans_id,  
                //                                 ESP_GATT_OK, &rsp);
                //     break;  

                // case STRG_CMD_RD_APP_PWD:
                //     ESP_LOGI(BLE_TAG, "Password received from storage: %s", (char *)bleRxMessage.msgDataPtr);
                //     ESP_LOGI(BLE_TAG, "conn_id %d, trans_id %" PRIu32 ", gatts_if %d", 
                //             blePwdReadConnId, blePwdReadTransId, blePwdGattsIf);

                //     memcpy(bleTmpStr, (char *)bleRxMessage.msgDataPtr, bleRxMessage.msgDataLen);
                //     memset(encryptedData,0,sizeof(encryptedData));

                //     encryptDataString(bleTmpStr, encryptionKey, &tmpLength, encryptedData);

                //     memset(&rsp, 0, sizeof(esp_gatt_rsp_t));  
                //     rsp.attr_value.handle = blePwdReadHandle;  
                //     rsp.attr_value.len = bleRxMessage.msgDataLen;  
                //     memcpy(rsp.attr_value.value,encryptedData, tmpLength);

                //     esp_ble_gatts_send_response(blePwdGattsIf,//gatts_if,  
                //                                 blePwdReadConnId,//param->read.conn_id,  
                //                                 blePwdReadTransId,//param->read.trans_id,  
                //                                 ESP_GATT_OK, &rsp);
                //     break;  

                //Responses to BLE commands
                // case SCALE_CMD_RD_SW:
                //     //ESP_LOGI(BLE_TAG, "Scale weight received :%.01f",bleRxMessage.msgData/10.0 );
                //     sprintf(bleTmpCharStr, "%.01f", bleRxMessage.msgData/10.0);
                //     ESP_LOGI(BLE_TAG, "Scale weight received :%s",bleTmpCharStr);
                //     ESP_LOGI(BLE_TAG, "conn_id %d, trans_id %" PRIu32 ", gatts_if %d", 
                //             bleRwReadConnId, bleRwReadTransId, bleRwGattsIf);
        
                //     memcpy(ble_charc_data.scale_weight, &bleTmpCharStr, strlen(bleTmpCharStr));
                //     memset(&rsp, 0, sizeof(esp_gatt_rsp_t));  
                //     rsp.attr_value.handle = bleRwReadHandle; //param->read.handle;  
                //     rsp.attr_value.len = strlen(bleTmpCharStr);  
                //     memcpy(rsp.attr_value.value,ble_charc_data.scale_weight, strlen(bleTmpCharStr));


                //     esp_ble_gatts_send_response(bleRwGattsIf,//gatts_if,  
                //                                 bleRwReadConnId,//param->read.conn_id,  
                //                                 bleRwReadTransId,//param->read.trans_id,  
                //                                 ESP_GATT_OK, &rsp);
                //     break;

                case EVT_CMD_BLE_WIFI_CONNECT:
                    sprintf(wifiConnNotifyBuff,"%d",(uint8_t)bleRxMessage.msgData);
                    RF_SendDataToMobile((uint8_t *)wifiConnNotifyBuff, WIFI_CONNECT_HANDLE);
                    ESP_LOGI(BLE_TAG, "Sent wifi connect notification %s to phone",wifiConnNotifyBuff);  

                    switch (bleRxMessage.msgData)
                    {
                        case WIFI_CONNECTING:
                            ESP_LOGI(BLE_TAG,"Notification to wifi connect UUID: WIFI_CONNECTING");
                            break;
                            
                        case WIFI_CONNECTION_FAILED:
                            ESP_LOGI(BLE_TAG,"Notification to wifi connect UUID: WIFI_CONNECTION_FAILED");
                            break;
                            
                        case WIFI_CONNECTION_PASSED:
                            ESP_LOGI(BLE_TAG,"Notification to wifi connect UUID: WIFI_CONNECTION_PASSED");
                            break;

                        case ENTER_TANK_CALIBRATION_MODE:
                            ESP_LOGI(BLE_TAG,"Notification to wifi connect UUID: ENTER_TANK_CALIBRATION_MODE");
                            break;

                        case TANK_WEIGHT_STABILIZING:
                            ESP_LOGI(BLE_TAG,"Notification to wifi connect UUID: TANK_WEIGHT_STABILIZING");
                            break;

                        case TANK_WEIGHT_READY:
                            ESP_LOGI(BLE_TAG,"Notification to wifi connect UUID: TANK_WEIGHT_READY");
                            break;

                        case TANK_WEIGHT_ERROR:
                            ESP_LOGI(BLE_TAG,"Notification to wifi connect UUID: TANK_WEIGHT_ERROR");
                            break;

                        case TANK_WEIGHT_REMOVED:
                            ESP_LOGI(BLE_TAG,"Notification to wifi connect UUID: TANK_WEIGHT_REMOVED");
                            break;

                        case EXIT_TANK_CALIBRATION_MODE:
                            ESP_LOGI(BLE_TAG,"Notification to wifi connect UUID: EXIT_TANK_CALIBRATION_MODE");
                            break;

                        case SCALE_GOING_TO_SLEEP:
                            ESP_LOGI(BLE_TAG,"Notification to wifi connect UUID: SCALE_GOING_TO_SLEEP");
                            break;

                        default:
                            ESP_LOGI(BLE_TAG,"Notification to wifi connect UUID: WIFI_CONNECTION_FAILED!");                           
                    }
                    break;

				default:;

			}

        }
        bleSendLog();   //Send any logs to the mobile device if connected and log notification is enabled.
        bleSendScaleData(); //Send notification data if available.
    }
 
}

/// @brief 
/// @param event 
/// @param param 
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    ESP_LOGI(BLE_TAG, "gap_event_handler - event:%d", event );

    //reset ble inactivity time
    bleLastActivityTime = sys_getMsgTimeStamp();

    switch (event)
    {
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        /* advertising start complete event to indicate advertising start successfully or failed */
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(BLE_TAG, "advertising start failed, error status = %x", param->adv_start_cmpl.status);
        }
        else
        {
            bleSendMessage(MSG_ADDR_SCTL, MSG_DATA_0, EVT_BLE_ADV_ON, NULL, MSG_DATA_COMMAND_ONLY, MSG_DATA_0_LEN);
            ESP_LOGI(BLE_TAG, "advertising start successfully");
        }

        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(BLE_TAG, "Advertising stop failed");
        }
        else
        {
            ESP_LOGI(BLE_TAG, "Stop adv successfully\n");
            bleSendMessage(MSG_ADDR_SCTL, MSG_DATA_0, EVT_BLE_ADV_OFF, NULL, MSG_DATA_COMMAND_ONLY, MSG_DATA_0_LEN);
        }
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
        ESP_LOGI(BLE_TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                 param->update_conn_params.status,
                 param->update_conn_params.min_int,
                 param->update_conn_params.max_int,
                 param->update_conn_params.conn_int,
                 param->update_conn_params.latency,
                 param->update_conn_params.timeout);

        break;

    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
        if (adv_config_done == 0){
            //esp_ble_gap_start_advertising(&lox_adv_params);
        }
        break;

    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~ADV_CONFIG_FLAG);
        if (adv_config_done == 0){
            //esp_ble_gap_start_advertising(&lox_adv_params);
        }
        break;

    case ESP_GAP_BLE_SET_LOCAL_PRIVACY_COMPLETE_EVT:
        if (param->local_privacy_cmpl.status != ESP_BT_STATUS_SUCCESS){
            ESP_LOGE(BLE_TAG, "config local privacy failed, error status = %x", param->local_privacy_cmpl.status);
            break;
        }

        esp_err_t ret = esp_ble_gap_config_adv_data(&lox_adv_config);
        if (ret){
            ESP_LOGE(BLE_TAG, "config adv data failed, error code = %x", ret);
        }else{
            adv_config_done |= ADV_CONFIG_FLAG;
        }

        ret = esp_ble_gap_config_adv_data(&lox_scan_rsp_config);
        if (ret){
            ESP_LOGE(BLE_TAG, "config adv data failed, error code = %x", ret);
        }else{
            adv_config_done |= SCAN_RSP_CONFIG_FLAG;
        }

        break;

    default:
        break;
    }
}

/// @brief API to Gatt Event handler
/// @param event 
/// @param gatts_if 
/// @param param 
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    ESP_LOGI(BLE_TAG, "gatts_event_handler - EVT %d, gatts if %d\n", event, gatts_if);

    //reset ble inactivity time
    bleLastActivityTime = sys_getMsgTimeStamp();

    // If event is register event, store the gatts_if for each profile
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            lox_profile_tab[PROFILE_APP_IDX].gatts_if = gatts_if;
        } else {          
            ESP_LOGI(BLE_TAG, "Reg app failed, app_id %04x, status %d\n",param->reg.app_id, param->reg.status);
            return;
        }
    }

    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                    gatts_if == lox_profile_tab[idx].gatts_if) {
                if (lox_profile_tab[idx].gatts_cb) {
                    lox_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);

}

/// @brief Initalize ble timer
/// @param  None
void ble_timer_init(void)
{
    results_timer = xTimerCreate("tick_timer", (NOTIFICATION_TIMER / portTICK_PERIOD_MS),
                                 pdTRUE,
                                 NULL,
                                 &notification_results_data_handler);
}


void bleClearNotificationFlags(void)
{
    bleStatus.send_scale_data           = false;
    bleStatus.scale_weight_received     = false;
    bleStatus.battery_volts_received  = false;
    bleStatus.alarm_status_received     = false;
}


/// @brief Stop ble timmer
/// @param  None
void ble_timers_stop(void)
{
    bleClearNotificationFlags();
    if (NULL != results_timer)
    {
        if (pdTRUE == xTimerIsTimerActive(results_timer))
        {
            xTimerStop(results_timer, portMAX_DELAY);
            ESP_LOGI(BLE_TAG, "results_timer stopped ");
        }
    }
}

/// @brief Start or Stop advertising depending on the state
/// @param state - see BLEAdvId_e enum
void ble_adv_start_stop(BLEAdvId_e state)
{
    ESP_LOGI(BLE_TAG, "ble_conn.is_ble_connected   %d ", ble_conn.is_ble_connected);
    switch (state)
    {
    case START_ADV:
    {
        ESP_LOGI(BLE_TAG, "ble_adv_start ");
        esp_ble_gap_start_advertising(&lox_adv_params);
    }
    break;
    case STOP_ADV:
    {
        ESP_LOGI(BLE_TAG, "ble_adv_stop ");
        esp_ble_gap_stop_advertising();
    }
    break;
    default:
        break;
    }
}


/// @brief Send data to mobile app
/// @param xTimer 
static void notification_results_data_handler(TimerHandle_t xTimer)
{
    if (!bleStatus.send_scale_data){
        bleStatus.send_scale_data  = true;

        bleStatus.alarm_status_received = true; //JAR 20230102 - need to remove this when Alarm module available

        //test code. remove when uncommenting the two messages above
        bleStatus.scale_weight_received    = true;    //Set when the requested scale data has been received
        bleStatus.battery_volts_received   = true;    //Set when the requested battery volts full has been received

        ESP_LOGI(BLE_TAG, "notification data request sent to tasks");
    }
}

/// @brief Send received notification data to the phone
/// @param  
static void bleSendScaleData(void)
{
    //esp_err_t err;

    uint8_t notify_buffer[MAX_MTU_SIZE] = {0};

    if( bleStatus.send_scale_data         &&
        bleStatus.scale_weight_received   &&
        bleStatus.battery_volts_received  &&
        bleStatus.alarm_status_received
        ){  
        ESP_LOGI(BLE_TAG, "notification data received from tasks");  

        memset(notify_buffer,0,sizeof(notify_buffer));
        sprintf(bleTmpStr, "{\"src\":\"wifi\",\"sn\":\"JIM064023\",\"fw\":\"0.1.29\",\"hw\":\"A\",\"md\":\"13.6,-40927100,7.8,1691780965000\",\"alm\":\"0\"}");
        memcpy(notify_buffer,bleTmpStr, strlen(bleTmpStr)+1); 

        if (bleStatus.is_scale_notify_enabled){
            RF_SendDataToMobile(notify_buffer, RESULTS_HANDLE);
        }

        if (bleStatus.is_log_notify_enabled){
            RF_SendDataToMobileLog(notify_buffer, LOG_HANDLE);
        }  
        bleStatus.send_scale_data = false;
        bleStatus.scale_weight_received = false;
        bleStatus.battery_volts_received = false; 
    }
}


/// @brief 
/// @param arg 
/// @param BLECharcId - see enum BLECharcId_e
/// @return ESP_OK or ESP_FAIL
esp_err_t RF_SendDataToMobile(void *arg, BLECharcId_e BLECharcId)
{
    esp_err_t ret = ESP_OK;
    uint16_t conn_id = 0x00;
    esp_gatt_if_t gatts_if = 3;

    if ((NULL != arg))
    {
        uint16_t len = strlen(arg);
        if (len <= ESP_GATT_MAX_MTU_SIZE)
        {
           ret = esp_ble_gatts_send_indicate(gatts_if, conn_id, BLECharcId,
                                              len, (uint8_t *)arg, false);
        }
        else
        {
            ret = ESP_FAIL;
        }
    }
    return ret;
}


esp_err_t RF_SendDataToMobileLog(void *arg, BLECharcId_e BLECharcId)
{
    uint8_t tempMsg[ESP_MAX_SEND_SIZE] = {0};
    uint8_t sendMsg[ESP_MAX_SEND_SIZE] = {0};
    uint8_t sentIdx = 0;
    esp_err_t ret = ESP_OK;
    uint16_t conn_id = 0x00;
    esp_gatt_if_t gatts_if = 3;
    uint16_t remainLen = 0;
    bool complete = false;

    if ((NULL != arg))
    {
        remainLen = strlen(arg);
        memcpy(tempMsg,arg,remainLen);   

        while (!complete)
        {
            if (remainLen <= ESP_MAX_NOTIFY_SIZE)
            {
                memset(sendMsg, 0, sizeof(sendMsg));
                memcpy(sendMsg,&tempMsg[sentIdx], remainLen);
                ret = esp_ble_gatts_send_indicate(gatts_if, conn_id, BLECharcId,
                                                remainLen, (uint8_t *)sendMsg, false);
                complete = true;                                
            }
            else
            {
                memcpy(sendMsg,&tempMsg[sentIdx], ESP_MAX_NOTIFY_SIZE);
                ret = esp_ble_gatts_send_indicate(gatts_if, conn_id, BLECharcId,
                                                ESP_MAX_NOTIFY_SIZE, (uint8_t *)sendMsg, false);
                sentIdx += ESP_MAX_NOTIFY_SIZE; 
                remainLen -= ESP_MAX_NOTIFY_SIZE;                               
            }
        }
    }
    return ret;
}

/**
 * @brief: gatts_profile_event  handler
 * @param[in] event
 * @param[in] gatts_if
 * @param[in] gatts parameters value
 * @param[out] void
 */

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    esp_ble_gatts_cb_param_t *p_data = (esp_ble_gatts_cb_param_t *) param;
    uint8_t res = 0xff;
    uint16_t descr_value = 0;

    ESP_LOGI(BLE_TAG, "gatts_profile_event_handler - event = %x\n",event);

    //reset ble inactivity time
    bleLastActivityTime = sys_getMsgTimeStamp();

    switch (event) {
    	case ESP_GATTS_MTU_EVT:
            ESP_LOGI(BLE_TAG, "ESP_GATTS_MTU_EVT, MTU%d", param->mtu.mtu);
            break;


    	case ESP_GATTS_REG_EVT:
            esp_ble_gap_set_device_name(advSerialNumber);
            esp_ble_gap_config_local_privacy(true);
            esp_ble_gatts_create_attr_tab(lox_gatt_db, gatts_if,                 
                                      LOX_IDX_NB, LOX_SVC_INST_ID);
            break;

    	case ESP_GATTS_READ_EVT:
            ESP_LOGI(BLE_TAG, "gatts_profile_event_handler - GATT_READ_EVT, conn_id %d, trans_id %" PRIu32 ", handle %d\n", p_data->read.conn_id, p_data->read.trans_id, p_data->read.handle);

            switch(p_data->read.handle)
            {
                case RESULTS_HANDLE:
                    ESP_LOGI(BLE_TAG, "process read from Scale handle");
                    break;
                    
                case SSID_HANDLE:
                    ESP_LOGI(BLE_TAG, "process read from SSID handle");
                    bleSsidGattsIf = gatts_if;
                    bleSsidReadHandle = p_data->read.handle;
                    bleSsidReadConnId = p_data->read.conn_id;
                    bleSsidReadTransId = p_data->read.trans_id; 
                    ESP_LOGI(BLE_TAG, "conn_id %d, trans_id %" PRIu32 ", handle %d, gatts_if %d", p_data->read.conn_id, p_data->read.trans_id, p_data->read.handle, gatts_if);   
                    ssid_read_handler();
                    break;
                    
                case PSWD_HANDLE:
                    ESP_LOGI(BLE_TAG, "process read from Pwd handle");
                    blePwdGattsIf = gatts_if;
                    blePwdReadHandle = p_data->read.handle;
                    blePwdReadConnId = p_data->read.conn_id;
                    blePwdReadTransId = p_data->read.trans_id;  
                    ESP_LOGI(BLE_TAG, "conn_id %d, trans_id %" PRIu32 ", handle %d,  gatts_if %d", p_data->read.conn_id, p_data->read.trans_id, p_data->read.handle, gatts_if);                  
                    pwd_read_handler();                    
                    break;
                    
                case SCALE_WEIGHT_HANDLE:
                    ESP_LOGI(BLE_TAG, "process read from SW handle");
                    bleRwGattsIf = gatts_if;
                    bleRwReadHandle = p_data->read.handle;
                    bleRwReadConnId = p_data->read.conn_id;
                    bleRwReadTransId = p_data->read.trans_id;
                    ESP_LOGI(BLE_TAG, "conn_id %d, trans_id %" PRIu32 ", handle %d,  gatts_if %d", p_data->read.conn_id, p_data->read.trans_id, p_data->read.handle, gatts_if);
                    scale_read_handler();
                    break;

                case WIFI_CONNECT_HANDLE:
                    ESP_LOGI(BLE_TAG, "process read from wifi handle");
                    break;

                case BLE_READ_INDEX_HANDLE:
                    ESP_LOGI(BLE_TAG, "process write to BLE_READ_INDEX_HANDLE handle");
                    ESP_LOGI(BLE_TAG, "conn_id %d, trans_id %" PRIu32 ", handle %d, gatts_if %d", p_data->read.conn_id, p_data->read.trans_id, p_data->read.handle, gatts_if);   

                    esp_gatt_rsp_t rsp;    
                    
                    ESP_LOGI(BLE_TAG, "read_index_handler");

                    memset(bleTmpCharStr,0,sizeof(bleTmpCharStr));
                    sprintf(bleTmpCharStr, "0");

                    memset(ble_charc_data.index, 0x00, sizeof(ble_charc_data.index));
                    (void)memcpy(ble_charc_data.index, bleTmpCharStr, strlen(bleTmpCharStr));    

                    memset(&rsp, 0, sizeof(esp_gatt_rsp_t));  
                    rsp.attr_value.handle = p_data->read.handle; //param->read.handle;  
                    rsp.attr_value.len = strlen(bleTmpCharStr);  
                    memcpy(rsp.attr_value.value,ble_charc_data.index, strlen(bleTmpCharStr));

                    ESP_LOGI(BLE_TAG,"Index:%s", ble_charc_data.index);

                    esp_ble_gatts_send_response(gatts_if,  
                                                param->read.conn_id,  
                                                param->read.trans_id,  
                                                ESP_GATT_OK, &rsp);


                    break;                    

                default:;
            }
       	    break;

    	case ESP_GATTS_WRITE_EVT: {
            ESP_LOGI(BLE_TAG, "ESP_GATTS_WRITE_EVT - handle index = %d", p_data->write.handle);
    	    res = find_char_and_desr_index(p_data->write.handle);
            ESP_LOGI(BLE_TAG, "ESP_GATTS_WRITE_EVT - handle = %d", res);

            switch(p_data->write.handle)
            {
                case RESULTS_HANDLE:
                    ESP_LOGI(BLE_TAG, "process write to Scale handle");
                    break;
                    
                case SSID_HANDLE:
                    ESP_LOGI(BLE_TAG, "process write to SSID handle");
                    ssid_write_handler(p_data->write.value, p_data->write.len, gatts_if,p_data->write.handle,p_data->write.conn_id,p_data->write.trans_id);
                    break;
                    
                case PSWD_HANDLE:
                    ESP_LOGI(BLE_TAG, "process write to Pwd handle");
                    pswd_write_handler(p_data->write.value, p_data->write.len, gatts_if,p_data->write.handle,p_data->write.conn_id,p_data->write.trans_id);
                    break;

                case CALIB_HANDLE:
                    ESP_LOGI(BLE_TAG, "process write to cal handle");
                    break;
                    
                case WIFI_CONNECT_HANDLE:
                    descr_value = param->write.value[1] << 8 | param->write.value[0];
                    ESP_LOGI(BLE_TAG, "process write to wifi handle, value:%d", descr_value);

                    switch(descr_value)
                    {
                        case CHAR_WIFI_CONNECT_FUNC:
                            ESP_LOGI(BLE_TAG, "Send EVT_CMD_BLE_WIFI_CONNECT to system controller");
                            bleSendMessage(MSG_ADDR_SCTL, MSG_DATA_0, EVT_CMD_BLE_WIFI_CONNECT, NULL, 0, MSG_DATA_0_LEN);
                            break;

                        case CHAR_WIFI_ENTER_TANK_CAL:
                            ESP_LOGI(BLE_TAG, "Send EVT_ENTER_TANK_CAL to system controller");
                            bleSendMessage(MSG_ADDR_SCTL, MSG_DATA_0, EVT_ENTER_TANK_CAL, NULL, 0, MSG_DATA_0_LEN);
                            break;

                        case CHAR_WIFI_EXIT_TANK_CAL:
                            ESP_LOGI(BLE_TAG, "Send EVT_EXIT_TANK_CAL to system controller");
                            bleSendMessage(MSG_ADDR_SCTL, MSG_DATA_0, EVT_EXIT_TANK_CAL, NULL, 0, MSG_DATA_0_LEN);
                            break;

                        default:
                            ESP_LOGE(BLE_TAG, "WIFI handle command undefined!");    
                    }

                    break;

                case ALIVE_CONNECT_HANDLE:   
                    descr_value = param->write.value[1] << 8 | param->write.value[0];
                    ESP_LOGI(BLE_TAG, "process write to keep alive handle, value:%d", descr_value);
                    bleSendMessage(MSG_ADDR_SCTL, MSG_DATA_0, EVT_BLE_KEEP_ALIVE, NULL, 0, MSG_DATA_0_LEN);
                    break;

                case RESULTS_NOTIFY_HANDLE:
                    ESP_LOGI(BLE_TAG, "process write Notifications to Scale data handle");
                    descr_value = param->write.value[1] << 8 | param->write.value[0];
                    if (descr_value == 0x0001)
                    {
                        ESP_LOGI(BLE_TAG, "Measure notify enabled");
                        bleStatus.is_scale_notify_enabled = true;
                        if (!bleStatus.is_log_notify_enabled){
                            ESP_LOGI(BLE_TAG, "results_timer started");
                            bleClearNotificationFlags();
#ifdef BLE_DATA_GATEWAY
                            xTimerStart(results_timer, portMAX_DELAY);
#endif                        
                        }
                    }
                    else if (descr_value == 0x0000)
                    {
                        ESP_LOGI(BLE_TAG, "Scale notify/indicate disabled");
                        bleStatus.is_scale_notify_enabled = false;                        
                        if (!bleStatus.is_log_notify_enabled)
                        {
                            xTimerStop(results_timer, portMAX_DELAY);
                            bleClearNotificationFlags();
                            ESP_LOGI(BLE_TAG, "results_timer stopped");
                        }
                    }
                    else
                    {
                        ESP_LOGI(BLE_TAG, "Measure unknown descr value");
                    }
                    break;                    
                    
                case WIFI_NOTIFY_HANDLE:
                    ESP_LOGI(BLE_TAG, "process write Notifications to Wifi data handle");
                    descr_value = param->write.value[1] << 8 | param->write.value[0];
                    if (descr_value == 0x0001){
                        ESP_LOGI(BLE_TAG, "Wifi notify enabled");
                        bleStatus.is_wifi_notify_enabled = true;
                    }else if (descr_value == 0x0000){
                        ESP_LOGI(BLE_TAG, "Wifi notify disabled");
                        bleStatus.is_wifi_notify_enabled = false;
                    }
                    break;                    
                    
                case LOG_HANDLE:
                    ESP_LOGI(BLE_TAG, "process write to log handle");
                    break;

                case LOG_NOTIFY_HANDLE:
                    ESP_LOGI(BLE_TAG, "process write Notifications to log data handle");
                    descr_value = param->write.value[1] << 8 | param->write.value[0];
                    if (descr_value == 0x0001)
                    {
                        ESP_LOGI(BLE_TAG, "log notify enabled");
                        bleStatus.is_log_notify_enabled = true;
                        if (!bleStatus.is_scale_notify_enabled){
                            ESP_LOGI(BLE_TAG, "results_timer started");
                        }                    }
                    else if (descr_value == 0x0000){
                        ESP_LOGI(BLE_TAG, "Log notify/indicate disabled");
                        bleStatus.is_log_notify_enabled = false;
                        if (!bleStatus.is_scale_notify_enabled){
                            ESP_LOGI(BLE_TAG, "results_timer stopped");
                            xTimerStop(results_timer, portMAX_DELAY);
                        }
                    }
                    else
                    {
                        ESP_LOGI(BLE_TAG, "log unknown descr value");
                    }
                    break;    

                case CON_HANDLE:
                    ESP_LOGI(BLE_TAG, "process write to console handle");
                    console_write_handler(param->write.value, param->write.len);
                    break;
                default:;
            }


            if(p_data->write.is_prep == false){
                ESP_LOGI(BLE_TAG, "ESP_GATTS_WRITE_EVT : handle = %d\n", res);
                }
      	 	break;
    	}
    	case ESP_GATTS_EXEC_WRITE_EVT:{
    	    ESP_LOGI(BLE_TAG, "ESP_GATTS_EXEC_WRITE_EVT\n");
    	    }
    	    break;
    	case ESP_GATTS_CONF_EVT:
    	    break;
    	case ESP_GATTS_UNREG_EVT:
        	break;
    	case ESP_GATTS_DELETE_EVT:
        	break;
    	case ESP_GATTS_START_EVT:
        	break;
    	case ESP_GATTS_STOP_EVT:
        	break;
    	case ESP_GATTS_CONNECT_EVT:
    	    lox_conn_id = p_data->connect.conn_id;
    	    lox_gatts_if = gatts_if;
    	    bleStatus.is_connected = true;
    	    memcpy(&lox_remote_bda,&p_data->connect.remote_bda,sizeof(esp_bd_addr_t));
            ESP_LOGI(BLE_TAG, "ESP_GATTS_CONNECT_EVT");
            bleSendMessage(MSG_ADDR_SCTL, MSG_DATA_0, EVT_BLE_CONNECT, NULL, MSG_DATA_COMMAND_ONLY, MSG_DATA_0_LEN);            
            /* start security connect with peer device when receive the connect event sent by the master */
            break;

    	case ESP_GATTS_DISCONNECT_EVT:
    	    bleStatus.is_connected = false;
    	    bleStatus.enable_data_ntf = false;
            ESP_LOGI(BLE_TAG, "Advertising on");
    	    esp_ble_gap_start_advertising(&lox_adv_params);
            ESP_LOGI(BLE_TAG, "ESP_GATTS_DISCONNECT_EVT, disconnect reason 0x%x", param->disconnect.reason);
            switch (param->disconnect.reason)
            {
                case ESP_GATT_CONN_UNKNOWN:                 ESP_LOGI(BLE_TAG,"Gatt connection unknown");                break;
                case ESP_GATT_CONN_L2C_FAILURE:             ESP_LOGI(BLE_TAG,"General L2cap failure");                  break;
                case ESP_GATT_CONN_TIMEOUT:                 ESP_LOGI(BLE_TAG,"Connection timeout");                     break;
                case ESP_GATT_CONN_TERMINATE_PEER_USER:     ESP_LOGI(BLE_TAG,"Connection terminate by peer user");      break;
                case ESP_GATT_CONN_TERMINATE_LOCAL_HOST:    ESP_LOGI(BLE_TAG,"Connection terminated by local host");    break;
                case ESP_GATT_CONN_FAIL_ESTABLISH:          ESP_LOGI(BLE_TAG,"Connection fail to establish");           break;
                case ESP_GATT_CONN_LMP_TIMEOUT:             ESP_LOGI(BLE_TAG,"Connection fail for LMP response tout");  break;
                case ESP_GATT_CONN_CONN_CANCEL:             ESP_LOGI(BLE_TAG,"L2CAP connection cancelled");             break;
                case ESP_GATT_CONN_NONE:                    ESP_LOGI(BLE_TAG,"No connection to cancel");                break;
                default:                                    ESP_LOGI(BLE_TAG,"Gatt connection unknown");                
            }
            bleSendMessage(MSG_ADDR_SCTL, MSG_DATA_0, EVT_BLE_DISCONNECT, NULL, MSG_DATA_COMMAND_ONLY, MSG_DATA_0_LEN);
    	    break;
    	case ESP_GATTS_OPEN_EVT:
    	    break;
    	case ESP_GATTS_CANCEL_OPEN_EVT:
    	    break;
    	case ESP_GATTS_CLOSE_EVT:
    	    break;
    	case ESP_GATTS_LISTEN_EVT:
    	    break;
    	case ESP_GATTS_CONGEST_EVT:
    	    break;
    	case ESP_GATTS_CREAT_ATTR_TAB_EVT:
    	    ESP_LOGI(BLE_TAG, "The number handle =%x\n",param->add_attr_tab.num_handle);
    	    if (param->add_attr_tab.status != ESP_GATT_OK){
    	        ESP_LOGE(BLE_TAG, "Create attribute table failed, error code=0x%x", param->add_attr_tab.status);
    	    }
     	    else {
    	        memcpy(lox_handle_table, param->add_attr_tab.handles, sizeof(lox_handle_table));
    	        esp_ble_gatts_start_service(lox_handle_table[IDX_SVC]);
    	    }
    	    break;

    	default:
    	    break;
        
    }
}

static uint8_t find_char_and_desr_index(uint16_t handle)
{
    uint8_t error = 0xff;

    for(int i = 0; i < LOX_IDX_NB ; i++){
        if(handle == lox_handle_table[i]){
            return i;
        }
    }

    return error;
}

/// @brief handle write event from mobile app
/// @param prepare_write_env 
/// @param param 
void exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gattc_cb_param_t *param)
{
    ESP_LOGE(BLE_TAG,"Error: exec_write_event_env involked!");
}

void scale_read_handler(void)
{
    ESP_LOGI(BLE_TAG, " scale_read_handler ");
    bleSendMessage(MSG_ADDR_SCTL, MSG_DATA_0, EVT_BLE_SCALE_RD, NULL, 0, MSG_DATA_0_LEN);
}

void read_index_handler(uint16_t readHandle,uint16_t connId, uint32_t transId, esp_gatt_if_t gattsIf)
{
    esp_gatt_rsp_t rsp;    
    
    ESP_LOGI(BLE_TAG, "read_index_handler");

    memset(bleTmpCharStr,0,sizeof(bleTmpCharStr));
    sprintf(bleTmpCharStr, "1");
    ESP_LOGI(BLE_TAG, "handle: %d, conn_id %d, trans_id %" PRIu32 ", gatts_if %d", readHandle, connId, transId, gattsIf);

    memset(ble_charc_data.index, 0x00, sizeof(ble_charc_data.index));
    (void)memcpy(ble_charc_data.index, bleTmpCharStr, strlen(bleTmpCharStr));    

    memset(&rsp, 0, sizeof(esp_gatt_rsp_t));  
    rsp.attr_value.handle = readHandle; //param->read.handle;  
    rsp.attr_value.len = strlen(bleTmpCharStr);  
    memcpy(rsp.attr_value.value,ble_charc_data.index, strlen(bleTmpCharStr));

    ESP_LOGI(BLE_TAG,"Index:%s", ble_charc_data.index);


    esp_ble_gatts_send_response(gattsIf,//gatts_if,  
                                connId,//param->read.conn_id,  
                                transId,//param->read.trans_id,  
                                ESP_GATT_OK, &rsp);

}



void ssid_read_handler(void)
{
    ESP_LOGI(BLE_TAG, "ssid_read_handler");
    bleSendMessage(MSG_ADDR_SCTL, MSG_DATA_0, EVT_APP_BLE_SSID_RD, NULL, 0, MSG_DATA_0_LEN); 
}

void pwd_read_handler(void)
{
    ESP_LOGI(BLE_TAG, "pwd_read_handler");
    bleSendMessage(MSG_ADDR_SCTL, MSG_DATA_0, EVT_APP_BLE_PWD_RD, NULL, 0, MSG_DATA_0_LEN);
}

void base_url_charc_handler(void)
{
    ESP_LOGI(BLE_TAG, " base_url_charc_handler ");
}

void ssid_write_handler(uint8_t *data , uint16_t len, esp_gatt_if_t gattsIf,uint16_t handle, uint16_t conn_id, uint32_t trans_id )
{
    unsigned char encryptedSsid[256];
    uint8_t tmpLen;
    esp_gatt_rsp_t rsp;

    memset(encryptedSsid, 0x00, sizeof(encryptedSsid));
    (void)memcpy(encryptedSsid, data, len);
    ESP_LOGI(BLE_TAG, "encryptedSsid len received:%d", len);

    printf("Encrypted SSID:");
    for(int i = 0; i < len; i++)
    {
        printf("%02X,",(uint8_t)encryptedSsid[i]); 
    }
    printf("\n\n");

    memset(ble_charc_data_update.ssid, 0x00, sizeof(ble_charc_data_update.ssid));
    decryptDataString(encryptedSsid, encryptionKey, len, ble_charc_data_update.ssid);

    ESP_LOGI(BLE_TAG, " ssid_write_handler  data received  %s ", (char *)ble_charc_data_update.ssid);
    sprintf(bleTmpStr, "%s", (char *)ble_charc_data_update.ssid);

    bleSendMessage(MSG_ADDR_STRG, MSG_DATA_STR, STRG_CMD_WR_SSID, (uint8_t *)bleTmpStr, 0, strlen(bleTmpStr)+1);

    memset(&rsp, 0, sizeof(esp_gatt_rsp_t));

    memset(bleTmpCharStr, 0x00, sizeof(bleTmpCharStr));
    sprintf(bleTmpCharStr, "**********");
    tmpLen = strlen(bleTmpCharStr);
    ESP_LOGI(BLE_TAG, "tmpLen:%d", tmpLen);

    memcpy(data,bleTmpCharStr, strlen(bleTmpCharStr));

    memset(rsp.attr_value.value,0,sizeof(rsp.attr_value.value));
    memcpy(rsp.attr_value.value,bleTmpCharStr, strlen(bleTmpCharStr));

    rsp.attr_value.handle = handle;  
    rsp.attr_value.len = strlen(bleTmpCharStr);  
    esp_ble_gatts_send_response(gattsIf,  
                                conn_id,  
                                trans_id,  
                                ESP_GATT_OK, &rsp);

}

void pswd_write_handler(uint8_t *data , uint16_t len, esp_gatt_if_t gattsIf,uint16_t handle, uint16_t conn_id, uint32_t trans_id)
{
    unsigned char encryptedPwd[256];
    uint8_t tmpLen;
    esp_gatt_rsp_t rsp;

    memset(encryptedPwd, 0x00, sizeof(encryptedPwd));
    (void)memcpy(encryptedPwd, data, len);
    ESP_LOGI(BLE_TAG, "encryptedPwd len received:%d", len);

    printf("Encrypted PWD:");
    for(int i = 0; i < len; i++)
    {
        printf("%02X,",(uint8_t)encryptedPwd[i]); 
    }
    printf("\n\n");

    memset(ble_charc_data_update.password, 0x00, sizeof(ble_charc_data_update.password));
    decryptDataString(encryptedPwd, encryptionKey, len, ble_charc_data_update.password);

    ESP_LOGI(BLE_TAG, " pswd_write_handler  data received  %s ", (char *)ble_charc_data_update.password);
    sprintf(bleTmpStr, "%s", (char *)ble_charc_data_update.password);

    bleSendMessage(MSG_ADDR_STRG, MSG_DATA_STR, STRG_CMD_WR_PWD, (uint8_t *)bleTmpStr, 0, strlen(bleTmpStr)+1);

    memset(&rsp, 0, sizeof(esp_gatt_rsp_t));  

    memset(bleTmpCharStr, 0x00, sizeof(bleTmpCharStr));
    sprintf(bleTmpCharStr, "**********");
    tmpLen = strlen(bleTmpCharStr);
    ESP_LOGI(BLE_TAG, "tmpLen:%d", tmpLen);

    memcpy(data,bleTmpCharStr, strlen(bleTmpCharStr));

    memset(rsp.attr_value.value,0,sizeof(rsp.attr_value.value));
    memcpy(rsp.attr_value.value,bleTmpCharStr, strlen(bleTmpCharStr));

    rsp.attr_value.handle = handle;  
    rsp.attr_value.len = len;  
    esp_ble_gatts_send_response(gattsIf,  
                                conn_id,  
                                trans_id,  
                                ESP_GATT_OK, &rsp);
}

void console_write_handler(uint8_t *data , uint16_t len)
{
    char* rest = (char *)data;
    char* token;
    uint8_t paramCnt = 0;
    char tmpStr[BLE_MAX_CON_JSON_STRING] = {0};
    char tmpStrConcat[BLE_MAX_CON_JSON_STRING] = {0};
    uint8_t offset = 0;

    memset(ble_charc_data_update.console_value, 0x00, sizeof(ble_charc_data_update.console_value));
    (void)memcpy(ble_charc_data_update.console_value, data, len-2); //To strip off CRLF
    ESP_LOGI(BLE_TAG, "console_write_handler data received:  %s ", (char *)ble_charc_data_update.console_value);

    while ((token = strtok_r(rest, " ", &rest))){

        if (paramCnt == 0){
            sprintf(tmpStrConcat, "{\"conc\":\"%s\"", token);
            ESP_LOGI(BLE_TAG, "Token:%s", token); 
            paramCnt++;
        }
        else if (paramCnt < BLE_MAX_CON_PARMS){
            memset(tmpStr,0x00,sizeof(tmpStr));
            sprintf( tmpStr, ",\"conp%d\":\"%s\"", paramCnt, token);
            ESP_LOGI(BLE_TAG, "Token:%s", token); 
            offset = strlen(tmpStrConcat);
            memcpy(tmpStrConcat + offset,tmpStr, strlen(tmpStr));
            paramCnt++;
        }
        ESP_LOGI(BLE_TAG, "paramCnt:%d, tmpStr:%s, tmpStrConcat:%s", paramCnt, tmpStr, tmpStrConcat);
    }
    memset(tmpStr,0x00,sizeof(tmpStr));
    memcpy(tmpStr, tmpStrConcat, strlen(tmpStrConcat)-3); //Strip off CR

    offset = strlen(tmpStr);    //Place }  at end of string
    sprintf(tmpStrConcat, "\"}");
    memcpy(tmpStr + offset,tmpStrConcat, strlen(tmpStrConcat));
    ESP_LOGI(BLE_TAG, "console_write_handler data sent:%s, strlen: %d", tmpStr, strlen(tmpStr)+1);
    bleSendMessage(MSG_ADDR_SER, MSG_DATA_STR, SER_CMD_CON_CMD, (uint8_t *)tmpStr, 0, strlen(tmpStr)+1);
}

void bleLogIF(char * strPtr)
{
    ESP_LOGI(BLE_TAG, "%s", strPtr);
    bleStoreLog(strPtr, true);
}

void bleLogEF(char * strPtr)
{
    ESP_LOGE(BLE_TAG, "%s", strPtr);
    bleStoreLog(strPtr, true);
}

void bleLogI(char * strPtr)
{
    ESP_LOGI(BLE_TAG, "%s", strPtr);
    bleStoreLog(strPtr, false);
}

void bleLogE(char * strPtr)
{
    ESP_LOGE(BLE_TAG, "%s", strPtr);
    bleStoreLog(strPtr, false);
}


void bleStoreLog(char * strPtr, bool forced)
{
    char logMsg[STRING_MAX_LOG_CHARACTERS] = {0};

    if ((bleStatus.is_connected && bleStatus.is_log_notify_enabled) || forced)
    {
        sprintf(logMsg, "%s\r\n" , strPtr);
        if (xQueueSend(logQueueHandle,logMsg,0) != pdPASS){
            ESP_LOGE(BLE_TAG, "Ble Log Queue is full! Message has been lost!");
        }
    }
    else{
        ESP_LOGE(BLE_TAG, "Ble Log notify is disabled and/or ble not conneted. Message has been lost!");
    }
}


void bleSendLog(void)
{
    char logMsg[STRING_MAX_LOG_CHARACTERS] = {0};

    if (bleStatus.is_connected && bleStatus.is_log_notify_enabled)
    {
        if (xQueueReceive(logQueueHandle,logMsg,10))
        {
            RF_SendDataToMobileLog(logMsg, LOG_HANDLE);
        }
    }
}

/// @brief Send message from ble task
/// @param dstAddr 
/// @param msgType 
/// @param msgCmd 
/// @param msgDataPtr 
/// @param msgData 
/// @param msgDataLen 
void bleSendMessage(uint8_t dstAddr, uint8_t msgType, uint16_t msgCmd, uint8_t * msgDataPtr, uint32_t msgData, uint32_t msgDataLen)
{  
    static uint8_t msgIndex = 0;
    RTOS_message_t sendRtosMsg;
    static char sendMsgArray[BLE_MESSAGE_MAX_ARRAY_ELEMENTS][BLE_STRING_MAX_ARRAY_CHARACTERS];

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
    sendRtosMsg.srcAddr = MSG_ADDR_BLE;
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
    if (msgIndex >= BLE_MESSAGE_MAX_ARRAY_ELEMENTS)
    {
        msgIndex = 0;
    }
}

//encryption
void encryptDataString(const char *toEncrypt, const char *key, uint16_t *strLen, unsigned char *result) {
    size_t toEncryptLen = strlen(toEncrypt);
    size_t keyLen = strlen(key);
    unsigned char iv = 0;
    uint16_t i;

    if (toEncryptLen > keyLen) {
        for (i = 0; i < toEncryptLen; i++) {
            result[i] = (unsigned char)(key[i % keyLen] ^ toEncrypt[i]);
            result[i] ^= iv;
        }
    } else {
       for (i = 0; i < keyLen; i++) {
            if (i < toEncryptLen) {
                result[i] = (unsigned char)(key[i] ^ toEncrypt[i]);
            } else {
                result[i] = (unsigned char)(key[i] ^ 0);
            }
            result[i] ^= iv;
        }
    }
    *strLen = i;
}

void decryptDataString(const unsigned char *toDecrypt, const char *key, const uint16_t strLen, unsigned char *result) {
    size_t keyLen = strlen(key);
    unsigned char iv = 0;

    for (size_t i = 0; i < keyLen; i++) {
        if (i < strLen) {
            result[i] = (char)(key[i] ^ toDecrypt[i]);
        } else {
            result[i] = (char)(key[i] ^ 0);
        }
        result[i] ^= iv;
    }
}

/* [] END OF FILE */

