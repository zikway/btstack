//
// btstack_config.h for Apollo 2 + EM9304 port
//
// Documentation: https://bluekitchen-gmbh.com/btstack/#how_to/
//

#ifndef BTSTACK_CONFIG_H
#define BTSTACK_CONFIG_H

#include "api_log.h"

//System properties
#define MAX_ATT_DB_SIZE 1
#define NVM_NUM_DEVICE_DB_ENTRIES 3
#define HAVE_FREERTOS_TASK_NOTIFICATIONS
#define ESP_PLATFORM  //���ڽ��portYIELD_FROM_ISR ����
//#define BTSTACK_PRINTF

// Port related features
#define HAVE_EMBEDDED_TIME_MS

// BTstack features that can be enabled
#define ENABLE_CLASSIC
#define NVM_NUM_LINK_KEYS 2
#define ENABLE_BLE
#define ENABLE_L2CAP_LE_CREDIT_BASED_FLOW_CONTROL_MODE
#define ENABLE_LE_DATA_LENGTH_EXTENSION
#define ENABLE_LE_PERIPHERAL
#define ENABLE_LOG_ERROR
#define ENABLE_PRINTF_HEXDUMP

// BTstack configuration. buffers, sizes, ...
#define HCI_ACL_PAYLOAD_SIZE 200
#define MAX_NR_GATT_CLIENTS 1
#define MAX_NR_HCI_CONNECTIONS 1
#define MAX_NR_HIDS_CLIENTS 1
#define MAX_NR_SM_LOOKUP_ENTRIES 3
#define MAX_NR_WHITELIST_ENTRIES 1

#define MAX_NR_LE_DEVICE_DB_ENTRIES 1

#endif
