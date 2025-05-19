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
#define HAVE_MALLOC

//#define BTSTACK_PRINTF
#define ENABLE_LOG_DEBUG
#define ENABLE_LOG_ERROR
#define ENABLE_LOG_INFO
#define ENABLE_PRINTF_HEXDUMP

// Port related features
#define HAVE_EMBEDDED_TIME_MS

// BTstack features that can be enabled
#define ENABLE_CLASSIC
#define NVM_NUM_LINK_KEYS 2
// #define ENABLE_BLE
// #define ENABLE_L2CAP_LE_CREDIT_BASED_FLOW_CONTROL_MODE
// #define ENABLE_LE_DATA_LENGTH_EXTENSION
// #define ENABLE_LE_PERIPHERAL



// BTstack configuration. buffers, sizes, ...
#define HAVE_HOST_CONTROLLER_API    //定义拥有hci 接口，不需要外挂controler
#define HCI_OUTGOING_PRE_BUFFER_SIZE  4  //定义HAVE_HOST_CONTROLLER_API 所需要的输出buffer数
#define HCI_ACL_PAYLOAD_SIZE 200
#define MAX_NR_GATT_CLIENTS 1
#define MAX_NR_HCI_CONNECTIONS 1
#define MAX_NR_HIDS_CLIENTS 1
#define MAX_NR_SM_LOOKUP_ENTRIES 3
#define MAX_NR_WHITELIST_ENTRIES 1
#define MAX_NR_LE_DEVICE_DB_ENTRIES 1


#ifdef ENABLE_CLASSIC

// ACL buffer large enough for Ethernet frame in BNEP/PAN
#define HCI_ACL_PAYLOAD_SIZE (1691 + 4)

#define HCI_HOST_ACL_PACKET_LEN 1024
#define HCI_HOST_ACL_PACKET_NUM 20
#define HCI_HOST_SCO_PACKET_LEN 60
#define HCI_HOST_SCO_PACKET_NUM 10

#else

// ACL buffer large enough to allow for 512 byte Characteristic
#define HCI_ACL_PAYLOAD_SIZE (512 + 4 + 3)

#define HCI_HOST_ACL_PACKET_LEN HCI_ACL_PAYLOAD_SIZE
#define HCI_HOST_ACL_PACKET_NUM 20
#define HCI_HOST_SCO_PACKET_LEN 0
#define HCI_HOST_SCO_PACKET_NUM 0

#endif //ENABLE_CLASSIC

#endif
