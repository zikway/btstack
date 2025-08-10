//
// btstack_config.h for Apollo 2 + EM9304 port
//
// Documentation: https://bluekitchen-gmbh.com/btstack/#how_to/
//

#ifndef BTSTACK_CONFIG_H
#define BTSTACK_CONFIG_H

#include "api_log.h"


typedef enum {
	BT_EVT_UNKNOW=0,							//上电事件
    BT_EVT_INIT, 								//init (pa)
	BT_EVT_IDLE,								//idle (dev_id)
	
	BT_EVT_ADV,									//广播中(edr进入通用/限制可发现状态)
    BT_EVT_ADV_DIR,                     		//广播(edr 处于定向回连)
	BT_EVT_ADV_TIMOUT,							//广播超时
	BT_EVT_SCAN = BT_EVT_ADV,					//扫描	(pa)
	BT_EVT_SCAN_DIR = BT_EVT_ADV_DIR,			//定向扫描(dev_id)
	BT_EVT_SCAN_TIMEOUT = BT_EVT_ADV_TIMOUT,	//扫描超时(dev_id)

	BT_EVT_CONNECTED,							//连接成功	(pa)
	BT_EVT_DISCONNECTED,						//断开连接	(pa)
	BT_EVT_CONNECT_FAIL,						//(回连)连接失败	(pa)
	BT_EVT_READY,								//蓝牙notyif打开,可以接收数据(must pa!)

	BT_EVT_REMOTE_TYPE,							//蓝牙识别 remote type event
	BT_EVT_RX,									//rx消息 (pa)
	BT_EVT_TX,									//READ 事件 (dev_id)
} bt_evt_t;

#define	BT_BLE 0
#define	BT_EDR 1
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
#define ENABLE_BLE
#define ENABLE_L2CAP_LE_CREDIT_BASED_FLOW_CONTROL_MODE
#define ENABLE_LE_DATA_LENGTH_EXTENSION
#define ENABLE_LE_PERIPHERAL
#define ENABLE_LE_CENTRAL    //用于打开ble 主机模式
#define MAX_NR_LE_DEVICE_DB_ENTRIES 1



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
