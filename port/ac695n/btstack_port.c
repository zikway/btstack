#define BTSTACK_FILE__ "btstack_port.c"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>

#include "btstack_config.h"
#include "btstack.h"
#include "btstack_config.h"
#include "btstack_event.h"
#include "btstack_memory.h"
#include "btstack_run_loop.h"
//#include "btstack_run_loop_freertos.h"
#include "btstack_tlv_flash_bank.h"
#include "le_device_db_tlv.h"
#include "hci.h"
#include "hci_dump.h"
#include "btstack_debug.h"

//ac695n
#include "system/timer.h"
#include "syscfg_id.h"
extern int btstack_main(int argc, const char * argv[]);

//蓝牙调度所需要的ms时间
uint32_t hal_time_ms(void)
{
	return sys_timer_get_ms();   //获取ms时间
}

//用于接收controler 的上行数据
int hci_packet_handler(u8 type, u8 *packet, u16 size)
{
   int err = 0;
   int msg[2];

   logd("HCI PH : 0x%x / %d", packet, size);
   /* log_info_hexdump(packet, size); */

//   switch (type) {
//   case HCI_EVENT_PACKET: {
//       struct hci_event *p;
//       p = lbuf_alloc(__this->pEVT, sizeof(struct hci_event) + size);
//       p->length = size;
//       memcpy(p->payload, packet, size);
//       lbuf_push(p, 1);
//   }
//   err = os_taskq_post_type(THIS_TASK_NAME, BTSTACK_HCI_EVENT, 0, NULL);
//   break;
//   case HCI_ACL_DATA_PACKET:
//       msg[0] = (int)packet;
//       msg[1] = size;
//       err = os_taskq_post_type(THIS_TASK_NAME, BTSTACK_HCI_ACL, 2, msg);
//       break;
//   default:
//       return 0;
//   }

//   if (err != OS_NO_ERR) {
//       log_error("OS_ERR : 0x%x", err);
//   }

   return err;
}

static void packet_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    // if (packet_type != HCI_EVENT_PACKET) return;
    // switch(hci_event_packet_get_type(packet)){
    //     // wait with TLV init (which uses HW semaphores to coordinate with CPU2) until CPU2 was started
    //     case HCI_EVENT_TRANSPORT_READY:
    //         setup_dbs();
    //         break;
    //     default:
    //         break;
    // }
}
static btstack_packet_callback_registration_t hci_event_callback_registration;

static void (*transport_packet_handler)(uint8_t packet_type, uint8_t *packet, uint16_t size);

/**
 * register packet handler for HCI packets: ACL and Events
 */
static void transport_register_packet_handler(void (*handler)(uint8_t packet_type, uint8_t *packet, uint16_t size)){
    logd("transport_register_packet_handler");
    transport_packet_handler = handler;
}
/**
 * open transport connection
 */
static int transport_open(void){
    logd("transport_open");
    return 0;
}

/**
 * close transport connection
 */
static int transport_close(void){
    logd("transport_close");
    return 0;
}

static void transport_send_hardware_error(uint8_t error_code){
    uint8_t event[] = { HCI_EVENT_HARDWARE_ERROR, 1, error_code};
    transport_packet_handler(HCI_EVENT_PACKET, &event[0], sizeof(event));
}

/**
 * send packet
 */
static int transport_send_packet(uint8_t packet_type, uint8_t *packet, int size){
	// TL_CmdPacket_t *ble_cmd_buff = &BleCmdBuffer;

    switch (packet_type){
        case HCI_COMMAND_DATA_PACKET:
            // ble_cmd_buff->cmdserial.type = packet_type;
            // ble_cmd_buff->cmdserial.cmd.plen = size;
            // memcpy((void *)&ble_cmd_buff->cmdserial.cmd, packet, size);
            // TL_BLE_SendCmd(NULL, 0);
            transport_notify_packet_send();
            break;

        case HCI_ACL_DATA_PACKET:
            // hci_acl_can_send_now = 0;
            // ((TL_AclDataPacket_t *)HciAclDataBuffer)->AclDataSerial.type = packet_type;
            // memcpy((void *)&(((TL_AclDataPacket_t *)HciAclDataBuffer)->AclDataSerial.handle),packet, size);
            // TL_BLE_SendAclData(NULL, 0);
            hci_send_acl_packet(packet,size);
            transport_notify_packet_send();
            break;

        default:
            logd("error ");
            transport_send_hardware_error(0x01);  // invalid HCI packet
            break;
    }
    return 0;
}

static const hci_transport_t transport = {
    "ac695n",
    // &transport_init,
    // &transport_open,
    // &transport_close,
    // &transport_register_packet_handler,
    // &transport_can_send_packet_now,
    // &transport_send_packet,
    NULL,
    &transport_open,
    &transport_close,
    &transport_register_packet_handler,
    NULL,
    NULL,
    NULL, // set baud rate
    NULL, // reset link
    NULL, // set SCO config
};

static const hci_transport_t * transport_get_instance(void){
    return &transport;
}

int btstack_demo_init()
{
   logd("bt_get_mac_addr : ");
   log_info_hexdump((u8 *)bt_get_mac_addr(), 6);


   /* le_controller_set_mac((void *)&le_mac_addr); */
   le_controller_set_mac((void *)bt_get_mac_addr());
   lmp_hci_write_local_address((void *)bt_get_mac_addr());

   /* void wdt_close(void); */
   /* wdt_close(); */
   /* btctrler_task_init((void *)hci_transport_usb_instance(), NULL); */
   /* return 0; */

   btctrler_task_init((void *)hci_transport_h4_controller_instance(), NULL);
   while (btctrler_task_ready() == 0) {
       os_time_dly(2);
   }

   set_bt_ldotrim(0,CFG_BT_LDOTRIM_INFO);
//    task_create(btstack_task, NULL, THIS_TASK_NAME);

//    btstack_task_ready();

   return 0;
}

void bt_task_handle(void *arg)
{
    btstack_demo_init();
    //hci_send_cmd(&hci_reset);   //测试hci send cmd 接口
    /// GET STARTED with BTstack ///
    btstack_memory_init();
    logd("btstack %d",__LINE__);
    btstack_run_loop_init(btstack_run_loop_freertos_get_instance());
    logd("btstack %d",__LINE__);
    // // init HCI
    hci_init(transport_get_instance(), NULL);
    logd("btstack %d",__LINE__);
    // inform about BTstack state
    hci_event_callback_registration.callback = &packet_handler;
    logd("btstack %d",__LINE__);
    hci_add_event_handler(&hci_event_callback_registration);

    // btstack_main(0, NULL);

    log_info("btstack executing run loop...");
    logd("btstack %d",__LINE__);
    btstack_run_loop_execute();
    while (1)
    {
        os_time_dly(500);
        logd("btstack %d",__LINE__);
    }
}
