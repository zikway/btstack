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

static void (*transport_packet_handler)(uint8_t packet_type, uint8_t *packet, uint16_t size);
static void transport_deliver_packets(void *context);

static btstack_packet_callback_registration_t hci_event_callback_registration;
static int hci_acl_can_send_now;
static btstack_ring_buffer_t hci_ringbuffer;
static SemaphoreHandle_t ring_buffer_mutex; 
#define MAX_NR_HOST_EVENT_PACKETS 4
static uint8_t hci_ringbuffer_storage[HCI_HOST_ACL_PACKET_NUM   * (2 + 1 + HCI_ACL_HEADER_SIZE + HCI_HOST_ACL_PACKET_LEN) +
                                      HCI_HOST_SCO_PACKET_NUM   * (2 + 1 + HCI_SCO_HEADER_SIZE + HCI_HOST_SCO_PACKET_LEN) +
                                      MAX_NR_HOST_EVENT_PACKETS * (2 + 1 + HCI_EVENT_BUFFER_SIZE)];
// incoming packet buffer
static uint8_t hci_packet_with_pre_buffer[HCI_INCOMING_PRE_BUFFER_SIZE + HCI_INCOMING_PACKET_BUFFER_SIZE]; // packet type + max(acl header + acl payload, event header + event data)
static uint8_t * hci_receive_buffer = &hci_packet_with_pre_buffer[HCI_INCOMING_PRE_BUFFER_SIZE];

//蓝牙调度所需要的ms时间
uint32_t hal_time_ms(void)
{
	return sys_timer_get_ms();   //获取ms时间
}

static btstack_context_callback_registration_t packet_receive_callback_context = {
        .callback = transport_deliver_packets,
        .context = NULL,
};

//用于接收controler 的上行数据
int hci_packet_handler(u8 type, u8 *packet, u16 size)
{
    int err = 0;
    int msg[2];

    logd("HCI PH : 0x%x %x %x %x %x %x %x / %d", packet,packet[0],packet[1],packet[2],packet[3],packet[4],packet[5] ,size);
   /* log_info_hexdump(packet, size); */
    // if (transport_packet_handler != NULL) {
    //     transport_packet_handler(type, packet, size);
    // }

    xSemaphoreTake(ring_buffer_mutex, portMAX_DELAY);

    // check space
    uint16_t space = btstack_ring_buffer_bytes_free(&hci_ringbuffer);
    if (space < (size + 1)){
        xSemaphoreGive(ring_buffer_mutex);
        log_error("transport_recv_pkt_cb packet %u, space %u -> dropping packet", (size + 1), space);
        return 0;
    }
    // store type in ringbuffer
    btstack_ring_buffer_write(&hci_ringbuffer, &type, 1);
    // store size in ringbuffer
    uint8_t len_tag[2];
    little_endian_store_16(len_tag, 0, size);
    btstack_ring_buffer_write(&hci_ringbuffer, len_tag, sizeof(len_tag));

    // store in ringbuffer
    btstack_ring_buffer_write(&hci_ringbuffer, packet, size);

    xSemaphoreGive(ring_buffer_mutex);
    btstack_run_loop_execute_on_main_thread(&packet_receive_callback_context);
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

static void transport_deliver_packets(void *context){
    UNUSED(context);
    xSemaphoreTake(ring_buffer_mutex, portMAX_DELAY);
    while (btstack_ring_buffer_bytes_available(&hci_ringbuffer)){
        uint32_t number_read;
        uint8_t len_tag[2];
        uint8_t type;
        btstack_ring_buffer_read(&hci_ringbuffer, &type, 1, &number_read);
        btstack_ring_buffer_read(&hci_ringbuffer, len_tag, 2, &number_read);
        uint32_t len = little_endian_read_16(len_tag, 0);
        btstack_ring_buffer_read(&hci_ringbuffer, hci_receive_buffer, len, &number_read);
        xSemaphoreGive(ring_buffer_mutex);
        transport_packet_handler(type, hci_receive_buffer, len);
        xSemaphoreTake(ring_buffer_mutex, portMAX_DELAY);
    }
    xSemaphoreGive(ring_buffer_mutex);
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

/**
 * register packet handler for HCI packets: ACL and Events
 */
static void transport_register_packet_handler(void (*handler)(uint8_t packet_type, uint8_t *packet, uint16_t size)){
    logd("transport_register_packet_handler");
    transport_packet_handler = handler;
}

/**
 * init transport
 * @param transport_config
 */
static void transport_init(const void *transport_config){
    log_info("transport_init");
    ring_buffer_mutex = xSemaphoreCreateMutex();
    btstack_ring_buffer_init(&hci_ringbuffer, hci_ringbuffer_storage, sizeof(hci_ringbuffer_storage));
    // set up polling data_source
    // btstack_run_loop_set_data_source_handler(&transport_data_source, &transport_process);
    // btstack_run_loop_enable_data_source_callbacks(&transport_data_source, DATA_SOURCE_CALLBACK_POLL);
    // btstack_run_loop_add_data_source(&transport_data_source);

	// log_debug("shared SRAM2 buffers");
	// log_debug(" *BleCmdBuffer          : 0x%08X", (void *)&BleCmdBuffer);
	// log_debug(" *HciAclDataBuffer      : 0x%08X", (void *)&HciAclDataBuffer);
	// log_debug(" *SystemCmdBuffer       : 0x%08X", (void *)&SystemCmdBuffer);
	// log_debug(" *EvtPool               : 0x%08X", (void *)&EvtPool);
	// log_debug(" *SystemSpareEvtBuffer  : 0x%08X", (void *)&SystemSpareEvtBuffer);
	// log_debug(" *BleSpareEvtBuffer     : 0x%08X", (void *)&BleSpareEvtBuffer);

    /**< FreeRTOS implementation variables initialization */
    hci_acl_can_send_now = 1;

	/**< Reference table initialization */

	/**< System channel initialization */

	/**< Memory Manager channel initialization */

	/**< BLE channel initialization */
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

static void transport_notify_packet_send(void){
    // notify upper stack that it might be possible to send again
    uint8_t event[] = { HCI_EVENT_TRANSPORT_PACKET_SENT, 0};
    transport_packet_handler(HCI_EVENT_PACKET, &event[0], sizeof(event));
}

static void transport_notify_ready(void){
    // notify upper stack that it transport is ready
    uint8_t event[] = { HCI_EVENT_TRANSPORT_READY, 0};
    transport_packet_handler(HCI_EVENT_PACKET, &event[0], sizeof(event));
}

/**
 * support async transport layers, e.g. IRQ driven without buffers
 */
static int transport_can_send_packet_now(uint8_t packet_type) {
    //if (cpu2_state != CPU2_STATE_READY) return 0;
    switch (packet_type)
    {
        case HCI_COMMAND_DATA_PACKET:
            return 1;

        case HCI_ACL_DATA_PACKET:
            return hci_acl_can_send_now;
    }
    return 1;
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
            hci_vendor_send_cmd(packet, size, 0);
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
    &transport_init,
    &transport_open,
    &transport_close,
    &transport_register_packet_handler,
    &transport_can_send_packet_now,
    &transport_send_packet,
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
//    uint8_t test[3] = {0x03,0x0c,0x00};   //测试hci_vendor_send_cmd 接口
//    hci_vendor_send_cmd(test,3,0);
    /// GET STARTED with BTstack ///
    btstack_memory_init();
    btstack_run_loop_init(btstack_run_loop_freertos_get_instance());
    // // init HCI
    hci_init(transport_get_instance(), NULL);
    // inform about BTstack state
    hci_event_callback_registration.callback = &packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);
    btstack_main(0, NULL);   //用于跑demo
    log_info("btstack executing run loop...");
    logd("btstack %d",__LINE__);
    btstack_run_loop_execute();  //包含while（1）
//    while (1)
//    {
//        os_time_dly(500);
//        logd("btstack %d",__LINE__);
//    }
}
