//*********************************************************************************************************************** */
/*
 * Copyright (C) 2017 BlueKitchen GmbH
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of1
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 * 4. Any redistribution, use, or modification is done solely for
 *    personal benefit and not for any commercial purpose or for
 *    monetary gain.
 *
 * THIS SOFTWARE IS PROVIDED BY BLUEKITCHEN GMBH AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL BLUEKITCHEN
 * GMBH OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * Please inquire about commercial licensing options at
 * contact@bluekitchen-gmbh.com
 *
 */

#define BTSTACK_FILE__ "hog_mouse_demo.c"

// *****************************************************************************
/* EXAMPLE_START(hog_mouse_demo): HID Mouse LE
 */
// *****************************************************************************

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include "btstack.h"

#include "ble/gatt-service/battery_service_server.h"
#include "ble/gatt-service/device_information_service_server.h"
#include "ble/gatt-service/hids_device.h"
#define HEARTBEAT_PERIOD_MS 1000
static btstack_timer_source_t heartbeat;

const uint8_t profile_data[] =
{
    // ATT DB Version
    1,
    // dgy add 私有服务
    0x0a, 0x00, 0x02, 0x00, 0x39, 0x00, 0x00, 0x28, 0x01, 0xff,
    // 0x0005 CHARACTERISTIC-ORG_BLUETOOTH_CHARACTERISTIC_BATTERY_LEVEL - DYNAMIC | READ | NOTIFY
    0x0d, 0x00, 0x02, 0x00, 0x41, 0x00, 0x03, 0x28, 0x10, 0x42, 0x00, 0xf2, 0xff,
    // 0x0006 VALUE CHARACTERISTIC-ORG_BLUETOOTH_CHARACTERISTIC_BATTERY_LEVEL - DYNAMIC | READ | NOTIFY
    // READ_ANYBODY
    0x08, 0x00, 0x02, 0x01, 0x42, 0x00, 0xf2, 0xff,
    // 0x0007 CLIENT_CHARACTERISTIC_CONFIGURATION
    // READ_ANYBODY, WRITE_ANYBODY
    0x0a, 0x00, 0x0e, 0x01, 0x43, 0x00, 0x02, 0x29, 0x00, 0x00,

    0x0a, 0x00, 0x02, 0x00, 0x44, 0x00, 0x00, 0x28, 0x02, 0xff,
    // 0x0005 CHARACTERISTIC-ORG_BLUETOOTH_CHARACTERISTIC_BATTERY_LEVEL - DYNAMIC | READ | NOTIFY
    0x0d, 0x00, 0x02, 0x00, 0x45, 0x00, 0x03, 0x28, 0x04, 0x46, 0x00, 0xf3, 0xff,
    // 0x0006 VALUE CHARACTERISTIC-ORG_BLUETOOTH_CHARACTERISTIC_BATTERY_LEVEL - DYNAMIC | READ | NOTIFY
    // READ_ANYBODY
    0x08, 0x00, 0x02, 0x01, 0x46, 0x00, 0xf3, 0xff,

    0x00, 0x00,
};


static btstack_packet_callback_registration_t hci_event_callback_registration;
static btstack_packet_callback_registration_t l2cap_event_callback_registration;
static btstack_packet_callback_registration_t sm_event_callback_registration;
static att_service_handler_t       owned;
static hci_con_handle_t con_handle = HCI_CON_HANDLE_INVALID;
static uint8_t  le_notification_enabled;
static char tx_data[256];
static char tx_len;

static void packet_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static void  heartbeat_handler(struct btstack_timer_source *ts);
static int att_write_callback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t transaction_mode, uint16_t offset, uint8_t *buffer, uint16_t buffer_size);

/* LISTING_START(attWrite): ATT Write */
static int att_write_callback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t transaction_mode, uint16_t offset, uint8_t *buffer, uint16_t buffer_size){
    switch (att_handle){
        case 0x0043:    //notiy handle
            le_notification_enabled = little_endian_read_16(buffer, 0) == GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION;
            con_handle = connection_handle;
            break;
        case 0x0046:
            printf("Write: transaction mode %u, offset %u, data (%u bytes): ", transaction_mode, offset, buffer_size);
            printf_hexdump(buffer, buffer_size);
            memcpy(tx_data,buffer,buffer_size);
            tx_len = buffer_size;
            att_server_request_can_send_now_event(con_handle);
            break;
        default:
            break;
    }
    return 0;
}

const uint8_t adv_data[] = {
    // Flags general discoverable, BR/EDR not supported
    0x02, BLUETOOTH_DATA_TYPE_FLAGS, 0x06,
    // Name
    0x0a, BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME, 'H', 'I', 'D', ' ', 'M', 'o', 'u', 's', 'e',
    // 16-bit Service UUIDs
    0x03, BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_16_BIT_SERVICE_CLASS_UUIDS, ORG_BLUETOOTH_SERVICE_HUMAN_INTERFACE_DEVICE & 0xff, ORG_BLUETOOTH_SERVICE_HUMAN_INTERFACE_DEVICE >> 8,
    // Appearance HID - Mouse (Category 15, Sub-Category 2)
    0x03, BLUETOOTH_DATA_TYPE_APPEARANCE, 0xC2, 0x03,
};
const uint8_t adv_data_len = sizeof(adv_data);

void owned_service_server_init(){

	// get service handle range
	uint16_t start_handle = 0;
	uint16_t end_handle   = 0xffff;
	int service_found = gatt_server_get_handle_range_for_service_with_uuid16(0xFF01, &start_handle, &end_handle);
	btstack_assert(service_found != 0);
	UNUSED(service_found);

	// register service with ATT Server
	owned.start_handle   = start_handle;
	owned.end_handle     = end_handle;
	owned.read_callback  = NULL;
	owned.write_callback = &att_write_callback;
	att_server_register_service_handler(&owned);
}

static void hog_mouse_setup(void){

    // setup l2cap and
    l2cap_init();

    // setup SM: Display only
    sm_init();
    sm_set_io_capabilities(IO_CAPABILITY_NO_INPUT_NO_OUTPUT);
    sm_set_authentication_requirements(SM_AUTHREQ_SECURE_CONNECTION | SM_AUTHREQ_BONDING);

    // setup ATT server
    att_server_init(profile_data, NULL, att_write_callback);

    // setup battery service
    owned_service_server_init();

    // setup advertisements
    uint16_t adv_int_min = 0x0030;
    uint16_t adv_int_max = 0x0030;
    uint8_t adv_type = 0;
    bd_addr_t null_addr;
    memset(null_addr, 0, 6);
    gap_advertisements_set_params(adv_int_min, adv_int_max, adv_type, 0, null_addr, 0x07, 0x00);
    gap_advertisements_set_data(adv_data_len, (uint8_t*) adv_data);
    gap_advertisements_enable(1);

    // register for events
    hci_event_callback_registration.callback = &packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    // register for connection parameter updates
    // l2cap_event_callback_registration.callback = &packet_handler;
    // l2cap_add_event_handler(&l2cap_event_callback_registration);

    // sm_event_callback_registration.callback = &packet_handler;
    // sm_add_event_handler(&sm_event_callback_registration);

    att_server_register_packet_handler(packet_handler);

    // heartbeat.process = &heartbeat_handler;
    // btstack_run_loop_set_timer(&heartbeat, HEARTBEAT_PERIOD_MS);
    // btstack_run_loop_add_timer(&heartbeat);

}

// static void heartbeat_handler(struct btstack_timer_source *ts){
//     if (le_notification_enabled) {
//         att_server_request_can_send_now_event(con_handle);
//     }
//     // att_server_request_can_send_now_event(con_handle);

//     btstack_run_loop_set_timer(ts, HEARTBEAT_PERIOD_MS);
//     btstack_run_loop_add_timer(ts);
// } 

static void packet_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    UNUSED(channel);
    UNUSED(size);
    uint16_t conn_interval;

    if (packet_type != HCI_EVENT_PACKET) return;

    switch (hci_event_packet_get_type(packet)) {
        case HCI_EVENT_DISCONNECTION_COMPLETE:
            con_handle = HCI_CON_HANDLE_INVALID;
            printf("Disconnected\n");
            break;
        case SM_EVENT_JUST_WORKS_REQUEST:
            printf("Just Works requested\n");
            sm_just_works_confirm(sm_event_just_works_request_get_handle(packet));
            break;
        case SM_EVENT_NUMERIC_COMPARISON_REQUEST:
            printf("Confirming numeric comparison: %"PRIu32"\n", sm_event_numeric_comparison_request_get_passkey(packet));
            sm_numeric_comparison_confirm(sm_event_passkey_display_number_get_handle(packet));
            break;
        case SM_EVENT_PASSKEY_DISPLAY_NUMBER:
            printf("Display Passkey: %"PRIu32"\n", sm_event_passkey_display_number_get_passkey(packet));
            break;
        case L2CAP_EVENT_CONNECTION_PARAMETER_UPDATE_RESPONSE:
            printf("L2CAP Connection Parameter Update Complete, response: %x\n", l2cap_event_connection_parameter_update_response_get_result(packet));
            break;
        case HCI_EVENT_META_GAP:
            switch (hci_event_gap_meta_get_subevent_code(packet)) {
                case GAP_SUBEVENT_LE_CONNECTION_COMPLETE:
                    // print connection parameters (without using float operations)
                    conn_interval = gap_subevent_le_connection_complete_get_conn_interval(packet);
                    printf("LE Connection Complete:\n");
                    printf("- Connection Interval: %u.%02u ms\n", conn_interval * 125 / 100, 25 * (conn_interval & 3));
                    printf("- Connection Latency: %u\n", gap_subevent_le_connection_complete_get_conn_latency(packet));
                    break;
                default:
                    break;
            }
            break;
        case HCI_EVENT_LE_META:
            switch (hci_event_le_meta_get_subevent_code(packet)) {
                case HCI_SUBEVENT_LE_CONNECTION_UPDATE_COMPLETE:
                    // print connection parameters (without using float operations)
                    conn_interval = hci_subevent_le_connection_update_complete_get_conn_interval(packet);
                    printf("LE Connection Update:\n");
                    printf("- Connection Interval: %u.%02u ms\n", conn_interval * 125 / 100, 25 * (conn_interval & 3));
                    printf("- Connection Latency: %u\n", hci_subevent_le_connection_update_complete_get_conn_latency(packet));
                    break;
                default:
                    break;
            }
            break;

        case ATT_EVENT_CAN_SEND_NOW:
            att_server_notify(con_handle, 0x0042, (uint8_t*) tx_data, tx_len);
            break;

        default:
            break;
    }
}

int btstack_main(void);
int btstack_main(void)
{
    hog_mouse_setup();

#ifdef HAVE_BTSTACK_STDIN
    btstack_stdin_setup(stdin_process);
#endif

    // turn on!
    hci_power_control(HCI_POWER_ON);

    return 0;
}
