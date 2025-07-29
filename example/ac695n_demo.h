#ifndef __AC695N_DEMO_H__
#define __AC695N_DEMO_H__

#include <stdint.h>

void classic_send_repot(uint8_t *message, uint8_t message_len);
void ble_send_report(uint8_t *message, uint8_t message_len);
#endif