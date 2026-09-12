#ifndef LORA_H
#define LORA_H

#include <stdint.h>

int lora_init(void);
int lora_self_test(void);
int lora_send(const uint8_t *data, uint8_t len);
int lora_receive(uint8_t *data, uint8_t *len, uint8_t max_len, uint8_t timeout_ms);
int lora_last_rssi(void);
unsigned int lora_tx_ok(void);
unsigned int lora_tx_fail(void);

#endif
