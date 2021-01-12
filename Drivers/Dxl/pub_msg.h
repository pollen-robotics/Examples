#ifndef PUB_MSG_H
#define PUB_MSG_H

#include "luos.h"

void send_dxl_byte_to_gate(container_t *src, uint8_t dxl_id, uint8_t reg, uint16_t error, uint8_t value);
void send_dxl_word_to_gate(container_t *src, uint8_t dxl_id, uint8_t reg, uint16_t error, uint16_t value);
void send_dxl_page_to_gate(container_t *src, uint8_t dxl_id, uint8_t reg, uint16_t error, uint8_t values[], uint8_t val_size);

#endif