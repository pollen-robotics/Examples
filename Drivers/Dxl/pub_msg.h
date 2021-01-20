#ifndef PUB_MSG_H
#define PUB_MSG_H

#include "luos.h"
#include "Dynamixel_Servo.h"

void send_to_gate(container_t *src, uint8_t payload[], uint8_t payload_size);

void send_dxl_byte_to_gate(container_t *src, uint8_t dxl_id, uint8_t reg, uint16_t error, uint8_t value);
void send_dxl_word_to_gate(container_t *src, uint8_t dxl_id, uint8_t reg, uint16_t error, uint16_t value);
void send_dxl_page_to_gate(container_t *src, uint8_t dxl_id, uint8_t reg, uint16_t error, uint8_t values[], uint8_t val_size);

void send_positions_to_gate(container_t *src, uint8_t dxl_ids[], uint16_t positions[], servo_error_t errors[], uint8_t nb_ids);


#endif