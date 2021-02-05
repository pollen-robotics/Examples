#ifndef GATE_H
#define GATE_H

#include "stdint.h"
#include "luos.h"
#include "main.h"

void Gate_Init(void);
void Gate_Loop(void);

void handle_inbound_msg(uint8_t msg[]);
void Gate_MsgHandler(container_t *src, msg_t *msg);

uint8_t is_alive();

void send_on_serial(uint8_t payload[], uint8_t payload_size);

void status_led(uint8_t state);
void _assert(uint8_t condition, char *file, uint32_t line);

uint16_t RoutingTB_Get_ContainerNB(uint16_t node_id);

#endif /* GATE_H */
