#ifndef GATE_H
#define GATE_H

#include "stdint.h"
#include "luos.h"
#include "main.h"

void Gate_Init(void);
void Gate_Loop(void);

void Dxl_MsgHandler(container_t *src, msg_t *msg);
void handle_inbound_msg(uint8_t msg[]);

uint8_t is_alive();

void status_led(uint8_t state);
void _assert(uint8_t condition, char *file, uint32_t line);

#endif /* GATE_H */
