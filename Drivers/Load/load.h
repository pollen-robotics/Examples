#ifndef _LOAD_H
#define _LOAD_H

#include "luos.h"

#define RIGHT_FORCE_GRIPPER 10
#define LEFT_FORCE_GRIPPER 20

#define LOAD_SENSOR_ID RIGHT_FORCE_GRIPPER

#define LOAD_PUB_PERIOD 90 // in ms
#define KEEP_ALIVE_PERIOD 1100  // in ms

void Load_Init(void);
void Load_Loop(void);
void Load_MsgHandler(container_t *dst, msg_t *msg);

void send_load_to_gate(container_t *src, float load);

uint8_t is_alive();
void status_led(uint8_t state);

#endif