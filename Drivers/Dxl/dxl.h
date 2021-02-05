#ifndef DXL_H
#define DXL_H

#include "stdint.h"
#include "luos.h"
#include "main.h"
#include "reachy.h"

#define NB_DXL_MAX 10
#define MAX_DXL_ID 100

#define DXL_TIMEOUT 2  // in ms
#define POS_PUBLISH_PERIOD 10 // in ms
#define TEMP_PUBLISH_PERIOD 5000  // in ms

#define KEEP_ALIVE_PERIOD 1100  // in ms

#define FAN_TRIGGER_TEMPERATURE 50  // in degree celsius
#define FAN_SHUTDOWN_TEMPERATURE 45  // in degree celsius

void Dxl_Init(void);
void Dxl_Loop(void);
void Dxl_MsgHandler(container_t *src, msg_t *msg);

uint8_t is_alive();

void dxl_detect();
uint8_t get_dxl_id(uint8_t id);
uint8_t dxl_id_exists(uint8_t id);
void check_temperature();

void status_led(uint8_t state);

#endif /* DXL_H */
