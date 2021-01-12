#ifndef DXL_H
#define DXL_H

#include "stdint.h"
#include "luos.h"
#include "main.h"
#include "reachy.h"

#define NB_DXL 8
extern uint8_t DXL_IDS[NB_DXL];

void Dxl_Init(void);
void Dxl_Loop(void);
void Dxl_MsgHandler(container_t *src, msg_t *msg);

uint8_t is_alive();

uint8_t get_dxl_id(uint8_t id);
uint8_t dxl_id_exists(uint8_t id);

void status_led(uint8_t state);

#endif /* DXL_H */
