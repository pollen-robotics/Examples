#ifndef FAN_H
#define FAN_H

#include "luos.h"

#define SHOULDER_FAN_ID 10
#define ELBOW_FAN_ID 11
#define WRIST_FAN_ID 12

#define SHOULDER_FAN_Pin GPIO_PIN_5
#define SHOULDER_FAN_GPIO_Port GPIOB
#define ELBOW_FAN_Pin GPIO_PIN_3
#define ELBOW_FAN_GPIO_Port GPIOB
#define WRIST_FAN_Pin GPIO_PIN_0
#define WRIST_FAN_GPIO_Port GPIOB

void Fan_Init(void);
void Fan_Loop(void);
void Fan_MsgHandler(container_t *src, msg_t *msg);

uint8_t fan_id_from_dxl_id(uint8_t dxl_id);

uint8_t get_fan_state(uint8_t fan_id);
void set_fan_state(uint8_t fan_id, uint8_t state);

#endif /* FAN_H */
