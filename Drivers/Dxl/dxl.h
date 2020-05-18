#ifndef DXL_H
#define DXL_H

#include "luos.h"
#include "Dynamixel_Servo.h"

#define MAX_ID 30
#define DXL_TIMEOUT 1
#define TEMP_REFRESH_MS 1000
#define FACTORY_RESET_REG 0xFF

#define DXL_NUMBER 8
#define GET_POS_PERIOD 1 // for a single motor (thus the period for all motors will be * DXL_NUMBER)
#define MAX_CMD 30

void dxl_init(void);
void dxl_loop(void);

typedef enum {
    AX12  = (12 + (0<<8)),
    AX18  = (18 + (0<<8)),
    RX24  = (24 + (0<<8)),
    RX28  = (28 + (0<<8)),
    MX28  = (29 + (0<<8)),
    RX64  = (64 + (0<<8)),
    MX12  = (104 + (1<<8)),
    MX64  = (54 + (1<<8)),
    MX106 = (64 + (1<<8)),
    XL320 = (94 + (1<<8))
}dxl_models_t;

typedef enum {
    MODE_WHEEL,
    MODE_ANGLE,
    MODE_ANGLE_LIMIT,
    MODE_POWER_LIMIT,
    MODE_PID,
    MODE_SPEED,
    MODE_COMPLIANT,
    MODE_DETECT,
    MODE_TEMP,
    MODE_REG,
    MODE_ID
}dxl_mode_t;

typedef struct {
    uint16_t motor_id;
    uint16_t val;
    servo_register_t reg;
    uint8_t waiting;
}dxl_command_t;

#endif /* DXL_H */
