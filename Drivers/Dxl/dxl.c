#include "main.h"
#include "dxl.h"
#include "Dynamixel_Servo.h"
#include "math.h"
#include "string.h"

module_t* my_module[DXL_NUMBER];
uint16_t dxl_table[DXL_NUMBER];
dxl_models_t dxl_model[DXL_NUMBER];
uint16_t position[DXL_NUMBER] = {0};

uint8_t motor_index_from_module(module_t *module) 
{
    for (uint8_t i = 0; i<=DXL_NUMBER; i++) 
    {
        if ((int)module == (int)my_module[i])
        {
            return i;
        }
    }
    while (1) {}
}

void rx_dxl_cb(module_t* module, msg_t *msg) {
    uint8_t index = motor_index_from_module(module);

    // if (msg->header.cmd == ANGULAR_POSITION) {
    //     angular_position_t ang_pos;
    //     angular_position_from_msg(&ang_pos, msg);

    //     if (dxl_model[index] == AX12 || dxl_model[index] == AX18 || dxl_model[index] == XL320) 
    //     {
    //         int pos = (int)((1024 - 1) * ((300 / 2 + ang_pos) / 300));
    //         servo_set_raw_word(dxl_table[index], SERVO_REGISTER_GOAL_ANGLE, pos, DXL_TIMEOUT);
    //     } 
    //     else 
    //     {
    //         int pos = (int)((4096 - 1) * ((360 / 2 + ang_pos) / 360));
    //         servo_set_raw_word(dxl_table[index], SERVO_REGISTER_GOAL_ANGLE, pos, DXL_TIMEOUT);
    //     }
    //     return;
    // }
    // if (msg->header.cmd == COMPLIANT) 
    // {
    //     servo_set_raw_byte(dxl_table[index], SERVO_REGISTER_TORQUE_ENABLE, (int)msg->data[0], DXL_TIMEOUT);
    //     return;
    // }
    if (msg->header.cmd == ASK_PUB_CMD) 
    {
        if (index == 0)
        {
            status_led(1);
        }
        msg_t pub_msg;
        pub_msg.header.target_mode = IDACK;
        pub_msg.header.target = msg->header.source;

        angular_position_t value;
        if (dxl_model[index] == AX12 || dxl_model[index] == AX18 || dxl_model[index] == XL320) 
        {
            value = ((300.0 * (float)position[index]) / (1024.0 - 1.0)) - (300.0 / 2);
        }
        else
        {
            value = ((360.0 * (float)position[index]) / (4096.0 - 1.0)) - (360.0 / 2);
        }

        angular_position_to_msg(&value, &pub_msg);
        luos_send(module, &pub_msg);
        return;
    }
}

void discover_dxl(void) {
    int y = 0;
    char my_string[15];
    // Clear module table
    luos_modules_clear();
    // Clear local tables
    memset(my_module, 0, sizeof(module_t*) * DXL_NUMBER);
    memset(dxl_table, 0, sizeof(uint16_t) * DXL_NUMBER);
    memset(dxl_model, 0, sizeof(dxl_models_t) * DXL_NUMBER);

    for (int i = 0; i<MAX_ID; i++) {
        if(!(servo_ping(i, DXL_TIMEOUT) & SERVO_ERROR_TIMEOUT)) {
            // no timeout occured, there is a servo here
            sprintf(my_string, "dxl_%d", i);
            my_module[y] = luos_module_create(rx_dxl_cb, DYNAMIXEL_MOD, my_string);
            dxl_table[y] = i;

            servo_get_raw_word(i, SERVO_REGISTER_MODEL_NUMBER, (uint16_t*)&dxl_model[y], DXL_TIMEOUT);
            // put a delay on motor response
            servo_set_raw_byte(i, SERVO_REGISTER_RETURN_DELAY_TIME, 10, DXL_TIMEOUT);
            // set limit temperature to 55°C
            servo_set_raw_byte(i, SERVO_REGISTER_MAX_TEMPERATURE, 55, DXL_TIMEOUT);
            y++;
        }
    }
    if (y == 0) {
        // there is no motor detected, create a Void module to only manage l0 things
        my_module[y] = luos_module_create(rx_dxl_cb, VOID_MOD, "void_dxl");
    }
}

void dxl_init(void) {
    servo_init(1000000);
    HAL_Delay(500);
    discover_dxl();
}


void get_motor_info(void) {
    static unsigned char id = 0;

    // Time management vars
    static uint32_t last_pos_systick = 0;
    uint32_t timestamp = HAL_GetTick();
    uint32_t deltatime = timestamp - last_pos_systick;

    if (deltatime >= GET_POS_PERIOD)
    {
        uint16_t tmp_val = 0;
        servo_error_t errors = servo_get_raw_word(dxl_table[id], SERVO_REGISTER_PRESENT_ANGLE, &tmp_val, DXL_TIMEOUT);
        if ((errors != SERVO_ERROR_TIMEOUT) & (errors != SERVO_ERROR_INVALID_RESPONSE)){
            position[id] = tmp_val;
        }

        // if (dxl_model[id] == AX12 || dxl_model[id] == AX18 || dxl_model[id] == XL320) 
        // {
        //     position[id] = 511;
        // }
        // else 
        // {
        //     position[id] = 2047;
        // }

        id++;
        if (id == DXL_NUMBER) {
            id = 0;
        }

        last_pos_systick = timestamp;
    }
}

void dxl_loop(void) 
{
    get_motor_info();
}