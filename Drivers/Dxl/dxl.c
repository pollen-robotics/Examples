#include "main.h"
#include "dxl.h"
#include "Dynamixel_Servo.h"
#include "math.h"
#include "string.h"

module_t* dxl_modules[DXL_NUMBER];
uint16_t dxl_table[DXL_NUMBER];
dxl_models_t dxl_model[DXL_NUMBER];
angular_position_t position[DXL_NUMBER];


volatile static uint8_t last_request = 0;
volatile uint8_t request_nb = 0;
volatile dxl_command_t cmd[MAX_CMD];

uint8_t motor_index_from_module(module_t *module) 
{
    for (uint8_t i = 0; i<=DXL_NUMBER; i++) 
    {
        if ((int)module == (int)dxl_modules[i])
        {
            return i;
        }
    }
    while (1) {}
}

void rx_dxl_pub_cb(module_t* module, msg_t *msg) {
    if (msg->header.cmd == ASK_PUB_CMD) 
    {
        msg_t pub_msg;
        pub_msg.header.target_mode = ID;
        pub_msg.header.target = msg->header.source;

        reachy_arm_pos_to_msg(position, DXL_NUMBER, &pub_msg);
        luos_send(module, &pub_msg);
        return;
    }
}

void rx_dxl_cmd_cb(module_t* module, msg_t *msg) {
    uint8_t index = motor_index_from_module(module);

    if (dxl_table[index] == 0)
    {
        return;
    }

    if (msg->header.cmd == COMPLIANT) 
    {
        cmd[last_request].reg = SERVO_REGISTER_TORQUE_ENABLE;
        cmd[last_request].val = (msg->data[0] != 1);
    }
    else if (msg->header.cmd == ANGULAR_POSITION) {
        angular_position_t ang_pos;
        angular_position_from_msg(&ang_pos, msg);

        uint16_t pos;
        if (dxl_model[index] == AX12 || dxl_model[index] == AX18 || dxl_model[index] == XL320) 
        {
            pos = (uint16_t)((1024 - 1) * ((300 / 2 + ang_pos) / 300));
        } 
        else 
        {
            pos = (uint16_t)((4096 - 1) * ((360 / 2 + ang_pos) / 360));
        }

        cmd[last_request].reg = SERVO_REGISTER_GOAL_ANGLE;
        cmd[last_request].val = pos;
    }
    else if (msg->header.cmd == SETID) {
        char id;
        memcpy(&id, msg->data, sizeof(char));

        cmd[last_request].reg = SERVO_REGISTER_ID;
        cmd[last_request].val = id;
    }
    else {
        return;
    }

    cmd[last_request].motor_id = dxl_table[index];
    cmd[last_request].waiting = 1;
    last_request++;
    request_nb++;

    if (last_request == MAX_CMD)
    {
        last_request = 0;
    }
}


void discover_dxl(void) {
    int y = 0;
    char my_string[15];
    // Clear module table
    luos_modules_clear();
    // Clear local tables
    memset(dxl_modules, 0, sizeof(module_t*) * DXL_NUMBER);
    memset(dxl_table, 0, sizeof(uint16_t) * DXL_NUMBER);
    memset(dxl_model, 0, sizeof(dxl_models_t) * DXL_NUMBER);

    for (int i = 0; i<MAX_ID; i++) {
        if(!(servo_ping(i, DXL_TIMEOUT) & SERVO_ERROR_TIMEOUT)) {
            // no timeout occured, there is a servo here
            sprintf(my_string, "dxl_%d", i);
            dxl_modules[y] = luos_module_create(rx_dxl_cmd_cb, DYNAMIXEL_MOD, my_string);
            luos_module_enable_rt(dxl_modules[y]);
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
        luos_module_create(rx_dxl_cmd_cb, VOID_MOD, "void_dxl");
    }
    else if (y == DXL_NUMBER) {
        char name[15];
        for (int i=0; i<DXL_NUMBER; i++) {
            if (dxl_table[i] == 10) {
                sprintf(name, "right_arm");
                break;
            }
            if (dxl_table[i] == 20) {
                sprintf(name, "left_arm");
                break;
            }
        }
        luos_module_create(rx_dxl_pub_cb, REACHY_ARM_MOD, name);
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
        if (dxl_table != 0) 
        {
            uint16_t tmp_val = 0;
            servo_error_t errors = servo_get_raw_word(dxl_table[id], SERVO_REGISTER_PRESENT_ANGLE, &tmp_val, DXL_TIMEOUT);
            if ((errors != SERVO_ERROR_TIMEOUT) & (errors != SERVO_ERROR_INVALID_RESPONSE)){
                if (dxl_model[id] == AX12 || dxl_model[id] == AX18 || dxl_model[id] == XL320) 
                {
                    position[id] = ((300.0 * (float)tmp_val) / (1024.0 - 1.0)) - (300.0 / 2);
                }
                else
                {
                    position[id] = ((360.0 * (float)tmp_val) / (4096.0 - 1.0)) - (360.0 / 2);
                }
            }
        }
        id++;
        if (id == DXL_NUMBER) {
            id = 0;
        }

        last_pos_systick = timestamp;
    }
}

void handle_cmd(void)
{
    if (request_nb > 0)
    {
        for (int i=0; i < MAX_CMD; i++) 
        {
            if (cmd[i].waiting != 0)
            {
                if ((cmd[i].reg == SERVO_REGISTER_TORQUE_ENABLE) ||
                    (cmd[i].reg == SERVO_REGISTER_ID))
                {
                    servo_set_raw_byte(cmd[i].motor_id, cmd[i].reg, cmd[i].val, DXL_TIMEOUT);
                }
                else 
                {
                    servo_set_raw_word(cmd[i].motor_id, cmd[i].reg, cmd[i].val, DXL_TIMEOUT);
                }
                cmd[i].waiting = 0;

                __disable_irq();
                request_nb--;
                __enable_irq();

                break;
            }
        }
    }
}

void dxl_loop(void) 
{
    get_motor_info();
    handle_cmd();
}