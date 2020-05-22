#include "main.h"
#include "dxl.h"
#include "Dynamixel_Servo.h"
#include "math.h"
#include "string.h"

module_t* dxl_modules[DXL_NUMBER];
uint16_t dxl_table[DXL_NUMBER];
dxl_models_t dxl_model[DXL_NUMBER];
angular_position_t position[DXL_NUMBER] = {0};
temperature_t temperature[DXL_NUMBER] = {0};


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
    static uint8_t temp_period_i = 0;

    if (msg->header.cmd == ASK_PUB_CMD) 
    {
        msg_t pub_msg;
        pub_msg.header.target_mode = ID;
        pub_msg.header.target = msg->header.source;

        reachy_arm_pos_to_msg(position, DXL_NUMBER, &pub_msg);
        luos_send(module, &pub_msg);

        if (temp_period_i == 0)
        {
            reachy_arm_temp_to_msg(temperature, DXL_NUMBER, &pub_msg);
            luos_send(module, &pub_msg);
        }
        temp_period_i++;
        if (temp_period_i == 100) 
        {
            temp_period_i = 0;
        }
        return;
    }
}

void rx_dxl_cmd_cb(module_t* module, msg_t *msg) {
    uint8_t index = motor_index_from_module(module);

    if (dxl_table[index] == 0)
    {
        return;
    }

    switch (msg->header.cmd) 
    {
    case ANGULAR_POSITION:
        {
            angular_position_t ang_pos;
            angular_position_from_msg(&ang_pos, msg);

            cmd[last_request].mode = MODE_ANGLE;
            cmd[last_request].val[0] = ang_pos;
        }
        break;
    case ANGULAR_SPEED:
        {
            angular_speed_t speed;
            angular_speed_from_msg(&speed, msg);

            cmd[last_request].mode = MODE_SPEED;
            cmd[last_request].val[0] = speed;
        }
        break;
    case COMPLIANT:
        {
            cmd[last_request].mode = MODE_COMPLIANT;
            cmd[last_request].val[0] = (msg->data[0] != 1);
        }
        break;
    case RATIO_LIMIT:
        {
            float load;
            memcpy(&load, msg->data, sizeof(float));

            cmd[last_request].mode = MODE_POWER_LIMIT;
            cmd[last_request].val[0] = load;
        }
        break;
    case PID:
        {
            float fpid[3];
            memcpy(&fpid, msg->data, 3 * sizeof(float));
            
            for (int i = 0; i < 3; i++)
            {
                if (fpid[i] > 254.0) fpid[i] = 254.0;
                if (fpid[i] < 0.0) fpid[i] = 0.0;
            }
            cmd[last_request].mode = MODE_PID;

            for (int i = 0; i < 3; i++)
            {
                cmd[last_request].val[i] = fpid[i];
            }
        }
        break;
    case ANGULAR_POSITION_LIMIT:
        {
            angular_position_t angle[2];
            memcpy(&angle, msg->data, 2 * sizeof(float));
            cmd[last_request].mode = MODE_ANGLE_LIMIT;
            cmd[last_request].val[0] = angle[0];
            cmd[last_request].val[1] = angle[1];
        }
        break;
    case SETID:
        {
            char id;
            memcpy(&id, msg->data, sizeof(char));

            cmd[last_request].mode = MODE_ID;
            cmd[last_request].val[0] = id;
        }
        break;
    default:
        return;
    }

    cmd[last_request].module_id = index;
    cmd[last_request].waiting = 1;

    last_request++;
    request_nb++;

    if (last_request == MAX_CMD)
    {
        last_request = 0;
    }
}

void rx_void_dxl_cb(module_t* module, msg_t *msg) {
    switch (msg->header.cmd) 
    {
    case REINIT:
        cmd[last_request].mode = MODE_REINIT;
        break;
    default:
        return;
    }

    cmd[last_request].module_id = SERVO_BROADCAST_ID;
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
        luos_module_create(rx_void_dxl_cb, VOID_MOD, "void_dxl");
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
    static unsigned char get_pos_id = 0;
    static unsigned char get_temp_id = 0;

    // Time management vars
    static uint32_t last_pos_systick = 0;
    uint32_t pos_timestamp = HAL_GetTick();
    uint32_t pos_deltatime = pos_timestamp - last_pos_systick;

    if (pos_deltatime >= GET_POS_PERIOD)
    {
        if (dxl_table[get_pos_id] != 0) 
        {
            uint16_t tmp_val = 0;
            servo_error_t errors = servo_get_raw_word(dxl_table[get_pos_id], SERVO_REGISTER_PRESENT_ANGLE, &tmp_val, DXL_TIMEOUT);
            if ((errors != SERVO_ERROR_TIMEOUT) & (errors != SERVO_ERROR_INVALID_RESPONSE)){
                if (dxl_model[get_pos_id] == AX12 || dxl_model[get_pos_id] == AX18 || dxl_model[get_pos_id] == XL320) 
                {
                    position[get_pos_id] = ((300.0 * (float)tmp_val) / (1024.0 - 1.0)) - (300.0 / 2);
                }
                else
                {
                    position[get_pos_id] = ((360.0 * (float)tmp_val) / (4096.0 - 1.0)) - (360.0 / 2);
                }
            }
        }
        get_pos_id++;
        if (get_pos_id == DXL_NUMBER) {
            get_pos_id = 0;
        }

        last_pos_systick = pos_timestamp;
    }

    static uint32_t last_temp_systick = 0;
    uint32_t temp_timestamp = HAL_GetTick();
    uint32_t temp_deltatime = temp_timestamp - last_temp_systick;

    if (temp_deltatime >= GET_TEMP_PERIOD)
    {
        if (dxl_table[get_temp_id] != 0) 
        {
            uint16_t tmp_val = 0;
            servo_error_t errors = servo_get_raw_word(dxl_table[get_temp_id], SERVO_REGISTER_PRESENT_TEMPERATURE, &tmp_val, DXL_TIMEOUT);
            if ((errors != SERVO_ERROR_TIMEOUT) & (errors != SERVO_ERROR_INVALID_RESPONSE))
            {
                temperature[get_temp_id] = tmp_val;
            }
        }
        get_temp_id++;
        if (get_temp_id == DXL_NUMBER) {
            get_temp_id = 0;
        }

        last_temp_systick = temp_timestamp;
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
                uint8_t mod_id = cmd[i].module_id;
                uint16_t dxl_id = dxl_table[mod_id];

                switch (cmd[i].mode)
                {
                case MODE_ANGLE:
                    {
                        float pos = cmd[i].val[0];
                        uint16_t dxl_pos;

                        if (dxl_model[mod_id] == AX12 || dxl_model[mod_id] == AX18 || dxl_model[mod_id] == XL320) 
                        {
                            dxl_pos = (uint16_t)((1024 - 1) * ((300 / 2 + pos) / 300));
                        } 
                        else 
                        {
                            dxl_pos = (uint16_t)((4096 - 1) * ((360 / 2 + pos) / 360));
                        }
                        servo_set_raw_word(dxl_id, SERVO_REGISTER_GOAL_ANGLE, dxl_pos, DXL_TIMEOUT);
                    }
                    break;

                case MODE_SPEED:
                    {
                        float speed = cmd[i].val[0];
                        // Set the direction bit
                        int direction = (speed < 0) << 10;
                        // find the speed factor and compute the max speed
                        float speed_factor = 0.111;
                        if (dxl_model[mod_id] == MX12 || dxl_model[mod_id] == MX64 || dxl_model[mod_id] == MX106) {
                            speed_factor = 0.114;
                        }
                        float speed_max = 1023.0 * speed_factor * 360.0 / 60.0;
                        // Maximisation
                        speed = fminf(fmaxf(speed, -speed_max), speed_max);
                        int dxl_speed = direction + (int)(fabs(speed) / (speed_factor * 360.0 / 60.0));
                        servo_set_raw_word(dxl_id, SERVO_REGISTER_MOVING_SPEED, dxl_speed, DXL_TIMEOUT);
                    }
                    break;

                case MODE_COMPLIANT:
                    servo_set_raw_byte(dxl_id, SERVO_REGISTER_TORQUE_ENABLE, (uint8_t)cmd[i].val[0], DXL_TIMEOUT);
                    break;

                case MODE_POWER_LIMIT:
                    {
                        unsigned short limit = (unsigned short)(cmd[i].val[0] * 1023.0 / 100.0);
                        servo_set_raw_word(dxl_id, SERVO_REGISTER_TORQUE_LIMIT, limit, DXL_TIMEOUT);
                    }
                    break;

                case MODE_ID:
                    if ((int)cmd[i].val[0] < 255) {
                        unsigned char id = (unsigned char)cmd[i].val[0];
                        servo_set_raw_byte(dxl_id, SERVO_REGISTER_ID, id, DXL_TIMEOUT);

                        dxl_table[mod_id] = id;
                    }
                    break;

                case MODE_PID:
                    if (dxl_model[mod_id] >= MX12) {
                        unsigned char pid[3];
                        for (int i=0; i<3; i++)
                        {
                            pid[i] = (unsigned char)cmd[i].val[i];
                        }

                        servo_set_raw_byte(dxl_id, SERVO_REGISTER_P_GAIN, pid[0], DXL_TIMEOUT);
                        servo_set_raw_byte(dxl_id, SERVO_REGISTER_I_GAIN, pid[1], DXL_TIMEOUT);
                        servo_set_raw_byte(dxl_id, SERVO_REGISTER_D_GAIN, pid[2], DXL_TIMEOUT);
                    }
                    break;

                case MODE_ANGLE_LIMIT:
                    {
                        float lower = cmd[i].val[0];
                        float upper = cmd[i].val[1];
                        if (dxl_model[mod_id] == AX12 || dxl_model[mod_id] == AX18 || dxl_model[mod_id] == XL320) 
                        {
                            int pos = (int)((1024 - 1) * ((300 / 2 + lower) / 300));
                            servo_set_raw_word(dxl_id, SERVO_REGISTER_MIN_ANGLE, pos, DXL_TIMEOUT);
                            pos = (int)((1024 - 1) * ((300 / 2 + upper) / 300));
                            servo_set_raw_word(dxl_id, SERVO_REGISTER_MAX_ANGLE, pos, DXL_TIMEOUT);
                        }
                        else
                        {
                            int pos = (int)((4096 - 1) * ((360 / 2 + lower) / 360));
                            servo_set_raw_word(dxl_id, SERVO_REGISTER_MIN_ANGLE, pos, DXL_TIMEOUT);
                            pos = (int)((4096 - 1) * ((360 / 2 + upper) / 360));
                            servo_set_raw_word(dxl_id, SERVO_REGISTER_MAX_ANGLE, pos, DXL_TIMEOUT);
                        }
                    }
                    break;

                case MODE_REINIT:
                    {
                        servo_init(57600);
                        HAL_Delay(500);
                        servo_set_raw_byte(SERVO_BROADCAST_ID, SERVO_REGISTER_BAUD_RATE, 1, DXL_TIMEOUT);
                    }

                default:
                    break;
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