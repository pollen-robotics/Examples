#include "dxl.h"
#include "pub_msg.h"
#include "Dynamixel_Servo.h"
#include "fan.h"

static uint32_t keep_alive = 0;

static uint8_t nb_ids;

uint8_t dxl_ids[NB_DXL_MAX];
container_t *dxl_container[NB_DXL_MAX];

uint8_t temperatures[NB_DXL_MAX];

void Dxl_Init(void)
{
    status_led(0);

    servo_init(1000000);
    HAL_Delay(500);

    dxl_detect();
}

void Dxl_Loop(void)
{
    if (!is_alive())
    {
        status_led(1);
        return;
    }
    status_led(0);

    // Send motor position (one motor per loop)
    static uint8_t motor_id = 0;
    uint16_t position;
    servo_error_t error = servo_get_raw_word(dxl_ids[motor_id], SERVO_REGISTER_PRESENT_ANGLE, &position, DXL_TIMEOUT);
    send_dxl_word_to_gate(dxl_container[motor_id], dxl_ids[motor_id], SERVO_REGISTER_PRESENT_ANGLE, error, position);
    motor_id++;
    if (motor_id == nb_ids)
    {
        motor_id = 0;
    }

    // Send motor temperature (one motor per ms)
    static uint32_t last_temperature_published = 0;
    static uint8_t temperature_id = 0;
    uint32_t current_tick = HAL_GetTick();

    if ((current_tick - last_temperature_published) > TEMP_PUBLISH_PERIOD)
    {
        uint8_t result;
        servo_error_t error = servo_get_raw_byte(dxl_ids[temperature_id], SERVO_REGISTER_PRESENT_TEMPERATURE, &result, DXL_TIMEOUT);
        send_dxl_byte_to_gate(dxl_container[temperature_id], dxl_ids[temperature_id], SERVO_REGISTER_PRESENT_TEMPERATURE, error, result);

        temperatures[temperature_id] = result;

        temperature_id++;
        if (temperature_id == nb_ids)
        {
            temperature_id = 0;
        }

        last_temperature_published = current_tick;

        check_temperature();
    }
}

void Dxl_MsgHandler(container_t *src, msg_t *msg)
{
    if ((msg->header.cmd == REGISTER) && (msg->data[0] == MSG_TYPE_KEEP_ALIVE))
    {
        keep_alive = HAL_GetTick();
    }

    else if ((msg->header.cmd == REGISTER) && (msg->data[0] == MSG_TYPE_DXL_GET_REG))
    {
        // [MSG_TYPE_DXL_GET_REG, DXL_REG, NB_BYTES, (DXL_ID)+]

        uint8_t reg = msg->data[1];
        uint8_t val_size = msg->data[2];
        LUOS_ASSERT (val_size > 0);

        uint8_t nb_ids = msg->header.size - 3;

        for (uint8_t i=0; i < nb_ids; i++)
        {
            uint8_t dxl_id = msg->data[3 + i];
            LUOS_ASSERT (dxl_id_exists(dxl_id));
            uint8_t container_id = get_dxl_id(dxl_id);

            if (val_size == 1)
            {
                uint8_t val;
                servo_error_t error = servo_get_raw_byte(dxl_id, reg, &val, DXL_TIMEOUT);
                send_dxl_byte_to_gate(dxl_container[container_id], dxl_id, reg, error, val);
            }
            else if (val_size == 2)
            {
                uint16_t val;
                servo_error_t error = servo_get_raw_word(dxl_id, reg, &val, DXL_TIMEOUT);
                send_dxl_word_to_gate(dxl_container[container_id], dxl_id, reg, error, val);
            }
            else 
            {
                uint8_t values[val_size];
                servo_error_t error = servo_get_raw_page(dxl_id, reg, values, val_size, DXL_TIMEOUT);
                send_dxl_page_to_gate(dxl_container[container_id], dxl_id, reg, error, values, val_size);
            }
        }
    }

    else if ((msg->header.cmd == REGISTER) && (msg->data[0] == MSG_TYPE_DXL_SET_REG))
    {        
        // [MSG_TYPE_DXL_SET_REG, DXL_REG, NB_BYTES, (DXL_ID, VAL_H, (VAL_L)*)+]

        // SYNC_WRITE
        uint8_t reg = msg->data[1];
        uint8_t num_bytes_per_servo = msg->data[2];

        uint8_t num_ids = (msg->header.size - 3) / (1 + num_bytes_per_servo);

        uint8_t dxl_ids[num_ids];
        uint8_t values[num_ids * num_bytes_per_servo];

        for (uint8_t i=0; i < num_ids; i++)
        {
            uint8_t *data = msg->data + 3 + i * (1 + num_bytes_per_servo);

            LUOS_ASSERT (dxl_id_exists(data[0]));
            dxl_ids[i] = data[0];

            memcpy(values + i * num_bytes_per_servo, data + 1, num_bytes_per_servo);
        }

        servo_error_t error = servo_set_multiple_raw(dxl_ids, reg, values, num_ids, num_bytes_per_servo);
        LUOS_ASSERT (error == 0);
    }

    else if ((msg->header.cmd == REGISTER) && (msg->data[0] == MSG_TYPE_FAN_GET_STATE))
    {
        // [MSG_TYPE_FAN_GET_STATE, (FAN_ID)+]
        Fan_MsgHandler(src, msg);
    }
    else if ((msg->header.cmd == REGISTER) && (msg->data[0] == MSG_TYPE_FAN_SET_STATE))
    {
        // [MSG_TYPE_FAN_SET_STATE, (FAN_ID, STATE)+]
        Fan_MsgHandler(src, msg);
    }
}

void dxl_detect()
{
    revision_t revision = {.unmap = REV};
    memset(dxl_container, 0, NB_DXL_MAX * sizeof(container_t *));
    memset(dxl_ids, 0, NB_DXL_MAX * sizeof(uint8_t));

    char dxl_name[15];
    nb_ids = 0;

    for (int id=0; id < MAX_DXL_ID; id++)
    {
        LUOS_ASSERT (!is_alive());
        if (servo_ping(id, DXL_TIMEOUT) == SERVO_NO_ERROR)
        {
            sprintf(dxl_name, "dxl_%d", id);
            dxl_container[nb_ids] = Luos_CreateContainer(Dxl_MsgHandler, DYNAMIXEL_MOD, dxl_name, revision);
            dxl_ids[nb_ids] = id;
            nb_ids++;
        }

        LUOS_ASSERT (nb_ids <= NB_DXL_MAX);
    }
}

uint8_t get_dxl_id(uint8_t id)
{
    for (uint8_t i=0; i < nb_ids; i++)
    {
        if (dxl_ids[i] == id)
        {
            return i;
        }
    }
    return 255;
}

uint8_t dxl_id_exists(uint8_t id)
{
    return get_dxl_id(id) != 255;
}


uint8_t one_temp_above_limit()
{
    for (uint8_t i=0; i < nb_ids; i++)
    {
        if (temperatures[i] >= FAN_TRIGGER_TEMPERATURE)
        {
            return 1;
        }
    }
    return 0;
}

uint8_t all_below_limit()
{
    for (uint8_t i=0; i < nb_ids; i++)
    {
        if (temperatures[i] > FAN_SHUTDOWN_TEMPERATURE)
        {
            return 0;
        }
    }
    return 1;
}

void check_temperature()
{
    if (one_temp_above_limit() == 1)
    {
        set_fan_state(SHOULDER_FAN_ID, 1);
        set_fan_state(ELBOW_FAN_ID, 1);
        set_fan_state(WRIST_FAN_ID, 1);
    }
    else if (all_below_limit() == 1)
    {
        set_fan_state(SHOULDER_FAN_ID, 0);
        set_fan_state(ELBOW_FAN_ID, 0);
        set_fan_state(WRIST_FAN_ID, 0);  
    }
}

uint8_t is_alive()
{
    if (keep_alive == 0)
    {
        return 0;
    }
    return (HAL_GetTick() - keep_alive) <= KEEP_ALIVE_PERIOD;
}

void status_led(uint8_t state)
{
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, (state == 0));
}
