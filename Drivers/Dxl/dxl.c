#include "dxl.h"
#include "pub_msg.h"
#include "Dynamixel_Servo.h"

// TODO: passer en mode detection + NB_DXL_MAX
#ifdef LEFT_ARM
uint8_t DXL_IDS[NB_DXL] = {20, 21, 22, 23, 24, 25, 26, 27};
#else
uint8_t DXL_IDS[NB_DXL] = {10, 11, 12, 13, 14, 15, 16, 17};
#endif

#define DXL_TIMEOUT 2  // in ms
#define TEMP_PUBLISH_PERIOD 1000  // in ms

#define KEEP_ALIVE_PERIOD 1100  // in ms
static uint32_t keep_alive = 0;

container_t* dxl_container[NB_DXL];


void Dxl_Init(void)
{
    status_led(0);

    servo_init(1000000);
    HAL_Delay(500);

    char dxl_name[15];
    revision_t revision = {.unmap = REV};

    for (int i=0; i < NB_DXL; i++)
    {
        sprintf(dxl_name, "dxl_%d", DXL_IDS[i]);
        dxl_container[i] = Luos_CreateContainer(Dxl_MsgHandler, DYNAMIXEL_MOD, dxl_name, revision);
    }
}

void Dxl_Loop(void)
{
    if (!is_alive())
    {
        status_led(1);
        return;
    }
    status_led(0);

    static uint8_t dxl_id = 0;

    uint16_t position_result = 0;
    servo_error_t error = servo_get_raw_word(DXL_IDS[dxl_id], SERVO_REGISTER_PRESENT_ANGLE, &position_result, DXL_TIMEOUT);
    send_dxl_word_to_gate(dxl_container[dxl_id], DXL_IDS[dxl_id], SERVO_REGISTER_PRESENT_ANGLE, error, position_result);

    dxl_id++;
    if (dxl_id == NB_DXL)
    {
        dxl_id = 0;
    }

    static uint32_t last_temperature_published = 0;
    static uint8_t temperature_id = 0;
    uint32_t current_tick = HAL_GetTick();

    if ((current_tick - last_temperature_published) > TEMP_PUBLISH_PERIOD)
    {
        uint8_t result;
        servo_error_t error = servo_get_raw_byte(DXL_IDS[temperature_id], SERVO_REGISTER_PRESENT_TEMPERATURE, &result, DXL_TIMEOUT);
        send_dxl_byte_to_gate(dxl_container[temperature_id], DXL_IDS[temperature_id], SERVO_REGISTER_PRESENT_TEMPERATURE, error, result);

        temperature_id++;
        if (temperature_id == NB_DXL)
        {
            temperature_id = 0;
        }

        last_temperature_published = current_tick;
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
        // [MSG_TYPE_DXL_GET_REG, DXL_ID, DXL_REG, NB_BYTES]

        uint8_t dxl_id = msg->data[1];
        LUOS_ASSERT (dxl_id_exists(dxl_id));
        uint8_t container_id = get_dxl_id(dxl_id);

        uint8_t reg = msg->data[2];

        uint8_t val_size = msg->data[3];
        LUOS_ASSERT (val_size > 0);

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

    else if ((msg->header.cmd == REGISTER) && (msg->data[0] == MSG_TYPE_DXL_SET_REG))
    {        
        // [MSG_TYPE_DXL_SET_REG, DXL_ID, DXL_REG, (VAL)+]

        uint8_t dxl_id = msg->data[1];
        LUOS_ASSERT (dxl_id_exists(dxl_id));

        uint8_t reg = msg->data[2];

        uint8_t val_size = msg->header.size - 3;
        LUOS_ASSERT (val_size > 0);

        if (val_size == 1)
        {
            uint8_t val = msg->data[3];
            servo_error_t error = servo_set_raw_byte(dxl_id, reg, val, DXL_TIMEOUT);
            LUOS_ASSERT (error == 0);
        }
        else if (val_size == 2)
        {
            uint16_t value;
            memcpy(&value, msg->data + 3, sizeof(uint16_t));
            servo_error_t error = servo_set_raw_word(dxl_id, reg, value, DXL_TIMEOUT);
            LUOS_ASSERT (error == 0);
        }
        else 
        {
            uint8_t values[val_size];
            memcpy(values, msg->data + 3, val_size);
            servo_error_t error = servo_set_raw_page(dxl_id, reg, values, val_size, DXL_TIMEOUT);
            LUOS_ASSERT (error == 0);
        }
    }
}

uint8_t get_dxl_id(uint8_t id)
{
    for (int i=0; i < NB_DXL; i++)
    {
        if (DXL_IDS[i] == id)
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

uint8_t is_alive()
{
    return (HAL_GetTick() - keep_alive) <= KEEP_ALIVE_PERIOD;
}

void status_led(uint8_t state)
{
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, (state == 0));
}
