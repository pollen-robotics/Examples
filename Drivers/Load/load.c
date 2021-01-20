#include "load.h"
#include "HX711.h"
#include "reachy.h"

static uint32_t keep_alive = 0;
container_t *my_container;

void Load_Init(void)
{
    hx711_init(128);
    hx711_set_scale(10000);

    revision_t revision = {.unmap = REV};

    char alias[15];
    sprintf(alias, "load_%d", LOAD_SENSOR_ID);
    my_container = Luos_CreateContainer(Load_MsgHandler, LOAD_MOD, alias, revision);
}

void Load_Loop(void)
{
    static uint32_t last_load_published = 0;

    if (!is_alive())
    {
        status_led(1);
        return;
    }
    status_led(0);

    if ((HAL_GetTick() - last_load_published) > LOAD_PUB_PERIOD)
    {
        if (hx711_is_ready())
        {
            float load = hx711_get_units(1);
            send_load_to_gate(my_container, load);

            last_load_published = HAL_GetTick();
        }
    }
}

void Load_MsgHandler(container_t *container, msg_t *msg)
{
    if ((msg->header.cmd == REGISTER) && (msg->data[0] == MSG_TYPE_KEEP_ALIVE))
    {
        keep_alive = HAL_GetTick();
    }
}

void send_load_to_gate(container_t *src, float load)
{
    // [MSG_TYPE_LOAD_PUB_DATA, (ID, FLOAT)+]

    msg_t msg;
    msg.header.target = 1;
    msg.header.target_mode = ID;
    msg.header.cmd = ASK_PUB_CMD;
    msg.header.size = 6;

    msg.data[0] = MSG_TYPE_LOAD_PUB_DATA;
    msg.data[1] = LOAD_SENSOR_ID;
    memcpy(msg.data + 2, &load, sizeof(float));

    Luos_SendMsg(src, &msg);
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