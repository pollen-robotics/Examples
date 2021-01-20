#include "gate.h"
#include "reachy.h"
#include "myserial.h"

#define SEND_BUFF_SIZE 64
uint8_t send_buff[SEND_BUFF_SIZE] = {0};

#define RECV_BUFF_SIZE 64
#define RECV_RING_BUFFER_SIZE 5
volatile uint8_t recv_buff[RECV_RING_BUFFER_SIZE][RECV_BUFF_SIZE] = {0};
static volatile uint8_t nb_recv_msg = 0;
static volatile uint8_t recv_buff_read_index = 0;
static volatile uint8_t recv_buff_write_index = 0;

#define KEEP_ALIVE_PERIOD 1100
static uint32_t keep_alive = 0;

#define ASSERT(cond) _assert(cond, __FILE__, __LINE__)

container_t *my_container;

void Gate_Init(void)
{
    status_led(0);

    LL_USART_ClearFlag_IDLE(USART3);
    LL_USART_EnableIT_IDLE(USART3);
    NVIC_DisableIRQ(DMA1_Channel2_3_IRQn);
    LL_DMA_DisableIT_TC(DMA1, LL_DMA_CHANNEL_3);
    LL_DMA_DisableIT_HT(DMA1, LL_DMA_CHANNEL_3);
    LL_DMA_DisableIT_TE(DMA1, LL_DMA_CHANNEL_3);
    LL_DMA_SetM2MDstAddress(DMA1, LL_DMA_CHANNEL_3, (uint32_t)recv_buff[recv_buff_write_index]);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, RECV_BUFF_SIZE);
    LL_DMA_SetM2MSrcAddress(DMA1, LL_DMA_CHANNEL_3, (uint32_t)&USART3->RDR);
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);
    LL_USART_EnableDMAReq_RX(USART3);

    revision_t revision = {.unmap = REV};
    my_container = Luos_CreateContainer(Dxl_MsgHandler, GATE_MOD, "gate", revision);
}

void Gate_Loop(void)
{
    static uint8_t detection_done = 0;

    if (!detection_done)
    {
        status_led(1);
        while (1)
        {
            HAL_Delay(1000);

            RoutingTB_DetectContainers(my_container);            

            if (RoutingTB_GetLastContainer() > 0)
            {
                break;
            }
        }
        detection_done = 1;
        status_led(0);
    }

    if (nb_recv_msg > 0)
    {
        handle_inbound_msg((uint8_t *)recv_buff[recv_buff_read_index]);

        recv_buff_read_index++;
        if (recv_buff_read_index == RECV_RING_BUFFER_SIZE)
        {
            recv_buff_read_index = 0;
        }

        __disable_irq();
        nb_recv_msg--;
        __enable_irq();
    }

    status_led(!is_alive());
}

void Dxl_MsgHandler(container_t *src, msg_t *msg)
{
    if (msg->header.cmd == ASSERT)
    {
        uint32_t line;
        memcpy(&line, msg->data, sizeof(uint32_t));

        char file[64];
        memcpy(file, msg->data + sizeof(uint32_t), msg->header.size - sizeof(uint32_t));

        _assert(0, file, line);
    }

    if (!is_alive())
    {
        return;
    }

    if (msg->header.cmd == ASK_PUB_CMD)
    {
        send_buff[0] = 255;
        send_buff[1] = 255;
        send_buff[2] = msg->header.size;
        memcpy(send_buff + 3, msg->data, msg->header.size);

        serial_write(send_buff, msg->header.size + 3);
    }
}

void handle_inbound_msg(uint8_t data[])
{
    ASSERT (data[0] == 255);
    ASSERT (data[1] == 255);
    uint8_t payload_size = data[2];

    uint8_t msg_type = data[3];

    if (msg_type == MSG_TYPE_KEEP_ALIVE)
    {
        msg_t keep_alive_msg;

        keep_alive_msg.header.target_mode = IDACK;
        keep_alive_msg.header.cmd = REGISTER;
        keep_alive_msg.header.size = 1;
        keep_alive_msg.data[0] = MSG_TYPE_KEEP_ALIVE;

        routing_table_t *tb = RoutingTB_Get();
        for (uint16_t i=1; i < RoutingTB_GetLastEntry(); i++)
        {
            if (tb[i].mode == NODE)
            {
                keep_alive_msg.header.target = RoutingTB_GetNodeID(i);
                Luos_SendMsg(my_container, &keep_alive_msg);
            }
        }

        keep_alive = HAL_GetTick();
    }

    else if ((msg_type == MSG_TYPE_DXL_GET_REG) || (msg_type == MSG_TYPE_DXL_SET_REG))
    {
        // [MSG_TYPE_DXL_GET_REG, DXL_REG, NB_BYTES, (DXL_ID)+]
        // [MSG_TYPE_DXL_SET_REG, DXL_REG, NB_BYTES, (DXL_ID, VAL_H, (VAL_L)*)+]

        msg_t msg;
        msg.header.target_mode = IDACK;
        msg.header.cmd = REGISTER;
        msg.header.size = payload_size;
        memcpy(msg.data, data + 3, payload_size);

        char alias[15];
        uint8_t first_dxl_id = data[6];
        sprintf(alias, "dxl_%d", first_dxl_id);
        uint16_t container_id = RoutingTB_IDFromAlias(alias);
        ASSERT (container_id != 0xFFFF);
        msg.header.target = container_id;

        Luos_SendMsg(my_container, &msg);
    }
    else if ((msg_type == MSG_TYPE_FAN_GET_STATE) || (msg_type == MSG_TYPE_FAN_SET_STATE))
    {
        // [MSG_TYPE_FAN_GET_STATE, (FAN_ID)+]
        // [MSG_TYPE_FAN_SET_STATE, (FAN_ID, STATE)+]

        msg_t msg;
        msg.header.target_mode = IDACK;
        msg.header.cmd = REGISTER;
        msg.header.size = payload_size;
        memcpy(msg.data, data + 3, payload_size);

        char alias[15];
        uint8_t first_fan_id = data[4];
        sprintf(alias, "fan_%d", first_fan_id);
        uint16_t container_id = RoutingTB_IDFromAlias(alias);
        ASSERT (container_id != 0xFFFF);
        msg.header.target = container_id;

        Luos_SendMsg(my_container, &msg);
    }
    else
    {
        ASSERT (0);
    }
}

void USART3_4_IRQHandler(void)
{
    // check if we receive an IDLE on usart3
    if (LL_USART_IsActiveFlag_IDLE(USART3))
    {
        LL_USART_ClearFlag_IDLE(USART3);

        // reset DMA
        __disable_irq();

        nb_recv_msg++;
        recv_buff_write_index++;

        if (recv_buff_write_index == RECV_RING_BUFFER_SIZE)
        {
            recv_buff_write_index = 0;
        }

        ASSERT (nb_recv_msg < RECV_RING_BUFFER_SIZE);

        LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);

        LL_DMA_SetM2MDstAddress(DMA1, LL_DMA_CHANNEL_3, (uint32_t)recv_buff[recv_buff_write_index]);
        LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, RECV_BUFF_SIZE);
        LL_DMA_SetM2MSrcAddress(DMA1, LL_DMA_CHANNEL_3, (uint32_t)&USART3->RDR);
        LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_3, (uint32_t)recv_buff[recv_buff_write_index]);
        LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);
        LL_USART_EnableDMAReq_RX(USART3);

        __enable_irq();
    }
}

uint8_t is_alive()
{
    return (HAL_GetTick() - keep_alive) <= KEEP_ALIVE_PERIOD;
}

void _assert(uint8_t condition, char *file, uint32_t line)
{
    if (condition == 0)
    {
        status_led(1);

        static char assert_msg[60];
        sprintf(assert_msg, "Assert %s %ld", file, line);
        uint8_t msg_len = strlen(assert_msg);
        send_buff[0] = 255;
        send_buff[1] = 255;
        send_buff[2] = msg_len + 1;
        send_buff[3] = MSG_MODULE_ASSERT;
        memcpy(send_buff + 4, assert_msg, msg_len);

        serial_write(send_buff, msg_len + 4);

        __disable_irq();
        while (1) ;
    }
}

void status_led(uint8_t state)
{
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, (state == 0));
}