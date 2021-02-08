#include "gate.h"
#include "reachy.h"
#include "myserial.h"

#define SEND_BUFF_SIZE 64
uint8_t send_buff[SEND_BUFF_SIZE] = {0};

#define RECV_BUFF_SIZE 64
#define RECV_RING_BUFFER_SIZE 15
volatile uint8_t recv_buff[RECV_RING_BUFFER_SIZE][RECV_BUFF_SIZE] = {0};
volatile uint8_t recv_buff_msg_size[RECV_RING_BUFFER_SIZE] = {0};
static volatile uint8_t nb_recv_buff = 0;
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
    my_container = Luos_CreateContainer(Gate_MsgHandler, GATE_MOD, "gate", revision);
}

void Gate_Loop(void)
{
    if (nb_recv_buff > 0)
    {
        uint8_t bytes_read = recv_buff_msg_size[recv_buff_read_index];
        uint8_t *msg = (uint8_t *)recv_buff[recv_buff_read_index];

        while (bytes_read > 0)
        {
            uint8_t header_offset = 0;

            ASSERT ((msg[header_offset] == 255));
            header_offset++;

            if (msg[header_offset] == 255)
            {
                header_offset++;
            }

            uint8_t payload_size = msg[header_offset];
            header_offset++;
            uint8_t msg_size = payload_size + header_offset;
            ASSERT (msg_size <= bytes_read);

            handle_inbound_msg(msg + header_offset, payload_size);

            bytes_read -= msg_size;
            msg += msg_size;
        }

        recv_buff_read_index++;
        if (recv_buff_read_index == RECV_RING_BUFFER_SIZE)
        {
            recv_buff_read_index = 0;
        }

        __disable_irq();
        nb_recv_buff--;
        __enable_irq();
    }

    status_led(!is_alive());
}

void Gate_MsgHandler(container_t *src, msg_t *msg)
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
        send_on_serial(msg->data, msg->header.size);
    }
}

void handle_inbound_msg(uint8_t data[], uint8_t payload_size)
{
    uint8_t msg_type = data[0];

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
    else if (msg_type == MSG_DETECTION_RUN)
    {
        RoutingTB_DetectContainers(my_container);
    }
    else if (msg_type == MSG_DETECTION_GET_NODES)
    {
        // <-- [MSG_DETECTION_GET_NODES]
        // --> // [MSG_DETECTION_PUB_NODES, (NODE_ID)+]

        uint8_t payload_size = 1 + RoutingTB_GetNodeNB() + 1;
        uint8_t payload[payload_size];
        payload[0] = MSG_DETECTION_PUB_NODES;

        routing_table_t *routing_table = RoutingTB_Get();
        uint8_t i = 1;
        for (uint16_t node_id=0; node_id < RoutingTB_GetLastEntry(); node_id++)
        {
            if (routing_table[node_id].mode == NODE)
            {
                payload[i++] = node_id;
            }
        }
        send_on_serial(payload, payload_size);
    }
    else if (msg_type == MSG_DETECTION_GET_CONTAINERS)
    {
        // <-- [MSG_DETECTION_GET_CONTAINERS, (NODE_ID)+]
        // --> N x [MSG_DETECTION_PUB_CONTAINERS, NODE_ID, (CONTAINER_ID)+]
        uint8_t num_nodes = payload_size - 1;
        routing_table_t *routing_table = RoutingTB_Get();
        for (uint8_t i=0; i < num_nodes; i++)
        {
            uint8_t node_id = data[1 + i];
            uint8_t num_containers = RoutingTB_Get_ContainerNB(node_id);
            uint8_t payload_size = 2 + num_containers;
            uint8_t payload[payload_size];
            payload[0] = MSG_DETECTION_PUB_CONTAINERS;
            payload[1] = node_id;

            for (uint8_t j=0; j < num_containers; j++)
            {
                uint8_t container_id = node_id + j + 1;
                payload[2 + j] = routing_table[container_id].id;
            }
            send_on_serial(payload, payload_size);
        }
    }
    else if (msg_type == MSG_DETECTION_GET_CONTAINER_INFO)
    {
        // <-- [MSG_DETECTION_GET_CONTAINER_INFO, (CONTAINER_ID)+]
        // --> N x [MSG_DETECTION_PUB_CONTAINER_INFO, CONTAINER_ID, TYPE, ALIAS]
        uint8_t num_containers = payload_size = 1;
        routing_table_t *routing_table = RoutingTB_Get();
        for (uint8_t i=0; i < num_containers; i++)
        {
            uint8_t container_id = data[1 + i];
            ASSERT (
                (routing_table[container_id].mode == CONTAINER)  ||
                (routing_table[container_id].mode == NODE)
            );

            char *alias = RoutingTB_AliasFromId(container_id);
            char *type = RoutingTB_StringFromType(RoutingTB_TypeFromID(container_id));

            char info[32];
            sprintf(info, "%s %s", alias, type);

            uint8_t payload_size = 2 + strlen(info);
            uint8_t payload[payload_size];
            payload[0] = MSG_DETECTION_PUB_CONTAINER_INFO;
            payload[1] = container_id;
            memcpy(payload + 2, info, strlen(info));

            send_on_serial(payload, payload_size);
        }
    }
    else if ((msg_type == MSG_TYPE_DXL_GET_REG) || (msg_type == MSG_TYPE_DXL_SET_REG))
    {
        // [MSG_TYPE_DXL_GET_REG, DXL_REG, NB_BYTES, (DXL_ID)+]
        // [MSG_TYPE_DXL_SET_REG, DXL_REG, NB_BYTES, (DXL_ID, VAL_H, (VAL_L)*)+]

        msg_t msg;
        msg.header.target_mode = IDACK;
        msg.header.cmd = REGISTER;
        msg.header.size = payload_size;
        memcpy(msg.data, data, payload_size);

        char alias[15];
        uint8_t first_dxl_id = data[3];
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
        memcpy(msg.data, data, payload_size);

        char alias[15];
        uint8_t first_fan_id = data[1];
        // We use the dxl container to circumvent
        // https://github.com/pollen-robotics/Luos-modules/issues/13
        sprintf(alias, "dxl_%d", first_fan_id);
        uint16_t container_id = RoutingTB_IDFromAlias(alias);
        ASSERT (container_id != 0xFFFF);
        msg.header.target = container_id;

        Luos_SendMsg(my_container, &msg);
    }
    else if ((msg_type == MSG_TYPE_ORBITA_GET_REG) || (msg_type == MSG_TYPE_ORBITA_SET_REG))
    {
        // [MSG_TYPE_ORBITA_GET_REG, ORBITA_ID, REG_TYPE]
        // [MSG_TYPE_ORBITA_SET_REG, ORBITA_ID, REG_TYPE, (MOTOR_ID, (VAL+))+]

        msg_t msg;
        msg.header.target_mode = IDACK;
        msg.header.cmd = REGISTER;
        msg.header.size = payload_size;
        memcpy(msg.data, data, payload_size);

        char alias[15];
        uint8_t orbita_id = data[1];
        sprintf(alias, "orbita_%d", orbita_id);
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

        recv_buff_msg_size[recv_buff_write_index] = RECV_BUFF_SIZE - LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_3);

        recv_buff_write_index++;
        if (recv_buff_write_index == RECV_RING_BUFFER_SIZE)
        {
            recv_buff_write_index = 0;
        }
        nb_recv_buff++;
        ASSERT (nb_recv_buff < (RECV_RING_BUFFER_SIZE - 1));

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

void send_on_serial(uint8_t payload[], uint8_t payload_size)
{
    send_buff[0] = 255;
    send_buff[1] = 255;
    send_buff[2] = payload_size;
    memcpy(send_buff + 3, payload, payload_size);

    serial_write(send_buff, payload_size + 3);
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

uint16_t RoutingTB_Get_ContainerNB(uint16_t node_id)
{
    routing_table_t *routing_table = RoutingTB_Get();
    ASSERT (routing_table[node_id].mode == NODE);

    uint16_t nb = 0;
    for (uint16_t i=node_id + 1; i < RoutingTB_GetLastEntry(); i++)
    {
        if (routing_table[i].mode == NODE)
        {
            break;
        }
        else if (routing_table[i].mode == CONTAINER)
        {
            nb++;
        }
    }
    return nb;
}