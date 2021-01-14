#include "pub_msg.h"
#include "dxl.h"

void send_to_gate(container_t *src, uint8_t payload[], uint8_t payload_size)
{
    msg_t msg;
    msg.header.target = 1;
    msg.header.target_mode = ID;
    msg.header.cmd = ASK_PUB_CMD;
    msg.header.size = payload_size;
    memcpy(msg.data, payload, payload_size);

    Luos_SendMsg(src, &msg);
}

void send_dxl_byte_to_gate(container_t *src, uint8_t dxl_id, uint8_t reg, uint16_t error, uint8_t value)
{
    uint8_t payload[7];

    payload[0] = MSG_TYPE_DXL_PUB_DATA;
    payload[1] = reg;
    payload[2] = 1;
    payload[3] = dxl_id;
    memcpy(payload + 4, &error, sizeof(uint16_t));
    payload[6] = value;

    send_to_gate(src, payload, 7);
}


void send_dxl_word_to_gate(container_t *src, uint8_t dxl_id, uint8_t reg, uint16_t error, uint16_t value)
{
    uint8_t payload[8];

    payload[0] = MSG_TYPE_DXL_PUB_DATA;
    payload[1] = reg;
    payload[2] = 2;
    payload[3] = dxl_id;
    memcpy(payload + 4, &error, sizeof(uint16_t));
    memcpy(payload + 6, &value, sizeof(uint16_t));
    
    send_to_gate(src, payload, 8);
}


void send_dxl_page_to_gate(container_t *src, uint8_t dxl_id, uint8_t reg, uint16_t error, uint8_t values[], uint8_t val_size)
{
    uint8_t payload[6 + val_size];

    payload[0] = MSG_TYPE_DXL_PUB_DATA;
    payload[1] = reg;
    payload[2] = val_size;
    payload[3] = dxl_id;
    memcpy(payload + 4, &error, sizeof(uint16_t));
    memcpy(payload + 6, values, val_size);

    send_to_gate(src, payload, 6 + val_size);
}

void send_positions_to_gate(container_t *src, uint8_t dxl_ids[], uint16_t positions[], servo_error_t errors[], uint8_t nb_ids)
{
    uint8_t data_size = 5;
    uint8_t payload_size = 3 + data_size * nb_ids;
    uint8_t payload[payload_size];

    payload[0] = MSG_TYPE_DXL_PUB_DATA;
    payload[1] = SERVO_REGISTER_PRESENT_ANGLE;
    payload[2] = 2;

    for (int i=0; i < nb_ids; i++)
    {
        uint8_t *data = payload + 3 + i * data_size;

        data[0] = dxl_ids[i];
        memcpy(data + 1, errors + i, sizeof(servo_error_t));
        memcpy(data + 3, positions + i, sizeof(uint16_t));
    }

    send_to_gate(src, payload, payload_size);
}