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
    uint8_t payload[6];

    payload[0] = MSG_TYPE_DXL_PUB_DATA;
    payload[1] = dxl_id;
    payload[2] = reg;
    memcpy(payload + 3, &error, sizeof(uint16_t));
    payload[5] = value;

    send_to_gate(src, payload, 6);
}


void send_dxl_word_to_gate(container_t *src, uint8_t dxl_id, uint8_t reg, uint16_t error, uint16_t value)
{
    uint8_t payload[7];

    payload[0] = MSG_TYPE_DXL_PUB_DATA;
    payload[1] = dxl_id;
    payload[2] = reg;
    memcpy(payload + 3, &error, sizeof(uint16_t));
    memcpy(payload + 5, &value, sizeof(uint16_t));
    
    send_to_gate(src, payload, 7);
}


void send_dxl_page_to_gate(container_t *src, uint8_t dxl_id, uint8_t reg, uint16_t error, uint8_t values[], uint8_t val_size)
{
    uint8_t payload[5 + val_size];

    payload[0] = MSG_TYPE_DXL_PUB_DATA;
    payload[1] = dxl_id;
    payload[2] = reg;
    memcpy(payload + 3, &error, sizeof(uint16_t));
    memcpy(payload + 5, values, val_size);

    send_to_gate(src, payload, 5 + val_size);
}
