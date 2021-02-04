#include "fan.h"
#include "main.h"
#include "luos.h"
#include "reachy.h"
#include "pub_msg.h"


void Fan_Init()
{
    // ********************* Gpio configuration ****************************
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    // GPIO Ports Clock Enable
    __HAL_RCC_GPIOB_CLK_ENABLE();
    // Set a safe pin state just in case
    HAL_GPIO_WritePin(SHOULDER_FAN_GPIO_Port, SHOULDER_FAN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ELBOW_FAN_GPIO_Port, ELBOW_FAN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(WRIST_FAN_GPIO_Port, WRIST_FAN_Pin, GPIO_PIN_RESET);
    // configure your pin using the GPIO_InitTypeDef structure
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    GPIO_InitStruct.Pin = SHOULDER_FAN_Pin;
    HAL_GPIO_Init(SHOULDER_FAN_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = ELBOW_FAN_Pin;
    HAL_GPIO_Init(ELBOW_FAN_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = WRIST_FAN_Pin;
    HAL_GPIO_Init(WRIST_FAN_GPIO_Port, &GPIO_InitStruct);

    HAL_GPIO_WritePin(SHOULDER_FAN_GPIO_Port, SHOULDER_FAN_Pin, 0);
    HAL_GPIO_WritePin(ELBOW_FAN_GPIO_Port, ELBOW_FAN_Pin, 0);
    HAL_GPIO_WritePin(WRIST_FAN_GPIO_Port, WRIST_FAN_Pin, 0);
}

void Fan_Loop(void)
{

}

void Fan_MsgHandler(container_t *src, msg_t *msg)
{
    if ((msg->header.cmd == REGISTER) && (msg->data[0] == MSG_TYPE_FAN_GET_STATE))
    {
        // <-- [MSG_TYPE_FAN_GET_STATE, (FAN_ID)+]
        // --> [MSG_TYPE_FAN_PUB_DATA, (FAN_ID, STATE)+]

        uint8_t num_ids = msg->header.size - 1;
        
        uint8_t payload_size = 1 + 2 * num_ids;
        uint8_t payload[payload_size];

        payload[0] = MSG_TYPE_FAN_PUB_DATA;

        for (uint8_t i=0; i < num_ids; i++)
        {
            uint8_t dxl_id = msg->data[1 + i];
            uint8_t fan_id = fan_id_from_dxl_id(dxl_id);

            payload[1 + 2 * i] = dxl_id;
            payload[2 + 2 * i] = get_fan_state(fan_id);
        }
        send_to_gate(src, payload, payload_size);
    }
    else if ((msg->header.cmd == REGISTER) && (msg->data[0] == MSG_TYPE_FAN_SET_STATE))
    {
        // [MSG_TYPE_FAN_SET_STATE, (FAN_ID, STATE)+]
        uint8_t num_ids = (msg->header.size - 1) / 2;

        for (uint8_t i=0; i < num_ids; i++)
        {
            uint8_t *fan_data = msg->data + 1 + i * 2;

            uint8_t dxl_id = fan_data[0];
            uint8_t fan_id = fan_id_from_dxl_id(dxl_id);
            uint8_t state = fan_data[1];

            set_fan_state(fan_id, state);
        }
    }
}

uint8_t fan_id_from_dxl_id(uint8_t dxl_id)
{
    if (dxl_id == 10 || dxl_id == 20)
    {
        return SHOULDER_FAN_ID;
    }
    else if (dxl_id == 13 || dxl_id == 23)
    {
        return ELBOW_FAN_ID;
    }
    else if (dxl_id == 15 || dxl_id == 25)
    {
        return WRIST_FAN_ID;
    }
    else 
    {
        LUOS_ASSERT (0);
    }
    return 255;
}

uint8_t get_fan_state(uint8_t fan_id)
{
    if (fan_id == SHOULDER_FAN_ID)
    {
        return HAL_GPIO_ReadPin(SHOULDER_FAN_GPIO_Port, SHOULDER_FAN_Pin);
    }
    else if (fan_id == ELBOW_FAN_ID)
    {
        return HAL_GPIO_ReadPin(ELBOW_FAN_GPIO_Port, ELBOW_FAN_Pin);
    }
    else if (fan_id == WRIST_FAN_ID)
    {
        return HAL_GPIO_ReadPin(WRIST_FAN_GPIO_Port, WRIST_FAN_Pin);
    }
    else 
    {
        LUOS_ASSERT (0);
        return 0;
    }
}

void set_fan_state(uint8_t fan_id, uint8_t state)
{
    LUOS_ASSERT (state == 0 || state == 1);

    if (fan_id == SHOULDER_FAN_ID)
    {
        HAL_GPIO_WritePin(SHOULDER_FAN_GPIO_Port, SHOULDER_FAN_Pin, state);
    }
    else if (fan_id == ELBOW_FAN_ID)
    {
        HAL_GPIO_WritePin(ELBOW_FAN_GPIO_Port, ELBOW_FAN_Pin, state);
    }
    else if (fan_id == WRIST_FAN_ID)
    {
        HAL_GPIO_WritePin(WRIST_FAN_GPIO_Port, WRIST_FAN_Pin, state);
    }
    else 
    {
        LUOS_ASSERT (0);
    }
}