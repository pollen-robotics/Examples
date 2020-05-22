#include "main.h"
#include "fan.h"


module_t* fans[1];

void rx_fan_cb(module_t *module, msg_t *msg) {
    if (msg->header.cmd == COLOR) {
        HAL_GPIO_WritePin(SHOULDER_FAN_GPIO_Port, SHOULDER_FAN_Pin, msg->data[0] != 0);
        HAL_GPIO_WritePin(ELBOW_FAN_GPIO_Port, ELBOW_FAN_Pin, msg->data[1] != 0);
        HAL_GPIO_WritePin(WRIST_FAN_GPIO_Port, WRIST_FAN_Pin, msg->data[2] != 0);
    }
}

void fan_init(void) {
    fans[0] = luos_module_create(rx_fan_cb, REACHY_FAN_MOD, "reachy_fans");

    luos_module_enable_rt(fans[0]);

    // ********************* Gpio configuration ****************************
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    // GPIO Ports Clock Enable
    __HAL_RCC_GPIOB_CLK_ENABLE();
    // Set a safe pin state just in case
    HAL_GPIO_WritePin(SHOULDER_FAN_GPIO_Port, SHOULDER_FAN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ELBOW_FAN_GPIO_Port, ELBOW_FAN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(WRIST_FAN_GPIO_Port, WRIST_FAN_Pin, GPIO_PIN_RESET);
    // configure your pin using the GPIO_InitTypeDef structure
    GPIO_InitStruct.Pin = SHOULDER_FAN_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(SHOULDER_FAN_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = ELBOW_FAN_Pin;
    HAL_GPIO_Init(ELBOW_FAN_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = WRIST_FAN_Pin;
    HAL_GPIO_Init(WRIST_FAN_GPIO_Port, &GPIO_InitStruct);
}

void fan_loop(void) {
}
