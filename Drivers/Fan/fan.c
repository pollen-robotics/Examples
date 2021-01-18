#include "main.h"
#include "fan.h"

#define STRINGIFY(s) STRINGIFY1(s)
#define STRINGIFY1(s) #s

module_t* fans[3];

void rx_fan_cb(module_t *module, msg_t *msg) {
    if (msg->header.cmd == IO_STATE) {
        if (module == fans[0]) {
            HAL_GPIO_WritePin(SHOULDER_FAN_GPIO_Port, SHOULDER_FAN_Pin, msg->data[0]);
        }
        if (module == fans[1]) {
            HAL_GPIO_WritePin(ELBOW_FAN_GPIO_Port, ELBOW_FAN_Pin, msg->data[0]);
        }
        if (module == fans[2]) {
            HAL_GPIO_WritePin(WRIST_FAN_GPIO_Port, WRIST_FAN_Pin, msg->data[0]);
        }
        
    }
}

void fan_init(void) {
    fans[0] = luos_module_create(rx_fan_cb, STATE_MOD, "shoulder_fan", STRINGIFY(VERSION));
    fans[1] = luos_module_create(rx_fan_cb, STATE_MOD, "elbow_fan", STRINGIFY(VERSION));
    fans[2] = luos_module_create(rx_fan_cb, STATE_MOD, "wrist_fan", STRINGIFY(VERSION));
    luos_module_enable_rt(fans[0]);
    luos_module_enable_rt(fans[1]);
    luos_module_enable_rt(fans[2]);

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
