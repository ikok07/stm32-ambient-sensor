//
// Created by Kok on 7/15/25.
//

#include "user_actions.h"

#include <gpio_driver.h>
#include <system_config.h>

static uint8_t userActionPins[TOTAL_USER_ACTIONS] = {USER_ACTION_1_PIN};

/**
 * @brief Initializes the required GPIOs for the user action buttons
 */
void InitUserBtnsGPIOS() {
    GPIO_PinConfig_t config = {
        .GPIO_PinMode = GPIO_ModeInputREdge,
        .GPIO_PinPuPdControl = GPIO_Pd,
        .GPIO_PinSpeed = GPIO_SpeedHigh,
        .GPIO_PinOPType = GPIO_OpTypePP,
    };

    system_handles.pGPIOHandle->pGPIOx = USER_ACTIONS_PORT;

    for (int i = 0; i < TOTAL_USER_ACTIONS; i++) {
        config.GPIO_PinNumber = userActionPins[i];
        system_handles.pGPIOHandle->GPIO_PinConfig = config;
        GPIO_Init(system_handles.pGPIOHandle);

        GPIO_IRQConfig(userActionPins[i], USER_ACTION_GPIOS_IRQ_PRIORITY, ENABLE);
    }
}