//
// Created by Kok on 7/12/25.
//

#include "gpios.h"
#include "system_config.h"

/**
 * @brief GPIOS: PA3, PA4
 */
void InitUserBtnsGPIOS() {
    GPIO_PinConfig_t config = {
        .GPIO_PinNumber = USER_BTN_1_PIN,
        .GPIO_PinSpeed = GPIO_SpeedMedium,
        .GPIO_PinOPType = GPIO_OpTypePP,
        .GPIO_PinPuPdControl = GPIO_Pd,
        .GPIO_PinMode = GPIO_ModeInputREdge,
    };

    // Button 1
    system_handles.pGPIOHandle->pGPIOx = USER_BTNS_PORT;
    system_handles.pGPIOHandle->GPIO_PinConfig = config;
    GPIO_Init(system_handles.pGPIOHandle);

    // Button 2
    system_handles.pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber = USER_BTN_2_PIN;
    GPIO_Init(system_handles.pGPIOHandle);
}