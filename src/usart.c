//
// Created by Kok on 7/12/25.
//

#include "usart.h"
#include "system_config.h"
#include "gpio_driver.h"

void ConfigureUSART_GPIOS() {
    GPIO_PinConfig_t config = {
        .GPIO_PinNumber = USART_TX_PIN,
        .GPIO_PinSpeed = GPIO_SpeedMedium,
        .GPIO_PinOPType = GPIO_OpTypePP,
        .GPIO_PinPuPdControl = GPIO_NoPuPd,
        .GPIO_PinMode = GPIO_ModeAlternate,
        .GPIO_PinAltFunMode = GPIO_AF7
    };

    // TX pin
    system_handles.pGPIOHandle->pGPIOx = USART_PORT;
    system_handles.pGPIOHandle->GPIO_PinConfig = config;
    GPIO_Init(system_handles.pGPIOHandle);

    // RX pin
    system_handles.pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber = USART_RX_PIN;
    GPIO_Init(system_handles.pGPIOHandle);
}
