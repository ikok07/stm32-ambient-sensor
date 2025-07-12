//
// Created by Kok on 7/12/25.
//

#include "usart.h"
#include "system_config.h"
#include "gpio_driver.h"

/**
 * @brief Initializes the required GPIOs for the USART1 peripheral
 */
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

/**
 * @brief Initializes the USART1 peripheral
 */
USART_Error_e InitUSART() {
    USART_Config_t config = {
        .Oversampling = USART_OversamplingBy8,
        .BaudRate = USART_BaudRate_9600,
        .BitMethod = USART_OneSampleBitMethod,
        .ParityControl = USART_ParityControlDisabled,
        .StopBits = USART_StopBits1,
        .WordLength = USART_WordLength8Bits
    };

    system_handles.pUSARTHandle->pUSARTx = USARTx;
    system_handles.pUSARTHandle->USART_Config = config;

    USART_PeriClockControl(system_handles.pUSARTHandle, ENABLE);

    USART_Error_e err = USART_Init(system_handles.pUSARTHandle);
    if (err != USART_ErrOK) return err;

    USART_PeripheralControl(system_handles.pUSARTHandle, ENABLE);

    return USART_ErrOK;
}
