//
// Created by Kok on 7/12/25.
//

#include "error.h"

#include <stdio.h>
#include <stdarg.h>
#include <system_config.h>

#include "gpio_driver.h"
#include "usart_driver.h"

int __io_putchar(int ch) {
    USART_PeripheralTXControl(system_handles.pUSARTHandle, ENABLE);
    USART_SendData(system_handles.pUSARTHandle, (uint8_t*)&ch, sizeof(ch));
    USART_PeripheralTXControl(system_handles.pUSARTHandle, DISABLE);
    return ch;
};

/**
 * @brief Initializes the GPIO for the error LED
 */
void InitErrorLED() {
    GPIO_PinConfig_t config = {
        .GPIO_PinNumber = ERR_LED_PIN,
        .GPIO_PinSpeed = GPIO_SpeedMedium,
        .GPIO_PinOPType = GPIO_OpTypePP,
        .GPIO_PinPuPdControl = GPIO_Pu,
        .GPIO_PinMode = GPIO_ModeOutput,
    };

    system_handles.pGPIOHandle->pGPIOx = ERR_LED_PORT;
    system_handles.pGPIOHandle->GPIO_PinConfig = config;
    GPIO_Init(system_handles.pGPIOHandle);
}

/**
 * @brief Enables the error LED and transmits error message via USART
 * @param msg Message which is transmitted via USART
 */
void TriggerError(char *msgFormat, ...) {
    // Enable error LED
    GPIO_WriteToOutputPin(ERR_LED_PORT, ERR_LED_PIN, ENABLE);

    // Send message
    va_list args;
    va_start(args, msgFormat);

    vprintf(msgFormat, args);

    va_end(args);
}

void ClearActiveError() {
    // Disable error LED
    GPIO_WriteToOutputPin(ERR_LED_PORT, ERR_LED_PIN, DISABLE);

    // Send message
    printf("Error cleared! Program continuing...\n");
}
