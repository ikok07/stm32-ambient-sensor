//
// Created by Kok on 7/12/25.
//

#include "gpios.h"
#include "system_config.h"

void StartGPIOPeripheral() {
    GPIO_PeriClockControl(GPIOA, ENABLE);
    GPIO_PeriClockControl(GPIOB, ENABLE);
}

/**
 * @brief GPIOS: PB7 (SDA), PB8 (SCL)
 */
void InitI2cGPIOS() {
    GPIO_PinConfig_t config = {
        .GPIO_PinNumber = I2C_SDA_PIN,
        .GPIO_PinSpeed = GPIO_SpeedMedium,
        .GPIO_PinOPType = GPIO_OpTypeOD,
        .GPIO_PinPuPdControl = GPIO_Pu,
        .GPIO_PinMode = GPIO_ModeAlternate,
        .GPIO_PinAltFunMode = GPIO_AF4
    };

    // SDA pin
    system_handles.pGPIOHandle->pGPIOx = I2C_PORT;
    system_handles.pGPIOHandle->GPIO_PinConfig = config;
    GPIO_Init(system_handles.pGPIOHandle);

    // SCL pin
    system_handles.pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber = I2C_SCL_PIN;
    GPIO_Init(system_handles.pGPIOHandle);
}

/**
 * @brief GPIOS: PA1 (Main battery), PA2 (Contrast)
 */
void InitAdcGPIOS() {
    GPIO_PinConfig_t config = {
        .GPIO_PinNumber = ADC_BATTERY_PIN,
        .GPIO_PinSpeed = GPIO_SpeedMedium,
        .GPIO_PinOPType = GPIO_OpTypePP,
        .GPIO_PinPuPdControl = GPIO_NoPuPd,
        .GPIO_PinMode = GPIO_ModeAnalog,
    };

    // Battery pin
    system_handles.pGPIOHandle->pGPIOx = ADC_PORT;
    system_handles.pGPIOHandle->GPIO_PinConfig = config;
    GPIO_Init(system_handles.pGPIOHandle);

    // Contrast pin
    system_handles.pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber = ADC_CONTRAST_PIN;
    GPIO_Init(system_handles.pGPIOHandle);
}

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

/**
 * @brief GPIO: PA8
 */
void InitErrorLedGPIOS() {
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