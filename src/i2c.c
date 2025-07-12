//
// Created by Kok on 7/12/25.
//

#include "i2c.h"

#include "system_config.h"
#include "gpio_driver.h"

/**
 * @brief Initializes the required GPIOs for the I2C1 peripheral
 */
void ConfigureI2C_GPIOS() {
    GPIO_PinConfig_t config = {
        .GPIO_PinNumber = I2C_SDA_PIN,
        .GPIO_PinSpeed = GPIO_SpeedHigh,
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
 * @brief Configures the I2C peripheral
 */
I2C_Error_e InitI2C() {
    I2C_Config_t config = {
        .I2C_DeviceAddressLen = I2C_DeviceAddr7Bits,
        .I2C_SCLSpeed = I2C_SclSpeedSM,
        .I2C_FMDutyCycle = I2C_FmDuty2
    };

    system_handles.pI2CHandle->pI2Cx = I2Cx;
    system_handles.pI2CHandle->I2C_Config = config;

    I2C_PeriClockControl(system_handles.pI2CHandle->pI2Cx, ENABLE);
    I2C_Error_e err = I2C_Init(system_handles.pI2CHandle);
    if (err != I2C_ErrOK) return err;

    return I2C_ErrOK;
}

void StartI2C() {
    I2C_PeripheralControl(system_handles.pI2CHandle->pI2Cx, ENABLE);
}

void StopI2C() {
    I2C_PeripheralControl(system_handles.pI2CHandle->pI2Cx, DISABLE);
}
