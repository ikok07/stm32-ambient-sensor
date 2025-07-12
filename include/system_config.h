//
// Created by Kok on 7/12/25.
//

#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

#include "gpio_driver.h"
#include "usart_driver.h"
#include "i2c_driver.h"
#include "bme280_i2c_driver.h"

typedef struct {
    GPIO_Handle_t *pGPIOHandle;
    USART_Handle_t *pUSARTHandle;
    I2C_Handle_t *pI2CHandle;
    BME280_Handle_t *pBme280Handle;
} SYSTEM_Handles_t;

extern SYSTEM_Handles_t system_handles;

#endif //SYSTEM_CONFIG_H
