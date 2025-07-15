//
// Created by Kok on 7/12/25.
//

#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

#include "gpio_driver.h"
#include "usart_driver.h"
#include "i2c_driver.h"
#include "bme280_i2c_driver.h"
#include "adc_driver.h"
#include "ssd1306_i2c_driver.h"
#include "timer_driver.h"

#define DMA_IRQ_PRIORITY                1
#define TIMER_IRQ_PRIORITY              2
#define USER_ACTION_GPIOS_IRQ_PRIORITY  3

#define TIMER_UPDATE_PERIOD_MS          500

typedef struct {
    GPIO_Handle_t *pGPIOHandle;
    USART_Handle_t *pUSARTHandle;
    I2C_Handle_t *pI2CHandle;
    BME280_Handle_t *pBme280Handle;
    ADC_Handle_t *pADCHandle;
    SSD1306_Handle_t *pSSD1306Handle;
    TIM_Handle_t *pTimerHandle;
} SYSTEM_Handles_t;

extern SYSTEM_Handles_t system_handles;

#endif //SYSTEM_CONFIG_H
