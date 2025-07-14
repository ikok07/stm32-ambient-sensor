//
// Created by Kok on 7/12/25.
//

#ifndef BATTERIES_H
#define BATTERIES_H

#include "adc_driver.h"

#define ADC_PERI_PORT           GPIOA
#define EXT_BAT_ADC_PIN         1
#define CONTRAST_ADC_PIN        2

void ConfigureADC_GPIOS();
ADC_Error_e InitADC();

uint8_t GetBatteryPercentage();
uint8_t GetVBATPercentage();
uint8_t GetContrastPercentage();

#endif //BATTERIES_H
