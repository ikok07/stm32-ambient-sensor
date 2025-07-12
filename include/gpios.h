//
// Created by Kok on 7/12/25.
//

#ifndef GPIOS_H
#define GPIOS_H

#define ADC_PORT                GPIOA
#define ADC_BATTERY_PIN         1
#define ADC_CONTRAST_PIN        2

#define USER_BTNS_PORT          GPIOA
#define USER_BTN_1_PIN          3
#define USER_BTN_2_PIN          4

void InitAdcGPIOS();
void InitUserBtnsGPIOS();

#endif //GPIOS_H
