//
// Created by Kok on 7/12/25.
//

#ifndef GPIOS_H
#define GPIOS_H

#define I2C_PORT                GPIOB
#define I2C_SDA_PIN             7
#define I2C_SCL_PIN             8

#define ADC_PORT                GPIOA
#define ADC_BATTERY_PIN         1
#define ADC_CONTRAST_PIN        2

#define USER_BTNS_PORT          GPIOA
#define USER_BTN_1_PIN          3
#define USER_BTN_2_PIN          4

#define ERR_LED_PORT            GPIOA
#define ERR_LED_PIN             8

void StartGPIOPeripheral();

void InitI2cGPIOS();
void InitAdcGPIOS();
void InitUserBtnsGPIOS();
void InitErrorLedGPIOS();


#endif //GPIOS_H
