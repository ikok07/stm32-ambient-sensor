//
// Created by Kok on 7/12/25.
//

#ifndef I2C_H
#define I2C_H

#include "i2c_driver.h"

#define I2Cx                    I2C1
#define I2C_PORT                GPIOB
#define I2C_SDA_PIN             7
#define I2C_SCL_PIN             8

void ConfigureI2C_GPIOS();
I2C_Error_e InitI2C();

void StartI2C();
void StopI2C();

#endif //I2C_H
