//
// Created by Kok on 7/12/25.
//

#ifndef USART_H
#define USART_H

#include "usart_driver.h"

#define USART_PORT              GPIOA
#define USART_TX_PIN            9
#define USART_RX_PIN            10

void ConfigureUSART_GPIOS();
USART_Error_e InitUSART();

#endif //USART_H
