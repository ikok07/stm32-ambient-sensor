//
// Created by Kok on 7/15/25.
//

#ifndef TIMER_H
#define TIMER_H

#include "timer_driver.h"

#define TIMER_PERIPHERAL            TIM2

void InitTimer();
void TimerControl(uint8_t Enable);

#endif //TIMER_H
