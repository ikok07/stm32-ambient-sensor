//
// Created by Kok on 7/15/25.
//

#include "timer.h"

#include "system_config.h"
#include "clock_driver.h"

/**
 * @brief Initializes the timer which will trigger every 100ms in order to update the MCU
 */
void InitTimer() {
    TIM_Config_t config = {
        .Direction = TIM_DirectionUp,
        .PreloadEnabled = ENABLE,
        .OnePulseMode = DISABLE,
        .UpdateEventToggle = TIM_UpdateEventEnabled,
        .UpdateRequestSourceMode = TIM_UpdateReqSrcMode1
    };

    system_handles.pTimerHandle->pTIMx = TIMER_PERIPHERAL;
    system_handles.pTimerHandle->TIM_Config = config;

    // Initialize the timer
    TIM_PeriClockControl(system_handles.pTimerHandle->pTIMx, ENABLE);
    TIM_Init(system_handles.pTimerHandle);

    // Set auto-reload values
    uint32_t timPclk1 = CLOCK_GetApb1TimerHz();
    TIM_SetPrescaler(system_handles.pTimerHandle, (timPclk1 / 1000) - 1);  // 1 KHz
    TIM_SetAutoReload(system_handles.pTimerHandle, TIMER_UPDATE_PERIOD_MS - 1);         // Every 100ms

    // Enable interrupts
    TIM_EnableInterrupts(system_handles.pTimerHandle, TIM_IT_UPDATE);
    TIM_IRQEnable(system_handles.pTimerHandle, TIMER_IRQ_PRIORITY);
}

/**
 * @brief Enables or disables the timer
 * @param Enable If the timer should be started on stopped
 */
void TimerControl(uint8_t Enable) {
    if (Enable) {
        // Reset auto reload register
        TIM_GenerateUpdateEvent(system_handles.pTimerHandle);
        TIM_Start(system_handles.pTimerHandle);
    } else {
        TIM_Stop(system_handles.pTimerHandle);
    }
}
