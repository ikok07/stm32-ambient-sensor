
#include <adc.h>
#include <error.h>
#include <i2c.h>
#include <string.h>

#include "power_driver.h"
#include "clock_driver.h"
#include "gpio_driver.h"
#include "usart_driver.h"
#include "i2c_driver.h"
#include "adc_driver.h"
#include "bme280_i2c_driver.h"
#include "ssd1306_i2c_driver.h"
#include "ssd1306.h"
#include "timer_driver.h"
#include "timer.h"
#include "user_actions.h"
#include "system_config.h"
#include "usart.h"
#include "bme280.h"
#include "generic_methods.h"

GPIO_Handle_t gpioHandle;
USART_Handle_t usartHandle;
I2C_Handle_t i2cHandle;
BME280_Handle_t bme280Handle;
ADC_Handle_t adcHandle;
SSD1306_Handle_t displayHandle;
TIM_Handle_t timerHandle;

SYSTEM_Handles_t system_handles = {
    .pGPIOHandle = &gpioHandle,
    .pUSARTHandle = &usartHandle,
    .pI2CHandle = &i2cHandle,
    .pBme280Handle = &bme280Handle,
    .pADCHandle = &adcHandle,
    .pSSD1306Handle = &displayHandle,
    .pTimerHandle = &timerHandle
};

PWR_Handle_t pwrHandle = {
    .Config = {
        .BackupRegulatorEnabled = ENABLE,
        .RegulatorVoltageScaling = PWR_RegulatorScale3,
        .PVDEnabled = DISABLE,
        .LowPowerRegulatorInStopMode = ENABLE,
        .LowPowerRegulatorLowVoltageEnabled = ENABLE,
        .FlashPowerDownInStopMode = ENABLE,
        .StandbyModeInDeepSleepEnabled = ENABLE,        // Use standby mode
        .WakeUpConfig = {
            .WKUPPinEnabled = DISABLE,
            .Source = PWR_WakeUpSrcInterrupt
        }
    }
};

volatile uint8_t displayUpdating = 0;

void UpdateState() {
    if (displayUpdating) return;
    displayUpdating = 1;
    BME280_Result_t sensorResult;
    BME280_Error_t sensorErr = BME280_GetSample(&bme280Handle, &sensorResult);
    if (sensorErr != BME280_ErrOK) {
        TriggerError("Failed to get sensor sample!");
        while (1);
    }

    SSD1306_Error_e displayErr = SetDisplayContrast((GetContrastPercentage() * 255) / 100);
    if (displayErr != SSD1306_ErrOK) {
        TriggerError("Contrast could not be set!");
        while (1);
    }

    displayErr = ShowMeasurements(sensorResult.Temperature, sensorResult.Pressure, sensorResult.Humidity);
    if (displayErr != SSD1306_ErrOK) {
        TriggerError("Temperature could not be showed!");
        while (1);
    }

    displayUpdating = 0;
}

/*
 * Setup the MCU clocks and power options
 */
void SystemInit() {
    // Select HSI Clock
    CLOCK_Enable(CLOCK_SrcHSI);
    CLOCK_SelectSysClock(CLOCK_SysClockHSI);

    // Set HCLK and APBs to 8MHz
    CLOCK_SetAHBBusPrescaler(CLOCK_AHBPresc2);

    // Enable FPU
    SCB->CPACR |= 0xF << 20;

    // Start SysTick
    Generic_InitSysTick();

    PWR_PeriClockControl(ENABLE);
    PWR_UnlockBackupRegisters();
    PWR_Error_e err = PWR_Init(&pwrHandle);
    // Other peripherals are still not setup, therefore the CPU is just halted when error occurs.
    if (err != PWR_ErrOK) while(1);
}

volatile uint8_t updateToggle = DISABLE;

int main(void) {
    // Start and initialize GPIOs
    GPIO_PeriClockControl(GPIOA, ENABLE);       // Enable GPIOA Clock
    GPIO_PeriClockControl(GPIOB, ENABLE);       // Enable GPIOB Clock

    InitErrorLED();                     // Configure error LED GPIO
    ConfigureUSART_GPIOS();             // Configure USART GPIOs
    ConfigureADC_GPIOS();               // Configure ADC GPIOs
    ConfigureI2C_GPIOS();               // Configure I2C GPIOs
    ConfigureDisplay_GPIOS();           // Configure Display GPIOs
    InitUserBtnsGPIOS();                // Configure user button GPIOs

    // Configure USART
    USART_Error_e usartErr = InitUSART();
    if (usartErr != USART_ErrOK) {
        TriggerError("USART could not be started!\n");
        goto infinite_loop;
    }

    // Configure ADC
    ADC_Error_e adcError = InitADC();
    if (adcError != ADC_ErrOK) {
        TriggerError("ADC could not be started!\n");
        goto infinite_loop;
    }

    // Configure I2C
    I2C_Error_e i2cError = InitI2C();
    if (i2cError != I2C_ErrOK) {
        TriggerError("I2C could not be started!\n");
        goto infinite_loop;
    }
    StartI2C();

    // Configure sensor
    BME280_Error_t sensorErr = InitBME280Sensor();
    if (sensorErr != BME280_ErrOK) {
        TriggerError("BME280 sensor could not be started!\n");
        goto infinite_loop;
    }

    // Configure display
    SSD1306_Error_e displayErr = InitDisplay(GetContrastPercentage() * 255);
    if (displayErr != SSD1306_ErrOK) {
        TriggerError("SSD1306 display could not be started!\n");
        goto infinite_loop;
    }

    displayErr = DisplayControl(ENABLE);
    if (displayErr != SSD1306_ErrOK) {
        TriggerError("SSD1306 display could not be enabled!\n");
        goto infinite_loop;
    }

    // Update with latest data
    UpdateState();

    // Start timer
    InitTimer();
    TimerControl(ENABLE);

    infinite_loop:
    while (1) {
        if (updateToggle) {
            updateToggle = DISABLE;

            // Disable timer
            TimerControl(DISABLE);

            // Update with latest data
            UpdateState();

            // Disable SysTick
            SYSTICK_CounterControl(DISABLE);

            // Enable timer
            TimerControl(ENABLE);

            // Enter sleep mode
            PWR_EnterSleepMode(&pwrHandle);

            // Enable SysTick
            SYSTICK_CounterControl(ENABLE);
        }
    };
}

void TIM2_IRQHandler() {
    TIM_ClearStatusFlag(&timerHandle, TIM_FlagUpdate);
    if (displayUpdating) return;
    updateToggle = ENABLE;
}

void EXTI3_IRQHandler() {
    GPIO_IRQHandling(USER_ACTION_1_PIN);
    if (displayUpdating) return;
    RotateMeasurements();
    updateToggle = ENABLE;
}

void DMA2_Stream4_IRQHandler() {
    DMA_IRQHanding(&system_handles.pADCHandle->DMAState.DMAHandle);
}

void DMA_ApplicationCallback(DMA_Handle_t *pDMAHandle, DMA_Flag_e Flag) {
    if (Flag == DMA_FlagTransferError) {
        TriggerError("DMA data transfer failed!\n");
        while (1);
    }

    if (Flag == DMA_FlagDirectModeError) {
        TriggerError("DMA direct mode error!\n");
        while (1);
    }

    if (Flag == DMA_FlagFIFOOverrun) {
        TriggerError("DMA FIFO overrun error!\n");
        while (1);
    }
}