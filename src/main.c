
#include <adc.h>
#include <error.h>
#include <i2c.h>
#include <string.h>

#include "power_driver.h"
#include "clock_driver.h"
#include "gpio_driver.h"
#include "timer_driver.h"
#include "rtc_driver.h"
#include "usart_driver.h"
#include "i2c_driver.h"
#include "adc_driver.h"
#include "bme280_i2c_driver.h"
#include "system_config.h"
#include "usart.h"
#include "bme280.h"
#include "generic_methods.h"

GPIO_Handle_t gpioHandle;
USART_Handle_t usartHandle;
I2C_Handle_t i2cHandle;
BME280_Handle_t bme280Handle;
ADC_Handle_t adcHandle;

SYSTEM_Handles_t system_handles = {
    .pGPIOHandle = &gpioHandle,
    .pUSARTHandle = &usartHandle,
    .pI2CHandle = &i2cHandle,
    .pBme280Handle = &bme280Handle,
    .pADCHandle = &adcHandle
};

/*
 * Setup the MCU clocks and power options
 */
void SystemInit() {
    // Select HSI Clock
    CLOCK_Enable(CLOCK_SrcHSI);
    CLOCK_SelectSysClock(CLOCK_SysClockHSI);

    // Set HCLK and APBs to 2MHz
    CLOCK_SetAHBBusPrescaler(CLOCK_AHBPresc8);

    // Enable FPU
    SCB->CPACR |= 0xF << 20;

    // Start SysTick
    Generic_InitSysTick();

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
    PWR_PeriClockControl(ENABLE);
    PWR_UnlockBackupRegisters();
    PWR_Error_e err = PWR_Init(&pwrHandle);
    // Other peripherals are still not setup, therefore the CPU is just halted when error occurs.
    if (err != PWR_ErrOK) while(1);
}

int main(void) {
    // Start and initialize GPIOs
    GPIO_PeriClockControl(GPIOA, ENABLE);       // Enable GPIOA Clock
    GPIO_PeriClockControl(GPIOB, ENABLE);       // Enable GPIOB Clock

    InitErrorLED();                     // Configure error LED GPIO
    ConfigureUSART_GPIOS();             // Configure USART GPIOs
    ConfigureADC_GPIOS();               // Configure ADC GPIOs
    ConfigureI2C_GPIOS();               // Configure I2C GPIOs
    // InitUserBtnsGPIOS();                // Configure user button GPIOs

    // Configure USART
    USART_Error_e usartErr = InitUSART();
    if (usartErr != USART_ErrOK) {
        TriggerError("USART could not be started!");
        goto infinite_loop;
    }

    // Configure ADC
    ADC_Error_e adcError = InitADC();
    if (adcError != ADC_ErrOK) {
        TriggerError("ADC could not be started!");
        goto infinite_loop;
    }

    while (1) {};

    // Configure I2C
    I2C_Error_e i2cError = InitI2C();
    if (i2cError != I2C_ErrOK) {
        TriggerError("I2C could not be started!");
        goto infinite_loop;
    }
    StartI2C();

    // Configure sensor
    BME280_Error_t sensorErr = InitBME280Sensor();
    if (sensorErr != BME280_ErrOK) {
        TriggerError("BME280 sensor could not be started!");
        goto infinite_loop;
    }

    infinite_loop:
    while (1);
}

void DMA2_Stream4_IRQHandler() {
    DMA_IRQHanding(&system_handles.pADCHandle->DMAState.DMAHandle);
}

void DMA_ApplicationCallback(DMA_Handle_t *pDMAHandle, DMA_Flag_e Flag) {
    if (Flag == DMA_FlagTransferError) {
        TriggerError("DMA data transfer failed!");
        while (1);
    }

    if (Flag == DMA_FlagDirectModeError) {
        TriggerError("DMA direct mode error!");
        while (1);
    }

    if (Flag == DMA_FlagFIFOOverrun) {
        TriggerError("DMA FIFO overrun error!");
        while (1);
    }
}