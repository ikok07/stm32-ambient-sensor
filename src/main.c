
#include <string.h>

#include "power_driver.h"
#include "clock_driver.h"
#include "gpio_driver.h"
#include "timer_driver.h"
#include "rtc_driver.h"
#include "usart_driver.h"
#include "i2c_driver.h"
#include "system_config.h"
#include "gpios.h"
#include "usart.h"
#include "generic_methods.h"

GPIO_Handle_t gpioHandle;
USART_Handle_t usartHandle;
I2C_Handle_t i2cHandle;

SYSTEM_Handles_t system_handles = {
    .pGPIOHandle = &gpioHandle,
    .pUSARTHandle = &usartHandle,
    .pI2CHandle = &i2cHandle
};


/*
 * Setup the MCU clocks and power options
 */
void SystemInit() {
    // Select HSI Clock
    CLOCK_Enable(CLOCK_SrcHSI);
    CLOCK_SelectSysClock(CLOCK_SysClockHSI);

    // Set HCLK and APBs to 1MHz
    CLOCK_SetAHBBusPrescaler(CLOCK_AHBPresc16);

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
    // Start SysTick
    Generic_InitSysTick();

    // Start and initialize GPIOs
    StartGPIOPeripheral();              // Start GPIO clocks
    ConfigureUSART_GPIOS();             // Configure USART GPIOs
    InitI2cGPIOS();                     // Configure I2C GPIOs
    InitAdcGPIOS();                     // Configure ADC GPIOs
    InitUserBtnsGPIOS();                // Configure user button GPIOs
    InitErrorLedGPIOS();                // Configure error LED GPIO

}