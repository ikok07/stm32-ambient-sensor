//
// Created by Kok on 7/12/25.
//

#include "adc.h"

#include <system_config.h>

/**
 * @brief Initializes the required GPIOs for the ADC1 peripheral
 */
void ConfigureADC_GPIOS() {
    GPIO_PinConfig_t config = {
        .GPIO_PinNumber = EXT_BAT_ADC_PIN,
        .GPIO_PinSpeed = GPIO_SpeedMedium,
        .GPIO_PinOPType = GPIO_OpTypePP,
        .GPIO_PinPuPdControl = GPIO_NoPuPd,
        .GPIO_PinMode = GPIO_ModeAnalog,
    };

    // Battery pin
    system_handles.pGPIOHandle->pGPIOx = ADC_PERI_PORT;
    system_handles.pGPIOHandle->GPIO_PinConfig = config;
    GPIO_Init(system_handles.pGPIOHandle);

    // Contrast pin
    system_handles.pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber = CONTRAST_ADC_PIN;
    GPIO_Init(system_handles.pGPIOHandle);
}

/**
 * @brief Initializes the ADC1 peripheral
 */
ADC_Error_e InitADC() {
    // DMA_Interrupt_e dmaInterrupts[3] = {DMA_InterruptTransferError, DMA_InterruptDirectModeError, DMA_InterruptFIFOOverrun};
    ADC_Config_t config = {
        .Prescaler = ADC_Prescaler2,
        .Resolution = ADC_Resolution12bit,
        .DataAlignment = ADC_DataAlignRight,
        .ContinuousModeEnabled = ENABLE,
        .ScanModeEnabled = ENABLE,
        .SampledRegularChannelsSequence = {ADC_Channel1, ADC_Channel2, ADC_Channel18},
        .SampledRegularChannelsSequenceLength = 3,
        .VBATEnabled = ENABLE,
        .WatchdogConfig = {
            .WatchdogChannel = ADC_WatchdogChan1,
            .WatchdogMode = ADC_WatchdogSingleChannel,
            .WatchdogHigherThreshold = 2048,
            .WatchdogLowerThreshold = 512
        },
        .DMAConfig = {
            .Enabled = ENABLE,
            // .EnabledInterrupts = dmaInterrupts,
            // .InterruptsLength = 3,
            // .InterruptsPriority = DMA_IRQ_PRIORITY
        }
    };

    system_handles.pADCHandle->Config = config;

    ADC_PeriClockControl(ENABLE);
    ADC_Error_e err = ADC_Init(system_handles.pADCHandle);
    if (err != ADC_ErrOK) return err;

    ADC_PeripheralControl(ENABLE);

    err = ADC_StartDMA(system_handles.pADCHandle);
    return err;
}

/**
 * @brief Reads the current VBAT percentage
 */
uint8_t GetVBATPercentage() {
    uint16_t adcValue = system_handles.pADCHandle->DMAState.Samples[1];
    return (adcValue * 100 * 4) / 4096;
}

/**
 * @brief Reads the current contrast potentiometer percentage
 */
uint8_t GetContrastPercentage() {
    uint16_t adcValue = system_handles.pADCHandle->DMAState.Samples[1];
    return (adcValue * 100) / 4096;
}
