//
// Created by Kok on 6/12/25.
//

#ifndef SPI_DRIVER_H
#define SPI_DRIVER_H

#include "stm32f4xx.h"

/**
 * @brief Enables the SPI peripheral clock
 * @param perIndex The index of the specific SPI perifpheral (SPI1, SPI2, SPI3, etc.)
 * @param offset The offset of the enable register specific to each SPI
 */
#define SPI_PCLK_EN(perIndex, offset)        (\
                                                perIndex == 1 || perIndex == 4 ? RCC->APB2ENR |= (1 << offset) :\
                                                perIndex == 2 || perIndex == 3 ? RCC->APB1ENR |= (1 << offset) :\
                                                (void)0\
                                             )

/**
 * @brief Disables the SPI peripheral clock
 * @param perIndex The index of the specific SPI perifpheral (SPI1, SPI2, SPI3, etc.)
 * @param offset The offset of the enable register specific to each SPI
 */
#define SPI_PCLK_DI(perIndex, offset)           (\
                                                    perIndex == 1 || perIndex == 4 ? RCC->APB2ENR &=~ (1U << offset) :\
                                                    perIndex == 2 || perIndex == 3 ? RCC->APB1ENR &=~ (1U << offset) :\
                                                    (void)0\
                                                 )

/**
 * @brief Resets the SPI peripheral
 * @param perIndex The index of the specific SPI perifpheral (SPI1, SPI2, SPI3, etc.)
 * @param offset The offset of the reset register specific to each SPI peripheral
 */
#define SPI_REG_RESET(perIndex, offset)        do {                                               \
                                                if (perIndex == 1 || perIndex == 4) {           \
                                                    RCC->APB2RSTR |= (1 << offset);             \
                                                    RCC->APB2RSTR &=~ (1 << offset);            \
                                                } else if (perIndex == 2 || perIndex == 3) {    \
                                                    RCC->APB1RSTR |= (1 << offset);             \
                                                    RCC->APB1RSTR &=~ (1 << offset);            \
                                                }                                               \
                                             } while(0)

#define SPI_INDEX_TO_IRQ_NUMBER(index) ((index == 1) ? SPI1_IRQn :\
                                        (index == 2) ? SPI2_IRQn :\
                                        (index == 3) ? SPI3_IRQn :\
                                        (index == 4) ? SPI4_IRQn :\
                                        SPI1_IRQn)

/**
 * @SPI_MODES
 */
#define SPI_DEVICE_MODE_MASTER              1
#define SPI_DEVICE_MODE_SLAVE               0

/**
 * @SPI_BUS_CONFIGURATIONS
 */
#define SPI_BUS_CFG_FULL_DUPLEX             0
#define SPI_BUS_CFG_FULL_DUPLEX_RXONLY      1
#define SPI_BUS_CFG_HALF_DUPLEX             2
#define SPI_BUS_CFG_SIMPLEX_TXONLY          3
#define SPI_BUS_CFG_SIMPLEX_RXONLY          4

/**
 * @SPI_SPEEDs
 */
#define SPI_SCLK_SPEED_DIV2                 0
#define SPI_SCLK_SPEED_DIV4                 1
#define SPI_SCLK_SPEED_DIV8                 2
#define SPI_SCLK_SPEED_DIV16                3
#define SPI_SCLK_SPEED_DIV32                4
#define SPI_SCLK_SPEED_DIV64                5
#define SPI_SCLK_SPEED_DIV128               6
#define SPI_SCLK_SPEED_DIV256               7

/**
 * @SPI_DATA_FRAMES
 */
#define SPI_DF_8BITS                       0
#define SPI_DF_16BITS                      1

/**
 * @SPI_FRAME_FORMATS
 */
#define SPI_FF_MOTOROLA                     0
#define SPI_FF_TI                           1   // Texas Instruments

/**
 * @SPI_BITS_ORDERS
 */
#define SPI_BO_MSBFIRST                     0
#define SPI_BO_LSBFIRST                     1

/**
 * @CLOCK_POLARITIES
 */
#define SPI_CPOL_HIGH                       1
#define SPI_CPOL_LOW                        0

/**
 * @CLOCK_PHASE
 */
#define SPI_CPHA_2EDGE                       1
#define SPI_CPHA_1EDGE                       0

/**
 * @SPI_SLAVE_SELECT_MANAGEMENT
 */
#define SPI_SSM_HW                          0
#define SPI_SSM_SW                          1

/**
 * @SPI_SSOE
 */
#define SPI_SS_OUTPUT_DISABLED              0
#define SPI_SS_OUTPUT_ENABLED               1

/**
 * @SPI_SS_ACTIVE_LEVEL
 */
#define SPI_SS_LOW                          0
#define SPI_SS_HIGH                         1

/**
 * @SPI_STATE
 */
#define SPI_STATE_READY                     0
#define SPI_STATE_BUSY_TX                   1
#define SPI_STATE_BUSY_RX                   2

/**
 * @SPI_EVENT
 */
#define SPI_EVENT_TX_COMPLETE               0
#define SPI_EVENT_RX_COMPLETE               1
#define SPI_EVENT_OVR_ERR_COMPLETE          2

typedef struct {
    uint8_t SPI_DeviceMode;                 /** Possible values from @SPI_MODES */
    uint8_t SPI_BusConfig;                  /** Possible values from @SPI_BUS_CONFIGURATIONS */
    uint8_t SPI_SclkSpeed;                  /** Possible values from @SPI_SPEEDs */
    uint8_t SPI_DF;                         /** Possible values from @SPI_DATA_FRAMES */
    uint8_t SPI_FrameFormat;                /** Possible values from @SPI_FRAME_FORMATS */
    uint8_t SPI_BitsOrder;                  /** Possible values from @SPI_BITS_ORDERS */
    uint8_t SPI_CPOL;                       /** Possible values from @CLOCK_POLARITIES */
    uint8_t SPI_CPHA;                       /** Possible values from @CLOCK_PHASE */
    uint8_t SPI_SSM;                        /** Possible values from @SPI_SLAVE_SELECT_MANAGEMENT */
    uint8_t SPI_SS_OutputEnabled;           /** Possible values from @SPI_SSOE */
    uint8_t SPI_SS_ActiveLevel;             /** Possible values from @SPI_SS_ACTIVE_LEVEL */
} SPI_Config_t;

typedef struct {
    SPI_TypeDef *pSPIx;
    SPI_Config_t SPIConfig;
    uint8_t *pTXBuffer;
    uint8_t *pRXBuffer;
    uint32_t TXLen;
    uint32_t RXLen;
    uint8_t TXState;                        /** Possible values from @SPI_STATE */
    uint8_t RXState;                        /** Possible values from @SPI_STATE */
} SPI_Handle_t;

/*
 * Peripheral Clock Setup
 */
void SPI_PeriClockControl(SPI_TypeDef *pSPIx, uint8_t Enabled);

/*
 * Init and De-Init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_TypeDef *pSPIx);

/*
 * Data send and receive
 */
uint8_t SPI_SendData(SPI_Handle_t *pSPIHandle, uint8_t *pTXBuffer, uint32_t len);
uint8_t SPI_ReceiveData(SPI_Handle_t *pSPIHandle, uint8_t *pRXBuffer, uint32_t len);
uint8_t SPI_SendReceiveData(SPI_Handle_t *pSPIHandle, uint8_t *pTXBuffer, uint8_t *pRXBuffer, uint32_t len);

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTXBuffer, uint32_t len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRXBuffer, uint32_t len);

/*
 * NSS Control
 */
void SPI_SlaveSelect(SPI_Handle_t *pSPIHandle, GPIO_TypeDef *pGPIOx, uint8_t pinNumber);
void SPI_SlaveDeSelect(SPI_Handle_t *pSPIHandle, GPIO_TypeDef *pGPIOx, uint8_t pinNumber);

/*
 * Other controls
 */
uint8_t SPI_PeripheralEnabled(SPI_TypeDef *pSPIx);
void SPI_PeripheralControl(SPI_Handle_t *pSPIHandle, uint8_t enabled);
void SPI_PeripheralForceDisable(SPI_Handle_t *pSPIHandle);
void SPI_BidirectionalModeDirection(SPI_TypeDef *pSPIx, uint8_t enableTx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);
void SPI_ClearOVRFlag(SPI_Handle_t *pSPIHandle);

/*
 * IRQ Configuration
 */
void SPI_IRQConfig(uint8_t PerIndex, uint8_t IRQPriority, uint8_t Enabled);
void IRQ_Handling(SPI_Handle_t *pSPIHandle);

/*
 * Application callback
 */
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEvent);

#endif //SPI_DRIVER_H
