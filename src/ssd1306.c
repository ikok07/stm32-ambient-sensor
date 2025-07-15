//
// Created by Kok on 7/15/25.
//

#include "ssd1306.h"

#include <stdio.h>
#include <string.h>

#include "system_config.h"
#include "gpio_driver.h"

static uint8_t temperatureImg[] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x87, 0xe0, 0x00, 0x01, 0x8e, 0x70, 0x03, 0x01, 0x8e, 0x70,
    0x03, 0xc0, 0x0e, 0x70, 0x00, 0x83, 0x8e, 0x70, 0x00, 0x1f, 0x8e, 0x70, 0x00, 0x38, 0x0e, 0x70,
    0x7e, 0x70, 0x0e, 0x70, 0x00, 0x30, 0x0e, 0x70, 0x00, 0x3c, 0x3c, 0x3c, 0x00, 0x0c, 0x70, 0x0e,
    0x03, 0xc0, 0x70, 0x0e, 0x03, 0x00, 0x3c, 0x3c, 0x00, 0x00, 0x0f, 0xf0, 0x00, 0x00, 0x00, 0x00
};

static uint8_t pressureImg[] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0x01, 0x80, 0x00,
    0x00, 0x01, 0x80, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0x01, 0x80, 0x00, 0x01, 0xc1, 0x83, 0x80,
    0x00, 0xf1, 0x8f, 0x00, 0x00, 0x3f, 0xfc, 0x00, 0x00, 0x0f, 0xf0, 0x00, 0x00, 0x03, 0xc0, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x03, 0xff, 0xff, 0xc0, 0x03, 0xff, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00
};

static uint8_t humidityImg[] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x70, 0x00, 0x00, 0xe0, 0xf8, 0x00,
    0x01, 0xf0, 0xdc, 0x00, 0x07, 0xf8, 0x07, 0x00, 0x0e, 0x1e, 0x03, 0xc0, 0x1c, 0x07, 0x00, 0xf0,
    0x1c, 0x07, 0x00, 0x38, 0x1e, 0x0e, 0x00, 0x38, 0x07, 0xfc, 0x00, 0x18, 0x00, 0x40, 0x00, 0x38,
    0x00, 0x00, 0x00, 0x70, 0x00, 0x1e, 0x03, 0xe0, 0x00, 0x07, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00
};

static volatile uint32_t updateCycles = 0;
static volatile uint8_t measurementsSequence[SSD1306_TOTAL_MEASUREMENTS] = {Display_MeasTemperature, Display_MeasPressure, Display_MeasHumidity};

static SSD1306_Error_e write_temperature(float temperature, uint8_t index);
static SSD1306_Error_e write_pressure(float pressure, uint8_t index);
SSD1306_Error_e write_humidity(float humidity, uint8_t index);

/**
 * @brief Initializes the required GPIOs for the display
 */
void ConfigureDisplay_GPIOS() {
    GPIO_PinConfig_t config = {
        .GPIO_PinNumber = SSD1306_PWR_CONTROL_PIN,
        .GPIO_PinMode = GPIO_ModeOutput,
        .GPIO_PinSpeed = GPIO_SpeedHigh,
        .GPIO_PinOPType = GPIO_OpTypePP,
        .GPIO_PinPuPdControl = GPIO_Pd
    };

    system_handles.pGPIOHandle->pGPIOx = SSD1306_PWR_CONTROL_PORT;
    system_handles.pGPIOHandle->GPIO_PinConfig = config;

    // Power control pin
    GPIO_Init(system_handles.pGPIOHandle);
}

/**
 * @brief Initializes the SSD1306 display
 * @param initialContrast The contrast to which the display will be initially set
 * @param rotatePeriod The period in seconds after which the showed measurement will rotate
 */
SSD1306_Error_e InitDisplay(uint8_t initialContrast) {
    SSD1306_Config_t config = {
        .AddressingMode = SSD1306_MemAddrHorizontal,
        .Contrast = initialContrast,
        .Font = SSD1306_DEFAULT_FONT,
        .DisplayOffset = 0,
        .DivideRatio = 0,
        .FontMirrored = DISABLE,
        .SegmentsRemapped = DISABLE,
        .DisplayStartLine = 0,
        .MUXRatio = 63,
        .OSCFreq = 0x08,
        .COMScanRemapped = DISABLE,
        .AlternativeCOMPinConfigEnabled = DISABLE,
        .COMLeftRightRemapEnabled = DISABLE
    };

    system_handles.pSSD1306Handle->PowerConfig.pGPIOx = SSD1306_PWR_CONTROL_PORT;
    system_handles.pSSD1306Handle->PowerConfig.PinNumber = SSD1306_PWR_CONTROL_PIN;
    system_handles.pSSD1306Handle->I2CHandle = system_handles.pI2CHandle;

    SSD1306_Error_e err = SSD1306_Init(system_handles.pSSD1306Handle, config);
    if (err != SSD1306_ErrOK) return err;

    if ((err = SSD1306_SetMemoryAddrMode(system_handles.pSSD1306Handle, SSD1306_MemAddrHorizontal)) != SSD1306_ErrOK) return err;
    if ((err = SSD1306_SetWriteAreaH(system_handles.pSSD1306Handle, SSD1306_MIN_WIDTH, SSD1306_MAX_WIDTH - 1, SSD1306_MIN_HEIGHT, SSD1306_MAX_HEIGHT)) != SSD1306_ErrOK) return err;

    return err;
}

/**
 * @brief Enables or disables the display power delivery
 * @param Enabled If the display power delivery should be enabled
 */
void DisplayPowerControl(uint8_t Enabled) {
    SSD1306_PowerControl(system_handles.pSSD1306Handle, Enabled);
}

/**
 * @brief Enables or disables the display
 * @param Enabled If the display should be enabled
 */
SSD1306_Error_e DisplayControl(uint8_t Enabled) {
    return SSD1306_DisplayControl(system_handles.pSSD1306Handle, Enabled);
}

/**
 * @brief Controls the display contrast
 * @param Value Value between 0 and 255
 */
SSD1306_Error_e SetDisplayContrast(uint8_t Value) {
    return SSD1306_SetContrast(system_handles.pSSD1306Handle, Value);
}

/**
 * @brief Clears the whole display frame buffer
 */
SSD1306_Error_e ClearDisplay() {
    return SSD1306_ClearH(system_handles.pSSD1306Handle);
}

/**
 * @brief Updates the display with the current frame buffer
 */
SSD1306_Error_e UpdateDisplay() {
    return SSD1306_UpdateH(system_handles.pSSD1306Handle);
}

/**
 * @brief Shows two measurements at the same time. The showed measurements rotate based on the configured rotate period
 * @param temperature Temperature value
 * @param pressure Pressure value
 * @param humidity Humidity value
 */
SSD1306_Error_e ShowMeasurements(float temperature, float pressure, float humidity) {
    SSD1306_Error_e err = SSD1306_SetWriteAreaH(system_handles.pSSD1306Handle, SSD1306_MIN_WIDTH, SSD1306_MAX_WIDTH - 1, SSD1306_MIN_HEIGHT, SSD1306_MAX_HEIGHT);;
    if (err != SSD1306_ErrOK) return err;

    if ((err = SSD1306_ClearH(system_handles.pSSD1306Handle)) != SSD1306_ErrOK) return err;

    // Write two measurements
    for (int i = 0; i < 2; i++) {
        switch (measurementsSequence[i]) {
            case Display_MeasTemperature:
                if ((err = write_temperature(temperature, i)) != SSD1306_ErrOK) return err;
            break;
            case Display_MeasPressure:
                if ((err = write_pressure(pressure, i)) != SSD1306_ErrOK) return err;
            break;
            case Display_MeasHumidity:
                if ((err = write_humidity(humidity, i)) != SSD1306_ErrOK) return err;
            break;
            default:
                if ((err = write_temperature(temperature, i)) != SSD1306_ErrOK) return err;
        }
    }

    // Update the screen
    if ((err = SSD1306_UpdateH(system_handles.pSSD1306Handle)) != SSD1306_ErrOK) return err;
    updateCycles++;

    // Check for rotation (temperature is always shown the first)
    if (updateCycles * TIMER_UPDATE_PERIOD_MS >= SSD1306_ROTATE_PERIOD_S * 1000) {
        Display_Measurement_e lastMeasurement = measurementsSequence[SSD1306_TOTAL_MEASUREMENTS - 1];
        measurementsSequence[SSD1306_TOTAL_MEASUREMENTS - 1] = measurementsSequence[1];
        measurementsSequence[1] = lastMeasurement;
        updateCycles = 0;
    }

    return err;
}

void RotateMeasurements() {
    Display_Measurement_e lastMeasurement = measurementsSequence[SSD1306_TOTAL_MEASUREMENTS - 1];
    measurementsSequence[SSD1306_TOTAL_MEASUREMENTS - 1] = measurementsSequence[1];
    measurementsSequence[1] = lastMeasurement;
    updateCycles = 0;
}

SSD1306_Error_e write_temperature(float temperature, uint8_t index) {
    const uint8_t IMG_WIDTH = 32;
    const uint8_t IMG_HEIGHT = 16;
    const uint8_t START_Y = index * IMG_HEIGHT;

    SSD1306_Error_e err = SSD1306_ErrOK;

    // Draw temperature symbol
    if ((err = SSD1306_DrawH(system_handles.pSSD1306Handle, 0, START_Y, IMG_WIDTH, IMG_HEIGHT, temperatureImg, sizeof(temperatureImg))) != SSD1306_ErrOK) return err;

    // Write temperature
    char str[10];
    uint8_t whole = (uint8_t)temperature;
    uint8_t fraction = (uint8_t)((temperature - whole) * 100);
    sprintf(str, "%d.%02d", whole, fraction);

    uint8_t degreesStartX = IMG_WIDTH + 5;
    uint8_t bigFontWidth = system_handles.pSSD1306Handle->FontConfig.Width;

    if ((err = SSD1306_WriteH(system_handles.pSSD1306Handle, degreesStartX, START_Y, str)) != SSD1306_ErrOK) return err;
    if ((err = SSD1306_SetFont(system_handles.pSSD1306Handle, SSD1306_Font8x16, DISABLE)) != SSD1306_ErrOK) return err;
    if ((err = SSD1306_WriteH(system_handles.pSSD1306Handle, degreesStartX + (strlen(str) * bigFontWidth) + 3, START_Y, "C")) != SSD1306_ErrOK) return err;
    if ((err = SSD1306_SetFont(system_handles.pSSD1306Handle, SSD1306_DEFAULT_FONT, DISABLE)) != SSD1306_ErrOK) return err;

    return err;
}

SSD1306_Error_e write_pressure(float pressure, uint8_t index) {
    const uint8_t IMG_WIDTH = 32;
    const uint8_t IMG_HEIGHT = 16;
    const uint8_t START_Y = index * IMG_HEIGHT;

    SSD1306_Error_e err = SSD1306_ErrOK;

    // Draw temperature symbol
    if ((err = SSD1306_DrawH(system_handles.pSSD1306Handle, 0, START_Y, IMG_WIDTH, IMG_HEIGHT, pressureImg, sizeof(pressureImg))) != SSD1306_ErrOK) return err;

    // Write pressure
    char str[10];
    uint16_t whole = (uint16_t)(pressure / 100);
    sprintf(str, "%d", whole);

    uint8_t degreesStartX = IMG_WIDTH + 5;
    uint8_t bigFontWidth = system_handles.pSSD1306Handle->FontConfig.Width;
    if ((err = SSD1306_WriteH(system_handles.pSSD1306Handle, degreesStartX, START_Y, str)) != SSD1306_ErrOK) return err;

    if ((err = SSD1306_SetFont(system_handles.pSSD1306Handle, SSD1306_Font8x16, DISABLE)) != SSD1306_ErrOK) return err;
    if ((err = SSD1306_WriteH(system_handles.pSSD1306Handle, degreesStartX + (strlen(str) * bigFontWidth) + 3, START_Y, "hPa")) != SSD1306_ErrOK) return err;
    if ((err = SSD1306_SetFont(system_handles.pSSD1306Handle, SSD1306_DEFAULT_FONT, DISABLE)) != SSD1306_ErrOK) return err;

    return err;
}

SSD1306_Error_e write_humidity(float humidity, uint8_t index) {
    const uint8_t IMG_WIDTH = 32;
    const uint8_t IMG_HEIGHT = 16;
    const uint8_t START_Y = index * IMG_HEIGHT;

    SSD1306_Error_e err = SSD1306_ErrOK;

    // Draw temperature symbol
    if ((err = SSD1306_DrawH(system_handles.pSSD1306Handle, 0, START_Y, IMG_WIDTH, IMG_HEIGHT, humidityImg, sizeof(humidityImg))) != SSD1306_ErrOK) return err;

    // Write temperature
    char str[10];
    uint8_t whole = (uint8_t)humidity;
    uint8_t fraction = (uint8_t)((humidity - whole) * 100);
    sprintf(str, "%d.%02d", whole, fraction);

    uint8_t degreesStartX = IMG_WIDTH + 5;
    uint8_t bigFontWidth = system_handles.pSSD1306Handle->FontConfig.Width;

    if ((err = SSD1306_WriteH(system_handles.pSSD1306Handle, degreesStartX, START_Y, str)) != SSD1306_ErrOK) return err;
    if ((err = SSD1306_SetFont(system_handles.pSSD1306Handle, SSD1306_Font8x16, DISABLE)) != SSD1306_ErrOK) return err;
    if ((err = SSD1306_WriteH(system_handles.pSSD1306Handle, degreesStartX + (strlen(str) * bigFontWidth) + 3, START_Y, "%")) != SSD1306_ErrOK) return err;
    if ((err = SSD1306_SetFont(system_handles.pSSD1306Handle, SSD1306_DEFAULT_FONT, DISABLE)) != SSD1306_ErrOK) return err;

    return err;
}