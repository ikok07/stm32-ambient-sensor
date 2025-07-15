//
// Created by Kok on 7/15/25.
//

#ifndef SSD1306_H
#define SSD1306_H

#include "ssd1306_i2c_driver.h"
#include "stm32f4xx.h"

#define SSD1306_PWR_CONTROL_PORT       GPIOA
#define SSD1306_PWR_CONTROL_PIN        7

#define SSD1306_TOTAL_MEASUREMENTS      3
#define SSD1306_DEFAULT_FONT           SSD1306_Font16x16

#define SSD1306_ROTATE_PERIOD_S         30

void ConfigureDisplay_GPIOS();

typedef enum {
    Display_MeasTemperature,
    Display_MeasPressure,
    Display_MeasHumidity,
} Display_Measurement_e;

SSD1306_Error_e InitDisplay(uint8_t initialContrast);
void DisplayPowerControl(uint8_t Enabled);
SSD1306_Error_e DisplayControl(uint8_t Enabled);
SSD1306_Error_e SetDisplayContrast(uint8_t Value);
SSD1306_Error_e ClearDisplay();
SSD1306_Error_e UpdateDisplay();

SSD1306_Error_e ShowMeasurements(float temperature, float pressure, float humidity);
void RotateMeasurements();

#endif //SSD1306_H
