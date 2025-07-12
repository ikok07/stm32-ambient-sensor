//
// Created by Kok on 7/12/25.
//

#include "bme280.h"

#include "system_config.h"

/**
 * @brief Configures the BME280 sensor
 */
BME280_Error_t InitBME280Sensor() {
    BME280_Config_t config = {
        .AddrPin = BME280_AddrPinLOW,
        .FilterCoeff = BME280_FilterCoeffOFF,
        .NormalModeStandbyDuration = BME280_StandbyDuration100,
        .HumidityOversampling = BME280_Oversampling1,
        .TemperatureOversampling = BME280_Oversampling1,
        .PressureOversampling = BME280_Oversampling1
    };

    system_handles.pBme280Handle->pI2C_Handle = system_handles.pI2CHandle;
    system_handles.pBme280Handle->BME280_Config = config;

    BME280_Error_t err = BME280_Configure(system_handles.pBme280Handle);
    if (err != BME280_ErrOK) return err;
    if ((err = BME280_CheckDeviceID(system_handles.pBme280Handle)) != BME280_ErrOK) return err;

    if ((err = BME280_SetMode(system_handles.pBme280Handle, BME280_ModeNormal)) != BME280_ErrOK) return err;

    return BME280_ErrOK;
}
