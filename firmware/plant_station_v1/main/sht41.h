#ifndef SHT41_SENSOR_H
#define SHT41_SENSOR_H

#include "driver/i2c_master.h"
#include "esp_err.h"

typedef struct
{
    i2c_master_dev_handle_t dev_handle;

    float humidity;
    float temperature;

} sht41_t;


/**
 * @brief Initialize the SHT41 I2C device.
 *
 * @param bus_handle I2C master bus handle
 * @param sensor_addr 7-bit SHT41 I2C address
 * @param frequency I2C clock frequency
 * @param sensor Pointer to SHT41 sensor structure
 *
 * @return ESP_OK on success
 */
esp_err_t init_hum_temp_sensor(
    i2c_master_bus_handle_t bus_handle,
    uint8_t sensor_addr,
    uint32_t frequency,
    sht41_t *sensor
);


/**
 * @brief Read the six raw bytes returned by the SHT41.
 *
 * The six bytes are:
 *
 * [0] Temperature MSB
 * [1] Temperature LSB
 * [2] Temperature CRC
 * [3] Humidity MSB
 * [4] Humidity LSB
 * [5] Humidity CRC
 *
 * @param data Buffer where the six bytes will be stored
 * @param data_len Size of data buffer
 * @param sht41 Pointer to SHT41 sensor structure
 *
 * @return ESP_OK on success
 */
esp_err_t sht41_read_raw(
    uint8_t *data,
    size_t data_len,
    sht41_t *sht41
);


/**
 * @brief Read and convert a complete SHT41 measurement.
 *
 * Updates:
 *     sht41->temperature
 *     sht41->humidity
 *
 * @param sht41 Pointer to SHT41 sensor structure
 *
 * @return ESP_OK on success
 */
esp_err_t sht41_read_measurement(
    sht41_t *sht41
);

#endif