#include "sht41.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


/* SHT41 commands */
#define SHT41_CMD_MEASURE_HIGH_PRECISION    0xFD


/* SHT41 measurement response length */
#define SHT41_MEASUREMENT_RESPONSE_SIZE     6


/* SHT41 measurement time */
#define SHT41_MEASUREMENT_DELAY_MS          10


/**
 * @brief Calculate SHT41 CRC-8.
 *
 * Polynomial: 0x31
 * Initialization value: 0xFF
 */
static uint8_t sht41_crc8(
    const uint8_t *data,
    size_t len)
{
    uint8_t crc = 0xFF;

    for (size_t i = 0; i < len; i++)
    {
        crc ^= data[i];

        for (uint8_t bit = 0; bit < 8; bit++)
        {
            if (crc & 0x80)
            {
                crc = (crc << 1) ^ 0x31;
            }
            else
            {
                crc <<= 1;
            }
        }
    }

    return crc;
}


/**
 * @brief Initialize SHT41 I2C device.
 */
esp_err_t init_hum_temp_sensor(
    i2c_master_bus_handle_t bus_handle,
    uint8_t sensor_addr,
    uint32_t clock_frequency,
    sht41_t *sensor)
{
    if (sensor == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = sensor_addr,
        .scl_speed_hz = clock_frequency,
    };

    return i2c_master_bus_add_device(
        bus_handle,
        &dev_cfg,
        &sensor->dev_handle
    );
}


/**
 * @brief Send measurement command and receive six raw bytes.
 */
esp_err_t sht41_read_raw(
    uint8_t *data,
    size_t data_len,
    sht41_t *sht41)
{
    if (data == NULL || sht41 == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (data_len < SHT41_MEASUREMENT_RESPONSE_SIZE)
    {
        return ESP_ERR_INVALID_SIZE;
    }


    /*
     * The SHT41 does not use a register-address
     * style transaction here.
     *
     * 0xFD is a command that tells the sensor:
     *
     * "Perform a high-precision measurement."
     */
    uint8_t cmd = SHT41_CMD_MEASURE_HIGH_PRECISION;


    /*
     * Step 1:
     * Send the measurement command.
     */
    esp_err_t err = i2c_master_transmit(
        sht41->dev_handle,
        &cmd,
        1,
        -1
    );

    if (err != ESP_OK)
    {
        return err;
    }


    /*
     * Step 2:
     * Give the SHT41 time to perform
     * the measurement.
     */
    vTaskDelay(pdMS_TO_TICKS(SHT41_MEASUREMENT_DELAY_MS));


    /*
     * Step 3:
     * Read the six-byte response.
     */
    err = i2c_master_receive(
        sht41->dev_handle,
        data,
        SHT41_MEASUREMENT_RESPONSE_SIZE,
        -1
    );

    if (err != ESP_OK)
    {
        return err;
    }


    /*
     * Verify temperature CRC.
     *
     * data[0] = Temperature MSB
     * data[1] = Temperature LSB
     * data[2] = Temperature CRC
     */
    uint8_t temperature_crc = sht41_crc8(
        &data[0],
        2
    );

    if (temperature_crc != data[2])
    {
        return ESP_ERR_INVALID_CRC;
    }


    /*
     * Verify humidity CRC.
     *
     * data[3] = Humidity MSB
     * data[4] = Humidity LSB
     * data[5] = Humidity CRC
     */
    uint8_t humidity_crc = sht41_crc8(
        &data[3],
        2
    );

    if (humidity_crc != data[5])
    {
        return ESP_ERR_INVALID_CRC;
    }


    return ESP_OK;
}


/**
 * @brief Read SHT41 and convert raw values into
 * temperature and relative humidity.
 */
esp_err_t sht41_read_measurement(
    sht41_t *sht41)
{
    if (sht41 == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }


    /*
     * SHT41 returns six bytes.
     */
    uint8_t data[SHT41_MEASUREMENT_RESPONSE_SIZE];


    /*
     * Get raw data and validate CRC.
     */
    esp_err_t err = sht41_read_raw(
        data,
        sizeof(data),
        sht41
    );

    if (err != ESP_OK)
    {
        return err;
    }


    /*
     * Combine temperature MSB and LSB
     * into a single 16-bit value.
     *
     * Example:
     *
     * data[0] = 0x66
     * data[1] = 0x42
     *
     * raw_temperature = 0x6642
     */
    uint16_t raw_temperature =
        ((uint16_t)data[0] << 8) |
        data[1];


    /*
     * Combine humidity MSB and LSB.
     */
    uint16_t raw_humidity =
        ((uint16_t)data[3] << 8) |
        data[4];


    /*
     * Convert raw temperature to Celsius.
     *
     * SHT41 transfer function:
     *
     * T = -45 + 175 * raw / 65535
     */
    sht41->temperature =
        -45.0f +
        175.0f *
        ((float)raw_temperature / 65535.0f);


    /*
     * Convert raw humidity to %RH.
     *
     * SHT41 transfer function:
     *
     * RH = -6 + 125 * raw / 65535
     */
    sht41->humidity =
        -6.0f +
        125.0f *
        ((float)raw_humidity / 65535.0f);


    /*
     * Limit humidity to a physically meaningful
     * range.
     */
    if (sht41->humidity > 100.0f)
    {
        sht41->humidity = 100.0f;
    }

    if (sht41->humidity < 0.0f)
    {
        sht41->humidity = 0.0f;
    }


    return ESP_OK;
}