#include "bh1750.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define BH1750_CMD_MEASURE_HIGH_RESOLUTION       (0x10) //0001_0000 

#define BH1750_MEASUREMENT_RESPONSE_SIZE            (2)

// Measurement Time is typically 120ms. 
#define BH1750_MEASUREMENT_DELAY_MS               (120)


esp_err_t init_lux_sensor(
    i2c_master_bus_handle_t bus_handle, 
    uint8_t sensor_addr, 
    uint32_t clock_frequency, 
    bh1750_t *sensor) 
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
    
    // This dynamically updates the dev_handle inside your specific struct instance
    return i2c_master_bus_add_device(
        bus_handle, 
        &dev_cfg,
        &sensor->dev_handle
    );
}

esp_err_t bh1750_read_raw(
    uint8_t *data,
    size_t data_len,
    bh1750_t *bh1750)
{
    if (data == NULL || bh1750 == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (data_len < BH1750_MEASUREMENT_RESPONSE_SIZE)
    {
        return ESP_ERR_INVALID_SIZE;
    }

    // Perform high precision measurement

    uint8_t cmd = BH1750_CMD_MEASURE_HIGH_RESOLUTION;

     /*
     * Step 1:
     * Send the measurement command.
     */
    esp_err_t err = i2c_master_transmit(
        bh1750->dev_handle,
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
     * Give the BH1750 time to perform
     * the measurement.
     */
    vTaskDelay(pdMS_TO_TICKS(BH1750_MEASUREMENT_DELAY_MS));

    /*
     * Step 3:
     * Read the six-byte response.
     */
    err = i2c_master_receive(
        bh1750->dev_handle,
        data,
        BH1750_MEASUREMENT_RESPONSE_SIZE,
        -1
    );

    if (err != ESP_OK)
    {
        return err;
    }
    return ESP_OK;
}

esp_err_t bh1750_read_measurement(bh1750_t *bh1750)

{
    if (bh1750 == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }


    /*
     * SHT41 returns six bytes.
     */
    uint8_t data[BH1750_MEASUREMENT_RESPONSE_SIZE];

     /*
     * Get raw data and validate.
     */
    esp_err_t err = bh1750_read_raw(
        data,
        sizeof(data),
        bh1750
    );

    if (err != ESP_OK)
    {
        return err;
    }

    /*
     * Combine MSB and LSB into a
     * single 16-bit raw value.
     * data[0] → MSB and data[1] → LSB
     */
    uint16_t raw_lux =
        ((uint16_t)data[0] << 8) |
        data[1];

    /*
     * Convert raw BH1750 measurement
     * to lux.
     */
    bh1750->lux =
        (float)raw_lux / 1.2f;

    return ESP_OK;
}