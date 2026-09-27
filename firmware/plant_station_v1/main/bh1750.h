#ifndef BH1750_SENSOR_H
#define BH1750_SENSOR_H

#include "driver/i2c_master.h"
#include "esp_err.h"

typedef struct 
{
    i2c_master_dev_handle_t dev_handle;

    float lux;

} bh1750_t;

esp_err_t init_lux_sensor
(
    i2c_master_bus_handle_t bus_handle, 
    uint8_t sensor_addr, 
    uint32_t clock_frequency, 
    bh1750_t *sensor
);

esp_err_t bh1750_read_raw(
    uint8_t *data,
    size_t data_len,
    bh1750_t *bh1750
);

esp_err_t bh1750_read_measurement(
    bh1750_t *bh1750
);


#endif 