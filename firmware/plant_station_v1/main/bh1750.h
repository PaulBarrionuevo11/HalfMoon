#ifndef BH1750_SENSOR_H
#define BH1750_SENSOR_H

#include "driver/i2c_master.h"

typedef struct 
{
    i2c_master_dev_handle_t dev_handle;

    float lux;

} bh1750_t;

esp_err_t init_lux_sensor(
    i2c_master_bus_handle_t bus_handle, 
    uint8_t sensor_addr, 
    uint32_t frequency, 
    bh1750_t *sensor);

#endif 