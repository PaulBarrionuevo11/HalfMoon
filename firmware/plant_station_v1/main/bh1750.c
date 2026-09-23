#include "bh1750.h"

esp_err_t init_lux_sensor(i2c_master_bus_handle_t bus_handle, uint8_t sensor_addr, 
    uint32_t frequency, bh1750_t *sensor) 
{
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = sensor_addr,
        .scl_speed_hz = frequency,
    };
    
    // This dynamically updates the dev_handle inside your specific struct instance
    return i2c_master_bus_add_device(bus_handle, &dev_cfg, &(sensor->dev_handle));
}