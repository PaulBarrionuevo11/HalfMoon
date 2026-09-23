#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"

#include "sht41.hpp"

namespace {
constexpr char kTag[] = "app_main";

constexpr gpio_num_t SDA_GPIO = GPIO_NUM_21;
constexpr gpio_num_t SCL_GPIO = GPIO_NUM_22;
constexpr i2c_port_t I2C_PORT = I2C_NUM_0;
}  // namespace

extern "C" void app_main(void)
{
    // 1. Create the I2C master bus (shared across any devices on this bus)
    i2c_master_bus_config_t bus_cfg = {};
    bus_cfg.i2c_port = I2C_PORT;
    bus_cfg.sda_io_num = SDA_GPIO;
    bus_cfg.scl_io_num = SCL_GPIO;
    bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_cfg.glitch_ignore_cnt = 7;
    bus_cfg.flags.enable_internal_pullup = true;

    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus_handle));

    // 2. Attach the SHT41 to the bus
    SHT41 sensor;
    esp_err_t err = SHT41::Create(bus_handle, sensor);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "Failed to init SHT41: %s", esp_err_to_name(err));
        return;
    }

    uint32_t serial = 0;
    if (sensor.ReadSerial(serial) == ESP_OK) {
        ESP_LOGI(kTag, "SHT41 serial number: 0x%08lX", static_cast<unsigned long>(serial));
    }

    // 3. Poll every 2 seconds
    while (true) {
        float temperature_c = 0.0f;
        float humidity_rh = 0.0f;

        err = sensor.Read(SHT41::Precision::High, temperature_c, humidity_rh);
        if (err == ESP_OK) {
            ESP_LOGI(kTag, "Temp: %.2f C   RH: %.2f %%", temperature_c, humidity_rh);
        } else {
            ESP_LOGE(kTag, "Read failed: %s", esp_err_to_name(err));
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}