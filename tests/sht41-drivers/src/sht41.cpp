#include "sht41.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {
constexpr char kTag[] = "sht41";
}

SHT41::SHT41(SHT41 &&other) noexcept : dev_handle_(other.dev_handle_)
{
    other.dev_handle_ = nullptr;
}

SHT41 &SHT41::operator=(SHT41 &&other) noexcept
{
    if (this != &other) {
        if (dev_handle_) {
            i2c_master_bus_rm_device(dev_handle_);
        }
        dev_handle_ = other.dev_handle_;
        other.dev_handle_ = nullptr;
    }
    return *this;
}

SHT41::~SHT41()
{
    if (dev_handle_) {
        i2c_master_bus_rm_device(dev_handle_);
    }
}

esp_err_t SHT41::Create(i2c_master_bus_handle_t bus_handle, SHT41 &out_sensor, uint8_t addr)
{
    if (!bus_handle) {
        return ESP_ERR_INVALID_ARG;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = addr,
        .scl_speed_hz    = 100000,
    };

    i2c_master_dev_handle_t handle = nullptr;
    esp_err_t err = i2c_master_bus_add_device(bus_handle, &dev_cfg, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "Failed to add device: %s", esp_err_to_name(err));
        return err;
    }

    out_sensor.dev_handle_ = handle;
    return ESP_OK;
}

uint8_t SHT41::Crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x31)
                                : static_cast<uint8_t>(crc << 1);
        }
    }
    return crc;
}

esp_err_t SHT41::PrecisionToCmd(Precision precision, uint8_t &cmd, uint32_t &delay_ms)
{
    switch (precision) {
        case Precision::High:   cmd = 0xFD; delay_ms = 10; return ESP_OK;
        case Precision::Medium: cmd = 0xF6; delay_ms = 5;  return ESP_OK;
        case Precision::Low:    cmd = 0xE0; delay_ms = 2;  return ESP_OK;
        default: return ESP_ERR_INVALID_ARG;
    }
}

esp_err_t SHT41::SoftReset()
{
    if (!dev_handle_) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t cmd = kCmdSoftReset;
    esp_err_t err = i2c_master_transmit(dev_handle_, &cmd, 1,
                                         kI2cTimeoutMs / portTICK_PERIOD_MS);
    if (err != ESP_OK) {
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(2));
    return ESP_OK;
}

esp_err_t SHT41::ReadSerial(uint32_t &serial)
{
    if (!dev_handle_) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t cmd = kCmdReadSerial;
    uint8_t rx[6] = {0};

    esp_err_t err = i2c_master_transmit(dev_handle_, &cmd, 1,
                                         kI2cTimeoutMs / portTICK_PERIOD_MS);
    if (err != ESP_OK) {
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(2));

    err = i2c_master_receive(dev_handle_, rx, sizeof(rx), kI2cTimeoutMs / portTICK_PERIOD_MS);
    if (err != ESP_OK) {
        return err;
    }

    if (Crc8(&rx[0], 2) != rx[2] || Crc8(&rx[3], 2) != rx[5]) {
        ESP_LOGE(kTag, "Serial number CRC mismatch");
        return ESP_ERR_INVALID_CRC;
    }

    serial = (static_cast<uint32_t>(rx[0]) << 24) | (static_cast<uint32_t>(rx[1]) << 16) |
             (static_cast<uint32_t>(rx[3]) << 8)  | rx[4];

    return ESP_OK;
}

esp_err_t SHT41::Read(Precision precision, float &temperature_c, float &humidity_rh)
{
    if (!dev_handle_) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t cmd;
    uint32_t delay_ms;
    esp_err_t err = PrecisionToCmd(precision, cmd, delay_ms);
    if (err != ESP_OK) {
        return err;
    }

    err = i2c_master_transmit(dev_handle_, &cmd, 1, kI2cTimeoutMs / portTICK_PERIOD_MS);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "Measure cmd failed: %s", esp_err_to_name(err));
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(delay_ms));

    uint8_t rx[6] = {0};
    err = i2c_master_receive(dev_handle_, rx, sizeof(rx), kI2cTimeoutMs / portTICK_PERIOD_MS);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "Read failed: %s", esp_err_to_name(err));
        return err;
    }

    if (Crc8(&rx[0], 2) != rx[2]) {
        ESP_LOGE(kTag, "Temperature CRC mismatch");
        return ESP_ERR_INVALID_CRC;
    }
    if (Crc8(&rx[3], 2) != rx[5]) {
        ESP_LOGE(kTag, "Humidity CRC mismatch");
        return ESP_ERR_INVALID_CRC;
    }

    uint16_t raw_t  = (static_cast<uint16_t>(rx[0]) << 8) | rx[1];
    uint16_t raw_rh = (static_cast<uint16_t>(rx[3]) << 8) | rx[4];

    temperature_c = -45.0f + 175.0f * (static_cast<float>(raw_t) / 65535.0f);

    float rh = -6.0f + 125.0f * (static_cast<float>(raw_rh) / 65535.0f);
    if (rh > 100.0f) rh = 100.0f;
    if (rh < 0.0f)   rh = 0.0f;
    humidity_rh = rh;

    return ESP_OK;
}