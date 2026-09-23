#pragma once

#include <cstdint>
#include "esp_err.h"
#include "driver/i2c_master.h"

class SHT41 {
public:
    enum class Precision {
        High,   // cmd 0xFD, ~10ms conversion
        Medium, // cmd 0xF6, ~5ms
        Low,    // cmd 0xE0, ~2ms
    };

    static constexpr uint8_t kDefaultI2cAddr = 0x44;

    SHT41() = default;

    // Non-copyable: this class owns an I2C device handle tied to the bus.
    SHT41(const SHT41 &) = delete;
    SHT41 &operator=(const SHT41 &) = delete;

    // Movable: transfers device-handle ownership.
    SHT41(SHT41 &&other) noexcept;
    SHT41 &operator=(SHT41 &&other) noexcept;

    ~SHT41();

    /**
     * @brief Attach an SHT41 to an already-initialized I2C master bus.
     *
     * Factory-style constructor returning esp_err_t instead of throwing,
     * since ESP-IDF builds run with C++ exceptions disabled by default.
     *
     * @param bus_handle   Handle from i2c_new_master_bus()
     * @param out_sensor   Constructed on success; left untouched on failure
     * @param addr         7-bit I2C address (default 0x44)
     */
    static esp_err_t Create(i2c_master_bus_handle_t bus_handle,
                             SHT41 &out_sensor,
                             uint8_t addr = kDefaultI2cAddr);

    /**
     * @brief Trigger a single-shot measurement and read temperature/humidity.
     * Blocking: sends the measure command, sleeps for the conversion time,
     * then reads and CRC-checks the 6-byte response.
     */
    esp_err_t Read(Precision precision, float &temperature_c, float &humidity_rh);

    /// Read the sensor's unique 32-bit serial number.
    esp_err_t ReadSerial(uint32_t &serial);

    /// Soft-reset the sensor (~1ms to complete).
    esp_err_t SoftReset();

    /// True if this instance owns a valid, attached I2C device handle.
    bool IsValid() const { return dev_handle_ != nullptr; }

private:

    static uint8_t Crc8(const uint8_t *data, size_t len);
    static esp_err_t PrecisionToCmd(Precision precision, uint8_t &cmd, uint32_t &delay_ms);

    i2c_master_dev_handle_t dev_handle_ = nullptr;

    static constexpr uint8_t kCmdSoftReset  = 0x94;
    static constexpr uint8_t kCmdReadSerial = 0x89;
    static constexpr uint32_t kI2cTimeoutMs = 1000;
};