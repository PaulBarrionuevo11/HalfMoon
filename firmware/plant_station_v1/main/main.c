#include <stdio.h>

#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "driver/i2c_master.h"

#include "bh1750.h"
#include "sht41.h"


/* ---------------------------------------------------------
 * Sensor objects
 * --------------------------------------------------------- */

bh1750_t bh1750;
sht41_t sht41;


/* ---------------------------------------------------------
 * Logging
 * --------------------------------------------------------- */

static const char *TAG = "MAIN";


/* ---------------------------------------------------------
 * I2C configuration
 * --------------------------------------------------------- */

#define I2C_MASTER_SCL_IO           9
#define I2C_MASTER_SDA_IO           8

#define I2C_MASTER_NUM              I2C_NUM_0

#define I2C_MASTER_FREQ_HZ          100000

#define I2C_MASTER_TIMEOUT_MS       1000


/* ---------------------------------------------------------
 * Sensor addresses
 * --------------------------------------------------------- */

#define BH1750_SENSOR_ADDR          0x23
#define SHT41_SENSOR_ADDR           0x44


/* ---------------------------------------------------------
 * Initialize I2C bus and sensors
 * --------------------------------------------------------- */

static void i2c_master_init(
    i2c_master_bus_handle_t *bus_handle)
{
    /*
     * Configure I2C master bus.
     */
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    /*
     * Create I2C master bus.
     */
    ESP_ERROR_CHECK(
        i2c_new_master_bus(
            &bus_config,
            bus_handle
        )
    );

    /*
     * Initialize BH1750.
     */
    esp_err_t bh1750_sensor = init_lux_sensor(
        *bus_handle,
        BH1750_SENSOR_ADDR,
        I2C_MASTER_FREQ_HZ,
        &bh1750
    );


    /*
     * Initialize SHT41.
     */
    esp_err_t sht41_sensor = init_hum_temp_sensor(
        *bus_handle,
        SHT41_SENSOR_ADDR,
        I2C_MASTER_FREQ_HZ,
        &sht41
    );


    /*
     * Stop the program if either device
     * failed to register with the I2C bus.
     */
    ESP_ERROR_CHECK(bh1750_sensor);
    ESP_ERROR_CHECK(sht41_sensor);
}


/* ---------------------------------------------------------
 * Main application
 * --------------------------------------------------------- */

void app_main(void)
{
    i2c_master_bus_handle_t bus_handle;

    /*
     * Initialize I2C and sensors.
     */
    i2c_master_init(&bus_handle);

    ESP_LOGI(
        TAG,
        "I2C initialized successfully"
    );


    while (1)
    {
        /* -------------------------------------------------
         * Check whether BH1750 is responding
         * ------------------------------------------------- */

        esp_err_t bht_i2c = i2c_master_probe(
            bus_handle,
            BH1750_SENSOR_ADDR,
            I2C_MASTER_TIMEOUT_MS
        );


        /* -------------------------------------------------
         * Check whether SHT41 is responding
         * ------------------------------------------------- */

        esp_err_t sht_i2c = i2c_master_probe(
            bus_handle,
            SHT41_SENSOR_ADDR,
            I2C_MASTER_TIMEOUT_MS
        );


        /* -------------------------------------------------
         * Both sensors connected
         * ------------------------------------------------- */

        if (bht_i2c == ESP_OK &&
            sht_i2c == ESP_OK)
        {
            ESP_LOGI(
                TAG,
                "BH1750 connected"
            );

            ESP_LOGI(
                TAG,
                "SHT41 connected"
            );


            /* ---------------------------------------------
             * Read SHT41
             * --------------------------------------------- */
            vTaskDelay(
                pdMS_TO_TICKS(200)  // Prevents initial read failure 
            );
            esp_err_t sht41_err =
                sht41_read_measurement(&sht41);


            if (sht41_err == ESP_OK)
            {
                ESP_LOGI(
                    TAG,
                    "SHT41 Temperature: %.2f C",
                    sht41.temperature
                );

                ESP_LOGI(
                    TAG,
                    "SHT41 Humidity: %.2f %%",
                    sht41.humidity
                );
            }
            else
            {
                ESP_LOGE(
                    TAG,
                    "SHT41 read failed: %s",
                    esp_err_to_name(sht41_err)
                );
            }
        }


        /* -------------------------------------------------
         * Neither sensor connected
         * ------------------------------------------------- */

        else if (bht_i2c == ESP_ERR_NOT_FOUND &&
                 sht_i2c == ESP_ERR_NOT_FOUND)
        {
            ESP_LOGW(
                TAG,
                "BH1750 not found"
            );

            ESP_LOGW(
                TAG,
                "SHT41 not found"
            );
        }


        /* -------------------------------------------------
         * One sensor connected / one disconnected
         * ------------------------------------------------- */

        else
        {
            if (bht_i2c == ESP_OK)
            {
                ESP_LOGI(
                    TAG,
                    "BH1750 connected"
                );
            }
            else
            {
                ESP_LOGW(
                    TAG,
                    "BH1750 disconnected"
                );
            }


            if (sht_i2c == ESP_OK)
            {
                ESP_LOGI(
                    TAG,
                    "SHT41 connected"
                );
            }
            else
            {
                ESP_LOGW(
                    TAG,
                    "SHT41 disconnected"
                );
            }
        }


        /*
         * Wait three seconds before the next measurement.
         */
        vTaskDelay(
            pdMS_TO_TICKS(2000)
        );
    }


    /*
     * Normally this code is never reached because
     * app_main() stays inside the while loop.
     *
     * If you later change the architecture so that
     * the loop exits, then de-initialize the devices
     * here.
     */

    ESP_ERROR_CHECK(
        i2c_master_bus_rm_device(
            bh1750.dev_handle
        )
    );

    ESP_ERROR_CHECK(
        i2c_master_bus_rm_device(
            sht41.dev_handle
        )
    );

    ESP_ERROR_CHECK(
        i2c_del_master_bus(
            bus_handle
        )
    );

    ESP_LOGI(
        TAG,
        "I2C de-initialized successfully"
    );
}