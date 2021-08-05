#pragma once

#include <driver/gpio.h>
#include <driver/i2c.h>
#include <driver/spi_master.h>
#include <libsensor.h>

//////////// SPI Devices ////////////

#define LIBSENSOR_NAME_RFM69HCW_LACROSSE "rfm69hcw-lacrosse"

esp_err_t libsensor_drv_register_rfm69hcw_lacrosse(spi_host_device_t host,
                                                   gpio_num_t pin_cs,
                                                   gpio_num_t pin_rst,
                                                   gpio_num_t pin_irq,
                                                   const char *tag);

//////////// I2C Devices ////////////

#define LIBSENSOR_NAME_BME280 "bme280"

typedef enum libsensor_drv_bme280_addr {
    BME280_I2C_ADDR_LOW = 0x76,
    BME280_I2C_ADDR_HIGH = 0x77,
} libsensor_drv_bme280_addr_t;

esp_err_t libsensor_drv_register_bme280(i2c_port_t port,
                                        libsensor_drv_bme280_addr_t addr,
                                        const char *tag);

#define LIBSENSOR_NAME_CCS811 "ccs811"

typedef enum libsensor_drv_ccs811_addr {
    CCS811_I2C_ADDR_LOW = 0x5A,
    CCS811_I2C_ADDR_HIGH = 0x5B,
} libsensor_drv_ccs811_addr_t;

esp_err_t libsensor_drv_register_ccs811(i2c_port_t port,
                                        libsensor_drv_ccs811_addr_t addr,
                                        const char *tag, bool enable_updates);

#define LIBSENSOR_NAME_DPS368 "dps368"

typedef enum libsensor_drv_dps368_addr {
    DPS368_I2C_ADDR_LOW = 0x76,
    DPS368_I2C_ADDR_HIGH = 0x77,
} libsensor_drv_dps368_addr_t;

esp_err_t libsensor_drv_register_dps368(i2c_port_t port,
                                        libsensor_drv_dps368_addr_t addr,
                                        const char *tag);

#define LIBSENSOR_NAME_SCD41 "scd41"

typedef enum libsensor_drv_scd41_addr {
    SCD41_I2C_ADDR = 0x62,
} libsensor_drv_scd41_addr_t;

esp_err_t libsensor_drv_register_scd41(i2c_port_t port,
                                       libsensor_drv_scd41_addr_t addr,
                                       const char *tag, bool enable_updates);

#define LIBSENSOR_NAME_SPS30 "sps30"

typedef enum libsensor_drv_sps30_addr {
    SPS30_I2C_ADDR = 0x69,
} libsensor_drv_sps30_addr_t;

esp_err_t libsensor_drv_register_sps30(i2c_port_t port,
                                       libsensor_drv_sps30_addr_t addr,
                                       const char *tag);

#define LIBSENSOR_NAME_SFA30 "sfa30"

typedef enum libsensor_drv_sfa30_addr {
    SFA30_I2C_ADDR = 0x5D,
} libsensor_drv_sfa30_addr_t;

esp_err_t libsensor_drv_register_sfa30(i2c_port_t port,
                                       libsensor_drv_sfa30_addr_t addr,
                                       const char *tag);

#define LIBSENSOR_NAME_SGP40 "sgp40"

typedef enum libsensor_drv_sgp40_addr {
    SGP40_I2C_ADDR = 0x59,
} libsensor_drv_sgp40_addr_t;

esp_err_t libsensor_drv_register_sgp40(i2c_port_t port,
                                       libsensor_drv_sgp40_addr_t addr,
                                       const char *tag, bool enable_updates);

#define LIBSENSOR_NAME_TMP117 "tmp117"

typedef enum libsensor_drv_tmp117_addr {
    TMP117_I2C_ADDR_GND = 0x48,
    TMP117_I2C_ADDR_VCC = 0x49,
    TMP117_I2C_ADDR_SDA = 0x4A,
    TMP117_I2C_ADDR_SCL = 0x4B,
} libsensor_drv_tmp117_addr_t;

esp_err_t libsensor_drv_register_tmp117(i2c_port_t port,
                                        libsensor_drv_tmp117_addr_t addr,
                                        const char *tag);
