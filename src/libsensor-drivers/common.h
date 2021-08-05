#pragma once

#include <driver/i2c.h>

#include "libsensor-drivers.h"

typedef esp_err_t (*i2c_init_fn_t)(i2c_port_t port, uint8_t addr,
                                   void** out_dev);

esp_err_t libsensor_drv_register_generic_i2c(i2c_init_fn_t init_fn,
                                             const sensor_type_t* type,
                                             i2c_port_t port, uint8_t addr,
                                             const char* tag);
