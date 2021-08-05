#include "common.h"

#include <libiot.h>

static const char* TAG = "libsensor-drivers";

esp_err_t libsensor_drv_register_generic_i2c(i2c_init_fn_t init_fn,
                                             const sensor_type_t* type,
                                             i2c_port_t port, uint8_t addr,
                                             const char* tag) {
    esp_err_t ret;

    void* dev;
    ret = init_fn(port, (uint8_t) addr, &dev);
    if (ret != ESP_OK) {
        libiot_logf_error(TAG, "couldn't init sensor for I2C address: 0x%02X",
                          addr);
        return ret;
    }

    ret = libsensor_register(type, tag, dev, NULL);
    if (ret != ESP_OK) {
        libiot_logf_error(TAG,
                          "couldn't register sensor for I2C address: 0x%02X",
                          addr);
        return ret;
    }

    // FIXME For the moment there is no sane way to allow unregistration of
    // these sensors, since the caller app (outside of this driver) has knows
    // nothing about the identity of the structure returned by
    // `libsensor_unregister()`---for many devices it is a raw driver handle,
    // but for others it is substantially more complicated.
    //
    // The caller doesn't even know how to free the resulting handle, so this
    // would be be a memory leak. There are a few solutions---we'll see what we
    // need.
    return ESP_OK;
}
