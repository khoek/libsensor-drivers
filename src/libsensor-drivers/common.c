#include "common.h"

esp_err_t libsensor_drv_register_generic_i2c(i2c_init_fn_t init_fn, const sensor_type_t* type,
                                             i2c_port_t port, uint8_t addr,
                                             const char* tag) {
    void* dev;

    esp_err_t ret = init_fn(port, (uint8_t) addr, &dev);
    if (ret != ESP_OK) {
        return ret;
    }

    // FIXME For the moment there is no sane way to allow unregistration of these sensors,
    // since the caller app (outside of this driver) has knows nothing about the identity
    // of the structure returned by `libsensor_unregister()`---for many devices it is a
    // raw driver handle, but for others it is substantially more complicated.
    //
    // The caller doesn't even know how to free the resulting handle, so this would be
    // be a memory leak. There are a few solutions---we'll see what we need.
    return libsensor_register(type, tag, dev, NULL);
}
