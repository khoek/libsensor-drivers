#include <cJSON.h>
#include <device/tmp117.h>
#include <esp_err.h>
#include <esp_log.h>
#include <libesp/json.h>
#include <libiot.h>
#include <libsensor.h>

#include "../common.h"

static const char* TAG = "sensor(tmp117)";

static void dev_start(tmp117_handle_t dev, void* unused) {
    // First place the TMP117 into shutdown mode to stop it converting, and
    // clear the EEPROM_BUSY flag by reading the TEMP_RESULT.
    ESP_ERROR_CHECK(tmp117_reg_write(dev, TMP117_REG_CONFIGURATION,
                                     TMP117_CONFIGURATION_MOD_SD));
    uint16_t val;
    ESP_ERROR_CHECK(tmp117_reg_read(dev, TMP117_REG_TEMP_RESULT, &val));

    // Next place the TMP117 into continuous conversion mode with 64 samples
    // averaged in order to obtain each final value. A value will be reported
    // roughly every one second.
    ESP_ERROR_CHECK(tmp117_reg_write(dev, TMP117_REG_CONFIGURATION,
                                     TMP117_CONFIGURATION_MOD_CC
                                         | TMP117_CONFIGURATION_AVG_64));
}

static void dev_reset(tmp117_handle_t dev, void* unused) {
    ESP_ERROR_CHECK(tmp117_reset(dev));
}

static sensor_poll_result_t poll(const char* tag, tmp117_handle_t dev,
                                 void* unused, sensor_output_handle_t output) {
    uint16_t cfg;
    ESP_ERROR_CHECK(tmp117_reg_read(dev, TMP117_REG_CONFIGURATION, &cfg));
    if (!(cfg & TMP117_CONFIGURATION_DATA_READY)) {
        return SENSOR_POLL_RESULT_UNEVENTFUL;
    }

    int16_t raw;
    ESP_ERROR_CHECK(
        tmp117_reg_read(dev, TMP117_REG_TEMP_RESULT, (uint16_t*) &raw));

    libsensor_output_item(output, (void*) (uint32_t) raw);
    return SENSOR_POLL_RESULT_MADE_PROGRESS;
}

static cJSON* report(const char* tag, tmp117_handle_t dev, void* unused,
                     void* ptr_raw) {
    uint16_t raw = (uint16_t) (uint32_t) ptr_raw;
    double temp_c = ((double) raw) * 0.0078125;

    ESP_LOGI(TAG, "(%s) measure: temp(oC)=%.7lf", tag, temp_c);

    cJSON* json_root;
    cJSON_CREATE_ROOT_OBJ_OR_GOTO(&json_root, report_fail);
    cJSON_INSERT_NUMBER_INTO_OBJ_OR_GOTO(json_root, "temp_c", temp_c,
                                         report_fail);

    return json_root;

report_fail:
    libiot_logf_error(TAG, "%s: JSON build fail", __func__);

    // It is safe to call this with `json_root == NULL`.
    cJSON_Delete(json_root);
    return NULL;
}

static const sensor_type_t SENSOR_TMP117 = {
    .model = LIBSENSOR_NAME_TMP117,

    .poll_delay_ms = 250,
    .max_uneventful_iters = 10,

    .poll_task_stack_size = 2048,
    .report_task_stack_size = 2560,

    .dev_destroy = (sensor_dev_destroy_fn_t) tmp117_destroy,
    .dev_start = (sensor_dev_start_fn_t) dev_start,
    .dev_reset = (sensor_dev_reset_fn_t) dev_reset,

    .poll = (sensor_poll_fn_t) poll,
    .report = (sensor_report_fn_t) report,
};

esp_err_t libsensor_drv_register_tmp117(i2c_port_t port,
                                        libsensor_drv_tmp117_addr_t addr,
                                        const char* tag) {
    return libsensor_drv_register_generic_i2c((i2c_init_fn_t) tmp117_init,
                                              &SENSOR_TMP117, port, addr, tag);
}
