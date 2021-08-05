#include <cJSON.h>
#include <device/sfa30.h>
#include <esp_err.h>
#include <esp_log.h>
#include <libesp.h>
#include <libesp/json.h>
#include <libiot.h>
#include <libsensor.h>

#include "../common.h"

static const char* TAG = "sensor(sfa30)";

#define INITIAL_DELAY_MS 11000

static void dev_start(sfa30_handle_t dev, void* unused) {
    ESP_ERROR_CHECK(
        sfa30_cmd_exec(dev, &SFA30_CMD_START_CONTINUOUS_MEASUREMENT));

    // As per spec, the sensor takes 10s before reporting a nonzero raw H2CO
    // value.
    vTaskDelay(INITIAL_DELAY_MS / portTICK_PERIOD_MS);

    // Read and discard the first measured value(s).
    uint16_t raw_data[3];
    ESP_ERROR_CHECK(
        sfa30_cmd_read(dev, &SFA30_CMD_READ_MEASURED_VALUES, raw_data, 3));
}

static void dev_reset(sfa30_handle_t dev, void* unused) {
    ESP_ERROR_CHECK(sfa30_reset(dev));
}

typedef struct measurement {
    uint16_t raw_h2co;
    uint16_t raw_hum;
    uint16_t raw_temp;
} measurement_t;

static sensor_poll_result_t poll(const char* tag, sfa30_handle_t dev,
                                 void* unused, sensor_output_handle_t output) {
    // The SFA30 has no "data available" flag which is set when a measurement
    // completes, so we resort to manually polling the device every 1117ms. The
    // device samples approximately every 500ms, and because of the
    // cumulative-averaging system it uses returns the arithmetic mean of all
    // measurements since the last sample (if it has been <60 seconds since the
    // last sample, in which case a geometric averaging system becomes
    // involved).
    //
    // Since 1003ms is coprime with 1000ms this should work reasonably well,
    // usually recording the average of the last two samples.

    uint16_t raw_data[3];
    ESP_ERROR_CHECK(
        sfa30_cmd_read(dev, &SFA30_CMD_READ_MEASURED_VALUES, raw_data, 3));

    measurement_t* result = malloc(sizeof(measurement_t));
    result->raw_h2co = raw_data[0];
    result->raw_hum = raw_data[1];
    result->raw_temp = raw_data[2];

    libsensor_output_item(output, result);
    return SENSOR_POLL_RESULT_MADE_PROGRESS;
}

static cJSON* report(const char* tag, sfa30_handle_t dev, void* unused,
                     measurement_t* result) {
    double h2co_ppb;
    double temp_c;
    double rel_humidity;
    sfa30_decode_h2co(result->raw_h2co, &h2co_ppb);
    sfa30_decode_temp(result->raw_temp, &temp_c);
    sfa30_decode_hum(result->raw_hum, &rel_humidity);

    free(result);

    ESP_LOGI(
        TAG,
        "(%s) measure: H2CO(ppb)=%.7lf, temp(oC)=%.7lf, rel_humidity(%%)=%.7lf",
        tag, h2co_ppb, temp_c, rel_humidity);

    cJSON* json_root;
    cJSON_CREATE_ROOT_OBJ_OR_GOTO(&json_root, report_fail);
    cJSON_INSERT_NUMBER_INTO_OBJ_OR_GOTO(json_root, "h2co_ppb", h2co_ppb,
                                         report_fail);
    cJSON_INSERT_NUMBER_INTO_OBJ_OR_GOTO(json_root, "temp_c", temp_c,
                                         report_fail);
    cJSON_INSERT_NUMBER_INTO_OBJ_OR_GOTO(json_root, "rel_humidity",
                                         rel_humidity, report_fail);

    return json_root;

report_fail:
    libiot_logf_error(TAG, "%s: JSON build fail", __func__);

    // It is safe to call this with `json_root == NULL`.
    cJSON_Delete(json_root);
    return NULL;
}

static const sensor_type_t SENSOR_SFA30 = {
    .model = LIBSENSOR_NAME_SFA30,

    .poll_delay_ms = 1003,  // Coprime to 1000, see comment in `poll()`.
    .max_uneventful_iters = 0,

    .poll_task_stack_size = 2048,
    .report_task_stack_size = 2560,

    .dev_destroy = (sensor_dev_destroy_fn_t) sfa30_destroy,
    .dev_start = (sensor_dev_start_fn_t) dev_start,
    .dev_reset = (sensor_dev_reset_fn_t) dev_reset,

    .poll = (sensor_poll_fn_t) poll,
    .report = (sensor_report_fn_t) report,
};

esp_err_t libsensor_drv_register_sfa30(i2c_port_t port,
                                       libsensor_drv_sfa30_addr_t addr,
                                       const char* tag) {
    return libsensor_drv_register_generic_i2c((i2c_init_fn_t) sfa30_init,
                                              &SENSOR_SFA30, port, addr, tag);
}
