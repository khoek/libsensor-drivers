#include <cJSON.h>
#include <device/sps30.h>
#include <esp_err.h>
#include <esp_log.h>
#include <libesp.h>
#include <libesp/json.h>
#include <libiot.h>
#include <libsensor.h>

#include "../common.h"

static const char* TAG = "sensor(sps30)";

static void dev_start(sps30_handle_t dev, void* unused) {
    const uint16_t output_format = SPS30_START_MEASUREMENT_OUTPUT_32BIT_FLOAT;
    ESP_ERROR_CHECK(
        sps30_cmd_write(dev, &SPS30_CMD_START_MEASUREMENT, &output_format, 1));

    // Allow time for the fan to spin up. The datasheet-specified time
    // before the `SPEED` warning status bit may be set by the device
    // is 3000ms, but empirically the `FAN` and `LASER` error bits can
    // be set if we check this early.
    vTaskDelay(10000 / portTICK_PERIOD_MS);
}

static void dev_reset(sps30_handle_t dev, void* unused) {
    ESP_ERROR_CHECK(sps30_reset(dev));
}

static sensor_poll_result_t poll(const char* tag, sps30_handle_t dev,
                                 void* unused, sensor_output_handle_t output) {
    uint16_t device_status[2];
    ESP_ERROR_CHECK(sps30_cmd_read(dev, &SPS30_CMD_READ_DEVICE_STATUS_REGISTER,
                                   device_status, 2));

    if (device_status[0] & SPS30_DEVICE_STATUS_REGISTER_WORD0_LASER) {
        libiot_logf_error(TAG, "(%s) ERROR: bad laser current", tag);
    }

    if (device_status[0] & SPS30_DEVICE_STATUS_REGISTER_WORD0_FAN) {
        libiot_logf_error(TAG, "(%s) ERROR: fan stop", tag);
    }

    if (device_status[1] & SPS30_DEVICE_STATUS_REGISTER_WORD1_SPEED) {
        libiot_logf_error(TAG, "(%s) WARNING: bad fan speed", tag);
    }

    if (device_status[0] & ~MASK_SPS30_DEVICE_STATUS_REGISTER_WORD0) {
        libiot_logf_error(TAG, "(%s) ERROR: unknown word0 code (0x%04X)", tag,
                          device_status[0]);
    }

    if (device_status[1] & ~MASK_SPS30_DEVICE_STATUS_REGISTER_WORD1) {
        libiot_logf_error(TAG, "(%s) ERROR: unknown word1 code (0x%04X)", tag,
                          device_status[1]);
    }

    uint16_t dataready_flag;
    ESP_ERROR_CHECK(sps30_cmd_read(dev, &SPS30_CMD_READ_DATAREADY_FLAG,
                                   &dataready_flag, 1));

    if (!(dataready_flag & SPS30_DATAREADY_FLAG_ENABLED)) {
        return SENSOR_POLL_RESULT_UNEVENTFUL;
    }

    sps30_measured_values_32bit_float_t* result =
        malloc(sizeof(sps30_measured_values_32bit_float_t));
    ESP_ERROR_CHECK(sps30_cmd_read_measured_values_32bit_float(dev, result));

    libsensor_output_item(output, (void*) result);
    return SENSOR_POLL_RESULT_MADE_PROGRESS;
}

static cJSON* report(const char* tag, sps30_handle_t dev, void* unused,
                     sps30_measured_values_32bit_float_t* result) {
    ESP_LOGI(TAG,
             "(%s) measure: mass_conc(ug/m3):                   [PM1.0]=%.7lf "
             "[PM2.5]=%.7lf [PM4.0]=%.7lf [PM10]=%.7lf",
             tag, result->mass_conc_pm1_0_ug_per_m3,
             result->mass_conc_pm2_5_ug_per_m3,
             result->mass_conc_pm4_0_ug_per_m3,
             result->mass_conc_pm10_ug_per_m3);
    ESP_LOGI(TAG,
             "(%s) measure: numb_conc(/cm3):  [PM0.5]=%.7lf [PM1.0]=%.7lf "
             "[PM2.5]=%.7lf [PM4.0]=%.7lf [PM10]=%.7lf",
             tag, result->numb_conc_pm0_5_per_cm3,
             result->numb_conc_pm1_0_per_cm3, result->numb_conc_pm2_5_per_cm3,
             result->numb_conc_pm4_0_per_cm3, result->numb_conc_pm10_per_cm3);
    ESP_LOGI(TAG, "(%s) measure: typical_particle_size(um)=%.7lf", tag,
             result->typical_particle_size_um);

    cJSON* json_root;
    cJSON_CREATE_ROOT_OBJ_OR_GOTO(&json_root, report_fail);

    cJSON_INSERT_NUMBER_INTO_OBJ_OR_GOTO(json_root, "mass_conc_pm1_0_ug_per_m3",
                                         result->mass_conc_pm1_0_ug_per_m3,
                                         report_fail);
    cJSON_INSERT_NUMBER_INTO_OBJ_OR_GOTO(json_root, "mass_conc_pm2_5_ug_per_m3",
                                         result->mass_conc_pm2_5_ug_per_m3,
                                         report_fail);
    cJSON_INSERT_NUMBER_INTO_OBJ_OR_GOTO(json_root, "mass_conc_pm4_0_ug_per_m3",
                                         result->mass_conc_pm4_0_ug_per_m3,
                                         report_fail);
    cJSON_INSERT_NUMBER_INTO_OBJ_OR_GOTO(json_root, "mass_conc_pm10_ug_per_m3",
                                         result->mass_conc_pm10_ug_per_m3,
                                         report_fail);

    cJSON_INSERT_NUMBER_INTO_OBJ_OR_GOTO(json_root, "numb_conc_pm0_5_per_cm3",
                                         result->numb_conc_pm0_5_per_cm3,
                                         report_fail);
    cJSON_INSERT_NUMBER_INTO_OBJ_OR_GOTO(json_root, "numb_conc_pm1_0_per_cm3",
                                         result->numb_conc_pm1_0_per_cm3,
                                         report_fail);
    cJSON_INSERT_NUMBER_INTO_OBJ_OR_GOTO(json_root, "numb_conc_pm2_5_per_cm3",
                                         result->numb_conc_pm2_5_per_cm3,
                                         report_fail);
    cJSON_INSERT_NUMBER_INTO_OBJ_OR_GOTO(json_root, "numb_conc_pm4_0_per_cm3",
                                         result->numb_conc_pm4_0_per_cm3,
                                         report_fail);
    cJSON_INSERT_NUMBER_INTO_OBJ_OR_GOTO(json_root, "numb_conc_pm10_per_cm3",
                                         result->numb_conc_pm10_per_cm3,
                                         report_fail);

    cJSON_INSERT_NUMBER_INTO_OBJ_OR_GOTO(json_root, "typical_particle_size_um",
                                         result->typical_particle_size_um,
                                         report_fail);

    free(result);
    return json_root;

report_fail:
    libiot_logf_error(TAG, "%s: JSON build fail", __func__);

    // It is safe to call this with `json_root == NULL`.
    cJSON_Delete(json_root);
    free(result);
    return NULL;
}

static const sensor_type_t SENSOR_SPS30 = {
    .model = LIBSENSOR_NAME_SPS30,

    .poll_delay_ms = 250,
    .max_uneventful_iters = 10,

    .poll_task_stack_size = 2048,
    .report_task_stack_size = 3072,

    .dev_destroy = (sensor_dev_destroy_fn_t) sps30_destroy,
    .dev_start = (sensor_dev_start_fn_t) dev_start,
    .dev_reset = (sensor_dev_reset_fn_t) dev_reset,

    .poll = (sensor_poll_fn_t) poll,
    .report = (sensor_report_fn_t) report,
};

esp_err_t libsensor_drv_register_sps30(i2c_port_t port,
                                       libsensor_drv_sps30_addr_t addr,
                                       const char* tag) {
    return libsensor_drv_register_generic_i2c((i2c_init_fn_t) sps30_init,
                                              &SENSOR_SPS30, port, addr, tag);
}
