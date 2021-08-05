#include <cJSON.h>
#include <device/dps368.h>
#include <esp_err.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <libesp/json.h>
#include <libiot.h>
#include <libsensor.h>

#include "../common.h"

#define TMP_COUNTS DPS368_TMP_CFG_TMP_PRC_128_COUNTS
#define PRS_COUNTS DPS368_PRS_CFG_PM_PRC_128_COUNTS

static const char* TAG = "sensor(dps368)";

typedef struct measurement {
    int32_t raw_tmp;
    int32_t raw_psr;
} measurement_t;

static void dev_start(dps368_handle_t dev, void* unused) {
    uint8_t meas_cfg;
    ESP_ERROR_CHECK(dps368_reg_read(dev, DPS368_REG_MEAS_CFG, &meas_cfg));
    if (meas_cfg & (DPS368_MEAS_CFG_TMP_RDY | DPS368_MEAS_CFG_PRS_RDY)) {
        libiot_logf_error(TAG,
                          "unexpected flag set on startup: MEAS_CFG=0x%02X",
                          meas_cfg);
        abort();
    }

    // Note for the datasheet says for >8 samples: Use in combination with a
    // bit shift. See Interrupt and FIFO configuration (CFG_REG) register.
    ESP_ERROR_CHECK(
        dps368_reg_write(dev, DPS368_REG_CFG_REG,
                         (TMP_COUNTS > DPS368_TMP_CFG_TMP_PRC_8_COUNTS
                              ? DPS368_CFG_REG_T_SHIFT
                              : 0)
                             | (PRS_COUNTS > DPS368_PRS_CFG_PM_PRC_8_COUNTS
                                    ? DPS368_CFG_REG_P_SHIFT
                                    : 0)));

    ESP_ERROR_CHECK(
        dps368_reg_write(dev, DPS368_REG_PRS_CFG,
                         DPS368_PRS_CFG_PM_RATE_1_PER_S | PRS_COUNTS));

    uint8_t coef_srce;
    ESP_ERROR_CHECK(dps368_reg_read(dev, DPS368_REG_COEF_SRCE, &coef_srce));
    bool coef_srce_is_external = coef_srce & DPS368_COEF_SRCE_TMP_COEF_SRCE;

    ESP_ERROR_CHECK(dps368_reg_write(dev, DPS368_REG_TMP_CFG,
                                     (coef_srce_is_external
                                          ? DPS368_TMP_CFG_TMP_EXT_EXTERNAL
                                          : DPS368_TMP_CFG_TMP_EXT_INTERNAL)
                                         | DPS368_TMP_CFG_TMP_RATE_1_PER_S
                                         | TMP_COUNTS));

    // Place the DPS368 into continuous conversion mode for both pressure and
    // temperature, reporting one result per second.
    ESP_ERROR_CHECK(
        dps368_reg_write(dev, DPS368_REG_MEAS_CFG,
                         DPS368_MEAS_CFG_MEAS_CTRL_BG_CTS_PRS_and_TMP));

    // FIXME To be honest, there is enough margin to do two measurements per
    // second...
}

static void dev_reset(dps368_handle_t dev, void* unused) {
    ESP_ERROR_CHECK(dps368_reset(dev));
}

static sensor_poll_result_t poll(const char* tag, dps368_handle_t dev,
                                 void* unused, sensor_output_handle_t output) {
    uint8_t meas_cfg;
    ESP_ERROR_CHECK(dps368_reg_read(dev, DPS368_REG_MEAS_CFG, &meas_cfg));

    if (!(meas_cfg & DPS368_MEAS_CFG_TMP_RDY)) {
        return SENSOR_POLL_RESULT_UNEVENTFUL;
    }

    if (!(meas_cfg & DPS368_MEAS_CFG_PRS_RDY)) {
        libiot_logf_error(TAG, "TMP ready but no PRS: MEAS_CFG=0x%02X",
                          meas_cfg);
        return SENSOR_POLL_RESULT_UNEVENTFUL;
    }

    measurement_t* result = malloc(sizeof(measurement_t));
    ESP_ERROR_CHECK(
        dps368_get_raw_measurements(dev, &result->raw_psr, &result->raw_tmp));

    libsensor_output_item(output, result);
    return SENSOR_POLL_RESULT_MADE_PROGRESS;
}

static cJSON* report(const char* tag, dps368_handle_t dev, void* unused,
                     measurement_t* result) {
    double temp_c =
        dps368_calc_compensated_tmp(dev, result->raw_tmp,
                                    DPS368_SCALE_FACTOR[TMP_COUNTS]);
    double press_pa =
        dps368_calc_compensated_psr(dev, result->raw_psr,
                                    DPS368_SCALE_FACTOR[PRS_COUNTS],
                                    result->raw_tmp,
                                    DPS368_SCALE_FACTOR[TMP_COUNTS]);

    free(result);

    ESP_LOGI(TAG, "(%s) measure: temp(oC)=%.7lf, press(Pa)=%.7lf", tag, temp_c,
             press_pa);

    cJSON* json_root;
    cJSON_CREATE_ROOT_OBJ_OR_GOTO(&json_root, report_fail);
    cJSON_INSERT_NUMBER_INTO_OBJ_OR_GOTO(json_root, "temp_c", temp_c,
                                         report_fail);
    cJSON_INSERT_NUMBER_INTO_OBJ_OR_GOTO(json_root, "press_pa", press_pa,
                                         report_fail);

    return json_root;

report_fail:
    libiot_logf_error(TAG, "%s: JSON build fail", __func__);

    // It is safe to call this with `json_root == NULL`.
    cJSON_Delete(json_root);
    return NULL;
}

static const sensor_type_t SENSOR_DPS368 = {
    .model = LIBSENSOR_NAME_DPS368,

    // Given this, so long as `PRESS_POLL_TASK_DELAY_MS` is quite a bit less
    // than a 1/2 second we won't miss any samples if we poll the temp/pressure
    // ready bits and just read the currently reported result straight after
    // they are set (we wait for both, so we can calculate compensated
    // pressure).
    .poll_delay_ms = 250,
    .max_uneventful_iters = 10,

    .poll_task_stack_size = 2048,
    .report_task_stack_size = 2560,

    .dev_destroy = (sensor_dev_destroy_fn_t) dps368_destroy,
    .dev_start = (sensor_dev_start_fn_t) dev_start,
    .dev_reset = (sensor_dev_reset_fn_t) dev_reset,

    .poll = (sensor_poll_fn_t) poll,
    .report = (sensor_report_fn_t) report,
};

esp_err_t libsensor_drv_register_dps368(i2c_port_t port,
                                        libsensor_drv_dps368_addr_t addr,
                                        const char* tag) {
    return libsensor_drv_register_generic_i2c((i2c_init_fn_t) dps368_init,
                                              &SENSOR_DPS368, port, addr, tag);
}
