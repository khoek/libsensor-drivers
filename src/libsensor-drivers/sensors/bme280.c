#include <cJSON.h>
#include <device/bme280.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <libesp.h>
#include <libiot.h>
#include <libsensor.h>
#include <math.h>

#include "../common.h"

static const char* TAG = "sensor(bme280)";

#define OSRS_H BME280_CTRL_HUM_OSRS_H_x16
#define OSRS_P BME280_CTRL_MEAS_OSRS_P_x16
#define OSRS_T BME280_CTRL_MEAS_OSRS_T_x16

static void dev_start(bme280_handle_t dev, void* unused) {
    // The BME280 has no "data available" flag which is set when a measurement completes,
    // which means we cannot reliably synchonize reading at 1 second per measurement rate
    // (every now and then we would miss a measurement, or double up on the previous measurement).
    //
    // To solve this problem we place the device into continuous sampling mode (gaps between sets of
    // samples of 0.5ms), and use the "filter" function to average previous measurements to reduce noise.
    bme280_reg_write(dev, BME280_REG_CONFIG, BME280_CONFIG_T_SB_0_5_ms | BME280_CONFIG_FILTER_16);

    // DANGER: Writes to REG_CTRL_HUM are only effective after a write operation
    // to REG_CTRL_MEAS.
    bme280_reg_write(dev, BME280_REG_CTRL_HUM, OSRS_H);
    bme280_reg_write(dev, BME280_REG_CTRL_MEAS, OSRS_T | OSRS_P | BME280_CTRL_MEAS_MODE_NORMAL);

    // The BME280 has no "data available" flag and `libsensor` will begin to poll right away
    // (only insterting a delay between the subsequent poll requests), so wait 3 seconds in
    // order for sensor measurements to be come valid (empirically this seems to happen after
    // around 1 second).
    vTaskDelay(3000 / portTICK_PERIOD_MS);
}

typedef struct measurement {
    uint32_t raw_press;
    uint32_t raw_temp;
    uint16_t raw_hum;
} measurement_t;

static sensor_poll_result_t poll(const char* tag, bme280_handle_t dev, QueueHandle_t queue) {
    // The BME280 has no "data available" flag which is set when a measurement completes,
    // so we resort to manually polling the device every 1000ms (and place the device
    // into a mode where it is continuously sampling at a rate which is much faster than
    // that).

    measurement_t result;
    bme280_read_sample_regs(dev, &result.raw_press, &result.raw_temp, &result.raw_hum);

    if (xQueueSend(queue, &result, 0) != pdTRUE) {
        libiot_logf_error(TAG, "can't queue result");
    }

    return SENSOR_POLL_RESULT_MADE_PROGRESS;
}

static cJSON* report(const char* tag, bme280_handle_t dev, measurement_t* result) {
    double temp_c;
    double t_param;
    bme280_calc_compensated_temp(dev, result->raw_temp, &temp_c, &t_param);

    double press_pa;
    double rel_humidity;
    bme280_calc_compensated_press(dev, result->raw_press, t_param, &press_pa);
    bme280_calc_compensated_hum(dev, result->raw_hum, t_param, &rel_humidity);

    ESP_LOGI(TAG, "(%s) measure: temp(oC)=%.7lf, press(Pa)=%.7lf, rel_humidity(%%)=%.7lf", tag, temp_c, press_pa, rel_humidity);

    cJSON* json_root = cJSON_CreateObject();
    if (!json_root) {
        goto report_out;
    }

    cJSON* json_temp_c = cJSON_CreateNumber(temp_c);
    if (!json_temp_c) {
        goto report_out;
    }
    cJSON_AddItemToObject(json_root, "temp_c", json_temp_c);

    cJSON* json_press_pa = cJSON_CreateNumber(press_pa);
    if (!json_press_pa) {
        goto report_out;
    }
    cJSON_AddItemToObject(json_root, "press_pa", json_press_pa);

    cJSON* json_rel_humidity = cJSON_CreateNumber(rel_humidity);
    if (!json_rel_humidity) {
        goto report_out;
    }
    cJSON_AddItemToObject(json_root, "rel_humidity", json_rel_humidity);

    return json_root;

report_out:
    libiot_logf_error(TAG, "%s: JSON build fail", __func__);

    // It is safe to call this with `json_root == NULL`.
    cJSON_Delete(json_root);
    return NULL;
}

static const sensor_type_t SENSOR_BME280 = {
    .name = LIBSENSOR_NAME_BME280,

    .queue_item_size = sizeof(measurement_t),
    .poll_delay_ms = 1000,
    .max_uneventful_iters = 0,

    .dev_start = (sensor_dev_start_fn_t) dev_start,
    .dev_reset = (sensor_dev_reset_fn_t) bme280_reset,

    .poll = (sensor_poll_fn_t) poll,
    .report = (sensor_report_fn_t) report,
};

esp_err_t libsensor_drv_register_bme280(i2c_port_t port, libsensor_drv_bme280_addr_t addr,
                                        const char* tag) {
    return libsensor_drv_register_generic_i2c((i2c_init_fn_t) bme280_init, &SENSOR_BME280, port, addr, tag);
}
