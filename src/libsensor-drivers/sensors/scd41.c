#include <cJSON.h>
#include <device/scd41.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/queue.h>
#include <libesp.h>
#include <libiot.h>
#include <libsensor.h>
#include <math.h>

#include "../common.h"

static const char* TAG = "sensor(scd41)";

#define FIRST_UPDATE_WAIT_MS 30000

typedef struct ctx {
    EventGroupHandle_t events;
#define CTX_EVENT_UPDATED_AT_LEAST_ONCE (1ULL << 0)
} ctx_t;

static ctx_t* ctx_init() {
    ctx_t* ctx = malloc(sizeof(ctx_t));
    ctx->events = xEventGroupCreate();

    return ctx;
}

static void ctx_destroy(ctx_t* ctx) {
    vEventGroupDelete(ctx->events);
    free(ctx);
}

static void recv_json(const char* tag, scd41_handle_t dev, ctx_t* ctx, const cJSON* json) {
    cJSON* json_press_pa = cJSON_GetObjectItem(json, "press_pa");
    if (!json_press_pa || !cJSON_IsNumber(json_press_pa)) {
        libiot_logf_error(TAG, "JSON message missing 'press_pa' or not a number");
        return;
    }

    double press_pa = json_press_pa->valuedouble;

    if (press_pa < 30000.0) {
        libiot_logf_error(TAG, "'press_pa' out of range: too low");
        press_pa = 30000.0;
    }

    if (press_pa > 150000.0) {
        libiot_logf_error(TAG, "'press_pa' out of range: too high");
        press_pa = 150000.0;
    }

    uint32_t ambient_pressure = round(press_pa / 100.0);
    assert(ambient_pressure <= UINT16_MAX);
    uint16_t val_press = (uint16_t) ambient_pressure;

    ESP_LOGI(TAG, "(%s) update: ambient_pressure_pa=%.7lf", tag, press_pa);
    ESP_ERROR_CHECK(scd41_cmd_write(dev, &SCD41_CMD_SET_AMBIENT_PRESSURE, &val_press, 1));

    xEventGroupSetBits(ctx->events, CTX_EVENT_UPDATED_AT_LEAST_ONCE);
}

static void dev_start(scd41_handle_t dev, ctx_t* ctx) {
    uint16_t self_test_err;
    ESP_ERROR_CHECK(scd41_cmd_read(dev, &SCD41_CMD_PERFORM_SELF_TEST, &self_test_err, 1));
    if (self_test_err) {
        libiot_logf_error(TAG, "self-test failed! (0x%04X)", self_test_err);
    }

    uint16_t asc_en = SCD41_SET_AUTOMATIC_SELF_CALIBRATION_ENABLED_TRUE;
    ESP_ERROR_CHECK(scd41_cmd_write(dev, &SCD41_CMD_SET_AUTOMATIC_SELF_CALIBRATION_ENABLED, &asc_en, 1));

    if (ctx) {
        EventBits_t result = xEventGroupWaitBits(ctx->events, CTX_EVENT_UPDATED_AT_LEAST_ONCE, false, false, FIRST_UPDATE_WAIT_MS / portTICK_PERIOD_MS);
        if (!(result & CTX_EVENT_UPDATED_AT_LEAST_ONCE)) {
            libiot_logf_error(TAG, "timed out waiting for first external update message, starting measurement anyway");
        }
    }

    ESP_ERROR_CHECK(scd41_cmd_send(dev, &SCD41_CMD_START_PERIODIC_MEASUREMENT));
}

typedef struct measurement {
    uint32_t raw_co2;
    uint32_t raw_temp;
    uint16_t raw_hum;
} measurement_t;

static sensor_poll_result_t poll(const char* tag, scd41_handle_t dev, QueueHandle_t queue) {
    uint16_t data_ready_status;
    ESP_ERROR_CHECK(scd41_cmd_read(dev, &SCD41_CMD_GET_DATA_READY_STATUS, &data_ready_status, 1));

    if (!(data_ready_status & MASK_SCD41_GET_DATA_READY_STATUS_READY)) {
        return SENSOR_POLL_RESULT_UNEVENTFUL;
    }

    uint16_t raw_data[3];
    ESP_ERROR_CHECK(scd41_cmd_read(dev, &SCD41_CMD_READ_MEASUREMENT, raw_data, 3));

    measurement_t result = {
        .raw_co2 = raw_data[0],
        .raw_temp = raw_data[1],
        .raw_hum = raw_data[2],
    };

    if (xQueueSend(queue, &result, 0) != pdTRUE) {
        libiot_logf_error(TAG, "can't queue result");
    }

    return SENSOR_POLL_RESULT_MADE_PROGRESS;
}

static cJSON* report(const char* tag, scd41_handle_t dev, measurement_t* result) {
    double co2_ppm = result->raw_co2;
    double temp_c;
    double rel_humidity;
    scd41_decode_temp(result->raw_temp, &temp_c);
    scd41_decode_hum(result->raw_hum, &rel_humidity);

    ESP_LOGI(TAG, "(%s) measure: CO2(ppm)=%.7lf, temp(oC)=%.7lf, rel_humidity(%%)=%.7lf", tag, co2_ppm, temp_c, rel_humidity);

    cJSON* json_root = cJSON_CreateObject();
    if (!json_root) {
        goto report_out;
    }

    cJSON* json_co2_ppm = cJSON_CreateNumber(co2_ppm);
    if (!json_co2_ppm) {
        goto report_out;
    }
    cJSON_AddItemToObject(json_root, "co2_ppm", json_co2_ppm);

    cJSON* json_temp_c = cJSON_CreateNumber(temp_c);
    if (!json_temp_c) {
        goto report_out;
    }
    cJSON_AddItemToObject(json_root, "temp_c", json_temp_c);

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

// As per the spec, the data-ready interval is 5s.
static const sensor_type_t SENSOR_SCD41 = {
    .name = LIBSENSOR_NAME_SCD41,

    .queue_item_size = sizeof(measurement_t),
    .poll_delay_ms = 1000,
    .max_uneventful_iters = 10,

    .dev_start = (sensor_dev_start_fn_t) dev_start,
    .dev_reset = (sensor_dev_reset_fn_t) scd41_reset,

    .poll = (sensor_poll_fn_t) poll,
    .report = (sensor_report_fn_t) report,

    .ctx_init = NULL,
    .ctx_destroy = NULL,
    .recv_json = NULL,
};

// As per the spec, the data-ready interval is 5s.
static const sensor_type_t SENSOR_SCD41_W_UPDATES_PRESS = {
    .name = LIBSENSOR_NAME_SCD41,

    .queue_item_size = sizeof(measurement_t),
    .poll_delay_ms = 1000,
    .max_uneventful_iters = 10,

    .dev_start = (sensor_dev_start_fn_t) dev_start,
    .dev_reset = (sensor_dev_reset_fn_t) scd41_reset,

    .poll = (sensor_poll_fn_t) poll,
    .report = (sensor_report_fn_t) report,

    .ctx_init = (sensor_ctx_init_fn_t) ctx_init,
    .ctx_destroy = (sensor_ctx_destroy_fn_t) ctx_destroy,
    .recv_json = (sensor_recv_json_fn_t) recv_json,
};

esp_err_t libsensor_drv_register_scd41(i2c_port_t port, libsensor_drv_scd41_addr_t addr,
                                       const char* tag, bool enable_updates) {
    return libsensor_drv_register_generic_i2c((i2c_init_fn_t) scd41_init,
                                              enable_updates ? &SENSOR_SCD41_W_UPDATES_PRESS : &SENSOR_SCD41,
                                              port, addr, tag);
}
