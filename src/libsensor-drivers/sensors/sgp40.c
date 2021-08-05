#include <cJSON.h>
#include <device/sgp40.h>
#include <esp_err.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/semphr.h>
#include <libesp.h>
#include <libesp/json.h>
#include <libiot.h>
#include <libsensor.h>
#include <math.h>

#include "../common.h"

static const char* TAG = "sensor(sgp40)";

#define FIRST_UPDATE_WAIT_MS 30000
#define INITIAL_DELAY_MS (150 * 1000)

typedef struct ctx {
    int64_t start_time_us;
    sgp40_voc_algorithm_ctx_t voc_ctx;

    EventGroupHandle_t events;
#define CTX_EVENT_UPDATED_AT_LEAST_ONCE (1ULL << 0)

    SemaphoreHandle_t lock;
    uint16_t raw_compensation_data[2];
} ctx_t;

static ctx_t* ctx_init(sgp40_handle_t dev) {
    ctx_t* ctx = malloc(sizeof(ctx_t));
    ctx->events = xEventGroupCreate();
    ctx->lock = xSemaphoreCreateMutex();
    ctx->raw_compensation_data[0] = SGP40_COMPENSATION_DEFAULT_RAW_HUMIDITY;
    ctx->raw_compensation_data[1] = SGP40_COMPENSATION_DEFAULT_RAW_TEMP;

    sgp40_voc_algorithm_ctx_init(&ctx->voc_ctx);

    return ctx;
}

static void ctx_destroy(ctx_t* ctx) {
    vEventGroupDelete(ctx->events);
    vSemaphoreDelete(ctx->lock);

    sgp40_voc_algorithm_ctx_destroy(ctx->voc_ctx);

    free(ctx);
}

static void recv_json(const char* tag, sgp40_handle_t dev, ctx_t* ctx,
                      const cJSON* json) {
    cJSON* json_temp_c = cJSON_GetObjectItem(json, "temp_c");
    if (!json_temp_c || !cJSON_IsNumber(json_temp_c)) {
        libiot_logf_error(TAG,
                          "JSON message missing 'json_temp_c' or not a number");
        return;
    }

    cJSON* json_rel_humidity = cJSON_GetObjectItem(json, "rel_humidity");
    if (!json_rel_humidity || !cJSON_IsNumber(json_rel_humidity)) {
        libiot_logf_error(
            TAG, "JSON message missing 'rel_humidity' or not a number");
        return;
    }

    double temp_c = json_temp_c->valuedouble;
    double rel_humidity = json_rel_humidity->valuedouble;

    if (temp_c < -45.0) {
        libiot_logf_error(TAG, "'temp_c' out of range: too low");
        temp_c = -45.0;
    }

    if (temp_c > 130.0) {
        libiot_logf_error(TAG, "'temp_c' out of range: too high");
        temp_c = 130.0;
    }

    if (rel_humidity < 0.0) {
        libiot_logf_error(TAG, "'rel_humidity' out of range: too low");
        rel_humidity = 0.0;
    }

    if (rel_humidity > 100.0) {
        libiot_logf_error(TAG, "'rel_humidity' out of range: too high");
        rel_humidity = 100.0;
    }

    uint16_t raw_hum;
    sgp40_encode_hum(rel_humidity, &raw_hum);

    uint16_t raw_temp;
    sgp40_encode_temp(temp_c, &raw_temp);

    ESP_LOGI(TAG, "(%s) update: raw_hum=0x%04X, raw_temp=0x%04X", tag, raw_hum,
             raw_temp);

    while (xSemaphoreTake(ctx->lock, portMAX_DELAY) != pdTRUE)
        ;

    ctx->raw_compensation_data[0] = raw_hum;
    ctx->raw_compensation_data[1] = raw_temp;

    xSemaphoreGive(ctx->lock);

    xEventGroupSetBits(ctx->events, CTX_EVENT_UPDATED_AT_LEAST_ONCE);
}

static void dev_start(sgp40_handle_t dev, ctx_t* ctx) {
    uint16_t raw_compensation_data[2] = {
        SGP40_COMPENSATION_DEFAULT_RAW_HUMIDITY,
        SGP40_COMPENSATION_DEFAULT_RAW_TEMP,
    };

    // As per spec, a first `MEASURE_RAW` command turns the heater permanently
    // on, to start the continuous measurement mode. We use the default
    // compensation values for this first call in order to prevent a data race.
    uint16_t raw;
    ESP_ERROR_CHECK(sgp40_cmd_readwrite(dev, &SGP40_CMD_MEASURE_RAW,
                                        raw_compensation_data, 2, &raw, 1));

    // Next we perform a self test.
    uint16_t self_test_result;
    ESP_ERROR_CHECK(
        sgp40_cmd_read(dev, &SGP40_CMD_MEASURE_TEST, &self_test_result, 1));

    switch (self_test_result) {
        case SGP40_MEASURE_TEST_PASS: {
            // Test pass
            break;
        }
        case SGP40_MEASURE_TEST_FAIL: {
            libiot_logf_error(TAG, "self-test failed!");
            break;
        }
        default: {
            libiot_logf_error(TAG, "self-test returned unknown code! (0x%04X)",
                              self_test_result);
            break;
        }
    }

    if (ctx) {
        EventBits_t result =
            xEventGroupWaitBits(ctx->events, CTX_EVENT_UPDATED_AT_LEAST_ONCE,
                                false, false,
                                FIRST_UPDATE_WAIT_MS / portTICK_PERIOD_MS);
        if (!(result & CTX_EVENT_UPDATED_AT_LEAST_ONCE)) {
            libiot_logf_error(TAG,
                              "timed out waiting for first external update "
                              "message, starting measurement anyway");
        }
    }

    // We can't just wait here for the `INITIAL_DELAY_MS` in order for the
    // sensor to have time to stabilize, since the VOC algorithm is processed on
    // our-side. Instead we drop the first `INITIAL_DELAY_MS`-worth of
    // measurements in `poll()`, but still pass them through to the voc
    // algorithm core.
    ctx->start_time_us = esp_timer_get_time();
}

static sensor_poll_result_t poll(const char* tag, sgp40_handle_t dev,
                                 ctx_t* ctx, sensor_output_handle_t output) {
    while (xSemaphoreTake(ctx->lock, portMAX_DELAY) != pdTRUE)
        ;

    uint16_t raw_compensation_data[2];
    memcpy(raw_compensation_data, ctx->raw_compensation_data,
           sizeof(raw_compensation_data));

    xSemaphoreGive(ctx->lock);

    uint16_t sraw;
    ESP_ERROR_CHECK(sgp40_cmd_readwrite(dev, &SGP40_CMD_MEASURE_RAW,
                                        ctx->raw_compensation_data, 2, &sraw,
                                        1));

    libsensor_output_item(output, (void*) (uint32_t) sraw);
    return SENSOR_POLL_RESULT_MADE_PROGRESS;
}

static cJSON* report(const char* tag, sgp40_handle_t dev, ctx_t* ctx,
                     void* ptr_sraw) {
    uint16_t sraw = (uint16_t) (uint32_t) ptr_sraw;

    int32_t voc_index;
    sgp40_voc_algorithm_ctx_process(ctx->voc_ctx, sraw, &voc_index);

    // If we are still within the start-up grace period, waiting for the
    // algorithm results to stabilize, then do not report the measurement.
    if (esp_timer_get_time() - ctx->start_time_us < INITIAL_DELAY_MS * 1000) {
        return NULL;
    }

    ESP_LOGI(TAG, "(%s) measure: voc_index(0-500)=%d, raw=0x%04X", tag,
             voc_index, sraw);

    cJSON* json_root;
    cJSON_CREATE_ROOT_OBJ_OR_GOTO(&json_root, report_fail);

    cJSON* json_voc_index;
    cJSON_INSERT_OBJ_INTO_OBJ_OR_GOTO(json_root, "voc_index", &json_voc_index,
                                      report_fail);
    cJSON_INSERT_NUMBER_INTO_OBJ_OR_GOTO(json_voc_index, "value", voc_index,
                                         report_fail);
    cJSON_INSERT_NUMBER_INTO_OBJ_OR_GOTO(json_voc_index, "sraw", sraw,
                                         report_fail);

    return json_root;

report_fail:
    libiot_logf_error(TAG, "%s: JSON build fail", __func__);

    // It is safe to call this with `json_root == NULL`.
    cJSON_Delete(json_root);
    return NULL;
}

// As per the spec for the VOC algorithm, the sample rate should be 1Hz.
// (Note that there is no "continuous sample" mode for the SGP40, and each
// measurement must be triggered manually.)
static const sensor_type_t SENSOR_SGP40 = {
    .model = LIBSENSOR_NAME_SGP40,

    .poll_delay_ms = 1000,
    .max_uneventful_iters = 0,

    .poll_task_stack_size = 2048,
    .report_task_stack_size = 2560,

    .dev_destroy = (sensor_dev_destroy_fn_t) sgp40_destroy,
    .dev_start = (sensor_dev_start_fn_t) dev_start,

    .poll = (sensor_poll_fn_t) poll,
    .report = (sensor_report_fn_t) report,
};

// As per the spec for the VOC algorithm, the sample rate should be 1Hz.
// (Note that there is no "continuous sample" mode for the SGP40, and each
// measurement must be triggered manually.)
static const sensor_type_t SENSOR_SGP40_W_UPDATES_PRESS = {
    .model = LIBSENSOR_NAME_SGP40,

    .poll_delay_ms = 1000,
    .max_uneventful_iters = 0,

    .poll_task_stack_size = 2048,
    .report_task_stack_size = 2560,

    .dev_destroy = (sensor_dev_destroy_fn_t) sgp40_destroy,
    .dev_start = (sensor_dev_start_fn_t) dev_start,

    .poll = (sensor_poll_fn_t) poll,
    .report = (sensor_report_fn_t) report,

    .ctx_init = (sensor_ctx_init_fn_t) ctx_init,
    .ctx_destroy = (sensor_ctx_destroy_fn_t) ctx_destroy,
    .recv_json = (sensor_recv_json_fn_t) recv_json,
};

esp_err_t libsensor_drv_register_sgp40(i2c_port_t port,
                                       libsensor_drv_sgp40_addr_t addr,
                                       const char* tag, bool enable_updates) {
    return libsensor_drv_register_generic_i2c(
        (i2c_init_fn_t) sgp40_init,
        enable_updates ? &SENSOR_SGP40_W_UPDATES_PRESS : &SENSOR_SGP40, port,
        addr, tag);
}
