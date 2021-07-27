#include <cJSON.h>
#include <device/ccs811.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/queue.h>
#include <libiot.h>
#include <libsensor.h>
#include <math.h>

#include "../common.h"

static const char* TAG = "sensor(ccs811)";

#define FIRST_UPDATE_WAIT_MS 30000

typedef struct ctx {
    EventGroupHandle_t events;
#define CTX_EVENT_APP_STARTED (1ULL << 0)
#define CTX_EVENT_UPDATED_AT_LEAST_ONCE (1ULL << 1)
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

static void recv_json(const char* tag, ccs811_handle_t dev, ctx_t* ctx, const cJSON* json) {
    cJSON* json_temp_c = cJSON_GetObjectItem(json, "temp_c");
    if (!json_temp_c || !cJSON_IsNumber(json_temp_c)) {
        libiot_logf_error(TAG, "JSON message missing 'json_temp_c' or not a number");
        return;
    }

    cJSON* json_rel_humidity = cJSON_GetObjectItem(json, "rel_humidity");
    if (!json_rel_humidity || !cJSON_IsNumber(json_rel_humidity)) {
        libiot_logf_error(TAG, "JSON message missing 'rel_humidity' or not a number");
        return;
    }

    double temp_c = json_temp_c->valuedouble;
    double rel_humidity = json_rel_humidity->valuedouble;

    if (temp_c < -25.0) {
        libiot_logf_error(TAG, "'temp_c' out of range: too low");
        temp_c = -25.0;
    }

    if (temp_c > 100.0) {
        libiot_logf_error(TAG, "'temp_c' out of range: too high");
        temp_c = 100.0;
    }

    if (rel_humidity < 0.0) {
        libiot_logf_error(TAG, "'rel_humidity' out of range: too low");
        rel_humidity = 0.0;
    }

    if (rel_humidity > 100.0) {
        libiot_logf_error(TAG, "'rel_humidity' out of range: too high");
        rel_humidity = 100.0;
    }

    uint32_t val_hum = round(rel_humidity * 512.0);
    uint32_t val_temp = round((temp_c + 25.0) * 512.0);

    assert(val_hum <= UINT16_MAX);
    assert(val_temp <= UINT16_MAX);

    uint8_t data[4];
    data[0] = (val_hum & 0xFF00) >> 8;
    data[1] = (val_hum & 0x00FF) >> 0;
    data[2] = (val_temp & 0xFF00) >> 8;
    data[3] = (val_temp & 0x00FF) >> 0;

    if (!(xEventGroupGetBits(ctx->events) & CTX_EVENT_APP_STARTED)) {
        // The chip isn't in the APP mode yet, so drop this message.
        return;
    }

    ESP_LOGI(TAG, "(%s) update: env_data=0x%02X%02X%02X%02X", tag, data[0], data[1], data[2], data[3]);
    ccs811_reg_write(dev, CCS811_REG_ENV_DATA, data, 4);

    xEventGroupSetBits(ctx->events, CTX_EVENT_UPDATED_AT_LEAST_ONCE);
}

static void dev_start(ccs811_handle_t dev, ctx_t* ctx) {
    ccs811_start_app(dev);

    const uint8_t reg_meas_mode = CCS811_MEAS_MODE_DRIVE_MODE_1;
    ccs811_reg_write(dev, CCS811_REG_MEAS_MODE, &reg_meas_mode, 1);

    if (ctx) {
        xEventGroupSetBits(ctx->events, CTX_EVENT_APP_STARTED);

        EventBits_t result = xEventGroupWaitBits(ctx->events, CTX_EVENT_UPDATED_AT_LEAST_ONCE, false, false, FIRST_UPDATE_WAIT_MS / portTICK_PERIOD_MS);
        if (!(result & CTX_EVENT_UPDATED_AT_LEAST_ONCE)) {
            libiot_logf_error(TAG, "timed out waiting for first external update message, starting measurement anyway");
        }
    }
}

typedef struct measurement {
    uint16_t eco2_ppm;
    uint16_t etvoc_ppb;
    uint16_t raw_data;
} measurement_t;

static sensor_poll_result_t poll(const char* tag, ccs811_handle_t dev, QueueHandle_t queue) {
    uint8_t reg_status;
    ccs811_reg_read(dev, CCS811_REG_STATUS, &reg_status, 1);
    if (!(reg_status & CCS811_STATUS_DATA_READY)) {
        return SENSOR_POLL_RESULT_UNEVENTFUL;
    }

    uint8_t error_id;
    measurement_t meas;
    ccs811_read_alg_result_data(dev, &meas.eco2_ppm, &meas.etvoc_ppb, NULL, &error_id, &meas.raw_data);

    if (error_id) {
        libiot_logf_error(TAG, "(%s) error! (0x%02X)", tag, error_id);
        return SENSOR_POLL_RESULT_FAIL;
    }

    if (xQueueSend(queue, &meas, 0) != pdTRUE) {
        libiot_logf_error(TAG, "can't queue result");
    }

    return SENSOR_POLL_RESULT_MADE_PROGRESS;
}

static cJSON* report(const char* tag, ccs811_handle_t dev, measurement_t* meas) {
    ESP_LOGI(TAG, "(%s) measure: eCO2(ppm)=%u, eTVOC(ppb)=%u, raw_data=0x%04X", tag, meas->eco2_ppm, meas->etvoc_ppb, meas->raw_data);

    cJSON* json_root = cJSON_CreateObject();
    if (!json_root) {
        goto report_out;
    }

    cJSON* json_eco2_ppm = cJSON_CreateNumber(meas->eco2_ppm);
    if (!json_eco2_ppm) {
        goto report_out;
    }
    cJSON_AddItemToObject(json_root, "eco2_ppm", json_eco2_ppm);

    cJSON* json_etvoc_ppb = cJSON_CreateNumber(meas->etvoc_ppb);
    if (!json_etvoc_ppb) {
        goto report_out;
    }
    cJSON_AddItemToObject(json_root, "etvoc_ppb", json_etvoc_ppb);

    return json_root;

report_out:
    libiot_logf_error(TAG, "%s: JSON build fail", __func__);

    // It is safe to call this with `json_root == NULL`.
    cJSON_Delete(json_root);
    return NULL;
}

static const sensor_type_t SENSOR_CCS811 = {
    .name = LIBSENSOR_NAME_CCS811,

    .queue_item_size = sizeof(measurement_t),
    .poll_delay_ms = 250,
    // The device has a several second startup time.
    .initial_max_uneventful_iters = 40,
    .max_uneventful_iters = 10,

    .dev_start = (sensor_dev_start_fn_t) dev_start,
    .dev_reset = (sensor_dev_reset_fn_t) ccs811_reset,

    .poll = (sensor_poll_fn_t) poll,
    .report = (sensor_report_fn_t) report,
};

static const sensor_type_t SENSOR_CCS811_W_UPDATES_TEMP_AND_HUM = {
    .name = LIBSENSOR_NAME_CCS811,

    .queue_item_size = sizeof(measurement_t),
    .poll_delay_ms = 250,
    // The device has a several second startup time.
    .initial_max_uneventful_iters = 40,
    .max_uneventful_iters = 10,

    .dev_start = (sensor_dev_start_fn_t) dev_start,
    .dev_reset = (sensor_dev_reset_fn_t) ccs811_reset,

    .poll = (sensor_poll_fn_t) poll,
    .report = (sensor_report_fn_t) report,

    .ctx_init = (sensor_ctx_init_fn_t) ctx_init,
    .ctx_destroy = (sensor_ctx_destroy_fn_t) ctx_destroy,
    .recv_json = (sensor_recv_json_fn_t) recv_json,
};

esp_err_t libsensor_drv_register_ccs811(i2c_port_t port, libsensor_drv_ccs811_addr_t addr,
                                        const char* tag, bool enable_updates) {
    return libsensor_drv_register_generic_i2c((i2c_init_fn_t) ccs811_init,
                                              enable_updates ? &SENSOR_CCS811_W_UPDATES_TEMP_AND_HUM : &SENSOR_CCS811,
                                              port, addr, tag);
}
