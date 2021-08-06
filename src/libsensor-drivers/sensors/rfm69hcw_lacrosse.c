#include <cJSON.h>
#include <device/rfm69hcw.h>
#include <esp_err.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <libcrc.h>
#include <libesp/json.h>
#include <libesp/marshall.h>
#include <libiot.h>
#include <libirq.h>
#include <libtask.h>
#include <stdbool.h>
#include <stdint.h>
#include <sys/cdefs.h>

#include "../common.h"

static const char* TAG = "sensor(rfm69hcw)";

typedef struct dev_info {
    rfm69hcw_handle_t dev;
    gpio_num_t pin_irq;
} dev_info_t;

typedef struct ctx {
    int32_t last_rain;

    libirq_source_handle_t src;
    libirq_waiter_t waiter;

    libtask_loop_handle_t monitor_task;

    EventGroupHandle_t events;
#define CTX_EVENT_MONITOR_STOP (1ULL << 0)
} ctx_t;

#define IRQ_CORE_NUM 1

#define IRQ_TASK_PRIORITY 10
#define IRQ_TASK_STACK_SIZE 4096

#define MONITOR_TASK_PRIORITY 10
#define MONITOR_TASK_STACK_SIZE 2048
#define MONITOR_TASK_DELAY_MS (60 * 1000)

static libtask_disposition_t monitor_trigger_once(ctx_t* ctx) {
    libirq_source_trigger(ctx->src);

    // This call will usually cause us to sleep for `MONITOR_TASK_DELAY_MS` ms,
    // and the event group is used to wake the task up from a sleep of this
    // (potentially quite long) duration early;
    EventBits_t bits =
        xEventGroupWaitBits(ctx->events, CTX_EVENT_MONITOR_STOP, false, false,
                            MONITOR_TASK_DELAY_MS / portTICK_PERIOD_MS);
    if (bits & CTX_EVENT_MONITOR_STOP) {
        return LIBTASK_DISPOSITION_STOP;
    }

    return LIBTASK_DISPOSITION_CONTINUE;
}

static void* ctx_init(dev_info_t* info) {
    ctx_t* ctx = malloc(sizeof(ctx_t));
    ctx->last_rain = -1;

    ESP_ERROR_CHECK(
        libirq_source_create(info->pin_irq, true, IRQ_CORE_NUM, &ctx->src));
    ctx->waiter = libirq_waiter_create(ctx->src);

    ctx->events = xEventGroupCreate();

    // TODO consider measuring FEI

    // This loop task just wakes up the poll task (using
    // `libirq_source_trigger()`) every 60 seconds in order to get it to do a
    // check of the current mode (so that we don't lock up if the RFM69HCW ever
    // spontaneously decides to switch out of RX mode).
    ESP_ERROR_CHECK(
        libtask_loop_spawn((libtask_do_once_fn_t) monitor_trigger_once, ctx,
                           "rfm69hcw_monitor", MONITOR_TASK_STACK_SIZE,
                           MONITOR_TASK_PRIORITY, &ctx->monitor_task));

    return ctx;
}

static void ctx_destroy(ctx_t* ctx) {
    // First we need to stop the monitor task, so that we can destroy the irq
    // source it has a reference to.

    // Mark this monitor task as "should stop", so that it will complete at most
    // one more iteration.
    libtask_loop_mask_should_stop(ctx->monitor_task);

    // Wait the monitor task up from its sleep early.
    xEventGroupSetBits(ctx->events, CTX_EVENT_MONITOR_STOP);

    // Wait for the monitor task to wake due to the message we sent.
    libtask_loop_join(ctx->monitor_task);

    // Now the monitor task has completed, and has been freed.
    libirq_source_destroy(ctx->src);
    libirq_waiter_destroy(ctx->waiter);

    free(ctx);
}

typedef struct lacrosse_payload {
    uint8_t b[6];
} lacrosse_payload_t;

typedef struct lacrosse_packet {
    union {
        struct {
            uint8_t id[3];
            uint8_t status;
            lacrosse_payload_t payload;
        } data;
        uint8_t raw[10];
    };
    uint8_t crc8;
    uint8_t trailer[4];
} __packed lacrosse_packet_t;

typedef struct lacrosse_packet_frame {
    lacrosse_packet_t pkt;
    uint8_t rssi;
} lacrosse_rx_t;

#define MASK_LACROSSE_ID_TYPE 0x00FFF000
#define MASK_LACROSSE_SEQ_NUM 0x0E
#define SHIFT_LACROSSE_ID_TYPE 12
#define SHIFT_LACROSSE_SEQ_NUM 1

#define LACROSSE_ID_TYPE_LTV_RV3 0x70F
#define LACROSSE_ID_TYPE_LTV_WSDTH04 0x88F

#define MIDDLE_BYTE_MAGIC 0xAA

static void check_magic_byte(uint8_t pos, uint8_t b) {
    if (b != 0xAA) {
        libiot_logf_error(TAG, "magic byte (%d) is 0x%02X not 0x%02X!", pos, b,
                          MIDDLE_BYTE_MAGIC);
    }
}

#define RAIN_MM_WARN_MAX (200.0)

static cJSON* handle_lacrosse_ltv_rv3_payload(const char* tag, ctx_t* ctx,
                                              const lacrosse_payload_t* payload,
                                              uint8_t rssi) {
    check_magic_byte(1, payload->b[1]);
    check_magic_byte(4, payload->b[4]);

    double rssi_db = ((double) rssi) * (-0.5);

    uint16_t rain_now;
    uint16_t rain_before;
    marshall_2u8_to_1u16_be_args(&rain_now, payload->b[0], payload->b[2]);
    marshall_2u8_to_1u16_be_args(&rain_before, payload->b[3], payload->b[5]);

    // Note: The initial value of `ctx->last_rain` is -1.
    if (rain_before != ctx->last_rain && ctx->last_rain != -1) {
        libiot_logf_error(TAG,
                          "ltv_rv3: packet skipped! rain was 0x%02X, but last "
                          "reported is 0x%02X",
                          ctx->last_rain, rain_before);
    }

    ctx->last_rain = rain_now;

    // Rain in 1/10ths of an inch.
    uint16_t rain_in = rain_now - rain_before;
    double rain_delta_mm = 0.254 * ((double) rain_in);

    if (rain_delta_mm >= RAIN_MM_WARN_MAX) {
        libiot_logf_error(TAG, "ltv_rv3: too much rain! %lf (0x%X,0x%X)",
                          rain_delta_mm, rain_now, rain_before);
    }

    ESP_LOGI(TAG,
             "(%s) measure: ltv_rv3: delta_rain(mm)=%.1lf (rain_now=0x%03X, "
             "rain_before=0x%03X)",
             tag, rain_delta_mm, rain_now, rain_before);

    cJSON* json_root;
    cJSON_CREATE_ROOT_OBJ_OR_GOTO(&json_root, handle_lacrosse_rv3_payload_out);
    cJSON_INSERT_NUMBER_INTO_OBJ_OR_GOTO(json_root, "rssi", rssi_db,
                                         handle_lacrosse_rv3_payload_out);

    cJSON* json_rain_delta_mm;
    cJSON_INSERT_OBJ_INTO_OBJ_OR_GOTO(json_root, "rain_delta_mm",
                                      &json_rain_delta_mm,
                                      handle_lacrosse_rv3_payload_out);
    cJSON_INSERT_NUMBER_INTO_OBJ_OR_GOTO(json_rain_delta_mm, "value",
                                         rain_delta_mm,
                                         handle_lacrosse_rv3_payload_out);
    cJSON_INSERT_NUMBER_INTO_OBJ_OR_GOTO(json_rain_delta_mm, "raw_now",
                                         rain_now,
                                         handle_lacrosse_rv3_payload_out);
    cJSON_INSERT_NUMBER_INTO_OBJ_OR_GOTO(json_rain_delta_mm, "raw_before",
                                         rain_before,
                                         handle_lacrosse_rv3_payload_out);

    return json_root;

handle_lacrosse_rv3_payload_out:
    libiot_logf_error(TAG, "%s: JSON build fail", __func__);

    // It is safe to call this with `json_root == NULL`.
    cJSON_Delete(json_root);
    return NULL;
}

static cJSON* handle_lacrosse_ltv_wsdth04_payload(
    const char* tag, const lacrosse_payload_t* payload, uint8_t rssi) {
    double rssi_db = ((double) rssi) * (-0.5);

    uint16_t raw_temp;
    uint16_t raw_hum;
    marshall_3u8_to_2u12_be(&raw_temp, &raw_hum, &payload->b[0]);

    uint16_t raw_wind_speed;
    uint16_t raw_wind_dir;
    marshall_3u8_to_2u12_be(&raw_wind_speed, &raw_wind_dir, &payload->b[3]);

    // Temp in degrees Celsius
    double temp_c = (((double) raw_temp) - 400.0) * 0.1;
    // Wind speed in kph
    double wind_speed_kph = ((double) raw_wind_speed) * 0.1;

    ESP_LOGI(TAG,
             "(%s) measure: ltv_wsdth04: temp(degC)=%.1lf, rel_hum=%d%%, "
             "wind_speed(kph)=%.1lf, wind_dir(deg)=%d",
             tag, temp_c, raw_hum, wind_speed_kph, raw_wind_dir);

    cJSON* json_root;
    cJSON_CREATE_ROOT_OBJ_OR_GOTO(&json_root,
                                  handle_lacrosse_breezepro_payload_out);
    cJSON_INSERT_NUMBER_INTO_OBJ_OR_GOTO(json_root, "rssi", rssi_db,
                                         handle_lacrosse_breezepro_payload_out);
    cJSON_INSERT_NUMBER_INTO_OBJ_OR_GOTO(json_root, "temp_c", temp_c,
                                         handle_lacrosse_breezepro_payload_out);
    cJSON_INSERT_NUMBER_INTO_OBJ_OR_GOTO(json_root, "rel_humidity", raw_hum,
                                         handle_lacrosse_breezepro_payload_out);
    cJSON_INSERT_NUMBER_INTO_OBJ_OR_GOTO(json_root, "wind_speed_kph",
                                         wind_speed_kph,
                                         handle_lacrosse_breezepro_payload_out);
    cJSON_INSERT_NUMBER_INTO_OBJ_OR_GOTO(json_root, "wind_dir_deg",
                                         raw_wind_dir,
                                         handle_lacrosse_breezepro_payload_out);

    return json_root;

handle_lacrosse_breezepro_payload_out:
    libiot_logf_error(TAG, "%s: JSON build fail", __func__);

    // It is safe to call this with `json_root == NULL`.
    cJSON_Delete(json_root);
    return NULL;
}

static cJSON* report(const char* tag, dev_info_t* info, ctx_t* ctx,
                     lacrosse_rx_t* frame) {
    const lacrosse_packet_t* pkt = &frame->pkt;

    uint32_t id;
    marshall_3u8_to_1u24_be(&id, pkt->data.id);

    uint8_t seq_num =
        (pkt->data.status & MASK_LACROSSE_SEQ_NUM) >> SHIFT_LACROSSE_SEQ_NUM;
    uint8_t status = pkt->data.status & ~MASK_LACROSSE_SEQ_NUM;

    uint8_t true_crc8 = crc8_calc_lacrosse(pkt->raw, sizeof(pkt->raw));
    bool crc8_valid = pkt->crc8 == true_crc8;

    ESP_LOGI(TAG,
             "(%s) measure: packet(rssi=0x%02X): id=0x%06X, seq=%d, "
             "status=0x%02X, crc=%s (0x%02X vs 0x%02X)",
             tag, frame->rssi, id, seq_num, status, crc8_valid ? "OK" : "BAD",
             pkt->crc8, true_crc8);

    cJSON* json = NULL;

    if (!crc8_valid) {
        ESP_LOGW(TAG, "bad crc!");
        goto report_out;
    }

    switch ((id & MASK_LACROSSE_ID_TYPE) >> SHIFT_LACROSSE_ID_TYPE) {
        case LACROSSE_ID_TYPE_LTV_RV3: {
            json = handle_lacrosse_ltv_rv3_payload(tag, ctx, &pkt->data.payload,
                                                   frame->rssi);
            break;
        }
        case LACROSSE_ID_TYPE_LTV_WSDTH04: {
            json = handle_lacrosse_ltv_wsdth04_payload(tag, &pkt->data.payload,
                                                       frame->rssi);
            break;
        }
        default: {
            libiot_logf_error(TAG,
                              "well-formed packet with unknown id type: 0x%X",
                              id);
            break;
        }
    }

report_out:
    free(frame);
    return json;
}

static void handle_payload_ready(rfm69hcw_handle_t dev,
                                 sensor_output_handle_t output) {
    lacrosse_rx_t* frame = malloc(sizeof(lacrosse_rx_t));

    // Must read RSSI_VALUE before emptying the FIFO, since this will cause an
    // RX restart and we get a race.
    ESP_ERROR_CHECK(
        rfm69hcw_reg_read(dev, RFM69HCW_REG_RSSI_VALUE, &frame->rssi));
    // Note that due to an idiosyncrasy with the RFM69HCW when
    // `RFM69HCW_AFC_FEI_AFC_AUTOCLEAR_ON` is set then at this point (i.e. even
    // before the FIFO has been emptied) the `RFM69HCW_REG_AFC_MSB/LSB`
    // registers have already been cleared to zero. However, if the packet never
    // arrives and the timeout IRQ triggers then the AFC value is safe to read.
    //
    // If we desperately want to know the AFC value then we can disable AFC
    // autoclear and perform the clear ourselves as part of the RSSI IRQ
    // interrupt handler.

    // Read out the FIFO contents.
    uint8_t* buff = (uint8_t*) &frame->pkt;
    for (int i = 0; i < sizeof(lacrosse_packet_t); i++) {
        ESP_ERROR_CHECK(rfm69hcw_reg_read(dev, RFM69HCW_REG_FIFO, buff + i));
    }

    libsensor_output_item(output, frame);
}

#define MAX_WAIT_ITERS 10

static sensor_poll_result_t poll(const char* tag, dev_info_t* info, ctx_t* ctx,
                                 sensor_output_handle_t output) {
    // Wait for RSSI IRQ
    if (!libirq_waiter_sleep_until_active(ctx->waiter)) {
        abort();
    }

    rfm69hcw_handle_t dev = info->dev;

    uint8_t op_mode;
    ESP_ERROR_CHECK(rfm69hcw_reg_read(dev, RFM69HCW_REG_OP_MODE, &op_mode));
    if ((op_mode & MASK_RFM69HCW_OP_MODE_MODE) != RFM69HCW_OP_MODE_MODE_RX) {
        uint8_t irq_flags_1;
        uint8_t irq_flags_2;

        ESP_ERROR_CHECK(
            rfm69hcw_reg_read(dev, RFM69HCW_REG_IRQ_FLAGS_1, &irq_flags_1));
        ESP_ERROR_CHECK(
            rfm69hcw_reg_read(dev, RFM69HCW_REG_IRQ_FLAGS_2, &irq_flags_2));

        libiot_logf_error(
            TAG,
            "state error: bad op mode (not rx), fixing! (0x%02X,0x%02X,0x%02X)",
            op_mode, irq_flags_1, irq_flags_2);

        return SENSOR_POLL_RESULT_FAIL;
    }

    const char* fail_msg = NULL;

    int i;

    for (i = 0; i < MAX_WAIT_ITERS; i++) {
        uint8_t irq_flags_2;
        ESP_ERROR_CHECK(
            rfm69hcw_reg_read(dev, RFM69HCW_REG_IRQ_FLAGS_2, &irq_flags_2));

        // Has a full packet been recieved?
        if (irq_flags_2 & RFM69HCW_IRQ_FLAGS_2_PAYLOAD_READY) {
            handle_payload_ready(dev, output);
            goto handle_rssi_irq_out;
        }

        uint8_t irq_flags_1;
        ESP_ERROR_CHECK(
            rfm69hcw_reg_read(dev, RFM69HCW_REG_IRQ_FLAGS_1, &irq_flags_1));

        // Is this interrupt spurious?
        if (!(irq_flags_1 & RFM69HCW_IRQ_FLAGS_1_RSSI)) {
            // Spurious IRQ, so just ignore it.
            goto handle_rssi_irq_out;
        }

        // Have we timed out waiting for a packet?
        if (irq_flags_1 & RFM69HCW_IRQ_FLAGS_1_TIMEOUT) {
            break;
        }

        vTaskDelay(1);
    }

    if (i >= MAX_WAIT_ITERS) {
        fail_msg = "max iters exceeded after RSSI sampling!";
        goto handle_rssi_irq_out;
    }

    uint8_t rssi;
    uint8_t afc_value_msb;
    uint8_t afc_value_lsb;

    ESP_ERROR_CHECK(rfm69hcw_reg_read(dev, RFM69HCW_REG_RSSI_VALUE, &rssi));
    ESP_ERROR_CHECK(
        rfm69hcw_reg_read(dev, RFM69HCW_REG_AFC_MSB, &afc_value_msb));
    ESP_ERROR_CHECK(
        rfm69hcw_reg_read(dev, RFM69HCW_REG_AFC_LSB, &afc_value_lsb));

    uint16_t afc_value =
        (((uint16_t) afc_value_msb) << 8) | (((uint16_t) afc_value_lsb) << 0);

    uint8_t pkt_cfg_2;
    ESP_ERROR_CHECK(
        rfm69hcw_reg_read(dev, RFM69HCW_REG_PACKET_CONFIG_2, &pkt_cfg_2));
    ESP_ERROR_CHECK(
        rfm69hcw_reg_write(dev, RFM69HCW_REG_PACKET_CONFIG_2,
                           pkt_cfg_2 | RFM69HCW_PACKET_CONFIG_2_RESTART_RX));

    for (i = 0; i < MAX_WAIT_ITERS; i++) {
        uint8_t irq_flags_1;
        ESP_ERROR_CHECK(
            rfm69hcw_reg_read(dev, RFM69HCW_REG_IRQ_FLAGS_1, &irq_flags_1));
        if (!(irq_flags_1 & RFM69HCW_IRQ_FLAGS_1_TIMEOUT)) {
            break;
        }

        vTaskDelay(1);
    }

    ESP_LOGI(TAG, "rssi timeout, restarting rx (rssi=0x%02X, afc_value=0x%04X)",
             rssi, afc_value);

    if (i >= MAX_WAIT_ITERS) {
        fail_msg = "max iters exceeded after rx restart!";
        goto handle_rssi_irq_out;
    }

handle_rssi_irq_out:
    if (fail_msg) {
        uint8_t irq_flags_1;
        uint8_t irq_flags_2;

        ESP_ERROR_CHECK(
            rfm69hcw_reg_read(dev, RFM69HCW_REG_IRQ_FLAGS_1, &irq_flags_1));
        ESP_ERROR_CHECK(
            rfm69hcw_reg_read(dev, RFM69HCW_REG_IRQ_FLAGS_2, &irq_flags_2));

        libiot_logf_error(TAG, "%s (0x%02X,0x%02X,0x%02X)", fail_msg, op_mode,
                          irq_flags_1, irq_flags_2);
    }

    return SENSOR_POLL_RESULT_MADE_PROGRESS;
}

static const rfm69hcw_rx_config_t RFM69HCW_CFG = {
    .data_mode = RFM69HCW_DATA_MODE_PACKET_MODE,
    .type = RFM69HCW_MODULATION_TYPE_FSK,
    .fsk_shaping = RFM69HCW_MODULATION_SHAPING_FSK_NONE,

    .freq_khz = 915000,
    .bit_period_ns = 106842,
    .rx_bw = RFM69HCW_RX_BW_100_kHz,
    .dcc_cutoff = RFM69HCW_DCC_CUTOFF_8_PERCENT,

    .sync_bit_tol = 1,
    .sync_value = {0xD2, 0xAA, 0x2D, 0xD4, 0x00},

    .payload_len = sizeof(lacrosse_packet_t),

    .rssi_thresh = 0xC0,         // -96dBm
    .inter_packet_rx_delay = 6,  // ~6.85ms
    .timeout_rssi_thresh = 30,   // ~50ms
};

static void dev_start(dev_info_t* info, ctx_t* ctx) {
    ESP_ERROR_CHECK(rfm69hcw_configure_rx(info->dev, &RFM69HCW_CFG));
}

static void dev_destroy(dev_info_t* info) {
    rfm69hcw_destroy(info->dev);
    free(info);
}

static sensor_type_t SENSOR_RFM69HCW_LACROSSE = {
    .model = LIBSENSOR_NAME_RFM69HCW_LACROSSE,

    .poll_delay_ms = 0,
    .max_uneventful_iters = 0,

    .poll_task_stack_size = 2048,
    .report_task_stack_size = 2560,

    .dev_destroy = (sensor_dev_destroy_fn_t) dev_destroy,
    .dev_start = (sensor_dev_start_fn_t) dev_start,

    .poll = (sensor_poll_fn_t) poll,
    .report = (sensor_report_fn_t) report,

    .ctx_init = (sensor_ctx_init_fn_t) ctx_init,
    .ctx_destroy = (sensor_ctx_destroy_fn_t) ctx_destroy,
};

esp_err_t libsensor_drv_register_rfm69hcw_lacrosse(spi_host_device_t host,
                                                   gpio_num_t pin_cs,
                                                   gpio_num_t pin_rst,
                                                   gpio_num_t pin_irq,
                                                   const char* tag) {
    esp_err_t ret;

    dev_info_t* info = malloc(sizeof(dev_info_t));
    info->pin_irq = pin_irq;

    // Configure the ESP32 to communicate with the RFM69HCW on `host`.
    ret = rfm69hcw_init(host, pin_cs, pin_rst, &info->dev);
    if (ret != ESP_OK) {
        free(info);
        return ret;
    }

    // Note that when `NULL` is passed, on failure libsensor calls
    // `type->dev_destroy()` on `dev`, consuming it.
    return libsensor_register(&SENSOR_RFM69HCW_LACROSSE, tag, info, NULL);
}
