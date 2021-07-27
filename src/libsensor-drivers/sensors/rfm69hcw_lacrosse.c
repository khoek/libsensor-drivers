#include <cJSON.h>
#include <device/rfm69hcw.h>
#include <esp_log.h>
#include <libcrc.h>
#include <libiot.h>
#include <libirq.h>
#include <libtask.h>
#include <stdbool.h>
#include <stdint.h>

#include "../common.h"

static const char* TAG = "sensor(rfm69hcw)";

typedef struct rfm69hcw_w_irq {
    rfm69hcw_handle_t dev;

    libirq_source_handle_t src;
    libirq_waiter_t waiter;

    libtask_loop_handle_t monitor_task;
} rfm69hcw_w_irq_t;

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
} __attribute__((packed)) lacrosse_packet_t;

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

static inline void convert_3b8_to_1b24(uint32_t* v, uint8_t o1, uint8_t o2, uint8_t o3) {
    *v = (((uint32_t) o1) << 16) | (((uint32_t) o2) << 8) | (((uint32_t) o3) << 0);
}

static inline void convert_2b8_to_1b16(uint16_t* v, uint8_t o1, uint8_t o2) {
    *v = (((uint32_t) o1) << 8) | (((uint32_t) o2) << 0);
}

static inline void convert_3b8_to_2b16(uint16_t* v1, uint16_t* v2, uint8_t o1, uint8_t o2, uint8_t o3) {
    *v1 = (((uint16_t) o1) << 4) | (((uint16_t) o2) >> 4);
    *v2 = (((uint16_t) o2 & 0x0F) << 8) | (((uint16_t) o3) >> 0);
}

#define MIDDLE_BYTE_MAGIC 0xAA

static void check_magic_byte(uint8_t pos, uint8_t b) {
    if (b != 0xAA) {
        libiot_logf_error(TAG, "magic byte (%d) is 0x%02X not 0x%02X!", pos, b, MIDDLE_BYTE_MAGIC);
    }
}

#define RAIN_MM_WARN_MAX (200.0)

static int32_t last_rain = -1;

static cJSON* handle_lacrosse_ltv_rv3_payload(const char* tag, const lacrosse_payload_t* payload, uint8_t rssi) {
    check_magic_byte(1, payload->b[1]);
    check_magic_byte(4, payload->b[4]);

    uint16_t rain_now;
    uint16_t rain_before;
    convert_2b8_to_1b16(&rain_now, payload->b[0], payload->b[2]);
    convert_2b8_to_1b16(&rain_before, payload->b[3], payload->b[5]);

    if (rain_before != last_rain && last_rain != -1) {
        libiot_logf_error(TAG, "ltv_rv3: packet skipped! rain was 0x%02X, but last reported is 0x%02X", last_rain, rain_before);
    }

    last_rain = rain_now;

    // FIXME detect skips

    // Rain in 1/10ths of an inch.
    uint16_t rain_in = rain_now - rain_before;
    double delta_rain_mm = 0.254 * ((double) rain_in);

    if (delta_rain_mm >= RAIN_MM_WARN_MAX) {
        libiot_logf_error(TAG, "ltv_rv3: too much rain! %lf (0x%X,0x%X)", delta_rain_mm, rain_now, rain_before);
    }

    ESP_LOGI(TAG, "(%s) measure: ltv_rv3: delta_rain(mm)=%.1lf (rain_now=0x%03X, rain_before=0x%03X)", tag, delta_rain_mm, rain_now, rain_before);

    cJSON* json_root = cJSON_CreateObject();
    if (!json_root) {
        goto handle_lacrosse_rv3_payload_out;
    }

    cJSON* json_delta_mm = cJSON_CreateNumber(delta_rain_mm);
    if (!json_delta_mm) {
        goto handle_lacrosse_rv3_payload_out;
    }
    cJSON_AddItemToObject(json_root, "delta_mm", json_delta_mm);

    cJSON* json_raw = cJSON_CreateObject();
    if (!json_raw) {
        goto handle_lacrosse_rv3_payload_out;
    }
    cJSON_AddItemToObject(json_root, "raw", json_raw);

    cJSON* json_rain_now = cJSON_CreateNumber(rain_now);
    if (!json_rain_now) {
        goto handle_lacrosse_rv3_payload_out;
    }
    cJSON_AddItemToObject(json_raw, "rain_now", json_rain_now);

    cJSON* json_rain_before = cJSON_CreateNumber(rain_before);
    if (!json_rain_before) {
        goto handle_lacrosse_rv3_payload_out;
    }
    cJSON_AddItemToObject(json_raw, "rain_before", json_rain_before);

    cJSON* json_rssi = cJSON_CreateNumber(((double) rssi) * (-0.5));
    if (!json_rssi) {
        goto handle_lacrosse_rv3_payload_out;
    }
    cJSON_AddItemToObject(json_root, "rssi", json_rssi);

    return json_root;

handle_lacrosse_rv3_payload_out:
    libiot_logf_error(TAG, "%s: JSON build fail", __func__);

    // It is safe to call this with `json_root == NULL`.
    cJSON_Delete(json_root);
    return NULL;
}

static cJSON* handle_lacrosse_ltv_wsdth04_payload(const char* tag, const lacrosse_payload_t* payload, uint8_t rssi) {
    uint16_t raw_temp;
    uint16_t raw_hum;
    convert_3b8_to_2b16(&raw_temp, &raw_hum, payload->b[0], payload->b[1], payload->b[2]);

    uint16_t raw_wind_speed;
    uint16_t raw_wind_dir;
    convert_3b8_to_2b16(&raw_wind_speed, &raw_wind_dir, payload->b[3], payload->b[4], payload->b[5]);

    // Temp in degrees Celsius
    double temp = ((double) (raw_temp - 400)) * 0.1;
    // Wind speed in kph
    double wind_speed = ((double) raw_wind_speed) * 0.1;

    ESP_LOGI(TAG, "(%s) measure: ltv_wsdth04: temp(degC)=%.1lf, rel_hum=%d%%, wind_speed(kph)=%.1lf, wind_dir(deg)=%d", tag, temp, raw_hum, wind_speed, raw_wind_dir);

    cJSON* json_root = cJSON_CreateObject();
    if (!json_root) {
        goto handle_lacrosse_breezepro_payload_out;
    }

    cJSON* json_temp_c = cJSON_CreateNumber(temp);
    if (!json_temp_c) {
        goto handle_lacrosse_breezepro_payload_out;
    }
    cJSON_AddItemToObject(json_root, "temp_c", json_temp_c);

    cJSON* json_rel_humidity = cJSON_CreateNumber(raw_hum);
    if (!json_rel_humidity) {
        goto handle_lacrosse_breezepro_payload_out;
    }
    cJSON_AddItemToObject(json_root, "rel_humidity", json_rel_humidity);

    cJSON* json_wind_speed = cJSON_CreateNumber(wind_speed);
    if (!json_wind_speed) {
        goto handle_lacrosse_breezepro_payload_out;
    }
    cJSON_AddItemToObject(json_root, "wind_speed_kph", json_wind_speed);

    cJSON* json_wind_dir = cJSON_CreateNumber(raw_wind_dir);
    if (!json_wind_dir) {
        goto handle_lacrosse_breezepro_payload_out;
    }
    cJSON_AddItemToObject(json_root, "wind_dir_deg", json_wind_dir);

    cJSON* json_rssi = cJSON_CreateNumber(((double) rssi) * (-0.5));
    if (!json_rssi) {
        goto handle_lacrosse_breezepro_payload_out;
    }
    cJSON_AddItemToObject(json_root, "rssi", json_rssi);

    return json_root;

handle_lacrosse_breezepro_payload_out:
    libiot_logf_error(TAG, "%s: JSON build fail", __func__);

    // It is safe to call this with `json_root == NULL`.
    cJSON_Delete(json_root);
    return NULL;
}

static cJSON* report(const char* tag, rfm69hcw_w_irq_t* data, const lacrosse_rx_t* frame) {
    const lacrosse_packet_t* pkt = &frame->pkt;

    uint32_t id;
    convert_3b8_to_1b24(&id, pkt->data.id[0], pkt->data.id[1], pkt->data.id[2]);

    uint8_t seq_num = (pkt->data.status & MASK_LACROSSE_SEQ_NUM) >> SHIFT_LACROSSE_SEQ_NUM;
    uint8_t status = pkt->data.status & ~MASK_LACROSSE_SEQ_NUM;

    uint8_t true_crc8 = crc8_calc_lacrosse(pkt->raw, sizeof(pkt->raw));
    bool crc8_valid = pkt->crc8 == true_crc8;

    ESP_LOGI(TAG, "(%s) measure: packet(rssi=0x%02X): id=0x%06X, seq=%d, status=0x%02X, crc=%s (0x%02X vs 0x%02X)",
             tag, frame->rssi, id, seq_num, status, crc8_valid ? "OK" : "BAD", pkt->crc8, true_crc8);

    if (!crc8_valid) {
        ESP_LOGW(TAG, "bad crc!");
        return NULL;
    }

    switch ((id & MASK_LACROSSE_ID_TYPE) >> SHIFT_LACROSSE_ID_TYPE) {
        case LACROSSE_ID_TYPE_LTV_RV3: {
            return handle_lacrosse_ltv_rv3_payload(tag, &pkt->data.payload, frame->rssi);
        }
        case LACROSSE_ID_TYPE_LTV_WSDTH04: {
            return handle_lacrosse_ltv_wsdth04_payload(tag, &pkt->data.payload, frame->rssi);
        }
        default: {
            libiot_logf_error(TAG, "well-formed packet with unknown id type: 0x%X", id);
            return NULL;
        }
    }
}

static void handle_payload_ready(rfm69hcw_handle_t dev, QueueHandle_t queue) {
    lacrosse_rx_t frame;

    // Must read RSSI_VALUE before emptying the FIFO, since this will cause an RX restart
    // and we get a race.
    frame.rssi = rfm69hcw_reg_read(dev, RFM69HCW_REG_RSSI_VALUE);
    // Note that due to an idiosyncrasy with the RFM69HCW when `RFM69HCW_AFC_FEI_AFC_AUTOCLEAR_ON` is
    // set then at this point (i.e. even before the FIFO has been emptied) the `RFM69HCW_REG_AFC_MSB/LSB`
    // registers have already been cleared to zero. However, if the packet never arrives and the timeout
    // IRQ triggers then the AFC value is safe to read.
    //
    // If we desperately want to know the AFC value then we can disable AFC autoclear and perform the clear
    // ourselves as part of the RSSI IRQ interrupt handler.

    // Read out the FIFO contents.
    uint8_t* buff = (uint8_t*) &frame.pkt;
    for (int i = 0; i < sizeof(lacrosse_packet_t); i++) {
        buff[i] = rfm69hcw_reg_read(dev, RFM69HCW_REG_FIFO);
    }

    if (xQueueSend(queue, &frame, 0) != pdTRUE) {
        libiot_logf_error(TAG, "can't queue result");
    }
}

#define MAX_WAIT_ITERS 10

static sensor_poll_result_t poll(const char* tag, rfm69hcw_w_irq_t* data, QueueHandle_t queue) {
    // Wait for RSSI IRQ
    if (!libirq_waiter_sleep_until_active(data->waiter)) {
        abort();
    }

    rfm69hcw_handle_t dev = data->dev;

    uint8_t op_mode = rfm69hcw_reg_read(dev, RFM69HCW_REG_OP_MODE);
    if ((op_mode & MASK_RFM69HCW_OP_MODE_MODE) != RFM69HCW_OP_MODE_MODE_RX) {
        libiot_logf_error(TAG, "state error: bad op mode (not rx), fixing! (0x%02X,0x%02X,0x%02X)",
                          op_mode,
                          rfm69hcw_reg_read(dev, RFM69HCW_REG_IRQ_FLAGS_1),
                          rfm69hcw_reg_read(dev, RFM69HCW_REG_IRQ_FLAGS_2));

        return SENSOR_POLL_RESULT_FAIL;
    }

    const char* fail_msg = NULL;

    int i;

    for (i = 0; i < MAX_WAIT_ITERS; i++) {
        // Has a full packet been recieved?
        if (rfm69hcw_reg_read(dev, RFM69HCW_REG_IRQ_FLAGS_2) & RFM69HCW_IRQ_FLAGS_2_PAYLOAD_READY) {
            handle_payload_ready(dev, queue);
            goto handle_rssi_irq_out;
        }

        uint8_t flags_1 = rfm69hcw_reg_read(dev, RFM69HCW_REG_IRQ_FLAGS_1);

        // Is this interrupt spurious?
        if (!(flags_1 & RFM69HCW_IRQ_FLAGS_1_RSSI)) {
            // Spurious IRQ, so just ignore it.
            goto handle_rssi_irq_out;
        }

        // Have we timed out waiting for a packet?
        if (flags_1 & RFM69HCW_IRQ_FLAGS_1_TIMEOUT) {
            break;
        }

        vTaskDelay(1);
    }

    if (i >= MAX_WAIT_ITERS) {
        fail_msg = "max iters exceeded after RSSI sampling!";
        goto handle_rssi_irq_out;
    }

    uint8_t rssi = rfm69hcw_reg_read(dev, RFM69HCW_REG_RSSI_VALUE);
    uint8_t afc_value_msb = rfm69hcw_reg_read(dev, RFM69HCW_REG_AFC_MSB);
    uint8_t afc_value_lsb = rfm69hcw_reg_read(dev, RFM69HCW_REG_AFC_LSB);
    uint16_t afc_value = (((uint16_t) afc_value_msb) << 8) | (((uint16_t) afc_value_lsb) << 0);

    uint8_t val = rfm69hcw_reg_read(dev, RFM69HCW_REG_PACKET_CONFIG_2);
    rfm69hcw_reg_write(dev, RFM69HCW_REG_PACKET_CONFIG_2, val | RFM69HCW_PACKET_CONFIG_2_RESTART_RX);

    for (i = 0; i < MAX_WAIT_ITERS; i++) {
        if (!(rfm69hcw_reg_read(dev, RFM69HCW_REG_IRQ_FLAGS_1) & RFM69HCW_IRQ_FLAGS_1_TIMEOUT)) {
            break;
        }

        vTaskDelay(1);
    }

    ESP_LOGI(TAG, "rssi timeout, restarting rx (rssi=0x%02X, afc_value=0x%04X)", rssi, afc_value);

    if (i >= MAX_WAIT_ITERS) {
        fail_msg = "max iters exceeded after rx restart!";
        goto handle_rssi_irq_out;
    }

handle_rssi_irq_out:
    if (fail_msg) {
        libiot_logf_error(TAG, "%s (0x%02X,0x%02X,0x%02X)",
                          fail_msg,
                          rfm69hcw_reg_read(dev, RFM69HCW_REG_OP_MODE),
                          rfm69hcw_reg_read(dev, RFM69HCW_REG_IRQ_FLAGS_1),
                          rfm69hcw_reg_read(dev, RFM69HCW_REG_IRQ_FLAGS_2));
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

static void dev_start(rfm69hcw_w_irq_t* data, void* unused) {
    rfm69hcw_configure_rx(data->dev, &RFM69HCW_CFG);
}

static sensor_type_t SENSOR_RFM69HCW_LACROSSE = {
    .name = LIBSENSOR_NAME_RFM69HCW_LACROSSE,

    .queue_item_size = sizeof(lacrosse_rx_t),
    .queue_length = 4,

    .poll_delay_ms = 0,
    .max_uneventful_iters = 0,

    .dev_start = (sensor_dev_start_fn_t) dev_start,

    .poll = (sensor_poll_fn_t) poll,
    .report = (sensor_report_fn_t) report,
};

#define IRQ_CORE_NUM 1

#define IRQ_TASK_PRIORITY 10
#define IRQ_TASK_STACK_SIZE 4096

#define MONITOR_TASK_PRIORITY 10
#define MONITOR_TASK_STACK_SIZE 2048
#define MONITOR_TASK_DELAY_MS (60 * 1000)

static libtask_disposition_t monitor_trigger_once(rfm69hcw_w_irq_t* data) {
    libirq_source_trigger(data->src);
    vTaskDelay(MONITOR_TASK_DELAY_MS / portTICK_PERIOD_MS);

    return LIBTASK_DISPOSITION_CONTINUE;
}

esp_err_t libsensor_drv_register_rfm69hcw_lacrosse(spi_host_device_t host,
                                                   gpio_num_t pin_cs, gpio_num_t pin_rst, gpio_num_t pin_irq,
                                                   const char* tag) {
    esp_err_t ret;

    rfm69hcw_w_irq_t* data = malloc(sizeof(rfm69hcw_w_irq_t));

    // Configure the ESP32 to communicate with the RFM69HCW on `host`.
    ret = rfm69hcw_init(host, pin_cs, pin_rst, pin_irq, &data->dev);
    if (ret != ESP_OK) {
        goto dev_init_fail;
    }

    ret = libirq_source_create(pin_irq, true, IRQ_CORE_NUM, &data->src);
    if (ret != ESP_OK) {
        goto source_create_fail;
    }
    data->waiter = libirq_waiter_create(data->src);

    // TODO consider measuring FEI

    // This loop task just wakes up the poll task (using `libirq_source_trigger()`) every
    // 60 seconds in order to get it to do a check of the current mode (so that we don't lock
    // up if the RFM69HCW ever spontaneously decides to switch out of RX mode).
    ret = libtask_loop_spawn((libtask_do_once_fn_t) monitor_trigger_once, data, "rfm69hcw_monitor_task",
                             MONITOR_TASK_STACK_SIZE, MONITOR_TASK_PRIORITY, &data->monitor_task);
    if (ret != ESP_OK) {
        goto monitor_create_fail;
    }

    return libsensor_register(&SENSOR_RFM69HCW_LACROSSE, tag, data, NULL);

monitor_create_fail:
    libirq_source_destroy(data->src);
    libirq_waiter_destroy(data->waiter);

source_create_fail:
    // TODO Implement a destructor!
    // rfm69hcw_destroy(data->dev);
    abort();

dev_init_fail:
    free(data);

    return ret;
}
