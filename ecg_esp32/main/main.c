#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ble_manager.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "nvs_flash.h"

static const char *TAG = "ECG_MAIN";

/* ═══════════════════════════════════════════════════════════════════════
 *  CHỌN CHẾ ĐỘ XUẤT DỮ LIỆU  —  chỉ sửa tại đây, không sửa chỗ nào khác
 *
 *  ECG_OUTPUT_MQTT   : lọc xong → gửi lên HiveMQ Cloud qua WiFi/MQTT
 *  ECG_OUTPUT_SERIAL : lọc xong → in ra USB-Serial để plot_serial.py vẽ
 *
 *  Bỏ comment đúng một dòng bên dưới:
 * ═══════════════════════════════════════════════════════════════════════ */
// #define ECG_OUTPUT_MQTT
#define ECG_OUTPUT_SERIAL

/* Kiểm tra để tránh định nghĩa cả hai cùng lúc */
#if defined(ECG_OUTPUT_MQTT) && defined(ECG_OUTPUT_SERIAL)
    #error "Chỉ được định nghĩa MỘT trong hai: ECG_OUTPUT_MQTT hoặc ECG_OUTPUT_SERIAL"
#endif
#if !defined(ECG_OUTPUT_MQTT) && !defined(ECG_OUTPUT_SERIAL)
    #error "Phải định nghĩa một trong hai: ECG_OUTPUT_MQTT hoặc ECG_OUTPUT_SERIAL"
#endif

/* Include có điều kiện — chỉ kéo vào những header thực sự cần */
#ifdef ECG_OUTPUT_MQTT
    #include "esp_crt_bundle.h"
    #include "esp_wifi.h"
    #include "mqtt_client.h"
#endif

/*
 * Cấu hình luồng ECG.
 *
 * XIAO nRF52840 gửi đúng 32 mẫu mỗi notification/batch BLE.
 * MAX30003 đang chạy 256 SPS, nên:
 *   5 giây dữ liệu = 256 * 5 = 1280 mẫu
 *   1 block 5 giây = 40 batch BLE, mỗi batch 32 mẫu
 *
 * Các giá trị này được giữ thành macro để tránh "số ma thuật" rải rác trong code.
 */
#define ECG_FS_HZ 256
#define ECG_BLE_BATCH_SAMPLES 32
#define ECG_BLOCK_SECONDS 5
#define ECG_BLOCK_SAMPLES (ECG_FS_HZ * ECG_BLOCK_SECONDS)
#define ECG_BATCHES_PER_BLOCK (ECG_BLOCK_SAMPLES / ECG_BLE_BATCH_SAMPLES)

/*
 * Triple buffer cho block 5 giây.
 *
 * Một buffer đang được task gom dữ liệu ghi vào.
 * Một buffer có thể đang nằm trong queue chờ xử lý.
 * Một buffer có thể đang được task lọc/MQTT đọc.
 *
 * Vì quyền sở hữu buffer được chuyển qua queue, mỗi buffer tại một thời điểm
 * chỉ thuộc một nơi. Nhờ vậy không cần mutex để bảo vệ chính mảng mẫu ECG.
 */
#define ECG_BLOCK_BUFFER_COUNT 3

/*
 * Queue batch BLE cần đủ lớn để hấp thụ jitter ngắn hạn giữa BLE callback và
 * task gom block. 64 batch = 8 giây dữ liệu tại 8 batch/giây.
 */
#define BLE_BATCH_QUEUE_LENGTH 64

/*
 * Queue block sẵn sàng xử lý. Chiều dài bằng số buffer vì mỗi buffer chỉ tạo
 * được một block đang chờ hoặc đang xử lý.
 */
#define ECG_READY_BLOCK_QUEUE_LENGTH ECG_BLOCK_BUFFER_COUNT
#define ECG_FREE_BLOCK_QUEUE_LENGTH ECG_BLOCK_BUFFER_COUNT

/*
 * MAX30003 raw ECG là mã ADC 18-bit đã sign-extend lên int32_t.
 * Với cấu hình XIAO hiện tại CNFG_ECG = 0x415000, gain ECG = 40 V/V.
 *
 * Công thức:
 *   ECG(V) = raw / (2^17 * gain)
 *   ECG(mV) = raw * 1000 / (131072 * 40)
 */
#define ECG_MV_PER_RAW_UNIT (1000.0f / (131072.0f * 40.0f))

/* ─── Cấu hình riêng cho từng chế độ ─────────────────────────────────── */
#ifdef ECG_OUTPUT_MQTT

    #define WIFI_CONNECTED_BIT BIT0
    #define WIFI_FAIL_BIT      BIT1
    #define WIFI_MAX_RETRY     10

    #define ECG_WIFI_SSID       "TECNO POVA 5"
    #define ECG_WIFI_PASSWORD   "12345678"
    #define ECG_MQTT_BROKER_URI "mqtts://7fca0eea573545b996b5e3b23e7e5613.s1.eu.hivemq.cloud:8883"
    #define ECG_MQTT_USERNAME   "iron-holter"
    #define ECG_MQTT_PASSWORD   "Vanh080105"
    #define ECG_MQTT_TOPIC      "devices/SN-ECG-0001/telemetry"
    #define ECG_SERIAL_NUMBER   "SN-ECG-0001"

    /*
     * JSON chứa 1280 mẫu float dạng text nên lớn hơn binary khá nhiều.
     * 24KB đủ cho format hiện tại với 4 chữ số sau dấu phẩy và vẫn có dư địa
     * cho phần metadata.
     */
    #define ECG_MQTT_JSON_BUFFER_SIZE (24 * 1024)

#else /* ECG_OUTPUT_SERIAL */

    /*
     * Mỗi mẫu được in ra UART theo format:
     *   >ECG_FILT:<value_mV>\n   ví dụ: >ECG_FILT:0.1234
     *
     * Tiền tố ">ECG_FILT:" giúp plot_serial.py lọc đúng dòng dữ liệu và
     * bỏ qua các dòng log ESP_LOGx khác đang dùng chung cổng UART0.
     */
    #define ECG_SERIAL_PREFIX ">ECG_FILT:"

#endif /* ECG_OUTPUT_MQTT / ECG_OUTPUT_SERIAL */

typedef struct {
    int32_t samples[ECG_BLE_BATCH_SAMPLES];
} ecg_ble_batch_t;

typedef struct {
    uint8_t buffer_index;
    uint32_t sequence;
    int64_t completed_at_us;
} ecg_block_msg_t;

/*
 * Cấu trúc một tầng lọc Biquad Direct Form I.
 *
 * b0/b1/b2/a1/a2 là hệ số lọc cố định.
 * x1/x2 là hai mẫu đầu vào trước đó.
 * y1/y2 là hai mẫu đầu ra trước đó.
 *
 * Dùng float thay vì double vì ESP32 xử lý single-precision rẻ hơn, và độ
 * chính xác float đã đủ cho ECG sau khi scale từ raw ADC.
 */
typedef struct {
    float b0;
    float b1;
    float b2;
    float a1;
    float a2;
    float x1;
    float x2;
    float y1;
    float y2;
} BiquadFilter;

/*
 * Bộ nhớ dữ liệu chính.
 *
 * ecg_blocks chứa raw int32_t để giữ nguyên dữ liệu gốc từ MAX30003.
 * filter_work_mv là buffer trung gian để xử lý lọc 2 chiều theo từng block.
 * filter_output_mv chứa block 5 giây sau khi đã lọc và đổi sang mV.
 * mqtt_payload_json chỉ cấp phát ở chế độ MQTT để tránh lãng phí RAM.
 */
static int32_t ecg_blocks[ECG_BLOCK_BUFFER_COUNT][ECG_BLOCK_SAMPLES];
static float filter_work_mv[ECG_BLOCK_SAMPLES];
static float filter_output_mv[ECG_BLOCK_SAMPLES];

#ifdef ECG_OUTPUT_MQTT
static char mqtt_payload_json[ECG_MQTT_JSON_BUFFER_SIZE];
#endif

static QueueHandle_t ble_batch_queue;
static QueueHandle_t free_block_queue;
static QueueHandle_t ready_block_queue;

static volatile uint32_t total_samples = 0;
static volatile uint32_t dropped_ble_batches = 0;
static volatile uint32_t ready_queue_wait_events = 0;

/*
 * Butterworth bandpass 0.5-40 Hz lấy từ Python:
 *
 *   sos = butter(6, [0.5, 40.0], btype="bandpass", fs=256, output="sos")
 *
 * Python trả mỗi SOS section theo thứ tự:
 *   b0, b1, b2, a0, a1, a2
 *
 * a0 của các section này đều bằng 1.0 nên trong BiquadFilter chỉ lưu:
 *   b0, b1, b2, a1, a2
 *
 * Không dùng dạng b/a bậc cao trực tiếp vì bộ lọc bandpass order=6 có đa thức
 * bậc 12; dạng đó dễ nhạy sai số hơn trên vi điều khiển. Tách thành 6 biquad
 * giúp ổn định số học hơn và phù hợp với hàm processBiquad() đã có.
 */
#define ECG_BUTTER_SOS_SECTIONS 6

static BiquadFilter butter_sos[ECG_BUTTER_SOS_SECTIONS] = {
    {2.966517794737948740e-03f, 5.933035589475897480e-03f, 2.966517794737948740e-03f,
     -6.366807269431551397e-01f, 1.160545192432670958e-01f, 0, 0, 0, 0},
    {1.000000000000000000e+00f, 2.000000000000000000e+00f, 1.000000000000000000e+00f,
     -7.143259153499533776e-01f, 2.679739533249513306e-01f, 0, 0, 0, 0},
    {1.000000000000000000e+00f, 2.000000000000000000e+00f, 1.000000000000000000e+00f,
     -9.196435814321769486e-01f, 6.522833317502231276e-01f, 0, 0, 0, 0},
    {1.000000000000000000e+00f, -2.000000000000000000e+00f, 1.000000000000000000e+00f,
     -1.975943627642759104e+00f, 9.760990623142598022e-01f, 0, 0, 0, 0},
    {1.000000000000000000e+00f, -2.000000000000000000e+00f, 1.000000000000000000e+00f,
     -1.982651254083403813e+00f, 9.828039800484642541e-01f, 0, 0, 0, 0},
    {1.000000000000000000e+00f, -2.000000000000000000e+00f, 1.000000000000000000e+00f,
     -1.993642583094657361e+00f, 9.937931557403766325e-01f, 0, 0, 0, 0},
};

#ifdef ECG_OUTPUT_MQTT
static EventGroupHandle_t wifi_event_group;
static esp_mqtt_client_handle_t mqtt_client;
static volatile bool mqtt_connected = false;
static int wifi_retry_count = 0;
#endif

static void resetBiquadState(BiquadFilter *f)
{
    f->x1 = 0.0f;
    f->x2 = 0.0f;
    f->y1 = 0.0f;
    f->y2 = 0.0f;
}

static void resetBiquadChainState(BiquadFilter *filters, size_t count)
{
    for (size_t i = 0; i < count; i++) {
        resetBiquadState(&filters[i]);
    }
}

static float processBiquad(BiquadFilter *f, float x)
{
    /*
     * Direct Form I:
     *   y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2]
     *        - a1*y[n-1] - a2*y[n-2]
     */
    float y = f->b0 * x + f->b1 * f->x1 + f->b2 * f->x2
            - f->a1 * f->y1 - f->a2 * f->y2;

    /*
     * Chặn NaN/Inf để một mẫu lỗi không làm hỏng toàn bộ state IIR phía sau.
     * Nếu phép tính lỗi, giữ đầu ra gần nhất.
     */
    if (isnan(y) || isinf(y)) {
        y = f->y1;
    }

    f->x2 = f->x1;
    f->x1 = x;
    f->y2 = f->y1;
    f->y1 = y;

    return y;
}

static float processBiquadChain(BiquadFilter *filters, size_t count, float sample)
{
    float out = sample;

    for (size_t i = 0; i < count; i++) {
        out = processBiquad(&filters[i], out);
    }

    return out;
}

static void applyButterworthForwardBlock(float *samples_mv, size_t count)
{
    /*
     * Chạy một lượt lọc tiến qua toàn bộ chuỗi 6 SOS section.
     * State của chuỗi phải được reset trước mỗi lượt forward/backward để hai
     * lượt lọc không dùng nhầm lịch sử của nhau.
     */
    resetBiquadChainState(butter_sos, ECG_BUTTER_SOS_SECTIONS);

    for (size_t i = 0; i < count; i++) {
        samples_mv[i] = processBiquadChain(butter_sos,
                                           ECG_BUTTER_SOS_SECTIONS,
                                           samples_mv[i]);
    }
}

static void reverseFloatBlock(float *samples, size_t count)
{
    if (count == 0) {
        return;
    }

    for (size_t left = 0, right = count - 1; left < right; left++, right--) {
        const float tmp = samples[left];
        samples[left] = samples[right];
        samples[right] = tmp;
    }
}

static void init_ecg_filters(void)
{
    /*
     * Chế độ hiện tại là thử nghiệm lọc hai chiều theo block 5 giây:
     *   1. raw -> mV bằng hệ số MAX30003 hiện tại
     *   2. lọc tiến Butterworth bandpass 0.5-40 Hz
     *   3. đảo block
     *   4. lọc tiến lần nữa trên block đã đảo
     *   5. đảo lại để khôi phục thứ tự thời gian
     *
     * Cách này giảm lệch pha tốt hơn lọc IIR một chiều, nhưng là lựa chọn có
     * đánh đổi:
     *   - Mỗi block 5 giây được lọc độc lập, không giữ state liên tục qua block.
     *   - Đầu/cuối block có thể méo hoặc nhìn như gãy khi ghép hai block.
     *   - Đây không phải bản sao hoàn toàn của scipy.signal.filtfilt vì chưa
     *     có padding và chưa tính điều kiện đầu/cuối như SciPy.
     *   - Bộ lọc được áp dụng 2 lần nên đáp ứng biên độ mạnh hơn lọc một chiều.
     *
     * Các rủi ro trên được chấp nhận để thử nghiệm hiển thị ECG ít lệch pha
     * hơn trong pipeline block 5 giây hiện tại.
     */
    resetBiquadChainState(butter_sos, ECG_BUTTER_SOS_SECTIONS);

    ESP_LOGI(TAG,
             "Bộ lọc ECG: Butterworth bandpass 0.5-40 Hz SOS, forward-backward theo block %d giây, fs=%d Hz",
             ECG_BLOCK_SECONDS,
             ECG_FS_HZ);
}

static void processRawEcg(const int32_t *raw_samples, float *filtered_mv, size_t count)
{
    /*
     * Hàm xử lý chính cho một block ECG raw.
     *
     * Input:
     *   raw_samples: 5 giây dữ liệu raw từ MAX30003, hiện tại là 1280 mẫu.
     *
     * Output:
     *   filtered_mv: 5 giây dữ liệu đã lọc, đơn vị mV, dạng float32.
     *
     * Thứ tự xử lý mới:
     *   1. đổi raw sang mV bằng ECG_MV_PER_RAW_UNIT hiện tại
     *   2. lọc tiến bằng Butterworth bandpass 0.5-40 Hz dạng SOS
     *   3. đảo mảng để biến lượt lọc tiến thứ hai thành lọc lùi theo thời gian
     *   4. lọc tiến lần hai
     *   5. đảo lại và xuất kết quả mV
     *
     * Không trừ mean của block. Baseline/DC chậm được xử lý bởi nhánh high-pass
     * 0.5 Hz nằm trong bộ lọc bandpass Butterworth.
     */
    if (count > ECG_BLOCK_SAMPLES) {
        ESP_LOGE(TAG, "processRawEcg count=%u vượt quá buffer %d mẫu",
                 (unsigned)count,
                 ECG_BLOCK_SAMPLES);
        count = ECG_BLOCK_SAMPLES;
    }

    for (size_t i = 0; i < count; i++) {
        filter_work_mv[i] = (float)raw_samples[i] * ECG_MV_PER_RAW_UNIT;
    }

    applyButterworthForwardBlock(filter_work_mv, count);
    reverseFloatBlock(filter_work_mv, count);
    applyButterworthForwardBlock(filter_work_mv, count);
    reverseFloatBlock(filter_work_mv, count);

    memcpy(filtered_mv, filter_work_mv, count * sizeof(filtered_mv[0]));
}

/*
 * Callback nhận batch ECG từ ble_manager.
 *
 * Đây không phải nơi lọc hoặc gửi MQTT. Callback BLE cần kết thúc nhanh để
 * không làm nghẽn NimBLE. Vì vậy nó chỉ copy 32 mẫu vào ble_batch_queue.
 */
static void process_ecg_data(int32_t *samples, int count)
{
    if (count != ECG_BLE_BATCH_SAMPLES) {
        ESP_LOGW(TAG, "Bỏ qua batch BLE sai kích thước: %d mẫu, kỳ vọng %d",
                 count, ECG_BLE_BATCH_SAMPLES);
        return;
    }

    ecg_ble_batch_t batch;
    memcpy(batch.samples, samples, sizeof(batch.samples));
    total_samples += ECG_BLE_BATCH_SAMPLES;

    if (xQueueSend(ble_batch_queue, &batch, 0) != pdPASS) {
        dropped_ble_batches++;
        ESP_LOGW(TAG, "ble_batch_queue đầy, mất %" PRIu32 " batch BLE", dropped_ble_batches);
    }
}

/*
 * Task 1: nhận batch 32 mẫu từ BLE callback và gom thành block 5 giây.
 *
 * Mỗi block gồm 1280 mẫu. Khi block đầy, task chuyển quyền sở hữu buffer sang
 * ready_block_queue cho task xử lý. Sau đó task lấy một buffer rảnh khác từ
 * free_block_queue để tiếp tục ghi dữ liệu mới.
 */
static void ecg_block_builder_task(void *arg)
{
    (void)arg;

    ecg_ble_batch_t batch;
    uint8_t current_buffer_index = 0;
    size_t write_offset = 0;
    uint32_t block_sequence = 0;

    xQueueReceive(free_block_queue, &current_buffer_index, portMAX_DELAY);

    for (;;) {
        xQueueReceive(ble_batch_queue, &batch, portMAX_DELAY);

        memcpy(&ecg_blocks[current_buffer_index][write_offset],
               batch.samples,
               sizeof(batch.samples));
        write_offset += ECG_BLE_BATCH_SAMPLES;

        if (write_offset == ECG_BLOCK_SAMPLES) {
            const ecg_block_msg_t msg = {
                .buffer_index = current_buffer_index,
                .sequence = block_sequence++,
                .completed_at_us = esp_timer_get_time(),
            };

            ESP_LOGI(TAG,
                     "Đã gom đủ %d giây ECG: seq=%" PRIu32 ", buffer=%u, samples=%d. Chuyển sang task xử lý.",
                     ECG_BLOCK_SECONDS,
                     msg.sequence,
                     msg.buffer_index,
                     ECG_BLOCK_SAMPLES);

            if (xQueueSend(ready_block_queue, &msg, 0) != pdPASS) {
                /*
                 * Nếu ready queue đầy, task xử lý đang chậm hơn tốc độ tạo block.
                 * Ta chờ để không ghi đè buffer chứa dữ liệu chưa xử lý.
                 * Trong thời gian chờ, ble_batch_queue vẫn hấp thụ batch mới.
                 */
                ready_queue_wait_events++;
                ESP_LOGW(TAG, "ready_block_queue đầy, chờ task xử lý. Sự kiện: %" PRIu32,
                         ready_queue_wait_events);
                xQueueSend(ready_block_queue, &msg, portMAX_DELAY);
            }

            xQueueReceive(free_block_queue, &current_buffer_index, portMAX_DELAY);
            write_offset = 0;
        }
    }
}

/* ═══════════════════════════════════════════════════════════════════════
 *  CÁC HÀM RIÊNG CHO TỪNG CHẾ ĐỘ
 * ═══════════════════════════════════════════════════════════════════════ */

#ifdef ECG_OUTPUT_MQTT

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    (void)arg;
    (void)event_data;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        mqtt_connected = false;
        if (wifi_retry_count < WIFI_MAX_RETRY) {
            esp_wifi_connect();
            wifi_retry_count++;
            ESP_LOGW(TAG, "WiFi mất kết nối, thử lại lần %d/%d", wifi_retry_count, WIFI_MAX_RETRY);
        } else {
            xEventGroupSetBits(wifi_event_group, WIFI_FAIL_BIT);
            ESP_LOGE(TAG, "WiFi kết nối thất bại sau %d lần thử", WIFI_MAX_RETRY);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        wifi_retry_count = 0;
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
        ESP_LOGI(TAG, "WiFi đã có IP");
    }
}

static void wifi_init_sta(void)
{
    wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    wifi_config_t wifi_config = {0};
    strlcpy((char *)wifi_config.sta.ssid, ECG_WIFI_SSID, sizeof(wifi_config.sta.ssid));
    strlcpy((char *)wifi_config.sta.password,
            ECG_WIFI_PASSWORD,
            sizeof(wifi_config.sta.password));
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Đang kết nối WiFi SSID='%s'", ECG_WIFI_SSID);
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    (void)handler_args;
    (void)base;
    (void)event_data;

    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        mqtt_connected = true;
        ESP_LOGI(TAG, "MQTT đã kết nối");
        break;
    case MQTT_EVENT_DISCONNECTED:
        mqtt_connected = false;
        ESP_LOGW(TAG, "MQTT mất kết nối");
        break;
    case MQTT_EVENT_ERROR:
        mqtt_connected = false;
        ESP_LOGE(TAG, "MQTT lỗi kết nối");
        break;
    default:
        break;
    }
}

static void mqtt_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = ECG_MQTT_BROKER_URI,
        .broker.verification.crt_bundle_attach = esp_crt_bundle_attach,
        .credentials.username = ECG_MQTT_USERNAME,
        .credentials.authentication.password = ECG_MQTT_PASSWORD,
    };

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    ESP_ERROR_CHECK(esp_mqtt_client_register_event(mqtt_client,
                                                   ESP_EVENT_ANY_ID,
                                                   mqtt_event_handler,
                                                   NULL));
    ESP_ERROR_CHECK(esp_mqtt_client_start(mqtt_client));
}

static bool append_json(char **cursor, size_t *remaining, const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    const int written = vsnprintf(*cursor, *remaining, fmt, args);
    va_end(args);

    if (written < 0 || (size_t)written >= *remaining) {
        return false;
    }

    *cursor += written;
    *remaining -= (size_t)written;
    return true;
}

static int build_ecg_json_payload(const ecg_block_msg_t *msg, const float *filtered_mv)
{
    /*
     * message_id dùng để server phân biệt từng block 5 giây và nhận diện gói
     * bị gửi lại. Format:
     *   SN-ECG-0001-<sent_at_seconds>-<sequence>
     *
     * sent_at_ms hiện là thời điểm block 5 giây được gom đủ, theo uptime của
     * thiết bị tính bằng millisecond. Nếu sau này cần Unix epoch thật như ví dụ
     * 1612345678123, cần thêm SNTP/RTC trước khi publish.
     */
    const int64_t sent_at_ms = msg->completed_at_us / 1000;
    const int64_t sent_at_seconds = sent_at_ms / 1000;
    char *cursor = mqtt_payload_json;
    size_t remaining = sizeof(mqtt_payload_json);

    if (!append_json(&cursor,
                     &remaining,
                     "{\"message_id\":\"%s-%" PRId64 "-%" PRIu32 "\","
                     "\"serial_number\":\"%s\","
                     "\"sent_at\":%" PRId64 ","
                     "\"ecg_signal\":[",
                     ECG_SERIAL_NUMBER,
                     sent_at_seconds,
                     msg->sequence,
                     ECG_SERIAL_NUMBER,
                     sent_at_ms)) {
        return -1;
    }

    for (size_t i = 0; i < ECG_BLOCK_SAMPLES; i++) {
        if (!append_json(&cursor,
                         &remaining,
                         (i == 0) ? "%.4f" : ",%.4f",
                         (double)filtered_mv[i])) {
            return -1;
        }
    }

    if (!append_json(&cursor,
                     &remaining,
                     "],\"sampling_rate\":{\"ecg_hz\":%d,\"duration\":%d}}",
                     ECG_FS_HZ,
                     ECG_BLOCK_SECONDS)) {
        return -1;
    }

    return (int)(cursor - mqtt_payload_json);
}

static bool mqtt_publish_filtered_block(const ecg_block_msg_t *msg, const float *filtered_mv)
{
    if (!mqtt_connected || mqtt_client == NULL) {
        ESP_LOGW(TAG, "MQTT chưa kết nối, chưa gửi block seq=%" PRIu32, msg->sequence);
        return false;
    }

    const int payload_len = build_ecg_json_payload(msg, filtered_mv);
    if (payload_len <= 0) {
        ESP_LOGE(TAG, "Không tạo được JSON MQTT, buffer %u byte không đủ",
                 (unsigned)sizeof(mqtt_payload_json));
        return false;
    }

    ESP_LOGI(TAG,
             "Chuẩn bị gửi MQTT: seq=%" PRIu32 ", topic=%s, payload=%d bytes",
             msg->sequence,
             ECG_MQTT_TOPIC,
             payload_len);

    const int msg_id = esp_mqtt_client_publish(mqtt_client,
                                               ECG_MQTT_TOPIC,
                                               mqtt_payload_json,
                                               payload_len,
                                               1,
                                               0);

    if (msg_id < 0) {
        ESP_LOGE(TAG, "MQTT publish thất bại ngay tại esp_mqtt_client_publish(), seq=%" PRIu32,
                 msg->sequence);
    } else {
        ESP_LOGI(TAG, "MQTT đã enqueue publish: seq=%" PRIu32 ", msg_id=%d",
                 msg->sequence,
                 msg_id);
    }

    return msg_id >= 0;
}

#else /* ECG_OUTPUT_SERIAL */

static void serial_print_filtered_block(const ecg_block_msg_t *msg, const float *filtered_mv)
{
    /*
     * In từng mẫu ra UART0 (USB-Serial) để plot_serial.py đọc và vẽ đồ thị.
     *
     * Dùng printf() thay vì ESP_LOG để không thêm timestamp/tag làm script
     * parse phức tạp hơn và giữ throughput cao:
     *   1280 mẫu × ~16 ký tự ≈ 20 KB / 5 giây → ~1.4 giây @ 115200 baud.
     */
    ESP_LOGI(TAG,
             "In block ECG seq=%" PRIu32 " ra Serial (%d mẫu)",
             msg->sequence,
             ECG_BLOCK_SAMPLES);

    for (size_t i = 0; i < ECG_BLOCK_SAMPLES; i++) {
        printf("%s%.4f\n", ECG_SERIAL_PREFIX, (double)filtered_mv[i]);
    }
}

#endif /* ECG_OUTPUT_MQTT / ECG_OUTPUT_SERIAL */

/*
 * Task 2: xử lý block 5 giây.
 *
 * Task này chờ ready_block_queue. Mỗi khi có block:
 *   1. lọc tín hiệu ECG
 *   2. xuất dữ liệu theo chế độ được chọn (MQTT hoặc Serial)
 *   3. log thời gian xử lý
 *   4. trả buffer về free_block_queue cho task gom dữ liệu dùng lại
 */
static void ecg_output_task(void *arg)
{
    (void)arg;

    ecg_block_msg_t msg;

    for (;;) {
        xQueueReceive(ready_block_queue, &msg, portMAX_DELAY);

        ESP_LOGI(TAG,
                 "Bắt đầu xử lý block ECG 5 giây: seq=%" PRIu32 ", buffer=%u",
                 msg.sequence,
                 msg.buffer_index);

        const int64_t start_us = esp_timer_get_time();
        processRawEcg(ecg_blocks[msg.buffer_index], filter_output_mv, ECG_BLOCK_SAMPLES);

#ifdef ECG_OUTPUT_MQTT
        const bool published = mqtt_publish_filtered_block(&msg, filter_output_mv);
        const int64_t elapsed_us = esp_timer_get_time() - start_us;

        ESP_LOGI(TAG,
                 "Block ECG seq=%" PRIu32 " xử lý trong %.2f ms, MQTT=%s",
                 msg.sequence,
                 (double)elapsed_us / 1000.0,
                 published ? "sent" : "not-sent");

        if (elapsed_us > (ECG_BLOCK_SECONDS * 1000000LL)) {
            ESP_LOGW(TAG,
                     "Task lọc/MQTT chậm hơn chu kỳ block %d giây: %.2f ms",
                     ECG_BLOCK_SECONDS,
                     (double)elapsed_us / 1000.0);
        }

#else /* ECG_OUTPUT_SERIAL */
        serial_print_filtered_block(&msg, filter_output_mv);
        const int64_t elapsed_us = esp_timer_get_time() - start_us;

        ESP_LOGI(TAG,
                 "Block ECG seq=%" PRIu32 " xử lý + in Serial trong %.2f ms",
                 msg.sequence,
                 (double)elapsed_us / 1000.0);

        if (elapsed_us > (ECG_BLOCK_SECONDS * 1000000LL)) {
            ESP_LOGW(TAG,
                     "Task lọc/Serial chậm hơn chu kỳ block %d giây: %.2f ms",
                     ECG_BLOCK_SECONDS,
                     (double)elapsed_us / 1000.0);
        }
#endif

        xQueueSend(free_block_queue, &msg.buffer_index, portMAX_DELAY);
    }
}

/*
 * Task giám sát tốc độ nhận mẫu thực tế.
 * Nó chỉ dùng để kiểm tra đường BLE có giữ khoảng 256 SPS hay không.
 */
static void monitor_sps_task(void *arg)
{
    (void)arg;
    uint32_t last_sample_count = 0;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        const uint32_t current_count = total_samples;
        const uint32_t sps = current_count - last_sample_count;
        last_sample_count = current_count;

        if (sps > 0) {
            ESP_LOGI(TAG,
                     "Tốc độ nhận: %" PRIu32 " SPS, batch BLE drop=%" PRIu32,
                     sps,
                     dropped_ble_batches);
        }
    }
}

static void init_queues(void)
{
    ble_batch_queue = xQueueCreate(BLE_BATCH_QUEUE_LENGTH, sizeof(ecg_ble_batch_t));
    free_block_queue = xQueueCreate(ECG_FREE_BLOCK_QUEUE_LENGTH, sizeof(uint8_t));
    ready_block_queue = xQueueCreate(ECG_READY_BLOCK_QUEUE_LENGTH, sizeof(ecg_block_msg_t));

    if (ble_batch_queue == NULL || free_block_queue == NULL || ready_block_queue == NULL) {
        ESP_LOGE(TAG, "Không thể tạo queue ECG");
        abort();
    }

    for (uint8_t i = 0; i < ECG_BLOCK_BUFFER_COUNT; i++) {
        xQueueSend(free_block_queue, &i, portMAX_DELAY);
    }
}

void app_main(void)
{
    /*
     * NVS cần được khởi tạo trước khi NimBLE và WiFi dùng để lưu/đọc trạng thái.
     */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

#ifdef ECG_OUTPUT_MQTT
    ESP_LOGI(TAG, "\n--- KHỞI ĐỘNG GATEWAY ESP32 ECG BLE -> FILTER -> MQTT ---\n");
#else
    ESP_LOGI(TAG, "\n--- KHỞI ĐỘNG GATEWAY ESP32 ECG BLE -> FILTER -> SERIAL ---\n");
#endif

    init_queues();
    init_ecg_filters();

#ifdef ECG_OUTPUT_MQTT
    wifi_init_sta();
    mqtt_start();
#endif

    /*
     * Task 1 gom dữ liệu BLE thành block 5 giây.
     * Task 2 lọc block và xuất ra theo chế độ đã chọn.
     * monitor_sps_task chỉ để quan sát tốc độ nhận thực tế.
     */
    xTaskCreate(ecg_block_builder_task, "ecg_block_builder", 4096, NULL, 5, NULL);
    xTaskCreate(ecg_output_task,        "ecg_output",        8192, NULL, 4, NULL);
    xTaskCreate(monitor_sps_task,       "monitor_sps",       2048, NULL, 3, NULL);

    /*
     * ble_manager sẽ scan, connect XIAO và gọi process_ecg_data() mỗi khi đủ
     * 32 mẫu. Callback không lọc, không MQTT, chỉ đẩy batch vào queue.
     */
    ble_manager_init(process_ecg_data);
}
