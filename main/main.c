#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "ble_manager.h" 

static const char *TAG = "ECG_MAIN";

/*
 * Tổng số mẫu ECG đã nhận từ BLE.
 * Callback BLE cập nhật biến này, còn task monitor đọc mỗi giây để tính SPS.
 */
volatile uint32_t total_samples = 0;

/*
 * Callback được ble_manager gọi khi có notification chứa mẫu ECG mới.
 * Hàm này chạy trong ngữ cảnh BLE callback nên chỉ làm việc ngắn: đếm mẫu
 * và đẩy dữ liệu ra UART/Serial cho công cụ plot.
 */
void process_ecg_data(int32_t *samples, int count) {
    total_samples += count;
    for (int i = 0; i < count; i++) {
        // plot_serial.py chỉ lấy các dòng có tiền tố ">ECG:" để vẽ đồ thị.
        printf(">ECG:%" PRId32 "\n", samples[i]);
    }
}

/*
 * Task giám sát tốc độ lấy mẫu thực tế.
 * Mỗi giây, task lấy chênh lệch total_samples để suy ra SPS đang nhận.
 */
void monitor_sps_task(void *arg) {
    uint32_t last_sample_count = 0;
    
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        uint32_t current_count = total_samples;
        uint32_t sps = current_count - last_sample_count;
        last_sample_count = current_count;
        if (sps > 0) {
            ESP_LOGI(TAG, "Tốc độ nhận: %" PRIu32 " SPS (Mẫu/giây)", sps);
        }
    }
}

void app_main(void) {
    // NVS cần được khởi tạo trước khi NimBLE dùng để lưu/đọc trạng thái BLE.
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // ESP-IDF yêu cầu xóa và khởi tạo lại NVS khi phân vùng đầy hoặc khác version.
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "\n--- KHỞI ĐỘNG GATEWAY ESP32 ---\n");
    // Task này chỉ ghi log SPS; luồng nhận dữ liệu ECG nằm trong callback BLE.
    xTaskCreate(monitor_sps_task, "monitor_sps", 2048, NULL, 4, NULL);
    // Đăng ký callback để ble_manager chuyển các mẫu ECG nhận qua BLE về main.c.
    ble_manager_init(process_ecg_data);

    // Giữ app_main tồn tại; NimBLE và task monitor chạy ở các task riêng.
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
